#coding=utf-8
"""geometry.py 的測試，不需要 ROS 也不需要相機。

    cd nstc/src/bottle_localization
    python3 -m pytest test/ -v
    python3 -m pytest test/ -v -s -k noise      # 印出加雜訊後的誤差分布

8.1 單元測試：內參換算、反投影、雜點過濾、半徑修正、合理性檢查、點數不足。
8.2 合成場景：給定圓柱真實位置、尺寸與相機姿態，光線追蹤產生深度圖
    （桌面 + 圓柱側面與頂面 + z-buffer 遮擋），照 /depth_mm 的方式截斷成公釐，
    再丟進 geometry.localize()，比對算出來的中心與真值。
    合成用的內參是假設值，不是機器人的實際內參。
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from bottle_localization import geometry as g  # noqa: E402


# =============================================================================
# 合成場景工具
# =============================================================================
W, H = 960, 600
#合成用的內參（假設值）。主點刻意不放在正中央，主點用錯時測試才會抓到
INTR = g.Intrinsics(fx=360.0, fy=362.0, cx=478.6, cy=303.2)
#optical frame（X 右、Y 下、Z 前）在相機本體座標（X 前、Y 左、Z 上）下的軸向
R_BODY_FROM_OPT = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0]])
Z_TABLE = 0.12          #桌面高度（base_link），故意不是 0
MIN_POINTS = 20
SIZE_RATIO = 0.3
#選用修正的設定（與 params.yaml 的建議值相同）
REFINED = dict(top_surface_margin=0.01, arc_iterations=3, arc_trim_pct=2.0)
SPEC = dict()


def camera_pose(pitch_deg, position, yaw_deg=0.0):
    """相機姿態：往下看 pitch_deg 度（正值往下），回 optical -> base 的 (R, t)。"""
    rot = g.rpy_to_matrix(0.0, math.radians(pitch_deg), math.radians(yaw_deg)) @ R_BODY_FROM_OPT
    return rot, np.asarray(position, dtype=np.float64)


def render(cylinders, rot, cam, z_table=Z_TABLE, intr=INTR):
    """光線追蹤一張深度圖。

    Args:
        cylinders (list): [(ax, ay, diameter, height), ...]，直立在桌面上。
        rot, cam: optical -> base 的旋轉與相機位置。

    Returns:
        tuple: (depth_m, labels)。depth_m 是沿光軸的 Z（公尺），打不到東西是 0；
            labels 是每個像素打到的物體：-1 沒打到、0 桌面、k+1 第 k 個圓柱。
    """
    u, v = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    d_opt = np.stack([(u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy, np.ones_like(u)], -1)
    d = d_opt @ rot.T                     #base 下的射線方向；optical 的 Z 分量是 1，所以 t 就是深度
    dx, dy, dz = d[..., 0], d[..., 1], d[..., 2]
    cx, cy, cz = cam

    best = np.full(u.shape, np.inf)
    labels = np.full(u.shape, -1, dtype=np.int32)

    with np.errstate(divide='ignore', invalid='ignore'):
        t = (z_table - cz) / dz
    hit = (dz < 0) & (t > 0)
    upd = hit & (t < best)
    best[upd] = t[upd]
    labels[upd] = 0

    for k, (ax, ay, dia, hgt) in enumerate(cylinders):
        r = dia / 2.0
        z_top = z_table + hgt
        ox, oy = cx - ax, cy - ay
        a = dx * dx + dy * dy
        b = 2.0 * (ox * dx + oy * dy)
        c = ox * ox + oy * oy - r * r
        disc = b * b - 4.0 * a * c
        with np.errstate(divide='ignore', invalid='ignore'):
            t_side = (-b - np.sqrt(disc)) / (2.0 * a)
            z_side = cz + t_side * dz
            side = (disc >= 0) & (t_side > 0) & (z_side >= z_table) & (z_side <= z_top)
            t_cap = (z_top - cz) / dz
            px = cx + t_cap * dx - ax
            py = cy + t_cap * dy - ay
            cap = (dz < 0) & (t_cap > 0) & (px * px + py * py <= r * r)
        t_obj = np.where(side, t_side, np.inf)
        t_obj = np.where(cap & (t_cap < t_obj), t_cap, t_obj)
        upd = t_obj < best
        best[upd] = t_obj[upd]
        labels[upd] = k + 1

    depth = np.where(np.isfinite(best), best, 0.0)
    return depth, labels


def to_depth_mm_and_back(depth_m):
    """模擬 depth_process_node：公尺 x 1000 截斷成 uint16，本節點再除以 1000。"""
    mm = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
    return mm.astype(np.float32) / 1000.0


def bbox_of(labels, label):
    """完美的 YOLO 框：該物體可見像素的外接矩形。"""
    vs, us = np.nonzero(labels == label)
    return (int(us.min()), int(vs.min()), int(us.max()), int(vs.max()))


def scene_for(pitch_deg, lateral, dims, distance=0.45, neighbor=None):
    """建一個場景：相機往下看 pitch_deg，目標瓶在前方 distance、側向 lateral。

    相機高度取「讓瓶身中心大約落在光軸上」，這樣不同俯角時瓶子都在畫面裡。
    neighbor=(dia, hgt) 時在目標的 +y 側（畫面左邊）間隔 2 cm 再放一瓶。
    """
    dia, hgt = dims
    cam_xy = np.array([0.03, -0.02])
    axis = cam_xy + np.array([distance, lateral])
    cam_z = Z_TABLE + hgt / 2.0 + distance * math.tan(math.radians(pitch_deg))
    rot, cam = camera_pose(pitch_deg, [cam_xy[0], cam_xy[1], cam_z])
    cylinders = [(axis[0], axis[1], dia, hgt)]
    if neighbor is not None:
        n_dia, n_hgt = neighbor
        gap = 0.02
        cylinders.append((axis[0], axis[1] + dia / 2.0 + gap + n_dia / 2.0, n_dia, n_hgt))
    return rot, cam, cylinders, np.array([axis[0], axis[1], Z_TABLE + hgt / 2.0])


def run_pipeline(depth_m, bbox, dims, rot, cam, mode=SPEC):
    dia, hgt = dims
    return g.localize(depth_m, bbox, INTR, dia, hgt, rot, cam, Z_TABLE,
                      MIN_POINTS, SIZE_RATIO, None, **mode)


def noise_free_error(pitch, side, dims_name, with_neighbor, mode):
    """無雜訊場景跑一次，回 (水平誤差 m, 結果)。"""
    dims = DIMS[dims_name]
    neighbor = DIMS['large' if dims_name == 'small' else 'small'] if with_neighbor else None
    rot, cam, cylinders, truth = scene_for(pitch, LATERALS[side], dims, neighbor=neighbor)
    depth, labels = render(cylinders, rot, cam)
    depth = to_depth_mm_and_back(depth)
    bbox = bbox_of(labels, 1)
    assert not g.touches_edge(bbox, W, H, 2), '合成場景的瓶子被畫面切掉了'
    res, reason = run_pipeline(depth, bbox, dims, rot, cam, mode)
    assert res is not None, reason
    assert res.center[2] == pytest.approx(truth[2], abs=1e-12)
    return float(np.linalg.norm(res.center[:2] - truth[:2])), res


# =============================================================================
# 8.1 單元測試
# =============================================================================
def test_intrinsics_scale_and_letterbox():
    """縮放 + 補黑：換算後的值，以及「原生投影再換算」= 「直接用換算後內參投影」。"""
    fx, fy, cx, cy = 700.0, 705.0, 640.2, 360.7          #1280x720 的原生內參（假設）
    s, left, top = 0.75, 0, 30                          #-> 960x540 再上下補 30 列
    intr = g.detection_intrinsics(fx, fy, cx, cy, s, left, top)
    assert intr.fx == pytest.approx(525.0)
    assert intr.fy == pytest.approx(528.75)
    assert intr.cx == pytest.approx(480.15)
    assert intr.cy == pytest.approx(300.525)

    rng = np.random.default_rng(1)
    pts = np.column_stack([rng.uniform(-0.3, 0.3, 50), rng.uniform(-0.2, 0.2, 50),
                           rng.uniform(0.3, 1.5, 50)])
    u_n, v_n = g.project(pts, g.Intrinsics(fx, fy, cx, cy))
    u_d, v_d = g.project(pts, intr)
    np.testing.assert_allclose(u_n * s + left, u_d, atol=1e-9)
    np.testing.assert_allclose(v_n * s + top, v_d, atol=1e-9)


def test_intrinsics_identity_for_zed():
    """ZED X Mini 的實際設定：camera_info 已是 960x600，s=1、黑邊 0、zoom 1 -> 恆等。"""
    intr = g.detection_intrinsics(366.1, 366.3, 481.2, 298.7, 1.0, 0, 0, 1.0, W, H)
    assert intr == pytest.approx((366.1, 366.3, 481.2, 298.7))


def test_intrinsics_zoom_matches_center_crop():
    """數位變焦：中心裁切 int(W/z) 再放大回 W，和 depth_process_node 的寫法一致。"""
    fx, fy, cx, cy = 360.0, 362.0, 478.6, 303.2
    for zoom in (1.2, 1.3, 1.5):
        intr = g.detection_intrinsics(fx, fy, cx, cy, 1.0, 0, 0, zoom, W, H)
        new_w, new_h = int(W / zoom), int(H / zoom)
        x0, y0 = (W - new_w) // 2, (H - new_h) // 2
        pts = np.array([[0.1, -0.05, 0.6], [-0.2, 0.08, 0.9]])
        u, v = g.project(pts, g.Intrinsics(fx, fy, cx, cy))
        u_z, v_z = g.project(pts, intr)
        np.testing.assert_allclose((u - x0) * W / new_w, u_z, atol=1e-9)
        np.testing.assert_allclose((v - y0) * H / new_h, v_z, atol=1e-9)


def test_intrinsics_rejects_bad_input():
    with pytest.raises(ValueError):
        g.detection_intrinsics(1, 1, 1, 1, 0.0, 0, 0)
    with pytest.raises(ValueError):
        g.detection_intrinsics(1, 1, 1, 1, 1.0, 0, 0, 0.8, W, H)
    with pytest.raises(ValueError):
        g.detection_intrinsics(1, 1, 1, 1, 1.0, 0, 0, 1.2)        #缺 width/height


def test_backproject_roundtrip():
    """已知 3D 點投影出像素與深度，反投影後應還原（誤差 < 1e-6）。"""
    rng = np.random.default_rng(2)
    pts = np.column_stack([rng.uniform(-0.4, 0.4, 100), rng.uniform(-0.3, 0.3, 100),
                           rng.uniform(0.2, 2.0, 100)])
    u, v = g.project(pts, INTR)
    back = g.backproject(u, v, pts[:, 2], INTR)
    assert np.max(np.abs(back - pts)) < 1e-6


def test_filter_keeps_only_bottle_surface():
    """深度 patch 混入 0 與背景深度，過濾後只剩瓶子表面的點。"""
    rng = np.random.default_rng(3)
    #中央區域以瓶子為主，兩側混到一點背景（框內縮後的典型情況）
    depth = np.full((40, 40), 1.20)                           #背景牆
    depth[:, 6:34] = 0.50 + rng.uniform(-0.004, 0.004, (40, 28))    #瓶子表面
    depth[::3, ::4] = 0.0                                     #破洞
    picked = g.filter_depth_points(depth, (0, 39, 0, 39), radius=0.025, min_valid_points=20)
    assert picked is not None
    u, v, z, z_med = picked
    assert np.all(np.abs(z - 0.50) < 0.005)
    assert np.all((u >= 6) & (u < 34))
    assert z_med == pytest.approx(0.50, abs=0.005)
    #瓶子區域內的有效點都要留下來
    expected = np.count_nonzero(depth[:, 6:34] > 0)
    assert z.size == expected


def test_radius_correction_is_horizontal():
    """相機 (0,0,1)、表面點 (0.5,0,0.8)、r=0.03 -> 中心 (0.53, 0, …)。"""
    xy = g.radius_correction([0.5, 0.0, 0.8], [0.0, 0.0, 1.0], 0.03)
    np.testing.assert_allclose(xy, [0.53, 0.0], atol=1e-12)

    #z 只看桌面高度 + H/2，不受表面點的 z 影響
    for p_z in (0.8, 0.3, 1.7):
        c = g.bottle_center([0.5, 0.0, p_z], [0.0, 0.0, 1.0], 0.03, z_table=0.7, height=0.1)
        np.testing.assert_allclose(c, [0.53, 0.0, 0.75], atol=1e-12)

    #相機在正上方時水平方向無法決定
    assert g.radius_correction([0.5, 0.2, 0.8], [0.5, 0.2, 1.0], 0.03) is None


def test_radius_correction_oblique_direction():
    """斜向觀看時沿水平視線方向推 r，推完到相機的水平距離正好多 r。"""
    cam = np.array([0.1, -0.3, 1.2])
    p = np.array([0.6, 0.1, 0.85])
    xy = g.radius_correction(p, cam, 0.04)
    assert np.linalg.norm(xy - cam[:2]) == pytest.approx(np.linalg.norm(p[:2] - cam[:2]) + 0.04)
    assert np.linalg.norm(xy - p[:2]) == pytest.approx(0.04)


def test_check_size():
    """步驟 7：YOLO 框和預期框比，寬、高任一項超過容許比例就擋下。"""
    expected = (100.0, 200.0, 140.0, 272.0)                #預期 40 x 72 px
    ok, reason = g.check_size((100, 200, 140, 272), expected, 0.3)
    assert ok and reason == ''
    assert g.check_size((98, 195, 143, 280), expected, 0.3)[0]          #差幾 px 照樣通過
    ok, reason = g.check_size((100, 200, 120, 272), expected, 0.3)      #框寬只剩一半
    assert not ok and '框寬' in reason
    ok, reason = g.check_size((100, 200, 140, 350), expected, 0.3)      #框高多一倍
    assert not ok and '框高' in reason
    assert not g.check_size((100, 200, 140, 272), None, 0.3)[0]


#實機內參與相機姿態（2026-10 上機量測），用來檢查廣角下畫面左右兩側的框
REAL_INTR = g.Intrinsics(364.95, 364.95, 472.73, 308.38)
REAL_ROT = g.rpy_to_matrix(0.0120, 0.5397, 0.0) @ R_BODY_FROM_OPT
REAL_CAM = np.array([0.0875, 0.1163, 0.1700])
REAL_Z_TABLE = -0.1803


@pytest.mark.parametrize('lateral', [0.12, 0.28, 0.40, -0.15])
@pytest.mark.parametrize('dims', [(0.04485, 0.08988), (0.0829, 0.0923), (0.03785, 0.07924)])
def test_expected_bbox_matches_render(lateral, dims):
    """圓柱投影的預期框和光線追蹤出來的外框一致（畫面中央到偏離 ±38 度）。"""
    dia, hgt = dims
    _, labels = render([(0.338, lateral, dia, hgt)], REAL_ROT, REAL_CAM,
                       z_table=REAL_Z_TABLE, intr=REAL_INTR)
    vs, us = np.nonzero(labels == 1)
    w, h = us.max() - us.min(), vs.max() - vs.min()
    e = g.expected_bbox((0.338, lateral), dia, hgt, REAL_Z_TABLE, REAL_ROT, REAL_CAM, REAL_INTR)
    assert abs((e[2] - e[0]) - w) / w < 0.05
    assert abs((e[3] - e[1]) - h) / h < 0.05
    assert abs((e[0] + e[2]) / 2 - (us.min() + us.max()) / 2) < 2.0      #位置也對得上


def _real_scene_localize(lateral, true_dims, used_dims):
    dia, hgt = true_dims
    depth, labels = render([(0.338, lateral, dia, hgt)], REAL_ROT, REAL_CAM,
                           z_table=REAL_Z_TABLE, intr=REAL_INTR)
    return g.localize(to_depth_mm_and_back(depth), bbox_of(labels, 1), REAL_INTR,
                      used_dims[0], used_dims[1], REAL_ROT, REAL_CAM, REAL_Z_TABLE,
                      MIN_POINTS, SIZE_RATIO)


@pytest.mark.parametrize('lateral', [0.33, 0.40, -0.15])
def test_off_center_bottle_passes_size_check(lateral):
    """實測情境：b3 放在畫面左右兩側（偏離 30~38 度），原規格的公式會擋掉，現在要通過。"""
    res, reason = _real_scene_localize(lateral, (0.04485, 0.08988), (0.04485, 0.08988))
    assert res is not None, reason
    assert np.linalg.norm(res.center[:2] - np.array([0.338, lateral])) < 0.005


def test_wrong_class_is_rejected():
    """b3 被 YOLO 認成 b1（尺寸差很多）時，合理性檢查要擋下來。"""
    res, reason = _real_scene_localize(0.33, (0.04485, 0.08988), (0.0829, 0.0923))
    assert res is None and '框寬' in reason


def test_not_enough_points_returns_none():
    depth = np.zeros((H, W))
    bbox = (400, 200, 480, 360)
    depth[280, 420:430] = 0.5                 #中央區域只有 10 個有效點
    assert g.estimate_surface_point(depth, bbox, INTR, 0.025, MIN_POINTS) is None
    res, reason = g.localize(depth, bbox, INTR, 0.05, 0.09, np.eye(3), np.zeros(3),
                             Z_TABLE, MIN_POINTS)
    assert res is None and str(MIN_POINTS) in reason


# --------------------------- 深度格式 ---------------------------
def test_depth_to_meters_mono16():
    """/depth_mm（mono16 公釐）：除以 1000，0 維持無效。"""
    raw = np.array([[0, 500, 1234], [65535, 1, 0]], dtype=np.uint16)
    out = g.depth_to_meters(raw, 'mono16')
    assert out.dtype == np.float32
    np.testing.assert_allclose(out, [[0.0, 0.5, 1.234], [65.535, 0.001, 0.0]], rtol=1e-6)
    np.testing.assert_allclose(g.depth_to_meters(raw, '16UC1'), out)


def test_depth_to_meters_32fc1():
    """ZED wrapper（32FC1 公尺）：NaN、±inf、負值都換成 0（無效）。"""
    raw = np.array([[0.5, np.nan, np.inf], [-np.inf, -0.2, 1.25]], dtype=np.float32)
    out = g.depth_to_meters(raw, '32FC1')
    np.testing.assert_allclose(out, [[0.5, 0.0, 0.0], [0.0, 0.0, 1.25]])
    assert np.isnan(raw[0, 1])                    #不改到原始陣列


def test_depth_to_meters_rejects_unknown_encoding():
    with pytest.raises(ValueError, match='不支援'):
        g.depth_to_meters(np.zeros((2, 2), np.uint8), 'mono8')


def test_synthetic_scene_from_zed_float_depth():
    """ZED wrapper 格式：float32 公尺、打不到東西的地方是 NaN，經 depth_to_meters 後照樣定位。"""
    dims = DIMS['small']
    rot, cam, cylinders, truth = scene_for(35, LATERALS['left'], dims)
    depth, labels = render(cylinders, rot, cam)
    zed = np.where(labels >= 0, depth, np.nan).astype(np.float32)
    zed[::7, ::5] = np.nan                         #ZED 量不到的零星破洞
    res, reason = run_pipeline(g.depth_to_meters(zed, '32FC1'), bbox_of(labels, 1),
                               dims, rot, cam, REFINED)
    assert res is not None, reason
    assert np.linalg.norm(res.center[:2] - truth[:2]) < 0.002


# --------------------------- 其他純函式 ---------------------------
def test_central_region_inset_and_clip():
    assert g.central_region((100, 200, 180, 280), W, H) == (120, 160, 220, 260)
    #框超出畫面時截斷在影像內
    u0, u1, v0, v1 = g.central_region((-40, -40, 40, 40), W, H)
    assert (u0, v0) == (0, 0) and (u1, v1) == (20, 20)
    assert g.central_region((10, 10, 10, 40), W, H) is None


def test_touches_edge():
    assert g.touches_edge((2, 100, 50, 200), W, H, 2)
    assert g.touches_edge((100, 100, 957, 200), W, H, 2)
    assert not g.touches_edge((3, 3, 956, 596), W, H, 2)
    #USB 模式上下各 30 列黑邊：貼著黑邊（ymin 約 30）算碰到
    assert g.touches_edge((100, 31, 200, 300), W, H, 2, letterbox_top=30)
    assert not g.touches_edge((100, 33, 200, 300), W, H, 2, letterbox_top=30)


def test_quaternion_and_inverse_roundtrip():
    rot = g.rpy_to_matrix(0.3, -0.7, 2.1)
    q = g.matrix_to_quaternion(rot)
    np.testing.assert_allclose(g.quaternion_to_matrix(*q), rot, atol=1e-12)
    trans = np.array([0.12, -0.05, 0.43])
    inv_r, inv_t = g.invert_transform(rot, trans)
    p = np.array([0.3, 0.2, -0.1])
    np.testing.assert_allclose(g.transform_point(inv_r, inv_t, g.transform_point(rot, trans, p)),
                               p, atol=1e-12)
    #pitch 往下為正：相機本體的 +X 轉向 -Z
    np.testing.assert_allclose(g.rpy_to_matrix(0, math.radians(30), 0) @ [1, 0, 0],
                               [math.cos(math.radians(30)), 0, -math.sin(math.radians(30))],
                               atol=1e-12)


def test_smoother_median_and_timeout():
    s = g.CenterSmoother(frames=3, timeout=1.0)
    s.update('b1', (0.10, 0.20), 10.0)
    s.update('b1', (0.30, 0.20), 10.1)
    np.testing.assert_allclose(s.update('b1', (0.11, 0.90), 10.2), [0.11, 0.20])
    np.testing.assert_allclose(s.update('b1', (0.12, 0.21), 10.3), [0.12, 0.21])   #最舊一筆被擠掉
    s.expire(11.0)
    assert 'b1' in s
    s.expire(11.31)
    assert 'b1' not in s
    np.testing.assert_allclose(s.update('b1', (0.5, 0.5), 11.4), [0.5, 0.5])        #重新開始


def test_parse_bottle_dims():
    names = ['b1', 'b2']
    ok = {'bottle_dims': {'b1': {'diameter': 0.05, 'height': 0.09},
                          'b2': {'diameter': 0.084, 'height': 0.093}}}
    assert g.parse_bottle_dims(ok, names) == {'b1': (0.05, 0.09), 'b2': (0.084, 0.093)}

    placeholder = {'bottle_dims': {'b1': {'diameter': 0.0, 'height': 0.0},
                                   'b2': {'diameter': 0.084, 'height': 0.093}}}
    with pytest.raises(ValueError, match='b1.diameter'):
        g.parse_bottle_dims(placeholder, names)
    with pytest.raises(ValueError, match='缺少類別 b2'):
        g.parse_bottle_dims({'bottle_dims': {'b1': {'diameter': 0.05, 'height': 0.09}}}, names)
    with pytest.raises(ValueError):
        g.parse_bottle_dims({'b1': {}}, names)


# =============================================================================
# 8.2 合成場景（無雜訊）
# =============================================================================
DIMS = {
    'small': (0.050, 0.090),
    'large': (0.084, 0.093),
}
PITCHES = (0, 20, 35, 45)
LATERALS = {'left': 0.12, 'center': 0.0, 'right': -0.12}     #+y 是 base_link 的左邊


@pytest.mark.parametrize('dims_name', sorted(DIMS))
@pytest.mark.parametrize('side', sorted(LATERALS))
@pytest.mark.parametrize('pitch', PITCHES)
@pytest.mark.parametrize('with_neighbor', [False, True])
def test_synthetic_noise_free_refined(pitch, side, dims_name, with_neighbor):
    """開啟兩個選用修正時：所有情況的水平誤差 < 2 mm，z 等於 z_table + H/2。"""
    err, _ = noise_free_error(pitch, side, dims_name, with_neighbor, REFINED)
    assert err < 0.002, f'水平誤差 {err * 1000:.2f} mm'


@pytest.mark.parametrize('dims_name', sorted(DIMS))
@pytest.mark.parametrize('side', sorted(LATERALS))
@pytest.mark.parametrize('with_neighbor', [False, True])
def test_synthetic_noise_free_spec_level_camera(side, dims_name, with_neighbor):
    """照原始規格（選用修正全關）：相機水平（俯角 0）時誤差 < 2 mm。"""
    err, _ = noise_free_error(0, side, dims_name, with_neighbor, SPEC)
    assert err < 0.002, f'水平誤差 {err * 1000:.2f} mm'


def test_synthetic_noise_free_spec_table():
    """照原始規格的完整誤差表（-s 印出）。俯角 >= 20 度時有些情況超過 2 mm，見 README。"""
    lines = []
    for with_neighbor in (False, True):
        for dims_name in sorted(DIMS):
            for pitch in PITCHES:
                cells = []
                for side in ('left', 'center', 'right'):
                    spec, _ = noise_free_error(pitch, side, dims_name, with_neighbor, SPEC)
                    refined, _ = noise_free_error(pitch, side, dims_name, with_neighbor, REFINED)
                    cells.append(f'{side:6s} {spec * 1000:5.2f} / {refined * 1000:4.2f}')
                lines.append(f'{"鄰瓶" if with_neighbor else "單瓶"} {dims_name:5s} {pitch:2d}°  '
                             + '   '.join(cells))
    print('\n無雜訊水平誤差 mm（規格 / 開啟選用修正）')
    print('\n'.join(lines))


def test_top_surface_filter_removes_cap_bias():
    """大罐、俯角 45 度、畫面中央：中央區域四成以上是瓶蓋頂面。

    規格演算法誤差 > 5 mm；只開「去瓶蓋頂面」就 < 2 mm。
    """
    spec, _ = noise_free_error(45, 'center', 'large', False, SPEC)
    fixed, _ = noise_free_error(45, 'center', 'large', False, dict(top_surface_margin=0.01))
    assert spec > 0.005
    assert fixed < 0.002


def test_symmetric_arc_removes_lateral_bias():
    """大罐、俯角 20 度、畫面左側：沒有頂面點，但取到的弧段左右不對稱。

    規格演算法誤差 > 2 mm（幾乎都在橫向）；只開「對稱弧」就 < 2 mm。
    """
    spec, _ = noise_free_error(20, 'left', 'large', False, SPEC)
    fixed, _ = noise_free_error(20, 'left', 'large', False,
                                dict(arc_iterations=3, arc_trim_pct=2.0))
    assert spec > 0.002
    assert fixed < 0.002


def test_line_of_sight_push_would_be_wrong():
    """反例：若沿完整視線方向推 r，俯角 45 度時誤差明顯大於水平推移。"""
    dims = DIMS['large']
    rot, cam, cylinders, truth = scene_for(45, 0.0, dims)
    depth, labels = render(cylinders, rot, cam)
    depth = to_depth_mm_and_back(depth)
    region = g.central_region(bbox_of(labels, 1), W, H)
    u, v, z, _ = g.filter_depth_points(depth, region, dims[0] / 2, MIN_POINTS)
    pts = g.drop_top_surface(g.backproject(u, v, z, INTR), rot, cam, Z_TABLE + dims[1] - 0.01)
    p = g.transform_point(rot, cam, np.median(pts, axis=0))
    d = p - cam
    wrong = p[:2] + dims[0] / 2 * d[:2] / np.linalg.norm(d)
    right = g.radius_correction(p, cam, dims[0] / 2)
    assert np.linalg.norm(right - truth[:2]) < 0.002
    assert np.linalg.norm(wrong - truth[:2]) > np.linalg.norm(right - truth[:2]) + 0.005


# =============================================================================
# 8.2 合成場景（加雜訊）：回報誤差分布
# =============================================================================
def add_noise(depth, labels, rng, sigma_rel=0.01, dropout=0.05, flying=True):
    """ZED 深度的簡化雜訊模型。

        sigma_rel   高斯雜訊，標準差 = sigma_rel * Z（50 cm 時 5 mm）
        dropout     隨機破洞比例（設成 0 = 無效）
        flying      物體輪廓上的飛點：邊界像素的深度改成前景與背景之間的隨機值
    """
    noisy = depth + rng.normal(0.0, 1.0, depth.shape) * sigma_rel * depth
    if flying:
        edge = np.zeros(labels.shape, dtype=bool)
        edge[:, 1:] |= labels[:, 1:] != labels[:, :-1]
        edge[1:, :] |= labels[1:, :] != labels[:-1, :]
        neighbor = np.maximum(depth, np.roll(depth, 1, axis=1))
        near = np.minimum(depth, np.roll(depth, 1, axis=1))
        mix = rng.uniform(0, 1, depth.shape)
        noisy = np.where(edge, near + mix * (neighbor - near), noisy)
    noisy[rng.uniform(0, 1, depth.shape) < dropout] = 0.0
    noisy[depth <= 0] = 0.0
    return np.clip(noisy, 0.0, None)


def jitter_bbox(bbox, rng, px=3):
    """YOLO 框的不準：每一邊各自 ±px 像素。"""
    return tuple(int(c + rng.integers(-px, px + 1)) for c in bbox)


def noise_trials(pitch, lateral, dims, neighbor, rng, trials, smooth_frames, mode):
    rot, cam, cylinders, truth = scene_for(pitch, lateral, dims, neighbor=neighbor)
    depth, labels = render(cylinders, rot, cam)
    bbox = bbox_of(labels, 1)
    single, smoothed = [], []
    rejected = 0
    for _ in range(trials):
        smoother = g.CenterSmoother(smooth_frames, timeout=1.0)
        xy = None
        for k in range(smooth_frames):
            noisy = to_depth_mm_and_back(add_noise(depth, labels, rng))
            res, _ = run_pipeline(noisy, jitter_bbox(bbox, rng), dims, rot, cam, mode)
            if res is None:
                rejected += 1
                continue
            if k == 0:
                single.append(np.linalg.norm(res.center[:2] - truth[:2]))
            xy = smoother.update('b', res.center[:2], 0.033 * k)
        if xy is not None:
            smoothed.append(np.linalg.norm(xy - truth[:2]))
    return np.array(single), np.array(smoothed), rejected


def describe(errors_m):
    mm = np.asarray(errors_m) * 1000.0
    if mm.size == 0:
        return '（沒有有效結果）'
    return (f'median {np.median(mm):5.2f}  mean {np.mean(mm):5.2f}  '
            f'p95 {np.percentile(mm, 95):5.2f}  max {np.max(mm):5.2f} mm')


def test_synthetic_scene_noise_distribution():
    """加雜訊後的誤差分布（-s 印出表格），照規格與開啟選用修正各跑一次。

    雜訊：深度 1% 高斯 + 5% 破洞 + 輪廓飛點 + 公釐截斷 + 框每邊 ±3 px。
    這是對 ZED 雜訊的粗略假設，實機誤差要上機量。
    斷言只檢查管線在雜訊下仍穩定輸出，不拿合成數字當精度保證。
    """
    trials, frames = 20, 5
    for mode_name, mode in (('照規格', SPEC), ('開啟選用修正', REFINED)):
        rng = np.random.default_rng(42)
        rows = []
        all_single, all_smoothed = [], []
        total_rejected = 0
        for dims_name in sorted(DIMS):
            for pitch in PITCHES:
                for side, lateral in LATERALS.items():
                    for neighbor in (None, DIMS['small']):
                        single, smoothed, rejected = noise_trials(
                            pitch, lateral, DIMS[dims_name], neighbor, rng, trials, frames, mode)
                        tag = (f'{dims_name:5s} 俯角 {pitch:2d}° {side:6s} '
                               f'{"鄰瓶" if neighbor else "    "}')
                        rows.append(f'{tag} | 單幀 {describe(single)} | '
                                    f'{frames} 幀中位數 {describe(smoothed)} | '
                                    f'被拒 {rejected}/{trials * frames}')
                        all_single.extend(single)
                        all_smoothed.extend(smoothed)
                        total_rejected += rejected

        print(f'\n[{mode_name}] 合成場景加雜訊（距離 45 cm，每組 {trials} 次 x {frames} 幀）')
        print('\n'.join(rows))
        print(f'[{mode_name}] 全部 單幀        {describe(all_single)}')
        print(f'[{mode_name}] 全部 {frames} 幀中位數 {describe(all_smoothed)}')
        print(f'[{mode_name}] 被拒比例 {total_rejected / (len(rows) * trials * frames):.1%}')

        assert len(all_smoothed) > 0.9 * len(rows) * trials
