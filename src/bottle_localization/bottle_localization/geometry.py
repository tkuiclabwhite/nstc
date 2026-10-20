#!/usr/bin/env python3
#coding=utf-8
"""藥罐 3D 定位的幾何計算。

本模組不依賴 ROS：輸入 numpy 陣列與數值，輸出結果，方便單獨測試。
ROS 節點（bottle_localizer.py）只負責訂閱、時間配對、TF 查詢，再呼叫這裡的函式。

座標系慣例:
    optical frame   X 右、Y 下、Z 前（相機光軸）
    base_link       X 前、Y 左、Z 上（ROS REP-103）
單位一律是公尺與像素。

流程（每個偵測框），對應 README 的步驟編號:
    detection_intrinsics()      前置：原生內參 -> 偵測框所在座標系
    central_region()            步驟 1：框內縮 1/4，取中央區域
    filter_depth_points()       步驟 2：去掉無效值與離中位數太遠的點
    backproject()               步驟 3：反投影成 optical frame 的 3D 點，取中位數
    transform_point()           步驟 4：轉到 base_link（R, t 由節點從 TF 取得）
    radius_correction()         步驟 5：沿水平方向往後推一個半徑
    bottle_center()             步驟 6：垂直位置用桌面高度 + H/2
    check_size()                步驟 7：框寬/框高與「圓柱投影的預期框」比對（已改，見函式說明）
    CenterSmoother              步驟 8：每個類別多幀取中位數
localize() 把步驟 1~7 串起來，節點與測試走的是同一條路。

另有兩個不在原始規格內的選用修正（預設關閉，見 README「選用修正」）:
    drop_top_surface()          去掉瓶蓋頂面的點（大俯角時的縱向偏差）
    symmetric_arc_center()      只用左右對稱的弧段（瓶子偏離畫面中央時的橫向偏差）
"""
import math
from collections import deque, namedtuple

import numpy as np


#偵測框所在座標系的內參（像素）
Intrinsics = namedtuple('Intrinsics', ['fx', 'fy', 'cx', 'cy'])

#步驟 1~3 的結果。point 是 optical frame 下的表面點（公尺）
SurfaceEstimate = namedtuple('SurfaceEstimate', ['point', 'z_med', 'n_points'])

#localize() 的結果。center 是 base_link 下的瓶身中心（公尺）
LocalizeResult = namedtuple(
    'LocalizeResult',
    ['center', 'surface_optical', 'surface_base', 'z_med', 'n_points'])

#步驟 1：框的四邊各內縮這個比例
INSET_FRAC = 0.25


# =============================================================================
# 前置：內參換算
# =============================================================================
def detection_intrinsics(fx, fy, cx, cy, scale, letterbox_left, letterbox_top,
                         zoom=1.0, width=None, height=None):
    """把相機原生 camera_info 的內參換算到偵測框（= 深度圖）所在的座標系。

    兩段換算，順序與上游一致:

    1. 等比縮放 s，再補黑邊（bottle_detect.py 的 map_to_strategy()）::

           fx' = s * fx        cx' = s * cx + letterbox_left
           fy' = s * fy        cy' = s * cy + letterbox_top

       ZED X Mini 發布的影像本身就是 960x600，s = 1、黑邊 = 0，這一段是恆等變換。

    2. 數位變焦（depth_process_node 的寫法）：中心裁成 int(W/z) x int(H/z)，
       再放大回 W x H。裁切原點 (x0, y0)，放大倍率 (W/new_w, H/new_h)::

           fx'' = fx' * W / new_w      cx'' = (cx' - x0) * W / new_w
           fy'' = fy' * H / new_h      cy'' = (cy' - y0) * H / new_h

       zoom = 1 時 new_w = W、x0 = 0，同樣是恆等變換。

    Args:
        fx, fy, cx, cy (float): 原生 camera_info 的 K。
        scale (float): 原生 -> 偵測座標系的縮放比 s。
        letterbox_left, letterbox_top (int): 左側/上方黑邊的行數/列數。
        zoom (float): 數位變焦倍率，>= 1。
        width, height (int): 偵測座標系的影像寬高。zoom != 1 時必填。

    Returns:
        Intrinsics: 偵測座標系的內參。
    """
    if scale <= 0:
        raise ValueError(f'scale 必須 > 0，收到 {scale}')
    if zoom < 1.0:
        raise ValueError(f'zoom 必須 >= 1，收到 {zoom}')

    fx1 = scale * fx
    fy1 = scale * fy
    cx1 = scale * cx + letterbox_left
    cy1 = scale * cy + letterbox_top
    if zoom == 1.0:
        return Intrinsics(fx1, fy1, cx1, cy1)

    if width is None or height is None:
        raise ValueError('zoom != 1 時必須給偵測座標系的 width / height')
    new_w = max(1, int(width / zoom))
    new_h = max(1, int(height / zoom))
    x0 = (width - new_w) // 2
    y0 = (height - new_h) // 2
    sx = width / float(new_w)
    sy = height / float(new_h)
    return Intrinsics(fx1 * sx, fy1 * sy, (cx1 - x0) * sx, (cy1 - y0) * sy)


def project(points, intr):
    """optical frame 的 3D 點投影成像素。

    Args:
        points (np.ndarray): (N, 3)。
        intr (Intrinsics): 內參。

    Returns:
        tuple: (u, v)，各為 (N,)。
    """
    p = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    u = intr.fx * p[:, 0] / p[:, 2] + intr.cx
    v = intr.fy * p[:, 1] / p[:, 2] + intr.cy
    return u, v


# =============================================================================
# 前置：深度格式
# =============================================================================
#編碼 -> 換成公尺要乘的倍數
DEPTH_SCALE_TO_M = {
    '32FC1': 1.0,       #ZED wrapper 的 depth_registered，公尺；無效是 NaN / ±inf
    'mono16': 0.001,    #depth_process_node 的 /depth_mm，公釐；無效是 0
    '16UC1': 0.001,     #ZED wrapper 開 openni_depth_mode 時，公釐；無效是 0
}


def depth_to_meters(depth, encoding):
    """深度圖統一換成公尺（float32），無效值一律變成 0。

    Args:
        depth (np.ndarray): (H, W) 原始深度圖。
        encoding (str): sensor_msgs/Image 的 encoding。

    Returns:
        np.ndarray: (H, W) float32，公尺；NaN、±inf、<= 0 都是 0。
    """
    scale = DEPTH_SCALE_TO_M.get(encoding)
    if scale is None:
        raise ValueError(f'不支援的深度編碼 {encoding!r}，'
                         f'只支援 {", ".join(DEPTH_SCALE_TO_M)}')
    out = np.asarray(depth, dtype=np.float32) * np.float32(scale)
    out[~np.isfinite(out) | (out < 0)] = 0.0
    return out


# =============================================================================
# 前置：邊緣檢查
# =============================================================================
def touches_edge(bbox, width, height, margin, letterbox_left=0, letterbox_top=0):
    """框是否碰到畫面邊緣（或黑邊），碰到代表瓶子可能被切掉，框不完整。

    同 med_detect.py 的 touches_edge()：真正的影像範圍要扣掉黑邊，
    否則貼著黑邊的框會被誤判成沒碰到邊緣。黑邊視為上下、左右對稱。

    Args:
        bbox (tuple): (xmin, ymin, xmax, ymax)，像素，含端點。
        width, height (int): 偵測座標系的影像寬高。
        margin (int): 距離邊緣幾個像素內算碰到。
    """
    xmin, ymin, xmax, ymax = bbox
    left = letterbox_left
    right = width - 1 - letterbox_left
    top = letterbox_top
    bottom = height - 1 - letterbox_top
    return (xmin <= left + margin or ymin <= top + margin or
            xmax >= right - margin or ymax >= bottom - margin)


# =============================================================================
# 步驟 1 ~ 3：框內深度 -> optical frame 的表面點
# =============================================================================
def central_region(bbox, width, height):
    """步驟 1：框的四邊各內縮 1/4，取中央區域，並截斷在影像內。

    這裡是**內縮**，跟舊程式的 expand_bbox() 相反：量尺寸需要整個物體的邊緣，
    求位置只需要一塊可靠的正面，內縮可以避開邊緣的飛點與旁邊的背景。

    Returns:
        tuple: (u0, u1, v0, v1)，含端點的整數範圍；範圍是空的時回 None。
    """
    x1, y1, x2, y2 = bbox
    w = x2 - x1
    h = y2 - y1
    if w <= 0 or h <= 0:
        return None
    u0 = max(0, int(math.ceil(x1 + w * INSET_FRAC)))
    u1 = min(width - 1, int(math.floor(x2 - w * INSET_FRAC)))
    v0 = max(0, int(math.ceil(y1 + h * INSET_FRAC)))
    v1 = min(height - 1, int(math.floor(y2 - h * INSET_FRAC)))
    if u1 < u0 or v1 < v0:
        return None
    return u0, u1, v0, v1


def filter_depth_points(depth_m, region, radius, min_valid_points):
    """步驟 2：在中央區域裡挑出瓶子表面的深度點。

    1. 去掉無效值（0）
    2. 算剩餘點的 Z 中位數 Z_med
    3. 只留 |Z - Z_med| < radius 的點（radius = D/2）
    4. 剩下的點少於 min_valid_points 時回 None

    Args:
        depth_m (np.ndarray): (H, W) 深度圖，公尺，0 代表無效。索引是 depth[v, u]。
        region (tuple): central_region() 的結果。

    Returns:
        tuple: (u, v, z, z_med)，前三者是 (N,) 陣列；點數不足時回 None。
    """
    u0, u1, v0, v1 = region
    patch = depth_m[v0:v1 + 1, u0:u1 + 1]
    rows, cols = np.nonzero(np.isfinite(patch) & (patch > 0))
    if rows.size == 0:
        return None
    z = patch[rows, cols].astype(np.float64)
    z_med = float(np.median(z))
    keep = np.abs(z - z_med) < radius
    if int(np.count_nonzero(keep)) < min_valid_points:
        return None
    u = (cols[keep] + u0).astype(np.float64)
    v = (rows[keep] + v0).astype(np.float64)
    return u, v, z[keep], z_med


def backproject(u, v, z, intr):
    """步驟 3：像素 + 深度 -> optical frame 的 3D 點（小孔成像的反向）::

        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy

    前提是深度值是「沿光軸的 Z」。ZED 的 MEASURE::DEPTH 就是，
    depth_process_node 也沒有轉成徑向距離。

    Returns:
        np.ndarray: (N, 3)，公尺。
    """
    u = np.asarray(u, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    z = np.asarray(z, dtype=np.float64)
    return np.stack([(u - intr.cx) * z / intr.fx,
                     (v - intr.cy) * z / intr.fy,
                     z], axis=-1)


def estimate_surface_point(depth_m, bbox, intr, radius, min_valid_points):
    """步驟 1~3：由框與深度圖求瓶子正面的表面點 P_s（optical frame）。

    Returns:
        SurfaceEstimate: 有效點數不足時回 None。
    """
    h, w = depth_m.shape[:2]
    region = central_region(bbox, w, h)
    if region is None:
        return None
    picked = filter_depth_points(depth_m, region, radius, min_valid_points)
    if picked is None:
        return None
    u, v, z, z_med = picked
    points = backproject(u, v, z, intr)
    return SurfaceEstimate(np.median(points, axis=0), z_med, int(points.shape[0]))


# =============================================================================
# 步驟 4：座標轉換（純 numpy，節點把 TF 換成 R, t 再傳進來）
# =============================================================================
def quaternion_to_matrix(x, y, z, w):
    """單位四元數 -> 3x3 旋轉矩陣。"""
    q = np.array([x, y, z, w], dtype=np.float64)
    n = float(np.linalg.norm(q))
    if n == 0.0:
        raise ValueError('四元數長度為 0')
    x, y, z, w = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(rot):
    """3x3 旋轉矩陣 -> 四元數 (x, y, z, w)，w >= 0。"""
    m = np.asarray(rot, dtype=np.float64)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = 2.0 * math.sqrt(tr + 1.0)
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return tuple(float(c) for c in q)


def rpy_to_matrix(roll, pitch, yaw):
    """roll/pitch/yaw（弧度）-> 旋轉矩陣。

    與 static_transform_publisher、URDF 的 rpy 同一個慣例：
    繞固定軸依序 X(roll) -> Y(pitch) -> Z(yaw)，即 R = Rz(yaw) * Ry(pitch) * Rx(roll)。
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def invert_transform(rot, trans):
    """(R, t) 的反轉換：p_a = R p_b + t  ->  p_b = R^T p_a - R^T t。"""
    r = np.asarray(rot, dtype=np.float64)
    t = np.asarray(trans, dtype=np.float64)
    return r.T, -r.T @ t


def transform_point(rot, trans, point):
    """p_target = R p_source + t。"""
    return np.asarray(rot, dtype=np.float64) @ np.asarray(point, dtype=np.float64) + \
        np.asarray(trans, dtype=np.float64)


# =============================================================================
# 步驟 5 ~ 6：表面點 -> 瓶身中心
# =============================================================================
def radius_correction(surface_base, camera_base, radius):
    """步驟 5：相機只看得到瓶子正面，P_s 在表面上，沿**水平方向**往後推一個半徑才是中軸::

        d   = P_s - C
        d_h = (d.x, d.y, 0)
        center.xy = P_s.xy + r * d_h / |d_h|

    不可以沿完整視線方向 d 推：相機往下看時 d 帶有垂直分量，
    水平推移會不足、垂直方向也會被推錯。

    Returns:
        np.ndarray: (2,) 中軸的 (x, y)；相機剛好在正上方（|d_h| = 0）時回 None。
    """
    p = np.asarray(surface_base, dtype=np.float64)
    c = np.asarray(camera_base, dtype=np.float64)
    d_h = p[:2] - c[:2]
    n = float(np.linalg.norm(d_h))
    if n == 0.0:
        return None
    return p[:2] + radius * d_h / n


def bottle_center(surface_base, camera_base, radius, z_table, height):
    """步驟 5 + 6：瓶身中心（base_link）。

    垂直位置不使用深度估計值，直接用桌面高度 + H/2 —— 藥罐直立放在固定高度的桌面上。

    Returns:
        np.ndarray: (3,)；radius_correction() 失敗時回 None。
    """
    xy = radius_correction(surface_base, camera_base, radius)
    if xy is None:
        return None
    return np.array([xy[0], xy[1], z_table + height / 2.0])


# =============================================================================
# 步驟 7：合理性檢查
# =============================================================================
def relative_error(estimate, actual):
    """|估計 - 實際| / 實際。"""
    return abs(estimate - actual) / abs(actual)


#圓柱上下兩圈各取幾個點來投影；72 點時外接矩形的誤差不到半徑的 0.1%
CYLINDER_RING_POINTS = 72


def expected_bbox(center_xy, diameter, height, z_table, rot, trans, intr):
    """把直徑 D、高 H、直立在桌面上的圓柱投影回影像，回傳預期的外接矩形。

    圓柱是上下兩個圓的凸包，凸包投影後仍是兩圈投影點的凸包，
    所以外接矩形只要看兩圈點投影的最小 / 最大值。透視、低頭角度、偏離畫面中心
    （廣角時越靠左右兩側框越寬）都自動包含在內。

    Args:
        center_xy: 瓶身中軸在 base_link 的 (x, y)。
        rot, trans: optical -> base_link（p_base = rot @ p_opt + trans）。

    Returns:
        (u0, v0, u1, v1)，像素；有點在相機後方時回 None。
    """
    a = np.linspace(0.0, 2.0 * np.pi, CYLINDER_RING_POINTS, endpoint=False)
    ring = np.column_stack([diameter / 2.0 * np.cos(a), diameter / 2.0 * np.sin(a)]) + \
        np.asarray(center_xy, dtype=np.float64)[:2]
    n = CYLINDER_RING_POINTS
    pts = np.vstack([np.column_stack([ring, np.full(n, z_table)]),
                     np.column_stack([ring, np.full(n, z_table + height)])])
    p_opt = (pts - np.asarray(trans, dtype=np.float64)) @ np.asarray(rot, dtype=np.float64)
    if np.any(p_opt[:, 2] <= 0.0):
        return None
    u, v = project(p_opt, intr)
    return float(u.min()), float(v.min()), float(u.max()), float(v.max())


def check_size(bbox, expected, ratio):
    """步驟 7：YOLO 框的寬、高和「圓柱投影的預期框」（expected_bbox()）比對。

    （偏離原始規格）原規格的公式是
        Z_size = fx' * D / w 對 Z_med、 h_expect = fy' * H / Z_med 對 h
    只在畫面中央成立。ZED 是廣角，瓶子放在畫面左右兩側時框會被透視拉寬：
    實測 b3（D 4.485 cm）在偏離中心約 30 度的位置框寬 74 px，原公式的相對誤差 0.37 > 0.3，正確的瓶子被擋掉；
    只乘 1/cos(偏角) 也修不乾淨（相機往下看，透視不只左右方向）。
    改成把圓柱放在算出來的中心、投影回影像得到預期的框，合成測試在偏離 ±38 度內誤差 3% 以內。
    類別認錯（例如把 b3 當成 b1）時框寬誤差約 60%，照樣擋得下來。

    Args:
        bbox: YOLO 的框 (xmin, ymin, xmax, ymax)。
        expected: expected_bbox() 的結果。
        ratio (float): 寬、高任一項的相對誤差（|預期 - 框| / 框）超過就不通過。

    Returns:
        tuple: (ok, reason)。不通過時 reason 說明是哪一項、數值多少；通過時是 ''。
    """
    w = bbox[2] - bbox[0]
    h = bbox[3] - bbox[1]
    if w <= 0 or h <= 0:
        return False, f'框不合法 (w={w}, h={h})'
    if expected is None:
        return False, '預期的框有一部分在相機後方'
    ew = expected[2] - expected[0]
    eh = expected[3] - expected[1]
    err_w = relative_error(ew, w)
    if err_w > ratio:
        return False, (f'框寬不符: 預期 {ew:.1f} px vs 框 {w} px '
                       f'(相對誤差 {err_w:.2f} > {ratio})')
    err_h = relative_error(eh, h)
    if err_h > ratio:
        return False, (f'框高不符: 預期 {eh:.1f} px vs 框 {h} px '
                       f'(相對誤差 {err_h:.2f} > {ratio})')
    return True, ''


def in_workspace(x, y, x_min, x_max, y_min, y_max):
    """中心 (x, y) 是否落在工作區域內（含邊界）。"""
    return x_min <= x <= x_max and y_min <= y <= y_max


# =============================================================================
# 選用的修正（不在原始規格內，預設關閉；理由與合成測試結果見 README）
# =============================================================================
def drop_top_surface(points, rot, trans, z_limit):
    """（選用）去掉瓶蓋頂面上的點。

    相機往下看時，框的中央區域會包含瓶蓋頂面。頂面上的點離相機的距離和正面差不多，
    步驟 2 的 |Z - Z_med| < r 濾不掉，但它們的水平位置在正面後方（中軸附近），
    會把 P_s 的中位數往後拉，推一個半徑之後中心就偏遠。
    合成測試：D = 8.4 cm、俯角 45 度時中央區域有四成以上是頂面，誤差約 8 mm。

    頂面在 base_link 下的高度已知（z_table + H），所以把點轉到 base_link，
    只留 z < z_limit 的點；點本身仍是 optical frame，後續步驟不變。

    Args:
        points (np.ndarray): (N, 3) optical frame 的點。
        rot, trans: optical -> base_link。
        z_limit (float): base_link 下的高度上限，通常是 z_table + H - margin。

    Returns:
        np.ndarray: 留下來的 optical frame 點。
    """
    z_base = points @ np.asarray(rot, dtype=np.float64)[2] + float(trans[2])
    return points[z_base < z_limit]


def symmetric_arc_center(points, rot, trans, radius, center_xy, iterations, trim_pct,
                         min_valid_points):
    """（選用）只用「正對相機方向」左右對稱的弧段重算中心，消除橫向偏差。

    相機往下看、瓶子又不在畫面正中時，瓶子在影像裡是斜的，框的中央區域
    取到的弧段左右不對稱，P_s 的中位數就偏向一側，步驟 5 推完半徑後中心
    在橫向（垂直視線的水平方向）偏掉。合成測試：俯角 45 度、偏離中央 15 度時
    取到的弧段中位角度偏 6 度，橫向誤差約 4 mm。

    作法（迭代）：以目前的中心估計，算每個點繞中軸的水平角度（0 = 正對相機），
    取兩側較窄的那一邊當半寬（兩端各去掉 trim_pct 百分位抵抗雜點），
    只留角度在 ±半寬內的點，重新取中位數 -> 步驟 5 -> 新的中心。

    Args:
        points (np.ndarray): (N, 3) optical frame 的點（步驟 2、3 之後）。
        rot, trans: optical -> base_link；trans 是相機位置 C。
        center_xy (np.ndarray): 步驟 5 的初始中心 (x, y)。
        iterations (int): 迭代次數。
        trim_pct (float): 決定半寬時兩端各去掉的百分位。

    Returns:
        np.ndarray: (2,) 修正後的中心 (x, y)。
    """
    rot = np.asarray(rot, dtype=np.float64)
    trans = np.asarray(trans, dtype=np.float64)
    xy = (points @ rot.T + trans)[:, :2]
    cam_xy = trans[:2]
    center = np.asarray(center_xy, dtype=np.float64)
    for _ in range(iterations):
        los = center - cam_xy
        n = float(np.linalg.norm(los))
        if n == 0.0:
            break
        los /= n
        tan = np.array([-los[1], los[0]])
        rel = xy - center
        ang = np.arctan2(rel @ tan, -(rel @ los))
        half = min(-np.percentile(ang, trim_pct), np.percentile(ang, 100.0 - trim_pct))
        if half <= 0.0:
            break
        sel = np.abs(ang) <= half
        if int(np.count_nonzero(sel)) < min_valid_points:
            break
        p_s = transform_point(rot, trans, np.median(points[sel], axis=0))
        new_center = radius_correction(p_s, trans, radius)
        if new_center is None:
            break
        center = new_center
    return center


# =============================================================================
# 步驟 1 ~ 7 串起來
# =============================================================================
def localize(depth_m, bbox, intr, diameter, height, rot, trans, z_table,
             min_valid_points, size_check_ratio=None, workspace=None,
             top_surface_margin=None, arc_iterations=0, arc_trim_pct=0.0):
    """單一偵測框走完步驟 1~7。

    Args:
        depth_m (np.ndarray): (H, W) 深度圖，公尺，0 代表無效。
        bbox (tuple): (xmin, ymin, xmax, ymax)，偵測座標系。
        intr (Intrinsics): detection_intrinsics() 換算後的內參。
        diameter, height (float): 這一類藥罐的直徑 D、高 H（公尺）。
        rot, trans: optical frame -> base_link 的轉換，p_base = rot @ p_opt + trans。
            trans 同時就是相機在 base_link 下的位置 C。
        z_table (float): 桌面在 base_link 下的高度。
        min_valid_points (int): 框內有效深度點的最少數量。
        size_check_ratio (float): 合理性檢查的容許比例；None 表示不檢查。
        workspace (tuple): (x_min, x_max, y_min, y_max)；None 表示不過濾。
        top_surface_margin (float): 選用。不是 None 時啟用 drop_top_surface()，
            去掉 base_link 高度 >= z_table + H - margin 的點。
        arc_iterations (int): 選用。> 0 時啟用 symmetric_arc_center()。
        arc_trim_pct (float): symmetric_arc_center() 的 trim_pct。
        兩個選用修正都關閉時，就是原始規格的演算法。

    Returns:
        tuple: (LocalizeResult, '') 或 (None, 跳過原因)。
    """
    radius = diameter / 2.0

    #步驟 1 ~ 3
    img_h, img_w = depth_m.shape[:2]
    region = central_region(bbox, img_w, img_h)
    picked = None if region is None else \
        filter_depth_points(depth_m, region, radius, min_valid_points)
    if picked is None:
        return None, f'框內有效深度點少於 {min_valid_points}'
    u, v, z, z_med = picked
    points = backproject(u, v, z, intr)
    if top_surface_margin is not None:
        points = drop_top_surface(points, rot, trans, z_table + height - top_surface_margin)
        if points.shape[0] < min_valid_points:
            return None, f'去掉瓶蓋頂面後有效深度點少於 {min_valid_points}'
    surface_opt = np.median(points, axis=0)

    #步驟 4 ~ 6
    surface_base = transform_point(rot, trans, surface_opt)
    center = bottle_center(surface_base, trans, radius, z_table, height)
    if center is None:
        return None, '相機在瓶子正上方，水平方向無法決定'
    if arc_iterations > 0:
        center[:2] = symmetric_arc_center(points, rot, trans, radius, center[:2],
                                          arc_iterations, arc_trim_pct, min_valid_points)

    #步驟 7：把圓柱放在算出來的中心，投影回影像和 YOLO 的框比
    if size_check_ratio is not None:
        ok, reason = check_size(
            bbox, expected_bbox(center[:2], diameter, height, z_table, rot, trans, intr),
            size_check_ratio)
        if not ok:
            return None, reason

    if workspace is not None and not in_workspace(center[0], center[1], *workspace):
        return None, f'中心 ({center[0]:.3f}, {center[1]:.3f}) 在工作區域外'

    return LocalizeResult(center, surface_opt, surface_base, z_med, int(points.shape[0])), ''


# =============================================================================
# 步驟 8：多幀平滑
# =============================================================================
class CenterSmoother:
    """每個類別一個長度 N 的緩衝，存 base_link 下的中心 (x, y)，發布值取中位數。

    某類別超過 timeout 秒沒有新的有效偵測，就清空它的緩衝，
    避免瓶子被拿走後還拿舊的位置去平均。時間用影像的 stamp（秒）。
    """

    def __init__(self, frames, timeout):
        if frames < 1:
            raise ValueError(f'frames 必須 >= 1，收到 {frames}')
        self.frames = int(frames)
        self.timeout = float(timeout)
        self._buf = {}
        self._last = {}

    def expire(self, now):
        """清掉超過 timeout 沒更新的類別。"""
        for label in list(self._last):
            if now - self._last[label] > self.timeout:
                del self._last[label]
                del self._buf[label]

    def update(self, label, xy, now):
        """加入一筆新的中心，回傳平滑後的 (x, y)。"""
        buf = self._buf.setdefault(label, deque(maxlen=self.frames))
        buf.append((float(xy[0]), float(xy[1])))
        self._last[label] = now
        return np.median(np.asarray(buf), axis=0)

    def __contains__(self, label):
        return label in self._buf


# =============================================================================
# 設定檔
# =============================================================================
def parse_bottle_dims(data, class_names):
    """檢查並取出 bottle_dims.yaml 的內容。

    尺寸 <= 0 或缺少某個類別時直接報錯，列出所有問題，不要靜默使用錯誤數值。

    Args:
        data (dict): yaml.safe_load() 的結果，格式
            {'bottle_dims': {'b1': {'diameter': .., 'height': ..}, ...}}。
        class_names (list): YOLO 的類別名稱，每一個都必須有尺寸。

    Returns:
        dict: {類別名稱: (diameter, height)}，單位公尺。
    """
    if not isinstance(data, dict) or not isinstance(data.get('bottle_dims'), dict):
        raise ValueError('bottle_dims.yaml 必須有頂層的 bottle_dims: 區塊')
    table = data['bottle_dims']

    problems = []
    dims = {}
    for name in class_names:
        entry = table.get(name)
        if not isinstance(entry, dict):
            problems.append(f'缺少類別 {name}')
            continue
        values = {}
        for key in ('diameter', 'height'):
            raw = entry.get(key)
            try:
                value = float(raw)
            except (TypeError, ValueError):
                problems.append(f'{name}.{key} 不是數字: {raw!r}')
                continue
            if not math.isfinite(value) or value <= 0.0:
                problems.append(f'{name}.{key} 必須 > 0，目前是 {raw}')
                continue
            values[key] = value
        if len(values) == 2:
            dims[name] = (values['diameter'], values['height'])

    extra = sorted(set(table) - set(class_names))
    if extra:
        problems.append(f'有 class_names 裡沒有的類別: {", ".join(map(str, extra))}')

    if problems:
        raise ValueError('bottle_dims.yaml 有問題:\n  ' + '\n  '.join(problems))
    return dims
