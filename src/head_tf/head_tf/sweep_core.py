#!/usr/bin/env python3
"""頭部掃描資料收集用的純計算函式。不依賴 ROS，可以在桌機上測試。

座標慣例和 bottle_localization 相同：
- optical frame：X 右、Y 下、Z 前，深度圖反投影出來的點在這個座標系。
- 相機本體：X 前、Y 左、Z 上。
- 相機相對桌面的 roll / pitch：姿態 R = Ry(pitch) Rx(roll)，往下看 pitch 為正（同 table_plane.py）。

桌面擬合、瓶子表面點的算法照抄 bottle_localization（tools/table_plane.py、geometry.py），
因為那兩支不是能 import 的模組，而且機器人上裝的版本可能和桌機不同。
"""
import math
import warnings
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# 馬達
# ---------------------------------------------------------------------------
TICKS_PER_REV = 4096
DEG_PER_TICK = 360.0 / TICKS_PER_REV

# /Head_Topic（HeadPackage）的 id；driver_node 的 head_map 把 1 -> 28 號、2 -> 29 號
HEAD_PAN_ID = 1
HEAD_TILT_ID = 2
# /joint_states 的 name 是馬達 ID 字串，position 是原始刻度
PAN_JOINT = '28'
TILT_JOINT = '29'

# 現在的 camera_extrinsics.yaml 是在這個頭部刻度校正的
REFERENCE_TICKS = (2048, 2300)

# 收集工具允許下達的刻度：基準 ±341 格（約 ±30°）。打錯數字時擋下來，避免頭撞到機構。
LIMIT_HALF_TICKS = 341
PAN_LIMITS = (REFERENCE_TICKS[0] - LIMIT_HALF_TICKS, REFERENCE_TICKS[0] + LIMIT_HALF_TICKS)
TILT_LIMITS = (REFERENCE_TICKS[1] - LIMIT_HALF_TICKS, REFERENCE_TICKS[1] + LIMIT_HALF_TICKS)

# HeadPackage.speed 直接寫進 Dynamixel 的 Profile Velocity（X 系列 1 = 0.229 rpm）。
# 0 代表「不限速」，頭會用最快速度甩過去，所以不允許。
SPEED_LIMITS = (1, 200)


def ticks_to_deg(ticks):
    return ticks * DEG_PER_TICK


def deg_to_ticks(deg):
    return int(round(deg / DEG_PER_TICK))


def build_postures(pan_ticks, tilt_ticks, reference=REFERENCE_TICKS):
    """收集順序：基準姿勢 -> 網格（蛇形走，少轉一點）-> 基準姿勢。

    頭尾各量一次基準姿勢，比較兩次的結果，就知道中間機器人、桌子或瓶子有沒有動過。

    Returns:
        list: [(kind, pan, tilt), ...]，kind 是 'ref' 或 'grid'。
    """
    pans = sorted(set(int(p) for p in pan_ticks))
    tilts = sorted(set(int(t) for t in tilt_ticks))
    if not pans or not tilts:
        raise ValueError('pan 和 tilt 至少各要一個刻度')
    ref = (int(reference[0]), int(reference[1]))
    postures = [('ref',) + ref]
    for i, tilt in enumerate(tilts):
        row = pans if i % 2 == 0 else pans[::-1]
        postures += [('grid', p, tilt) for p in row]
    postures.append(('ref',) + ref)
    return postures


def check_postures(postures, pan_limits=PAN_LIMITS, tilt_limits=TILT_LIMITS):
    """有刻度超出允許範圍就丟 ValueError，訊息列出是哪幾個。"""
    bad = []
    for _, pan, tilt in postures:
        if not pan_limits[0] <= pan <= pan_limits[1]:
            bad.append(f'pan {pan}')
        if not tilt_limits[0] <= tilt <= tilt_limits[1]:
            bad.append(f'tilt {tilt}')
    if bad:
        raise ValueError(
            f'刻度超出允許範圍（pan {pan_limits[0]}~{pan_limits[1]}、'
            f'tilt {tilt_limits[0]}~{tilt_limits[1]}）：{", ".join(sorted(set(bad)))}')


def check_speed(speed):
    if not SPEED_LIMITS[0] <= int(speed) <= SPEED_LIMITS[1]:
        raise ValueError(f'速度要在 {SPEED_LIMITS[0]}~{SPEED_LIMITS[1]}（0 是不限速，不允許）')


def is_still(samples, window_s, tol_ticks, now):
    """最近 window_s 秒內兩顆馬達的刻度變化都不超過 tol_ticks，就算停穩。

    samples: [(t, pan, tilt), ...]。最近一段的資料要涵蓋視窗的八成以上才算數，
    避免剛開始只有一兩筆就誤判。
    """
    recent = [s for s in samples if s[0] >= now - window_s]
    if len(recent) < 3 or recent[-1][0] - recent[0][0] < 0.8 * window_s:
        return False
    a = np.asarray(recent, dtype=np.float64)[:, 1:]
    return bool(np.all(a.max(axis=0) - a.min(axis=0) <= tol_ticks))


def joint_stats(samples):
    """記錄期間的刻度統計。samples: [(t, pan, tilt), ...]。"""
    out = {'joint_n': len(samples)}
    if not samples:
        for name in ('pan', 'tilt'):
            for s in ('mean', 'std', 'min', 'max'):
                out[f'{name}_{s}'] = float('nan')
        return out
    a = np.asarray(samples, dtype=np.float64)
    for col, name in ((1, 'pan'), (2, 'tilt')):
        v = a[:, col]
        out[f'{name}_mean'] = float(v.mean())
        out[f'{name}_std'] = float(v.std())
        out[f'{name}_min'] = float(v.min())
        out[f'{name}_max'] = float(v.max())
    return out


# ---------------------------------------------------------------------------
# 影像
# ---------------------------------------------------------------------------
DEPTH_SCALE = {'32FC1': 1.0, '16UC1': 0.001, 'mono16': 0.001}
# encoding -> (每像素幾個 byte, 轉成 BGR 的通道順序)
COLOR_LAYOUT = {'bgra8': (4, [0, 1, 2]), 'bgr8': (3, [0, 1, 2]),
                'rgba8': (4, [2, 1, 0]), 'rgb8': (3, [2, 1, 0])}


def depth_to_array(encoding, height, width, step, data):
    """sensor_msgs/Image 的欄位 -> (H, W) float32 公尺，無效值為 NaN。"""
    if encoding not in DEPTH_SCALE:
        raise ValueError(f'不支援的深度編碼 {encoding}')
    dtype = np.float32 if encoding == '32FC1' else np.uint16
    row = step // np.dtype(dtype).itemsize
    d = np.frombuffer(bytes(data), dtype=dtype).reshape(height, row)[:, :width]
    d = d.astype(np.float32) * np.float32(DEPTH_SCALE[encoding])
    d[~np.isfinite(d) | (d <= 0)] = np.nan
    return d


def color_to_bgr(encoding, height, width, step, data):
    """sensor_msgs/Image 的欄位 -> (H, W, 3) uint8 BGR；不認得的編碼回 None。"""
    if encoding not in COLOR_LAYOUT:
        return None
    ch, order = COLOR_LAYOUT[encoding]
    img = np.frombuffer(bytes(data), dtype=np.uint8).reshape(height, step)[:, :width * ch]
    return np.ascontiguousarray(img.reshape(height, width, ch)[:, :, order])


def median_depth(frames):
    """多張深度圖逐像素取中位數，壓掉雜訊。全部無效的像素保持 NaN。"""
    stack = np.stack(frames).astype(np.float32)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        return np.nanmedian(stack, axis=0).astype(np.float32)


# ---------------------------------------------------------------------------
# 瓶子表面點（同 bottle_localization geometry.py 的步驟 1~3）
# ---------------------------------------------------------------------------
INSET_FRAC = 0.25
# 只留深度中位數前後這麼多以內的點。原程式用瓶子半徑，這裡不讀 bottle_dims.yaml，固定用 3 cm
SURFACE_BAND_M = 0.03
MIN_SURFACE_POINTS = 20


def central_region(bbox, width, height):
    """框的四邊各內縮 1/4，回 (u0, u1, v0, v1)（含端點）；範圍是空的時回 None。"""
    x1, y1, x2, y2 = bbox
    w, h = x2 - x1, y2 - y1
    if w <= 0 or h <= 0:
        return None
    u0 = max(0, int(math.ceil(x1 + w * INSET_FRAC)))
    u1 = min(width - 1, int(math.floor(x2 - w * INSET_FRAC)))
    v0 = max(0, int(math.ceil(y1 + h * INSET_FRAC)))
    v1 = min(height - 1, int(math.floor(y2 - h * INSET_FRAC)))
    if u1 < u0 or v1 < v0:
        return None
    return u0, u1, v0, v1


def surface_point(depth, bbox, k, band=SURFACE_BAND_M, min_points=MIN_SURFACE_POINTS):
    """瓶子正面的表面點（optical frame）。

    Returns:
        (point, z_med, n)：point 是 (3,) 公尺；有效點不夠時回 None。
    """
    h, w = depth.shape
    region = central_region(bbox, w, h)
    if region is None:
        return None
    u0, u1, v0, v1 = region
    patch = depth[v0:v1 + 1, u0:u1 + 1]
    rows, cols = np.nonzero(np.isfinite(patch))
    if rows.size == 0:
        return None
    z = patch[rows, cols].astype(np.float64)
    z_med = float(np.median(z))
    keep = np.abs(z - z_med) < band
    if int(keep.sum()) < min_points:
        return None
    fx, fy, cx, cy = k
    u = cols[keep] + u0
    v = rows[keep] + v0
    z = z[keep]
    pts = np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z])
    return np.median(pts, axis=0), z_med, int(pts.shape[0])


# ---------------------------------------------------------------------------
# 桌面平面（同 bottle_localization tools/table_plane.py）
# ---------------------------------------------------------------------------
# optical frame 的軸在相機本體座標裡的方向
R_BODY_FROM_OPT = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0]])
# 桌面擬合區（畫面高、寬的比例）。table_plane.py 用 70~95% / 25~75%，但那塊太小、離相機太近，
# sweep_01 實測深度差 1 mm 角度就差 0.6°；放大到這裡之後同一個姿勢頭尾只差 0.2°
PLANE_ROWS = (0.60, 0.98)
PLANE_COLS = (0.20, 0.80)
PLANE_THRESHOLD_M = 0.004
# 擬合桌面時，瓶子的框往外多挖掉幾個像素
BOX_MARGIN_PX = 15
MIN_PLANE_POINTS = 500


def plane_points(depth, k, rows=PLANE_ROWS, cols=PLANE_COLS, exclude=(),
                 margin=BOX_MARGIN_PX, max_points=20000):
    """取樣區域裡、不在瓶子框內的像素反投影成 optical frame 的點，(N, 3)。"""
    h, w = depth.shape
    mask = np.zeros((h, w), dtype=bool)
    mask[int(rows[0] * h):int(rows[1] * h), int(cols[0] * w):int(cols[1] * w)] = True
    for x1, y1, x2, y2 in exclude:
        mask[max(0, int(y1) - margin):min(h, int(y2) + margin + 1),
             max(0, int(x1) - margin):min(w, int(x2) + margin + 1)] = False
    mask &= np.isfinite(depth)
    v, u = np.nonzero(mask)
    z = depth[v, u].astype(np.float64)
    fx, fy, cx, cy = k
    pts = np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z])
    if pts.shape[0] > max_points:
        idx = np.random.default_rng(0).choice(pts.shape[0], max_points, replace=False)
        pts = pts[idx]
    return pts


def _refine(pts, sel):
    c = pts[sel].mean(axis=0)
    n = np.linalg.svd(pts[sel] - c, full_matrices=False)[2][2]
    return n, c


def fit_plane(pts, threshold=PLANE_THRESHOLD_M, iterations=300):
    """RANSAC 找點最多的平面，再用平面附近的點細修。

    Returns:
        (normal, centroid, rms, n_inliers)；normal 指向相機那一側（桌面的「上」）。
    """
    rng = np.random.default_rng(0)
    probe = pts if pts.shape[0] <= 20000 else pts[rng.choice(pts.shape[0], 20000, replace=False)]
    best_n, best_p, best_count = None, None, -1
    for _ in range(iterations):
        p0, p1, p2 = probe[rng.choice(probe.shape[0], 3, replace=False)]
        n = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        count = int((np.abs((probe - p0) @ n) < threshold).sum())
        if count > best_count:
            best_n, best_p, best_count = n, p0, count
    sel = np.abs((pts - best_p) @ best_n) < threshold
    for _ in range(3):
        n, c = _refine(pts, sel)
        sel = np.abs((pts - c) @ n) < threshold
    n, c = _refine(pts, sel)
    if n @ c > 0:
        n = -n
    rms = float(np.sqrt(np.mean(((pts[sel] - c) @ n) ** 2)))
    return n, c, rms, int(sel.sum())


def camera_roll_pitch(normal_opt):
    """桌面法向量（optical frame）-> 相機相對桌面的 (roll, pitch)，弧度。"""
    up = R_BODY_FROM_OPT @ normal_opt
    return math.atan2(up[1], up[2]), math.asin(max(-1.0, min(1.0, -up[0])))


# ---------------------------------------------------------------------------
# IMU
# ---------------------------------------------------------------------------
def quat_roll_pitch(x, y, z, w):
    """四元數 -> (roll, pitch)，弧度。同 zed_imu_average.py（ZYX，往下看 pitch 為正）。"""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


# ---------------------------------------------------------------------------
# 一個姿勢的分析
# ---------------------------------------------------------------------------
@dataclass
class Snapshot:
    """一個姿勢記錄期間收到的資料。時間都是本機的 monotonic 秒。"""
    depth_frames: List[np.ndarray]
    k: Tuple[float, float, float, float]          # fx, fy, cx, cy
    size: Tuple[int, int]                         # 深度圖寬、高
    frame_id: str = ''
    color: Optional[np.ndarray] = None            # BGR
    joints: list = field(default_factory=list)    # [(t, pan, tilt)]
    imu: list = field(default_factory=list)       # [(t, roll, pitch, ax, ay, az, wx, wy, wz)]
    detections: list = field(default_factory=list)  # [(t, YOLO 的 JSON dict)]


POSTURE_FIELDS = [
    'index', 'kind', 'pan_cmd', 'tilt_cmd', 'reached', 'still', 'move_s',
    'pan_mean', 'pan_std', 'pan_min', 'pan_max',
    'tilt_mean', 'tilt_std', 'tilt_min', 'tilt_max', 'joint_n',
    'depth_frames', 'plane_points', 'plane_inliers', 'plane_rms_mm',
    'table_roll_deg', 'table_pitch_deg', 'table_height_m', 'normal_x', 'normal_y', 'normal_z',
    'imu_n', 'imu_roll_deg', 'imu_pitch_deg', 'imu_roll_std_deg', 'imu_pitch_std_deg',
    'acc_x', 'acc_y', 'acc_z', 'gyro_rms_dps',
    'det_msgs', 'det_size_mismatch', 'bottles',
]
BOTTLE_FIELDS = [
    'index', 'label', 'n_det', 'conf', 'x1', 'y1', 'x2', 'y2', 'n_points',
    'x', 'y', 'z', 'z_med', 'bearing_deg', 'elevation_deg',
]
NAN = float('nan')


def group_detections(detections, size):
    """記錄期間的 YOLO 結果按類別整理：每個類別取框的中位數。

    同一張影像裡同類別有多個框時只留信心值最高的（同 bottle_localizer）。
    image_size 和深度圖不同的訊息不用（數位變焦不是 1 時會這樣）。

    Returns:
        (groups, n_msgs, n_mismatch)：groups 是 {label: (n, 平均信心值, (x1, y1, x2, y2))}。
    """
    boxes, n_mismatch = {}, 0
    for _, data in detections:
        img_size = data.get('image_size')
        if img_size and [int(v) for v in img_size] != [int(size[0]), int(size[1])]:
            n_mismatch += 1
            continue
        best = {}
        for b in data.get('bottles', []):
            try:
                label = str(b['label'])
                conf = float(b['confidence'])
                bbox = [float(v) for v in b['bbox']]
            except (KeyError, TypeError, ValueError):
                continue
            if len(bbox) != 4:
                continue
            if label not in best or conf > best[label][0]:
                best[label] = (conf, bbox)
        for label, (conf, bbox) in best.items():
            boxes.setdefault(label, []).append((conf, bbox))
    groups = {}
    for label, items in boxes.items():
        conf = float(np.mean([c for c, _ in items]))
        med = np.median(np.asarray([b for _, b in items]), axis=0)
        groups[label] = (len(items), conf, tuple(int(round(v)) for v in med))
    return groups, len(detections), n_mismatch


def analyze_capture(snap, plane_rows=PLANE_ROWS, plane_cols=PLANE_COLS,
                    plane_threshold=PLANE_THRESHOLD_M, box_margin=BOX_MARGIN_PX):
    """一個姿勢的記錄 -> (CSV 的一列, 瓶子的列, 深度中位數圖)。

    算不出來的欄位填 NaN，不丟例外：資料照樣存下來，離線再看。
    """
    row = dict(joint_stats(snap.joints))
    row['depth_frames'] = len(snap.depth_frames)
    depth = median_depth(snap.depth_frames) if snap.depth_frames else None

    groups, n_msgs, n_mismatch = group_detections(snap.detections, snap.size)
    row['det_msgs'] = n_msgs
    row['det_size_mismatch'] = n_mismatch

    bottle_rows = []
    if depth is not None:
        for label in sorted(groups):
            n_det, conf, bbox = groups[label]
            sp = surface_point(depth, bbox, snap.k)
            b = {'label': label, 'n_det': n_det, 'conf': conf,
                 'x1': bbox[0], 'y1': bbox[1], 'x2': bbox[2], 'y2': bbox[3]}
            if sp is None:
                b.update(n_points=0, x=NAN, y=NAN, z=NAN, z_med=NAN,
                         bearing_deg=NAN, elevation_deg=NAN)
            else:
                p, z_med, n = sp
                b.update(n_points=n, x=float(p[0]), y=float(p[1]), z=float(p[2]), z_med=z_med,
                         bearing_deg=math.degrees(math.atan2(p[0], p[2])),
                         elevation_deg=math.degrees(math.atan2(p[1], p[2])))
            bottle_rows.append(b)
    row['bottles'] = sum(1 for b in bottle_rows if b['n_points'] > 0)

    plane = dict(plane_points=0, plane_inliers=NAN, plane_rms_mm=NAN, table_roll_deg=NAN,
                 table_pitch_deg=NAN, table_height_m=NAN, normal_x=NAN, normal_y=NAN, normal_z=NAN)
    if depth is not None:
        pts = plane_points(depth, snap.k, plane_rows, plane_cols,
                           [g[2] for g in groups.values()], box_margin)
        plane['plane_points'] = int(pts.shape[0])
        if pts.shape[0] >= MIN_PLANE_POINTS:
            n, c, rms, n_in = fit_plane(pts, plane_threshold)
            roll, pitch = camera_roll_pitch(n)
            plane.update(plane_inliers=n_in / pts.shape[0], plane_rms_mm=rms * 1000,
                         table_roll_deg=math.degrees(roll), table_pitch_deg=math.degrees(pitch),
                         table_height_m=float(-(n @ c)),
                         normal_x=float(n[0]), normal_y=float(n[1]), normal_z=float(n[2]))
    row.update(plane)

    row['imu_n'] = len(snap.imu)
    if snap.imu:
        a = np.asarray(snap.imu, dtype=np.float64)
        row.update(imu_roll_deg=math.degrees(a[:, 1].mean()),
                   imu_pitch_deg=math.degrees(a[:, 2].mean()),
                   imu_roll_std_deg=math.degrees(a[:, 1].std()),
                   imu_pitch_std_deg=math.degrees(a[:, 2].std()),
                   acc_x=float(a[:, 3].mean()), acc_y=float(a[:, 4].mean()),
                   acc_z=float(a[:, 5].mean()),
                   gyro_rms_dps=math.degrees(float(np.sqrt(np.mean(np.sum(a[:, 6:9] ** 2, axis=1))))))
    else:
        row.update(imu_roll_deg=NAN, imu_pitch_deg=NAN, imu_roll_std_deg=NAN,
                   imu_pitch_std_deg=NAN, acc_x=NAN, acc_y=NAN, acc_z=NAN, gyro_rms_dps=NAN)
    return row, bottle_rows, depth


# ---------------------------------------------------------------------------
# 掃描結束後的快速檢查
# ---------------------------------------------------------------------------
def _finite(*values):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def _ref_rows(rows):
    return [r for r in rows if r.get('kind') == 'ref']


def direction_check(rows, bottle_rows):
    """用資料確認馬達方向和角度比例。

    - tilt：桌面 pitch 的變化 ≈ a × tilt 角度變化（+ b × pan）。a ≈ +1 代表刻度變大往下看。
    - pan：瓶子方位角（optical frame 裡 atan2(x, z)，往右為正）的變化 ≈ c × pan 角度變化。
      頭往左轉時，靜止的瓶子在畫面裡往右移，所以 c ≈ +1 代表刻度變大往左轉。
    相機往下看約 30°，pan 的比例會略小於 1。

    Returns:
        dict：tilt_slope、tilt_n、pan_slope、pan_n；資料不夠時 slope 是 None。
    """
    refs = [r for r in _ref_rows(rows) if _finite(r.get('pan_mean'), r.get('tilt_mean'))]
    out = {'tilt_slope': None, 'tilt_n': 0, 'pan_slope': None, 'pan_n': 0}
    if not refs:
        return out
    pan0 = float(np.mean([r['pan_mean'] for r in refs]))
    tilt0 = float(np.mean([r['tilt_mean'] for r in refs]))

    pitch_ref = [r['table_pitch_deg'] for r in refs if _finite(r.get('table_pitch_deg'))]
    if pitch_ref:
        p0 = float(np.mean(pitch_ref))
        X, y = [], []
        for r in rows:
            if _finite(r.get('table_pitch_deg'), r.get('pan_mean'), r.get('tilt_mean')):
                X.append([ticks_to_deg(r['tilt_mean'] - tilt0), ticks_to_deg(r['pan_mean'] - pan0)])
                y.append(r['table_pitch_deg'] - p0)
        X = np.asarray(X)
        if len(y) >= 3 and np.ptp(X[:, 0]) > 2.0:
            out['tilt_slope'] = float(np.linalg.lstsq(X, np.asarray(y), rcond=None)[0][0])
            out['tilt_n'] = len(y)

    by_index = {r['index']: r for r in rows}
    ref_idx = {r['index'] for r in refs}
    bearing0 = {}
    for b in bottle_rows:
        if b['index'] in ref_idx and _finite(b.get('bearing_deg')):
            bearing0.setdefault(b['label'], []).append(b['bearing_deg'])
    bearing0 = {k: float(np.mean(v)) for k, v in bearing0.items()}
    X, y = [], []
    for b in bottle_rows:
        r = by_index.get(b['index'])
        if r is None or b['label'] not in bearing0:
            continue
        if _finite(b.get('bearing_deg'), r.get('pan_mean'), r.get('tilt_mean')):
            X.append([ticks_to_deg(r['pan_mean'] - pan0), ticks_to_deg(r['tilt_mean'] - tilt0)])
            y.append(b['bearing_deg'] - bearing0[b['label']])
    X = np.asarray(X)
    if len(y) >= 3 and np.ptp(X[:, 0]) > 2.0:
        out['pan_slope'] = float(np.linalg.lstsq(X, np.asarray(y), rcond=None)[0][0])
        out['pan_n'] = len(y)
    return out


def repeatability(rows, bottle_rows):
    """第一次和最後一次基準姿勢的差。差太多代表中間有東西動過，這次資料不能用。

    Returns:
        dict 或 None（基準姿勢不到兩次）：pitch_deg、roll_deg、height_mm、imu_pitch_deg、imu_roll_deg、
        bottle_mm（{label: 表面點距離 mm}）、ticks（兩次刻度差的最大值）。
    """
    refs = _ref_rows(rows)
    if len(refs) < 2:
        return None
    a, b = refs[0], refs[-1]

    def diff(key, scale=1.0):
        if _finite(a.get(key), b.get(key)):
            return (b[key] - a[key]) * scale
        return None

    ticks = None
    if _finite(a.get('pan_mean'), b.get('pan_mean'), a.get('tilt_mean'), b.get('tilt_mean')):
        ticks = max(abs(b['pan_mean'] - a['pan_mean']), abs(b['tilt_mean'] - a['tilt_mean']))
    pa = {x['label']: x for x in bottle_rows if x['index'] == a['index'] and x['n_points'] > 0}
    pb = {x['label']: x for x in bottle_rows if x['index'] == b['index'] and x['n_points'] > 0}
    bottle_mm = {}
    for label in sorted(set(pa) & set(pb)):
        d = np.array([pb[label][k] - pa[label][k] for k in ('x', 'y', 'z')])
        bottle_mm[label] = float(np.linalg.norm(d) * 1000)
    return {'pitch_deg': diff('table_pitch_deg'), 'roll_deg': diff('table_roll_deg'),
            'height_mm': diff('table_height_m', 1000.0), 'imu_pitch_deg': diff('imu_pitch_deg'),
            'imu_roll_deg': diff('imu_roll_deg'),
            'bottle_mm': bottle_mm, 'ticks': ticks}
