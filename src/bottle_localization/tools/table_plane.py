#!/usr/bin/env python3
#coding=utf-8
"""用 ZED 深度圖擬合桌面，量相機相對桌面的 roll / pitch，以及左眼光心離桌面多高。

在機器人上執行（ZED 要開著），機器人擺好抓取姿勢、保持不動：

    python3 table_plane.py                          # 預設取畫面下方中間那一塊
    python3 table_plane.py --rows 0.6 0.9 --cols 0.3 0.7 --frames 20

取樣區域（--rows、--cols 是畫面高、寬的比例）裡**只能有桌面**：先把瓶子、手臂移開，
或改用畫面裡空的那一塊桌面。

為什麼不用 IMU：ZED IMU 晶片和鏡頭之間有出廠的安裝角度（/zed/zed_node/left_cam_imu_transform），
wrapper 發的姿態是 IMU 晶片自己的方向。這裡直接從深度量「相機相對桌面」，正好是演算法需要的。

輸出的 roll / pitch 是 zed_left_camera_frame（= zed_camera_link 的方向）相對桌面的角度，
慣例與 camera_extrinsics.yaml 相同（繞固定軸 X -> Y，往下看 pitch 為正）。
"""
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

#optical frame（X 右、Y 下、Z 前）在相機本體（X 前、Y 左、Z 上）下的軸向
R_BODY_FROM_OPT = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0]])
DEPTH_SCALE = {'32FC1': 1.0, '16UC1': 0.001, 'mono16': 0.001}
#Stereolabs 對 ZED X Mini 建議的最近距離（zedxm.yaml 的註解），更近的深度不準
MIN_RELIABLE_DEPTH = 0.30
#診斷表把取樣區域切成幾 x 幾格
GRID = 3


def depth_image_to_array(msg):
    """sensor_msgs/Image -> (H, W) 公尺，無效為 NaN。"""
    if msg.encoding not in DEPTH_SCALE:
        raise ValueError(f'不支援的深度編碼 {msg.encoding}')
    dtype = np.float32 if msg.encoding == '32FC1' else np.uint16
    row = msg.step // np.dtype(dtype).itemsize
    d = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, row)[:, :msg.width]
    d = d.astype(np.float64) * DEPTH_SCALE[msg.encoding]
    d[~np.isfinite(d) | (d <= 0)] = np.nan
    return d


def roi_points(depth, k, rows, cols, max_points):
    """取樣區域內的像素反投影成 optical frame 的點。

    Returns:
        (pts, cell)：pts 是 (N, 3)；cell 是 (N,) 每個點落在 GRID x GRID 診斷表的第幾格。
    """
    h, w = depth.shape
    r0, r1 = int(rows[0] * h), int(rows[1] * h)
    c0, c1 = int(cols[0] * w), int(cols[1] * w)
    v, u = np.mgrid[r0:r1, c0:c1]
    z = depth[r0:r1, c0:c1]
    ok = np.isfinite(z)
    u, v, z = u[ok].astype(np.float64), v[ok].astype(np.float64), z[ok]
    fx, fy, cx, cy = k
    pts = np.column_stack([(u - cx) * z / fx, (v - cy) * z / fy, z])
    gr = np.minimum(((v - r0) * GRID / max(r1 - r0, 1)).astype(int), GRID - 1)
    gc = np.minimum(((u - c0) * GRID / max(c1 - c0, 1)).astype(int), GRID - 1)
    cell = gr * GRID + gc
    if pts.shape[0] > max_points:
        idx = np.random.default_rng(0).choice(pts.shape[0], max_points, replace=False)
        pts, cell = pts[idx], cell[idx]
    return pts, cell


def print_grid(pts, cell, n, c):
    """3 x 3 診斷表：每格的深度中位數與「離擬合平面」的中位距離。

    桌面那幾格應該都在 ±幾 mm 內；明顯偏離的那格就是混進來的東西（瓶子、手臂、桌緣）。
    正值 = 比平面更靠近相機（凸出桌面），負值 = 比平面更遠（桌面後面，例如地板）。
    """
    dist = (pts - c) @ n
    print(f'取樣區域 {GRID}x{GRID} 診斷（上 -> 下 = 畫面上 -> 下；每格：深度中位數 / 離平面中位距離）')
    for r in range(GRID):
        cells = []
        for col in range(GRID):
            sel = cell == r * GRID + col
            if sel.sum() < 50:
                cells.append(f'{"（點太少）":>22}')
                continue
            cells.append(f'{np.median(pts[sel, 2]) * 100:6.1f} cm / {np.median(dist[sel]) * 1000:+6.1f} mm')
        print('   ' + '   '.join(cells))


def _refine(pts, sel):
    """用選到的點做 SVD 平面擬合，回 (法向量, 重心)。"""
    c = pts[sel].mean(axis=0)
    n = np.linalg.svd(pts[sel] - c, full_matrices=False)[2][2]
    return n, c


def fit_plane(pts, threshold, iterations=300):
    """RANSAC 找出點最多的那個平面，再用平面附近的點細修。

    取樣區域混到瓶子、手臂、桌緣時，最小平方會被拉歪；RANSAC 只看「最多點同意」的平面，
    混進來的東西只要不是佔大多數就不影響。

    Returns:
        (normal, centroid, rms, n_inliers)；normal 朝向相機那一側（也就是桌面的「上」）。
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
    if n @ c > 0:              #相機在原點，法向量要指向相機
        n = -n
    rms = float(np.sqrt(np.mean(((pts[sel] - c) @ n) ** 2)))
    return n, c, rms, int(sel.sum())


def camera_roll_pitch(normal_opt):
    """桌面法向量（optical frame）-> 相機相對桌面的 roll、pitch（弧度）。

    相機姿態 R = Ry(pitch) Rx(roll) 時，「上」在相機本體座標裡是
    R^T (0, 0, 1) = (-sin p, cos p sin r, cos p cos r)。
    """
    up = R_BODY_FROM_OPT @ normal_opt
    return math.atan2(up[1], up[2]), math.asin(max(-1.0, min(1.0, -up[0])))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--depth', default='/zed/zed_node/depth/depth_registered')
    ap.add_argument('--info', default='/zed/zed_node/left/camera_info')
    ap.add_argument('--rows', type=float, nargs=2, default=(0.70, 0.95), help='取樣區域的上下（畫面高的比例）')
    ap.add_argument('--cols', type=float, nargs=2, default=(0.25, 0.75), help='取樣區域的左右（畫面寬的比例）')
    ap.add_argument('--frames', type=int, default=10, help='用幾張深度圖')
    ap.add_argument('--timeout', type=float, default=15.0)
    ap.add_argument('--threshold', type=float, default=0.004,
                    help='離平面多近（m）算桌面上的點')
    opts = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('table_plane')
    state = {'k': None, 'frames': []}

    def on_info(m):
        state['k'] = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_depth(m):
        if len(state['frames']) < opts.frames:
            state['frames'].append(depth_image_to_array(m))

    node.create_subscription(CameraInfo, opts.info, on_info, qos_profile_sensor_data)
    node.create_subscription(Image, opts.depth, on_depth, qos_profile_sensor_data)
    end = time.time() + opts.timeout
    while time.time() < end and (state['k'] is None or len(state['frames']) < opts.frames):
        rclpy.spin_once(node, timeout_sec=0.1)
    if state['k'] is None or not state['frames']:
        print(f'{opts.timeout:g} 秒內沒收到 {opts.info} 或 {opts.depth}，ZED 有在跑嗎？')
        return 1

    parts = [roi_points(d, state['k'], opts.rows, opts.cols, 20000) for d in state['frames']]
    pts = np.vstack([p for p, _ in parts])
    cell = np.concatenate([c for _, c in parts])
    if pts.shape[0] < 500:
        print(f'取樣區域只有 {pts.shape[0]} 個有效深度點，換一塊有桌面的區域（--rows / --cols）')
        return 1
    n, c, rms, n_in = fit_plane(pts, opts.threshold)
    roll, pitch = camera_roll_pitch(n)
    height = float(-(n @ c))

    print(f'{len(state["frames"])} 張深度圖，取樣 {pts.shape[0]} 點，離平面 {opts.threshold * 1000:.0f} mm 以內'
          f' {n_in} 點（{n_in / pts.shape[0]:.0%}），這些點的殘差 {rms * 1000:.1f} mm')
    print(f'相機相對桌面：roll {math.degrees(roll):+.2f} 度 = {roll:+.4f} rad，'
          f'pitch {math.degrees(pitch):+.2f} 度 = {pitch:+.4f} rad')
    print(f'左眼光心離桌面 {height * 100:.1f} cm')
    z5, z50, z95 = np.percentile(pts[:, 2], [5, 50, 95])
    print(f'取樣區域的深度：中位數 {z50 * 100:.1f} cm（5%~95%：{z5 * 100:.1f} ~ {z95 * 100:.1f} cm）')
    print_grid(pts, cell, n, c)
    if n_in < 0.9 * pts.shape[0]:
        print('⚠ 取樣區域不是一塊乾淨的平面：看上面的診斷表，避開偏離的那幾格，'
              '用 --rows / --cols 換一塊空的桌面再量一次')
    if z5 < MIN_RELIABLE_DEPTH:
        print(f'⚠ 取樣區域有一部分離鏡頭不到 {MIN_RELIABLE_DEPTH * 100:.0f} cm，ZED X Mini 在這麼近的距離深度不準，'
              f'改取畫面中間一點的區域（例如 --rows 0.45 0.65）')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
