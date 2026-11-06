#!/usr/bin/env python3
#coding=utf-8
"""已知位置實測：記錄每個點的輸出、算誤差，最後擬合出外參的修正量。

在機器人上執行（bottle_localization 要在跑），同一瓶依序放到量好位置的點上:

    python3 position_check.py --label b2
    python3 position_check.py --label b2 --seconds 3 --out ~/points_b2.csv

每個點輸入「瓶子圓心相對基準點」往前、往左的水平距離（cm），程式收 --seconds 秒
/bottle_detections_3d 裡這個類別的座標取平均，印出誤差並存進 CSV。直接按 Enter 結束，
接著印出整體誤差、擬合出的修正（x、y 偏移與 yaw 旋轉），以及修正後的 camera_extrinsics.yaml。

桌邊兩點量距離（不用在桌上畫點，瓶子隨便放）:

    python3 position_check.py --label b2 --edge 14.0 14.2 27.5

--edge 給三個 cm：右肩垂線到桌邊的距離 a_R、左肩垂線到桌邊的距離 a_L、桌邊上兩個垂足
P0（右肩正前方）與 P0_L（左肩正前方）之間的距離。每個點輸入捲尺從 P0、P0_L 量到瓶身的
最短距離（cm），程式加上瓶身半徑（bottle_dims.yaml）求出瓶子圓心，再換成相對基準點的
往前、往左。桌子不用擺正，兩肩連線和桌邊的夾角由 a_R、a_L 算出來。

只分析現有的 CSV（不需要 ROS，任何電腦都可以）:

    python3 position_check.py --analyze ~/points_b2.csv

擬合：找一個平面上的旋轉 δ 與平移 d，讓「輸出經過修正」最接近真值（最小平方）:
    真值 ≈ Rz(δ) · 輸出 + d
相機位置也跟著做同樣的修正，所以外參的 yaw 加上 δ，(x, y) 換成 Rz(δ)·(x, y) + d，z 不變。
pitch、roll 的小誤差在同一張桌面上會表現成接近固定的 x、y 偏移，也一起被修掉。
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

#設定檔預設用這支工具旁邊的 config/（機器人和桌機都一樣）
CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
DEFAULT_EXTRINSICS = os.path.join(CONFIG_DIR, 'camera_extrinsics.yaml')
DEFAULT_DIMS = os.path.join(CONFIG_DIR, 'bottle_dims.yaml')
#基準點（量外參、實測位置時在機器人身上量的那一點，目前是右肩關節點）
DEFAULT_REFERENCE = os.path.join(CONFIG_DIR, 'robot_reference.yaml')
FIELDS = ['point', 'dx_cm', 'dy_cm', 'true_x', 'true_y', 'out_x', 'out_y', 'std_x', 'std_y', 'n']
#--edge 量法的原始讀數（只做紀錄，分析不會用到）
EXTRA_FIELDS = ['dist_p0_cm', 'dist_p0l_cm']


def load_reference(path):
    """robot_reference.yaml -> (基準點名稱, 基準點在 base_link 的 (x, y)（m）)。"""
    import yaml
    with open(os.path.expanduser(path), 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    point = [float(v) for v in cfg['reference_point']]
    return str(cfg.get('reference_name', '基準點')), (point[0], point[1])


def to_base(dx_cm, dy_cm, ref_xy):
    """基準點往前、往左的距離（cm）-> base_link 的 (x, y)（m）。"""
    return dx_cm / 100.0 + ref_xy[0], dy_cm / 100.0 + ref_xy[1]


def edge_to_shoulder(r0, r1, a_right, a_left, span):
    """桌邊兩點到瓶子圓心的距離 -> 瓶子圓心相對基準點（右肩）的 (往前, 往左)，單位都是 cm。

    桌邊座標：P0 為原點，u 沿桌邊往 P0_L（機器人左邊），v 垂直桌邊往桌面裡面。
    右、左肩關節點的投影分別在 (0, -a_right)、(span, -a_left)，兩點連線就是 base_link 的 Y 軸。
    兩個距離對不起來（兩圓不相交，或交點在桌邊外）時回 None。
    """
    u = (r0 ** 2 - r1 ** 2 + span ** 2) / (2.0 * span)
    v2 = r0 ** 2 - u ** 2
    if v2 <= 0.0:
        return None
    bu, bv = u, math.sqrt(v2) + a_right
    s = math.hypot(span, a_right - a_left)
    forward = (bu * (a_left - a_right) + bv * span) / s
    left = (bu * span + bv * (a_right - a_left)) / s
    return forward, left


def bottle_diameter(dims_path, label):
    import yaml
    with open(os.path.expanduser(dims_path), 'r', encoding='utf-8') as f:
        return float(yaml.safe_load(f)['bottle_dims'][label]['diameter'])


def fit_rigid_2d(out_xy, true_xy):
    """最小平方擬合 真值 ≈ R(δ)·輸出 + d。回 (δ 弧度, d (2,))。需要至少 2 個點。"""
    o = np.asarray(out_xy, dtype=np.float64)
    t = np.asarray(true_xy, dtype=np.float64)
    oc, tc = o.mean(axis=0), t.mean(axis=0)
    h = (o - oc).T @ (t - tc)
    delta = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    r = np.array([[math.cos(delta), -math.sin(delta)], [math.sin(delta), math.cos(delta)]])
    return delta, tc - r @ oc


def apply_rigid_2d(delta, d, xy):
    r = np.array([[math.cos(delta), -math.sin(delta)], [math.sin(delta), math.cos(delta)]])
    return (np.asarray(xy, dtype=np.float64) @ r.T) + d


def corrected_extrinsics(ext, delta, d):
    """把擬合的修正套到 camera_extrinsics.yaml 的值上（只動 x、y、yaw）。"""
    x, y = apply_rigid_2d(delta, d, [[float(ext['x']), float(ext['y'])]])[0]
    yaw = float(ext['yaw']) + delta
    return x, y, yaw


def describe(errors_m):
    e = np.asarray(errors_m) * 1000.0
    return f'平均 {e.mean():.1f} mm、RMS {math.sqrt((e ** 2).mean()):.1f} mm、最大 {e.max():.1f} mm'


def analyze(rows, extrinsics_path):
    """整體誤差、擬合修正、修正後的殘差與建議外參。"""
    if not rows:
        print('沒有任何點，無法分析')
        return
    true_xy = np.array([[r['true_x'], r['true_y']] for r in rows], dtype=np.float64)
    out_xy = np.array([[r['out_x'], r['out_y']] for r in rows], dtype=np.float64)
    err = out_xy - true_xy
    dist = np.linalg.norm(err, axis=1)

    print('\n==== 實測結果（輸出 − 真值）====')
    print(f'{"點":>4} {"真值 x":>8} {"真值 y":>8} {"輸出 x":>8} {"輸出 y":>8} {"誤差 x":>8} {"誤差 y":>8} {"距離":>7}')
    for r, e, dd in zip(rows, err, dist):
        print(f'{r["point"]:>4} {r["true_x"]:+8.3f} {r["true_y"]:+8.3f} {r["out_x"]:+8.3f} {r["out_y"]:+8.3f}'
              f' {e[0] * 1000:+7.1f}mm {e[1] * 1000:+7.1f}mm {dd * 1000:6.1f}mm')
    print(f'修正前：{describe(dist)}；整體偏移 x {err[:, 0].mean() * 1000:+.1f} mm、y {err[:, 1].mean() * 1000:+.1f} mm')

    if len(rows) < 3:
        print('點少於 3 個，不擬合修正量（建議至少 6 個、前後左右分散）')
        return
    delta, d = fit_rigid_2d(out_xy, true_xy)
    resid = np.linalg.norm(apply_rigid_2d(delta, d, out_xy) - true_xy, axis=1)
    print(f'\n擬合的修正：旋轉 {math.degrees(delta):+.2f} 度、平移 x {d[0] * 1000:+.1f} mm、y {d[1] * 1000:+.1f} mm')
    print(f'修正後的殘差：{describe(resid)}')
    print('（殘差反映真值量測誤差與無法用平移、旋轉修掉的部分，例如瓶子尺寸或距離相關的誤差）')

    try:
        import yaml
        with open(os.path.expanduser(extrinsics_path), 'r', encoding='utf-8') as f:
            ext = yaml.safe_load(f)
    except Exception as e:
        print(f'\n讀不到外參檔 {extrinsics_path}（{e}），略過建議值')
        return
    x, y, yaw = corrected_extrinsics(ext, delta, d)
    print(f'\n---- 建議的 camera_extrinsics.yaml（目前 x {float(ext["x"]):.4f}、y {float(ext["y"]):.4f}、'
          f'yaw {float(ext["yaw"]):.4f}）----')
    print(f'x: {x:.4f}\ny: {y:.4f}\nz: {float(ext["z"]):.4f}\nroll: {float(ext["roll"]):.4f}\n'
          f'pitch: {float(ext["pitch"]):.4f}\nyaw: {yaw:.4f}')
    print('改完重新 launch，用「不同的點」再測一次確認，避免只是剛好擬合這幾個點。')


def read_csv(path):
    rows = []
    with open(os.path.expanduser(path), newline='', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            row = {k: (r[k] if k == 'point' else float(r[k])) for k in FIELDS}
            row.update({k: r[k] for k in EXTRA_FIELDS if r.get(k)})
            rows.append(row)
    return rows


def record(label, seconds, out_path, extrinsics_path, reference, edge=None, diameter=None, max_diff_cm=None):
    """edge=(a_R, a_L, span) 時改用桌邊兩點量距離，diameter（m）用來把讀數加上半徑。

    reference：load_reference() 的結果。
    max_diff_cm：真值和輸出差超過這個距離時先問要不要記錄（多半是量錯或輸入錯），None 不檢查。
    """
    ref_name, ref_xy = reference
    import rclpy
    from vision_msgs.msg import Detection3DArray

    rclpy.init()
    node = rclpy.create_node('position_check')
    samples = []
    collecting = [False]

    def callback(msg):
        if not collecting[0]:
            return
        for det in msg.detections:
            if det.id == label and det.results:
                p = det.results[0].pose.pose.position
                samples.append((p.x, p.y))

    node.create_subscription(Detection3DArray, '/bottle_detections_3d', callback, 10)
    rows = read_csv(out_path) if os.path.exists(os.path.expanduser(out_path)) else []
    if rows:
        print(f'接著 {out_path} 既有的 {len(rows)} 個點繼續記錄')
    if edge:
        radius_cm = diameter * 100.0 / 2.0
        a_r, a_l, span = edge
        print(f'兩肩關節點間距 {math.hypot(span, a_r - a_l):.1f} cm，兩肩連線和桌邊的夾角 '
              f'{math.degrees(math.atan2(a_r - a_l, span)):+.1f} 度（擺正時是 0）')
        print(f'類別 {label}，每個點收 {seconds:g} 秒。輸入捲尺從 P0、P0_L 量到瓶身的最短距離（cm），'
              f'例如 30.2 25.1，程式會加上半徑 {radius_cm:.2f} cm；直接按 Enter 結束並分析。')
        prompt = '到 P0 到 P0_L（cm）'
    else:
        print(f'類別 {label}，每個點收 {seconds:g} 秒。輸入瓶子圓心相對{ref_name}的「往前 往左」距離（cm），'
              f'例如 32.5 -4；直接按 Enter 結束並分析。')
        prompt = '往前 往左（cm）'
    try:
        while True:
            line = input(f'\n點 {len(rows) + 1} {prompt}> ').strip()
            if not line:
                break
            try:
                v1, v2 = (float(v) for v in line.replace(',', ' ').split())
            except ValueError:
                print('格式：兩個數字，例如 30.2 25.1' if edge else '格式：兩個數字，例如 32.5 -4')
                continue
            raw = {}
            if edge:
                pos = edge_to_shoulder(v1 + radius_cm, v2 + radius_cm, *edge)
                if pos is None:
                    print('這兩個距離對不起來（距離和比 P0 到 P0_L 還短，或差太多），請重量')
                    continue
                dx_cm, dy_cm = pos
                raw = {'dist_p0_cm': f'{v1:g}', 'dist_p0l_cm': f'{v2:g}'}
                print(f'瓶子圓心：{ref_name}往前 {dx_cm:.1f} cm、往左 {dy_cm:+.1f} cm')
            else:
                dx_cm, dy_cm = v1, v2
            samples.clear()
            collecting[0] = True
            end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
            while node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            collecting[0] = False
            if len(samples) < 3:
                print(f'{seconds:g} 秒內只收到 {len(samples)} 筆 {label}，瓶子在畫面裡嗎？這個點不記錄')
                continue
            s = np.asarray(samples)
            tx, ty = to_base(dx_cm, dy_cm, ref_xy)
            ox, oy = s.mean(axis=0)
            sx, sy = s.std(axis=0)
            e = math.hypot(ox - tx, oy - ty) * 1000
            print(f'真值 ({tx:+.3f}, {ty:+.3f})  輸出 ({ox:+.3f}, {oy:+.3f})  誤差 x {(ox - tx) * 1000:+.1f} mm、'
                  f'y {(oy - ty) * 1000:+.1f} mm、距離 {e:.1f} mm（{len(s)} 筆，抖動 {sx * 1000:.1f}/{sy * 1000:.1f} mm）')
            if max_diff_cm is not None and e > max_diff_cm * 10.0:
                ans = input(f'差 {e / 10.0:.1f} cm，超過 --max-diff {max_diff_cm:g} cm，多半是量錯或輸入錯。'
                            f'輸入 y 仍然記錄，直接按 Enter 不記錄、重量這個點> ').strip().lower()
                if ans != 'y':
                    continue
            rows.append({'point': str(len(rows) + 1), 'dx_cm': dx_cm, 'dy_cm': dy_cm,
                         'true_x': tx, 'true_y': ty, 'out_x': float(ox), 'out_y': float(oy),
                         'std_x': float(sx), 'std_y': float(sy), 'n': float(len(s)), **raw})
            with open(os.path.expanduser(out_path), 'w', newline='', encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=FIELDS + EXTRA_FIELDS, restval='')
                w.writeheader()
                w.writerows(rows)
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f'已存 {len(rows)} 個點到 {out_path}')
    analyze(rows, extrinsics_path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--label', help='要量的類別，例如 b2（記錄模式必填）')
    ap.add_argument('--seconds', type=float, default=3.0, help='每個點收幾秒取平均')
    ap.add_argument('--out', default='~/position_check.csv', help='記錄存到哪個 CSV')
    ap.add_argument('--analyze', metavar='CSV', help='只分析既有的 CSV，不需要 ROS')
    ap.add_argument('--extrinsics', default=DEFAULT_EXTRINSICS, help='目前的 camera_extrinsics.yaml')
    ap.add_argument('--edge', nargs=3, type=float, metavar=('A_R', 'A_L', 'SPAN'),
                    help='桌邊兩點量距離：右肩到桌邊、左肩到桌邊、P0 到 P0_L 的距離（cm）')
    ap.add_argument('--dims', default=DEFAULT_DIMS, help='bottle_dims.yaml（--edge 用來取瓶身直徑）')
    ap.add_argument('--reference', default=DEFAULT_REFERENCE, help='robot_reference.yaml（基準點的位置）')
    ap.add_argument('--max-diff', type=float, default=3.0,
                    help='真值和輸出差超過幾 cm 時先確認再記錄（多半是量錯），0 表示不檢查')
    opts = ap.parse_args()
    if opts.analyze:
        analyze(read_csv(opts.analyze), opts.extrinsics)
    elif opts.label:
        diameter = None
        if opts.edge:
            if opts.edge[2] <= 0.0:
                ap.error('--edge 的 P0 到 P0_L 距離要大於 0')
            try:
                diameter = bottle_diameter(opts.dims, opts.label)
            except Exception as e:
                ap.error(f'從 {opts.dims} 讀不到 {opts.label} 的直徑（{e}）')
        try:
            reference = load_reference(opts.reference)
        except Exception as e:
            ap.error(f'讀不到基準點 {opts.reference}（{e}）')
        record(opts.label, opts.seconds, opts.out, opts.extrinsics, reference, opts.edge, diameter,
               opts.max_diff if opts.max_diff > 0.0 else None)
    else:
        ap.error('記錄模式要給 --label，或用 --analyze 分析既有的 CSV')


if __name__ == '__main__':
    sys.exit(main())
