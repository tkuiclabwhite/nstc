#!/usr/bin/env python3
#coding=utf-8
"""兩瓶距離檢查：不用量瓶子在哪裡，只檢查相機量到的「形狀」對不對。

兩個不同類別的瓶子直立在桌上，中間平放一把尺，尺的兩端分別貼著兩個瓶身，並大致對準兩瓶圓心。
兩瓶圓心的真實距離 = 尺的全長 + 兩瓶半徑（bottle_dims.yaml）。每擺好一次按 Enter，
程式收 --seconds 秒 /bottle_detections_3d 裡兩瓶的座標取平均，比較兩瓶圓心距離和真實距離。

    python3 pair_check.py --labels b2 b3 --length 32.0

--length 是尺從一端到另一端的實際全長（cm），不是刻度的長度。
在工作範圍內換位置、換方向（前後、左右、斜放）各擺幾次。外參（x、y、z、roll、pitch、yaw）的小誤差
只會讓所有點一起平移、旋轉，不會改變兩瓶的距離，所以這個檢查只看深度與定位演算法本身。
整體偏移與旋轉要用 position_check.py。
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

DEFAULT_DIMS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'bottle_dims.yaml')
FIELDS = ['placement', 'label_a', 'label_b', 'ax', 'ay', 'bx', 'by', 'distance', 'expected', 'error', 'angle_deg']


def bottle_diameter(dims_path, label):
    import yaml
    with open(os.path.expanduser(dims_path), 'r', encoding='utf-8') as f:
        return float(yaml.safe_load(f)['bottle_dims'][label]['diameter'])


def pair_result(a_xy, b_xy, expected):
    """回 (兩瓶距離 m, 誤差 m, 方向角 度)。方向角 0 = 前後擺，±90 = 左右擺。"""
    d = np.asarray(b_xy, dtype=np.float64) - np.asarray(a_xy, dtype=np.float64)
    dist = float(np.hypot(d[0], d[1]))
    angle = math.degrees(math.atan2(d[1], d[0]))
    if angle > 90.0:            #方向不分頭尾，換到 (-90, 90]
        angle -= 180.0
    elif angle <= -90.0:
        angle += 180.0
    return dist, dist - expected, angle


def summarize(rows):
    if not rows:
        print('沒有任何一次擺放')
        return
    print('\n==== 兩瓶距離（量到 − 真實）====')
    print(f'{"次":>3} {"中點 x":>8} {"中點 y":>8} {"方向":>7} {"量到":>8} {"真實":>8} {"誤差":>8}')
    for r in rows:
        mx, my = (r['ax'] + r['bx']) / 2.0, (r['ay'] + r['by']) / 2.0
        print(f'{r["placement"]:>3} {mx:+8.3f} {my:+8.3f} {r["angle_deg"]:+6.0f}° {r["distance"] * 1000:6.1f}mm'
              f' {r["expected"] * 1000:6.1f}mm {r["error"] * 1000:+6.1f}mm')
    e = np.array([r['error'] for r in rows]) * 1000.0
    print(f'誤差：平均 {e.mean():+.1f} mm、絕對值平均 {np.abs(e).mean():.1f} mm、最大 {np.abs(e).max():.1f} mm')
    print('（方向 0° 是前後擺、±90° 是左右擺；某個方向的誤差特別偏一邊，代表那個方向被拉長或壓縮）')


def read_csv(path):
    with open(os.path.expanduser(path), newline='', encoding='utf-8') as f:
        return [{k: (r[k] if k in ('placement', 'label_a', 'label_b') else float(r[k])) for k in FIELDS}
                for r in csv.DictReader(f)]


def record(labels, length_cm, seconds, dims_path, out_path):
    import rclpy
    from vision_msgs.msg import Detection3DArray

    expected = length_cm / 100.0 + sum(bottle_diameter(dims_path, l) / 2.0 for l in labels)
    rclpy.init()
    node = rclpy.create_node('pair_check')
    samples = {l: [] for l in labels}
    collecting = [False]

    def callback(msg):
        if not collecting[0]:
            return
        for det in msg.detections:
            if det.id in samples and det.results:
                p = det.results[0].pose.pose.position
                samples[det.id].append((p.x, p.y))

    node.create_subscription(Detection3DArray, '/bottle_detections_3d', callback, 10)
    rows = read_csv(out_path) if os.path.exists(os.path.expanduser(out_path)) else []
    if rows:
        print(f'接著 {out_path} 既有的 {len(rows)} 次繼續記錄')
    print(f'{labels[0]}、{labels[1]} 中間放全長 {length_cm:g} cm 的尺，兩瓶圓心的真實距離 {expected * 100:.2f} cm。'
          f'\n每次擺好、手離開畫面後按 Enter，收 {seconds:g} 秒；輸入 q 結束並總結。')
    try:
        while True:
            line = input(f'\n第 {len(rows) + 1} 次，擺好按 Enter（q 結束）> ').strip().lower()
            if line == 'q':
                break
            for s in samples.values():
                s.clear()
            collecting[0] = True
            end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
            while node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            collecting[0] = False
            missing = [f'{l}（{len(s)} 筆）' for l, s in samples.items() if len(s) < 3]
            if missing:
                print(f'{seconds:g} 秒內收到的太少：{"、".join(missing)}，兩瓶都在畫面裡嗎？這次不記錄')
                continue
            a, b = (np.mean(samples[l], axis=0) for l in labels)
            dist, err, angle = pair_result(a, b, expected)
            print(f'{labels[0]} ({a[0]:+.3f}, {a[1]:+.3f})  {labels[1]} ({b[0]:+.3f}, {b[1]:+.3f})  '
                  f'方向 {angle:+.0f}°  距離 {dist * 100:.2f} cm，誤差 {err * 1000:+.1f} mm')
            rows.append({'placement': str(len(rows) + 1), 'label_a': labels[0], 'label_b': labels[1],
                         'ax': float(a[0]), 'ay': float(a[1]), 'bx': float(b[0]), 'by': float(b[1]),
                         'distance': dist, 'expected': expected, 'error': err, 'angle_deg': angle})
            with open(os.path.expanduser(out_path), 'w', newline='', encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=FIELDS)
                w.writeheader()
                w.writerows(rows)
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f'已存 {len(rows)} 次到 {out_path}')
    summarize(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--labels', nargs=2, metavar=('A', 'B'), help='兩個不同的類別，例如 b2 b3')
    ap.add_argument('--length', type=float, help='尺的實際全長（cm）')
    ap.add_argument('--seconds', type=float, default=3.0, help='每次收幾秒取平均')
    ap.add_argument('--out', default='~/pair_check.csv', help='記錄存到哪個 CSV')
    ap.add_argument('--dims', default=DEFAULT_DIMS, help='bottle_dims.yaml（取瓶身直徑）')
    ap.add_argument('--analyze', metavar='CSV', help='只總結既有的 CSV，不需要 ROS')
    opts = ap.parse_args()
    if opts.analyze:
        summarize(read_csv(opts.analyze))
        return
    if not opts.labels or opts.length is None:
        ap.error('要給 --labels 和 --length，或用 --analyze 總結既有的 CSV')
    if opts.labels[0] == opts.labels[1]:
        ap.error('兩瓶要不同類別：節點每個類別只輸出一個座標')
    if opts.length <= 0.0:
        ap.error('--length 要大於 0')
    try:
        for label in opts.labels:
            bottle_diameter(opts.dims, label)
    except Exception as e:
        ap.error(f'從 {opts.dims} 讀不到瓶身直徑（{e}）')
    record(opts.labels, opts.length, opts.seconds, opts.dims, opts.out)


if __name__ == '__main__':
    sys.exit(main())
