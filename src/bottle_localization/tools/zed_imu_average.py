#!/usr/bin/env python3
#coding=utf-8
"""讀 ZED IMU 的 roll / pitch，取幾秒平均（在機器人上執行，ZED 要開著）。

    python3 zed_imu_average.py            # 平均 5 秒
    python3 zed_imu_average.py 10         # 平均 10 秒

量的時候機器人擺好抓取姿勢、保持不動。算法與 walking/zed_imu_node.py 的 abs_roll / abs_pitch
相同（ZYX，往下看 pitch 為正），不受網頁上「IMU 歸零」影響。
"""
import math
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
rclpy.init()
node = rclpy.create_node('zed_imu_average')
samples = []


def callback(m):
    x, y, z, w = m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w
    samples.append((math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                    math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))))


node.create_subscription(Imu, '/zed/zed_node/imu/data', callback, qos_profile_sensor_data)
end = time.time() + seconds
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)

if not samples:
    print(f'{seconds:g} 秒內沒收到 /zed/zed_node/imu/data，ZED 有在跑嗎？')
    sys.exit(1)
n = len(samples)
for name, k in (('abs_roll ', 0), ('abs_pitch', 1)):
    v = [s[k] for s in samples]
    mean = sum(v) / n
    std = (sum((t - mean) ** 2 for t in v) / n) ** 0.5
    print(f'{name} = {math.degrees(mean):+.2f} 度（跳動 ±{math.degrees(std):.2f}）= {mean:+.4f} rad')
print(f'{n} 筆，{seconds:g} 秒平均。跳動大於 ±0.3 度代表機器人在晃，等穩定再量。')
