#!/usr/bin/env python3
#coding=utf-8
"""從 ZED X Mini 的 3D 模型讀出「前面玻璃在螺絲孔前方多遠」（a），在機器人上執行。

    source ~/ros2_adult/install/setup.bash
    python3 zed_stl_front.py                 # 自動找 zed_msgs/meshes/zedxm.stl
    python3 zed_stl_front.py /path/to/zedxm.stl

ZED 的 URDF 把模型原點放在 zed_camera_link（底部螺絲孔），所以 x 的最大值就是 a。
z 的範圍應該大約是 0 ~ 0.032（機身高 3.2 cm）；差很多代表模型原點不在螺絲孔，先不要用。
"""
import os
import sys

import numpy as np

if len(sys.argv) > 1:
    path = sys.argv[1]
else:
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('zed_msgs'), 'meshes', 'zedxm.stl')
data = open(path, 'rb').read()
n = int.from_bytes(data[80:84], 'little')
if len(data) == 84 + 50 * n:              #binary STL
    rec = np.frombuffer(data[84:], np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    v = rec['v'].reshape(-1, 3).astype(np.float64)
else:                                     #ASCII STL
    v = np.array([[float(t) for t in line.split()[1:4]]
                  for line in data.decode(errors='ignore').splitlines()
                  if line.strip().startswith('vertex')])
lo, hi = v.min(axis=0), v.max(axis=0)
print(path)
for k, axis in enumerate('xyz'):
    print(f'{axis}  min {lo[k]:+.4f}  max {hi[k]:+.4f}')
print(f'a = {hi[0]:.4f} m（x 的最大值）')
