#!/usr/bin/env python3
#coding=utf-8
"""由量測值算出 camera_extrinsics.yaml 要填的值（純計算，在哪台電腦執行都可以）。

    python3 extrinsics_calc.py --dx 2.5 --dy 12.8 --dz 19.5 --pitch 38.5 --roll 0.1 --a 0.02

    --dx/--dy/--dz   抓取姿勢下，右肩關節點 -> 鏡頭中點（前面玻璃上兩顆鏡頭圓心連線的中點），
                     沿 base_link 的 前 / 左 / 上 分解，單位 cm
    --pitch/--roll   zed_imu_average.py 的 abs_pitch / abs_roll，單位度
    --a              zed_stl_front.py 的 a，單位 m

換算:
    鏡頭中點(base_link) = 量測值 + 右肩關節點在 base_link 的位置（armurdf4 的 RshoulderJoint）
    螺絲孔 = 鏡頭中點 - R(roll, pitch) * (a, 0, 0.016)   （鏡頭光軸比螺絲孔高 1.6 cm，ZED URDF）
yaw 量不到，先填 0，之後用已知位置的瓶子微調。
"""
import argparse
import math

#右肩關節點在 base_link 的位置（armurdf4_with_hand.urdf 的 RshoulderJoint origin）
SHOULDER_JOINT = (-0.01375, -0.0436, 0.01475)
#鏡頭光軸比底部螺絲孔高多少（ZED URDF：zed_camera_center 在 zed_camera_link 上方 height/2）
LENS_ABOVE_SCREW = 0.016
#roll 小於這個角度（度）就當 0
ROLL_DEADBAND_DEG = 0.3

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
for name, unit in (('dx', 'cm'), ('dy', 'cm'), ('dz', 'cm'), ('pitch', '度'), ('roll', '度'), ('a', 'm')):
    p.add_argument(f'--{name}', type=float, required=True, help=unit)
o = p.parse_args()

pitch = math.radians(o.pitch)
roll = 0.0 if abs(o.roll) < ROLL_DEADBAND_DEG else math.radians(o.roll)
lens = (o.dx / 100 + SHOULDER_JOINT[0], o.dy / 100 + SHOULDER_JOINT[1], o.dz / 100 + SHOULDER_JOINT[2])
cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
c = LENS_ABOVE_SCREW
#Ry(pitch) * Rx(roll) * (a, 0, c)
off = (o.a * cp + c * cr * sp, -c * sr, -o.a * sp + c * cr * cp)
x, y, z = (lens[i] - off[i] for i in range(3))

print(f'鏡頭中點在 base_link：({lens[0]:.4f}, {lens[1]:.4f}, {lens[2]:.4f}) m')
print(f'roll {"視為 0" if roll == 0.0 else f"{o.roll:.2f} 度"}，pitch {o.pitch:.2f} 度')
print('---- 貼進 camera_extrinsics.yaml（取代原本的 measured 與 x ~ yaw）----')
print('measured: true')
print(f'x: {x:.4f}\ny: {y:.4f}\nz: {z:.4f}\nroll: {roll:.4f}\npitch: {pitch:.4f}\nyaw: 0.0')
