#!/usr/bin/env python3
"""新平台（seven_axis_arm_head_urdf）三軸脖子的頭部模型：由 28/29/30 號馬達刻度算相機在 base_link 的姿態。

純 numpy，桌機的測試、驗證工具和機器人上的節點共用。舊機器人兩軸頭的模型在 head_model.py，兩者無關。

模型：

    T_base_cam(刻度) = FK(關節角) · M
    關節角 = sign × (刻度 − zero_tick) × 360° / 4096
    M = FK(基準刻度)⁻¹ · T_calib

- FK：URDF 從 base_link 經 neck_base -> neckYaw -> neckPitch -> headRoll 到 head_Link 的連桿（config/neck_head.yaml）。
- M：相機（zed_camera_link，ZED 底部螺絲孔）固定在 head_Link 上的位置，用 camera_extrinsics.yaml
  （在基準刻度校正好的外參 T_calib）反推。所以在基準刻度時，結果和 camera_extrinsics.yaml 完全一樣；
  頭轉到別的角度，才用 URDF 算相對的移動。相機實際裝得和 URDF 設計值不一樣也沒關係，M 會吸收。
- neck_tilt_deg（量測修正，可省略）：實際的轉頭軸相對桌面歪了多少，套在連桿第一節（neck_base）之後。
  URDF 的數字不動。
- 結果主要看「離基準刻度轉了多少」和正負號，零點（zero_tick）的影響：
  - 歪頭、低頭：歪頭固定在基準時完全不影響。
  - 轉頭：低頭軸跟著轉頭轉，轉頭零點差 1°，低頭軸方向就差 1°。低頭離基準 Δ 時，相機角度約差 Δ × 1°（弧度），
    允許範圍的角落（低頭差 9°）約 0.16°、位置約 1 mm。
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from head_tf.head_model import (LINK_TO_LEFT_OPTICAL, R_BODY_FROM_OPT, TICKS_PER_REV,
                                axis_angle_matrix, load_extrinsics, matrix_rpy, rpy_matrix)

RAD_PER_TICK = 2.0 * math.pi / TICKS_PER_REV
# ZED wrapper 的 zed_camera_link（螺絲孔）-> zed_camera_center（URDF 裡的前玻璃中心）沿相機 Z 軸 0.016 m
LINK_TO_CENTER = np.array([0.0, 0.0, 0.016])


def transform(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def invert(T):
    R, t = T[:3, :3], T[:3, 3]
    return transform(R.T, -R.T @ t)


def chain_from_urdf(path, base_link='base_link', head_link='head_Link'):
    """從 URDF 讀出 base_link 到 head_link 經過的關節（由 base 往 head 排），格式同 neck_head.yaml 的 chain。"""
    root = ET.parse(path).getroot()
    by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
    chain = []
    link = head_link
    while link != base_link:
        j = by_child.get(link)
        if j is None:
            raise ValueError(f'URDF 裡找不到從 {base_link} 到 {head_link} 的連桿（卡在 {link}）')
        origin = j.find('origin')
        xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
        rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
        entry = {'name': j.get('name'), 'type': j.get('type'), 'xyz': xyz, 'rpy': rpy}
        if j.get('type') in ('revolute', 'continuous'):
            axis = j.find('axis')
            entry['axis'] = [float(v) for v in (axis.get('xyz') if axis is not None else '1 0 0').split()]
            limit = j.find('limit')
            if limit is not None:
                entry['limit'] = [float(limit.get('lower')), float(limit.get('upper'))]
        elif j.get('type') != 'fixed':
            raise ValueError(f'不支援的關節型別 {j.get("type")}（{j.get("name")}）')
        chain.append(entry)
        link = j.find('parent').get('link')
    return chain[::-1]


def child_offset_from_urdf(path, parent_link, child_link):
    """URDF 裡 parent_link -> child_link 固定關節的 xyz（讀相機設計位置用）。"""
    root = ET.parse(path).getroot()
    for j in root.findall('joint'):
        if j.find('parent').get('link') == parent_link and j.find('child').get('link') == child_link:
            return [float(v) for v in j.find('origin').get('xyz').split()]
    raise ValueError(f'URDF 裡沒有 {parent_link} -> {child_link}')


class NeckModel:
    """config 是 neck_head.yaml 的內容；R0、t0 是基準刻度時 zed_camera_link 在 base_link 的姿態（camera_extrinsics.yaml）。"""

    def __init__(self, config, R0, t0):
        self.chain = config['chain']
        self.motors = {name: dict(m) for name, m in config['motors'].items()}
        for joint in self.chain:
            if joint['type'] != 'fixed' and joint['name'] not in self.motors:
                raise ValueError(f'neck_head.yaml 的 motors 沒有 {joint["name"]}')
        self.motor_ids = [self.motors[j['name']]['motor_id'] for j in self.chain if j['type'] != 'fixed']
        self.reference_ticks = {int(k): float(v) for k, v in config['reference_ticks'].items()}
        self.allowed_ticks = {int(k): (float(v[0]), float(v[1])) for k, v in config['allowed_ticks'].items()}
        for mid in self.motor_ids:
            if mid not in self.reference_ticks or mid not in self.allowed_ticks:
                raise ValueError(f'neck_head.yaml 的 reference_ticks / allowed_ticks 少了 {mid} 號馬達')
            lo, hi = self.allowed_ticks[mid]
            if not lo <= self.reference_ticks[mid] <= hi:
                raise ValueError(f'{mid} 號馬達的基準刻度 {self.reference_ticks[mid]:.0f} 不在允許範圍 {lo:.0f}～{hi:.0f}')
        self.design_center = np.asarray(config.get('camera_center_in_head', [0, 0, 0]), dtype=float)
        tilt = config.get('neck_tilt_deg') or {}
        self.neck_tilt_deg = (float(tilt.get('roll', 0.0)), float(tilt.get('pitch', 0.0)))
        self.tilt_R = rpy_matrix(math.radians(self.neck_tilt_deg[0]), math.radians(self.neck_tilt_deg[1]), 0.0)
        self.margin = float(config.get('range_margin_ticks', 0))
        T_calib = transform(np.asarray(R0, dtype=float), np.asarray(t0, dtype=float))
        self.mount = invert(self.head_transform(self.reference_ticks)) @ T_calib

    @classmethod
    def load(cls, config_path, extrinsics_path):
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        R0, t0 = load_extrinsics(extrinsics_path)
        return cls(config, R0, t0)

    def joint_angles(self, ticks):
        """{馬達 ID: 刻度} -> {URDF 關節名稱: 角度（rad）}。"""
        out = {}
        for joint in self.chain:
            if joint['type'] == 'fixed':
                continue
            m = self.motors[joint['name']]
            mid = m['motor_id']
            if mid not in ticks:
                raise KeyError(f'沒有 {mid} 號馬達的刻度')
            out[joint['name']] = m['sign'] * (float(ticks[mid]) - m.get('zero_tick', 2048)) * RAD_PER_TICK
        return out

    def head_transform(self, ticks):
        """head_Link 在 base_link 的 4x4。"""
        angles = self.joint_angles(ticks)
        T = np.eye(4)
        for i, joint in enumerate(self.chain):
            T = T @ transform(rpy_matrix(*joint['rpy']), joint['xyz'])
            if i == 0:
                T = T @ transform(self.tilt_R, np.zeros(3))
            if joint['type'] != 'fixed':
                T = T @ transform(axis_angle_matrix(joint['axis'], angles[joint['name']]), np.zeros(3))
        return T

    def camera_link_pose(self, ticks):
        """zed_camera_link（螺絲孔）在 base_link 的 (R, t)。"""
        T = self.head_transform(ticks) @ self.mount
        return T[:3, :3], T[:3, 3]

    def optical_pose(self, ticks):
        """左眼 optical frame 在 base_link 的 (R, 光心位置)。"""
        R, t = self.camera_link_pose(ticks)
        return R @ R_BODY_FROM_OPT, t + R @ LINK_TO_LEFT_OPTICAL

    def check(self, ticks):
        """回 (可以用, 原因列表)。超出允許範圍（兩端各放寬 range_margin_ticks）或 URDF 關節極限都算不能用。"""
        reasons = []
        for mid in self.motor_ids:
            if mid not in ticks:
                reasons.append(f'沒有 {mid} 號馬達的刻度')
                continue
            lo, hi = self.allowed_ticks[mid]
            if not lo - self.margin <= float(ticks[mid]) <= hi + self.margin:
                reasons.append(f'{mid} 號馬達刻度 {float(ticks[mid]):.0f} 超出允許範圍 {lo:.0f}～{hi:.0f}')
        if not reasons:
            for joint in self.chain:
                lim = joint.get('limit')
                a = self.joint_angles(ticks).get(joint['name'])
                if lim and a is not None and not lim[0] <= a <= lim[1]:
                    reasons.append(f'{joint["name"]} {math.degrees(a):+.1f}° 超出 URDF 極限')
        return not reasons, reasons

    def mount_vs_design(self):
        """反推的相機固定位置和 URDF 設計值比：回 (前玻璃中心位置差 m 的向量, 相機相對 head_Link 的 rpy 度)。

        URDF 設計：相機前玻璃中心 zed_camera_center 在 head_Link 的 camera_center_in_head，方向和 head_Link 一樣。
        差很多（例如超過 1 cm 或 3°）代表外參、基準刻度或刻度方向有問題。
        """
        R, t = self.mount[:3, :3], self.mount[:3, 3]
        a = 0.0129     # ZED X Mini 前玻璃在螺絲孔前方的距離（zed_stl_front.py）
        center = t + R @ (LINK_TO_CENTER + np.array([a, 0.0, 0.0]))
        return center - self.design_center, [math.degrees(v) for v in matrix_rpy(R)]


def default_config():
    """neck_head.yaml：先找安裝的 share，找不到用原始碼旁邊的。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('head_tf'), 'config', 'neck_head.yaml')
        if os.path.exists(path):
            return path
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'neck_head.yaml')


def load_config(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def sweep_plan(config, tilt_steps=5):
    """head_sweep 在新平台的預設：從 neck_head.yaml 的基準刻度和允許範圍算出要量的姿勢。

    轉頭量允許範圍的兩端和基準；低頭在允許範圍內平均取 tilt_steps 個（含基準和兩端）。
    收集範圍就是允許範圍，安全上限也用允許範圍。歪頭不轉，只回它應該停的範圍。

    Returns:
        dict：pan_id / tilt_id / roll_id（馬達 ID）、reference (pan, tilt)、pan、tilt（刻度列表）、
        pan_limits、tilt_limits、roll_limits（含兩端）。
    """
    ids = {name: int(m['motor_id']) for name, m in config['motors'].items()}
    pan_id, tilt_id, roll_id = ids['neckYawJoint'], ids['neckPitchJoint'], ids['headRollJoint']
    ref = {int(k): int(v) for k, v in config['reference_ticks'].items()}
    lim = {int(k): (int(v[0]), int(v[1])) for k, v in config['allowed_ticks'].items()}
    p_lo, p_hi = lim[pan_id]
    t_lo, t_hi = lim[tilt_id]
    if tilt_steps < 3 or tilt_steps % 2 == 0:
        raise ValueError('tilt_steps 要是 3 以上的奇數（基準在中間）')
    half = tilt_steps // 2
    tilt = ([round(t_lo + (ref[tilt_id] - t_lo) * i / half) for i in range(half)] + [ref[tilt_id]]
            + [round(ref[tilt_id] + (t_hi - ref[tilt_id]) * i / half) for i in range(1, half + 1)])
    return {
        'pan_id': pan_id, 'tilt_id': tilt_id, 'roll_id': roll_id,
        'reference': (ref[pan_id], ref[tilt_id]), 'roll_reference': ref[roll_id],
        'pan': sorted({p_lo, ref[pan_id], p_hi}), 'tilt': sorted(set(int(t) for t in tilt)),
        'pan_limits': lim[pan_id], 'tilt_limits': lim[tilt_id], 'roll_limits': lim[roll_id],
    }


def describe(model):
    """印給人看的摘要：基準姿勢、相機裝在頭上的位置和設計值比、允許範圍四個角的相機姿態。"""
    lines = []
    ref = model.reference_ticks
    ang = model.joint_angles(ref)
    lines.append('基準刻度：' + '、'.join(f'{mid} 號 {ref[mid]:.0f}' for mid in model.motor_ids)
                 + '（' + '、'.join(f'{k} {math.degrees(v):+.2f}°' for k, v in ang.items()) + '）')
    r, p = model.neck_tilt_deg
    if r or p:
        lines.append(f'轉頭軸傾斜修正：roll {r:+.2f}°、pitch {p:+.2f}°（neck_tilt_deg）')
    d, rpy = model.mount_vs_design()
    lines.append(f'相機裝在頭上的位置和 URDF 設計值差：前 {d[0]*100:+.1f}、左 {d[1]*100:+.1f}、上 {d[2]*100:+.1f} cm；'
                 f'角度 roll {rpy[0]:+.2f}°、pitch {rpy[1]:+.2f}°、yaw {rpy[2]:+.2f}°')
    lines.append('允許範圍內的相機姿態（左眼光心，base_link，cm / 度）：')
    yaw_id, pitch_id = model.motor_ids[0], model.motor_ids[1]
    for y in (model.allowed_ticks[yaw_id][0], ref[yaw_id], model.allowed_ticks[yaw_id][1]):
        for p in (model.allowed_ticks[pitch_id][0], ref[pitch_id], model.allowed_ticks[pitch_id][1]):
            ticks = {**ref, yaw_id: y, pitch_id: p}
            R, c = model.optical_pose(ticks)
            r, pi, ya = (math.degrees(v) for v in matrix_rpy(R @ R_BODY_FROM_OPT.T))
            lines.append(f'  {yaw_id} 號 {y:.0f}、{pitch_id} 號 {p:.0f}：位置 ({c[0]*100:+6.1f}, {c[1]*100:+6.1f}, '
                         f'{c[2]*100:+6.1f})  roll {r:+5.1f}  pitch {pi:+5.1f}  yaw {ya:+6.1f}')
    return lines


def main(argv=None):
    import argparse
    import os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description='印出新平台頭部模型的摘要（檢查設定用）')
    ap.add_argument('--config', default=os.path.join(here, 'config', 'neck_head.yaml'))
    ap.add_argument('--extrinsics', default=os.path.join(os.path.dirname(here), 'bottle_localization',
                                                         'config', 'camera_extrinsics.yaml'))
    args = ap.parse_args(argv)
    for line in describe(NeckModel.load(args.config, args.extrinsics)):
        print(line)


if __name__ == '__main__':
    main()
