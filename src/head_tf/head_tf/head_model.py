#!/usr/bin/env python3
"""頭部模型：由頭部馬達刻度算 zed_camera_link（ZED 底部螺絲孔）在 base_link 的姿態。

純 numpy，桌機的擬合工具和機器人上的節點共用。

模型（T0 = 基準刻度時的姿態，也就是 camera_extrinsics.yaml）：

    T(pan, tilt) = 繞 pan 軸轉 Δpan ∘ 繞 tilt 軸轉 Δtilt ∘ T0
    Δ = sign × scale × (刻度 − 基準刻度) × 360 / 4096

- tilt 馬達裝在 pan 馬達上，tilt 軸會跟著 pan 一起轉，所以先轉 tilt、再轉 pan。
- 兩個軸的方向和位置都用「基準刻度時的 base_link」表示。
- 位置存成「軸上一點 -> 螺絲孔」的向量（和手量的方式相同）：螺絲孔的位置來自 camera_extrinsics.yaml，
  外參重新校正時，軸會跟著螺絲孔一起移。
"""
import math

import numpy as np
import yaml

TICKS_PER_REV = 4096

# ZED X Mini 的 URDF（zed_wrapper 發的 /tf_static，2026-10 機器人上錄的 bag）：
#   zed_camera_link -> zed_camera_center (0, 0, 0.016) -> zed_left_camera_frame (-0.010, 0.025, 0)
#   -> zed_left_camera_optical_frame（只有軸向旋轉）
LINK_TO_LEFT_OPTICAL = np.array([-0.010, 0.025, 0.016])
# optical frame（X 右、Y 下、Z 前）的軸在相機本體座標（X 前、Y 左、Z 上）裡的方向
R_BODY_FROM_OPT = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0]])


def axis_angle_matrix(axis, angle):
    """繞單位向量 axis 轉 angle（弧度）的旋轉矩陣（Rodrigues）。"""
    x, y, z = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def rpy_matrix(roll, pitch, yaw):
    """繞固定軸 X -> Y -> Z（URDF / static_transform_publisher 的慣例）= Rz(yaw) Ry(pitch) Rx(roll)。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def matrix_rpy(R):
    """rpy_matrix 的反函數，回 (roll, pitch, yaw)。"""
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    return math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])


def load_extrinsics(path):
    """camera_extrinsics.yaml -> (R0, t0)：基準刻度時 zed_camera_link 在 base_link 的姿態。"""
    with open(path, 'r', encoding='utf-8') as f:
        e = yaml.safe_load(f)
    R0 = rpy_matrix(float(e['roll']), float(e['pitch']), float(e['yaw']))
    t0 = np.array([float(e['x']), float(e['y']), float(e['z'])])
    return R0, t0


class Joint:
    """一個轉軸。direction 是單位向量；offset 是「軸上一點 -> 螺絲孔」（m），都在基準刻度時的 base_link。"""

    def __init__(self, direction, offset, sign=1, scale=1.0):
        d = np.asarray(direction, dtype=float)
        self.direction = d / np.linalg.norm(d)
        self.offset = np.asarray(offset, dtype=float)
        self.sign = int(sign)
        self.scale = float(scale)

    def to_dict(self):
        return {'sign': self.sign, 'scale': round(self.scale, 6),
                'direction': [round(float(v), 6) for v in self.direction],
                'offset_to_screw': [round(float(v), 5) for v in self.offset]}

    @classmethod
    def from_dict(cls, d):
        return cls(d['direction'], d['offset_to_screw'], d.get('sign', 1), d.get('scale', 1.0))


class HeadModel:
    def __init__(self, pan, tilt, reference_ticks=(2048, 2300), deg_per_tick=360.0 / TICKS_PER_REV):
        self.pan = pan
        self.tilt = tilt
        self.reference_ticks = (float(reference_ticks[0]), float(reference_ticks[1]))
        self.rad_per_tick = math.radians(deg_per_tick)

    def angles(self, pan_ticks, tilt_ticks):
        """(Δpan, Δtilt)，弧度。"""
        return (self.pan.sign * self.pan.scale * (pan_ticks - self.reference_ticks[0]) * self.rad_per_tick,
                self.tilt.sign * self.tilt.scale * (tilt_ticks - self.reference_ticks[1]) * self.rad_per_tick)

    def camera_link_pose(self, R0, t0, pan_ticks, tilt_ticks):
        """zed_camera_link 在 base_link 的 (R, t)。R0、t0 是基準刻度時的姿態（camera_extrinsics.yaml）。"""
        a_pan, a_tilt = self.angles(pan_ticks, tilt_ticks)
        t0 = np.asarray(t0, dtype=float)
        c_tilt = t0 - self.tilt.offset
        c_pan = t0 - self.pan.offset
        Rt = axis_angle_matrix(self.tilt.direction, a_tilt)
        R = Rt @ R0
        t = Rt @ (t0 - c_tilt) + c_tilt
        Rp = axis_angle_matrix(self.pan.direction, a_pan)
        return Rp @ R, Rp @ (t - c_pan) + c_pan

    def optical_pose(self, R0, t0, pan_ticks, tilt_ticks):
        """左眼 optical frame 在 base_link 的 (R, 光心位置)。"""
        R, t = self.camera_link_pose(R0, t0, pan_ticks, tilt_ticks)
        return R @ R_BODY_FROM_OPT, t + R @ LINK_TO_LEFT_OPTICAL

    def to_dict(self):
        return {'reference_ticks': {'pan': int(self.reference_ticks[0]), 'tilt': int(self.reference_ticks[1])},
                'deg_per_tick': math.degrees(self.rad_per_tick),
                'pan': self.pan.to_dict(), 'tilt': self.tilt.to_dict()}

    @classmethod
    def from_dict(cls, d):
        ref = d.get('reference_ticks', {'pan': 2048, 'tilt': 2300})
        return cls(Joint.from_dict(d['pan']), Joint.from_dict(d['tilt']),
                   (ref['pan'], ref['tilt']), d.get('deg_per_tick', 360.0 / TICKS_PER_REV))

    @classmethod
    def load(cls, path):
        with open(path, 'r', encoding='utf-8') as f:
            return cls.from_dict(yaml.safe_load(f))

    @classmethod
    def from_measured(cls, path):
        """head_axes.yaml（手量值）-> 模型：方向用檔案裡的（預設 pan 垂直、tilt 水平），比例 1。"""
        with open(path, 'r', encoding='utf-8') as f:
            m = yaml.safe_load(f)
        p, t = m['pan_axis_to_screw_cm'], m['tilt_axis_to_screw_cm']
        pan = Joint(m.get('pan_axis_direction', [0, 0, 1]),
                    [p['forward'] / 100, p.get('left', 0.0) / 100, 0.0], m.get('pan_sign') or 1)
        tilt = Joint(m.get('tilt_axis_direction', [0, 1, 0]),
                     [t['forward'] / 100, 0.0, t['up'] / 100], m.get('tilt_sign') or 1)
        ref = m.get('reference_ticks', {'pan': 2048, 'tilt': 2300})
        return cls(pan, tilt, (ref['pan'], ref['tilt']))
