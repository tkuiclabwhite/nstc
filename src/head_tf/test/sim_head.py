"""測試用的模擬器：有 pan / tilt 兩軸的頭、桌面、圓柱形的瓶子。

頭部模型（和 head_tf/head_model.py 同一個形式，但這裡獨立寫一份，用來檢查那邊）：
    螺絲孔姿態(pan, tilt) = 繞 pan 軸轉 Δpan ∘ 繞 tilt 軸轉 Δtilt ∘ 基準姿態
pan 軸預設是鉛直線，tilt 軸預設平行 base 的 y 軸，兩個都可以傾斜；角度 = 刻度差 × 360/4096 × 方向。
tilt 方向 +1 = 刻度變大往下看；pan 方向 +1 = 刻度變大往左轉。
cam_pos0 是螺絲孔（zed_camera_link）；左眼光心在它前方 -1.0、左 2.5、上 1.6 cm（相機本體座標）。

SimIO 實作 sweep_runner 要的 io 介面，時間是假的（sleep 只是把時鐘往前推）。
"""
import math

import numpy as np

from head_tf import sweep_core as core


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rot_axis(axis, a):
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


LINK_TO_OPTICAL = np.array([-0.010, 0.025, 0.016])


class HeadSim:
    def __init__(self, pan_sign=1, tilt_sign=1, pitch0_deg=31.0, roll0_deg=0.7,
                 pan_dir=(0.0, 0.0, 1.0), tilt_dir=(0.0, 1.0, 0.0), imu_rpy_deg=(0.0, 0.0, 0.0)):
        self.pan_sign = pan_sign
        self.tilt_sign = tilt_sign
        self.pan_dir = np.asarray(pan_dir, dtype=float) / np.linalg.norm(pan_dir)
        self.tilt_dir = np.asarray(tilt_dir, dtype=float) / np.linalg.norm(tilt_dir)
        r, p, y = (math.radians(v) for v in imu_rpy_deg)
        self.imu_mount = rot_z(y) @ rot_y(p) @ rot_x(r)
        self.ref = core.REFERENCE_TICKS
        self.z_table = -0.18
        self.cam_pos0 = np.array([0.09, 0.09, 0.15])
        self.R_body0 = rot_y(math.radians(pitch0_deg)) @ rot_x(math.radians(roll0_deg))
        self.pan_point = np.array([0.03, 0.09, 0.0])
        self.tilt_point = np.array([0.06, 0.0, 0.12])
        # (x, y, 半徑, 高)
        self.bottles = {'b1': (0.55, 0.05, 0.035, 0.10),
                        'b2': (0.62, 0.17, 0.030, 0.09),
                        'b3': (0.68, -0.04, 0.040, 0.12)}
        self.width, self.height = 240, 150
        self.k = (120.0, 120.0, 119.5, 74.5)

    def pose(self, pan, tilt):
        """(相機本體的旋轉, 螺絲孔位置)，都在 base 座標。"""
        a_pan = self.pan_sign * math.radians((pan - self.ref[0]) * core.DEG_PER_TICK)
        a_tilt = self.tilt_sign * math.radians((tilt - self.ref[1]) * core.DEG_PER_TICK)
        Rt = rot_axis(self.tilt_dir, a_tilt)
        R = Rt @ self.R_body0
        p = Rt @ (self.cam_pos0 - self.tilt_point) + self.tilt_point
        Rp = rot_axis(self.pan_dir, a_pan)
        R = Rp @ R
        p = Rp @ (p - self.pan_point) + self.pan_point
        return R, p

    def expected_table(self, pan, tilt):
        """相機相對桌面的 (roll 度, pitch 度, 光心離桌面 m)。"""
        R, _ = self.pose(pan, tilt)
        _, c = self.optical(pan, tilt)
        up = R.T @ np.array([0.0, 0.0, 1.0])
        return (math.degrees(math.atan2(up[1], up[2])), math.degrees(math.asin(-up[0])),
                c[2] - self.z_table)

    def optical(self, pan, tilt):
        R, p = self.pose(pan, tilt)
        return R @ core.R_BODY_FROM_OPT, p + R @ LINK_TO_OPTICAL

    def render(self, pan, tilt):
        R_opt, c = self.optical(pan, tilt)
        fx, fy, cx, cy = self.k
        v, u = np.mgrid[0:self.height, 0:self.width].astype(float)
        d_opt = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=-1)
        d = d_opt @ R_opt.T
        depth = np.full((self.height, self.width), np.nan)
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (self.z_table - c[2]) / d[..., 2]
        depth = np.where((d[..., 2] < 0) & (t > 0), t, depth)
        for bx, by, r, h in self.bottles.values():
            ox, oy = c[0] - bx, c[1] - by
            a = d[..., 0] ** 2 + d[..., 1] ** 2
            b = 2 * (ox * d[..., 0] + oy * d[..., 1])
            cc = ox ** 2 + oy ** 2 - r ** 2
            disc = b ** 2 - 4 * a * cc
            with np.errstate(invalid='ignore', divide='ignore'):
                t1 = (-b - np.sqrt(disc)) / (2 * a)
                z_hit = c[2] + t1 * d[..., 2]
                side = (disc > 0) & (t1 > 0) & (z_hit >= self.z_table) & (z_hit <= self.z_table + h)
                t_cap = (self.z_table + h - c[2]) / d[..., 2]
                px = c[0] + t_cap * d[..., 0] - bx
                py = c[1] + t_cap * d[..., 1] - by
                cap = (t_cap > 0) & (px ** 2 + py ** 2 <= r ** 2)
            hit = np.where(side, t1, np.inf)
            hit = np.minimum(hit, np.where(cap, t_cap, np.inf))
            closer = np.isfinite(hit) & ~(depth <= hit)
            depth = np.where(closer, hit, depth)
        return depth.astype(np.float32)

    def bbox(self, label, pan, tilt):
        R_opt, c = self.optical(pan, tilt)
        bx, by, r, h = self.bottles[label]
        ang = np.linspace(0, 2 * math.pi, 48, endpoint=False)
        ring = np.column_stack([bx + r * np.cos(ang), by + r * np.sin(ang)])
        pts = np.vstack([np.column_stack([ring, np.full(48, self.z_table)]),
                         np.column_stack([ring, np.full(48, self.z_table + h)])])
        q = (pts - c) @ R_opt
        if np.any(q[:, 2] <= 0):
            return None
        fx, fy, cx, cy = self.k
        u = fx * q[:, 0] / q[:, 2] + cx
        v = fy * q[:, 1] / q[:, 2] + cy
        box = (u.min(), v.min(), u.max(), v.max())
        if box[0] < 3 or box[1] < 3 or box[2] > self.width - 4 or box[3] > self.height - 4:
            return None
        return box

    def imu_roll_pitch(self, pan, tilt):
        R, _ = self.pose(pan, tilt)
        R = R @ self.imu_mount
        return math.atan2(R[2, 1], R[2, 2]), -math.asin(R[2, 0]), R.T @ np.array([0, 0, 9.81])


class SimIO:
    """模擬馬達（一階逼近目標）、/joint_states（50 Hz）、ZED、YOLO。"""

    def __init__(self, sim, start=None, stuck=False, stale_after=None, tilt_sag=0,
                 noise_m=0.0005, seed=0):
        self.sim = sim
        self.t = 100.0
        self.pan, self.tilt = map(float, start or sim.ref)
        self.goal = (self.pan, self.tilt)
        self.speed = 40
        self.stuck = stuck
        self.stale_after = stale_after
        self.tilt_sag = tilt_sag
        self.noise_m = noise_m
        self.rng = np.random.default_rng(seed)
        self.joints = []
        self.sent = []
        self._last_sample = -1.0
        self._sample()

    def _sample(self):
        if self.stale_after is not None and self.t > self.stale_after:
            return
        self.joints.append((self.t, float(round(self.pan)), float(round(self.tilt))))
        self._last_sample = self.t

    def now(self):
        return self.t

    def sleep(self, seconds):
        end = self.t + max(0.0, seconds)
        while self.t < end - 1e-9:
            dt = min(0.02, end - self.t)
            self.t += dt
            rate = self.speed * 0.229 * 4096 / 60 * dt
            for attr, target in (('pan', self.goal[0]), ('tilt', self.goal[1] - self.tilt_sag)):
                cur = getattr(self, attr)
                step = max(-rate, min(rate, target - cur))
                setattr(self, attr, cur + step)
            if self.t - self._last_sample >= 0.02 - 1e-9:
                self._sample()

    def send_head(self, pan, tilt, speed):
        self.sent.append((pan, tilt, speed))
        if not self.stuck:
            self.goal = (float(pan), float(tilt))
            self.speed = speed

    def latest_joint(self):
        return self.joints[-1] if self.joints else None

    def joints_since(self, t):
        return [s for s in self.joints if s[0] >= t]

    def snapshot(self, t0, t1, max_frames):
        pan, tilt = round(self.pan), round(self.tilt)
        base = self.sim.render(pan, tilt)
        n = min(max_frames, max(1, int((t1 - t0) * 5)))
        frames = [base + self.rng.normal(0, self.noise_m, base.shape).astype(np.float32)
                  for _ in range(n)]
        roll, pitch, acc = self.sim.imu_roll_pitch(pan, tilt)
        imu = [(t, roll, pitch, acc[0], acc[1], acc[2], 0.0, 0.0, 0.0)
               for t in np.arange(t0, t1, 0.01)]
        bottles = []
        for label in self.sim.bottles:
            box = self.sim.bbox(label, pan, tilt)
            if box is not None:
                bottles.append({'label': label, 'confidence': 0.9, 'bbox': list(box)})
        dets = [(t, {'stamp': {'sec': int(t), 'nanosec': 0}, 'bottles': bottles,
                     'image_size': [self.sim.width, self.sim.height]})
                for t in np.arange(t0, t1, 0.1)]
        color = np.zeros((self.sim.height, self.sim.width, 3), dtype=np.uint8)
        return core.Snapshot(depth_frames=frames, k=self.sim.k, size=(self.sim.width, self.sim.height),
                             frame_id='zed_left_camera_optical_frame', color=color,
                             joints=[s for s in self.joints if t0 <= s[0] <= t1], imu=imu, detections=dets)
