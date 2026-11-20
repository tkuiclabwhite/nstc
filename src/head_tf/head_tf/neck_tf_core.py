#!/usr/bin/env python3
"""動態相機 TF 的計算部分（不用 ROS）：/joint_states 的刻度 -> base_link -> zed_camera_link，加上可不可用的判斷。

ROS 節點在 neck_tf_publisher.py；這裡分出來是為了在桌機不開 ROS 就能測。

可不可用（不可用時節點停止發 TF，bottle_localizer 查不到 TF 就跳過那一幀，不會發出錯的藥罐座標）：
- 三顆馬達（28/29/30）的刻度都要有；
- 都在 neck_head.yaml 的允許範圍內（兩端各放寬 range_margin_ticks）；
- 最近 stale_s 秒內收到過有頭部馬達刻度的 /joint_states（MoveIt 發的手臂 /joint_states 不算）。

「頭在動」只寫在狀態裡，TF 照發：最近 still_s 秒內任何一顆馬達變化超過 still_ticks 格。
頭在動時，馬達回授和影像的時間對不太準，藥罐位置會差一點，要準的話等頭停下來再用。
"""
import collections
import math

import numpy as np

from head_tf.head_model import matrix_rpy


def parse_joint_state(names, positions, motor_ids):
    """/joint_states 的 name、position -> ({馬達 ID: 刻度}, 缺少的 ID 列表)。

    回 (None, []) 的訊息直接略過：
    - driver_node 每一輪發三筆 JointState（位置、速度、電流），速度和電流那兩筆的 position 是空的；
    - MoveIt（ros2_control 的 joint_state_broadcaster）也發 /joint_states，名稱是手臂關節名、單位弧度，
      完全沒有頭部馬達（2026-11-20 .11 上發現：沒略過的話狀態會在可用 / 不可用之間一直跳）。
    driver_node 的名稱是馬達 ID 的字串（'28'），position 是原始刻度。有一部分頭部馬達、缺一部分才回缺少的 ID。
    """
    if len(positions) != len(names) or not names:
        return None, []
    index = {str(n): i for i, n in enumerate(names)}
    if not any(str(mid) in index for mid in motor_ids):
        return None, []
    ticks, missing = {}, []
    for mid in motor_ids:
        i = index.get(str(mid))
        if i is None or not math.isfinite(float(positions[i])):
            missing.append(mid)
        else:
            ticks[mid] = float(positions[i])
    return ticks, missing


def matrix_to_quaternion(R):
    """3x3 旋轉矩陣 -> 四元數 (x, y, z, w)。"""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return tuple(float(v) for v in q)


class NeckTfCore:
    """收刻度、判斷可不可用、算 zed_camera_link 在 base_link 的姿態。時間一律用呼叫端給的秒數（節點用 monotonic）。"""

    def __init__(self, model, stale_s=0.5, still_ticks=3.0, still_s=0.3):
        self.model = model
        self.stale_s = float(stale_s)
        self.still_ticks = float(still_ticks)
        self.still_s = float(still_s)
        self.last_t = None          # 最後一次收到 /joint_states（有位置的那筆）的時間
        self.ticks = None           # 最後一次的刻度 {ID: 刻度}
        self.reasons = ['還沒收到有 28/29/30 號刻度的 /joint_states（driver_node 有在跑嗎？頭部馬達有上電嗎？）']
        self.pose = None            # 最後一次可用的 (R, t)
        self._history = collections.deque()

    def set_model(self, model):
        """外參改了（bottle_tuner）時換模型；最後一次的刻度重新算。"""
        self.model = model
        if self.ticks is not None and self.last_t is not None:
            self._evaluate()

    def update(self, ticks, missing, t):
        """收到一筆有位置的 /joint_states。回 (可以用, (R, t) 或 None)。"""
        self.last_t = float(t)
        if missing:
            self.ticks = dict(ticks)
            self.reasons = [f'/joint_states 沒有 {", ".join(str(m) for m in missing)} 號馬達的刻度（頭部馬達有上電嗎？）']
            self.pose = None
            return False, None
        self.ticks = dict(ticks)
        self._history.append((self.last_t, dict(ticks)))
        while self._history and self._history[0][0] < self.last_t - self.still_s:
            self._history.popleft()
        return self._evaluate()

    def _evaluate(self):
        ok, reasons = self.model.check(self.ticks)
        self.reasons = reasons
        self.pose = self.model.camera_link_pose(self.ticks) if ok else None
        return ok, self.pose

    def moving(self):
        if len(self._history) < 2:
            return False
        for mid in self.model.motor_ids:
            values = [h[1][mid] for h in self._history if mid in h[1]]
            if values and max(values) - min(values) > self.still_ticks:
                return True
        return False

    def at_reference(self, tolerance=3.0):
        if not self.ticks:
            return False
        return all(abs(self.ticks.get(mid, math.inf) - ref) <= tolerance
                   for mid, ref in self.model.reference_ticks.items())

    def status(self, now):
        """給 /head_tf/status 的 dict。"""
        age = None if self.last_t is None else float(now) - self.last_t
        reasons = list(self.reasons)
        if age is not None and age > self.stale_s:
            # 秒數放 age_s，原因文字固定（節點用原因有沒有變來決定要不要寫 log）
            reasons = [f'超過 {self.stale_s:g} 秒沒收到 28/29/30 號的刻度（driver_node 有在跑嗎？頭部馬達有上電嗎？）']
        valid = not reasons and self.pose is not None
        out = {
            'valid': valid,
            'reasons': reasons,
            'ticks': {str(k): round(v, 1) for k, v in (self.ticks or {}).items()},
            'moving': self.moving() if valid else False,
            'at_reference': self.at_reference(),
            'age_s': None if age is None else round(age, 3),
            # 給監控網頁畫刻度條用（VisionInterface.html），網頁就不用再寫一份 neck_head.yaml 的數字
            'reference_ticks': {str(k): v for k, v in self.model.reference_ticks.items()},
            'allowed_ticks': {str(k): list(v) for k, v in self.model.allowed_ticks.items()},
            'margin_ticks': self.model.margin,
        }
        if valid:
            R, t = self.pose
            r, p, y = (math.degrees(v) for v in matrix_rpy(R))
            out['camera'] = {'x': round(float(t[0]), 4), 'y': round(float(t[1]), 4), 'z': round(float(t[2]), 4),
                             'roll_deg': round(r, 2), 'pitch_deg': round(p, 2), 'yaw_deg': round(y, 2)}
        return out
