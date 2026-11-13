"""任務管理對其他模組的轉接層。

測試平台現在用的介面全部包在這裡（藥瓶辨識、arm_ws 的 moveit_bridge、handpackage）。
之後換成正式硬體、MoveIt action 或子一的 /hand/* 時，只要換這個檔，task_manager 的流程不用動。

每個 client 都有真實版與 dry_run 版，介面相同。等待一律透過 task.wait_until / task.sleep，
這兩個會順便檢查取消、急停與任務逾時，所以任何一步都能被中斷。
"""
import json
import time

import numpy as np
from std_msgs.msg import Bool, Float64, String

from sp3_task_manager.errors import (BOTTLE_NOT_FOUND, GRASP_FAILED, HAND_STATE_LOST,
                                     JOINT_STATE_LOST, TaskError)


# ======================================================================
# 藥瓶辨識：bottle_localization 的 /bottle_detections_3d
# ======================================================================
class VisionClient:
    def __init__(self, node, cfg, cb_group):
        from vision_msgs.msg import Detection3DArray
        self.node, self.cfg = node, cfg['detection']
        self.timeout = cfg['timeouts']['detect_s']
        self._frames = []          # [(收到時間, frame_id, {class_id: (xyz, size, score)})]
        node.create_subscription(Detection3DArray, '/bottle_detections_3d',
                                 self._cb, 10, callback_group=cb_group)

    def _cb(self, msg):
        dets = {}
        for d in msg.detections:
            if not d.results:
                continue
            h = d.results[0]
            if h.hypothesis.score < self.cfg['min_score']:
                continue
            p, s = d.bbox.center.position, d.bbox.size
            dets[h.hypothesis.class_id] = ((p.x, p.y, p.z), (s.x, s.y, s.z), h.hypothesis.score)
        self._frames = (self._frames + [(time.monotonic(), msg.header.frame_id, dets)])[-30:]

    def locate(self, task, bottle_class):
        """等到連續 stable_frames 幀都看到這個類別，回傳平均的 (中心 m, 尺寸 m)。"""
        t0 = time.monotonic()
        need = self.cfg['stable_frames']

        def ready():
            new = [f for f in self._frames if f[0] >= t0]
            run = []
            for _, _, dets in new:
                run = run + [dets[bottle_class]] if bottle_class in dets else []
            return run if len(run) >= need else None

        run = task.wait_until(ready, self.timeout, BOTTLE_NOT_FOUND,
                              f'{self.timeout:g} 秒內沒有連續 {need} 幀看到 {bottle_class}'
                              '（bottle_localization 有開嗎？）')
        run = run[-need:]
        xyz = np.mean([r[0] for r in run], axis=0)
        size = np.mean([r[1] for r in run], axis=0)
        return xyz, size


class DryVisionClient:
    def __init__(self, node, cfg, cb_group):
        self.cfg = cfg['dry_run']

    def locate(self, task, bottle_class):
        task.sleep(self.cfg['step_s'])
        if self.cfg['fail'] == 'detect':
            raise TaskError(BOTTLE_NOT_FOUND, f'（dry_run）看不到 {bottle_class}')
        return np.array(self.cfg['bottle_base_link_m']), np.array(self.cfg['bottle_size_m'])


# ======================================================================
# 手臂：arm_ws seven_axis_arm_viewer 的 moveit_bridge（/arm_move JSON API）
# ======================================================================
class ArmClient:
    def __init__(self, node, cfg, cb_group):
        self.node, self.cfg = node, cfg['grasp']
        self.timeouts = cfg['timeouts']
        self.pub_move = node.create_publisher(String, '/arm_move', 10)
        self.pub_stand = node.create_publisher(Bool, '/arm_stand', 10)
        self.pub_speed = node.create_publisher(Float64, '/arm_speed_cmd', 10)
        self._result = None        # (收到時間, dict)：最後一筆不是 stage_only 的回報
        self.stage_text = ''
        self._joint_t = None       # driver 最後一次發 /joint_states 的時間
        node.create_subscription(String, '/arm_ik_status', self._status_cb, 10,
                                 callback_group=cb_group)
        from sensor_msgs.msg import JointState
        node.create_subscription(JointState, '/joint_states', self._joint_cb, 10,
                                 callback_group=cb_group)

    def _status_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if d.get('stage_only'):
            self.stage_text = d.get('stage', '')
        else:
            self._result = (time.monotonic(), d)

    def _joint_cb(self, msg):
        # MoveIt 的假硬體也發 /joint_states（名稱是關節名）；driver 的名稱是馬達 ID 數字
        if msg.name and msg.name[0].isdigit():
            self._joint_t = time.monotonic()

    def check_joint_states(self):
        stale = self.timeouts['joint_states_stale_s']
        if self._joint_t is None or time.monotonic() - self._joint_t > stale:
            raise TaskError(JOINT_STATE_LOST,
                            f'driver 的 /joint_states 超過 {stale:g} 秒沒更新（馬達 driver 有開嗎？）')

    def set_speed(self):
        self.pub_speed.publish(Float64(data=float(self.cfg['arm_speed'])))

    def _send_and_wait(self, task, publish, what):
        self.check_joint_states()
        t0 = time.monotonic()
        publish()
        res = task.wait_until(
            lambda: self._result[1] if self._result and self._result[0] > t0 else None,
            self.timeouts['arm_move_s'], GRASP_FAILED,
            f'{what}：{self.timeouts["arm_move_s"]:g} 秒沒有回報（moveit_bridge 有開嗎？）')
        if not res.get('ok'):
            raise TaskError(GRASP_FAILED, f'{what}：{res.get("note", "")}')
        task.sleep(self.cfg['arm_settle_s'])
        self.check_joint_states()
        return res.get('note', '')

    def move(self, task, pos_cm, approach, arm, what):
        d = {'pos': [round(float(v), 1) for v in pos_cm], 'dir': approach,
             'fingers': self.cfg['fingers'], 'arm': arm}
        return self._send_and_wait(
            task, lambda: self.pub_move.publish(String(data=json.dumps(d))), what)

    def stand(self, task):
        return self._send_and_wait(task, lambda: self.pub_stand.publish(Bool(data=True)), '回站姿')


class DryArmClient:
    def __init__(self, node, cfg, cb_group):
        self.cfg = cfg['dry_run']
        self.stage_text = ''

    def set_speed(self):
        pass

    def move(self, task, pos_cm, approach, arm, what):
        task.sleep(self.cfg['step_s'])
        if self.cfg['fail'] == 'arm':
            raise TaskError(GRASP_FAILED, f'（dry_run）{what}：到不了')
        return f'（dry_run）{arm} {approach} {[round(float(v), 1) for v in pos_cm]}'

    def stand(self, task):
        task.sleep(self.cfg['step_s'])
        return '（dry_run）回站姿'


# ======================================================================
# 手：handpackage（/hand/cmd、/hand/status JSON）
# ======================================================================
class HandClient:
    def __init__(self, node, cfg, cb_group):
        self.node, self.cfg = node, cfg['hand']
        self.stale = cfg['timeouts']['hand_status_stale_s']
        self.pub = node.create_publisher(String, '/hand/cmd', 10)
        self._status = None        # (收到時間, dict)
        node.create_subscription(String, '/hand/status', self._cb, 10, callback_group=cb_group)

    def _cb(self, msg):
        try:
            self._status = (time.monotonic(), json.loads(msg.data))
        except ValueError:
            pass

    def _cmd(self, d):
        self.pub.publish(String(data=json.dumps(d)))

    def _check(self, side):
        if self._status is None or time.monotonic() - self._status[0] > self.stale:
            raise TaskError(HAND_STATE_LOST,
                            f'/hand/status 超過 {self.stale:g} 秒沒更新（handpackage 有開嗎？）')
        h = (self._status[1].get('hands') or {}).get(side) or {}
        if not h.get('online'):
            raise TaskError(HAND_STATE_LOST, f'{side} 手離線')
        return self._status[1]

    def open(self, task, side):
        self._check(side)
        self._cmd({'cmd': 'manual', 'hand': side,
                   'speed': self.cfg['open_spd'], 'position': self.cfg['open_pos']})
        task.sleep(self.cfg['open_wait_s'])

    def grip(self, task, side):
        """夾到壓力門檻才算抓到；'閉合到目標位置' = 整隻手握到底都沒碰到東西。"""
        self._check(side)
        t0 = time.monotonic()
        self._cmd({'cmd': 'grip', 'action': 'start', 'hand': side,
                   'mode': self.cfg['grip_mode'], 'threshold': self.cfg['grip_threshold'],
                   'target': [0] * 6, 'timeout': self.cfg['grip_timeout_s']})

        def grip_state():
            if self._status is None or self._status[0] <= t0:
                return None
            return self._check(side).get('grip') or {}

        task.wait_until(lambda: (grip_state() or {}).get('active') or None, 2.0,
                        HAND_STATE_LOST, '夾持沒有啟動（手離線或沒有壓感？）')
        g = task.wait_until(lambda: (lambda s: s if s and not s.get('active') else None)(grip_state()),
                            self.cfg['grip_timeout_s'] + 2.0, GRASP_FAILED, '夾持沒有結束')
        reason = g.get('reason', '')
        if f'{side}: 壓力到門檻' not in reason:
            raise TaskError(GRASP_FAILED, f'沒夾到：{reason}')
        return reason

    def holding(self, task, side):
        """抬起後讀五指壓力，總和夠才算還夾著。"""
        self._cmd({'cmd': 'touch', 'on': True})
        try:
            task.sleep(0.5)                     # 等壓感訂閱的第一筆資料進來
            mass = (self._check(side)['hands'][side].get('mass')) or []
        finally:
            self._cmd({'cmd': 'touch', 'on': False})
        total = sum(mass)
        if total < self.cfg['hold_min_mass']:
            raise TaskError(GRASP_FAILED, f'抬起後沒夾住（壓力總和 {total} < {self.cfg["hold_min_mass"]}）')
        return total

    def stop(self):
        self._cmd({'cmd': 'stop'})


class DryHandClient:
    def __init__(self, node, cfg, cb_group):
        self.cfg = cfg['dry_run']

    def open(self, task, side):
        task.sleep(self.cfg['step_s'])

    def grip(self, task, side):
        task.sleep(self.cfg['step_s'])
        if self.cfg['fail'] == 'grip':
            raise TaskError(GRASP_FAILED, '（dry_run）沒夾到：閉合到目標位置')
        return f'（dry_run）{side}: 壓力到門檻'

    def holding(self, task, side):
        task.sleep(self.cfg['step_s'])
        if self.cfg['fail'] == 'hold':
            raise TaskError(GRASP_FAILED, '（dry_run）抬起後沒夾住')
        return 500

    def stop(self):
        pass
