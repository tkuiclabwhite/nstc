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

from sp3_task_manager import grasp_plan as gp
from sp3_task_manager.errors import (BOTTLE_NOT_FOUND, DRIVER_ERROR, GRASP_FAILED,
                                     HAND_STATE_LOST, JOINT_STATE_LOST, OUT_OF_RANGE, TaskError)

# moveit_bridge 結果的 reason → 錯誤碼（S3-05 §9）。到不了、超出關節範圍 = MAN-E-002（不重試、不換手）；
# 其他（會撞到、規劃或執行失敗…）= MAN-E-004（可以換手、重試）
ARM_REASON_CODE = {
    'unreachable': OUT_OF_RANGE,
    'joint_limit': OUT_OF_RANGE,
    'bad_request': OUT_OF_RANGE,
}


# ======================================================================
# 藥瓶辨識：bottle_localization 的 /bottle_detections_3d
# 信心值門檻、多幀平滑都在 bottle_localization（params.yaml 的 score_threshold、smoothing_frames），
# 這裡直接用它發出來的值。
# ======================================================================
class VisionClient:
    def __init__(self, node, cfg, cb_group):
        from vision_msgs.msg import Detection3DArray
        self.timeout = cfg['timeouts']['detect_s']
        self._last = None          # (收到時間, {class_id: (xyz, size)})
        node.create_subscription(Detection3DArray, '/bottle_detections_3d',
                                 self._cb, 10, callback_group=cb_group)

    def _cb(self, msg):
        dets = {}
        for d in msg.detections:
            if d.results:
                p, s = d.bbox.center.position, d.bbox.size
                dets[d.results[0].hypothesis.class_id] = ((p.x, p.y, p.z), (s.x, s.y, s.z))
        self._last = (time.monotonic(), dets)

    def locate(self, task, bottle_class):
        """等一筆開始找之後才收到、有這個類別的偵測結果。
        回傳 (中心 m, 尺寸 m, 其他藥瓶 [(類別, 中心 m, 尺寸 m)])，其他藥瓶給 MoveIt 當障礙物。"""
        t0 = time.monotonic()

        def found():
            if self._last is None or self._last[0] < t0 or bottle_class not in self._last[1]:
                return None
            return self._last[1]

        dets = task.wait_until(found, self.timeout, BOTTLE_NOT_FOUND,
                               f'{self.timeout:g} 秒內沒看到 {bottle_class}'
                               '（bottle_localization 有開嗎？）')
        xyz, size = dets[bottle_class]
        others = [(c, np.array(p), np.array(s)) for c, (p, s) in sorted(dets.items()) if c != bottle_class]
        return np.array(xyz), np.array(size), others


class DryVisionClient:
    """設定檔 dry_run.bottles 的假藥瓶：要抓的是目標，其他的當障礙物（同 VisionClient 的回傳格式）。"""

    def __init__(self, node, cfg, cb_group):
        self.cfg = cfg['dry_run']
        self.dets = gp.dry_bottles(cfg)

    def locate(self, task, bottle_class):
        task.sleep(self.cfg['step_s'])
        if self.cfg['fail'] == 'detect' or bottle_class not in self.dets:
            raise TaskError(BOTTLE_NOT_FOUND, f'（dry_run）看不到 {bottle_class}')
        xyz, size = self.dets[bottle_class]
        others = [(c, p, s) for c, (p, s) in sorted(self.dets.items()) if c != bottle_class]
        return xyz, size, others


# ======================================================================
# 手臂：armpackage 的 moveit_bridge（/arm_move JSON API）+ real_arm_bridge（/arm_real_status）
# ======================================================================
class ArmClient:
    """MoveIt 回報到了不代表實機到了：實機跟隨沒開、或中途被 real_arm_bridge 停掉時，模擬照樣會動完。
    所以 real_arm.required = true 時，每一步移動前、移動中都要實機跟隨是 follow，
    MoveIt 到了之後再等 real_arm_bridge 回報實機停穩（settled），不是固定等幾秒。"""

    def __init__(self, node, cfg, cb_group):
        self.node, self.cfg = node, cfg['grasp']
        self.timeouts = cfg['timeouts']
        self.require_real = bool((cfg.get('real_arm') or {}).get('required', True))
        self._real = None          # (收到時間, dict)：real_arm_bridge 最後一筆 /arm_real_status
        node.create_subscription(String, '/arm_real_status', self._real_cb, 10,
                                 callback_group=cb_group)
        self.pub_move = node.create_publisher(String, '/arm_move', 10)
        self.pub_stand = node.create_publisher(Bool, '/arm_stand', 10)
        self.pub_speed = node.create_publisher(Float64, '/arm_speed_cmd', 10)
        self.pub_real_enable = node.create_publisher(Bool, '/arm_real_enable', 10)
        self.pub_obstacles = node.create_publisher(String, '/arm_ik_obstacles', 10)
        self.pub_attach = node.create_publisher(String, '/arm_attach', 10)
        self.pub_stop = node.create_publisher(Bool, '/arm_stop', 10)
        self.held = None           # 掛在手上的物體 (手, id)，失敗時要拿掉
        self.scene_wait = cfg['obstacles']['scene_wait_s']
        self.scene_confirm = cfg['obstacles']['scene_confirm_s']
        from moveit_msgs.srv import GetPlanningScene
        self._scene_cli = node.create_client(GetPlanningScene, '/get_planning_scene', callback_group=cb_group)
        # 最後一次 set_obstacles 的物體，每秒重發一次：/arm_ik_obstacles 不是 latched，網頁（RobotViewer）晚開也看得到。
        # 內容一樣時 moveit_bridge 不會重送 MoveIt
        self._objs = None
        node.create_timer(1.0, self._republish, callback_group=cb_group)
        self._result = None        # (收到時間, dict)：最後一筆指令結果（cmd 不是 None）
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
        elif d.get('cmd'):         # cmd = None 是狀態通知（例如停住後的目前姿勢），不是指令的結果
            self._result = (time.monotonic(), d)

    def _joint_cb(self, msg):
        # MoveIt 的假硬體也發 /joint_states（名稱是關節名）；driver 的名稱是馬達 ID 數字
        if msg.name and msg.name[0].isdigit():
            self._joint_t = time.monotonic()

    def _real_cb(self, msg):
        try:
            self._real = (time.monotonic(), json.loads(msg.data))
        except ValueError:
            pass

    def check_joint_states(self):
        stale = self.timeouts['joint_states_stale_s']
        if self._joint_t is None or time.monotonic() - self._joint_t > stale:
            raise TaskError(JOINT_STATE_LOST,
                            f'driver 的 /joint_states 超過 {stale:g} 秒沒更新（馬達 driver 有開嗎？）')

    def check_real(self, what=''):
        """實機跟隨要是 follow。real_arm.required = false（只測 MoveIt）時不檢查。"""
        if not self.require_real:
            return
        head = f'{what}：' if what else ''
        stale = self.timeouts['real_status_stale_s']
        if self._real is None or time.monotonic() - self._real[0] > stale:
            raise TaskError(DRIVER_ERROR,
                            f'{head}/arm_real_status 超過 {stale:g} 秒沒更新'
                            '（real_arm_bridge 有開嗎？nstc.launch.py 要 arm:=true）')
        d = self._real[1]
        if d.get('state') != 'follow':
            raise TaskError(DRIVER_ERROR,
                            f'{head}實機跟隨不是開著的（{d.get("state")}）：{d.get("msg", "")}'
                            '｜網頁 IK 面板按「實機跟隨」，或送 /arm_real_enable true')

    def set_speed(self):
        self.pub_speed.publish(Float64(data=float(self.cfg['arm_speed'])))

    def _scene_ids(self, task):
        """MoveIt 場景裡現在有哪些物體（id 集合）；問不到回 None。"""
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene
        if not self._scene_cli.service_is_ready():
            return None
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        fut = self._scene_cli.call_async(req)
        end = time.monotonic() + 2.0
        while not fut.done() and time.monotonic() < end:
            task.sleep(0.02)
        if not fut.done() or fut.result() is None:
            return None
        return {o.id for o in fut.result().scene.world.collision_objects}

    def set_obstacles(self, task, objs):
        """整份取代任務管理（source sp3）放進 MoveIt 場景的物體（base_link，m）。網頁的物體是另一個來源（web），不會被蓋掉。
        發完去問 MoveIt 確認真的套用了（沒有就重發）；確認不了就不動手臂。"""
        end = time.monotonic() + self.scene_confirm
        missing = extra = None
        self._objs = objs
        while True:
            self.pub_obstacles.publish(String(data=json.dumps({'source': 'sp3', 'objects': objs})))
            task.sleep(self.scene_wait)
            have = self._scene_ids(task)
            if have is not None:
                missing, extra = gp.scene_missing('sp3', objs, have)
                if not missing and not extra:
                    return
            if time.monotonic() > end:
                why = ('問不到 MoveIt 的場景（/get_planning_scene，MoveIt 有開嗎？）' if have is None else
                       f'MoveIt 場景少了 {sorted(missing)}、多了 {sorted(extra)}')
                raise TaskError(GRASP_FAILED, f'障礙物沒有套用到 MoveIt，不動手臂：{why}')

    def _republish(self):
        if self._objs is not None:
            self.pub_obstacles.publish(String(data=json.dumps({'source': 'sp3', 'objects': self._objs})))

    def attach(self, task, arm, obj):
        """夾起來的藥瓶掛到手上（obj = /arm_ik_obstacles 的 stick 格式），之後規劃會算它、跟著手移動。"""
        d = dict(obj, action='attach', arm=arm)
        self._send_and_wait(task, lambda: self.pub_attach.publish(String(data=json.dumps(d))),
                            '藥瓶掛到手上', cmd='attach')
        self.held = (arm, obj['id'])

    def detach(self, task):
        """手上的藥瓶從場景拿掉（放下後、或重試前）。"""
        if self.held is None:
            return
        arm, oid = self.held
        d = {'action': 'detach', 'arm': arm, 'id': oid}
        self._send_and_wait(task, lambda: self.pub_attach.publish(String(data=json.dumps(d))),
                            '藥瓶從手上拿掉', cmd='detach')
        self.held = None

    def drop_held(self):
        """任務失敗或取消時：不等回報，直接送 detach。"""
        if self.held is not None:
            arm, oid = self.held
            self.pub_attach.publish(String(data=json.dumps({'action': 'detach', 'arm': arm, 'id': oid})))
            self.held = None

    def stop(self, why):
        """取消 MoveIt 執行中的軌跡（/arm_stop），並關掉實機跟隨：real_arm_bridge 不再送新指令，
        馬達停在最後送出的目標。之後要再跑任務，得重新打開實機跟隨（網頁 IK 面板或 /arm_real_enable true）。"""
        self.pub_stop.publish(Bool(data=True))
        self.pub_real_enable.publish(Bool(data=False))
        self.node.get_logger().warning(f'手臂停止（取消軌跡、關閉實機跟隨）：{why}')

    def _send_and_wait(self, task, publish, what, cmd):
        """送出指令、等 moveit_bridge 回報這個 cmd 的結果，回傳結果 dict。
        移動中被中斷（急停、取消、逾時、狀態中斷）就讓實機停下，不然它會繼續走到目標。"""
        self._moving = False
        try:
            return self._move_and_wait(task, publish, what, cmd)
        except Exception as e:
            if self._moving:
                self.stop(f'{what}中斷：{e!r}')
            raise
        finally:
            self._moving = False

    def _move_and_wait(self, task, publish, what, cmd):
        self.check_joint_states()
        self.check_real(what)
        t0 = time.monotonic()
        publish()
        self._moving = cmd in ('move', 'stand')   # 從這裡到實機停穩之間被打斷，才需要叫實機停

        def moveit_done():
            self.check_real(what)        # 移動中實機跟隨被停掉（卡住、超出限制）就不用再等 MoveIt
            r = self._result
            return r[1] if r and r[0] > t0 and r[1].get('cmd') == cmd else None

        res = task.wait_until(
            moveit_done, self.timeouts['arm_move_s'], GRASP_FAILED,
            f'{what}：{self.timeouts["arm_move_s"]:g} 秒沒有回報（moveit_bridge 有開嗎？）')
        if not res.get('ok'):
            reason = res.get('reason')
            # 沒規劃出來 = 手臂沒動，不用停；執行到一半失敗或被取消才要停
            self._moving = reason in ('exec_failed', 'canceled')
            raise TaskError(ARM_REASON_CODE.get(reason, GRASP_FAILED),
                            f'{what}：[{reason}] {res.get("note", "")}')
        if cmd in ('move', 'stand'):
            self._wait_settled(task, what)
        self.check_joint_states()
        return res

    def _wait_settled(self, task, what):
        """MoveIt 到了之後等實機停穩。不接實機（real_arm.required = false）時固定等 arm_settle_s。"""
        if not self.require_real:
            task.sleep(self.cfg['arm_settle_s'])
            return
        t_ok = time.monotonic()
        limit = self.timeouts['real_settle_s']

        def settled():
            self.check_real(what)
            # 只認 MoveIt 回報之後才收到的狀態，免得拿到上一個目標的「停穩」
            return True if self._real[0] > t_ok and self._real[1].get('settled') else None

        task.wait_until(settled, limit, DRIVER_ERROR,
                        f'{what}：MoveIt 到了，實機 {limit:g} 秒還沒停穩'
                        '（/arm_real_status 沒有 settled 欄位的話，armpackage 要重新 build）')

    def move(self, task, pos_cm, approach, arm, what):
        """arm = right / left / auto（auto 由 moveit_bridge 選）。回傳實際用的那隻手。"""
        d = {'pos': [round(float(v), 1) for v in pos_cm], 'dir': approach,
             'fingers': self.cfg['fingers'], 'arm': arm}
        res = self._send_and_wait(
            task, lambda: self.pub_move.publish(String(data=json.dumps(d))), what, cmd='move')
        return res.get('arm') or arm

    def stand(self, task):
        self._send_and_wait(task, lambda: self.pub_stand.publish(Bool(data=True)), '回站姿', cmd='stand')


class DryArmClient:
    def __init__(self, node, cfg, cb_group):
        self.cfg = cfg['dry_run']
        self.stage_text = ''

    def set_speed(self):
        pass

    def set_obstacles(self, task, objs):
        pass

    def stop(self, why):
        pass

    def move(self, task, pos_cm, approach, arm, what):
        task.sleep(self.cfg['step_s'])
        if self.cfg['fail'] == 'arm':
            raise TaskError(OUT_OF_RANGE, f'（dry_run）{what}：[unreachable] 到不了')
        if arm == 'auto':      # 模擬 moveit_bridge 的 auto：藥瓶在右邊（右 ≥ 0）用右手
            arm = 'right' if pos_cm[1] >= 0 else 'left'
        return arm

    def attach(self, task, arm, obj):
        task.sleep(self.cfg['step_s'])

    def detach(self, task):
        pass

    def drop_held(self):
        pass

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
