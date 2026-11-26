"""任務管理對其他模組的轉接層。

測試平台現在用的介面全部包在這裡（藥瓶辨識、arm_ws 的 moveit_bridge、handpackage）。
之後換成正式硬體、MoveIt action 或子一的 /hand/* 時，只要換這個檔，task_manager 的流程不用動。

每個 client 都有真實版與 dry_run 版，介面相同。等待一律透過 task.wait_until / task.sleep，
這兩個會順便檢查取消、急停與任務逾時，所以任何一步都能被中斷。
"""
import json
import os
import time

import numpy as np
from std_msgs.msg import Bool, Float64, String
from rclpy.qos import DurabilityPolicy, QoSProfile
# /arm_ik_obstacles、/manip/target 用 TRANSIENT_LOCAL 發：rosbridge 訂閱時，若現有的發布者全是 TRANSIENT_LOCAL
# （網頁經 rosbridge 發的 /arm_ik_obstacles 就是）會用 TRANSIENT_LOCAL 訂，VOLATILE 的發布者就收不到（QoS 不相容）。
# TRANSIENT_LOCAL 發布者兩種訂閱都相容，晚開的網頁也會先收到最後一筆
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
START_TOUCH_FRAC = 0.2   # 錄好的路線：起點就碰到的物體，只在路程前這麼多內放行（之後還碰到 = 一路貼著，不走）
START_WINDOW_FRAC = 0.05  # 「起點就碰到」算到路程前這麼多：站姿左手拇指離加厚的桌子只有 0.6，差零點幾度就變成第 2 點才碰到
                          # （2026-11-19 22:27：第 2/85 點 lh_thumb_distal↔桌子，起點沒碰到 → 整條不走）

from sp3_task_manager import grasp_plan as gp
from sp3_task_manager import ready_path as rp
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
    IN_FLIGHT = 8              # 路線碰撞檢查同時最多送幾個請求（服務佇列 10 筆）

    def __init__(self, node, cfg, cb_group):
        self.node, self.cfg = node, cfg['grasp']
        self.full_cfg = cfg          # auto 一隻一隻試時要看可及範圍表
        self.speed = float(self.cfg['arm_speed'])     # 網頁 /manip/speed 可以改，重開任務管理回到設定檔的值
        self.timeouts = cfg['timeouts']
        self.require_real = bool((cfg.get('real_arm') or {}).get('required', True))
        self._real = None          # (收到時間, dict)：real_arm_bridge 最後一筆 /arm_real_status
        node.create_subscription(String, '/arm_real_status', self._real_cb, 10,
                                 callback_group=cb_group)
        self.pub_move = node.create_publisher(String, '/arm_move', 10)
        self.pub_stand = node.create_publisher(Bool, '/arm_stand', 10)
        # 預備姿勢（ready pose，2026-11-19 決定）：設定檔 ready_pose。
        # via = path：照錄好的路線，兩隻手同時走（ready_path.py；MoveIt /execute_trajectory，不經 moveit_bridge），
        #             回站姿倒過來走；還沒錄、位置改了、會撞到時改用 move
        # via = move：用 /arm_move 一隻一隻移到設定檔的兩點
        # via = arm_ready：交給手臂端 /arm_ready（還沒做；格式先定 std_msgs/String {"arm": "both" / "left" / "right"}，
        #                  結果同 /arm_ik_status、cmd = ready。沒有人訂閱 = 手臂端還沒做）
        self.ready_via, self.ready_approach, self.ready_pos = gp.ready_targets(cfg)
        self.ready_cfg = cfg.get('ready_pose') or {}
        self.ready_file = os.path.expanduser(self.ready_cfg.get('path_file', ''))
        self.ready_speed = float(self.ready_cfg.get('speed', self.cfg['arm_speed']))
        self.ready_accel = self.ready_cfg.get('accel')   # rad/s²，None = 跟著速度縮（見 ready_path.time_path）
        rp.mirror_src(self.ready_cfg)              # ready_pose.mirror 設錯就不開（同 ready_targets）
        self.pub_ready = node.create_publisher(String, '/arm_ready', 10)
        from moveit_msgs.action import ExecuteTrajectory
        from moveit_msgs.srv import GetStateValidity
        from rclpy.action import ActionClient
        self._exec_ac = ActionClient(node, ExecuteTrajectory, '/execute_trajectory', callback_group=cb_group)
        self._valid_cli = node.create_client(GetStateValidity, '/check_state_validity', callback_group=cb_group)
        self._exec_gh = None       # 照路線走時 MoveIt 執行中的 goal（急停要取消）
        self._q_sim = {}           # MoveIt（假硬體）的關節角 {關節名: rad}
        self._rec = None           # 錄路線時：[{手: 7 軸}]
        self.frames = cfg['frames']
        self._fk_shape = None      # URDF FK（ready_pose.adjust_cm 改路線用，第一次用到才讀）
        self._adj_cache = None     # (檔案時間 + adjust_cm, 改好的路線)
        self.last_goal = None      # 最後送出的掌心目標（量測座標 cm）、用哪隻手：給網頁顯示
        self.pub_speed = node.create_publisher(Float64, '/arm_speed_cmd', 10)
        self.pub_real_enable = node.create_publisher(Bool, '/arm_real_enable', 10)
        self.pub_obstacles = node.create_publisher(String, '/arm_ik_obstacles', LATCHED_QOS)
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
        if not msg.name:
            return
        if msg.name[0].isdigit():
            self._joint_t = time.monotonic()
            return
        self._q_sim.update(zip(msg.name, msg.position))
        if self._rec is not None:
            q = self.q_now()
            if q is not None:
                self._rec.append(q)

    def q_now(self):
        """MoveIt 目前兩隻手的關節角 {手: 7 軸 rad}；還沒收到回 None。"""
        try:
            return {a: np.array([self._q_sim[n] for n in rp.joint_names(a)]) for a in rp.ARMS}
        except KeyError:
            return None

    def sim_z_comp(self, pos_cm):
        """實機跟隨沒開時，目標要自己往上補多少 cm（moveit_bridge 只在實機跟隨開著時補，公式見 rp.real_z_comp_cm）：
        模擬才會停在跟開實機跟隨時一樣的地方（可及範圍一樣、放置時藥瓶底不會插進桌子的加厚）。move 每一步都會加。"""
        if self.real_following():
            return 0.0
        return rp.real_z_comp_cm(self.ready_cfg, pos_cm[0])

    def real_following(self):
        return self._real is not None and time.monotonic() - self._real[0] < self.timeouts['real_status_stale_s'] \
            and self._real[1].get('state') == 'follow'

    def _real_cb(self, msg):
        try:
            self._real = (time.monotonic(), json.loads(msg.data))
        except ValueError:
            pass

    def check_joint_states(self):
        if not self.require_real:      # 只測 MoveIt：馬達可以不上電（driver 沒找到馬達時名稱是空的）
            return
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

    def set_speed(self, speed=None):
        if speed is not None:
            self.speed = float(speed)
        self.pub_speed.publish(Float64(data=self.speed))

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
        gh = self._exec_gh
        if gh is not None:         # 照錄好的路線走（任務管理自己送給 MoveIt 的軌跡）
            gh.cancel_goal_async()
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
        self._moving = cmd in ('move', 'stand', 'ready')   # 從這裡到實機停穩之間被打斷，才需要叫實機停

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
        if cmd in ('move', 'stand', 'ready'):
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
        """arm = right / left / auto（auto 由 moveit_bridge 選）。回傳實際用的那隻手。
        grasp.arm_z_offset_cm：某隻手的目標高度另外補（左手實機比右手低約 1 cm，機構，2026-11-19）。
        兩隻手補的不一樣時 auto 不能交給 moveit_bridge 選（它不知道要補），改成照 arm_order 一隻一隻試。"""
        off = self.cfg.get('arm_z_offset_cm') or {}
        if arm == 'auto' and float(off.get('left', 0.0)) != float(off.get('right', 0.0)):
            # 只試可及範圍表裡搆得到的手（都搆不到就兩隻都試，讓 moveit_bridge 說為什麼）；都失敗時每隻手的原因都要講，
            # 不然只看到最後一隻（2026-11-19：右 30 前 32 回報「左手離肩膀太遠」，右手為什麼失敗看不到）
            order = gp.arm_order('auto', pos_cm)
            order = [a for a in order if a in gp.reachable_arms(pos_cm, self.full_cfg)] or order
            errs = []
            for a in order:
                try:
                    return self.move(task, pos_cm, approach, a, what)
                except TaskError as e:
                    if e.code not in (GRASP_FAILED, OUT_OF_RANGE):
                        raise
                    errs.append((a, e))
            if len(errs) == 1:
                raise errs[0][1]
            raise TaskError(errs[0][1].code, '｜'.join(f'{"左" if a == "left" else "右"}手：{e.detail}' for a, e in errs))
        self.last_goal = (np.array(pos_cm, dtype=float), arm, what)     # 網頁顯示要的位置（沒補）
        # 只看模擬（實機跟隨沒開）時自己補 0.17 × 前：可及範圍是開實機跟隨量的（moveit_bridge 有補），
        # 不補的話模擬的目標比較低、離肩膀比較遠，邊緣的點到不了（2026-11-19 右 30 前 32：離肩膀 56 > 55）
        target = np.array(pos_cm, dtype=float) + [0.0, 0.0, float(off.get(arm, 0.0)) + self.sim_z_comp(pos_cm)]
        d = {'pos': [round(float(v), 1) for v in target], 'dir': approach,
             'fingers': self.cfg['fingers'], 'arm': arm}
        res = self._send_and_wait(
            task, lambda: self.pub_move.publish(String(data=json.dumps(d))), what, cmd='move')
        return res.get('arm') or arm

    def stand(self, task):
        """回站姿。via = path：照錄好的路線倒過來走（兩隻手一起）；不能用就交給 moveit_bridge（/arm_stand）。回傳說明。"""
        if self.ready_via == 'path':
            why = self._path_go(task, 'stand', rp.ARMS)
            if why is None:
                return '照錄好的路線倒過來走'
            self.node.get_logger().warning(f'回站姿不照錄好的路線（{why}），交給 moveit_bridge')
        self._send_and_wait(task, lambda: self.pub_stand.publish(Bool(data=True)), '回站姿', cmd='stand')
        return 'moveit_bridge 回站姿' + (f'（錄好的路線不能用：{why}）' if self.ready_via == 'path' else '')

    def ready(self, task, arm='both'):
        """回預備姿勢。arm = both / left / right。回傳說明（用了哪種方法）。
        via = path：照錄好的路線兩隻手一起走；不能用（還沒錄、位置改了、會撞到）就改用 move。
        via = move：一隻一隻把掌心移到設定檔 ready_pose 的點（已經在那裡的手，moveit_bridge 當微調，很快就回到了）"""
        arms = rp.ARMS if arm == 'both' else (arm,)
        why = None
        if self.ready_via == 'path':
            why = self._path_go(task, 'ready', arms)
            if why is None:
                return '照錄好的路線走（兩隻手一起）'
            self.node.get_logger().warning(f'回預備姿勢不照錄好的路線（{why}），改成一隻一隻移')
        if self.ready_via in ('path', 'move'):
            for a in arms:
                self.move(task, self.ready_pos[a], self.ready_approach, a,
                          f'回預備姿勢（{"左" if a == "left" else "右"}手）')
            return '一隻一隻用 /arm_move 移' + (f'（錄好的路線不能用：{why}）' if why else '')
        if self.pub_ready.get_subscription_count() == 0:
            raise TaskError(DRIVER_ERROR, '手臂端還沒有 /arm_ready（預備姿勢），等手臂負責人做好'
                                          '（或把設定檔 ready_pose.via 改回 move）')
        self._send_and_wait(task, lambda: self.pub_ready.publish(String(data=json.dumps({'arm': arm}))),
                            '回預備姿勢', cmd='ready')
        return '手臂端 /arm_ready'

    # ---------------- 錄好的 ready 路線（ready_path.py）----------------
    def record_ready(self, task):
        """錄站姿 → ready 的路線：兩隻手要在站姿，用 /arm_move 一隻一隻舉到 ready（moveit_bridge 規劃、檢查過碰撞），
        途中記 MoveIt 的關節角，存到 ready_pose.path_file（蓋掉舊的）。場景（桌子）呼叫前要先放好。回傳說明。"""
        q0 = self.q_now()
        if q0 is None:
            raise TaskError(JOINT_STATE_LOST, '收不到 MoveIt 的 /joint_states（關節名），沒辦法錄')
        away = [a for a in rp.ARMS if not rp.at(q0[a], np.zeros(7))]
        if away:
            raise TaskError(OUT_OF_RANGE, f'{"、".join("左" if a == "left" else "右" for a in away)}手不在站姿，'
                                          '先按 Stand 回站姿再錄')
        follow = self.real_following()
        # 實機跟隨開著時 moveit_bridge 自己會往上補；沒開就自己補，錄到的關節角才會跟實機要的一樣
        comp = {a: 0.0 if follow else rp.real_z_comp_cm(self.ready_cfg, self.ready_pos[a][0]) for a in rp.ARMS}
        src = rp.mirror_src(self.ready_cfg)        # 兩隻手共用一隻手的路線：只舉那隻手
        self._rec = [q0]
        try:
            for a in ((src,) if src else rp.ARMS):
                # 只看模擬時 move 自己會補 comp（同上面算的）；左手另外 +arm_z_offset_cm
                self.move(task, self.ready_pos[a], self.ready_approach, a,
                          f'錄 ready 路線（{"左" if a == "left" else "右"}手）')
            q1 = self.q_now()
        finally:
            rec, self._rec = self._rec, None
        rec.append(q1)
        paths = {a: rp.resample(np.array([q[a] for q in rec])) for a in rp.ARMS}
        if src:
            paths[rp.other(src)] = paths[src].copy()
        rp.save(self.ready_file, paths, self.ready_cfg, follow, comp)
        note = ('實機跟隨開著（moveit_bridge 有補高度）' if follow else
                f'只有模擬：目標自己往上補了 {comp["left"]:.1f} cm（實機會低這麼多），所以模擬的 ready 會比設定的高')
        if src:
            note += f'。只錄了{"左" if src == "left" else "右"}手，另一隻手用同一條（鏡像），還在站姿，按 Ready 會一起走'
        return f'錄好了：{rp.describe(paths, self.ready_speed, self.ready_accel)}。{note}。存在 {self.ready_file}'

    def _path_go(self, task, kind, arms):
        """照錄好的路線走（kind = ready / stand）。走完（或本來就在）回 None；沒動（不能用）回原因；走到一半出錯丟 TaskError。"""
        rec, why = self._ready_rec()
        if why:
            return why
        q = self.q_now()
        if q is None:
            return '收不到 MoveIt 的 /joint_states（關節名）'
        paths = rp.plan(rec, q, arms, kind)
        if not paths:
            return None
        names, P0 = rp.merge(paths)
        P = rp.prepare(P0)          # 磨圓（2026-11-26：折線每個轉角都減速，實機走起來一頓一頓的）
        hit = self._path_hits(task, names, P)
        if hit:
            # 磨圓會切轉角、離東西近一點：會撞到就走原本的折線（轉角處會慢下來，跟以前一樣）
            self.node.get_logger().info(f'磨圓的{hit}，改走原本的路線')
            P = rp.resample(P0, rp.CHECK_STEP)
            hit = self._path_hits(task, names, P)
            if hit:
                return hit
        self._exec_path(task, names, P, '回站姿（照錄好的路線）' if kind == 'stand' else '回預備姿勢（照錄好的路線）')
        return None

    def _ready_rec(self):
        """讀錄好的路線，套上 ready_pose.adjust_cm（只改某隻手，逐點逆解；同一個檔案、同一組設定只算一次）。
        回傳 (rec, None) 或 (None, 不能用的原因)。"""
        rec, why = rp.load(self.ready_file, self.ready_cfg)
        if why or not self.ready_cfg.get('adjust_cm'):
            return rec, why
        key = (os.path.getmtime(self.ready_file), json.dumps(self.ready_cfg['adjust_cm'], sort_keys=True))
        if self._adj_cache is not None and self._adj_cache[0] == key:
            return self._adj_cache[1], None
        try:
            if self._fk_shape is None:
                from ament_index_python.packages import get_package_share_directory
                from sp3_task_manager.arm_shape import ArmShape
                urdf = os.path.join(get_package_share_directory('seven_axis_arm_urdf'), 'urdf', 'seven_axis_arm_urdf.urdf')
                self._fk_shape = ArmShape(urdf, get_package_share_directory, meshes=False)
            rec = rp.apply_adjust(rec, self.ready_cfg, self._fk_shape, self.frames)
        except Exception as e:     # noqa: BLE001  讀不到 URDF、逆解不到：不照路線走（改用 move），說原因
            return None, f'ready_pose.adjust_cm 套不上：{e}'
        self._adj_cache = (key, rec)
        return rec, None

    def _path_hits(self, task, names, P):
        """路線上每一點問 MoveIt 會不會撞到（場景物體、支架、兩隻手互撞）。沒撞到回 None，撞到回說明。
        起點（路程前 START_WINDOW_FRAC）就碰到的物體（站姿時左手拇指在桌子的加厚裡，2026-11-19 .11 測到，錄好的路線因此每次都被拒）先不算，
        跟 moveit_bridge 規劃時一樣；那一對一分開就開始算，而且只在路程前 START_TOUCH_FRAC 內不算。"""
        from moveit_msgs.srv import GetStateValidity
        if not self._valid_cli.wait_for_service(timeout_sec=2.0):
            return '連不到 MoveIt 的 /check_state_validity'
        # 同時最多 IN_FLIGHT 個請求、回來一個補一個：一點一點等每點至少 20 ms（task.sleep 的間隔）；
        # 一次全送也不行，服務的佇列只有 10 筆（QoS depth 10），多的會被丟掉、永遠等不到（2026-11-19 本機測到）
        n = len(P)
        contacts = [None] * n      # 每一點撞到的 {(物體, 物體)}；None = 還沒回來
        pending, nxt = {}, 0
        end = time.monotonic() + 5.0 + 0.05 * n
        while nxt < n or pending:
            for i, f in list(pending.items()):
                if not f.done():
                    continue
                del pending[i]
                res = f.result()
                if res is None:
                    return f'路線第 {i}/{n - 1} 點 MoveIt 碰撞檢查沒有結果'
                contacts[i] = set() if res.valid else \
                    {tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts} or {('?', '?')}
            while nxt < n and len(pending) < self.IN_FLIGHT:
                req = GetStateValidity.Request()
                req.group_name = 'both_arms'
                req.robot_state.is_diff = True
                req.robot_state.joint_state.name = list(names)
                req.robot_state.joint_state.position = [float(v) for v in P[nxt]]
                pending[nxt] = self._valid_cli.call_async(req)
                nxt += 1
            if time.monotonic() > end:
                raise TaskError(DRIVER_ERROR, 'MoveIt 碰撞檢查（/check_state_validity）沒有回應')
            task.sleep(0.005)
        allowed = set(contacts[0])
        early = set().union(*contacts[:int(START_WINDOW_FRAC * (n - 1)) + 1])
        if early:
            self.node.get_logger().info(f'路線一開始就碰到 {"、".join("↔".join(c) for c in sorted(early))}，離開前先不算')
        for i in range(1, n):
            if i <= START_WINDOW_FRAC * (n - 1):
                allowed |= contacts[i]     # 一開始（前 START_WINDOW_FRAC）才碰到的也算「起點就碰到」
            still = allowed                # 從一開始一路碰到現在、還沒分開的
            if i > START_TOUCH_FRAC * (n - 1):
                allowed = set()
            bad = contacts[i] - allowed
            if bad:
                pairs = sorted('↔'.join(c) for c in bad)
                stuck = f'（一開始就碰到，路程前 {START_TOUCH_FRAC:.0%} 還沒離開）' if bad & still else ''
                return f'路線第 {i}/{n - 1} 點會撞到：{"、".join(pairs[:3])}{stuck}'
            allowed &= contacts[i]         # 分開了就不再放行
        return None

    def _exec_path(self, task, names, P, what):
        """關節角清單排好時間（ready_pose.speed），交給 MoveIt /execute_trajectory 執行（同 moveit_bridge 回站姿），
        等 MoveIt 走完、實機停穩。中途被打斷就取消軌跡、關實機跟隨（同 _send_and_wait）。"""
        from builtin_interfaces.msg import Duration
        from moveit_msgs.action import ExecuteTrajectory
        from moveit_msgs.msg import RobotTrajectory
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        t, V = rp.time_path(P, self.ready_speed, accel=self.ready_accel, **rp.READY_TIMING)
        jt = JointTrajectory(joint_names=list(names))
        for q, v, ti in zip(P, V, t):
            ns = int(round(float(ti) * 1e9))
            jt.points.append(JointTrajectoryPoint(
                positions=[float(x) for x in q], velocities=[float(x) for x in v],
                time_from_start=Duration(sec=ns // 1000000000, nanosec=ns % 1000000000)))
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = RobotTrajectory(joint_trajectory=jt)
        self.check_joint_states()
        self.check_real(what)
        if not self._exec_ac.wait_for_server(timeout_sec=2.0):
            raise TaskError(DRIVER_ERROR, f'{what}：連不到 MoveIt 的 /execute_trajectory（MoveIt 有開嗎？）')
        self._moving = False
        try:
            # 實機馬達速度跟軌跡一樣（real_arm_bridge 用 /arm_speed_cmd 當 Profile Velocity），走完 finally 切回 grasp.arm_speed
            self.pub_speed.publish(Float64(data=self.ready_speed))
            task.sleep(0.2)
            fut = self._exec_ac.send_goal_async(goal)
            gh = task.wait_until(lambda: fut.result() if fut.done() else None, 5.0, DRIVER_ERROR,
                                 f'{what}：MoveIt 5 秒沒有回應')
            if not gh.accepted:
                raise TaskError(DRIVER_ERROR, f'{what}：MoveIt 不接受這條軌跡')
            self._exec_gh, self._moving = gh, True
            rf = gh.get_result_async()

            def done():
                self.check_real(what)
                return rf.result() if rf.done() else None

            limit = float(t[-1]) + 15.0
            res = task.wait_until(done, limit, GRASP_FAILED, f'{what}：{limit:.0f} 秒還沒走完')
            code = res.result.error_code.val
            if code != 1:          # moveit_msgs/MoveItErrorCodes：1 = SUCCESS
                raise TaskError(GRASP_FAILED, f'{what}：MoveIt 執行失敗（錯誤碼 {code}）')
            self._wait_settled(task, what)
            self.check_joint_states()
        except Exception as e:
            if self._moving:
                self.stop(f'{what}中斷：{e!r}')
            raise
        finally:
            self._moving = False
            self._exec_gh = None
            self.set_speed()


class DryArmClient:
    def __init__(self, node, cfg, cb_group):
        self.cfg = cfg['dry_run']
        self.stage_text = ''
        self.last_goal = None
        self.speed = float(cfg['grasp']['arm_speed'])

    def set_speed(self, speed=None):
        if speed is not None:
            self.speed = float(speed)

    def sim_z_comp(self, pos_cm):
        return 0.0

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
        self.last_goal = (np.array(pos_cm, dtype=float), arm, what)
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

    def ready(self, task, arm='both'):
        task.sleep(self.cfg['step_s'])
        return '（dry_run）回預備姿勢'

    def record_ready(self, task):
        task.sleep(self.cfg['step_s'])
        return '（dry_run）沒有手臂，沒錄'


# ======================================================================
# 手：handpackage（/hand/cmd、/hand/status JSON）
# ======================================================================
class HandClient:
    def __init__(self, node, cfg, cb_group):
        self.node, self.cfg = node, cfg['hand']
        self.stale = cfg['timeouts']['hand_status_stale_s']
        self.pub = node.create_publisher(String, '/hand/cmd', 10)
        self._status = None        # (收到時間, dict)
        self._base = {}            # 每隻手上次夾持的壓力零點（handpackage 開始閉合時的讀數）
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

    def pose(self, task, sides, which):
        """ready／站姿時把手擺到設定的手勢（which = 'open_pos' / 'stand_pos'）。
        離線的手跳過、不算失敗（例如 hands:=right 只開一隻手），回傳說明（全部擺好 = ''）。"""
        done, skipped = [], []
        for side in sides:
            try:
                self._check(side)
            except TaskError as e:
                skipped.append(f'{side}：{e.detail}')
                continue
            self._cmd({'cmd': 'manual', 'hand': side,
                       'speed': self.cfg['open_spd'], 'position': self.cfg[which]})
            done.append(side)
        if done:
            task.sleep(self.cfg['open_wait_s'])
        return f'手勢沒改（{"；".join(skipped)}）' if skipped else ''

    def grip(self, task, side):
        """夾到壓力門檻才算抓到；'閉合到目標位置' = 整隻手握到底都沒碰到東西。"""
        self._check(side)
        t0 = time.monotonic()
        self._cmd({'cmd': 'grip', 'action': 'start', 'hand': side,
                   'mode': self.cfg['grip_mode'], 'threshold': self.cfg['grip_threshold'],
                   'target': list(self.cfg.get('grip_target', [0] * 6)),
                   'timeout': self.cfg['grip_timeout_s']})

        def grip_state():
            if self._status is None or self._status[0] <= t0:
                return None
            return self._check(side).get('grip') or {}

        task.wait_until(lambda: (grip_state() or {}).get('active') or None, 2.0,
                        HAND_STATE_LOST, '夾持沒有啟動（手離線或沒有壓感？）')
        g = task.wait_until(lambda: (lambda s: s if s and not s.get('active') else None)(grip_state()),
                            self.cfg['grip_timeout_s'] + 2.0, GRASP_FAILED, '夾持沒有結束')
        reason = g.get('reason', '')
        self._base[side] = (g.get('base') or {}).get(side)
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
        base = self._base.get(side) or [0] * len(mass)   # 扣掉夾持開始時的讀數（實機右手中指沒碰東西就 50 以上）
        total = sum(m - b for m, b in zip(mass, base))
        if total < self.cfg['hold_min_mass']:
            raise TaskError(GRASP_FAILED, f'抬起後沒夾住（壓力總和 {total} < {self.cfg["hold_min_mass"]}，'
                                          f'讀數 {list(mass)}、零點 {list(base)}）')
        return total

    def stop(self):
        self._cmd({'cmd': 'stop'})


class DryHandClient:
    def __init__(self, node, cfg, cb_group):
        self.cfg = cfg['dry_run']

    def open(self, task, side):
        task.sleep(self.cfg['step_s'])

    def pose(self, task, sides, which):
        return ''

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
