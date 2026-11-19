"""子三任務管理：接主控的取放任務，串接藥瓶辨識、手臂、手。

對外（L2，給主控）
    /manip/pick_place  Action，型別用主控的 interfaces/action/ExecuteTaskPlan
                       goal 的 keys/values 欄位見 goal.py
    /manip/status      std_msgs/String（JSON），1 Hz：state IDLE / BUSY / ERROR / ESTOP
    /estop             std_msgs/Bool，True = 急停（測試平台用；正式型別待定）
給網頁（SP3TaskInterface）
    /manip/test_cmd    std_msgs/String（JSON）：測試區指令（回站姿、預備姿勢、移到某點、移到某瓶上方），格式見 try_cmd.py
    /manip/test_result std_msgs/String（JSON）：{"id", "cmd", "ok", "message"}
    /manip/pose        std_msgs/String（JSON），2 Hz：左右手掌心（模擬、實機）、要抓的藥瓶、最後的掌心目標，
                       各有 base_link（m）與桌面座標（cm）
    /manip/target      std_msgs/String（JSON）：要抓的藥瓶（RobotViewer 畫綠色）
    /manip/speed       std_msgs/Float64：手臂速度（Dynamixel 單位，1～50），之後的任務、測試都用這個；
                       馬上轉送 /arm_speed_cmd。/manip/status 的 speed 是現在的值

流程階段（S3-05 feedback.stage）：0 辨識、1 規劃、2 接近、3 抓取、4 驗證、5 放置、6 退回。
Feedback.status 格式：'<階段號> <階段名>｜retry=<n>｜<說明>'，progress = 完成的階段 / 7。
Result.error_code 是 errors.py 的序號（0 = 成功），message 開頭是完整錯誤碼。

參數
    config   設定檔路徑（預設 share/sp3_task_manager/config/task_manager.yaml）
    dry_run  要模擬哪幾段：true / all = 全部、false = 全部接實機、或 vision,arm,hand 任意組合
             （例如 vision = 假藥瓶、手臂和手真的動）。沒給（config）= 用設定檔的 dry_run.modules。
             設定檔的 dry_run.fail 可模擬模擬段的失敗
"""
import json
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from interfaces.action import ExecuteTaskPlan
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from sp3_task_manager.adapters import LATCHED_QOS
from sp3_task_manager import adapters, dry_run as dry_mod, goal as goal_mod, grasp_plan, try_cmd
from sp3_task_manager.arm_shape import ArmShape, real_ticks_converter
from sp3_task_manager.errors import (ESTOP, GRASP_FAILED, OK, OUT_OF_RANGE, TIMEOUT,
                                     TaskError, code_text)

SWITCH_ARM = (GRASP_FAILED, OUT_OF_RANGE)    # 這隻手到不了或規劃失敗：換另一隻手試

SPEED_MAX = 50.0   # 實機最多 50（Dynamixel 單位）
STAGES = ['辨識', '規劃', '接近', '抓取', '驗證', '放置', '退回']


class Canceled(Exception):
    pass


class TaskManager(Node):
    def __init__(self):
        super().__init__('sp3_task_manager')
        default_cfg = os.path.join(get_package_share_directory('sp3_task_manager'),
                                   'config', 'task_manager.yaml')
        self.declare_parameter('config', default_cfg)
        # ros2 run -p dry_run:=true 會被當成 bool，所以不限型別，一律轉字串解析
        self.declare_parameter('dry_run', 'config', ParameterDescriptor(dynamic_typing=True))
        with open(self.get_parameter('config').value, encoding='utf-8') as f:
            self.cfg = yaml.safe_load(f)
        dry = self.get_parameter('dry_run').value
        if dry in ('', 'config'):      # launch 沒給 → 用設定檔
            dry = self.cfg['dry_run'].get('modules')
        self.dry_run = dry_mod.parse(dry)
        # real_arm：true / false 覆蓋設定檔的 real_arm.required（launch real_arm:=false = 只測 MoveIt，不用另一份設定檔）
        self.declare_parameter('real_arm', 'config', ParameterDescriptor(dynamic_typing=True))
        real = str(self.get_parameter('real_arm').value).strip().lower()
        if real in ('true', 'false'):
            self.cfg.setdefault('real_arm', {})['required'] = real == 'true'
        elif real not in ('', 'config'):
            raise ValueError(f'real_arm 只接受 true / false / config，收到 {real!r}')

        self.state = 'IDLE'
        self.task_id = ''
        self.stage = -1
        self.last_error = ''
        self.feedback_text = ''     # 最新一筆 feedback.status、上一個任務的 result.message：放進 /manip/status，
        self.last_message = ''      # 讓不是送任務的那一方（網頁）也看得到（rosbridge 訂不到 action 的 feedback topic）
        self.estop = False
        self._lock = threading.Lock()
        self._busy = False
        self._gh = None
        self._deadline = float('inf')
        self._arm_used = None
        self._test_cancel = False   # 網頁測試區按了取消

        # 執行中的任務要一邊等其他 topic，所以訂閱放另一個 callback group、用多執行緒 executor
        sub_group = MutuallyExclusiveCallbackGroup()
        def client(name, real, dry):
            return (dry if name in self.dry_run else real)(self, self.cfg, sub_group)

        self.vision = client('vision', adapters.VisionClient, adapters.DryVisionClient)
        self.arm = client('arm', adapters.ArmClient, adapters.DryArmClient)
        self.hand = client('hand', adapters.HandClient, adapters.DryHandClient)

        self.create_subscription(Bool, '/estop', self._estop_cb, 10, callback_group=sub_group)
        self.pub_status = self.create_publisher(String, '/manip/status', 10)
        # 要抓的藥瓶（只給 RobotViewer 畫，跟 /arm_ik_obstacles 分開：它不是障礙物）。
        # {"class", "objects": [stick]}，objects 空 = 沒有目標。辨識到時發、之後跟 /manip/status 一起每秒重發，任務結束 10 秒後清掉
        self.pub_target = self.create_publisher(String, '/manip/target', LATCHED_QOS)
        self._target = None
        self._target_until = float('inf')
        self.create_timer(1.0, self._publish_status, callback_group=sub_group)

        # 網頁測試區、座標顯示
        self.create_subscription(String, '/manip/test_cmd', self._test_cb, 10, callback_group=sub_group)
        self.create_subscription(Float64, '/manip/speed', self._speed_cb, 10, callback_group=sub_group)
        self.pub_test = self.create_publisher(String, '/manip/test_result', 10)
        self.pub_pose = self.create_publisher(String, '/manip/pose', 10)
        self._shape, self._ticks_to_q = self._load_fk()
        self._q_sim, self._q_real = {}, (0.0, {})     # 模擬的關節角；(收到時間, 實機馬達刻度)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10, callback_group=sub_group)
        self.create_timer(0.5, self._publish_pose, callback_group=sub_group)

        self._server = ActionServer(
            self, ExecuteTaskPlan, '/manip/pick_place',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        dry = '、'.join(m for m in dry_mod.MODULES if m in self.dry_run)
        real = '要實機跟隨' if self.cfg.get('real_arm', {}).get('required', True) else '只看 MoveIt，不管實機'
        self.get_logger().info(f'等待取放任務：/manip/pick_place{f"（模擬：{dry}）" if dry else ""}（手臂：{real}）')

    # ------------------------------------------------------------------
    # 狀態
    # ------------------------------------------------------------------
    def _publish_status(self):
        d = {'state': self.state, 'task_id': self.task_id,
             'stage': self.stage, 'stage_name': STAGES[self.stage] if self.stage >= 0 else '',
             'last_error': self.last_error, 'feedback': self.feedback_text, 'last_message': self.last_message,
             'dry_run': [m for m in dry_mod.MODULES if m in self.dry_run], 'speed': self.arm.speed,
             'stamp': time.time()}
        self.pub_status.publish(String(data=json.dumps(d, ensure_ascii=False)))
        if self._target is not None and time.monotonic() > self._target_until:
            self._target = None
        self.pub_target.publish(String(data=json.dumps(self._target or {'class': '', 'objects': []})))

    def _estop_cb(self, msg):
        if msg.data and not self.estop:
            self.get_logger().error('收到急停')
            self.hand.stop()
            self.arm.stop('急停')
            self.state = 'ESTOP'
        elif not msg.data and self.estop:
            self.get_logger().info('急停解除')
            self.state = 'BUSY' if self._busy else 'IDLE'
        self.estop = bool(msg.data)
        self._publish_status()

    # ------------------------------------------------------------------
    # 等待（轉接層都透過這兩個等，取消、急停、逾時都在這裡檢查）
    # ------------------------------------------------------------------
    def _check(self):
        if self.estop:
            raise TaskError(ESTOP, '急停中')
        if (self._gh is not None and self._gh.is_cancel_requested) or self._test_cancel:
            raise Canceled()
        if time.monotonic() > self._deadline:
            raise TaskError(TIMEOUT, f'超過任務時限（階段：{STAGES[self.stage] if self.stage >= 0 else self.task_id}）')

    def sleep(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            self._check()
            time.sleep(0.02)

    def wait_until(self, pred, timeout, code, what):
        """pred() 回傳非 None 就結束並回傳該值；超過 timeout 丟 TaskError(code, what)。"""
        end = time.monotonic() + timeout
        while True:
            self._check()
            v = pred()
            if v is not None:
                return v
            if time.monotonic() > end:
                raise TaskError(code, what)
            time.sleep(0.02)

    # ------------------------------------------------------------------
    # Action
    # ------------------------------------------------------------------
    def _on_goal(self, request):
        with self._lock:
            if self.estop:
                self.get_logger().warning('拒絕任務：急停中')
                return GoalResponse.REJECT
            if self._busy:
                self.get_logger().warning(f'拒絕任務：{self.task_id} 還在執行')
                return GoalResponse.REJECT
            self._busy = True     # 在這裡就佔住，免得兩個 goal 同時被接受
        return GoalResponse.ACCEPT

    def _feedback(self, stage, retry, detail):
        self.stage = stage
        fb = ExecuteTaskPlan.Feedback()
        fb.status = f'{stage} {STAGES[stage]}｜retry={retry}｜{detail}'
        fb.progress = float(stage) / len(STAGES)
        self._gh.publish_feedback(fb)
        self.feedback_text = fb.status
        self._publish_status()
        self.get_logger().info(f'[{self.task_id}] {fb.status}')

    def _execute(self, gh):
        t0 = time.monotonic()
        self._gh = gh
        self._deadline = float('inf')
        self._arm_used = None
        self.task_id = ''
        self.state, self.stage, self.last_error = 'BUSY', -1, ''
        self.feedback_text = ''
        self._target, self._target_until = None, float('inf')
        result = ExecuteTaskPlan.Result()
        try:
            g = goal_mod.parse(gh.request.subtask.parameter_keys,
                               gh.request.subtask.parameter_values, self.cfg)
            self.task_id = g.task_id
            self._deadline = t0 + g.timeout_s
            note = self._run(g)
            gh.succeed()
            result.success, result.error_code = True, OK
            result.message = f'OK task_id={g.task_id} duration_s={time.monotonic() - t0:.1f}｜{note}'
            self.state = 'IDLE'
        except Canceled:
            self.hand.stop()
            self.arm.drop_held()
            gh.canceled()
            result.success, result.error_code = False, OK
            result.message = f'已取消 task_id={self.task_id} duration_s={time.monotonic() - t0:.1f}'
            self.state = 'IDLE'
        except TaskError as e:
            self.hand.stop()
            self.arm.drop_held()
            gh.abort()
            result.success, result.error_code = False, e.code
            result.message = f'{e} task_id={self.task_id} duration_s={time.monotonic() - t0:.1f}'
            self.last_error = code_text(e.code)
            self.state = 'ESTOP' if self.estop else 'ERROR'
        except Exception as e:      # 程式錯誤也要讓主控收到結果，不能卡住
            self.get_logger().error(f'未預期的錯誤：{e!r}')
            self.hand.stop()
            self.arm.drop_held()
            gh.abort()
            result.success, result.error_code = False, GRASP_FAILED
            result.message = f'{code_text(GRASP_FAILED)} 程式錯誤：{e!r}'
            self.state = 'ERROR'
        finally:
            self.get_logger().info(f'[{self.task_id}] 結果：{result.message}')
            self.last_message = result.message
            self._target_until = time.monotonic() + 10.0
            self._gh = None
            with self._lock:
                self._busy = False
            self._publish_status()
        return result

    def _speed_cb(self, msg):
        v = float(msg.data)
        if not 1.0 <= v <= SPEED_MAX:
            self.get_logger().warn(f'速度 {v:g} 不在 1～{SPEED_MAX:g}，不理它')
            return
        self.arm.set_speed(v)
        self.get_logger().info(f'手臂速度改成 {v:g}')
        self._publish_status()

    # ------------------------------------------------------------------
    # 網頁測試區（不跑完整流程）：跟取放任務共用 _busy，同時只做一件事
    # ------------------------------------------------------------------
    def _test_cb(self, msg):
        try:
            c = try_cmd.parse(msg.data, self.cfg)
        except ValueError as e:
            try:
                d = json.loads(msg.data)
                c = {'id': d.get('id', ''), 'cmd': d.get('cmd', '?')}
            except (ValueError, AttributeError):
                c = {'id': '', 'cmd': '?'}
            self._test_result(c, False, f'指令格式不對：{e}')
            return
        if c['cmd'] == 'cancel':
            if self._busy and self.task_id == 'TEST':
                self._test_cancel = True
            return
        with self._lock:
            reason = '急停中' if self.estop else (f'{self.task_id or "任務"} 還在執行' if self._busy else '')
            if not reason:
                self._busy = True
        if reason:
            self._test_result(c, False, f'沒有執行：{reason}')
            return
        threading.Thread(target=self._run_test, args=(c,), daemon=True).start()

    def _test_status(self, text):
        self.feedback_text = text
        self._publish_status()
        self.get_logger().info(f'[TEST] {text}')

    def _run_test(self, c):
        t0 = time.monotonic()
        self.task_id, self.state, self.stage, self.last_error = 'TEST', 'BUSY', -1, ''
        self._test_cancel = False
        self._deadline = t0 + self.cfg['timeouts'].get('test_s', 120.0)
        ok, text = False, ''
        try:
            self.arm.set_speed()
            if c['cmd'] == 'stand':
                self._test_status('回站姿')
                self.arm.stand(self)
                text = '回到站姿'
            elif c['cmd'] == 'ready':
                self._test_status(f'回預備姿勢（{c["arm"]}）')
                self.arm.ready(self, c['arm'])
                text = '回到預備姿勢'
            elif c['cmd'] == 'move':
                tf = try_cmd.both_frames(c['pos_cm'], self.cfg)['table']
                self._test_status(f'移到 桌面 右 {tf[0]}、前 {tf[1]}、上 {tf[2]}（{c["arm"]}、{c["approach"]}）')
                self.arm.set_obstacles(self, grasp_plan.scene(self.cfg, []))
                arm = self.arm.move(self, c['pos_cm'], c['approach'], c['arm'], '測試：移到指定點')
                text = f'{arm} 手到了' + self._reach_note(c['pos_cm'], arm)
            else:   # bottle
                self._test_status(f'找 {c["cls"]}')
                xyz, size, others = self.vision.locate(self, c['cls'])
                self._target = {'class': c['cls'],
                                'objects': [grasp_plan.bottle_obstacle(c['cls'], xyz, size, 0.0, oid='sp3_target')]}
                self._target_until = float('inf')
                pos = try_cmd.above_bottle(xyz, size, c['above'], self.cfg['frames'])
                self._test_status(f'移到 {c["cls"]} 瓶頂上方 {c["above"]:g} cm')
                self.arm.set_obstacles(self, grasp_plan.scene(self.cfg, others))
                arm = self.arm.move(self, pos, 'top', c['arm'], f'測試：移到 {c["cls"]} 上方')
                text = f'{arm} 手到了 {c["cls"]} 瓶頂上方 {c["above"]:g} cm' + self._reach_note(
                    grasp_plan.to_meas(xyz, self.cfg['frames']), arm)
            ok = True
            self.state = 'IDLE'
        except Canceled:
            self.arm.stop('網頁測試取消')
            text, self.state = '已取消', 'IDLE'
        except TaskError as e:
            text, self.last_error = str(e), code_text(e.code)
            self.state = 'ESTOP' if self.estop else 'ERROR'
        except Exception as e:      # noqa: BLE001  程式錯誤也要回報，不能卡在 BUSY
            self.get_logger().error(f'[TEST] 未預期的錯誤：{e!r}')
            text, self.state = f'程式錯誤：{e!r}', 'ERROR'
        finally:
            text += f' duration_s={time.monotonic() - t0:.1f}'
            self.last_message = text
            self._target_until = time.monotonic() + 10.0
            self._deadline = float('inf')
            with self._lock:
                self._busy = False
            self._test_result(c, ok, text)
            self._publish_status()

    def _reach_note(self, pos_cm, arm):
        """workspace 表（設定檔的可及範圍）算不算得到：測試時順便驗證表對不對。"""
        inside = arm in grasp_plan.reachable_arms(pos_cm, self.cfg)
        return f'（可及範圍表：{"範圍內" if inside else "範圍外"}）'

    def _test_result(self, c, ok, text):
        self.get_logger().info(f'[TEST] 結果：{"OK" if ok else "失敗"} {text}')
        self.pub_test.publish(String(data=json.dumps(
            {'id': c.get('id', ''), 'cmd': c.get('cmd'), 'ok': ok, 'message': text}, ensure_ascii=False)))

    # ------------------------------------------------------------------
    # 座標顯示（/manip/pose）：掌心用 URDF 算（只讀關節，不讀 STL），實機用馬達刻度經 arm_config 換算
    # ------------------------------------------------------------------
    def _load_fk(self):
        try:
            urdf = os.path.join(get_package_share_directory('seven_axis_arm_urdf'), 'urdf',
                                'seven_axis_arm_urdf.urdf')
            shape = ArmShape(urdf, get_package_share_directory, meshes=False)
        except Exception as e:      # noqa: BLE001
            self.get_logger().warning(f'讀不到手臂 URDF，網頁不顯示掌心座標：{e}')
            return None, None
        try:
            conv = real_ticks_converter(os.path.join(get_package_prefix('armpackage'), 'lib', 'armpackage'))
        except Exception:           # noqa: BLE001
            conv = None
        return shape, conv

    def _joint_cb(self, msg):
        if not msg.name:
            return
        if msg.name[0].isdigit():   # driver：馬達 ID、刻度
            self._q_real = (time.monotonic(), {int(n): p for n, p in zip(msg.name, msg.position)})
        else:                       # MoveIt：關節名、rad
            self._q_sim.update(zip(msg.name, msg.position))

    def _publish_pose(self):
        d = {'palm': {}, 'target': None, 'goal': None, 'stamp': time.time()}
        if self._shape is not None:
            t_real, ticks = self._q_real
            q_real = self._ticks_to_q(ticks) if self._ticks_to_q and ticks and time.monotonic() - t_real < 1.0 else {}
            for arm in ('left', 'right'):
                d['palm'][arm] = {
                    k: try_cmd.both_frames(grasp_plan.to_meas(self._shape.palm(q, arm), self.cfg['frames']), self.cfg)
                    if q else None for k, q in (('sim', self._q_sim), ('real', q_real))}
        if self._target and self._target['objects']:
            o = self._target['objects'][0]
            c = [(a + b) / 2 for a, b in zip(o['a'], o['b'])]
            d['target'] = dict(try_cmd.both_frames(grasp_plan.to_meas(c, self.cfg['frames']), self.cfg),
                               **{'class': self._target['class']})
        goal = getattr(self.arm, 'last_goal', None)
        if goal is not None:
            d['goal'] = dict(try_cmd.both_frames(goal[0], self.cfg), arm=goal[1], what=goal[2])
        self.pub_pose.publish(String(data=json.dumps(d, ensure_ascii=False)))

    # ------------------------------------------------------------------
    # 取放流程
    # ------------------------------------------------------------------
    def _run(self, g):
        gc = self.cfg['grasp']
        max_retry = self.cfg['retries']['grasp']
        # 先回預備姿勢（ready pose，2026-11-19 決定）：第一個任務從 stand 舉上來，之後本來就在 ready（手臂端直接回成功）。
        # 手臂端還沒有 /arm_ready 時這裡會失敗（MAN-E-007）。場景先放桌子
        self._feedback(0, 0, '回預備姿勢')
        self.arm.set_speed()
        self.arm.set_obstacles(self, grasp_plan.scene(self.cfg, []))
        self.arm.ready(self)
        for retry in range(max_retry + 1):
            try:
                poses = self._pick(g, retry)
                break
            except TaskError as e:
                if e.code != GRASP_FAILED or retry == max_retry:
                    raise
                self.get_logger().warning(f'[{g.task_id}] 抓取失敗，重試：{e}')
                self._back_off()

        if g.place_target:
            tgt = self.cfg['place_targets'][g.place_target]
            above, place = grasp_plan.place_poses(tgt, gc)
            approach = tgt.get('approach', 'top')
            self._feedback(5, 0, f'移到 {g.place_target} 上方')
            self.arm.move(self, above, approach, poses.arm, '移到放置點上方')
            self.arm.move(self, place, approach, poses.arm, '放下')
            self.hand.open(self, poses.arm)
            self.arm.detach(self)
            self.arm.move(self, above, approach, poses.arm, '放下後抬起')

        self._feedback(6, 0, '回預備姿勢')      # 任務之間停在 ready，收工才按 Stand
        self.arm.ready(self)
        held = '' if g.place_target else '（沒有放置點，藥瓶還在手上）'
        return f'{poses.arm} 手 {g.bottle_class} → {g.place_target or "—"}{held}'

    def _pick(self, g, retry):
        self._feedback(0, retry, f'找 {g.bottle_class}')
        xyz, size, others = self.vision.locate(self, g.bottle_class)
        center = grasp_plan.to_meas(xyz, self.cfg['frames'])
        self._target = {'class': g.bottle_class,
                        'objects': [grasp_plan.bottle_obstacle(g.bottle_class, xyz, size, 0.0, oid='sp3_target')]}
        self._publish_status()
        reachable = grasp_plan.reachable_arms(center, self.cfg)
        if not reachable:
            raise TaskError(OUT_OF_RANGE, f'{g.bottle_class} 在 {center.round(1).tolist()} cm，'
                                          '超出兩隻手的可及範圍')
        if g.place_target:          # 抓起來就不換手，所以要選放置點也搆得到的那隻
            place_reach = grasp_plan.reachable_arms(self.cfg['place_targets'][g.place_target]['pos_cm'], self.cfg)
            both = [a for a in reachable if a in place_reach]
            if not both:
                raise TaskError(OUT_OF_RANGE, f'沒有一隻手同時搆得到 {g.bottle_class}（{"、".join(reachable)} 手）'
                                              f'和 {g.place_target}（{"、".join(place_reach) or "都搆不到"}）')
            reachable = both
        if g.arm in ('left', 'right') and g.arm not in reachable:
            raise TaskError(OUT_OF_RANGE, f'{g.bottle_class} 在 {center.round(1).tolist()} cm，'
                                          f'{g.arm} 手搆不到藥瓶或放置點（{"、".join(reachable)} 手可以）')

        self._feedback(1, retry, f'{g.bottle_class} 在 {center.round(1).tolist()} cm，'
                                 f'障礙物：桌子{"、" + "、".join(o[0] for o in others) if others else ""}')
        self.arm.set_obstacles(self, grasp_plan.scene(self.cfg, others))
        self.arm.set_speed()

        # 選手（S3-01：運動規劃負責）：主控有指定就用那隻；auto 時兩隻手算出來的位置一樣（top / front）
        # 就送 auto 讓 moveit_bridge 依 IK、碰撞挑，不一樣（side）才由這裡照左右依序試
        cands = [a for a in grasp_plan.arm_order(g.arm, center) if a in reachable]
        plans = {a: grasp_plan.plan(center, size, a, self.cfg['grasp']) for a in cands}
        if len(cands) == 2 and grasp_plan.same_poses(plans['right'], plans['left']):
            tries = [('auto', cands)]
        else:
            tries = [(a, [a]) for a in cands]

        err = None
        for choice, hands in tries:
            self._feedback(2, retry, f'{"、".join(hands)} 手開掌、{choice} 移到預抓位置')
            try:
                for h in hands:
                    self.hand.open(self, h)
                arm = self.arm.move(self, plans[hands[0]].pregrasp, plans[hands[0]].approach,
                                    choice, '移到預抓位置')
                break
            except TaskError as e:
                if e.code not in SWITCH_ARM:
                    raise
                err = e
        else:
            raise err

        poses = plans[arm]
        self._arm_used = arm
        self._feedback(3, retry, f'{arm} 手靠近並夾持')
        self.arm.move(self, poses.grasp, poses.approach, arm, '移到抓取位置')
        self.hand.grip(self, arm)
        self.arm.attach(self, arm, grasp_plan.held_obstacle(xyz, size))   # 之後的移動會算手上的藥瓶

        self._feedback(4, retry, f'抬起 {self.cfg["grasp"]["lift_cm"]:g} cm 確認夾住')
        self.arm.move(self, poses.lift, poses.approach, arm, '抬起')
        self.hand.holding(self, arm)
        return poses

    def _back_off(self):
        """重試前：剛剛用的那隻手張開。手臂留在原地，下一輪會先移到預抓位置。"""
        if self._arm_used is None:
            return
        try:
            self.arm.detach(self)
            self.hand.open(self, self._arm_used)
        except TaskError as e:
            self.get_logger().warning(f'重試前張開手失敗：{e}')


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
