"""子三任務管理：接主控的取放任務，串接藥瓶辨識、手臂、手。

對外（L2，給主控）
    /manip/pick_place  Action，型別用主控的 interfaces/action/ExecuteTaskPlan
                       goal 的 keys/values 欄位見 goal.py
    /manip/status      std_msgs/String（JSON），1 Hz：state IDLE / BUSY / ERROR / ESTOP
    /estop             std_msgs/Bool，True = 急停（測試平台用；正式型別待定）

流程階段（S3-05 feedback.stage）：0 辨識、1 規劃、2 接近、3 抓取、4 驗證、5 放置、6 退回。
Feedback.status 格式：'<階段號> <階段名>｜retry=<n>｜<說明>'，progress = 完成的階段 / 7。
Result.error_code 是 errors.py 的序號（0 = 成功），message 開頭是完整錯誤碼。

參數
    config   設定檔路徑（預設 share/sp3_task_manager/config/task_manager.yaml）
    dry_run  true = 不接硬體，各轉接層模擬成功（設定檔的 dry_run.fail 可模擬失敗）
"""
import json
import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from interfaces.action import ExecuteTaskPlan
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from sp3_task_manager import adapters, goal as goal_mod, grasp_plan
from sp3_task_manager.errors import (ESTOP, GRASP_FAILED, OK, OUT_OF_RANGE, TIMEOUT,
                                     TaskError, code_text)

STAGES = ['辨識', '規劃', '接近', '抓取', '驗證', '放置', '退回']


class Canceled(Exception):
    pass


class TaskManager(Node):
    def __init__(self):
        super().__init__('sp3_task_manager')
        default_cfg = os.path.join(get_package_share_directory('sp3_task_manager'),
                                   'config', 'task_manager.yaml')
        self.declare_parameter('config', default_cfg)
        self.declare_parameter('dry_run', False)
        with open(self.get_parameter('config').value, encoding='utf-8') as f:
            self.cfg = yaml.safe_load(f)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        self.state = 'IDLE'
        self.task_id = ''
        self.stage = -1
        self.last_error = ''
        self.estop = False
        self._lock = threading.Lock()
        self._busy = False
        self._gh = None
        self._deadline = float('inf')
        self._arm_used = None

        # 執行中的任務要一邊等其他 topic，所以訂閱放另一個 callback group、用多執行緒 executor
        sub_group = MutuallyExclusiveCallbackGroup()
        if self.dry_run:
            self.vision = adapters.DryVisionClient(self, self.cfg, sub_group)
            self.arm = adapters.DryArmClient(self, self.cfg, sub_group)
            self.hand = adapters.DryHandClient(self, self.cfg, sub_group)
        else:
            self.vision = adapters.VisionClient(self, self.cfg, sub_group)
            self.arm = adapters.ArmClient(self, self.cfg, sub_group)
            self.hand = adapters.HandClient(self, self.cfg, sub_group)

        self.create_subscription(Bool, '/estop', self._estop_cb, 10, callback_group=sub_group)
        self.pub_status = self.create_publisher(String, '/manip/status', 10)
        self.create_timer(1.0, self._publish_status, callback_group=sub_group)

        self._server = ActionServer(
            self, ExecuteTaskPlan, '/manip/pick_place',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info(
            f'等待取放任務：/manip/pick_place{"（dry_run，不接硬體）" if self.dry_run else ""}')

    # ------------------------------------------------------------------
    # 狀態
    # ------------------------------------------------------------------
    def _publish_status(self):
        d = {'state': self.state, 'task_id': self.task_id,
             'stage': self.stage, 'stage_name': STAGES[self.stage] if self.stage >= 0 else '',
             'last_error': self.last_error, 'dry_run': self.dry_run, 'stamp': time.time()}
        self.pub_status.publish(String(data=json.dumps(d, ensure_ascii=False)))

    def _estop_cb(self, msg):
        if msg.data and not self.estop:
            self.get_logger().error('收到急停')
            self.hand.stop()
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
        if self._gh is not None and self._gh.is_cancel_requested:
            raise Canceled()
        if time.monotonic() > self._deadline:
            raise TaskError(TIMEOUT, f'超過任務時限（階段：{STAGES[self.stage]}）')

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
        self.get_logger().info(f'[{self.task_id}] {fb.status}')

    def _execute(self, gh):
        t0 = time.monotonic()
        self._gh = gh
        self._deadline = float('inf')
        self._arm_used = None
        self.task_id = ''
        self.state, self.stage, self.last_error = 'BUSY', -1, ''
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
            gh.canceled()
            result.success, result.error_code = False, OK
            result.message = f'已取消 task_id={self.task_id} duration_s={time.monotonic() - t0:.1f}'
            self.state = 'IDLE'
        except TaskError as e:
            self.hand.stop()
            gh.abort()
            result.success, result.error_code = False, e.code
            result.message = f'{e} task_id={self.task_id} duration_s={time.monotonic() - t0:.1f}'
            self.last_error = code_text(e.code)
            self.state = 'ESTOP' if self.estop else 'ERROR'
        except Exception as e:      # 程式錯誤也要讓主控收到結果，不能卡住
            self.get_logger().error(f'未預期的錯誤：{e!r}')
            self.hand.stop()
            gh.abort()
            result.success, result.error_code = False, GRASP_FAILED
            result.message = f'{code_text(GRASP_FAILED)} 程式錯誤：{e!r}'
            self.state = 'ERROR'
        finally:
            self.get_logger().info(f'[{self.task_id}] 結果：{result.message}')
            self._gh = None
            with self._lock:
                self._busy = False
            self._publish_status()
        return result

    # ------------------------------------------------------------------
    # 取放流程
    # ------------------------------------------------------------------
    def _run(self, g):
        gc = self.cfg['grasp']
        max_retry = self.cfg['retries']['grasp']
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
            self.arm.move(self, above, approach, poses.arm, '放下後抬起')

        self._feedback(6, 0, '回站姿')
        self.arm.stand(self)
        held = '' if g.place_target else '（沒有放置點，藥瓶還在手上）'
        return f'{poses.arm} 手 {g.bottle_class} → {g.place_target or "—"}{held}'

    def _pick(self, g, retry):
        self._feedback(0, retry, f'找 {g.bottle_class}')
        xyz, size = self.vision.locate(self, g.bottle_class)
        center = grasp_plan.to_meas(xyz, self.cfg['frames'])
        if not grasp_plan.in_workspace(center, self.cfg['workspace']):
            raise TaskError(OUT_OF_RANGE, f'{g.bottle_class} 在 {center.round(1).tolist()} cm，'
                                          '超出手臂工作範圍')

        self._feedback(1, retry, f'{g.bottle_class} 在 {center.round(1).tolist()} cm')
        arms = grasp_plan.arm_order(g.arm, center)
        self.arm.set_speed()

        err = None
        for arm in arms:
            poses = grasp_plan.plan(center, size, arm, self.cfg['grasp'])
            self._feedback(2, retry, f'{arm} 手開掌、移到預抓位置')
            try:
                self.hand.open(self, arm)
                self.arm.move(self, poses.pregrasp, poses.approach, arm, '移到預抓位置')
                break
            except TaskError as e:
                if e.code != GRASP_FAILED:
                    raise
                err = e
        else:
            raise err

        self._arm_used = arm
        self._feedback(3, retry, f'{arm} 手靠近並夾持')
        self.arm.move(self, poses.grasp, poses.approach, arm, '移到抓取位置')
        self.hand.grip(self, arm)

        self._feedback(4, retry, f'抬起 {self.cfg["grasp"]["lift_cm"]:g} cm 確認夾住')
        self.arm.move(self, poses.lift, poses.approach, arm, '抬起')
        self.hand.holding(self, arm)
        return poses

    def _back_off(self):
        """重試前：剛剛用的那隻手張開。手臂留在原地，下一輪會先移到預抓位置。"""
        if self._arm_used is None:
            return
        try:
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
