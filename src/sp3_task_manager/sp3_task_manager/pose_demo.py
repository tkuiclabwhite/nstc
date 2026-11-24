"""成果展示：手到藥瓶旁邊 2 cm，左右環繞、換姿態，展示掌心的位置、姿態都控制得到（2026-11-24）。

預設（--parts ring，2026-11-24 晚上使用者選的）：手離藥瓶表面 --gap、手指一直朝前，正上方 → 傾 30° 繞一圈
    （外、外前、前、內前、內、內後、後、外後 8 個方向）→ 傾 45° 再繞一圈 → 往外、往內傾 60° → 回正上方。
    右 18.9、前 28（b1）離線算 20 步到 19 步（17 個不同的點，只有 45° 靠身體那側到不了），手指都剛好朝前。
    --finger-test 在最後加手指朝上／朝下的測試（多半手腕轉不到，預設不做）。
--parts sides（使用者 2026-11-24 下午改的，只有 4 個不同的到達點）：手離藥瓶表面 --gap（2 cm）、速度 50，
    正上方（掌心朝下、手指朝前）→ 往外側傾 30°、45° → 測試：45° 時手指朝上、朝下 → 回正上方
    → 往內側傾 30°、45° → 測試：手指朝上、朝下 → 回正上方 → 回 ready → 站姿。
    測試步一定送給 moveit_bridge，終端機印它的判斷（到了／會撞到 X 擋下／到不了）。不是測試的步手會碰到桌子就自動離遠一點。
    終端機每一步一行（錄影用），最後一張總表。
舊的、比較細的版本：--parts spin orbit（掌心離瓶頂中心 --radius 15 cm，原地換姿態、每 15° 繞一步）。

不經任務管理，直接對 moveit_bridge 下 /arm_ik_target（PoseStamped：掌心位置 + 整個手掌姿態）。/arm_move 只有 6 種固定的
掌心方向，做不到「掌心一直對著藥瓶、角度一點一點變」；/arm_ik_target 的碰撞檢查、實機高度補償、刻度檢查都跟 /arm_move 一樣。
姿態序列、手指角度怎麼決定見 demo_poses.py。

    ros2 run sp3_task_manager pose_demo --bottle b1 --plan-only   # 影像找 b1，只規劃、印每一步（手臂不動）
    ros2 run sp3_task_manager pose_demo --bottle b1               # 影像找 b1，照做（先印規劃，按 Enter 才動）
    ros2 run sp3_task_manager pose_demo --at 18.9 28              # 不開影像：藥瓶在桌面座標 右 18.9、前 28（尺寸照 --bottle，預設 b1）
    ros2 run sp3_task_manager pose_demo --at 18.9 28 --gap 3      # 實機第一次建議離遠一點（實機有幾 cm 的誤差）
    ros2 run sp3_task_manager pose_demo --at 18.9 20 --parts spin orbit   # 舊的細版本
    ros2 run sp3_task_manager pose_demo --bottle b1 --real        # 實機跟隨開著：手臂會真的動
    ros2 run sp3_task_manager pose_demo --replay ~/reach_test_out/<上次的資料夾> --real   # 錄影：照上次錄好的做（demo_replay.py）

藥瓶建議擺位（2026-11-24 離線算過）：
  sides（預設）：右 18.9～25（左手 −18.9～−25）、前 28：實機跟隨開不開、--gap 2～4，手指都剛好朝前、7 步都到得了。
    實機跟隨開著時 moveit_bridge 把模擬的目標往上補 0.17 × 前（約 5～6 cm），掌心一高、手指就轉不到朝前：
    前 ≤ 22 手指就轉不到朝前（前 26 + --gap 4 也是）→ 自動改成最接近的手指角度（終端機會寫偏幾度）。
    往內側傾 45° 離瓶子 2 cm 時手會碰到桌子，自動改離 3 cm。
    手指朝上／朝下的測試多半是手腕轉不到（到不了）。
  spin / orbit（舊）：右 10～30、前 18～21（不開實機補償時 39 步都是連續的微調）。

流程：先規劃、印每一步 → Enter 開始 → 場景放桌子、小桌子、藥瓶（要繞的那瓶也當障礙物）→ 張開手 → 這隻手到 ready
（任務管理開著就照錄好的路線）→ 藥瓶正上方 → 依實際的姿勢重新規劃 → 一步一步做（每步等實機停穩、停 --hold 秒、記誤差）
→ 回 ready → 回站姿（--end）。到不了的那一步略過（測試步 = 擋下），接著做下一步；執行失敗、取消、MoveIt 沒回應就停在原地。
Ctrl+C = /arm_stop（停在原地，不會自己回站姿）。
開始時實機跟隨開著、中途斷掉（real_arm_bridge 停了，或 3 秒沒收到 /arm_real_status）就馬上停（送 /arm_stop），
印 real_arm_bridge 寫的原因（2026-11-24 使用者遇過：跟隨中斷、肩膀洩力，懷疑電池沒電）。中途停下時場景（藥瓶）留著，手臂回去時才會避開它；
下次跑 pose_demo／reach_test 會換掉，或 `pose_demo --clear` 清掉。

結果在 ~/reach_test_out/<時間>_pose_demo_<手>/：plan.csv（規劃）、result.csv（每一步要的、實際的掌心位置與姿態誤差）。
實際的位置是用關節角經 URDF 算的，都跟「模擬的目標」比（實機跟隨開著時 = 目標 + moveit_bridge 往上補的 z_comp_cm）：
  模擬（sim_*）= MoveIt 的關節角：IK、軌跡執行準不準
  實機（real_*）= 馬達讀數（編碼器）：馬達有沒有跟上模擬。實機馬達是照模擬的關節角轉的，算出來的掌心本來就跟模擬一樣高
               （比目標高 z_comp）；真正的掌心會因為機構下垂再低一點，這個馬達讀數看不到，要用尺量（reach_test point）

任務管理開著沒關係（不要同時送任務）。moveit_bridge 的 /arm_ik_arm 會改成這次用的手，做完改回 auto。
"""
import argparse
import csv
import datetime
import json
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory, PackageNotFoundError
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64, String

from sp3_task_manager import demo_poses as dp
from sp3_task_manager import demo_replay as dr
from sp3_task_manager import grasp_plan as gp
from sp3_task_manager import ready_path
from sp3_task_manager.arm_shape import distance, real_ticks_converter
from sp3_task_manager.reach_test import SOURCE, Tester, load_shape, short, table_with_scene

SKIP_REASONS = ('unreachable', 'joint_limit', 'collision', 'plan_failed')   # 這一步沒做，接著做下一步
REAL_STALE_S = 3.0      # real_arm_bridge 每秒發一次 /arm_real_status：超過這麼久沒收到 = 它沒在跑
REPLAN_DEG = 3.0        # 實際的關節角跟規劃的差超過這麼多（moveit_bridge 挑了別的姿勢）：剩下的從實際的姿勢重新規劃
INSIDE_TOL = 0.003      # moveit_bridge：掌心目標離物體表面不到這麼多（m）就直接拒絕（同它的 INSIDE_TOL）
HOW_TEXT = {'small': '連續', 'general': '退開再靠近', '': '略過'}
REASON_TEXT = {'unreachable': '到不了', 'joint_limit': '到不了（關節轉不到）', 'collision': '會撞到東西',
               'plan_failed': '找不到路', 'exec_failed': '執行失敗', 'canceled': '被取消', 'moveit_down': 'MoveIt 沒回應',
               'busy': 'moveit_bridge 忙碌中', 'timeout': '沒有回報', 'bad_request': '指令格式不對', 'internal': 'moveit_bridge 出錯'}
OBJ_NAMES = (('held', '手上的藥瓶'), ('table', '桌子'), ('shelf', '後面的小桌子'), ('bottle', '藥瓶'))
BAR = '═' * 18


def why_text(res):
    """moveit_bridge 失敗的結果 → 給人看的一句話（會撞到時說撞到什麼）。"""
    r = res.get('reason') or ''
    txt = REASON_TEXT.get(r, r or '失敗')
    if r == 'collision':
        note = res.get('note') or ''
        hit = [name for key, name in OBJ_NAMES if key in note]
        if hit:
            txt = '會撞到' + '、'.join(dict.fromkeys(hit))
    return txt


class FollowLost(RuntimeError):
    """實機跟隨中途斷掉。"""


class FollowTurnedOn(FollowLost):
    """開始時實機跟隨關著、中途被打開。"""


class Demo(Tester):
    def __init__(self, cfg, args):
        super().__init__(cfg, args)
        self.pub_target = self.create_publisher(PoseStamped, '/arm_ik_target', 10)
        self.pub_arm = self.create_publisher(String, '/arm_ik_arm', 10)
        self.pub_mode = self.create_publisher(String, '/arm_ik_mode', 10)
        self.pub_stop = self.create_publisher(Bool, '/arm_stop', 10)
        self.pub_hand = self.create_publisher(String, '/hand/cmd', 10)
        self.q_sim, self.ticks, self.real_t, self.manip = {}, (0.0, {}), 0.0, None
        self.follow_at_start = False
        self._valid_cli = self._exec_ac = None
        self.create_subscription(String, '/manip/status', self._manip_cb, 10)

    def _joint_cb(self, msg):
        super()._joint_cb(msg)          # reach_test：移動途中記路徑
        if not msg.name:
            return
        if msg.name[0].isdigit():       # driver：馬達 ID、刻度
            self.ticks = (time.monotonic(), {int(n): p for n, p in zip(msg.name, msg.position)})
        else:                           # MoveIt：關節名、rad（可能只有部分關節，累積起來）
            self.q_sim.update(zip(msg.name, msg.position))

    def _real_cb(self, msg):
        super()._real_cb(msg)
        self.real_t = time.monotonic()

    def _manip_cb(self, msg):
        try:
            self.manip = json.loads(msg.data)
        except ValueError:
            pass

    def following(self):
        return (self.real or {}).get('state') == 'follow'

    def task_manager_busy(self):
        """任務管理正在跑任務（/manip/status = BUSY）就不做。等 1.5 秒（1 Hz），沒收到 = 沒開。"""
        end = time.monotonic() + 1.5
        while self.manip is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return (self.manip or {}).get('state') == 'BUSY'

    def set_arm(self, arm):
        """moveit_bridge 的 /arm_ik_mode = full、/arm_ik_arm = 這隻手（沒有回報：等連上再送，送完等一下）。"""
        end = time.monotonic() + 5.0
        while time.monotonic() < end and (self.pub_arm.get_subscription_count() == 0 or
                                          self.pub_mode.get_subscription_count() == 0):
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.pub_arm.get_subscription_count() == 0:
            raise RuntimeError('moveit_bridge 沒有訂閱 /arm_ik_arm（手臂 MoveIt 有開嗎？）')
        self.pub_mode.publish(String(data='full'))
        self.pub_arm.publish(String(data=arm))
        self.spin_for(0.5)

    def move_pose(self, pos, R):
        """掌心移到 pos（base_link m）、姿態 R：/arm_ik_target，等 /arm_ik_status（cmd = move）。"""
        if not self.args.real and self.following():
            raise RuntimeError('實機跟隨被打開了，沒加 --real：停止，手臂不再動')
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in pos)
        x, y, z, w = (float(v) for v in dp.quat(R))
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = x, y, z, w
        return self.send(lambda: self.pub_target.publish(msg), 'move')

    def watch_follow(self):
        """開始動手臂前叫：實機跟隨現在開著，之後 check_follow 就會檢查它有沒有中途斷掉。"""
        self.follow_at_start = self.following()

    def check_follow(self):
        """開始時實機跟隨開著、現在斷了（real_arm_bridge 停了或沒在跑）就丟 FollowLost。
        開始時關著、中途被打開也丟：moveit_bridge 會開始往上補 0.17 × 前，跟規劃（沒補）對不上
        （2026-11-24 17:56 就是這樣：手離瓶子量到 10 cm、掌心誤差 7 cm）。"""
        rclpy.spin_once(self, timeout_sec=0.0)
        if not self.follow_at_start:
            if self.following():
                raise FollowTurnedOn('實機跟隨中途被打開了（開始時是關的）：高度補償跟規劃的對不上。'
                                 '要實機就先打開實機跟隨、再重跑 pose_demo（加 --real）')
            return
        if time.monotonic() - self.real_t > REAL_STALE_S:
            # 剛剛可能在算規劃（沒在收訊息）：再等一下新的狀態，真的沒有才算斷掉
            t0 = time.monotonic()
            while time.monotonic() - self.real_t > REAL_STALE_S and time.monotonic() - t0 < REAL_STALE_S:
                rclpy.spin_once(self, timeout_sec=0.05)
            age = time.monotonic() - self.real_t
            if age > REAL_STALE_S:
                raise FollowLost(f'{age:.0f} 秒沒收到 /arm_real_status（real_arm_bridge 沒在跑？）')
        st = self.real or {}
        if st.get('state') != 'follow':
            raise FollowLost(f'real_arm_bridge：{st.get("msg") or st.get("state")}')

    def wait_settled(self, t_res, limit=8.0):
        """實機跟隨開著：等 MoveIt 回報之後才收到、settled = true 的 /arm_real_status。沒開實機直接 True。
        等的時候實機跟隨斷掉就丟 FollowLost。"""
        if not self.follow_at_start and not self.following():
            return True
        end = time.monotonic() + limit
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.check_follow()
            if self.real_t > t_res and (self.real or {}).get('settled'):
                return True
        return False

    def moveit_hits(self, joints, P, group):
        """路線每一點問 MoveIt /check_state_validity（同任務管理照錄好的路線走之前的檢查）。
        沒撞到回 None，撞到回說明。同時最多 8 個請求：服務佇列只有 10 筆，一次全送會被丟掉。"""
        from moveit_msgs.srv import GetStateValidity
        if self._valid_cli is None:
            self._valid_cli = self.create_client(GetStateValidity, '/check_state_validity')
        if not self._valid_cli.wait_for_service(timeout_sec=2.0):
            return '連不到 MoveIt 的 /check_state_validity'
        n, pending, nxt = len(P), {}, 0
        end = time.monotonic() + 5.0 + 0.05 * n
        while nxt < n or pending:
            for i, f in list(pending.items()):
                if not f.done():
                    continue
                del pending[i]
                r = f.result()
                if r is None:
                    return f'第 {i}/{n - 1} 點 MoveIt 碰撞檢查沒有結果'
                if not r.valid:
                    pairs = sorted({f'{c.contact_body_1}↔{c.contact_body_2}' for c in r.contacts}) or ['?']
                    return f'MoveIt：第 {i}/{n - 1} 點會撞到 {"、".join(pairs[:2])}'
            while nxt < n and len(pending) < 8:
                req = GetStateValidity.Request()
                req.group_name = group
                req.robot_state.is_diff = True
                req.robot_state.joint_state.name = list(joints)
                req.robot_state.joint_state.position = [float(v) for v in P[nxt]]
                pending[nxt] = self._valid_cli.call_async(req)
                nxt += 1
            if time.monotonic() > end:
                return 'MoveIt 碰撞檢查（/check_state_validity）沒有回應'
            rclpy.spin_once(self, timeout_sec=0.005)
        return None

    def run_traj(self, joints, P, speed):
        """關節角路線排時間（ready_path.time_path，同任務管理照錄好的路線走），直接交給 MoveIt /execute_trajectory
        （不經 moveit_bridge、不用規劃）。等它走完，回傳 (ok, 說明)。等的時候實機跟隨斷掉、Ctrl+C 就取消這條軌跡。"""
        from builtin_interfaces.msg import Duration
        from moveit_msgs.action import ExecuteTrajectory
        from moveit_msgs.msg import RobotTrajectory
        from rclpy.action import ActionClient
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        if self._exec_ac is None:
            self._exec_ac = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        if not self._exec_ac.wait_for_server(timeout_sec=3.0):
            return False, '連不到 MoveIt 的 /execute_trajectory（MoveIt 有開嗎？）'
        tt, V = ready_path.time_path(P, speed)
        jt = JointTrajectory(joint_names=list(joints))
        for q, v, ti in zip(P, V, tt):
            ns = int(round(float(ti) * 1e9))
            jt.points.append(JointTrajectoryPoint(
                positions=[float(x) for x in q], velocities=[float(x) for x in v],
                time_from_start=Duration(sec=ns // 1000000000, nanosec=ns % 1000000000)))
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = RobotTrajectory(joint_trajectory=jt)
        fut = self._exec_ac.send_goal_async(goal)
        end = time.monotonic() + 5.0
        while not fut.done():
            if time.monotonic() > end:
                return False, 'MoveIt 5 秒沒有回應'
            rclpy.spin_once(self, timeout_sec=0.02)
        gh = fut.result()
        if not gh.accepted:
            return False, 'MoveIt 不接受這條軌跡'
        rf = gh.get_result_async()
        limit = float(tt[-1]) + 15.0
        end = time.monotonic() + limit
        try:
            while not rf.done():
                if time.monotonic() > end:
                    gh.cancel_goal_async()
                    return False, f'{limit:.0f} 秒還沒走完'
                rclpy.spin_once(self, timeout_sec=0.02)
                self.check_follow()
        except BaseException:
            gh.cancel_goal_async()
            raise
        code = rf.result().result.error_code.val
        return code == 1, '' if code == 1 else f'MoveIt 執行失敗（錯誤碼 {code}）'

    def open_hand(self, arm):
        """張開手（同網頁「張開」），掌心看得清楚，也跟碰撞模型的手指伸直一致。handpackage 沒開就算了。"""
        if self.pub_hand.get_subscription_count() == 0:
            self.spin_for(1.0)
        if self.pub_hand.get_subscription_count() == 0:
            print('（handpackage 沒開，手不張開）')
            return
        h = self.cfg['hand']
        self.pub_hand.publish(String(data=json.dumps({'cmd': 'manual', 'hand': arm, 'speed': h['open_spd'],
                                                      'position': h['open_pos']})))
        self.spin_for(h.get('open_wait_s', 1.0))


# ----------------------------------------------------------------------
def load_kinematics(arm):
    """armpackage 的 IK（kinematics.ChainIK）與同 moveit_bridge 的關節範圍：URDF ∩ joint_limits.yaml，
    再加 moveit_bridge check() 的線材刻度範圍、離站姿最多 MAX_TRAVEL。回傳 (ik, 關節名)。"""
    lib = os.path.join(get_package_prefix('armpackage'), 'lib', 'armpackage')
    if lib not in sys.path:
        sys.path.insert(0, lib)
    import arm_config as ac
    from kinematics import ChainIK, chain_limits
    urdf = os.path.join(get_package_share_directory('seven_axis_arm_urdf'), 'urdf', 'seven_axis_arm_urdf.urdf')
    with open(os.path.join(get_package_share_directory('seven_axis_arm_moveit_config'), 'config',
                           'joint_limits.yaml'), encoding='utf-8') as f:
        lim = yaml.safe_load(f)
    over = {k: (v['min_position'], v['max_position']) for k, v in lim['joint_limits'].items()
            if v.get('has_position_limits')}
    joints = ac.ARMS[arm]['joints']
    lo, hi = chain_limits(urdf, joints, over)
    tlo, thi = ac.tick_range(arm)
    lo = np.maximum.reduce([lo, tlo, ac.STAND - ac.MAX_TRAVEL])
    hi = np.minimum.reduce([hi, thi, ac.STAND + ac.MAX_TRAVEL])
    return ChainIK(urdf, 'base_link', ac.ARMS[arm]['tip'], ac.TOOL_OFFSET, lo, hi), joints


def detect(t, cls, sec=2.0):
    """收 sec 秒 /bottle_detections_3d，cls 那瓶的中心取平均 → (中心 base_link m, 尺寸, 其他瓶 [(類別, 中心, 尺寸)], 抖動 mm)。"""
    from vision_msgs.msg import Detection3DArray
    seen = []

    def cb(msg):
        dets = {d.results[0].hypothesis.class_id: d for d in msg.detections if d.results}
        if cls in dets:
            def xyz(d):
                return np.array([d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z])

            def size(d):
                return np.array([d.bbox.size.x, d.bbox.size.y, d.bbox.size.z])
            seen.append((xyz(dets[cls]), size(dets[cls]),
                         [(c, xyz(d), size(d)) for c, d in sorted(dets.items()) if c != cls]))

    sub = t.create_subscription(Detection3DArray, '/bottle_detections_3d', cb, 10)
    t.spin_for(sec)
    t.destroy_subscription(sub)
    if not seen:
        raise RuntimeError(f'{sec:g} 秒內沒看到 {cls}（bottle_detect_v2、bottle_localization 有開嗎？瓶子在畫面裡嗎？）'
                           '｜不開影像就用 --at 右 前 給位置')
    P = np.array([s[0] for s in seen])
    return P.mean(axis=0), seen[-1][1], seen[-1][2], (P.max(axis=0) - P.min(axis=0)) * 1000


def table_xyz(p_m, cfg):
    """base_link（m）→ 桌面座標 [右, 前, 上]（cm）。"""
    return gp.meas_to_table(gp.to_meas(p_m, cfg['frames']), cfg['table'], cfg['frames'])


def table_delta(a_m, b_m, cfg):
    """兩點的差（a − b）換成桌面座標的方向 [右, 前, 上]（cm）。"""
    d = gp.to_meas(a_m, cfg['frames']) - gp.to_meas(b_m, cfg['frames'])
    return np.array([d[1], d[0], d[2]])


def make_z_comp(cfg, on):
    """實機跟隨開著時 moveit_bridge 把模擬的目標往上補 REAL_Z_COMP × 前（設定檔 ready_pose.real_z_comp 同一個值）。
    回傳 f(p) → 往上補幾 m。"""
    k = float((cfg.get('ready_pose') or {}).get('real_z_comp', 0.17)) if on else 0.0

    def f(p):
        return k * max(float(gp.to_meas(p, cfg['frames'])[0]), 0.0) / 100.0
    return f


def bottle_box(c, size, margin_cm, oid):
    """要圍著轉的藥瓶發給場景時用方柱（不用 stick）：moveit_bridge 規劃前的「掌心在物體裡面」檢查把 stick 當成
    兩端半圓球的膠囊（頂端多凸出一個半徑，約 5 cm），MoveIt 碰撞卻是平頂圓柱 → 手在瓶頂上方幾 cm 一定被擋
    （2026-11-24 實測）。方柱兩邊都是平頂、一致；左右傾時手對著的是面，跟圓柱一樣遠。"""
    m = margin_cm / 100.0
    return {'id': oid, 'type': 'box', 'c': [float(v) for v in c],
            'h': [float(size[0]) / 2 + m, float(size[1]) / 2 + m, float(size[2]) / 2 + m], 'R': np.eye(3).tolist()}


def bridge_surface_dist(o, p):
    """同 moveit_bridge.surface_dist：掌心點 p 到物體表面（m，負的 = 在裡面）；stick 當膠囊算（它就是這樣算的）。"""
    p = np.asarray(p, float)
    if o.get('type') == 'ball':
        return float(np.linalg.norm(p - np.asarray(o['c'], float)) - o['r'])
    if o.get('type') == 'box':
        d = np.abs(np.asarray(o['R'], float).T @ (p - np.asarray(o['c'], float))) - np.asarray(o['h'], float)
        return float(np.linalg.norm(np.maximum(d, 0.0)) + min(d.max(), 0.0))
    if o.get('type') == 'stick':
        a, b = np.asarray(o['a'], float), np.asarray(o['b'], float)
        s = np.clip((p - a) @ (b - a) / max((b - a) @ (b - a), 1e-12), 0.0, 1.0)
        return float(np.linalg.norm(p - (a + s * (b - a))) - o['r'])
    return None


def make_clearance(shape, joints, arm, objs, ik, z_comp):
    """手（手臂 + 手）離場景物體最近多遠（m）。實機跟隨開著時 moveit_bridge 也檢查實機（比模擬低 z_comp）的位置，
    這裡也一起算（手往下移 z_comp 再量）。另外照 moveit_bridge 規劃前的檢查：掌心目標（沒補高度的）離物體表面
    不到 INSIDE_TOL 就算撞到（不然規劃說到得了、送出去卻被擋）。shape = None 不檢查。"""
    if shape is None:
        return None
    solid = [o for o in objs if o.get('type') != 'plane']

    def f(q):
        pts = shape.points(dict(zip(joints, q)), arm)
        palm = ik(q)[:3, 3]
        dz = z_comp(palm)
        sets = [pts] + ([pts - [0.0, 0.0, dz]] if dz > 1e-4 else [])
        c = min(distance(P, o)[0] for P in sets for o in solid)
        target = palm - [0.0, 0.0, dz]
        inside = [bridge_surface_dist(o, target) for o in solid]
        return min([c] + [d - INSIDE_TOL for d in inside if d is not None])
    return f


def ready_q(ik, cfg, arm, seeds):
    """ready pose 的關節角（IK 估的，只用來排起點的候選順序）；解不到用站姿（全 0）。"""
    _, _, pos = gp.ready_targets(cfg)
    p = gp.from_meas(pos[arm], cfg['frames'])
    sols = ik.solve_many(p, seeds, axis=-dp.UP, iters=200)
    return min(sols, key=lambda q: np.abs(q).max()) if sols else np.zeros(7)


def print_plan(steps, cfg, title):
    n_ok = sum(s.ok for s in steps)
    n_gen = sum(s.ok and s.how == 'general' for s in steps[1:])
    n_test = sum(s.test and not s.ok for s in steps)
    print(f'\n== {title}：{len(steps)} 步，預測到得了 {n_ok}（其中 {n_gen} 步要先退開再靠近）、到不了 {len(steps) - n_ok}'
          + (f'（{n_test} 步是測試，照樣送）' if n_test else ''))
    print(' #  動作                              掌心 桌面[右, 前, 上] cm  離瓶子   走法        離物體（含加厚）')
    for i, st in enumerate(steps):
        x = table_xyz(st.pos, cfg)
        g = '   -   ' if st.extra.get('gap_cm') is None else f'{st.extra["gap_cm"]:4.1f} cm'
        c = '   -   ' if st.clear_cm is None else f'{st.clear_cm:5.1f} cm'
        how = HOW_TEXT[st.how if st.ok else ''] if st.ok or not st.test else '測試（照送）'
        print(f'{i + 1:2d}  {st.label:30s} [{x[0]:6.1f},{x[1]:6.1f},{x[2]:6.1f}]  {g}  {how:10s}{c}  {st.note}')


def write_plan(path, steps, cfg):
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['step', 'part', 'label', 'ok', 'how', 'right_cm', 'front_cm', 'up_cm',
                    'axis_x', 'axis_y', 'axis_z', 'fingers_deg', 'clear_cm', 'note'])
        for i, st in enumerate(steps):
            x = table_xyz(st.pos, cfg)
            w.writerow([i + 1, st.part, st.label, int(st.ok), st.how] + [round(float(v), 2) for v in x] +
                       [round(float(v), 4) for v in st.axis] +
                       ['' if st.fingers is None else round(st.fingers, 1),
                        '' if st.clear_cm is None else round(st.clear_cm, 2), st.note])


RESULT_COLS = ['step', 'part', 'label', 'result', 'reason', 'target_right', 'target_front', 'target_up', 'fingers_deg',
               'sim_right', 'sim_front', 'sim_up', 'sim_err_right', 'sim_err_front', 'sim_err_up', 'sim_err_cm',
               'sim_rot_err_deg', 'real_right', 'real_front', 'real_up', 'real_err_right', 'real_err_front',
               'real_err_up', 'real_err_cm', 'real_rot_err_deg', 'z_comp_cm', 'bottle_gap_cm', 'settled', 'note',
               'bridge']


def make_gauge(shape, joints, arm, bottle):
    """手（整隻手臂 + 手，URDF 外形）離藥瓶（不加厚）表面多遠（cm）：g(關節角, 往下移 dz m)。
    實機跟隨開著時模擬的手比目標高 dz（moveit_bridge 補的），往下移 dz = 實機應該在的位置。"""
    if shape is None:
        return None

    def g(q, dz=0.0):
        pts = shape.points(dict(zip(joints, q)), arm) - [0.0, 0.0, dz]
        return distance(pts, bottle)[0] * 100
    return g


def measure(t, st, ik, joints, conv, z_comp, t_res, cfg, gauge=None):
    """停穩之後：模擬（MoveIt 關節角）、實機（馬達讀數）的掌心經 URDF 算的位置、姿態，跟要的比；手離藥瓶多遠。"""
    out = {}
    q = np.array([t.q_sim.get(j, np.nan) for j in joints])
    dz = z_comp(st.pos)
    out['z_comp_cm'] = round(dz * 100, 2)
    if not np.isnan(q).any():
        T = ik(q)
        want = st.pos + dp.UP * dz                       # 模擬的目標（實機跟隨開著時 moveit_bridge 往上補）
        e = table_delta(T[:3, 3], want, cfg)
        x = table_xyz(T[:3, 3], cfg)
        out.update(sim_right=x[0], sim_front=x[1], sim_up=x[2], sim_err_right=e[0], sim_err_front=e[1],
                   sim_err_up=e[2], sim_err_cm=float(np.linalg.norm(e)), sim_rot_err_deg=dp.rot_deg(T[:3, :3], st.R))
        out['q'] = q
        if gauge is not None:
            out['bottle_gap_cm'] = gauge(q, dz)
    if t.following() and conv is not None and t.ticks[0] > t_res:
        qr = conv(t.ticks[1])
        if all(j in qr for j in joints):
            T = ik(np.array([qr[j] for j in joints]))
            e = table_delta(T[:3, 3], st.pos + dp.UP * dz, cfg)    # 實機馬達照模擬的關節角轉：跟模擬的目標比
            x = table_xyz(T[:3, 3], cfg)
            out.update(real_right=x[0], real_front=x[1], real_up=x[2], real_err_right=e[0], real_err_front=e[1],
                       real_err_up=e[2], real_err_cm=float(np.linalg.norm(e)),
                       real_rot_err_deg=dp.rot_deg(T[:3, :3], st.R))
    return out


def row_of(i, st, result, reason, m, cfg, settled='', bridge=''):
    x = table_xyz(st.pos, cfg)
    r = {'step': i + 1, 'part': st.part, 'label': st.label, 'result': result, 'reason': reason or '', 'bridge': bridge,
         'target_right': x[0], 'target_front': x[1], 'target_up': x[2],
         'fingers_deg': '' if st.fingers is None else st.fingers, 'settled': settled, 'note': st.note}
    r.update({k: v for k, v in (m or {}).items() if k in RESULT_COLS})
    return {k: (round(float(v), 2) if isinstance(v, (float, np.floating)) else v) for k, v in r.items()}


RESULT_TEXT = {'ok': '✓ 到了', 'blocked': '✗ 擋下', 'failed': '✗ 沒做', 'skipped': '－ 略過'}


def summary(rows):
    done = [r for r in rows if r['result'] == 'ok']
    print(f'\n{BAR} 總結 {BAR}')
    print(' #   動作                              結果                     離瓶子    掌心誤差   姿態誤差')
    for r in rows:
        res = RESULT_TEXT.get(r['result'], r['result'])
        if r['result'] in ('blocked', 'failed') and r.get('reason'):
            res += f'（{r["bridge"] or REASON_TEXT.get(r["reason"], r["reason"])}）'
        gap = r.get('bottle_gap_cm')
        err, rot = r.get('sim_err_cm'), r.get('sim_rot_err_deg')
        if r.get('real_err_cm') not in (None, ''):
            err, rot = r['real_err_cm'], r['real_rot_err_deg']
        print(f'{r["step"]:2d}   {r["label"]:30s} {res:22s} '
              + (f'{gap:5.1f} cm' if gap not in (None, '') else '   -    ')
              + (f'  {err * 10:6.1f} mm  {rot:6.2f}°' if err not in (None, '') else ''))
    print(f'做了 {len(done)} 步、擋下 {sum(r["result"] == "blocked" for r in rows)}、'
          f'沒做 {sum(r["result"] == "failed" for r in rows)}、略過 {sum(r["result"] == "skipped" for r in rows)}'
          '（誤差：有實機就是馬達讀數，不然是模擬）')
    for kind, name in (('sim', '模擬（MoveIt 關節角）'), ('real', '實機馬達跟隨（馬達讀數 vs 模擬的目標，不含機構下垂）')):
        e = [r[f'{kind}_err_cm'] for r in done if r.get(f'{kind}_err_cm') not in (None, '')]
        a = [r[f'{kind}_rot_err_deg'] for r in done if r.get(f'{kind}_rot_err_deg') not in (None, '')]
        if e:
            print(f'   {name}：掌心位置誤差 平均 {np.mean(e) * 10:.1f} mm、最大 {np.max(e) * 10:.1f} mm；'
                  f'姿態誤差 平均 {np.mean(a):.2f}°、最大 {np.max(a):.2f}°（{len(e)} 步）')


def first_line(res):
    return (res.get('note') or '').split('\n')[0] + (f'［{res["reason"]}］' if res.get('reason') else '')


def go_ready(t, cfg, arm):
    """這隻手到 ready：任務管理開著就照錄好的路線，沒開用 /arm_move。回傳 (ok, 說明)。"""
    r = t.task_cmd({'cmd': 'ready', 'arm': arm})
    if r is not None:
        return bool(r.get('ok')), f'任務管理：{r.get("message", "")}'
    _, approach, pos = gp.ready_targets(cfg)
    res = t.move_meas(pos[arm], approach, arm)
    return bool(res.get('ok')), first_line(res)


def go_stand(t):
    r = t.task_cmd({'cmd': 'stand'})
    if r is not None:
        return bool(r.get('ok')), f'任務管理：{r.get("message", "")}'
    res = t.stand()
    return bool(res.get('ok')), first_line(res)


def write_result(d, rows):
    path = os.path.join(d, 'result.csv')
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=RESULT_COLS, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    return path


def step_line(m, settled, following):
    """一步到了之後印的那一行（錄影用）。"""
    line = '✓ 到了'
    if m.get('bottle_gap_cm') is not None:
        line += f'｜手離瓶子 {m["bottle_gap_cm"]:.1f} cm'
    if 'sim_err_cm' in m:
        line += f'｜掌心誤差 {m["sim_err_cm"] * 10:.1f} mm、姿態 {m["sim_rot_err_deg"]:.2f}°'
    if 'real_err_cm' in m:
        line += f'｜實機馬達 {m["real_err_cm"] * 10:.1f} mm、{m["real_rot_err_deg"]:.2f}°'
    if following and not settled:
        line += '｜⚠ 實機 8 秒內沒停穩'
    return line


def save_poses(d, args, cfg, arm, joints, objs, c, size, follow, ready_q, steps, rows):
    """錄下來給 --replay 用（demo_replay.py）：每一步到了的關節角、途中的模擬路徑、場景、實機跟隨開關。"""
    by = {r['step']: r for r in rows}
    out = []
    for i, st in enumerate(steps):
        r = by.get(i + 1, {})
        ok = r.get('result') == 'ok'
        out.append({'label': st.label, 'part': st.part, 'test': bool(st.test), 'result': r.get('result', ''),
                    'reason': r.get('reason', ''), 'bridge': r.get('bridge', ''), 'note': st.note,
                    'fingers': st.fingers, 'pos': st.pos, 'R': st.R,
                    'q': st.extra.get('q_done') if ok else None, 'path': st.extra.get('path') if ok else None})
    return dr.save(d, {'arm': arm, 'joints': list(joints), 'bottle': args.bottle, 'center': c, 'size': size,
                       'at_table': table_xyz(c, cfg), 'gap': args.gap, 'follow': bool(follow), 'speed': args.speed,
                       'objs': objs, 'ready_q': ready_q, 'steps': out,
                       'created': datetime.datetime.now().isoformat(timespec='seconds')})


def execute(t, pl, steps, ik, joints, conv, z_comp, cfg, args, d, rows, gauge=None):
    """照規劃一步一步做，每一步的結果加進 rows（中途停下時，做過的也留著）。終端機每一步一行（錄影用）。
    測試步（st.test）一定送：moveit_bridge 擋下（到不了、會撞到）= 預期中的結果，記「擋下」、接著做下一步。
    實機跟隨中途斷掉（FollowLost）：這一步記「沒做（實機跟隨中斷）」，往外丟。"""
    t.set_arm(args.arm)
    last = None                     # 最後一個到了的那一步（q = 實際的關節角）
    need_range = any(s.yaw is not None for s in steps)
    i = 0
    try:
        while i < len(steps):       # 起點重新規劃時 steps 的長度可能會變
            st = steps[i]
            head = f'【{i + 1:2d}/{len(steps)}】{st.label} …… '
            if not st.ok and not st.test:
                print(f'{head}略過（{st.note}）')
                rows.append(row_of(i, st, 'skipped', '', None, cfg))
                i += 1
                continue
            t.check_follow()
            print(head, end='', flush=True)
            t.path, t.recording = [], True          # 錄途中的模擬路徑（重播時直接轉會碰到的那段照這個走）
            try:
                res = t.move_pose(st.pos, st.R)
            except BaseException:
                t.recording = False
                raise
            t.check_follow()
            if not res.get('ok'):
                t.recording = False
                reason, txt = res.get('reason') or '', why_text(res)
                if reason in SKIP_REASONS:
                    rows.append(row_of(i, st, 'blocked' if st.test else 'failed', reason, None, cfg, bridge=txt))
                    print(f'✗ {txt} → moveit_bridge 擋下，手臂沒動' + ('' if st.test else '（這一步沒做）'))
                    if last is None:
                        raise RuntimeError('第一步就到不了，停止')
                    pl.plan_rest(steps, i + 1, last)    # 手臂沒動：剩下的從最後到的那一步接著規劃
                    i += 1
                    continue
                rows.append(row_of(i, st, 'failed', reason, None, cfg, bridge=txt))
                print(f'✗ {txt}')
                print(f'      {short(res)}')
                raise RuntimeError(f'第 {i + 1} 步 {txt}，停在原地（確認沒問題後在網頁按 Ready pose／Stand 回去）')
            t_res = t.result[0] if t.result else time.monotonic()
            try:
                settled = t.wait_settled(t_res)
            finally:
                t.recording = False
            st.extra['path'] = dr.sim_path(t.path, joints)
            t.spin_for(args.hold)
            t.check_follow()
            m = measure(t, st, ik, joints, conv, z_comp, t_res, cfg, gauge)
            rows.append(row_of(i, st, 'ok', '', m, cfg, settled='' if not t.following() else int(settled)))
            print(step_line(m, settled, t.following()))
            q = m.get('q')
            st.extra['q_done'] = q
            if q is not None:
                if i == 0:                              # 起點：moveit_bridge 挑的姿勢可能跟規劃的不一樣，從實際的姿勢重新規劃
                    same = st.q is not None and dp._jump(q, st.q) <= REPLAN_DEG and not need_range
                    pl.at_start(st, q, need_range=need_range)
                    if not same:                        # 跟規劃的一樣就不用重算（AGX 上要好幾十秒，手臂會停在起點等）
                        pl.fit_yaw(steps)               # 幅度可能變了：原地轉手掌那段重排（後面的步數會變）
                        pl.plan_rest(steps, 1, st)
                        write_plan(os.path.join(d, 'plan_start.csv'), steps, cfg)
                elif dp._jump(q, st.q) > REPLAN_DEG:
                    st.q = q
                    pl.plan_rest(steps, i + 1, st)
            last = st
            i += 1
    except FollowLost:
        if i < len(steps) and (not rows or rows[-1]['step'] != i + 1):
            rows.append(row_of(i, steps[i], 'failed', 'follow_lost', None, cfg, bridge='實機跟隨中斷'))
        raise


def replay(t, args, cfg, rows, ctx):
    """--replay：照上次錄好的關節角做（demo_replay.py），不經 moveit_bridge、不用規劃。
    每一步從上一個姿勢每顆馬達直接轉過去；會碰到（本機 URDF 外形含實機位置、MoveIt 碰撞檢查）的那段照上次錄到的路線走。
    上次被擋下的測試步不送，照上次的結果印。ctx 回傳 moved / done / d / scene 給 main 收尾。"""
    src = os.path.expanduser(args.replay)
    data = dr.load(src)
    arm, joints = data['arm'], list(data['joints'])
    follow = t.following()
    if follow != bool(data['follow']):
        raise RuntimeError(f'錄的時候實機跟隨{"開" if data["follow"] else "關"}著、現在{"開" if follow else "關"}著：'
                           '實機的高度補償已經算在錄好的關節角裡，要一樣才能重播'
                           f'（先把實機跟隨{"打開" if data["follow"] else "關掉"}）')
    ik, joints2 = load_kinematics(arm)
    if list(joints2) != joints:
        raise RuntimeError(f'錄的關節名跟現在的 URDF 不一樣：{joints} vs {list(joints2)}')
    objs = data['objs']
    t.scene(objs)
    ctx['scene'] = True
    name = "左" if arm == "left" else "右"
    x = data['at_table']
    print(f'重播 {src}（{data.get("created", "")} 錄的）：{name}手，藥瓶 {data["bottle"]} 要擺在 '
          f'桌面 右 {x[0]:.1f}、前 {x[1]:.1f}（跟錄的時候一樣），手離瓶子 {data["gap"]:g} cm')

    shape = load_shape()
    z_comp = make_z_comp(cfg, follow)
    clr = make_clearance(shape, joints, arm, objs, ik, z_comp)
    group = 'right_arm' if arm == 'right' else 'left_arm'

    def check(P):
        if clr is not None:
            for k, q in enumerate(P):
                c = clr(q)
                if c < 0:
                    return f'第 {k}/{len(P) - 1} 點手會碰到東西（離加厚後的物體 {c * 100:.1f} cm）'
        return t.moveit_hits(joints, P, group)

    steps = data['steps']
    q0 = data.get('ready_q')
    if q0 is None:
        raise RuntimeError('錄的時候沒記到 ready 的關節角，不能重播')
    t0 = time.monotonic()
    segs = dr.segments(steps, q0, check)
    total = 0.0
    print(f'\n== 重播：{len(steps)} 步（檢查路線花了 {time.monotonic() - t0:.1f} 秒）')
    for i, (s_, g) in enumerate(zip(steps, segs)):
        if g is None:
            how = f'不動：上次{RESULT_TEXT.get(s_["result"], s_["result"] or "沒做")}' + \
                  (f'（{s_["bridge"]}）' if s_.get('bridge') else '')
        elif g['how'] == 'direct':
            sec = float(ready_path.time_path(g['P'], args.speed)[0][-1])
            total += sec + args.hold
            how = f'直接轉過去（{sec:.1f} 秒）'
        elif g['how'] == 'recorded':
            sec = float(ready_path.time_path(g['P'], args.speed)[0][-1])
            total += sec + args.hold
            how = f'照上次的路線（{sec:.1f} 秒；直接轉不行：{g["why"]}）'
        else:
            how = f'✗ 走不過去：{g["why"]}'
        print(f'{i + 1:2d}  {s_["label"]:30s} {how}')
    bad = [i + 1 for i, g in enumerate(segs) if g is not None and g['how'] is None]
    if bad:
        raise RuntimeError(f'第 {"、".join(map(str, bad))} 步走不過去，不重播（重新正常跑一次 pose_demo 錄新的）')
    print(f'預計 {total:.0f} 秒（加上回 ready、回站姿）')
    d = os.path.expanduser(os.path.join(args.out, datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                                        + f'_pose_replay_{arm}'))
    ctx['d'] = d
    if args.plan_only:
        return
    os.makedirs(d, exist_ok=True)
    if not args.yes:
        input(f'\n{"實機跟隨開著，手臂會真的動。" if follow else ""}按 Enter 開始（Ctrl+C 取消）…')
    t.spin_for(0.3)
    if t.following() != follow:
        raise RuntimeError('等 Enter 的時候實機跟隨被切換了，跟錄的時候不一樣：停止，手臂不動')

    try:
        conv = real_ticks_converter(os.path.join(get_package_prefix('armpackage'), 'lib', 'armpackage'))
    except PackageNotFoundError:
        conv = None
    size = np.asarray(data['size'], float)
    gauge = make_gauge(shape, joints, arm, gp.bottle_obstacle(data['bottle'], data['center'], size, 0.0, oid='target'))
    t.open_hand(arm)
    t.pub_speed.publish(Float64(data=args.speed))
    ctx['moved'] = True
    t.watch_follow()
    ok, txt = go_ready(t, cfg, arm)
    t.check_follow()
    print(f'到 ready：{"OK" if ok else "失敗"} {txt}')
    if not ok:
        raise RuntimeError('到不了 ready，停止')
    t.pub_speed.publish(Float64(data=args.speed))           # 任務管理走完 ready 會換回它自己的速度
    print(f'\n{BAR} 成果展示（重播）：{name}手圍著藥瓶 {data["bottle"]}（桌面 右 {x[0]:.1f}、前 {x[1]:.1f}），'
          f'手離瓶子 {data["gap"]:g} cm，速度 {args.speed:g}{"，實機" if follow else "，只動模擬"} {BAR}')
    for i, (s_, g) in enumerate(zip(steps, segs)):
        R = None if s_['R'] is None else np.asarray(s_['R'], float)
        st = dp.Step(part=s_['part'], label=s_['label'], pos=np.asarray(s_['pos'], float),
                     axis=np.zeros(3) if R is None else R[:, 0], test=s_['test'])
        st.R = R
        st.fingers, st.note = s_.get('fingers'), s_.get('note') or ''
        head = f'【{i + 1:2d}/{len(steps)}】{st.label} …… '
        if g is None:
            if s_['test'] and s_['result'] == 'blocked':
                txt = s_.get('bridge') or REASON_TEXT.get(s_.get('reason'), '到不了')
                print(f'{head}✗ {txt} → moveit_bridge 擋下，手臂沒動（上次的結果）')
                rows.append(row_of(i, st, 'blocked', s_.get('reason'), None, cfg, bridge=txt))
            else:
                print(f'{head}略過（上次沒做）')
                rows.append(row_of(i, st, 'skipped', s_.get('reason'), None, cfg, bridge=s_.get('bridge', '')))
            continue
        t.spin_for(0.05)
        q_now = np.array([t.q_sim.get(j, np.nan) for j in joints])
        if np.isnan(q_now).any():
            raise RuntimeError('收不到 MoveIt 的 /joint_states')
        P, extra = dr.start_from(g['P'], q_now)
        if extra is not None:
            why = check(extra)
            if why:
                raise RuntimeError(f'第 {i + 1} 步：手臂現在的姿勢跟錄的時候差 {dr.deg(g["P"][0], q_now):.0f}°，'
                                   f'接過去會碰到：{why}')
        t.check_follow()
        print(head, end='', flush=True)
        ok, why = t.run_traj(joints, P, args.speed)
        if not ok:
            print(f'✗ {why}')
            rows.append(row_of(i, st, 'failed', 'exec_failed', None, cfg, bridge=why))
            raise RuntimeError(f'第 {i + 1} 步 {why}，停在原地（確認沒問題後在網頁按 Ready pose／Stand 回去）')
        t_res = time.monotonic()
        settled = t.wait_settled(t_res)
        t.spin_for(args.hold)
        t.check_follow()
        m = measure(t, st, ik, joints, conv, z_comp, t_res, cfg, gauge)
        rows.append(row_of(i, st, 'ok', '', m, cfg, settled='' if not t.following() else int(settled)))
        print(step_line(m, settled, t.following()))
    summary(rows)
    print(f'結果存在 {write_result(d, rows)}')
    ctx['done'] = True
    if args.end in ('ready', 'stand'):
        ok, txt = go_ready(t, cfg, arm)
        print(f'回 ready：{"OK" if ok else "失敗"} {txt}')
        if ok and args.end == 'stand':
            ok, txt = go_stand(t)
            print(f'回站姿：{"OK" if ok else "失敗"} {txt}')
        try:
            t.check_follow()
        except FollowLost as e:
            print(f'⚠ 回去的時候實機跟隨斷了：{e}')


def main():
    ap = argparse.ArgumentParser(description='成果展示：掌心到藥瓶正上方，原地換姿態，再繞著藥瓶轉（直接對 moveit_bridge）')
    ap.add_argument('--bottle', default='b1', help='哪一類藥瓶（b1～b4）。沒給 --at = 用影像找這一瓶')
    ap.add_argument('--at', type=float, nargs=2, metavar=('右', '前'),
                    help='不開影像：藥瓶在桌面座標 右、前（cm），尺寸照設定檔 dry_run.bottles 裡 --bottle 那一類')
    ap.add_argument('--size', type=float, nargs=2, metavar=('直徑', '高'), help='配合 --at：藥瓶直徑、高（cm）')
    ap.add_argument('--arm', choices=['auto', 'left', 'right'], default='auto', help='auto = 藥瓶在哪一邊用哪隻手')
    ap.add_argument('--parts', nargs='+', choices=['ring', 'sides', 'spin', 'orbit'], default=['ring'],
                    help='ring = 繞圈（預設：30°、45° 各繞 8 個方向，再往外、往內傾 60°）；sides = 只有左右環繞 + 手指朝上下測試；'
                         '舊的細版本：spin = 原地換姿態、orbit = 繞著藥瓶轉')
    ap.add_argument('--gap', type=float, default=2.0, help='ring / sides：手離藥瓶表面多遠（cm）')
    ap.add_argument('--tilts', type=float, nargs='+', default=[30.0, 45.0],
                    help='ring：每圈傾幾度（照順序）；sides：往左右傾幾度')
    ap.add_argument('--side-tilts', type=float, nargs='*', default=[60.0], help='ring：繞完再往外、往內各傾幾度')
    ap.add_argument('--finger-test', action='store_true', help='ring：最後加手指朝上／朝下的測試（多半到不了，預設不做）')
    ap.add_argument('--no-finger-test', action='store_true', help='sides：不做手指朝上／朝下的測試')
    ap.add_argument('--radius', type=float, default=15.0, help='spin / orbit：掌心離瓶頂中心多遠（cm）')
    ap.add_argument('--tilt', type=float, default=45.0, help='orbit：繞的時候掌心離垂直傾斜幾度')
    ap.add_argument('--spin-tilt', type=float, default=30.0, help='spin：原地傾斜幾度（0 = 不做原地傾斜）')
    ap.add_argument('--yaw', type=float, default=60.0, help='spin：原地轉手掌最多轉幾度（實際看手指轉得到的範圍）')
    ap.add_argument('--hold', type=float, default=1.0, help='每一步到了、停穩之後停幾秒')
    ap.add_argument('--speed', type=float, default=50.0, help='/arm_speed_cmd（Dynamixel 單位，實機最多 50）')
    ap.add_argument('--timeout', type=float, default=30.0, help='每個指令最多等幾秒')
    ap.add_argument('--end', choices=['stand', 'ready', 'stay'], default='stand',
                    help='做完：回站姿（預設，手舉太久馬達會過熱）/ 停在 ready / 停在最後一步')
    ap.add_argument('--plan-only', action='store_true', help='只規劃、印每一步，手臂不動')
    ap.add_argument('--yes', action='store_true', help='不用按 Enter 確認就開始')
    ap.add_argument('--real', action='store_true', help='實機跟隨開著也照樣執行（手臂會真的動）')
    ap.add_argument('--clear', action='store_true', help='只清掉上次留下的場景（藥瓶），手臂不動')
    ap.add_argument('--replay', metavar='資料夾',
                    help='照上次錄好的動作做（那次的輸出資料夾，裡面有 poses.json）：不用規劃、每一步直接轉過去；'
                         '藥瓶位置、手離瓶子多遠都照錄的時候，--at / --gap 不用給')
    ap.add_argument('--out', default='~/reach_test_out', help='結果存在這個資料夾底下')
    ap.add_argument('--config', default=os.path.join(get_package_share_directory('sp3_task_manager'),
                                                     'config', 'task_manager.yaml'))
    args = ap.parse_args()
    circ = args.parts[0] in ('ring', 'sides')      # 手離藥瓶 --gap 的版本
    if circ and len(args.parts) > 1:
        ap.error(f'--parts {args.parts[0]} 要單獨用（不能跟其他的一起）')
    if circ and args.gap < 1.0:
        ap.error('--gap 至少 1 cm（藥瓶在場景裡加厚 1 cm，再近 moveit_bridge 會判定撞到）')
    if not circ and args.radius < 10.0:
        ap.error('--radius 至少 10 cm（手掌本身厚 4.5 cm，太近會撞到藥瓶）')
    with open(args.config, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    try:    # Ctrl+C 只丟 KeyboardInterrupt、不讓 rclpy 先關掉 context，才送得出 /arm_stop
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    t = Demo(cfg, args)
    moved = done = scene_set = False
    rows, d, save_rec = [], None, None
    try:
        if args.clear:
            t.scene([])
            print(f'場景來源 {SOURCE} 清掉了')
            return
        if t.task_manager_busy():
            print('任務管理正在跑任務（/manip/status = BUSY），等它做完再跑')
            return
        if args.plan_only:
            end = time.monotonic() + 3.0
            while t.real is None and time.monotonic() < end:
                rclpy.spin_once(t, timeout_sec=0.05)
            print(f'實機跟隨：{"開著" if t.following() else "關著"}（規劃照這個算高度補償）')
        elif not t.check_controllers() or not t.check_real():
            return
        if args.replay:
            ctx = {}
            try:
                replay(t, args, cfg, rows, ctx)
            finally:
                moved, done, d, scene_set = (ctx.get('moved', False), ctx.get('done', False), ctx.get('d'),
                                             ctx.get('scene', False))
            return

        # 藥瓶在哪
        others = []
        if args.at:
            dims = (cfg.get('dry_run', {}).get('bottles', {}).get(args.bottle) or {}).get('size_m', [0.0829, 0.0923])
            d_m, h_m = (args.size[0] / 100, args.size[1] / 100) if args.size else (float(dims[0]), float(dims[-1]))
            c = gp.from_meas(gp.table_to_meas((args.at[0], args.at[1], h_m * 50.0), cfg['table'], cfg['frames']),
                             cfg['frames'])
            size = np.array([d_m, d_m, h_m])
            print(f'藥瓶（手動給的）：桌面座標 右 {args.at[0]:g}、前 {args.at[1]:g}，直徑 {d_m * 100:.1f}、高 {h_m * 100:.1f} cm')
        else:
            c, size, others, jit = detect(t, args.bottle)
            x = table_xyz(c, cfg)
            print(f'影像：{args.bottle} 瓶身中心 桌面座標 右 {x[0]:.1f}、前 {x[1]:.1f}、上 {x[2]:.1f} cm'
                  f'（抖動 {jit[0]:.0f}/{jit[1]:.0f}/{jit[2]:.0f} mm），高 {size[2] * 100:.1f} cm')
            for o in others:
                print(f'   {o[0]} 也看到了，當障礙物')
        right = table_xyz(c, cfg)[0]
        if args.arm == 'auto':
            args.arm = 'right' if right >= 0 else 'left'
        arm = args.arm
        focus = c + [0.0, 0.0, float(size[2]) / 2]           # 瓶頂中心
        m = cfg['obstacles']['margin_cm']
        objs = table_with_scene(cfg, [bottle_box(c, size, m, oid=f'{SOURCE}_bottle_{args.bottle}')] +
                                [gp.bottle_obstacle(k, xo, so, m, oid=f'{SOURCE}_bottle_{k}') for k, xo, so in others])
        try:            # 先放場景：規劃失敗、--plan-only 時網頁（RobotViewer）也看得到藥瓶在哪
            t.scene(objs)
            scene_set = True
        except RuntimeError as e:
            if not args.plan_only:
                raise
            print(f'（{e}；只規劃，繼續）')

        # 規劃
        ik, joints = load_kinematics(arm)
        try:
            conv = real_ticks_converter(os.path.join(get_package_prefix('armpackage'), 'lib', 'armpackage'))
        except PackageNotFoundError:
            conv = None
        shape = load_shape()
        rng = np.random.default_rng(0)
        seeds = [np.zeros(7)] + [rng.uniform(ik.lower, ik.upper) for _ in range(30)]
        bottle_true = gp.bottle_obstacle(args.bottle, c, size, 0.0, oid='target')     # 不加厚：量手離瓶子多遠
        sides = None
        if circ:
            if shape is None:
                raise dp.PlanError('讀不到 URDF 的手部外形，算不了手離瓶子多遠')
            sides = {'bottle': bottle_true, 'hand': dp.hand_in_palm(shape, arm, joints), 'gap': args.gap / 100,
                     'tilts': tuple(args.tilts)}
            if args.parts[0] == 'ring':
                sides.update(side_tilts=tuple(args.side_tilts), finger_test=args.finger_test)
            else:
                sides.update(finger_test=not args.no_finger_test)
        name = "左" if arm == "left" else "右"
        what = f'手離瓶子 {args.gap:g} cm' if sides else f'掌心離瓶頂中心 {args.radius:g} cm'

        def make_plan(follow):
            """照實機跟隨開／關（moveit_bridge 補不補高度）規劃。回傳 (z_comp, planner, steps)。"""
            zc = make_z_comp(cfg, follow)
            t0 = time.monotonic()
            planner = dp.Planner(arm, ik, seeds, clearance=make_clearance(shape, joints, arm, objs, ik, zc),
                                 sim_pos=lambda p: np.asarray(p, float) + dp.UP * zc(p), yaw_max=args.yaw)
            seq = dp.sequence(arm, focus, args.radius / 100, parts=args.parts,
                              spin={'tilt': args.spin_tilt}, orbit={'tilt': args.tilt}, sides=sides)
            planner.plan(seq, ready_q(ik, cfg, arm, seeds))
            print_plan(seq, cfg, f'規劃（{name}手，{what}，實機跟隨{"開" if follow else "關"}著，'
                                 f'算了 {time.monotonic() - t0:.1f} 秒）')
            return zc, planner, seq

        planned_follow = t.following()
        z_comp, pl, steps = make_plan(planned_follow)
        gauge = make_gauge(shape, joints, arm, bottle_true)
        d = os.path.expanduser(os.path.join(args.out, datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                                            + f'_pose_demo_{arm}'))
        os.makedirs(d, exist_ok=True)
        write_plan(os.path.join(d, 'plan.csv'), steps, cfg)
        print(f'規劃存在 {d}/plan.csv')
        if args.plan_only:
            return
        if not args.yes:
            input(f'\n{"實機跟隨開著，手臂會真的動。" if t.following() else ""}按 Enter 開始（Ctrl+C 取消）…')
        t.spin_for(0.3)
        if t.following() != planned_follow:     # 等 Enter 的時候實機跟隨被打開／關掉了：moveit_bridge 補的高度不一樣
            if t.following() and not args.real:
                raise RuntimeError('實機跟隨在規劃之後被打開了，沒加 --real：停止，手臂不動')
            print(f'\n實機跟隨在規劃之後被{"打開" if t.following() else "關掉"}了（高度補償不一樣），重新規劃…')
            planned_follow = t.following()
            z_comp, pl, steps = make_plan(planned_follow)
            write_plan(os.path.join(d, 'plan.csv'), steps, cfg)

        # 執行
        t.scene(objs)
        t.open_hand(arm)
        t.pub_speed.publish(Float64(data=args.speed))
        moved = True
        t.watch_follow()
        ok, txt = go_ready(t, cfg, arm)
        t.check_follow()
        print(f'到 ready：{"OK" if ok else "失敗"} {txt}')
        if not ok:
            raise RuntimeError('到不了 ready，停止')
        t.spin_for(0.3)
        rq = [t.q_sim.get(j) for j in joints]

        def save_rec():
            return save_poses(d, args, cfg, arm, joints, objs, c, size, planned_follow,
                              None if None in rq else rq, steps, rows)
        x = table_xyz(c, cfg)
        print(f'\n{BAR} 成果展示：{name}手圍著藥瓶 {args.bottle}（桌面 右 {x[0]:.1f}、前 {x[1]:.1f}），{what}，'
              f'速度 {args.speed:g}{"，實機" if t.following() else "，只動模擬"} {BAR}')
        execute(t, pl, steps, ik, joints, conv, z_comp, cfg, args, d, rows, gauge)
        summary(rows)
        print(f'結果存在 {write_result(d, rows)}')
        print(f'錄好的動作存在 {save_rec()}（錄影時用 pose_demo --replay {d}：不用規劃、直接照做）')
        done = True                                 # 展示做完了；回去的路上實機跟隨斷掉只提醒
        if args.end in ('ready', 'stand'):
            ok, txt = go_ready(t, cfg, arm)
            print(f'回 ready：{"OK" if ok else "失敗"} {txt}')
            if ok and args.end == 'stand':
                ok, txt = go_stand(t)
                print(f'回站姿：{"OK" if ok else "失敗"} {txt}')
            try:
                t.check_follow()
            except FollowLost as e:
                print(f'⚠ 回去的時候實機跟隨斷了：{e}')
    except KeyboardInterrupt:
        if moved:
            t.pub_stop.publish(Bool(data=True))
            t.pub_test.publish(String(data=json.dumps({'cmd': 'cancel'})))     # 照錄好的路線走時是任務管理在動
            t.spin_for(0.3)
            print('\n中斷：已送 /arm_stop，手臂停在原地（確認沒問題後在網頁按 Ready pose／Stand 回去）')
        else:
            print('\n取消')
    except FollowLost as e:
        t.pub_stop.publish(Bool(data=True))         # 模擬也停，不要自己繼續動
        t.pub_test.publish(String(data=json.dumps({'cmd': 'cancel'})))
        t.spin_for(0.3)
        print(f'\n\n⚠ {e}' if isinstance(e, FollowTurnedOn) else f'\n\n⚠ 實機跟隨中斷：{e}')
        if not isinstance(e, FollowTurnedOn):
            print('   已停止（送了 /arm_stop），手臂不再動。肩膀洩力的話是馬達自己的保護（過載／過熱）把扭力關了，'
                  '電池電量低時常見：\n   關掉 nstc.launch.py 後跑 ros2 run armpackage self_check.py 看哪一顆記了錯誤、電壓多少；'
                  '觸發保護的馬達要重新上電。')
        else:
            print('   已停止（送了 /arm_stop），手臂不再動。')
    except (RuntimeError, dp.PlanError) as e:
        print(f'\n⚠ {e}')
    finally:
        if rows and not done:                       # 中途停下：做過的那幾步也存起來
            summary(rows)
            print(f'做到一半的結果存在 {write_result(d, rows)}')
            if save_rec is not None:
                try:
                    print(f'做到的那幾步也錄下來了：{save_rec()}')
                except Exception as e:      # noqa: BLE001  存不了不影響其他收尾
                    print(f'⚠ 錄的動作存不了：{e}')
        if moved:
            if done:
                try:
                    t.scene([])
                except RuntimeError as e:
                    print(f'⚠ {e}')
            else:
                print('場景（藥瓶）留著，手臂回去時才會避開它；不用了就 pose_demo --clear')
            t.pub_arm.publish(String(data='auto'))
            t.spin_for(0.3)
        elif scene_set:
            print('場景（藥瓶）留著，網頁看得到瓶子在哪；下次跑會換掉，不用了就 pose_demo --clear')
        t.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
