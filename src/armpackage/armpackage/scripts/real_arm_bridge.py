#!/usr/bin/env python3
"""
MoveIt 的手臂角度 → 實機。driver_node（motor_control）吃 /joint_commands：name = 馬達 ID、position = 刻度、velocity = Profile Velocity

MoveIt 執行軌跡時，假硬體發的 /joint_states 會照軌跡時間一路更新；這支把它換成刻度（arm_config.rad_to_ticks）送給 driver。
driver 也發 /joint_states（name = 馬達 ID、單位刻度），這支拿來當實機目前的位置。

    ros2 launch armpackage sim.launch.py rviz:=false rosbridge:=false real:=true
    開始跟：網頁 IK 面板按「實機跟隨」，或 ros2 topic pub --once /arm_real_enable std_msgs/msg/Bool "{data: true}"
    停止：  再按一次，或把 true 改成 false 再送一次
    狀態：  /arm_real_status（JSON {state, msg, warn}），網頁 IK 面板會顯示

開啟時實機不動，讓模擬去對齊實機：
- 實機目前的刻度 → 角度，直接叫模擬的手臂控制器（ros2_control 假硬體）移過去，MoveIt 之後就從實機的姿勢開始規劃
- 模擬到了才開始跟。實機的站姿（stand.ini，1~14 全 2048）就是 MoveIt 的站姿，從那裡開最單純
- 刻度換算用 arm_config（含 stand.ini 的修正）；讀不到 stand.ini 就不開

安全設計：
- 開起來是關的，收到 /arm_real_enable 才送
- 設定檔檢查（同 self_check.py config，啟動時自動跑）要通過，不然不開（刻度換算對不對）
- 開機自我檢查（self_check.py 讀馬達的那部分）：2026-11-10 起只提醒、不擋。沒做、沒過、超過 12 小時都照樣開，
  終端機和網頁狀態會附提醒
- 只送 arm_config 裡 calibrated 的手（2026-10-28 起雙手：右手 8~14、左手 1~7）
- 實機姿勢超出 MoveIt 的關節範圍（joint_limits.yaml、URDF）或實機限制就不開；MoveIt 正在動也不開
- 每次送之前做跟網頁一樣的實機限制檢查（刻度 0~4095、離站姿 180°），不過就停
- 一定帶速度（driver 收到 0 會變全速）：網頁的速度，最多 MAX_SPEED
- MoveIt 停住後實機沒有往目標走（扭力沒開、卡住、被別的程式移動）就停
停 = 不再送新的指令，馬達會停在最後送出的目標

到位紀錄（2026-11-10 加，位置測試用）：MoveIt 停住、實機停穩後，對剛剛動過的手各記一行到 SETTLE_LOG
（預設 ~/.ros/arm_settle_log.csv）：模擬掌心、「照馬達讀數算的掌心」（馬達刻度 → 角度 → 同一個 URDF 的 FK）、
兩者的差、每軸差幾度。狀態列和終端機會印「紀錄 #N」，捲尺量到的掌心記在同一個 #N 就對得起來。
  照馬達讀數算的掌心也偏低 → 馬達沒轉到目標（重力、扭力、電量）
  跟模擬一樣、捲尺量到的卻偏低 → 模型的問題（零點、URDF、掌心點的定義）

手掌（2026-11-12 起）：這支不再控制手掌（以前 moveit_bridge 抓取時會經過這裡叫 handpackage 夾持 / 張開）。
  手臂只負責把掌心送到位置 + 姿態，手掌用 handpackage（網頁或 /hand/cmd）另外控制。
"""
import csv
import json
import os
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from arm_config import ARMS, LABELS, STAND_INI, STAND_REAL, TOOL_OFFSET, TPR, rad_to_ticks, ticks_to_rad, to_meas
from kinematics import ChainFK, chain_limits
from moveit_bridge import check
from self_check import config_checks, last_check

RATE = 50.0           # 送 /joint_commands 的頻率（Hz），跟 driver 的控制迴圈一樣
MAX_SPEED = 50        # 速度上限（Dynamixel 單位，1 = 0.229 rpm；50 ≈ 11 rpm）。網頁速度比這個大就用這個
MIN_SPEED = 5         # 速度下限（0 在 driver 是全速，不能送）
DEFAULT_SPEED = 30    # 網頁還沒送過速度時用這個
SYNC_TIME = 1.5       # 模擬的手臂用幾秒移到實機的姿勢（只動模擬）
SYNC_TIMEOUT = 5.0    # 模擬幾秒還沒到就放棄
SYNC_TOL = 0.01       # 模擬跟目標差幾 rad 以內算到了（≈0.6°）
LIMIT_TOL = 0.05      # 實機超出 MoveIt 關節範圍這麼多（rad，≈3°）以內就夾回範圍內，再多就不開
STILL = 0.3           # MoveIt 的手臂幾秒沒變算停住
SETTLE_TIME = 0.5     # MoveIt 停住後，實機幾秒沒再更接近目標算停穩（停穩後回報跟目標差多少）
FOLLOW_TOL = 57       # 停住後實機跟目標差幾個刻度以內算正常（≈5°）
STUCK_TIME = 2.0      # 停住後實機幾秒沒有更接近目標就停
REAL_TIMEOUT = 1.0    # 幾秒沒收到 driver 的馬達位置就停
SETTLE_LOG = os.environ.get("SETTLE_LOG", os.path.expanduser("~/.ros/arm_settle_log.csv"))   # 到位紀錄（見最上面）
SETTLE_COLS = ["n", "time", "arm", "sim_front", "sim_right", "sim_height", "enc_front", "enc_right", "enc_height",
               "d_enc_front", "d_enc_right", "d_enc_height", "max_tick_err", "max_tick_motor",
               "dq1", "dq2", "dq3", "dq4", "dq5", "dq6", "dq7", "tick_err", "speed",
               "q1", "q2", "q3", "q4", "q5", "q6", "q7"]   # q = 模擬（MoveIt）每軸角度（度），之後可以拿來擬合每軸零點


class RealArmBridge(Node):
    def __init__(self):
        super().__init__("real_arm_bridge")
        self.arms = [a for a in ARMS if ARMS[a]["calibrated"]]
        self.motors = [m for a in self.arms for m in ARMS[a]["motors"]]
        self.arm_joints = {j for a in self.arms for j in ARMS[a]["joints"]}
        # MoveIt 規劃用的關節範圍（URDF ∩ joint_limits.yaml），實機姿勢要在裡面 MoveIt 才能從那裡開始規劃
        urdf = os.path.join(get_package_share_directory("seven_axis_arm_urdf"), "urdf", "seven_axis_arm_urdf.urdf")
        moveit_limits = os.path.join(get_package_share_directory("seven_axis_arm_moveit_config"), "config", "joint_limits.yaml")
        over = {k: (v["min_position"], v["max_position"]) for k, v in yaml.safe_load(open(moveit_limits))["joint_limits"].items()
                if v.get("has_position_limits")}
        self.limits = {a: chain_limits(urdf, ARMS[a]["joints"], over) for a in self.arms}
        self.fk = {a: ChainFK(urdf, "base_link", ARMS[a]["tip"], TOOL_OFFSET) for a in self.arms}   # 掌心 FK（同 moveit_bridge）
        self.moved = set()             # 上次到位紀錄之後，目標有變的手
        self.n_log = self._log_count()  # 到位紀錄的編號（接著檔案裡的往下編）
        # 模擬的手臂控制器：開啟時直接叫它移到實機的姿勢
        self.sim = {a: ActionClient(self, FollowJointTrajectory, f"/{ARMS[a]['group']}_controller/follow_joint_trajectory")
                    for a in self.arms}
        self.q = {}                    # MoveIt：關節名稱 → 弧度
        self.q_t = 0.0                 # MoveIt 的手臂上次在動的時間
        self.real = {}                 # driver：馬達 ID → 刻度
        self.real_t = 0.0              # 上次收到 driver 馬達位置的時間
        self.speed = DEFAULT_SPEED     # 網頁的速度（/arm_speed_cmd）
        self.state = "off"             # off：不送 / sync：模擬移到實機的姿勢 / follow：跟著 MoveIt
        self.sync_q, self.sync_t = {}, 0.0     # 模擬要移到的姿勢（每隻手 7 軸）、什麼時候開始
        self.sent = None               # 上次送出去的 {馬達 ID: 刻度}
        self.changed = 0.0             # 目標上次改變的時間
        self.best, self.best_t = None, 0.0     # MoveIt 停住後，實機離目標最近的距離、什麼時候
        self.settle = None             # 到位回報：None 還沒開始 / (誤差, 時間) 等實機停穩 / True 這個目標報過了
        self.status = {}
        self.cmd_pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.status_pub = self.create_publisher(String, "/arm_real_status", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_states_cb, 50)
        self.create_subscription(Bool, "/arm_real_enable", self._enable_cb, 10)
        self.create_subscription(Float64, "/arm_speed_cmd", lambda m: setattr(self, "speed", m.data), 10)
        self.create_timer(1.0 / RATE, self._tick)
        self.create_timer(1.0, self._publish_status)          # 每秒補送狀態：網頁晚連上也看得到
        skipped = [ARMS[a]["label"] for a in ARMS if a not in self.arms]
        self.who = "、".join(ARMS[a]["label"] for a in self.arms)
        self.skip_note = f"；{'、'.join(skipped)}還沒校正，只動模擬" if skipped else ""
        if STAND_REAL is None:
            self.get_logger().warn(f"讀不到 {STAND_INI}（刻度換算要用 stand.ini 的站姿），實機跟隨不會開")
        else:
            self.get_logger().info(f"實機站姿刻度（{STAND_INI}）：" +
                                   " ".join(f"{m}:{STAND_REAL[m]}" for m in self.motors))
        # 設定檔檢查（同 self_check.py config）：有不通過的，實機跟隨不會開
        self.cfg_fail = [msg for level, msg in config_checks() if level == "fail"]
        self.check_note = ""       # 開機自我檢查沒過時，附在狀態後面的提醒（不擋）
        for msg in self.cfg_fail:
            self.get_logger().error(f"設定檢查沒過：{msg}")
        self._say(f"關閉。按「實機跟隨」開始（實機：{self.who}，馬達 {self.motors}{self.skip_note}）。{last_check()[1]}")

    def _publish_status(self):
        # 網頁只看 state / msg / warn。另外附給任務管理（sp3_task_manager）用的：
        #   settled：跟隨中、MoveIt 停住、實機也停穩了（同「實機停穩」那則訊息的條件；目標沒變就一直是 true）
        #   stamp：發布時間
        d = dict(self.status, settled=self.state == "follow" and self.settle is True, stamp=time.time())
        self.status_pub.publish(String(data=json.dumps(d, ensure_ascii=False)))

    def _say(self, msg, warn=False):
        """狀態 → 終端機 + 網頁（/arm_real_status）"""
        if warn:               # rclpy 同一行呼叫 logger 不能換嚴重度，所以分兩行
            self.get_logger().warn(msg)
        else:
            self.get_logger().info(msg)
        self.status = {"state": self.state, "msg": msg, "warn": warn}
        self._publish_status()

    def _joint_states_cb(self, msg):
        """同一個 topic 有兩種：driver 的（name = 馬達 ID、刻度）和 MoveIt 的（URDF 關節名、弧度）"""
        from_driver = False
        for name, pos in zip(msg.name, msg.position):
            if name.isdigit():
                self.real[int(name)] = pos
                from_driver = True
            else:
                if name in self.arm_joints and abs(self.q.get(name, pos) - pos) > 1e-4:
                    self.q_t = time.time()
                self.q[name] = pos
        if from_driver:
            self.real_t = time.time()

    def _target(self):
        """MoveIt 目前姿勢 → ({馬達 ID: 刻度}, 實機限制的警告)；還沒收到 MoveIt 回傳 (None, None)"""
        target, warn = {}, []
        for a in self.arms:
            if not all(j in self.q for j in ARMS[a]["joints"]):
                return None, None
            q = np.array([self.q[j] for j in ARMS[a]["joints"]])
            warn += check(a, q)
            target.update(rad_to_ticks(a, q))
        return target, warn

    def _speed(self):
        return int(np.clip(self.speed, MIN_SPEED, MAX_SPEED))

    # ---------- 到位紀錄：分出「馬達沒轉到」還是「模型跟實機不一樣」----------
    def _diag(self, arm, target):
        """這隻手：模擬掌心、照馬達讀數算的掌心（實機量測座標 cm）、每軸差多少"""
        ms = ARMS[arm]["motors"]
        q_sim = np.array([self.q[j] for j in ARMS[arm]["joints"]])
        q_enc = ticks_to_rad(arm, [self.real[m] for m in ms])
        p_sim = np.array(to_meas(self.fk[arm](q_sim)[:3, 3]))
        p_enc = np.array(to_meas(self.fk[arm](q_enc)[:3, 3]))
        tick_err = {m: self.real[m] - target[m] for m in ms}          # 實機 − 目標（刻度）
        worst = max(ms, key=lambda m: abs(tick_err[m]))
        d = p_enc - p_sim
        text = (f"{ARMS[arm]['label']}最多差 {abs(tick_err[worst]):.0f} 刻度（≈{np.degrees(abs(tick_err[worst]) / TPR):.1f}°，"
                f"{worst} 號），照馬達讀數算的掌心比模擬 前 {d[0]:+.1f}、右 {d[1]:+.1f}、高 {d[2]:+.1f} cm")
        return {"arm": arm, "text": text, "p_sim": p_sim, "p_enc": p_enc, "d": d, "worst": worst,
                "max_tick": abs(tick_err[worst]), "dq": np.degrees(q_enc - q_sim), "tick_err": tick_err,
                "q": np.degrees(q_sim)}

    def _log_count(self):
        try:
            with open(SETTLE_LOG, encoding="utf-8") as f:
                return max(0, sum(1 for _ in f) - 1)
        except OSError:
            return 0

    def _log(self, diags):
        """每隻手一行到 SETTLE_LOG；回傳這次的編號（寫不進去回傳 None，只警告、不停）"""
        self.n_log += 1
        rows = [[self.n_log, time.strftime("%Y-%m-%d %H:%M:%S"), g["arm"], *np.round(g["p_sim"], 1),
                 *np.round(g["p_enc"], 1), *np.round(g["d"], 1), f"{g['max_tick']:.0f}", g["worst"],
                 *np.round(g["dq"], 2), " ".join(f"{m}:{e:+.0f}" for m, e in g["tick_err"].items()), self._speed(),
                 *np.round(g["q"], 2)]
                for g in diags]
        try:
            new = not os.path.exists(SETTLE_LOG)
            os.makedirs(os.path.dirname(SETTLE_LOG) or ".", exist_ok=True)
            with open(SETTLE_LOG, "a", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                if new:
                    w.writerow(SETTLE_COLS)
                w.writerows(rows)
        except OSError as e:
            self.get_logger().warn(f"到位紀錄寫不進 {SETTLE_LOG}：{e}")
            return None
        return self.n_log

    def _send(self, target, speed):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [str(m) for m in target]
        msg.position = [float(t) for t in target.values()]
        msg.velocity = [float(speed)] * len(target)
        self.cmd_pub.publish(msg)
        self.sent = target

    def _stop(self, why):
        self.state, self.sent, self.best = "off", None, None
        self._say(f"停了：{why}（不再送新的指令，馬達停在最後送出的目標）", warn=True)

    def _refuse(self, why):
        self._say(f"沒開：{why}", warn=True)

    def _enable_cb(self, msg):
        if not msg.data:
            if self.state != "off":
                self.state, self.sent, self.best = "off", None, None
                self._say("關閉（不再送新的指令，馬達停在最後送出的目標）")
            return
        if self.state != "off":
            return
        if self.cfg_fail:
            return self._refuse("設定檢查沒過：" + "；".join(self.cfg_fail))
        ok, note = last_check()    # 2026-11-10 起只提醒、不擋（使用者決定：測試階段自我檢查當工具用）
        self.check_note = "" if ok else f"。提醒：{note}（自我檢查只提醒，不擋）"
        if not ok:
            self.get_logger().warn(f"提醒：{note}。自我檢查只提醒、不擋實機跟隨")
        if STAND_REAL is None:
            return self._refuse(f"讀不到 stand.ini（{STAND_INI}），刻度換算不確定")
        if self._target()[0] is None:
            return self._refuse("還沒收到 MoveIt 的 /joint_states（MoveIt 有開嗎？）")
        missing = [m for m in self.motors if m not in self.real]
        if time.time() - self.real_t > REAL_TIMEOUT or missing:
            return self._refuse("收不到 driver 的馬達位置（driver_node 有開嗎？" +
                                (f"沒有 ID {missing}" if missing else "超過 1 秒沒更新") + "）")
        if time.time() - self.q_t < STILL:
            return self._refuse("MoveIt 正在動，等它停了再開")
        goal_q = {}
        for a in self.arms:
            ticks = [self.real[m] for m in ARMS[a]["motors"]]
            q = ticks_to_rad(a, ticks)
            lo, hi = self.limits[a]
            out = [f"{ARMS[a]['label']}第 {i + 1} 軸{LABELS[i]}（{m} 號 刻度 {t:.0f}）= {np.degrees(q[i]):.0f}°，"
                   f"MoveIt 範圍 {np.degrees(lo[i]):.0f}~{np.degrees(hi[i]):.0f}°"
                   for i, (m, t) in enumerate(zip(ARMS[a]["motors"], ticks))
                   if q[i] < lo[i] - LIMIT_TOL or q[i] > hi[i] + LIMIT_TOL]
            if out:
                return self._refuse("實機姿勢超出 MoveIt 的關節範圍，先把實機轉回範圍內：" + "；".join(out))
            q = np.clip(q, lo + 1e-4, hi - 1e-4)              # 差一點點的夾回範圍內（MoveIt 起點不能超出範圍）
            warn = check(a, q)
            if warn:
                return self._refuse("實機目前的姿勢不符合實機限制：" + "；".join(warn))
            if not self.sim[a].server_is_ready():
                return self._refuse(f"找不到模擬的控制器 {ARMS[a]['group']}_controller（MoveIt 有開嗎？）")
            goal_q[a] = q
        self.state, self.sync_q, self.sync_t = "sync", goal_q, time.time()
        for a, q in goal_q.items():
            pt = JointTrajectoryPoint(positions=[float(v) for v in q], velocities=[0.0] * len(q),
                                      time_from_start=Duration(sec=int(SYNC_TIME), nanosec=int(SYNC_TIME % 1 * 1e9)))
            goal = FollowJointTrajectory.Goal(trajectory=JointTrajectory(joint_names=ARMS[a]["joints"], points=[pt]))
            self.sim[a].send_goal_async(goal).add_done_callback(lambda f, a=a: self._sync_accepted(a, f))
        self._say("對齊中：模擬的手臂移到實機目前的姿勢（實機不動）" + self.check_note)

    def _sync_accepted(self, arm, fut):
        if self.state == "sync" and not fut.result().accepted:
            self._stop(f"模擬的控制器 {ARMS[arm]['group']}_controller 不接受對齊的指令")

    def _tick(self):
        if self.state == "off":
            return
        now = time.time()
        if now - self.real_t > REAL_TIMEOUT:
            return self._stop("超過 1 秒沒收到 driver 的馬達位置")
        if self.state == "sync":
            err = max(abs(self.q[j] - v) for a, q in self.sync_q.items() for j, v in zip(ARMS[a]["joints"], q))
            if err <= SYNC_TOL:
                self.state, self.sent, self.best, self.changed = "follow", None, None, now
                self._say(f"跟隨中：網頁的動作{self.who}實機會跟著做，速度上限 {MAX_SPEED}{self.skip_note}{self.check_note}")
            elif now - self.sync_t > SYNC_TIMEOUT:
                self._stop(f"模擬 {SYNC_TIMEOUT:.0f} 秒還沒移到實機的姿勢（還差 {np.degrees(err):.1f}°）")
            return
        target, warn = self._target()
        if target is None:
            return
        if warn:
            return self._stop("MoveIt 的姿勢實機做不到：" + "；".join(warn))
        if target != self.sent:
            self.moved |= {a for a in self.arms
                           if self.sent is None or any(target[m] != self.sent.get(m) for m in ARMS[a]["motors"])}
            self._send(target, self._speed())
            self.changed, self.settle = now, None
        err = max(abs(target[m] - self.real[m]) for m in self.motors)
        # MoveIt 停住、實機也停穩後回報一次差多少：分辨「馬達沒轉到」還是「轉到了但位置跟模型不同」
        if now - self.changed >= STILL and err <= FOLLOW_TOL and self.settle is not True:
            if self.settle is None or err < self.settle[0] - 1:
                self.settle = (err, now)
            elif now - self.settle[1] > SETTLE_TIME:
                self.settle = True
                diags = [self._diag(a, target) for a in self.arms if a in self.moved] or \
                        [self._diag(a, target) for a in self.arms]
                self.moved = set()
                n = self._log(diags)
                self._say(f"跟隨中：實機停穩{f'（紀錄 #{n}）' if n else ''}。" + "；".join(g["text"] for g in diags) +
                          f"。速度上限 {MAX_SPEED}{self.skip_note}")
        # 跟著 MoveIt 動的時候實機一定會落後；MoveIt 停住後才檢查實機有沒有在往目標走
        if now - self.changed < STILL or err <= FOLLOW_TOL:
            self.best = None
            return
        if self.best is None or err < self.best - 2:
            self.best, self.best_t = err, now
        elif now - self.best_t > STUCK_TIME:
            m = max(self.motors, key=lambda i: abs(target[i] - self.real[i]))
            arm = next(a for a in self.arms if m in ARMS[a]["motors"])
            self._stop(f"MoveIt 停了，但實機 {STUCK_TIME:.0f} 秒沒有更接近目標（{m} 號 實機 {self.real[m]:.0f} / 目標 {target[m]}）："
                       f"扭力關了、卡住、撐不住（重力），或被別的程式移動了。{self._diag(arm, target)['text']}")


def main():
    rclpy.init()
    node = RealArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
