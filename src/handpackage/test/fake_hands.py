#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""假的 linker_hand_sdk：沒有真手也能把三個網頁跑起來看。

發出的 topic 與資料形狀跟真的 SDK 一模一樣（依 2026-10 在 172.17.121.11 上
實機量到的規格）：

    /cb_{left,right}_hand_state              sensor_msgs/JointState
    /cb_{left,right}_hand_info               std_msgs/String  (JSON)
    /cb_{left,right}_hand_matrix_touch       std_msgs/String  (JSON, 每指 10x4)
    /cb_{left,right}_hand_matrix_touch_mass  std_msgs/String  (JSON)

而且會訂 /cb_{side}_hand_control_cmd，把收到的位置當成手真的到位 ——
所以 HandControlInterface 的 slider、夾持迴路都能實際操作看反應。

刻意模仿的幾個真實行為（網頁要擋得住這些）：
  * 沒有掌心感測器：palm_matrix 回 [-1]，palm_mass 回 list [-1] 而不是數字
  * touch_type = 4，矩陣是 10 列 x 4 行
  * matrix_touch_mass 的 unit 標 "g" 但其實是 40 格 0~255 的總和（上限 10200）
  * 速度讀不回來（state.velocity 與 info.speed 全 0）、電流不支援（全 -1）
  * 只有在有人訂閱時才發，跟真 SDK 的 get_subscription_count() 行為一致

壓力是模擬的：手指越彎（position 越小）壓力越大，再疊一個沿著指節游走的
壓力團，這樣波形和熱力圖都會動。

用法：
    # 終端 1（不要同時跑真的 SDK）
    python3 src/handpackage/test/fake_hands.py
    # 終端 2
    ros2 run handpackage handpackage
    # 終端 3
    ros2 run rosbridge_server rosbridge_websocket
    # 然後瀏覽器開 HandControlInterface.html / HandTouchInterface.html
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

SIDES = ('left', 'right')
NJ = 6
ROWS, COLS = 10, 4          # touch_type 4 的矩陣尺寸
FINGER_ORDER = ['thumb_cmc_pitch', 'thumb_cmc_yaw', 'index_mcp_pitch',
                'middle_mcp_pitch', 'ring_mcp_pitch', 'pinky_mcp_pitch']
MATRIX_KEYS = ['thumb_matrix', 'index_matrix', 'middle_matrix',
               'ring_matrix', 'little_matrix']
MASS_KEYS = ['thumb_mass', 'index_mass', 'middle_mass', 'ring_mass', 'little_mass']
# 六軸對應哪一根手指（拇指兩軸共用 thumb），與 hand_config.JOINT_TO_FINGER 一致
JOINT_TO_FINGER = [0, 0, 1, 2, 3, 4]


class FakeHands(Node):

    def __init__(self):
        super().__init__('fake_hands')
        self.declare_parameter('squeeze', True)   # 是否模擬「彎曲就有壓力」
        self.squeeze = bool(self.get_parameter('squeeze').value)

        self.pos = {s: [200, 255, 255, 255, 255, 180] for s in SIDES}   # SDK 的初始姿態
        self.t0 = time.time()

        self.p_state, self.p_info, self.p_mat, self.p_mass = {}, {}, {}, {}
        for s in SIDES:
            self.p_state[s] = self.create_publisher(JointState, f'/cb_{s}_hand_state', 10)
            self.p_info[s] = self.create_publisher(String, f'/cb_{s}_hand_info', 10)
            self.p_mat[s] = self.create_publisher(
                String, f'/cb_{s}_hand_matrix_touch', 10)
            self.p_mass[s] = self.create_publisher(
                String, f'/cb_{s}_hand_matrix_touch_mass', 10)
            self.create_subscription(JointState, f'/cb_{s}_hand_control_cmd',
                                     self._mk_cmd(s), 10)

        # 真 SDK 的 pub_state 執行緒是 1/60 s，這裡用一樣的節奏
        self.create_timer(1.0 / 55.0, self.tick)
        self.get_logger().info(
            f'fake_hands 啟動：矩陣 {ROWS}x{COLS}、無掌心、touch_type=4、'
            f'squeeze={"開" if self.squeeze else "關"}')

    def _mk_cmd(self, side):
        def cb(msg):
            if len(msg.position) == NJ:
                self.pos[side] = [int(round(v)) for v in msg.position]
        return cb

    # ------------------------------------------------------------------
    def finger_press(self, side, finger):
        """這根手指的「握力」0~1：越彎越大。"""
        if not self.squeeze:
            return 0.0
        joints = [j for j in range(NJ) if JOINT_TO_FINGER[j] == finger]
        # 位置 255 = 張開 = 沒壓力，0 = 握緊 = 壓力最大
        closed = sum((255 - self.pos[side][j]) / 255.0 for j in joints) / len(joints)
        return max(0.0, min(1.0, closed))

    def matrix_for(self, side, finger):
        """10x4 矩陣：底噪 + 一個沿指節游走的壓力團，整體強度由握力決定。"""
        press = self.finger_press(side, finger)
        if press <= 0.01:
            return [[0] * COLS for _ in range(ROWS)]
        t = time.time() - self.t0
        # 壓力團中心沿著列方向來回走，左右手相位錯開比較好分辨
        phase = t * 0.7 + (0.0 if side == 'left' else math.pi) + finger * 0.5
        cr = (ROWS - 1) * (0.5 + 0.45 * math.sin(phase))
        cc = (COLS - 1) * 0.5
        out = []
        for r in range(ROWS):
            row = []
            for c in range(COLS):
                d2 = ((r - cr) / 2.2) ** 2 + ((c - cc) / 1.4) ** 2
                v = 255.0 * press * math.exp(-d2)
                row.append(int(max(0, min(255, round(v)))))
            out.append(row)
        return out

    def tick(self):
        now = self.get_clock().now().to_msg()
        for s in SIDES:
            if self.p_state[s].get_subscription_count() > 0:
                js = JointState()
                js.header.stamp = now
                js.name = list(FINGER_ORDER)
                js.position = [float(v) for v in self.pos[s]]
                js.velocity = [0.0] * NJ          # 真機讀不回來，固定 0
                js.effort = [0.0] * NJ
                self.p_state[s].publish(js)

            if self.p_info[s].get_subscription_count() > 0:
                base = 36 if s == 'left' else 38
                temps = [base + int(4 * self.finger_press(s, JOINT_TO_FINGER[j]))
                         + (j % 3) for j in range(NJ)]
                torque = [int(60 * self.finger_press(s, JOINT_TO_FINGER[j]))
                          for j in range(NJ)]
                self.p_info[s].publish(String(data=json.dumps({
                    'version': [3, 1, 3],
                    'hand_joint': 'O6',
                    'speed': [0] * NJ,            # O6 讀不回來
                    'current': [-1] * NJ,         # O6 不支援
                    'fault': [0] * NJ,
                    'motor_temperature': temps,
                    'torque': torque,
                    'is_touch': True,
                    'touch_type': 4,
                    'finger_order': FINGER_ORDER,
                })))

            need_touch = (self.p_mat[s].get_subscription_count() > 0
                          or self.p_mass[s].get_subscription_count() > 0)
            if not need_touch:
                continue

            mats = [self.matrix_for(s, f) for f in range(5)]
            sec = now.sec
            nsec = now.nanosec

            if self.p_mat[s].get_subscription_count() > 0:
                d = {'stamp': {'sec': 0, 'nanosec': 0, 'secs': sec, 'nsecs': nsec}}
                for k, m in zip(MATRIX_KEYS, mats):
                    d[k] = m
                d['palm_matrix'] = [-1]           # 這批 O6 沒有掌心感測器
                self.p_mat[s].publish(String(data=json.dumps(d)))

            if self.p_mass[s].get_subscription_count() > 0:
                d = {'stamp': {'secs': sec, 'nsecs': nsec}}
                for k, m in zip(MASS_KEYS, mats):
                    d[k] = sum(sum(row) for row in m)
                d['palm_mass'] = [-1]             # 注意是 list 不是數字
                d['unit'] = 'g'                   # 上游標錯，其實是 0~10200 的總和
                self.p_mass[s].publish(String(data=json.dumps(d)))


def main(args=None):
    rclpy.init(args=args)
    node = FakeHands()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
