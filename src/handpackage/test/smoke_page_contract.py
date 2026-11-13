#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HandControlInterface.html 與 handpackage 的介面合約測試。

把網頁裡每一個 sendCmd(...) 的 payload 照原樣重建一次丟給節點，確認節點
全部接得住、而且不會有副作用（例如夾持還沒實作時不應該讓手動起來）。
改了網頁的指令格式、或改了節點的 _dispatch，就重跑這支。

檔名刻意不是 test_*.py，colcon test / pytest 不會自動收集。

用法：
    # 終端 1
    ros2 run handpackage handpackage
    # 終端 2
    python3 src/handpackage/test/smoke_page_contract.py
"""
import json, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

SIDES = ('left', 'right')

class H(Node):
    def __init__(self):
        super().__init__('contract')
        self.cmds = {s: [] for s in SIDES}
        self.settings = {s: [] for s in SIDES}
        self.status = None
        for s in SIDES:
            self.create_subscription(JointState, f'/cb_{s}_hand_control_cmd',
                (lambda k: lambda m: self.cmds[k].append([int(round(x)) for x in m.position]))(s), 10)
            self.create_subscription(String, f'/cb_{s}_hand_setting_cmd',
                (lambda k: lambda m: self.settings[k].append(json.loads(m.data)))(s), 10)
        self.create_subscription(String, '/hand/status',
                                 lambda m: setattr(self, 'status', json.loads(m.data)), 10)
        self.pub = self.create_publisher(String, '/hand/cmd', 10)
        self.st = {s: self.create_publisher(JointState, f'/cb_{s}_hand_state', 10)
                   for s in SIDES}
    def spin(self, t):
        t0 = time.time()
        while time.time() - t0 < t:
            rclpy.spin_once(self, timeout_sec=0.02)
    def send(self, payload):
        m = String(); m.data = json.dumps(payload); self.pub.publish(m); self.spin(0.35)
    def state(self, side, v):
        m = JointState(); m.position = [float(v)]*6; m.velocity=[0.0]*6; m.effort=[0.0]*6
        self.st[side].publish(m)
    def clear(self):
        for s in SIDES: self.cmds[s].clear(); self.settings[s].clear()

rclpy.init(); h = H()
fails = []
def chk(label, cond, extra=''):
    print(('  PASS  ' if cond else '  FAIL  ') + label + (f'   {extra}' if extra else ''))
    if not cond: fails.append(label)

# 連續跑好幾支測試時，上一個行程剛退出、訂閱還沒建立完，
# 第一批訊息會掉。先讓 discovery 穩定下來再開始
h.spin(2.0)
for _ in range(12):
    h.state('left', 100); h.state('right', 100); h.spin(0.05)

print('\n--- 頁面 payload 逐一驗證 ---')

# 1. slider 拖動（20Hz 節流送出的形狀）
h.clear(); h.send({"cmd":"manual","hand":"left","position":[10,20,30,40,50,60]})
chk('manual', h.cmds['left'] and h.cmds['left'][0] == [10,20,30,40,50,60],
    str(h.cmds['left'][0]) if h.cmds['left'] else '無')

# 2. applySpeed（0 原樣送出，不夾到 10）
h.clear(); h.send({"cmd":"speed","hand":"left","speed":[200,200,200,200,200,0]})
chk('speed', len(h.cmds['left']) >= 1)

# 3. applyTorque
h.clear(); h.send({"cmd":"torque","hand":"right","torque":[250,250,250,250,250,250]})
chk('torque', h.settings['right'] and
    h.settings['right'][0]['setting_cmd'] == 'set_max_torque_limits')

# 4. 預設動作（scope=both）。網頁送的是 manual + 位置 + 速度，
#    定義在 hurocup_interface/js/hand_presets.json，節點不認動作名稱
h.clear(); h.send({"cmd":"manual","hand":"both",
                   "position":[102,18,0,0,0,0],"speed":[150,150,150,150,150,150]})
chk('preset both', h.cmds['left'] and h.cmds['right'] and
    h.cmds['left'][0] == [102,18,0,0,0,0], str(h.cmds['left'][0]) if h.cmds['left'] else '')

# 5. 同步
h.send({"cmd":"sync","on":True})
chk('sync on', h.status and h.status['sync']['on'] is True)
h.send({"cmd":"sync","on":False})
chk('sync off', h.status and h.status['sync']['on'] is False)

# 6. 停止
h.send({"cmd":"stop"})
chk('stop → manual', h.status and h.status['mode'] == 'manual', h.status['mode'] if h.status else '')

# 7. 夾持（階段 ⑤ 尚未實作，驗證節點不會死、不會誤動手）
h.clear()
grip = {"cmd":"grip","action":"start","hand":"both","mode":"each",
        "threshold":[800,800,800,800,800],"target":[0,0,0,0,0,0],
        "step":4,"timeout":10,"release_margin":200}
h.send(grip)
chk('grip start 不會動手', len(h.cmds['left']) == 0 and len(h.cmds['right']) == 0,
    f"left={len(h.cmds['left'])}")
h.send({**grip, "action":"stop"})
chk('grip stop 不會死', h.status is not None)
for mode in ('any','anyhand','hold'):
    h.send({**grip, "mode":mode})
chk('四種夾持模式都接得住', h.status is not None)

# 8. STOP 按鈕（單純停下，不是切換）
h.send({"cmd":"stop"})
chk('stop -> manual', h.status and h.status['mode'] == 'manual',
    h.status['mode'] if h.status else '')

# 9. 壓感常駐訂閱（HandTouch 想在沒夾持時讀壓力）
h.send({"cmd":"touch","on":True})
chk('touch on', h.status and h.status.get('touch_hold') is True,
    str(h.status.get('touch_hold')) if h.status else '')
h.send({"cmd":"touch","on":False})
chk('touch off', h.status and h.status.get('touch_hold') is False,
    str(h.status.get('touch_hold')) if h.status else '')

# 10. 節點還活著
h.clear(); h.send({"cmd":"manual","hand":"right","position":[7,7,7,7,7,7]})
chk('全部跑完後節點正常', h.cmds['right'] and h.cmds['right'][0] == [7]*6)

print('\n' + ('全部通過' if not fails else f'{len(fails)} 項失敗: ' + ', '.join(fails)))
h.destroy_node(); rclpy.shutdown()
