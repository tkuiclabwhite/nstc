#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""handpackage 煙霧測試：不需要硬體，裸 Humble 就能跑。

這支會假扮成「SDK + 網頁」兩邊：發假的 /cb_*_hand_state 讓節點以為手在線，
從 /hand/cmd 下指令，再檢查 /cb_*_hand_control_cmd 和 /cb_*_hand_setting_cmd
收到的內容對不對。改完 handpackage 之後重跑一次，可以確認沒改壞。

檔名刻意不是 test_*.py，這樣 colcon test / pytest 不會自動收集它
（它需要另一個終端先把節點跑起來，自動跑會失敗）。

用法：
    # 終端 1
    ros2 run handpackage handpackage
    # 終端 2
    python3 src/handpackage/test/smoke_hands.py

兩邊的 ROS_DOMAIN_ID 要一樣。想跟正在跑的真手隔開，兩邊都另外設一個，例如
export ROS_DOMAIN_ID=77。
"""
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

SIDES = ('left', 'right')

class Harness(Node):
    def __init__(self):
        super().__init__('harness')
        self.cmds = {s: [] for s in SIDES}      # 收到的 control_cmd
        self.settings = {s: [] for s in SIDES}  # 收到的 setting_cmd
        self.status = None
        self.config = None
        for s in SIDES:
            self.create_subscription(JointState, f'/cb_{s}_hand_control_cmd',
                                     (lambda side: lambda m: self.cmds[side].append(
                                         ([int(round(x)) for x in m.position],
                                          [int(round(x)) for x in m.velocity])))(s), 10)
            self.create_subscription(String, f'/cb_{s}_hand_setting_cmd',
                                     (lambda side: lambda m: self.settings[side].append(
                                         json.loads(m.data)))(s), 10)
        self.create_subscription(String, '/hand/status', lambda m: setattr(self, 'status', json.loads(m.data)), 10)
        self.create_subscription(String, '/hand/config', lambda m: setattr(self, 'config', json.loads(m.data)), 10)
        self.pub_cmd = self.create_publisher(String, '/hand/cmd', 10)
        self.pub_state = {s: self.create_publisher(JointState, f'/cb_{s}_hand_state', 10) for s in SIDES}

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self, **kw):
        m = String(); m.data = json.dumps(kw); self.pub_cmd.publish(m); self.spin(0.35)

    def fake_state(self, side, pos):
        m = JointState(); m.position = [float(v) for v in pos]
        m.velocity = [0.0]*6; m.effort = [0.0]*6
        self.pub_state[side].publish(m)

    def clear(self):
        for s in SIDES: self.cmds[s].clear(); self.settings[s].clear()

rclpy.init()
h = Harness()
fails = []
def check(label, cond, extra=''):
    print(('  PASS  ' if cond else '  FAIL  ') + label + (f'   {extra}' if extra else ''))
    if not cond: fails.append(label)

# 讓節點以為兩手都在線（持續送 state，否則會被判離線）
for _ in range(12):
    h.fake_state('left',  [100, 100, 100, 100, 100, 100])
    h.fake_state('right', [200, 200, 200, 200, 200, 200])
    h.spin(0.05)

print('\n=== /hand/config ===')
# config 是 1 Hz 發的，等它到（別用固定 sleep，discovery 時間不固定）
t0 = time.time()
while h.config is None and time.time() - t0 < 4.0:
    h.spin(0.1)
check('有收到 config', h.config is not None)
if h.config:
    check('關節名正確', h.config['joint_names'][4] == 'ring_mcp_pitch', h.config['joint_names'][4])
    # 預設動作已經搬到 hurocup_interface/js/hand_presets.js，
    # 節點不再認動作名稱，所以 config 裡不該有 presets
    check('config 沒有 presets（已移到網頁）', 'presets' not in h.config,
          str(list(h.config.keys())))
    check('mass 上限 10200', h.config['mass_max'] == 10200)

print('\n=== 1. 啟動後不該亂動手 ===')
check('沒下命令前不發 control_cmd', len(h.cmds['left']) == 0 and len(h.cmds['right']) == 0,
      f"left={len(h.cmds['left'])} right={len(h.cmds['right'])}")

print('\n=== 2. 手動單手 ===')
h.clear()
h.send(cmd='manual', hand='left', position=[10, 20, 30, 40, 50, 60])
check('左手收到指令', len(h.cmds['left']) >= 1)
check('左手位置正確', h.cmds['left'] and h.cmds['left'][0][0] == [10,20,30,40,50,60],
      str(h.cmds['left'][0][0]) if h.cmds['left'] else '')
check('沒改速度時 velocity 全 0', h.cmds['left'] and h.cmds['left'][0][1] == [0]*6,
      str(h.cmds['left'][0][1]) if h.cmds['left'] else '')
check('右手不受影響（同步關）', len(h.cmds['right']) == 0)
check('模式變 manual', h.status and h.status['mode'] == 'manual', h.status['mode'] if h.status else '')

print('\n=== 3. 相同值不重發（省 CAN）===')
h.clear()
h.send(cmd='manual', hand='left', position=[10, 20, 30, 40, 50, 60])
check('重複同一組值不再發', len(h.cmds['left']) == 0, f"發了 {len(h.cmds['left'])} 筆")

print('\n=== 4. -1 = 這軸不動，要換成實際位置 ===')
h.clear()
h.send(cmd='manual', hand='right', position=[-1, 77, -1, -1, -1, -1])
got = h.cmds['right'][0][0] if h.cmds['right'] else None
check('-1 換成實際位置 200', got == [200, 77, 200, 200, 200, 200], str(got))
check('沒有把 -1 送給 SDK', got is not None and all(v >= 0 for v in got))

print('\n=== 5. 速度：0 = 沿用舊值，而且送出去絕不含 0 ===')
# 上游 set_joint_speed 要求六個值全部在 10~255，只要有一個 0 就整包丟掉。
# 所以 0 必須在 handpackage 這邊換成舊值，不能原樣送出去。
# 節點會記住速度狀態，所以先下一組已知基準，測試才不會受前一次執行影響
h.send(cmd='speed', hand='left', speed=[150] * 6)
h.clear()
h.send(cmd='speed', hand='left', speed=[0, 5, 300, 128, 10, 255])
got = h.cmds['left'][0][1] if h.cmds['left'] else None
check('0 沿用基準 150、<10 抬到 10、>255 夾到 255',
      got == [150, 10, 255, 128, 10, 255], str(got))
check('送出的速度不含 0（否則上游會整包丟掉）',
      got is not None and all(v >= 10 for v in got), str(got))
h.clear()
h.send(cmd='speed', hand='left', speed=[0, 0, 0, 0, 0, 77])
got = h.cmds['left'][0][1] if h.cmds['left'] else None
check('只改一軸，其餘沿用上一次的值',
      got == [150, 10, 255, 128, 10, 77], str(got))
h.clear()
h.send(cmd='speed', hand='left', speed=[0] * 6)
check('全 0 = 什麼都不改，不發指令', len(h.cmds['left']) == 0,
      f"發了 {len(h.cmds['left'])} 筆")

print('\n=== 6. 雙手同步 ===')
h.send(cmd='sync', on=True)
h.clear()
h.send(cmd='manual', hand='left', position=[1, 2, 3, 4, 5, 6])
lp = h.cmds['left'][0][0] if h.cmds['left'] else None
rp = h.cmds['right'][0][0] if h.cmds['right'] else None
check('左手套用', lp == [1,2,3,4,5,6], str(lp))
check('右手跟著動（恆等鏡像）', rp == [1,2,3,4,5,6], str(rp))

print('\n=== 7. 12 個值直接指定兩手 ===')
h.clear()
h.send(cmd='manual', position=[11,12,13,14,15,16, 21,22,23,24,25,26])
lp = h.cmds['left'][0][0] if h.cmds['left'] else None
rp = h.cmds['right'][0][0] if h.cmds['right'] else None
check('左手 L1..L6', lp == [11,12,13,14,15,16], str(lp))
check('右手 R1..R6', rp == [21,22,23,24,25,26], str(rp))

print('\n=== 8. 預設動作（網頁自己送位置+速度，節點不認名稱）===')
h.send(cmd='sync', on=False)
h.clear()
# 這就是 js/hand_presets.js 的「握拳」送出來的形狀
h.send(cmd='manual', hand='both', position=[102, 18, 0, 0, 0, 0], speed=[150] * 6)
lp = h.cmds['left'][0][0] if h.cmds['left'] else None
check('握拳 = [102,18,0,0,0,0]', lp == [102, 18, 0, 0, 0, 0], str(lp))
# 速度要在同一筆裡就生效（節點是先套速度再套位置），否則第一次按會用舊速度
sp = h.cmds['left'][0][1] if h.cmds['left'] else None
check('速度同一筆就生效', sp == [150] * 6, str(sp))
h.clear()
h.send(cmd='preset', name='握拳')
check('preset 指令已移除，不會動手', len(h.cmds['left']) == 0,
      f"發了 {len(h.cmds['left'])} 筆")

print('\n=== 9. 扭矩（走 remap 後的分手 topic）===')
h.clear()
h.send(cmd='torque', hand='right', torque=[250]*6)
st = h.settings['right'][0] if h.settings['right'] else None
check('右手收到 setting_cmd', st is not None)
check('setting_cmd 內容正確',
      st == {'setting_cmd': 'set_max_torque_limits',
             'params': {'hand_type': 'right', 'torque': [250]*6}}, json.dumps(st, ensure_ascii=False) if st else '')
check('左手沒收到（分手 topic 生效）', len(h.settings['left']) == 0)

print('\n=== 10. 停止（單純停下，不鎖定）===')
# 原本是「緊急停止」：凍結兩手並鎖住所有指令，要再按一次才解除。
# 現在改成單純停下 —— 停掉夾持與播放、回手動，手保持在最後下達的位置，
# 不鎖定，所以停完馬上還能下手動命令。
for _ in range(6):
    h.fake_state('left', [123]*6); h.fake_state('right', [231]*6); h.spin(0.05)
h.send(cmd='manual', hand='left', position=[88]*6)
h.clear()
h.send(cmd='stop')
check('停止後模式回 manual', h.status and h.status['mode'] == 'manual',
      h.status['mode'] if h.status else '')
check('status 不再有 estop 欄位', h.status and 'estop' not in h.status,
      str(sorted(h.status.keys())) if h.status else '')
h.clear()
h.send(cmd='manual', hand='left', position=[77]*6)
check('停止後不會鎖住，手動照常', len(h.cmds['left']) >= 1,
      f"發了 {len(h.cmds['left'])} 筆")
h.clear()
h.send(cmd='estop', on=True)
check('estop 指令已移除，不會有反應', len(h.cmds['left']) == 0,
      f"發了 {len(h.cmds['left'])} 筆")

print('\n=== 11. status 內容 ===')
# online 有 1 秒 timeout，檢查前先補送 state（不補就會正確地判成離線）
for _ in range(6):
    h.fake_state('left', [88]*6); h.fake_state('right', [88]*6); h.spin(0.05)
h.spin(0.2)
s = h.status
check('有 hands.left.position', s and s['hands']['left']['position'] is not None)
check('online 判定正確', s and s['hands']['left']['online'] is True)
check('grip/play 空殼存在', s and 'grip' in s and 'play' in s)
# 不要寫死 'ar' —— 策略是執行期的值（strategy.ini 會被網頁改掉），
# 真正要驗的是「節點讀到的跟 strategy.ini 一致」
def _ini_strategy():
    import os
    path = os.path.join(os.path.expanduser('~'),
                        'ros2_adult/src/strategy/strategy/strategy.ini')
    try:
        with open(path, encoding='utf-8') as f:
            parts = [x for x in f.read().strip().split('/') if x]
        if 'Parameter' in parts:
            i = parts.index('Parameter')
            return parts[i - 1] if i > 0 else ''
        return parts[-1] if parts else ''
    except OSError:
        return ''


want = _ini_strategy()
check('策略與 strategy.ini 一致', s and want and s.get('strategy') == want,
      f"節點={s.get('strategy') if s else None} strategy.ini={want}")

print('\n=== 11b. 停送 state 後要判離線 ===')
h.spin(1.4)
check('超過 timeout 判離線', h.status and h.status['hands']['left']['online'] is False,
      str(h.status['hands']['left']['online']) if h.status else '')

print('\n=== 12. 壞輸入不該讓節點死 ===')
m = String(); m.data = 'not json at all'; h.pub_cmd.publish(m); h.spin(0.3)
h.send(cmd='manual', hand='middle', position=[0]*6)
h.send(cmd='manual', position=[1,2,3])
h.send(cmd='沒這個指令')
h.clear(); h.send(cmd='manual', hand='left', position=[42]*6)
check('壞輸入後節點還活著', len(h.cmds['left']) >= 1)

print('\n' + ('全部通過' if not fails else f'{len(fails)} 項失敗: ' + ', '.join(fails)))
h.destroy_node(); rclpy.shutdown()
