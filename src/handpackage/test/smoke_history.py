#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""執行歷史測試：Undo / Redo / Clear，以及預設動作自帶速度。

重點是「Undo 要讓手真的回到執行前的位置」—— 與 motionpackage 同一套語意，
記的是執行過哪些 sector，不是編輯層的復原。

需要先跑 handpackage（不要同時跑 fake_hands）。
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int16

SIDES = ('left', 'right')
rclpy.init()
n = Node('t_hist')
cmds = {s: [] for s in SIDES}
st = {}
for s in SIDES:
    n.create_subscription(JointState, f'/cb_{s}_hand_control_cmd',
        (lambda k: lambda m: cmds[k].append([int(round(v)) for v in m.position]))(s), 10)
n.create_subscription(String, '/hand/status', lambda m: st.update(json.loads(m.data)), 10)
pub = n.create_publisher(String, '/hand/cmd', 10)
pst = {s: n.create_publisher(JointState, f'/cb_{s}_hand_state', 10) for s in SIDES}
psec = n.create_publisher(Int16, '/hand/Sector', 10)

def spin(t):
    t0 = time.time()
    while time.time() - t0 < t:
        rclpy.spin_once(n, timeout_sec=0.02)

def send(p):
    pub.publish(String(data=json.dumps(p))); spin(0.35)

def state(side, v):
    m = JointState(); m.position = [float(v)]*6; m.velocity=[0.0]*6; m.effort=[0.0]*6
    pst[side].publish(m)

fails = []
def chk(l, c, e=''):
    print(('  PASS  ' if c else '  FAIL  ') + l + (f'   {e}' if e else ''))
    if not c: fails.append(l)

for _ in range(12):
    state('left', 100); state('right', 100); spin(0.05)

print('\n=== 歷史在 status 裡 ===')
chk('status 有 history 欄位', 'history' in st, str(list(st.keys())))
chk('一開始是空的', st.get('history', {}).get('past') == [], str(st.get('history')))

print('\n=== 手動設一個位置，再 Undo（此時沒有歷史，不該動）===')
send({'cmd': 'manual', 'hand': 'both', 'position': [200]*6})
cmds['left'].clear()
send({'cmd': 'undo'})
chk('沒有歷史時 Undo 不動手', len(cmds['left']) == 0, f"{len(cmds['left'])} 筆")

print('\n=== 播放會記一筆歷史 ===')
send({'cmd': 'play', 'action': 'start', 'left': 999, 'right': None, 'loop': 1})
spin(0.5)
h = st.get('history', {})
chk('歷史多一筆', len(h.get('past', [])) == 1, str(h.get('past')))
chk('標籤是 Play 999', h.get('past', [''])[0] == 'Play 999', str(h.get('past')))

print('\n=== Undo 要回到執行前的位置 ===')
cmds['left'].clear()
send({'cmd': 'undo'})
chk('Undo 有送位置回去', len(cmds['left']) >= 1, f"{len(cmds['left'])} 筆")
chk('回到執行前的 200', cmds['left'] and cmds['left'][-1] == [200]*6,
    str(cmds['left'][-1]) if cmds['left'] else '')
h = st.get('history', {})
chk('past 清空、future 有一筆',
    h.get('past') == [] and len(h.get('future', [])) == 1, str(h))

print('\n=== Redo ===')
send({'cmd': 'manual', 'hand': 'left', 'position': [50]*6})
cmds['left'].clear()
send({'cmd': 'redo'})
chk('Redo 有動作', len(cmds['left']) >= 1, f"{len(cmds['left'])} 筆")
h = st.get('history', {})
chk('future 清空、past 有一筆',
    h.get('future') == [] and len(h.get('past', [])) == 1, str(h))

print('\n=== Clear ===')
send({'cmd': 'clear_history'})
h = st.get('history', {})
chk('兩邊都清空', h.get('past') == [] and h.get('future') == [], str(h))

print('\n=== 預設動作（網頁送 manual + 位置 + 速度）===')
cmds['left'].clear()
send({'cmd': 'manual', 'hand': 'left',
      'position': [102, 18, 0, 0, 0, 0], 'speed': [150] * 6})
chk('握拳位置正確', cmds['left'] and cmds['left'][0] == [102, 18, 0, 0, 0, 0],
    str(cmds['left'][0]) if cmds['left'] else '')

print('\n' + ('全部通過' if not fails else f'{len(fails)} 項失敗: ' + ', '.join(fails)))
sys.stdout.flush()
# 乾淨關閉 DDS。用 os._exit 強制退出會留下幽靈 participant，
# 害下一支測試的 discovery 要等逾時才接得上（連續跑會假性失敗）
try:
    n.destroy_node()
except Exception:
    pass
rclpy.try_shutdown()
sys.exit(0 if not fails else 1)
