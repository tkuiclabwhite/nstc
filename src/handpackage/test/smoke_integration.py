#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""整合測試：fake_hands + handpackage，真正的閉迴路。

fake_hands 的壓力會隨命令位置變化（越彎越大），所以夾持迴路在這裡是真的在
收斂 —— 這是沒有硬體時最接近實機的驗證。會檢查夾持停在合理位置、門檻越高
夾得越緊、anyhand 模式兩手同時停。

需要同時跑兩個節點：
    # 終端 1
    python3 src/handpackage/test/fake_hands.py
    # 終端 2
    ros2 run handpackage handpackage
    # 終端 3
    python3 src/handpackage/test/smoke_integration.py
"""
import json, time
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

MASS_KEYS = ['thumb_mass','index_mass','middle_mass','ring_mass','little_mass']

class W(Node):
    def __init__(self):
        super().__init__('integ')
        self.status=None; self.state={}
        self.create_subscription(String,'/hand/status',
            lambda m: setattr(self,'status',json.loads(m.data)),10)
        for s in ('left','right'):
            self.create_subscription(JointState, f'/cb_{s}_hand_state',
                (lambda k: lambda m: self.state.__setitem__(k,[int(round(v)) for v in m.position]))(s),10)
        self.pub=self.create_publisher(String,'/hand/cmd',10)
    def spin(self,t):
        t0=time.time()
        while time.time()-t0<t: rclpy.spin_once(self,timeout_sec=0.02)
    def send(self,p):
        self.pub.publish(String(data=json.dumps(p))); self.spin(0.3)
    def g(self): return (self.status or {}).get('grip',{})
    def mode(self): return (self.status or {}).get('mode')

rclpy.init(); w=W()
fails=[]
def chk(l,c,e=''):
    print(('  PASS  ' if c else '  FAIL  ')+l+(f'   {e}' if e else ''))
    if not c: fails.append(l)

w.spin(3.0)
print('\n=== handpackage 看得到兩隻假手 ===')
h=(w.status or {}).get('hands',{})
chk('左手 online', (h.get('left') or {}).get('online') is True)
chk('右手 online', (h.get('right') or {}).get('online') is True)
chk('有讀到溫度', len(((h.get('left') or {}).get('temperature') or []))==6,
    str((h.get('left') or {}).get('temperature')))
chk('touch_type 傳到網頁層', (h.get('left') or {}).get('touch_type')==4,
    str((h.get('left') or {}).get('touch_type')))

print('\n=== 先全張開 ===')
w.send({'cmd':'manual','hand':'both','position':[255]*6})
w.spin(1.0)
chk('兩手都張開', w.state.get('left')==[255]*6, str(w.state.get('left')))

print('\n=== each 模式夾持：壓力會隨閉合上升，應該在中途停住 ===')
w.send({'cmd':'grip','action':'start','hand':'both','mode':'each',
        'threshold':800,'target':[0]*6,'step':3,'timeout':12,'release_margin':200})
# 等它收斂
t0=time.time(); 
while time.time()-t0<14:
    w.spin(0.3)
    if not w.g().get('active'): break
gs=w.g()
chk('夾持已結束', not gs.get('active'))
chk('結束理由是壓力到門檻', '壓力到門檻' in gs.get('reason',''), gs.get('reason',''))
# 理由長這樣：'left: 壓力到門檻（thumb, index, middle, ring, little）；right: ...'
# 要數的是括號裡的手指名稱，不是冒號 —— 冒號只有 'left:' / 'right:' 那兩個，
# 數冒號永遠是 2，這個檢查等於沒在檢查（之前就是這樣假性紅/假性綠）
def fingers(reason, side):
    import re as _re
    m = _re.search(side + r': 壓力到門檻（([^）]*)）', reason)
    return [x.strip() for x in m.group(1).split(',')] if m else []

chk('each：每指各自到門檻，五指都該被列出',
    len(fingers(gs.get('reason',''),'left'))==5
    and len(fingers(gs.get('reason',''),'right'))==5,
    gs.get('reason',''))
lp=w.state.get('left',[])
chk('沒有閉合到底（有被壓力擋下來）', lp and all(v>20 for v in lp), str(lp))
chk('確實有往閉合方向走', lp and all(v<255 for v in lp), str(lp))
fr=gs.get('frozen',{}).get('left',[])
chk('快照顯示六軸全凍結', len(fr)==6 and all(fr), str(fr))
chk('回到 manual', w.mode()=='manual', str(w.mode()))

print('\n=== 門檻越高應該夾得越緊 ===')
w.send({'cmd':'manual','hand':'both','position':[255]*6}); w.spin(1.0)
w.send({'cmd':'grip','action':'start','hand':'left','mode':'each',
        'threshold':1600,'target':[0]*6,'step':3,'timeout':12,'release_margin':200})
t0=time.time()
while time.time()-t0<14:
    w.spin(0.3)
    if not w.g().get('active'): break
lp2=w.state.get('left',[])
chk('門檻 1600 比 800 夾得更緊', lp2 and sum(lp2) < sum(lp), f'800->{sum(lp)} 1600->{sum(lp2)}')

print('\n=== anyhand 模式：一手到門檻兩手都停 ===')
w.send({'cmd':'manual','hand':'both','position':[255]*6}); w.spin(1.0)
w.send({'cmd':'grip','action':'start','hand':'both','mode':'anyhand',
        'threshold':800,'target':[0]*6,'step':3,'timeout':12,'release_margin':200})
t0=time.time()
while time.time()-t0<14:
    w.spin(0.3)
    if not w.g().get('active'): break
gs=w.g()
chk('兩手都停在相近位置', abs(sum(w.state.get('left',[0]))-sum(w.state.get('right',[0]))) < 120,
    f"L={sum(w.state.get('left',[0]))} R={sum(w.state.get('right',[0]))}")
# anyhand 是「任一指到就兩手一起停」，所以停下來那一刻通常只有一兩根到門檻
nl = len(fingers(gs.get('reason',''),'left'))
nr = len(fingers(gs.get('reason',''),'right'))
chk('anyhand：只列出先到門檻的那一兩根', 0 < max(nl, nr) <= 3,
    f"L={nl} R={nr}  {gs.get('reason','')}")

print('\n' + ('全部通過' if not fails else f'{len(fails)} 項失敗: ' + ', '.join(fails)))
sys.stdout.flush()
# 乾淨關閉 DDS。用 os._exit 強制退出會留下幽靈 participant，
# 害下一支測試的 discovery 要等逾時才接得上（連續跑會假性失敗）
try:
    w.destroy_node()
except Exception:
    pass
rclpy.try_shutdown()
sys.exit(0 if not fails else 1)
