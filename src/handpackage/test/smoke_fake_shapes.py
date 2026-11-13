#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""驗證 fake_hands 發出來的資料形狀，跟 .11 實機量到的一致。

這是「測試的測試」：如果 fake_hands 跟真手不一樣，拿它驗過的網頁就等於沒驗。
順便把網頁裡實際用的守門判斷式（palm 要被擋掉、mass 只取數字）也跑一遍。

需要先跑 fake_hands：
    # 終端 1
    python3 src/handpackage/test/fake_hands.py
    # 終端 2
    python3 src/handpackage/test/smoke_fake_shapes.py
"""
import json, time
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

SIDES = ('left', 'right')
MATRIX_KEYS = ['thumb_matrix','index_matrix','middle_matrix','ring_matrix','little_matrix']
MASS_KEYS = ['thumb_mass','index_mass','middle_mass','ring_mass','little_mass']

class V(Node):
    def __init__(self):
        super().__init__('verify_fake')
        self.d = {}
        for s in SIDES:
            self.create_subscription(String, f'/cb_{s}_hand_matrix_touch',
                (lambda k: lambda m: self.d.__setitem__(k, json.loads(m.data)))(f'{s}_mat'), 10)
            self.create_subscription(String, f'/cb_{s}_hand_matrix_touch_mass',
                (lambda k: lambda m: self.d.__setitem__(k, json.loads(m.data)))(f'{s}_mass'), 10)
            self.create_subscription(String, f'/cb_{s}_hand_info',
                (lambda k: lambda m: self.d.__setitem__(k, json.loads(m.data)))(f'{s}_info'), 10)
            self.create_subscription(JointState, f'/cb_{s}_hand_state',
                (lambda k: lambda m: self.d.__setitem__(k, m))(f'{s}_state'), 10)
        self.p_cmd = {s: self.create_publisher(JointState, f'/cb_{s}_hand_control_cmd', 10)
                      for s in SIDES}
    def spin(self, t):
        t0=time.time()
        while time.time()-t0 < t: rclpy.spin_once(self, timeout_sec=0.02)
    def cmd(self, side, pos):
        m=JointState(); m.position=[float(v) for v in pos]
        m.velocity=[0.0]*6; m.effort=[0.0]*6
        self.p_cmd[side].publish(m)

rclpy.init(); v=V()
fails=[]
def chk(l,c,e=''):
    print(('  PASS  ' if c else '  FAIL  ')+l+(f'   {e}' if e else ''))
    if not c: fails.append(l)

v.spin(3.0)

print('\n=== 形狀要和 .11 實機一致 ===')
mat = v.d.get('left_mat'); mass = v.d.get('left_mass'); info = v.d.get('left_info')
st = v.d.get('left_state')
chk('收到 matrix_touch', mat is not None)
chk('收到 matrix_touch_mass', mass is not None)
chk('收到 hand_info', info is not None)
chk('收到 state', st is not None)
if mat:
    shp = [(len(mat[k]), len(mat[k][0])) for k in MATRIX_KEYS]
    chk('五指矩陣都是 10x4', all(x == (10,4) for x in shp), str(set(shp)))
    chk('palm_matrix 是 [-1]（無掌心）', mat.get('palm_matrix') == [-1], str(mat.get('palm_matrix')))
    chk('有 stamp.secs', 'secs' in mat.get('stamp', {}))
    # 網頁的守門式：Array.isArray(m) && Array.isArray(m[0])
    def is2d(m): return isinstance(m, list) and m and isinstance(m[0], list)
    chk('網頁判斷式：五指是 2D', all(is2d(mat[k]) for k in MATRIX_KEYS))
    chk('網頁判斷式：palm 被擋掉', not is2d(mat.get('palm_matrix')))
if mass:
    chk('palm_mass 是 list 不是數字', isinstance(mass.get('palm_mass'), list),
        str(mass.get('palm_mass')))
    chk('五指 mass 是數字', all(isinstance(mass[k], (int,float)) for k in MASS_KEYS))
    chk('mass 在 0~10200', all(0 <= mass[k] <= 10200 for k in MASS_KEYS),
        str([mass[k] for k in MASS_KEYS]))
    chk("unit 照抄上游的 'g'", mass.get('unit') == 'g', str(mass.get('unit')))
    # 網頁的守門式：typeof v !== 'number' 就跳過
    nums = [k for k in list(MASS_KEYS)+['palm_mass'] if isinstance(mass.get(k), (int,float))]
    chk('網頁只會取到 5 根手指', len(nums) == 5, str(nums))
if info:
    chk('touch_type = 4', info.get('touch_type') == 4, str(info.get('touch_type')))
    chk('version = [3,1,3]', info.get('version') == [3,1,3], str(info.get('version')))
    chk('current 全 -1（O6 不支援）', info.get('current') == [-1]*6, str(info.get('current')))
    chk('speed 全 0（讀不回來）', info.get('speed') == [0]*6, str(info.get('speed')))
    chk('有溫度 6 筆', len(info.get('motor_temperature', [])) == 6,
        str(info.get('motor_temperature')))
    chk('fault 全 0', info.get('fault') == [0]*6)
if st:
    chk('state.name 是 finger_order', list(st.name)[4] == 'ring_mcp_pitch',
        str(list(st.name)[4:]))
    chk('state.velocity 全 0', all(x == 0 for x in st.velocity))

print('\n=== 握緊會產生壓力（波形/熱力圖才看得到東西）===')
# SDK 的初始姿態 [200,255,255,255,255,180] 並不是完全張開（拇指 200、小指 180），
# 所以那兩指本來就會有一點壓力。要量「零壓力」得先下全張開
v.cmd('left', [255]*6); v.spin(1.0)
m0 = [v.d['left_mass'][k] for k in MASS_KEYS]
chk('全張開時壓力為 0', sum(m0) == 0, str(m0))
v.cmd('left', [0,0,0,0,0,0])       # 全部握緊
v.spin(1.0)
m1 = [v.d['left_mass'][k] for k in MASS_KEYS]
chk('握緊後五指都有壓力', all(x > 500 for x in m1), str(m1))
mat1 = v.d['left_mat']
peak = max(max(r) for r in mat1['index_matrix'])
chk('熱力圖有峰值', peak > 100, f'peak={peak}')
v.cmd('left', [255]*6)
v.spin(1.0)
m2 = [v.d['left_mass'][k] for k in MASS_KEYS]
chk('放開後壓力回到接近 0', sum(m2) < 500, str(m2))

print('\n=== 左右手要能分辨（相位錯開）===')
v.cmd('left', [0]*6); v.cmd('right', [0]*6); v.spin(1.2)
ml = max(max(r) for r in v.d['left_mat']['thumb_matrix'])
mr = max(max(r) for r in v.d['right_mat']['thumb_matrix'])
chk('兩手都有資料', ml > 50 and mr > 50, f'L={ml} R={mr}')

print('\n' + ('全部通過' if not fails else f'{len(fails)} 項失敗: ' + ', '.join(fails)))
sys.stdout.flush()
# 乾淨關閉 DDS。用 os._exit 強制退出會留下幽靈 participant，
# 害下一支測試的 discovery 要等逾時才接得上（連續跑會假性失敗）
try:
    v.destroy_node()
except Exception:
    pass
rclpy.try_shutdown()
sys.exit(0 if not fails else 1)
