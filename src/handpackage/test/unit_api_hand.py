#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""strategy/API.py 手部函式的單元測試。

把 API 類別裡「靈巧手」那一段原始碼抽出來掛在假物件上跑，所以**不用 ROS、
不用編譯、不用真的手**，純驗參數檢查與送出去的 payload 形狀。

這支會抓到的問題：送錯 topic 格式（節點收到會整包忽略，很難從現象看出來）、
sendHandMotor 忘記把其他軸填 -1（會把別的手指一起拉走）、
預設動作的 JSON 路徑或欄位改掉之後沒跟上。

用法（在 ros2_adult 根目錄）：
    python3 src/handpackage/test/unit_api_hand.py
"""
import io, json, os, sys, types
from typing import List, Optional

import os
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')
src = io.open(os.path.join(ROOT, 'src/strategy/strategy/API.py'), encoding='utf-8').read()
block = src[src.index('    HAND_SIDES = '):src.index('    def drawImageFunction')]

sent = []
class FakeLogger:
    def warn(self, m): print('  [warn]', m)
class Fake:
    HAND_SIDES = ('left', 'right')
    def get_logger(self): return FakeLogger()
    def _hand_cmd(self, p): sent.append(p)

ns = {'json': json, 'os': os, 'List': List, 'Optional': Optional,
      'String': lambda: types.SimpleNamespace(data=''),
      'Int16': lambda: types.SimpleNamespace(data=0), 'API': Fake}
exec('class _T:\n' + block, ns)
for name, fn in vars(ns['_T']).items():
    if not name.startswith('__'):
        setattr(Fake, name, fn)
# 真的 _hand_cmd 會去碰 publisher，換回攔截版（要在上面的迴圈之後）
Fake._hand_cmd = lambda self, p: sent.append(p)
a = Fake()
a.hand_presets = a._load_hand_presets()

fails = []
def chk(label, cond, extra=''):
    print(('  PASS  ' if cond else '  FAIL  ') + label + (f'   {extra}' if extra else ''))
    if not cond: fails.append(label)

print('\n=== 預設動作從共用的 js/hand_presets.js 讀進來 ===')
chk('讀到 16 組', len(a.hand_presets) == 16, str(len(a.hand_presets)))
chk('是從 .js 讀的（不是 .json）', a.HAND_PRESETS_PATH.endswith('hand_presets.js'),
    a.HAND_PRESETS_PATH)
chk('張開在裡面', '張開' in a.hand_presets)
chk('握拳數值正確', a.hand_presets.get('握拳', {}).get('pos') == [102, 18, 10, 10, 10, 10],
    str(a.hand_presets.get('握拳')))

print('\n=== side 解析 ===')
chk('both -> 兩手', a._hand_sides('both') == ['left', 'right'])
chk('left -> 單手', a._hand_sides('left') == ['left'])
try:
    a._hand_sides('middle'); chk('亂填要擋下來', False)
except ValueError as e:
    chk('亂填要擋下來', True, str(e)[:34])

print('\n=== setHand ===')
sent.clear(); a.setHand('both', [250]*6)
chk('both 發兩筆', len(sent) == 2, str(len(sent)))
chk('沒給速度就不帶 speed', 'speed' not in sent[0], str(sent[0]))
sent.clear(); a.setHand('left', [1,2,3,4,5,6], [10]*6)
chk('有給速度就帶上', sent[0].get('speed') == [10]*6, str(sent[0]))
try:
    a.setHand('left', [1,2,3]); chk('位置長度不對要擋', False)
except ValueError: chk('位置長度不對要擋', True)

print('\n=== sendHandMotor：其他軸要填 -1 ===')
sent.clear(); a.sendHandMotor('right', 3, 0)
chk('只動第 3 軸', sent[0]['position'] == [-1,-1,0,-1,-1,-1], str(sent[0]['position']))
sent.clear(); a.sendHandMotor('right', 3, 0, 50)
chk('速度只改第 3 軸、其餘 0 沿用', sent[0]['speed'] == [0,0,50,0,0,0], str(sent[0]['speed']))
try:
    a.sendHandMotor('left', 7, 0); chk('關節編號要 1~6', False)
except ValueError: chk('關節編號要 1~6', True)

print('\n=== handPreset 用共用定義 ===')
sent.clear(); a.handPreset('握拳', 'left')
chk('位置來自 JSON', sent[0]['position'] == [102,18,10,10,10,10], str(sent[0]['position']))
chk('速度也來自 JSON', sent[0]['speed'] == [100,200,200,200,200,200], str(sent[0]['speed']))
try:
    a.handPreset('不存在'); chk('沒有的手勢要報錯', False)
except KeyError: chk('沒有的手勢要報錯', True)

print('\n=== handGrip ===')
sent.clear(); a.handGrip('left', 250)
g = sent[0]
chk('預設 each / 目標全 0', g['mode'] == 'each' and g['target'] == [0]*6, str(g)[:70])
chk('門檻傳對', g['threshold'] == 250)
try:
    a.handGrip('left', 250, mode='bogus'); chk('模式亂填要擋', False)
except ValueError: chk('模式亂填要擋', True)

print('\n=== 其他指令形狀 ===')
sent.clear(); a.handStop();  chk('handStop', sent[0] == {'cmd':'stop'}, str(sent[0]))
sent.clear(); a.handTouch(); chk('handTouch', sent[0] == {'cmd':'touch','on':True}, str(sent[0]))
sent.clear(); a.handPlay(left=10)
chk('handPlay', sent[0]['left'] == 10 and sent[0]['action'] == 'start', str(sent[0]))
try:
    a.handPlay(); chk('左右都沒給要擋', False)
except ValueError: chk('左右都沒給要擋', True)

print('\n=== _hand_status_callback ===')
a.hand_mode='x'; a.hand_online={'left':False,'right':False}
a.hand_position={'left':None,'right':None}; a.hand_target={'left':None,'right':None}
a.hand_temperature={'left':None,'right':None}; a.hand_mass={'left':None,'right':None}
a.hand_grip_active=False; a.hand_grip_reason=''; a.hand_play_active=False
a.hand_strategy=''; a.hand_touch_hold=False
a._hand_status_callback(types.SimpleNamespace(data=json.dumps({
    'mode':'grip','strategy':'mar','touch_hold':True,
    'hands':{'left':{'online':True,'position':[1,2,3,4,5,6],'mass':[10,0,0,0,0]}},
    'grip':{'active':True,'reason':''},'play':{'active':False}})))
chk('模式同步', a.hand_mode == 'grip')
chk('策略同步', a.hand_strategy == 'mar')
chk('左手位置同步', a.hand_position['left'] == [1,2,3,4,5,6])
chk('右手沒資料就是 None', a.hand_position['right'] is None)
chk('壓力同步', a.hand_mass['left'] == [10,0,0,0,0])
chk('夾持中', a.hand_grip_active is True)
a._hand_status_callback(types.SimpleNamespace(data='not json'))
chk('壞 JSON 不會炸', a.hand_mode == 'grip')

print('\n' + ('全部通過' if not fails else f'{len(fails)} 項失敗: ' + ', '.join(fails)))
sys.exit(0 if not fails else 1)
