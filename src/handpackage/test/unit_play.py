#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""播放與 sector 套用的單元測試：在行程內直接驅動節點物件，不經過 ROS 訊息。

_apply_sector / _play_tick / _start_players 這幾段不需要 nstc_msgs（只有那組
動作檔的 topic/service 需要），所以即使在沒編 nstc_msgs 的機器上也測得到。
做法是自己建一個 HandPackage、手動灌假的回讀位置和 sector，然後一步一步呼叫
_play_tick，檢查 tgt 有沒有被正確改寫。

用法：
    python3 src/handpackage/test/unit_play.py
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import rclpy                                                    # noqa: E402

from handpackage import hand_config as C                         # noqa: E402
from handpackage.handpackage import HandPackage                  # noqa: E402

fails = []


def chk(label, cond, extra=''):
    print(('  PASS  ' if cond else '  FAIL  ') + label + (f'   {extra}' if extra else ''))
    if not cond:
        fails.append(label)


def interleave(spd, pos):
    """speed/pos 交錯成 24 個值，與網頁送出的 payload 同格式。"""
    out = []
    for i in range(C.NUM_JOINTS_BOTH):
        out += [spd[i], pos[i]]
    return out


rclpy.init()
node = HandPackage()

# 假的回讀位置，讓兩手都算在線
for side in C.HANDS:
    node.hw[side]['position'] = [100] * C.NUM_JOINTS
    node.hw[side]['stamp'] = time.time()

print('\n=== 1. 絕對 sector ===')
# L 全 200、R 前三軸 50 其餘 -1（不動）
pos = [200] * 6 + [50, 50, 50, C.POS_HOLD, C.POS_HOLD, C.POS_HOLD]
node.store.sectors['10'] = {'opcode': C.OPCODE_ABS,
                            'data': interleave([200] * 12, pos)}
node._apply_sector('left', '10')
chk('左手取前 6 欄', node.tgt['left']['pos'] == [200] * 6, str(node.tgt['left']['pos']))
node._apply_sector('right', '10')
chk('右手取後 6 欄，-1 換成實際位置 100',
    node.tgt['right']['pos'] == [50, 50, 50, 100, 100, 100],
    str(node.tgt['right']['pos']))
chk('速度有被設定', node.tgt['left']['spd'] == [200] * 6, str(node.tgt['left']['spd']))

print('\n=== 2. 相對 sector（0 = 不動，基準是上次下的命令）===')
node.tgt['left']['pos'] = [120] * 6
delta = [10, -10, 0, 5, 0, -5] + [0] * 6
node.store.sectors['11'] = {'opcode': C.OPCODE_REL,
                            'data': interleave([0] * 12, delta)}
node._apply_sector('left', '11')
chk('增減量套在上次命令上、0 的軸不動',
    node.tgt['left']['pos'] == [130, 110, 120, 125, 120, 115],
    str(node.tgt['left']['pos']))
node.tgt['left']['pos'] = [250] * 6
node.store.sectors['12'] = {'opcode': C.OPCODE_REL,
                            'data': interleave([0] * 12, [50] * 6 + [0] * 6)}
node._apply_sector('left', '12')
chk('加到超過 255 會被夾住', node.tgt['left']['pos'] == [255] * 6,
    str(node.tgt['left']['pos']))

print('\n=== 3. 找不到的 sector / id 0 不應該改到任何東西 ===')
node.tgt['left']['pos'] = [77] * 6
node._apply_sector('left', '999')
chk('找不到就不動', node.tgt['left']['pos'] == [77] * 6, str(node.tgt['left']['pos']))
node._apply_sector('left', '0')
chk('id 0 跳過（與全身一致）', node.tgt['left']['pos'] == [77] * 6)

print('\n=== 4. _seq_of：把 motionlist 切成 (id, delay) 並濾掉補零 ===')
ml = [10, 500, 11, 800, 12, 300] + [0] * 34
node.store.sectors['70'] = {'opcode': C.OPCODE_MOTIONLIST, 'data': ml}
seq = node._seq_of('70')
chk('切出 3 步', seq == [(10, 500), (11, 800), (12, 300)], str(seq))
chk('非 MotionList 回 None', node._seq_of('10') is None)
chk('不存在的回 None', node._seq_of('999') is None)

print('\n=== 5. 雙手同步播同一序列 ===')
node.store.sectors['71'] = {'opcode': C.OPCODE_MOTIONLIST,
                            'data': [10, 200, 12, 200] + [0] * 36}
ok = node._start_players({'left': '70', 'right': '70'}, loop=1)
chk('啟動成功', ok)
chk('模式變 motion', node.mode == 'motion', node.mode)
chk('兩個播放器都 active',
    node.players['left'].active and node.players['right'].active)
chk('status 有進度', node._play_status()['left']['total'] == 3,
    str(node._play_status()['left']))
node.stop_play('測試')
chk('停止後都不 active',
    not node.players['left'].active and not node.players['right'].active)

print('\n=== 6. 左右手各播不同序列（獨立時間軸）===')
node.tgt['left']['pos'] = [100] * 6
node.tgt['right']['pos'] = [100] * 6
node._start_players({'left': '70', 'right': '71'}, loop=1)
chk('左手 3 步', node.players['left'].status()['total'] == 3)
chk('右手 2 步', node.players['right'].status()['total'] == 2)

t = time.time()
node._play_tick(t)                     # 兩手各做第一步（都是 sector 10）
chk('第一步：左手套到 200', node.tgt['left']['pos'] == [200] * 6,
    str(node.tgt['left']['pos']))
chk('第一步：右手套到 50/實際值',
    node.tgt['right']['pos'] == [50, 50, 50, 100, 100, 100],
    str(node.tgt['right']['pos']))

node.tgt['left']['pos'] = [120] * 6
node._play_tick(t + 0.55)              # 左手第二步 = sector 11（相對）
chk('左手 500ms 後走第二步',
    node.tgt['left']['pos'] == [130, 110, 120, 125, 120, 115],
    str(node.tgt['left']['pos']))
chk('右手還沒到時間（delay 200ms 已過，走第二步 sector 12）',
    node.players['right'].status()['step'] == 2,
    str(node.players['right'].status()))

print('\n=== 7. 單手播放：另一手完全不要被動到 ===')
node.stop_play('換測試')
node.tgt['left']['pos'] = [60] * 6
node.tgt['right']['pos'] = [61] * 6
node._start_players({'left': '70', 'right': None}, loop=1)
chk('只有左手 active',
    node.players['left'].active and not node.players['right'].active)
t = time.time()
node._play_tick(t)
chk('左手有動', node.tgt['left']['pos'] == [200] * 6, str(node.tgt['left']['pos']))
chk('右手完全沒變', node.tgt['right']['pos'] == [61] * 6, str(node.tgt['right']['pos']))

print('\n=== 8. 播完會自己回 manual ===')
node.stop_play('換測試')
node.store.sectors['72'] = {'opcode': C.OPCODE_MOTIONLIST,
                            'data': [10, 100] + [0] * 38}
node._start_players({'left': '72'}, loop=1)
t = time.time()
node._play_tick(t)                     # 第一步
chk('播放中', node.play.get('active') is True)
node._play_tick(t + 0.15)              # 序列結束 -> DONE
node._play_tick(t + 0.20)              # 下一個 tick 偵測到沒人 active
chk('播完自動回 manual', node.mode == 'manual', node.mode)
chk('play 狀態收乾淨', node.play.get('active') is False)

print('\n=== 9. loop=0 無限循環 ===')
node._start_players({'left': '72'}, loop=0)
t = time.time()
for k in range(5):
    node._play_tick(t + k * 0.15)
chk('無限循環還在跑', node.players['left'].active is True)
chk('已經跑完好幾圈', node.players['left'].status()['done_loops'] >= 2,
    str(node.players['left'].status()))
node.stop_play('收尾')

print('\n=== 10. MotionList 裡引用 MotionList 要被拒絕（不能遞迴）===')
node.store.sectors['73'] = {'opcode': C.OPCODE_MOTIONLIST,
                            'data': [70, 100] + [0] * 38}
node.tgt['left']['pos'] = [88] * 6
node._start_players({'left': '73'}, loop=1)
node._play_tick(time.time())
chk('沒有被套用（位置不變）', node.tgt['left']['pos'] == [88] * 6,
    str(node.tgt['left']['pos']))
node.stop_play('收尾')

print('\n=== 11. _cmd_play：網頁送來的 payload 原樣 ===')
node.stop_play('換測試')
node.tgt['left']['pos'] = [90] * 6
node.tgt['right']['pos'] = [90] * 6
# 這就是 HandMotionInterface.html 的 doPlay() 送出的形狀
node._cmd_play({'cmd': 'play', 'action': 'start', 'loop': 1,
                'left': 70, 'right': 71})
chk('兩手都開始播', node.players['left'].active and node.players['right'].active)
chk('模式變 motion', node.mode == 'motion', node.mode)
node._cmd_play({'cmd': 'play', 'action': 'stop'})
chk('stop 停得掉', node.play.get('active') is False)
chk('停止後回 manual', node.mode == 'manual', node.mode)
node._cmd_play({'cmd': 'play', 'action': 'start', 'loop': 1,
                'left': 70, 'right': None})
chk('right=null 只播左手',
    node.players['left'].active and not node.players['right'].active)
node.stop_play('收尾')
node._cmd_play({'cmd': 'play', 'action': 'start', 'loop': 1,
                'left': 999, 'right': None})
chk('不存在的序列不會啟動', node.play.get('active') is False)

print('\n=== 12. 手動介入會中斷播放 ===')
node._start_players({'left': '70', 'right': '70'}, loop=0)
chk('播放中', node.play.get('active') is True)
node._cmd_manual({'cmd': 'manual', 'hand': 'left', 'position': [33] * 6})
chk('播放被中斷', node.play.get('active') is False)
chk('模式回 manual', node.mode == 'manual', node.mode)
chk('手動值生效', node.tgt['left']['pos'] == [33] * 6, str(node.tgt['left']['pos']))

print('\n=== 13. MotionList 裡的夾持步驟 ===')
# 讓兩手看起來有壓感、而且壓力資料是新的
for side in C.HANDS:
    node.hw[side]['info'] = {'touch_type': 4, 'motor_temperature': [40] * 6}
    node.hw[side]['mass'] = [0] * 5
    node.hw[side]['mass_stamp'] = time.time()
    node.hw[side]['position'] = [255] * C.NUM_JOINTS
    node.hw[side]['stamp'] = time.time()
    node.tgt[side]['pos'] = [255] * C.NUM_JOINTS

# 一個帶停止條件的絕對姿態（整隻手收到 0），門檻 800、模式 each
node.store.sectors['20'] = {
    'opcode': C.OPCODE_ABS,
    'data': interleave([200] * 12, [0] * 12),
    'stop': ('each', 800),
}
# 一個普通姿態
node.store.sectors['21'] = {
    'opcode': C.OPCODE_ABS,
    'data': interleave([200] * 12, [128] * 12),
    'stop': None,
}
# 清單：先夾持（delay 2000 當逾時上限），再做一個普通姿態
node.store.sectors['80'] = {
    'opcode': C.OPCODE_MOTIONLIST,
    'data': [20, 2000, 21, 200] + [0] * 36,
    'stop': None,
}

node.stop_play('換測試'); node.stop_grip('換測試')
node._start_players({'left': '80'}, loop=1)
t = time.time()
node._play_tick(t)
chk('第一步進入夾持等待', node.players['left'].grip_wait is True)
chk('夾持已啟動', node.grip.get('active') is True, str(node.grip.get('active')))
chk('只有左手在夾', node.grip.get('hands') == ['left'], str(node.grip.get('hands')))
chk('模式取自 stop 條件', node.grip['mode'].get('left') == 'each',
    str(node.grip.get('mode')))
chk('門檻取自 stop 條件', node.grip['threshold'].get('left') == [800] * 5,
    str(node.grip['threshold'].get('left')))
chk('逾時 = 該步的 delay（2000ms → 2s）',
    abs(node.grip['timeout']['left'] - 2.0) < 0.01,
    str(node.grip['timeout'].get('left')))
chk('夾持目標 = 該步的位置', node.grip['target']['left'] == [0] * C.NUM_JOINTS,
    str(node.grip['target'].get('left')))

print('\n   -- 等壓力資料階段不會往下走 --')
node.players['left'].next_t = 0      # 就算時間到了也不該前進
node._play_tick(time.time())
chk('還卡在夾持步驟', node.players['left'].status()['step'] == 1,
    str(node.players['left'].status()))

print('\n   -- 壓力到門檻 → 夾持結束 → 播放往下走 --')
node.grip['phase']['left'] = 'closing'
for side in C.HANDS:
    node.hw[side]['mass_stamp'] = time.time()
node.hw['left']['mass'] = [900] * 5          # 五指全部到門檻
node._grip_tick()
chk('夾持結束', node.grip.get('active') is False, str(node.grip.get('active')))
chk('播放器被放行', node.players['left'].grip_wait is False)
node.tgt['left']['pos'] = [111] * C.NUM_JOINTS
node._play_tick(time.time())
chk('接著做第二步（普通姿態 128）',
    node.tgt['left']['pos'] == [128] * C.NUM_JOINTS,
    str(node.tgt['left']['pos']))
node.stop_play('收尾'); node.stop_grip('收尾')

print('\n=== 14. 兩手各自夾不同的東西 ===')
for side in C.HANDS:
    node.hw[side]['mass'] = [0] * 5
    node.hw[side]['mass_stamp'] = time.time()
    node.hw[side]['position'] = [255] * C.NUM_JOINTS
    node.tgt[side]['pos'] = [255] * C.NUM_JOINTS
node.store.sectors['22'] = {
    'opcode': C.OPCODE_ABS,
    'data': interleave([200] * 12, [0] * 12),
    'stop': ('any', 500),
}
node.store.sectors['81'] = {
    'opcode': C.OPCODE_MOTIONLIST, 'data': [22, 3000] + [0] * 38, 'stop': None,
}
node._start_players({'left': '80', 'right': '81'}, loop=1)
node._play_tick(time.time())
chk('兩手都在夾', sorted(node.grip.get('hands', [])) == ['left', 'right'],
    str(node.grip.get('hands')))
chk('左手 each:800', node.grip['mode']['left'] == 'each'
    and node.grip['threshold']['left'] == [800] * 5, str(node.grip['mode']))
chk('右手 any:500', node.grip['mode']['right'] == 'any'
    and node.grip['threshold']['right'] == [500] * 5,
    str(node.grip['threshold'].get('right')))
chk('兩手逾時各自不同（2s / 3s）',
    abs(node.grip['timeout']['left'] - 2.0) < 0.01
    and abs(node.grip['timeout']['right'] - 3.0) < 0.01,
    str(node.grip['timeout']))

print('\n   -- 只有右手到門檻，左手要繼續夾 --')
for side in C.HANDS:
    node.grip['phase'][side] = 'closing'
    node.hw[side]['mass_stamp'] = time.time()
node.hw['right']['mass'] = [600] * 5
node._grip_tick()
chk('右手結束', 'right' in node.grip.get('done', {}), str(node.grip.get('done')))
chk('左手還在夾', node.grip.get('hands') == ['left'], str(node.grip.get('hands')))
chk('右手的播放器被放行', node.players['right'].grip_wait is False)
chk('左手的播放器還在等', node.players['left'].grip_wait is True)
node.stop_play('收尾'); node.stop_grip('收尾')

print('\n=== 15. 相對列也能帶停止條件 ===')
# 使用者要的是「從現在的姿勢再往內收 40，收到壓力就停」。
# 節點要先把相對位移解成絕對目標，才有辦法交給夾持迴路。
node.stop_play('換測試'); node.stop_grip('換測試')
for side in C.HANDS:
    node.hw[side]['info'] = {'touch_type': 4, 'motor_temperature': [40] * 6}
    node.hw[side]['mass'] = [0] * 5
    node.hw[side]['mass_stamp'] = time.time()
    node.hw[side]['position'] = [200] * C.NUM_JOINTS
    node.hw[side]['stamp'] = time.time()
    node.tgt[side]['pos'] = [180] * C.NUM_JOINTS      # 基準是「上次下的命令」

# 相對 -40，第 3 軸位移 0（這軸不動）。步長 9
node.store.sectors['30'] = {
    'opcode': C.OPCODE_REL,
    'data': interleave([200] * 12, [-40, -40, 0, -40, -40, -40] * 2),
    'stop': ('any', 700, 9),
}
node._apply_sector('left', '30', delay_ms=1500)
chk('相對列也會起夾持', node.grip.get('active') is True, str(node.grip.get('active')))
chk('模式是 any', node.grip['mode'].get('left') == 'any', str(node.grip.get('mode')))
# 目標 = 上次命令 180 + (-40) = 140；位移 0 的那軸維持基準 180，
# 不能變成量測值 200（那就把回授誤差灌進來了）
chk('目標 = 上次命令 + 位移，位移 0 的軸維持基準',
    node.grip['target']['left'] == [140, 140, 180, 140, 140, 140],
    str(node.grip['target'].get('left')))
chk('步長取自 stop 條件（每手一份）', node.grip['step'].get('left') == 9,
    str(node.grip.get('step')))
chk('逾時 = 該步的 delay（1500ms）',
    abs(node.grip['timeout']['left'] - 1.5) < 0.01,
    str(node.grip['timeout'].get('left')))

print('\n   -- 舊的兩元組 stop 要還能用（步長取預設）--')
node.stop_grip('換測試')
node.store.sectors['31'] = {
    'opcode': C.OPCODE_ABS,
    'data': interleave([200] * 12, [0] * 12),
    'stop': ('each', 800),          # 只有兩個元素，沒有步長
}
node._apply_sector('right', '31', delay_ms=1000)
chk('兩元組不會炸', node.grip.get('active') is True, str(node.grip.get('active')))
chk('步長用預設', node.grip['step'].get('right') == C.GRIP_MIN_STEP + 1,
    str(node.grip.get('step')))

print('\n   -- 兩手步長各自獨立 --')
node.add_grip_for('left', 'each', [800] * 5, [0] * C.NUM_JOINTS, 2.0, step=20)
chk('左手 20 / 右手 4', node.grip['step'] == {'right': C.GRIP_MIN_STEP + 1, 'left': 20},
    str(node.grip.get('step')))
node.stop_play('收尾'); node.stop_grip('收尾')

print('\n=== 16. execute 做完要回報 /hand/Callback ===')
# 網頁在 execute 期間會把按鈕鎖起來，就是等這一筆才解開。
# 原本單筆姿態 execute 完全不回報，網頁只能等 6 秒的安全網才解鎖 ——
# 現象就是「execute 完等超級久才解開鎖定」。


class CbRec:
    def __init__(self):
        self.sent = []

    def publish(self, m):
        self.sent.append(bool(m.data))


class Int16Like:
    def __init__(self, v):
        self.data = v


node.stop_play('換測試'); node.stop_grip('換測試')
rec = node.pub_callback = CbRec()
for side in C.HANDS:
    node.hw[side]['position'] = [200] * C.NUM_JOINTS
    node.hw[side]['stamp'] = time.time()
    node.hw[side]['mass'] = [0] * 5
    node.hw[side]['mass_stamp'] = time.time()
    node.hw[side]['info'] = {'touch_type': 4, 'motor_temperature': [40] * 6}

print('\n   -- 單筆姿態：位置下去就算做完，馬上回報 --')
node.store.sectors['40'] = {'opcode': C.OPCODE_ABS,
                            'data': interleave([200] * 12, [128] * 12), 'stop': None}
node._on_sector_execute(Int16Like(40))
chk('姿態 execute 立刻回報', rec.sent == [True], str(rec.sent))

print('\n   -- 找不到 sector：也要回報（失敗），不然網頁會一直鎖著 --')
rec.sent = []
node._on_sector_execute(Int16Like(999))
chk('找不到也會回報失敗', rec.sent == [False], str(rec.sent))

print('\n   -- 帶停止條件：要等夾持結束才回報 --')
rec.sent = []
node.store.sectors['41'] = {'opcode': C.OPCODE_ABS,
                            'data': interleave([200] * 12, [0] * 12),
                            'stop': ('each', 250, 4)}
node._on_sector_execute(Int16Like(41))
chk('夾持中還不回報', rec.sent == [], str(rec.sent))
chk('夾持有起來', node.grip.get('active') is True, str(node.grip.get('active')))
for side in C.HANDS:
    node.grip['phase'][side] = 'closing'
    node.hw[side]['mass_stamp'] = time.time()
    node.hw[side]['mass'] = [500] * 5
node._grip_tick()
chk('夾持結束才回報', rec.sent == [True], str(rec.sent))
chk('只回報一次', len(rec.sent) == 1, str(rec.sent))
node.stop_play('收尾'); node.stop_grip('收尾')

print('\n=== 17. 策略切換（/location）===')
# /location 上同時跑著兩種格式：ImageProcess 送相對路徑、MotionControl 送
# 絕對路徑。兩種都要解析得出來，不然切策略之後動作檔會存讀到錯的目錄。


class LocLike:
    def __init__(self, d):
        self.data = d


for raw, want in [
    ('ar/Parameter', 'ar'),
    ('/ar/Parameter', 'ar'),
    ('mar/Parameter', 'mar'),
    ('/home/iclab/ros2_adult/src/strategy/strategy/mar/Parameter', 'mar'),
    ('bb', 'bb'),
]:
    chk(f'解析 {raw!r}', node._parse_strategy(raw) == want, node._parse_strategy(raw))

for bad in ('', None, '/Parameter'):
    chk(f'看不懂就回空字串 {bad!r}', node._parse_strategy(bad) == '',
        repr(node._parse_strategy(bad)))

print('\n   -- 收到 /location 要真的換掉策略與 sector 目錄 --')
node.location_folder = 'ar'
before = node.store.motion_dir()
node._on_location(LocLike('/home/iclab/ros2_adult/src/strategy/strategy/mar/Parameter'))
chk('策略換成 mar', node.location_folder == 'mar', node.location_folder)
chk('動作檔目錄跟著換',
    node.store.motion_dir().endswith('/mar/Parameter/hand')
    and node.store.motion_dir() != before, node.store.motion_dir())

print('\n   -- 同一個策略再送一次不該重載 --')
node.location_folder = 'mar'
node._on_location(LocLike('mar/Parameter'))
chk('沒變就不動', node.location_folder == 'mar', node.location_folder)

print('\n   -- 看不懂的內容要沿用舊的，不能把策略清掉 --')
node._on_location(LocLike(''))
chk('壞訊息不會弄壞策略', node.location_folder == 'mar', node.location_folder)
node.location_folder = 'ar'

node.destroy_node()
print('\n' + ('全部通過' if not fails else f'{len(fails)} 項失敗:\n  - '
               + '\n  - '.join(fails)))
sys.stdout.flush()
# 乾淨關閉 DDS，不要用 os._exit —— 強制退出會留下幽靈
# participant，害下一支測試的 discovery 要等逾時才接得上
try:
    _n.destroy_node()
except Exception:
    pass
rclpy.try_shutdown()
sys.exit(0 if not fails else 1)
