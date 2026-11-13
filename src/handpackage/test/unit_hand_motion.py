#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hand_motion.py 的純 Python 單元測試：不需要 ROS、不需要硬體、不需要節點。

測檔案讀寫、sector 快取、位元封包解析、播放器時序、單手切半。
會寫在暫存目錄，不會動到真的策略資料夾。

用法：
    python3 src/handpackage/test/unit_hand_motion.py
"""
import os
import shutil
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from handpackage import hand_config as C                       # noqa: E402
from handpackage.hand_motion import (MotionStore, PacketParser,  # noqa: E402
                                     Player, split_pose)


class Log:
    def info(self, m): pass
    def warn(self, m): pass
    def error(self, m): print('   [log.error]', m)


fails = []


def chk(label, cond, extra=''):
    print(('  PASS  ' if cond else '  FAIL  ') + label + (f'   {extra}' if extra else ''))
    if not cond:
        fails.append(label)


root = tempfile.mkdtemp(prefix='handmotion_')
log = Log()
store = MotionStore(log, lambda: 'ar', root=root)

print('\n=== 1. 路徑 ===')
chk('motion_dir 在 Parameter/hand', store.motion_dir().endswith('ar/Parameter/hand'),
    store.motion_dir())
chk('sector_dir 在 Parameter/hand/sector',
    store.sector_dir().endswith('ar/Parameter/hand/sector'), store.sector_dir())
chk('借讀別的策略', store.motion_dir('sp').endswith('sp/Parameter/hand'))

print('\n=== 2. 位元封包解析（12 對，長度跟全身不同）===')
pp = PacketParser(log)
abs_payload = []
for j in range(12):
    abs_payload += [200, 10 * j]          # speed, pos
pkt = list(C.PKT_HEAD) + [C.OPCODE_ABS] + abs_payload + list(C.PKT_TAIL)
chk('絕對封包長度 29', len(pkt) == 29, str(len(pkt)))
got = pp.feed(pkt)
chk('解出一個封包', len(got) == 1, str(len(got)))
chk('opcode 正確', got and got[0][0] == C.OPCODE_ABS)
chk('payload 24 個值', got and len(got[0][1]) == 24, str(len(got[0][1]) if got else 0))
chk('payload 內容正確', got and got[0][1] == abs_payload)

ml_payload = [10, 500, 11, 800] + [0] * 36
pkt2 = list(C.PKT_HEAD) + [C.OPCODE_MOTIONLIST] + ml_payload + list(C.PKT_TAIL)
chk('MotionList 封包長度 45', len(pkt2) == 45, str(len(pkt2)))
got2 = pp.feed(pkt2)
chk('MotionList 解得開', len(got2) == 1 and len(got2[0][1]) == 40)

print('\n=== 3. 垃圾資料不能讓解析器卡死 ===')
pp2 = PacketParser(log)
chk('前面有垃圾也能找到封包', len(pp2.feed([1, 2, 3] + pkt)) == 1)
pp3 = PacketParser(log)
chk('尾碼錯就丟掉', len(pp3.feed(list(C.PKT_HEAD) + [C.OPCODE_ABS]
                                + abs_payload + [0, 0])) == 0)
pp4 = PacketParser(log)
half = pkt[:15]
chk('封包切兩半也能拼回來', len(pp4.feed(half)) == 0 and len(pp4.feed(pkt[15:])) == 1)
pp5 = PacketParser(log)
chk('未知 opcode 不會卡住', len(pp5.feed([83, 84, 99] + pkt)) == 1)

print('\n=== 4. sector 存 / 讀 / 重載 ===')
chk('存絕對 sector', store.save_sector('10', C.OPCODE_ABS, abs_payload))
chk('記憶體裡有了', store.has_sector('10'))
chk('數字 id 也查得到', store.has_sector(10))
chk('.ini 真的寫出來', os.path.isfile(os.path.join(store.sector_dir(), '10.ini')))
chk('存 MotionList sector', store.save_sector('70', C.OPCODE_MOTIONLIST, ml_payload))
rel_payload = [50, 0] * 12
rel_payload[1] = 5
chk('存相對 sector', store.save_sector('11', C.OPCODE_REL, rel_payload))

# 絕對位置全 0 = 握拳，是合法姿勢，一定要存得進去
zero = [0] * 24
chk('絕對位置全 0（握拳）要存得進去', store.save_sector('99', C.OPCODE_ABS, zero) is True)
chk('握拳有進記憶體', store.has_sector('99'))
# 相對位移全 0 代表這一步什麼都不做，才當誤觸
chk('相對位移全 0 當誤觸不存', store.save_sector('98', C.OPCODE_REL, zero) is False)
chk('誤觸的沒進記憶體', not store.has_sector('98'))

store2 = MotionStore(log, lambda: 'ar', root=root)
n = store2.load_all()
chk('重開後載回 4 筆', n == 4, str(n))
chk('絕對 sector 資料一致', store2.get_sector('10')['data'] == abs_payload)
chk('opcode 還原正確（絕對）', store2.get_sector('10')['opcode'] == C.OPCODE_ABS)
chk('opcode 還原正確（相對）', store2.get_sector('11')['opcode'] == C.OPCODE_REL)
chk('opcode 還原正確（清單）',
    store2.get_sector('70')['opcode'] == C.OPCODE_MOTIONLIST)

print('\n=== 5. 編輯檔存 / 讀（五種 motionstate）===')
rows = [
    {'savestate': 0, 'motionstate': C.MOTIONSTATE_MOTIONLIST, 'id': 70,
     'item_name': '抓取流程', 'motionlist': [10, 500, 11, 800] + [0] * 36,
     'motordata': []},
    {'savestate': 0, 'motionstate': C.MOTIONSTATE_ABS_POS, 'id': 10,
     'item_name': '張開', 'motionlist': [], 'motordata': [250] * 12},
    {'savestate': 0, 'motionstate': C.MOTIONSTATE_ABS_SPD, 'id': 10,
     'item_name': '張開', 'motionlist': [], 'motordata': [200] * 12},
    {'savestate': 0, 'motionstate': C.MOTIONSTATE_REL_POS, 'id': 11,
     'item_name': '微調', 'motionlist': [], 'motordata': [0, 5] + [0] * 10},
    {'savestate': 0, 'motionstate': C.MOTIONSTATE_REL_SPD, 'id': 11,
     'item_name': '微調', 'motionlist': [], 'motordata': [120] * 12},
]
for r in rows:
    store.buffer_row('mytest', r)
chk('寫檔成功', store.write_ini('mytest'))
path = os.path.join(store.motion_dir(), 'mytest.ini')
chk('.ini 存在', os.path.isfile(path))

out = store.read_ini('mytest')
chk('讀檔成功', out['ok'])
chk('5 筆', out['cnt'] == 5, str(out['cnt']))
chk('motionstate 都在', sorted(out['motionstate']) == [0, 1, 2, 3, 4],
    str(sorted(out['motionstate'])))
chk('item_name 有保留', '抓取流程' in out['item_names'], str(out['item_names']))
# MotionList 列吐 40 個，另外 4 筆姿態列各吐 3 個停止條件（沒設就是 0,0,0）
chk('motionlist = 40 + 4x3', len(out['motionlist']) == 40 + 4 * 3,
    str(len(out['motionlist'])))
chk('沒設停止條件的姿態列是 0,0,0', out['motionlist'][40:] == [0] * 12,
    str(out['motionlist'][40:]))
chk('absolutedata 24 個值（兩列 x 12）', len(out['absolutedata']) == 24,
    str(len(out['absolutedata'])))
chk('relativedata 24 個值（兩列 x 12）', len(out['relativedata']) == 24,
    str(len(out['relativedata'])))
chk('讀不存在的檔不會炸', store.read_ini('nope')['ok'] is False)
chk('list_files 找得到', 'mytest' in store.list_files(), str(store.list_files()))

print('\n=== 6. item_name 夾帶停止條件（Q-A 的編碼）===')
store.buffer_row('stoptest', {
    'savestate': 0, 'motionstate': C.MOTIONSTATE_ABS_POS, 'id': 5,
    'item_name': '握到夾緊|stop=any:800', 'motionlist': [], 'motordata': [0] * 12})
store.write_ini('stoptest')
o = store.read_ini('stoptest')
nm = o['item_names'][0]
chk('含 | 的名稱能原樣存讀', nm == '握到夾緊|stop=any:800', nm)
disp, _, meta = nm.partition(C.ITEM_NAME_SEP)
chk('切得出顯示名稱', disp == '握到夾緊', disp)
chk('切得出停止條件', meta == 'stop=any:800', meta)

print('\n=== 7. split_pose：單手只取自己那一半 ===')
data = []
for j in range(12):
    data += [100 + j, 10 + j]        # speed, pos
ls, lp = split_pose(data, 'left')
rs, rp = split_pose(data, 'right')
chk('左手取前 6 對', lp == [10, 11, 12, 13, 14, 15], str(lp))
chk('右手取後 6 對', rp == [16, 17, 18, 19, 20, 21], str(rp))
chk('速度也切對', ls == [100, 101, 102, 103, 104, 105], str(ls))
chk('資料不足時補 POS_HOLD',
    split_pose([1, 2], 'right')[1] == [C.POS_HOLD] * 6,
    str(split_pose([1, 2], 'right')[1]))

print('\n=== 8. 播放器時序（非阻塞）===')
p = Player('left', log)
chk('空序列不啟動', p.start([]) is False)
chk('沒啟動時 tick 回 None', p.tick(time.time()) is None)

p = Player('left', log)
chk('啟動成功', p.start([(10, 100), (11, 100), (12, 100)], loop=1))
t = time.time()
chk('第一步立刻執行', p.tick(t) == '10')
chk('還沒到時間不動', p.tick(t + 0.05) is None)
chk('100ms 後第二步', p.tick(t + 0.11) == '11')
chk('第三步', p.tick(t + 0.22) == '12')
chk('跑完回 DONE', p.tick(t + 0.33) == 'DONE')
chk('DONE 之後不再 active', p.active is False)

p = Player('left', log)
p.start([(10, 60)], loop=1)
t = time.time()
p.tick(t)
chk('delay 低於 50ms 會被抬到 50ms', p.tick(t + 0.03) is None)
chk('50ms 後才算下一步', p.tick(t + 0.06) == 'DONE')

print('\n=== 9. 播放器 loop ===')
p = Player('left', log)
p.start([(10, 100), (11, 100)], loop=2)
t = time.time()
seen = []
for k in range(12):
    r = p.tick(t + k * 0.11)
    if r:
        seen.append(r)
chk('兩圈 = 4 步 + DONE', seen == ['10', '11', '10', '11', 'DONE'], str(seen))

p = Player('right', log)
p.start([(10, 100)], loop=0)
t = time.time()
seen = []
for k in range(6):
    r = p.tick(t + k * 0.11)
    if r:
        seen.append(r)
chk('loop=0 是無限', seen == ['10'] * 6 and p.active, str(seen))
p.stop()
chk('stop 之後不 active', p.active is False)

print('\n=== 10. 播放器 status ===')
p = Player('left', log)
p.start([(10, 100), (11, 100)], loop=3, name='70')
st = p.status()
chk('status 有 name/total/loop',
    st['name'] == '70' and st['total'] == 2 and st['loop'] == 3, str(st))

print('\n=== 11. 網頁 Save -> 寫檔 -> Read -> 網頁重建模型（offset 對位）===')
# 這段是整個功能最容易寫錯的地方：relativedata / absolutedata 是把所有同類別的
# 列依檔案順序攤平成一維，網頁要靠各自的 offset 往前走才切得對。
# 下面完整模擬一次：網頁的 doSave 送什麼、節點怎麼寫、網頁的 parseRead 怎麼切回來。
PAIRS, NJ = C.MOTIONLIST_PAIRS, C.NUM_JOINTS_BOTH

page_model = {
    'list': [
        {'id': 70, 'name': '流程A', 'pairs': [[10, 500], [11, 800]] + [[0, 0]] * (PAIRS - 2)},
        {'id': 71, 'name': '流程B', 'pairs': [[12, 300]] + [[0, 0]] * (PAIRS - 1)},
    ],
    'rel': [
        {'id': 11, 'name': '微調1', 'pos': list(range(1, 13)),
         'spd': [100 + i for i in range(12)]},
        {'id': 12, 'name': '微調2', 'pos': list(range(21, 33)),
         'spd': [120 + i for i in range(12)]},
    ],
    'abs': [
        {'id': 10, 'name': '張開', 'pos': [250] * 12, 'spd': [200] * 12},
        # 停止條件現在是獨立欄位，名稱可以正常取名（之前塞成 |stop=... 後綴，
        # 結果 Name 那一格就不能打字了）
        {'id': 13, 'name': '握拳 收到 900', 'pos': [0] * 6 + [-1] * 6,
         'spd': [150] * 12, 'stop': ('each', 900, 6)},
    ],
}

store3 = MotionStore(log, lambda: 'ar', root=root)
# 網頁 doSave 的順序：list 全部、rel 的 pos+spd、abs 的 pos+spd
for r in page_model['list']:
    flat = []
    for a, d in r['pairs']:
        flat += [a, d]
    store3.buffer_row('rt', {'savestate': 0, 'motionstate': C.MOTIONSTATE_MOTIONLIST,
                             'id': r['id'], 'item_name': r['name'],
                             'motionlist': flat, 'motordata': []})
STOP_NUM = {'each': 1, 'any': 2, 'anyhand': 3, 'hold': 4}


def triple(r):
    """網頁 stopTriple()：姿態列的 motionlist 固定三個值。"""
    st = r.get('stop')
    return [STOP_NUM[st[0]], st[1], st[2]] if st else [0, 0, 0]


for r in page_model['rel']:
    store3.buffer_row('rt', {'savestate': 0, 'motionstate': C.MOTIONSTATE_REL_POS,
                             'id': r['id'], 'item_name': r['name'],
                             'motionlist': triple(r), 'motordata': r['pos']})
    store3.buffer_row('rt', {'savestate': 0, 'motionstate': C.MOTIONSTATE_REL_SPD,
                             'id': r['id'], 'item_name': r['name'],
                             'motionlist': triple(r), 'motordata': r['spd']})
for r in page_model['abs']:
    store3.buffer_row('rt', {'savestate': 0, 'motionstate': C.MOTIONSTATE_ABS_POS,
                             'id': r['id'], 'item_name': r['name'],
                             'motionlist': triple(r), 'motordata': r['pos']})
    store3.buffer_row('rt', {'savestate': 0, 'motionstate': C.MOTIONSTATE_ABS_SPD,
                             'id': r['id'], 'item_name': r['name'],
                             'motionlist': triple(r), 'motordata': r['spd']})
chk('寫檔成功', store3.write_ini('rt'))

res = store3.read_ini('rt')
chk('讀回 10 筆', res['cnt'] == 10, str(res['cnt']))

# 以下完全照 HandMotionInterface.html 的 parseRead 實作
rebuilt = {'list': [], 'rel': [], 'abs': []}
oList = oRel = oAbs = 0


def pick(group, rid, name):
    for row in rebuilt[group]:
        if row['id'] == rid:
            if name:
                row['name'] = name
            return row
    row = {'id': rid, 'name': name, 'pos': [0] * NJ, 'spd': [0] * NJ, 'stop': None}
    rebuilt[group].append(row)
    return row


NUM_STOP = {1: 'each', 2: 'any', 3: 'anyhand', 4: 'hold'}


def to_stop(a, off):
    """網頁 tripleToStop()。"""
    m = NUM_STOP.get(a[off]) if off + 2 < len(a) else None
    return (m, a[off + 1], a[off + 2]) if m else None


for k in range(res['cnt']):
    ms, rid = res['motionstate'][k], res['id'][k]
    nm = res['item_names'][k]
    if nm == '-1':
        nm = ''
    if ms == C.MOTIONSTATE_MOTIONLIST:
        prs = [[res['motionlist'][oList + i * 2], res['motionlist'][oList + i * 2 + 1]]
               for i in range(PAIRS)]
        oList += PAIRS * 2
        rebuilt['list'].append({'id': rid, 'name': nm, 'pairs': prs})
    elif ms in (C.MOTIONSTATE_REL_POS, C.MOTIONSTATE_REL_SPD):
        row = pick('rel', rid, nm)
        row['pos' if ms == C.MOTIONSTATE_REL_POS else 'spd'] = \
            res['relativedata'][oRel:oRel + NJ]
        oRel += NJ
        st = to_stop(res['motionlist'], oList)
        if st:
            row['stop'] = st
        oList += 3
    else:
        row = pick('abs', rid, nm)
        row['pos' if ms == C.MOTIONSTATE_ABS_POS else 'spd'] = \
            res['absolutedata'][oAbs:oAbs + NJ]
        oAbs += NJ
        st = to_stop(res['motionlist'], oList)
        if st:
            row['stop'] = st
        oList += 3

chk('offset 剛好用完 motionlist', oList == len(res['motionlist']),
    f"{oList} vs {len(res['motionlist'])}")
chk('offset 剛好用完 relativedata', oRel == len(res['relativedata']),
    f"{oRel} vs {len(res['relativedata'])}")
chk('offset 剛好用完 absolutedata', oAbs == len(res['absolutedata']),
    f"{oAbs} vs {len(res['absolutedata'])}")

for grp in ('list', 'rel', 'abs'):
    chk(f'{grp} 列數一致', len(rebuilt[grp]) == len(page_model[grp]),
        f"{len(rebuilt[grp])} vs {len(page_model[grp])}")

chk('MotionList 的 pairs 完全還原',
    rebuilt['list'][0]['pairs'] == page_model['list'][0]['pairs'],
    str(rebuilt['list'][0]['pairs'][:3]))
chk('第二個 MotionList 沒被串錯',
    rebuilt['list'][1]['pairs'] == page_model['list'][1]['pairs'],
    str(rebuilt['list'][1]['pairs'][:2]))
for i, r in enumerate(page_model['rel']):
    chk(f"rel[{i}] pos 還原", rebuilt['rel'][i]['pos'] == r['pos'],
        str(rebuilt['rel'][i]['pos']))
    chk(f"rel[{i}] spd 還原", rebuilt['rel'][i]['spd'] == r['spd'])
for i, r in enumerate(page_model['abs']):
    chk(f"abs[{i}] pos 還原", rebuilt['abs'][i]['pos'] == r['pos'],
        str(rebuilt['abs'][i]['pos']))
    chk(f"abs[{i}] spd 還原", rebuilt['abs'][i]['spd'] == r['spd'])
chk('-1 哨符原樣保留', rebuilt['abs'][1]['pos'][6:] == [-1] * 6,
    str(rebuilt['abs'][1]['pos']))
chk('名稱可以正常取名（不再被 stop 佔走）',
    rebuilt['abs'][1]['name'] == '握拳 收到 900', rebuilt['abs'][1]['name'])
chk('停止條件三元組往返一致',
    rebuilt['abs'][1]['stop'] == ('each', 900, 6), str(rebuilt['abs'][1]['stop']))
chk('沒設停止條件的列是 None',
    rebuilt['abs'][0]['stop'] is None and rebuilt['rel'][0]['stop'] is None,
    f"abs0={rebuilt['abs'][0]['stop']} rel0={rebuilt['rel'][0]['stop']}")

print('\n=== 12. 欄位數不對的 .ini 不可以讓後面整個錯位 ===')
# 回報的現象：「RelativePosition 是空的、MotionList 有東西，MotionList 的內容
# 會顯示到 RelativePosition 裡」。原因是 read_ini 沒有把每一列的貢獻正規化成
# 固定長度，而網頁是用固定 stride（MotionList 40 / 停止條件 3 / 位置 12）
# 去切平行陣列的，所以只要有一列數量不對，從那一列之後全部錯位。
import configparser as _cp

store4 = MotionStore(log, lambda: 'ar', root=root)
_d = store4.motion_dir()
os.makedirs(_d, exist_ok=True)
_cfg = _cp.ConfigParser()
# 第 0 列：MotionList 給 44 個（超過 40）
_cfg['0'] = {'savestate': '0', 'motionstate': str(C.MOTIONSTATE_MOTIONLIST),
             'id': '70', 'item_name': '流程',
             'motionlist': ','.join(['1'] * 44), 'motordata': ''}
# 第 1/2 列：相對位置，motordata 只給 5 個（不足 12）
_cfg['1'] = {'savestate': '0', 'motionstate': str(C.MOTIONSTATE_REL_POS),
             'id': '11', 'item_name': '微調',
             'motionlist': '2,900,5', 'motordata': '7,7,7,7,7'}
_cfg['2'] = {'savestate': '0', 'motionstate': str(C.MOTIONSTATE_REL_SPD),
             'id': '11', 'item_name': '微調',
             'motionlist': '2,900,5', 'motordata': '9,9,9'}
with open(os.path.join(_d, 'ragged.ini'), 'w', encoding='utf-8') as _f:
    _cfg.write(_f)

r4 = store4.read_ini('ragged')
chk('讀得到', r4['ok'])
chk('MotionList 截到 40 + 兩列各 3 = 46',
    len(r4['motionlist']) == C.MOTIONLIST_PAIRS * 2 + 6, str(len(r4['motionlist'])))
chk('停止條件落在第 40 格（不是被 MotionList 的資料擠掉）',
    r4['motionlist'][40:43] == [2, 900, 5], str(r4['motionlist'][40:46]))
chk('relativedata 兩列各補滿 12', len(r4['relativedata']) == NJ * 2,
    str(len(r4['relativedata'])))
chk('不足的位置補 0 而不是吃到下一列',
    r4['relativedata'][:NJ] == [7, 7, 7, 7, 7] + [0] * 7, str(r4['relativedata'][:NJ]))
chk('第二列從第 12 格開始',
    r4['relativedata'][NJ:NJ + 3] == [9, 9, 9], str(r4['relativedata'][NJ:NJ + 4]))

shutil.rmtree(root, ignore_errors=True)
print('\n' + ('全部通過' if not fails else f'{len(fails)} 項失敗:\n  - '
               + '\n  - '.join(fails)))
sys.exit(0 if not fails else 1)
