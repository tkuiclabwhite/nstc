#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""量 O6 的電流回授到底拿不拿得到（CAN frame 0x36）。

為什麼需要實測：協議層有、SDK 也有 `get_current()`，但

  * O6 的節點 `linker_hand_advanced_o6.py` 從來沒呼叫它，
    只有 G20 / L20 的節點會發 `/cb_*_hand_current`，所以 ROS 上看不到；
  * `linker_hand_o6_can.py` 的 `get_current()` docstring 寫著
    "Not supported yet."，而 `self.x36` 的初值是 `[-1] * 6`。
    韌體不回 0x36 的話就會一直是 -1 —— 這只能問實機。

順便釐清一個容易誤會的欄位：`/cb_*_hand_state` 的 `effort` 是
`get_torque()` 來的，而那支讀的是 frame 0x02 = **轉矩上限**
（`self.x02 = [-1] * 6  # 转矩限制`，`set_torque()` 寫的是同一個 frame）。
它是你設定的值的回讀，不是量測值，所以不會隨負載變。這支會一起印出來對照。

這支**只讀不寫**，不會讓手動。要看電流隨負載變化，請在它跑的時候
用網頁或用手去壓手指。

用法：
    # 先把手的節點關掉 —— 兩個行程對同一個 CAN id 收發會互相吃掉回應
    python3 src/handpackage/test/probe_current.py            # 右手，can3
    python3 src/handpackage/test/probe_current.py left can2
    python3 src/handpackage/test/probe_current.py right can3 20   # 量 20 秒

CAN 介面對不對用 `ip -br link show type can` 看。
"""
import collections
import statistics
import sys
import time

from linker_hand_ros2_sdk.LinkerHand.linker_hand_api import LinkerHandApi

NJ = 6
JOINTS = ['大拇指彎曲', '大拇指橫擺', '食指彎曲', '中指彎曲', '無名指彎曲', '小拇指彎曲']


def main():
    side = sys.argv[1] if len(sys.argv) > 1 else 'right'
    can = sys.argv[2] if len(sys.argv) > 2 else ('can2' if side == 'left' else 'can3')
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
    if side not in ('left', 'right'):
        sys.exit('第一個參數要是 left 或 right')

    print(f'=== {side} / {can} / 量 {secs:g} 秒 ===')
    api = LinkerHandApi(hand_type=side, hand_joint='O6', can=can)
    time.sleep(0.5)

    # 先看幾個一次性的值
    print('\n--- 一次性讀取 ---')
    print(f'  get_state()   = {api.get_state()}')
    tq = api.get_torque()
    print(f'  get_torque()  = {tq}   ← frame 0x02，是「轉矩上限」的回讀，不是量測值')

    first = api.get_current()
    print(f'  get_current() = {first}   ← frame 0x36')
    if first is None:
        sys.exit('\n✗ get_current() 回 None —— 這個 SDK 版本的 O6 沒接電流')
    if all(v == -1 for v in first):
        print('\n✗ 全部是 -1：送了 0x36 但韌體沒回（或還沒回就讀了）。'
              '\n  再多試幾次確認，還是 -1 就是這批韌體不支援電流回授。')

    # 連續讀，看值會不會動、讀得多快
    print(f'\n--- 連續讀 {secs:g} 秒（現在可以去壓手指 / 用網頁夾東西）---')
    samples = collections.defaultdict(list)
    n, t0, worst = 0, time.time(), 0.0
    while time.time() - t0 < secs:
        t1 = time.time()
        cur = api.get_current()
        dt = time.time() - t1
        worst = max(worst, dt)
        if cur is not None:
            for j in range(min(NJ, len(cur))):
                samples[j].append(cur[j])
        n += 1
    elapsed = time.time() - t0

    print(f'\n讀了 {n} 次／{elapsed:.1f} 秒 = {n / elapsed:.1f} Hz'
          f'（單次最慢 {worst * 1000:.1f} ms）')
    print(f'{"軸":>2} {"名稱":<12} {"最小":>6} {"最大":>6} {"平均":>7} {"標準差":>7}  判讀')
    moving = 0
    for j in range(NJ):
        v = samples.get(j, [])
        if not v:
            print(f'{j:>2} {JOINTS[j]:<12} {"-":>6} {"-":>6} {"-":>7} {"-":>7}  沒有資料')
            continue
        sd = statistics.pstdev(v) if len(v) > 1 else 0.0
        if all(x == -1 for x in v):
            note = '固定 -1（不支援）'
        elif min(v) == max(v):
            note = '常數（沒有回授，或完全沒負載）'
        else:
            note = '會變 ✓'
            moving += 1
        print(f'{j:>2} {JOINTS[j]:<12} {min(v):>6} {max(v):>6} '
              f'{statistics.fmean(v):>7.1f} {sd:>7.2f}  {note}')

    print()
    if moving:
        print(f'✓ {moving}/{NJ} 軸的電流會變 —— 有真正的電流回授可以用。')
        print('  0x36 的 sleep 只有 3 ms，比壓感矩陣（五指各 2 ms + 掌心 6 ms）便宜得多，')
        print('  所以它可以跟 state 一起跑在高頻率，是比觸覺更適合做夾持閉環的訊號。')
    else:
        print('✗ 沒有任何一軸的電流有變化。')
        print('  如果量的時候手指確實有受力，那就是這批韌體不回 0x36，')
        print('  夾持回授只能靠壓感（/cb_*_hand_matrix_touch）。')


if __name__ == '__main__':
    main()
