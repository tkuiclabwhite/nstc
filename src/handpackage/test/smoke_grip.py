#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""夾持迴路測試：假扮兩隻 O6（state / info / 壓力），觀察節點送出的位置命令。

不需要真的手。壓力值由測試直接設定（不做物理模擬），門檻跨越的時機才完全可控。
涵蓋 18 組情境 / 61 項檢查：離線與無壓感時拒絕啟動、等不到壓力資料時絕對不動手、
each / any / anyhand / hold 四種模式、拇指兩軸共用壓力、釋放邊界、逾時、
壓力中斷、過溫、手動介入、緊急停止、壓感訂閱的生命週期。

檔名刻意不是 test_*.py，colcon test / pytest 不會自動收集。

用法：
    # 終端 1
    ros2 run handpackage handpackage
    # 終端 2
    python3 src/handpackage/test/smoke_grip.py

兩邊 ROS_DOMAIN_ID 要一致，建議另設一個（例如 77）跟真的手隔開。
"""
import json, sys, threading, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

SIDES = ('left', 'right')
NJ = 6


class Sim(Node):
    """假的 SDK：發 state/info/mass，並把節點下的位置當成手真的到位了。"""

    def __init__(self):
        super().__init__('grip_sim')
        self.pos = {s: [60] * NJ for s in SIDES}
        self.mass = {s: [0] * 5 for s in SIDES}
        self.temp = {s: [40] * NJ for s in SIDES}
        self.touch_type = {s: 4 for s in SIDES}
        self.send_state = {s: True for s in SIDES}
        self.send_mass = {s: True for s in SIDES}
        self.cmds = {s: [] for s in SIDES}

        self.p_state = {s: self.create_publisher(JointState, f'/cb_{s}_hand_state', 10)
                        for s in SIDES}
        self.p_info = {s: self.create_publisher(String, f'/cb_{s}_hand_info', 10)
                       for s in SIDES}
        self.p_mass = {s: self.create_publisher(
            String, f'/cb_{s}_hand_matrix_touch_mass', 10) for s in SIDES}
        for s in SIDES:
            self.create_subscription(JointState, f'/cb_{s}_hand_control_cmd',
                                     self._mk(s), 10)
        self.status = None
        self.grip_hist = []      # 夾持 active 期間的每一筆 grip 區塊
        self.create_subscription(String, '/hand/status', self._on_status, 10)
        self.p_cmd = self.create_publisher(String, '/hand/cmd', 10)
        self.create_timer(0.02, self._pub)       # 50 Hz

    def _on_status(self, m):
        st = json.loads(m.data)
        self.status = st
        g = st.get('grip', {})
        if g.get('active'):
            self.grip_hist.append(g)

    def _mk(self, side):
        def cb(m):
            p = [int(round(v)) for v in m.position]
            self.cmds[side].append(p)
            self.pos[side] = p                   # 假設手瞬間到位
        return cb

    def _pub(self):
        for s in SIDES:
            if self.send_state[s]:
                js = JointState()
                js.position = [float(v) for v in self.pos[s]]
                js.velocity = [0.0] * NJ
                js.effort = [0.0] * NJ
                self.p_state[s].publish(js)
                self.p_info[s].publish(String(data=json.dumps({
                    'version': [3, 1, 3], 'hand_joint': 'O6',
                    'motor_temperature': self.temp[s], 'fault': [0] * NJ,
                    'torque': [0] * NJ, 'is_touch': True,
                    'touch_type': self.touch_type[s],
                })))
            if self.send_mass[s]:
                d = {'stamp': {'secs': 0, 'nsecs': 0}, 'unit': 'g', 'palm_mass': [-1]}
                for i, k in enumerate(['thumb_mass', 'index_mass', 'middle_mass',
                                       'ring_mass', 'little_mass']):
                    d[k] = self.mass[s][i]
                self.p_mass[s].publish(String(data=json.dumps(d)))

    def send(self, payload):
        self.p_cmd.publish(String(data=json.dumps(payload)))

    def clear(self):
        for s in SIDES:
            self.cmds[s].clear()
        self.grip_hist.clear()

    def reset(self, pos=255):
        for s in SIDES:
            self.pos[s] = [pos] * NJ
            self.mass[s] = [0] * 5
            self.temp[s] = [40] * NJ
            self.touch_type[s] = 4
            self.send_state[s] = True
            self.send_mass[s] = True
        self.clear()


rclpy.init()
sim = Sim()
spin = threading.Thread(target=lambda: rclpy.spin(sim), daemon=True)
spin.start()

fails = []
def chk(label, cond, extra=''):
    print(('  PASS  ' if cond else '  FAIL  ') + label + (f'   {extra}' if extra else ''))
    if not cond:
        fails.append(label)

def wait(t):
    time.sleep(t)

def grip(**kw):
    p = {'cmd': 'grip', 'action': 'start', 'hand': 'both', 'mode': 'each',
         'threshold': 800, 'target': [0] * NJ, 'step': 3, 'timeout': 15,
         'release_margin': 200}
    p.update(kw)
    sim.send(p)

def gstat():
    return (sim.status or {}).get('grip', {})

def last_active():
    """最後一筆「夾持還在跑」時的狀態。夾持停掉之後 status 只剩 active/reason，
    所以凍結旗標這種東西要從歷程裡回頭看。"""
    return sim.grip_hist[-1] if sim.grip_hist else {}

def wait_closing(timeout=4.0):
    """等到真的開始閉合。grip start 之後要先訂閱壓感、等 DDS discovery、
    等第一筆壓力資料進來，這段時間節點刻意不動手，所以不能用固定 sleep。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if gstat().get('phase') == 'closing':
            return True
        if sim.grip_hist and gstat().get('active') is False:
            return False      # 已經跑完或被中止
        time.sleep(0.05)
    return False

def mode():
    return (sim.status or {}).get('mode')

def stop_all():
    sim.send({'cmd': 'stop'})
    wait(0.4)

wait(1.5)
sim.reset(); wait(0.5)

print('\n=== 1. 離線的手不准夾 ===')
sim.send_state['right'] = False
wait(1.4)                                    # 等 online timeout
sim.clear(); grip(hand='both'); wait(0.6)
chk('拒絕啟動', mode() != 'grip', f'mode={mode()}')
chk('沒有送出任何位置命令', not sim.cmds['left'] and not sim.cmds['right'])
sim.reset(); wait(0.6)

print('\n=== 2. 沒有壓感的手不准夾 ===')
sim.touch_type['left'] = -1
wait(0.4); sim.clear(); grip(hand='left'); wait(0.6)
chk('拒絕啟動', mode() != 'grip', f'mode={mode()}')
chk('沒有送出位置命令', not sim.cmds['left'])
sim.reset(); wait(0.6)

print('\n=== 3. 等不到壓力資料要中止，而且絕對不能先動 ===')
sim.send_mass['left'] = sim.send_mass['right'] = False
wait(0.3); sim.clear(); grip(); wait(0.8)
chk('phase 停在 wait_touch', gstat().get('phase') == 'wait_touch', str(gstat().get('phase')))
chk('等待期間沒有動手', not sim.cmds['left'] and not sim.cmds['right'],
    f"left={len(sim.cmds['left'])}")
wait(2.0)
chk('逾時後中止', not gstat().get('active'), str(gstat()))
chk('理由正確', '等不到壓力' in gstat().get('reason', ''), gstat().get('reason', ''))
chk('整個過程完全沒動手', not sim.cmds['left'] and not sim.cmds['right'])
sim.reset(); wait(0.6)

print('\n=== 4. 無壓力時應該一路閉合到目標然後完成 ===')
sim.clear(); grip(target=[0]*NJ, step=12, timeout=10)
chk('有進入閉合階段', wait_closing())
chk('夾持中 phase=closing', gstat().get('phase') == 'closing', str(gstat().get('phase')))
wait(2.5)
chk('閉合到目標 0', sim.pos['left'] == [0]*NJ, str(sim.pos['left']))
chk('夾持自動結束', not gstat().get('active'))
chk('結束理由是閉合到目標', '閉合到目標' in gstat().get('reason', ''), gstat().get('reason', ''))
chk('回到 manual', mode() == 'manual', str(mode()))
sim.reset(); wait(0.6)

print('\n=== 5. 步長會被遵守，且不低於 3 ===')
sim.clear(); grip(step=5, target=[0]*NJ)
wait_closing(); wait(0.1)
d = [255 - sim.cmds['left'][0][i] for i in range(NJ)] if sim.cmds['left'] else []
chk('第一步 = 5', d and all(x == 5 for x in d), str(d))
stop_all(); sim.reset(); wait(0.5)
sim.clear(); grip(step=1, target=[0]*NJ)
wait_closing(); wait(0.1)
d = [255 - sim.cmds['left'][0][i] for i in range(NJ)] if sim.cmds['left'] else []
chk('step=1 會被抬到 3', d and all(x == 3 for x in d), str(d))
stop_all(); sim.reset(); wait(0.5)

print('\n=== 6. each 模式：只有到門檻的那根手指停 ===')
sim.clear(); grip(mode='each', step=3, target=[0]*NJ)
wait_closing(); wait(0.5)
sim.mass['left'][1] = 900          # 食指 → joint 2
wait(0.5)
fr = last_active().get('frozen', {}).get('left', [])
chk('食指那軸凍結', len(fr) == NJ and fr[2] is True, str(fr))
chk('其他軸沒凍結', len(fr) == NJ and not fr[3] and not fr[4] and not fr[5], str(fr))
p2 = sim.pos['left'][2]
wait(0.5)
chk('食指軸真的停住了', sim.pos['left'][2] == p2, f'{p2} -> {sim.pos["left"][2]}')
chk('其他軸繼續閉合', sim.pos['left'][3] < p2, f'j3={sim.pos["left"][3]} j2={p2}')
stop_all(); sim.reset(); wait(0.5)

print('\n=== 7. each 模式：拇指兩軸共用一個壓力值，會一起停 ===')
sim.clear(); grip(mode='each', step=3, target=[0]*NJ)
wait_closing(); wait(0.5)
sim.mass['left'][0] = 900          # thumb → joint 0 和 1
wait(0.5)
fr = last_active().get('frozen', {}).get('left', [])
chk('拇指兩軸都凍結', len(fr) == NJ and fr[0] and fr[1], str(fr))
chk('其他軸沒被連帶凍結', len(fr) == NJ and not fr[2], str(fr))
p0, p1 = sim.pos['left'][0], sim.pos['left'][1]
wait(0.5)
chk('拇指兩軸真的停住', sim.pos['left'][0] == p0 and sim.pos['left'][1] == p1,
    f'{[p0, p1]} -> {sim.pos["left"][:2]}')
stop_all(); sim.reset(); wait(0.5)

print('\n=== 8. any 模式：一指到門檻 → 該手全停，另一手不受影響 ===')
sim.clear(); grip(mode='any', step=3, target=[0]*NJ)
wait_closing(); wait(0.5)
sim.mass['left'][3] = 900
wait(0.5)
frl = last_active().get('frozen', {}).get('left', [])
frr = last_active().get('frozen', {}).get('right', [])
chk('左手六軸全凍結', len(frl) == NJ and all(frl), str(frl))
chk('右手完全沒凍結', len(frr) == NJ and not any(frr), str(frr))
pl = list(sim.pos['left']); pr = list(sim.pos['right'])
wait(0.5)
chk('左手真的全停', sim.pos['left'] == pl, f'{pl} -> {sim.pos["left"]}')
chk('右手繼續閉合', sim.pos['right'][0] < pr[0], f'{pr[0]} -> {sim.pos["right"][0]}')
stop_all(); sim.reset(); wait(0.5)

print('\n=== 9. anyhand 模式：任一手到門檻 → 兩手全停 ===')
sim.clear(); grip(mode='anyhand', step=3, target=[0]*NJ)
wait_closing(); wait(0.5)
sim.mass['right'][4] = 900
wait(0.5)
# anyhand 全軸同時凍結 → all_done 當下就成立、夾持立刻結束，
# 所以要看停止後保留的快照，而不是 active 期間的 status
g9 = gstat()
frl = g9.get('frozen', {}).get('left', [])
frr = g9.get('frozen', {}).get('right', [])
chk('兩手都全凍結（從停止快照）', len(frl) == NJ and all(frl) and all(frr),
    f'L={frl} R={frr}')
chk('理由指出是壓力到門檻', '壓力到門檻' in g9.get('reason', ''), g9.get('reason', ''))
# 結束理由現在是每手各自一段（兩手各夾各的之後，單一句話講不清楚）
chk('理由指出是哪一手', 'right' in g9.get('reason', ''), g9.get('reason', ''))
chk('理由指出哪根手指', 'little' in g9.get('reason', ''), g9.get('reason', ''))
pl = list(sim.pos['left']); pr = list(sim.pos['right'])
wait(0.6)
chk('兩手都真的停住', sim.pos['left'] == pl and sim.pos['right'] == pr,
    f'L {pl} -> {sim.pos["left"]}')
sim.reset(); wait(0.5)

print('\n=== 10. 釋放邊界（hysteresis）===')
sim.clear(); grip(mode='each', step=3, target=[0]*NJ, release_margin=200)
wait_closing(); wait(0.4)
sim.mass['left'][2] = 900          # 中指 → joint 3
wait(0.5)
frozen_at = sim.pos['left'][3]
sim.mass['left'][2] = 750          # 掉一點，但還在 800-200=600 以上 → 維持凍結
wait(0.6)
chk('在釋放邊界內維持凍結', sim.pos['left'][3] == frozen_at,
    f'{frozen_at} -> {sim.pos["left"][3]}')
sim.mass['left'][2] = 500          # 低於 600 → 解除，繼續閉合
wait(0.6)
chk('低於釋放邊界後恢復閉合', sim.pos['left'][3] < frozen_at,
    f'{frozen_at} -> {sim.pos["left"][3]}')
stop_all(); sim.reset(); wait(0.5)

print('\n=== 11. hold 模式：壓太大往回退、壓不夠繼續夾 ===')
sim.clear(); grip(mode='hold', step=3, target=[0]*NJ, release_margin=200)
wait_closing(); wait(0.6)
sim.mass['left'] = [2000] * 5      # 遠高於門檻 → 往回退（值變大，回 origin 255）
p0 = sim.pos['left'][3]
wait(0.7)
chk('壓力過大時往回退', sim.pos['left'][3] > p0, f'{p0} -> {sim.pos["left"][3]}')
wait(1.5)
chk('不會退過起始位置 255', sim.pos['left'][3] <= 255, str(sim.pos['left'][3]))
sim.mass['left'] = [100] * 5       # 遠低於門檻 → 繼續夾
p1 = sim.pos['left'][3]
wait(0.7)
chk('壓力不足時繼續夾', sim.pos['left'][3] < p1, f'{p1} -> {sim.pos["left"][3]}')
chk('hold 不會自己結束', gstat().get('active') is True)
stop_all(); sim.reset(); wait(0.5)

print('\n=== 12. 逾時 ===')
sim.clear(); grip(mode='each', step=3, target=[0]*NJ, timeout=1.5)
wait_closing(); wait(0.3)
sim.mass['left'] = [900, 0, 0, 0, 0]
wait(1.8)
chk('逾時後停止', not gstat().get('active'))
chk('理由是逾時', '逾時' in gstat().get('reason', ''), gstat().get('reason', ''))
sim.reset(); wait(0.5)

print('\n=== 13. 壓力中途斷掉要停，不能盲夾 ===')
sim.clear(); grip(step=3, target=[0]*NJ)
wait_closing(); wait(0.4)
# 只斷左手的壓力。改成每手獨立之後，**只有左手該停、右手要繼續** ——
# 動作序列裡兩手各夾各的，一手的感測器掛掉不該把另一手也停掉
sim.send_mass['left'] = False
wait(1.2)
g13 = gstat()
done13 = g13.get('done', {})
chk('左手停了', 'left' in done13 or not g13.get('active'), str(g13.get('done')))
chk('左手的理由是資料中斷',
    '中斷' in (done13.get('left', '') or g13.get('reason', '')),
    str(done13) + ' / ' + g13.get('reason', ''))
chk('右手還在夾（不該被連帶停掉）',
    g13.get('active') and 'right' in (g13.get('hands') or []),
    f"active={g13.get('active')} hands={g13.get('hands')}")
n = len(sim.cmds['left']); wait(0.6)
chk('左手停止後不再送位置命令', len(sim.cmds['left']) == n,
    f'{n} -> {len(sim.cmds["left"])}')
stop_all()
sim.reset(); wait(0.6)

print('\n=== 14. 過溫保護 ===')
sim.clear(); grip(step=3, target=[0]*NJ)
wait_closing(); wait(0.4)
# 只有左手過溫。每手獨立之後，右手不該被連帶停掉
sim.temp['left'] = [40, 40, 70, 40, 40, 40]
wait(1.0)
g14 = gstat()
d14 = g14.get('done', {})
chk('左手因過溫停止', 'left' in d14 or not g14.get('active'), str(d14))
chk('理由是溫度', '\u00b0C' in (d14.get('left', '') or g14.get('reason', '')),
    str(d14) + ' / ' + g14.get('reason', ''))
chk('右手不受影響', g14.get('active') and 'right' in (g14.get('hands') or []),
    f"active={g14.get('active')} hands={g14.get('hands')}")
stop_all()
sim.reset(); wait(0.6)

print('\n=== 15. 手動介入可以中斷夾持 ===')
sim.clear(); grip(step=3, target=[0]*NJ)
wait_closing(); wait(0.3)
chk('夾持中', gstat().get('active') is True, str(gstat().get('active')))
sim.send({'cmd': 'manual', 'hand': 'left', 'position': [200]*NJ}); wait(0.5)
chk('夾持被中斷', not gstat().get('active'))
chk('模式回 manual', mode() == 'manual', str(mode()))
chk('手動命令生效', sim.pos['left'] == [200]*NJ, str(sim.pos['left']))
sim.reset(); wait(0.6)

print('\n=== 16. stop 可以中斷夾持 ===')
sim.clear(); grip(step=3, target=[0]*NJ)
wait_closing(); wait(0.3)
sim.send({'cmd': 'stop'}); wait(0.5)
chk('夾持被中斷', not gstat().get('active'))
chk('回到手動', (sim.status or {}).get('mode') == 'manual',
    str((sim.status or {}).get('mode')))
n = len(sim.cmds['left']); wait(0.6)
chk('停止後不再送命令', len(sim.cmds['left']) <= n + 1,
    f'{n} -> {len(sim.cmds["left"])}')
sim.reset(); wait(0.5)

print('\n=== 17. 壓感訂閱只在夾持期間存在 ===')
wait(0.8)
before = sim.p_mass['left'].get_subscription_count()
sim.clear(); grip(step=3, target=[0]*NJ)
wait_closing()
during = sim.p_mass['left'].get_subscription_count()
stop_all(); wait(0.8)
after = sim.p_mass['left'].get_subscription_count()
chk('夾持前沒訂閱', before == 0, f'count={before}')
chk('夾持中有訂閱', during >= 1, f'count={during}')
chk('夾持後退訂', after == 0, f'count={after}')

print('\n=== 18. 壓力零點：開始時就有讀數的手指不會一開始就觸發（2026-11-26 實機右手中指）===')
sim.clear(); sim.mass['left'][2] = 80; sim.mass['right'][2] = 80   # 沒碰東西就讀 80，比門檻 50 大
grip(mode='any', threshold=50, release_margin=80, step=3, target=[0]*NJ)
chk('有進入閉合', wait_closing())
p3 = sim.pos['left'][3]
wait(0.5)
chk('零點之後還在夾', gstat().get('active') is True, str(gstat().get('reason')))
chk('中指軸繼續閉合', sim.pos['left'][3] < p3, f'{p3} -> {sim.pos["left"][3]}')
chk('status 有零點', (gstat().get('base') or {}).get('left', [None] * 5)[2] == 80,
    str(gstat().get('base')))
sim.mass['left'][2] = 140          # 比零點多 60 ≥ 50 → 觸發
wait(0.5)
fr = last_active().get('frozen', {}).get('left', [])
chk('比零點多到門檻才凍結（any：整隻手）', len(fr) == NJ and all(fr), str(fr))
stop_all(); sim.reset(); wait(0.5)

print('\n' + ('全部通過' if not fails else f'{len(fails)} 項失敗:\n  - ' + '\n  - '.join(fails)))
# spin 在 daemon thread 裡還在跑，正常 shutdown 會讓 rclpy 的 C++ 端抱怨
# "terminate called without an active exception"，測試腳本直接硬退出就好
sys.stdout.flush()
# 乾淨關閉 DDS。用 os._exit 強制退出會留下幽靈 participant，
# 害下一支測試的 discovery 要等逾時才接得上（連續跑會假性失敗）
try:
    sim.destroy_node()
except Exception:
    pass
rclpy.try_shutdown()
sys.exit(0 if not fails else 1)
