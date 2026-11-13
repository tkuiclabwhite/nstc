#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手部動作檔的讀寫、sector 快取、以及序列播放。

結構刻意比照全身的 motionpackage，只是 motordata 從 27 欄變成 12 欄
（L1..L6, R1..R6），這樣 HandMotionInterface.html 幾乎可以照抄
MotionControlInterface.html 的操作邏輯。

檔案位置（跟著 strategy.ini 的全域策略走）：

    src/strategy/strategy/<策略>/Parameter/hand/<檔名>.ini        編輯檔
    src/strategy/strategy/<策略>/Parameter/hand/sector/<id>.ini    單筆可執行動作

編輯檔（多個 section，與全身同構）：

    [0]
    savestate = 0
    motionstate = 0          ; 0=MotionList 1=RelPos 2=RelSpd 3=AbsPos 4=AbsSpd
    id = 70
    item_name = 抓取流程
    motionlist = 10,500,11,800,...    ; (動作ID, delay_ms) x 20
    motordata =

sector 檔（單一 [Data]，speed/pos 交錯 12 對 = 24 個值）：

    [Data]
    id = 10
    motionstate = 4          ; 0=motionlist 2=relative 4=absolute
    motordata = 200,0,200,0,...

數值語意（與全身一致）：
    AbsolutePosition  0~255，-1 = 這一軸不動
    RelativePosition  增減量，0 = 不動
    速度              10~255，0 = 不改速度
"""
import configparser
import os
import time

from handpackage import hand_config as C

NPAIR = C.NUM_JOINTS_BOTH          # 12 對 speed/pos
NDATA = NPAIR * 2                  # 24 個值


class MotionStore:
    """動作檔讀寫 + sector 的記憶體快取。

    sector 同時存在記憶體（執行用）和 .ini（重開機後還在），
    與 motionpackage 的 saved_sectors + save_sector_to_disk 同一套做法。
    """

    def __init__(self, log, strategy_getter, root=None):
        self.log = log
        self._strategy = strategy_getter
        # root 預設是 workspace 根目錄。開成參數是為了能用暫存目錄跑測試，
        # 不要讓測試寫進真的策略資料夾（motionpackage 那邊是寫死的）
        self.root = root or os.path.join(os.path.expanduser('~'), 'nstc')
        self.sectors = {}            # {'10': {'opcode': 241, 'data': [24 個值]}}
        self.save_buffer = {}        # {檔名: [row, ...]}，saveflag 時才寫檔

    # ---- 路徑 ----
    def motion_dir(self, strategy=None):
        return os.path.join(self.root, 'config',
                            strategy or self._strategy(), 'Parameter', C.MOTION_DIR)

    def sector_dir(self, strategy=None):
        return os.path.join(self.motion_dir(strategy), C.SECTOR_DIR)

    # ---- sector ----
    def save_sector(self, sector_id, opcode, data, stop=None):
        """存進記憶體並備份成 .ini。

        「位置全 0 就當誤觸」這個保護是從 motionpackage 抄來的，但**不能原樣套用
        在絕對位置上**：全身的絕對位置是 Dynamixel 原始刻度（上千），全 0 代表
        那一列是空的；手的絕對位置是 0~255，**全 0 就是握拳**，是完全合法的姿勢。
        原樣照抄的結果是「存握拳 → 被靜默丟掉 → execute 跑到上一次的舊內容」。
        所以只有相對量（全 0 真的等於這一步什麼都不做）和空清單才套用這個保護。
        """
        sector_id = str(sector_id)
        if opcode == C.OPCODE_REL:
            positions = data[1::2]
            if positions and all(p == 0 for p in positions):
                self.log.info(f'[Sector] id={sector_id} 相對位移全為 0（這一步不會動），忽略')
                return False
        elif opcode == C.OPCODE_MOTIONLIST:
            if all(v == 0 for v in data):
                self.log.info(f'[Sector] id={sector_id} 清單全為 0，當誤觸忽略')
                return False

        # MotionList 與姿態共用同一個 sector 編號空間。如果一個 MotionList 存到
        # 它自己引用到的動作編號上（網頁的 Sector 欄填了跟子動作一樣的號碼），
        # 後存的會把先存的蓋掉，播放時那一步就變成「引用到 MotionList」而被跳過。
        # 這種情形很難從現象看出來，所以換型別時一定要吼一聲。
        old = self.sectors.get(sector_id)
        if old is not None and old['opcode'] != opcode:
            was = 'MotionList' if old['opcode'] == C.OPCODE_MOTIONLIST else '姿態'
            now = 'MotionList' if opcode == C.OPCODE_MOTIONLIST else '姿態'
            self.log.warn(f'[Sector] id={sector_id} 從「{was}」被覆蓋成「{now}」。'
                          f'MotionList 的 Sector 編號不要跟它引用到的動作編號重複')
        self.sectors[sector_id] = {'opcode': opcode, 'data': list(data), 'stop': stop}

        try:
            d = self.sector_dir()
            os.makedirs(d, exist_ok=True)
            cfg = configparser.ConfigParser()
            cfg['Data'] = {'id': sector_id}
            if opcode == C.OPCODE_MOTIONLIST:
                cfg['Data']['motionstate'] = '0'
                cfg['Data']['motionlist'] = ','.join(map(str, data))
            else:
                cfg['Data']['motionstate'] = '2' if opcode == C.OPCODE_REL else '4'
                cfg['Data']['motordata'] = ','.join(map(str, data))
            # 停止條件：播放到這一步時改用夾持迴路，壓力到門檻才往下走
            if stop:
                cfg['Data']['stop'] = C.format_stop(stop)
            path = os.path.join(d, f'{sector_id}.ini')
            with open(path, 'w', encoding='utf-8') as f:
                cfg.write(f)
            self.log.info(f'[Sector] id={sector_id} 已存：{path}')
            return True
        except Exception as e:
            self.log.error(f'[Sector] 存檔失敗：{e}')
            return False

    def has_sector(self, sector_id):
        return str(sector_id) in self.sectors

    def get_sector(self, sector_id):
        return self.sectors.get(str(sector_id))

    def load_all(self):
        """啟動與策略切換時，把該策略的 sector 全部載進記憶體。"""
        self.sectors.clear()
        d = self.sector_dir()
        if not os.path.isdir(d):
            self.log.info(f'[載入] 還沒有 sector 目錄：{d}')
            return 0
        n = 0
        for fn in sorted(os.listdir(d)):
            if not fn.endswith('.ini'):
                continue
            try:
                cfg = configparser.ConfigParser()
                cfg.read(os.path.join(d, fn), encoding='utf-8')
                if 'Data' not in cfg:
                    continue
                sec = cfg['Data']
                sid = sec.get('id', fn[:-4])
                ms = int(sec.get('motionstate', '4'))
                if ms == 0:
                    data = self._ints(sec.get('motionlist', ''))
                    opcode = C.OPCODE_MOTIONLIST
                else:
                    data = self._ints(sec.get('motordata', ''))
                    opcode = C.OPCODE_REL if ms == 2 else C.OPCODE_ABS
                if data:
                    self.sectors[str(sid)] = {
                        'opcode': opcode, 'data': data,
                        'stop': C.parse_stop(sec.get('stop', '')),
                    }
                    n += 1
            except Exception as e:
                self.log.warn(f'[載入] {fn} 失敗：{e}')
        self.log.info(f'[載入] {n} 筆 sector 來自 {d}')
        return n

    @staticmethod
    def _ints(s):
        s = (s or '').strip()
        if not s:
            return []
        out = []
        for x in s.split(','):
            x = x.strip()
            if x:
                try:
                    out.append(int(float(x)))
                except ValueError:
                    out.append(0)
        return out

    # ---- 編輯檔 ----
    def buffer_row(self, filename, row):
        self.save_buffer.setdefault(filename, []).append(row)

    def write_ini(self, filename):
        """把累積的列寫成 .ini（整檔覆蓋，與 motionpackage 相同）。

        key 用 id_motionstate，所以同一個 id 的五種 motionstate 各佔一個 section，
        重複送同一組會覆蓋而不是長出兩份。

        motionlist 欄位依 motionstate 有兩種意思：
          motionstate 0（MotionList）：40 個值 = (動作 id, delay) x 20
          motionstate 1~4（姿態）    ：3 個值 = (停止條件模式, 門檻, 步長)
        姿態列那一欄在全身是永遠空的，手這邊拿來放夾持的停止條件。
        之所以不用 item_name，是因為那一欄要留給使用者取名 —— 早期版本把
        「|stop=模式:門檻」接在名稱後面，結果名字就不能打字了。
        兩種長度都是固定的，而讀的那一邊知道每一列的 motionstate，
        所以攤平成同一個陣列也能一列一列正確切回來。
        """
        rows = self.save_buffer.pop(filename, [])
        base = self.motion_dir()
        try:
            os.makedirs(base, exist_ok=True)
            db = {}
            for r in rows:
                db[f"{r['id']}_{r['motionstate']}"] = {
                    'savestate': str(r.get('savestate', 0)),
                    'motionstate': str(r['motionstate']),
                    'id': str(r['id']),
                    'item_name': str(r.get('item_name', '')),
                    'motionlist': ','.join(map(str, r.get('motionlist', []))),
                    'motordata': ','.join(map(str, r.get('motordata', []))),
                }
            cfg = configparser.ConfigParser()
            for i, k in enumerate(db):
                cfg[str(i)] = db[k]
            path = os.path.join(base, f'{filename}.ini')
            with open(path, 'w', encoding='utf-8') as f:
                cfg.write(f)
            self.log.info(f'[存檔] {path}（{len(db)} 筆）')
            return True
        except Exception as e:
            self.log.error(f'[存檔] 失敗：{e}')
            return False

    def _fixed(self, text, n):
        """把一欄切成**剛好 n 個**整數：不足補 0、超過截斷。

        網頁端用固定 stride 切平行陣列，所以每一列的貢獻長度必須是定值。
        """
        v = self._ints(text)[:n]
        return v + [0] * (n - len(v))

    def read_ini(self, filename, strategy=None):
        """讀編輯檔，回傳 ReadMotion 回應需要的那幾個平行陣列。

        relativedata / absolutedata 各自把該類別的列依序攤平成一維，
        與全身的 cb_read_motion 相同 —— 網頁端是靠 motionstate 的順序去切的。
        """
        base = self.motion_dir(strategy)
        path = os.path.join(base, f'{filename}.ini')
        out = {'ok': False, 'cnt': 0, 'motionstate': [], 'id': [],
               'motionlist': [], 'relativedata': [], 'absolutedata': [], 'item_names': []}
        if not os.path.isfile(path):
            self.log.warn(f'[讀檔] 找不到 {path}')
            return out
        try:
            cfg = configparser.ConfigParser()
            cfg.read(path, encoding='utf-8')
            for name in cfg.sections():
                sec = cfg[name]
                if 'motionstate' not in sec:
                    continue
                ms = int(sec['motionstate'])
                out['motionstate'].append(ms)
                out['id'].append(int(sec.get('id', '0')))
                out['item_names'].append(sec.get('item_name', ''))
                # 三個平行陣列都**必須**是固定 stride，因為網頁端是靠
                # 「第幾列、哪種 motionstate」去算 offset 切的，不是靠分隔符號：
                #     motionlist   MotionList 列吃 40（20 對），姿態列吃 3（停止條件）
                #     relativedata / absolutedata   每個姿態列吃 12
                # 只要有一列給的數量不對，從那一列之後全部錯位 —— 現象是
                # 「某一列的內容跑到別列去」，例如 MotionList 的數字出現在
                # RelativePosition 的停止條件欄。手改過的 .ini、舊版存的檔、
                # 或欄位數不同的檔案都會這樣，所以這裡一律補齊/截斷。
                if ms == C.MOTIONSTATE_MOTIONLIST:
                    out['motionlist'] += self._fixed(sec.get('motionlist', ''),
                                                     C.MOTIONLIST_PAIRS * 2)
                elif ms in (C.MOTIONSTATE_REL_POS, C.MOTIONSTATE_REL_SPD):
                    out['motionlist'] += self._fixed(sec.get('motionlist', ''),
                                                     C.STOP_TRIPLE_LEN)
                    out['relativedata'] += self._fixed(sec.get('motordata', ''),
                                                       C.NUM_JOINTS_BOTH)
                else:
                    out['motionlist'] += self._fixed(sec.get('motionlist', ''),
                                                     C.STOP_TRIPLE_LEN)
                    out['absolutedata'] += self._fixed(sec.get('motordata', ''),
                                                       C.NUM_JOINTS_BOTH)
                out['cnt'] += 1
            out['ok'] = True
            self.log.info(f'[讀檔] {path}（{out["cnt"]} 筆）')
        except Exception as e:
            self.log.error(f'[讀檔] 失敗：{e}')
        return out

    def list_files(self, strategy=None):
        d = self.motion_dir(strategy)
        if not os.path.isdir(d):
            return []
        return sorted(f[:-4] for f in os.listdir(d) if f.endswith('.ini'))


class PacketParser:
    """網頁送來的位元封包：[83, 84, opcode, payload..., 78, 69]

    沿用全身那套協定（nstc_msgs/InterfaceSend2Sector），只是 payload 從 27 對
    變成 12 對，所以長度不同。opcode 242（全身起身專屬）手這邊沒有。
    """

    def __init__(self, log):
        self.log = log
        self.buf = []

    def feed(self, values):
        """吃一段資料，回傳 [(opcode, payload), ...]。"""
        self.buf.extend(int(v) for v in values)
        out = []
        while len(self.buf) >= 3:
            if self.buf[0] != C.PKT_HEAD[0] or self.buf[1] != C.PKT_HEAD[1]:
                self.buf.pop(0)
                continue
            opcode = self.buf[2]
            if opcode not in C.PKT_LEN:
                self.log.warn(f'[封包] 未知 opcode {opcode}')
                self.buf.pop(0)
                continue
            need = C.PKT_LEN[opcode]
            if len(self.buf) < need:
                break
            if (self.buf[need - 2], self.buf[need - 1]) != C.PKT_TAIL:
                self.buf.pop(0)
                continue
            out.append((opcode, self.buf[3:need - 2]))
            self.buf = self.buf[need:]
        return out


class Player:
    """單手的 MotionList 播放器。

    刻意做成非阻塞（外部每個控制週期呼叫 tick）。全身的 execute_motion_list
    是在 callback 裡用 time.sleep 一路睡完，那會卡住整個節點 —— 手這邊有 30 Hz
    控制迴路要跑，而且左右手要能各播各的，所以不能那樣做。
    """

    def __init__(self, side, log):
        self.side = side
        self.log = log
        self.reset()

    def reset(self):
        self.active = False
        self.seq = []          # [(sector_id, delay_ms), ...]
        self.idx = 0
        self.next_t = 0.0
        self.loop = 1
        self.done_loops = 0
        self.name = ''
        # 這一步是夾持步驟時，播放器會卡在這裡等夾持結束才往下走
        self.grip_wait = False
        self.last_delay_ms = 0

    def start(self, seq, loop=1, name=''):
        """seq 是 (sector_id, delay_ms) 的清單。id 0 代表跳過這一步（與全身一致）。"""
        self.reset()
        self.seq = [(str(a), int(b)) for a, b in seq]
        if not self.seq:
            return False
        self.active = True
        self.loop = int(loop)
        self.name = name
        self.next_t = time.time()      # 第一步立刻執行
        self.log.info(f'[播放] {self.side} 開始：{len(self.seq)} 步，'
                      f'loop={"無限" if self.loop <= 0 else self.loop}')
        return True

    def stop(self):
        if self.active:
            self.log.info(f'[播放] {self.side} 停止')
        self.reset()

    def tick(self, now):
        """時間到就回傳這一步的 sector_id，否則回 None。整段播完回 'DONE'。"""
        # 夾持步驟：壓力條件還沒滿足就一直等，不要照時間往下跑
        if self.grip_wait:
            return None
        if not self.active or now < self.next_t:
            return None
        if self.idx >= len(self.seq):
            self.done_loops += 1
            if self.loop <= 0 or self.done_loops < self.loop:
                self.idx = 0
            else:
                self.active = False
                return 'DONE'
        sid, delay = self.seq[self.idx]
        self.idx += 1
        self.last_delay_ms = delay
        # 與全身一致：delay 下限 50 ms，否則一個控制週期內會連跑好幾步
        self.next_t = now + max(0.05, delay / 1000.0)
        return sid

    def release_grip_wait(self, now):
        """夾持結束，馬上接著做下一步（delay 已經當成夾持的逾時用掉了）。"""
        self.grip_wait = False
        self.next_t = now

    def status(self):
        return {
            'active': self.active,
            'name': self.name,
            'step': self.idx,
            'total': len(self.seq),
            'loop': self.loop,
            'done_loops': self.done_loops,
            'grip_wait': self.grip_wait,
        }


def split_pose(data, side):
    """把 24 個值（speed/pos 交錯 12 對）切出單手的 6 對。

    L1..L6 在前、R1..R6 在後。綁在某一手的播放器只套用自己那一半，
    另一半直接忽略 —— 單手序列就自然不用去填 -1。
    """
    off = 0 if side == 'left' else C.NUM_JOINTS
    spd, pos = [], []
    for j in range(C.NUM_JOINTS):
        i = (off + j) * 2
        if i + 1 >= len(data):
            spd.append(0)
            pos.append(C.POS_HOLD)
            continue
        spd.append(data[i])
        pos.append(data[i + 1])
    return spd, pos
