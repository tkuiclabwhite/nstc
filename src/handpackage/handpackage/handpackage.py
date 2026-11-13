#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LinkerHand O6 雙手控制節點。

硬體通訊由 linker_hand_ros2_sdk 負責，這個節點只做它做不到的事：

  1. 當 /cb_*_hand_control_cmd 的「唯一寫入者」，仲裁手動 / 夾持 / 動作播放三種來源
     —— 少了這層，三個功能同時開會互相蓋掉（SDK 只保留最後一筆命令）。
  2. 雙手同步（逐軸 a*v+b，預設恆等）。
  3. 夾持到指定觸覺壓力就停（階段 ⑤）。
  4. 手部動作序列的檔案讀寫與播放（階段 ⑦）。
  5. 補 SDK 缺的東西：分左右手的扭矩設定、0~255 ↔ 弧度換算、整理給網頁用的狀態包。

網頁介面刻意收斂成兩個通道，好 log 好 debug：
  /hand/cmd     (std_msgs/String JSON)  網頁 → 這裡，所有即時操作
  /hand/status  (std_msgs/String JSON)  這裡 → 網頁，20 Hz
  /hand/config  (std_msgs/String JSON)  這裡 → 網頁，1 Hz，關節名/範圍/預設動作
動作檔那組介面（InterfaceSaveMotion / InterfaceReadSaveMotion / InterfaceSend2Sector /
CheckSector / Sector / Callback）完全比照全身的 motionpackage，見階段 ⑦。

註：/cb_*_hand_setting_cmd 是 hands.launch.py remap 出來的分手 topic。上游的
/cb_hand_setting_cmd 是全域的，而 linker_hand.py 的 hand_setting_cb() 沒有比對
params.hand_type 跟節點自己的 hand_type，所以不 remap 的話對一隻手下的設定
兩隻手都會執行。詳見 src/linkerhand-ros2-sdk/UPSTREAM.md 地雷 3。
"""
import json
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int16, String

from handpackage import hand_config as C
from handpackage.hand_motion import MotionStore, PacketParser, Player, split_pose

# nstc_msgs 做成選用：動作檔那組介面沿用全身現成的 msg/srv（刻意不新增型別），
# 但沒編 nstc_msgs 的機器（例如只想在桌機上開發網頁）照樣要能跑起控制與夾持。
try:
    from nstc_msgs.msg import InterfaceSend2Sector, Location, SaveMotion
    from nstc_msgs.srv import CheckSector, ReadMotion
    HAVE_TKU_MSGS = True
    TKU_MSGS_ERR = ''
except Exception as _e:          # noqa: BLE001  —— 什麼原因都要能降級啟動
    HAVE_TKU_MSGS = False
    TKU_MSGS_ERR = f'{type(_e).__name__}: {_e}'


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class HandPackage(Node):

    def __init__(self):
        super().__init__('handpackage')

        self.declare_parameter('location', 'ar')
        self.declare_parameter('control_hz', C.CONTROL_HZ)
        self.declare_parameter('status_hz', C.STATUS_HZ)

        self.location_folder = self.get_parameter('location').value
        # 防零：參數被寫成 0 的話 1.0/hz 會炸掉
        control_hz = max(1.0, float(self.get_parameter('control_hz').value))
        status_hz = max(1.0, float(self.get_parameter('status_hz').value))

        # strategy.ini 是全域策略的單一來源，作法與 motionpackage 一致
        self._refresh_strategy(initial=True)

        # --- 硬體回讀 ---
        self.hw = {
            side: {
                'position': None,    # list[int] * 6，來自 /cb_*_hand_state
                'stamp': 0.0,
                'info': {},          # /cb_*_hand_info 的 JSON
                'info_stamp': 0.0,
                'mass': None,        # list[int] * 5，只在夾持模式才訂
                'mass_stamp': 0.0,
            } for side in C.HANDS
        }

        # --- 命令目標 ---
        # pos/spd 是 None 表示「這隻手還沒被下過任何命令」，此時不發 control_cmd，
        # 讓 SDK 啟動時下的初始姿態留著，不要一連上就把手拉走。
        # 執行歷史。與 motionpackage 同一套語意：記的是「執行過哪些 sector」，
        # 每一筆存的是**執行前**兩手的目標位置與速度，所以 Undo 會讓手真的動回去，
        # 不是編輯層的復原。標籤就是 "Sector <id>"
        self._hist = []
        self._future = []
        self._hist_max = 20

        # 完整的速度狀態（見 set_target_speed 的說明）
        self.spd_state = {side: list(C.SDK_INIT_SPEED) for side in C.HANDS}
        self.tgt = {
            side: {
                'pos': None,
                'spd': None,
                'torque': None,
                'pos_dirty': False,
                'spd_dirty': False,
            } for side in C.HANDS
        }

        # --- 模式 ---
        # idle / manual / grip / motion
        # 優先權：manual > grip > motion
        #   —— 夾持或播放中只要收到手動命令就中斷，切回 manual。
        # 沒有「緊急停止」這一層：stop 指令就是單純停下（停掉夾持與播放、
        # 回手動），手保持在最後下達的位置，不鎖定也不需要解除。
        self.mode = 'idle'
        # 網頁 execute 期間會鎖按鈕，等 /hand/Callback 才解開。
        # 這個旗標記著「有一筆 execute 還沒回報」
        self._exec_pending = False
        # 有人要求常駐訂閱壓感（{'cmd':'touch','on':true}）。
        # 平常只有夾持期間才訂，因為訂了控制迴路會變鈍
        self._touch_hold = False
        # 同步的「來源手」是由指令指定哪隻手決定的（你動哪隻，哪隻就是來源），
        # 所以不需要另外一個 master 設定 —— 原本有一個，但它從來沒有影響任何行為
        self.sync_on = False

        self.grip = {'active': False}
        self.play = {'active': False}

        # 動作檔 / sector / 播放器
        self.store = MotionStore(self.get_logger(), lambda: self.location_folder)
        self.parser = PacketParser(self.get_logger())
        # 左右手各一個播放器，所以「同步播同一序列」和「各播各的」是同一套機制：
        # 同步 = 把同一個 MotionList 派給兩個播放器
        self.players = {side: Player(side, self.get_logger()) for side in C.HANDS}

        # --- 對 SDK ---
        self.pub_cmd = {
            side: self.create_publisher(JointState, f'/cb_{side}_hand_control_cmd', 10)
            for side in C.HANDS
        }
        self.pub_setting = {
            side: self.create_publisher(String, f'/cb_{side}_hand_setting_cmd', 10)
            for side in C.HANDS
        }
        for side in C.HANDS:
            self.create_subscription(
                JointState, f'/cb_{side}_hand_state', self._make_state_cb(side), 10)
            self.create_subscription(
                String, f'/cb_{side}_hand_info', self._make_info_cb(side), 10)
        # mass 只在夾持模式啟用時才訂 —— 訂了會讓 SDK 的控制迴路變鈍
        # （見 UPSTREAM.md 地雷 7），所以平常不要掛著。
        self._mass_sub = {side: None for side in C.HANDS}

        # --- 對網頁 ---
        self.create_subscription(String, '/hand/cmd', self._on_cmd, 20)
        self.pub_status = self.create_publisher(String, '/hand/status', 10)
        self.pub_config = self.create_publisher(String, '/hand/config', 10)

        self._setup_motion_interfaces()

        self.create_timer(1.0 / control_hz, self._on_control)
        self.create_timer(1.0 / status_hz, self._on_status)
        # config 刻意用 1 Hz 重複發而不是 latched：rosbridge 預設以 volatile QoS
        # 訂閱，收不到 transient_local 的保留訊息，網頁晚開就永遠拿不到。
        self.create_timer(1.0, self._on_config)

        self._dispatch = {
            'mode': self._cmd_mode,
            'manual': self._cmd_manual,
            'speed': self._cmd_speed,
            'torque': self._cmd_torque,
            'sync': self._cmd_sync,
            'stop': self._cmd_stop,
            'touch': self._cmd_touch,
            'grip': self._cmd_grip,
            'play': self._cmd_play,
            'undo': self._cmd_undo,
            'redo': self._cmd_redo,
            'clear_history': self._cmd_clear_history,
        }

        self.get_logger().info(
            f'handpackage 啟動，策略 ={self.location_folder}，'
            f'控制 {control_hz:g} Hz / 狀態 {status_hz:g} Hz')

    # ======================================================================
    # 動作檔介面（比照 motionpackage，名稱換成 /hand/*）
    # ======================================================================
    def _setup_motion_interfaces(self):
        self.pub_callback = None
        # 載入 sector 只是讀 .ini，不需要 nstc_msgs。放在守門外面，
        # 這樣沒編 nstc_msgs 的機器雖然不能從網頁存讀，但還是能播磁碟上已有的動作
        self.store.load_all()
        if not HAVE_TKU_MSGS:
            self.get_logger().warn(
                '沒有 nstc_msgs，手部動作檔功能停用（控制與夾持不受影響）。'
                f'原因：{TKU_MSGS_ERR}')
            self.get_logger().warn(
                '要啟用請編 nstc_msgs：'
                'colcon build --symlink-install --packages-select nstc_msgs')
            return

        self.create_subscription(InterfaceSend2Sector, '/hand/InterfaceSend2Sector',
                                 self._on_send2sector, 50)
        self.create_subscription(SaveMotion, '/hand/InterfaceSaveMotion',
                                 self._on_save_motion, 50)
        self.create_subscription(Int16, '/hand/Sector', self._on_sector_execute, 50)
        # 全域策略切換。image.py / overlap_node / walking_web_bridge / usb_cam_node
        # 都訂這一個 topic，手這邊也要跟上，否則動作檔會一直存讀到啟動時那個策略。
        # Location 是 nstc_msgs 的型別，所以只能放在守門之後 —— 沒編 nstc_msgs 的
        # 機器本來就不能存讀動作檔，策略跟不跟都沒差
        self.create_subscription(Location, '/location', self._on_location, 10)
        self.pub_callback = self.create_publisher(Bool, '/hand/Callback', 10)
        self.create_service(ReadMotion, '/hand/InterfaceReadSaveMotion',
                            self._srv_read_motion)
        self.create_service(CheckSector, '/hand/InterfaceCheckSector',
                            self._srv_check_sector)

    def _on_send2sector(self, msg):
        """網頁的 Send：位元封包 -> sector。sectorname 是要存成哪個 id。

        停止條件走 delay / cnt 這兩個本來沒在用的欄位（見 hand_config.STOP_MODE_IDS），
        跟封包在同一筆訊息裡，不會有兩個 topic 先後到達的問題。
        delay = [模式編號, 門檻, 步長]；舊版只送前兩個，步長就用預設。
        """
        stop = None
        if getattr(msg, 'cnt', 0) == 1 and len(msg.delay) >= 2:
            mode = C.STOP_MODE_IDS.get(int(msg.delay[0]))
            if mode:
                step = int(msg.delay[2]) if len(msg.delay) >= 3 else C.GRIP_MIN_STEP + 1
                stop = (mode,
                        int(clamp(int(msg.delay[1]), 1, C.MASS_MAX)),
                        max(C.GRIP_MIN_STEP, step))
        for opcode, payload in self.parser.feed(msg.package):
            # Send 只是把姿態存進 sector，不是執行，所以不發 /hand/Callback ——
            # 那個 topic 的語意是「execute 做完了」，網頁靠它解開鎖定
            self.store.save_sector(msg.sectorname, opcode, payload, stop=stop)

    def _on_save_motion(self, msg):
        """網頁的 Save：逐列送進來累積，最後一筆 saveflag=True 才真的寫檔。"""
        filename = msg.name if msg.name else 'untitled'
        if msg.saveflag:
            self.store.write_ini(filename)
        else:
            self.store.buffer_row(filename, {
                'savestate': msg.savestate,
                'motionstate': msg.motionstate,
                'id': msg.id,
                'item_name': getattr(msg, 'item_name', ''),
                'motionlist': list(msg.motionlist),
                'motordata': list(msg.motordata),
            })

    def _srv_read_motion(self, request, response):
        # 每次讀檔前確認一下策略有沒有被別人切掉，與 motionpackage 的作法一致。
        # 借讀別的策略（網頁的 Import）時不要去動自己的 location_folder
        borrow = (request.strategy or '').strip()
        if not borrow:
            self._refresh_strategy()
        out = self.store.read_ini(request.name, borrow or None)
        response.readcheck = out['ok']
        response.vectorcnt = out['cnt']
        response.motionstate = out['motionstate']
        response.id = out['id']
        response.motionlist = out['motionlist']
        response.relativedata = out['relativedata']
        response.absolutedata = out['absolutedata']
        response.item_names = out['item_names']
        return response

    def _srv_check_sector(self, request, response):
        response.checkflag = self.store.has_sector(request.data)
        return response

    def _on_sector_execute(self, msg):
        """網頁的 execute：單筆 sector 直接做。是 MotionList 就交給播放器。

        做完一定要發 /hand/Callback —— 網頁在 execute 期間會把按鈕鎖起來，
        就是等這一筆才解開（與全身的 /package/executecallback 同一個用途）。
        三種情況各自的「做完」：
          MotionList  -> 播放器整段跑完時發（_play_tick）
          帶停止條件  -> 夾持結束時發（_grip_finish_hand 的最後一手）
          單純姿態    -> 位置下去就算完成，這裡直接發
        """
        sid = str(msg.data)
        # 旗標要在查表之前就立起來 —— 找不到 sector 也必須回報一次（False），
        # 不然網頁會一直鎖著等到安全網逾時
        self._exec_pending = True
        rec = self.store.get_sector(sid)
        if rec is None:
            self.get_logger().warn(f'[執行] 找不到 sector id={sid}')
            self._exec_done(False)
            return
        self._interrupt_running('執行 sector')
        # 與 motionpackage 相同：執行前先記一筆，Undo 才回得去
        self._push_history(f'Sector {sid}')
        if rec['opcode'] == C.OPCODE_MOTIONLIST:
            self._start_players({s: sid for s in C.HANDS}, loop=1)
            return
        self._set_mode('manual', f'執行 sector {sid}')
        for side in C.HANDS:
            self._apply_sector(side, sid)
        # 夾持步驟要等壓力到門檻，不能現在就說做完了
        if not self.grip.get('active'):
            self._exec_done(True)

    def _exec_done(self, ok=True):
        """回報一次 execute 結束。沒有在等的時候就不發，免得網頁亂解鎖。"""
        if not self._exec_pending:
            return
        self._exec_pending = False
        if self.pub_callback is not None:
            self.pub_callback.publish(Bool(data=bool(ok)))

    # ======================================================================
    # 播放
    # ======================================================================
    def _seq_of(self, sector_id):
        """把 MotionList 的 motionlist 攤平陣列切成 (id, delay_ms) 清單。

        與全身一致：id 0 代表這一步跳過，尾端補的 0,0 自然會被濾掉。
        """
        rec = self.store.get_sector(sector_id)
        if rec is None or rec['opcode'] != C.OPCODE_MOTIONLIST:
            return None
        data = rec['data']
        seq = []
        for i in range(0, len(data) - 1, 2):
            sid, delay = data[i], data[i + 1]
            if sid == 0 and delay == 0:
                continue
            seq.append((sid, delay))
        return seq

    def _start_players(self, per_side, loop=1):
        """per_side = {'left': MotionList id 或 None, 'right': ...}"""
        started = False
        for side in C.HANDS:
            sid = per_side.get(side)
            if sid is None:
                self.players[side].stop()
                continue
            seq = self._seq_of(str(sid))
            if seq is None:
                self.get_logger().warn(
                    f'[播放] sector {sid} 不是 MotionList，{side} 手不播')
                self.players[side].stop()
                continue
            if self.players[side].start(seq, loop, name=str(sid)):
                started = True
        if started:
            self.play = {'active': True}
            self._set_mode('motion', '動作播放')
        else:
            self.play = {'active': False}
            if not self.store.sectors:
                self.get_logger().error(
                    '沒有任何 sector 可播。要先用網頁的 Send 存成 sector'
                    + ('（而且這台沒有 nstc_msgs，存不了）' if not HAVE_TKU_MSGS else ''))
        return started

    def _cmd_play(self, d):
        action = d.get('action', 'start')
        if action == 'stop':
            self.stop_play('網頁要求停止')
            self._set_mode('manual', '播放停止')
            return
        self._interrupt_running('開始播放')
        lbl = d.get('left') if d.get('left') is not None else d.get('right')
        self._push_history(f'Play {lbl}')
        self._start_players({'left': d.get('left'), 'right': d.get('right')},
                            loop=int(d.get('loop', 1)))

    def _apply_sector(self, side, sector_id, delay_ms=None):
        """把一個 sector 的姿態套到指定手。只取這隻手那一半的 6 對。

        如果這個 sector 帶了停止條件，就不是單純下位置，而是改用夾持迴路把這一半
        收到壓力門檻為止。此時該步的 delay 當成夾持的逾時上限，播放器會卡住等它
        結束（夾不到東西逾時了也照樣往下走）。

        相對列同樣可以帶停止條件：先把相對位移解成絕對目標，再交給夾持迴路，
        所以「從現在的姿勢再往內收 40，收到壓力 800 就停」是可以表達的。
        """
        if str(sector_id) == '0':
            return
        rec = self.store.get_sector(sector_id)
        if rec is None:
            self.get_logger().warn(f'[執行] {side} 找不到 sector id={sector_id}，跳過')
            return
        if rec['opcode'] == C.OPCODE_MOTIONLIST:
            # 不支援 MotionList 裡再放 MotionList，全身也沒有這種用法
            self.get_logger().warn(f'[執行] sector {sector_id} 是 MotionList，不能當一步')
            return

        spd, pos = split_pose(rec['data'], side)

        stop = rec.get('stop')
        if stop:
            # 夾持步驟的目標一律要是絕對位置。
            #   絕對列：-1（這軸不動）用目前位置補上
            #   相對列：以「上次下的命令」為基準加上位移，位移 0 就維持基準，
            #           與下面一般相對列同一套算法（不要把回授誤差灌進來）
            if rec['opcode'] == C.OPCODE_REL:
                base = (self.tgt[side]['pos'] or self.hw[side]['position']
                        or list(C.SDK_INIT_POSE))
                tgt = [int(base[j]) if pos[j] == 0
                       else int(clamp(base[j] + pos[j], C.POS_MIN, C.POS_MAX))
                       for j in range(C.NUM_JOINTS)]
            else:
                tgt = [int(self.hw[side]['position'][j]) if pos[j] == C.POS_HOLD
                       else int(clamp(pos[j], C.POS_MIN, C.POS_MAX))
                       for j in range(C.NUM_JOINTS)]
            if any(v != 0 for v in spd):
                self.set_target_speed(side, spd)
            to = (delay_ms / 1000.0) if delay_ms else 10.0
            step = stop[2] if len(stop) > 2 else C.GRIP_MIN_STEP + 1
            ok = self.add_grip_for(side, stop[0], [stop[1]] * len(C.MASS_KEYS),
                                   tgt, to, step=step)
            if ok:
                self.players[side].grip_wait = True
                if self.mode != 'grip':
                    self.get_logger().info(f'[播放] {side} 進入夾持步驟 {sector_id}')
            return

        if rec['opcode'] == C.OPCODE_REL:
            # 相對量的基準是「上次下的命令」，不是量測值 —— 與全身的 last_goals
            # 同一個理由：一連串相對動作要能正確接續，不能讓回授誤差一步一步累積。
            # 所以 delta=0（這軸不動）要保留 base[j] 本身，不能寫成 POS_HOLD；
            # POS_HOLD 會被 _resolve_pos 換成量測位置，等於每一步都把誤差灌進來。
            base = (self.tgt[side]['pos'] or self.hw[side]['position']
                    or list(C.SDK_INIT_POSE))
            pos = [int(base[j]) if pos[j] == 0
                   else int(clamp(base[j] + pos[j], C.POS_MIN, C.POS_MAX))
                   for j in range(C.NUM_JOINTS)]
        # 絕對量的 -1 就維持 POS_HOLD，由 set_target_pos 換成量測位置 ——
        # 那是「停在它現在的地方」，對應全身 execute_pose 裡 raw_val==-1 直接
        # continue（那顆馬達不下命令）的實際效果

        if any(v != 0 for v in spd):
            self.set_target_speed(side, spd)
        self.set_target_pos(side, pos)

    def _play_tick(self, now):
        any_active = False
        for side in C.HANDS:
            p = self.players[side]
            r = p.tick(now)
            if r == 'DONE':
                self.get_logger().info(f'[播放] {side} 整段完成')
                self._exec_pending = False     # 已經由這裡回報，不要再發第二次
                if self.pub_callback is not None:
                    self.pub_callback.publish(Bool(data=True))
            elif r is not None:
                self._apply_sector(side, r, p.last_delay_ms)
            any_active = any_active or p.active
        if not any_active and self.play.get('active'):
            self.play = {'active': False}
            self._set_mode('manual', '播放完成')

    def _play_status(self):
        out = {'active': bool(self.play.get('active'))}
        for side in C.HANDS:
            out[side] = self.players[side].status()
        return out

    # ======================================================================
    # 策略
    # ======================================================================
    @staticmethod
    def _parse_strategy(raw):
        """從 /location 或 strategy.ini 的內容取出策略名稱。

        /location 上同時跑著兩種格式，兩種都要吃：
            'ar/Parameter'
                ImageProcessInterface 送的，也是 strategy.ini 的內容
            '/home/iclab/ros2_adult/src/strategy/strategy/ar/Parameter'
                MotionControlInterface 送的（它組的是絕對路徑）
        所以不能用「split('Parameter')[0] 再把斜線拿掉」那種寫法 —— 絕對路徑
        會變成 'homeiclabros2_adultsrcstrategystrategyar'。
        規則與 image.py 的 location_callback、網頁的 parseStrategyName 一致：
        切成路徑片段，取 'Parameter' 前面那一段；沒有 Parameter 就取最後一段。
        """
        parts = [p for p in str(raw or '').split('/') if p]
        if not parts:
            return ''
        if 'Parameter' in parts:
            i = parts.index('Parameter')
            return parts[i - 1] if i > 0 else ''
        return parts[-1]

    def _on_location(self, msg):
        """網頁切策略時廣播的 /location。

        image.py / walking_web_bridge 收到之後會把 strategy.ini 寫掉，但**寫檔跟
        這筆訊息是同時發生的**，所以這裡直接解析訊息內容，不去讀 strategy.ini
        —— 讀檔會有競態，可能讀到還沒被改寫的舊值。

        之前沒有訂這個 topic，結果是手的節點與三個網頁永遠顯示啟動時那個策略
        （預設 ar），動作檔也一直存讀到 ar 底下。
        """
        raw = (getattr(msg, 'data', '') or '').strip()
        new_loc = self._parse_strategy(raw)
        if not new_loc:
            self.get_logger().warn(f'/location 看不懂：{raw!r}，沿用 {self.location_folder}')
            return
        if new_loc == self.location_folder:
            return
        self.get_logger().info(f'[切換] 策略 {self.location_folder} -> {new_loc}（/location）')
        self.location_folder = new_loc
        # 記憶體裡的 sector 是舊策略的，要整批重載
        self.store.load_all()

    def _refresh_strategy(self, initial=False):
        """讀 src/strategy/strategy/strategy.ini 決定目前策略。

        只在啟動時、以及網頁呼叫 Read 服務時用。執行中的策略切換走
        /location（見 _on_location）—— 那個是即時的，不用等下一次 Read。

        檔案內容長這樣：`ar/Parameter`（image.py 寫檔前會把絕對路徑正規化成
        相對的）。解析走 _parse_strategy，與 /location 同一套。
        """
        path = os.path.join(os.path.expanduser('~'),
                            'nstc/config/strategy.ini')
        try:
            if not os.path.exists(path):
                return
            with open(path, 'r', encoding='utf-8') as f:
                content = f.read().strip()
            new_loc = self._parse_strategy(content)
            if new_loc and new_loc != self.location_folder:
                tag = '[Init]' if initial else '[切換]'
                self.get_logger().info(
                    f'{tag} 策略 {self.location_folder} -> {new_loc}')
                self.location_folder = new_loc
                if not initial and HAVE_TKU_MSGS:
                    # 策略換了，記憶體裡的 sector 是舊策略的，要整批重載
                    self.store.load_all()
        except Exception as e:
            self.get_logger().warn(f'讀取 strategy.ini 失敗，沿用 {self.location_folder}: {e}')

    def motion_dir(self):
        """<策略>/Parameter/hand/ —— 手部動作編輯檔的目錄（階段 ⑦ 用）"""
        return os.path.join(os.path.expanduser('~'), 'nstc/config',
                            self.location_folder, 'Parameter', C.MOTION_DIR)

    def sector_dir(self):
        """<策略>/Parameter/hand/sector/ —— 單筆可執行動作的目錄（階段 ⑦ 用）"""
        return os.path.join(self.motion_dir(), C.SECTOR_DIR)

    # ======================================================================
    # 硬體回讀
    # ======================================================================
    def _make_state_cb(self, side):
        def cb(msg):
            # msg.position 是 array.array('d')，轉成 int list 再存
            self.hw[side]['position'] = [int(round(v)) for v in msg.position]
            self.hw[side]['stamp'] = time.time()
        return cb

    def _make_info_cb(self, side):
        def cb(msg):
            try:
                self.hw[side]['info'] = json.loads(msg.data)
                self.hw[side]['info_stamp'] = time.time()
            except Exception:
                pass
        return cb

    def _make_mass_cb(self, side):
        def cb(msg):
            try:
                d = json.loads(msg.data)
            except Exception:
                return
            vals = []
            for k in C.MASS_KEYS:
                v = d.get(k)
                # 沒有的感測器會回 list（例如 palm_mass 回 [-1]），不是數字就當沒有
                vals.append(int(v) if isinstance(v, (int, float)) else -1)
            self.hw[side]['mass'] = vals
            self.hw[side]['mass_stamp'] = time.time()
        return cb

    def set_mass_subscription(self, active):
        """夾持迴路要用壓力值時才訂 mass，用完就退訂。

        但如果有人用 {'cmd':'touch','on':true} 要求常駐，夾持結束時不可以把它
        退掉 —— 否則「邊讀壓力邊做別的事」的用法會在每次夾持結束後斷掉。
        """
        if not active and self._touch_hold:
            return
        for side in C.HANDS:
            if active and self._mass_sub[side] is None:
                self._mass_sub[side] = self.create_subscription(
                    String, f'/cb_{side}_hand_matrix_touch_mass',
                    self._make_mass_cb(side), 10)
                self.get_logger().info(f'開始訂閱 {side} 壓感（控制迴路會略微變鈍）')
            elif not active and self._mass_sub[side] is not None:
                self.destroy_subscription(self._mass_sub[side])
                self._mass_sub[side] = None
                self.hw[side]['mass'] = None
                self.get_logger().info(f'停止訂閱 {side} 壓感')

    def online(self, side):
        return (self.hw[side]['position'] is not None
                and time.time() - self.hw[side]['stamp'] < C.STATE_TIMEOUT)

    # ======================================================================
    # 目標值
    # ======================================================================
    def mirror(self, pos):
        """逐軸套用 a*v+b。預設恆等；設成 (-1, 255) 的軸會左右翻轉。"""
        out = []
        for i, v in enumerate(pos):
            if v is None or v == C.POS_HOLD:
                out.append(C.POS_HOLD)
                continue
            a, b = C.MIRROR_GAIN_OFFSET[i]
            out.append(int(round(clamp(a * v + b, C.POS_MIN, C.POS_MAX))))
        return out

    def _resolve_pos(self, side, pos):
        """把 -1（這軸不動）換成實際位置，並夾到 0~255。

        一定要換掉：finger_move() 會檢查 0 <= v <= 255，含 -1 的整包指令會被丟掉。
        在下命令的當下就換算成快照，行為才可預測。
        """
        cur = self.hw[side]['position']
        prev = self.tgt[side]['pos']
        out = []
        for i, v in enumerate(pos):
            if v is None or v == C.POS_HOLD:
                if cur is not None:
                    out.append(int(cur[i]))
                elif prev is not None:
                    out.append(int(prev[i]))
                else:
                    out.append(C.SDK_INIT_POSE[i])
            else:
                out.append(int(round(clamp(v, C.POS_MIN, C.POS_MAX))))
        return out

    def set_target_pos(self, side, pos):
        resolved = self._resolve_pos(side, pos)
        if self.tgt[side]['pos'] != resolved:
            self.tgt[side]['pos'] = resolved
            self.tgt[side]['pos_dirty'] = True

    def set_target_speed(self, side, spd):
        """速度走 control_cmd 的 velocity 欄位 → set_joint_speed()。

        這裡有個上游的陷阱：set_joint_speed() 是
            if any(x < 10 or x > 255 for x in speed): 整包丟掉
        也就是「六個值必須全部」落在 10~255。但 0 在 control_cmd 的語意又是
        「這次不改速度」（linker_hand.py:186 全 0 才跳過），兩者直接衝突 ——
        只要有一軸填 0，整筆速度都會被丟掉並印
        "The numerical range cannot be less than 10 or greater than 255"。

        所以 handpackage 自己維護一份完整有效的速度：收到的 0 代表「沿用舊值」，
        送出去的永遠是六個合法值。起點用 SDK 自己下的初始速度，
        這樣第一次只改某幾軸時，其他軸不會被拉到錯的值。
        """
        cur = self.spd_state[side]
        changed = False
        for j, v in enumerate(spd[:C.NUM_JOINTS]):
            v = int(round(v))
            if v == C.SPEED_NOCHANGE:
                continue
            nv = int(clamp(v, C.SPEED_MIN, C.SPEED_MAX))
            if nv != cur[j]:
                cur[j] = nv
                changed = True
        if not changed:
            return
        self.tgt[side]['spd'] = list(cur)
        self.tgt[side]['spd_dirty'] = True

    def set_torque(self, side, torque):
        """扭矩上限只有 setting_cmd 這條路，走 remap 後的分手 topic。"""
        vals = [int(clamp(int(round(v)), C.TORQUE_MIN, C.TORQUE_MAX)) for v in torque]
        self.tgt[side]['torque'] = vals
        msg = String()
        msg.data = json.dumps({
            'setting_cmd': 'set_max_torque_limits',
            'params': {'hand_type': side, 'torque': vals},
        })
        self.pub_setting[side].publish(msg)
        self.get_logger().info(f'{side} 扭矩上限 -> {vals}')

    # ======================================================================
    # /hand/cmd
    # ======================================================================
    def _on_cmd(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            self.get_logger().warn(f'/hand/cmd 不是合法 JSON: {msg.data[:120]}')
            return
        name = d.get('cmd')
        fn = self._dispatch.get(name)
        if fn is None:
            self.get_logger().warn(f'/hand/cmd 未知指令: {name}')
            return
        try:
            fn(d)
        except Exception as e:
            self.get_logger().error(f'處理 {name} 失敗: {e}')

    def _per_hand(self, d, key):
        """把命令裡的值整理成 {side: 6 個值}，順便把雙手同步一起處理掉。

        6 個值  → 套到 d['hand'] 指定的手（預設 both）
        12 個值 → L1..L6, R1..R6，此時忽略 d['hand']
        同步開著且只指定一隻手時，另一隻手跟著動：位置會過 mirror()，
        速度和扭矩是量值，直接複製。
        """
        vals = d[key]
        if len(vals) == C.NUM_JOINTS_BOTH:
            return {'left': list(vals[:C.NUM_JOINTS]),
                    'right': list(vals[C.NUM_JOINTS:])}
        if len(vals) != C.NUM_JOINTS:
            raise ValueError(
                f'{key} 長度要 {C.NUM_JOINTS} 或 {C.NUM_JOINTS_BOTH}，收到 {len(vals)}')

        h = d.get('hand', 'both')
        sides = list(C.HANDS) if h == 'both' else ([h] if h in C.HANDS else [])
        if not sides:
            raise ValueError(f'hand 只接受 left / right / both，收到 {h!r}')

        out = {s: list(vals) for s in sides}
        if self.sync_on and len(sides) == 1:
            dst = 'right' if sides[0] == 'left' else 'left'
            out[dst] = self.mirror(vals) if key == 'position' else list(vals)
        return out

    def _set_mode(self, new, reason=''):
        if self.mode != new:
            self.get_logger().info(f'模式 {self.mode} -> {new}'
                                   + (f'（{reason}）' if reason else ''))
            self.mode = new

    def _interrupt_running(self, reason):
        """手動介入時中斷夾持與播放。"""
        if self.grip.get('active'):
            self.stop_grip(reason)
        if self.play.get('active'):
            self.stop_play(reason)

    def _cmd_mode(self, d):
        m = d.get('mode')
        if m in ('idle', 'manual'):
            self._interrupt_running(f'切到 {m}')
            self._set_mode(m)
        else:
            self.get_logger().warn(f'mode 只接受 idle / manual，收到 {m}')

    def _cmd_manual(self, d):
        self._interrupt_running('手動介入')
        self._set_mode('manual')
        if 'position' not in d and 'speed' not in d:
            self.get_logger().warn('manual 指令沒帶 position 也沒帶 speed')
            return
        # 速度一定要先下。反過來的話這一筆位置會用「上一次的速度」跑完，
        # 新速度要等下一筆位置才生效 —— 預設動作用一個封包同時帶兩者時
        # 就會看到「第一次按很快、第二次才對」
        if 'speed' in d:
            for side, vals in self._per_hand(d, 'speed').items():
                self.set_target_speed(side, vals)
        if 'position' in d:
            for side, vals in self._per_hand(d, 'position').items():
                self.set_target_pos(side, vals)

    def _cmd_speed(self, d):
        for side, vals in self._per_hand(d, 'speed').items():
            self.set_target_speed(side, vals)

    def _cmd_torque(self, d):
        for side, vals in self._per_hand(d, 'torque').items():
            self.set_torque(side, vals)

    def _cmd_sync(self, d):
        self.sync_on = bool(d.get('on', False))
        self.get_logger().info(
            f'雙手同步 {"開" if self.sync_on else "關"}'
            '（來源是指令指定的那隻手，動哪隻哪隻就是來源）')

    def _cmd_touch(self, d):
        """開 / 關壓感的常駐訂閱。

        平常壓力只在夾持期間才有值（訂閱 /cb_*_hand_matrix_touch_mass 會讓
        控制迴路變鈍，所以用完就退訂）。策略或網頁想在沒夾持的時候也讀壓力
        —— 例如先碰到東西再決定要不要夾 —— 就送 {'cmd':'touch','on':true}。
        """
        on = bool(d.get('on', True))
        if on == self._touch_hold:
            return
        self._touch_hold = on
        if on:
            self.set_mass_subscription(True)
            self.get_logger().info('壓感常駐訂閱：開')
        else:
            # 旗標已經放掉，所以這一次退訂不會被守門擋住。
            # 夾持還在跑的話留著，等夾持自己結束時再退
            if not self.grip.get('active'):
                self.set_mass_subscription(False)
            self.get_logger().info('壓感常駐訂閱：關')

    def _cmd_stop(self, d):
        self._interrupt_running('停止')
        self._set_mode('manual', '停止')

    def stop_play(self, reason=''):
        was = self.play.get('active')
        for side in C.HANDS:
            self.players[side].stop()   # reset() 會把 grip_wait 一起清掉
        self.play = {'active': False}
        if was:
            self.get_logger().info(f'播放結束（{reason}）')

    # ======================================================================
    # 夾持到指定壓力
    # ======================================================================
    # 壓力來源是 /cb_{side}_hand_matrix_touch_mass 的五指值（0~10200，是矩陣
    # 40 格數值的總和，不是公克）。手只有五個壓力值但有六軸，拇指兩軸共用
    # thumb 那一個值，所以 each 模式下拇指兩軸會一起凍結。
    #
    # 這是軟保護。硬保護請同時把 set_max_torque_limits 設低一點，兩層比較安心。
    def _cmd_grip(self, d):
        action = d.get('action')
        if action == 'stop':
            if self.grip.get('active'):
                self.stop_grip('網頁要求停止')
                self._set_mode('manual', '夾持停止')
            return
        if action != 'start':
            self.get_logger().warn(f'grip 的 action 只接受 start / stop，收到 {action!r}')
            return

        mode = d.get('mode', 'each')
        if mode not in C.STOP_MODES:
            self.get_logger().warn(f'未知夾持模式 {mode!r}，可用：{C.STOP_MODES}')
            return

        h = d.get('hand', 'both')
        hands = list(C.HANDS) if h == 'both' else ([h] if h in C.HANDS else [])
        if not hands:
            self.get_logger().warn(f'hand 只接受 left / right / both，收到 {h!r}')
            return

        # 門檻可以給一個數字（五指同值）或五個數字
        th = d.get('threshold', 250)
        if isinstance(th, (int, float)):
            th = [int(th)] * len(C.MASS_KEYS)
        else:
            th = [int(x) for x in th]
        if len(th) != len(C.MASS_KEYS):
            self.get_logger().warn(f'threshold 要 1 個或 {len(C.MASS_KEYS)} 個值')
            return
        th = [int(clamp(x, 1, C.MASS_MAX)) for x in th]

        tgt_in = d.get('target', [0] * C.NUM_JOINTS)
        if len(tgt_in) == C.NUM_JOINTS_BOTH:
            target = {'left': list(tgt_in[:C.NUM_JOINTS]),
                      'right': list(tgt_in[C.NUM_JOINTS:])}
        elif len(tgt_in) == C.NUM_JOINTS:
            target = {s: list(tgt_in) for s in C.HANDS}
        else:
            self.get_logger().warn(
                f'target 要 {C.NUM_JOINTS} 或 {C.NUM_JOINTS_BOTH} 個值')
            return

        # 參與的手必須在線，而且真的有壓感 —— 否則就是閉著眼睛夾
        for side in hands:
            if not self.online(side):
                self.get_logger().error(f'{side} 手離線，不啟動夾持')
                return
            tt = self.hw[side]['info'].get('touch_type')
            if tt is not None and tt <= 0:
                self.get_logger().error(
                    f'{side} 手沒有壓力感測器（touch_type={tt}），不啟動夾持')
                return

        self.grip = {
            'active': True,
            # 先等壓力資料進來才開始動。訂閱到第一筆要經過 DDS discovery，
            # 不等的話一開始那幾十毫秒等於沒有回授在閉合
            'phase': {s: 'wait_touch' for s in hands},
            # 模式 / 門檻 / 逾時都改成「每手一份」—— 動作序列播放時，
            # 左右手是各自獨立的時間軸，可能同時各自在夾不同的東西
            'mode': {s: mode for s in hands},
            'hands': list(hands),
            'threshold': {s: list(th) for s in hands},
            'target': {s: [int(clamp(v, C.POS_MIN, C.POS_MAX)) for v in target[s]]
                       for s in hands},
            'origin': {s: list(self.hw[s]['position']) for s in hands},
            # 斜坡自己記住目前下到哪，不要每個週期去讀 self.tgt['pos']。
            # 那個是「上次下的命令」，手被外力移動、或上一次夾持留下的舊目標，
            # 都會讓起點算錯（最糟的情況是一開始就誤判成已經到目標）。
            # 起點一律取實際量測位置。
            'cmd': {s: list(self.hw[s]['position']) for s in hands},
            'step': {s: max(C.GRIP_MIN_STEP, int(d.get('step', 4))) for s in hands},
            'timeout': {s: max(0.5, float(d.get('timeout', 10.0))) for s in hands},
            'release_margin': int(clamp(int(d.get('release_margin', 80)),
                                        0, C.MASS_MAX)),
            'started': {s: time.time() for s in hands},
            'frozen': {s: [False] * C.NUM_JOINTS for s in hands},
            'hit': {s: [False] * len(C.MASS_KEYS) for s in hands},
            'reason': '',
            'done': {},
        }
        self.set_mass_subscription(True)
        self._set_mode('grip', f'夾持 {mode}')
        g = self.grip
        self.get_logger().info(
            f"夾持開始：mode={mode} hands={hands} 門檻={th} "
            f"步長={list(g['step'].values())[0]} """
            f"逾時={list(g['timeout'].values())[0]}s "
            f"釋放邊界={g['release_margin']}")

    def stop_grip(self, reason='', detail=None):
        """停止夾持。

        detail 是停下來那一刻的快照（哪根手指到門檻、哪些軸凍結）。
        any / anyhand 模式下所有軸會同時凍結，all_done 在同一個控制週期就成立、
        夾持立刻結束，所以那個狀態根本來不及出現在 20 Hz 的 status 裡 ——
        不把快照留下來，網頁就永遠看不到是哪根手指觸發的。
        """
        if self.grip.get('active'):
            self.get_logger().info(f'夾持結束（{reason}）')
        out = {'active': False, 'reason': reason}
        if detail:
            out.update(detail)
        self.grip = out
        self.set_mass_subscription(False)
        # 夾持沒了就不能讓播放器繼續卡著等
        for side in C.HANDS:
            if self.players[side].grip_wait:
                self.players[side].release_grip_wait(time.time())

    def _grip_detail(self):
        g = self.grip
        if not g.get('active'):
            return None
        return {'mode': self._first(g, 'mode'), 'hands': list(g['hands']),
                'threshold': self._first(g, 'threshold'),
                'hit': g['hit'], 'frozen': g['frozen'],
                'elapsed': self._elapsed(g)}

    @staticmethod
    def _first(g, key):
        """每手一份的參數對外還是給單一值（手動夾持時兩手一樣）。

        動作序列讓兩手各夾各的時，兩手參數可能不同，所以另外附 per_hand。
        """
        for s in g['hands']:
            return g[key][s]
        for v in g[key].values():
            return v
        return None

    @staticmethod
    def _elapsed(g):
        starts = [g['started'][s] for s in g['hands'] if s in g['started']]
        return round(time.time() - min(starts), 2) if starts else 0.0

    @staticmethod
    def _step_toward(cur, goal, step):
        """往 goal 走一步，不會走過頭。回傳 (新值, 是否已到)。"""
        if cur == goal:
            return cur, True
        if cur < goal:
            v = min(goal, cur + step)
        else:
            v = max(goal, cur - step)
        return v, v == goal

    def _grip_finish_hand(self, side, reason):
        """一隻手結束夾持，其他手繼續。全部結束時整個夾持才落幕。"""
        g = self.grip
        if side in g['hands']:
            g['hands'].remove(side)
        g.setdefault('done', {})[side] = reason
        self.get_logger().info(f'[夾持] {side} 結束（{reason}）')
        # 這隻手如果是被動作序列叫起來的，通知它的播放器可以往下走
        self.players[side].release_grip_wait(time.time())
        if not g['hands']:
            allr = '；'.join(f'{k}: {v}' for k, v in g['done'].items())
            self.stop_grip(allr, self._grip_detail())
            if self.mode == 'grip':
                self._set_mode('manual', '夾持完成')
            # 如果這個夾持是 execute 一個帶停止條件的 sector 起來的，
            # 到這裡才算做完，網頁的按鈕要到這一刻才解開
            self._exec_done(True)

    def _grip_tick(self):
        g = self.grip
        now = time.time()

        for side in list(g['hands']):
            # ---- 安全檢查（每手各自）----
            if not self.online(side):
                self._grip_finish_hand(side, '手離線')
                continue
            temps = self.hw[side]['info'].get('motor_temperature') or []
            if temps and max(temps) >= C.GRIP_MAX_TEMP:
                self._grip_finish_hand(
                    side, f'馬達 {max(temps)}°C 超過 {C.GRIP_MAX_TEMP}°C')
                continue
            if now - g['started'][side] > g['timeout'][side]:
                self._grip_finish_hand(side, f"逾時 {g['timeout'][side]}s")
                continue

            # ---- 壓力資料要夠新 ----
            fresh = (self.hw[side]['mass'] is not None
                     and now - self.hw[side]['mass_stamp'] < C.GRIP_MASS_TIMEOUT)
            if g['phase'][side] == 'wait_touch':
                if fresh:
                    g['phase'][side] = 'closing'
                    self.get_logger().info(f'[夾持] {side} 收到壓力資料，開始閉合')
                elif now - g['started'][side] > C.GRIP_WAIT_TOUCH_TIMEOUT:
                    self._grip_finish_hand(side, '等不到壓力資料')
                continue
            if not fresh:
                # 夾持途中壓力斷掉就停。沒有回授還繼續閉合會把東西夾爆
                self._grip_finish_hand(side, '壓力資料中斷')
                continue

            # ---- 每指是否到門檻（帶釋放邊界，避免在門檻附近來回抖）----
            mass = self.hw[side]['mass']
            th = g['threshold'][side]
            for f in range(len(C.MASS_KEYS)):
                if g['hit'][side][f]:
                    if mass[f] < th[f] - g['release_margin']:
                        g['hit'][side][f] = False
                elif mass[f] >= th[f]:
                    g['hit'][side][f] = True

        if not g['hands']:
            return

        # anyhand 要跨手看，所以放在逐手處理之外
        hit_anywhere = any(any(g['hit'][s]) for s in g['hands'])

        for side in list(g['hands']):
            if g['phase'][side] != 'closing':
                continue
            mode = g['mode'][side]
            hit_this_hand = any(g['hit'][side])
            for j in range(C.NUM_JOINTS):
                f = C.JOINT_TO_FINGER[j]
                if mode == 'anyhand':
                    g['frozen'][side][j] = hit_anywhere
                elif mode == 'any':
                    g['frozen'][side][j] = hit_this_hand
                elif mode == 'each':
                    g['frozen'][side][j] = g['hit'][side][f]
                # hold 不凍結，下面用壓力回授來回微調

            # ---- 算新位置並送出 ----
            cur = list(g['cmd'][side])
            new_pos = list(cur)
            mass = self.hw[side]['mass']
            th = g['threshold'][side]
            done = True
            for j in range(C.NUM_JOINTS):
                f = C.JOINT_TO_FINGER[j]
                step = g['step'][side]
                if mode == 'hold':
                    # 壓太大往回退、壓不夠繼續夾、死區內不動。
                    # 回退最多只回到起始位置，不會張得比開始時更開
                    if mass[f] > th[f]:
                        new_pos[j], _ = self._step_toward(
                            cur[j], g['origin'][side][j], step)
                    elif mass[f] < th[f] - g['release_margin']:
                        new_pos[j], _ = self._step_toward(
                            cur[j], g['target'][side][j], step)
                    done = False      # hold 會一直維持到逾時或被停掉
                elif g['frozen'][side][j]:
                    pass              # 凍結：維持現值
                else:
                    new_pos[j], reached = self._step_toward(
                        cur[j], g['target'][side][j], step)
                    if not reached:
                        done = False
            g['cmd'][side] = new_pos
            if new_pos != cur:
                self.set_target_pos(side, new_pos)

            if done:
                if any(g['hit'][side]):
                    names = [C.FINGER_NAMES[f] for f in range(len(C.MASS_KEYS))
                             if g['hit'][side][f]]
                    self._grip_finish_hand(side, '壓力到門檻（' + ', '.join(names) + '）')
                else:
                    self._grip_finish_hand(side, '閉合到目標位置')

    def add_grip_for(self, side, mode, threshold, target, timeout,
                     step=4, release_margin=80):
        """替單一隻手起一個夾持。動作序列播到帶停止條件的步驟時用這個。

        如果另一隻手已經在夾，就加進同一個 grip 物件裡（參數各自獨立），
        這樣兩手可以同時各夾各的。
        """
        if not self.online(side):
            self.get_logger().error(f'{side} 手離線，不啟動夾持')
            return False
        tt = self.hw[side]['info'].get('touch_type')
        if tt is not None and tt <= 0:
            self.get_logger().error(f'{side} 手沒有壓力感測器，不啟動夾持')
            return False

        th = [int(clamp(int(x), 1, C.MASS_MAX)) for x in threshold]
        g = self.grip
        if not g.get('active'):
            g = self.grip = {
                'active': True, 'phase': {}, 'mode': {}, 'hands': [],
                'threshold': {}, 'target': {}, 'origin': {}, 'cmd': {},
                'timeout': {}, 'started': {}, 'frozen': {}, 'hit': {},
                'step': {},
                'release_margin': int(clamp(int(release_margin), 0, C.MASS_MAX)),
                'reason': '', 'done': {},
            }
        if side not in g['hands']:
            g['hands'].append(side)
        g['phase'][side] = 'wait_touch'
        g['mode'][side] = mode
        g['step'][side] = max(C.GRIP_MIN_STEP, int(step))
        g['threshold'][side] = th
        g['target'][side] = [int(clamp(v, C.POS_MIN, C.POS_MAX)) for v in target]
        g['origin'][side] = list(self.hw[side]['position'])
        g['cmd'][side] = list(self.hw[side]['position'])
        g['timeout'][side] = max(0.5, float(timeout))
        g['started'][side] = time.time()
        g['frozen'][side] = [False] * C.NUM_JOINTS
        g['hit'][side] = [False] * len(C.MASS_KEYS)
        g.setdefault('done', {}).pop(side, None)
        self.set_mass_subscription(True)
        self.get_logger().info(
            f'[夾持] {side} 開始：mode={mode} 門檻={th} 逾時={g["timeout"][side]}s')
        return True


    # ======================================================================
    # 執行歷史（比照 motionpackage 的 Undo / Redo）
    # ======================================================================
    def _snapshot(self, label):
        """把「執行前」的兩手目標位置與速度記下來。"""
        return {
            'label': label,
            'pos': {s: list(self.tgt[s]['pos']) if self.tgt[s]['pos'] else None
                    for s in C.HANDS},
            'spd': {s: list(self.spd_state[s]) for s in C.HANDS},
        }

    def _push_history(self, label):
        self._hist.append(self._snapshot(label))
        if len(self._hist) > self._hist_max:
            self._hist.pop(0)
        self._future.clear()      # 有新動作，redo 就失效

    def _restore(self, snap):
        for side in C.HANDS:
            if snap['spd'].get(side):
                self.spd_state[side] = list(snap['spd'][side])
                self.tgt[side]['spd'] = list(snap['spd'][side])
                self.tgt[side]['spd_dirty'] = True
            if snap['pos'].get(side):
                self.set_target_pos(side, snap['pos'][side])

    def _cmd_undo(self, d):
        if not self._hist:
            self.get_logger().warn('[Undo] 沒有歷史可以回去')
            return
        self._interrupt_running('Undo')
        snap = self._hist.pop()
        self._future.append(self._snapshot(snap['label']))
        self._set_mode('manual', 'Undo')
        self._restore(snap)
        self.get_logger().info(f"[Undo] 回到執行 {snap['label']} 之前")

    def _cmd_redo(self, d):
        if not self._future:
            self.get_logger().warn('[Redo] 沒有可以重做的')
            return
        self._interrupt_running('Redo')
        snap = self._future.pop()
        self._hist.append(self._snapshot(snap['label']))
        self._set_mode('manual', 'Redo')
        self._restore(snap)
        self.get_logger().info(f"[Redo] 回到 {snap['label']}")

    def _cmd_clear_history(self, d):
        self._hist.clear()
        self._future.clear()
        self.get_logger().info('[History] 已清除')

    def _history_status(self):
        return {
            'past': [e['label'] for e in self._hist],
            'future': [e['label'] for e in self._future],
        }

    def _grip_status(self):
        """給 /hand/status 用的精簡版，不要把 origin/target 這些每 20 Hz 送出去。"""
        g = self.grip
        if not g.get('active'):
            # 不只回 active/reason —— stop_grip 存下來的那份快照（hit / frozen）
            # 也要一起送出去，網頁才看得到是哪根手指觸發停止的
            out = {'active': False, 'reason': g.get('reason', '')}
            for k in ('mode', 'hands', 'threshold', 'hit', 'frozen', 'elapsed'):
                if k in g:
                    out[k] = g[k]
            return out
        return {
            'active': True,
            'phase': self._first(g, 'phase'),
            'mode': self._first(g, 'mode'),
            'hands': list(g['hands']),
            'threshold': self._first(g, 'threshold'),
            'hit': g['hit'],
            'frozen': g['frozen'],
            'elapsed': self._elapsed(g),
            'timeout': self._first(g, 'timeout'),
            # 兩手各夾各的時（動作序列），上面的單值不夠用，這裡給完整的
            'per_hand': {s: {'mode': g['mode'].get(s),
                             'phase': g['phase'].get(s),
                             'threshold': g['threshold'].get(s),
                             'timeout': g['timeout'].get(s)}
                         for s in g['hands']},
            'done': g.get('done', {}),
        }

    # ======================================================================
    # 控制迴路
    # ======================================================================
    def _on_control(self):
        # 夾持與播放都要在發布之前跑，新算出來的位置才會在同一個週期送出去
        # 注意這裡不能用 elif：動作序列裡的夾持步驟需要「播放中同時跑夾持」，
        # 兩個都要 tick
        if self.grip.get('active'):
            self._grip_tick()
        if self.mode == 'motion' and self.play.get('active'):
            self._play_tick(time.time())

        for side in C.HANDS:
            t = self.tgt[side]
            if t['pos'] is None:
                continue  # 還沒下過命令，不要動它
            if not (t['pos_dirty'] or t['spd_dirty']):
                continue  # 沒有變化就不發，省 CAN 頻寬
            self._publish_cmd(side, send_speed=t['spd_dirty'])
            t['pos_dirty'] = False
            t['spd_dirty'] = False

    def _publish_cmd(self, side, send_speed):
        """發一筆 control_cmd。

        velocity 全 0 在 SDK 的語意是「這次不改速度」，所以只有速度真的變了才帶
        實際值 —— 否則每一筆都重設一次速度，白耗 CAN 頻寬。
        """
        t = self.tgt[side]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(C.JOINT_NAMES)
        msg.position = [float(v) for v in t['pos']]
        if send_speed and t['spd'] is not None:
            msg.velocity = [float(v) for v in t['spd']]
        else:
            msg.velocity = [0.0] * C.NUM_JOINTS
        msg.effort = [0.0] * C.NUM_JOINTS
        self.pub_cmd[side].publish(msg)

    # ======================================================================
    # 狀態
    # ======================================================================
    def _on_status(self):
        hands = {}
        for side in C.HANDS:
            hw, t = self.hw[side], self.tgt[side]
            info = hw['info']
            hands[side] = {
                'online': self.online(side),
                'position': hw['position'],
                'target': t['pos'],
                # 速度讀不回來（O6 driver 的 get_speed 回 0），所以只報最後下達值
                'speed_cmd': t['spd'],
                'torque_cmd': t['torque'],
                'temperature': info.get('motor_temperature'),
                'fault': info.get('fault'),
                'torque': info.get('torque'),
                'version': info.get('version'),
                'touch_type': info.get('touch_type'),
                'mass': hw['mass'],
            }
        payload = {
            'stamp': time.time(),
            'mode': self.mode,
            'sync': {'on': self.sync_on},
            'strategy': self.location_folder,
            # 壓感是不是常駐訂閱中（沒開的話 hands.*.mass 只在夾持期間有值）
            'touch_hold': self._touch_hold,
            'grip': self._grip_status(),
            'play': self._play_status(),
            'history': self._history_status(),
            'hands': hands,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_status.publish(msg)

    def _on_config(self):
        """關節名、範圍、預設動作都從節點發出去，網頁不要再抄一份常數。"""
        msg = String()
        msg.data = json.dumps({
            'joint_names': C.JOINT_NAMES,
            'joint_names_zh': C.JOINT_NAMES_ZH,
            'num_joints': C.NUM_JOINTS,
            'hands': list(C.HANDS),
            'pos_range': [C.POS_MIN, C.POS_MAX],
            'speed_range': [C.SPEED_MIN, C.SPEED_MAX],
            'torque_range': [C.TORQUE_MIN, C.TORQUE_MAX],
            'arc_max': C.ARC_MAX,
            'finger_names': C.FINGER_NAMES,
            'joint_to_finger': C.JOINT_TO_FINGER,
            'mass_max': C.MASS_MAX,
            'matrix_default': [C.MATRIX_ROWS, C.MATRIX_COLS],
            'stop_modes': list(C.STOP_MODES),
            'strategy': self.location_folder,
            'motion_available': HAVE_TKU_MSGS,
            'motion_error': TKU_MSGS_ERR,
            'motionlist_pairs': C.MOTIONLIST_PAIRS,
            'num_joints_both': C.NUM_JOINTS_BOTH,
        })
        self.pub_config.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandPackage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # ros2 launch 關閉時會送 SIGTERM，rclpy 的 signal handler 會把 context
        # 關掉並從 spin 丟出這個例外。不攔的話每次關 launch 都會吐一整串 traceback。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
