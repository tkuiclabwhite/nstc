#!/usr/bin/env python3
# coding=utf-8
# adult
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int16, Int16MultiArray, String

from nstc_msgs.msg import InterfaceSend2Sector, SaveMotion, SingleMotorData
from nstc_msgs.srv import CheckSector, ReadMotion, TorqueState
from rcl_interfaces.msg import SetParametersResult

import configparser
import json
import os
import time
import threading

class MotionNode(Node):
    def __init__(self):
        super().__init__("motion_strategy_node")
        qos = QoSProfile(depth=1000)

        self.declare_parameter("location", "ar")
        self.location_folder = self.get_parameter("location").get_parameter_value().string_value
        self.add_on_set_parameters_callback(self.cb_param_update)

        self.get_logger().info(f"📂 Current Strategy Location: .../strategy/{self.location_folder}/Parameter/")

        # Publishers
        self.cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.torque_pub = self.create_publisher(Int16MultiArray, '/set_torque', 10)

        # 扭力狀態快取：訂閱 /set_torque 而非掛在封包解碼處，這樣連
        # ImageProcessInterface 直接發布的頭部扭力指令也記錄得到。
        # 開機時 driver_node 為了寫入 indirect address 必定會關閉每顆馬達的扭力，
        # 故初始值全部視為 OFF。這是指令快取，不是硬體實際狀態。
        self.MOTOR_ID_MAX = 30
        self.torque_states = [0] * self.MOTOR_ID_MAX
        self.torque_state_sub = self.create_subscription(
            Int16MultiArray, '/set_torque', self.cb_torque_state, 10)
        self.torque_state_srv = self.create_service(
            TorqueState, '/package/GetTorqueState', self.cb_get_torque_state)

        # Subscribers
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.web_sub = self.create_subscription(InterfaceSend2Sector, "/package/InterfaceSend2Sector", self.web_interface_cb, qos)
        self.save_sub = self.create_subscription(SaveMotion, '/package/InterfaceSaveMotion', self.cb_save_motion, qos)
        self.singlemotor_sub = self.create_subscription(SingleMotorData, '/package/SingleMotorData', self.cb_single_motor, 10)
        self.SingleAbsolutePosition_sub = self.create_subscription(SingleMotorData, '/package/SingleAbsolutePosition', self.cb_SingleAbsolutePosition, 10)

        # Variables
        self.current_joints = {}
        self.joints_lock = threading.Lock()
        self.web_buffer = []
        self.current_sector_name = "0"
        self.saved_sectors = {}
        self.id_source_map = {}
        self.file_save_buffer = {}

        self.last_goals = {}

        # Services
        self.check_sector_srv = self.create_service(CheckSector, '/package/InterfaceCheckSector', self.cb_check_sector)
        self.sector_execute_sub = self.create_subscription(Int16, '/package/Sector', self.cb_sector_execute, qos)
        self.stand_execute_sub = self.create_subscription(Bool, '/package/StandExecute', self.cb_stand_execute, 10)
        self.read_motion_srv = self.create_service(ReadMotion, '/package/InterfaceReadSaveMotion', self.cb_read_motion)

        self.motion_callback_pub = self.create_publisher(Bool, '/package/motioncallback', qos)
        self.execute_callback_pub = self.create_publisher(Bool, '/package/executecallback', qos)
        self.anchor_reset_pub = self.create_publisher(Bool, '/walking_reset_anchor', 10)
        self.get_logger().info("Motion Strategy Node initialized. Waiting for Driver...")

        home_path = os.path.expanduser("~")
        self.stand_dir = os.path.join(home_path, "nstc/config/Parameter")
        self.stand_file = os.path.join(self.stand_dir, "stand.ini")
        self.ini29_file = os.path.join(self.stand_dir, "29.ini")

        # 啟動時先讀 strategy.ini，確保 location_folder 正確，再載入動作檔
        strategy_ini_path = os.path.join(home_path, "nstc/config/strategy.ini")
        try:
            if os.path.exists(strategy_ini_path):
                with open(strategy_ini_path, 'r', encoding='utf-8') as f:
                    content = f.read().strip()
                if "Parameter" in content:
                    new_loc = content.split('Parameter')[0].replace("/", "").strip()
                else:
                    new_loc = content.replace("/", "").strip()
                if new_loc and new_loc != self.location_folder:
                    self.get_logger().info(f"[Init] strategy.ini 指定策略: {self.location_folder} -> {new_loc}")
                    self.location_folder = new_loc
        except Exception as e:
            self.get_logger().warn(f"[Init] 讀取 strategy.ini 失敗，使用預設: {e}")

        self.load_startup_stand_motion()
        self.load_all_strategy_motions()

        # --- Undo / Redo ---
        self._undo_speed = 50
        self._special_undo_speeds = {16:3478,17:3478,18:3478,19:3478,22:3478,23:3478,24:3478,25:3478}  # {motor_id: speed}，特殊馬達的 undo 速度
        self._max_history = 20
        self._history_stack = []   # entries: {'state': {motor_id: pos}, 'label': str}
        self._future_stack = []
        self._in_motion_list = False
        self._executing_sector_id = "?"
        self._orig_ep_ref  = None
        self._orig_eml_ref = None
        self._ml_step_captures = None

        # Record mode
        self.record_load_resp_pub  = self.create_publisher(String, '/package/RecordLoadResponse', 10)
        self.record_result_pub     = self.create_publisher(String, '/package/RecordExportResult', 10)
        self.create_subscription(String, '/package/RecordPlayFrame',  self.cb_record_play,   10)
        self.create_subscription(String, '/package/RecordSave',       self.cb_record_save,   10)
        self.create_subscription(String, '/package/RecordLoadRequest',self.cb_record_load,   10)
        self.create_subscription(String, '/package/RecordExport',     self.cb_record_export, 10)

        self.history_state_pub = self.create_publisher(Int16MultiArray, '/package/HistoryState', 10)
        self.undo_status_pub   = self.create_publisher(String, '/package/UndoRedoStatus', 10)
        self.history_info_pub  = self.create_publisher(String, '/package/HistoryInfo', 10)
        self.create_subscription(Bool, '/package/Undo', self.cb_undo, 10)
        self.create_subscription(Bool, '/package/Redo', self.cb_redo, 10)
        self.create_subscription(Bool, '/package/ClearHistory', self.cb_clear_history, 10)

        # 重建 /package/Sector 訂閱，在執行前記錄 sector 名稱
        self.destroy_subscription(self.sector_execute_sub)
        def _track_and_execute(msg):
            sector_id = str(msg.data)
            self._executing_sector_id = "Stand" if sector_id == "29" else f"Sector {sector_id}"
            self.cb_sector_execute(msg)
        self.sector_execute_sub = self.create_subscription(
            Int16, '/package/Sector', _track_and_execute, QoSProfile(depth=1000))

        _orig_ep = self.execute_pose
        self._orig_ep_ref = _orig_ep
        def _wrapped_ep(data, mode, trigger_callback=True):
            if not self._in_motion_list:
                self._push_history(mode=mode, exec_data=list(data))
                self._future_stack.clear()
                self._publish_history_state()
            else:
                if self._ml_step_captures is not None:
                    self._ml_step_captures.append({
                        'opcode':    242 if mode == 'ABSOLUTE' else 243,
                        'data':      list(data),
                        'pre_state': dict(self.last_goals),
                    })
            _orig_ep(data, mode, trigger_callback)
            if not self._in_motion_list and self._history_stack:
                speeds = {i + 1: data[i * 2] for i in range(len(data) // 2)}
                self._history_stack[-1]['speeds'] = speeds
        self.execute_pose = _wrapped_ep

        _orig_eml = self.execute_motion_list
        self._orig_eml_ref = _orig_eml
        def _wrapped_eml(data):
            self._push_history(motion_data=data)
            self._future_stack.clear()
            self._publish_history_state()
            self._in_motion_list = True
            self._ml_step_captures = []
            _orig_eml(data)
            captures = list(self._ml_step_captures)
            self._ml_step_captures = None
            self._in_motion_list = False
            undo_steps = []
            cap_idx = 0
            for i in range(0, len(data), 2):
                if i + 1 >= len(data): break
                sec_id_str = str(data[i])
                delay_ms   = data[i + 1]
                if sec_id_str == '0':
                    continue
                if sec_id_str in self.saved_sectors and cap_idx < len(captures):
                    step = captures[cap_idx]
                    step['delay_ms'] = delay_ms
                    undo_steps.append(step)
                    cap_idx += 1
            if self._history_stack:
                self._history_stack[-1]['undo_steps'] = undo_steps
        self.execute_motion_list = _wrapped_eml

    # ==========================================================
    # Web Communication
    # ==========================================================
    def web_interface_cb(self, msg):
        self.get_logger().info(f"[DEBUG] 收到網頁數據: {msg.package}")
        if msg.sectorname: self.current_sector_name = msg.sectorname

        self.web_buffer.extend(msg.package)
        self.parse_and_process_buffer()

    def parse_and_process_buffer(self):
        while len(self.web_buffer) >= 3:
            if self.web_buffer[0] != 83 or self.web_buffer[1] != 84:
                self.web_buffer.pop(0)
                continue

            opcode = self.web_buffer[2]
            expected_len = 0
            if opcode == 246: expected_len = 7
            elif opcode == 241: expected_len = 59  # adult: Absolute (Lockedstand unchecked)
            elif opcode == 242: expected_len = 59  # adult: Absolute (Lockedstand checked)
            elif opcode == 243: expected_len = 59  # adult: Relative
            elif opcode == 244: expected_len = 45
            elif opcode == 245: expected_len = 59
            else:
                self.get_logger().warn(f"未知 Opcode: {opcode}")
                self.web_buffer.pop(0)
                continue

            if len(self.web_buffer) < expected_len: return

            if self.web_buffer[expected_len-2] == 78 and self.web_buffer[expected_len-1] == 69:
                packet = self.web_buffer[:expected_len]
                self.process_packet_data(packet, opcode)
                self.web_buffer = self.web_buffer[expected_len:]
            else:
                self.web_buffer.pop(0)

    def process_packet_data(self, packet, opcode):
        if opcode == 246: # Torque Control
            motor_id = packet[3]
            state = packet[4]
            self.get_logger().info(f"★ 執行扭力指令 -> ID: {motor_id}, State: {state}")
            msg = Int16MultiArray()
            msg.data = [motor_id, state]
            self.torque_pub.publish(msg)

            # 恢復扭力時，將 last_goals 同步至當前實際位置，
            # 確保後續相對動作以正確刻度為基準
            if state == 1:
                with self.joints_lock:
                    snapshot = dict(self.current_joints)
                if motor_id == 0:
                    self.last_goals.update(snapshot)
                elif motor_id in snapshot:
                    self.last_goals[motor_id] = snapshot[motor_id]
                self.get_logger().info(f"[Torque ON] last_goals 已同步，ID={motor_id}")
                # 通知 walking node 重新定錨，避免 baseline 使用舊資料
                reset_msg = Bool()
                reset_msg.data = True
                self.anchor_reset_pub.publish(reset_msg)

        elif opcode in [241, 242, 243, 244]: # Save Sector Data (RAM) & Backup to Sector Folder
            data_content = list(packet[3:-2])
            sector_id = str(self.current_sector_name)

            # 1. 更新記憶體
            self.saved_sectors[sector_id] = {
                'opcode': opcode,
                'data': data_content
            }

            # 2. 自動備份到 sector 資料夾
            self.save_sector_to_disk(sector_id, opcode, data_content)

            # 3. 回報網頁
            msg = Bool(); msg.data = True; self.motion_callback_pub.publish(msg)

    # ==========================================================
    # 單一 Sector 存檔功能
    # ==========================================================
    def save_sector_to_disk(self, sector_id, opcode, data_list):
        is_empty_data = False

        if opcode in [241, 242, 243]: # Absolute or Relative
            positions = data_list[1::2]
            if all(p == 0 for p in positions):
                is_empty_data = True

        elif opcode == 244:
            if all(v == 0 for v in data_list):
                is_empty_data = True

        if is_empty_data:
            self.get_logger().info(f"[Sector Save] ID {sector_id} (Opcode {opcode}) 數值全為 0，忽略存檔。")
            return

        try:
            # sector 29 存到公用資料夾（與 stand.ini 同層），其餘存到策略專屬 sector/ 子資料夾
            if sector_id == "29":
                file_path = self.ini29_file
            else:
                home_path = os.path.expanduser("~")
                sector_dir = os.path.join(home_path, "nstc/config", self.location_folder, "Parameter", "sector")
                if not os.path.exists(sector_dir):
                    os.makedirs(sector_dir)
                file_path = os.path.join(sector_dir, f"{sector_id}.ini")

            config = configparser.ConfigParser()
            config['Data'] = {}
            section = config['Data']

            section['id'] = str(sector_id)

            if opcode == 244: # MotionList
                section['motionstate'] = '0'
                section['motionlist'] = ','.join(map(str, data_list))
            elif opcode == 243: # Relative
                section['motionstate'] = '2'
                section['motordata'] = ','.join(map(str, data_list))
            elif opcode in [241, 242]: # Absolute
                section['motionstate'] = '4'
                section['motordata'] = ','.join(map(str, data_list))

            with open(file_path, 'w', encoding='utf-8') as f:
                config.write(f)

            self.get_logger().info(f"[Sector Save] ID {sector_id} 已備份至: {file_path}")

        except Exception as e:
            self.get_logger().error(f"[Sector Save] 存檔失敗: {e}")

    # ==========================================================
    # Initialization & File Loading
    # ==========================================================
    def load_startup_stand_motion(self):
        full_path = self.stand_file
        if not os.path.exists(full_path):
            self.get_logger().warn(f"[Init] 找不到 Stand 檔案: {full_path}")
            return
        self.get_logger().info(f"[Init] 載入 Stand: {full_path}")
        self._internal_load_ini(full_path, is_common=True)
        # 若 29.ini 存在則覆蓋 saved_sectors["29"]（working copy）
        if os.path.exists(self.ini29_file):
            self.get_logger().info(f"[Init] 載入 29.ini: {self.ini29_file}")
            self._internal_load_ini(self.ini29_file, "29.ini", is_common=True)

    def load_all_strategy_motions(self):
        home_path = os.path.expanduser("~")
        base_strategy_path = os.path.join(home_path, "nstc/config")

        path_common = os.path.join(base_strategy_path, "common", "Parameter")
        if os.path.exists(path_common):
            self.get_logger().info(f"[Init] 載入 Common: {path_common}")
            self._load_folder(path_common, is_common=True)
            path_common_sector = os.path.join(path_common, "sector")
            if os.path.exists(path_common_sector):
                self._load_folder(path_common_sector, is_common=True)

        path_specific = os.path.join(base_strategy_path, self.location_folder, "Parameter")
        if os.path.exists(path_specific):
            self.get_logger().info(f"[Init] 載入 Specific ({self.location_folder}): {path_specific}")
            self._load_folder(path_specific, is_common=False)
            path_specific_sector = os.path.join(path_specific, "sector")
            if os.path.exists(path_specific_sector):
                self.get_logger().info(f"[Init] 載入 Sector 資料夾: {path_specific_sector}")
                self._load_folder(path_specific_sector, is_common=False)

    def _load_folder(self, folder_path, is_common):
        try:
            files = [f for f in os.listdir(folder_path) if f.endswith('.ini')]
            for filename in files:
                self._internal_load_ini(os.path.join(folder_path, filename), filename, is_common)
        except Exception as e:
            self.get_logger().error(f"Folder Load Error: {e}")

    def _internal_load_ini(self, full_path, filename="unknown", is_common=False):
        config = configparser.ConfigParser()
        try:
            config.read(full_path)
            temp_load = {}
            for section in config.sections():
                try:
                    m_state = int(config[section]['motionstate'])
                    m_id = int(config[section]['id'])
                    def parse_list(key):
                        if key in config[section] and config[section][key]:
                            return [int(x) for x in config[section][key].split(',')]
                        return []
                    data_array = []
                    if m_state == 0: data_array = parse_list('motionlist')
                    elif m_state in [1, 2, 3, 4]: data_array = parse_list('motordata')

                    if m_id not in temp_load: temp_load[m_id] = {}
                    temp_load[m_id][m_state] = data_array
                except: pass

            self.auto_load_sectors(temp_load, filename, is_common)

        except Exception as e:
            self.get_logger().error(f"INI Parse Error ({full_path}): {e}")

    def auto_load_sectors(self, temp_load, source_filename="unknown", is_common=False):
        for m_id, data_map in temp_load.items():
            sector_name = str(m_id)
            if sector_name in self.saved_sectors:
                if not is_common: pass

            self.id_source_map[sector_name] = source_filename

            if 3 in data_map and 4 in data_map:
                pos_list = data_map[3]; spd_list = data_map[4]
                merged = []; [merged.extend([s,p]) for p,s in zip(pos_list, spd_list)]
                self.saved_sectors[sector_name] = {'opcode': 242, 'data': merged}
            elif 4 in data_map:  # sector 檔的 merged absolute 格式 [spd,pos,...]
                self.saved_sectors[sector_name] = {'opcode': 242, 'data': data_map[4]}
            elif 1 in data_map and 2 in data_map:
                pos_list = data_map[1]; spd_list = data_map[2]
                merged = []; [merged.extend([s,p]) for p,s in zip(pos_list, spd_list)]
                self.saved_sectors[sector_name] = {'opcode': 243, 'data': merged}
            elif 2 in data_map:  # sector 檔的 merged relative 格式
                self.saved_sectors[sector_name] = {'opcode': 243, 'data': data_map[2]}
            elif 0 in data_map:
                self.saved_sectors[sector_name] = {'opcode': 244, 'data': data_map[0]}

        self.get_logger().info(f"[AutoLoad] 載入 {len(temp_load)} 個 Sector ({source_filename})")

    # ==========================================================
    # Read / Save / Callbacks
    # ==========================================================
    def cb_torque_state(self, msg: Int16MultiArray):
        """記錄每次扭力開關的結果。ID 0 代表全部馬達。"""
        if len(msg.data) < 2:
            return
        motor_id, state = int(msg.data[0]), 1 if int(msg.data[1]) else 0
        if motor_id == 0:
            self.torque_states = [state] * self.MOTOR_ID_MAX
        elif 1 <= motor_id <= self.MOTOR_ID_MAX:
            self.torque_states[motor_id - 1] = state

    def cb_get_torque_state(self, request, response):
        """回傳扭力狀態快取，供網頁開啟時初始化介面。"""
        del request
        response.states = list(self.torque_states)
        return response

    def cb_read_motion(self, request, response):
        home_path = os.path.expanduser("~")
        strategy_ini_path = os.path.join(home_path, "nstc/config/strategy.ini")

        # request.strategy 指定時，只是「借讀」別的策略的檔案（例如 Import Sector），
        # 不應該切換本節點的策略，也不該觸發重載 —— 直接沿用 location_folder 以外的路徑即可。
        borrow_strategy = (getattr(request, "strategy", "") or "").strip().strip("/")

        # 借讀時完全不碰 strategy.ini，避免誤判成策略切換而觸發整批重載
        try:
            if not borrow_strategy and os.path.exists(strategy_ini_path):
                with open(strategy_ini_path, 'r', encoding='utf-8') as f:
                    content = f.read().strip()

                new_loc = ""
                if "Parameter" in content:
                    parts = content.split('Parameter')
                    raw_loc = parts[0]
                    new_loc = raw_loc.replace("/", "").strip()
                else:
                    new_loc = content.replace("/", "").strip()

                if new_loc and new_loc != self.location_folder:
                    self.get_logger().info(f"🔄 原始內容 '{content}' -> 解析為 '{new_loc}'")
                    self.get_logger().info(f"🔄 偵測到策略變更: {self.location_folder} -> {new_loc}")

                    self.location_folder = new_loc

                    with self.joints_lock:
                        self.saved_sectors.clear()
                        self.id_source_map.clear()

                    # 清空歷史（策略切換後歷史無效）
                    self._history_stack.clear()
                    self._future_stack.clear()
                    self._publish_history_state()

                    self.load_startup_stand_motion()
                    self.load_all_strategy_motions()
                    self.get_logger().info(f"✅ 已切換至 {self.location_folder} 並完成重載")

        except Exception as e:
            self.get_logger().warn(f"[Check Strategy] 讀取 strategy.ini 失敗: {e}")

        filename = request.name
        read_state = request.readstate

        if read_state == 1:
            base_path = self.stand_dir
        else:
            folder = borrow_strategy or self.location_folder
            if borrow_strategy:
                self.get_logger().info(f"[Read] 借讀策略 '{borrow_strategy}'（不切換目前策略）")
            base_path = os.path.join(home_path, "nstc/config", folder, "Parameter")

        full_path = os.path.join(base_path, f"{filename}.ini")
        self.get_logger().info(f"[Read] 嘗試讀取檔案: {full_path}")

        if not os.path.exists(full_path):
            self.get_logger().error(f"[Read] 找不到檔案: {full_path}")
            response.readcheck = False
            return response

        # 1. 載入到 RAM
        self._internal_load_ini(full_path, f"{filename}.ini", is_common=False)

        # 2. 回填 Response
        config = configparser.ConfigParser()
        try:
            config.read(full_path)
            response.vectorcnt = 0
            response.motionstate = []
            response.id = []
            response.motionlist = []
            response.relativedata = []
            response.absolutedata = []
            response.item_names = []

            for section in config.sections():
                try:
                    m_id = int(config[section]['id'])
                    m_state = int(config[section]['motionstate'])
                    response.vectorcnt += 1
                    response.id.append(m_id)
                    response.motionstate.append(m_state)
                    response.item_names.append(config[section].get('item_name', ''))

                    def parse_list(key):
                        if key in config[section] and config[section][key]:
                            return [int(x) for x in config[section][key].split(',')]
                        return []

                    if m_state == 0: response.motionlist.extend(parse_list('motionlist'))
                    elif m_state in [1, 2]: response.relativedata.extend(parse_list('motordata'))
                    elif m_state in [3, 4]: response.absolutedata.extend(parse_list('motordata'))
                except: pass

            response.readcheck = True
            self.get_logger().info(f"[Read] 回傳 {response.vectorcnt} 筆數據給網頁")
        except Exception as e:
            self.get_logger().error(f"[Read] Response 打包錯誤: {e}")
            response.readcheck = False

        return response

    def joint_state_cb(self, msg: JointState):
        with self.joints_lock:
            for i, name in enumerate(msg.name):
                try: self.current_joints[int(name)] = int(msg.position[i])
                except: pass

    def publish_command(self, target_joints_dict):
        if not target_joints_dict: return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []

        for mid, (pos, vel) in target_joints_dict.items():
            msg.name.append(str(mid))
            msg.position.append(float(pos))
            msg.velocity.append(float(vel))

        self.cmd_pub.publish(msg)

    def cb_single_motor(self, msg: SingleMotorData):
        mid = int(msg.id)
        pos = int(msg.position)
        spd = int(msg.speed)

        if mid in self.current_joints:
            base_pos = self.current_joints[mid]
        else:
            base_pos = self.last_goals.get(mid, 2048)
            self.get_logger().warn(f"[SingleMotor] Motor {mid} no feedback, using last goal/default.")

        final_target = base_pos + pos

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [str(mid)]
        js.position = [float(final_target)]
        js.velocity = [float(spd)]
        self.cmd_pub.publish(js)

        self.last_goals[mid] = final_target

        reset_msg = Bool()
        reset_msg.data = True
        self.anchor_reset_pub.publish(reset_msg)

        self.get_logger().info(f"[SingleMotor] ID={mid} Base={base_pos} Target={final_target} (Rel={pos})")

    def cb_SingleAbsolutePosition(self, msg: SingleMotorData):
        mid = int(msg.id)
        pos = int(msg.position)
        spd = int(msg.speed)

        if mid in self.current_joints:
            base_pos = self.current_joints[mid]
        else:
            base_pos = self.last_goals.get(mid, 2048)
            self.get_logger().warn(f"[SingleMotor] Motor {mid} no feedback, using last goal/default.")

        final_target = pos

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [str(mid)]
        js.position = [float(final_target)]
        js.velocity = [float(spd)]
        self.cmd_pub.publish(js)

        self.last_goals[mid] = final_target

        reset_msg = Bool()
        reset_msg.data = True
        self.anchor_reset_pub.publish(reset_msg)

        self.get_logger().info(f"[SingleMotor] ID={mid} Base={base_pos} Target={final_target} (Rel={pos})")

    def cb_param_update(self, params):
        for p in params:
            if p.name == "location" and p.type_ == p.Type.STRING:
                new_location = p.value

                if new_location != self.location_folder:
                    self.get_logger().info(f"收到策略切換指令: {self.location_folder} -> {new_location}")

                    self.location_folder = new_location

                    with self.joints_lock:
                        self.saved_sectors.clear()
                        self.id_source_map.clear()

                    # 清空歷史（策略切換後歷史無效）
                    self._history_stack.clear()
                    self._future_stack.clear()
                    self._publish_history_state()

                    self.get_logger().info("正在重新載入策略檔案...")
                    self.load_startup_stand_motion()
                    self.load_all_strategy_motions()

                    self.get_logger().info(f"✅ 策略已切換為: {self.location_folder}")

        return SetParametersResult(successful=True)

    def cb_save_motion(self, msg):
        filename = msg.name if msg.name else "untitled"
        if filename not in self.file_save_buffer: self.file_save_buffer[filename] = []
        if msg.saveflag:
            self.write_to_ini(filename, msg.savestate)
            self.file_save_buffer[filename] = []
        else:
            record = {'savestate': msg.savestate, 'motionstate': msg.motionstate, 'id': msg.id,
                      'item_name': getattr(msg, 'item_name', ''), 'motionlist': list(msg.motionlist), 'motordata': list(msg.motordata)}
            self.file_save_buffer[filename].append(record)

    def write_to_ini(self, filename, savestate):
        if savestate == 1:
            base_path = self.stand_dir
        else:
            home_path = os.path.expanduser("~")
            base_path = os.path.join(home_path, "nstc/config", self.location_folder, "Parameter")

        if not os.path.exists(base_path): os.makedirs(base_path)
        full_path = os.path.join(base_path, f"{filename}.ini")

        database_to_save = {}

        if filename in self.file_save_buffer:
            for record in self.file_save_buffer[filename]:
                data_entry = {
                    'savestate': str(record['savestate']),
                    'motionstate': str(record['motionstate']),
                    'id': str(record['id']),
                    'item_name': str(record['item_name']),
                    'motionlist': ','.join(map(str, record['motionlist'])),
                    'motordata': ','.join(map(str, record['motordata']))
                }
                key = f"{record['id']}_{record['motionstate']}"
                database_to_save[key] = data_entry

        new_config = configparser.ConfigParser()

        for idx, key in enumerate(database_to_save):
            new_config[str(idx)] = database_to_save[key]

        try:
            with open(full_path, 'w', encoding='utf-8') as f:
                new_config.write(f)
            self.get_logger().info(f"Saved (Overwrite Mode): {full_path} | 筆數: {len(database_to_save)}")
        except Exception as e:
            self.get_logger().error(f"Save Failed: {e}")

        # SaveStand 同時寫入 29.ini（working copy 與 stand.ini 同步）
        if savestate == 1:
            try:
                with open(self.ini29_file, 'w', encoding='utf-8') as f:
                    new_config.write(f)
                self.get_logger().info(f"Saved 29.ini: {self.ini29_file}")
                self._internal_load_ini(self.ini29_file, "29.ini", is_common=True)
            except Exception as e:
                self.get_logger().error(f"29.ini Save Failed: {e}")

    def cb_check_sector(self, request, response):
        response.checkflag = str(request.data) in self.saved_sectors
        return response

    # Stand 按鈕專用：直接從 stand.ini 讀取並執行，不修改 saved_sectors
    def cb_stand_execute(self, msg):
        self.get_logger().info("[Stand] Executing from stand.ini")
        self._executing_sector_id = "Stand"
        self._execute_pose_from_file(self.stand_file)

    def _execute_pose_from_file(self, file_path):
        config = configparser.ConfigParser()
        try:
            config.read(file_path)
            temp = {}
            for section in config.sections():
                try:
                    m_state = int(config[section]['motionstate'])
                    m_id = int(config[section]['id'])
                    def parse_list(key, sec=section):
                        if key in config[sec] and config[sec][key]:
                            return [int(x) for x in config[sec][key].split(',')]
                        return []
                    if m_id not in temp: temp[m_id] = {}
                    if m_state in [1, 2, 3, 4]: temp[m_id][m_state] = parse_list('motordata')
                    elif m_state == 0: temp[m_id][m_state] = parse_list('motionlist')
                except: pass

            for m_id, data_map in temp.items():
                if 3 in data_map and 4 in data_map:
                    pos_list = data_map[3]; spd_list = data_map[4]
                    merged = []; [merged.extend([s, p]) for p, s in zip(pos_list, spd_list)]
                    self.execute_pose(merged, "ABSOLUTE")
                    return
                elif 4 in data_map:
                    self.execute_pose(data_map[4], "ABSOLUTE")
                    return
            self.get_logger().error(f"[Stand] No valid pose in {file_path}")
        except Exception as e:
            self.get_logger().error(f"[Stand] Read error {file_path}: {e}")

    def cb_sector_execute(self, msg):
        sector_id = str(msg.data)

        # Execute 29 → 從 29.ini 重新載入（working copy）
        if sector_id == "29":
            if os.path.exists(self.ini29_file):
                self.get_logger().info(f"[Execute29] Reloading 29.ini: {self.ini29_file}")
                self._internal_load_ini(self.ini29_file, "29.ini", is_common=True)
            else:
                self.get_logger().info("[Execute29] 29.ini not found, using stand.ini fallback in RAM")

        if sector_id in self.saved_sectors:
            record = self.saved_sectors[sector_id]
            if record['opcode'] in [241, 242]:
                self.execute_pose(record['data'], "ABSOLUTE")
            elif record['opcode'] == 243:
                self.execute_pose(record['data'], "RELATIVE")
            elif record['opcode'] == 244:
                self.execute_motion_list(record['data'])
        else:
            self.get_logger().warn(f"[Execute] Sector ID {sector_id} not found in RAM (saved_sectors).")

    def execute_motion_list(self, data):
        self.get_logger().info(f"[MotionList] 收到清單長度: {len(data)}, 內容: {data}")
        for i in range(0, len(data), 2):
            if i+1 >= len(data): break
            sec_id = str(data[i])
            delay = float(data[i+1])/1000.0

            self.get_logger().info(f" -> Step {i//2 + 1}: ID={sec_id}, Delay={data[i+1]}")

            if sec_id == '0':
                self.get_logger().info(" -> 遇到 ID 0，跳過動作")
            elif sec_id in self.saved_sectors:
                rec = self.saved_sectors[sec_id]
                self.execute_pose(rec['data'], "ABSOLUTE" if rec['opcode'] in [241, 242] else "RELATIVE", False)
            else:
                self.get_logger().warn(f" -> [警告] 找不到 Sector ID: {sec_id}，跳過！")

            time.sleep(max(0.05, delay))
        self.get_logger().info("[MotionList] 執行完畢，發送 Callback")
        msg = Bool(); msg.data = True; self.execute_callback_pub.publish(msg)

    def execute_pose(self, data, mode, trigger_callback=True):
        count = len(data) // 2
        target_joints = {}

        for i in range(count):
            raw_spd = data[i*2]
            raw_val = data[i*2+1]

            # adult: 大刻度馬達數值可到 3 萬多，閾值設高避免誤判為負數
            if raw_val > 1000000: raw_val -= 65536
            if mode == "ABSOLUTE" and raw_val == -1:
                continue
            mid = i + 1

            if mode == "RELATIVE":
                if raw_val == 0:
                    continue  # delta=0 不發指令，也不更新 last_goals
                # 統一用 last_goals 當 base，避免感測器回授誤差累積；
                # last_goals 在每次 execute_pose 後即時更新，motion list 內多步也能正確接續
                base_pos = self.last_goals.get(mid, self.current_joints.get(mid, 2048))
                final_target = base_pos + raw_val
            else:
                final_target = raw_val

            target_joints[mid] = (final_target, raw_spd)
            self.last_goals[mid] = final_target

        self.publish_command(target_joints)

        reset_msg = Bool()
        reset_msg.data = True
        self.anchor_reset_pub.publish(reset_msg)
        self.get_logger().info(f"[Notify] Mode: {mode}, Anchor Reset Flag sent to Walking Node.")

        if trigger_callback:
            msg = Bool()
            msg.data = True
            self.execute_callback_pub.publish(msg)

    # ==========================================================
    # Undo / Redo
    # ==========================================================
    def _push_history(self, motion_data=None, mode=None, exec_data=None):
        entry = {
            'state':       dict(self.last_goals),
            'label':       self._executing_sector_id,
            'speeds':      {},
            'type':        'motion_list' if motion_data is not None else 'pose',
            'motion_data': list(motion_data) if motion_data is not None else None,
            'exec_mode':   mode,
            'exec_data':   exec_data,
            'undo_steps':  None,
        }
        self._history_stack.append(entry)
        if len(self._history_stack) > self._max_history:
            self._history_stack.pop(0)

    def _undo_motion_list(self, undo_steps):
        for step in reversed(undo_steps):
            if step['opcode'] == 243:  # Relative：反向 delta，固定 undo 速度
                data = step['data']
                reverse_data = []
                for i in range(len(data) // 2):
                    reverse_data.extend([self._get_undo_speed(i + 1), -data[i * 2 + 1]])
                self._orig_ep_ref(reverse_data, 'RELATIVE', False)
            else:                      # Absolute：還原到該步執行前的刻度
                self._restore_state(step['pre_state'])
            time.sleep(max(0.05, step['delay_ms'] / 1000.0))

    def _get_undo_speed(self, motor_id):
        return self._special_undo_speeds.get(motor_id, self._undo_speed)

    def _publish_history_state(self):
        msg = Int16MultiArray()
        msg.data = [len(self._history_stack), len(self._future_stack)]
        self.history_state_pub.publish(msg)

        info = {
            'history': [e['label'] for e in self._history_stack],
            'future':  [e['label'] for e in self._future_stack]
        }
        info_msg = String()
        info_msg.data = json.dumps(info)
        self.history_info_pub.publish(info_msg)

    def cb_undo(self, _msg):
        if not self._history_stack:
            self.get_logger().warn("[Undo] History is empty.")
            return
        prev_entry = self._history_stack.pop()
        self._future_stack.append({
            'state':       dict(self.last_goals),
            'label':       prev_entry['label'],
            'speeds':      prev_entry.get('speeds', {}),
            'type':        prev_entry.get('type', 'pose'),
            'motion_data': prev_entry.get('motion_data'),
            'exec_mode':   prev_entry.get('exec_mode'),
            'exec_data':   prev_entry.get('exec_data'),
            'undo_steps':  prev_entry.get('undo_steps'),
        })
        if prev_entry.get('type') == 'motion_list' and prev_entry.get('undo_steps'):
            self._undo_motion_list(prev_entry['undo_steps'])
        elif prev_entry.get('exec_mode') == 'RELATIVE' and prev_entry.get('exec_data'):
            data = prev_entry['exec_data']
            reverse_data = []
            for i in range(len(data) // 2):
                reverse_data.extend([self._get_undo_speed(i + 1), -data[i * 2 + 1]])
            self._orig_ep_ref(reverse_data, 'RELATIVE', False)
        else:
            self._restore_state(prev_entry['state'])

        status = String()
        status.data = f"↩ Undo: {prev_entry['label']}"
        self.undo_status_pub.publish(status)
        self.get_logger().info(f"[Undo] {status.data} | History: {len(self._history_stack)}, Future: {len(self._future_stack)}")
        self._publish_history_state()

    def cb_redo(self, _msg):
        if not self._future_stack:
            self.get_logger().warn("[Redo] Future is empty.")
            return
        next_entry = self._future_stack.pop()
        self._history_stack.append({
            'state':       dict(self.last_goals),
            'label':       next_entry['label'],
            'speeds':      next_entry.get('speeds', {}),
            'type':        next_entry.get('type', 'pose'),
            'motion_data': next_entry.get('motion_data'),
            'exec_mode':   next_entry.get('exec_mode'),
            'exec_data':   next_entry.get('exec_data'),
            'undo_steps':  next_entry.get('undo_steps'),
        })
        if next_entry.get('type') == 'motion_list' and next_entry.get('motion_data'):
            self._in_motion_list = True
            self._orig_eml_ref(next_entry['motion_data'])
            self._in_motion_list = False
        elif next_entry.get('exec_data') and next_entry.get('exec_mode'):
            self._orig_ep_ref(next_entry['exec_data'], next_entry['exec_mode'], False)
        else:
            self._restore_state(next_entry['state'], speeds=next_entry.get('speeds'))

        status = String()
        status.data = f"↪ Redo: {next_entry['label']}"
        self.undo_status_pub.publish(status)
        self.get_logger().info(f"[Redo] {status.data} | History: {len(self._history_stack)}, Future: {len(self._future_stack)}")
        self._publish_history_state()

    def cb_clear_history(self, _msg):
        self._history_stack.clear()
        self._future_stack.clear()
        self._publish_history_state()
        status = String()
        status.data = "History cleared !!"
        self.undo_status_pub.publish(status)
        self.get_logger().info("[ClearHistory] History and future cleared.")

    def _restore_state(self, snapshot, speeds=None):
        if not snapshot:
            self.get_logger().warn("[Restore] Snapshot is empty, nothing to restore.")
            return
        if speeds:
            target_joints = {mid: (pos, speeds.get(mid, self._get_undo_speed(mid))) for mid, pos in snapshot.items()}
        else:
            target_joints = {mid: (pos, self._get_undo_speed(mid)) for mid, pos in snapshot.items()}
        self.publish_command(target_joints)
        self.last_goals.update(snapshot)
        reset_msg = Bool()
        reset_msg.data = True
        self.anchor_reset_pub.publish(reset_msg)

    # ==========================================================
    # Record Mode
    # ==========================================================
    def cb_record_play(self, msg):
        try:
            p = json.loads(msg.data)
            num_motors  = p.get('num_motors', 27)
            selected    = set(int(x) for x in p.get('selected', []))
            motors_data = p.get('motors', {})
            data = []
            for mid in range(1, num_motors + 1):
                key = str(mid)
                if mid in selected and key in motors_data:
                    m   = motors_data[key]
                    spd = max(50, int(m.get('vel', 50)))
                    data.extend([spd, int(m['pos'])])
                else:
                    data.extend([0, -1])
            self.execute_pose(data, 'ABSOLUTE', False)
        except Exception as e:
            self.get_logger().error(f'[RecordPlay] {e}')

    def cb_record_save(self, msg):
        try:
            payload = json.loads(msg.data)
            name    = payload.get('name', 'recording').strip()
            folder  = os.path.join(os.path.expanduser('~'),
                                   'nstc/config',
                                   self.location_folder, 'Parameter')
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, f'{name}.rec.json')
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
            resp      = String()
            resp.data = json.dumps({'save_ok': True, 'path': path})
            self.record_result_pub.publish(resp)
            self.get_logger().info(f'[RecordSave] {path}')
        except Exception as e:
            self.get_logger().error(f'[RecordSave] {e}')

    def cb_record_load(self, msg):
        try:
            name   = msg.data.strip()
            folder = os.path.join(os.path.expanduser('~'),
                                  'nstc/config',
                                  self.location_folder, 'Parameter')
            path   = os.path.join(folder, f'{name}.rec.json')
            resp   = String()
            if not os.path.exists(path):
                resp.data = json.dumps({'error': f'{name}.rec.json 不存在'})
            else:
                with open(path, 'r', encoding='utf-8') as f:
                    resp.data = f.read()
            self.record_load_resp_pub.publish(resp)
            self.get_logger().info(f'[RecordLoad] {path}')
        except Exception as e:
            self.get_logger().error(f'[RecordLoad] {e}')

    def cb_record_export(self, msg):
        try:
            p          = json.loads(msg.data)
            # 新格式: motion_list_id + frame_start_id
            # 舊格式向下相容: base_id → motion_list_id=base_id, frame_start_id=base_id+1
            if 'motion_list_id' in p:
                ml_id       = int(p['motion_list_id'])
                frame_start = int(p['frame_start_id'])
            else:
                ml_id       = int(p['base_id'])
                frame_start = ml_id + 1

            def_spd    = int(p.get('default_speed', 50))
            num_motors = int(p.get('num_motors', 27))
            selected   = set(int(x) for x in p.get('selected_motors', []))
            frames     = p.get('frames', [])
            resp       = String()

            if ml_id < 1 or frame_start < 1000:
                resp.data = json.dumps({'error': 'Frame Start ID 須 ≥ 1000'})
                self.record_result_pub.publish(resp)
                return

            if ml_id == frame_start:
                resp.data = json.dumps({'error': 'MotionList ID 不可等於 Frame Start ID'})
                self.record_result_pub.publish(resp)
                return

            sector_dir = os.path.join(os.path.expanduser('~'),
                                      'nstc/config',
                                      self.location_folder, 'Parameter', 'sector')
            os.makedirs(sector_dir, exist_ok=True)

            # 衝突檢查：以磁碟檔案為準（RAM 可能有舊資料殘留）
            all_ids   = [ml_id] + [frame_start + i for i in range(len(frames))]
            conflicts = [i for i in all_ids if os.path.exists(os.path.join(sector_dir, f'{i}.ini'))]
            if conflicts:
                preview = conflicts[:5]
                suffix  = '...' if len(conflicts) > 5 else ''
                resp.data = json.dumps({'error': f'磁碟 ID 衝突: {preview}{suffix}（共 {len(conflicts)} 個）'})
                self.record_result_pub.publish(resp)
                return

            ml_data   = []
            frame_ids = []
            for i, frame in enumerate(frames):
                sector_id   = frame_start + i
                motors_data = frame.get('motors', {})
                data = []
                for mid in range(1, num_motors + 1):
                    key = str(mid)
                    if mid in selected and key in motors_data:
                        m   = motors_data[key]
                        spd = max(def_spd, int(m.get('vel', def_spd)))
                        data.extend([spd, int(m['pos'])])
                    else:
                        data.extend([0, -1])

                self.saved_sectors[str(sector_id)] = {'opcode': 242, 'data': data}
                self.save_sector_to_disk(str(sector_id), 242, data)

                delay = int(frames[i+1]['t_ms'] - frame['t_ms']) if i < len(frames)-1 else 200
                ml_data.extend([sector_id, delay])
                frame_ids.append(sector_id)

            self.saved_sectors[str(ml_id)] = {'opcode': 244, 'data': ml_data}
            self.save_sector_to_disk(str(ml_id), 244, ml_data)

            resp.data = json.dumps({'status': 'ok', 'motion_list_id': ml_id, 'frame_ids': frame_ids})
            self.record_result_pub.publish(resp)
            self.get_logger().info(f'[RecordExport] list={ml_id}, frames={frame_start}~{frame_start+len(frames)-1}')
        except Exception as e:
            self.get_logger().error(f'[RecordExport] {e}')
            resp      = String()
            resp.data = json.dumps({'error': str(e)})
            self.record_result_pub.publish(resp)

def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok(): time.sleep(1)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
