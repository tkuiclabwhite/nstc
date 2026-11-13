#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
'''
编译: colcon build --symlink-install
启动命令:ros2 run linker_hand_ros2_sdk linker_hand_sdk
'''
import json
import sys  # ROS2 Python接口库
import threading
import time

import numpy as np
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node  # ROS2 节点类
from sensor_msgs.msg import JointState, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Header, String

from linker_hand_ros2_sdk.LinkerHand.linker_hand_api import LinkerHandApi
from linker_hand_ros2_sdk.LinkerHand.utils.color_msg import ColorMsg


class LinkerHand(Node):
    def __init__(self, name):
        super().__init__(name)
        # 声明参数（带默认值）
        self.declare_parameter('hand_type', 'left')
        self.declare_parameter('hand_joint', 'L6')
        self.declare_parameter('is_touch', False)
        self.declare_parameter('can', 'can0')
        self.declare_parameter('modbus', "None")        

        # ros时间获取
        self.stamp_clock = Clock()
        # 获取参数值
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value
        self.is_touch = self.get_parameter('is_touch').value
        self.can = self.get_parameter('can').value
        self.modbus = self.get_parameter('modbus').value
        self.sdk_v = 2
        self.sleep_time = 0.005
        self.cmd_lock = False
        self.last_hand_post_cmd = None # 最新手指位置命令
        self.last_hand_vel_cmd = None # 最新手指速度命令
        self.last_hand_eff_cmd = None # 最新手指力矩命令

        self.last_hand_state = [-1] * 10
        self.last_hand_vel = [-1] * 10
        self.force = [[-1] * 5] * 4
        self.matrix_dic = {
            "stamp":{
                "sec": 0,
                "nanosec": 0,
            },
            "thumb_matrix":[[-1] * 6 for _ in range(12)],
            "index_matrix":[[-1] * 6 for _ in range(12)],
            "middle_matrix":[[-1] * 6 for _ in range(12)],
            "ring_matrix":[[-1] * 6 for _ in range(12)],
            "little_matrix":[[-1] * 6 for _ in range(12)],
            "palm_matrix":[-1],
        }
        # 压感矩阵合值，单位g 克
        self.matrix_mass_dic = {
            "stamp":{
                "secs": 0,
                "nsecs": 0,
            },
            "thumb_mass":[-1],
            "index_mass":[-1],
            "middle_mass":[-1],
            "ring_mass":[-1],
            "little_mass":[-1]
        }
        self.last_hand_info = {
            "version": [-1], # Dexterous hand version number
            "hand_joint": self.hand_joint, # Dexterous hand joint type
            "speed": [-1] * 10, # Current speed threshold of the dexterous hand
            "current": [-1] * 10, # Current of the dexterous hand
            "fault": [-1] * 10, # Current fault of the dexterous hand
            "motor_temperature": [-1] * 10, # Current motor temperature of the dexterous hand
            "torque": [-1] * 10, # Current torque of the dexterous hand
            "is_touch":self.is_touch,
            "touch_type": -1,
            "finger_order": None # Finger motor order
        }
        self.version = []
        self.touch_type = -1
        self.hz = 1.0/60.0

        self.hand_setting_sub = self.create_subscription(String,'/cb_hand_setting_cmd', self.hand_setting_cb, 10)
        self._init_hand()
        time.sleep(1)
        self.run_count = 0 # 计数器，用于记录运行次数
        self.timer = self.create_timer(0.01, self.run)  # 100 Hz
        self.thread_pub_state = threading.Thread(target=self.pub_state)
        self.thread_pub_state.daemon = True
        self.thread_pub_state.start()

    def _init_hand(self):
        self.api = LinkerHandApi(hand_type=self.hand_type, hand_joint=self.hand_joint,modbus=self.modbus,can=self.can)
        time.sleep(0.1)
        self.touch_type = self.api.get_touch_type()
        self.hand_cmd_sub = self.create_subscription(JointState, f'/cb_{self.hand_type}_hand_control_cmd', self.hand_control_cb,10)
        self.hand_state_pub = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_state',10)
        self.hand_info_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_info', 10)
        if self.is_touch:
            if self.modbus != "None":
                self.matrix_touch_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_matrix_touch', 10)
                #self.matrix_touch_pub_pc = self.create_publisher(PointCloud2, f'/cb_{self.hand_type}_hand_matrix_touch_pc', 10)
                self.matrix_touch_mass_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_matrix_touch_mass', 10)
            elif self.touch_type > 1:
                ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Equipped with matrix pressure sensing", color='green')
                self.matrix_touch_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_matrix_touch', 10)
                #self.matrix_touch_pub_pc = self.create_publisher(PointCloud2, f'/cb_{self.hand_type}_hand_matrix_touch_pc', 10)
                self.matrix_touch_mass_pub = self.create_publisher(String, f'/cb_{self.hand_type}_hand_matrix_touch_mass', 10)
            elif self.touch_type != -1 and self.modbus == "None":
                ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Equipped with pressure sensor", color="green")
                self.touch_pub = self.create_publisher(Float32MultiArray, f'/cb_{self.hand_type}_hand_force', 10)
            else:
                ColorMsg(msg=f"{self.hand_type} {self.hand_joint} Not equipped with any pressure sensors", color="red")
                self.is_touch = False
            
        self.embedded_version = self.api.get_embedded_version()
        pose = None
        torque = [200, 200, 200, 200, 200]
        speed = [200, 250, 250, 250, 250]
        if self.hand_joint.upper() == "O6" or self.hand_joint.upper() == "L6" or self.hand_joint.upper() == "L6P":
            pose = [200, 255, 255, 255, 255, 180]
            torque = [250, 250, 250, 250, 250, 250]
            # O6 最大速度阈值
            speed = [200, 250, 250, 250, 250, 250]
        elif self.hand_joint == "L7":
            # The data length of L7 is 7, reinitialize here
            pose = [255, 200, 255, 255, 255, 255, 180]
            torque = [250, 250, 250, 250, 250, 250, 250]
            speed = [120, 250, 250, 250, 250, 250, 250]
        elif self.hand_joint == "L10":
            torque = [255] * 10
            pose = [255, 200, 255, 255, 255, 255, 180, 180, 180, 41]
            speed = [200, 250, 250, 250, 250, 250, 250, 250, 250, 250]
        elif self.hand_joint == "L20":
            pose = [255,255,255,255,255,255,10,100,180,240,245,255,255,255,255,255,255,255,255,255]
        elif self.hand_joint == "L21":
            pose = [75, 255, 255, 255, 255, 176, 97, 81, 114, 147, 202, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
        elif self.hand_joint == "L25":
            pose = [75, 255, 255, 255, 255, 176, 97, 81, 114, 147, 202, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255]
        if pose is not None:
            self.api.set_speed(speed=speed)
            time.sleep(0.1)
            self.api.set_torque(torque=torque)
            time.sleep(0.1)
            self.api.finger_move(pose=pose)
            time.sleep(0.1)

    def list_check(self,pose):
        if not isinstance(pose, list):
            return False
        if len(self.last_hand_post_cmd) != len(pose):
            return False
        return any(abs(self.last_hand_post_cmd - pose) >= 3 for self.last_hand_post_cmd, pose in zip(self.last_hand_post_cmd, pose))

    def hand_control_cb(self, msg):
        if self.last_hand_post_cmd is None or self.list_check(msg.position):
            self.last_hand_post_cmd = msg.position
        if self.last_hand_vel_cmd is None or self.list_check(msg.velocity):
            self.last_hand_vel_cmd = msg.velocity
        if self.last_hand_eff_cmd is None or self.list_check(msg.effort):
            self.last_hand_eff_cmd = msg.effort

    def run(self):
        if self.sdk_v == 1:
            self.sleep_time = 0.009
        if self.hand_state_pub.get_subscription_count() > 0:
            # 优先获取手指状态并且发布
            self.last_hand_state = self.api.get_state()
            time.sleep(0.003)
            self.last_hand_vel = self.api.get_joint_speed()
            time.sleep(0.002)
        if not self.cmd_lock:
            if self.last_hand_post_cmd is not None:
                self.api.finger_move(pose=self.last_hand_post_cmd)
                self.last_hand_post_cmd = None
            if self.last_hand_vel_cmd is not None:
                vel = list(self.last_hand_vel_cmd)
                if all(x == 0 for x in vel):
                    pass
                else:
                    if (str(self.hand_joint).upper() == "O6" or str(self.hand_joint).upper() == "L6" or str(self.hand_joint).upper() == "L6P") and len(vel) == 6:
                        speed = vel
                        self.api.set_joint_speed(speed=speed)
                    elif self.hand_joint == "L7" and len(vel) == 7:
                        speed = vel
                        self.api.set_joint_speed(speed=speed)
                    elif self.hand_joint == "L10" and len(vel) == 10:
                        speed = [vel[0],vel[2],vel[3],vel[4],vel[5]]
                        self.api.set_joint_speed(speed=speed)
                    elif self.hand_joint == "L20" and len(vel) == 20:
                        speed = [vel[10],vel[1],vel[2],vel[3],vel[4]]
                        self.api.set_joint_speed(speed=speed)
                    elif self.hand_joint == "L21" and len(vel) == 25:
                        speed = vel
                        self.api.set_joint_speed(speed=speed)
                    elif self.hand_joint == "L25" and len(vel) == 25:
                        speed = vel
                        self.api.set_joint_speed(speed=speed)
                self.last_hand_vel_cmd = None
            time.sleep(0.003)
            if self.run_count == 3 and self.is_touch and self.touch_type == 1 and self.modbus == "None" and self.touch_pub.get_subscription_count() > 0:
                """单点式压力传感器"""
                self.force = self.api.get_force()
            if self.is_touch and (self.touch_type > 1 or self.modbus != "None") and (self.matrix_touch_pub.get_subscription_count() > 0 or self.matrix_touch_mass_pub.get_subscription_count() > 0):
                """矩阵式压力传感器"""
                if self.run_count == 3:
                    self.matrix_dic["thumb_matrix"] = self.api.get_thumb_matrix_touch(sleep_time=self.sleep_time).tolist()
                if self.run_count == 4:
                    self.matrix_dic["index_matrix"] = self.api.get_index_matrix_touch(sleep_time=self.sleep_time).tolist()
                if self.run_count == 5:
                    self.matrix_dic["middle_matrix"] = self.api.get_middle_matrix_touch(sleep_time=self.sleep_time).tolist()
                if self.run_count == 6:
                    self.matrix_dic["ring_matrix"] = self.api.get_ring_matrix_touch(sleep_time=self.sleep_time).tolist()
                if self.run_count == 7:
                    self.matrix_dic["little_matrix"] = self.api.get_little_matrix_touch(sleep_time=self.sleep_time).tolist()
                if self.run_count == 8:
                    if self.hand_joint == "O6" or self.hand_joint == "L6":
                        self.matrix_dic["palm_matrix"] = self.api.get_palm_matrix_touch(sleep_time=0.006)
                time.sleep(0.005)
            if self.run_count == 8 and self.hand_info_pub.get_subscription_count() > 0:
                """手部信息"""
                self.last_hand_info = {
                    "version": self.embedded_version, # Dexterous hand version number
                    "hand_joint": self.hand_joint, # Dexterous hand joint type
                    "speed": self.api.get_speed(), # Current speed threshold of the dexterous hand
                    "current": self.api.get_current(), # Current of the dexterous hand
                    "fault": self.api.get_fault(), # Current fault of the dexterous hand
                    "motor_temperature": self.api.get_temperature(), # Current motor temperature of the dexterous hand
                    "torque": self.api.get_torque(), # Current torque of the dexterous hand
                    "is_touch":self.is_touch,
                    "touch_type": self.touch_type,
                    "finger_order": self.api.get_finger_order() # Finger motor order
                }
                
            if self.run_count == 9:
                self.api.clear_faults() # 自动清除错误编码
                self.run_count = 0
            self.run_count += 1
            time.sleep(0.003)


    def pub_state(self):
        while True:
            if self.hand_state_pub.get_subscription_count() > 0:
                msg = self.joint_state_msg(self.last_hand_state, self.last_hand_vel)
                self.hand_state_pub.publish(msg)
            if self.is_touch and self.touch_type == 1 and self.modbus == "None" and self.touch_pub.get_subscription_count() > 0:
                msg = Float32MultiArray()
                msg.data = [float(val) for sublist in self.force for val in sublist]
                self.touch_pub.publish(msg)
            if self.is_touch and (self.touch_type > 1 or self.modbus != "None") and (self.matrix_touch_pub.get_subscription_count() > 0 or self.matrix_touch_mass_pub.get_subscription_count() > 0):
                # 发布矩阵压感数据JSON格式
                self.pub_matrix_dic()
                # 发布矩阵压感和值JSON格式
                self.pub_matrix_mass(dic=self.matrix_dic)
            if self.hand_info_pub.get_subscription_count() > 0:
                msg = String()
                msg.data = json.dumps(self.last_hand_info)
                self.hand_info_pub.publish(msg)
            time.sleep(self.hz)

    def pub_matrix_mass(self, dic):
        """发布矩阵数据合值 单位g 克 JSON格式"""
        msg = String()
        # 获取当前的 ROS 时间
        current_time = self.stamp_clock.now()
        # 提取 secs 和 nsecs
        t_secs = current_time.to_msg().sec
        t_nsecs = current_time.to_msg().nanosec
        self.matrix_mass_dic["stamp"]["secs"] = t_secs
        self.matrix_mass_dic["stamp"]["nsecs"] = t_nsecs
        self.matrix_mass_dic["thumb_mass"] = sum(sum(row) for row in dic["thumb_matrix"])
        self.matrix_mass_dic["index_mass"] = sum(sum(row) for row in dic["index_matrix"])
        self.matrix_mass_dic["middle_mass"] = sum(sum(row) for row in dic["middle_matrix"])
        self.matrix_mass_dic["ring_mass"] = sum(sum(row) for row in dic["ring_matrix"])
        self.matrix_mass_dic["little_mass"] = sum(sum(row) for row in dic["little_matrix"])
        if dic["palm_matrix"] == [-1]:
            self.matrix_mass_dic["palm_mass"] = [-1]
        else:
            self.matrix_mass_dic["palm_mass"] = sum(sum(row) for row in dic["palm_matrix"])
        self.matrix_mass_dic["unit"] = "g"
        msg.data = json.dumps(self.matrix_mass_dic)
        self.matrix_touch_mass_pub.publish(msg)

    def pub_matrix_point_cloud(self):
        """发布矩阵数据点云格式"""
        tmp_dic = self.matrix_dic.copy()
        del tmp_dic['stamp']               # 去掉时间戳字段
        all_matrices = list(tmp_dic.values())  # 5 帧，每帧 6×12=72 个数
        # 摊平到一维：360 个 float
        flat_list = [v for frame in all_matrices for v in frame]  # 360
        flat = np.concatenate([np.asarray(np.clip(c, 0, 255), dtype=np.uint8) for c in flat_list])
        fields = [PointField(
            name='val',
            offset=0,
            datatype=PointField.UINT8,
            count=1
        )]
        pc = PointCloud2()
        pc.header.stamp = self.stamp_clock.now().to_msg()
        pc.header.frame_id = ''
        pc.height = 1
        pc.width = flat.size         # 360
        pc.fields = fields
        pc.is_bigendian = False
        pc.point_step = 1            # 1 个 float32
        pc.row_step = pc.point_step * pc.width
        pc.data = flat.tobytes()     # 1440 字节
        self.matrix_touch_pub_pc.publish(pc)

    def pub_matrix_dic(self):
        """发布矩阵数据JSON格式"""
        msg = String()
        # 获取当前的 ROS 时间
        current_time = self.stamp_clock.now()
        # 提取 secs 和 nsecs
        t_secs = current_time.to_msg().sec
        t_nsecs = current_time.to_msg().nanosec
        self.matrix_dic["stamp"]["secs"] = t_secs
        self.matrix_dic["stamp"]["nsecs"] = t_nsecs
        msg.data = json.dumps(self.matrix_dic)
        self.matrix_touch_pub.publish(msg)

    def joint_state_msg(self, pose,vel=None):
        if vel is None:
            vel = []

        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.api.get_finger_order()
        joint_state.position = [float(x) for x in pose]
        if len(vel) > 1:
            joint_state.velocity = [float(x) for x in vel]
        else:
            joint_state.velocity = [0.0] * len(pose)
        joint_state.effort = [0.0] * len(pose)
        return joint_state
    
    

    

    def hand_setting_cb(self,msg):
        '''控制命令回调'''
        try:
            data = json.loads(msg.data)
            print(f"Received setting command: {data['setting_cmd']}",flush=True)
            hand_left = False
            hand_right = False
            if data["params"]["hand_type"] == "left":
                hand = self.api
                hand_left = True
            elif data["params"]["hand_type"] == "right":
                hand = self.api
                hand_right = True
            else:
                print("Please specify the hand part to be set",flush=True)
                return
            self.cmd_lock = True
            # Set maximum torque
            if data["setting_cmd"] == "set_max_torque_limits": # Set maximum torque
                torque = list(data["params"]["torque"])
                hand.set_torque(torque=torque)
                
            if data["setting_cmd"] == "set_speed": # Set speed
                if isinstance(data["params"]["speed"], list):
                    speed = data["params"]["speed"]
                    hand.set_speed(speed=speed)
                else:
                    ColorMsg(msg="Speed parameter error, speed must be a list", color="red")
            if data["setting_cmd"] == "clear_faults": # Clear faults
                if hand_left and self.hand_joint == "L10" :
                    ColorMsg(msg="L10 left hand cannot clear faults")
                elif hand_right and self.hand_joint == "L10" :
                    ColorMsg(msg="L10 right hand cannot clear faults")
                else:
                    hand.clear_faults()
            if data["setting_cmd"] == "get_faults": # Get faults
                f = hand.get_fault()
                ColorMsg(msg=f"Get faults: {f}")
            if data["setting_cmd"] == "electric_current": # Get current
                ColorMsg(msg=f"Get current: {hand.get_current()}")
            if data["setting_cmd"] == "set_electric_current": # Set current
                if isinstance(data["params"]["current"], list):
                    hand.set_current(data["params"]["current"])
            if data["setting_cmd"] == "show_fun_table": # Get faults
                f = hand.show_fun_table()
        except Exception:
            print("命令参数错误")
            self.cmd_lock = False
        finally:
            self.cmd_lock = False


    def close_can(self):
        self.api.open_can.close_can(can=self.can)
        sys.exit(0)

        
def main(args=None):
    try:
        rclpy.init(args=args)
        node = LinkerHand("linker_hand_sdk")
        embedded_version = node.embedded_version
        # ── 本地修改（見 UPSTREAM.md 地雷 8）──────────────────────────────
        # get_version() 在手沒有回應查詢時會回 None，原本這裡直接 len(None)，
        # 結果是一個指向 SDK 內部的 TypeError traceback，完全看不出真正的問題。
        # 手不回話一定是硬體那側（電源 / CAN 接線 / USB-CAN 需要重插），
        # 所以改成講清楚再乾淨退出。
        if embedded_version is None or len(embedded_version) == 0:
            ColorMsg(msg="=" * 72, color="red")
            ColorMsg(msg=f"[{node.hand_type} {node.hand_joint}] 讀不到硬體版本 —— "
                         f"手完全沒有回應 CAN 查詢（{node.can}）", color="red")
            ColorMsg(msg="  CAN 介面本身是通的（封包有被 ACK），所以不是介面沒開。"
                         "請依序確認：", color="red")
            ColorMsg(msg="   1. 手的電源有沒有接上、有沒有開（USB-CAN 只提供匯流排，"
                         "不供電給手）", color="red")
            ColorMsg(msg="   2. 把 USB-CAN 盒重新插一次，再重新 ip link set "
                         f"{node.can} up type can bitrate 1000000", color="red")
            ColorMsg(msg="   3. CAN H / L 有沒有接反、終端電阻在不在", color="red")
            ColorMsg(msg="   4. 手的 CAN ID 是不是還是 0x28(左) / 0x27(右)", color="red")
            ColorMsg(msg="  自己驗一次：candump " + str(node.can)
                         + " 另開一個終端，然後 cansend " + str(node.can)
                         + " 028#64 —— 正常會看到手回一筆 028 的版本資料", color="red")
            ColorMsg(msg="=" * 72, color="red")
            node.destroy_node()
            rclpy.shutdown()
            return
        if len(embedded_version) == 3 or node.hand_joint.upper() == "O6" or node.hand_joint.upper() == "L6" or node.hand_joint.upper() == "G20"  or node.hand_joint.upper() == "L20-10" or node.hand_joint.upper() == "L20-11":
            ColorMsg(msg="New Matrix Touch For SDK V2", color="green")
            node.sdk_v = 2
        elif len(embedded_version) == 6 and node.hand_joint == "L10":
            ColorMsg(msg="New Matrix Touch For SDK V2", color="green")
            node.sdk_v = 2
        elif len(embedded_version) > 4 and ((embedded_version[0]==10 and embedded_version[4]>35) or (embedded_version[0]==7 and embedded_version[4]>50) or (embedded_version[0] == 6)):
            ColorMsg(msg="New Matrix Touch For SDK V2", color="green")
            node.sdk_v = 2
        else:
            ColorMsg(msg="SDK V1", color="green")
            node.sdk_v = 1
        rclpy.spin(node)         # 主循环，监听 ROS 回调
    except KeyboardInterrupt:
        print("收到 Ctrl+C，准备退出...")
    finally:
        # node.close_can()         # 关闭 CAN 或其他硬件资源
        # node.destroy_node()      # 销毁 ROS 节点
        # rclpy.shutdown()         # 关闭 ROS
        print("程序已退出。")
