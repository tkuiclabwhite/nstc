#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
L20 灵巧手 Modbus RTU (RS485) 控制类。

协议依据:
    《L20_V10_V11_Modbus_RTU通信接口说明书》V1.0 (2026-07-16)
    《L20_V10_V11_Modbus寄存器说明快速浏览.xlsx》

要点摘录:
  * 物理层 RS485 半双工，默认 115200 / 8 / N / 1，默认从站地址 1。
    L20 使用拨码开关切换 CAN / RS485 到 XT30(2+2)PB-M.G.B，用 485 前需确认拨码位置。
  * 支持功能码 0x03(读保持) / 0x04(读输入) / 0x06(写单个保持) / 0x10(写多个保持)。
  * 寄存器地址从 0 开始；16 位寄存器高字节在前，CRC16 低字节在前。
  * 地址空间按 6 个关节位 x 5 个手指 = 30 个寄存器编排，关节位顺序为
    ROLL、YAW、ROOT1、ROOT2、ROOT3、TIP，每个关节位内按
    大拇指、食指、中指、无名指、小拇指排列。
    L20 实际有效关节: ROLL 仅大拇指、YAW 五指、ROOT1 五指、TIP 五指(共 16 个驱动)；
    ROOT2 / ROOT3 全部以及四指 ROLL 均为预留占位，写入无效、读出不反映真实状态。
"""
import time
from typing import Dict, List, Optional, Sequence, Union

import numpy as np
from pymodbus.client import ModbusSerialClient

try:  # pymodbus 3.x 的异常基类，用于把通信异常统一成 RuntimeError
    from pymodbus.exceptions import ModbusException
except ImportError:  # pragma: no cover - 老版本 pymodbus 兜底
    ModbusException = Exception


# --------------------------------------------------------------------------
# 通信默认参数 (说明书 2.3 通信参数)
# --------------------------------------------------------------------------
DEFAULT_BAUDRATE = 115200
DEFAULT_SLAVE_ID = 0x01          # 说明书默认 Slave ID = 1

# 连续请求之间的间隔 (说明书 2.5: 间隔视返回帧所携带数据量大小而定)。
# 普通帧响应最长 65 字节 (读 30 个寄存器)，3 ms 足够；
# 矩阵数据帧响应可达 149 字节 (读 72 个寄存器)，单独放宽到 5 ms。
_INTERVAL = 0.003                # 普通帧间隔
_MATRIX_INTERVAL = 0.005         # 矩阵传感器相关帧的间隔

# --------------------------------------------------------------------------
# 关节编排 (说明书 5.2 关节排列与有效关节)
# --------------------------------------------------------------------------
FINGERS = ["thumb", "index", "middle", "ring", "little"]
JOINT_GROUPS = ["roll", "yaw", "root1", "root2", "root3", "tip"]
_FINGER_NUM = len(FINGERS)          # 5
_GROUP_NUM = len(JOINT_GROUPS)      # 6
_REG_BLOCK = _FINGER_NUM * _GROUP_NUM   # 30 个寄存器为一个功能块

# 30 个寄存器的名称，顺序与地址一一对应 (地址 = 索引)
REG_KEYS = [f"{finger}_{group}" for group in JOINT_GROUPS for finger in FINGERS]

# L20 实际具备独立驱动的关节位；其余为预留占位
VALID_JOINTS = {
    "roll":  ["thumb"],
    "yaw":   FINGERS,
    "root1": FINGERS,
    "root2": [],
    "root3": [],
    "tip":   FINGERS,
}

# --------------------------------------------------------------------------
# SDK 20 元素向量 <-> 30 个 Modbus 寄存器 的映射
#
# LinkerHandApi 对 L20 统一使用 20 元素向量，其分段含义见
# core/can/linker_hand_l20_can.py 的 pose_slice() 与 utils/mapping.py 的 l20_* 表:
#     [0:5]   ROOT1 指根弯曲
#     [5:10]  YAW   侧摆
#     [10:15] ROLL  大拇指旋转 (11~14 为预留占位)
#     [15:20] TIP   指尖弯曲
# Modbus 侧关节位顺序是 ROLL/YAW/ROOT1/ROOT2/ROOT3/TIP，所以两者需要重排。
# --------------------------------------------------------------------------
_ROLL_BASE, _YAW_BASE, _ROOT1_BASE, _ROOT2_BASE, _ROOT3_BASE, _TIP_BASE = (
    i * _FINGER_NUM for i in range(_GROUP_NUM)
)

# SDK_TO_REG[i] = SDK 向量第 i 个元素对应的寄存器偏移
SDK_TO_REG: List[int] = (
    [_ROOT1_BASE + i for i in range(_FINGER_NUM)]    # 0~4   -> 10~14
    + [_YAW_BASE + i for i in range(_FINGER_NUM)]    # 5~9   -> 5~9
    + [_ROLL_BASE + i for i in range(_FINGER_NUM)]   # 10~14 -> 0~4
    + [_TIP_BASE + i for i in range(_FINGER_NUM)]    # 15~19 -> 25~29
)
_SDK_LEN = len(SDK_TO_REG)  # 20

# SDK 20 元素向量的名称
KEYS = [REG_KEYS[r] for r in SDK_TO_REG]

# 预留占位的寄存器偏移 (ROOT2 / ROOT3 全部 + 四指 ROLL)，下发时需补任意 0~0xFF 值
_RESERVED_REGS = sorted(set(range(_REG_BLOCK)) - set(SDK_TO_REG))
_RESERVED_FILL = 0

# --------------------------------------------------------------------------
# 保持寄存器地址 (功能码 0x03 读 / 0x06、0x10 写)  —— 说明书附录 C
# --------------------------------------------------------------------------
HR = {
    "position":       0,    # 0~29    位置命令
    "speed":          30,   # 30~59   速度命令
    "torque":         60,   # 60~89   目标扭矩命令
    "clear_fault":    90,   # 90~119  清除故障命令 (1:清除 0:不清除)
    "temp_limit":     120,  # 120~149 温度阈值
    "current_limit":  150,  # 150~179 电流限制
    "baudrate":       180,  # 180~181 波特率 uint32，低 16 位在前
    "device_id":      182,  # 设备 ID 设置
    "erase_pos_cali": 183,  # 清除位置标定数据 (1:擦除)
    "save":           184,  # 保存配置到 flash (1:保存)
    "factory_reset":  185,  # 恢复出厂设置
    "password":       186,  # 186~188 密码
    "uid":            189,  # 189~191 UID 编号
    "reserve":        192,  # 192~197 保留位
    "matrix_index":   198,  # 矩阵传感器索引设置
    "matrix_size":    199,  # 矩阵传感器输出尺寸设置 (高 4 位行、低 4 位列)
}

# --------------------------------------------------------------------------
# 输入寄存器地址 (功能码 0x04 只读)  —— 说明书附录 D
# --------------------------------------------------------------------------
IR = {
    "position":          0,    # 0~29    位置反馈
    "speed":             30,   # 30~59   速度反馈
    "torque":            60,   # 60~89   扭矩反馈
    "fault":             90,   # 90~119  故障码，按附录 E 的 bit 位解析
    "temperature":       120,  # 120~149 温度反馈
    "current":           150,  # 150~179 电流反馈
    "normal_force":      180,  # 180~184 五指法向力
    "tangential_force":  185,  # 185~189 五指切向力
    "tangential_dir":    190,  # 190~194 五指切向力方向
    "approach_inc":      195,  # 195~199 五指接近增量
    "hardware_version":  200,  # 200~201 硬件版本 低/高字节
    "software_version":  202,  # 202~203 软件版本 低/高字节
    "structure_version": 204,  # 204~205 结构版本 低/高字节
    "reserve":           206,  # 206~211 保留位
    "uid":               212,  # 212~222 UID 字符串，按半字存放 (11 个寄存器)
    "matrix_index":      223,  # 当前矩阵数据区的实际传感器索引
    "matrix_size":       224,  # 当前矩阵数据区的实际尺寸 (高 4 位行、低 4 位列)
    "matrix_data":       225,  # 225~320 矩阵数据，共 96 个寄存器，行优先展开
}

_UID_REG_COUNT = 11          # 输入寄存器 212~222
_MATRIX_DATA_MAX = 96        # 输入寄存器 225~320

# --------------------------------------------------------------------------
# 矩阵传感器索引 (保持寄存器 198 / 输入寄存器 223)
# 说明书 6.4 与 7.5: 0 无效，1~5 为五指，6 为手掌
# --------------------------------------------------------------------------
MATRIX_SENSOR = {
    "none": 0, "thumb": 1, "index": 2, "middle": 3,
    "ring": 4, "little": 5, "palm": 6,
}
# 指尖矩阵传感器原始尺寸 12 行 x 6 列 (说明书 6.4 举例 0xC6，与 CAN 实现一致)
_TIP_MATRIX_ROWS = 12
_TIP_MATRIX_COLS = 6

# --------------------------------------------------------------------------
# 故障码 bit 位定义 (说明书附录 E)
# 注: 文档中 bit3 与 bit4 的英文名同为 current_sta，此处按中文含义取用可区分的键名。
# --------------------------------------------------------------------------
FAULT_BITS = {
    0: ("motor_rotor_lock",      "电机堵转"),
    1: ("motor_over_current",    "电机过流"),
    2: ("motor_stall_fault",     "电机异常(失速)"),
    3: ("motor_voltage_abnormal", "电机电压异常"),
    4: ("motor_self_check_error", "电机自检异常"),
    5: ("temperature_sta",       "过温度判断"),
    6: ("soft_rotor_lock",       "软件判断过流"),
    7: ("motor_comm_abnormal",   "电机通信异常"),
}

# Modbus 异常码 (说明书 3.8)
MODBUS_EXCEPTION_CODES = {
    0x01: "非法功能码",
    0x02: "非法数据地址",
    0x03: "非法数据值",
    0x04: "从站设备故障",
    0x06: "从站设备忙",
}


class LinkerHandL20RS485:
    """
    L20 灵巧手 Modbus RTU (RS485) 控制类。

    对外提供两套视图:
      * SDK 视图: 20 元素向量，顺序为 ROOT1 x5、YAW x5、ROLL x5、TIP x5，
        与 LinkerHandApi / CAN 版 LinkerHandL20Can 保持一致。
        对应方法 set_joint_positions() / get_state() / get_speed() 等。
      * 寄存器视图: 30 元素向量，顺序即 Modbus 地址顺序
        (ROLL/YAW/ROOT1/ROOT2/ROOT3/TIP，每组 5 指)。
        对应方法 write_position_block() / read_position_block() 等。
    """

    KEYS = KEYS                  # 20 元素 SDK 向量的名称
    REG_KEYS = REG_KEYS          # 30 元素寄存器向量的名称
    FINGERS = FINGERS
    JOINT_GROUPS = JOINT_GROUPS
    VALID_JOINTS = VALID_JOINTS

    # ==================================================================
    # 构造 / 连接
    # ==================================================================
    def __init__(self,
                 hand_id: int = DEFAULT_SLAVE_ID,
                 modbus_port: str = "/dev/ttyUSB0",
                 baudrate: int = DEFAULT_BAUDRATE,
                 timeout: float = 0.05,
                 retries: int = 3,
                 interval: float = _INTERVAL,
                 matrix_interval: float = _MATRIX_INTERVAL):
        """
        :param hand_id:     Modbus 从站地址，L20 出厂默认 1，有效范围 1~247
        :param modbus_port: 串口设备名
        :param baudrate:    波特率，默认 115200
        :param timeout:     单帧响应超时(秒)
        :param retries:     失败重试次数
        :param interval:    普通帧之间的最小间隔(秒)，默认 3ms
        :param matrix_interval: 矩阵传感器帧之间的最小间隔(秒)，默认 5ms
        """
        #self.slave = hand_id
        self.slave = DEFAULT_SLAVE_ID
        self.interval = interval
        self.matrix_interval = matrix_interval
        self.touch_type = 2      # L20 指尖为矩阵式压力传感器

        client_kwargs = dict(
            port=modbus_port,
            baudrate=baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=timeout,
            retries=retries,
        )
        try:
            # pymodbus 3.5.x 支持这两个参数，更高版本已移除
            self.cli = ModbusSerialClient(
                retry_on_empty=True, handle_local_echo=False, **client_kwargs
            )
        except TypeError:
            self.cli = ModbusSerialClient(**client_kwargs)

        self.connected = self.cli.connect()
        if not self.connected:
            raise ConnectionError(
                f"RS485 connect fail to {modbus_port} with slave id {hex(hand_id)}"
            )

        # 矩阵传感器当前选择的缓存，避免每帧都重复下发尺寸配置
        self._matrix_index: Optional[int] = None
        self._matrix_size: Optional[int] = None

    # ==================================================================
    # 底层读写封装
    # ==================================================================
    @staticmethod
    def _describe(rsp) -> str:
        """把 pymodbus 响应里的异常码翻译成中文，便于定位问题。"""
        code = getattr(rsp, "exception_code", None)
        if code in MODBUS_EXCEPTION_CODES:
            return f"{rsp} (异常码 0x{code:02X}: {MODBUS_EXCEPTION_CODES[code]})"
        return str(rsp)

    def _read_input(self, address: int, count: int,
                    interval: Optional[float] = None) -> List[int]:
        """功能码 0x04 读输入寄存器。interval 为 None 时使用普通帧间隔。"""
        time.sleep(self.interval if interval is None else interval)
        try:
            rsp = self.cli.read_input_registers(
                address=address, count=count, slave=self.slave)
        except ModbusException as e:
            raise RuntimeError(f"FC04 通信异常, 地址 {address}, 数量 {count}: {e}") from e
        if rsp.isError():
            raise RuntimeError(
                f"FC04 读取失败, 地址 {address}, 数量 {count}: {self._describe(rsp)}")
        return list(rsp.registers)

    def _read_holding(self, address: int, count: int,
                      interval: Optional[float] = None) -> List[int]:
        """功能码 0x03 读保持寄存器。interval 为 None 时使用普通帧间隔。"""
        time.sleep(self.interval if interval is None else interval)
        try:
            rsp = self.cli.read_holding_registers(
                address=address, count=count, slave=self.slave)
        except ModbusException as e:
            raise RuntimeError(f"FC03 通信异常, 地址 {address}, 数量 {count}: {e}") from e
        if rsp.isError():
            raise RuntimeError(
                f"FC03 读取失败, 地址 {address}, 数量 {count}: {self._describe(rsp)}")
        return list(rsp.registers)

    def _write_holding(self, address: int, values: Sequence[int],
                       interval: Optional[float] = None):
        """功能码 0x10 写多个保持寄存器 (单个值时退化为 0x06)。"""
        values = [int(v) & 0xFFFF for v in values]
        if not values:
            return
        time.sleep(self.interval if interval is None else interval)
        try:
            if len(values) == 1:
                rsp = self.cli.write_register(
                    address=address, value=values[0], slave=self.slave)
            else:
                rsp = self.cli.write_registers(
                    address=address, values=values, slave=self.slave)
        except ModbusException as e:
            raise RuntimeError(f"写入通信异常, 地址 {address}: {e}") from e
        if rsp.isError():
            raise RuntimeError(
                f"写入失败, 地址 {address}, 数量 {len(values)}: {self._describe(rsp)}")

    # ==================================================================
    # SDK 20 元素向量 <-> 30 元素寄存器向量
    # ==================================================================
    @staticmethod
    def sdk_to_registers(vals: Sequence[Union[int, float]],
                         fill: int = _RESERVED_FILL) -> List[int]:
        """
        把命令向量展开成 30 个寄存器值。

        支持三种长度:
          30 - 已经是寄存器顺序，原样使用；
          20 - SDK 顺序，按 SDK_TO_REG 散布，预留位补 fill；
           5 - 每指一个值，广播到该手指的全部关节位 (与 CAN 版 5 元素语义一致)。
        标量将广播到全部 30 个寄存器。
        """
        if isinstance(vals, (int, float)):
            return [int(vals) & 0xFF] * _REG_BLOCK

        vals = list(vals)
        if len(vals) == _REG_BLOCK:
            return [int(v) & 0xFF for v in vals]

        if len(vals) == _SDK_LEN:
            regs = [fill] * _REG_BLOCK
            for i, reg in enumerate(SDK_TO_REG):
                regs[reg] = int(vals[i]) & 0xFF
            return regs

        if len(vals) == _FINGER_NUM:
            regs = [fill] * _REG_BLOCK
            for group in range(_GROUP_NUM):
                for finger in range(_FINGER_NUM):
                    regs[group * _FINGER_NUM + finger] = int(vals[finger]) & 0xFF
            return regs

        raise ValueError(
            f"命令长度必须为 {_FINGER_NUM}/{_SDK_LEN}/{_REG_BLOCK}，当前为 {len(vals)}")

    @staticmethod
    def registers_to_sdk(regs: Sequence[int]) -> List[int]:
        """把 30 个寄存器值抽取成 SDK 的 20 元素向量。"""
        if len(regs) != _REG_BLOCK:
            raise ValueError(f"需要 {_REG_BLOCK} 个寄存器值，当前为 {len(regs)}")
        return [int(regs[r]) for r in SDK_TO_REG]

    @classmethod
    def to_dict(cls, regs: Sequence[int]) -> Dict[str, int]:
        """把 30 个寄存器值或 20 元素向量转成带名称的字典，便于打印调试。"""
        if len(regs) == _REG_BLOCK:
            return dict(zip(cls.REG_KEYS, [int(v) for v in regs]))
        if len(regs) == _SDK_LEN:
            return dict(zip(cls.KEYS, [int(v) for v in regs]))
        raise ValueError(f"长度必须为 {_SDK_LEN} 或 {_REG_BLOCK}，当前为 {len(regs)}")

    # ==================================================================
    # 保持寄存器: 批量写入 (30 寄存器块)
    # ==================================================================
    def write_position_block(self, vals: Sequence[int]):
        """写位置命令 (保持寄存器 0~29)。"""
        self._write_holding(HR["position"], self.sdk_to_registers(vals))

    def write_speed_block(self, vals: Sequence[int]):
        """写速度命令 (保持寄存器 30~59)，数值越大速度越大。"""
        self._write_holding(HR["speed"], self.sdk_to_registers(vals))

    def write_torque_block(self, vals: Sequence[int]):
        """写目标扭矩命令 (保持寄存器 60~89)。"""
        self._write_holding(HR["torque"], self.sdk_to_registers(vals))

    def write_temp_limit_block(self, vals: Sequence[int]):
        """写温度阈值 (保持寄存器 120~149)。"""
        self._write_holding(HR["temp_limit"], self.sdk_to_registers(vals))

    def write_current_limit_block(self, vals: Sequence[int]):
        """写电流限制 (保持寄存器 150~179)。"""
        self._write_holding(HR["current_limit"], self.sdk_to_registers(vals))

    # ==================================================================
    # 保持寄存器: 回读 (读回下发的命令值，非实时反馈)
    # ==================================================================
    def read_position_command(self) -> List[int]:
        return self._read_holding(HR["position"], _REG_BLOCK)

    def read_speed_command(self) -> List[int]:
        return self._read_holding(HR["speed"], _REG_BLOCK)

    def read_torque_command(self) -> List[int]:
        return self._read_holding(HR["torque"], _REG_BLOCK)

    def read_temp_limit(self) -> List[int]:
        return self._read_holding(HR["temp_limit"], _REG_BLOCK)

    def read_current_limit(self) -> List[int]:
        return self._read_holding(HR["current_limit"], _REG_BLOCK)

    # ==================================================================
    # 输入寄存器: 实时反馈 (30 寄存器块)
    # ==================================================================
    def read_positions(self) -> List[int]:
        """位置反馈 (输入寄存器 0~29)，返回 30 个寄存器原始值。"""
        return self._read_input(IR["position"], _REG_BLOCK)

    def read_speeds(self) -> List[int]:
        """速度反馈 (输入寄存器 30~59)。"""
        return self._read_input(IR["speed"], _REG_BLOCK)

    def read_torques(self) -> List[int]:
        """扭矩反馈 (输入寄存器 60~89)。"""
        return self._read_input(IR["torque"], _REG_BLOCK)

    def read_faults(self) -> List[int]:
        """故障码 (输入寄存器 90~119)，按 FAULT_BITS 的 bit 位解析。"""
        return self._read_input(IR["fault"], _REG_BLOCK)

    def read_temperatures(self) -> List[int]:
        """温度反馈 (输入寄存器 120~149)。"""
        return self._read_input(IR["temperature"], _REG_BLOCK)

    def read_currents(self) -> List[int]:
        """电流反馈 (输入寄存器 150~179)。"""
        return self._read_input(IR["current"], _REG_BLOCK)

    # ==================================================================
    # 输入寄存器: 力 / 接近传感器 (需搭载相应传感器才有有效数据)
    # ==================================================================
    def read_normal_force(self) -> List[int]:
        """五指法向力 (输入寄存器 180~184)。"""
        return self._read_input(IR["normal_force"], _FINGER_NUM)

    def read_tangential_force(self) -> List[int]:
        """五指切向力 (输入寄存器 185~189)。"""
        return self._read_input(IR["tangential_force"], _FINGER_NUM)

    def read_tangential_force_dir(self) -> List[int]:
        """五指切向力方向 (输入寄存器 190~194)。"""
        return self._read_input(IR["tangential_dir"], _FINGER_NUM)

    def read_approach_inc(self) -> List[int]:
        """五指接近增量 (输入寄存器 195~199)。"""
        return self._read_input(IR["approach_inc"], _FINGER_NUM)

    def read_force_block(self) -> List[int]:
        """一次性读取 180~199 全部力/接近传感器数据，返回 20 个值。"""
        return self._read_input(IR["normal_force"], 4 * _FINGER_NUM)

    # ==================================================================
    # 输入寄存器: 版本与 UID
    # ==================================================================
    def read_versions(self) -> List[int]:
        """
        读取版本信息 (输入寄存器 200~205)，返回 6 个寄存器:
        [硬件低, 硬件高, 软件低, 软件高, 结构低, 结构高]
        """
        return self._read_input(IR["hardware_version"], 6)

    def get_version_dict(self) -> Dict[str, str]:
        """把 200~205 组合成 '高.低' 形式的版本字符串。"""
        v = self.read_versions()
        return {
            "hardware_version": f"{v[1]}.{v[0]}",
            "software_version": f"{v[3]}.{v[2]}",
            "structure_version": f"{v[5]}.{v[4]}",
        }

    def read_uid_registers(self) -> List[int]:
        """UID 原始寄存器 (输入寄存器 212~222)。"""
        return self._read_input(IR["uid"], _UID_REG_COUNT)

    # ==================================================================
    # 矩阵传感器 (保持寄存器 198/199 选择，输入寄存器 223~320 读取)
    # ==================================================================
    @staticmethod
    def encode_matrix_size(rows: int, cols: int) -> int:
        """
        按说明书 6.4 编码尺寸: 仅低 8 位有效，高 4 位为行数，低 4 位为列数。
        例如 12 行 6 列 -> 0xC6。行列各占 4 位，因此上限均为 15。
        """
        if not (1 <= rows <= 0xF and 1 <= cols <= 0xF):
            raise ValueError(f"矩阵行列均需在 1~15 之间，当前 rows={rows}, cols={cols}")
        return ((rows & 0xF) << 4) | (cols & 0xF)

    @staticmethod
    def decode_matrix_size(size: int) -> tuple:
        """解析输入寄存器 224 的实际尺寸，返回 (rows, cols)。"""
        size &= 0xFF
        return (size >> 4) & 0xF, size & 0xF

    def select_matrix_sensor(self, sensor: Union[int, str],
                             rows: Optional[int] = None,
                             cols: Optional[int] = None):
        """
        选择要读取的矩阵传感器，并可选地设置池化降采样后的输出尺寸。

        :param sensor: 传感器索引 0~6，或 MATRIX_SENSOR 中的名称
        :param rows:   期望行数；None 表示不改动设备当前的尺寸设置
        :param cols:   期望列数；None 表示不改动设备当前的尺寸设置

        期望行列应分别为该传感器原始最大行列数的整数因数，否则可能无法得到有效输出。
        """
        if isinstance(sensor, str):
            key = sensor.lower()
            if key not in MATRIX_SENSOR:
                raise ValueError(
                    f"未知的矩阵传感器名称 {sensor}，可选: {list(MATRIX_SENSOR)}")
            sensor = MATRIX_SENSOR[key]
        sensor = int(sensor)
        if not 0 <= sensor <= 6:
            raise ValueError(f"矩阵传感器索引需在 0~6 之间，当前为 {sensor}")

        if rows is not None and cols is not None:
            size = self.encode_matrix_size(rows, cols)
            if size != self._matrix_size:
                self._write_holding(HR["matrix_size"], [size],
                                    interval=self.matrix_interval)
                self._matrix_size = size

        self._write_holding(HR["matrix_index"], [sensor],
                            interval=self.matrix_interval)
        self._matrix_index = sensor

    def read_matrix(self, sensor: Union[int, str],
                    rows: Optional[int] = None,
                    cols: Optional[int] = None,
                    verify_retries: int = 3) -> np.ndarray:
        """
        读取指定矩阵传感器，返回按行优先还原的二维 ndarray (dtype=uint8)。

        流程 (说明书 6.4 / 7.5):
          1. 保持寄存器 199 设置期望输出尺寸、198 选择传感器；
          2. 读输入寄存器 223/224 拿到实际来源与实际尺寸；
          3. 按实际 rows*cols 读取输入寄存器 225 起的数据区并 reshape。

        :param rows/cols: None 表示沿用设备当前尺寸设置(适用于原始尺寸未知的传感器，
                          例如手掌)，此时完全依据寄存器 224 的回报解析。
        :param verify_retries: 等待寄存器 223 切换到目标传感器的重试次数。
        """
        self.select_matrix_sensor(sensor, rows, cols)
        target = self._matrix_index

        actual_index = actual_size = 0
        for _ in range(max(1, verify_retries)):
            actual_index, actual_size = self._read_input(
                IR["matrix_index"], 2, interval=self.matrix_interval)
            if actual_index == target:
                break
            time.sleep(self.matrix_interval)

        if actual_index != target:
            raise RuntimeError(
                f"矩阵传感器切换未生效: 期望索引 {target}，寄存器 223 回报 {actual_index}")

        act_rows, act_cols = self.decode_matrix_size(actual_size)
        count = act_rows * act_cols
        if count == 0:
            raise RuntimeError(
                f"矩阵传感器 {target} 回报尺寸为 0 (寄存器 224 = 0x{actual_size:02X})，"
                f"请确认该传感器已搭载且尺寸设置为原始行列数的整数因数")
        if count > _MATRIX_DATA_MAX:
            raise RuntimeError(
                f"矩阵尺寸 {act_rows}x{act_cols}={count} 超出数据区容量 "
                f"{_MATRIX_DATA_MAX} (输入寄存器 225~320)")

        regs = self._read_input(IR["matrix_data"], count,
                                interval=self.matrix_interval)
        data = np.array([r & 0xFF for r in regs], dtype=np.uint8)
        return data.reshape((act_rows, act_cols))

    def _tip_matrix(self, sensor: int, sleep_time: float = 0.0) -> np.ndarray:
        """读取指尖矩阵传感器，原始尺寸 12 行 x 6 列。"""
        if sleep_time:
            time.sleep(sleep_time)
        return self.read_matrix(sensor, _TIP_MATRIX_ROWS, _TIP_MATRIX_COLS)

    # ==================================================================
    # 设备配置 (通信参数、保存、标定 —— 均为高风险操作)
    # ==================================================================
    def get_baudrate(self) -> int:
        """读取波特率设置 (保持寄存器 180 低 16 位 + 181 高 16 位)。"""
        lo, hi = self._read_holding(HR["baudrate"], 2)
        return (hi << 16) | lo

    def set_baudrate(self, baudrate: int, save: bool = False):
        """
        设置波特率 (保持寄存器 180/181)，范围 115200~921600。

        修改需写 184 保存并重启后生效；修改后主站必须用新波特率通信，
        请务必记录新值，否则将无法继续与设备通信。
        """
        if not 115200 <= baudrate <= 921600:
            raise ValueError(f"波特率需在 115200~921600 之间，当前为 {baudrate}")
        self._write_holding(HR["baudrate"], [baudrate & 0xFFFF, (baudrate >> 16) & 0xFFFF])
        if save:
            self.save_config()

    def get_device_id(self) -> int:
        """读取设备 ID 设置 (保持寄存器 182)。"""
        return self._read_holding(HR["device_id"], 1)[0]

    def set_device_id(self, device_id: int, save: bool = False):
        """
        设置设备 ID (保持寄存器 182)，有效从站地址 1~247。

        修改需写 184 保存并重启后生效；生效后主站必须用新 ID 通信。
        """
        if not 1 <= device_id <= 247:
            raise ValueError(f"从站地址需在 1~247 之间，当前为 {device_id}")
        self._write_holding(HR["device_id"], [device_id])
        if save:
            self.save_config()

    def save_config(self):
        """保存配置到 flash (保持寄存器 184 写 1)，写入前应确认供电稳定。"""
        self._write_holding(HR["save"], [1])

    def erase_position_calibration(self, confirm: bool = False):
        """清除位置标定数据 (保持寄存器 183 写 1)。高风险操作，需显式 confirm=True。"""
        if not confirm:
            raise RuntimeError("清除位置标定数据是高风险操作，请显式传入 confirm=True")
        self._write_holding(HR["erase_pos_cali"], [1])

    def factory_reset(self, confirm: bool = False):
        """恢复出厂设置 (保持寄存器 185 写 1)。高风险操作，需显式 confirm=True。"""
        if not confirm:
            raise RuntimeError("恢复出厂设置是高风险操作，请显式传入 confirm=True")
        self._write_holding(HR["factory_reset"], [1])

    # ==================================================================
    # 故障处理
    # ==================================================================
    @staticmethod
    def decode_fault(code: int) -> List[str]:
        """把单个故障码按 bit 位解析成故障名列表。"""
        code = int(code)
        return [FAULT_BITS[bit][0] for bit in sorted(FAULT_BITS) if code & (1 << bit)]

    def get_fault_detail(self) -> Dict[str, List[str]]:
        """
        读取全部故障码并按关节名解析，只返回存在故障的关节。

        注: soft_rotor_lock 在抓握物体时会正常上报，无需手动清除，松开后自动恢复。
        """
        faults = self.read_faults()
        detail = {}
        for name, code in zip(REG_KEYS, faults):
            names = self.decode_fault(code)
            if names:
                detail[name] = names
        return detail

    def clear_faults(self, vals: Optional[Sequence[int]] = None):
        """清除故障 (保持寄存器 90~119 写 1)。默认清除全部关节。"""
        regs = [1] * _REG_BLOCK if vals is None else self.sdk_to_registers(vals, fill=0)
        self._write_holding(HR["clear_fault"], regs)

    # ==================================================================
    # 固定 API 接口 (与 LinkerHandApi / CAN 版 LinkerHandL20Can 对齐)
    # ==================================================================
    def set_joint_positions(self, joint_angles: Optional[Sequence[int]] = None):
        """下发位置命令，接受 20 元素 SDK 向量 (也兼容 5 / 30 元素)。"""
        if joint_angles is None:
            return
        self.write_position_block(joint_angles)

    def set_speed(self, speed: Optional[Sequence[int]] = None):
        """设置速度，默认全部 200。"""
        self.write_speed_block([200] * _SDK_LEN if speed is None else speed)

    def set_torque(self, torque: Optional[Sequence[int]] = None):
        """设置目标扭矩，默认全部 200。"""
        self.write_torque_block([200] * _SDK_LEN if torque is None else torque)

    def set_current(self, current: Optional[Sequence[int]] = None):
        """设置电流限制 (保持寄存器 150~179)，默认全部 250。"""
        self.write_current_limit_block([250] * _SDK_LEN if current is None else current)

    def get_version(self) -> List[int]:
        """返回 200~205 的 6 个版本寄存器；异常时返回空列表交由上层告警。"""
        # try:
        #     return self.read_versions()
        # except Exception:
        #     return []
        v = self.read_versions()
        a = float(f"{v[1]}.{v[0]}")
        b = float(f"{v[3]}.{v[2]}")
        c = float(f"{v[5]}.{v[4]}")
        return [a, b, c]

    def get_serial_number(self) -> Union[str, List[int]]:
        """
        读取 UID (输入寄存器 212~222)。寄存器按半字存放 ASCII，
        可解码则返回字符串，否则返回原始寄存器列表。
        """
        try:
            regs = self.read_uid_registers()
        except Exception:
            return []
        chars = []
        for reg in regs:
            chars.append((reg >> 8) & 0xFF)
            chars.append(reg & 0xFF)
        text = bytes(chars).split(b"\x00")[0]
        try:
            decoded = text.decode("ascii").strip()
        except UnicodeDecodeError:
            return regs
        return decoded if decoded.isprintable() and decoded else regs

    def get_state(self) -> List[int]:
        """当前关节位置，返回 20 元素 SDK 向量。"""
        return self.registers_to_sdk(self.read_positions())

    def get_state_for_pub(self) -> List[int]:
        return self.get_state()

    def get_current_status(self) -> List[int]:
        return self.get_state()

    def get_current_pub_status(self) -> List[int]:
        return self.get_state()

    def get_speed(self) -> List[int]:
        """当前速度反馈，返回 20 元素 SDK 向量。"""
        return self.registers_to_sdk(self.read_speeds())

    def get_joint_speed(self) -> List[int]:
        return self.get_speed()

    def get_torque(self) -> List[int]:
        """当前扭矩反馈，返回 20 元素 SDK 向量。"""
        return self.registers_to_sdk(self.read_torques())

    def get_current(self) -> List[int]:
        """当前电流反馈，返回 20 元素 SDK 向量。"""
        return self.registers_to_sdk(self.read_currents())

    def get_temperature(self) -> List[int]:
        """当前温度反馈，返回 20 元素 SDK 向量。"""
        return self.registers_to_sdk(self.read_temperatures())

    def get_fault(self) -> List[int]:
        """当前故障码，返回 20 元素 SDK 向量。"""
        return self.registers_to_sdk(self.read_faults())

    def get_touch_type(self) -> int:
        """L20 指尖为矩阵式压力传感器，与 CAN 版保持一致返回 2。"""
        return self.touch_type

    def get_touch_sensor_type(self) -> int:
        """与 CAN 版接口同名，供 LinkerHandApi 初始化时调用。

        Modbus 寄存器表未提供"传感器类型"查询点，无法从硬件回读，
        因此直接返回构造函数中约定的 self.touch_type(=2)。
        """
        return self.touch_type

    def get_normal_force(self) -> List[int]:
        return self.read_normal_force()

    def get_tangential_force(self) -> List[int]:
        return self.read_tangential_force()

    def get_tangential_force_dir(self) -> List[int]:
        return self.read_tangential_force_dir()

    def get_approach_inc(self) -> List[int]:
        return self.read_approach_inc()

    def get_touch(self) -> List[int]:
        """法向力 / 切向力 / 切向力方向 / 接近增量，共 20 个值。"""
        return self.read_force_block()

    def get_thumb_matrix_touch(self, sleep_time: float = 0.0) -> np.ndarray:
        return self._tip_matrix(MATRIX_SENSOR["thumb"], sleep_time)

    def get_index_matrix_touch(self, sleep_time: float = 0.0) -> np.ndarray:
        return self._tip_matrix(MATRIX_SENSOR["index"], sleep_time)

    def get_middle_matrix_touch(self, sleep_time: float = 0.0) -> np.ndarray:
        return self._tip_matrix(MATRIX_SENSOR["middle"], sleep_time)

    def get_ring_matrix_touch(self, sleep_time: float = 0.0) -> np.ndarray:
        return self._tip_matrix(MATRIX_SENSOR["ring"], sleep_time)

    def get_little_matrix_touch(self, sleep_time: float = 0.0) -> np.ndarray:
        return self._tip_matrix(MATRIX_SENSOR["little"], sleep_time)

    def get_palm_matrix_touch(self, sleep_time: float = 0.0,
                              rows: Optional[int] = None,
                              cols: Optional[int] = None) -> np.ndarray:
        """
        读取手掌矩阵传感器 (索引 6)。

        协议文档未给出手掌传感器的原始行列数，因此 rows/cols 默认为 None，
        即沿用设备当前的尺寸设置并完全依据输入寄存器 224 的回报解析。
        """
        if sleep_time:
            time.sleep(sleep_time)
        return self.read_matrix(MATRIX_SENSOR["palm"], rows, cols)

    def get_matrix_touch(self) -> tuple:
        """五个指尖矩阵，顺序为 拇指、食指、中指、无名指、小指。"""
        return (self.get_thumb_matrix_touch(),
                self.get_index_matrix_touch(),
                self.get_middle_matrix_touch(),
                self.get_ring_matrix_touch(),
                self.get_little_matrix_touch())

    def get_matrix_touch_v2(self) -> tuple:
        return self.get_matrix_touch()

    def get_finger_order(self) -> List[str]:
        """SDK 20 元素向量各位置对应的关节名。"""
        return list(self.KEYS)

    # ==================================================================
    # CRC16 (说明书附录 A) —— 便于自行拼帧或校验抓包数据
    # ==================================================================
    @staticmethod
    def crc16(data: bytes) -> int:
        """Modbus RTU CRC16，初值 0xFFFF、多项式 0xA001，帧中低字节先发送。"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    # ==================================================================
    # 上下文管理
    # ==================================================================
    def close(self):
        if self.connected:
            self.cli.close()
            self.connected = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# ------------------------------- demo -------------------------------
if __name__ == "__main__":
    PORT = "/dev/ttyUSB0"
    SLAVE = DEFAULT_SLAVE_ID  # L20 出厂默认从站地址为 1

    try:
        with LinkerHandL20RS485(hand_id=SLAVE, modbus_port=PORT,
                               baudrate=DEFAULT_BAUDRATE) as hand:
            print(f"L20 已连接: {PORT}, slave={SLAVE}")
            print("版本信息:", hand.get_version_dict())
            print("序列号:", hand.get_serial_number())

            print("\n--- 实时反馈 (20 元素 SDK 顺序) ---")
            print("位置:", hand.get_state())
            print("速度:", hand.get_speed())
            print("扭矩:", hand.get_torque())
            print("温度:", hand.get_temperature())
            print("电流:", hand.get_current())
            print("故障:", hand.get_fault())          # 0 表示该关节无故障

            detail = hand.get_fault_detail()
            if detail:
                print("故障明细:", detail)

            print("\n--- 按关节名查看位置 ---")
            for name, val in hand.to_dict(hand.get_state()).items():
                print(f"  {name:16s} = {val}")

            # 运动示例: 张开 -> 握拳。执行前请确认设备处于安全状态。
            print("\n--- 运动示例 ---")
            hand.set_speed([180] * _SDK_LEN)
            open_pose = [255] * _SDK_LEN
            hand.set_joint_positions(open_pose)
            time.sleep(1.5)
            print("张开后位置:", hand.get_state())

            print("\n--- 指尖矩阵传感器 (12x6) ---")
            print("拇指矩阵:\n", hand.get_thumb_matrix_touch())
    except Exception as e:
        print(f"错误: {e}")
