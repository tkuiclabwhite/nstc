#!/usr/bin/env python3
"""
開機自我檢查：設定檔 + 14 顆手臂馬達。只讀，不開扭力、不動馬達、不改任何馬達設定

    ros2 run armpackage self_check.py                  # 全部。要在 driver_node 開之前跑（開著會佔住 U2D2）
    ros2 run armpackage self_check.py config           # 只檢查設定檔，不碰馬達，隨時可以跑
    ros2 run armpackage self_check.py --save-baseline  # 全部檢查，沒問題就把 14 顆的馬達設定存成基準
    （還沒 build 也可以 python3 ~/nstc/src/armpackage/armpackage/scripts/self_check.py）

每一項 ✓ 通過、△ 提醒、✗ 不通過。real_arm_bridge 開「實機跟隨」前會看：
最近一次完整檢查（含馬達）要通過、在 MAX_AGE 小時內，設定檔檢查也要通過，不然不開。

設定：
  - URDF、MoveIt 關節範圍讀得到；站姿掌心位置跟 v2 URDF 一樣（URDF、量測原點、掌心位置被改過會抓到）
  - 網頁 RobotViewer.html 是用目前的樣板和 URDF 產生的；網頁的 DIR、站姿刻度、馬達 ID、量測原點、掌心位置跟 arm_config.py 一樣
  - stand.ini 讀得到，1~14 跟校正時（全 2048）一樣；兩隻手都校正過
馬達（1~14 號，直接讀 Dynamixel 的暫存器）：
  - 有回應；型號是 X 系列（driver 照 X 系列的位址讀寫 1~14 號）
  - 沒有硬體錯誤（過熱、過載、電壓、編碼器、電路）；電壓、溫度在馬達自己設的範圍內，14 顆電壓差不多
  - 位置控制模式、速度式 profile；目前姿勢在 MoveIt 關節範圍內；馬達限位不會擋到 MoveIt 的範圍
  - 方向、零點偏移、模式、限位、保護設定跟基準一樣（被改過的話之前的校正不算數）
  - 連續讀 COMM_READS 次位置不掉

結果存在 ~/.ros/arm_self_check/（一次一個檔；latest.json = 最近一次完整檢查）
"""
import argparse
import datetime
import json
import os
import re
import socket
import subprocess
import sys
import time

import numpy as np
import yaml

from arm_config import (ARMS, CENTER, LABELS, MEAS_ORIGIN, STAND, STAND_INI, STAND_REAL, TOOL_OFFSET, rad_to_ticks,
                        ticks_to_rad, to_meas)
from kinematics import ChainFK, chain_limits

HERE = os.path.dirname(os.path.realpath(__file__))           # realpath：從 install/（連結）執行也找得到原始碼
SRC = os.path.dirname(os.path.dirname(HERE))
LOG_DIR = os.path.expanduser("~/.ros/arm_self_check")
BASELINE = os.path.join(HERE, "motor_baseline.json")        # 馬達設定的基準（--save-baseline 存的）
MAX_AGE = 12          # 完整檢查幾小時內有效（real_arm_bridge 看這個）
# compliance.py apply 過、現在應該是順應模式（模式 5）的馬達 → 電流上限：這些的模式、電流上限不跟基準比
COMPLIANCE_APPLIED = os.path.expanduser("~/.ros/arm_compliance/applied.json")
PORT, BAUD = "/dev/U2D2_P1", 1000000                        # 同 driver_node（motor_control）的預設
# 站姿（全 2048）掌心在實機量測座標的位置（cm），v2 URDF 算的；故意改了 URDF、MEAS_ORIGIN、TOOL_OFFSET 的話這裡也要改
STAND_PALM = {"right": (0.0, 20.9, 67.0), "left": (0.0, -20.9, 67.0)}
PALM_TOL = 0.5        # 站姿掌心差幾 cm 以內算一樣
STAND_INI_TOL = 100   # stand.ini 的站姿刻度離 2048 超過這麼多（≈ 9°）不通過；1 ~ 這個數提醒
LIMIT_TOL = 0.05      # 同 real_arm_bridge：超出 MoveIt 關節範圍 3° 以內不算
TEMP_WARN = 55        # 開機時馬達溫度（°C）超過這個提醒；離馬達自己的溫度上限 5° 以內不通過
VOLT_WARN = 1.0       # 電壓離馬達的下限不到這麼多（V）提醒；14 顆電壓最高最低差超過這麼多也提醒
# 電池充飽的電壓（V）。比馬達規格 14.8 V 高，也比馬達能設的電壓上限 16.0 V 高（暫存器最大 160），
# 2026-11-03 決定接受：電池比馬達上限高時馬達只記一筆「輸入電壓」錯誤、照樣動（Shutdown 沒勾電壓），不擋
VOLT_ACCEPT = 16.8
COMM_READS = 100      # 通訊測試：連續讀幾次位置
COMM_FAIL = 5         # 失敗幾次以上不通過（1 ~ 這個數提醒）
MA_PER_UNIT = 2.69    # XM540-W270、XH540-W270 的電流單位（mA）

# Dynamixel X 系列的暫存器（位址, 長度），一次讀 0~146
REG = {"model": (0, 2), "firmware": (6, 1), "drive_mode": (10, 1), "operating_mode": (11, 1),
       "homing_offset": (20, 4), "temperature_limit": (31, 1), "max_voltage": (32, 2), "min_voltage": (34, 2),
       "current_limit": (38, 2), "velocity_limit": (44, 4), "max_position": (48, 4), "min_position": (52, 4),
       "shutdown": (63, 1), "torque": (64, 1), "hw_error": (70, 1), "position": (132, 4),
       "voltage": (144, 2), "temperature": (146, 1), "goal_current": (102, 2)}
SIGNED = {"homing_offset", "position", "goal_current"}
ADDR_POSITION = 132
# 跟基準比的設定 → 中文；韌體不一樣只提醒，其他不通過
BASE_KEYS = {"model": "型號", "firmware": "韌體", "drive_mode": "方向 / profile 設定（Drive Mode）", "operating_mode": "控制模式",
             "homing_offset": "零點偏移", "temperature_limit": "溫度上限", "max_voltage": "電壓上限",
             "min_voltage": "電壓下限", "current_limit": "電流上限", "velocity_limit": "速度上限",
             "max_position": "位置上限", "min_position": "位置下限", "shutdown": "保護（Shutdown）"}
X_MODELS = {1000: "XH430-W350", 1010: "XH430-W210", 1020: "XM430-W350", 1030: "XM430-W210", 1040: "XH430-V350",
            1050: "XH430-V210", 1060: "XL430-W250", 1070: "XC430-W150", 1080: "XC430-W240", 1090: "2XL430-W250",
            1100: "XH540-W270", 1110: "XH540-W150", 1120: "XM540-W270", 1130: "XM540-W150", 1140: "XH540-V270",
            1150: "XH540-V150", 1160: "2XC430-W250", 1170: "XW540-T260", 1180: "XW540-T140", 1190: "XL330-M077",
            1200: "XL330-M288", 1210: "XC330-T181", 1220: "XC330-T288", 1230: "XC330-M181", 1240: "XC330-M288"}
HW_ERRORS = {0: "輸入電壓超出範圍", 2: "過熱", 3: "編碼器異常", 4: "電路異常", 5: "過載"}
SHUTDOWN_NEED = {2: "過熱", 4: "電路異常", 5: "過載"}     # 這幾種錯誤發生時馬達應該自己斷扭力（出廠預設）
MARK = {"ok": "✓", "warn": "△", "fail": "✗"}
ORDER = {"ok": 0, "warn": 1, "fail": 2}


def _share(pkg):
    """套件的 share 目錄；沒 source ROS 時用 src 裡的（同 build_viewer）"""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except (ImportError, LookupError):
        return os.path.join(SRC, pkg)


def _paths():
    urdf = os.path.join(_share("seven_axis_arm_urdf"), "urdf", "seven_axis_arm_urdf.urdf")
    limits = os.path.join(_share("seven_axis_arm_moveit_config"), "config", "joint_limits.yaml")
    from build_viewer import WEB      # ~/nstc/web
    return urdf, limits, os.path.join(WEB, "RobotViewer.template.html"), os.path.join(WEB, "RobotViewer.html")


def moveit_limits():
    """每隻手 MoveIt 規劃用的關節範圍（URDF ∩ joint_limits.yaml），同 real_arm_bridge"""
    urdf, limits, _, _ = _paths()
    over = {k: (v["min_position"], v["max_position"]) for k, v in yaml.safe_load(open(limits))["joint_limits"].items()
            if v.get("has_position_limits")}
    return {a: chain_limits(urdf, ARMS[a]["joints"], over) for a in ARMS}


def _js_object(src, name):
    """樣板裡的 const NAME = { 8:1, RshoulderJoint:8, … } → {鍵: 整數}"""
    m = re.search(r"const %s\s*=\s*\{(.*?)\};" % name, src, re.S)
    if m is None:
        raise ValueError(f"樣板裡找不到 const {name}")
    return {k: int(v) for k, v in re.findall(r"(\w+)\s*:\s*(-?\d+)", re.sub(r"//[^\n]*", "", m.group(1)))}


def _js_vec3(src, name):
    m = re.search(r"const %s\s*=\s*new THREE\.Vector3\(([^)]*)\)" % name, src)
    if m is None:
        raise ValueError(f"樣板裡找不到 const {name}")
    return [float(v) for v in m.group(1).split(",")]


def _diff(web, want):
    """兩個 dict 不一樣的鍵 →「8 號 網頁 1 / arm_config -1」"""
    keys = sorted(set(web) | set(want), key=str)
    return "、".join(f"{k}：網頁 {web.get(k, '沒有')} / arm_config {want.get(k, '沒有')}" for k in keys if web.get(k) != want.get(k))


def _web_built_from(html, tpl, urdf):
    """RobotViewer.html 是不是用這份樣板 + URDF 產生的（網格不比，STL 很少改）"""
    from build_viewer import parse_urdf
    links, joints = parse_urdf(urdf)
    if any(json.dumps(v, separators=(",", ":"), ensure_ascii=False) not in html for v in (links, joints)):
        return False
    pos = 0
    for part in re.split(r"__(?:URDF_LINKS|URDF_JOINTS|MESH_DATA)__", tpl):     # 樣板的程式碼要依序原封不動出現
        i = html.find(part, pos)
        if i < 0:
            return False
        pos = i + len(part)
    return True


def config_checks():
    """設定檔檢查（不碰硬體）→ [(等級, 說明)]；real_arm_bridge 啟動時也跑這個"""
    out = []
    urdf, limits, tpl_path, html_path = _paths()
    # URDF、MoveIt 關節範圍、站姿掌心
    try:
        moveit_limits()
        for a in ARMS:
            p = np.round(to_meas(ChainFK(urdf, "base_link", ARMS[a]["tip"], TOOL_OFFSET)(STAND)[:3, 3]), 1) + 0.0
            txt = f"{ARMS[a]['label']}站姿掌心 前 {p[0]:.1f}、右 {p[1]:.1f}、高 {p[2]:.1f} cm"
            if np.abs(p - STAND_PALM[a]).max() <= PALM_TOL:
                out.append(("ok", txt + "（跟 v2 URDF 一樣）"))
            else:
                out.append(("fail", txt + f"，應該是 {STAND_PALM[a]}：URDF、MEAS_ORIGIN 或 TOOL_OFFSET 被改過"
                                          "（故意改的話也要改 self_check.py 的 STAND_PALM）"))
    except Exception as e:             # 讀不到、解析失敗、少關節……都算這項沒過
        out.append(("fail", f"URDF / MoveIt 關節範圍讀不到：{e}（{urdf}、{limits}）"))
    # 網頁：樣板的常數跟 arm_config 一樣、RobotViewer.html 有重建
    try:
        tpl = open(tpl_path, encoding="utf-8").read()
        want = {"DIR": {str(m): d for a in ARMS.values() for m, d in zip(a["motors"], a["dir"])},
                "STAND_GP": {str(m): g for a in ARMS.values() for m, g in zip(a["motors"], a["stand_gp"])},
                "JOINT_TO_MOTOR": {j: m for a in ARMS.values() for j, m in zip(a["joints"], a["motors"])}}
        bad = [f"{k}（{_diff(_js_object(tpl, k), v)}）" for k, v in want.items() if _js_object(tpl, k) != v]
        for k, v in (("MEAS_ORIGIN", MEAS_ORIGIN), ("TOOL_OFFSET", TOOL_OFFSET)):
            if not np.allclose(_js_vec3(tpl, k), v, atol=1e-6):
                bad.append(f"{k}（網頁 {_js_vec3(tpl, k)} / arm_config {list(v)}）")
        if bad:
            out.append(("fail", "網頁樣板跟 arm_config.py 不一樣：" + "；".join(bad) + "。兩邊改成一樣再重建網頁"))
        else:
            out.append(("ok", "網頁的 DIR、站姿刻度、馬達 ID、量測原點、掌心位置跟 arm_config.py 一樣"))
        if _web_built_from(open(html_path, encoding="utf-8").read(), tpl, urdf):
            out.append(("ok", "RobotViewer.html 是用目前的樣板和 URDF 產生的"))
        else:
            out.append(("fail", "RobotViewer.html 跟目前的樣板或 URDF 不一樣（改了沒重建）：ros2 run armpackage build_viewer.py"))
    except Exception as e:
        out.append(("fail", f"網頁檢查失敗：{e}"))
    # stand.ini：刻度換算用它；校正是在全 2048 做的
    if STAND_REAL is None:
        out.append(("fail", f"讀不到 {STAND_INI}（motionstate 3）：刻度換算要用它"))
    else:
        offs = [(m, g + off, off) for a in ARMS.values() for m, g, off in zip(a["motors"], a["stand_gp"], a["offset"]) if off]
        txt = "、".join(f"{m} 號 {t}" for m, t, _ in offs)
        if any(abs(off) > STAND_INI_TOL for _, _, off in offs):
            out.append(("fail", f"stand.ini 的站姿刻度跟校正時差很多（{txt}）：站姿被重存過？刻度換算會整個偏掉"))
        elif offs:
            out.append(("warn", f"stand.ini 有幾顆不是 2048（{txt}）：實機跟隨會把這個差當成修正"))
        else:
            out.append(("ok", "stand.ini 的站姿 1~14 號全 2048（跟校正時一樣）"))
    not_cal = [a["label"] for a in ARMS.values() if not a["calibrated"]]
    if not_cal:
        out.append(("warn", f"{'、'.join(not_cal)}還沒校正（calibrated = False）：實機跟隨只動模擬"))
    else:
        out.append(("ok", "兩隻手都校正過（arm_config.py 的 calibrated）"))
    return out


def _driver_running():
    try:
        # 比對執行檔路徑（…/lib/motor_control/driver_node），只寫 driver_node 會抓到 tail、grep 之類的
        return subprocess.run(["pgrep", "-f", "motor_control/driver_node"], stdout=subprocess.DEVNULL).returncode == 0
    except FileNotFoundError:
        return False


def _read_motors(port, baud, rep):
    """直接讀 1~14 號的暫存器 → ({馬達 ID: {名稱: 值}}, (通訊失敗次數, 每次幾 ms))；開不了 port 回傳 ({}, None)"""
    from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler
    ph, pk = PortHandler(port), PacketHandler(2.0)
    try:
        opened = os.path.exists(port) and ph.openPort() and ph.setBaudRate(baud)
        why = "U2D2 有接上嗎？"
    except OSError as e:
        opened, why = False, str(e)
    if not opened:
        rep.add("fail", f"打不開 {port}（{baud} bps）：{why}")
        return {}, None
    try:
        regs = {}
        for mid in sorted(m for a in ARMS.values() for m in a["motors"]):
            _, res, _ = pk.ping(ph, mid)
            if res != COMM_SUCCESS:
                rep.add("fail", f"{mid} 號沒回應（{pk.getTxRxResult(res)}）：線、電源、ID")
                continue
            data, res, _ = pk.readTxRx(ph, mid, 0, 147)
            if res != COMM_SUCCESS:
                rep.add("fail", f"{mid} 號讀不到暫存器（{pk.getTxRxResult(res)}）")
                continue
            regs[mid] = {k: int.from_bytes(bytes(data[a:a + n]), "little", signed=k in SIGNED) for k, (a, n) in REG.items()}
        if not regs:
            return regs, None
        # 通訊：照 driver 的方式（GroupSyncRead）連續讀位置
        gr = GroupSyncRead(ph, pk, ADDR_POSITION, 4)
        for mid in regs:
            gr.addParam(mid)
        fails, t0 = 0, time.time()
        for _ in range(COMM_READS):
            if gr.txRxPacket() != COMM_SUCCESS or not all(gr.isAvailable(m, ADDR_POSITION, 4) for m in regs):
                fails += 1
        return regs, (fails, (time.time() - t0) / COMM_READS * 1000)
    finally:
        ph.closePort()


def compliance_applied():
    """compliance.py apply 過的馬達 → 電流上限（單位）；沒有回傳 {}"""
    try:
        return {int(k): v for k, v in json.load(open(COMPLIANCE_APPLIED, encoding="utf-8")).items()}
    except (OSError, ValueError):
        return {}


def _volt_high(r):
    """這顆馬達的「輸入電壓」錯誤／電壓超出，是不是電池比它的電壓上限高（已接受的那種，不是電壓太低）"""
    return (r["voltage"] > r["max_voltage"] or r["hw_error"] & 1) and r["min_voltage"] <= r["voltage"] <= VOLT_ACCEPT * 10


def _check_motor(arm, i, r, lim, comp=None):
    """一顆馬達 → (等級, 問題清單)；comp = compliance.py 給這顆設的電流上限（沒 apply 過是 None）"""
    a, probs = ARMS[arm], []
    if r["model"] not in X_MODELS:
        if 1000 <= r["model"] < 1300:
            probs.append(("warn", f"不認得的型號 {r['model']}（當 X 系列）"))
        else:
            probs.append(("fail", f"型號 {r['model']} 不是 X 系列：driver 照 X 系列的位址讀寫 1~14 號，會讀錯"))
    v, vmin, vmax = r["voltage"] / 10, r["min_voltage"] / 10, r["max_voltage"] / 10
    err = r["hw_error"]
    if _volt_high(r):              # 「輸入電壓」是電池比馬達上限高：已接受，不算（motor_checks 會統一說一次）
        err &= ~1
    if err:
        bits = "、".join(t for b, t in HW_ERRORS.items() if err >> b & 1) or str(err)
        probs.append(("fail", f"硬體錯誤：{bits}（要斷電重開才會清掉，先找原因）"))
    if v < vmin:
        probs.append(("fail", f"電壓 {v:.1f} V 低於這顆馬達的下限 {vmin:.1f} V"))
    elif v > VOLT_ACCEPT:
        probs.append(("warn", f"電壓 {v:.1f} V 比電池充飽（{VOLT_ACCEPT} V）還高：電源接對了嗎？"))
    elif v < vmin + VOLT_WARN:
        probs.append(("warn", f"電壓 {v:.1f} V 快到下限 {vmin:.1f} V"))
    t, tlim = r["temperature"], r["temperature_limit"]
    if t >= tlim - 5:
        probs.append(("fail", f"溫度 {t}°C，快到上限 {tlim}°C"))
    elif t >= TEMP_WARN:
        probs.append(("warn", f"溫度 {t}°C 偏高（剛開機應該接近室溫）"))
    mode = r["operating_mode"]
    if comp is not None and mode != 5:
        probs.append(("fail", f"compliance.py 記錄這顆是順應模式，但控制模式是 {mode}：跑 compliance.py status 看看"))
    elif mode == 5 and comp is not None:                # 順應控制（compliance.py apply 過的）
        if r["current_limit"] != comp:
            probs.append(("fail", f"電流上限 {r['current_limit']} 跟 compliance.py 設的 {comp} 不一樣"))
        elif r["goal_current"] <= 0:
            probs.append(("fail", f"Goal Current = {r['goal_current']}：開扭力會沒力，手臂會掉"))
    elif mode in (4, 5):
        probs.append(("warn", f"控制模式 {mode}（多圈）：刻度可能超出 0~4095，換算是照單圈寫的"))
    elif mode != 3:
        probs.append(("fail", f"控制模式 {mode} 不是位置控制（3）"))
    if r["drive_mode"] >> 2 & 1:
        probs.append(("fail", "Drive Mode 是時間式 profile：driver 送的速度會被當成毫秒，動作會非常快"))
    missing = [t for b, t in SHUTDOWN_NEED.items() if not r["shutdown"] >> b & 1]
    if missing:
        probs.append(("warn", f"{'、'.join(missing)}時不會自己斷扭力（Shutdown = {r['shutdown']}）"))
    pos = r["position"]
    if not 0 <= pos <= 4095:
        probs.append(("fail", f"刻度 {pos} 超出 0~4095"))
    elif lim is not None:
        lo, hi = lim
        q = ticks_to_rad(arm, [pos if k == i else CENTER for k in range(7)])[i]
        if q < lo[i] - LIMIT_TOL or q > hi[i] + LIMIT_TOL:
            probs.append(("warn", f"現在 {np.degrees(q):.0f}°，MoveIt 範圍 {np.degrees(lo[i]):.0f}~{np.degrees(hi[i]):.0f}°："
                                  "實機跟隨開不了，先轉回範圍內"))
        if mode == 3:              # 位置控制模式下，目標會被夾在馬達自己的限位裡
            ends = []
            for qe in (lo[i], hi[i]):
                qq = STAND.copy()
                qq[i] = qe
                ends.append(rad_to_ticks(arm, qq)[a["motors"][i]])
            t1, t2 = sorted(ends)
            if t1 < r["min_position"] or t2 > r["max_position"]:
                probs.append(("warn", f"馬達限位 {r['min_position']}~{r['max_position']} 比 MoveIt 的範圍（刻度 {t1}~{t2}）窄："
                                      "MoveIt 規劃到限位外時實機會停住"))
    level = max((lv for lv, _ in probs), key=ORDER.get, default="ok")
    return level, [t for _, t in probs]


def motor_checks(port, baud, rep, save_baseline=False):
    """馬達檢查（driver 開著時不能跑）→ {馬達 ID: 暫存器}（沒回應的不在裡面）"""
    try:
        regs, comm = _read_motors(port, baud, rep)
    except ImportError:
        rep.add("fail", "找不到 dynamixel_sdk（driver_node 也用它）：pip install dynamixel-sdk")
        return {}
    if not regs:
        return regs
    try:
        lims = moveit_limits()
    except Exception:              # 設定那邊已經報過了，這裡只是不檢查範圍
        lims = {}
    comp = compliance_applied()
    for arm, a in ARMS.items():
        for i, mid in enumerate(a["motors"]):
            if mid not in regs:
                continue
            r = regs[mid]
            level, probs = _check_motor(arm, i, r, lims.get(arm), comp.get(mid))
            info = (f"{mid:2d} 號 {a['label']}第 {i + 1} 軸{LABELS[i]}：{X_MODELS.get(r['model'], r['model'])}、"
                    f"{r['voltage'] / 10:.1f} V、{r['temperature']}°C、刻度 {r['position']}、扭力{'開' if r['torque'] else '關'}"
                    + (f"、順應模式（電流上限 {r['current_limit'] * MA_PER_UNIT:.0f} mA）" if mid in comp else ""))
            rep.add(level, info + ("" if not probs else "。" + "；".join(probs)))
    volts = [r["voltage"] / 10 for r in regs.values()]
    high = sorted(m for m, r in regs.items() if _volt_high(r))
    if high:
        rep.add("ok", f"{len(high)} 顆記了「輸入電壓超出範圍」：電池 {max(regs[m]['voltage'] for m in high) / 10:.1f} V"
                      " 比馬達能設的上限 16.0 V 高"
                      "（剛充飽會這樣）。馬達只記錄、照樣動，不擋（2026-11-03 決定接受這個電壓）")
    if max(volts) - min(volts) > VOLT_WARN:
        rep.add("warn", f"各顆電壓差 {max(volts) - min(volts):.1f} V（{min(volts):.1f}~{max(volts):.1f}）：電源線、接頭壓降？")
    # 保護設定 14 顆應該一樣；跟大多數不一樣的，多半是以前被人改過（例如 8 號的電壓上限 10.0 V）
    for k, name, scale in (("max_voltage", "電壓上限", 10), ("min_voltage", "電壓下限", 10),
                           ("temperature_limit", "溫度上限", 1), ("shutdown", "保護（Shutdown）", 1)):
        vals = [r[k] for r in regs.values()]
        common = max(set(vals), key=vals.count)
        odd = {m: r[k] for m, r in regs.items() if r[k] != common}
        if odd:
            fmt = (lambda x: f"{x / scale:.1f} V") if scale == 10 else str
            rep.add("warn", "、".join(f"{m} 號" for m in odd) + f"的{name}是 " + "、".join(fmt(x) for x in odd.values())
                    + f"，其他顆都是 {fmt(common)}：是馬達裡的設定跟別顆不一樣（多半是以前被改過），不是電源或馬達壞了")
    if comm is not None:
        fails, ms = comm
        txt = f"通訊：連續讀 {COMM_READS} 次 {len(regs)} 顆的位置，失敗 {fails} 次，每次 {ms:.1f} ms"
        rep.add("fail" if fails >= COMM_FAIL else "warn" if fails else "ok", txt)
    # 基準：馬達設定被改過的話，之前的校正（方向、零點）不算數
    rep.group("基準")
    base = None
    try:
        base = json.load(open(BASELINE, encoding="utf-8"))
    except FileNotFoundError:
        rep.add("warn", "還沒有馬達設定的基準：確認實機都正常後跑 self_check.py --save-baseline")
    except ValueError as e:
        rep.add("fail", f"{BASELINE} 壞了：{e}")
    if base:
        same = True
        for mid, r in regs.items():
            b = base["motors"].get(str(mid))
            if b is None:
                rep.add("fail", f"{mid} 號不在基準裡")
                same = False
                continue
            for k, name in BASE_KEYS.items():
                if mid in comp and k in ("operating_mode", "current_limit"):      # 順應控制改的，上面已經檢查過
                    continue
                if r[k] != b[k]:
                    rep.add("warn" if k == "firmware" else "fail", f"{mid} 號的{name}跟基準不一樣：{b[k]} → {r[k]}")
                    same = same and k == "firmware"
        if same:
            rep.add("ok", f"{len(regs)} 顆的設定跟基準一樣（{base['saved'].replace('T', ' ')} 存的）")
    if save_baseline:
        if comp:
            rep.add("fail", "沒存基準：有馬達在順應模式，基準要存原本的位置控制。先 compliance.py restore")
        elif len(regs) < 14 or rep.count("fail", skip=("基準",)):
            rep.add("fail", "沒存基準：14 顆都要讀到、除了基準以外都要通過")
        else:
            json.dump({"saved": datetime.datetime.now().isoformat(timespec="seconds"),
                       "motors": {str(m): {k: r[k] for k in BASE_KEYS} for m, r in regs.items()}},
                      open(BASELINE, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
            rep.add("ok", f"基準存到 {BASELINE}")
    return regs


class Report:
    def __init__(self):
        self.items, self.cur = [], ""

    def group(self, name):
        self.cur = name
        print(f"\n[{name}]")

    def add(self, level, msg):
        self.items.append({"level": level, "group": self.cur, "msg": msg})
        print(f"  {MARK[level]} {msg}")

    def count(self, level, skip=()):
        return sum(1 for i in self.items if i["level"] == level and i["group"] not in skip)


def last_check():
    """real_arm_bridge 開實機跟隨前問：最近一次完整檢查能不能用 → (能不能, 說明)"""
    try:
        rec = json.load(open(os.path.join(LOG_DIR, "latest.json"), encoding="utf-8"))
    except (OSError, ValueError):
        return False, "還沒做開機自我檢查"
    when = rec["time"].replace("T", " ")[:16]
    if not rec["passed"]:
        fails = [i["msg"] for i in rec["items"] if i["level"] == "fail"]
        more = f"…等 {len(fails)} 項" if len(fails) > 2 else ""
        return False, f"開機自我檢查（{when}）沒過：" + "；".join(fails[:2]) + more
    age = (time.time() - rec["stamp"]) / 3600
    if age > MAX_AGE:
        return False, f"開機自我檢查是 {age:.0f} 小時前（{when}），超過 {MAX_AGE} 小時"
    warns = sum(1 for i in rec["items"] if i["level"] == "warn")
    return True, f"開機自我檢查 {when} 通過" + (f"（{warns} 個提醒）" if warns else "")


def main():
    ap = argparse.ArgumentParser(description="開機自我檢查（只讀，不動馬達）")
    ap.add_argument("what", nargs="?", choices=["all", "config"], default="all",
                    help="all：設定 + 馬達（driver_node 開之前跑）；config：只檢查設定檔")
    ap.add_argument("--save-baseline", action="store_true", help="檢查完把 14 顆的馬達設定存成基準")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--baud", type=int, default=BAUD)
    args = ap.parse_args()
    now = datetime.datetime.now()
    print(f"開機自我檢查 {now:%Y-%m-%d %H:%M:%S}（{'只有設定' if args.what == 'config' else '設定 + 馬達'}）")
    rep = Report()
    rep.group("設定")
    for level, msg in config_checks():
        rep.add(level, msg)
    regs = None                    # None = 沒檢查馬達
    if args.what == "all":
        rep.group("馬達")
        if _driver_running():
            rep.add("fail", "driver_node 開著（佔住 U2D2），沒辦法檢查馬達：先關掉 driver_node（camera.launch.py）再跑")
        else:
            regs = motor_checks(args.port, args.baud, rep, args.save_baseline)
    fails, warns = rep.count("fail"), rep.count("warn")
    passed = fails == 0
    rec = {"time": now.isoformat(timespec="seconds"), "stamp": now.timestamp(), "host": socket.gethostname(),
           "mode": args.what, "passed": passed, "items": rep.items,
           "motors": {str(m): r for m, r in (regs or {}).items()}}
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, f"{now:%Y%m%d_%H%M%S}_{args.what}.json")
    # latest.json 給 real_arm_bridge 看：有檢查到馬達才更新（沒過也要蓋掉之前通過的那次）
    for p in [path] + ([os.path.join(LOG_DIR, "latest.json")] if regs is not None else []):
        with open(p, "w", encoding="utf-8") as f:
            json.dump(rec, f, ensure_ascii=False, indent=1)
    print(f"\n{'通過' if passed else f'沒過：{fails} 項 ✗'}" + (f"，{warns} 個提醒 △" if warns else "") + f"。結果存在 {path}")
    if args.what == "all" and passed:
        print(f"{MAX_AGE} 小時內可以開實機跟隨")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
