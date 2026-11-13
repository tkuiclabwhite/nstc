#!/usr/bin/env python3
"""
順應控制（實驗）：手臂馬達切到「電流上限位置控制」（Dynamixel 模式 5）。
撞到東西時每一顆最多只出 Current Limit 這麼多力，被擋住或被推會讓開，不會一直硬推

    ros2 run armpackage compliance.py status          # 1~14 號現在的模式、電流上限
    ros2 run armpackage compliance.py apply           # 照最近一次 current_log.py 的建議上限切到模式 5
    ros2 run armpackage compliance.py apply 11 4      # 只切這幾顆
    ros2 run armpackage compliance.py apply --scale 0.8   # 上限都乘 0.8（想更軟、更安全時）
    ros2 run armpackage compliance.py restore         # 改回原本的位置控制（模式 3、原本的電流上限）

要在 driver_node 關著、馬達扭力關著的時候跑：改的是馬達的 EEPROM，扭力開著不能改。
這支不會幫你關扭力（一關手臂就會掉）：手臂在站姿時把手臂電源關掉再開，馬達開機時扭力是關的。

模式 5 下 driver、實機跟隨照常用（一樣送目標位置和速度），差別只在出力被上限卡住：
  上限太低 → 撐不住手臂會往下垂、跟不上目標（實機跟隨 2 秒沒更接近目標會停）
  上限太高 → 撞到東西時跟原本差不多
改的設定存在馬達裡，斷電不會回去，motionpackage 播的動作也一樣受影響，測完記得 restore。
原本的值存在 ~/.ros/arm_compliance/backup.json（第一次 apply 時存），restore 照它改回來。
開機自我檢查認得 apply 過的設定（applied.json），不會當成「設定被改過」。
"""
import argparse
import glob
import json
import os
import sys

from arm_config import ARMS, LABELS
from self_check import (BAUD, COMPLIANCE_APPLIED as APPLIED, MA_PER_UNIT, PORT, REG, SIGNED, _driver_running,
                        compliance_applied as applied)

ADDR_MODE, ADDR_CURRENT_LIMIT = 11, 38
MODE_POSITION, MODE_COMPLIANT = 3, 5
DIR = os.path.expanduser("~/.ros/arm_compliance")
BACKUP = os.path.join(DIR, "backup.json")      # 原本（模式 3）的模式、電流上限
CURRENT_LOG = os.path.expanduser("~/.ros/arm_current_log")
MOTORS = sorted(m for a in ARMS.values() for m in a["motors"])
WHERE = {m: f"{a['label']}第 {i + 1} 軸{LABELS[i]}" for a in ARMS.values() for i, m in enumerate(a["motors"])}


def load(path):
    try:
        return json.load(open(path, encoding="utf-8"))
    except FileNotFoundError:
        return {}


def save(path, data):
    os.makedirs(DIR, exist_ok=True)
    if data:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=1)
    elif os.path.exists(path):
        os.remove(path)


class Bus:
    def __init__(self, port, baud):
        from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
        self.ok, self.ph, self.pk = COMM_SUCCESS, PortHandler(port), PacketHandler(2.0)
        try:
            opened = os.path.exists(port) and self.ph.openPort() and self.ph.setBaudRate(baud)
        except OSError:
            opened = False
        if not opened:
            sys.exit(f"打不開 {port}：U2D2 有接上嗎？")

    def read(self, mid):
        data, res, _ = self.pk.readTxRx(self.ph, mid, 0, 147)
        if res != self.ok:
            return None
        return {k: int.from_bytes(bytes(data[a:a + n]), "little", signed=k in SIGNED) for k, (a, n) in REG.items()}

    def write(self, mid, addr, n, value):
        fn = self.pk.write1ByteTxRx if n == 1 else self.pk.write2ByteTxRx
        res, err = fn(self.ph, mid, addr, value)
        return res == self.ok and err == 0

    def close(self):
        self.ph.closePort()


def read_all(bus, ids):
    regs = {m: bus.read(m) for m in ids}
    missing = [m for m, r in regs.items() if r is None]
    if missing:
        sys.exit(f"{missing} 號沒回應：線、電源？")
    return regs


def need_torque_off(bus, regs):
    on = [m for m, r in regs.items() if r["torque"]]
    if on:
        sys.exit(f"{on} 號扭力開著，EEPROM 不能改。這支不幫你關扭力（一關手臂會掉）：\n"
                 "手臂在站姿（垂直）時把手臂電源關掉再開，馬達開機時扭力是關的，再跑一次")


def latest_summary(path):
    if path is None:
        files = sorted(glob.glob(os.path.join(CURRENT_LOG, "*_summary.json")))
        if not files:
            sys.exit("還沒有電流紀錄：先跑 current_log.py（driver、扭力開著，手臂動一輪）")
        path = files[-1]
    return path, json.load(open(path, encoding="utf-8"))


def run_status(bus):
    regs = read_all(bus, MOTORS)
    app, backup = applied(), load(BACKUP)
    print("馬達  位置               模式            電流上限        Goal Current  扭力")
    for m in MOTORS:
        r = regs[m]
        mode = {3: "3 位置控制", 5: "5 順應（電流上限）"}.get(r["operating_mode"], str(r["operating_mode"]))
        flag = "" if (r["operating_mode"] == MODE_COMPLIANT) == (m in app) else "  ⚠ 跟 applied.json 對不起來"
        print(f"{m:3d}   {WHERE[m]:<14} {mode:<14} {r['current_limit']:5d}（{r['current_limit'] * MA_PER_UNIT:4.0f} mA）"
              f"  {r['goal_current']:5d}        {'開' if r['torque'] else '關'}{flag}")
    print(f"\n原本的值：{'有備份（' + BACKUP + '）' if backup else '還沒備份（第一次 apply 時存）'}")


def run_apply(bus, ids, summary_path, scale):
    path, s = latest_summary(summary_path)
    ids = ids or [m for m in MOTORS if str(m) in s["motors"]]
    lacking = [m for m in ids if str(m) not in s["motors"]]
    if lacking:
        sys.exit(f"{path} 裡沒有 {lacking} 號的電流紀錄")
    regs = read_all(bus, MOTORS)
    need_torque_off(bus, regs)
    target = {m: int(min(2047, round(s["motors"][str(m)]["limit"] * scale))) for m in ids}
    print(f"電流紀錄：{path}（記了 {s['seconds']} 秒，建議上限 = 99% × {s['margin']}" +
          (f"，再 × {scale}" if scale != 1 else "") + "）")
    print("馬達  位置               模式       電流上限")
    for m in ids:
        r = regs[m]
        print(f"{m:3d}   {WHERE[m]:<14} {r['operating_mode']} → 5     {r['current_limit'] * MA_PER_UNIT:4.0f} → "
              f"{target[m] * MA_PER_UNIT:4.0f} mA（平常最多 {s['motors'][str(m)]['p99_ma']} mA）")
    if input("\n會改馬達的 EEPROM。輸入 yes 開始改：").strip() != "yes":
        return print("沒改")
    # 第一次切到模式 5 前，記下原本（位置控制）的值，restore 用
    backup = load(BACKUP)
    for m, r in regs.items():
        if str(m) not in backup and r["operating_mode"] == MODE_POSITION:
            backup[str(m)] = {"operating_mode": r["operating_mode"], "current_limit": r["current_limit"]}
    save(BACKUP, backup)
    app, bad = load(APPLIED), []
    for m in ids:
        # 先改電流上限再改模式：換模式時 Goal Current 會被設成 Current Limit
        ok = bus.write(m, ADDR_CURRENT_LIMIT, 2, target[m]) and bus.write(m, ADDR_MODE, 1, MODE_COMPLIANT)
        r = bus.read(m)
        if ok and r and r["operating_mode"] == MODE_COMPLIANT and r["current_limit"] == target[m] and r["goal_current"] > 0:
            app[str(m)] = target[m]
        else:
            bad.append(m)
    save(APPLIED, app)
    if bad:
        print(f"✗ {bad} 號沒改成功（其他的改好了）：再跑一次，或 restore 改回原本的")
    print(f"改好了 {len(ids) - len(bad)} 顆。下一步：self_check.py → 開 driver_node、實機跟隨，照 ~/arm_ws/README.md「順應控制」測試")


def run_restore(bus):
    backup, app = load(BACKUP), load(APPLIED)
    regs = read_all(bus, MOTORS)
    ids = [m for m in MOTORS if regs[m]["operating_mode"] == MODE_COMPLIANT or str(m) in app]
    if not ids:
        return print("沒有馬達在順應模式，不用改")
    need_torque_off(bus, regs)
    bad = []
    for m in ids:
        b = backup.get(str(m), {"operating_mode": MODE_POSITION, "current_limit": 2047})      # 沒備份就用出廠值
        ok = bus.write(m, ADDR_CURRENT_LIMIT, 2, b["current_limit"]) and bus.write(m, ADDR_MODE, 1, b["operating_mode"])
        r = bus.read(m)
        if ok and r and r["operating_mode"] == b["operating_mode"] and r["current_limit"] == b["current_limit"]:
            app.pop(str(m), None)
            print(f"{m:3d} 號 → 模式 {b['operating_mode']}、電流上限 {b['current_limit']}")
        else:
            bad.append(m)
    save(APPLIED, app)
    print(f"✗ {bad} 號沒改回來，再跑一次" if bad else "都改回原本的位置控制了")


def main():
    ap = argparse.ArgumentParser(description="順應控制（實驗）：切換手臂馬達的模式 5 / 改回模式 3")
    ap.add_argument("what", choices=["status", "apply", "restore"])
    ap.add_argument("ids", nargs="*", type=int, help="apply 只改這幾顆（預設全部）")
    ap.add_argument("--summary", help="用這個電流紀錄（預設最近一次的 *_summary.json）")
    ap.add_argument("--scale", type=float, default=1.0, help="建議上限再乘這個數")
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--baud", type=int, default=BAUD)
    args = ap.parse_args()
    if [m for m in args.ids if m not in MOTORS]:
        sys.exit(f"手臂馬達是 {MOTORS}")
    if _driver_running():
        sys.exit("driver_node 開著（佔住 U2D2）：先關掉 driver_node（camera.launch.py）再跑")
    bus = Bus(args.port, args.baud)
    try:
        if args.what == "status":
            run_status(bus)
        elif args.what == "apply":
            run_apply(bus, args.ids, args.summary, args.scale)
        else:
            run_restore(bus)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
