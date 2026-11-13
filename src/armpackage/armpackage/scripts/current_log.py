#!/usr/bin/env python3
"""
順應控制第一步：記錄手臂馬達平常用多少電流（driver 發的 /joint_currents），算出每一顆的建議電流上限

    ros2 run armpackage current_log.py       # 開始記，Ctrl+C 結束：印統計、建議上限，存檔

driver_node 要開著、扭力要開。記的時候用實機跟隨把手臂移到平常會用到的姿勢：
回站姿、伸到最遠、抬到最高、對準幾個點；手臂往前平伸停幾秒（肩膀最吃力）。兩隻手都要動到。
速度用平常會用的，上限是照「平常最多用多少」訂的，記的時候動得比較慢，之後快一點就可能不夠

只讀，不送任何指令。存在 ~/.ros/arm_current_log/：
  <時間>.csv           每一筆：時間、1~14 號電流（driver 的原始單位）、1~14 號刻度
  <時間>_summary.json  每一顆的統計和建議上限；compliance.py apply 預設用最近一次的
"""
import csv
import datetime
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_config import ARMS, LABELS

MA_PER_UNIT = 2.69     # XM540-W270、XH540-W270 的 Present Current / Current Limit 單位（mA）
MARGIN = 1.5           # 建議上限 = 平常用到的電流（第 99 百分位）× 這個倍數
MIN_LIMIT = 112        # 建議上限最少這麼多（單位；112 ≈ 300 mA）：手腕平常幾乎不出力，太低會連自己都撐不住
MAX_LIMIT = 2047       # Current Limit 的最大值（≈ 5.5 A，出廠預設）
MIN_SECONDS = 30       # 記不到這麼多秒，建議值不可靠
LOG_DIR = os.path.expanduser("~/.ros/arm_current_log")
MOTORS = sorted(m for a in ARMS.values() for m in a["motors"])
WHERE = {m: (a["label"], i) for a in ARMS.values() for i, m in enumerate(a["motors"])}


class CurrentLog(Node):
    def __init__(self, path):
        super().__init__("current_log")
        self.cur = {m: [] for m in MOTORS}
        self.pos = {}
        self.t0, self.last_print = time.time(), time.time()
        self.f = open(path, "w", newline="")
        self.csv = csv.writer(self.f)
        self.csv.writerow(["t"] + [f"cur_{m}" for m in MOTORS] + [f"pos_{m}" for m in MOTORS])
        self.create_subscription(JointState, "/joint_currents", self._cur_cb, 50)
        self.create_subscription(JointState, "/joint_states", self._pos_cb, 50)

    def _pos_cb(self, msg):
        for name, p in zip(msg.name, msg.position):
            if name.isdigit():                      # driver 的（MoveIt 的是 URDF 關節名）
                self.pos[int(name)] = p

    def _cur_cb(self, msg):
        got = {int(n): c for n, c in zip(msg.name, msg.effort) if n.isdigit() and int(n) in self.cur}
        if not got:
            return
        for m, c in got.items():
            self.cur[m].append(abs(c))
        self.csv.writerow([f"{time.time() - self.t0:.3f}"] + [got.get(m, "") for m in MOTORS]
                          + [self.pos.get(m, "") for m in MOTORS])
        if time.time() - self.last_print > 5:
            self.last_print = time.time()
            top = sorted(((max(v), m) for m, v in self.cur.items() if v), reverse=True)[:3]
            print(f"已記 {time.time() - self.t0:.0f} 秒；目前最大："
                  + "、".join(f"{m} 號 {c * MA_PER_UNIT:.0f} mA" for c, m in top), flush=True)


def summarize(cur, seconds):
    """每一顆：中位數、第 99 百分位、最大（mA），建議上限（單位）"""
    out = {}
    for m, v in cur.items():
        if not v:
            continue
        a = np.array(v, float)
        p99 = float(np.percentile(a, 99))
        out[str(m)] = {"samples": len(a), "p50_ma": round(float(np.median(a)) * MA_PER_UNIT),
                       "p99_ma": round(p99 * MA_PER_UNIT), "max_ma": round(float(a.max()) * MA_PER_UNIT),
                       "limit": int(np.clip(np.ceil(p99 * MARGIN), MIN_LIMIT, MAX_LIMIT))}
    return {"time": datetime.datetime.now().isoformat(timespec="seconds"), "seconds": round(seconds),
            "margin": MARGIN, "ma_per_unit": MA_PER_UNIT, "motors": out}


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = f"{datetime.datetime.now():%Y%m%d_%H%M%S}"
    rclpy.init()
    node = CurrentLog(os.path.join(LOG_DIR, stamp + ".csv"))
    print("開始記錄 /joint_currents。用實機跟隨把兩隻手移到平常會用的姿勢，完成後按 Ctrl+C", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    seconds = time.time() - node.t0
    node.f.close()
    node.destroy_node()
    rclpy.try_shutdown()
    s = summarize(node.cur, seconds)
    if not s["motors"]:
        raise SystemExit("沒收到 /joint_currents：driver_node 有開嗎？")
    path = os.path.join(LOG_DIR, stamp + "_summary.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(s, f, ensure_ascii=False, indent=1)
    print(f"\n記了 {seconds:.0f} 秒。電流（mA）：中位數 / 99% / 最大 → 建議上限（= 99% × {MARGIN}）")
    for m in MOTORS:
        d = s["motors"].get(str(m))
        label, i = WHERE[m]
        if d is None:
            print(f"{m:3d} 號 {label}第 {i + 1} 軸{LABELS[i]}：沒收到")
            continue
        note = "（幾乎沒電流：扭力有開嗎？）" if d["max_ma"] < 15 else ""
        print(f"{m:3d} 號 {label}第 {i + 1} 軸{LABELS[i]}：{d['p50_ma']:5d} / {d['p99_ma']:5d} / {d['max_ma']:5d}"
              f" → {d['limit']:4d}（{d['limit'] * MA_PER_UNIT:.0f} mA）{note}")
    if seconds < MIN_SECONDS:
        print(f"⚠ 只記了 {seconds:.0f} 秒，建議值不可靠：至少記 {MIN_SECONDS} 秒，各種姿勢都要動到")
    print(f"存在 {path}\n下一步：compliance.py apply（driver_node 關掉、手臂在站姿時馬達斷電再開，扭力是關的）")


if __name__ == "__main__":
    main()
