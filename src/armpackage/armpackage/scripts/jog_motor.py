#!/usr/bin/env python3
"""
校正用：一次只動一顆手臂馬達，看實機跟網頁的藍色模型是不是往同一個方向轉（對 DIR、確認馬達 ID）

    python3 ~/nstc/src/armpackage/armpackage/scripts/jog_motor.py 1 +100    # 1 號從目前位置 +100 刻度（≈ 9°）
    python3 ~/nstc/src/armpackage/armpackage/scripts/jog_motor.py 1 -100    # 往另一邊
    python3 ~/nstc/src/armpackage/armpackage/scripts/jog_motor.py 1 2048    # 回 2048
    python3 ~/nstc/src/armpackage/armpackage/scripts/jog_motor.py list      # 只列出 driver 有回報的馬達和刻度
    python3 ~/nstc/src/armpackage/armpackage/scripts/jog_motor.py check left   # 7 顆依序各動 +100 再回來，
                                                                                  # 每顆問同向 / 反向，最後印出新的 DIR
    （driver_node 要開著、扭力要開；先 source ~/nstc/install/setup.bash）

送 /joint_commands（跟 real_arm_bridge.py 同格式）給 driver_node（motor_control）。網頁的藍色模型也吃這個 topic，
會照「目前網頁的 DIR」畫出這顆馬達轉過去的樣子：
  實機跟藍色模型往同一個方向 → DIR 對；相反 → 這顆的 DIR 要翻（arm_config.py 跟網頁樣板都要改）
另外印出模型預測的掌心移動（照 arm_config.py 的 DIR），藍色模型看不清楚時可以對照

安全：
- 一次最多離目前位置 MAX_STEP 刻度，速度固定 SPEED（慢）
- 收到 driver 回報這顆馬達的位置才送：收不到 = 沒接上、ID 不對，或 driver 沒管這顆
- 實機跟隨開著時不動它在管的手（arm_config 裡 calibrated 的手），兩邊會搶
"""
import json
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from arm_config import ARMS, LABELS, TOOL_OFFSET, TPR, CENTER, ticks_to_rad, to_meas
from kinematics import ChainFK

MAX_STEP = 400       # 一次最多離目前位置多少刻度（≈ 35°）
SPEED = 15           # Dynamixel 速度單位（1 = 0.229 rpm）；15 ≈ 3.4 rpm，100 刻度約 0.4 秒
WAIT_REAL = 3.0      # 等 driver 回報馬達位置最多幾秒
REACH_TOL = 20       # 實機離目標幾個刻度以內算到了（≈ 1.8°；手臂受重力，常停在差 10 刻度左右）
CHECK_STEP = 100     # check 模式每顆動多少刻度（≈ 9°）
TICKS_PER_S = SPEED * 0.229 * 4096 / 60


def find_motor(mid):
    """馬達 ID → (手, 第幾軸)；不是手臂馬達回傳 (None, None)"""
    for arm, a in ARMS.items():
        if mid in a["motors"]:
            return arm, a["motors"].index(mid)
    return None, None


def _words(v, arm):
    """base_link 的向量 → (前, 外, 上) 三個分量；外 = 這隻手的外側"""
    return np.array([v[0], v[1] * ARMS[arm]["side"], v[2]])          # side：右手 −1（外 = −Y）、左手 +1


def describe_move(d, arm):
    """掌心位移（m，base_link）→「往前 3.1、往外 0.8 cm」"""
    parts = []
    for val, pos, neg in zip(_words(d, arm) * 100, ("前", "外", "上"), ("後", "內", "下")):
        if abs(val) >= 0.5:
            parts.append(f"往{pos if val > 0 else neg} {abs(val):.1f}")
    return "、".join(parts) + " cm" if parts else "幾乎沒動（< 0.5 cm）"


def describe_dir(v, arm, minor=True):
    """向量（base_link）→「下偏前」（minor=False 只回最大的那個方向）"""
    w = _words(v, arm)
    names = [("前", "後"), ("外", "內"), ("上", "下")]
    order = np.argsort(-np.abs(w))
    word = lambda i: names[i][0] if w[i] > 0 else names[i][1]
    second = minor and abs(w[order[1]]) >= 0.35 * np.linalg.norm(w)
    return word(order[0]) + (f"偏{word(order[1])}" if second else "")


def predict(fk, arm, ticks, i, target):
    """照 arm_config 的 DIR，這隻手第 i 軸從 ticks[i] 轉到 target 時，模型的掌心怎麼動。回傳說明文字"""
    q0 = ticks_to_rad(arm, ticks)
    t1 = list(ticks)
    t1[i] = target
    q1 = ticks_to_rad(arm, t1)
    T0, T1 = fk(q0), fk(q1)
    n0, n1 = T0[:3, 0], T1[:3, 0]                # 掌心法線（手掌 +X）
    ang = np.degrees(np.arccos(np.clip(n0 @ n1, -1, 1)))
    turn = f"，往{describe_dir(n1 - n0, arm, minor=False)}轉 {ang:.0f}°" if ang >= 1 else ""
    f, r, h = to_meas(T1[:3, 3])
    return (f"模型（目前 DIR = {ARMS[arm]['dir'][i]:+d}）：第 {i + 1} 軸 {np.degrees(q0[i]):+.1f}° → {np.degrees(q1[i]):+.1f}°\n"
            f"  掌心{describe_move(T1[:3, 3] - T0[:3, 3], arm)}（到 前 {f:.1f}、右 {r:.1f}、高 {h:.1f}）\n"
            f"  掌心朝{describe_dir(n0, arm)}{turn}")


def observable(fk, arm, ticks, i, target):
    """第 i 軸轉到 target 時，模型（目前 DIR）最明顯的動作：("移動" 或 "轉", (正向詞, 反向詞), 模型是不是正向)
    掌心移動 ≥ 1.5 cm 就問往哪邊移動（前/後、外/內、上/下）；滾轉軸掌心幾乎不動，問掌心面對的方向往哪邊轉"""
    t1 = list(ticks)
    t1[i] = target
    T0, T1 = fk(ticks_to_rad(arm, ticks)), fk(ticks_to_rad(arm, t1))
    names = [("前", "後"), ("外", "內"), ("上", "下")]
    d = _words(T1[:3, 3] - T0[:3, 3], arm)
    if np.abs(d).max() >= 0.015:
        k = int(np.argmax(np.abs(d)))
        return "移動", names[k], d[k] > 0
    t = _words(T1[:3, 0] - T0[:3, 0], arm)               # 掌心法線的變化
    k = int(np.argmax(np.abs(t)))
    return "轉", names[k], t[k] > 0


class Jog(Node):
    def __init__(self):
        super().__init__("jog_motor")
        self.real, self.follow = {}, None
        self.pub = self.create_publisher(JointState, "/joint_commands", 10)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 50)
        self.create_subscription(String, "/arm_real_status", self._status_cb, 10)

    def _js_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name.isdigit():                   # driver 的（MoveIt 的是 URDF 關節名）
                self.real[int(name)] = pos

    def _status_cb(self, msg):
        try:
            self.follow = json.loads(msg.data).get("state")
        except ValueError:
            pass

    def spin_for(self, sec, until=lambda: False):
        end = time.time() + sec
        while time.time() < end and not until():
            rclpy.spin_once(self, timeout_sec=0.05)

    def send(self, mid, tick):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position, msg.velocity = [str(mid)], [float(tick)], [float(SPEED)]    # 一定帶速度：0 在 driver 是全速
        self.pub.publish(msg)


def main():
    args = sys.argv[1:]
    ok = (args[:1] == ["list"] or (len(args) == 2 and args[0] == "check" and args[1] in ARMS)
          or (len(args) == 2 and args[0].isdigit()))
    if not ok:
        sys.exit(__doc__)
    rclpy.init()
    node = Jog()
    try:
        if args[0] == "list":
            run_list(node)
        elif args[0] == "check":
            run_check(node, args[1])
        else:
            run_jog(node, int(args[0]), args[1])
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def make_fk(arm):
    return ChainFK(get_package_share_directory("seven_axis_arm_urdf") + "/urdf/seven_axis_arm_urdf.urdf",
                   "base_link", ARMS[arm]["tip"], TOOL_OFFSET)


def ready(node, arm, motors):
    """等 driver 回報這幾顆的位置、確認可以送；不行就結束程式"""
    node.spin_for(WAIT_REAL, lambda: all(m in node.real for m in motors) and node.follow is not None)
    missing = [m for m in motors if m not in node.real]
    if missing:
        seen = sorted(node.real)
        sys.exit(f"driver 沒有回報 {missing} 號的位置：沒接上、ID 不對，或 driver 沒管這幾顆。\n"
                 f"driver 有回報的馬達：{seen if seen else '（一顆都沒有，driver_node 有開嗎？）'}")
    if ARMS[arm]["calibrated"] and node.follow in ("sync", "follow"):
        sys.exit(f"實機跟隨開著，{ARMS[arm]['label']}由 real_arm_bridge 在送指令。先在網頁關掉「實機跟隨」")
    node.spin_for(2.0, lambda: node.pub.get_subscription_count() > 0)
    if node.pub.get_subscription_count() == 0:
        sys.exit("沒有人在收 /joint_commands（driver_node 有開嗎？）")


def move(node, mid, target):
    """送 mid 號到 target、等它到。回傳 (結果 "ok" / "none" 沒動 / "stuck" 沒到, 說明)"""
    node.spin_for(0.2)                           # 拿最新的位置
    now = node.real[mid]
    for _ in range(3):                           # 多送幾次：第一筆偶爾會掉
        node.send(mid, target)
        node.spin_for(0.1)
    node.spin_for(abs(target - now) / TICKS_PER_S + 2.0, lambda: abs(node.real[mid] - target) <= REACH_TOL)
    node.spin_for(0.3)                           # 等它停穩再讀
    end = node.real[mid]
    if abs(end - target) <= REACH_TOL:
        return "ok", f"實機到了：刻度 {end:.0f}"
    if abs(end - now) <= REACH_TOL:
        return "none", f"實機沒動（刻度還是 {end:.0f}）：扭力有開嗎？或 driver 沒把指令送到這顆"
    return "stuck", f"實機動了但沒到：刻度 {end:.0f}（目標 {target}），可能卡住或被擋到"


def run_list(node):
    node.spin_for(WAIT_REAL, lambda: len(node.real) >= 14)
    if not node.real:
        sys.exit("收不到 driver 的 /joint_states（driver_node 有開嗎？）")
    for mid in sorted(node.real):
        arm, i = find_motor(mid)
        where = f"{ARMS[arm]['label']}第 {i + 1} 軸{LABELS[i]}" if arm else "（不是手臂馬達）"
        print(f"{mid:3d} 號  刻度 {node.real[mid]:7.1f}  {where}")
    missing = [m for a in ARMS.values() for m in a["motors"] if m not in node.real]
    if missing:
        print(f"沒有回報的手臂馬達：{missing}")


def run_jog(node, mid, v):
    arm, i = find_motor(mid)
    if arm is None:
        sys.exit(f"{mid} 號不是手臂馬達（arm_config.py：右手 {ARMS['right']['motors']}、左手 {ARMS['left']['motors']}）")
    ready(node, arm, [mid])
    now = node.real[mid]
    target = int(round(now + int(v))) if v[0] in "+-" else int(v)
    if not 0 <= target <= 4095:
        sys.exit(f"目標刻度 {target} 超出 0~4095")
    if abs(target - now) > MAX_STEP:
        sys.exit(f"一次最多動 {MAX_STEP} 刻度（目前 {now:.0f}，目標 {target}），分幾次送")
    ticks = [node.real.get(m, CENTER) for m in ARMS[arm]["motors"]]
    print(f"{mid} 號 = {ARMS[arm]['label']}第 {i + 1} 軸{LABELS[i]}：刻度 {now:.0f} → {target}"
          f"（{target - now:+.0f}，≈ {np.degrees((target - now) / TPR):+.1f}°）")
    print(predict(make_fk(arm), arm, ticks, i, target))
    res, text = move(node, mid, target)
    print(text + ("。看實機跟藍色模型是不是往同一個方向" if res == "ok" else ""))


def run_check(node, arm):
    """7 顆依序：轉 +CHECK_STEP 停住，問「實機往哪邊動」（不給預測、不跟藍色模型比），回原位；
    最後拿回答跟模型比，算出 DIR。之前問「跟預測一樣嗎」答案會前後矛盾，所以改成只問看到什麼"""
    a = ARMS[arm]
    ready(node, arm, a["motors"])
    fk = make_fk(arm)
    print(f"{a['label']} 7 顆依序檢查：每顆轉 +{CHECK_STEP} 刻度（≈ 9°）停住，回答實機**往哪邊動**，再回原位。\n"
          "前 = 機器人面向的那邊；內 = 往兩根立柱（身體中間）；外 = 離開身體。只看實機，不用看網頁。\n"
          "隨時按 Ctrl+C 停（正在動的那顆會送回原位）")
    results, mid, start = [], None, None
    try:
        for i, mid in enumerate(a["motors"]):
            node.spin_for(0.2)
            start = int(round(node.real[mid]))
            target = start + CHECK_STEP
            verb, (pos, neg), model_pos = observable(fk, arm, [node.real[m] for m in a["motors"]], i, target)
            what = "掌心面對的方向" if verb == "轉" else "掌心"
            print(f"\n[{i + 1}/7] {mid} 號 = {a['label']}第 {i + 1} 軸{LABELS[i]}：{start} → {target} → {start}"
                  f"（看{what}往{pos}{verb}還是往{neg}{verb}）")
            key = input("按 Enter 開始（s 跳過這顆、q 結束）：").strip().lower()
            if key == "q":
                break
            if key == "s":
                results.append((i, mid, "略過"))
                continue
            while True:
                res, text = move(node, mid, target)
                print("  去：" + text)
                if res != "ok":
                    print("  回：" + move(node, mid, start)[1])
                    results.append((i, mid, text))
                    break
                ans = ""
                while ans not in ("1", "2", "w", "r"):
                    ans = input(f"現在停在 +{CHECK_STEP}。實機的{what}往哪邊{verb}？\n"
                                f"  1 往{pos}{verb} / 2 往{neg}{verb} / w 動的不是這個關節 / r 回原位再動一次：").strip().lower()
                print("  回：" + move(node, mid, start)[1])
                if ans == "w":
                    results.append((i, mid, "動的不是這個關節（ID 不對）"))
                    break
                if ans != "r":
                    seen_pos = ans == "1"
                    results.append((i, mid, (("往" + (pos if seen_pos else neg) + verb), ("往" + (pos if model_pos else neg) + verb),
                                             seen_pos == model_pos)))
                    break
    except KeyboardInterrupt:
        if mid is not None:
            move(node, mid, start)
            print(f"\n停了，{mid} 號已送回 {start}")
    summary(arm, results)


def summary(arm, results):
    a = ARMS[arm]
    print(f"\n===== {a['label']}結果（每顆 +{CHECK_STEP} 刻度）=====")
    new_dir, done = list(a["dir"]), {mid for _, mid, _ in results}
    problems = [m for m in a["motors"] if m not in done]           # 沒測到的（按了 q）
    for i, mid, r in results:
        if isinstance(r, tuple):
            seen, model, same = r
            print(f"{mid:3d} 號 第 {i + 1} 軸{LABELS[i]}：實機{seen}，模型（DIR {a['dir'][i]:+d}）{model}"
                  + ("" if same else " → DIR 要翻"))
            if not same:
                new_dir[i] = -new_dir[i]
        else:
            print(f"{mid:3d} 號 第 {i + 1} 軸{LABELS[i]}：{r}")
            problems.append(mid)
    print(f"\n目前 DIR：{list(a['dir'])}\n建議 DIR：{new_dir}")
    if problems:
        print(f"⚠ {problems} 號沒有確認到方向（沒測、沒動、卡住或 ID 不對），處理完再重跑：jog_motor.py check {arm}")
    else:
        print("7 顆都確認了。把這段結果貼給 Claude，或照著改 arm_config.py 的 ARMS 跟網頁樣板的 DIR")


if __name__ == "__main__":
    main()
