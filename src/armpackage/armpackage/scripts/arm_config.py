"""
雙手的設定都在這裡：關節名稱、手指、站姿、馬達刻度換算、掌心位置、抓取方向

右手的刻度換算跟 RobotViewer.html 的 ticksToRad 同一組：
    刻度 = 2048 + DIR × 角度 × TPR + (實機站姿刻度 − 理想站姿刻度)
  理想站姿刻度 stand_gp：URDF 零位 = 2048 時，站姿（STAND）的刻度。2026-10-21 起全部 2048（站姿 = URDF 零位）
  實機站姿刻度 stand_real：~/nstc/config/Parameter/stand.ini（motionstate 3，目前 1~14 全 2048）
  所以實機的站姿（全 2048）= MoveIt 的站姿 STAND（全 0）
  2026-10-20/21 實機一次轉一顆確認過方向（全 2048、單顆送刻度）：8 號增加 = 往前舉、9 號增加 = 往外翻、
  10 號 → 2389 往外、11 號 → 1707 往前、12 號 → 2276 掌心轉向朝前、13 號增加 = 往後、14 號增加 = 往外
  （2026-10-06 ~ 10-21 的理想站姿是 10 號 0、11 號 1792、12 號 ±180°：站姿第 3、5 軸 ±180°、手肘彎 22.5°。
    改成全 2048 後第 3、5 軸差 180°，手肘的正負也跟著反過來，所以 11 號的 DIR 翻轉，實機的方向不變；
    唯一的差別是站姿的手肘從彎 22.5° 變成打直）
左手：馬達 ID 1~7（沿用原 AD 左手），2026-10-28 用 jog_motor.py check left 實機確認過：ID 都對，
  左右手馬達鏡像安裝，7 顆方向都跟右手相反（URDF 左手是鏡像，同樣的角度 = 鏡像的姿勢）。
  站姿刻度同右手（全 2048）。
  以後重新對方向：jog_motor.py check left，改 ARMS["left"]，網頁 RobotViewer.template.html 也要一起改。
"""
import configparser
import os

import numpy as np

TPR = 4096.0 / (2 * np.pi)           # 每弧度幾個刻度
CENTER = 2048
CABLE_TICKS = (0, 4095)              # 單圈位置模式的刻度範圍（線材量過後可以改窄）
MAX_TRAVEL = np.pi                   # 任一軸離站姿最多轉多少（線材保護）

# 掌心在手掌座標（*h_hand_base_link）的位置；+X = 掌心朝向、+Z = 手指方向。左手是鏡像，y = 0 所以一樣
# 實機上 = 手掌面上、左右置中、離手掌底部（接手腕那一端）7 cm（手掌座標 x = 0.02 就是手掌面）
TOOL_OFFSET = (0.02, 0.0, 0.07)

# 實機量測座標（cm），拿捲尺量實機用；網頁下方工具列的「前 / 右 / 高」也是這一套
#   前 = 離兩根立柱的前面、右 = 離兩根立柱中間往右（左邊是負的）、高 = 離地（支架底座的底面）
# 原點在 base_link 的位置，從 stand.STL 量的：立柱前面 x = −1.35 cm、地板 z = −120.2 cm
# （2026-11-02 換 v2 URDF：手臂實際裝得比舊模型往前 3.5 cm，base_link 跟著手臂，所以支架往後移 3.5 cm；之前是 2.15 cm）
# 改了的話網頁 RobotViewer.template.html 的 MEAS_ORIGIN 也要一起改
MEAS_ORIGIN = (-0.0135, 0.0, -1.202)


def to_meas(p):
    """base_link 座標（m）→ 實機量測座標 (前, 右, 高)（cm）"""
    return ((p[0] - MEAS_ORIGIN[0]) * 100, -(p[1] - MEAS_ORIGIN[1]) * 100, (p[2] - MEAS_ORIGIN[2]) * 100)

# 站姿（肩 → 腕 7 軸，rad）= URDF 零位 = 實機全 2048
STAND = np.zeros(7)
JOINT_SUFFIX = ["shoulderJoint", "armJoint", "arm1Joint", "forarmJoint", "forarm1Joint", "wristJoint", "wrist1Joint"]
LABELS = ["肩", "肩", "上臂滾轉", "手肘", "前臂滾轉", "手腕", "手腕"]

# Linker Hand O6 握住時的主動關節角度（rad）；dip、thumb_ip 是 mimic，網頁會自動跟
HAND_CLOSE = {"thumb_cmc_yaw": 1.0, "thumb_cmc_pitch": 0.5, "index_mcp_pitch": 1.2,
              "middle_mcp_pitch": 1.2, "ring_mcp_pitch": 1.2, "pinky_mcp_pitch": 1.2}

# 方向：原本沿用舊 AD，2026-10-20/21 實機一次轉一顆確認後，8、9、12 號翻轉（12 號是機構的關係：轉盤帶動 vs 馬達本體轉）；
# 理想站姿改全 2048 時 11 號跟著翻轉（見上面）。網頁 RobotViewer.template.html 的 DIR、STAND_GP 也要一起改
_R_DIR = [1, -1, -1, -1, -1, -1, -1]
# 理想站姿刻度（同網頁 STAND_GP）：全部 2048 = 站姿就是 URDF 零位。改了的話 STAND、MoveIt 的站姿
# （initial_positions.yaml、SRDF 的 *_stand）、joint_limits.yaml 都要跟著改
_R_STAND_GP = [2048] * 7
# 左手（1~7）：左右手馬達鏡像安裝，每一顆都跟右手相反。URDF 左手是鏡像（同樣的角度 = 鏡像的姿勢），
# 實體馬達 +刻度 的轉向鏡像過來剛好反向，所以 DIR 全部 × −1。2026-10-28 jog_motor.py check left 實機確認
# （+100 刻度：1 往後、2 往內、3 掌心往後轉、4 往前、5 掌心往後轉、6 往前、7 往內，7 顆都跟這組 DIR 的模型一樣）。
# 分開寫，改其中一隻不會動到另一隻
_L_DIR = [-1, 1, 1, 1, 1, 1, 1]
_L_STAND_GP = [2048] * 7

# nstc 的 stand.ini（motionpackage 存的同一個檔）；nstc 不在家目錄時用環境變數 STAND_INI 指定
STAND_INI = os.environ.get("STAND_INI", os.path.expanduser("~/nstc/config/Parameter/stand.ini"))


def _read_stand_ini(path):
    """stand.ini 的 motionstate 3 → {馬達 ID: 刻度}（同網頁 loadStandIni，只取 1~14 手臂）；讀不到回傳 None"""
    ini = configparser.ConfigParser(interpolation=None, strict=False)
    try:
        if not ini.read(path, encoding="utf-8"):
            return None
        for sec in ini.sections():
            if ini[sec].get("motionstate", "").strip() == "3":
                data = [int(v) for v in ini[sec]["motordata"].split(",")]
                return {m: data[m - 1] for m in range(1, 15)}
    except (configparser.Error, KeyError, ValueError, IndexError):
        pass
    return None


STAND_REAL = _read_stand_ini(STAND_INI)     # None = 讀不到：先當 2048，實機橋接不會開
ARMS = {
    "right": {"label": "右手", "group": "right_arm", "tip": "rh_hand_base_link", "prefix": "R", "hand": "rh_",
              "motors": [8, 9, 10, 11, 12, 13, 14], "dir": _R_DIR, "stand_gp": _R_STAND_GP,
              "side": -1, "calibrated": True},
    "left": {"label": "左手", "group": "left_arm", "tip": "lh_hand_base_link", "prefix": "L", "hand": "lh_",
             "motors": [1, 2, 3, 4, 5, 6, 7], "dir": _L_DIR, "stand_gp": _L_STAND_GP,
             "side": +1, "calibrated": True},
}
for _a in ARMS.values():
    _a["joints"] = [_a["prefix"] + s for s in JOINT_SUFFIX]
    _a["fingers"] = {_a["hand"] + k: v for k, v in HAND_CLOSE.items()}
    _a["stand_real"] = [(STAND_REAL or {}).get(m, CENTER) for m in _a["motors"]]
    _a["offset"] = [r - g for r, g in zip(_a["stand_real"], _a["stand_gp"])]      # 實機站姿 − 理想站姿


def ticks_to_rad(arm, ticks):
    """ticks：7 個刻度（肩 → 腕）→ 弧度（同網頁 ticksToRad：先減掉 stand.ini 跟理想站姿的差）"""
    a = ARMS[arm]
    return np.array([d * (t - off - CENTER) / TPR for d, t, off in zip(a["dir"], ticks, a["offset"])])


def rad_to_ticks(arm, q):
    """回傳 {馬達 ID: 刻度}（夾在 0~4095）"""
    a = ARMS[arm]
    return {m: int(np.clip(round(CENTER + off + d * v * TPR), 0, 4095))
            for m, d, v, off in zip(a["motors"], a["dir"], q, a["offset"])}


def tick_range(arm):
    """CABLE_TICKS 對應的每軸角度範圍 (lo, hi)"""
    q_a, q_b = ticks_to_rad(arm, [CABLE_TICKS[0]] * 7), ticks_to_rad(arm, [CABLE_TICKS[1]] * 7)
    return np.minimum(q_a, q_b), np.maximum(q_a, q_b)


# 抓取時手掌靠近物體的方向（掌心朝這個方向推過去），base_link 座標：X 前、Y 左、Z 上。
# 「外側」：右手從右邊（−Y 那側往 +Y 推）、左手從左邊（往 −Y 推）
_APPROACH_BASE = {
    "front": ((1.0, 0, 0), "從前方"),
    "top": ((0, 0, -1.0), "從上方"),
    "side": ((0, 1.0, 0), "從外側"),
    "front_down": ((1.0, 0, -1.0), "斜前上方"),
    "side_down": ((0, 1.0, -1.0), "斜外上方"),
    "front_side": ((1.0, 1.0, 0), "斜外前方"),
}


def approaches(arm):
    """{名稱: (單位方向, 中文)}；左手把 Y 反過來"""
    flip = np.array([1.0, -ARMS[arm]["side"], 1.0])          # 右手 side=-1 → Y 不變；左手 → Y 反向
    out = {}
    for name, (d, label) in _APPROACH_BASE.items():
        v = np.array(d) * flip
        out[name] = (v / np.linalg.norm(v), label.replace("外", "右" if arm == "right" else "左"))
    return out
