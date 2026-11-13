#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LinkerHand O6 的規格常數，全部集中在這裡，節點和網頁都以這份為準。

數字的來源都標在註解裡，不要憑印象改。實機量測是 2026-10 在 172.17.121.11
的雙 O6（嵌入式版本 [3,1,3]、touch_type 4）上做的。
"""

HANDS = ('left', 'right')
NUM_JOINTS = 6              # O6 單手軸數
NUM_JOINTS_BOTH = 12        # 動作檔的 motordata 欄數：L1..L6, R1..R6

# ----------------------------------------------------------------------------
# 關節順序
# ----------------------------------------------------------------------------
# 以 driver 的 get_finger_order() 為準（linker_hand_o6_can.py:504），
# 它也等於 /cb_*_hand_state 的 name 欄位。
# 注意 gui_control/config/constants.py 的 O6 joint_names_en 把 index 4/5 寫反了
# （寫成 pinky 在前），不要照抄那個。
JOINT_NAMES = [
    'thumb_cmc_pitch',      # 0 大拇指彎曲
    'thumb_cmc_yaw',        # 1 大拇指橫擺
    'index_mcp_pitch',      # 2 食指彎曲
    'middle_mcp_pitch',     # 3 中指彎曲
    'ring_mcp_pitch',       # 4 無名指彎曲
    'pinky_mcp_pitch',      # 5 小拇指彎曲
]
JOINT_NAMES_ZH = ['大拇指彎曲', '大拇指橫擺', '食指彎曲', '中指彎曲', '無名指彎曲', '小拇指彎曲']

# ----------------------------------------------------------------------------
# 數值範圍
# ----------------------------------------------------------------------------
POS_MIN, POS_MAX = 0, 255

# 速度走 /cb_{side}_hand_control_cmd 的 velocity 欄位 → set_joint_speed()。
# linker_hand_api.py:184 會把「任一個值 <10 或 >255」的整包速度指令丟掉，
# 而 linker_hand.py:186 把「6 個值全為 0」當成「這次不改速度」。
SPEED_MIN, SPEED_MAX = 10, 255
SPEED_NOCHANGE = 0

# 扭矩上限走 /cb_{side}_hand_setting_cmd 的 set_max_torque_limits。
TORQUE_MIN, TORQUE_MAX = 0, 255

# 動作檔 AbsolutePosition 欄位的哨符：-1 = 這一軸不動。
# 一定要在送出前換成該軸目前的實際位置 —— linker_hand_api.py:128 的 finger_move()
# 會檢查 0 <= v <= 255，含 -1 的整包指令會被它整包丟掉。
POS_HOLD = -1

# 0~255 對應的弧度上限（LinkerHand/utils/mapping.py，O6 左右手相同）
ARC_MIN = [0.0] * NUM_JOINTS
ARC_MAX = [0.58, 1.36, 1.6, 1.6, 1.6, 1.6]

# ----------------------------------------------------------------------------
# 壓感
# ----------------------------------------------------------------------------
# /cb_{side}_hand_matrix_touch_mass 的 key。只有五指，沒有掌心
# （實機 palm_matrix 回 [-1]，palm_mass 回 list [-1] 而不是數字，要擋掉）。
MASS_KEYS = ['thumb_mass', 'index_mass', 'middle_mass', 'ring_mass', 'little_mass']
FINGER_NAMES = ['thumb', 'index', 'middle', 'ring', 'little']

# mass 是 10x4=40 格 0~255 的總和，所以上限 10200。
# JSON 裡的 "unit": "g" 是上游標錯的，它不是公克。
#
# 實測（2026-10-13，.11 上兩隻 O6，各取 215 筆）：
#   空載時 40 格「全部都是 0」，五指都一樣 —— 沒有雜訊底。
#   所以讀到任何非零值就是真的碰到東西了，門檻不需要為雜訊留餘裕。
#   換算：門檻 800 = 平均每格 20，或 8 格各 100 —— 那是很紮實的一握。
#   參考值：輕觸 50~150、輕握 200~400、紮實 600~1000、夾死 2000 以上。
# palm_matrix 回 [-1]（這一版沒有掌心壓感），所以只有五指。
MASS_MAX = 255 * 40

# 每一軸對應哪一根手指的壓力值（拇指兩軸共用 thumb）。
# 夾持的 each 模式因此是「拇指兩軸一起凍結」。
JOINT_TO_FINGER = [0, 0, 1, 2, 3, 4]

# 實機矩陣尺寸（10 列 x 4 行，touch_type 4 / SN 第 5 段 F）。
# 網頁的熱力圖一律按收到的 JSON 動態決定行列，這裡只當預設值用。
MATRIX_ROWS, MATRIX_COLS = 10, 4

# ----------------------------------------------------------------------------
# 雙手同步
# ----------------------------------------------------------------------------
# out = a * v + b，逐軸套用。實機確認左右手數值可以完全一樣，所以預設恆等轉換。
# 哪天發現某一軸要反，只改這裡那一組（例如橫擺要反就寫 (-1.0, 255.0)）。
MIRROR_GAIN_OFFSET = [
    (1.0, 0.0),   # 0 thumb_cmc_pitch
    (1.0, 0.0),   # 1 thumb_cmc_yaw
    (1.0, 0.0),   # 2 index_mcp_pitch
    (1.0, 0.0),   # 3 middle_mcp_pitch
    (1.0, 0.0),   # 4 ring_mcp_pitch
    (1.0, 0.0),   # 5 pinky_mcp_pitch
]

# ----------------------------------------------------------------------------
# 預設動作
# ----------------------------------------------------------------------------
# 預設動作（張開 / 握拳 / OK …）在網頁那邊：
#   hurocup_interface/js/hand_presets.js
# 放那邊的理由是改完重新載入瀏覽器就生效，不用重啟節點；放在這個檔案的話每次
# 調個數值都要重啟，而且很容易變成節點一份、網頁一份互相不一致。
# 網頁直接送 {cmd:'manual', hand, position:[6], speed:[6]}，節點不必知道名字。
# 原始數值出處是 gui_control/config/constants.py 的 O6 preset_actions
# （上游左右手共用同一組，不需要鏡像轉換），速度是我們自己加的。

# SDK 啟動時自己會下的初始姿態（linker_hand.py:129），列出來備查，我們不用它
# —— Q-C 決定不做共用初始姿態檔，要張開就用網頁的預設動作。
SDK_INIT_POSE = [200, 255, 255, 255, 255, 180]
# SDK 啟動時下的初始速度（linker_hand.py:132），當作速度狀態的起點。
# 這個很重要：set_joint_speed 要求六個值全部有效，不能只送要改的那幾軸，
# 所以 handpackage 必須自己記住一份完整的速度。
SDK_INIT_SPEED = [200, 250, 250, 250, 250, 250]

# ----------------------------------------------------------------------------
# 迴路頻率
# ----------------------------------------------------------------------------
CONTROL_HZ = 30.0   # /cb_*_hand_control_cmd 的上限就是 30Hz（README 明寫）
STATUS_HZ = 20.0    # /hand/status 給網頁用
STATE_TIMEOUT = 1.0 # 超過這麼久沒收到 state 就當那隻手離線

# 夾持迴路遞進步長的下限。SDK 的 hand_control_cb/list_check 在前一筆指令還沒被
# run() 消化掉時，會丟掉「任一軸變化 <3」的新指令，所以步長別小於 3。
GRIP_MIN_STEP = 3

# 壓力值超過這個秒數沒更新就當斷線。壓感 topic 實機約 55 Hz，0.5 秒很寬鬆了。
GRIP_MASS_TIMEOUT = 0.5

# 按下開始後，等壓力資料的上限。等不到就中止 —— 不能閉著眼睛夾。
# （訂閱壓感到第一筆資料進來要經過 DDS discovery，不是立即的）
GRIP_WAIT_TOUCH_TIMEOUT = 2.0

# 馬達溫度保護：任一軸到這個溫度就中止夾持。
# 實機靜止時量到 36~42°C，持續施力會上升，所以留了不少餘裕。
GRIP_MAX_TEMP = 65

# ----------------------------------------------------------------------------
# 動作檔
# ----------------------------------------------------------------------------
# 位置：src/strategy/strategy/<策略>/Parameter/hand/<檔名>.ini
#       src/strategy/strategy/<策略>/Parameter/hand/sector/<id>.ini
# 結構比照全身（motionpackage），只是 motordata 從 27 欄變 12 欄。
MOTION_DIR = 'hand'
SECTOR_DIR = 'sector'

MOTIONSTATE_MOTIONLIST = 0
MOTIONSTATE_REL_POS = 1
MOTIONSTATE_REL_SPD = 2
MOTIONSTATE_ABS_POS = 3
MOTIONSTATE_ABS_SPD = 4

# MotionList 的 (動作ID, delay_ms) 對數，與全身一致（opcode 244 的 payload 40 個值）
MOTIONLIST_PAIRS = 20

# 網頁 → handpackage 的位元封包協定，沿用全身那套（nstc_msgs/InterfaceSend2Sector）：
#   [83, 84, opcode, <payload>, 78, 69]
# 手的 payload 是 speed/pos 交錯 12 對 = 24 個值，所以長度比全身短。
PKT_HEAD = (83, 84)
PKT_TAIL = (78, 69)
OPCODE_ABS = 241        # Absolute，payload 24 → 全長 29
OPCODE_REL = 243        # Relative，payload 24 → 全長 29
OPCODE_MOTIONLIST = 244 # MotionList，payload 40 → 全長 45（與全身相同）
PKT_LEN = {
    OPCODE_ABS: 2 + 1 + NUM_JOINTS_BOTH * 2 + 2,
    OPCODE_REL: 2 + 1 + NUM_JOINTS_BOTH * 2 + 2,
    OPCODE_MOTIONLIST: 2 + 1 + MOTIONLIST_PAIRS * 2 + 2,
}
# 全身的 opcode 242（Absolute + Lockedstand）是「全身起身姿態」專屬，手沒有對應物。

# item_name 夾帶停止條件的編碼：<顯示名稱>|stop=<模式>:<門檻>
#   each    每指各自到門檻各自凍結（拇指兩軸一起）
#   any     同一隻手任一指到門檻 → 該手全部凍結
#   anyhand 任一隻手任一指到門檻 → 兩手全部凍結
#   hold    到門檻後維持（小幅修正）
# 不帶 |stop= 就是沒有停止條件。顯示名稱裡不准出現 "|"（網頁會擋）。
ITEM_NAME_SEP = '|'
STOP_MODES = ('each', 'any', 'anyhand', 'hold')

# sector 檔裡的停止條件：  stop = each:800
# 網頁 Send 過來時沒有字串欄位可用（封包是 int32[]），所以借用
# nstc_msgs/InterfaceSend2Sector 本來就沒在用的 delay / cnt 兩個欄位傳：
#     cnt   = 1 代表「這一筆帶了停止條件」
#     delay = [模式編號, 門檻]
# 跟封包在同一筆訊息裡，不會有兩個 topic 先後到達的問題。
STOP_MODE_IDS = {1: 'each', 2: 'any', 3: 'anyhand', 4: 'hold'}
STOP_MODE_NUM = {v: k for k, v in STOP_MODE_IDS.items()}


def parse_stop(text):
    """'each:800:4' -> ('each', 800, 4)。沒給步長就用預設。看不懂回 None。"""
    if not text:
        return None
    parts = str(text).split(':')
    if len(parts) < 2 or parts[0] not in STOP_MODES:
        return None
    try:
        th = int(parts[1])
        step = int(parts[2]) if len(parts) > 2 else GRIP_MIN_STEP + 1
        return (parts[0], th, max(GRIP_MIN_STEP, step))
    except ValueError:
        return None


def format_stop(stop):
    if not stop:
        return ''
    step = stop[2] if len(stop) > 2 else GRIP_MIN_STEP + 1
    return f'{stop[0]}:{stop[1]}:{step}'


# 停止條件在編輯檔裡是放在 pose 列的 motionlist 欄（那一欄對姿態列本來就是空的），
# 固定三個值 [模式編號, 門檻, 步長]，沒有條件就是 [0, 0, 0]。
# 這樣就不用把它塞進 item_name —— 名字要留給人取名用。
STOP_TRIPLE_LEN = 3
