#!/usr/bin/env python3
"""
雙手 + 支架：把掌心移到指定的位置 + 姿態（網頁 RobotViewer.html 的「移動」，或自己的程式發 topic），MoveIt 檢查碰撞、執行

2026-11-12 起只有一種動作「移動」（使用者決定：知道物體位置後，手臂把掌心送到那裡、姿態可以控制，越準越好）。
以前的「抓取」「對準」「放開」、自動夾持實機手掌都拿掉了；手掌用 handpackage 另外控制。

網頁的「移動」：
  /arm_move         std_msgs/String（JSON）{"pos": [前, 右, 高]（cm，實機量測座標）, "dir": 掌心方向, "fingers": 手指朝向（度，
                    null = 自動）, "arm": auto / right / left}
                    掌心方向 = auto（先從上方，不行再試其他方向）或 arm_config.approaches 的 top（從上方）、front（從前方）、
                    side（從外側）、front_down、side_down、front_side。手指朝向每個方向的 0° 和正負見 FINGER_HINT
                    （從上方：0 = 手指朝前、+ 往外、− 往內）。手指轉不到指定的角度時，同一個掌心方向改用最接近的角度
                    （_closest_fingers，2026-11-13 起；不換掌心方向）。到不了時說原因：手指能轉到哪些角度、這個位置哪些方向可以
之後整合用（API，網頁沒用到）：
  /arm_ik_target    geometry_msgs/PoseStamped，frame_id = base_link（m）：掌心要到的位置 + 任意手掌姿態
                    手掌座標：+X = 掌心朝向、+Z = 手指方向。姿態全 0（quaternion 0,0,0,1）= 掌心朝前、手指朝上；
                    掌心朝下、手指朝前 = roll 0、pitch 90°、yaw 0（rpy 同 URDF：Rz(yaw)·Ry(pitch)·Rx(roll)）
  /arm_ik_mode      std_msgs/String：full（位置 + 姿態，預設）/ pos_only（只對位置，姿態讓 IK 挑）
  /arm_ik_arm       std_msgs/String：auto（肩膀離目標近的先試，不行換另一隻，預設）/ right / left
  /arm_speed_cmd    std_msgs/Float64：速度（Dynamixel 單位，1 = 0.229 rpm）
  /arm_stand        std_msgs/Bool：回站姿（兩隻手）
  /arm_ik_obstacles std_msgs/String：場景物體，會避開。{"source": "sp3", "objects": [...]}，或舊格式（只有清單）= 網頁（source web）。
                    依來源分開存、合併後給 MoveIt：同一來源每次送整份（空清單 = 清空這個來源），不會蓋掉別的來源。
                    物體格式見 to_collision_object；MoveIt 裡的 id = html_<來源>_<id>（id 本來就以「來源_」開頭就不重複加）
  /arm_attach       std_msgs/String：夾起來的物體掛到手上 / 拿掉（格式見 _attach_cb），結果 cmd = attach / detach
  /arm_stop         std_msgs/Bool：true = 取消執行中的軌跡，這個動作不再執行下一段（結果 reason = canceled）
  結果 /arm_ik_status std_msgs/String（JSON）：{"stage_only": true, "stage": "..."} = 進行中；
        其他 = 做完或失敗：{"ok": true / false, "note": 說明, "q_deg": 關節角, "ticks": 馬達刻度,
                            "cmd": move / stand / attach / detach（None = 不是指令的結果，例如停住後的「MoveIt 目前姿勢」）,
                            "arm": 實際用的那隻手（兩隻都不行、回站姿 = null）,
                            "reason": 失敗分類（成功 = null）：bad_request / busy / unreachable / joint_limit / collision /
                                      plan_failed / exec_failed / canceled / moveit_down / internal（見 R_*）}
        移動成功時 note 第二行 = 這次掌心的姿態（R / P / Y 和 quaternion，pose_text）：網頁用「掌心方向 + 手指」移好，
        之後自己的程式要同一個姿態就把這組填進 /arm_ik_target
  base_link ↔ 實機量測座標（前 / 右 / 高，cm）：前 = x·100 + 1.35、右 = −y·100、高 = z·100 + 120.2（arm_config.to_meas）

「移動」怎麼做（_do_move）：
  1. 實機跟隨開著時，目標往上補 REAL_Z_COMP × 前（實機掌心比模擬低約 0.17 × 前，見 position_test_report.md）
  2. 自己的 IK（numpy）解目標：從很多起點解出好幾組姿勢，挑「離站姿近、離關節極限遠」的（_pose_cost），
     不管上一個動作停在哪，同一個目標都挑差不多的姿勢
  3. 檢查會不會撞到東西；高度補償時，實機那個比較低的位置也檢查（場景物體暫時往上移）
  4. 從目標沿掌心反方向退 PRE_DIST = 預備點；預備點 → 目標走直線（姿態不變），越接近越慢、到目標停
  5. 目前姿勢 → 預備點：每顆馬達直接轉過去（不繞路，線扭最少）；會撞到就先沿直線往正上方抬過物體、再直接轉過去
     （_via_path，2026-11-18）；
     還是不行才請 MoveIt 規劃（繞遠路、路線會撞到就重新規劃；Humble 的 MoveIt 不檢查路線，這裡逐點查）。
     結果的 note 會寫「路線：…」是哪一種。回站姿也一樣：直接轉 → 往後退開 → 從物體上方繞回 → MoveIt
  微調（目標離目前掌心不到 PRE_DIST、姿態差不到 SMALL_ROT）：不退到預備點，從目前姿勢直接轉過去
MoveIt → 這裡 → 網頁：
  /joint_states → /ik_viewer/joint_state（橘色雙手）、/ik_viewer/target（洋紅色球）
在 RViz 自己拖末端 Plan / Execute 也可以，網頁一樣會跟著動、一樣會檢查

檢查的是實機才有、MoveIt 不知道的限制（有警告就不執行）：
  - 換成馬達刻度會不會超出 CABLE_TICKS；任一軸離站姿超過 MAX_TRAVEL（180°，線材保護）
  - 超出 URDF ∩ joint_limits.yaml 的範圍（real_arm_bridge 送給實機前也用這個 check）
  - 還沒對過實機的手（arm_config.py 的 calibrated = False），面板會標「刻度未校正」

通常用 launch 開：ros2 launch armpackage sim.launch.py
"""
import json
import os
import threading
import time
from collections import Counter

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import String, Bool, Float64
from shape_msgs.msg import SolidPrimitive
import xml.etree.ElementTree as ET
from moveit_msgs.msg import (AttachedCollisionObject, DisplayTrajectory, CollisionObject, PlanningScene, Constraints,
                             JointConstraint, RobotTrajectory)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import yaml
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from moveit_msgs.action import MoveGroup, ExecuteTrajectory

from arm_config import (ARMS, STAND, LABELS, TOOL_OFFSET, MAX_TRAVEL, MEAS_ORIGIN, rad_to_ticks, tick_range, to_meas,
                        approaches)
from kinematics import ChainFK, ChainIK, chain_limits, quat_to_R, R_to_quat

BASE = "base_link"
RATE = 30.0          # 網頁更新頻率（Hz）
SETTLE = 0.3         # 手臂停住多久（s）才報一次目前姿勢的檢查結果
SCENE_RESEND = 5.0   # 每幾秒把網頁的物體整批重送給 MoveIt（move_group 晚開或重開也會補上）
SCENE_READY = 5.0    # 每個動作開始前，場景物體還沒套用到 MoveIt 時最多等幾秒（_ensure_scene），還不行就不動
FLOOR_SIZE = 3.0     # 地板在 MoveIt 裡用多大的薄板代表（m）
PLAN_TIME = 3.0      # MoveIt 每次規劃最多想幾秒
MAX_VEL = 2.3824     # joint_limits.yaml 的 max_velocity，網頁速度換成倍率用
RAD_PER_SEC_PER_SPEED = 0.229 * 2 * np.pi / 60
SLOW = 0.3           # 最後直線靠近用的速度比例
DECEL_DIST = 0.05    # 直線靠近的最後幾公尺開始減速（速度 ∝ √剩下的距離，到目標 = 0）
MAX_ACC = 1.0        # joint_limits.yaml 的 max_acceleration（rad/s²），乘上速度倍率（同 MoveIt 的 acceleration scaling）
TABLE_SIZE = 20000   # IK 起點表：每隻手事先擺這麼多個隨機姿勢，記下掌心位置和朝向
IK_ITERS = 100       # IK 疊代上限（解得出來的通常 20~40 次就收斂；固定整個姿態時給兩倍）
N_SEED = 32          # IK 除了目前姿勢、站姿以外，再從起點表挑幾個「掌心已經很接近目標」的當起點
N_SOLS = 16          # 每次最多找幾組姿勢（找到的越多，_pose_cost 越挑得到像站姿的那組；8 組時常常第一次沒找到最好的）
MAX_FULL = 4         # 找到幾組「直線靠近也可行」的就不再找
MAX_ROUTE_TRIES = 5  # 最多找幾個候選的「移到預備點」的路
REACH = 0.62         # 目標離肩膀超過這麼遠（m）就不用試這隻手
PRE_DIST = 0.10      # 預備點離目標多遠（沿掌心反方向）
# 先往上抬過物體再過去（_via_path，2026-11-18）：直接轉會撞到時，在 MoveIt 之前先試。
# 掌心沿直線往正上方抬到「路線下方物體的最高點 + VIA_CLEARS」，再每顆馬達直接轉到目標。
# 不能整段都用直接轉：從站姿（手掛在桌面下、桌緣後）一開始轉，手指會先往前掃進桌子（2026-11-18 本機：走到 8～20% 拇指就碰到）
VIA_CLEARS = (0.10, 0.18)       # 抬到掌心比物體最高點高多少（m）；手掌朝下時手最低處在掌心下 4.5 cm，手指朝下約 11 cm
VIA_CORRIDOR = 0.15             # 起點、終點掌心的水平範圍往外多少（m）以內的物體算「路線下方」
VIA_TRIES = 2                   # 移到預備點時，最好的幾個目標姿勢試這個方法
SMALL_ROT = np.radians(30)      # 微調：目標離目前掌心不到 PRE_DIST、姿態差不到這麼多，就不退到預備點，直接轉過去
SMALL_JOINT = np.radians(45)    # 微調時任一軸要轉超過這麼多，就當作不是微調（走一般流程）
REAL_Z_COMP = 0.17   # 實機跟隨開著時，目標往上補「前 × 這個值」（前 = 離立柱前面多遠，前 50 cm → +8.5 cm）。
                     # 實機掌心比模擬低約 0.17 × 前（2026-11-11 右手 11 組，見 position_test_report.md）。0 = 不補
HOLD_DIST = 0.05     # 直線靠近整段固定姿態轉不過去（預備點比較高，同一個姿態手肘到極限）時，只固定目標前這麼遠（m）
POSE_W_NOW = 0.3     # 挑姿勢的成本 = 離站姿轉多少 + 這個 × 離目前姿勢轉多少 + 靠近關節範圍兩端的扣分（_pose_cost）
LIMIT_MARGIN = np.radians(15)   # 離關節範圍兩端不到這麼多就扣分（極限邊上：實機容易卡住、線拉緊）
LIMIT_PENALTY = 4.0  # 每差 1 rad 扣多少（跟轉動量同單位）
DIRECT_STEP = np.radians(1.5)   # 「每顆馬達直接轉過去」每 1.5° 檢查一次碰撞
SWING_TOL = np.radians(20)      # MoveIt 的路任一軸轉到起點、終點之間的範圍外超過這麼多 = 繞遠路，不用
SWING_TOL_STAND = np.radians(90)  # 回站姿用 MoveIt 時放寬：手在物體正上方時要繞開（往下會掃到物體），MoveIt 的路常多轉 40~75°；
                                  # 終點是站姿（線轉回原位），繞一點可以接受
ROUTE_PLANS = 3      # MoveIt 繞遠路時重新規劃幾次（每次規劃出來的路不一樣）
RANGE_TOL = 0.02     # check()：超出關節範圍這麼多（rad，≈1°）以內不算
REAL_STALE = 3.0     # /arm_real_status 幾秒沒更新就當實機跟隨沒開（real_arm_bridge 每秒送一次）
INSIDE_TOL = 0.003   # 掌心目標離物體表面不到這麼多（m，含在裡面）就直接說「在物體裡面」，不用規劃
SKIP_FINGER = np.radians(45)    # 指定手指朝向時：只要求掌心方向的解，手指離指定的都超過這麼多 → 這個方向直接當到不了（不再慢慢解）
UP = np.array([0.0, 0.0, 1.0])
CART_STEP = 0.01     # 直線段每幾公尺解一次 IK
MAX_JUMP = 0.35      # 直線段相鄰兩點任一軸轉超過這麼多（rad）就當作姿勢跳了
JOINT_WEIGHTS = np.array([3.0, 3.0, 2.0, 2.0, 1.0, 1.0, 1.0])   # 比較轉動量時肩膀權重大
STAND_MOVEIT = np.clip(STAND, -3.14, 3.14)                     # 站姿夾進 URDF 的 ±3.14（現在站姿全 0，不會被夾）
ERR = {-1: "規劃失敗（找不到路，或目標到不了）", -2: "規劃結果無效", -4: "控制器執行失敗", -6: "逾時",
       -7: "被取消（/arm_stop）",
       -10: "起點姿勢已經撞到東西", -12: "目標姿勢會撞到東西", -13: "MoveIt 規劃的路線會撞到東西",
       -31: "IK 解不到（到不了，或在關節範圍內做不到）",
       99999: "MoveIt 失敗",
       -9999: "連不到 move_group（MoveIt 有開嗎？）", -9998: "MoveIt 沒接受請求"}

# 失敗原因：結果（/arm_ik_status）的 reason 欄位，固定的英文代碼給程式分類（任務管理對應 S3-05 錯誤碼），note 給人看
R_BAD = "bad_request"      # 指令格式不對、不認得的方向或手、frame 不支援（沒有動）
R_BUSY = "busy"            # 上一個動作還沒做完（沒有動）
R_UNREACH = "unreachable"  # IK 解不到、離肩膀太遠：這個位置／方向到不了
R_LIMIT = "joint_limit"    # 超出關節或線材範圍（check：CABLE_TICKS、MAX_TRAVEL、URDF ∩ joint_limits），直線靠近途中到極限或翻轉
R_COLL = "collision"       # 目標、直線靠近、路徑、站姿會撞到（含目標在物體裡面、起點就碰到東西）
R_PLAN = "plan_failed"     # MoveIt 規劃失敗、只找到繞遠路的路
R_EXEC = "exec_failed"     # MoveIt 沒接受執行、執行失敗
R_CANCEL = "canceled"      # 被 /arm_stop 取消
R_DOWN = "moveit_down"     # 連不到 move_group 或它的服務
R_INTERNAL = "internal"    # 這支程式出錯（例外）
# 自動試了好幾隻手、好幾個方向，原因不一樣時報哪一個：先報換手或重試還有機會的，全部都到不了才報 unreachable
REASON_ORDER = [R_COLL, R_PLAN, R_EXEC, R_LIMIT, R_UNREACH]
LINE_REASON = {"途中碰到關節極限": R_LIMIT, "途中姿勢會突然翻轉": R_LIMIT, "途中會撞到東西": R_COLL}   # _line 的原因


def plan_reason(code):
    """MoveIt 錯誤碼 → reason"""
    return {-10: R_COLL, -12: R_COLL, -13: R_COLL, -31: R_UNREACH, -7: R_CANCEL, -9999: R_DOWN}.get(code, R_PLAN)


def err_text(code, res=None):
    """錯誤碼 → 中文。_plan 回傳的 res 是清單時列出誰碰到誰（-10 起點就撞到、-13 路線會撞到），是字串時就是說明"""
    if code == -10 and isinstance(res, list):
        return "起點姿勢已經碰到東西：" + "、".join(res) + "（把物體移開一點再試）"
    if code == -13 and isinstance(res, list):
        return "MoveIt 規劃的路線會撞到：" + "、".join(res)
    if isinstance(res, str):
        return res
    return ERR.get(code, f"MoveIt 錯誤碼 {code}")


def _dur(t):
    return Duration(sec=int(t), nanosec=int((t % 1) * 1e9))


# 場景物體依來源分開存（/arm_ik_obstacles）。舊格式（只有清單）= 網頁。合併時 id 加上來源前綴：
# 網頁的 3 → web_3（MoveIt 裡是 html_web_3）；本來就有前綴的（任務管理的 sp3_table）不重複加
WEB = "web"
FLOOR_ID = f"{WEB}_floor"            # 網頁的地板（plane）


def _obj_label(oid):
    """合併後的物體 id → 中文：web_3 → 物體 #3、web_floor → 地板、sp3_table → 物體 sp3_table"""
    if oid == FLOOR_ID:
        return "地板"
    if oid.startswith(WEB + "_"):
        return f"物體 #{oid[len(WEB) + 1:]}"
    return f"物體 {oid}"


def _contact_name(body):
    """MoveIt 的碰撞物件名稱 → 中文（html_web_3 = 網頁的物體 #3、attached_sp3_held = 掛在手上的 sp3_held）"""
    if body.startswith("html_"):
        return _obj_label(body[5:])
    if body.startswith("attached_"):
        return f"手上的 {body[9:]}"
    return body


def _pose(c, R=np.eye(3)):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in c)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in R_to_quat(R))
    return p


def _axis_frame(d, k):
    """旋轉矩陣：第 k 個軸（0 = X、2 = Z）指向方向 d，另外兩軸隨便挑垂直的"""
    a = np.asarray(d, float) / np.linalg.norm(d)
    u = np.cross(a, np.eye(3)[np.argmin(np.abs(a))])
    u /= np.linalg.norm(u)
    cols = [None] * 3
    cols[k], cols[(k + 1) % 3], cols[(k + 2) % 3] = a, u, np.cross(a, u)
    return np.column_stack(cols)


def rpy_deg(R):
    """旋轉矩陣 → (roll, pitch, yaw)（度），R = Rz(yaw)·Ry(pitch)·Rx(roll)（同 URDF、網頁的 R / P / Y）。
    pitch ±90°（掌心正朝下 / 正朝上）時 roll、yaw 分不開，roll 給 0、轉動全部算在 yaw
    （差 0.5° 內都當正朝下：實際姿勢的小誤差會讓 roll、yaw 亂跳。網頁 ikRpy 同一個算法）"""
    R = np.asarray(R, float)
    p = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    if abs(R[2, 0]) > np.cos(np.radians(0.5)):
        r, y = 0.0, np.arctan2(-R[0, 1], R[1, 1])
    else:
        r, y = np.arctan2(R[2, 1], R[2, 2]), np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([r, p, y]) + 0.0          # + 0.0：-0 → 0


def rot_angle(Ra, Rb):
    """兩個姿態差幾度"""
    return float(np.degrees(np.arccos(np.clip((np.trace(np.asarray(Ra).T @ Rb) - 1) / 2, -1.0, 1.0))))


def pose_text(R):
    """手掌姿態 → 「姿態 R 0、P 90、Y -30°（…quaternion…）」：移動完狀態列寫這行，之後自己的程式要同一個姿態就照抄
    （/arm_ik_target 的 PoseStamped，base_link）"""
    r, p, y = (int(v) for v in np.round(rpy_deg(R)))
    q = np.asarray(R_to_quat(R), float)
    q = np.round(-q if q[3] < 0 else q, 3) + 0.0          # q 和 −q 是同一個姿態，統一 w ≥ 0；+ 0.0：-0 → 0
    return f"姿態 R {r}、P {p}、Y {y}°（base_link；quaternion x {q[0]:g}、y {q[1]:g}、z {q[2]:g}、w {q[3]:g}）"


def from_meas(f, r, h):
    """實機量測座標 (前, 右, 高)（cm）→ base_link（m），to_meas 的反過來"""
    return np.array([f / 100 + MEAS_ORIGIN[0], -r / 100 + MEAS_ORIGIN[1], h / 100 + MEAS_ORIGIN[2]])


# 網頁「掌心方向」：用 arm_config.approaches 的方向（掌心朝這個方向推過去）。
# 「手指朝向」= 手掌繞掌心法線轉幾度：每個方向的 0° 和正的往哪轉（finger_ref），網頁旁邊會寫同一段說明
FINGER_HINT = {
    "top": "0 = 手指朝前、+ 往外、− 往內",
    "front": "0 = 手指朝上、+ 往外、− 往內",
    "side": "0 = 手指朝前、+ 往上、− 往下",
    "front_down": "0 = 手指朝前上、+ 往外、− 往內",
    "side_down": "0 = 手指朝前、+ 往外下、− 往內上",
    "front_side": "0 = 手指朝前外、+ 往上、− 往下",
}


FINGER_ZERO = {"top": "手指朝前", "front": "手指朝上（掌心朝前時手指沒辦法朝前）", "side": "手指朝前",
               "front_down": "手指朝前上", "side_down": "手指朝前", "front_side": "手指朝前外"}   # 手指朝向 0° 的說法


def _plane_dir(v, *normals):
    """v 去掉沿 normals（單位向量）的分量後的單位向量；剩太少（< 0.3）回傳 None"""
    v = np.asarray(v, float).copy()
    for nrm in normals:
        v -= nrm * (nrm @ v)
    nv = np.linalg.norm(v)
    return v / nv if nv > 0.3 else None


def finger_ref(arm, a):
    """掌心朝 a 時，手指朝向的基準：(a, 0° 的手指方向, 正的轉向)。
    0° = 手指盡量朝前（掌心朝前時改成朝上）；正的 = 往外（右手往右、左手往左），往外不在手掌面上（掌心朝外側）時改成往上"""
    a = np.asarray(a, float) / np.linalg.norm(a)
    up = np.array([0.0, 0.0, 1.0])
    z0 = _plane_dir([1.0, 0.0, 0.0], a)
    if z0 is None:
        z0 = _plane_dir(up, a)
    w = _plane_dir([0.0, ARMS[arm]["side"], 0.0], a, z0)
    if w is None:
        w = _plane_dir(up, a, z0)
    return a, z0, w


def palm_R_fingers(arm, a, deg):
    """掌心朝 a、手指朝向 deg 度（finger_ref 的定義）的手掌姿態（+X = 掌心朝向、+Z = 手指方向）"""
    a, z0, w = finger_ref(arm, a)
    th = np.radians(deg)
    z = np.cos(th) * z0 + np.sin(th) * w
    return np.column_stack([a, np.cross(z, a), z])


def finger_angle(arm, a, R):
    """手掌姿態 R 的手指朝向（度，finger_ref 的定義）"""
    a, z0, w = finger_ref(arm, a)
    z = np.asarray(R)[:, 2]
    return float(np.degrees(np.arctan2(z @ w, z @ z0)))


def finger_text(dirname, ang):
    """手指朝向 → 說明。從上方：「手指朝前偏內 18°」；其他方向：「手指 +18°（0 = …）」"""
    ang = (ang + 180) % 360 - 180
    if dirname != "top":
        if abs(ang) < 5:                            # 鎖定朝前（0°）：直接說手指朝哪
            return FINGER_ZERO.get(dirname, "手指 0°")
        return f"手指 {ang:+.0f}°（{FINGER_HINT.get(dirname, '')}）"
    side = "外" if ang > 0 else "內"
    if abs(ang) < 5:
        return "手指朝前"
    if abs(abs(ang) - 90) < 5:
        return f"手指朝{side}"
    if abs(ang) < 90:
        return f"手指朝前偏{side} {abs(ang):.0f}°"
    return f"手指朝後偏{side} {180 - abs(ang):.0f}°"


def _segments(vals, gap=20.0):
    """角度清單 → 「-56° ~ -36°、+43° ~ +50°」（中間斷開超過 gap 度就分段）"""
    vals = sorted(vals)
    segs = [[vals[0], vals[0]]]
    for v in vals[1:]:
        if v - segs[-1][1] > gap:
            segs.append([v, v])
        else:
            segs[-1][1] = v
    return "、".join(f"{lo:+.0f}°" if hi - lo < 3 else f"{lo:+.0f}° ~ {hi:+.0f}°" for lo, hi in segs)


def _travel(path):
    """加權總轉動量（rad）"""
    path = np.asarray(path)
    return float((np.abs(np.diff(path, axis=0)) * JOINT_WEIGHTS).sum()) if len(path) > 1 else 0.0


def time_path(path, v_lim, a_max):
    """關節角清單（N×7）排時間：起點、終點速度 0，每一點不超過 v_lim[i]，加減速不超過 a_max。
    路程 s = 最快那一軸轉的角度，v_lim、a_max 也是那一軸的（rad/s、rad/s²）。
    回傳 (各點時間, 各點關節速度, keep)：跟前一點幾乎一樣的點會被拿掉（時間不能重複），
    留下的是 path[keep]，時間和速度跟它一一對應"""
    P = np.asarray(path, float)
    keep = [0]
    for i in range(1, len(P)):
        if np.abs(P[i] - P[keep[-1]]).max() > 1e-5:
            keep.append(i)
    if keep[-1] != len(P) - 1:                      # 終點一定留（換掉最後一個太近的）
        keep[-1] = len(P) - 1
    P, v = P[keep], np.minimum(np.asarray(v_lim, float)[keep], 1e9)
    if len(P) < 2:
        return np.full(1, 0.05), np.zeros((1, P.shape[1])), keep
    ds = np.maximum(np.abs(np.diff(P, axis=0)).max(axis=1), 1e-9)
    # 轉角（例如移到預備點接直線靠近的地方）：方向變很多時要先慢下來，不然關節速度會瞬間換方向
    d = np.diff(P, axis=0) / ds[:, None]
    turn = np.abs(np.diff(d, axis=0)).max(axis=1)
    v[1:-1] = np.minimum(v[1:-1], np.sqrt(a_max * np.minimum(ds[:-1], ds[1:]) / np.maximum(turn, 1e-9)))
    v[0] = v[-1] = 0.0
    for i in range(len(P) - 1):                     # 往前：從 0 加速
        v[i + 1] = min(v[i + 1], np.sqrt(v[i] ** 2 + 2 * a_max * ds[i]))
    for i in range(len(P) - 2, -1, -1):             # 往回：來得及減速到下一點的速度
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * a_max * ds[i]))
    # 每段等加速度。兩端速度都是 0 的段（整條只有 2 點：要轉的不到 DIRECT_STEP，例如手已經在目標又按一次移動）
    # 用「加速一半、減速一半」的時間；以前 2·ds / 0 → 幾萬秒，控制器一直跑、網頁等到逾時，之後 MoveIt 都不接受執行（2026-11-13）
    vv = v[:-1] + v[1:]
    dt = np.where(vv > 1e-6, 2 * ds / np.maximum(vv, 1e-9), np.maximum(2 * np.sqrt(ds / max(a_max, 1e-9)), 0.05))
    t = np.concatenate([[0.0], np.cumsum(dt)])
    s = np.concatenate([[0.0], np.cumsum(ds)])
    tangent = np.gradient(P, s, axis=0)             # dq/ds（最快那軸約 1）
    return t, tangent * v[:, None], keep


def _traj_q(traj, joints):
    """RobotTrajectory → N×7（joints 的順序）；軌跡裡沒有這些關節就回傳 None"""
    jt = traj.joint_trajectory
    if not jt.points or not all(n in jt.joint_names for n in joints):
        return None
    idx = [jt.joint_names.index(n) for n in joints]
    return np.array([[p.positions[i] for i in idx] for p in jt.points])


def shift_up(o, dz):
    """物體往上移 dz（m）的複本，給高度補償檢查實機的位置用（物體往上 = 手往下）。
    平面（地板）不移：支架底座貼著地板，移了會一直算撞到"""
    if not dz or o.get("type") == "plane":
        return o
    o = dict(o)
    for k in ("c", "a", "b"):
        if k in o:
            o[k] = [float(o[k][0]), float(o[k][1]), float(o[k][2]) + dz]
    return o


def to_collision_object(o):
    """/arm_ik_obstacles 的一個物體（base_link，m）→ MoveIt CollisionObject。格式：
    {"id", "type": "ball", "c": [x, y, z], "r"} / {"id", "type": "box", "c", "R": 3×3, "h": [半長 x, y, z]}
    / {"id", "type": "stick", "a": 端點, "b": 端點, "r"} / {"id", "type": "plane", "p": 平面上一點, "n": 法線}"""
    co = CollisionObject()
    co.header.frame_id = BASE
    co.id = f"html_{o['id']}"
    co.operation = CollisionObject.ADD          # 同 id 再 ADD 一次 = 取代（物體被拖動時）
    t = o["type"]
    if t == "ball":
        prim, pose = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[float(o["r"])]), _pose(o["c"])
    elif t == "box":
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2.0 * float(v) for v in o["h"]])
        pose = _pose(o["c"], o["R"])
    elif t == "stick":
        a, b = np.asarray(o["a"], float), np.asarray(o["b"], float)
        prim = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[float(np.linalg.norm(b - a)), float(o["r"])])
        pose = _pose((a + b) / 2, _axis_frame(b - a, 2))
    elif t == "plane":
        n = np.asarray(o["n"], float) / np.linalg.norm(o["n"])
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[FLOOR_SIZE, FLOOR_SIZE, 0.01])
        pose = _pose(np.asarray(o["p"], float) - n * 0.005, _axis_frame(n, 2))
    else:
        return None
    co.primitives, co.primitive_poses = [prim], [pose]
    return co


def surface_dist(o, p):
    """點 p（base_link，m）到網頁物體 o 表面的距離（m，負的 = 在裡面）；地板（plane）回傳 None"""
    t, p = o.get("type"), np.asarray(p, float)
    if t == "ball":
        return float(np.linalg.norm(p - np.asarray(o["c"], float)) - o["r"])
    if t == "box":
        d = np.abs(np.asarray(o["R"], float).T @ (p - np.asarray(o["c"], float))) - np.asarray(o["h"], float)
        return float(np.linalg.norm(np.maximum(d, 0.0)) + min(d.max(), 0.0))
    if t == "stick":
        a, b = np.asarray(o["a"], float), np.asarray(o["b"], float)
        s = np.clip((p - a) @ (b - a) / max((b - a) @ (b - a), 1e-12), 0.0, 1.0)
        return float(np.linalg.norm(p - (a + s * (b - a))) - o["r"])
    return None


def object_top_bottom(o):
    """網頁物體的頂面、底面高（實機量測座標，cm）"""
    if o["type"] == "stick":
        zs = [o["a"][2], o["b"][2]]
        lo, hi = min(zs) - o["r"], max(zs) + o["r"]
    else:
        c = np.asarray(o["c"], float)
        e = o["r"] if o["type"] == "ball" else float(np.abs(np.asarray(o["R"], float)[2]) @ np.asarray(o["h"], float))
        lo, hi = c[2] - e, c[2] + e
    return to_meas([0.0, 0.0, hi])[2], to_meas([0.0, 0.0, lo])[2]


_RANGE = {}


def joint_range(arm):
    """這隻手每軸能轉的範圍 (lo, hi)（rad）= URDF ∩ MoveIt 的 joint_limits.yaml：
    MoveIt 規劃、這裡的 IK、check()（也就是 real_arm_bridge 送給實機前的檢查）都用這一份"""
    if arm not in _RANGE:
        share = get_package_share_directory
        urdf = os.path.join(share("seven_axis_arm_urdf"), "urdf", "seven_axis_arm_urdf.urdf")
        lim = yaml.safe_load(open(os.path.join(share("seven_axis_arm_moveit_config"), "config", "joint_limits.yaml")))
        over = {k: (v["min_position"], v["max_position"]) for k, v in lim["joint_limits"].items() if v.get("has_position_limits")}
        _RANGE[arm] = chain_limits(urdf, ARMS[arm]["joints"], over)
    return _RANGE[arm]


def _swing(path):
    """路徑上任一軸轉到起點、終點之間的範圍外最多多遠（rad）：MoveIt 有時候會繞一大圈才到"""
    P = np.atleast_2d(path)
    lo, hi = np.minimum(P[0], P[-1]), np.maximum(P[0], P[-1])
    return float(np.max(np.maximum(P.max(axis=0) - hi, 0.0) + np.maximum(lo - P.min(axis=0), 0.0)))


def check(arm, path):
    """path：這隻手的一組角度或一整條軌跡（N×7）。回傳警告清單（空的 = 實機可以照做），每軸只報最嚴重的那一點"""
    path = np.atleast_2d(path)
    lo, hi = tick_range(arm)
    rlo, rhi = joint_range(arm)
    travel = np.abs(path - STAND).max(axis=0)
    warn, name = [], ARMS[arm]["label"]
    for i, label in enumerate(LABELS):
        if travel[i] > MAX_TRAVEL + 1e-3:
            warn.append(f"{name}{label}（第 {i + 1} 軸）離站姿最多 {np.degrees(travel[i]):.0f}°（超過 180°，可能扭到線）")
        out = path[(path[:, i] < rlo[i] - RANGE_TOL) | (path[:, i] > rhi[i] + RANGE_TOL), i]
        if out.size:
            warn.append(f"{name}{label}（第 {i + 1} 軸，{ARMS[arm]['motors'][i]} 號）會轉到 "
                        f"{np.degrees(out[np.argmax(np.abs(out))]):.0f}°，超出範圍 {np.degrees(rlo[i]):.0f}~{np.degrees(rhi[i]):.0f}°")
        out = path[(path[:, i] < lo[i] - 1e-3) | (path[:, i] > hi[i] + 1e-3), i]
        if out.size:
            warn.append(f"{name}{label}（第 {i + 1} 軸）會轉到 {np.degrees(out[np.argmax(np.abs(out))]):.0f}°，"
                        "超出馬達刻度範圍（實機轉不到）")
    return warn


class MoveItBridge(Node):
    def __init__(self):
        super().__init__("moveit_bridge")
        urdf = os.path.join(get_package_share_directory("seven_axis_arm_urdf"), "urdf", "seven_axis_arm_urdf.urdf")
        self.fk = {a: ChainFK(urdf, BASE, ARMS[a]["tip"], TOOL_OFFSET) for a in ARMS}          # 掌心 FK
        # 自己的 IK（numpy）：比 MoveIt 的 IK 快很多、能一次解很多組來挑，範圍跟 MoveIt 一致
        self.ik = {a: ChainIK(urdf, BASE, ARMS[a]["tip"], TOOL_OFFSET, *joint_range(a)) for a in ARMS}
        # 起點表：比純隨機起點容易收斂（手伸到很遠、能用的姿勢很少時差最多），兩隻手約 2 秒
        self.table = {}
        for a, ik in self.ik.items():
            Q = np.random.default_rng(1).uniform(ik.lower, ik.upper, (TABLE_SIZE, len(ik.joints)))
            T = np.array([ik(q) for q in Q])
            self.table[a] = (Q, T[:, :3, 3], T[:, :3, 0], T[:, :3, 2])   # 姿勢、掌心位置、掌心朝向（+X）、手指方向（+Z）
        self.shoulder = {a: ChainFK(urdf, BASE, ARMS[a]["prefix"] + "shoulder_Link")([0.0])[:3, 3] for a in ARMS}
        # 手臂伸直時掌心離肩膀多遠（站姿就是伸直往下），到不了時拿來說「太遠」
        self.arm_len = {a: float(np.linalg.norm(self.fk[a](STAND_MOVEIT)[:3, 3] - self.shoulder[a])) for a in ARMS}
        self.q_pub = self.create_publisher(JointState, "/ik_viewer/joint_state", 10)
        self.tgt_pub = self.create_publisher(Point, "/ik_viewer/target", 10)
        self.status_pub = self.create_publisher(String, "/arm_ik_status", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_states_cb, 10)
        self.create_subscription(DisplayTrajectory, "/display_planned_path", self._plan_cb, 10)
        self.create_subscription(String, "/arm_ik_obstacles", self._obstacles_cb, 10)
        self.create_subscription(PoseStamped, "/arm_ik_target", self._target_cb, 10)
        self.create_subscription(String, "/arm_move", self._move_cb, 10)
        self.create_subscription(String, "/arm_ik_mode", lambda m: setattr(self, "mode", m.data), 10)
        self.create_subscription(String, "/arm_ik_arm", lambda m: setattr(self, "arm_choice", m.data), 10)
        self.create_subscription(Float64, "/arm_speed_cmd", self._speed_cb, 10)
        self.create_subscription(Bool, "/arm_stand", self._stand_cb, 10)
        self.create_subscription(String, "/arm_attach", self._attach_cb, 10)
        self.create_subscription(Bool, "/arm_stop", self._stop_cb, 10)
        # 每隻手的手掌、手指 link：掛在手上的物體碰到它們不算碰撞（touch_links）
        links = [l.get("name") for l in ET.parse(urdf).getroot().findall("link")]
        self.hand_links = {a: [l for l in links if l.startswith(ARMS[a]["hand"])] for a in ARMS}
        self.real_state, self.real_t = "off", 0.0          # 實機跟隨開著才做高度補償
        self.create_subscription(String, "/arm_real_status", self._real_status_cb, 10)
        self.scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.valid_cli = self.create_client(GetStateValidity, "/check_state_validity")
        self.get_scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.applied_key = None        # MoveIt 確定套用了的場景（_scene_now 的 key）；跟最新的不一樣 = 還沒進 MoveIt
        self.scene_lock = threading.Lock()      # 同一時間只送一包場景
        self.move_ac = ActionClient(self, MoveGroup, "/move_action")
        self.exec_ac = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.create_timer(1.0 / RATE, self._tick)
        self.create_timer(SCENE_RESEND, lambda: self._sync_scene(force=True))
        self.obstacle_src = {}         # 來源 → 最新送來的物體清單（web = 網頁、sp3 = 任務管理…）；self.obstacles 是合併後的
        self.attached = {}             # 掛在手上的物體 id → 手（/arm_attach）
        self.dz_now = 0.0              # 上一個動作的高度補償（m）：模擬的手比實機高這麼多，掛物體時用
        self.ctx = {"cmd": None, "arm": None}   # 執行中的動作（結果的 cmd、arm 欄位）
        self.fail_codes = {"IK": R_UNREACH}     # 這一次移動的失敗原因（中文）→ reason
        self.stop_req = False          # 收到 /arm_stop：這個動作不再執行下一段
        self.route_how = ""            # 這次移到預備點走哪一種路（_plan_move 第 3 步），寫在結果裡
        self.exec_gh = None            # 執行中的軌跡（/arm_stop 取消用）
        self.in_scene = set()          # 目前 MoveIt 場景裡由這裡放的物體 id
        self.scene_key = None          # 上次同步的內容，沒變就不重送
        self.scene_shift = 0.0         # 高度補償檢查時，MoveIt 場景裡的物體暫時往上移多少（m）
        self.ignore = set()            # 這一段規劃暫時不算的物體 id（字串）：起點就碰到它們時用
        self.mode, self.arm_choice, self.speed = "full", "auto", 0.5
        self.ax_cache = {}             # 這一次移動裡「只要求掌心方向」的 IK 解（寫失敗原因時不用再解一次）
        self.busy = False              # 正在執行一個動作
        self.q = {}                    # 關節名稱 → 角度（MoveIt 的 /joint_states）
        self.sent = None
        self.still = 0                 # 手臂已經停住幾個週期
        self.reported = False          # 這次停住後報過狀態了沒
        self.get_logger().info("等 MoveIt 的 /joint_states（MoveIt 有開嗎？）")

    def qa(self, arm):
        """這隻手目前的 7 軸角度（還沒收到就當站姿）"""
        return np.array([self.q.get(j, s) for j, s in zip(ARMS[arm]["joints"], STAND_MOVEIT)])

    # ---------- MoveIt 狀態 → 網頁 ----------
    def _joint_states_cb(self, msg):
        self.q.update(zip(msg.name, msg.position))

    def _tick(self):
        if not all(j in self.q for a in ARMS for j in ARMS[a]["joints"]):
            return
        names, vals = [], []
        for a in ARMS:                                  # 手指一直是張開的（0）：手掌另外控制，網頁畫的手指不代表實機
            names += ARMS[a]["joints"] + list(ARMS[a]["fingers"])
            vals += list(self.qa(a)) + [0.0] * len(ARMS[a]["fingers"])
        vals = np.array(vals)
        moved = self.sent is None or np.abs(vals - self.sent).max() > 1e-4
        if moved:
            self.still, self.reported = 0, False
        else:
            self.still += 1
        if moved or self.still % int(RATE) == 0:      # 動的時候每次都送，停住也每秒補送（網頁晚連上才看得到）
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = BASE
            msg.name, msg.position = names, [float(v) for v in vals]
            self.q_pub.publish(msg)
            self.sent = vals
        if not self.reported and not self.busy and self.still >= SETTLE * RATE:
            self.reported = True
            self._report(list(ARMS), "MoveIt 目前姿勢")

    def _plan_cb(self, msg):
        """在 RViz 自己按 Plan 的時候：先檢查整條軌跡（這裡發起的規劃由動作自己報告）"""
        if self.busy or not msg.trajectory:
            return
        paths = {a: p for a in ARMS if (p := _traj_q(msg.trajectory[-1], ARMS[a]["joints"])) is not None}
        if not paths:
            return
        moved = max(paths, key=lambda a: np.abs(paths[a][-1] - paths[a][0]).max())
        p_end = self.fk[moved](paths[moved][-1])[:3, 3]
        self.tgt_pub.publish(Point(x=float(p_end[0]), y=float(p_end[1]), z=float(p_end[2])))
        n = len(next(iter(paths.values())))
        self._report(list(paths), f"MoveIt 規劃好了（{n} 點，還沒執行）", paths=paths)

    # ---------- 場景物體（網頁、任務管理）→ MoveIt 場景 ----------
    def _obstacles_cb(self, msg):
        """/arm_ik_obstacles：{"source": "sp3", "objects": [...]}，或舊格式（只有清單）= 網頁。
        同一個來源每次送的是整份（空清單 = 清空這個來源），不會蓋掉別的來源"""
        try:
            d = json.loads(msg.data)
            if isinstance(d, list):
                src, objs = WEB, d
            else:
                src, objs = str(d["source"]), d["objects"]
                if not src or not isinstance(objs, list):
                    raise ValueError("source 不能是空的、objects 要是清單")
            if not all(isinstance(o, dict) and "type" in o for o in objs):
                raise ValueError("每個物體都要是 {id, type, …}")
        except (ValueError, KeyError, TypeError, AttributeError) as e:
            self.get_logger().warn(f"/arm_ik_obstacles 格式不對：{e}")
            return
        self.obstacle_src[src] = objs
        self._sync_scene()

    @property
    def obstacles(self):
        """所有來源合併後的物體清單；id 加上來源前綴（見 WEB）"""
        out = []
        for src, objs in sorted(self.obstacle_src.copy().items()):
            for o in objs:
                oid = str(o.get("id"))
                out.append(dict(o, id=oid if oid.startswith(src + "_") else f"{src}_{oid}"))
        return out

    def _scene_now(self):
        """要給 MoveIt 的物體（扣掉這一段暫時不算的、高度補償檢查時往上移）和代表它的 key"""
        objs = [shift_up(o, self.scene_shift) for o in self.obstacles if str(o.get("id")) not in self.ignore]
        return objs, json.dumps(objs, sort_keys=True)

    def _sync_scene(self, force=False, wait=False):
        """場景物體 → MoveIt（/apply_planning_scene）。
        wait=True（背景執行緒用）：先問 MoveIt 現在有哪些物體，照實際差異送（不會叫它刪不存在的物體、整包被判失敗），
          等它回覆；回傳 MoveIt 有沒有套用。
        wait=False（callback、計時器用）：送出就回來，回傳 None；MoveIt 回覆成功時才記 applied_key。
        同一時間只送一包：背景執行緒在等回覆時 callback 這邊直接跳過（計時器、下一個動作開始前的 _ensure_scene 會補）"""
        if not self.scene_lock.acquire(blocking=wait):
            return None
        try:
            objs, key = self._scene_now()
            if (key == self.scene_key and not force) or not self.scene_cli.service_is_ready():
                return (key == self.applied_key) if wait else None
            now = self._moveit_ids() if wait else None
            have = self.in_scene if now is None else {i for i in now if i.startswith("html_")}
            scene = PlanningScene(is_diff=True)
            ids = set()
            for o in objs:
                co = to_collision_object(o)
                if co is not None:
                    scene.world.collision_objects.append(co)
                    ids.add(co.id)
            for gone in have - ids:                # 刪掉的（或這一段暫時不算的）也要從 MoveIt 拿掉
                scene.world.collision_objects.append(CollisionObject(id=gone, operation=CollisionObject.REMOVE))
            ok = None
            if scene.world.collision_objects:
                fut = self.scene_cli.call_async(ApplyPlanningScene.Request(scene=scene))
                if wait:
                    res = self._wait(fut, 3.0)
                    ok = res is not None and bool(res.success)
                    if ok:
                        self.applied_key = key
                else:
                    fut.add_done_callback(lambda f, k=key: self._applied(f, k))
                if key != self.scene_key:
                    self.get_logger().info(f"MoveIt 場景更新：{len(ids)} 個物體" +
                                           (f"（{sorted(self.ignore)} 暫時不算）" if self.ignore else ""))
            elif now is not None:                  # 問過 MoveIt、本來就一樣：不用送
                ok = True
                self.applied_key = key
            self.in_scene, self.scene_key = ids, key
            return ok
        finally:
            self.scene_lock.release()

    def _applied(self, fut, key):
        res = fut.result()
        if res is not None and res.success:
            self.applied_key = key

    def _moveit_ids(self):
        """MoveIt 場景裡現在有哪些物體（id 集合）；問不到回傳 None（背景執行緒用）"""
        if not self.get_scene_cli.service_is_ready():
            return None
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        res = self._wait(self.get_scene_cli.call_async(req), 3.0)
        return None if res is None else {o.id for o in res.scene.world.collision_objects}

    def _ensure_scene(self, timeout=SCENE_READY):
        """動作開始前：確認 MoveIt 裡的場景物體就是最新收到的那一份，不是就同步、等 MoveIt 回覆。
        兩個都要對：MoveIt 回覆套用過這一份（applied_key：形狀、位置）、MoveIt 現在真的有這些物體（問 /get_planning_scene：
        move_group 重開會清空場景，applied_key 卻還是舊的）。
        2026-11-17：/apply_planning_scene 還沒好時收到的場景，要等下一次重送（最多 SCENE_RESEND 秒）才會進 MoveIt，
        中間的移動 MoveIt 看不到桌子，路線穿過桌子。回傳 None = 好了；不然回傳原因（背景執行緒用）"""
        end = time.time() + timeout
        while True:
            objs, key = self._scene_now()
            want = {"html_" + str(o["id"]) for o in objs if o.get("type") in ("ball", "box", "stick", "plane")}
            now = self._moveit_ids()
            if key == self.applied_key and now is not None and {i for i in now if i.startswith("html_")} == want:
                return None
            if self.scene_cli.wait_for_service(timeout_sec=min(1.0, max(0.05, end - time.time()))):
                if self._sync_scene(force=True, wait=True):
                    return None
                why = "MoveIt 沒有回覆，或不接受"
            else:
                why = "連不到 /apply_planning_scene（MoveIt 有開嗎？）"
            if time.time() > end:
                return why
            time.sleep(0.2)

    # ---------- 指令 ----------
    def _speed_cb(self, msg):
        self.speed = float(np.clip(msg.data * RAD_PER_SEC_PER_SPEED / MAX_VEL, 0.05, 1.0))

    def _real_status_cb(self, msg):
        try:
            self.real_state, self.real_t = json.loads(msg.data).get("state", "off"), time.time()
        except (ValueError, AttributeError):
            pass

    def _real_on(self):
        return self.real_state == "follow" and time.time() - self.real_t < REAL_STALE

    def _target_cb(self, msg):
        """之後整合用的 API：PoseStamped（base_link，m）= 掌心位置 + 任意手掌姿態（網頁不用這個，網頁用 /arm_move）"""
        if msg.header.frame_id not in ("", BASE, "world"):
            return self._note(f"目標的 frame_id '{msg.header.frame_id}' 不支援，只收 base_link", ok=False,
                              reason=R_BAD, cmd="move", arm=None)
        p, o = msg.pose.position, msg.pose.orientation
        p = np.array([p.x, p.y, p.z])
        R = None
        if self.mode != "pos_only":
            qn = np.linalg.norm([o.x, o.y, o.z, o.w])
            if qn < 1e-6:
                return self._note("位置 + 姿態模式要給姿態（收到的 quaternion 是 0）；只要位置的話 /arm_ik_mode 送 pos_only",
                                  ok=False, reason=R_BAD, cmd="move", arm=None)
            R = quat_to_R(o.x / qn, o.y / qn, o.z / qn, o.w / qn)
        self.tgt_pub.publish(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        spec = {"dir": None, "label": "", "axis": None, "fingers": None, "R": R}
        self._run("move", self._do_move, p, self.arm_choice, lambda arm: [spec])

    def _move_cb(self, msg):
        """網頁的「移動」：{"pos": [前, 右, 高]（cm）, "dir": 掌心方向, "fingers": 手指朝向（度，null = 自動）, "arm": auto / right / left}
        掌心方向：auto（先從上方，不行再試其他方向）/ arm_config.approaches 的 top、front、side、front_down、side_down、front_side"""
        try:
            d = json.loads(msg.data)
            f, r, h = (float(v) for v in d["pos"])
            dirname = d.get("dir") or "auto"
            fingers = d.get("fingers")
            fingers = None if fingers in (None, "") else float(fingers)
            choice = d.get("arm") or "auto"
        except (ValueError, KeyError, TypeError) as e:
            return self._note(f"/arm_move 格式不對：{e}", ok=False, reason=R_BAD, cmd="move", arm=None)
        if dirname != "auto" and dirname not in approaches("right"):
            return self._note(f"不認得的掌心方向 {dirname!r}", ok=False, reason=R_BAD, cmd="move", arm=None)
        if choice != "auto" and choice not in ARMS:
            return self._note(f"arm 只接受 auto / right / left，收到 {choice!r}", ok=False, reason=R_BAD, cmd="move", arm=None)
        p = from_meas(f, r, h)
        self.tgt_pub.publish(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        self._run("move", self._do_move, p, choice, self._dir_specs(p, dirname, fingers))

    def _dir_specs(self, p, dirname, fingers):
        """掌心方向 + 手指朝向 → 「手 → 要試的姿態清單」。每個姿態 {dir, label, axis = 掌心朝向, fingers, R（手指有指定才有）}。
        auto：先試從上方，不行再試其他方向，順手的（從肩膀往目標推的方向）先試。手指朝向（例如鎖定朝前 = 0°）每個方向都一樣"""
        def make(arm):
            A = approaches(arm)
            if dirname == "auto":
                reach = (p - self.shoulder[arm]) / max(np.linalg.norm(p - self.shoulder[arm]), 1e-9)
                names = ["top"] + sorted((k for k in A if k != "top"), key=lambda k: -(A[k][0] @ reach))
            else:
                names = [dirname]
            specs = []
            for k in names:
                a, label = A[k]
                fg = fingers                        # 手指鎖定（例如朝前 = 0°）時每個方向都盡量朝前，轉不到用最接近的角度
                                                    # （_closest_fingers，不換方向）；None = 每個方向都自動
                specs.append({"dir": k, "label": label, "axis": a, "fingers": fg,
                              "R": palm_R_fingers(arm, a, fg) if fg is not None else None})
            return specs
        return make

    def _stand_cb(self, msg):
        if msg.data:
            self._run("stand", self._do_stand)

    def _stop_cb(self, msg):
        """/arm_stop（Bool true）：取消執行中的軌跡（模擬停在目前的位置），這個動作也不再執行下一段。
        只停這一個動作，下一個指令照常做。實機要停另外送 /arm_real_enable false（real_arm_bridge）"""
        if not msg.data:
            return
        self.stop_req = True
        gh = self.exec_gh
        if gh is not None:
            gh.cancel_goal_async()
        self.get_logger().warn("收到 /arm_stop：" + ("取消執行中的軌跡" if gh is not None else
                                                   "目前沒有在執行軌跡" + ("，規劃中的動作不會執行" if self.busy else "")))

    def _run(self, cmd, fn, *args):
        """在背景執行一個動作：規劃 + 執行要好幾秒，不能卡住轉發手臂角度的 callback。
        cmd = 結果的 cmd 欄位（move / stand）"""
        if self.busy:
            self._note("上一個動作還沒做完，等它結束再送", ok=False, reason=R_BUSY, cmd=cmd, arm=None)
            return
        self.busy = True
        self.stop_req = False
        self.ctx = {"cmd": cmd, "arm": None}

        def work():
            try:
                if not self.move_ac.wait_for_server(timeout_sec=2.0):   # 沒開的話每個碰撞檢查都要等 3 秒逾時
                    return self._note(err_text(-9999), ok=False, reason=R_DOWN)
                why = self._ensure_scene()              # 場景物體（桌子等）確定在 MoveIt 裡才規劃
                if why:
                    return self._note(f"場景物體 {SCENE_READY:g} 秒內沒有套用到 MoveIt（{why}），手臂不動",
                                      ok=False, reason=R_DOWN)
                fn(*args)
            except Exception as e:      # 背景執行緒的例外不會自己印出來
                self._note(f"出錯：{e}", ok=False, reason=R_INTERNAL)
            finally:
                self.reported = True    # 動作自己報過結果了，停住後不要再用「目前姿勢」蓋掉
                self.ctx = {"cmd": None, "arm": None}
                self.busy = False
        threading.Thread(target=work, daemon=True).start()

    # ---------- 掛在手上的物體（任務管理：夾起來的藥瓶）----------
    def _attach_cb(self, msg):
        """/arm_attach（std_msgs/String，JSON；base_link，m）：
          {"action": "attach", "arm": "right", "id": "sp3_held", "type": "stick", "a": [x, y, z], "b": [x, y, z], "r": 0.026}
            type / 欄位同 /arm_ik_obstacles 的物體（ball / box / stick）。做成 AttachedCollisionObject 掛在那隻手的手掌 link，
            手掌、手指碰到它不算碰撞（touch_links）。之後的規劃、碰撞檢查（自己的 IK 也是問 MoveIt）都會算它，跟著手移動
          {"action": "detach", "arm": "right", "id": "sp3_held"}：拿掉，不留在場景裡
        實機跟隨開著時，模擬的手比實機高 dz_now（高度補償），物體跟著往上 dz_now，跟手的相對位置才會跟實機一樣。
        結果用 /arm_ik_status 回報（cmd = attach / detach）。不管手臂是不是正在動都可以送"""
        action, arm = "attach", None
        try:
            d = json.loads(msg.data)
            action, arm, oid = d["action"], d["arm"], str(d["id"])
            if action not in ("attach", "detach"):
                raise ValueError(f"action 只接受 attach / detach，收到 {action!r}")
            if arm not in ARMS:
                raise ValueError(f"arm 只接受 right / left，收到 {arm!r}")
            mid = f"attached_{oid}"
            scene = PlanningScene(is_diff=True)
            scene.robot_state.is_diff = True
            aco = AttachedCollisionObject(link_name=ARMS[arm]["tip"])
            if action == "attach":
                dz = self.dz_now if self._real_on() else 0.0
                co = to_collision_object(shift_up(dict(d, id=oid), dz)) if d.get("type") != "plane" else None
                if co is None:
                    raise ValueError(f"type 只接受 ball / box / stick，收到 {d.get('type')!r}")
                co.id = mid
                aco.object, aco.touch_links = co, list(self.hand_links[arm])
            else:
                dz = 0.0
                aco.object = CollisionObject(id=mid, operation=CollisionObject.REMOVE)
                # MoveIt 拿下來的物體會放回場景，要再從場景刪掉。沒掛著的時候不刪（刪不存在的物體 MoveIt 會回失敗）
                if oid in self.attached:
                    scene.world.collision_objects.append(CollisionObject(id=mid, operation=CollisionObject.REMOVE))
            scene.robot_state.attached_collision_objects = [aco]
        except (ValueError, KeyError, TypeError, AttributeError) as e:
            return self._note(f"/arm_attach 格式不對：{e}", ok=False, reason=R_BAD, cmd=action,
                              arm=arm if arm in ARMS else None)
        if not self.scene_cli.service_is_ready():
            return self._note("連不到 /apply_planning_scene（MoveIt 有開嗎？）", ok=False, reason=R_DOWN,
                              cmd=action, arm=arm, arms=(arm,))

        def work():                 # 等 MoveIt 回覆要在背景執行緒（callback 裡等會卡住）
            res = self._wait(self.scene_cli.call_async(ApplyPlanningScene.Request(scene=scene)), 3.0)
            name, was = ARMS[arm]["label"], self.attached.get(oid)
            if res is None:
                return self._note(f"{action} {oid}：MoveIt 3 秒沒有回覆", ok=False, reason=R_DOWN,
                                  cmd=action, arm=arm, arms=(arm,))
            if action == "attach":
                if not res.success:
                    return self._note(f"{oid} 掛到{name}失敗（MoveIt 不接受）", ok=False, reason=R_PLAN,
                                      cmd=action, arm=arm, arms=(arm,))
                self.attached[oid] = arm
                self._note(f"{oid} 掛到{name}" + (f"（高度補償：跟著模擬的手往上 {dz * 100:.1f} cm）" if dz else ""),
                           cmd=action, arm=arm, arms=(arm,))
            else:
                if not res.success and was is not None:
                    return self._note(f"{oid} 從{name}拿掉失敗（MoveIt 不接受）", ok=False, reason=R_PLAN,
                                      cmd=action, arm=arm, arms=(arm,))
                self.attached.pop(oid, None)
                self._note(f"{oid} 從{name}拿掉" if was else f"{oid} 本來就沒有掛在手上", cmd=action, arm=arm, arms=(arm,))
        threading.Thread(target=work, daemon=True).start()

    def _arm_order(self, p, choice):
        """要試哪幾隻手：指定的那隻；auto = 肩膀離目標近的先試"""
        if choice in ARMS:
            return [choice]
        return sorted(ARMS, key=lambda a: np.linalg.norm(np.asarray(p) - self.shoulder[a]))

    # ---------- 動作（在背景執行緒跑）----------
    def _do_move(self, p, choice, make_specs):
        """掌心移到 p（base_link，m）。make_specs(手) = 這隻手要依序試的姿態（_dir_specs）。步驟見最上面"""
        t0 = time.time()
        self.ax_cache = {}
        self.fail_codes = {"IK": R_UNREACH}
        dz = REAL_Z_COMP * max(to_meas(p)[0], 0.0) / 100 if self._real_on() else 0.0
        p_sim = p + UP * dz                         # 模擬（MoveIt）的掌心目標：實機會低 dz，剛好落在 p
        f, r, h = to_meas(p)
        goal = f"前 {f:.1f}、右 {r:.1f}、高 {h:.1f} cm"
        for o in self.obstacles:                    # 掌心目標在物體裡面（或貼著表面）：每個方向都會撞到，先直接講清楚
            d = surface_dist(o, p)
            if d is not None and d < INSIDE_TOL:
                top, bottom = object_top_bottom(o)
                lb = _obj_label(o["id"])
                return self._note(
                    f"掌心目標（{goal}）{'在' + lb + '裡面' if d < 0 else '貼著' + lb + '的表面'}。\n"
                    f"{lb} 底 {bottom:.1f}、頂 {top:.1f} cm。左邊的「高」是掌心的高度、不是物體的：從上方要停在物體上面，"
                    f"掌心高要比 {top:.1f} 高（例如 {top + 2:.1f}）。點物體再按「← 選取的物體」會自動算", ok=False, reason=R_COLL)
        fails, best = {}, None
        for arm in self._arm_order(p_sim, choice):
            fails[arm] = []
            for spec in make_specs(arm):
                cnt = Counter()
                res = self._plan_move(arm, p_sim, spec["R"], dz, cnt, spec["axis"])
                if not res and spec["dir"] and spec["fingers"] is not None and cnt and cnt.most_common(1)[0][0] == "IK":
                    spec2, res = self._closest_fingers(arm, p_sim, spec, dz, cnt)   # 手指轉不到：同方向改最接近的角度
                    spec = spec2 or spec
                if res:
                    best = (spec,) + res
                    break
                fails[arm].append((spec, cnt))
            if best:
                break
        if best is None:
            return self._note(f"到不了 {goal}（試了 {time.time() - t0:.0f} 秒）：\n" + self._fail_text(p_sim, fails), ok=False,
                              reason=self._overall_reason(fails))
        spec, arm, route, appr = best
        self.ctx["arm"] = arm                       # 結果的 arm 欄位：實際用的那隻手（執行失敗也寫這隻）
        name = ARMS[arm]["label"]
        t_plan = time.time() - t0
        if appr is None:                            # 微調：直接轉過去
            if not self._exec(route, f"{name}移動（微調，直接轉過去）"):
                return
        else:
            merged = self._pre_and_approach(arm, route, appr)
            if merged is not None:                  # 一段走完：預備點不停，最後直線靠近越來越慢
                if not self._exec(merged, f"{name}移動：移到預備點、直線靠近"):
                    return
            elif not (self._exec(route, f"{name}移動：移到預備點") and
                      self._exec(self._timed(arm, appr, self._approach_vlim(len(appr))), f"{name}移動：直線靠近")):
                return
        self.dz_now = dz                            # 掛物體（/arm_attach）時用：模擬的手現在比實機高 dz
        T = self.fk[arm](self.qa(arm))
        head = f"{name}{spec['label']}到了：{goal}"
        R_show = spec["R"]                          # 鎖定手指 / API 給的姿態：就是要的那個（IK 誤差 < 0.1°）
        if spec["dir"]:
            ang = finger_angle(arm, spec["axis"], T[:3, :3])
            head += f"（{finger_text(spec['dir'], ang)}）"
            if R_show is None:                      # 手指自動：用掌心方向 + 實際的手指角度（整數度）重組，
                R_show = palm_R_fingers(arm, spec["axis"], round(ang))   # 掌心正朝下（P 90°）時 R、Y 才不會因為小誤差亂跳
        elif R_show is None:
            head += "（只對位置）"
            R_show = T[:3, :3]
        lines = [head, pose_text(R_show)]
        if "want" in spec:                          # 手指轉不到指定的角度，用了最接近的（_closest_fingers）
            want = (FINGER_ZERO.get(spec["dir"], "手指 0°").split("（")[0] if abs(spec["want"]) < 1e-6
                    else f"手指 {spec['want']:+.0f}°")
            lines.append(f"{want}轉不到（再轉過去{spec['limit'] or '就到不了'}），"
                         f"改用最接近的：{finger_text(spec['dir'], spec['fingers'])}")
        for s, cnt in fails.get(arm, []):           # 自動：前面試過不行的方向（例如從上方到不了）
            lines.append(f"{s['label']}不行（{self._explain(arm, p_sim, s, cnt)}），改用{spec['label']}")
        if dz:
            lines.append(f"高度補償 +{dz * 100:.1f} cm：模擬的手停得比較高，實機掌心應該在 {goal}")
        if appr is not None and spec["R"] is not None:
            turn = rot_angle(self.fk[arm](appr[0])[:3, :3], spec["R"])
            if turn > 10:
                lines.append(f"靠近途中手掌轉了 {turn:.0f}°（預備點那麼高同一個姿態轉不到；最後 {HOLD_DIST * 100:.0f} cm 姿態不變）")
        lines.append(f"路線：{self.route_how}；找姿勢、路花了 {t_plan:.0f} 秒")
        self._report([arm], "\n".join(lines))

    def _fail_text(self, p, fails):
        """到不了時，每隻手、每個試過的方向說明原因；只指定一個方向、又是到不了（不是撞到）時，列出這個位置可以用的方向"""
        lines = []
        for arm, items in fails.items():
            name = ARMS[arm]["label"]
            whys = [self._explain(arm, p, spec, cnt) for spec, cnt in items]
            dist = float(np.linalg.norm(p - self.shoulder[arm]))
            far = (f"（目標離肩膀 {dist * 100:.0f} cm，手臂伸直只有 {self.arm_len[arm] * 100:.0f} cm，伸不到）"
                   if dist > 0.98 * self.arm_len[arm] else "")
            if len(items) > 1 and len(set(whys)) == 1:     # 自動試了好幾個方向、原因都一樣：寫一次就好
                lines.append(f"{name}試了 {len(items)} 個掌心方向都不行：{whys[0]}{far}")
                continue
            for (spec, cnt), why in zip(items, whys):
                lines.append(f"{name}{spec['label']}：{why}")
            if far:
                lines.append(f"{name}{far}")
            if len(items) == 1 and items[0][0]["dir"] and "IK" in items[0][1]:
                ok_dirs = self._dirs_ok(arm, p)
                lines.append(f"{name}在這個位置可以用的掌心方向：{'、'.join(ok_dirs)}" if ok_dirs
                             else f"{name}在這個位置哪個掌心方向都到不了（手伸不到或會撞到）")
        return "\n".join(lines)

    def _explain(self, arm, p, spec, cnt, short=False):
        """一個姿態為什麼不行（白話）。IK 解不到時：有指定手指朝向的話，算這個方向手指能轉到哪些角度"""
        if not cnt:
            return "?"
        why = cnt.most_common(1)[0][0]
        if why != "IK":
            return why
        if spec["dir"]:
            if spec["fingers"] is not None and not short:
                self._stage(f"{spec['label']}手指轉不到 {spec['fingers']:+.0f}°，找能轉的範圍")
                free = [q for q in self._axis_sols(arm, p, spec["axis"]) if self._free(arm, q)]
                if free:
                    angs = [finger_angle(arm, spec["axis"], self.fk[arm](q)[:3, :3]) for q in free]
                    want = (FINGER_ZERO.get(spec["dir"], "手指 0°").split("（")[0] + "（鎖定）" if abs(spec["fingers"]) < 1e-6
                            else f"你填的 {spec['fingers']:+.0f}°")
                    return (f"掌心方向到得了，但手指只能轉 {_segments(angs)}（{FINGER_HINT[spec['dir']]}），"
                            f"{want}轉不到")
            return "這個位置到不了（手伸不到，或關節轉不到這個方向）" if spec["fingers"] is None or short else \
                "這個位置用這個掌心方向到不了（手伸不到，或關節轉不到）"
        if spec["R"] is not None and not short:     # API 給的任意姿態：找做得到的姿態當例子
            out, seen = [], set()
            same = [q for q in self._axis_sols(arm, p, spec["R"][:, 0]) if self._free(arm, q)]
            for q in sorted(same, key=lambda q: self._pose_cost(arm, q)):
                rpy = rpy_deg(self.fk[arm](q)[:3, :3])
                if tuple(np.round(rpy / 5).astype(int)) not in seen:
                    seen.add(tuple(np.round(rpy / 5).astype(int)))
                    out.append("R {:.0f} P {:.0f} Y {:.0f}".format(*rpy))
            if out:
                return "掌心朝這個方向可以，手指轉不到這個角度；做得到的例如 " + "；".join(out[:3])
        return "這個位置到不了（手伸不到，或關節範圍內做不到）"

    def _dirs_ok(self, arm, p):
        """這個位置哪些掌心方向到得了（不會撞到），回傳方向說明的清單"""
        self._stage(f"找這個位置可以用的掌心方向：{ARMS[arm]['label']}")
        return [label for a, label in approaches(arm).values()
                if any(self._free(arm, q) for q in self._axis_sols(arm, p, a)[:4])]

    def _closest_fingers(self, arm, p, spec, dz, cnt):
        """手指指定了角度（例如鎖定朝前 = 0°）但這個位置轉不到：同一個掌心方向，改用做得到、最接近的角度。
        2026-11-13 使用者決定：不換掌心方向。以前會換到別的方向（例如球上方 前 40 高 98 換成斜右上方），
        掌心位置是照從上方算的，換方向後對物體來說不對，姿勢很怪（第 7 軸差 3° 到極限），試很多方向也慢。
        「最接近」：只看 IK 隨便解到的幾組會差很多（同一個例子解到 +26°，其實偏內 11° 就做得到），
        所以每一組都往要的角度逼近（_finger_edge），挑最接近的；規劃不行（撞到等）再試下一個，最多 3 個。
        回傳 (改過的 spec：fingers = 用的角度、want = 原本要的、limit = 卡住的關節, _plan_move 的結果)；
        都不行回傳 (None, None)，原因寫回 cnt"""
        want, axis = spec["fingers"], spec["axis"]
        free = [q for q in self._axis_sols(arm, p, axis) if self._free(arm, q)]
        if not free:
            return None, None
        self._stage(f"{spec['label']}手指轉不到 {want:+.0f}°，找最接近的角度")
        off = lambda ang: (ang - want + 180) % 360 - 180        # 離要的角度差幾度（帶正負）
        edges = sorted((self._finger_edge(arm, p, axis, want, q) for q in free), key=lambda c: abs(off(c[0])))
        lim = edges[0][2]                           # 為什麼轉不到要的角度：最接近的那組卡在哪個關節
        # 候選：逼近到的邊界 + 原本解到的（邊界上的規劃不一定過：靠近、碰撞），由近到遠，最多試 4 個
        cands = edges + [(finger_angle(arm, axis, self.fk[arm](q)[:3, :3]), q, "") for q in free]
        tried, why = [], None
        for ang, q, _ in sorted(cands, key=lambda c: abs(off(c[0]))):
            if len(tried) >= 4:
                break
            if any(abs(off(ang) - off(t)) < 3 for t in tried):
                continue
            tried.append(ang)
            R, c2 = palm_R_fingers(arm, axis, ang), Counter()
            res = self._plan_move(arm, p, R, dz, c2, axis, seeds=[q])
            if res:
                return dict(spec, fingers=ang, R=R, want=want, limit=lim), res
            why = why or c2.most_common(1)[0][0]
        cnt.clear()
        self._fail(cnt, f"手指轉不到 {want:+.0f}°，最接近的（{finger_text(spec['dir'], tried[0])}）也不行："
                        f"{'到不了' if why == 'IK' else why}", self.fail_codes.get(why, R_PLAN))
        return None, None

    def _finger_edge(self, arm, p, axis, want, q):
        """從做得到的姿勢 q 往要的手指角度 want 逼近（二分法到 1° 內，每一步從上一個做得到的姿勢接著解，
        同一種手臂擺法連續轉過去）。回傳 (做得到、最接近的角度, 那個姿勢, 卡在哪個關節的說明)；碰撞之後 _plan_move 再檢查"""
        ik = self.ik[arm]
        lo = (finger_angle(arm, axis, self.fk[arm](q)[:3, :3]) - want + 180) % 360 - 180   # 做得到（差幾度）
        hi = 0.0                                                                            # 做不到（要的本身）
        while abs(lo - hi) > 1.0:
            mid = (lo + hi) / 2
            q2, conv = ik.solve(p, q, R=palm_R_fingers(arm, axis, want + mid), q_rest=q, iters=2 * IK_ITERS)
            if conv and not check(arm, q2):
                lo, q = mid, q2
            else:
                hi = mid
        return want + lo, q, self._limit_text(arm, q)

    def _limit_text(self, arm, q):
        """姿勢 q 最靠近範圍兩端（5° 內）的關節 →「第 4 軸（手肘，11 號）會超過極限 80°」；都不靠近回傳 ''"""
        lo, hi = self.ik[arm].lower, self.ik[arm].upper
        m = np.minimum(q - lo, hi - q)
        j = int(np.argmin(m))
        if m[j] > np.radians(5):
            return ""
        lim = hi[j] if hi[j] - q[j] < q[j] - lo[j] else lo[j]
        return f"第 {j + 1} 軸（{LABELS[j]}，{ARMS[arm]['motors'][j]} 號）會超過極限 {np.degrees(lim):.0f}°"

    def _plan_move(self, arm, p, R, dz, fails, axis=None, seeds=None):
        """一隻手移到 p（模擬的目標，已經補過高度）。R = 整個手掌姿態；axis = 只要求掌心朝向（手指讓 IK 挑）；都 None = 只對位置。
        seeds = 已經知道的解（_closest_fingers 逼近到的），固定姿態解的時候先從這些開始
        回傳 (手, 軌跡, 直線靠近的關節角清單) 或 None（原因記在 fails；IK 解不到記 "IK"，由 _explain 說明）；
        直線靠近 = None 是微調：軌跡直接到目標"""
        if np.linalg.norm(p - self.shoulder[arm]) > REACH:
            self._fail(fails, "離肩膀太遠", R_UNREACH)
            return None
        name = ARMS[arm]["label"]
        small = self._small_move(arm, p, R, dz, axis)
        if small:
            return small
        # 1. 目標姿勢：解好幾組，不會撞到的留下
        self._stage(f"找姿勢：{name}")
        sols = self._solve(arm, p, R, axis, seeds)
        ok = [q for q in sols if self._free(arm, q)]
        if not ok:
            if sols:
                self._fail(fails, "會撞到：" + "、".join(self._contacts(arm, sols[0])) +
                           "（掌心太靠近物體，或手會碰到旁邊的東西）", R_COLL)
            else:
                self._fail(fails, "IK", R_UNREACH)
            return None
        if dz > 0.001:
            self._stage(f"檢查實機的位置（比模擬低 {dz * 100:.1f} cm）：{name}")
            ok = self._real_free(arm, ok, dz)
            if not ok:
                self._fail(fails, f"實機的位置（比模擬低 {dz * 100:.1f} cm）會撞到東西", R_COLL)
                return None
        # 2. 預備點 → 目標走直線（每一步都檢查碰撞，含預備點），像站姿的先試
        cands = []
        for q in sorted(ok, key=lambda q: self._pose_cost(arm, q)):
            if len(cands) >= MAX_FULL:
                break
            self._stage(f"檢查最後 {PRE_DIST * 100:.0f} cm 直線靠近：{name}")
            a = self.fk[arm](q)[:3, 0]              # 掌心朝向
            back, why = self._line(arm, q, p, p - a * PRE_DIST, a, R)
            if back is None:
                self._fail(fails, f"最後 {PRE_DIST * 100:.0f} cm 直線靠近{why}", LINE_REASON.get(why, R_LIMIT))
                continue
            appr = back[::-1] + [q]                 # 預備點 → 目標
            if check(arm, appr):
                self._fail(fails, "直線靠近途中會轉過頭", R_LIMIT)
                continue
            cands.append((self._pose_cost(arm, q) + _travel(appr), appr))
        # 3. 目前姿勢 → 預備點，三種路依序試（每一種都逐點檢查碰撞）：
        #   ① 每顆馬達直接轉過去（不繞路，線扭最少）
        #   ② 先往上抬過物體再過去（_via_path；2026-11-18 使用者要的收斂路線，例如從站姿先抬過桌面再往前）
        #   ③ 請 MoveIt 規劃（會繞開物體，但路線是隨機找的，常往旁邊、往後甩）
        top = sorted(cands, key=lambda c: c[0])[:MAX_ROUTE_TRIES]
        for _, appr in top:
            self._stage(f"找移到預備點的路：{name}")
            route = self._direct({arm: appr[0]})
            if route is not None:
                self.route_how = "每顆馬達直接轉過去"
                return arm, route, appr
        for _, appr in top[:VIA_TRIES]:
            self._stage(f"直接轉會撞到，試先往上抬過物體再過去：{name}")
            path = self._via_path(arm, self.qa(arm), appr[0])
            if path is not None:
                self.route_how = "先往上抬過物體再過去"
                return arm, self._path_route(arm, path), appr
        for _, appr in top:
            self._stage(f"請 MoveIt 規劃移到預備點的路：{name}")
            code, res, why = self._plan_route(ARMS[arm]["group"], self._joint_goal(ARMS[arm]["joints"], appr[0], tol=1e-3))
            if code == 1 and not check(arm, _traj_q(res.planned_trajectory, ARMS[arm]["joints"])):
                self.route_how = "MoveIt 規劃（直接轉、先抬都會撞到）"
                return arm, res.planned_trajectory, appr
            self._fail(fails, f"移到預備點：{why if code != 1 else '路徑會轉過頭'}",
                       R_LIMIT if code == 1 else plan_reason(code))
        return None

    def _obstacle_top(self, p0, p1):
        """起點、終點掌心（base_link，m）的水平範圍往外 VIA_CORRIDOR 以內，場景物體的最高點（m）；沒有物體回傳 None。
        地板（plane）不算"""
        lo = np.minimum(p0[:2], p1[:2]) - VIA_CORRIDOR
        hi = np.maximum(p0[:2], p1[:2]) + VIA_CORRIDOR
        top = None
        for o in self.obstacles:
            t = o.get("type")
            if t == "box":
                c, h = np.asarray(o["c"], float), np.asarray(o["h"], float)
                ext = np.abs(np.asarray(o["R"], float)) @ h
                omin, omax, otop = c[:2] - ext[:2], c[:2] + ext[:2], c[2] + ext[2]
            elif t == "stick":
                a, b, r = np.asarray(o["a"], float), np.asarray(o["b"], float), float(o["r"])
                omin, omax, otop = np.minimum(a, b)[:2] - r, np.maximum(a, b)[:2] + r, max(a[2], b[2]) + r
            elif t == "ball":
                c, r = np.asarray(o["c"], float), float(o["r"])
                omin, omax, otop = c[:2] - r, c[:2] + r, c[2] + r
            else:
                continue
            if np.all(omax >= lo) and np.all(omin <= hi):
                top = otop if top is None else max(top, otop)
        return top

    def _via_path(self, arm, q0, q_goal):
        """先往上抬過物體、再過去：q0 → 掌心沿直線往正上方抬（掌心朝向不變，每 CART_STEP 解一次 IK、檢查碰撞，同 _line）
        → 每顆馬達直接轉到 q_goal（每 DIRECT_STEP 檢查碰撞，同 _direct）。比 MoveIt 隨機找的路收斂，手不會往旁邊、往後甩。
        抬到的高度 = 起點、終點之間物體的最高點 + VIA_CLEARS，低的先試。
        回傳關節角清單（含 q0、q_goal）或 None（沒有物體擋、起點已經在物體上方、或都不行）。
        回站姿時用 _via_path(站姿, 目前) 再倒過來走：沒撞到的路倒過來也不會撞"""
        q0, q_goal = np.asarray(q0, float), np.asarray(q_goal, float)
        T0 = self.fk[arm](q0)
        p0, p1 = T0[:3, 3], self.fk[arm](q_goal)[:3, 3]
        top = self._obstacle_top(p0, p1)
        if top is None or p0[2] >= top + VIA_CLEARS[0]:
            return None
        for clear in VIA_CLEARS:
            z = top + clear
            up, why = self._line(arm, q0, p0, np.array([p0[0], p0[1], z]), T0[:3, 0])
            if up is None:
                self.get_logger().info(f"{ARMS[arm]['label']}往正上方抬到 高 {to_meas([0, 0, z])[2]:.1f} cm 不行：{why}")
                break                               # 抬不到這麼高，更高的也不行
            rest = self._direct_path(arm, up[-1], q_goal)
            if rest is not None:
                path = np.vstack([q0, np.array(up), rest[1:]])
                if not check(arm, path):
                    self.get_logger().info(f"{ARMS[arm]['label']}先往上抬到 高 {to_meas([0, 0, z])[2]:.1f} cm"
                                           f"（物體最高 {to_meas([0, 0, top])[2]:.1f}）再過去")
                    return path
        return None

    def _direct_path(self, arm, qa, qb):
        """一隻手 qa → qb 每顆馬達直接轉（同 _direct）：關節角清單（含兩端）或 None（實機做不到、途中會撞到）"""
        qa, qb = np.asarray(qa, float), np.asarray(qb, float)
        path = np.linspace(qa, qb, max(2, int(np.ceil(np.abs(qb - qa).max() / DIRECT_STEP)) + 1))
        if check(arm, path) or any(self._contacts(arm, q) for q in path[1:]):
            return None
        return path

    def _path_route(self, arm, path):
        """關節角清單 → 排好時間的軌跡（設定的速度，轉角處 time_path 會先慢下來）"""
        return self._timed(arm, path, np.full(len(path), MAX_VEL * max(self.speed, 0.02)))

    def _fail(self, cnt, text, reason):
        """失敗原因記到 cnt（中文，_explain、_fail_text 用），同時記下它的分類（reason）"""
        cnt[text] += 1
        self.fail_codes[text] = reason

    def _overall_reason(self, fails):
        """每隻手、每個方向的主要原因 → 一個 reason（REASON_ORDER：先報換手或重試還有機會的）"""
        codes = {self.fail_codes.get(cnt.most_common(1)[0][0], R_PLAN)
                 for items in fails.values() for _, cnt in items if cnt}
        return next((r for r in REASON_ORDER if r in codes), R_UNREACH)

    def _small_move(self, arm, p, R, dz, axis=None):
        """微調：目標離目前掌心不到 PRE_DIST、姿態差不到 SMALL_ROT，就從目前姿勢解（姿勢跟現在接近）、直接轉過去，
        不退到預備點。不行（姿勢要換一大段、會撞到）回傳 None，走一般流程"""
        q0 = self.qa(arm)
        T = self.fk[arm](q0)
        if np.linalg.norm(T[:3, 3] - p) > PRE_DIST:
            return None
        if R is not None and rot_angle(T[:3, :3], R) > np.degrees(SMALL_ROT):
            return None
        if R is None and axis is not None and np.arccos(np.clip(T[:3, 0] @ axis, -1, 1)) > SMALL_ROT:
            return None
        q, conv = self.ik[arm].solve(p, q0, R=R, axis=None if R is not None else axis, q_rest=q0, iters=2 * IK_ITERS)
        why = ("從目前姿勢解不到（姿態在邊緣，要換姿勢）" if not conv else
               f"有一軸要轉 {np.degrees(np.abs(q - q0).max()):.0f}°" if np.abs(q - q0).max() > SMALL_JOINT else
               "實機做不到" if check(arm, q) else
               "會撞到東西" if not self._free(arm, q) else
               "實機的位置（比模擬低）會撞到東西" if dz > 0.001 and not self._real_free(arm, [q], dz) else None)
        route = None if why else self._direct({arm: q})
        if route is None:
            self.get_logger().info(f"{ARMS[arm]['label']}不能微調（{why or '直接轉過去途中會撞到'}），走一般流程（退到預備點再靠近）")
            return None
        self.route_how = "微調（直接轉過去）"
        return arm, route, None

    def _solve(self, arm, p, R, axis=None, seeds=None):
        """掌心到 p 的 IK 解（R = 手掌姿態；axis = 只要求掌心朝向；都 None = 只對位置），只留實機做得到的（check）。
        給整個姿態時：從起點表直接固定姿態解常常不收斂（明明到得了也說到不了），
        先解一輪「只對掌心朝向」的，手指方向最接近的那幾組當起點，再固定整個姿態解"""
        ik = self.ik[arm]
        if R is not None:
            ax = self._axis_sols(arm, p, R[:, 0])
            # 掌心方向本身就到不了、或手指離指定的都很遠：固定整個姿態也解不到，不用再試（到不了時省很多時間）
            if not seeds and (not ax or min(np.arccos(np.clip(self.fk[arm](q)[:3, 2] @ R[:, 2], -1, 1)) for q in ax) > SKIP_FINGER):
                return []
            near = list(seeds or []) + sorted(ax, key=lambda q: -(self.fk[arm](q)[:3, 2] @ R[:, 2]))
            sols = ik.solve_many(p, near + self._seeds(arm, p, R[:, 0], fingers=R[:, 2]), R=R,
                                 max_sols=N_SOLS, iters=2 * IK_ITERS)
        elif axis is not None:
            return self._axis_sols(arm, p, axis)
        else:
            sols = ik.solve_many(p, self._seeds(arm, p), max_sols=N_SOLS, iters=IK_ITERS)
        return [q for q in sols if not check(arm, q)]

    def _axis_sols(self, arm, p, a):
        """掌心到 p、掌心朝 a（手指朝哪都可以）的 IK 解，只留實機做得到的。同一次移動裡記起來（ax_cache）"""
        key = (arm, tuple(np.round(p, 4)), tuple(np.round(a, 3)))
        if key not in self.ax_cache:
            sols = self.ik[arm].solve_many(p, self._seeds(arm, p, a), axis=a, max_sols=N_SOLS, iters=IK_ITERS)
            self.ax_cache[key] = [q for q in sols if not check(arm, q)]
        return self.ax_cache[key]

    def _real_free(self, arm, qs, dz):
        """高度補償時實機的掌心會低 dz：把場景物體往上移 dz（= 手往下移）再檢查一次，留下不會撞到的。
        不是用 IK 解實機的位置：實機到得了模型說到不了的地方（例如從上方碰桌上的東西），IK 會解不到"""
        self.scene_shift = dz
        self._sync_scene(force=True, wait=True)
        try:
            return [q for q in qs if self._free(arm, q)]
        finally:
            self.scene_shift = 0.0
            self._sync_scene(force=True, wait=True)

    def _do_stand(self):
        for arm in ARMS:                            # 站姿本身會碰到物體的話，MoveIt 只會說「失敗」，先查出是哪個
            hits = self._contacts(arm, STAND_MOVEIT)
            if hits:
                return self._note(f"{ARMS[arm]['label']}的站姿會碰到東西：" + "、".join(hits) + "，先把它移開", ok=False,
                                  reason=R_COLL)
        stand = {a: STAND_MOVEIT for a in ARMS}
        self._stage("回站姿：先試每顆馬達直接轉回去（不繞路，線直接轉回原位）")
        route, how = self._direct(stand), "每顆馬達直接轉回去"
        backs = {}
        if route is None:
            # 直接轉會撞到（例如手就在物體旁邊）：不在站姿的手先沿掌心反方向退開（= 移動進來的反方向），
            # 再直接轉回去；退 10 cm 還會掃到物體就退 20 cm
            for dist in (PRE_DIST, 2 * PRE_DIST):
                self._stage(f"直接轉回去會撞到：試試手先往後退 {dist * 100:.0f} cm")
                cand = self._retreats(dist) or backs
                backs = cand
                if cand:
                    route = self._direct(stand, start={a: b[-1] for a, b in cand.items()})
                    how = f"{'、'.join(ARMS[a]['label'] for a in cand)}先往後退 {dist * 100:.0f} cm，再每顆馬達直接轉回去"
                if route is not None:
                    break
        if route is None:
            # 往後退開還是會撞到：從物體上方繞回來，到站姿正上方再往下（_via_path 的路倒過來走，2026-11-18，比 MoveIt 收斂）。
            # 只處理一隻手不在站姿的情況（兩隻手都伸出去時交給 MoveIt 一起規劃）
            start = {a: (backs[a][-1] if a in backs else self.qa(a)) for a in ARMS}
            off = [a for a in ARMS if np.abs(np.asarray(start[a]) - STAND_MOVEIT).max() >= 0.05]
            if len(off) == 1:
                self._stage(f"直接轉回去會撞到：試從物體上方繞回站姿正上方再往下（{ARMS[off[0]]['label']}）")
                path = self._via_path(off[0], STAND_MOVEIT, start[off[0]])      # 站姿 → 往上抬 → 目前
                if path is not None:
                    route = self._path_route(off[0], path[::-1])
                    how = ("先往後退開，再" if backs else "") + "從物體上方繞回站姿正上方再往下"
        # 退開（往後退是移動進來的反方向，一定安全）：之後直接轉回去，或從空一點的地方請 MoveIt 規劃
        for a, path in backs.items():
            if not self._exec(self._timed(a, path, np.full(len(path), MAX_VEL * max(self.speed * SLOW, 0.02))),
                              f"{ARMS[a]['label']}往後退開"):
                return
        if route is None:                            # 退開後還是會撞到：請 MoveIt 繞開（只規劃不在站姿的手，比較好找）
            off = [a for a in ARMS if np.abs(self.qa(a) - STAND_MOVEIT).max() >= 0.05] or list(ARMS)
            self._stage(f"MoveIt 規劃回站姿（{'、'.join(ARMS[a]['label'] for a in off)}）")
            group = ARMS[off[0]]["group"] if len(off) == 1 else "both_arms"
            joints = [j for a in off for j in ARMS[a]["joints"]]
            code, res, why = self._plan_route(group, self._joint_goal(joints, np.tile(STAND_MOVEIT, len(off))),
                                              SWING_TOL_STAND)
            if code != 1:
                return self._note(f"回站姿規劃失敗：{why}" + ("（手已經往後退開了，可以再按一次回站姿，或用「移動」把手移到空一點的地方）"
                                                         if backs else "（手可能被物體擋住，先用「移動」把手移開）"), ok=False,
                                  reason=plan_reason(code))
            route, how = res.planned_trajectory, ("先往後退開，再 " if backs else "") + "MoveIt 規劃（直接轉會撞到）"
        if self._exec_checked(route, "回站姿"):
            self.dz_now = 0.0
            self._report(list(ARMS), f"雙手回到站姿（{how}）")

    def _retreats(self, dist):
        """不在站姿的手沿掌心反方向直線退 dist（m）的關節角清單 {手: [目前, ..., 退開後]}；退不了的手不放"""
        out = {}
        for a in ARMS:
            q0 = self.qa(a)
            if np.abs(q0 - STAND_MOVEIT).max() < 0.05:
                continue
            T = self.fk[a](q0)
            back, _ = self._line(a, q0, T[:3, 3], T[:3, 3] - T[:3, 0] * dist, T[:3, 0])
            if back:
                out[a] = [q0] + back
        return out

    def _pose_cost(self, arm, q):
        """挑姿勢用的成本（越小越好）：離站姿轉多少 + POSE_W_NOW × 離目前姿勢轉多少 + 靠近關節範圍兩端的扣分。
        主要看離站姿：不管上一個動作停在哪，同一個目標都挑差不多的姿勢（肩膀少轉 = 線少扭），不會越動越歪"""
        lo, hi = self.ik[arm].lower, self.ik[arm].upper
        edge = np.maximum(LIMIT_MARGIN - np.minimum(q - lo, hi - q), 0.0).sum()
        return _travel([STAND_MOVEIT, q]) + POSE_W_NOW * _travel([self.qa(arm), q]) + LIMIT_PENALTY * edge

    def _direct(self, goal, start=None):
        """從目前姿勢（或 start {手: 姿勢}）每顆馬達直接轉到 goal {手: 姿勢}（各軸同時從起點轉到終點，不繞路，線扭最少）。
        每 DIRECT_STEP 檢查一次碰撞；會撞到、實機做不到回傳 None，不然回傳排好時間的軌跡"""
        arms = list(goal)
        names = [j for a in arms for j in ARMS[a]["joints"]]
        q0 = np.concatenate([np.asarray((start or {}).get(a, self.qa(a)), float) for a in arms])
        q1 = np.concatenate([np.asarray(goal[a], float) for a in arms])
        n = max(2, int(np.ceil(np.abs(q1 - q0).max() / DIRECT_STEP)) + 1)
        path = np.linspace(q0, q1, n)
        if any(check(a, path[:, 7 * i:7 * i + 7]) for i, a in enumerate(arms)):
            return None
        group = ARMS[arms[0]]["group"] if len(arms) == 1 else "both_arms"
        if any(self._contacts_q(group, names, q) for q in path[1:]):
            return None
        return self._timed(None, path, np.full(n, MAX_VEL * max(self.speed, 0.02)), names)

    def _plan_route(self, group, constraints, swing_tol=SWING_TOL):
        """MoveIt 規劃，繞遠路（任一軸轉到起點、終點之間的範圍外超過 swing_tol）或路線會撞到東西就重新規劃，
        最多 ROUTE_PLANS 次。回傳 (錯誤碼, 結果, 失敗說明)；繞遠路的錯誤碼是 0、會撞到是 -13"""
        why, code = None, 0
        for _ in range(ROUTE_PLANS):               # MoveIt 每次規劃都是隨機找的：失敗、繞遠路、撞到都再試
            code, res = self._plan(group, constraints, swing_tol)
            if code in (-9999, -9998, -10):         # 連不到 MoveIt、起點就撞到：再試也一樣
                return code, res, err_text(code, res)
            if code == 1:
                return 1, res, None
            why = err_text(code, res)
        return code, None, f"{why}（試了 {ROUTE_PLANS} 次）"

    def _seeds(self, arm, p, a=None, fingers=None, n=N_SEED):
        """IK 起點：目前姿勢、站姿，加上起點表裡掌心位置離 p 近（a：掌心朝向也接近；fingers：手指方向也接近）的 n 個
        （5 cm 的距離 ≈ 30° 的方向差）"""
        lo, hi = self.ik[arm].lower, self.ik[arm].upper
        Q, P, X, Z = self.table[arm]
        score = np.linalg.norm(P - p, axis=1) / 0.05
        if a is not None:
            score += np.arccos(np.clip(X @ a, -1, 1)) / 0.5
        if fingers is not None:
            score += np.arccos(np.clip(Z @ fingers, -1, 1)) / 0.5
        return [np.clip(self.qa(arm), lo, hi), np.clip(STAND_MOVEIT, lo, hi)] + list(Q[np.argpartition(score, n)[:n]])

    def _line(self, arm, q0, p0, p1, a, R=None, hold=None):
        """從 q0（掌心在 p0）直線走到 p1，每 CART_STEP 解一次 IK，每一步都檢查碰撞。
        R = 手掌姿態固定：離 p0 hold（m）以內不讓它轉，再過去只要求掌心朝 a。
            hold 沒給：先試整段都固定，轉不過去（例如預備點比較高，同一個姿態手肘到極限）再試只固定 HOLD_DIST，
            再不行就只有目標點是指定的姿態、一離開就可以轉（目標姿勢已經在關節極限邊上時）
        R = None：只要求掌心朝 a，手掌繞掌心法線可以微調
        回傳 (關節角清單（不含 q0）, None) 或 (None, 原因)"""
        if R is not None and hold is None:
            for h in (np.inf, HOLD_DIST, 0.0):
                out, why = self._line(arm, q0, p0, p1, a, R, h)
                if out is not None:
                    break
            return out, why
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        n = max(1, int(np.ceil(np.linalg.norm(p1 - p0) / CART_STEP)))
        out, q = [], np.asarray(q0, float)
        for k in range(1, n + 1):
            pk = p0 + (p1 - p0) * k / n
            Rk = R if R is not None and np.linalg.norm(pk - p0) <= hold + 1e-6 else None
            qn, ok = self.ik[arm].solve(pk, q, R=Rk, axis=a, q_rest=q, iters=100)
            if not ok:
                return None, "途中碰到關節極限"
            if np.abs(qn - q).max() > MAX_JUMP:
                return None, "途中姿勢會突然翻轉"
            if not self._free(arm, qn):
                return None, "途中會撞到東西"
            out.append(qn)
            q = qn
        return out, None

    def _free(self, arm, q):
        """這隻手擺成 q 會不會撞到東西（場景裡的物體、支架、另一隻手、自己）"""
        return self._contacts(arm, q) == []

    def _contacts(self, arm, q):
        """這隻手擺成 q 會碰到什麼；回傳 ["rh_thumb_distal↔物體 #3", ...]（空的 = 沒碰到；MoveIt 沒回應也當作會碰到）"""
        return self._contacts_q(ARMS[arm]["group"], ARMS[arm]["joints"], q)

    def _contacts_q(self, group, names, q):
        """同 _contacts，關節自己給（兩隻手一起檢查用 group = both_arms、14 個關節）"""
        req = GetStateValidity.Request()
        req.group_name = group
        req.robot_state.is_diff = True
        req.robot_state.joint_state.name = list(names)
        req.robot_state.joint_state.position = [float(v) for v in q]
        res = self._wait(self.valid_cli.call_async(req), 3.0)
        if res is None:
            return ["（MoveIt 沒回應）"]
        if res.valid:
            return []
        return sorted({f"{_contact_name(c.contact_body_1)}↔{_contact_name(c.contact_body_2)}" for c in res.contacts}) or ["?"]

    def _approach_vlim(self, n):
        """直線靠近 n 個點（等距，最後一點 = 目標）每點的速度上限：最後 DECEL_DIST 照 √剩下的距離 減到 0"""
        rem = PRE_DIST * (n - 1 - np.arange(n)) / max(n - 1, 1)
        return MAX_VEL * max(self.speed * SLOW, 0.02) * np.sqrt(np.clip(rem / DECEL_DIST, 0.0, 1.0))

    def _timed(self, arm, path, v_lim, names=None):
        """關節角清單 → 排好時間的軌跡（起點、終點速度 0）。names = 關節名稱（不給就用這隻手的 7 軸）"""
        t, qd, keep = time_path(path, v_lim, MAX_ACC * max(self.speed, 0.02))
        P = np.asarray(path, float)[keep]
        jt = JointTrajectory(joint_names=list(names or ARMS[arm]["joints"]))
        for q, v, ti in zip(P, qd, t):
            jt.points.append(JointTrajectoryPoint(positions=[float(x) for x in q], velocities=[float(x) for x in v],
                                                  time_from_start=_dur(ti)))
        return RobotTrajectory(joint_trajectory=jt)

    def _pre_and_approach(self, arm, pre, appr):
        """「移到預備點」+ 直線靠近 → 一條軌跡（預備點不停）：
        移到預備點用設定的速度，快到預備點時減到直線靠近的速度，靠近的最後 DECEL_DIST 越來越慢、到目標停"""
        q_pre = _traj_q(pre, ARMS[arm]["joints"])
        if q_pre is None:
            return None
        v_fast = MAX_VEL * max(self.speed, 0.02)
        path = np.vstack([q_pre[:-1], appr])        # 預備點的終點 = appr[0]（誤差 1e-3 內），換成 appr[0] 接上去
        v_lim = np.concatenate([np.full(len(q_pre) - 1, v_fast), self._approach_vlim(len(appr))])
        return self._timed(arm, path, v_lim)

    def _wait(self, fut, timeout):
        done = threading.Event()
        fut.add_done_callback(lambda _: done.set())
        return fut.result() if done.wait(timeout) else None

    def _joint_goal(self, joints, q, tol=0.01):
        c = Constraints()
        for name, v in zip(joints, q):
            c.joint_constraints.append(JointConstraint(joint_name=name, position=float(v), tolerance_above=tol,
                                                       tolerance_below=tol, weight=1.0))
        return c

    def _start_contacts(self, group):
        """目前姿勢（這個 group 的部分）碰到什麼；回傳 [(物件1, 物件2)]（空的 = 沒碰到）"""
        req = GetStateValidity.Request()
        req.group_name = group
        req.robot_state.is_diff = True                       # 空的 diff = 用 MoveIt 目前的狀態
        res = self._wait(self.valid_cli.call_async(req), 3.0)
        if res is None or res.valid:
            return []
        return sorted({(c.contact_body_1, c.contact_body_2) for c in res.contacts}) or [("?", "?")]

    def _plan(self, group, constraints, swing_tol=SWING_TOL):
        """只規劃不執行（先檢查再決定要不要動）。回傳 (錯誤碼, 結果)；起點就撞到東西、又沒辦法處理時回傳 (-10, 碰撞清單)
        起點只碰到場景物體（例如手正貼著物體）：這一段先不算那些物體，讓手可以先離開，不然會卡住動不了
        （規劃完的檢查 _plan_checked 也在同一段裡做，一樣不算那些物體）"""
        if not self.move_ac.wait_for_server(timeout_sec=2.0):
            return -9999, None
        contacts = self._start_contacts(group)
        if contacts:                                         # MoveIt 自己只會回「失敗」，先查出是誰碰到誰
            objs = {b for pair in contacts for b in pair if b.startswith("html_") and b != "html_" + FLOOR_ID}
            if not objs or any(not any(b in objs for b in pair) for pair in contacts):
                return -10, [f"{_contact_name(a)}↔{_contact_name(b)}" for a, b in contacts]   # 碰到支架、自己、地板：不能忽略
            names = "、".join(_contact_name(b) for b in sorted(objs))
            self._stage(f"起點碰到{names}，這一段先不算它，讓手先離開")
            self.ignore = {b[5:] for b in objs}
            self._sync_scene(force=True, wait=True)
            try:
                return self._plan_checked(group, constraints, swing_tol)
            finally:
                self.ignore = set()
                self._sync_scene(force=True, wait=True)
        return self._plan_checked(group, constraints, swing_tol)

    def _plan_checked(self, group, constraints, swing_tol):
        """規劃一次再自己檢查：繞遠路 → (0, 說明)；路線上會撞到 → (-13, 碰撞清單)；都沒問題 → (1, 結果)。
        Humble 的 MoveIt 不會檢查規劃出來的路線（規劃流程沒有 ValidateSolution），撞到東西的路線也回成功，
        之前直接執行（2026-11-17 站姿 → 預備點穿過桌子 13 cm）。所以跟「每顆馬達直接轉過去」一樣每 DIRECT_STEP 查一次"""
        code, res = self._plan_once(group, constraints)
        if code != 1:
            return code, res
        jt = res.planned_trajectory.joint_trajectory
        P = np.array([p.positions for p in jt.points])
        s = _swing(P)
        if s > swing_tol:
            return 0, f"MoveIt 規劃的路會繞遠路（有一軸多轉 {np.degrees(s):.0f}°）"
        self._stage("檢查 MoveIt 規劃的路線會不會撞到")
        hits = self._route_contacts(group, jt.joint_names, P)
        if hits:
            self.get_logger().warn("MoveIt 規劃的路線會撞到（不執行，重新規劃）：" + "、".join(hits))
            return -13, hits
        return 1, res

    def _route_contacts(self, group, names, P):
        """關節角清單（N × len(names)）每 DIRECT_STEP 檢查一次碰撞（起點不查：起點碰到的情況 _plan 已經處理）。
        回傳第一個撞到的地方碰到什麼（空的 = 整條都沒撞到）。MoveIt 的軌跡點很密：離上一個檢查點不到 DIRECT_STEP 的跳過"""
        P = np.asarray(P, float)
        last, pts = P[0], []
        for i in range(1, len(P)):
            d = np.abs(P[i] - last).max()
            if d < DIRECT_STEP and i < len(P) - 1:
                continue
            n = max(1, int(np.ceil(d / DIRECT_STEP)))
            pts += [last + (P[i] - last) * k / n for k in range(1, n + 1)]
            last = P[i]
        for q in pts:
            hits = self._contacts_q(group, names, q)
            if hits:
                return hits
        return []

    def _plan_once(self, group, constraints):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name, r.num_planning_attempts, r.allowed_planning_time = group, 5, PLAN_TIME
        r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = self.speed
        r.start_state.is_diff = True
        r.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        gh = self._wait(self.move_ac.send_goal_async(goal), 5.0)
        if gh is None or not gh.accepted:
            return -9998, None
        res = self._wait(gh.get_result_async(), PLAN_TIME * 5 + 10)
        return (-6, None) if res is None else (res.result.error_code.val, res.result)

    def _exec(self, traj, label):
        if self.stop_req:                           # /arm_stop 之後，這個動作剩下的段都不執行
            self._note(f"{label}：已經收到 /arm_stop，不執行", ok=False, reason=R_CANCEL)
            return False
        self._stage(label)
        gh = self._wait(self.exec_ac.send_goal_async(ExecuteTrajectory.Goal(trajectory=traj)), 5.0)
        if gh is None or not gh.accepted:
            self._note(f"{label}：MoveIt 沒接受執行", ok=False, reason=R_EXEC)
            return False
        self.exec_gh = gh
        if self.stop_req:                           # 送出去的同時收到 /arm_stop（那時 exec_gh 還沒設）
            gh.cancel_goal_async()
        try:
            res = self._wait(gh.get_result_async(), 120.0)
        finally:
            self.exec_gh = None
        code = -6 if res is None else res.result.error_code.val
        if self.stop_req:
            self._note(f"{label}：被 /arm_stop 取消（模擬停在目前的位置）", ok=False, reason=R_CANCEL)
            return False
        if code != 1:
            self._note(f"{label}執行失敗：{err_text(code)}", ok=False, reason=R_EXEC)
            return False
        return True

    def _exec_checked(self, traj, label):
        paths = {a: p for a in ARMS if (p := _traj_q(traj, ARMS[a]["joints"])) is not None}
        if any(check(a, p) for a, p in paths.items()):
            self._report(list(paths), f"{label}：MoveIt 的解實機做不到，不執行", paths=paths)
            return False
        return self._exec(traj, label)

    # ---------- 回報 ----------
    def _stage(self, text):
        self.status_pub.publish(String(data=json.dumps({"stage_only": True, "stage": text}, ensure_ascii=False)))

    def _note(self, text, ok=True, arms=tuple(ARMS), reason=None, **extra):
        """結果（網頁 IK 面板的文字、API 的回覆），附這幾隻手目前的關節角、刻度。
        cmd（move / stand / attach / detach；None = 不是指令的結果，例如停住後的「MoveIt 目前姿勢」）、
        arm（實際用的那隻手；兩隻都不行、回站姿 = None）預設取執行中的動作（self.ctx），extra 可以蓋過；
        reason：失敗時的分類（R_*），成功是 None"""
        q_deg, ticks = [], {}
        for a in arms:
            q = self.qa(a)
            q_deg += [round(float(v), 1) for v in np.degrees(q)]
            ticks.update({str(m): t for m, t in rad_to_ticks(a, q).items()})
        uncal = [ARMS[a]["label"] for a in arms if not ARMS[a]["calibrated"]]
        if uncal:
            text += f"\n（{'、'.join(uncal)}刻度未校正：實機跟隨不會送這隻手）"
        d = {"ok": ok, "note": text, "q_deg": q_deg, "ticks": ticks, **self.ctx,
             "reason": None if ok else (reason or R_INTERNAL)}
        d.update(extra)
        self.status_pub.publish(String(data=json.dumps(d, ensure_ascii=False)))
        text = text.replace("\n", "｜")
        if ok:                         # rclpy 不准同一行程式用不同等級 log，所以分兩行寫
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)

    def _report(self, arms, title, paths=None):
        warn, lines = [], [title]
        for a in arms:
            q = self.qa(a) if paths is None else paths[a][-1]
            warn += check(a, q if paths is None else paths[a])
            p = self.fk[a](q)[:3, 3]
            lines.append(f"{ARMS[a]['label']}掌心（模擬）前 {{:.1f}}、右 {{:.1f}}、高 {{:.1f}} cm".format(
                *(np.round(to_meas(p), 1) + 0.0)))         # + 0.0：-0.0 → 0.0
        lines += ["實機可以照做"] if not warn else ["⚠ " + w for w in warn]
        self._note("\n".join(lines), ok=not warn, arms=arms, reason=R_LIMIT if warn else None)


def main():
    rclpy.init()
    node = MoveItBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
