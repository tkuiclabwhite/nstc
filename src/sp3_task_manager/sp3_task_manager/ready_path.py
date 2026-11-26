"""ready pose 的「錄好的路線」（2026-11-19 使用者要的：ready pose 定了就存起來，之後兩隻手同時走）。不依賴 ROS，方便單元測試。

為什麼存整條路、不是只存 ready 的關節角：從站姿每顆馬達直接轉到 ready，手指會先往前掃進桌子（2026-11-18 測過），
所以要繞路（MoveIt 規劃），而 MoveIt 每次找的路都不一樣、有時找不到（左手約一半失敗）、一次只能動一隻手。
做法：
  錄：手在站姿，用 /arm_move 照舊一隻一隻舉到 ready（moveit_bridge 規劃、每 1.5° 檢查過碰撞），途中記 MoveIt 的 /joint_states，
      左右手各自的路線存成檔案（ready_pose.path_file）。
  走：站姿 → ready = 兩隻手各照自己的路線、按「走了幾成」同步，排成一條 14 軸軌跡一起走（MoveIt /execute_trajectory，
      同 moveit_bridge 回站姿的做法）；ready → 站姿 = 倒過來走。走之前把折線磨圓（smooth，2026-11-26），
      每 1.5° 問 MoveIt 會不會撞到（場景可能變了）：磨圓的會撞到就改走原本的折線，也會撞到就不走、改用原本的方法。
  手不在站姿也不在 ready（例如放完藥瓶）：先每顆馬達直接轉到 ready（一樣檢查碰撞），要回站姿就再接倒過來的路線。
"""
import datetime
import os

import numpy as np
import yaml

ARMS = ('left', 'right')
PREFIX = {'left': 'L', 'right': 'R'}
# 手臂關節名 = 字首 + 這些（armpackage arm_config.JOINT_SUFFIX）
JOINT_SUFFIX = ['shoulderJoint', 'armJoint', 'arm1Joint', 'forarmJoint', 'forarm1Joint', 'wristJoint', 'wrist1Joint']
SAVE_STEP = np.radians(1.0)     # 存檔的點距（最快那一軸轉幾度一點）
CHECK_STEP = np.radians(1.5)    # 走之前碰撞檢查的點距（同 moveit_bridge DIRECT_STEP）
AT_TOL = np.radians(2.0)        # 每軸跟某個姿勢差這麼多以內，算「在那個姿勢」
# 速度換算（同 moveit_bridge）：Dynamixel 速度 1 = 0.229 rpm；joint_limits.yaml max_velocity 2.3824、max_acceleration 1.0，
# 加速度跟著速度倍率縮（MoveIt 的 acceleration scaling）
RAD_PER_SEC_PER_SPEED = 0.229 * 2 * np.pi / 60
MAX_VEL = 2.3824
MAX_ACC = 1.0
# 照路線走之前先磨圓、排時間時轉角不要每個都慢下來（2026-11-26 使用者：實機 Stand↔Ready「小卡卡的」）。
# 錄到的是 MoveIt 規劃的折線，轉角很多；原本每個轉角都減速，速度 50 時加速度只有 0.5 rad/s²，一趟 8.7 秒
# 掌心忽快忽慢 6 次（0→25→9→22→13→32→21→26→6 cm/s…），從來沒有定速；最快一軸只到 0.51 rad/s（上限 1.2）。
# 實機錄的（motion_probe）：馬達一直在轉（速度 = 平均的 0.9 以上、沒有停頓），所以不是馬達 Profile 的問題，是軌跡本身。
# 離線算（11/26 的路線）：σ 5° 磨圓 + 轉角加速度 ×2 + 轉角限速前後 15° 一起算 → 7.3 秒、只剩 1 次加減速，
# 掌心離原路線最多 1.1 cm，最快一軸加速度最大 0.99 rad/s²（joint_limits 1.0）。
SMOOTH_DEG = 5.0                    # 磨圓：沿路程的高斯濾波 σ（最快那一軸轉幾度）
SMOOTH_FINE = np.radians(0.25)      # 濾波前先細切成這個點距
READY_TIMING = {'corner_k': 2.0, 'plateau_deg': 15.0}   # 照路線走（站姿 ↔ ready）的排時間方式，見 time_path


def joint_names(arm):
    return [PREFIX[arm] + s for s in JOINT_SUFFIX]


def real_z_comp_cm(rp_cfg, front_cm):
    """實機跟隨開著時 moveit_bridge 把模擬的目標往上補多少 cm（同 moveit_bridge REAL_Z_COMP、REAL_Z_FROM，要一起改）：
    設定檔 ready_pose.real_z_comp ×（前 − real_z_from_cm），前 = 量測座標的「前」（cm）。
    2026-11-26 從 0.17 × 前 改成 0.14 ×（前 − 14.5）：0.17 補太多（前 25 實機高 2.9 cm，見 CLAUDE.md）。"""
    k = float(rp_cfg.get('real_z_comp', 0.14))
    f0 = float(rp_cfg.get('real_z_from_cm', 14.5))
    return k * max(float(front_cm) - f0, 0.0)


def arc(P):
    """路線每一點的累計路程（最快那一軸轉了多少 rad）。"""
    P = np.asarray(P, float)
    if len(P) < 2:
        return np.zeros(len(P))
    return np.concatenate([[0.0], np.cumsum(np.abs(np.diff(P, axis=0)).max(axis=1))])


def resample(P, step=SAVE_STEP):
    """關節角清單（N×k）→ 沿路線每 step 一點（含起點、終點），停著不動的點拿掉。"""
    P = np.asarray(P, float)
    keep = [P[0]]
    for q in P[1:]:
        if np.abs(q - keep[-1]).max() > 1e-5:
            keep.append(q)
    if np.abs(P[-1] - keep[-1]).max() > 0:      # 終點一定是最後收到的那一點
        keep[-1] = P[-1]
    P = np.array(keep)
    s = arc(P)
    if len(P) < 2 or s[-1] <= 0:
        return P[:1]
    n = max(2, int(np.ceil(s[-1] / step)) + 1)
    u = np.linspace(0.0, s[-1], n)
    return np.column_stack([np.interp(u, s, P[:, j]) for j in range(P.shape[1])])


def direct(qa, qb, step=SAVE_STEP):
    """每顆馬達直接從 qa 轉到 qb（各軸同時開始、同時到）。"""
    qa, qb = np.asarray(qa, float), np.asarray(qb, float)
    n = max(2, int(np.ceil(np.abs(qb - qa).max() / step)) + 1)
    return np.linspace(qa, qb, n)


def merge(paths):
    """{手: N×7} → (關節名, M×(7·手數))：每隻手照自己路線的進度（路程走了幾成）一起走，同時開始、同時到。"""
    arms = [a for a in ARMS if a in paths]
    progs = []
    for a in arms:
        P = np.asarray(paths[a], float)
        s = arc(P)
        progs.append((P, s / s[-1] if len(P) > 1 and s[-1] > 0 else None))
    n = max(len(P) for P, _ in progs)
    u = np.linspace(0.0, 1.0, max(n, 2))
    cols = []
    for P, f in progs:
        if f is None:                               # 不動的手：一直停在那裡
            cols.append(np.repeat(P[:1], len(u), axis=0))
        else:
            cols.append(np.column_stack([np.interp(u, f, P[:, j]) for j in range(P.shape[1])]))
    return [n for a in arms for n in joint_names(a)], np.hstack(cols)


def limits(speed):
    """Dynamixel 速度 → (最快那一軸的速度上限 rad/s, 加速度上限 rad/s²)。"""
    v = max(float(speed), 1.0) * RAD_PER_SEC_PER_SPEED
    return v, MAX_ACC * min(v / MAX_VEL, 1.0)


def smooth(P, sigma_deg=SMOOTH_DEG):
    """路線（N×k）沿路程做高斯濾波（σ = sigma_deg，最快那一軸轉的度數），把折線的轉角磨圓。
    起點、終點不動，兩端的方向也不變（兩端用奇對稱延伸）。回傳細切的路線（點距約 SMOOTH_FINE），用之前再 resample。
    磨圓會切轉角，離東西可能近一點，走之前一樣要做碰撞檢查。"""
    P = np.asarray(P, float)
    s = arc(P)
    if len(P) < 3 or s[-1] <= 0 or sigma_deg <= 0:
        return P
    n = int(np.ceil(s[-1] / SMOOTH_FINE)) + 1
    u = np.linspace(0.0, s[-1], n)
    Q = np.column_stack([np.interp(u, s, P[:, j]) for j in range(P.shape[1])])
    sig = min(np.radians(sigma_deg) / (s[-1] / (n - 1)), (n - 1) / 4.0)   # 單位：點；路線太短時 σ 跟著縮
    k = int(np.ceil(4 * sig))
    w = np.exp(-0.5 * (np.arange(-k, k + 1) / sig) ** 2)
    w /= w.sum()
    Z = np.vstack([2 * Q[0] - Q[1:k + 1][::-1], Q, 2 * Q[-1] - Q[-k - 1:-1][::-1]])
    return np.column_stack([np.convolve(Z[:, j], w, mode='valid') for j in range(Z.shape[1])])


def prepare(P):
    """照路線走之前：磨圓、每 CHECK_STEP 一點（碰撞檢查、排時間都用這組點）。"""
    return resample(smooth(P), CHECK_STEP)


def time_path(P, speed, corner_k=1.0, plateau_deg=0.0, accel=None):
    """關節角清單（N×k，點距要細）排時間：起點、終點速度 0，最快那一軸不超過 speed，加減速不超過對應的加速度。
    預設跟 moveit_bridge.time_path 同一個做法（轉角處先慢下來）。回傳 (各點時間 s, 各點關節速度 N×k)。
    corner_k：轉角處允許的加速度是幾倍（不超過 joint_limits 的 MAX_ACC）；plateau_deg：轉角限速往前後各延伸這麼多度、
    取範圍內最慢的，連續幾個轉角時維持定速，不會每個轉角都減速再加速。照路線走用 READY_TIMING（pose_demo 重播用預設）。
    accel：加速度上限（rad/s²），None = 跟著速度縮（limits，MoveIt 的慣例）；給了就不跟速度綁在一起、轉角也用這個。"""
    P = np.asarray(P, float)
    v_max, a_max = limits(speed)
    if accel:
        a_max = float(accel)
    ds = np.maximum(np.abs(np.diff(P, axis=0)).max(axis=1), 1e-9)
    v = np.full(len(P), v_max)
    if len(P) > 2:
        d = np.diff(P, axis=0) / ds[:, None]
        turn = np.abs(np.diff(d, axis=0)).max(axis=1)
        a_turn = min(corner_k * a_max, max(a_max, MAX_ACC))
        v[1:-1] = np.minimum(v[1:-1], np.sqrt(a_turn * np.minimum(ds[:-1], ds[1:]) / np.maximum(turn, 1e-9)))
        if plateau_deg > 0:
            s = np.concatenate([[0.0], np.cumsum(ds)])
            w = np.radians(plateau_deg)
            lo, hi = np.searchsorted(s, s - w, 'left'), np.searchsorted(s, s + w, 'right')
            v = np.array([v[a:b].min() for a, b in zip(lo, hi)])
    v[0] = v[-1] = 0.0
    for i in range(len(P) - 1):                     # 往前：從 0 加速
        v[i + 1] = min(v[i + 1], np.sqrt(v[i] ** 2 + 2 * a_max * ds[i]))
    for i in range(len(P) - 2, -1, -1):             # 往回：來得及減速到下一點
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * a_max * ds[i]))
    vv = v[:-1] + v[1:]
    dt = np.where(vv > 1e-6, 2 * ds / np.maximum(vv, 1e-9), np.maximum(2 * np.sqrt(ds / a_max), 0.05))
    t = np.concatenate([[0.0], np.cumsum(dt)])
    s = np.concatenate([[0.0], np.cumsum(ds)])
    return t, np.gradient(P, s, axis=0) * v[:, None]


def at(q, ref):
    return ref is not None and np.abs(np.asarray(q, float) - np.asarray(ref, float)).max() <= AT_TOL


def plan(rec, q_now, arms, kind):
    """要走的路線 {手: N×7}（已經在終點的手不列）。rec = load() 的結果，q_now = {手: 目前 7 軸}。
    kind = ready：在站姿 → 錄好的路線；在別的地方 → 直接轉到 ready。
    kind = stand：在 ready → 錄好的路線倒過來；在別的地方 → 直接轉到 ready 再倒過來走。"""
    out = {}
    for a in arms:
        P = np.asarray(rec['paths'][a], float)
        start, end, q = P[0], P[-1], np.asarray(q_now[a], float)
        # 第一點一定換成目前的姿勢：MoveIt 執行前會比對軌跡起點跟目前姿勢（差太多不執行），「在那個姿勢」可以差到 AT_TOL
        if kind == 'ready':
            if at(q, end):
                continue
            out[a] = np.vstack([q, P[1:]]) if at(q, start) else direct(q, end)
        else:
            if at(q, start):
                continue
            back = P[::-1]
            out[a] = np.vstack([q, back[1:]]) if at(q, end) else np.vstack([direct(q, end), back[1:]])
    return out


# ----------------------------------------------------------------------
# 只改某隻手的路線（adjust_cm）：實機某隻手跟另一隻不一樣（2026-11-19：左手比右手低約 1 cm，馬達讀數一樣 = 機構），
# 共用同一條路線時那隻手會擦到東西。不用重錄：把那隻手路線上的掌心平移一點（姿態不變），逐點逆解。
#   edge：經過桌角那段才移（路程 EDGE[0]→EDGE[1] 慢慢加上、維持到 EDGE[2]、EDGE[3] 前拿掉），起點、終點不變
#   end ：從 END[0] 起慢慢加上、一直到 ready（ready 也跟著移）
# 2026-11-19 的路線：經過桌角在 31～37%（拇指比桌面高 1.8、在桌緣後 2.9 cm）。那段手臂幾乎伸直往外，
# 往上抬 1 cm 要轉 10～13°（手肘），往後 1 cm 只要 2～5° → 經過桌角時用往後退比較好；終點附近往上抬便宜（1～2°/cm）
# ----------------------------------------------------------------------
EDGE = (0.24, 0.31, 0.45, 0.58)
END = (0.45, 0.60)
ADJ_MAX_JOINT = np.radians(15)   # 改完任一軸跟原本差超過這麼多 = 不對勁，不用
TOOL_OFFSET = np.array([0.02, 0.0, 0.07])   # 掌心在手掌 link 裡的位置（m），同 arm_shape / armpackage arm_config
HAND_BASE = {'right': 'rh_hand_base_link', 'left': 'lh_hand_base_link'}


def _ss(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def weights(u):
    """路程進度 u（0～1）→ (edge 的權重, end 的權重)。"""
    a, b, c, d = EDGE
    edge = _ss((u - a) / (b - a)) * (1 - _ss((u - c) / (d - c)))
    return edge, _ss((u - END[0]) / (END[1] - END[0]))


def palm_pose(shape, arm, q):
    """掌心位置（base_link m）、手掌姿態（3×3）。shape = arm_shape.ArmShape。"""
    T = shape.link_T(HAND_BASE[arm], dict(zip(joint_names(arm), q)))
    return T[:3, :3] @ TOOL_OFFSET + T[:3, 3], T[:3, :3]


def _rotvec(R):
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    ang = np.arccos(c)
    if ang < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w / (2 * np.sin(ang)) * ang


def ik(shape, arm, q0, p_t, R_t, iters=40):
    """掌心到 p_t、手掌姿態 R_t：從 q0 開始的阻尼最小平方（數值 Jacobian）。回傳 (q, 收斂了沒)。"""
    q = np.asarray(q0, float).copy()
    for _ in range(iters):
        p, R = palm_pose(shape, arm, q)
        e = np.concatenate([p_t - p, _rotvec(R_t @ R.T)])
        if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) < 1e-3:
            return q, True
        J = np.zeros((6, 7))
        for j in range(7):
            dq = q.copy()
            dq[j] += 1e-6
            p2, R2 = palm_pose(shape, arm, dq)
            J[:3, j] = (p2 - p) / 1e-6
            J[3:, j] = _rotvec(R2 @ R.T) / 1e-6
        q = q + J.T @ np.linalg.solve(J @ J.T + 1e-6 * np.eye(6), e)
    return q, False


def adjust(P, arm, edge_m, end_m, shape):
    """路線 P（N×7）的掌心平移 edge_m × edge 權重 + end_m × end 權重（base_link m，3 維），姿態不變，逐點逆解。
    做不到（逆解不收斂、超出關節範圍、某軸改太多、相鄰兩點跳太多）丟 ValueError。"""
    P = np.asarray(P, float)
    edge_m, end_m = np.asarray(edge_m, float), np.asarray(end_m, float)
    if len(P) < 2 or (not edge_m.any() and not end_m.any()):
        return P
    s = arc(P)
    we, wn = weights(s / s[-1])
    lim = [shape.limits.get(n, (-np.inf, np.inf)) for n in joint_names(arm)]
    lo, hi = np.array([a for a, _ in lim]), np.array([b for _, b in lim])
    out = [P[0]]
    for i in range(1, len(P)):
        d = edge_m * we[i] + end_m * wn[i]
        if not d.any():
            out.append(P[i])
            continue
        p, R = palm_pose(shape, arm, P[i])
        seed = out[-1] + (P[i] - P[i - 1])                   # 接著上一點的解走，路線才連續
        q, ok = ik(shape, arm, seed, p + d, R)
        where = f'{arm} 路線第 {i} 點（{s[i] / s[-1] * 100:.0f}%）掌心移 {np.round(d * 100, 1).tolist()} cm'
        if not ok:
            raise ValueError(f'{where}：逆解不到')
        if np.any(q < lo - 1e-3) or np.any(q > hi + 1e-3):
            raise ValueError(f'{where}：會超出關節範圍')
        if np.abs(q - P[i]).max() > ADJ_MAX_JOINT:
            raise ValueError(f'{where}：要改 {np.degrees(np.abs(q - P[i]).max()):.0f}°，不對勁')
        if np.abs(q - out[-1]).max() > np.abs(P[i] - P[i - 1]).max() + np.radians(3):
            raise ValueError(f'{where}：跟上一點差太多（逆解跳到別的姿勢）')
        out.append(q)
    return np.array(out)


def apply_adjust(rec, rp_cfg, shape, frames):
    """設定檔 ready_pose.adjust_cm {手: {edge: [前, 右, 上], end: [前, 右, 上]}}（量測座標 cm 的位移）：
    把 load() 的路線改好（回傳新的 rec，不改原本的）。frames = 設定檔 frames（量測座標 ↔ base_link 的方向）。"""
    from sp3_task_manager import grasp_plan as gp
    adj = rp_cfg.get('adjust_cm') or {}
    out = dict(rec, paths=dict(rec['paths']))
    to_m = lambda v: gp.from_meas(np.asarray(v, float), frames) - gp.from_meas(np.zeros(3), frames)   # noqa: E731
    for a in ARMS:
        c = adj.get(a) or {}
        e, n = c.get('edge') or [0, 0, 0], c.get('end') or [0, 0, 0]
        if any(e) or any(n):
            out['paths'][a] = adjust(rec['paths'][a], a, to_m(e), to_m(n), shape)
    return out


def describe(paths, speed, accel=None):
    """給人看：每隻手轉多少度、一起走要幾秒。"""
    names, P = merge(paths)
    t, _ = time_path(prepare(P), speed, accel=accel, **READY_TIMING)
    each = '、'.join(f'{"左" if a == "left" else "右"}手 {np.degrees(arc(paths[a])[-1]):.0f}°' for a in ARMS if a in paths)
    return f'{each}，兩隻手一起走約 {t[-1]:.1f} 秒（速度 {speed:g}）'


# ----------------------------------------------------------------------
# 存檔
# ----------------------------------------------------------------------
def mirror_src(rp_cfg):
    """設定檔 ready_pose.mirror：兩隻手都用哪隻手的路線（None = 各用各的）。左右 ready 不對稱丟 ValueError。
    URDF 左右鏡像（同一組關節角 = 左右相反的姿勢，2026-11-19 用 FK 確認過），所以路線可以直接共用。"""
    src = rp_cfg.get('mirror')
    if src in (None, '', 'none'):
        return None
    if src not in ARMS:
        raise ValueError(f'ready_pose.mirror 只接受 left / right / null，收到 {src!r}')
    l, r = (np.asarray(rp_cfg[a], float) for a in ARMS)
    if not np.allclose(l, r * np.array([-1.0, 1.0, 1.0]), atol=0.05):
        raise ValueError(f'ready_pose.mirror = {src} 時左右 ready 要對稱（左 = 右的「右」取負），現在 左 {l.tolist()}、右 {r.tolist()}')
    return src


def other(arm):
    return 'right' if arm == 'left' else 'left'


def pose_key(rp_cfg):
    """設定檔 ready_pose 裡決定 ready 在哪的欄位：換了位置，舊的路線就不能用。"""
    return {'left': [float(v) for v in rp_cfg['left']], 'right': [float(v) for v in rp_cfg['right']],
            'approach': rp_cfg.get('approach', 'top')}


def save(path_file, paths, rp_cfg, real_follow, z_comp_cm):
    d = {
        'recorded': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'ready_pose': pose_key(rp_cfg),
        'real_follow': bool(real_follow),
        'z_comp_cm': {a: round(float(z_comp_cm.get(a, 0.0)), 2) for a in ARMS},
        'mirror': mirror_src(rp_cfg),
        'joints': {a: joint_names(a) for a in ARMS},
        'paths': {a: [[round(float(v), 5) for v in q] for q in resample(paths[a])] for a in ARMS},
    }
    head = ('# 站姿 → ready pose 的路線（任務管理錄的：網頁「錄 ready 路線」或 /manip/test_cmd {"cmd": "record_ready"}）。\n'
            '# 每一點 = 7 軸關節角（rad，順序見 joints），第一點是站姿、最後一點是 ready。回站姿倒過來走。\n'
            '# real_follow：錄的時候實機跟隨有沒有開；沒開時目標有自己往上補 z_comp_cm（同 moveit_bridge 的高度補償），\n'
            '# 所以關節角跟開實機跟隨時錄的一樣（只看模擬時 ready 會比設定的高這麼多）。\n'
            '# 設定檔 ready_pose 的位置改了，這條路線就不用（任務管理會改回一隻一隻移），要重錄。\n')
    os.makedirs(os.path.dirname(os.path.abspath(path_file)), exist_ok=True)
    tmp = path_file + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(head)
        yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False, default_flow_style=None, width=200)
    os.replace(tmp, path_file)
    return d


def load(path_file, rp_cfg):
    """讀錄好的路線。回傳 (rec, None) 或 (None, 不能用的原因)。"""
    if not path_file or not os.path.exists(path_file):
        return None, f'還沒錄（{path_file} 不存在）'
    try:
        with open(path_file, encoding='utf-8') as f:
            d = yaml.safe_load(f)
        if d.get('ready_pose') != pose_key(rp_cfg):
            return None, (f'路線是 ready pose 在 {d.get("ready_pose")} 時錄的，設定檔現在是 {pose_key(rp_cfg)}，要重錄')
        for a in ARMS:
            if d['joints'][a] != joint_names(a):
                return None, f'{a} 的關節名稱不對：{d["joints"][a]}'
            P = np.asarray(d['paths'][a], float)
            if P.ndim != 2 or P.shape[1] != 7 or len(P) < 1:
                return None, f'{a} 的路線格式不對'
            if not at(P[0], np.zeros(7)):
                return None, f'{a} 的路線不是從站姿開始的'
            d['paths'][a] = P
        src = mirror_src(rp_cfg)
        if src:                                     # 兩隻手都用 src 的路線（舊檔案兩隻手都有錄也照這個）
            d['paths'][other(src)] = d['paths'][src].copy()
    except (OSError, ValueError, KeyError, TypeError, AttributeError, yaml.YAMLError) as e:
        return None, f'讀不了 {path_file}：{e}'
    return d, None
