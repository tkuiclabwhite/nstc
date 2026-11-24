"""成果展示（pose_demo）的姿態序列與規劃。只用 numpy（IK 由外面傳進來，armpackage kinematics.ChainIK 的介面），方便單元測試。

預設（ring，2026-11-24 晚上使用者選的）：手離藥瓶表面 gap、手指一直朝前，正上方 → 傾 30° 繞一圈（8 個方向）
    → 傾 45° 再繞一圈 → 往外、往內傾 60° → 回正上方（ring_sequence；左右環繞只有 4 個到達點太少）。
左右環繞（sides，2026-11-24 下午使用者改的）：手離藥瓶表面 gap（2 cm，用 URDF 的手部外形算掌心要停在哪），
    正上方（掌心朝下、手指朝前）→ 往外側傾 30°、45° → 測試：45° 時手指朝上、朝下 → 回正上方
    → 往內側傾 30°、45° → 測試：手指朝上、朝下 → 回正上方。
    手指固定（朝前 = 往前方投影到手掌面；朝上／朝下同理）。不是測試的步手會碰到桌子等就自動離瓶子遠一點（每次 0.5 cm）；
    測試步照 gap 算、一定送給 moveit_bridge，讓它判斷會不會撞到（多半是手腕轉不到 → 到不了）。
以下是舊的、比較細的版本（spin / orbit，掌心離瓶頂中心固定 radius）：

座標都是 base_link（m）：X 前、Y 左、Z 上。手掌姿態 R 的三欄 = [掌心朝向, 手掌橫向, 手指方向]
（同 moveit_bridge /arm_ik_target：手掌座標 +X = 掌心朝向、+Z = 手指方向）。
手指朝向（度）的定義同 moveit_bridge finger_ref：0° = 手指盡量朝前（掌心朝前時改朝上），
正的 = 往外（右手往右、左手往左），往外不在手掌面上時改往上。

    focus = 藥瓶瓶頂中心
    起點：掌心在 focus 正上方 radius、掌心朝下
    原地（spin）：位置不動、只換姿態
        1. 手掌繞掌心法線轉：手指從中間往外轉、往內轉、回中間（幅度看手指轉得到的範圍，每步最多 25°）
        2. 掌心傾斜 tilt 度，傾斜的方向繞一圈（前 → 外 → 後 → 內 → 前）
    繞（orbit）：掌心在 focus 外 radius 的球面上、一直對著 focus
        從正上方往外側傾到 tilt，水平繞過前方（遠離身體那側）到內側，再回正上方。
        靠身體那側不繞：手肘最多彎 80°，掌心要退到藥瓶和身體之間時多半到不了（2026-11-24 離線算過）

手指為什麼不固定：掌心朝下時，手指能轉的範圍每個位置都不一樣，常常只有 30～90°（例如右肩正前方、離桌緣 12 cm
只轉得到「朝內 65～80°」），固定朝前幾乎都到不了。所以 plan() 用 IK 先算：起點取轉得到的範圍的中間，
之後每一步盡量維持手指角度（轉不到就就近調整）。每一步離上一步不到 10 cm、轉不到 30°、每軸轉不到 45°，
moveit_bridge 就會「微調」（從目前姿勢直接轉過去，不退到預備點），看起來是連續的。
"""
from dataclasses import dataclass, field

import numpy as np

SIDE = {'right': -1.0, 'left': 1.0}     # 外側在 base_link Y 的哪一邊（同 armpackage arm_config ARMS 的 side）
UP = np.array([0.0, 0.0, 1.0])
FWD = np.array([1.0, 0.0, 0.0])

# moveit_bridge 的「微調」門檻（PRE_DIST、SMALL_ROT、SMALL_JOINT），規劃時留一點餘裕
STEP_MAX_M = 0.10
STEP_MAX_DEG = 28.0
JUMP_MAX_DEG = 42.0
YAW_STEP_DEG = 25.0     # 原地轉手掌每一步最多轉幾度（幅度規劃出來之後才知道，fit_yaw 依幅度重排）

AZ_NAME = {0: '前方', 90: '外側', 180: '靠身體', 270: '內側'}
TILT_NAME = {0: '前', 90: '外', 180: '後', 270: '內'}


@dataclass
class Step:
    part: str                   # 'start'（起點）/ 'spin'（原地換姿態）/ 'orbit'（繞著藥瓶）
    label: str                  # 給人看的說明
    pos: np.ndarray             # 掌心位置（base_link，m）
    axis: np.ndarray            # 掌心朝向（單位向量）
    yaw: float = None           # 原地轉手掌：手指相對起點轉多少（−1～1，乘上規劃出來的幅度）；None = 跟著上一步
    fingers_want: float = None  # 手指固定這個角度（sides 用，finger_ref 定義）；None = 照 yaw / 跟著上一步
    test: bool = False          # 測試步：預測到不了、會撞到也照樣送給 moveit_bridge，看它怎麼判斷
    alts: list = None           # 候選位置 [(手離藥瓶 m, 掌心位置)]：依序試，第一個是要的距離，後面越來越遠（sides）
    info: str = ''              # 排序列時的說明（plan 不會清掉）
    # 以下 plan() 填
    ok: bool = False
    how: str = ''               # 'small'（微調，連續）/ 'general'（moveit_bridge 會先退到預備點再靠近）
    R: np.ndarray = None        # 手掌姿態
    q: np.ndarray = None        # 預測的關節角（rad）
    fingers: float = None       # 手指朝向（度，finger_ref 定義）
    clear_cm: float = None      # 手離場景物體最近多遠（cm，有給 clearance 才有）
    note: str = ''
    extra: dict = field(default_factory=dict)


class PlanError(Exception):
    pass


# ----------------------------------------------------------------------
# 姿態的幾何
# ----------------------------------------------------------------------
def _plane_dir(v, *normals):
    """v 去掉沿 normals（單位向量）的分量後的單位向量；剩太少（< 0.3）回傳 None（同 moveit_bridge）。"""
    v = np.asarray(v, float).copy()
    for n in normals:
        v -= n * (n @ v)
    nv = np.linalg.norm(v)
    return v / nv if nv > 0.3 else None


def finger_ref(arm, a):
    """掌心朝 a 時，手指朝向的基準 (a, 0° 的手指方向, 正的轉向)，同 moveit_bridge finger_ref。"""
    a = np.asarray(a, float) / np.linalg.norm(a)
    z0 = _plane_dir(FWD, a)
    if z0 is None:
        z0 = _plane_dir(UP, a)
    w = _plane_dir([0.0, SIDE[arm], 0.0], a, z0)
    if w is None:
        w = _plane_dir(UP, a, z0)
    return a, z0, w


def palm_R(arm, a, fingers_deg=0.0):
    """掌心朝 a、手指朝向 fingers_deg 度的手掌姿態（同 moveit_bridge palm_R_fingers）。"""
    a, z0, w = finger_ref(arm, a)
    th = np.radians(fingers_deg)
    z = np.cos(th) * z0 + np.sin(th) * w
    return np.column_stack([a, np.cross(z, a), z])


def finger_angle(arm, a, R):
    """手掌姿態 R 的手指朝向（度，finger_ref 定義；a = 掌心朝向）。"""
    a, z0, w = finger_ref(arm, a)
    z = np.asarray(R)[:, 2]
    return float(np.degrees(np.arctan2(z @ w, z @ z0)))


def rot_deg(R1, R2):
    """兩個姿態差幾度。"""
    c = (np.trace(np.asarray(R1).T @ np.asarray(R2)) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def quat(R):
    """旋轉矩陣 → quaternion (x, y, z, w)。"""
    R = np.asarray(R, float)
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def horiz(arm, az_deg):
    """水平方向：az 0 = 前方（遠離身體）、90 = 外側、180 = 靠身體、270（−90）= 內側。"""
    t = np.radians(az_deg)
    return np.cos(t) * FWD + np.sin(t) * np.array([0.0, SIDE[arm], 0.0])


def _name(table, az):
    az = round(az) % 360
    return table.get(az, f'{az}°方向')


def _steps(a, b, step):
    """a → b（不含 a、含 b），每步不超過 step。"""
    n = max(1, int(np.ceil(abs(b - a) / step - 1e-9)))
    return [a + (b - a) * i / n for i in range(1, n + 1)]


# ----------------------------------------------------------------------
# 序列（只定掌心位置、朝向；手指由 plan() 決定）
# ----------------------------------------------------------------------
def yaw_fracs(n):
    """原地轉手掌：0 → +1 → −1 → 0（幅度的幾分之幾），每 1/n 一步。"""
    k = 1.0 / n
    return _steps(0.0, 1.0, k) + _steps(1.0, -1.0, k) + _steps(-1.0, 0.0, k)


def spin_steps(arm, focus, radius, tilt=30.0, tilt_step=15.0, az_step=30.0):
    """原地換姿態：掌心停在 focus 正上方 radius（m）。原地轉手掌先排一個大概（幅度未知），plan() 的 fit_yaw 依幅度重排。"""
    p = np.asarray(focus, float) + UP * radius
    out = []
    for y in yaw_fracs(2):
        out.append(Step('spin', '原地轉手掌', p, -UP, yaw=y))
    if tilt > 0:
        def tilted(t, az):
            return -np.cos(np.radians(t)) * UP + np.sin(np.radians(t)) * horiz(arm, az)
        for t in _steps(0.0, tilt, tilt_step):
            out.append(Step('spin', f'原地傾斜：掌心往前傾 {t:.0f}°', p, tilted(t, 0.0)))
        for az in _steps(0.0, 360.0, az_step):
            out.append(Step('spin', f'原地傾斜：掌心往{_name(TILT_NAME, az)}傾 {tilt:.0f}°', p, tilted(tilt, az)))
        for t in _steps(tilt, 0.0, tilt_step):
            out.append(Step('spin', '原地傾斜：回到掌心朝下' if t < 1e-6 else f'原地傾斜：掌心往前傾 {t:.0f}°',
                            p, tilted(t, 0.0)))
    return out


def orbit_point(arm, focus, radius, el, az):
    """掌心在 focus 外 radius 的球面上（仰角 el：90 = 正上方；方位 az 見 horiz）→ (位置, 掌心朝向 = 對著 focus)。"""
    d = np.cos(np.radians(el)) * horiz(arm, az) + np.sin(np.radians(el)) * UP
    return np.asarray(focus, float) + radius * d, -d


def orbit_steps(arm, focus, radius, tilt=45.0, el_step=15.0, az_from=90.0, az_to=-90.0, az_step=15.0):
    """繞著 focus：從正上方往 az_from 傾到 tilt（離垂直的角度），水平繞到 az_to，再回正上方。
    預設 90 → −90 = 外側 → 前方 → 內側。"""
    el_low = 90.0 - tilt
    out = []

    def add(el, az, what):
        p, a = orbit_point(arm, focus, radius, el, az)
        out.append(Step('orbit', what, p, a))

    for el in _steps(90.0, el_low, el_step):
        add(el, az_from, f'繞：往{_name(AZ_NAME, az_from)}傾 {90 - el:.0f}°')
    for az in _steps(az_from, az_to, az_step):
        add(el_low, az, f'繞：{_name(AZ_NAME, az)}（傾 {tilt:.0f}°）')
    for el in _steps(el_low, 90.0, el_step):
        add(el, az_to, '繞：回到正上方' if abs(el - 90) < 1e-6 else f'繞：往{_name(AZ_NAME, az_to)}傾 {90 - el:.0f}°')
    return out


FINGER_DIRS = {'fwd': ('朝前', FWD), 'up': ('朝上', UP), 'down': ('朝下', -UP)}


def fingers_R(arm, a, kind):
    """掌心朝 a、手指朝 kind（fwd / up / down：那個方向投影到手掌面上）的手掌姿態。投影太短時退回手指 0°。"""
    a = np.asarray(a, float) / np.linalg.norm(a)
    z = _plane_dir(FINGER_DIRS[kind][1], a)
    if z is None:
        return palm_R(arm, a, 0.0)
    return np.column_stack([a, np.cross(z, a), z])


def hand_in_palm(shape, arm, joints, seed=0):
    """手（最後一軸之後、跟手掌一起動的 link：手腕最後一節、手掌、伸直的手指）的頂點，手掌座標（原點 = 掌心點，
    軸 = 手掌 link 的軸：+X 掌心朝向、+Z 手指）。shape = arm_shape.ArmShape（要讀 STL）。
    用兩組隨機關節角找「相對手掌 link 不動」的 link。"""
    from sp3_task_manager.arm_shape import ARM_PREFIX, HAND_BASE, TOOL_OFFSET
    rng = np.random.default_rng(seed)
    qa = dict(zip(joints, rng.uniform(-1.5, 1.5, len(joints))))
    qb = dict(zip(joints, rng.uniform(-1.5, 1.5, len(joints))))
    hb = HAND_BASE[arm]

    def rel(link, q):
        return np.linalg.inv(shape.link_T(hb, q)) @ shape.link_T(link, q)
    pts = []
    for link, v in shape.meshes.items():
        if not link.startswith(ARM_PREFIX[arm]):
            continue
        Ta = rel(link, qa)
        if np.allclose(Ta, rel(link, qb), atol=1e-6):
            pts.append(v @ Ta[:3, :3].T + Ta[:3, 3])
    if not pts:
        raise ValueError('URDF 裡找不到手的外形')
    return np.vstack(pts) - TOOL_OFFSET


def hand_world(hand, p, R):
    """手掌座標的點 → base_link：掌心在 p、姿態 R。"""
    return np.asarray(hand) @ np.asarray(R).T + np.asarray(p)


def standoff(focus, d, R, hand, obj, gap, hi=0.4, iters=30):
    """掌心沿 d（從 focus 往外的單位向量）離 focus 多遠（m），手離物體 obj（/arm_ik_obstacles 格式、不加厚）的表面剛好 gap。"""
    from sp3_task_manager.arm_shape import distance
    lo = 0.0
    for _ in range(iters):
        m = (lo + hi) / 2
        if distance(hand_world(hand, np.asarray(focus) + m * np.asarray(d), R), obj)[0] < gap:
            lo = m
        else:
            hi = m
    return hi


def tilt_dir(arm, tilt, az):
    """從 focus 往外的方向：離垂直 tilt 度、往 az（見 horiz）那邊傾。"""
    el = np.radians(90.0 - tilt)
    return np.cos(el) * horiz(arm, az) + np.sin(el) * UP


def _side_step(arm, focus, bottle, hand, gap, part, label, tilt, az, kind, test, extra_step, max_extra):
    """手離藥瓶表面 gap、從 focus 往 tilt_dir(tilt, az) 那邊、掌心對著藥瓶、手指 kind（FINGER_DIRS）的一步。
    不是測試的步多給幾個候選位置（每次離遠 extra_step，最多多 max_extra）：plan() 手會碰到東西時依序往外試。"""
    d = tilt_dir(arm, tilt, az) if tilt else UP
    a = -d
    R = fingers_R(arm, a, kind)
    gaps = [gap] if test else [gap + k * extra_step for k in range(int(round(max_extra / extra_step)) + 1)]
    alts = [(g, np.asarray(focus, float) + standoff(focus, d, R, hand, bottle, g) * d) for g in gaps]
    st = Step(part, label, alts[0][1], a, fingers_want=finger_angle(arm, a, R), test=test, alts=alts)
    st.extra['fingers_kind'] = FINGER_DIRS[kind][0]
    return st


def sides_sequence(arm, focus, bottle, hand, gap, tilts=(30.0, 45.0), finger_test=True, extra_step=0.005,
                   max_extra=0.04):
    """左右環繞（預設）：手離藥瓶表面 gap（m）。bottle = 藥瓶（不加厚）、hand = hand_in_palm 的點。
    不是測試的步多給幾個候選位置（每次離遠 extra_step，最多多 max_extra）：plan() 手會碰到東西時依序往外試。"""
    word = {90: '右' if arm == 'right' else '左', 270: '左' if arm == 'right' else '右'}
    out = []

    def add(part, label, tilt, az, kind, test=False):
        out.append(_side_step(arm, focus, bottle, hand, gap, part, label, tilt, az, kind, test, extra_step, max_extra))

    add('start', '起點：正上方，掌心朝下、手指朝前', 0.0, 0.0, 'fwd')
    for az in (90.0, 270.0):
        for t in tilts:
            add('sides', f'往{word[az]}傾 {t:g}°（手指朝前）', t, az, 'fwd')
        if finger_test:
            t = tilts[-1]
            for kind in ('up', 'down'):
                add('test', f'測試：往{word[az]}傾 {t:g}°、手指{FINGER_DIRS[kind][0]}', t, az, kind, test=True)
        add('sides', '回到正上方（手指朝前）', 0.0, 0.0, 'fwd')
    return out


RING_AZ = (90.0, 45.0, 0.0, 315.0, 270.0, 225.0, 180.0, 135.0)   # 外 → 外前 → 前 → 內前 → 內 → 內後 → 後 → 外後


def ring_words(arm):
    """horiz 的方位角 → 給人看的方向（右手外側 = 右）。"""
    o, i = ('右', '左') if arm == 'right' else ('左', '右')
    return {90.0: o, 45.0: o + '前', 0.0: '前', 315.0: i + '前', 270.0: i, 225.0: i + '後', 180.0: '後（靠身體）',
            135.0: o + '後'}


def ring_sequence(arm, focus, bottle, hand, gap, tilts=(30.0, 45.0), side_tilts=(60.0,), finger_test=False,
                  extra_step=0.005, max_extra=0.04):
    """繞圈（預設，2026-11-24 使用者選的）：手離藥瓶表面 gap（m），手指一直朝前。
    正上方 → 每個 tilts 繞一圈（RING_AZ 8 個方向：外側開始、經過前方、內側、靠身體那側）→ side_tilts 往外、往內各一步
    → （finger_test：45° 外、內側時手指朝上／朝下）→ 回正上方。
    左右環繞（sides）只有 4 個不同的到達點，太少；繞圈在右 18.9、前 28 離線算 17 個（只有 45° 靠身體那側到不了）。"""
    word = ring_words(arm)
    out = []

    def add(part, label, tilt, az, kind, test=False):
        out.append(_side_step(arm, focus, bottle, hand, gap, part, label, tilt, az, kind, test, extra_step, max_extra))

    add('start', '起點：正上方，掌心朝下、手指朝前', 0.0, 0.0, 'fwd')
    for t in tilts:
        for az in RING_AZ:
            add('ring', f'{t:g}° 繞圈：往{word[az]}傾', t, az, 'fwd')
    for t in side_tilts:
        for az in (90.0, 270.0):
            add('ring', f'往{word[az]}傾 {t:g}°', t, az, 'fwd')
    if finger_test:
        t = max(tilts) if tilts else 45.0
        for az in (90.0, 270.0):
            for kind in ('up', 'down'):
                add('test', f'測試：往{word[az]}傾 {t:g}°、手指{FINGER_DIRS[kind][0]}', t, az, kind, test=True)
    add('ring', '回到正上方（手指朝前）', 0.0, 0.0, 'fwd')
    return out


def sequence(arm, focus, radius, parts=('spin', 'orbit'), spin=None, orbit=None, sides=None):
    """整個展示：第一個是起點（藥瓶正上方，掌心朝下），接著 parts 依序。
    parts = ('ring',) / ('sides',)：繞圈（ring_sequence）／左右環繞（sides_sequence），sides = 它們的參數
    bottle、hand、gap…；要單獨用，不能跟 spin / orbit 一起。"""
    for kind, fn in (('ring', ring_sequence), ('sides', sides_sequence)):
        if kind in parts:
            if len(parts) > 1:
                raise ValueError(f'{kind} 要單獨用（不能跟其他 parts 一起）')
            return fn(arm, focus, **(sides or {}))
    p0 = np.asarray(focus, float) + UP * radius
    out = [Step('start', '起點：藥瓶正上方，掌心朝下', p0, -UP)]
    for part in parts:
        if part == 'spin':
            out += spin_steps(arm, focus, radius, **(spin or {}))
        elif part == 'orbit':
            out += orbit_steps(arm, focus, radius, **(orbit or {}))
        else:
            raise ValueError(f'parts 只接受 spin / orbit，收到 {part!r}')
    return out


# ----------------------------------------------------------------------
# 規劃：用 IK 決定手指角度，先知道哪幾步到得了
# ----------------------------------------------------------------------
def _jump(q1, q0):
    return float(np.degrees(np.abs(np.asarray(q1) - np.asarray(q0)).max()))


def _wrap(d):
    return (d + 180.0) % 360.0 - 180.0


class Planner:
    """ik：armpackage kinematics.ChainIK（solve、solve_many、ik(q) = FK 4×4）。
    seeds：IK 初始值（關節角清單），起點、接不過去時用。
    clearance(q) → 手離場景物體最近的距離（m，負的 = 撞到），None = 不檢查。
    sim_pos(p) → 模擬（MoveIt）的掌心目標：實機跟隨開著時 moveit_bridge 把目標往上補 0.17 × 前，
        規劃要用補過的位置，算出來的關節角才會跟 moveit_bridge 一樣（None = 不補）。"""

    def __init__(self, arm, ik, seeds, clearance=None, sim_pos=None, iters=200,
                 yaw_max=60.0, yaw_margin=3.0, search=60.0, search_step=5.0):
        self.arm, self.ik, self.seeds, self.clearance, self.iters = arm, ik, list(seeds), clearance, iters
        self.sim_pos = sim_pos or (lambda p: np.asarray(p, float))
        self.yaw_max, self.yaw_margin = yaw_max, yaw_margin
        self.deltas = [0.0] + [s * d for d in np.arange(search_step, search + 1e-9, search_step) for s in (1.0, -1.0)]
        self.f0 = self.amp = self.lo = self.hi = None

    def _clear(self, q):
        return None if self.clearance is None else self.clearance(q)

    def _free(self, q):
        c = self._clear(q)
        return c is None or c >= 0.0

    def _solve(self, p, R, q0):
        """從 q0 接著解（零空間往 q0 靠）：同 moveit_bridge 微調，解出來的姿勢跟現在接近。"""
        q, conv = self.ik.solve(p, q0, R=R, q_rest=q0, iters=self.iters)
        return q if conv else None

    def _accept(self, q, q_prev, R, R_prev):
        """微調的條件：每軸轉不到 JUMP_MAX_DEG、姿態差不到 STEP_MAX_DEG、不會撞到。回傳 (可以嗎, 離物體最近 m)。"""
        if q is None or _jump(q, q_prev) > JUMP_MAX_DEG or rot_deg(R, R_prev) > STEP_MAX_DEG:
            return False, None
        c = self._clear(q)
        return (c is None or c >= 0.0), c

    def finger_range(self, p, a, q, f, step=5.0, span=180.0):
        """掌心在 p（模擬的目標）、朝 a，從 (q, f) 開始手指一次轉 step 度、連續轉得到的範圍 → (lo, hi, {角度: q})。"""
        qs = {round(f, 3): q}
        lo = hi = f
        for sgn in (1.0, -1.0):
            qq, ff = q, f
            while abs(ff + sgn * step - f) <= span:
                fn = ff + sgn * step
                R0, R1 = palm_R(self.arm, a, ff), palm_R(self.arm, a, fn)
                qn = self._solve(p, R1, qq)
                if not self._accept(qn, qq, R1, R0)[0]:
                    break
                qq, ff = qn, fn
                qs[round(ff, 3)] = qq
            if sgn > 0:
                hi = ff
            else:
                lo = ff
        return lo, hi, qs

    def _set_yaw(self, step, f0, lo, hi):
        """原地轉手掌的幅度：從起點的手指角度往兩邊都轉得到、留 yaw_margin（範圍是每 5° 試出來的，邊界本身解得到），
        取 5° 的倍數，不到 10° 就不做。"""
        amp = float(np.floor(min(self.yaw_max, hi - f0 - self.yaw_margin, f0 - lo - self.yaw_margin) / 5.0) * 5.0)
        self.f0, self.lo, self.hi = f0, lo, hi
        self.amp = amp if amp >= 10.0 else 0.0
        step.extra.update(lo=lo, hi=hi, yaw_amp=self.amp)

    def start(self, step, q_from, candidates=4):
        """起點。手指固定（fingers_want，sides）：照候選位置依序解，挑離 q_from 近的。
        沒固定：只對掌心朝向解好幾組，挑手指轉得最開的那組，手指放在範圍中間。"""
        if step.fingers_want is not None:
            seeds = [np.asarray(q_from, float)] + self.seeds
            for exact in (True, False):         # 先要手指剛好朝前；轉不到再改成最接近的角度
                for gap_m, pos in (step.alts or [(None, step.pos)]):
                    q, f, why = self._fixed_fingers(step, pos, seeds, q_from, exact)
                    if why == 'ik':
                        break               # 解不到：離瓶子更遠（掌心更高）也不會比較好解
                    if q is None:
                        continue
                    step.pos = pos
                    step.ok, step.how, step.q, step.fingers = True, 'general', q, f
                    step.R = palm_R(self.arm, step.axis, f)
                    step.clear_cm = None if self._clear(q) is None else self._clear(q) * 100
                    self._note_gap(step, gap_m)
                    self._note_fingers(step)
                    self._set_yaw(step, f, f, f)
                    return
            raise PlanError('起點（藥瓶正上方、掌心朝下）手指轉成什麼角度都到不了或會撞到：'
                            '藥瓶離身體太近／太遠（建議擺 右 18.9～30、前 28），或 --gap 太大')
        p = self.sim_pos(step.pos)
        sols = [q for q in self.ik.solve_many(p, [np.asarray(q_from, float)] + self.seeds, axis=step.axis,
                                              iters=self.iters) if self._free(q)]
        if not sols:
            raise PlanError('起點（藥瓶正上方）到不了或會撞到：藥瓶離身體太近／太遠，或 --radius 太大')
        best = None
        for q in sorted(sols, key=lambda q: _jump(q, q_from))[:candidates]:
            f = finger_angle(self.arm, step.axis, self.ik(q)[:3, :3])
            q_f = self._solve(p, palm_R(self.arm, step.axis, f), q)
            if q_f is None:
                continue
            lo, hi, qs = self.finger_range(p, step.axis, q_f, f)
            if best is None or hi - lo > best[1] - best[0] + 1e-6:
                best = (lo, hi, qs)
        if best is None:
            raise PlanError('起點解得到掌心朝下，但固定手指角度時都解不到')
        lo, hi, qs = best
        mid = float(np.round((lo + hi) / 2.0))
        k = min(qs, key=lambda a: abs(a - mid))
        R = palm_R(self.arm, step.axis, mid)
        q = self._solve(p, R, qs[k])
        if q is None:
            mid, q, R = float(k), qs[k], palm_R(self.arm, step.axis, k)
        step.ok, step.how, step.R, step.q, step.fingers = True, 'general', R, q, mid
        step.clear_cm = None if self._clear(q) is None else self._clear(q) * 100
        step.note = f'手指轉得到 {lo:+.0f}～{hi:+.0f}°，放中間 {mid:+.0f}°'
        self._set_yaw(step, mid, lo, hi)

    def at_start(self, step, q, need_range=True):
        """起點已經到了，q = 實際的關節角（moveit_bridge 挑的姿勢可能跟規劃的不一樣）。
        need_range：有原地轉手掌時從 q 重算手指範圍（AGX 上要好幾秒，沒有就不算）。"""
        f = finger_angle(self.arm, step.axis, self.ik(q)[:3, :3])
        step.q, step.fingers, step.R = np.asarray(q, float), f, palm_R(self.arm, step.axis, f)
        if not need_range:
            self._set_yaw(step, f, f, f)
            return
        lo, hi, _ = self.finger_range(self.sim_pos(step.pos), step.axis, q, f)
        step.note = f'手指轉得到 {lo:+.0f}～{hi:+.0f}°（實際的姿勢）'
        self._set_yaw(step, f, lo, hi)

    def _fixed_fingers(self, st, pos, seeds, q_ref, exact):
        """掌心在 pos、手指 st.fingers_want：exact = 剛好這個角度（解好幾組挑離 q_ref 近的）；
        否則只對掌心朝向解一次，看轉得到哪些手指角度，挑離 fingers_want 最近、不會撞到的。
        回傳 (q, 手指角度, '') 或 (None, None, 'ik' 解不到 / 'collision' 解得到但會撞到)。"""
        p = self.sim_pos(pos)
        if exact:
            R = palm_R(self.arm, st.axis, st.fingers_want)
            sols = self.ik.solve_many(p, seeds, R=R, iters=self.iters, max_sols=8)
            free = [q for q in sols if self._free(q)]
            if free:
                return min(free, key=lambda q: _jump(q, q_ref)), float(st.fingers_want), ''
            return None, None, ('collision' if sols else 'ik')
        sols = self.ik.solve_many(p, seeds, axis=st.axis, iters=self.iters)

        def fa(q):
            return finger_angle(self.arm, st.axis, self.ik(q)[:3, :3])
        for q in sorted(sols, key=lambda q: abs(_wrap(fa(q) - st.fingers_want)))[:6]:
            f = st.fingers_want + float(np.round(_wrap(fa(q) - st.fingers_want)))
            q2 = self._solve(p, palm_R(self.arm, st.axis, f), q)
            if q2 is not None and self._free(q2):
                return q2, f, ''
        return None, None, ('collision' if sols else 'ik')

    @staticmethod
    def _note_fingers(st):
        """手指沒照要的角度（fingers_want）：說明改成往哪偏幾度。"""
        if st.fingers_want is None or st.fingers is None:
            return
        d = _wrap(st.fingers - st.fingers_want)
        if abs(d) > 0.5:
            kind = st.extra.get('fingers_kind', '')
            txt = f'手指{kind}轉不到，往{"外" if d > 0 else "內"}偏 {abs(d):.0f}°'
            st.note = f'{st.note}；{txt}' if st.note else txt

    @staticmethod
    def _note_gap(st, gap_m):
        if gap_m is None:
            return
        st.extra['gap_cm'] = gap_m * 100
        want = st.alts[0][0] if st.alts else gap_m
        if gap_m > want + 1e-9:
            st.note = f'離瓶子 {want * 100:g} cm 會撞到，改離 {gap_m * 100:.1f} cm'

    def _general(self, st, last, f_want, exact):
        """微調接不過去：找一組到得了的姿勢（手指盡量接近 f_want），moveit_bridge 會退到預備點再靠近。
        回傳 (q, 手指角度, 為什麼不行)：到不了 = 'ik'、解得到但都會撞到 = 'collision'。"""
        p = self.sim_pos(st.pos)
        seeds = [last.q] + (self.seeds[:10] if st.test else self.seeds)   # 測試步只是預測（照樣送），少試一點
        if exact:       # 找到 8 組就夠挑了（AGX 上 IK 慢）
            sols = self.ik.solve_many(p, seeds, R=palm_R(self.arm, st.axis, f_want), iters=self.iters, max_sols=8)
        else:
            sols = self.ik.solve_many(p, seeds, axis=st.axis, iters=self.iters)
        free = [q for q in sols if self._free(q)]
        if not free:
            return None, None, ('collision' if sols else 'ik')
        if exact:
            return min(free, key=lambda q: _jump(q, last.q)), f_want, ''

        def fa(q):
            return finger_angle(self.arm, st.axis, self.ik(q)[:3, :3])
        q = min(free, key=lambda q: abs(_wrap(fa(q) - f_want)))
        f = f_want + float(np.round(_wrap(fa(q) - f_want)))
        q2 = self._solve(p, palm_R(self.arm, st.axis, f), q)
        return (q2, f, '') if q2 is not None else (q, f_want + _wrap(fa(q) - f_want), '')

    def fit_yaw(self, steps):
        """原地轉手掌那段依幅度重排（直接改 steps）：每步不超過 YAW_STEP_DEG；不做（幅度 0）就只留一步標「不做」。
        start() / at_start() 之後、plan_rest() 之前叫。"""
        idx = [i for i, s in enumerate(steps) if s.yaw is not None]
        if not idx:
            return steps
        i0, i1 = idx[0], idx[-1] + 1
        tmpl = steps[i0]
        fr = yaw_fracs(max(1, int(np.ceil(self.amp / YAW_STEP_DEG - 1e-9)))) if self.amp > 0 else [0.0]
        steps[i0:i1] = [Step(tmpl.part, tmpl.label, tmpl.pos, tmpl.axis, yaw=y) for y in fr]
        return steps

    def plan(self, steps, q_from):
        """填好每一步的 R、q、fingers、ok、how。steps[0] 要是起點。到不了的那一步 ok = False（執行時略過）。
        原地轉手掌那段會依幅度重排（steps 的長度可能會變）。"""
        self.start(steps[0], q_from)
        self.fit_yaw(steps)
        return self.plan_rest(steps, 1, steps[0])

    def plan_rest(self, steps, i0, last):
        """從 last（ok 的那一步；執行中 = 剛到的那一步、q 換成實際的關節角）接著規劃 steps[i0:]。
        有候選位置（alts）的步：先每個位置試微調，都不行再每個位置試一般走法；都不行 = 到不了（測試步也是，但執行時照樣送）。"""
        for st in steps[i0:]:
            st.ok, st.how, st.note, st.R, st.q, st.fingers, st.clear_cm = False, '', '', None, None, None, None
            st.extra.pop('predict', None)
            exact = True
            if st.yaw is not None:
                if self.amp <= 0.0:
                    st.note = f'手指只轉得到 {self.lo:+.0f}～{self.hi:+.0f}°，不做原地轉手掌'
                    continue
                want = self.f0 + st.yaw * self.amp
                d = want - self.f0
                st.label = ('原地轉手掌：回到起點的手指方向' if abs(d) < 1e-6 else
                            f'原地轉手掌：手指往{"外" if d > 0 else "內"}轉 {abs(d):.0f}°')
                cands = [want]
            elif st.fingers_want is not None:
                cands = [st.fingers_want]
            else:
                cands, exact = [last.fingers + d for d in self.deltas], False     # 盡量維持上一步的手指角度
            alts = st.alts or [(None, st.pos)]
            for gap_m, pos in alts:                 # 1. 微調（直接轉過去）
                if np.linalg.norm(pos - last.pos) > STEP_MAX_M:
                    continue
                p = self.sim_pos(pos)
                for f in cands:
                    R = palm_R(self.arm, st.axis, f)
                    q = self._solve(p, R, last.q)
                    ok, c = self._accept(q, last.q, R, last.R)
                    if ok:
                        st.pos = pos
                        st.ok, st.how, st.R, st.q, st.fingers = True, 'small', R, q, float(f)
                        st.clear_cm = None if c is None else c * 100
                        self._note_gap(st, gap_m)
                        break
                if st.ok:
                    break
            if not st.ok:                           # 2. 一般走法（moveit_bridge 先退到預備點再靠近）
                why = 'ik'
                for gap_m, pos in alts:
                    st.pos = pos
                    q, f, why1 = self._general(st, last, cands[0], exact=exact)
                    if q is not None:
                        st.ok, st.how, st.q, st.fingers = True, 'general', q, float(f)
                        st.R = palm_R(self.arm, st.axis, f)
                        c = self._clear(q)
                        st.clear_cm = None if c is None else c * 100
                        self._note_gap(st, gap_m)
                        if not st.note:
                            st.note = '從上一步接不過去（要轉太多），moveit_bridge 會先退開 10 cm 再靠近'
                        break
                    why = 'collision' if why1 == 'collision' else why
                    if why1 == 'ik':
                        break               # 解不到（不是撞到）：離瓶子更遠也不會比較好解，不再往外試（AGX 上每次要好幾秒）
                if not st.ok and st.fingers_want is not None and not st.test:   # 3. 手指改最接近的角度
                    for gap_m, pos in alts:
                        q, f, why3 = self._fixed_fingers(st, pos, [last.q] + self.seeds, last.q, exact=False)
                        if why3 == 'ik':
                            break
                        if q is not None:
                            st.pos = pos
                            st.ok, st.how, st.q, st.fingers = True, 'general', q, f
                            st.R = palm_R(self.arm, st.axis, f)
                            c = self._clear(q)
                            st.clear_cm = None if c is None else c * 100
                            self._note_gap(st, gap_m)
                            self._note_fingers(st)
                            break
                if not st.ok:
                    st.pos = alts[0][1]
                    if alts[0][0] is not None:
                        st.extra['gap_cm'] = alts[0][0] * 100
                    st.R = palm_R(self.arm, st.axis, cands[0])      # 測試步照樣送：要有姿態
                    st.fingers = float(cands[0])
                    st.extra['predict'] = why
                    st.note = {'ik': '預測：到不了（關節轉不到）', 'collision': '預測：會撞到東西'}[why] + \
                        ('（測試步照樣送，讓 moveit_bridge 判斷）' if st.test else '：略過')
                    continue
            last = st
        return steps


def step_sizes(steps):
    """每一步（只算 ok 的）離上一個 ok 的多遠（m）、轉幾度；起點回 (nan, nan)。"""
    out, last = [], None
    for st in steps:
        if not st.ok:
            out.append((float('nan'), float('nan')))
            continue
        if last is None:
            out.append((float('nan'), float('nan')))
        else:
            out.append((float(np.linalg.norm(st.pos - last.pos)), rot_deg(st.R, last.R)))
        last = st
    return out
