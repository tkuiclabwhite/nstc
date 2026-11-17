"""由藥瓶位置算出手臂要去的掌心位置。純計算，不依賴 ROS，方便單元測試。

座標：
- 藥瓶辨識輸出 base_link（公尺）。
- 手臂的 /arm_move 吃「實機量測座標」[前, 右, 高]（公分）。
  換算式寫在設定檔 frames（預設值取自 arm_ws README：前 = x·100 + 1.35、右 = −y·100、高 = z·100 + 120.2）。
  兩邊的 base_link 是不是同一個原點由各自負責人確認，這裡只照設定檔換算。

掌心方向（approach）是掌心推過去的方向，用量測座標的單位向量表示：
- top   從上方，掌心朝下
- front 從前方，掌心朝前
- side  從外側，右手往左推、左手往右推
"""
from dataclasses import dataclass

import numpy as np

AXES = {
    'top': {'right': (0.0, 0.0, -1.0), 'left': (0.0, 0.0, -1.0)},
    'front': {'right': (1.0, 0.0, 0.0), 'left': (1.0, 0.0, 0.0)},
    'side': {'right': (0.0, -1.0, 0.0), 'left': (0.0, 1.0, 0.0)},
}


@dataclass
class GraspPoses:
    arm: str
    approach: str
    center: np.ndarray      # 瓶身中心（量測座標，cm）
    pregrasp: np.ndarray    # 預抓位置：抓取位置再往外退
    grasp: np.ndarray       # 抓取位置：掌心離瓶身表面 palm_gap_cm
    lift: np.ndarray        # 抓到後往上抬


def to_meas(xyz_m, frames):
    """base_link（m）→ 量測座標 [前, 右, 高]（cm）。"""
    x, y, z = (float(v) for v in xyz_m)
    return np.array([
        frames['front_sign'] * x * 100.0 + frames['front_offset_cm'],
        frames['right_sign'] * y * 100.0 + frames['right_offset_cm'],
        frames['height_sign'] * z * 100.0 + frames['height_offset_cm'],
    ])


def arm_order(arm, center):
    """要試哪幾隻手。指定的就只試那隻；auto = 藥瓶在右邊（右 ≥ 0）先試右手，不行換左手。"""
    if arm in ('left', 'right'):
        return [arm]
    return ['right', 'left'] if center[1] >= 0 else ['left', 'right']


def table_to_meas(p, table, frames):
    """桌面座標 [右, 前, 上]（cm，原點 = 桌緣上正對 base_link 中線、桌面高）→ 量測座標 [前, 右, 高]（cm）。"""
    edge = to_meas((table['edge_from_base_link_cm'] / 100.0, 0.0, 0.0), frames)
    return np.array([edge[0] + float(p[1]), edge[1] + float(p[0]), table['top_cm'] + float(p[2])])


def meas_to_table(p_cm, table, frames):
    """量測座標 [前, 右, 高]（cm）→ 桌面座標 [右, 前, 上]（cm），table_to_meas 的反過來。"""
    o = table_to_meas((0.0, 0.0, 0.0), table, frames)
    return np.array([float(p_cm[1]) - o[1], float(p_cm[0]) - o[0], float(p_cm[2]) - o[2]])


def reach_front(lateral_cm, ws):
    """離中線往這隻手那一側 lateral_cm 時，掌心最遠能到桌緣往前多少（cm，內插）；表外回 None。"""
    rows = sorted(ws['reach_front_cm'])
    xs, ys = [r[0] for r in rows], [r[1] for r in rows]
    if not xs[0] <= lateral_cm <= xs[-1]:
        return None
    return float(np.interp(lateral_cm, xs, ys))


def reachable_arms(center, cfg):
    """藥瓶中心（量測座標 cm）在哪幾隻手的可及範圍內（從上方抓：掌心在藥瓶正上方，左右、前後同藥瓶中心）。"""
    ws = cfg['workspace']
    right, front, _ = meas_to_table(center, cfg['table'], cfg['frames'])
    if front < ws['front_min_cm']:
        return []
    out = []
    for arm, lateral in (('left', -right), ('right', right)):
        limit = reach_front(lateral, ws)
        if limit is not None and front <= limit:
            out.append(arm)
    return out


def plan(center, size_m, arm, grasp_cfg):
    """center：量測座標（cm）；size_m：(直徑, 直徑, 高)（m，Detection3D 的 bbox.size）。"""
    approach = grasp_cfg['approach']
    if approach not in AXES:
        raise ValueError(f'approach 只接受 {sorted(AXES)}，收到 {approach!r}')
    axis = np.array(AXES[approach][arm])
    radius_cm = float(size_m[0]) * 50.0
    half_h_cm = float(size_m[2]) * 50.0
    standoff = half_h_cm if approach == 'top' else radius_cm
    grasp = center - axis * (standoff + grasp_cfg['palm_gap_cm'])
    pregrasp = grasp - axis * grasp_cfg['pregrasp_cm']
    lift = grasp + np.array([0.0, 0.0, grasp_cfg['lift_cm']])
    return GraspPoses(arm, approach, center, pregrasp, grasp, lift)


def from_meas(p_cm, frames):
    """量測座標 [前, 右, 高]（cm）→ base_link（m），to_meas 的反過來。"""
    return np.array([
        (float(p_cm[0]) - frames['front_offset_cm']) / 100.0 / frames['front_sign'],
        (float(p_cm[1]) - frames['right_offset_cm']) / 100.0 / frames['right_sign'],
        (float(p_cm[2]) - frames['height_offset_cm']) / 100.0 / frames['height_sign'],
    ])


def table_obstacle(table, frames):
    """桌子 → /arm_ik_obstacles 的 box（base_link，m）：從地板到桌面，桌緣往前 depth_cm，
    桌面、桌緣、左右各加厚 margin_cm（沒設 = 0）。"""
    m = table.get('margin_cm', 0.0) / 100.0
    x0 = table['edge_from_base_link_cm'] / 100.0 - m
    x1 = x0 + table['depth_cm'] / 100.0 + 2 * m
    y = from_meas((0.0, table['center_right_cm'], 0.0), frames)[1]
    z_top = from_meas((0.0, 0.0, table['top_cm']), frames)[2] + m
    z_floor = from_meas((0.0, 0.0, 0.0), frames)[2]
    return {'id': 'sp3_table', 'type': 'box',
            'c': [(x0 + x1) / 2, y, (z_top + z_floor) / 2],
            'R': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            'h': [(x1 - x0) / 2, table['width_cm'] / 200.0 + m, (z_top - z_floor) / 2]}


def bottle_obstacle(name, xyz_m, size_m, margin_cm, oid=None):
    """藥瓶 → /arm_ik_obstacles 的 stick（直立圓柱，base_link，m）。size_m = (直徑, 直徑, 高)。
    oid 沒給 = sp3_bottle_<類別>。"""
    m = margin_cm / 100.0
    x, y, z = (float(v) for v in xyz_m)
    half = float(size_m[2]) / 2 + m
    return {'id': oid or f'sp3_bottle_{name}', 'type': 'stick',
            'a': [x, y, z - half], 'b': [x, y, z + half], 'r': float(size_m[0]) / 2 + m}


def held_obstacle(xyz_m, size_m):
    """夾起來、要掛到手上的藥瓶（/arm_attach），不加餘裕（手指本來就貼著它）。"""
    return bottle_obstacle('held', xyz_m, size_m, 0.0, oid='sp3_held')


def same_poses(a, b):
    """兩隻手算出來的預抓、抓取位置一樣（top / front 時）→ 可以讓 moveit_bridge 自己選手。"""
    return np.allclose(a.pregrasp, b.pregrasp) and np.allclose(a.grasp, b.grasp)


def moveit_ids(source, objs):
    """/arm_ik_obstacles 的物體在 MoveIt 場景裡的 id（moveit_bridge：html_<來源>_<id>，id 已經以「來源_」開頭就不重複加）。"""
    out = set()
    for o in objs:
        oid = str(o['id'])
        out.add('html_' + (oid if oid.startswith(source + '_') else f'{source}_{oid}'))
    return out


def scene_missing(source, objs, have):
    """MoveIt 場景（have = 物體 id 集合）跟這次要的比：(少了哪些, 這個來源多出來、應該被拿掉的)。"""
    want = moveit_ids(source, objs)
    mine = {h for h in have if h.startswith(f'html_{source}_')}
    return want - set(have), mine - want


def scene(cfg, others):
    """要發給 /arm_ik_obstacles 的整份清單：桌子 + others [(類別, 中心 m, 尺寸 m)]（要抓的那瓶不在裡面）。"""
    margin = cfg['obstacles']['margin_cm']
    return [table_obstacle(cfg['table'], cfg['frames'])] + [
        bottle_obstacle(name, xyz, size, margin) for name, xyz, size in others]


def place_poses(target, grasp_cfg):
    """放置點（量測座標，cm，掌心要到的位置）→ (上方預備位置, 放置位置)。從上方往下放。"""
    p = np.array(target['pos_cm'], dtype=float)
    above = p + np.array([0.0, 0.0, grasp_cfg['lift_cm']])
    return above, p
