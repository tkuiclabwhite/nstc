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


def in_workspace(p, ws):
    return all(ws['min_cm'][i] <= p[i] <= ws['max_cm'][i] for i in range(3))


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


def place_poses(target, grasp_cfg):
    """放置點（量測座標，cm，掌心要到的位置）→ (上方預備位置, 放置位置)。從上方往下放。"""
    p = np.array(target['pos_cm'], dtype=float)
    above = p + np.array([0.0, 0.0, grasp_cfg['lift_cm']])
    return above, p
