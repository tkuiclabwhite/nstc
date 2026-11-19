"""測試用的新平台模擬器：相機姿態由一個「真的」三軸脖子模型（NeckModel）決定，其他沿用 sim_head 的
桌面、瓶子、深度圖、YOLO、IMU。真的模型可以和設定檔不一樣（例如支架歪了），用來檢查驗證工具抓不抓得到。

刻度 (pan, tilt) = (28 號, 29 號)；30 號固定在 roll_tick。
"""
import math
import os

import numpy as np
import yaml

from head_tf import neck_model as nm
from head_tf.head_model import rpy_matrix
from sim_head import HeadSim, SimIO

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config', 'neck_head.yaml')


def load_config():
    with open(CONFIG, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


# 和實際外參差不多：螺絲孔在支架中心前 9.4、上 11.5 cm，低頭約 30.4°
R0 = rpy_matrix(math.radians(0.3), math.radians(30.4), 0.0)
T0 = np.array([0.094, -0.001, 0.115])
Z_TABLE = -0.39


class NeckSim(HeadSim):
    def __init__(self, truth_config=None, roll_tick=None):
        super().__init__()
        cfg = truth_config or load_config()
        self.truth = nm.NeckModel(cfg, R0, T0)
        ref = self.truth.reference_ticks
        self.ref = (ref[28], ref[29])
        self.roll_tick = float(ref[30] if roll_tick is None else roll_tick)
        self.z_table = Z_TABLE
        # 相機在桌面上方約 52 cm、往下看 20°～40°：瓶子放在支架前 60～75 cm，每個姿勢都看得到
        self.bottles = {'b1': (0.62, 0.06, 0.041, 0.092),
                        'b2': (0.68, 0.20, 0.025, 0.078),
                        'b3': (0.74, -0.08, 0.022, 0.090)}
        # 瓶子比 sim_head 遠，解析度加倍，每個瓶子才有足夠的深度點（fit_head_model 要 50 點以上）
        self.width, self.height = 480, 300
        self.k = (240.0, 240.0, 239.5, 149.5)

    def pose(self, pan, tilt):
        return self.truth.camera_link_pose({28: pan, 29: tilt, 30: self.roll_tick})


class NeckSimIO(SimIO):
    def snapshot(self, t0, t1, max_frames):
        snap = super().snapshot(t0, t1, max_frames)
        snap.roll = [(t, self.sim.roll_tick) for t in np.arange(t0, t1, 0.02)]
        return snap
