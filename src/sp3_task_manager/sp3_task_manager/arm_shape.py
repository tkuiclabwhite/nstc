"""從 URDF 算手臂的外形（碰撞模型的頂點），給 reach_test 畫圖、算離障礙物多近用。只用 numpy。

    shape = ArmShape(urdf_path)
    pts = shape.points({'Rshoulder...': q, ...}, 'right')   # 這隻手（手臂 + 手）的頂點，base_link（m）
    line = shape.skeleton(q, 'right')                       # 各關節位置連成的線

手指一律當作伸直（關節角 0），跟 moveit_bridge 移動時張開的手差不多。
"""
import os
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np

# 每隻手的 link 名稱開頭（URDF：手臂 R… / L…，手 rh_… / lh_…）與手掌 link
ARM_PREFIX = {'right': ('R', 'rh_'), 'left': ('L', 'lh_')}
HAND_BASE = {'right': 'rh_hand_base_link', 'left': 'lh_hand_base_link'}


def _vec(s, default='0 0 0'):
    return np.array([float(v) for v in (s or default).split()])


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _origin(el):
    T = np.eye(4)
    if el is not None:
        T[:3, :3] = _rpy(*_vec(el.get('rpy')))
        T[:3, 3] = _vec(el.get('xyz'))
    return T


def _axis_rot(axis, q):
    k = axis / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K
    return T


def load_stl(path):
    b = open(path, 'rb').read()
    if b[:5] == b'solid' and b'facet' in b[:300]:
        return np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', b), float)
    n = struct.unpack('<I', b[80:84])[0]
    d = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return d['v'].reshape(-1, 3).astype(float)


class ArmShape:
    def __init__(self, urdf_path, resolve, per_link=400):
        """resolve(package_name) → 該套件的 share 目錄（解析 package://）。per_link：每個 link 最多留幾個頂點。"""
        root = ET.parse(urdf_path).getroot()
        self.joints = {}           # child link → (parent, origin, 類型, 軸, 關節名)
        for j in root.findall('joint'):
            ax = j.find('axis')
            self.joints[j.find('child').get('link')] = (
                j.find('parent').get('link'), _origin(j.find('origin')), j.get('type'),
                _vec(ax.get('xyz') if ax is not None else None, '1 0 0'), j.get('name'))
        self.meshes = {}           # link → 頂點（link 座標，m）
        rng = np.random.default_rng(0)
        for link in root.findall('link'):
            pts = []
            for c in link.findall('collision') or link.findall('visual'):
                m = c.find('geometry/mesh')
                if m is None:
                    continue
                fn = m.get('filename')
                if fn.startswith('package://'):
                    pkg, rel = fn[len('package://'):].split('/', 1)
                    fn = os.path.join(resolve(pkg), rel)
                v = load_stl(fn) * _vec(m.get('scale'), '1 1 1')
                T = _origin(c.find('origin'))
                pts.append(v @ T[:3, :3].T + T[:3, 3])
            if pts:
                v = np.vstack(pts)
                if len(v) > per_link:
                    v = v[rng.choice(len(v), per_link, replace=False)]
                self.meshes[link.get('name')] = v

    def link_T(self, link, q):
        """link 在 base_link 的位姿；q = {關節名: 角度 rad}，沒給的關節當 0。"""
        chain = []
        while link in self.joints:
            chain.append(link)
            link = self.joints[link][0]
        T = np.eye(4)
        for l in reversed(chain):
            _, origin, kind, axis, name = self.joints[l]
            T = T @ origin
            if kind in ('revolute', 'continuous'):
                T = T @ _axis_rot(axis, q.get(name, 0.0))
        return T

    def points(self, q, arm):
        """這隻手（手臂 + 手）所有 link 的頂點（base_link，m）。"""
        out = []
        for link, v in self.meshes.items():
            if link.startswith(ARM_PREFIX[arm]):
                T = self.link_T(link, q)
                out.append(v @ T[:3, :3].T + T[:3, 3])
        return np.vstack(out) if out else np.zeros((0, 3))

    def points_of(self, q, link):
        """一個 link 的頂點（base_link，m）。"""
        T = self.link_T(link, q)
        return self.meshes[link] @ T[:3, :3].T + T[:3, 3]

    def skeleton(self, q, arm):
        """base_link → 手掌：沿途每個 link 原點的位置（畫手臂的線用）。"""
        link, chain = HAND_BASE[arm], []
        while link in self.joints:
            chain.append(link)
            link = self.joints[link][0]
        return np.array([self.link_T(l, q)[:3, 3] for l in reversed(chain)])


def distance(points, ob):
    """點群到一個場景物體（/arm_ik_obstacles 格式，base_link，m）表面的最短距離（m，負的 = 插進去）與最近的點。
    plane（地板）回傳 (None, None)。"""
    p = np.asarray(points, float)
    t = ob.get('type')
    if t == 'ball':
        d = np.linalg.norm(p - np.asarray(ob['c']), axis=1) - float(ob['r'])
    elif t == 'box':
        R, h = np.asarray(ob['R'], float), np.asarray(ob['h'], float)
        local = np.abs((p - np.asarray(ob['c'])) @ R)
        out = np.linalg.norm(np.maximum(local - h, 0.0), axis=1)
        inside = np.min(h - local, axis=1)
        d = np.where(out > 0, out, -inside)
    elif t == 'stick':
        a, b = np.asarray(ob['a'], float), np.asarray(ob['b'], float)
        L = np.linalg.norm(b - a)
        k = (b - a) / L
        ax = (p - a) @ k                                     # 沿軸
        rad = np.linalg.norm((p - a) - np.outer(ax, k), axis=1)
        dr, da = rad - float(ob['r']), np.maximum(-ax, ax - L)  # 圓柱：半徑方向、軸向各超出多少
        out = np.hypot(np.maximum(dr, 0.0), np.maximum(da, 0.0))
        d = np.where((dr > 0) | (da > 0), out, np.maximum(dr, da))
    else:
        return None, None
    i = int(np.argmin(d))
    return float(d[i]), p[i]
