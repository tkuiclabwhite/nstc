#!/usr/bin/env python3
"""用 head_sweep 收的資料擬合頭部模型（在桌機上跑，需要 scipy）。

    cd ~/nstc_ws/nstc/src/head_tf
    env -u PYTHONPATH ~/nstc_ws/nstc/foundationpose_trial/.venv/bin/python tools/fit_head_model.py \\
        ~/nstc_ws/head_sweeps/sweep_01 --out config/head_model.yaml

擬合的未知數：
    pan 軸、tilt 軸的方向（各 2 個角度）和位置（各 2 個距離）
    IMU 晶片相對相機的安裝角（3 個）、每次收集的桌面高度、每個瓶子的位置
用到的量測：
    IMU 的 roll / pitch（相機姿態，最準，約 0.05°）
    瓶子的水平位置（照 bottle_localization：表面點轉到 base_link、往外推一個半徑）。瓶子整個過程沒動，
    所以每個姿勢算出來的都要是同一點。不用表面點的高度：頭上下轉時框住瓶子的部位不同，
    表面點的高度會跟著變，拿來擬合會被誤當成相機平移（模擬測試裡讓轉軸位置偏 6~10 mm）
    相機光心離桌面的高度（從存下來的深度圖重新擬合，用比 head_sweep 更大的範圍）
    手量的轉軸位置（head_axes.yaml，給很寬的誤差，只用來穩住資料看不太到的方向）
刻度的比例固定為 1（Dynamixel 的編碼器在輸出軸上），--fit-scale 可以一起擬合來檢查。

輸出的 head_model.yaml 給之後的動態 TF 節點用；同時印出驗證：同一個瓶子在各個姿勢算出的
水平位置（照 bottle_localization 的算法，表面點往外推半徑），和基準姿勢差多少。
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np
import yaml
from scipy.optimize import least_squares

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
from head_tf import head_model as hm  # noqa: E402
from head_tf import sweep_core as core  # noqa: E402

DEFAULT_AXES = os.path.join(PKG, 'config', 'head_axes.yaml')
DEFAULT_DIMS = os.path.join(os.path.dirname(PKG), 'bottle_localization', 'config', 'bottle_dims.yaml')

# 桌面重新擬合的範圍。sweep_01 收的時候 head_sweep 還是用 70~95% / 25~75%（太小），所以一律從深度圖重算
PLANE_ROWS = core.PLANE_ROWS
PLANE_COLS = core.PLANE_COLS
MIN_PLANE_INLIERS = 0.6
EDGE_MARGIN_PX = 3            # 框離畫面邊緣幾個像素內算被切到，表面點會偏，不用
MIN_LABEL_POSTURES = 5        # 出現次數比這少的類別當成誤判
MIN_BOTTLE_POINTS = 50

SIGMA_IMU = math.radians(0.05)
SIGMA_BOTTLE = 0.002          # 瓶子水平位置（m）
SIGMA_TABLE = 0.0015
SIGMA_PRIOR = 0.02            # 手量值的誤差（m），故意給寬
LOSS_SCALE = 3.0              # soft_l1：超過 3 個 sigma 的殘差權重開始變小


# =============================================================================
# 讀資料
# =============================================================================
def _float(row, key):
    try:
        return float(row[key])
    except (KeyError, ValueError):
        return float('nan')


def load_dims(path):
    with open(path, 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f)['bottle_dims']
    return {k: float(v['diameter']) / 2 for k, v in d.items()}


def load_sweep(path, radii, plane_rows=PLANE_ROWS, plane_cols=PLANE_COLS, log=print):
    """讀一次收集的資料。桌面用存下來的深度圖重新擬合；瓶子去掉碰邊、誤判的。"""
    name = os.path.basename(os.path.normpath(path))
    with open(os.path.join(path, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    w, h = meta['camera']['size']
    with open(os.path.join(path, 'postures.csv'), encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    with open(os.path.join(path, 'bottles.csv'), encoding='utf-8') as f:
        brows = list(csv.DictReader(f))

    postures = []
    for r in rows:
        idx = int(r['index'])
        z = np.load(os.path.join(path, f'posture_{idx:02d}.npz'))
        table_h, inliers = float('nan'), 0.0
        if 'depth' in z.files:
            dets = json.loads(str(z['detections']))
            groups, _, _ = core.group_detections([(0, d) for d in dets], (w, h))
            pts = core.plane_points(z['depth'], tuple(z['k']), plane_rows, plane_cols,
                                    [g[2] for g in groups.values()], core.BOX_MARGIN_PX, 60000)
            if pts.shape[0] >= core.MIN_PLANE_POINTS:
                n, c, _, n_in = core.fit_plane(pts, iterations=500)
                inliers = n_in / pts.shape[0]
                if inliers >= MIN_PLANE_INLIERS:
                    table_h = float(-(n @ c))
        postures.append({
            'index': idx, 'kind': r['kind'], 'pan': _float(r, 'pan_mean'), 'tilt': _float(r, 'tilt_mean'),
            'imu_roll': math.radians(_float(r, 'imu_roll_deg')),
            'imu_pitch': math.radians(_float(r, 'imu_pitch_deg')),
            'table_h': table_h, 'plane_inliers': inliers})

    counts = {}
    for b in brows:
        counts[b['label']] = counts.get(b['label'], 0) + 1
    bottles, dropped = [], {'碰邊': 0, '少見類別': 0, '點太少': 0, '沒有尺寸': 0}
    for b in brows:
        if counts[b['label']] < MIN_LABEL_POSTURES:
            dropped['少見類別'] += 1
            continue
        if b['label'] not in radii:
            dropped['沒有尺寸'] += 1
            continue
        if int(b['n_points']) < MIN_BOTTLE_POINTS or not math.isfinite(_float(b, 'z')):
            dropped['點太少'] += 1
            continue
        x1, y1, x2, y2 = (int(b[k]) for k in ('x1', 'y1', 'x2', 'y2'))
        if (x1 <= EDGE_MARGIN_PX or y1 <= EDGE_MARGIN_PX
                or x2 >= w - 1 - EDGE_MARGIN_PX or y2 >= h - 1 - EDGE_MARGIN_PX):
            dropped['碰邊'] += 1
            continue
        bottles.append({'index': int(b['index']), 'label': b['label'],
                        'q': np.array([_float(b, 'x'), _float(b, 'y'), _float(b, 'z')])})
    ext = os.path.join(path, 'robot_config', 'camera_extrinsics.yaml')
    sweep = {'name': name, 'path': path, 'postures': postures, 'bottles': bottles,
             'extrinsics_file': ext if os.path.exists(ext) else None}
    n_table = sum(1 for p in postures if math.isfinite(p['table_h']))
    log(f'{name}：{len(postures)} 個姿勢；IMU {2 * len(postures)} 筆、桌面 {n_table} 筆、瓶子 {len(bottles)} 筆'
        f'（不用：{"、".join(f"{k} {v}" for k, v in dropped.items() if v) or "無"}）')
    return sweep


# =============================================================================
# 參數
# =============================================================================
class Layout:
    """參數向量的排列。方向用兩個小量表示：pan = normalize(a, b, 1)，tilt = normalize(a, 1, b)。
    位置是「軸上一點 -> 螺絲孔」，只存和軸垂直的兩個分量：pan (前, 左, 0)、tilt (前, 0, 上)。"""

    def __init__(self, sweeps, fit_scale):
        self.fit_scale = fit_scale
        self.labels = [(s['name'], lab) for s in sweeps for lab in sorted({b['label'] for b in s['bottles']})]
        self.sweep_names = [s['name'] for s in sweeps]
        # IMU 安裝角只擬合 roll、pitch：重力看不到方位，IMU 的 yaw 和兩個軸的水平方向會一起轉而讀數不變，
        # 所以固定為 0（ZED 出廠已校正 IMU 和鏡頭的相對角度，偏差很小）
        self.n_head = 8 + 2 + (2 if fit_scale else 0)
        self.n = self.n_head + len(self.sweep_names) + 2 * len(self.labels)

    def unpack(self, x, signs):
        pa, pb, pdx, pdy, ta, tb, tdx, tdz = x[:8]
        imu = hm.rpy_matrix(x[8], x[9], 0.0)
        ks, kt = (x[10], x[11]) if self.fit_scale else (1.0, 1.0)
        model = hm.HeadModel(hm.Joint([pa, pb, 1.0], [pdx, pdy, 0.0], signs[0], ks),
                             hm.Joint([ta, 1.0, tb], [tdx, 0.0, tdz], signs[1], kt))
        k = self.n_head
        z_table = dict(zip(self.sweep_names, x[k:k + len(self.sweep_names)]))
        k += len(self.sweep_names)
        bottles = {key: x[k + 2 * i:k + 2 * i + 2] for i, key in enumerate(self.labels)}
        return model, imu, z_table, bottles


def center_xy(R_opt, c, q, radius):
    """照 bottle_localization：表面點轉到 base_link，水平方向往遠離相機推一個半徑。"""
    P = R_opt @ q + c
    d = P[:2] - c[:2]
    return P[:2] + radius * d / np.linalg.norm(d)


def residuals(x, layout, sweeps, ext, radii, signs, prior, skip=None, parts=False):
    model, imu, z_table, bottles = layout.unpack(x, signs)
    R0, t0 = ext
    out = {'imu': [], 'table': [], 'bottle': [], 'prior': []}
    for s in sweeps:
        poses = {}
        for p in s['postures']:
            if skip is not None and (s['name'], p['index']) == skip:
                continue
            R, t = model.camera_link_pose(R0, t0, p['pan'], p['tilt'])
            R_opt, c = R @ hm.R_BODY_FROM_OPT, t + R @ hm.LINK_TO_LEFT_OPTICAL
            poses[p['index']] = (R_opt, c)
            Ri = R @ imu
            roll, pitch = math.atan2(Ri[2, 1], Ri[2, 2]), -math.asin(max(-1.0, min(1.0, Ri[2, 0])))
            if math.isfinite(p['imu_roll']):
                out['imu'] += [(roll - p['imu_roll']) / SIGMA_IMU, (pitch - p['imu_pitch']) / SIGMA_IMU]
            if math.isfinite(p['table_h']):
                out['table'].append((c[2] - z_table[s['name']] - p['table_h']) / SIGMA_TABLE)
        for b in s['bottles']:
            if b['index'] not in poses:
                continue
            R_opt, c = poses[b['index']]
            xy = center_xy(R_opt, c, b['q'], radii[b['label']])
            out['bottle'] += list((xy - bottles[(s['name'], b['label'])]) / SIGMA_BOTTLE)
    out['prior'] = list((x[2:4] - prior[0]) / SIGMA_PRIOR) + list((x[6:8] - prior[1]) / SIGMA_PRIOR)
    if parts:
        return {k: np.asarray(v) for k, v in out.items()}
    return np.concatenate([np.asarray(out[k], dtype=float) for k in ('imu', 'table', 'bottle', 'prior')])


def initial_guess(layout, sweeps, ext, radii, measured):
    """方向從垂直 / 水平開始，位置用手量值；瓶子位置用基準姿勢 + 現在的外參；桌面用基準姿勢的高度。"""
    x = np.zeros(layout.n)
    x[2:4] = [measured.pan.offset[0], measured.pan.offset[1]]
    x[6:8] = [measured.tilt.offset[0], measured.tilt.offset[2]]
    if layout.fit_scale:
        x[10:12] = 1.0
    R0, t0 = ext
    R_opt, c = R0 @ hm.R_BODY_FROM_OPT, t0 + R0 @ hm.LINK_TO_LEFT_OPTICAL
    k = layout.n_head
    for i, s in enumerate(sweeps):
        hs = [p['table_h'] for p in s['postures'] if p['kind'] == 'ref' and math.isfinite(p['table_h'])]
        x[k + i] = c[2] - (np.mean(hs) if hs else 0.35)
    k += len(sweeps)
    for j, (name, label) in enumerate(layout.labels):
        s = next(s for s in sweeps if s['name'] == name)
        refs = {p['index'] for p in s['postures'] if p['kind'] == 'ref'}
        obs = [b for b in s['bottles'] if b['label'] == label and b['index'] in refs] or \
              [b for b in s['bottles'] if b['label'] == label]
        x[k + 2 * j:k + 2 * j + 2] = np.mean([center_xy(R_opt, c, b['q'], radii[label]) for b in obs], axis=0)
    return x


def fit(sweeps, ext, radii, signs, measured, fit_scale=False, skip=None, x0=None):
    layout = Layout(sweeps, fit_scale)
    prior = (np.array([measured.pan.offset[0], measured.pan.offset[1]]),
             np.array([measured.tilt.offset[0], measured.tilt.offset[2]]))
    if x0 is None:
        x0 = initial_guess(layout, sweeps, ext, radii, measured)
    r = least_squares(residuals, x0, args=(layout, sweeps, ext, radii, signs, prior, skip),
                      loss='soft_l1', f_scale=LOSS_SCALE, x_scale='jac', max_nfev=2000)
    return layout, r, prior


def param_std(r):
    J = r.jac
    dof = max(1, len(r.fun) - len(r.x))
    s2 = float(np.sum(r.fun ** 2)) / dof
    try:
        return np.sqrt(np.diag(np.linalg.inv(J.T @ J)) * s2)
    except np.linalg.LinAlgError:
        return np.full(len(r.x), float('nan'))


# =============================================================================
# 驗證：同一個瓶子在各姿勢算出的水平位置，和基準姿勢差多少
# =============================================================================
def static_model():
    """固定外參（現在的做法）：刻度比例 0，所以任何刻度都是 T0。"""
    return hm.HeadModel(hm.Joint([0, 0, 1], [0, 0, 0], 1, 0.0), hm.Joint([0, 1, 0], [0, 0, 0], 1, 0.0))


def bottle_xy(model, ext, posture, q, radius):
    R_opt, c = model.optical_pose(ext[0], ext[1], posture['pan'], posture['tilt'])
    return center_xy(R_opt, c, q, radius)


def drift(sweeps, ext, radii, model_for):
    """model_for(sweep, posture) -> 模型。回傳 [(sweep, index, label, 誤差 m), ...]，基準姿勢本身不算。"""
    out = []
    for s in sweeps:
        by_index = {p['index']: p for p in s['postures']}
        ref = {}
        for b in s['bottles']:
            p = by_index[b['index']]
            if p['kind'] == 'ref':
                ref.setdefault(b['label'], []).append(
                    bottle_xy(model_for(s, p), ext, p, b['q'], radii[b['label']]))
        ref = {k: np.mean(v, axis=0) for k, v in ref.items()}
        for b in s['bottles']:
            p = by_index[b['index']]
            if p['kind'] == 'ref' or b['label'] not in ref:
                continue
            xy = bottle_xy(model_for(s, p), ext, p, b['q'], radii[b['label']])
            out.append((s['name'], b['index'], b['label'], float(np.linalg.norm(xy - ref[b['label']]))))
    return out


def stats_mm(errs):
    e = np.array([x[3] for x in errs]) * 1000
    return float(np.median(e)), float(np.percentile(e, 95)), float(e.max())


# =============================================================================
# 主程式
# =============================================================================
def describe(model, std, layout):
    """把參數換成人看得懂的句子。"""
    pu, tu = model.pan.direction, model.tilt.direction
    deg = math.degrees
    lines = [
        f'pan 軸：往前傾 {deg(math.atan2(pu[0], pu[2])):+.2f}°（±{deg(std[0]):.2f}）、往左傾 '
        f'{deg(math.atan2(pu[1], pu[2])):+.2f}°（±{deg(std[1]):.2f}）；螺絲孔在軸的前方 '
        f'{model.pan.offset[0] * 100:.2f} cm（±{std[2] * 100:.2f}）、左方 {model.pan.offset[1] * 100:.2f} cm（±{std[3] * 100:.2f}）',
        f'tilt 軸：前後偏 {deg(math.atan2(tu[0], tu[1])):+.2f}°（±{deg(std[4]):.2f}）、上下偏 '
        f'{deg(math.atan2(tu[2], tu[1])):+.2f}°（±{deg(std[5]):.2f}）；螺絲孔在軸的前方 '
        f'{model.tilt.offset[0] * 100:.2f} cm（±{std[6] * 100:.2f}）、上方 {model.tilt.offset[2] * 100:.2f} cm（±{std[7] * 100:.2f}）',
    ]
    if layout.fit_scale:
        lines.append(f'刻度比例：pan {model.pan.scale:.4f}（±{std[10]:.4f}）、tilt {model.tilt.scale:.4f}（±{std[11]:.4f}）')
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sweeps', nargs='+', help='head_sweep 的資料夾（可以多個）')
    ap.add_argument('--out', help='輸出的 head_model.yaml；不給就只印結果')
    ap.add_argument('--axes', default=DEFAULT_AXES, help='手量值 head_axes.yaml')
    ap.add_argument('--dims', default=DEFAULT_DIMS, help='bottle_dims.yaml（瓶子半徑，和 bottle_localization 用同一份）')
    ap.add_argument('--extrinsics', help='基準姿態；預設用第一個資料夾裡 robot_config/camera_extrinsics.yaml')
    ap.add_argument('--fit-scale', action='store_true', help='刻度比例也一起擬合（檢查用）')
    ap.add_argument('--no-loo', action='store_true', help='不做留一驗證（比較快）')
    opts = ap.parse_args(argv)

    radii = load_dims(opts.dims)
    measured = hm.HeadModel.from_measured(opts.axes)
    signs = (measured.pan.sign, measured.tilt.sign)
    sweeps = [load_sweep(p, radii) for p in opts.sweeps]
    ext_file = opts.extrinsics or sweeps[0]['extrinsics_file']
    if ext_file is None:
        print('找不到 camera_extrinsics.yaml，用 --extrinsics 指定')
        return 1
    ext = hm.load_extrinsics(ext_file)
    print(f'基準姿態：{ext_file}')

    layout, r, prior = fit(sweeps, ext, radii, signs, measured, opts.fit_scale)
    model, imu, z_table, bottles = layout.unpack(r.x, signs)
    std = param_std(r)
    parts = residuals(r.x, layout, sweeps, ext, radii, signs, prior, parts=True)
    rms = {k: float(np.sqrt(np.mean(v ** 2))) if len(v) else float('nan') for k, v in parts.items()}

    print('\n=== 擬合結果 ===')
    for line in describe(model, std, layout):
        print(line)
    mr, mp, my = (math.degrees(v) for v in hm.matrix_rpy(imu))
    print(f'IMU 安裝角（相對相機，yaw 固定 0）：roll {mr:+.2f}° pitch {mp:+.2f}°')
    for name, z in z_table.items():
        print(f'桌面高度（base_link z）：{name} {z * 100:.2f} cm')
    sb = parts['bottle'].reshape(-1, 2) * SIGMA_BOTTLE * 1000
    print(f'殘差：IMU RMS {rms["imu"] * math.degrees(SIGMA_IMU):.3f}°（最大 '
          f'{np.abs(parts["imu"]).max() * math.degrees(SIGMA_IMU):.3f}°）、'
          f'桌面 RMS {rms["table"] * SIGMA_TABLE * 1000:.2f} mm、'
          f'瓶子水平位置 RMS {np.sqrt(np.mean(np.sum(sb ** 2, axis=1))):.2f} mm')

    print('\n=== 驗證：同一個瓶子在各姿勢算出的水平位置，和基準姿勢差多少（mm）===')
    print(f'{"方法":24s} {"中位數":>6s} {"95%":>6s} {"最大":>6s}')
    static = static_model()
    rows = [('固定外參（現在的做法）', drift(sweeps, ext, radii, lambda s, p: static)),
            ('手量的轉軸（pan 當垂直）', drift(sweeps, ext, radii, lambda s, p: measured)),
            ('擬合的模型', drift(sweeps, ext, radii, lambda s, p: model))]
    if not opts.no_loo:
        loo = {}
        for s in sweeps:
            for p in s['postures']:
                if p['kind'] == 'ref':
                    continue
                lay, rr, _ = fit(sweeps, ext, radii, signs, measured, opts.fit_scale,
                                 skip=(s['name'], p['index']), x0=r.x)
                loo[(s['name'], p['index'])] = lay.unpack(rr.x, signs)[0]
        rows.append(('擬合的模型（留一驗證）', drift(
            sweeps, ext, radii, lambda s, p: loo.get((s['name'], p['index']), model))))
    for name, errs in rows:
        med, p95, mx = stats_mm(errs)
        print(f'{name:24s} {med:6.1f} {p95:6.1f} {mx:6.1f}')
    print('（留一驗證：每次拿掉一個姿勢擬合，再用那個模型算被拿掉的姿勢，比較接近實際使用時的誤差）')

    worst = sorted(rows[-1][1], key=lambda e: -e[3])[:3]
    print('最大的三筆：' + '、'.join(f'姿勢 {i} {lab} {e * 1000:.1f} mm' for _, i, lab, e in worst))

    if opts.out:
        d = model.to_dict()
        d['fit'] = {
            'date': time.strftime('%Y-%m-%d %H:%M'),
            'sweeps': [s['path'] for s in sweeps],
            'extrinsics': ext_file,
            'residual_rms': {'imu_deg': round(rms['imu'] * math.degrees(SIGMA_IMU), 4),
                             'table_mm': round(rms['table'] * SIGMA_TABLE * 1000, 3)},
            'imu_mount_roll_pitch_deg': [round(mr, 3), round(mp, 3)],
            'table_z_m': {k: round(float(v), 5) for k, v in z_table.items()},
            'validation_mm': {name: dict(zip(('median', 'p95', 'max'), (round(v, 2) for v in stats_mm(e))))
                              for name, e in rows},
        }
        header = ('# 頭部模型：由頭部馬達刻度算 zed_camera_link 的姿態（head_tf/head_model.py）。\n'
                  '# 由 tools/fit_head_model.py 擬合產生，不要手改；重新收資料後重跑那支程式。\n'
                  '# offset_to_screw：軸上一點 -> 螺絲孔（m，基準刻度時的 base_link）；基準姿態用 camera_extrinsics.yaml。\n')
        with open(opts.out, 'w', encoding='utf-8') as f:
            f.write(header)
            yaml.safe_dump(d, f, allow_unicode=True, sort_keys=False)
        print(f'\n已寫入 {opts.out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
