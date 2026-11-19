#!/usr/bin/env python3
"""驗證新平台的頭部模型（head_tf/neck_model.py）：用 head_sweep 收的資料，看頭轉到各姿勢時模型算得準不準。

在桌機上跑（需要 scipy，因為共用 fit_head_model.py 的讀檔函式）：

    cd ~/nstc_ws/nstc/src/head_tf
    env -u PYTHONPATH ~/nstc_ws/nstc/foundationpose_trial/.venv/bin/python tools/check_neck_model.py \\
        ~/nstc_ws/head_sweeps/neck_01

設定檔預設用資料夾裡 robot_config/ 的（收集當時機器人上的），沒有才用 repo 裡的。

比較三種東西，都和頭尾的基準姿勢比。外參本身的誤差、IMU 裝歪、桌子微斜，在基準姿勢和其他姿勢都一樣，
相減就消掉了，剩下的就是「頭轉動後模型算得準不準」：
1. 藥罐水平位置（最重要，直接是抓取會差多少）：同一個藥罐在每個姿勢照 bottle_localization 的算法
   （表面點轉到 base_link、往外推一個半徑）算出的位置，和基準姿勢差幾 mm。
   同時列出「固定外參」（不管頭怎麼轉都用 camera_extrinsics.yaml，也就是沒有動態 TF）的結果對照。
2. IMU：模型算的相機 roll / pitch 變化 vs IMU 量到的變化。pitch 對不上 = 低頭角度不準；
   轉頭時 roll 跟著變 = 轉頭軸不是鉛直的（支架或 base_link 歪了）。
3. 桌面高度：模型算的左眼光心離桌面 vs 深度圖擬合的距離。對不上 = 相機的上下位置不準。
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(PKG, 'tools'))
from head_tf import neck_model as nm  # noqa: E402
from head_tf.head_model import matrix_rpy  # noqa: E402
import fit_head_model as fit  # noqa: E402

BL_CONFIG = os.path.join(os.path.dirname(PKG), 'bottle_localization', 'config')
DEFAULTS = {
    'config': os.path.join(PKG, 'config', 'neck_head.yaml'),
    'extrinsics': os.path.join(BL_CONFIG, 'camera_extrinsics.yaml'),
    'params': os.path.join(BL_CONFIG, 'params.yaml'),
    'reference': os.path.join(BL_CONFIG, 'robot_reference.yaml'),
    'dims': os.path.join(BL_CONFIG, 'bottle_dims.yaml'),
}
SWEEP_FILES = {'config': 'neck_head.yaml', 'extrinsics': 'camera_extrinsics.yaml', 'params': 'params.yaml',
               'reference': 'robot_reference.yaml', 'dims': 'bottle_dims.yaml'}

# 判斷標準：藥罐位置和基準姿勢差的 95 百分位（mm）。
# 模型完全正確也不會是 0：bottle_localization 的中心算法（框中間的表面點往外推半徑）在藥罐位於畫面不同位置時
# 本身有幾 mm 的偏差。模擬（test_check_neck_model.py）：轉頭 15°、低頭 ±10°，95% 約 4 mm、最大約 6 mm，
# 大瓶子（b1 直徑 8.2 cm）在畫面邊緣最明顯。舊機器人擬合後的留一驗證是中位數 1.5、最大 3.6 mm（範圍 ±10°）。
GOOD_MM = 5.0
OK_MM = 10.0


def resolve_files(sweep, given):
    """每個設定檔：有指定用指定的，否則用資料夾 robot_config/ 裡的，再沒有用 repo 的。回 {key: (path, 來源)}。"""
    out = {}
    for key, name in SWEEP_FILES.items():
        if given.get(key):
            out[key] = (given[key], '指定')
        elif os.path.exists(os.path.join(sweep, 'robot_config', name)):
            out[key] = (os.path.join(sweep, 'robot_config', name), '收集當時')
        else:
            out[key] = (DEFAULTS[key], 'repo')
    return out


def load_z_table(params_path, reference_path):
    """照 bottle_localization：z_table = (桌高 − 基準點離地) / 100 + 基準點的 z。"""
    with open(params_path, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f)['bottle_localizer']['ros__parameters']
    with open(reference_path, 'r', encoding='utf-8') as f:
        ref = yaml.safe_load(f)
    return (float(params['table_height_cm']) - float(ref['reference_height_cm'])) / 100.0 \
        + float(ref['reference_point'][2])


def load_roll(sweep):
    """postures.csv 的 roll_mean（30 號），{index: 刻度}；沒有這欄（舊資料）就空的。"""
    out = {}
    with open(os.path.join(sweep, 'postures.csv'), encoding='utf-8') as f:
        for r in csv.DictReader(f):
            v = fit._float(r, 'roll_mean')
            if math.isfinite(v):
                out[int(r['index'])] = v
    return out


def ticks_of(posture, roll, model):
    pan_id, tilt_id, roll_id = model.motor_ids
    return {pan_id: posture['pan'], tilt_id: posture['tilt'],
            roll_id: roll.get(posture['index'], model.reference_ticks[roll_id])}


def _stats(values):
    v = np.abs(np.asarray(values, dtype=float))
    if v.size == 0:
        return None
    return {'median': float(np.median(v)), 'p95': float(np.percentile(v, 95)), 'max': float(v.max()),
            'rms': float(np.sqrt(np.mean(v ** 2))), 'n': int(v.size)}


def evaluate(sweep_data, model, radii, z_table, roll):
    """回 dict：每個姿勢的結果（postures）和整體統計（bottle、static、imu_roll、imu_pitch、table）。"""
    postures = sweep_data['postures']
    by_index = {p['index']: p for p in postures}
    ref_ticks = model.reference_ticks
    static_pose = model.optical_pose(ref_ticks)

    def bottle_xy(pose, q, radius):
        return fit.center_xy(pose[0], pose[1], q, radius)

    poses = {p['index']: model.optical_pose(ticks_of(p, roll, model)) for p in postures}
    # 藥罐：基準姿勢的平均位置當真值
    refs = {'model': {}, 'static': {}}
    for b in sweep_data['bottles']:
        p = by_index[b['index']]
        if p['kind'] == 'ref':
            r = radii[b['label']]
            refs['model'].setdefault(b['label'], []).append(bottle_xy(poses[p['index']], b['q'], r))
            refs['static'].setdefault(b['label'], []).append(bottle_xy(static_pose, b['q'], r))
    refs = {k: {lab: np.mean(v, axis=0) for lab, v in d.items()} for k, d in refs.items()}

    rows = {p['index']: {'index': p['index'], 'kind': p['kind'], 'ticks': ticks_of(p, roll, model),
                         'valid': model.check(ticks_of(p, roll, model))[0],
                         'bottle_mm': {}, 'static_mm': {}} for p in postures}
    errs, errs_static = [], []
    for b in sweep_data['bottles']:
        p = by_index[b['index']]
        if p['kind'] == 'ref' or b['label'] not in refs['model']:
            continue
        r = radii[b['label']]
        e = float(np.linalg.norm(bottle_xy(poses[p['index']], b['q'], r) - refs['model'][b['label']])) * 1000
        es = float(np.linalg.norm(bottle_xy(static_pose, b['q'], r) - refs['static'][b['label']])) * 1000
        rows[p['index']]['bottle_mm'][b['label']] = e
        rows[p['index']]['static_mm'][b['label']] = es
        errs.append(e)
        errs_static.append(es)

    # IMU 和桌面高度：模型預測值，扣掉基準姿勢的平均差（IMU 裝歪、桌子微斜、外參誤差）
    pred = {}
    for p in postures:
        R_opt, c = poses[p['index']]
        R_body = R_opt @ nm.R_BODY_FROM_OPT.T
        r_, p_, _ = matrix_rpy(R_body)
        pred[p['index']] = (math.degrees(r_), math.degrees(p_), c[2] - z_table)
    meas = {p['index']: (math.degrees(p['imu_roll']), math.degrees(p['imu_pitch']), p['table_h'])
            for p in postures}
    offsets = []
    for k in range(3):
        d = [meas[i][k] - pred[i][k] for i, p in by_index.items()
             if p['kind'] == 'ref' and math.isfinite(meas[i][k])]
        offsets.append(float(np.mean(d)) if d else float('nan'))
    res = {'imu_roll': [], 'imu_pitch': [], 'table': []}
    for i, p in by_index.items():
        row = rows[i]
        row['pred'] = pred[i]
        row['meas'] = meas[i]
        for k, key in enumerate(('imu_roll', 'imu_pitch', 'table')):
            v = meas[i][k] - pred[i][k] - offsets[k]
            row[key] = v if math.isfinite(v) else float('nan')
            if math.isfinite(v) and p['kind'] != 'ref':
                res[key].append(v * (1000 if key == 'table' else 1))

    return {
        'postures': [rows[i] for i in sorted(rows)],
        'bottle': _stats(errs), 'static': _stats(errs_static),
        'imu_roll': _stats(res['imu_roll']), 'imu_pitch': _stats(res['imu_pitch']),
        'table': _stats(res['table']),
        'offsets': {'imu_roll_deg': offsets[0], 'imu_pitch_deg': offsets[1], 'table_m': offsets[2]},
        'table_pred_ref_m': float(np.mean([pred[i][2] for i, p in by_index.items() if p['kind'] == 'ref'])),
    }


def _fmt(s, unit, spec='.1f'):
    if s is None:
        return '沒有資料'
    return (f'中位數 {s["median"]:{spec}}、95% {s["p95"]:{spec}}、最大 {s["max"]:{spec}} {unit}'
            f'（{s["n"]} 筆）')


def report(result, model):
    pan_id, tilt_id, roll_id = model.motor_ids
    lines = ['', '=== 每個姿勢（和基準姿勢比）===',
             f'{"#":>3} {"種類":4} {pan_id:>5} {tilt_id:>5} {roll_id:>5}  {"藥罐 mm（模型 / 固定外參）":28}'
             f'  {"IMU roll":>8} {"IMU pitch":>9} {"桌高 mm":>7}']
    for r in result['postures']:
        t = r['ticks']
        if r['bottle_mm']:
            worst = max(r['bottle_mm'], key=r['bottle_mm'].get)
            b = f'{max(r["bottle_mm"].values()):5.1f} / {max(r["static_mm"].values()):5.1f}（{worst}）'
        elif r['kind'] == 'ref':
            b = '基準'
        else:
            b = '沒有藥罐'
        flag = '' if r['valid'] else '  超出允許範圍'
        lines.append(f'{r["index"]:>3} {r["kind"]:4} {t[pan_id]:5.0f} {t[tilt_id]:5.0f} {t[roll_id]:5.0f}  {b:28}'
                     f'  {r["imu_roll"]:+8.2f} {r["imu_pitch"]:+9.2f} {r["table"] * 1000:+7.1f}{flag}')
    o = result['offsets']
    lines += [
        '', '=== 整體 ===',
        f'藥罐水平位置（動態 TF）：{_fmt(result["bottle"], "mm")}',
        f'藥罐水平位置（固定外參）：{_fmt(result["static"], "mm")}',
        f'IMU roll 殘差：{_fmt(result["imu_roll"], "°", ".2f")}',
        f'IMU pitch 殘差：{_fmt(result["imu_pitch"], "°", ".2f")}',
        f'桌面高度殘差：{_fmt(result["table"], "mm")}',
        f'基準姿勢：IMU 比模型 roll {o["imu_roll_deg"]:+.2f}°、pitch {o["imu_pitch_deg"]:+.2f}°（IMU 裝在相機裡的角度，'
        f'正常約 1°）；光心離桌面 模型 {result["table_pred_ref_m"] * 100:.1f} cm、深度圖 '
        f'{(result["table_pred_ref_m"] + o["table_m"]) * 100:.1f} cm',
    ]
    b = result['bottle']
    if b is None:
        lines.append('⚠ 沒有可以比較的藥罐（基準姿勢和其他姿勢都要看到同一個藥罐）')
    elif b['p95'] <= GOOD_MM:
        lines.append(f'結論：藥罐位置 95% 在 {b["p95"]:.1f} mm 內（≤ {GOOD_MM:g}），模型可以直接用')
    elif b['p95'] <= OK_MM:
        lines.append(f'結論：藥罐位置 95% 在 {b["p95"]:.1f} mm 內，堪用；最大 {b["max"]:.1f} mm，看上面哪個姿勢最差')
    else:
        lines.append(f'結論：藥罐位置 95% 差到 {b["p95"]:.1f} mm，模型需要修正（看 IMU 和桌面高度殘差找原因）')
    if b is not None:
        lines.append('（模型完全正確時藥罐也會差幾 mm：定位算法本身的偏差和藥罐在畫面中的位置有關，'
                     '模擬中 95% 約 4 mm、最大約 6 mm。IMU 和桌面高度殘差不受這個影響，比較能看出模型本身的誤差）')
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sweep', help='head_sweep 的資料夾')
    for key in SWEEP_FILES:
        ap.add_argument(f'--{key}', help=f'{SWEEP_FILES[key]}；預設用資料夾 robot_config/ 裡的，沒有用 repo 的')
    opts = ap.parse_args(argv)

    files = resolve_files(opts.sweep, vars(opts))
    for key, (path, src) in files.items():
        print(f'{SWEEP_FILES[key]:24}（{src}）{path}')
    radii = fit.load_dims(files['dims'][0])
    model = nm.NeckModel.load(files['config'][0], files['extrinsics'][0])
    z_table = load_z_table(files['params'][0], files['reference'][0])
    print(f'桌面 z_table = {z_table:+.4f} m')
    data = fit.load_sweep(opts.sweep, radii)
    roll = load_roll(opts.sweep)
    if not roll:
        print('⚠ 資料裡沒有 30 號（歪頭）的刻度，當成一直在基準')
    result = evaluate(data, model, radii, z_table, roll)
    for line in report(result, model):
        print(line)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
