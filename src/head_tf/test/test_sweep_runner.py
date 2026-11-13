import csv
import json
import math

import numpy as np
import pytest

from head_tf import sweep_core as core
from head_tf import sweep_runner as runner
from sim_head import HeadSim, SimIO

PAN = [1934, 2048, 2162]
TILT = [2186, 2300, 2414]


def run(tmp_path, sim, io, cfg=None, pan=PAN, tilt=TILT):
    cfg = cfg or runner.SweepConfig()
    postures = core.build_postures(pan, tilt)
    writer = runner.SweepWriter(tmp_path / 'sweep')
    writer.write_meta({'name': 'test'})
    logs = []
    rows, bottles = runner.run_sweep(io, postures, cfg, writer, logs.append)
    writer.finish('finished')
    return postures, rows, bottles, logs, writer


def to_base(sim, pan, tilt, p_opt):
    R_opt, c = sim.optical(pan, tilt)
    return R_opt @ np.asarray(p_opt) + c


def test_full_sweep_matches_simulated_head(tmp_path):
    sim = HeadSim()
    io = SimIO(sim)
    postures, rows, bottles, logs, writer = run(tmp_path, sim, io)
    assert len(rows) == len(postures) == 11 and len(logs) == 11
    for (kind, pan, tilt), row in zip(postures, rows):
        assert row['kind'] == kind and row['reached'] and row['still']
        assert abs(row['pan_mean'] - pan) <= 1 and abs(row['tilt_mean'] - tilt) <= 1
        er, ep, eh = sim.expected_table(row['pan_mean'], row['tilt_mean'])
        assert row['table_roll_deg'] == pytest.approx(er, abs=0.1)
        assert row['table_pitch_deg'] == pytest.approx(ep, abs=0.1)
        assert row['table_height_m'] == pytest.approx(eh, abs=1e-3)
        ir, ip, _ = sim.imu_roll_pitch(row['pan_mean'], row['tilt_mean'])
        assert row['imu_pitch_deg'] == pytest.approx(math.degrees(ip), abs=1e-6)
        assert row['bottles'] >= 2

    # 瓶子表面點換回 base 後要落在圓柱表面附近。弧面上的點取中位數會落在正面後方
    # 1~4 mm（bottle_localization 也一樣），所以允許往內 5 mm
    by_index = {r['index']: r for r in rows}
    checked = 0
    for b in bottles:
        if b['n_points'] == 0:
            continue
        r = by_index[b['index']]
        P = to_base(sim, r['pan_mean'], r['tilt_mean'], (b['x'], b['y'], b['z']))
        bx, by, rad, h = sim.bottles[b['label']]
        assert rad - 5e-3 <= math.hypot(P[0] - bx, P[1] - by) <= rad + 1e-3
        assert sim.z_table <= P[2] <= sim.z_table + h
        checked += 1
    assert checked >= 2 * len(rows)

    d = core.direction_check(rows, bottles)
    assert d['tilt_slope'] == pytest.approx(1.0, abs=0.05)
    assert 0.7 < d['pan_slope'] < 1.05
    rep = core.repeatability(rows, bottles)
    assert abs(rep['pitch_deg']) < 0.05 and abs(rep['height_mm']) < 0.5
    assert all(v < 1.0 for v in rep['bottle_mm'].values())
    summary = '\n'.join(runner.format_summary(rows, bottles))
    assert '刻度變大 → 往下看' in summary and '刻度變大 → 往左轉' in summary
    assert '頭尾一致' in summary

    out = tmp_path / 'sweep'
    with open(out / 'postures.csv', encoding='utf-8') as f:
        prow = list(csv.DictReader(f))
    assert len(prow) == 11 and list(prow[0]) == core.POSTURE_FIELDS
    with open(out / 'bottles.csv', encoding='utf-8') as f:
        brow = list(csv.DictReader(f))
    assert len(brow) == len(bottles) and list(brow[0]) == core.BOTTLE_FIELDS
    npz = np.load(out / 'posture_03.npz')
    assert npz['depth'].shape == (sim.height, sim.width)
    assert npz['joints'].shape[1] == 3 and npz['imu'].shape[1] == 9
    assert json.loads(str(npz['detections']))[0]['bottles']
    meta = json.loads((out / 'meta.json').read_text(encoding='utf-8'))
    assert meta['status'] == 'finished' and meta['postures_recorded'] == 11
    # 頭最後停在基準姿勢
    assert io.sent[-1][:2] == (2048, 2300)


def test_flipped_directions_are_reported(tmp_path):
    sim = HeadSim(pan_sign=-1, tilt_sign=-1)
    _, rows, bottles, _, _ = run(tmp_path, sim, SimIO(sim))
    d = core.direction_check(rows, bottles)
    assert d['tilt_slope'] == pytest.approx(-1.0, abs=0.05)
    assert d['pan_slope'] < -0.7
    summary = '\n'.join(runner.format_summary(rows, bottles))
    assert '往上看' in summary and '往右轉' in summary


def test_head_not_moving_stops(tmp_path):
    sim = HeadSim()
    io = SimIO(sim, stuck=True)
    with pytest.raises(runner.HeadNotMoving, match='扭力'):
        run(tmp_path, sim, io)
    # 第一個基準姿勢不用動，已經存下來了
    with open(tmp_path / 'sweep' / 'postures.csv', encoding='utf-8') as f:
        assert len(list(csv.DictReader(f))) == 1


def test_stale_joint_states_stop(tmp_path):
    sim = HeadSim()
    io = SimIO(sim, stale_after=103.0)
    with pytest.raises(runner.SweepError, match='沒更新'):
        run(tmp_path, sim, io)


def test_short_of_target_is_recorded_with_warning(tmp_path):
    sim = HeadSim()
    io = SimIO(sim, tilt_sag=20)
    cfg = runner.SweepConfig(move_timeout=1.0)
    _, rows, _, logs, _ = run(tmp_path, sim, io, cfg)
    assert not rows[1]['reached'] and rows[1]['still']
    assert '沒到目標刻度' in logs[1]
    # 分析用的是實際刻度：桌面角度照樣和模擬一致
    er, ep, _ = sim.expected_table(rows[1]['pan_mean'], rows[1]['tilt_mean'])
    assert rows[1]['table_pitch_deg'] == pytest.approx(ep, abs=0.1)


def test_moved_bottle_is_flagged(tmp_path):
    sim = HeadSim()
    io = SimIO(sim)
    cfg = runner.SweepConfig()
    postures = core.build_postures(PAN, TILT)
    writer = runner.SweepWriter(tmp_path / 'sweep')
    rows, bottles = [], []
    for i, (kind, pan, tilt) in enumerate(postures):
        if i == len(postures) - 1:
            x, y, r, h = sim.bottles['b1']
            sim.bottles['b1'] = (x + 0.01, y, r, h)
        move = runner.move_and_settle(io, pan, tilt, cfg)
        row, b = runner.record_posture(io, cfg, writer, i, kind, pan, tilt, move)
        rows.append(row)
        bottles += [dict(x, index=i) for x in b]
    summary = '\n'.join(runner.format_summary(rows, bottles))
    assert '頭尾差太多' in summary


def test_manual_mode(tmp_path):
    sim = HeadSim()
    io = SimIO(sim)
    plan = iter([None, (2162, 2300), (2048, 2414), (2048, 2300), 'q'])

    def ask(_prompt):
        step = next(plan)
        if step == 'q':
            return 'q'
        if step is not None:
            io.send_head(step[0], step[1], 40)
            io.sleep(2.0)
        return ''

    writer = runner.SweepWriter(tmp_path / 'manual')
    rows, bottles = runner.run_manual(io, runner.SweepConfig(), writer, lambda s: None, ask)
    assert [r['kind'] for r in rows] == ['ref', 'manual', 'manual', 'ref']
    assert math.isnan(rows[1]['pan_cmd'])
    d = core.direction_check(rows, bottles)
    assert d['tilt_slope'] is None or d['tilt_slope'] == pytest.approx(1.0, abs=0.05)
    rep = core.repeatability(rows, bottles)
    assert abs(rep['pitch_deg']) < 0.05


def test_config_validation():
    runner.SweepConfig().validate()
    with pytest.raises(ValueError):
        runner.SweepConfig(speed=0).validate()
    with pytest.raises(ValueError):
        runner.SweepConfig(max_depth_frames=1).validate()


def test_writer_refuses_existing_dir(tmp_path):
    (tmp_path / 'x').mkdir()
    with pytest.raises(FileExistsError):
        runner.SweepWriter(tmp_path / 'x')


def test_table_pitch_alone_does_not_flag():
    """sweep_01 的誤報：桌面角度頭尾差 1°，但 IMU、瓶子、桌面高度都一致，不該叫人重收。"""
    def row(i, pitch, imu_pitch, h):
        return {'index': i, 'kind': 'ref', 'pan_mean': 2048.0, 'tilt_mean': 2305.0,
                'table_pitch_deg': pitch, 'table_roll_deg': 0.5, 'table_height_m': h,
                'imu_pitch_deg': imu_pitch, 'imu_roll_deg': 0.9}
    rows = [row(0, 30.87, 31.81, 0.3438), row(16, 31.88, 31.94, 0.3462)]
    bottles = [{'index': i, 'label': 'b1', 'n_points': 500, 'x': x, 'y': 0.0, 'z': 0.5, 'bearing_deg': 0.0}
               for i, x in ((0, 0.0), (16, 0.0021))]
    summary = '\n'.join(runner.format_summary(rows, bottles))
    assert '頭尾一致' in summary and '頭尾差太多' not in summary
    rows[1]['imu_pitch_deg'] = 32.4      # IMU 差 0.6°：真的動了
    assert '頭尾差太多' in '\n'.join(runner.format_summary(rows, bottles))
