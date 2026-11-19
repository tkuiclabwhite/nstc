"""新平台的收集流程（sweep_plan、30 號記錄）和驗證工具 check_neck_model.py 的測試。"""
import copy
import csv
import os
import sys

import numpy as np
import pytest

from head_tf import neck_model as nm
from head_tf import sweep_core as core
from head_tf import sweep_runner as runner
from sim_neck import NeckSim, NeckSimIO, R0, T0, Z_TABLE, load_config

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'tools'))
import check_neck_model as check  # noqa: E402
import fit_head_model as fit  # noqa: E402

RADII = {'b1': 0.041, 'b2': 0.025, 'b3': 0.022}


def test_sweep_plan_from_config():
    plan = nm.sweep_plan(load_config())
    assert (plan['pan_id'], plan['tilt_id'], plan['roll_id']) == (28, 29, 30)
    assert plan['reference'] == (2047, 1708) and plan['roll_reference'] == 2024
    assert plan['pan'] == [1877, 2047, 2218]
    assert plan['tilt'] == [1594, 1651, 1708, 1764, 1820]
    assert plan['pan_limits'] == (1877, 2218) and plan['tilt_limits'] == (1594, 1820)
    assert plan['roll_limits'] == (2014, 2034)
    postures = core.build_postures(plan['pan'], plan['tilt'], plan['reference'])
    assert len(postures) == 17
    core.check_postures(postures, plan['pan_limits'], plan['tilt_limits'])
    with pytest.raises(ValueError):
        core.check_postures([('grid', 2250, 1708)], plan['pan_limits'], plan['tilt_limits'])
    assert nm.sweep_plan(load_config(), tilt_steps=3)['tilt'] == [1594, 1708, 1820]
    with pytest.raises(ValueError):
        nm.sweep_plan(load_config(), tilt_steps=4)


def collect(tmp_path, sim, name='neck'):
    """跑一次完整的模擬收集，回資料夾路徑。"""
    plan = nm.sweep_plan(load_config())
    postures = core.build_postures(plan['pan'], plan['tilt'], plan['reference'])
    writer = runner.SweepWriter(tmp_path / name)
    writer.write_meta({'name': name, 'camera': {'size': [sim.width, sim.height]}})
    io = NeckSimIO(sim)
    rows, bottles = runner.run_sweep(io, postures, runner.SweepConfig(), writer, lambda s: None)
    writer.finish('finished')
    return tmp_path / name, rows, bottles, io


@pytest.fixture(scope='module')
def good_sweep(tmp_path_factory):
    return collect(tmp_path_factory.mktemp('good'), NeckSim())


def test_roll_is_recorded(good_sweep):
    path, rows, bottles, io = good_sweep
    assert all(r['roll_mean'] == 2024 and r['roll_min'] == 2024 for r in rows)
    with open(path / 'postures.csv', encoding='utf-8') as f:
        assert float(next(csv.DictReader(f))['roll_mean']) == 2024
    assert np.load(path / 'posture_00.npz')['roll'].shape[1] == 2
    summary = '\n'.join(runner.format_summary(rows, bottles))
    assert '歪頭（30 號）：2024～2024（變化 0 格），沒有動' in summary
    # 新平台：29 號刻度變大往上看、28 號刻度變大往左轉
    assert '刻度變大 → 往上看' in summary and '刻度變大 → 往左轉' in summary
    assert '頭尾一致' in summary
    assert io.sent[-1][:2] == (2047, 1708)


def test_roll_drift_is_flagged():
    rows = [{'roll_min': 2024.0, 'roll_max': 2025.0}, {'roll_min': 2027.0, 'roll_max': 2029.0}]
    lines = runner.roll_check(rows)
    assert '變化 5 格' in lines[0] and '動了超過' in lines[1]
    assert runner.roll_check([{'roll_min': float('nan'), 'roll_max': float('nan')}]) == []


def evaluate(path, config=None):
    model = nm.NeckModel(config or load_config(), R0, T0)
    data = fit.load_sweep(str(path), RADII, log=lambda s: None)
    return check.evaluate(data, model, RADII, Z_TABLE, check.load_roll(str(path))), model


def test_correct_model_passes(good_sweep):
    path = good_sweep[0]
    result, model = evaluate(path)
    # 每個非基準姿勢都看得到三個瓶子
    assert result['bottle']['n'] == 15 * 3
    # 模型正確：剩下的是 bottle_localization 算法本身的偏差（和藥罐在畫面中的位置有關）和深度雜訊。
    # 轉頭 15° 時 b1（最大的瓶子）最多約 6 mm
    assert result['bottle']['median'] < 2.0 and result['bottle']['p95'] < 5.0 and result['bottle']['max'] < 8.0
    # 沒有動態 TF：轉頭 15°、低頭 10° 時藥罐差好幾公分
    assert result['static']['max'] > 50
    assert result['imu_roll']['max'] < 0.01 and result['imu_pitch']['max'] < 0.01
    assert result['table']['max'] < 1.5
    assert abs(result['offsets']['table_m']) < 0.002
    lines = check.report(result, model)
    assert any('模型可以直接用' in s for s in lines)
    assert sum(1 for s in lines if s[:3].strip().isdigit()) == 17


def test_tilted_stand_is_detected(tmp_path):
    """真的支架往前傾 2°（設定檔以為是正的）：轉頭時 IMU roll 跟著變，藥罐也會偏。"""
    truth = copy.deepcopy(load_config())
    truth['chain'][0]['rpy'] = [0.0, 0.035, 0.0]
    path = collect(tmp_path, NeckSim(truth))[0]
    result, model = evaluate(path)
    assert result['imu_roll']['max'] > 0.4
    assert result['bottle']['max'] > result['imu_roll']['max']


def test_main_runs(good_sweep, capsys, tmp_path):
    path = good_sweep[0]
    cfg = os.path.join(os.path.dirname(check.DEFAULTS['config']), 'neck_head.yaml')
    ext = tmp_path / 'ext.yaml'
    ext.write_text('x: 0.094\ny: -0.001\nz: 0.115\nroll: 0.005235987755982988\n'
                   'pitch: 0.5305800926062943\nyaw: 0.0\n', encoding='utf-8')
    params = tmp_path / 'params.yaml'
    params.write_text('bottle_localizer:\n  ros__parameters:\n    table_height_cm: 80.5\n', encoding='utf-8')
    ref = tmp_path / 'ref.yaml'
    ref.write_text('reference_point: [0.0, 0.0, 0.0]\nreference_height_cm: 119.5\n', encoding='utf-8')
    dims = tmp_path / 'dims.yaml'
    dims.write_text('bottle_dims:\n  b1: {diameter: 0.082, height: 0.092}\n  b2: {diameter: 0.05, height: 0.078}\n'
                    '  b3: {diameter: 0.044, height: 0.09}\n', encoding='utf-8')
    assert check.main([str(path), '--config', cfg, '--extrinsics', str(ext), '--params', str(params),
                       '--reference', str(ref), '--dims', str(dims)]) == 0
    out = capsys.readouterr().out
    assert 'z_table = -0.3900' in out and '模型可以直接用' in out


def test_resolve_files_prefers_sweep_copy(tmp_path):
    (tmp_path / 'robot_config').mkdir()
    (tmp_path / 'robot_config' / 'neck_head.yaml').write_text('x', encoding='utf-8')
    files = check.resolve_files(str(tmp_path), {'params': '/given/params.yaml'})
    assert files['config'][1] == '收集當時' and files['params'] == ('/given/params.yaml', '指定')
    assert files['extrinsics'][1] == 'repo'
