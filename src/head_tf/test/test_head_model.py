import math
import os
import sys

import numpy as np
import pytest
import yaml

from head_tf import head_model as hm
from head_tf import sweep_core as core
from head_tf import sweep_runner as runner
from sim_head import HeadSim, SimIO

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'tools'))


def tilted_sim():
    """接近 sweep_01 的情況：pan 軸往前傾 7°，tilt 軸稍微歪，IMU 有安裝角。"""
    sim = HeadSim(pan_dir=(math.tan(math.radians(7.0)), 0.0, 1.0),
                  tilt_dir=(math.tan(math.radians(0.5)), 1.0, 0.0),
                  imu_rpy_deg=(0.3, 0.6, 0.0))
    sim.cam_pos0 = np.array([0.087, 0.0916, 0.1507])
    sim.pan_point = sim.cam_pos0 - np.array([0.062, 0.004, 0.0])
    sim.tilt_point = sim.cam_pos0 - np.array([0.013, 0.0, 0.046])
    return sim


def sim_model(sim):
    """模擬器的真值寫成 HeadModel。"""
    return hm.HeadModel(hm.Joint(sim.pan_dir, sim.cam_pos0 - sim.pan_point, sim.pan_sign),
                        hm.Joint(sim.tilt_dir, sim.cam_pos0 - sim.tilt_point, sim.tilt_sign))


def test_rpy_round_trip():
    R = hm.rpy_matrix(0.1, 0.5, -0.2)
    assert np.allclose(hm.matrix_rpy(R), (0.1, 0.5, -0.2))
    assert np.allclose(hm.axis_angle_matrix([0, 0, 1], 0.3), hm.rpy_matrix(0, 0, 0.3))


@pytest.mark.parametrize('pan,tilt', [(2048, 2300), (1934, 2186), (2162, 2414), (2000, 2350)])
def test_model_matches_independent_simulator(pan, tilt):
    sim = tilted_sim()
    model = sim_model(sim)
    R, t = model.camera_link_pose(sim.R_body0, sim.cam_pos0, pan, tilt)
    Rs, ts = sim.pose(pan, tilt)
    assert np.allclose(R, Rs, atol=1e-12) and np.allclose(t, ts, atol=1e-12)
    R_opt, c = model.optical_pose(sim.R_body0, sim.cam_pos0, pan, tilt)
    Rs_opt, cs = sim.optical(pan, tilt)
    assert np.allclose(R_opt, Rs_opt) and np.allclose(c, cs)


def test_reference_ticks_give_extrinsics():
    sim = tilted_sim()
    R, t = sim_model(sim).camera_link_pose(sim.R_body0, sim.cam_pos0, 2048, 2300)
    assert np.allclose(R, sim.R_body0) and np.allclose(t, sim.cam_pos0)


def test_yaml_round_trip(tmp_path):
    sim = tilted_sim()
    m = sim_model(sim)
    path = tmp_path / 'm.yaml'
    path.write_text(yaml.safe_dump(m.to_dict()), encoding='utf-8')
    m2 = hm.HeadModel.load(path)
    for pan, tilt in [(1934, 2186), (2162, 2414)]:
        a = m.camera_link_pose(sim.R_body0, sim.cam_pos0, pan, tilt)
        b = m2.camera_link_pose(sim.R_body0, sim.cam_pos0, pan, tilt)
        assert np.allclose(a[0], b[0], atol=1e-5) and np.allclose(a[1], b[1], atol=1e-5)


def test_from_measured_reads_head_axes():
    m = hm.HeadModel.from_measured(os.path.join(os.path.dirname(HERE), 'config', 'head_axes.yaml'))
    assert np.allclose(m.pan.offset, [0.065, 0.0, 0.0]) and np.allclose(m.tilt.offset, [0.01, 0.0, 0.05])
    assert np.allclose(m.pan.direction, [0, 0, 1]) and m.pan.sign == 1 and m.tilt.sign == 1


def make_sweep(tmp_path, sim, name='sweep'):
    io = SimIO(sim, noise_m=0.0008)
    out = tmp_path / name
    writer = runner.SweepWriter(out)
    writer.write_meta({'camera': {'size': [sim.width, sim.height]}})
    postures = core.build_postures([1934, 2048, 2162], [2186, 2243, 2300, 2357, 2414])
    runner.run_sweep(io, postures, runner.SweepConfig(), writer, lambda s: None)
    (out / 'robot_config').mkdir()
    r, p, y = hm.matrix_rpy(sim.R_body0)
    x, yy, z = sim.cam_pos0
    (out / 'robot_config' / 'camera_extrinsics.yaml').write_text(
        yaml.safe_dump({'x': float(x), 'y': float(yy), 'z': float(z),
                        'roll': float(r), 'pitch': float(p), 'yaw': float(y)}), encoding='utf-8')
    dims = tmp_path / 'dims.yaml'
    dims.write_text(yaml.safe_dump({'bottle_dims': {k: {'diameter': 2 * v[2], 'height': v[3]}
                                                    for k, v in sim.bottles.items()}}), encoding='utf-8')
    axes = tmp_path / 'axes.yaml'
    # 手量值故意差 1 cm 左右
    axes.write_text(yaml.safe_dump({
        'reference_ticks': {'pan': 2048, 'tilt': 2300},
        'pan_axis_to_screw_cm': {'forward': 7.0, 'left': 0.0},
        'tilt_axis_to_screw_cm': {'forward': 0.5, 'up': 5.5},
        'pan_sign': 1, 'tilt_sign': 1}), encoding='utf-8')
    return out, dims, axes


def sim_with_bottles():
    sim = tilted_sim()
    # 讓瓶子在頭轉到邊緣時也看得到（低解析度模擬器的視野比較小）
    sim.bottles = {'b1': (0.50, 0.06, 0.035, 0.10), 'b2': (0.56, 0.14, 0.030, 0.09),
                   'b3': (0.60, 0.02, 0.040, 0.12)}
    return sim


def fit_sim(tmp_path, sim, exact_points):
    import fit_head_model as fh
    out, dims, axes = make_sweep(tmp_path, sim)
    radii = fh.load_dims(dims)
    measured = hm.HeadModel.from_measured(axes)
    sweeps = [fh.load_sweep(str(out), radii, log=lambda s: None)]
    if exact_points:
        # 換成瓶子正面的真值（中心往相機走一個半徑、半高），排除 bottle_localization 算法本身的偏差
        by_index = {p['index']: p for p in sweeps[0]['postures']}
        for b in sweeps[0]['bottles']:
            p = by_index[b['index']]
            R_opt, c = sim.optical(p['pan'], p['tilt'])
            bx, by, r, h = sim.bottles[b['label']]
            d = c[:2] - np.array([bx, by])
            d /= np.linalg.norm(d)
            b['q'] = R_opt.T @ (np.array([bx + r * d[0], by + r * d[1], sim.z_table + h / 2]) - c)
    ext = hm.load_extrinsics(out / 'robot_config' / 'camera_extrinsics.yaml')
    layout, r, _ = fh.fit(sweeps, ext, radii, (1, 1), measured)
    model, imu, z_table, _ = layout.unpack(r.x, (1, 1))
    return fh, sweeps, ext, radii, model, imu, z_table


def angle_between(a, b):
    return math.degrees(math.acos(min(1.0, abs(float(a @ b)))))


def test_fit_recovers_simulated_head_exactly(tmp_path):
    """量測沒有偏差時，所有參數都要擬合回真值。"""
    pytest.importorskip('scipy')
    sim = sim_with_bottles()
    _, _, _, _, model, imu, z_table = fit_sim(tmp_path, sim, exact_points=True)
    truth = sim_model(sim)
    assert angle_between(model.pan.direction, truth.pan.direction) < 0.05
    assert angle_between(model.tilt.direction, truth.tilt.direction) < 0.1
    assert np.allclose(model.pan.offset[:2], truth.pan.offset[:2], atol=0.001)
    assert np.allclose(model.tilt.offset[[0, 2]], truth.tilt.offset[[0, 2]], atol=0.001)
    assert z_table['sweep'] == pytest.approx(sim.z_table, abs=0.0005)
    assert np.allclose(imu, sim.imu_mount, atol=math.radians(0.02))


def test_fit_with_localizer_points(tmp_path):
    """用 bottle_localization 的算法量瓶子（實際的情況）。

    相機往下看，偏離畫面中央的直立瓶子會變斜，框跟著偏，算出的位置每偏 1° 方位角約橫移 0.16 mm。
    頭左右轉時這個偏差和「相機橫移」分不開，所以 pan 軸的前後距離會偏差約 1 cm
    （對定位結果的影響約 1.6 mm）；其他參數和定位漂移都要正確。
    """
    pytest.importorskip('scipy')
    sim = sim_with_bottles()
    fh, sweeps, ext, radii, model, imu, z_table = fit_sim(tmp_path, sim, exact_points=False)
    truth = sim_model(sim)
    assert angle_between(model.pan.direction, truth.pan.direction) < 0.1
    assert angle_between(model.tilt.direction, truth.tilt.direction) < 0.5
    assert np.allclose(model.tilt.offset[[0, 2]], truth.tilt.offset[[0, 2]], atol=0.003)
    assert abs(model.pan.offset[0] - truth.pan.offset[0]) < 0.012
    assert abs(model.pan.offset[1] - truth.pan.offset[1]) < 0.003
    assert z_table['sweep'] == pytest.approx(sim.z_table, abs=0.001)
    assert np.allclose(imu, sim.imu_mount, atol=math.radians(0.05))
    # 漂移：固定外參（完全不管頭轉）會差好幾公分；擬合的模型要跟真值模型一樣好。
    # 真值模型也有幾 mm 的漂移，那是 bottle_localization 算法本身的偏差（量測的下限）
    drift = lambda m: fh.stats_mm(fh.drift(sweeps, ext, radii, lambda s, p: m))
    static, floor, fitted = drift(fh.static_model()), drift(truth), drift(model)
    assert static[0] > 50.0
    assert fitted[0] <= floor[0] + 0.5 and fitted[2] <= floor[2] + 0.5


def test_fit_tool_main_writes_yaml(tmp_path, capsys):
    pytest.importorskip('scipy')
    import fit_head_model as fh
    sim = sim_with_bottles()
    out, dims, axes = make_sweep(tmp_path, sim)
    target = tmp_path / 'head_model.yaml'
    assert fh.main([str(out), '--axes', str(axes), '--dims', str(dims), '--out', str(target), '--no-loo']) == 0
    text = capsys.readouterr().out
    assert '擬合的模型' in text and '固定外參' in text
    m = hm.HeadModel.load(target)
    R, t = m.camera_link_pose(sim.R_body0, sim.cam_pos0, 2162, 2414)
    Rs, ts = sim.pose(2162, 2414)
    # 位置誤差主要來自 pan 軸前後距離的偏差（見 test_fit_with_localizer_points）
    assert np.linalg.norm(t - ts) < 0.003
    assert math.degrees(math.acos(min(1.0, (np.trace(R.T @ Rs) - 1) / 2))) < 0.5
