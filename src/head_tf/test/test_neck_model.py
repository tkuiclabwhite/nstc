"""新平台三軸脖子模型（neck_model.py）的測試。"""
import copy
import math
import os

import numpy as np
import pytest
import yaml

from head_tf import neck_model as nm
from head_tf.head_model import load_extrinsics, matrix_rpy, rpy_matrix

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
CONFIG = os.path.join(PKG, 'config', 'neck_head.yaml')
EXTRINSICS = os.path.join(os.path.dirname(PKG), 'bottle_localization', 'config', 'camera_extrinsics.yaml')
URDF = os.path.expanduser('~/nstc_ws/seven_axis_arm_head_urdf/urdf/seven_axis_arm_head_urdf.urdf')
DEG_PER_TICK = 360.0 / 4096


@pytest.fixture
def config():
    with open(CONFIG, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.fixture
def model(config):
    # 和實際外參差不多的假外參：頭朝前、低頭約 30°。
    # 這組測試檢查 URDF 本身的幾何，所以拿掉量測修正（傾斜、範圍容許值），修正另外測
    config = dict(config, neck_tilt_deg=None, range_margin_ticks=0)
    R0 = rpy_matrix(math.radians(0.3), math.radians(30.4), 0.0)
    return nm.NeckModel(config, R0, [0.094, -0.001, 0.115])


def ref_with(model, **changes):
    ticks = dict(model.reference_ticks)
    for k, v in changes.items():
        ticks[int(k[1:])] = v
    return ticks


def camera_rpy_deg(model, ticks):
    R, _ = model.camera_link_pose(ticks)
    return [math.degrees(v) for v in matrix_rpy(R)]


@pytest.mark.skipif(not os.path.exists(URDF), reason='桌機沒有 seven_axis_arm_head_urdf')
def test_chain_matches_urdf(config):
    chain = nm.chain_from_urdf(URDF)
    assert [j['name'] for j in chain] == [j['name'] for j in config['chain']]
    for a, b in zip(chain, config['chain']):
        assert a['type'] == b['type']
        assert np.allclose(a['xyz'], b['xyz'], atol=1e-9) and np.allclose(a['rpy'], b['rpy'], atol=1e-9)
        if a['type'] != 'fixed':
            assert np.allclose(a['axis'], b['axis']) and np.allclose(a['limit'], b['limit'])
    center = nm.child_offset_from_urdf(URDF, 'head_Link', 'zed_camera_center')
    assert np.allclose(center, config['camera_center_in_head'], atol=1e-9)


def test_zero_posture_matches_urdf_readme(model):
    """URDF README：零位（三顆都 2048）時 zed_camera_center 在 base_link (0.0619, 0, 0.1662)。"""
    T = model.head_transform({28: 2048, 29: 2048, 30: 2048})
    assert np.allclose(T[:3, :3], np.eye(3))
    center = T[:3, :3] @ model.design_center + T[:3, 3]
    assert center == pytest.approx([0.0619, 0.0, 0.1662], abs=1e-4)


def test_reference_reproduces_extrinsics(model):
    R, t = model.camera_link_pose(model.reference_ticks)
    assert t == pytest.approx([0.094, -0.001, 0.115], abs=1e-12)
    assert np.allclose(R, rpy_matrix(math.radians(0.3), math.radians(30.4), 0.0), atol=1e-12)


def test_signs_match_observed_directions(model):
    r0, p0, y0 = camera_rpy_deg(model, model.reference_ticks)
    # 28 號刻度變大往左轉：yaw 變大
    assert camera_rpy_deg(model, ref_with(model, m28=2147))[2] > y0 + 8
    # 29 號刻度變小往下看：pitch 變大
    assert camera_rpy_deg(model, ref_with(model, m29=1608))[1] > p0 + 8
    # 30 號刻度變大頭往左歪：右邊抬高，roll（左邊抬高為正）變小
    assert camera_rpy_deg(model, ref_with(model, m30=2124))[0] < r0 - 8


def test_yaw_rotates_about_vertical_axis(model):
    """轉頭：相機繞 x = 0.00875、y = 0 的鉛直線轉，高度和離軸距離不變，yaw 變化 = 刻度差。"""
    _, c0 = model.optical_pose(model.reference_ticks)
    y0 = camera_rpy_deg(model, model.reference_ticks)[2]
    axis = np.array([0.00875, 0.0])
    for tick in (1877, 1950, 2150, 2218):
        ticks = ref_with(model, m28=tick)
        _, c = model.optical_pose(ticks)
        assert c[2] == pytest.approx(c0[2], abs=1e-12)
        assert np.linalg.norm(c[:2] - axis) == pytest.approx(np.linalg.norm(c0[:2] - axis), abs=1e-12)
        dyaw = (tick - model.reference_ticks[28]) * DEG_PER_TICK
        assert camera_rpy_deg(model, ticks)[2] == pytest.approx(y0 + dyaw, abs=1e-9)


def test_pitch_rotates_about_pitch_axis(model):
    """低頭：相機繞低頭軸（z = 0.052，跟著轉頭轉）轉，離軸距離不變；pitch 變化約 = 刻度差。"""
    ticks0 = model.reference_ticks
    # 低頭軸過 (0.00875, *, 0.052)，方向跟著轉頭（基準刻度）轉
    yaw = math.radians((ticks0[28] - 2048) * DEG_PER_TICK)
    direction = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    point = np.array([0.00875, 0.0, 0.052])

    def dist(c):
        v = c - point
        return np.linalg.norm(v - (v @ direction) * direction)

    _, c0 = model.optical_pose(ticks0)
    p0 = camera_rpy_deg(model, ticks0)[1]
    for tick in (1500, 1594, 1650, 1780, 1820):
        ticks = ref_with(model, m29=tick)
        _, c = model.optical_pose(ticks)
        assert dist(c) == pytest.approx(dist(c0), abs=1e-12)
        dpitch = -(tick - ticks0[29]) * DEG_PER_TICK
        assert camera_rpy_deg(model, ticks)[1] == pytest.approx(p0 + dpitch, abs=0.05)


def test_zero_tick_of_last_joint_does_not_matter(config):
    """歪頭固定時，歪頭和低頭的零點差多少都不影響（相機固定位置會吸收）。
    轉頭零點差 1°：低頭軸方向差 1°，允許範圍角落（低頭差 9°）相機角度差約 0.16°、位置 1 mm 內。"""
    R0 = rpy_matrix(0.005, 0.53, 0.0)
    t0 = [0.094, -0.001, 0.115]
    base = nm.NeckModel(config, R0, t0)
    moved = ref_with(base, m28=2200, m29=1600)
    Rb, tb = base.camera_link_pose(moved)
    shifted = copy.deepcopy(config)
    shifted['motors']['headRollJoint']['zero_tick'] = 2000
    shifted['motors']['neckPitchJoint']['zero_tick'] = 2100
    R, t = nm.NeckModel(shifted, R0, t0).camera_link_pose(moved)
    assert np.allclose(R, Rb, atol=1e-12) and np.allclose(t, tb, atol=1e-12)
    shifted = copy.deepcopy(config)
    shifted['motors']['neckYawJoint']['zero_tick'] = 2048 + 11      # 約 1°
    R, t = nm.NeckModel(shifted, R0, t0).camera_link_pose(moved)
    assert np.linalg.norm(t - tb) < 1e-3
    assert 0.1 < math.degrees(math.acos(min(1.0, (np.trace(R.T @ Rb) - 1) / 2))) < 0.2


def test_check_ranges(model):
    assert model.check(model.reference_ticks) == (True, [])
    assert model.check(ref_with(model, m28=1877, m29=1820))[0]
    ok, why = model.check(ref_with(model, m28=2219))
    assert not ok and '28 號' in why[0]
    ok, why = model.check(ref_with(model, m30=2040))
    assert not ok and '30 號' in why[0]
    ticks = dict(model.reference_ticks)
    del ticks[29]
    ok, why = model.check(ticks)
    assert not ok and '29 號' in why[0]


def test_range_margin(config):
    """29 號在最低點下垂幾格（neck_01：目標 1594 停在 1590）還算可用，超過容許值（6 格）才不行。"""
    m = nm.NeckModel(config, np.eye(3), np.zeros(3))
    assert m.margin == 6
    ticks = {**m.reference_ticks, 29: 1494}
    assert m.check(ticks)[0]
    ticks[29] = 1493
    ok, why = m.check(ticks)
    assert not ok and '29 號' in why[0] and '1500～1820' in why[0]


def test_neck_tilt(config):
    """轉頭軸傾斜修正：基準刻度時照樣重現外參；轉頭時轉軸是傾斜的線，相機 pitch 跟著轉頭變（neck_01 IMU 看到的）。"""
    R0 = rpy_matrix(math.radians(0.3), math.radians(30.4), 0.0)
    t0 = [0.094, -0.001, 0.115]
    m = nm.NeckModel(dict(config, neck_tilt_deg={'roll': -1.3, 'pitch': 0.63}), R0, t0)
    flat = nm.NeckModel(dict(config, neck_tilt_deg=None), R0, t0)
    assert m.neck_tilt_deg == (-1.3, 0.63) and flat.neck_tilt_deg == (0.0, 0.0)
    R, t = m.camera_link_pose(m.reference_ticks)
    assert np.allclose(R, R0, atol=1e-12) and np.allclose(t, t0, atol=1e-12)
    # 轉頭軸 = head_Link 往上的方向（在 base_link），往左傾（+y）、往前傾（+x）
    axis = m.head_transform({28: 2048, 29: 2048, 30: 2048})[:3, 2]
    assert axis[1] == pytest.approx(math.sin(math.radians(1.3)), abs=1e-3)
    assert axis[0] == pytest.approx(math.sin(math.radians(0.63)), abs=1e-3)

    def pitch(model, tick28):
        Rc, _ = model.camera_link_pose({**model.reference_ticks, 28: tick28})
        return math.degrees(matrix_rpy(Rc)[1])
    # 往左轉 15°：沒有修正時 pitch 不變；有修正時多往下約 0.3～0.4°（和 neck_01 的 IMU 同方向）
    assert pitch(flat, 2218) == pytest.approx(pitch(flat, 2047), abs=1e-9)
    assert 0.25 < pitch(m, 2218) - pitch(m, 2047) < 0.45
    assert -0.45 < pitch(m, 1877) - pitch(m, 2047) < -0.2


def test_bad_config_rejected(config):
    bad = copy.deepcopy(config)
    bad['reference_ticks'][29] = 1450
    with pytest.raises(ValueError, match='基準刻度'):
        nm.NeckModel(bad, np.eye(3), np.zeros(3))
    bad = copy.deepcopy(config)
    del bad['motors']['headRollJoint']
    with pytest.raises(ValueError, match='headRollJoint'):
        nm.NeckModel(bad, np.eye(3), np.zeros(3))


def test_real_config_is_consistent():
    """實際的設定檔：反推的相機位置要接近 URDF 設計值；左眼光心高度要和 table_plane 實測對得起來。"""
    model = nm.NeckModel.load(CONFIG, EXTRINSICS)
    d, rpy = model.mount_vs_design()
    assert np.linalg.norm(d) < 0.01
    assert abs(rpy[0]) < 3 and abs(rpy[1]) < 3 and abs(rpy[2]) < 3
    # 2026-11-16 table_plane：左眼光心離桌 52.4 cm；桌面 z_table = -0.39
    _, c = model.optical_pose(model.reference_ticks)
    assert c[2] + 0.39 == pytest.approx(0.524, abs=0.005)
    R0, t0 = load_extrinsics(EXTRINSICS)
    R, t = model.camera_link_pose(model.reference_ticks)
    assert np.allclose(R, R0, atol=1e-12) and np.allclose(t, t0, atol=1e-12)


def test_main_prints_summary(capsys):
    nm.main(['--config', CONFIG, '--extrinsics', EXTRINSICS])
    out = capsys.readouterr().out
    assert '基準刻度' in out and '設計值差' in out and out.count('位置 (') == 9
