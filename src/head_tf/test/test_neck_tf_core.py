"""動態 TF 計算部分（neck_tf_core.py）的測試：不用 ROS。"""
import math
import os

import numpy as np
import pytest

from head_tf import neck_model as nm
from head_tf.head_model import load_extrinsics, matrix_rpy, rpy_matrix
from head_tf.neck_tf_core import NeckTfCore, matrix_to_quaternion, parse_joint_state

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(os.path.dirname(HERE), 'config', 'neck_head.yaml')
R0 = rpy_matrix(0.0051, 0.5301, 0.0)
T0 = np.array([0.0937, -0.0006, 0.1155])
REF = {28: 2047.0, 29: 1708.0, 30: 2024.0}


@pytest.fixture
def model():
    return nm.NeckModel(nm.load_config(CONFIG), R0, T0)


def quat_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def test_parse_joint_state():
    names = ['1', '28', '29', '30']
    ticks, missing = parse_joint_state(names, [2048, 2047, 1708, 2024], [28, 29, 30])
    assert ticks == REF and missing == []
    # driver_node 的速度、電流那兩筆 position 是空的
    assert parse_joint_state(names, [], [28, 29, 30]) == (None, [])
    assert parse_joint_state([], [], [28, 29, 30]) == (None, [])
    # MoveIt（joint_state_broadcaster）的 /joint_states：手臂關節名、弧度，完全沒有頭部馬達 -> 略過
    assert parse_joint_state(['Lshoulder1Joint', 'Rshoulder1Joint'], [0.1, -0.2], [28, 29, 30]) == (None, [])
    # driver_node 但頭部馬達沒上電：只有手臂的 ID -> 也略過（交給「超過 0.5 秒沒收到」判斷）
    assert parse_joint_state([str(i) for i in range(1, 15)], [2048.0] * 14, [28, 29, 30]) == (None, [])
    ticks, missing = parse_joint_state(['28', '29'], [2047, 1708], [28, 29, 30])
    assert missing == [30] and ticks == {28: 2047.0, 29: 1708.0}
    assert parse_joint_state(['28', '29', '30'], [2047, float('nan'), 2024], [28, 29, 30])[1] == [29]


@pytest.mark.parametrize('rpy', [(0, 0, 0), (0.3, -1.2, 2.5), (math.pi, 0.1, -0.2), (0.1, math.pi / 2 - 1e-3, 0),
                                 (-2.9, 0.4, 3.1), (0.0051, 0.5301, 0.0)])
def test_quaternion_roundtrip(rpy):
    R = rpy_matrix(*rpy)
    q = matrix_to_quaternion(R)
    assert abs(np.linalg.norm(q) - 1) < 1e-12 and q[3] >= 0
    assert np.allclose(quat_matrix(*q), R, atol=1e-9)


def test_reference_pose_equals_extrinsics(model):
    core = NeckTfCore(model)
    ok, (R, t) = core.update(REF, [], 10.0)
    assert ok
    assert np.allclose(R, R0, atol=1e-9) and np.allclose(t, T0, atol=1e-9)
    s = core.status(10.1)
    assert s['valid'] and s['reasons'] == [] and s['at_reference'] and not s['moving']
    assert s['camera']['x'] == pytest.approx(0.0937) and s['camera']['pitch_deg'] == pytest.approx(30.37, abs=0.01)
    assert s['ticks'] == {'28': 2047.0, '29': 1708.0, '30': 2024.0}
    assert s['reference_ticks'] == {'28': 2047.0, '29': 1708.0, '30': 2024.0} and s['margin_ticks'] == 6
    assert s['allowed_ticks']['28'] == [1877.0, 2218.0] and s['allowed_ticks']['30'] == [2014.0, 2034.0]
    import json
    json.dumps(s)          # 要能變成 JSON


def test_real_extrinsics_file_reproduced():
    """bottle_localization 的 camera_extrinsics.yaml（同一個 repo）在基準刻度時原封不動。"""
    path = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'bottle_localization', 'config', 'camera_extrinsics.yaml')
    if not os.path.exists(path):
        pytest.skip('沒有 bottle_localization')
    R0f, t0f = load_extrinsics(path)
    core = NeckTfCore(nm.NeckModel.load(CONFIG, path))
    ok, (R, t) = core.update(nm.load_config(CONFIG)['reference_ticks'], [], 0.0)
    assert ok and np.allclose(R, R0f, atol=1e-9) and np.allclose(t, t0f, atol=1e-9)


def test_turning_changes_pose(model):
    core = NeckTfCore(model)
    _, (R_ref, t_ref) = core.update(REF, [], 0.0)
    ok, (R, t) = core.update({28: 2218.0, 29: 1708.0, 30: 2024.0}, [], 1.0)
    assert ok
    yaw = math.degrees(matrix_rpy(R)[2] - matrix_rpy(R_ref)[2])
    assert yaw == pytest.approx(15.0, abs=0.5)        # 28 號 +171 格 = 往左轉約 15°
    ok, (R, t) = core.update({28: 2047.0, 29: 1594.0, 30: 2024.0}, [], 2.0)
    assert ok and math.degrees(matrix_rpy(R)[1] - matrix_rpy(R_ref)[1]) == pytest.approx(10.0, abs=0.5)


def test_out_of_range_stops(model):
    core = NeckTfCore(model)
    ok, pose = core.update({28: 2300.0, 29: 1708.0, 30: 2024.0}, [], 0.0)
    assert not ok and pose is None
    s = core.status(0.1)
    assert not s['valid'] and '28 號馬達刻度 2300 超出允許範圍' in s['reasons'][0] and 'camera' not in s
    # 容許值內（29 號目標 1500、下垂到 1494）還是可以用，再低就不行
    assert core.update({28: 2047.0, 29: 1494.0, 30: 2024.0}, [], 1.0)[0]
    assert not core.update({28: 2047.0, 29: 1493.0, 30: 2024.0}, [], 2.0)[0]


def test_missing_motor(model):
    core = NeckTfCore(model)
    ok, pose = core.update({28: 2047.0, 29: 1708.0}, [30], 0.0)
    assert not ok and pose is None
    assert '沒有 30 號馬達' in core.status(0.0)['reasons'][0]
    assert core.status(0.0)['ticks'] == {'28': 2047.0, '29': 1708.0}


def test_not_received_and_stale(model):
    core = NeckTfCore(model, stale_s=0.5)
    s = core.status(0.0)
    assert not s['valid'] and '還沒收到有 28/29/30 號刻度的 /joint_states' in s['reasons'][0] and s['age_s'] is None
    core.update(REF, [], 10.0)
    assert core.status(10.4)['valid']
    s = core.status(10.8)
    assert not s['valid'] and '超過 0.5 秒沒收到 28/29/30 號的刻度' in s['reasons'][0] and s['age_s'] == pytest.approx(0.8)
    core.update(REF, [], 11.0)
    assert core.status(11.0)['valid']


def test_moving(model):
    core = NeckTfCore(model, still_ticks=3, still_s=0.3)
    for i in range(10):          # 馬達抖 ±1 格不算在動
        core.update({28: 2047.0 + (i % 2), 29: 1708.0, 30: 2024.0 - (i % 2)}, [], i * 0.02)
    assert not core.status(0.2)['moving']
    for i in range(10):          # 轉頭中
        core.update({28: 2047.0 + 5 * i, 29: 1708.0, 30: 2024.0}, [], 1.0 + i * 0.02)
    s = core.status(1.2)
    assert s['valid'] and s['moving'] and not s['at_reference']
    for i in range(20):          # 停下來 0.3 秒後就不算在動
        core.update({28: 2092.0, 29: 1708.0, 30: 2024.0}, [], 2.0 + i * 0.02)
    assert not core.status(2.4)['moving']


def test_set_model_recomputes(model):
    core = NeckTfCore(model)
    core.update(REF, [], 0.0)
    t_new = T0 + np.array([0.01, 0.0, 0.0])
    core.set_model(nm.NeckModel(nm.load_config(CONFIG), R0, t_new))
    assert np.allclose(core.pose[1], t_new)
    assert core.status(0.1)['camera']['x'] == pytest.approx(0.1037)
