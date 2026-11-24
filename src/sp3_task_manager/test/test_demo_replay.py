"""demo_replay（pose_demo --replay 的錄放）單元測試，純 numpy。"""
import numpy as np
import pytest

from sp3_task_manager import demo_replay as dr


def test_interp_step_and_ends():
    q0, q1 = np.zeros(7), np.radians([10, -4.5, 0, 0, 0, 0, 0])
    P = dr.interp(q0, q1, step_deg=1.0)
    assert len(P) == 11
    assert np.allclose(P[0], q0) and np.allclose(P[-1], q1)
    assert max(dr.deg(a, b) for a, b in zip(P[:-1], P[1:])) <= 1.0 + 1e-9


def test_interp_same_pose():
    q = np.ones(7)
    P = dr.interp(q, q)
    assert len(P) == 2 and np.allclose(P, q)


def test_thin_keeps_ends_and_drops_duplicates():
    a = np.zeros(7)
    b = a + np.radians(5)
    P = [a, a, a + 1e-4, b, b, b]
    T = dr.thin(P)
    assert np.allclose(T[0], a) and np.allclose(T[-1], b)
    assert len(T) == 2


def test_sim_path_accumulates_partial_and_skips_real():
    joints = [f'j{i}' for i in range(7)]
    samples = [(0.0, 'real', {1: 2048}),
               (0.1, 'sim', {j: 0.0 for j in joints[:3]}),          # 還沒齊
               (0.2, 'sim', {j: 0.0 for j in joints[3:]}),          # 齊了 → 第一點
               (0.3, 'sim', {'j0': 0.5, 'other': 9.0}),
               (0.4, 'sim', {'j0': 0.5})]
    P = dr.sim_path(samples, joints)
    assert P.shape == (2, 7)
    assert P[0][0] == 0.0 and P[-1][0] == 0.5
    assert dr.sim_path([(0, 'real', {1: 1})], joints) is None


def test_start_from_small_and_big_difference():
    P = dr.interp(np.zeros(7), np.radians([20] + [0] * 6))
    near = P[0] + np.radians(0.2)
    R, head = dr.start_from(P, near)
    assert head is None and np.allclose(R[0], near) and len(R) == len(P)
    far = P[0] + np.radians(5)
    R, head = dr.start_from(P, far)
    assert head is not None and np.allclose(R[0], far) and np.allclose(R[-1], P[-1])
    assert np.allclose(head[-1], P[0])


def _steps():
    q1 = np.radians([30, 0, 0, 0, 0, 0, 0])
    q2 = np.radians([30, 20, 0, 0, 0, 0, 0])
    rec1 = dr.interp(np.zeros(7), q1, 0.5)                 # 上次錄到的路線
    return [{'label': 'a', 'test': False, 'q': q1, 'path': rec1},
            {'label': 't', 'test': True, 'q': None, 'path': None, 'result': 'blocked'},
            {'label': 'b', 'test': False, 'q': q2, 'path': dr.interp(q1, q2, 0.5)}]


def test_segments_direct_when_clear():
    segs = dr.segments(_steps(), np.zeros(7), lambda P: None)
    assert [None if g is None else g['how'] for g in segs] == ['direct', None, 'direct']
    assert np.allclose(segs[2]['P'][0], _steps()[0]['q'])      # 被擋下的測試步不動：從上一個到了的姿勢接著走


def test_segments_fallback_to_recorded():
    def check(P):          # 第一軸過 20° 時「會碰到」，只有直接轉的那段（第二軸同時在動）以外都放行
        return '碰到' if len(P) > 0 and np.degrees(P[:, 1]).max() > 1 and np.degrees(P[:, 0]).max() > 25 else None
    segs = dr.segments(_steps(), np.zeros(7), check)
    assert segs[0]['how'] == 'direct'
    assert segs[2]['how'] == 'recorded' and segs[2]['why'] == '碰到'
    assert np.allclose(segs[2]['P'][-1], _steps()[2]['q'])


def test_segments_no_route():
    steps = _steps()
    steps[0]['path'] = None
    segs = dr.segments(steps, np.zeros(7), lambda P: '碰到')
    assert segs[0]['how'] is None and '沒錄到' in segs[0]['why']


def test_save_load_roundtrip(tmp_path):
    steps = _steps()
    data = {'arm': 'right', 'joints': [f'j{i}' for i in range(7)], 'follow': True, 'ready_q': np.zeros(7),
            'steps': [dict(s, pos=np.array([0.3, -0.2, 0.1]), R=np.eye(3)) for s in steps]}
    dr.save(str(tmp_path), data)
    got = dr.load(str(tmp_path))
    assert got['version'] == dr.VERSION and got['follow'] is True
    assert np.allclose(got['steps'][0]['q'], steps[0]['q'])
    assert got['steps'][1]['q'] is None


def test_load_errors(tmp_path):
    with pytest.raises(ValueError):
        dr.load(str(tmp_path))                                  # 沒有 poses.json
    dr.save(str(tmp_path), {'steps': [{'q': None}]})
    with pytest.raises(ValueError):
        dr.load(str(tmp_path))                                  # 一步都沒到
