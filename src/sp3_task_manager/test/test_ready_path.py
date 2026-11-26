"""錄好的 ready 路線（ready_path.py，不需要 ROS）。

    cd ~/nstc/src/sp3_task_manager && python3 -m pytest test -q
"""
import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sp3_task_manager import ready_path as rp  # noqa: E402

CFG = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), '..', 'config',
                                       'task_manager.yaml'), encoding='utf-8'))
D = np.radians


def fake_paths():
    """站姿（全 0）→ 某個 ready：左手先繞一下（第 2 軸先轉 40° 再回 20°），右手直直轉。"""
    left = np.vstack([np.zeros((5, 7)),                                     # 一開始停著（右手在動時也停著）
                      np.linspace(np.zeros(7), [0, D(40), 0, 0, 0, 0, 0], 30),
                      np.linspace([0, D(40), 0, 0, 0, 0, 0], [D(10), D(20), 0, D(60), 0, 0, 0], 40),
                      np.repeat([[D(10), D(20), 0, D(60), 0, 0, 0]], 20, axis=0)])
    right = np.vstack([np.zeros((50, 7)), np.linspace(np.zeros(7), [0, D(-30), 0, D(70), 0, 0, 0], 40)])
    return {'left': left, 'right': right}


def test_resample_keeps_ends_and_step():
    P = rp.resample(fake_paths()['left'])
    assert np.allclose(P[0], 0) and np.allclose(P[-1], [D(10), D(20), 0, D(60), 0, 0, 0])
    steps = np.abs(np.diff(P, axis=0)).max(axis=1)
    assert steps.max() <= rp.SAVE_STEP + 1e-9
    assert len(P) < 200                     # 停著的點拿掉了


def test_merge_moves_together():
    paths = {a: rp.resample(p) for a, p in fake_paths().items()}
    names, P = rp.merge(paths)
    assert names == rp.joint_names('left') + rp.joint_names('right')
    assert P.shape[1] == 14
    assert np.allclose(P[0], 0) and np.allclose(P[-1, :7], paths['left'][-1]) and np.allclose(P[-1, 7:], paths['right'][-1])
    mid = P[len(P) // 2]
    assert not np.allclose(mid[:7], 0) and not np.allclose(mid[7:], 0)   # 中間兩隻手都在動（同時走）


def test_time_path_limits():
    names, P = rp.merge({a: rp.resample(p) for a, p in fake_paths().items()})
    P = rp.resample(P, rp.CHECK_STEP)
    t, V = rp.time_path(P, 30)
    v_max, _ = rp.limits(30)
    assert np.all(np.diff(t) > 0)
    assert np.abs(V).max() <= v_max * 1.05
    assert np.allclose(V[0], 0) and np.allclose(V[-1], 0)
    # 速度 30 ≈ 0.72 rad/s：最快那一軸總共 70° 左右，含加減速要幾秒，不會是幾十秒
    assert 2.0 < t[-1] < 10.0


def test_plan_ready_and_stand():
    rec = {'paths': {a: rp.resample(p) for a, p in fake_paths().items()}}
    end = {a: rec['paths'][a][-1] for a in rp.ARMS}
    stand = {a: np.zeros(7) for a in rp.ARMS}
    # 站姿 → ready：照錄的路線，第一點換成目前姿勢
    near = {a: np.full(7, D(1)) for a in rp.ARMS}
    out = rp.plan(rec, near, rp.ARMS, 'ready')
    assert set(out) == {'left', 'right'}
    assert np.allclose(out['left'][0], D(1)) and np.allclose(out['left'][1:], rec['paths']['left'][1:])
    # 已經在 ready：不用動
    assert rp.plan(rec, end, rp.ARMS, 'ready') == {}
    # ready → 站姿：倒過來
    back = rp.plan(rec, end, rp.ARMS, 'stand')
    assert np.allclose(back['right'][-1], 0) and np.allclose(back['right'][1:], rec['paths']['right'][::-1][1:])
    assert rp.plan(rec, stand, rp.ARMS, 'stand') == {}
    # 在別的地方：回 ready 直接轉；回站姿先轉到 ready 再倒過來
    other = {'left': end['left'] + D(15), 'right': end['right']}
    out = rp.plan(rec, other, rp.ARMS, 'ready')
    assert set(out) == {'left'} and np.allclose(out['left'][-1], end['left'])
    out = rp.plan(rec, other, ('left',), 'stand')
    assert np.allclose(out['left'][0], other['left']) and np.allclose(out['left'][-1], 0)
    # 只要一隻手
    assert set(rp.plan(rec, stand, ('right',), 'ready')) == {'right'}


def test_save_load(tmp_path):
    f = str(tmp_path / 'ready_path.yaml')
    rp_cfg = dict(CFG['ready_pose'], mirror=None)     # 各用各的路線
    paths = {a: rp.resample(p) for a, p in fake_paths().items()}
    rp.save(f, paths, rp_cfg, False, {'left': 4.8, 'right': 4.8})
    rec, why = rp.load(f, rp_cfg)
    assert why is None
    assert np.allclose(rec['paths']['left'], paths['left'], atol=1e-4)
    assert rec['real_follow'] is False
    # 位置改了 → 不能用
    moved = dict(rp_cfg, right=[30.0, 15.0, 15.0])
    assert rp.load(f, moved)[0] is None and '重錄' in rp.load(f, moved)[1]
    # 沒錄
    assert rp.load(str(tmp_path / 'none.yaml'), rp_cfg)[0] is None


def test_describe():
    text = rp.describe({a: rp.resample(p) for a, p in fake_paths().items()}, 30)
    assert '左手' in text and '右手' in text and '秒' in text


def test_mirror(tmp_path):
    rp_cfg = CFG['ready_pose']
    assert rp.mirror_src(rp_cfg) == 'right'
    assert rp.mirror_src(dict(rp_cfg, mirror=None)) is None
    with pytest.raises(ValueError):
        rp.mirror_src(dict(rp_cfg, right=[30.0, 15.0, 15.0]))      # 左右不對稱
    with pytest.raises(ValueError):
        rp.mirror_src(dict(rp_cfg, mirror='both'))
    f = str(tmp_path / 'ready_path.yaml')
    paths = {a: rp.resample(p) for a, p in fake_paths().items()}
    rp.save(f, paths, dict(rp_cfg, mirror=None), True, {})
    rec, why = rp.load(f, rp_cfg)                                   # 各錄各的舊檔：mirror = right 時左手用右手的
    assert why is None and np.allclose(rec['paths']['left'], rec['paths']['right'])
    rec, _ = rp.load(f, dict(rp_cfg, mirror=None))
    assert not np.allclose(rec['paths']['left'][-1], rec['paths']['right'][-1])


URDF = os.path.join(os.path.dirname(__file__), '..', '..', 'armpackage', 'seven_axis_arm_urdf', 'urdf',
                    'seven_axis_arm_urdf.urdf')


@pytest.mark.skipif(not os.path.exists(URDF), reason='沒有 armpackage 的 URDF')
def test_adjust_shifts_palm():
    from sp3_task_manager.arm_shape import ArmShape
    from sp3_task_manager import grasp_plan as gp
    fk = ArmShape(URDF, lambda pkg: None, meshes=False)
    P = rp.resample(np.linspace(D([0, -30, 0, 60, 30, 0, 0]), D([20, -50, 30, 50, 50, 40, -40]), 50))
    edge = gp.from_meas([-1.0, 0, 0], CFG['frames']) - gp.from_meas([0, 0, 0], CFG['frames'])   # 往後 1 cm
    end = gp.from_meas([0, 0, 1.0], CFG['frames']) - gp.from_meas([0, 0, 0], CFG['frames'])     # 往上 1 cm
    Q = rp.adjust(P, 'left', edge, end, fk)
    s = rp.arc(P) / rp.arc(P)[-1]
    we, wn = rp.weights(s)
    assert np.allclose(Q[0], P[0])
    for i in (0, int(np.argmax(we)), len(P) - 1):
        (p0, R0), (p1, R1) = rp.palm_pose(fk, 'left', P[i]), rp.palm_pose(fk, 'left', Q[i])
        assert np.allclose(p1 - p0, edge * we[i] + end * wn[i], atol=2e-4)
        assert np.linalg.norm(rp._rotvec(R1 @ R0.T)) < 2e-3
    assert we.max() == pytest.approx(1.0) and we[-1] == 0 and wn[-1] == 1


def stairs(n=6, step=8.0):
    """站姿往外走的階梯（第 1、2 軸輪流各轉 step°）：轉角很多的折線，像 MoveIt 規劃出來的路線。"""
    pts, q = [np.zeros(7)], np.zeros(7)
    for i in range(n):
        q = q.copy()
        q[i % 2] += D(step)
        pts.append(q)
    return rp.resample(np.array(pts))


def test_smooth_keeps_ends_rounds_corners():
    P = stairs()
    Q = rp.smooth(P)
    assert np.allclose(Q[0], P[0]) and np.allclose(Q[-1], P[-1])
    # 兩端的方向不變（奇對稱延伸）：起點往第 1 軸走、終點從第 2 軸進來
    d0, d1 = Q[1] - Q[0], Q[-1] - Q[-2]
    assert abs(d0[0]) > 5 * abs(d0[1]) and abs(d1[1]) > 5 * abs(d1[0])
    # 轉角磨掉了（不會再經過折點），但離原路線不遠（σ 5°、每段 8°）
    corner = P[np.argmin(np.abs(rp.arc(P) - D(16)))]           # 第 2 個折點 = (16°, 8°)
    gap = np.abs(Q - corner).max(axis=1).min()
    assert D(0.3) < gap < D(4.0)
    # 細切過的：點距約 SMOOTH_FINE，prepare 之後每 CHECK_STEP 一點
    assert np.abs(np.diff(Q, axis=0)).max() <= rp.SMOOTH_FINE * 1.01
    R = rp.prepare(P)
    assert np.abs(np.diff(R, axis=0)).max(axis=1).max() <= rp.CHECK_STEP + 1e-9
    assert np.allclose(R[0], P[0]) and np.allclose(R[-1], P[-1])


def test_smooth_short_paths():
    one = np.zeros((1, 7))
    two = np.vstack([np.zeros(7), np.full(7, D(1))])
    assert np.allclose(rp.smooth(one), one) and np.allclose(rp.smooth(two), two)
    short = rp.resample(np.vstack([np.zeros(7), [D(1), 0, 0, 0, 0, 0, 0], [D(1), D(1), 0, 0, 0, 0, 0]]), D(0.5))
    Q = rp.smooth(short)                                        # 比 σ 短很多也不會出錯
    assert np.allclose(Q[0], short[0]) and np.allclose(Q[-1], short[-1]) and np.all(np.isfinite(Q))


def dips(t, V):
    """最快那一軸的速度在中間掉下去又加速幾次（局部最低點，掉 10% 以上才算）。"""
    v = np.abs(V).max(axis=1)
    n = 0
    for i in range(1, len(v) - 1):
        if v[i] < v[i - 1] and v[i] <= v[i + 1]:
            left, right = v[:i].max(), v[i + 1:].max()
            if v[i] < 0.9 * min(left, right):
                n += 1
    return n


def test_ready_timing_smoother():
    P = stairs()
    t0, V0 = rp.time_path(rp.resample(P, rp.CHECK_STEP), 50)             # 原本的：每個轉角都減速
    R = rp.prepare(P)
    t1, V1 = rp.time_path(R, 50, **rp.READY_TIMING)
    v_max, _ = rp.limits(50)
    assert dips(t0, V0) >= 3 and dips(t1, V1) <= 1
    assert t1[-1] < t0[-1]
    assert np.all(np.diff(t1) > 0)
    assert np.abs(V1).max() <= v_max * 1.05
    assert np.allclose(V1[0], 0) and np.allclose(V1[-1], 0)
    acc = np.abs(np.diff(V1, axis=0) / np.diff(t1)[:, None]).max()
    assert acc <= rp.MAX_ACC * 1.15                                      # 有限差分的誤差
    # 預設參數 = 原本的排法（pose_demo 重播用）
    t2, V2 = rp.time_path(rp.resample(P, rp.CHECK_STEP), 50, corner_k=1.0, plateau_deg=0.0)
    assert np.allclose(t2, t0) and np.allclose(V2, V0)


def test_real_z_comp_formula():
    cfg = CFG['ready_pose']
    assert rp.real_z_comp_cm(cfg, 10.0) == 0.0                      # 前 14.5 以內不補
    assert rp.real_z_comp_cm(cfg, 25.4) == pytest.approx(0.14 * (25.4 - 14.5))   # 11-26 實量：要補 ≈ 1.4
    assert 1.2 < rp.real_z_comp_cm(cfg, 25.4) < 1.8
    assert rp.real_z_comp_cm({}, 50.0) == pytest.approx(0.14 * 35.5)   # 沒設定 = 預設同一組


def test_real_z_comp_matches_moveit_bridge():
    """moveit_bridge 的 REAL_Z_COMP、REAL_Z_FROM 跟設定檔要一樣（任務管理只看模擬時自己補、pose_demo 算實機位置都靠這個）。"""
    import re
    src = os.path.join(os.path.dirname(__file__), '..', '..', 'armpackage', 'armpackage', 'scripts', 'moveit_bridge.py')
    if not os.path.exists(src):
        pytest.skip('找不到 moveit_bridge.py')
    text = open(src, encoding='utf-8').read()
    k = float(re.search(r'^REAL_Z_COMP\s*=\s*([0-9.]+)', text, re.M).group(1))
    f0 = float(re.search(r'^REAL_Z_FROM\s*=\s*([0-9.]+)', text, re.M).group(1))
    assert k == pytest.approx(CFG['ready_pose']['real_z_comp'])
    assert f0 == pytest.approx(CFG['ready_pose']['real_z_from_cm'])


def test_time_path_accel_override():
    """ready_pose.accel：加速度不跟著速度縮。比較快，但加速度、速度都不超過上限。"""
    R = rp.prepare(stairs())
    t0, V0 = rp.time_path(R, 50, **rp.READY_TIMING)
    t1, V1 = rp.time_path(R, 50, accel=1.5, **rp.READY_TIMING)
    t2, V2 = rp.time_path(R, 50, accel=None, **rp.READY_TIMING)
    assert np.allclose(t2, t0) and np.allclose(V2, V0)               # None = 原本的
    assert t1[-1] < 0.9 * t0[-1]
    v_max, _ = rp.limits(50)
    assert np.abs(V1).max() <= v_max * 1.05
    acc = np.abs(np.diff(V1, axis=0) / np.diff(t1)[:, None]).max()
    assert acc <= 1.5 * 1.15
    assert np.allclose(V1[0], 0) and np.allclose(V1[-1], 0)
    assert rp.describe({'left': stairs(), 'right': stairs()}, 50, 1.5).endswith('（速度 50）')
