"""成果展示的姿態序列與規劃（demo_poses.py，不需要 ROS）。

    cd ~/nstc/src/sp3_task_manager && python3 -m pytest test -q
規劃的測試用 repo 裡 armpackage 的 kinematics.py、arm_config.py（純 numpy），找不到就跳過。
"""
import os
import sys

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..'))

from sp3_task_manager import demo_poses as dp  # noqa: E402
from sp3_task_manager import grasp_plan as gp  # noqa: E402

CFG = yaml.safe_load(open(os.path.join(HERE, '..', 'config', 'task_manager.yaml'), encoding='utf-8'))
ARM_SCRIPTS = os.path.join(HERE, '..', '..', 'armpackage', 'armpackage', 'scripts')
URDF = os.path.join(HERE, '..', '..', 'armpackage', 'seven_axis_arm_urdf', 'urdf', 'seven_axis_arm_urdf.urdf')
LIMITS = os.path.join(HERE, '..', '..', 'armpackage', 'seven_axis_arm_moveit_config', 'config', 'joint_limits.yaml')


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def bottle_focus(right, front, h=0.0923):
    c = gp.from_meas(gp.table_to_meas((right, front, h * 50), CFG['table'], CFG['frames']), CFG['frames'])
    return c + [0, 0, h / 2]


@pytest.mark.parametrize('arm', ['right', 'left'])
@pytest.mark.parametrize('axis', [(0, 0, -1), (1, 0, -1), (0, 1, -1), (-1, 0.3, -1), (1, 0, 0)])
@pytest.mark.parametrize('f', [0.0, 35.0, -80.0, 170.0])
def test_palm_R(arm, axis, f):
    """手掌姿態是正交矩陣、掌心朝 axis，算回來的手指角度一樣，quaternion 換回來也一樣。"""
    a = np.array(axis, float) / np.linalg.norm(axis)
    R = dp.palm_R(arm, a, f)
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-9) and np.isclose(np.linalg.det(R), 1.0)
    assert np.allclose(R[:, 0], a)
    assert abs(dp._wrap(dp.finger_angle(arm, a, R) - f)) < 1e-6
    assert np.allclose(quat_to_R(*dp.quat(R)), R, atol=1e-9)


def test_finger_convention():
    """同 moveit_bridge：從上方 0° = 手指朝前、正的 = 往外（右手往右 = base_link −Y）。"""
    R = dp.palm_R('right', (0, 0, -1), 0.0)
    assert np.allclose(R[:, 2], [1, 0, 0])
    assert np.allclose(dp.palm_R('right', (0, 0, -1), 90.0)[:, 2], [0, -1, 0])
    assert np.allclose(dp.palm_R('left', (0, 0, -1), 90.0)[:, 2], [0, 1, 0])


def test_sequence_geometry():
    """起點在瓶頂正上方；原地那段位置都不動；繞的那段都在球面上、掌心對著瓶頂中心；外側 → 前方 → 內側。"""
    focus, r = bottle_focus(18.9, 20.0), 0.15
    steps = dp.sequence('right', focus, r)
    assert steps[0].part == 'start' and np.allclose(steps[0].pos, focus + [0, 0, r])
    spin = [s for s in steps if s.part == 'spin']
    orbit = [s for s in steps if s.part == 'orbit']
    assert spin and orbit
    assert all(np.allclose(s.pos, steps[0].pos) for s in spin)
    for s in orbit:
        assert np.isclose(np.linalg.norm(s.pos - focus), r)
        assert np.allclose(s.axis, (focus - s.pos) / r)
    ys = [s.pos[1] - focus[1] for s in orbit]
    xs = [s.pos[0] - focus[0] for s in orbit]
    assert min(ys) < -0.1 and max(ys) > 0.1                 # 右手：外側 = −Y、內側 = +Y 都有到
    assert max(xs) > 0.1 and min(xs) > -1e-9                # 有繞到前方，沒有繞到靠身體那側
    assert all(s.pos[2] > focus[2] for s in orbit)          # 都在瓶頂上方


def test_sequence_steps_small():
    """手指不變時，每一步離上一步不到 10 cm、掌心朝向轉不到 28°（moveit_bridge 的微調）。"""
    steps = dp.sequence('left', bottle_focus(-18.9, 20.0), 0.15)
    for a, b in zip(steps, steps[1:]):
        assert np.linalg.norm(b.pos - a.pos) < dp.STEP_MAX_M
        assert np.degrees(np.arccos(np.clip(a.axis @ b.axis, -1, 1))) < dp.STEP_MAX_DEG


def test_sequence_parts():
    focus = bottle_focus(18.9, 20.0)
    only = dp.sequence('right', focus, 0.15, parts=('orbit',))
    assert {s.part for s in only} == {'start', 'orbit'}
    no_tilt = dp.sequence('right', focus, 0.15, parts=('spin',), spin={'tilt': 0.0})
    assert all(s.yaw is not None for s in no_tilt[1:])
    with pytest.raises(ValueError):
        dp.sequence('right', focus, 0.15, parts=('dance',))


@pytest.fixture(scope='module')
def kin():
    if not os.path.exists(os.path.join(ARM_SCRIPTS, 'kinematics.py')):
        pytest.skip('找不到 armpackage 的 kinematics.py')
    sys.path.insert(0, ARM_SCRIPTS)
    import arm_config as ac
    from kinematics import ChainIK, chain_limits
    lim = yaml.safe_load(open(LIMITS))
    over = {k: (v['min_position'], v['max_position']) for k, v in lim['joint_limits'].items()
            if v.get('has_position_limits')}
    out = {}
    for arm in ('right', 'left'):
        lo, hi = chain_limits(URDF, ac.ARMS[arm]['joints'], over)
        out[arm] = ChainIK(URDF, 'base_link', ac.ARMS[arm]['tip'], ac.TOOL_OFFSET, np.maximum(lo, -np.pi),
                           np.minimum(hi, np.pi))
    return out


def seeds(ik, n=30):
    rng = np.random.default_rng(0)
    return [np.zeros(7)] + [rng.uniform(ik.lower, ik.upper) for _ in range(n)]


def ready_q(ik, arm):
    """同 pose_demo.ready_q：設定檔 ready_pose 的關節角（IK 估的），規劃從這裡出發。"""
    _, _, pos = gp.ready_targets(CFG)
    sols = ik.solve_many(gp.from_meas(pos[arm], CFG['frames']), seeds(ik), axis=-dp.UP, iters=200)
    return min(sols, key=lambda q: np.abs(q).max())


@pytest.mark.parametrize('front', [18.0, 21.0])
@pytest.mark.parametrize('arm,right', [('right', 18.9), ('left', -18.9)])
def test_plan_recommended_position(kin, arm, right, front):
    """建議的位置（右 ±18.9、前 18～21）、從 ready 出發：整套都到得了、都是連續的微調，
    規劃出來的姿勢真的是要的掌心位置、姿態。"""
    ik = kin[arm]
    steps = dp.sequence(arm, bottle_focus(right, front), 0.15)
    pl = dp.Planner(arm, ik, seeds(ik))
    pl.plan(steps, ready_q(ik, arm))
    assert all(s.ok for s in steps)
    assert all(s.how == 'small' for s in steps[1:])
    assert pl.amp >= 10.0
    for a, b in zip(steps, steps[1:]):
        assert dp._jump(b.q, a.q) <= dp.JUMP_MAX_DEG and dp.rot_deg(a.R, b.R) <= dp.STEP_MAX_DEG
    for s in steps:
        T = ik(s.q)
        assert np.linalg.norm(T[:3, 3] - s.pos) < 1e-3 and dp.rot_deg(T[:3, :3], s.R) < 0.2
        assert np.allclose(s.R[:, 0], s.axis, atol=1e-9)
    yaws = [s.fingers - steps[0].fingers for s in steps if s.yaw is not None]
    assert max(yaws) == pytest.approx(pl.amp) and min(yaws) == pytest.approx(-pl.amp)


def test_plan_too_close_skips_yaw(kin):
    """藥瓶太靠近身體（前 12）：掌心朝下時手指轉不開，原地轉手掌整段略過，其他照規劃。"""
    ik = kin['right']
    steps = dp.sequence('right', bottle_focus(18.9, 12.0), 0.15)
    dp.Planner('right', ik, seeds(ik)).plan(steps, np.zeros(7))
    assert steps[0].ok
    assert not any(s.ok for s in steps if s.yaw is not None)
    assert sum(s.ok for s in steps) > len(steps) // 2


def test_replan_from_start(kin):
    """到了起點之後用實際的姿勢重新規劃（at_start + plan_rest）：給規劃的姿勢，結果一樣。"""
    ik = kin['right']
    steps = dp.sequence('right', bottle_focus(18.9, 21.0), 0.15)
    pl = dp.Planner('right', ik, seeds(ik))
    pl.plan(steps, ready_q(ik, 'right'))
    before = [(s.ok, s.how, s.fingers) for s in steps[1:]]
    pl.at_start(steps[0], steps[0].q)            # 手指角度改成從實際姿勢的 FK 算回來的（差不到 0.01°）
    pl.plan_rest(steps, 1, steps[0])
    after = [(s.ok, s.how, s.fingers) for s in steps[1:]]
    assert [x[:2] for x in after] == [x[:2] for x in before]
    assert np.allclose([x[2] for x in after], [x[2] for x in before], atol=0.1)


def test_plan_unreachable_start(kin):
    ik = kin['right']
    steps = dp.sequence('right', bottle_focus(18.9, 70.0), 0.15)
    with pytest.raises(dp.PlanError):
        dp.Planner('right', ik, seeds(ik, 10)).plan(steps, np.zeros(7))


@pytest.mark.parametrize('amp,n', [(10.0, 4), (25.0, 4), (40.0, 8), (60.0, 12), (0.0, 1)])
def test_fit_yaw(amp, n):
    """原地轉手掌依幅度重排：每步不超過 YAW_STEP_DEG、0 → +幅度 → −幅度 → 0；幅度 0 只留一步（標不做）。"""
    steps = dp.sequence('right', bottle_focus(18.9, 20.0), 0.15)
    others = [s for s in steps if s.yaw is None]
    pl = dp.Planner('right', None, [])
    pl.amp = amp
    pl.fit_yaw(steps)
    yaws = [s.yaw for s in steps if s.yaw is not None]
    assert len(yaws) == n
    assert [s for s in steps if s.yaw is None] == others          # 其他步沒動
    if amp > 0:
        assert max(yaws) == 1.0 and min(yaws) == -1.0 and yaws[-1] == 0.0
        d = np.abs(np.diff([0.0] + yaws)) * amp
        assert d.max() <= dp.YAW_STEP_DEG + 1e-9


# ----------------------------------------------------------------------
# 左右環繞（sides）：手離藥瓶 gap
# ----------------------------------------------------------------------
def fake_hand():
    """假的手：手掌座標裡一塊 10×20×4 cm 的板子（掌心面在 x = 0～4 cm，掌心朝 +x）。"""
    g = np.linspace(-1, 1, 9)
    return np.array([[x, 0.05 * y, 0.1 * z] for x in (0.0, 0.04) for y in g for z in g])


def bottle_obj(right=18.9, front=20.0, d=0.0829, h=0.0923):
    c = gp.from_meas(gp.table_to_meas((right, front, h * 50), CFG['table'], CFG['frames']), CFG['frames'])
    return c, gp.bottle_obstacle('t', c, (d, d, h), 0.0)


@pytest.mark.parametrize('arm', ['right', 'left'])
@pytest.mark.parametrize('axis', [(0, 0, -1), (0, 1, -1), (0, -1, -1), (1, 0, -1)])
def test_fingers_R(arm, axis):
    """手指朝前／朝上／朝下 = 那個方向投影到手掌面；掌心朝 axis。"""
    a = np.array(axis, float) / np.linalg.norm(axis)
    for kind, v in (('fwd', dp.FWD), ('up', dp.UP), ('down', -dp.UP)):
        R = dp.fingers_R(arm, a, kind)
        assert np.allclose(R.T @ R, np.eye(3), atol=1e-9) and np.allclose(R[:, 0], a)
        proj = v - (v @ a) * a
        if np.linalg.norm(proj) > 0.3:
            assert np.allclose(R[:, 2], proj / np.linalg.norm(proj))
        assert np.allclose(dp.palm_R(arm, a, dp.finger_angle(arm, a, R)), R, atol=1e-9)


def test_standoff_gap():
    from sp3_task_manager.arm_shape import distance
    c, b = bottle_obj()
    focus = c + [0, 0, 0.0923 / 2]
    hand = fake_hand()
    for tilt, az in ((0, 0), (30, 90), (45, 270)):
        d = dp.tilt_dir('right', tilt, az) if tilt else dp.UP
        R = dp.fingers_R('right', -d, 'fwd')
        for gap in (0.02, 0.04):
            r = dp.standoff(focus, d, R, hand, b, gap)
            assert distance(dp.hand_world(hand, focus + r * d, R), b)[0] == pytest.approx(gap, abs=1e-4)


@pytest.mark.parametrize('arm', ['right', 'left'])
def test_sides_sequence(arm):
    """正上方 → 外側 30、45 → 測試朝上、朝下 → 正上方 → 內側 30、45 → 測試 → 正上方；手離瓶子 gap、測試步只有一個位置。"""
    from sp3_task_manager.arm_shape import distance
    c, b = bottle_obj(18.9 if arm == 'right' else -18.9)
    focus = c + [0, 0, 0.0923 / 2]
    hand = fake_hand()
    steps = dp.sequence(arm, focus, 0.15, parts=('sides',), sides={'bottle': b, 'hand': hand, 'gap': 0.02})
    assert len(steps) == 11 and steps[0].part == 'start'
    assert [s.test for s in steps] == [False, False, False, True, True, False, False, False, True, True, False]
    out_word, in_word = ('右', '左') if arm == 'right' else ('左', '右')
    assert out_word in steps[1].label and in_word in steps[6].label and '朝上' in steps[3].label
    side = dp.SIDE[arm]
    assert (steps[2].pos[1] - focus[1]) * side > 0.02 and (steps[7].pos[1] - focus[1]) * side < -0.02
    for s in steps:
        R = dp.palm_R(arm, s.axis, s.fingers_want)
        assert distance(dp.hand_world(hand, s.pos, R), b)[0] == pytest.approx(0.02, abs=1e-4)
        assert len(s.alts) == (1 if s.test else 9)
        assert all(g2 > g1 for (g1, _), (g2, _) in zip(s.alts, s.alts[1:]))
    assert np.allclose(dp.palm_R(arm, steps[0].axis, steps[0].fingers_want)[:, 2], dp.FWD)   # 起點手指朝前
    with pytest.raises(ValueError):
        dp.sequence(arm, focus, 0.15, parts=('sides', 'spin'), sides={'bottle': b, 'hand': hand, 'gap': 0.02})


@pytest.fixture(scope='module')
def shape_right():
    from sp3_task_manager.arm_shape import ArmShape
    if not os.path.exists(URDF):
        pytest.skip('找不到 URDF')
    return ArmShape(URDF, lambda pkg: os.path.join(HERE, '..', '..', 'armpackage', pkg))


def test_plan_sides(kin, shape_right):
    """真的手部外形、整隻手臂碰撞（桌子、小桌子、加厚的藥瓶）：不是測試的步都到得了；
    往內側傾 45° 離瓶子 2 cm 會碰到桌子 → 自動離遠一點；測試步沒到得了也有姿態（照樣送）。"""
    from sp3_task_manager.arm_shape import distance
    sys.path.insert(0, ARM_SCRIPTS)
    import arm_config as ac
    ik, joints = kin['right'], ac.ARMS['right']['joints']
    c, b = bottle_obj()
    focus = c + [0, 0, 0.0923 / 2]
    objs = [gp.table_obstacle(CFG['table'], CFG['frames'])] + gp.fixture_obstacles(CFG) + \
        [gp.bottle_obstacle('t', c, (0.0829, 0.0829, 0.0923), CFG['obstacles']['margin_cm'])]

    def clearance(q):
        pts = shape_right.points(dict(zip(joints, q)), 'right')
        return min(distance(pts, o)[0] for o in objs)
    hand = dp.hand_in_palm(shape_right, 'right', joints)
    steps = dp.sequence('right', focus, 0.15, parts=('sides',), sides={'bottle': b, 'hand': hand, 'gap': 0.02})
    pl = dp.Planner('right', ik, seeds(ik), clearance=clearance)
    pl.plan(steps, ready_q(ik, 'right'))
    assert all(s.ok for s in steps if not s.test)
    assert all(s.clear_cm >= 0 for s in steps if s.ok)
    assert steps[7].extra['gap_cm'] > 2.0 and '改離' in steps[7].note       # 往左（內側）傾 45°
    assert steps[2].extra['gap_cm'] == pytest.approx(2.0)
    for s in steps:
        assert s.R is not None and np.allclose(s.R[:, 0], s.axis)
        if s.test and not s.ok:
            assert s.extra['predict'] in ('ik', 'collision') and '照樣送' in s.note


def test_plan_sides_finger_fallback(kin, shape_right):
    """實機補償開著（模擬的目標往上 0.17 × 前）、藥瓶太近（前 20）：正上方手指朝前轉不到 → 改最接近的角度、說明偏幾度，
    不是測試的步還是全部到得了（2026-11-24：--gap 4 前 26 起點到不了，整個停掉）。"""
    from sp3_task_manager.arm_shape import distance
    sys.path.insert(0, ARM_SCRIPTS)
    import arm_config as ac
    ik, joints = kin['right'], ac.ARMS['right']['joints']
    c, b = bottle_obj(18.9, 20.0)
    focus = c + [0, 0, 0.0923 / 2]
    objs = [gp.table_obstacle(CFG['table'], CFG['frames'])] + gp.fixture_obstacles(CFG) + \
        [gp.bottle_obstacle('t', c, (0.0829, 0.0829, 0.0923), CFG['obstacles']['margin_cm'])]

    def clearance(q):
        pts = shape_right.points(dict(zip(joints, q)), 'right')
        return min(distance(pts, o)[0] for o in objs)

    def sim_pos(p):
        return np.asarray(p) + dp.UP * 0.17 * max(gp.to_meas(p, CFG['frames'])[0], 0.0) / 100

    hand = dp.hand_in_palm(shape_right, 'right', joints)
    steps = dp.sequence('right', focus, 0.15, parts=('sides',), sides={'bottle': b, 'hand': hand, 'gap': 0.02})
    pl = dp.Planner('right', ik, seeds(ik), clearance=clearance, sim_pos=sim_pos)
    pl.plan(steps, ready_q(ik, 'right'))
    assert all(s.ok for s in steps if not s.test)
    assert '朝前轉不到' in steps[0].note and abs(dp._wrap(steps[0].fingers - steps[0].fingers_want)) > 1
    for s in steps:
        if s.ok:
            T = ik(s.q)
            assert np.linalg.norm(T[:3, 3] - sim_pos(s.pos)) < 1e-3 and dp.rot_deg(T[:3, :3], s.R) < 0.2


@pytest.mark.parametrize('arm', ['right', 'left'])
def test_ring_sequence(arm):
    """正上方 → 30°、45° 各繞 8 個方向（外側開始）→ 往外、往內 60° → 正上方；預設不做手指測試。"""
    from sp3_task_manager.arm_shape import distance
    c, b = bottle_obj(18.9 if arm == 'right' else -18.9, 28.0)
    focus = c + [0, 0, 0.0923 / 2]
    hand = fake_hand()
    steps = dp.sequence(arm, focus, 0.15, parts=('ring',), sides={'bottle': b, 'hand': hand, 'gap': 0.03})
    assert len(steps) == 1 + 16 + 2 + 1 and steps[0].part == 'start' and not any(s.test for s in steps)
    out_word = '右' if arm == 'right' else '左'
    assert steps[1].label == f'30° 繞圈：往{out_word}傾' and steps[9].label.startswith('45° 繞圈')
    side = dp.SIDE[arm]
    assert (steps[1].pos[1] - focus[1]) * side > 0.02          # 第一步往外側
    assert steps[3].pos[0] - focus[0] > 0.02                    # 第三步往前
    assert steps[7].pos[0] - focus[0] < -0.02                   # 往後（靠身體）
    for s in steps:
        R = dp.palm_R(arm, s.axis, s.fingers_want)
        assert distance(dp.hand_world(hand, s.pos, R), b)[0] == pytest.approx(0.03, abs=1e-4)
    t = dp.sequence(arm, focus, 0.15, parts=('ring',),
                    sides={'bottle': b, 'hand': hand, 'gap': 0.03, 'finger_test': True, 'side_tilts': ()})
    assert len(t) == 1 + 16 + 4 + 1 and sum(s.test for s in t) == 4
    with pytest.raises(ValueError):
        dp.sequence(arm, focus, 0.15, parts=('ring', 'sides'), sides={'bottle': b, 'hand': hand, 'gap': 0.03})


def test_plan_ring(kin, shape_right):
    """建議擺位（右 18.9、前 28）、實機補償開著、手離 3 cm：20 步至少 19 步到得了、手指都剛好朝前
    （2026-11-24 晚上：左右環繞只有 4 個到達點，改成繞圈）。"""
    from sp3_task_manager.arm_shape import distance
    sys.path.insert(0, ARM_SCRIPTS)
    import arm_config as ac
    ik, joints = kin['right'], ac.ARMS['right']['joints']
    c, b = bottle_obj(18.9, 28.0)
    focus = c + [0, 0, 0.0923 / 2]
    m = CFG['obstacles']['margin_cm'] / 100
    box = {'id': 'b', 'type': 'box', 'c': [float(v) for v in c], 'h': [0.0829 / 2 + m, 0.0829 / 2 + m, 0.0923 / 2 + m],
           'R': np.eye(3).tolist()}
    objs = [gp.table_obstacle(CFG['table'], CFG['frames'])] + gp.fixture_obstacles(CFG) + [box]

    def sim_pos(p):
        return np.asarray(p) + dp.UP * 0.17 * max(gp.to_meas(p, CFG['frames'])[0], 0.0) / 100

    def clearance(q):
        pts = shape_right.points(dict(zip(joints, q)), 'right')
        dz = sim_pos(ik(q)[:3, 3])[2] - ik(q)[:3, 3][2]
        return min(distance(P, o)[0] for P in (pts, pts - [0, 0, dz]) for o in objs)

    hand = dp.hand_in_palm(shape_right, 'right', joints)
    steps = dp.sequence('right', focus, 0.15, parts=('ring',), sides={'bottle': b, 'hand': hand, 'gap': 0.03})
    pl = dp.Planner('right', ik, seeds(ik), clearance=clearance, sim_pos=sim_pos)
    pl.plan(steps, ready_q(ik, 'right'))
    assert sum(s.ok for s in steps) >= 19
    assert all('轉不到' not in s.note for s in steps if s.ok)
