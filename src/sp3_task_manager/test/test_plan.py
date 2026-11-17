"""goal 解析與抓取位置計算（不需要 ROS）。

    cd ~/nstc/src/sp3_task_manager && python3 -m pytest test -q
"""
import os
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sp3_task_manager import goal, grasp_plan  # noqa: E402
from sp3_task_manager.errors import OUT_OF_RANGE, TaskError  # noqa: E402

CFG = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), '..', 'config',
                                       'task_manager.yaml'), encoding='utf-8'))


def test_parse_full():
    g = goal.parse(['task_id', 'bottle_class', 'arm', 'place_target', 'timeout_s'],
                   ['T-20261113-0001', 'b1', 'right', 'tray_1', '45'], CFG)
    assert (g.task_id, g.bottle_class, g.arm, g.place_target, g.timeout_s) == \
        ('T-20261113-0001', 'b1', 'right', 'tray_1', 45.0)


def test_parse_defaults_and_alias():
    g = goal.parse(['bottle_class', 'arm'], ['b2', '1'], CFG)
    assert g.arm == 'left' and g.place_target == '' and g.timeout_s == CFG['timeouts']['task_s']
    assert goal.TASK_ID_RE.match(g.task_id)


@pytest.mark.parametrize('keys, values', [
    (['bottle_class'], ['water_bottle']),             # 不認得的種類
    (['Grasp'], ['b1']),                              # 不認得的欄位
    (['bottle_class', 'bottle_class'], ['b1', 'b2']),  # 重複
    (['arm'], ['auto']),                              # 缺 bottle_class
    (['bottle_class', 'arm'], ['b1', 'both']),
    (['bottle_class', 'place_target'], ['b1', 'nowhere']),
    (['bottle_class', 'timeout_s'], ['b1', '-1']),
    (['bottle_class', 'task_id'], ['b1', '20261113-1']),
    (['bottle_class'], ['b1', 'extra']),
])
def test_parse_rejects(keys, values):
    with pytest.raises(TaskError) as e:
        goal.parse(keys, values, CFG)
    assert e.value.code == OUT_OF_RANGE


def test_to_meas_matches_arm_readme():
    # README：前 = x·100 + 1.35、右 = −y·100、高 = z·100 + 120.2
    p = grasp_plan.to_meas((0.4, -0.1, -0.15), CFG['frames'])
    assert np.allclose(p, [41.35, 10.0, 105.2])


def test_arm_order():
    assert grasp_plan.arm_order('auto', np.array([40, 10, 100])) == ['right', 'left']
    assert grasp_plan.arm_order('auto', np.array([40, -10, 100])) == ['left', 'right']
    assert grasp_plan.arm_order('left', np.array([40, 10, 100])) == ['left']


@pytest.mark.parametrize('approach, arm, axis', [
    ('front', 'right', (1, 0, 0)), ('top', 'left', (0, 0, -1)),
    ('side', 'right', (0, -1, 0)), ('side', 'left', (0, 1, 0)),
])
def test_plan_offsets(approach, arm, axis):
    gc = dict(CFG['grasp'], approach=approach)
    center = np.array([40.0, 0.0, 100.0])
    size = (0.06, 0.06, 0.10)                       # 半徑 3 cm、半高 5 cm
    p = grasp_plan.plan(center, size, arm, gc)
    standoff = (5.0 if approach == 'top' else 3.0) + gc['palm_gap_cm']
    axis = np.array(axis, dtype=float)
    assert np.allclose(p.grasp, center - axis * standoff)
    assert np.allclose(p.pregrasp, p.grasp - axis * gc['pregrasp_cm'])
    assert np.allclose(p.lift, p.grasp + [0, 0, gc['lift_cm']])


def test_meas_roundtrip():
    p = np.array([41.35, 10.0, 105.2])
    assert np.allclose(grasp_plan.to_meas(grasp_plan.from_meas(p, CFG['frames']), CFG['frames']), p)


def test_table_obstacle():
    # 桌緣 base_link 前 16 cm、深 60、寬 120、桌面離地 80 cm（base_link 離地 120.2）
    table = {'edge_from_base_link_cm': 16.0, 'top_cm': 80.0, 'depth_cm': 60.0, 'width_cm': 120.0,
             'center_right_cm': 0.0}
    o = grasp_plan.table_obstacle(table, CFG['frames'])
    c, h = np.array(o['c']), np.array(o['h'])
    assert np.allclose(c - h, [0.16, -0.60, -1.202])      # 靠機器人、左、地板那一角
    assert np.allclose(c + h, [0.76, 0.60, -0.402])       # 桌面 = 80 − 120.2 cm
    o = grasp_plan.table_obstacle(dict(table, margin_cm=3.0), CFG['frames'])   # 加厚 3 cm：上、前後、左右
    c, h = np.array(o['c']), np.array(o['h'])
    assert np.allclose(c - h, [0.13, -0.63, -1.202])
    assert np.allclose(c + h, [0.79, 0.63, -0.372])


def test_scene_excludes_target():
    others = [('b2', np.array([0.4, 0.1, -0.36]), np.array([0.05, 0.05, 0.08]))]
    objs = grasp_plan.scene(CFG, others)
    assert [o['id'] for o in objs] == ['sp3_table', 'sp3_bottle_b2']
    b = objs[1]
    m = CFG['obstacles']['margin_cm'] / 100
    assert b['r'] == pytest.approx(0.025 + m)
    assert b['b'][2] - b['a'][2] == pytest.approx(0.08 + 2 * m)


def test_table_to_meas():
    # 桌面原點 = base_link 前 16 cm（量測座標前 17.35）、右 0、高 = top_cm
    p = grasp_plan.table_to_meas((-67.0, 0.0, 0.0), CFG['table'], CFG['frames'])
    assert np.allclose(p, [17.35, -67.0, CFG['table']['top_cm']])


def test_meas_table_roundtrip():
    p = np.array([-30.0, 12.0, 10.0])
    m = grasp_plan.table_to_meas(p, CFG['table'], CFG['frames'])
    assert np.allclose(grasp_plan.meas_to_table(m, CFG['table'], CFG['frames']), p)


def test_reach_front_interp():
    ws = CFG['workspace']
    assert grasp_plan.reach_front(18.9, ws) == pytest.approx(24.0)
    assert grasp_plan.reach_front(5.0, ws) == pytest.approx(19.0)      # 0 → 16、10 → 22 中間
    assert grasp_plan.reach_front(-1.0, ws) is None                    # 跨到另一側
    assert grasp_plan.reach_front(51.0, ws) is None


def test_reachable_arms():
    t = lambda right, front: grasp_plan.table_to_meas((right, front, 10.0), CFG['table'], CFG['frames'])
    assert grasp_plan.reachable_arms(t(0, 10), CFG) == ['left', 'right']     # 中線、近：兩隻都可以
    assert grasp_plan.reachable_arms(t(-40, 5), CFG) == ['left']             # 左邊：只有左手
    assert grasp_plan.reachable_arms(t(40, 5), CFG) == ['right']
    assert grasp_plan.reachable_arms(t(18.9, 24), CFG) == ['right']          # 肩膀前方的極限
    assert grasp_plan.reachable_arms(t(18.9, 25), CFG) == []
    assert grasp_plan.reachable_arms(t(0, 17), CFG) == []                    # 中線最遠 16
    assert grasp_plan.reachable_arms(t(30, 1), CFG) == []                    # 太靠身體


def test_dry_run_parse():
    from sp3_task_manager import dry_run
    assert dry_run.parse('true') == {'vision', 'arm', 'hand'}
    assert dry_run.parse('False') == set()
    assert dry_run.parse(' vision , hand') == {'vision', 'hand'}
    assert dry_run.parse(['arm']) == {'arm'}
    assert dry_run.parse([]) == set()
    assert dry_run.parse(None) == set()
    with pytest.raises(ValueError):
        dry_run.parse('vision,camera')


def test_auto_arm_only_when_poses_match():
    center, size = np.array([40.0, 0.0, 85.0]), np.array([0.05, 0.05, 0.08])
    for approach, same in (('front', True), ('top', True), ('side', False)):
        gc = dict(CFG['grasp'], approach=approach)
        r, l = (grasp_plan.plan(center, size, a, gc) for a in ('right', 'left'))
        assert grasp_plan.same_poses(r, l) == same


def test_held_obstacle():
    o = grasp_plan.held_obstacle([0.4, -0.1, -0.35], [0.05, 0.05, 0.08])
    assert o['id'] == 'sp3_held' and o['r'] == pytest.approx(0.025)
    assert o['b'][2] - o['a'][2] == pytest.approx(0.08)


def test_dry_bottle_reachable():
    center = grasp_plan.to_meas(CFG['dry_run']['bottle_base_link_m'], CFG['frames'])
    assert 'right' in grasp_plan.reachable_arms(center, CFG)


def test_clearance_distance():
    from sp3_task_manager.arm_shape import distance
    box = {'type': 'box', 'c': [0, 0, 0], 'R': np.eye(3).tolist(), 'h': [0.1, 0.1, 0.05]}
    assert distance(np.array([[0, 0, 0.1]]), box)[0] == pytest.approx(0.05)
    assert distance(np.array([[0, 0, 0.0]]), box)[0] == pytest.approx(-0.05)     # 在裡面：負的
    stick = {'type': 'stick', 'a': [0, 0, -0.05], 'b': [0, 0, 0.05], 'r': 0.03}
    assert distance(np.array([[0.1, 0, 0]]), stick)[0] == pytest.approx(0.07)    # 側面
    assert distance(np.array([[0, 0, 0.08]]), stick)[0] == pytest.approx(0.03)   # 頂上
    assert distance(np.array([[0.01, 0, 0]]), stick)[0] == pytest.approx(-0.02)


def test_scene_missing():
    objs = grasp_plan.scene(CFG, [('b2', np.array([0.3, 0.1, -0.35]), np.array([0.05, 0.05, 0.08]))])
    want = grasp_plan.moveit_ids('sp3', objs)
    assert want == {'html_sp3_table', 'html_sp3_bottle_b2'}
    assert grasp_plan.scene_missing('sp3', objs, want | {'html_web_3'}) == (set(), set())     # 網頁的不管
    assert grasp_plan.scene_missing('sp3', objs, {'html_sp3_table'}) == ({'html_sp3_bottle_b2'}, set())
    assert grasp_plan.scene_missing('sp3', [], {'html_sp3_bottle_b1'}) == (set(), {'html_sp3_bottle_b1'})
    assert grasp_plan.moveit_ids('sp3_test', [{'id': 'sp3_test_table'}]) == {'html_sp3_test_table'}
