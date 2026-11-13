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


def test_workspace():
    assert grasp_plan.in_workspace(np.array([40, 0, 100]), CFG['workspace'])
    assert not grasp_plan.in_workspace(np.array([90, 0, 100]), CFG['workspace'])
