import json
import os

import numpy as np
import pytest
import yaml

from sp3_task_manager import grasp_plan, try_cmd

CFG = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), '..', 'config', 'task_manager.yaml'), encoding='utf-8'))


def p(d):
    return try_cmd.parse(json.dumps(d), CFG)


def test_move_frames_agree():
    t = p({'cmd': 'move', 'frame': 'table', 'pos': [-18.9, 12, 10]})
    b = p({'cmd': 'move', 'frame': 'base_link', 'pos': list(grasp_plan.from_meas(t['pos_cm'], CFG['frames']))})
    assert np.allclose(t['pos_cm'], b['pos_cm'])
    assert t['arm'] == 'auto' and t['approach'] == 'top'


def test_bad_commands():
    for d in [{'cmd': 'fly'}, {'cmd': 'move', 'pos': [1, 2]}, {'cmd': 'move', 'pos': [1, 2, 3], 'frame': 'map'},
              {'cmd': 'bottle', 'class': 'b9'}, {'cmd': 'ready', 'arm': 'auto'}, {'cmd': 'move', 'pos': [1, 2, 3], 'arm': 'both'}]:
        with pytest.raises(ValueError):
            p(d)
    with pytest.raises(ValueError):
        try_cmd.parse('not json', CFG)


def test_above_bottle_and_frames():
    xyz = grasp_plan.from_meas(grasp_plan.table_to_meas((-18.9, 12, 4.6), CFG['table'], CFG['frames']), CFG['frames'])
    pos = try_cmd.above_bottle(xyz, [0.0829, 0.0829, 0.092], 10.0, CFG['frames'])
    f = try_cmd.both_frames(pos, CFG)
    assert f['table'] == pytest.approx([-18.9, 12.0, 4.6 + 4.6 + 10.0], abs=0.05)
    assert f['base_link'][0] == pytest.approx((CFG['table']['edge_from_base_link_cm'] + 12) / 100, abs=1e-3)


def test_record_ready():
    assert p({'cmd': 'record_ready', 'id': 'w1'}) == {'cmd': 'record_ready', 'id': 'w1'}
