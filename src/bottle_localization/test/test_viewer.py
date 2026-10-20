#coding=utf-8
"""bottle_viewer 的統計與排版測試，不需要 ROS。"""
import math
import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from bottle_localization.bottle_viewer import ViewerState, detections_to_dict  # noqa: E402

TOPIC = '/bottle_detections_3d'


def test_waiting_before_any_message():
    assert '等待' in ViewerState(3.0).render(0.0, TOPIC)


def test_rate_ratio_and_jitter():
    s = ViewerState(window=3.0)
    #20 Hz 共 2 秒；b1 每一筆都有、x 在 ±1 mm 之間跳，b2 只有一半的訊息有
    for k in range(41):
        t = k * 0.05
        dets = {'b1': (0.400 + (0.001 if k % 2 else -0.001), -0.050, 0.046, 0.9)}
        if k % 2 == 0:
            dets['b2'] = (0.350, 0.100, 0.039, 0.8)
        s.add(t, 'base_link', dets)

    assert s.rate() == pytest.approx(20.0)
    ratio, sx, sy = s.class_stats('b1')
    assert ratio == pytest.approx(1.0)
    assert sx == pytest.approx(0.001, rel=0.05)
    assert sy == pytest.approx(0.0)
    ratio2, _, _ = s.class_stats('b2')
    assert ratio2 == pytest.approx(21 / 41)

    text = s.render(2.0, TOPIC)
    assert 'base_link' in text and '20.0 Hz' in text
    assert 'b1' in text and '+0.399' in text and '100%' in text     #最後一筆 k=40 是 0.399
    assert 'b2' in text and '51%' in text


def test_old_messages_leave_the_window():
    s = ViewerState(window=1.0)
    s.add(0.0, 'base_link', {'b1': (0.4, 0.0, 0.05, 0.9)})
    s.add(0.5, 'base_link', {})
    text = s.render(5.0, TOPIC)
    assert '沒有收到訊息' in text
    assert '超過 1 秒沒看到' in text           #b1 還列著，標示是舊值
    assert s.class_stats('b1') == (0.0, None, None)


def test_detections_to_dict():
    def det(label, x, y, z, score):
        pos = SimpleNamespace(x=x, y=y, z=z)
        hyp = SimpleNamespace(hypothesis=SimpleNamespace(class_id=label, score=score),
                              pose=SimpleNamespace(pose=SimpleNamespace(position=pos)))
        return SimpleNamespace(id=label, results=[hyp],
                               bbox=SimpleNamespace(center=SimpleNamespace(position=pos)))
    msg = SimpleNamespace(detections=[det('b1', 0.4, -0.05, 0.046, 0.91),
                                      det('b3', 0.38, 0.08, 0.045, 0.88)])
    assert detections_to_dict(msg) == {'b1': (0.4, -0.05, 0.046, 0.91),
                                       'b3': (0.38, 0.08, 0.045, 0.88)}
    #沒有 results 時用 bbox 中心，信心值是 NaN
    bare = SimpleNamespace(id='b2', results=[],
                           bbox=SimpleNamespace(center=SimpleNamespace(
                               position=SimpleNamespace(x=0.1, y=0.2, z=0.3))))
    x, y, z, score = detections_to_dict(SimpleNamespace(detections=[bare]))['b2']
    assert (x, y, z) == (0.1, 0.2, 0.3) and math.isnan(score)
