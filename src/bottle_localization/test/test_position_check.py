#coding=utf-8
"""tools/position_check.py、tools/pair_check.py 的換算與擬合測試，不需要 ROS。"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
from position_check import apply_rigid_2d, edge_to_shoulder, fit_rigid_2d  # noqa: E402
from pair_check import pair_result  # noqa: E402


def edge_readings(forward, left, a_right, a_left, span):
    """反過來算：瓶子圓心（相對右肩，cm）-> 從 P0、P0_L 到圓心的距離。"""
    s = math.hypot(span, a_right - a_left)
    y_hat = np.array([span, a_right - a_left]) / s          #桌邊座標 (u, v) 下的左方
    x_hat = np.array([a_left - a_right, span]) / s          #桌邊座標下的前方
    b = np.array([0.0, -a_right]) + forward * x_hat + left * y_hat
    return float(np.linalg.norm(b)), float(np.linalg.norm(b - np.array([span, 0.0])))


@pytest.mark.parametrize('a_left', [14.0, 14.6, 13.1])     #擺正、左肩遠 6 mm、左肩近 9 mm
@pytest.mark.parametrize('forward, left', [(36.0, -5.0), (41.0, 10.0), (46.0, 25.0), (30.0, 0.0)])
def test_edge_roundtrip(forward, left, a_left):
    r0, r1 = edge_readings(forward, left, 14.0, a_left, 27.5)
    got = edge_to_shoulder(r0, r1, 14.0, a_left, 27.5)
    assert got == pytest.approx((forward, left), abs=1e-9)


def test_edge_squared_table_is_plain_offset():
    """桌子擺正時：往前 = a + 離桌邊的距離，往左 = 沿桌邊離 P0 的距離。"""
    r0, r1 = math.hypot(-5.0, 22.0), math.hypot(-5.0 - 27.5, 22.0)
    assert edge_to_shoulder(r0, r1, 14.0, 14.0, 27.5) == pytest.approx((36.0, -5.0))


def test_edge_inconsistent_readings():
    assert edge_to_shoulder(10.0, 10.0, 14.0, 14.0, 27.5) is None     #兩圓不相交


def test_fit_recovers_rotation_and_offset():
    rng = np.random.default_rng(0)
    truth = rng.uniform([0.30, -0.10], [0.46, 0.22], size=(8, 2))
    delta, d = math.radians(1.5), np.array([0.006, -0.004])
    out = apply_rigid_2d(-delta, np.zeros(2), truth - d)              #輸出 = 真值經過反向的修正
    got_delta, got_d = fit_rigid_2d(out, truth)
    assert np.allclose(apply_rigid_2d(got_delta, got_d, out), truth, atol=1e-12)
    assert got_delta == pytest.approx(delta)



@pytest.mark.parametrize('b, angle', [((0.50, 0.05), 0.0),       #前後擺
                                      ((0.10, 0.05), 0.0),       #前後擺，順序相反
                                      ((0.30, 0.25), 90.0),      #左右擺
                                      ((0.30, -0.15), 90.0),     #左右擺，順序相反（-90 換成 90）
                                      ((0.44, 0.19), 45.0)])
def test_pair_result_distance_and_direction(b, angle):
    a = (0.30, 0.05)
    dist, err, ang = pair_result(a, b, 0.20)
    assert dist == pytest.approx(math.hypot(b[0] - a[0], b[1] - a[1]))
    assert err == pytest.approx(dist - 0.20)
    assert ang == pytest.approx(angle)
