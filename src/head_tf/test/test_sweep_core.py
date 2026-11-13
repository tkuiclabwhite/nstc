import math

import numpy as np
import pytest

from head_tf import sweep_core as core
from sim_head import HeadSim


def test_build_postures_ref_first_last_and_serpentine():
    p = core.build_postures([2162, 1934, 2048, 2048], [2300, 2186], (2048, 2300))
    assert p[0] == ('ref', 2048, 2300) and p[-1] == ('ref', 2048, 2300)
    grid = p[1:-1]
    assert [g[0] for g in grid] == ['grid'] * 6
    # tilt 由小到大；第一列 pan 由小到大，第二列反過來
    assert [(g[1], g[2]) for g in grid] == [(1934, 2186), (2048, 2186), (2162, 2186),
                                            (2162, 2300), (2048, 2300), (1934, 2300)]


def test_build_postures_needs_values():
    with pytest.raises(ValueError):
        core.build_postures([], [2300])


def test_check_postures_lists_bad_values():
    core.check_postures(core.build_postures([1934, 2162], [2186, 2414]))
    with pytest.raises(ValueError, match='pan 1500.*tilt 2900|tilt 2900.*pan 1500'):
        core.check_postures([('grid', 1500, 2300), ('grid', 2048, 2900)])


def test_check_speed_rejects_zero():
    core.check_speed(40)
    for bad in (0, -5, 500):
        with pytest.raises(ValueError):
            core.check_speed(bad)


def test_is_still():
    moving = [(t, 2048 + 10 * t, 2300) for t in np.arange(0, 1, 0.02)]
    assert not core.is_still(moving, 0.5, 2, 1.0)
    still = [(t, 2048 + (1 if int(t * 50) % 2 else 0), 2300) for t in np.arange(0, 1, 0.02)]
    assert core.is_still(still, 0.5, 2, 1.0)
    # 資料不夠涵蓋整個視窗
    assert not core.is_still(still[-5:], 0.5, 2, 1.0)


def test_joint_stats_empty_and_values():
    s = core.joint_stats([])
    assert s['joint_n'] == 0 and math.isnan(s['pan_mean'])
    s = core.joint_stats([(0, 2040, 2300), (1, 2050, 2302)])
    assert s['pan_mean'] == 2045 and s['tilt_max'] == 2302 and s['joint_n'] == 2


def test_depth_to_array_float_with_padding():
    h, w, pad = 3, 4, 2
    d = np.arange(h * (w + pad), dtype=np.float32).reshape(h, w + pad) / 10
    d[0, 0] = 0
    d[1, 1] = np.nan
    out = core.depth_to_array('32FC1', h, w, (w + pad) * 4, d.tobytes())
    assert out.shape == (h, w) and out.dtype == np.float32
    assert np.isnan(out[0, 0]) and np.isnan(out[1, 1])
    assert out[2, 3] == pytest.approx(d[2, 3])


def test_depth_to_array_mm():
    d = np.array([[0, 500], [1000, 65535]], dtype=np.uint16)
    out = core.depth_to_array('16UC1', 2, 2, 4, d.tobytes())
    assert np.isnan(out[0, 0]) and out[0, 1] == pytest.approx(0.5) and out[1, 0] == pytest.approx(1.0)
    with pytest.raises(ValueError):
        core.depth_to_array('8UC1', 2, 2, 2, b'\0' * 4)


def test_color_to_bgr():
    bgra = np.zeros((2, 3, 4), dtype=np.uint8)
    bgra[..., 0], bgra[..., 1], bgra[..., 2], bgra[..., 3] = 10, 20, 30, 255
    out = core.color_to_bgr('bgra8', 2, 3, 12, bgra.tobytes())
    assert out.shape == (2, 3, 3) and tuple(out[0, 0]) == (10, 20, 30)
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    rgb[..., 0], rgb[..., 2] = 200, 7
    out = core.color_to_bgr('rgb8', 2, 3, 9, rgb.tobytes())
    assert tuple(out[1, 2]) == (7, 0, 200)
    assert core.color_to_bgr('yuv422', 2, 3, 6, b'\0' * 12) is None


def test_median_depth_ignores_nan():
    a = np.full((2, 2), np.nan, dtype=np.float32)
    b = np.array([[1, 2], [3, np.nan]], dtype=np.float32)
    c = np.array([[3, 2], [5, np.nan]], dtype=np.float32)
    m = core.median_depth([a, b, c])
    assert m[0, 0] == 2 and m[1, 0] == 4 and np.isnan(m[1, 1])


def test_surface_point_flat_patch():
    k = (100.0, 100.0, 50.0, 40.0)
    depth = np.full((80, 100), 0.8, dtype=np.float32)
    depth[30:50, 40:60] = 0.5
    p, z_med, n = core.surface_point(depth, (40, 30, 60, 50), k)
    assert z_med == pytest.approx(0.5) and n == 11 * 11
    # 框中心 (50, 40) 正好是主點，所以 x、y 接近 0
    assert p == pytest.approx([0.0, 0.0, 0.5], abs=1e-6)
    assert core.surface_point(np.full((80, 100), np.nan, np.float32), (40, 30, 60, 50), k) is None
    assert core.surface_point(depth, (40, 30, 40, 50), k) is None


@pytest.mark.parametrize('pan,tilt', [(2048, 2300), (1934, 2186), (2162, 2414)])
def test_plane_fit_recovers_sim_camera(pan, tilt):
    sim = HeadSim()
    depth = sim.render(pan, tilt)
    boxes = [b for b in (sim.bbox(l, pan, tilt) for l in sim.bottles) if b is not None]
    pts = core.plane_points(depth, sim.k, exclude=boxes)
    n, c, rms, n_in = core.fit_plane(pts)
    roll, pitch = core.camera_roll_pitch(n)
    er, ep, eh = sim.expected_table(pan, tilt)
    assert math.degrees(roll) == pytest.approx(er, abs=0.05)
    assert math.degrees(pitch) == pytest.approx(ep, abs=0.05)
    assert -(n @ c) == pytest.approx(eh, abs=5e-4)
    assert n_in == pts.shape[0]


def test_plane_points_excludes_boxes():
    k = (100.0, 100.0, 50.0, 40.0)
    depth = np.full((80, 100), 0.6, dtype=np.float32)
    all_pts = core.plane_points(depth, k, rows=(0, 1), cols=(0, 1), margin=0)
    cut = core.plane_points(depth, k, rows=(0, 1), cols=(0, 1), exclude=[(10, 10, 19, 19)], margin=0)
    assert all_pts.shape[0] - cut.shape[0] == 100


def test_quat_roll_pitch():
    r, p = 0.1, 0.5
    # q = qy(p) * qx(r)
    qx = (math.sin(r / 2), 0, 0, math.cos(r / 2))
    qy = (0, math.sin(p / 2), 0, math.cos(p / 2))
    x1, y1, z1, w1 = qy
    x2, y2, z2, w2 = qx
    q = (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
         w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
         w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
         w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)
    roll, pitch = core.quat_roll_pitch(*q)
    assert roll == pytest.approx(r) and pitch == pytest.approx(p)


def test_group_detections():
    size = (960, 600)
    dets = [
        (0.0, {'image_size': [960, 600], 'bottles': [
            {'label': 'b1', 'confidence': 0.8, 'bbox': [10, 10, 50, 90]},
            {'label': 'b1', 'confidence': 0.9, 'bbox': [12, 10, 52, 90]},
            {'label': 'b2', 'confidence': 0.7, 'bbox': [100, 10, 140, 90]}]}),
        (0.1, {'image_size': [960, 600], 'bottles': [
            {'label': 'b1', 'confidence': 0.7, 'bbox': [14, 12, 54, 92]},
            {'label': 'bad'}]}),
        (0.2, {'image_size': [1280, 720], 'bottles': [
            {'label': 'b1', 'confidence': 0.9, 'bbox': [0, 0, 1, 1]}]}),
    ]
    groups, n_msgs, n_mismatch = core.group_detections(dets, size)
    assert n_msgs == 3 and n_mismatch == 1
    assert groups['b1'][0] == 2 and groups['b1'][1] == pytest.approx(0.8)
    assert groups['b1'][2] == (13, 11, 53, 91)
    assert groups['b2'][0] == 1


def test_analyze_capture_without_depth():
    snap = core.Snapshot(depth_frames=[], k=(1, 1, 0, 0), size=(4, 4))
    row, bottles, depth = core.analyze_capture(snap)
    assert depth is None and bottles == [] and row['bottles'] == 0
    assert math.isnan(row['table_pitch_deg']) and math.isnan(row['imu_pitch_deg'])
    assert set(core.POSTURE_FIELDS) - set(row) == {'index', 'kind', 'pan_cmd', 'tilt_cmd',
                                                   'reached', 'still', 'move_s'}


def test_direction_check_without_reference():
    d = core.direction_check([{'index': 0, 'kind': 'grid', 'pan_mean': 2048, 'tilt_mean': 2300}], [])
    assert d['tilt_slope'] is None and d['pan_slope'] is None
    assert core.repeatability([], []) is None
