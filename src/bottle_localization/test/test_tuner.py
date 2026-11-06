#coding=utf-8
"""bottle_tuner 的狀態、單位換算與設定檔改寫，以及設定檔本身的一致性。不需要 ROS。"""
import ast
import math
import os
import shutil
import sys

import numpy as np
import pytest
import yaml

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, ROOT)
from bottle_localization import geometry as g  # noqa: E402
from bottle_localization.bottle_tuner import (FIELDS, TunerState, clip, find_key, format_number,  # noqa: E402
                                              read_file_values, save_yaml, update_yaml_text)

CONFIG = os.path.join(ROOT, 'config')
PATHS = {'params': os.path.join(CONFIG, 'params.yaml'),
         'extrinsics': os.path.join(CONFIG, 'camera_extrinsics.yaml')}
FILE_VALUES = {'table_height_cm': 74.5, 'x': 0.087, 'y': 0.0916, 'z': 0.1507,
               'roll': 0.012, 'pitch': 0.5397, 'yaw': 0.0}


def field(name):
    return next(f for f in FIELDS if f.name == name)


# ----------------------------- geometry -----------------------------
def test_table_z_matches_old_values():
    """原本手算的 z_table：正式桌 80.5 cm -> -0.1203，測試桌 74.5 cm -> -0.1803（肩關節點 94 cm）。"""
    assert g.table_z(80.5, 94.0, 0.01475) == pytest.approx(-0.12025)
    assert g.table_z(74.5, 94.0, 0.01475) == pytest.approx(-0.18025)


def test_inverse_extrinsics_composes_to_identity():
    ext = (0.087, 0.0916, 0.1507, 0.012, 0.5397, 0.03)
    trans, quat = g.inverse_extrinsics(*ext)
    rot = g.rpy_to_matrix(*ext[3:])
    inv_rot = g.quaternion_to_matrix(*quat)
    p = np.array([0.3, -0.1, 0.05])
    back = inv_rot @ (rot @ p + np.array(ext[:3])) + trans
    assert np.allclose(back, p)


# ----------------------------- TunerState -----------------------------
def test_adjust_uses_display_units():
    st = TunerState(FILE_VALUES)
    st.live.update(FILE_VALUES)
    st.selected = [f.name for f in FIELDS].index('x')
    assert st.adjusted(+1) == pytest.approx(0.087 + 0.001)          #預設步距 1 mm
    st.change_step(+1)
    assert st.adjusted(-1) == pytest.approx(0.087 - 0.005)
    st.change_step(+5)                                               #超過範圍就停在最大
    assert st.step() == 5.0
    st.selected = [f.name for f in FIELDS].index('yaw')
    assert st.adjusted(+1) == pytest.approx(math.radians(0.1))


def test_adjust_needs_live_value():
    st = TunerState(FILE_VALUES)
    assert st.adjusted(+1) is None


def test_parse_and_dirty():
    st = TunerState(FILE_VALUES)
    st.live.update(FILE_VALUES)
    assert st.parse('80.5') == pytest.approx(80.5)                  #桌高（選中第一列）
    with pytest.raises(ValueError):
        st.parse('-3')
    with pytest.raises(ValueError):
        st.parse('abc')
    assert st.dirty() == []
    st.live['x'] = 0.08700004                                        #比存檔精度（0.1 mm）小的差不算修改
    assert st.dirty() == []
    st.live['x'] = 0.0875
    assert [f.name for f in st.dirty()] == ['x']


def test_move_wraps():
    st = TunerState(FILE_VALUES)
    st.move(-1)
    assert st.field.name == FIELDS[-1].name


def test_format_number():
    assert format_number(74.5, 2) == '74.5'
    assert format_number(0.087, 4) == '0.087'
    assert format_number(0.0, 5) == '0.0'
    assert format_number(-0.000001, 4) == '0.0'
    assert format_number(80.0, 2) == '80.0'
    assert isinstance(yaml.safe_load(f'v: {format_number(80.0, 2)}')['v'], float)


def test_clip_counts_wide_chars():
    assert clip('桌高abc', 5) == '桌高a'
    assert clip('abc', 10) == 'abc'


# ----------------------------- 設定檔改寫 -----------------------------
def test_update_yaml_text_keeps_comments():
    text = ('# 註解 x: 1\nx: 0.0870   # 往前\ny: 0.0916\nbottle:\n  ros__parameters:\n'
            '    table_height_cm: 80.5\n')
    out = update_yaml_text(text, {'x': '0.088', 'table_height_cm': '74.5'})
    assert out == ('# 註解 x: 1\nx: 0.088   # 往前\ny: 0.0916\nbottle:\n  ros__parameters:\n'
                   '    table_height_cm: 74.5\n')


def test_update_yaml_text_rejects_missing_or_duplicate():
    with pytest.raises(ValueError):
        update_yaml_text('y: 1.0\n', {'x': '2.0'})
    with pytest.raises(ValueError):
        update_yaml_text('x: 1.0\nx: 2.0\n', {'x': '3.0'})


def test_find_key():
    assert find_key({'a': {'b': {'c': 3}}}, 'c') == 3
    with pytest.raises(ValueError):
        find_key({'a': {'c': 1}, 'b': {'c': 2}}, 'c')


@pytest.mark.parametrize('kind', ['params', 'extrinsics'])
def test_real_config_roundtrip(tmp_path, kind):
    """實際的設定檔：每個欄位剛好一行、改完只動那幾行、讀回來的值正確、留了 .bak。"""
    src = PATHS[kind]
    dst = tmp_path / os.path.basename(src)
    shutil.copy(src, dst)
    fields = [f for f in FIELDS if f.file == kind]
    updates = {f.name: format_number(0.1234 if f.name != 'table_height_cm' else 77.7, f.save_decimals)
               for f in fields}
    save_yaml(str(dst), updates)
    old = open(src, encoding='utf-8').read().splitlines()
    new = dst.read_text(encoding='utf-8').splitlines()
    changed = [i for i, (a, b) in enumerate(zip(old, new)) if a != b]
    assert len(old) == len(new) and len(changed) == len(fields)
    values = read_file_values({kind: str(dst)})
    for f in fields:
        assert values[f.name] == pytest.approx(float(updates[f.name]))
    assert (tmp_path / (os.path.basename(src) + '.bak')).read_text(encoding='utf-8') == '\n'.join(old) + '\n'


def test_save_through_symlink_keeps_link(tmp_path):
    """--symlink-install 時 share 裡是 symlink：要寫到真正的檔案，連結本身留著。"""
    real = tmp_path / 'real.yaml'
    real.write_text('x: 1.0\n', encoding='utf-8')
    link = tmp_path / 'link.yaml'
    link.symlink_to(real)
    save_yaml(str(link), {'x': '2.0'})
    assert link.is_symlink()
    assert real.read_text(encoding='utf-8') == 'x: 2.0\n'


# ----------------------------- 設定檔一致性 -----------------------------
def _param_spec_names():
    """從 bottle_localizer.py 讀出 PARAM_SPEC 的參數名稱（不 import，避免需要 ROS）。"""
    src = open(os.path.join(ROOT, 'bottle_localization', 'bottle_localizer.py'), encoding='utf-8').read()
    tree = ast.parse(src)
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'PARAM_SPEC':
            return [elt.elts[0].value for elt in node.value.elts]
    raise AssertionError('找不到 PARAM_SPEC')


def test_params_yaml_has_every_param():
    """params.yaml + launch 帶進來的參數，剛好涵蓋節點需要的全部參數。"""
    from_launch = {'bottle_dims_file', 'target_frame', 'reference_height_cm', 'reference_z'}
    with open(PATHS['params'], encoding='utf-8') as f:
        params = yaml.safe_load(f)['bottle_localizer']['ros__parameters']
    needed = set(_param_spec_names())
    assert needed - from_launch - set(params) == set()
    assert set(params) - needed == set(), '設定檔裡有節點不認得的參數'
    assert not set(params) & from_launch, '這些參數由 launch 帶進來，不該寫在 params.yaml'


def test_reference_yaml():
    with open(os.path.join(CONFIG, 'robot_reference.yaml'), encoding='utf-8') as f:
        ref = yaml.safe_load(f)
    assert len(ref['reference_point']) == 3
    assert ref['reference_height_cm'] > 0
    assert isinstance(ref['base_frame'], str)
