"""網頁測試區（檔名不叫 test_*：pytest 會當成測試檔）的指令（/manip/test_cmd，std_msgs/String JSON）解析，不跑完整取放流程。不依賴 ROS，方便單元測試。

    {"cmd": "stand"}                                  回站姿（ready_pose.via = path：錄好的路線倒過來走）
    {"cmd": "ready", "arm": "both"}                   回預備姿勢（設定檔 ready_pose：錄好的路線 / 一隻一隻移 / 手臂端 /arm_ready）
    {"cmd": "record_ready"}                           錄站姿 → ready 的路線（手要在站姿；用 /arm_move 一隻一隻舉上去，存起來）
    {"cmd": "move", "frame": "table", "pos": [右, 前, 上], "arm": "auto", "approach": "top"}
                                                      掌心移到指定點：frame = table（桌面座標 cm）或 base_link（m）
    {"cmd": "bottle", "class": "b1", "above": 10, "arm": "auto"}
                                                      掌心移到影像看到的那一瓶瓶頂正上方 above cm
    {"cmd": "cancel"}                                 停止執行中的測試
id（可省略）原樣放進結果 /manip/test_result，網頁用來對是哪一筆。
"""
import json

import numpy as np

from sp3_task_manager import grasp_plan as gp

ARMS = ('auto', 'left', 'right')
READY_ARMS = ('both', 'left', 'right')


def parse(text, cfg):
    """JSON 字串 → 指令 dict（move 會多一個 pos_cm = 量測座標）。格式不對丟 ValueError（訊息給人看）。"""
    try:
        d = json.loads(text)
    except ValueError:
        raise ValueError(f'不是 JSON：{text!r}')
    if not isinstance(d, dict):
        raise ValueError('指令要是 JSON 物件')
    cmd = d.get('cmd')
    out = {'cmd': cmd, 'id': d.get('id', '')}
    if cmd in ('stand', 'cancel', 'record_ready'):
        return out
    if cmd == 'ready':
        arm = d.get('arm', 'both')
        if arm not in READY_ARMS:
            raise ValueError(f'ready 的 arm 只接受 {"/".join(READY_ARMS)}，收到 {arm!r}')
        return dict(out, arm=arm)
    arm = d.get('arm', 'auto')
    if arm not in ARMS:
        raise ValueError(f'arm 只接受 {"/".join(ARMS)}，收到 {arm!r}')
    if cmd == 'move':
        frame = d.get('frame', 'table')
        try:
            pos = [float(v) for v in d['pos']]
        except (KeyError, TypeError, ValueError):
            raise ValueError('move 要給 pos：三個數字')
        if len(pos) != 3:
            raise ValueError('move 的 pos 要三個數字')
        if frame == 'table':
            pos_cm = gp.table_to_meas(pos, cfg['table'], cfg['frames'])
        elif frame == 'base_link':
            pos_cm = gp.to_meas(pos, cfg['frames'])
        else:
            raise ValueError(f'frame 只接受 table / base_link，收到 {frame!r}')
        approach = d.get('approach', 'top')
        if approach not in gp.AXES:
            raise ValueError(f'approach 只接受 {"/".join(gp.AXES)}，收到 {approach!r}')
        return dict(out, arm=arm, approach=approach, pos_cm=pos_cm)
    if cmd == 'bottle':
        cls = d.get('class')
        if cls not in cfg['bottle_classes']:
            raise ValueError(f'class 只接受 {"/".join(cfg["bottle_classes"])}，收到 {cls!r}')
        try:
            above = float(d.get('above', 10.0))
        except (TypeError, ValueError):
            raise ValueError('above 要是數字（cm）')
        return dict(out, arm=arm, cls=cls, above=above)
    raise ValueError(f'不認得的指令 {cmd!r}（stand / ready / record_ready / move / bottle / cancel）')


def above_bottle(xyz_m, size_m, above_cm, frames):
    """藥瓶中心（base_link m）、尺寸 → 掌心在瓶頂正上方 above_cm 的位置（量測座標 cm）。"""
    return gp.to_meas(xyz_m, frames) + np.array([0.0, 0.0, float(size_m[2]) * 50.0 + above_cm])


def both_frames(pos_cm, cfg):
    """量測座標（cm）→ {'base_link': [x, y, z] m, 'table': [右, 前, 上] cm}，給網頁顯示。"""
    b = gp.from_meas(pos_cm, cfg['frames'])
    t = gp.meas_to_table(pos_cm, cfg['table'], cfg['frames'])
    return {'base_link': [round(float(v), 4) for v in b], 'table': [round(float(v), 1) for v in t]}
