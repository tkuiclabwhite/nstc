#!/usr/bin/env python3
#coding=utf-8
"""bottle_tuner：執行中調整桌高與相機外參的終端機介面（TUI）。

另開一個終端機（SSH 也可以）執行，bottle_localization.launch.py 要先在跑：

    ros2 run bottle_localization bottle_tuner

按鍵：
    ↑ ↓      選參數
    ← →      減少 / 增加一個步距
    [ ]      換小 / 大一級的步距
    Enter    直接輸入數值（量到的桌高用這個）
    s        存回設定檔（保留註解，舊檔留一份 .bak）
    r        放棄修改，還原成設定檔裡的值
    q        離開（有沒存的修改會先問）

調整立刻生效，節點不用重新 launch；但要按 s 才會寫回設定檔，下次 launch 才會沿用。
設定檔預設是 launch 用的那幾個（套件 share 目錄的 config/，--symlink-install 時就是 src 裡的檔案）。
launch 時用 params_file:= / extrinsics_file:= 換過檔案的話，這裡也要用 --params-file / --extrinsics-file
指定同一個。
"""
import argparse
import math
import os
import re
import shutil
import sys
import threading
import time
from collections import namedtuple

import yaml

from bottle_localization.bottle_viewer import _ljust, _width, detections_to_dict

#name: 參數名稱，也是設定檔裡的 key；node: 'localizer' 或 'tf'；file: 'params' 或 'extrinsics'
#scale: 參數值 x scale = 顯示值；decimals: 顯示的小數位數；save_decimals: 寫回設定檔的小數位數（參數單位）
#steps: ←→ 的步距（顯示單位），[ ] 切換
Field = namedtuple('Field', 'name label node file unit scale decimals save_decimals steps')
FIELDS = (
    Field('table_height_cm', '桌高', 'localizer', 'params', 'cm', 1.0, 1, 2, (0.1, 0.5, 1.0)),
    Field('x', '外參 x（往前）', 'tf', 'extrinsics', 'mm', 1000.0, 1, 4, (0.5, 1.0, 5.0)),
    Field('y', '外參 y（往左）', 'tf', 'extrinsics', 'mm', 1000.0, 1, 4, (0.5, 1.0, 5.0)),
    Field('z', '外參 z（往上）', 'tf', 'extrinsics', 'mm', 1000.0, 1, 4, (0.5, 1.0, 5.0)),
    Field('roll', '外參 roll（歪頭）', 'tf', 'extrinsics', '度', 180.0 / math.pi, 2, 5, (0.05, 0.1, 0.5)),
    Field('pitch', '外參 pitch（點頭）', 'tf', 'extrinsics', '度', 180.0 / math.pi, 2, 5, (0.05, 0.1, 0.5)),
    Field('yaw', '外參 yaw（轉頭）', 'tf', 'extrinsics', '度', 180.0 / math.pi, 2, 5, (0.05, 0.1, 0.5)),
)
DEFAULT_STEP_INDEX = 1
REFRESH_SEC = 1.0         #多久向節點讀一次目前的值（別人用 ros2 param set 改了也看得到）
STALE_SEC = 2.0           #瓶子超過這麼久沒出現就標示「沒看到」


def format_number(value, decimals):
    """寫回設定檔用：固定小數位數再去掉多餘的 0，至少留一位小數（YAML 才會讀成浮點數）。"""
    text = f'{value:.{decimals}f}'.rstrip('0')
    if text.endswith('.'):
        text += '0'
    return '0.0' if float(text) == 0.0 else text


def find_key(data, key):
    """在巢狀的 dict 裡找 key，回它的值；不是剛好一個時報錯。"""
    found = []

    def walk(node):
        if isinstance(node, dict):
            for k, v in node.items():
                if k == key:
                    found.append(v)
                walk(v)
    walk(data)
    if len(found) != 1:
        raise ValueError(f'設定檔裡 {key} 有 {len(found)} 個，應該剛好 1 個')
    return found[0]


def update_yaml_text(text, updates):
    """把 `key: 值` 那一行的值換掉，註解與排版原樣保留。每個 key 必須剛好一行。"""
    for key, value in updates.items():
        pattern = re.compile(rf'^(\s*{re.escape(key)}\s*:\s*)([^\s#]+)(.*)$', re.M)
        count = len(pattern.findall(text))
        if count != 1:
            raise ValueError(f'設定檔裡「{key}:」有 {count} 行，應該剛好 1 行')
        text = pattern.sub(lambda m, v=value: m.group(1) + v + m.group(3), text)
    return text


def save_yaml(path, updates):
    """改寫設定檔並留一份 .bak。回 (實際寫入的檔案, 備份檔)。

    --symlink-install 時 share 裡的設定檔是連到 src 的 symlink，寫到 realpath 才不會把連結換成一般檔案。
    """
    real = os.path.realpath(path)
    with open(real, 'r', encoding='utf-8') as f:
        old = f.read()
    new = update_yaml_text(old, updates)
    yaml.safe_load(new)                 #改完還要是合法的 YAML
    backup = real + '.bak'
    shutil.copy2(real, backup)
    tmp = real + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(new)
    os.replace(tmp, real)
    return real, backup


def read_file_values(paths):
    """從設定檔讀出每個欄位目前的值（參數單位）。paths: {'params': 路徑, 'extrinsics': 路徑}。"""
    values = {}
    for kind, path in paths.items():
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        for field in FIELDS:
            if field.file == kind:
                values[field.name] = float(find_key(data, field.name))
    return values


def clip(text, width):
    """依顯示寬度（中文算 2）截斷，curses 寫超過視窗寬度會出錯。"""
    out, used = [], 0
    for ch in text:
        w = _width(ch)
        if used + w > width:
            break
        out.append(ch)
        used += w
    return ''.join(out)


class TunerState:
    """TUI 的狀態與顯示，不依賴 ROS 和 curses，可以單獨測試。值一律用參數單位（cm / m / rad）存。"""

    def __init__(self, file_values):
        self.file = dict(file_values)
        self.live = {f.name: None for f in FIELDS}
        self.selected = 0
        self.step_index = {f.name: DEFAULT_STEP_INDEX for f in FIELDS}

    @property
    def field(self):
        return FIELDS[self.selected]

    def move(self, delta):
        self.selected = (self.selected + delta) % len(FIELDS)

    def step(self, field=None):
        field = field or self.field
        return field.steps[self.step_index[field.name]]

    def change_step(self, delta):
        f = self.field
        self.step_index[f.name] = min(max(self.step_index[f.name] + delta, 0), len(f.steps) - 1)

    def adjusted(self, direction):
        """選中的欄位往 direction（+1 / -1）調一個步距後的參數值；還沒讀到目前的值時回 None。"""
        f = self.field
        if self.live[f.name] is None:
            return None
        return self.live[f.name] + direction * self.step() / f.scale

    def parse(self, text):
        """使用者輸入的數字（顯示單位）-> 參數值。不合理時丟 ValueError。"""
        f = self.field
        value = float(text)
        if not math.isfinite(value):
            raise ValueError('不是有效的數字')
        if f.name == 'table_height_cm' and value <= 0:
            raise ValueError('桌高必須大於 0')
        return value / f.scale

    def is_dirty(self, field):
        live = self.live[field.name]
        return live is not None and abs(live - self.file[field.name]) >= 0.5 * 10.0 ** -field.save_decimals

    def dirty(self):
        return [f for f in FIELDS if self.is_dirty(f)]

    @staticmethod
    def show(field, value):
        if value is None:
            return '-'
        return f'{value * field.scale:.{field.decimals}f} {field.unit}'

    def table_lines(self):
        """參數表，每列 (文字, 是否選中)。"""
        cols = (24, 14, 3, 14, 10)
        head = (_ljust('   參數', cols[0]) + _ljust('目前生效', cols[1]) + ' ' * cols[2] +
                _ljust('設定檔', cols[3]) + '步距')
        lines = [(head, False)]
        for i, f in enumerate(FIELDS):
            text = (_ljust((' > ' if i == self.selected else '   ') + f.label, cols[0]) +
                    _ljust(self.show(f, self.live[f.name]), cols[1]) +
                    _ljust(' * ' if self.is_dirty(f) else '', cols[2]) +
                    _ljust(self.show(f, self.file[f.name]), cols[3]) +
                    f'{self.step(f):g} {f.unit}')
            lines.append((text, i == self.selected))
        return lines


# =============================================================================
# ROS + curses（main 才 import，上面的部分測試時不需要 ROS）
# =============================================================================
class RosLink:
    """參數服務的讀寫與即時座標訂閱。回呼在 executor 的執行緒，改狀態一律拿 lock。"""

    def __init__(self, node, state, lock, node_names, topic):
        from rcl_interfaces.srv import GetParameters, SetParameters
        from vision_msgs.msg import Detection3DArray
        self._GetParameters, self._SetParameters = GetParameters, SetParameters
        self.node, self.state, self.lock = node, state, lock
        self.node_names = node_names
        self.clients = {}
        for key, name in node_names.items():
            self.clients[key] = (node.create_client(GetParameters, f'{name}/get_parameters'),
                                 node.create_client(SetParameters, f'{name}/set_parameters'))
        self.detections = {}            #類別 -> (x, y, z, 收到的時間)
        self.frame = ''
        self.message = ''
        self._pending_get = {}          #節點 -> 送出讀取請求的時間
        self._set_time = {}             #欄位 -> 最後一次送出修改的時間
        node.create_subscription(Detection3DArray, topic, self.on_detections, 10)

    def connected(self, key):
        get_client, set_client = self.clients[key]
        return get_client.service_is_ready() and set_client.service_is_ready()

    def on_detections(self, msg):
        now = time.monotonic()
        with self.lock:
            self.frame = msg.header.frame_id
            for label, (x, y, z, _) in detections_to_dict(msg).items():
                self.detections[label] = (x, y, z, now)

    def refresh(self):
        """向兩個節點讀目前的值（非同步，回來時更新 state.live）。"""
        for key in self.clients:
            sent = self._pending_get.get(key)
            if (sent is not None and time.monotonic() - sent < 3 * REFRESH_SEC) or not self.connected(key):
                continue        #上一個請求還沒回來（太久沒回來就重送）
            names = [f.name for f in FIELDS if f.node == key]
            sent = time.monotonic()
            self._pending_get[key] = sent
            future = self.clients[key][0].call_async(self._GetParameters.Request(names=names))
            future.add_done_callback(lambda fut, k=key, n=names, t=sent: self._on_get(k, n, t, fut))

    def _on_get(self, key, names, sent, future):
        self._pending_get.pop(key, None)
        try:
            values = future.result().values
        except Exception as e:
            with self.lock:
                self.message = f'讀取 {self.node_names[key]} 的參數失敗：{e}'
            return
        with self.lock:
            for name, v in zip(names, values):
                if self._set_time.get(name, -math.inf) >= sent:
                    continue    #讀取送出後又改過，這筆是舊值，等下一次讀取
                #ParameterValue：type 2 = 整數、3 = 浮點數
                if v.type == 3:
                    self.state.live[name] = v.double_value
                elif v.type == 2:
                    self.state.live[name] = float(v.integer_value)

    def set_value(self, field, value):
        """送出新值；先樂觀地更新畫面，失敗時顯示原因並重新讀取。"""
        from rclpy.parameter import Parameter
        if not self.connected(field.node):
            with self.lock:
                self.message = f'{self.node_names[field.node]} 沒有連上，launch 有在跑嗎？'
            return
        with self.lock:
            self.state.live[field.name] = value
            self._set_time[field.name] = time.monotonic()
        request = self._SetParameters.Request(
            parameters=[Parameter(field.name, Parameter.Type.DOUBLE, float(value)).to_parameter_msg()])
        future = self.clients[field.node][1].call_async(request)
        future.add_done_callback(lambda fut, f=field: self._on_set(f, fut))

    def _on_set(self, field, future):
        try:
            result = future.result().results[0]
            ok, reason = result.successful, result.reason
        except Exception as e:
            ok, reason = False, str(e)
        with self.lock:
            self.message = (f'{field.label} = {TunerState.show(field, self.state.live[field.name])}' if ok
                            else f'{field.label} 修改失敗：{reason}')
        if not ok:
            self.refresh()


def _draw(stdscr, state, link, paths, lock):
    import curses
    height, width = stdscr.getmaxyx()
    lines = [('bottle_tuner   ↑↓ 選擇  ←→ 調整  [ ] 步距  Enter 輸入數值  s 存檔  r 還原  q 離開', False)]
    with lock:
        status = '　'.join(f'{name} {"已連線" if link.connected(key) else "未連線（launch 有在跑嗎？）"}'
                          for key, name in link.node_names.items())
        lines.append(('節點：' + status, False))
        lines.append(('', False))
        lines.extend(state.table_lines())
        lines.append(('   * = 已經生效但還沒存檔，按 s 寫回設定檔', False))
        lines.append(('', False))
        lines.append((f'即時座標（{link.frame or "-"}，m）', False))
        now = time.monotonic()
        if not link.detections:
            lines.append(('   還沒收到任何瓶子', False))
        for label in sorted(link.detections):
            x, y, z, t = link.detections[label]
            age = now - t
            note = f'{age:5.1f} 秒前' + ('（沒看到）' if age > STALE_SEC else '')
            lines.append((f'   {_ljust(label, 6)} x {x:+.3f}   y {y:+.3f}   z {z:+.3f}   {note}', False))
        lines.append(('', False))
        lines.append(('設定檔：' + '、'.join(os.path.realpath(p) for p in paths.values()), False))
        lines.append(('訊息：' + link.message, False))
    stdscr.erase()
    for row, (text, selected) in enumerate(lines[:height]):
        try:
            stdscr.addstr(row, 0, clip(text, width - 1), curses.A_REVERSE if selected else curses.A_NORMAL)
        except curses.error:
            pass
    stdscr.refresh()


def _ask(stdscr, prompt, lock, link, echo):
    """在最後一行問問題。echo=True 時讀一行文字，否則讀一個按鍵。"""
    import curses
    height, width = stdscr.getmaxyx()
    stdscr.timeout(-1)
    try:
        stdscr.move(height - 1, 0)
        stdscr.clrtoeol()
        stdscr.addstr(height - 1, 0, clip(prompt, width - 1))
        if echo:
            curses.echo()
            curses.curs_set(1)
            raw = stdscr.getstr(height - 1, min(_width(prompt), width - 2), 20)
            return raw.decode('utf-8', 'ignore').strip()
        return stdscr.getkey()
    except curses.error:
        return ''
    finally:
        curses.noecho()
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        stdscr.timeout(100)


def _save(state, link, paths, lock):
    with lock:
        dirty = state.dirty()
        values = {f.name: state.live[f.name] for f in dirty}
    if not dirty:
        with lock:
            link.message = '沒有需要存的修改'
        return True
    saved = []
    for kind, path in paths.items():
        fields = [f for f in dirty if f.file == kind]
        if not fields:
            continue
        updates = {f.name: format_number(values[f.name], f.save_decimals) for f in fields}
        try:
            real, backup = save_yaml(path, updates)
        except (OSError, ValueError, yaml.YAMLError) as e:
            with lock:
                link.message = f'存檔失敗（{path}）：{e}'
            return False
        with lock:
            for f in fields:
                state.file[f.name] = float(updates[f.name])
        saved.append(f'{os.path.basename(real)}（舊檔 {os.path.basename(backup)}）')
    with lock:
        link.message = '已存檔：' + '、'.join(saved)
    return True


def _revert(state, link, lock):
    with lock:
        targets = [(f, state.file[f.name]) for f in state.dirty()]
    for f, value in targets:
        link.set_value(f, value)
    with lock:
        link.message = '已還原成設定檔的值' if targets else '沒有需要還原的修改'


def _run(stdscr, state, link, paths, lock):
    import curses
    curses.curs_set(0)
    stdscr.keypad(True)
    stdscr.timeout(100)
    last_refresh = 0.0
    while True:
        now = time.monotonic()
        if now - last_refresh >= REFRESH_SEC:
            link.refresh()
            last_refresh = now
        _draw(stdscr, state, link, paths, lock)
        key = stdscr.getch()
        if key == -1:
            continue
        if key in (curses.KEY_UP, ord('k')):
            with lock:
                state.move(-1)
        elif key in (curses.KEY_DOWN, ord('j')):
            with lock:
                state.move(1)
        elif key in (curses.KEY_LEFT, curses.KEY_RIGHT, ord('h'), ord('l')):
            direction = 1 if key in (curses.KEY_RIGHT, ord('l')) else -1
            with lock:
                field, value = state.field, state.adjusted(direction)
            if value is None:
                with lock:
                    link.message = f'還沒讀到 {field.label} 目前的值（{link.node_names[field.node]} 有在跑嗎？）'
            else:
                link.set_value(field, value)
        elif key in (ord('['), ord(']')):
            with lock:
                state.change_step(1 if key == ord(']') else -1)
        elif key in (curses.KEY_ENTER, 10, 13):
            with lock:
                field = state.field
            text = _ask(stdscr, f'輸入新的 {field.label}（{field.unit}），Enter 確定、空白取消：', lock, link, True)
            if text:
                try:
                    with lock:
                        value = state.parse(text)
                    link.set_value(field, value)
                except ValueError as e:
                    with lock:
                        link.message = f'輸入「{text}」不對：{e}'
        elif key == ord('s'):
            _save(state, link, paths, lock)
        elif key == ord('r'):
            _revert(state, link, lock)
        elif key == ord('q'):
            with lock:
                n = len(state.dirty())
            if not n:
                return
            ans = _ask(stdscr, f'有 {n} 個修改還沒存檔：s 存檔後離開，r 還原後離開，'
                               f'y 直接離開（節點繼續用目前的值，下次 launch 回到設定檔），其他鍵取消 ',
                       lock, link, False)
            if ans == 's' and _save(state, link, paths, lock):
                return
            if ans == 'r':
                _revert(state, link, lock)
                time.sleep(0.5)          #讓還原的請求送出去
                return
            if ans == 'y':
                return


def main(args=None):
    import curses
    import locale

    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.utilities import remove_ros_args

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--params-file', help='bottle_localizer 的參數檔（存桌高），預設是 launch 用的那個')
    ap.add_argument('--extrinsics-file', help='相機外參檔，預設是 launch 用的那個')
    ap.add_argument('--localizer-node', default='/bottle_localizer', help='定位節點的名稱')
    ap.add_argument('--tf-node', default='/bottle_localization_camera_tf', help='外參 TF 節點的名稱')
    ap.add_argument('--topic', default='/bottle_detections_3d', help='即時座標的 topic')
    opts = ap.parse_args(remove_ros_args(args if args is not None else sys.argv)[1:])

    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print('bottle_tuner 要在終端機裡直接執行（需要鍵盤輸入），不能放進 launch 或導向檔案')
        return 1
    if opts.params_file is None or opts.extrinsics_file is None:
        try:
            config = os.path.join(get_package_share_directory('bottle_localization'), 'config')
        except Exception as e:
            print(f'找不到 bottle_localization 的設定檔目錄（{e}），請用 --params-file / --extrinsics-file 指定')
            return 1
        opts.params_file = opts.params_file or os.path.join(config, 'params.yaml')
        opts.extrinsics_file = opts.extrinsics_file or os.path.join(config, 'camera_extrinsics.yaml')
    paths = {'params': opts.params_file, 'extrinsics': opts.extrinsics_file}
    try:
        state = TunerState(read_file_values(paths))
    except (OSError, ValueError, yaml.YAMLError) as e:
        print(f'讀不到設定檔：{e}')
        return 1

    rclpy.init(args=args)
    node = rclpy.create_node('bottle_tuner')
    lock = threading.Lock()
    link = RosLink(node, state, lock, {'localizer': opts.localizer_node, 'tf': opts.tf_node}, opts.topic)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    locale.setlocale(locale.LC_ALL, '')         #curses 才畫得出中文
    try:
        curses.wrapper(_run, state, link, paths, lock)
    except KeyboardInterrupt:
        pass
    finally:
        #先停 executor 並等執行緒結束，再銷毀節點；直接結束會在 C++ 那層 abort
        executor.shutdown(timeout_sec=1.0)
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    with lock:
        left = state.dirty()
    if left:
        print('沒存檔的修改（節點仍在使用，下次 launch 會回到設定檔的值）：' +
              '、'.join(f'{f.label} {TunerState.show(f, state.live[f.name])}' for f in left))
    return 0


if __name__ == '__main__':
    sys.exit(main())
