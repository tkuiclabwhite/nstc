#!/usr/bin/env python3
"""頭部掃描資料收集工具：把頭轉到一組刻度，每個姿勢記錄馬達刻度、桌面、IMU、瓶子。

新平台（三軸脖子）：轉頭（28 號）和低頭（29 號），歪頭（30 號）不轉、只記錄。
基準刻度和範圍讀 config/neck_head.yaml（--config），預設姿勢 = 允許範圍的網格。收到的資料用
tools/check_neck_model.py 驗證頭部模型。在機器人上執行：

    ros2 run head_tf head_sweep neck_01                  # 預設網格：轉頭 3 個 x 低頭 5 個
    ros2 run head_tf head_sweep neck_02 --pan 1962 2047 2133 --tilt 1651 1708 1765
    ros2 run head_tf head_sweep neck_03 --manual         # 自己轉頭，按 Enter 記錄

開始前：
- nstc.launch.py（ZED 和馬達 driver）和 bottle_detect_v2 要開著。
  任務管理（sp3_task_manager）不要開、網頁的頭部拉條不要動，它們會搶頭部。
- 頭部扭力開著、歪頭在基準刻度；桌上放 2~4 個藥罐，整個過程不要動機器人、桌子和瓶子。
- 畫面下方中間那塊（桌面擬合區）盡量只有桌面。

資料存在 ~/head_sweeps/<名稱>/，格式見 README.md。工具結束或按 Ctrl+C 時，頭會回到開始時的刻度。
"""
import argparse
import json
import shutil
import signal
import socket
import sys
import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import String

from head_tf import neck_model
from head_tf import sweep_core as core
from head_tf import sweep_runner as runner

# 舊機器人（兩軸頭）用的是 pan 1934 2048 2162、tilt 2186~2414、基準 2048 2300（sweep_01）。
# 新平台的預設從 neck_head.yaml 算（neck_model.sweep_plan）。
# bottle_localization 的設定檔一起複製進資料夾，下一階段要用當時的外參當起點
ROBOT_CONFIG_FILES = ['camera_extrinsics.yaml', 'robot_reference.yaml', 'params.yaml', 'bottle_dims.yaml']


class SweepNode(Node):
    """訂閱需要的 topic、發頭部指令，並提供 sweep_runner 要的 io 介面。

    callback 在背景執行緒的 executor 裡跑，主執行緒跑收集流程，緩衝區用鎖保護。
    時間一律用收到訊息時本機的 monotonic 時間，不用 header.stamp（機器人時鐘可能不準）。
    """

    def __init__(self, opts, cfg):
        super().__init__('head_sweep')
        self.opts = opts
        self._lock = threading.Lock()
        self._joints = deque(maxlen=5000)
        self._roll = deque(maxlen=5000)
        self._imu = deque(maxlen=8000)
        self._dets = deque(maxlen=500)
        # 深度圖很大（960x600 float32 約 2.3 MB），只每隔一段時間留一張，記錄期間平均分布
        self._depth = deque(maxlen=cfg.max_depth_frames + 5)
        self._depth_interval = cfg.capture / cfg.max_depth_frames
        self._color = None
        self._camera = None
        self._head_pub = None
        self._head_cls = None
        self.depth_encoding = None

        # BEST_EFFORT 的訂閱可以接 RELIABLE 和 BEST_EFFORT 兩種 publisher
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(JointState, opts.joint_topic, self._on_joint, qos)
        self.create_subscription(Image, opts.depth_topic, self._on_depth, qos)
        self.create_subscription(CameraInfo, opts.info_topic, self._on_info, qos)
        self.create_subscription(Image, opts.color_topic, self._on_color, qos)
        self.create_subscription(Imu, opts.imu_topic, self._on_imu, qos)
        self.create_subscription(String, opts.detection_topic, self._on_det, qos)

    # ---------------- callback ----------------
    def _on_joint(self, msg):
        try:
            pan = msg.position[msg.name.index(core.PAN_JOINT)]
            tilt = msg.position[msg.name.index(core.TILT_JOINT)]
        except (ValueError, IndexError):
            return
        try:
            roll = float(msg.position[msg.name.index(core.ROLL_JOINT)])
        except (ValueError, IndexError):
            roll = None
        now = time.monotonic()
        with self._lock:
            self._joints.append((now, float(pan), float(tilt)))
            if roll is not None:
                self._roll.append((now, roll))

    def _on_depth(self, msg):
        now = time.monotonic()
        with self._lock:
            self.depth_encoding = msg.encoding
            if self._depth and now - self._depth[-1][0] < self._depth_interval:
                return
            self._depth.append((now, msg))

    def _on_info(self, msg):
        with self._lock:
            self._camera = ((float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5])),
                            (int(msg.width), int(msg.height)), msg.header.frame_id,
                            [float(v) for v in msg.k], [float(v) for v in msg.p])

    def _on_color(self, msg):
        with self._lock:
            self._color = (time.monotonic(), msg)

    def _on_imu(self, msg):
        q, a, w = msg.orientation, msg.linear_acceleration, msg.angular_velocity
        roll, pitch = core.quat_roll_pitch(q.x, q.y, q.z, q.w)
        with self._lock:
            self._imu.append((time.monotonic(), roll, pitch, a.x, a.y, a.z, w.x, w.y, w.z))

    def _on_det(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(data, dict):
            with self._lock:
                self._dets.append((time.monotonic(), data))

    # ---------------- 狀態查詢 ----------------
    def camera(self):
        with self._lock:
            return self._camera

    def have_depth(self):
        with self._lock:
            return bool(self._depth)

    def have_color(self):
        with self._lock:
            return self._color is not None

    def imu_count_since(self, t):
        with self._lock:
            return sum(1 for s in self._imu if s[0] >= t)

    def latest_roll(self):
        with self._lock:
            return self._roll[-1][1] if self._roll else None

    def latest_labels(self, seconds=1.0):
        now = time.monotonic()
        with self._lock:
            dets = [d for t, d in self._dets if t >= now - seconds]
        labels = set()
        for d in dets:
            for b in d.get('bottles', []):
                if isinstance(b, dict) and 'label' in b:
                    labels.add(str(b['label']))
        return sorted(labels), len(dets)

    def other_head_publishers(self):
        own = self.get_name()
        return [f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                for i in self.get_publishers_info_by_topic(self.opts.head_topic)
                if i.node_name != own]

    # ---------------- 頭部指令 ----------------
    def make_head_publisher(self):
        try:
            from nstc_msgs.msg import HeadPackage
        except ImportError:     # 舊機器人（ros2_adult）
            from tku_msgs.msg import HeadPackage
        self._head_cls = HeadPackage
        self._head_pub = self.create_publisher(HeadPackage, self.opts.head_topic, 10)

    def send_head(self, pan, tilt, speed):
        core.check_speed(speed)
        for hid, pos in ((core.HEAD_PAN_ID, pan), (core.HEAD_TILT_ID, tilt)):
            m = self._head_cls()
            m.id, m.position, m.speed = int(hid), int(pos), int(speed)
            self._head_pub.publish(m)

    # ---------------- sweep_runner 的 io 介面 ----------------
    def now(self):
        return time.monotonic()

    def sleep(self, seconds):
        time.sleep(max(0.0, seconds))

    def latest_joint(self):
        with self._lock:
            return self._joints[-1] if self._joints else None

    def joints_since(self, t):
        with self._lock:
            return [s for s in self._joints if s[0] >= t]

    def snapshot(self, t0, t1, max_frames):
        with self._lock:
            depth_msgs = [m for t, m in self._depth if t0 <= t <= t1][-max_frames:]
            joints = [s for s in self._joints if t0 <= s[0] <= t1]
            roll = [s for s in self._roll if t0 <= s[0] <= t1]
            imu = [s for s in self._imu if t0 <= s[0] <= t1]
            dets = [d for d in self._dets if t0 <= d[0] <= t1]
            color = self._color
            camera = self._camera
        frames = [core.depth_to_array(m.encoding, m.height, m.width, m.step, m.data)
                  for m in depth_msgs]
        bgr = None
        if color is not None and t0 - 1.0 <= color[0]:
            m = color[1]
            bgr = core.color_to_bgr(m.encoding, m.height, m.width, m.step, m.data)
        if frames:
            size = (frames[0].shape[1], frames[0].shape[0])
        else:
            size = camera[1]
        return core.Snapshot(depth_frames=frames, k=camera[0], size=size, frame_id=camera[2],
                             color=bgr, joints=joints, roll=roll, imu=imu, detections=dets)


def default_config():
    """neck_head.yaml：先找安裝的 share，找不到用原始碼旁邊的。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('head_tf')) / 'config' / 'neck_head.yaml'
        if path.exists():
            return str(path)
    except Exception:
        pass
    return str(Path(__file__).resolve().parents[1] / 'config' / 'neck_head.yaml')


def parse_args(argv):
    ap = argparse.ArgumentParser(
        prog='head_sweep', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('name', help='這次收集的名稱，資料存在 <--out>/<name>/')
    ap.add_argument('--out', default='~/head_sweeps', help='資料的上層資料夾（預設 ~/head_sweeps）')
    ap.add_argument('--config', default=None,
                    help='頭部設定（基準刻度、允許範圍），預設是 head_tf 的 config/neck_head.yaml')
    ap.add_argument('--pan', type=int, nargs='+', default=None,
                    help='轉頭（28 號）要量的刻度，預設是允許範圍兩端和基準')
    ap.add_argument('--tilt', type=int, nargs='+', default=None,
                    help='低頭（29 號）要量的刻度，預設在允許範圍內平均取 5 個')
    ap.add_argument('--reference', type=int, nargs=2, default=None, metavar=('PAN', 'TILT'),
                    help='基準姿勢（頭尾各量一次），預設是 neck_head.yaml 的 reference_ticks（外參校正時的刻度）')
    ap.add_argument('--manual', action='store_true', help='自己轉頭，按 Enter 記錄；工具不會發頭部指令')
    ap.add_argument('--speed', type=int, default=runner.SweepConfig.speed, help='頭部轉速（1~200，預設 40）')
    ap.add_argument('--settle', type=float, default=runner.SweepConfig.settle, help='停穩後再等幾秒才記錄')
    ap.add_argument('--capture', type=float, default=runner.SweepConfig.capture, help='每個姿勢記錄幾秒')
    ap.add_argument('--frames', type=int, default=runner.SweepConfig.max_depth_frames,
                    help='每個姿勢最多用幾張深度圖')
    ap.add_argument('--note', default='', help='備註，存進 meta.json（例如瓶子擺在哪、桌高）')
    ap.add_argument('--yes', action='store_true', help='不等 Enter 直接開始')
    ap.add_argument('--force', action='store_true', help='有其他程式也在發 /Head_Topic 時照樣執行')
    ap.add_argument('--wait', type=float, default=10.0, help='等 topic 的秒數')
    ap.add_argument('--joint-topic', default='/joint_states')
    ap.add_argument('--head-topic', default='/Head_Topic')
    ap.add_argument('--depth-topic', default='/zed/zed_node/depth/depth_registered')
    ap.add_argument('--info-topic', default='/zed/zed_node/left/camera_info')
    ap.add_argument('--color-topic', default='/zed/zed_node/left/image_rect_color')
    ap.add_argument('--imu-topic', default='/zed/zed_node/imu/data')
    ap.add_argument('--detection-topic', default='/med_detect/bottles')
    return ap.parse_args(argv)


def wait_for(cond, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.1)
    return cond()


def wait_inputs(node, opts):
    """必要的 topic（刻度、內參、深度）沒到就停；IMU、YOLO、彩色影像沒有只警告。"""
    wait_for(lambda: node.latest_joint() and node.camera() and node.have_depth(), opts.wait)
    missing = []
    if node.latest_joint() is None:
        missing.append(f'{opts.joint_topic}（28、29 號馬達）：nstc.launch.py 的馬達 driver 有開嗎？')
    if node.camera() is None:
        missing.append(f'{opts.info_topic}：ZED 有在跑嗎？')
    if not node.have_depth():
        missing.append(f'{opts.depth_topic}：ZED 有在跑嗎？')
    if missing:
        print(f'{opts.wait:g} 秒內沒收到：')
        for m in missing:
            print(f'  - {m}')
        return False
    t = time.monotonic()
    wait_for(lambda: node.imu_count_since(t) > 0 and node.latest_labels()[1] > 0 and node.have_color(), 3.0)
    if node.imu_count_since(t) == 0:
        print(f'⚠ 沒收到 {opts.imu_topic}，只記錄桌面和瓶子')
    if node.latest_labels()[1] == 0:
        print(f'⚠ 沒收到 {opts.detection_topic}（bottle_detect_v2 沒開？），只記錄桌面和 IMU；'
              '要用瓶子確認 pan 方向的話先開 YOLO')
    if not node.have_color():
        print(f'⚠ 沒收到 {opts.color_topic}，不存彩色影像')
    return True


def copy_robot_config(out_dir, neck_config):
    """把機器人當下的 bottle_localization 設定檔和 neck_head.yaml 複製一份。用 bottle_tuner 改過的值不會在檔案裡。"""
    dst = Path(out_dir) / 'robot_config'
    dst.mkdir(exist_ok=True)
    copied = []
    if neck_config and Path(neck_config).exists():
        shutil.copy2(neck_config, dst / 'neck_head.yaml')
        copied.append('neck_head.yaml')
    try:
        from ament_index_python.packages import get_package_share_directory
        src = Path(get_package_share_directory('bottle_localization')) / 'config'
    except Exception as e:  # 套件沒裝也照樣收集
        return f'找不到 bottle_localization 的設定檔（{e}），只複製了 {", ".join(copied) or "無"}'
    for name in ROBOT_CONFIG_FILES:
        if (src / name).exists():
            shutil.copy2(src / name, dst / name)
            copied.append(name)
    return f'已複製 {", ".join(copied)} 到 robot_config/' if copied else f'{src} 裡沒有設定檔'


def build_meta(opts, cfg, node, postures, start, plan=None):
    k, size, frame_id, k_full, p_full = node.camera()
    return {
        'name': opts.name,
        'note': opts.note,
        'mode': 'manual' if opts.manual else 'grid',
        'started_at': time.strftime('%Y-%m-%d %H:%M:%S'),
        'started_at_note': '機器人時鐘可能沒校時，日期僅供參考',
        'host': socket.gethostname(),
        'reference_ticks': list(opts.reference),
        'start_ticks': [start[1], start[2]],
        'start_roll': node.latest_roll(),
        'neck_config': opts.config,
        'limits': {k: list(plan[k]) for k in ('pan_limits', 'tilt_limits', 'roll_limits')} if plan else None,
        'postures': [list(p) for p in postures] if postures else None,
        'deg_per_tick': core.DEG_PER_TICK,
        'joints': {'pan': core.PAN_JOINT, 'tilt': core.TILT_JOINT, 'roll': core.ROLL_JOINT},
        'camera': {'k': list(k), 'size': list(size), 'frame_id': frame_id,
                   'K': k_full, 'P': p_full, 'depth_encoding': node.depth_encoding},
        'config': runner.config_dict(cfg),
        'topics': {name: getattr(opts, name) for name in (
            'joint_topic', 'head_topic', 'depth_topic', 'info_topic', 'color_topic',
            'imu_topic', 'detection_topic')},
        'status': 'running',
    }


def return_head(node, ticks, cfg):
    """把頭轉回開始時的刻度，最多等 5 秒。"""
    print('把頭轉回開始時的刻度（最多 5 秒，請稍等）…')
    try:
        node.send_head(int(round(ticks[0])), int(round(ticks[1])), cfg.speed)
        ok = wait_for(lambda: node.latest_joint() is not None
                      and abs(node.latest_joint()[1] - ticks[0]) <= cfg.reach_tol
                      and abs(node.latest_joint()[2] - ticks[1]) <= cfg.reach_tol, 5.0)
        print(f'頭{"已" if ok else "沒有在 5 秒內"}回到開始時的刻度 pan {ticks[0]:.0f} tilt {ticks[1]:.0f}')
    except Exception as e:
        print(f'⚠ 頭沒辦法轉回開始的刻度：{e}')


def print_plan(opts, cfg, node, postures, start):
    roll = node.latest_roll()
    print(f'目前頭部刻度：轉頭（28）{start[1]:.0f}、低頭（29）{start[2]:.0f}、'
          f'歪頭（30）{"沒收到" if roll is None else f"{roll:.0f}"}')
    if postures:
        per = 1.0 + cfg.still_window + cfg.settle + cfg.capture + 1.0
        print(f'要量 {len(postures)} 個姿勢（頭尾各一次基準 {opts.reference[0]}/{opts.reference[1]}），'
              f'預估 {len(postures) * per / 60:.1f} 分鐘')
        print(f'  轉頭（28）：{" ".join(str(p) for p in sorted(set(opts.pan)))}'
              f'（{core.ticks_to_deg(min(opts.pan) - opts.reference[0]):+.1f}° ~ '
              f'{core.ticks_to_deg(max(opts.pan) - opts.reference[0]):+.1f}°）')
        print(f'  低頭（29）：{" ".join(str(t) for t in sorted(set(opts.tilt)))}'
              f'（{core.ticks_to_deg(min(opts.tilt) - opts.reference[1]):+.1f}° ~ '
              f'{core.ticks_to_deg(max(opts.tilt) - opts.reference[1]):+.1f}°）')
        print(f'  速度 {cfg.speed}，停穩後等 {cfg.settle:g} 秒、記錄 {cfg.capture:g} 秒')
    else:
        print('手動模式：工具不會轉頭。第一個和最後一個姿勢請放在基準刻度，才能檢查中間有沒有東西動過')
    labels, _ = node.latest_labels()
    print(f'現在看到的瓶子：{" ".join(labels) if labels else "無"}')
    print('整個過程不要動機器人、桌子和瓶子。Ctrl+C 可以隨時停止，已經量的會留著。')


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def _ignore_signals():
    """收尾（轉回頭、寫 meta）最多幾秒，這段時間再按 Ctrl+C 不理，免得頭停在半路。"""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)


def main(argv=None):
    args = remove_ros_args(sys.argv if argv is None else argv)[1:]
    opts = parse_args(args)
    cfg = runner.SweepConfig(speed=opts.speed, settle=opts.settle, capture=opts.capture,
                             max_depth_frames=opts.frames)
    postures = None
    opts.config = opts.config or default_config()
    try:
        plan = neck_model.sweep_plan(neck_model.load_config(opts.config))
    except (OSError, KeyError, ValueError) as e:
        print(f'讀不到頭部設定 {opts.config}：{e}')
        return 2
    if (str(plan['pan_id']), str(plan['tilt_id']), str(plan['roll_id'])) != (
            core.PAN_JOINT, core.TILT_JOINT, core.ROLL_JOINT):
        print(f'{opts.config} 的馬達 ID 和工具不一致（工具固定用 28 轉頭、29 低頭、30 歪頭）')
        return 2
    opts.pan = opts.pan or plan['pan']
    opts.tilt = opts.tilt or plan['tilt']
    opts.reference = opts.reference or list(plan['reference'])
    try:
        cfg.validate()
        if not opts.manual:
            postures = core.build_postures(opts.pan, opts.tilt, opts.reference)
            core.check_postures(postures, plan['pan_limits'], plan['tilt_limits'])
    except ValueError as e:
        print(f'參數錯誤：{e}')
        return 2
    out_dir = Path(opts.out).expanduser() / opts.name
    if out_dir.exists():
        print(f'{out_dir} 已經存在，換一個名稱')
        return 2

    # 不用 rclpy 的訊號處理：它收到 Ctrl+C 會先關掉 ROS，頭就沒辦法轉回去了。
    # 改成 Ctrl+C（和 kill 的 SIGTERM）都變成 KeyboardInterrupt，由下面的 finally 收尾。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _raise_interrupt)
    node = SweepNode(opts, cfg)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    writer = None
    status = 'aborted'
    start = None
    commanded = False
    code = 1
    try:
        if not wait_inputs(node, opts):
            return 1
        start = node.latest_joint()
        roll = node.latest_roll()
        lo, hi = plan['roll_limits']
        if roll is None or not lo <= roll <= hi:
            got = '沒收到' if roll is None else f'{roll:.0f}'
            print(f'歪頭（30 號）是 {got}，要在 {lo}～{hi}（基準 {plan["roll_reference"]}）。'
                  '頭部模型假設歪頭固定在基準，先把 30 號轉回基準再跑')
            return 1
        if not opts.manual:
            others = node.other_head_publishers()
            if others and not opts.force:
                print(f'還有其他程式在發 {opts.head_topic}：{", ".join(others)}')
                print('它們會跟這支工具搶頭部（例如任務管理、網頁），先關掉再跑；確定它們不會動頭的話加 --force')
                return 1
            node.make_head_publisher()
        print_plan(opts, cfg, node, postures, start)
        if not opts.yes:
            try:
                input('按 Enter 開始（Ctrl+C 取消）')
            except EOFError:
                print('\n沒有可以按 Enter 的終端機；不用確認直接開始的話加 --yes')
                return 1
        start = node.latest_joint()
        writer = runner.SweepWriter(out_dir)
        writer.write_meta(build_meta(opts, cfg, node, postures, start, plan))
        print(copy_robot_config(out_dir, opts.config))
        if opts.manual:
            rows, bottles = runner.run_manual(node, cfg, writer)
        else:
            commanded = True
            rows, bottles = runner.run_sweep(node, postures, cfg, writer)
        for line in runner.format_summary(rows, bottles):
            print(line)
        status = 'finished'
        code = 0
    except KeyboardInterrupt:
        _ignore_signals()
        print('\n中斷了。' + ('已經量的姿勢都存好了' if writer is not None else '還沒開始量'))
        status = 'interrupted'
        code = 130
    except runner.SweepError as e:
        print(f'\n停止：{e}')
        status = f'error: {e}'
    finally:
        _ignore_signals()
        if commanded and start is not None:
            return_head(node, start[1:], cfg)
        if writer is not None:
            writer.finish(status)
            print(f'資料在 {out_dir}（{writer.count} 個姿勢）')
        # 背景執行緒還在 spin 時就銷毀 node 會讓 rclpy 當掉（core dump），所以先停、等它結束
        executor.shutdown(timeout_sec=2.0)
        spin.join(timeout=3.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
