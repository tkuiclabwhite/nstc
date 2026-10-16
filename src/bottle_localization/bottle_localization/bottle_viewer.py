#!/usr/bin/env python3
#coding=utf-8
"""即時顯示 /bottle_detections_3d 的座標（終端機版，SSH 進機器人也能看）。

    ros2 run bottle_localization bottle_viewer
    ros2 run bottle_localization bottle_viewer --ros-args -p window:=5.0

畫面每秒更新 refresh_hz 次，每個類別一行:
    x / y / z     最新的瓶身中心（frame_id 座標系，公尺）
    信心          這一筆的 YOLO 信心值
    多久前        最後一次出現在輸出裡是幾秒前
    出現率        最近 window 秒的訊息裡，有幾成包含這個類別。
                  偏低代表 YOLO 時有時無，或被 bottle_localizer 跳過
                  （用 log_level:=debug 啟動 bottle_localizer 看原因）
    抖動          最近 window 秒內 x、y 的標準差（mm）。瓶子靜止時就是量測的重複性

統計與排版（ViewerState）不依賴 ROS，可以單獨測試；ROS 的部分在 main() 裡才 import。
"""
import statistics
import sys
import time
import unicodedata
from collections import deque


def _width(text):
    """終端機上的顯示寬度：中文等全形字佔兩格。"""
    return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in text)


def _ljust(text, width):
    return text + ' ' * max(0, width - _width(text))


def _rjust(text, width):
    return ' ' * max(0, width - _width(text)) + text


class ViewerState:
    """收到的訊息與統計。時間一律用呼叫端給的 now（秒），方便測試。"""

    def __init__(self, window):
        if window <= 0:
            raise ValueError(f'window 必須 > 0，收到 {window}')
        self.window = float(window)
        self._messages = deque()      #(收到的時間, frame_id, {類別: (x, y, z, score)})
        self._last_seen = {}          #類別 -> (收到的時間, (x, y, z, score))

    def add(self, now, frame_id, detections):
        """加入一筆 Detection3DArray（已整理成 {類別: (x, y, z, score)}）。"""
        self._messages.append((now, frame_id, dict(detections)))
        for label, value in detections.items():
            self._last_seen[label] = (now, value)
        self._trim(now)

    def _trim(self, now):
        while self._messages and now - self._messages[0][0] > self.window:
            self._messages.popleft()

    def rate(self):
        """最近 window 秒的訊息頻率（Hz）。"""
        if len(self._messages) < 2:
            return 0.0
        span = self._messages[-1][0] - self._messages[0][0]
        return (len(self._messages) - 1) / span if span > 0 else 0.0

    def class_stats(self, label):
        """最近 window 秒內這個類別的出現率與 x、y 標準差（公尺；少於 2 筆時是 None）。"""
        n = len(self._messages)
        values = [m[2][label] for m in self._messages if label in m[2]]
        ratio = len(values) / n if n else 0.0
        if len(values) < 2:
            return ratio, None, None
        return (ratio,
                statistics.pstdev(v[0] for v in values),
                statistics.pstdev(v[1] for v in values))

    def render(self, now, topic):
        """排成要顯示的文字。"""
        self._trim(now)
        if not self._last_seen and not self._messages:
            return f'等待 {topic} ...（bottle_localizer 有在跑嗎？）'

        lines = []
        if self._messages:
            t_last, frame_id, latest = self._messages[-1]
            lines.append(f'{topic}   座標系 {frame_id}   {self.rate():5.1f} Hz   '
                         f'最後一筆 {now - t_last:4.2f} s 前   '
                         f'這一筆有 {len(latest)} 個瓶子')
        else:
            lines.append(f'{topic}   最近 {self.window:g} 秒沒有收到訊息（bottle_localizer 有在跑嗎？）')
        lines.append(f'統計範圍：最近 {self.window:g} 秒')
        lines.append('')
        #欄寬（顯示寬度）要和下面數字那一列的格式一致
        lines.append(_ljust('類別', 8) + _rjust('x (m)', 9) + _rjust('y (m)', 9) +
                     _rjust('z (m)', 9) + _rjust('信心', 7) + _rjust('多久前', 9) +
                     _rjust('出現率', 8) + '   抖動 x / y (mm)')

        for label in sorted(self._last_seen):
            t_seen, (x, y, z, score) = self._last_seen[label]
            ratio, sx, sy = self.class_stats(label)
            jitter = '   -' if sx is None else f'{sx * 1000:5.1f} / {sy * 1000:4.1f}'
            row = (f'{label:<8}{x:+9.3f}{y:+9.3f}{z:+9.3f}{score:7.2f}'
                   f'{now - t_seen:8.2f}s{ratio * 100:7.0f}%   {jitter}')
            if now - t_seen > self.window:
                row += f'   （超過 {self.window:g} 秒沒看到，顯示的是最後一次的值）'
            lines.append(row)
        return '\n'.join(lines)


def detections_to_dict(msg):
    """Detection3DArray -> {類別: (x, y, z, score)}。"""
    out = {}
    for d in msg.detections:
        if d.results:
            hyp = d.results[0]
            label = d.id or hyp.hypothesis.class_id
            p = hyp.pose.pose.position
            score = hyp.hypothesis.score
        else:
            label = d.id
            p = d.bbox.center.position
            score = float('nan')
        out[label] = (p.x, p.y, p.z, score)
    return out


def main(args=None):
    #ROS 在這裡才 import，ViewerState 的測試不需要 ROS 環境
    import rclpy
    from rclpy.node import Node
    from vision_msgs.msg import Detection3DArray

    class BottleViewer(Node):
        def __init__(self):
            super().__init__('bottle_viewer')
            self.declare_parameter('topic', '/bottle_detections_3d')
            self.declare_parameter('window', 3.0)
            self.declare_parameter('refresh_hz', 5.0)
            self.topic = str(self.get_parameter('topic').value)
            refresh_hz = float(self.get_parameter('refresh_hz').value)
            self.state = ViewerState(float(self.get_parameter('window').value))
            self.tty = sys.stdout.isatty()
            self.create_subscription(Detection3DArray, self.topic, self.callback, 10)
            self.create_timer(1.0 / max(refresh_hz, 0.1), self.draw)

        def callback(self, msg):
            self.state.add(time.monotonic(), msg.header.frame_id, detections_to_dict(msg))

        def draw(self):
            text = self.state.render(time.monotonic(), self.topic)
            if self.tty:
                #游標回左上角、清掉畫面再重畫，看起來就是原地更新
                sys.stdout.write('\033[H\033[J' + text + '\n')
            else:
                sys.stdout.write(text + '\n' + '-' * 40 + '\n')
            sys.stdout.flush()

    rclpy.init(args=args)
    node = BottleViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
