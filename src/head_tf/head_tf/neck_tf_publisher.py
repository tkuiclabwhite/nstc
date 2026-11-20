#!/usr/bin/env python3
"""動態相機 TF：由頭部馬達刻度（/joint_states 的 28/29/30 號）即時發 base_link -> zed_camera_link。

取代 bottle_localization 的固定外參（camera_tf_publisher）。平常由 bottle_localization.launch.py 開
（camera_tf:=head，預設），節點名稱用 bottle_localization_camera_tf，所以 bottle_tuner 調外參照常可以用。

- 每收到一筆有位置的 /joint_states 就發一次 /tf，時間戳記用 /joint_states 的（driver_node 約 50 Hz）。
- 頭在基準刻度（neck_head.yaml 的 reference_ticks）時，結果和 camera_extrinsics.yaml 完全一樣。
- 不可用（超出允許範圍、沒有某顆馬達的刻度、/joint_states 停了）就不發 TF；bottle_localizer 查不到 TF 會跳過那一幀。
- /head_tf/status（std_msgs/String，JSON，5 Hz，可不可用改變時馬上發一筆）：valid、reasons、ticks、moving、
  at_reference、age_s、camera（zed_camera_link 在 base_link 的位置 m 與 rpy 度）。
- x、y、z、roll、pitch、yaw 是「基準刻度時」的外參，執行中可以改（bottle_tuner、ros2 param set），改了模型立刻重算。
  調外參要先把頭轉回基準刻度，否則調出來的值不對。

需要 ZED 不發 odom -> zed_camera_link（nstc.launch.py 的 ZED 已設 publish_tf: false），否則 zed_camera_link 會有兩個 parent。
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros.transform_broadcaster import TransformBroadcaster

from head_tf import neck_model as nm
from head_tf.head_model import rpy_matrix
from head_tf.neck_tf_core import NeckTfCore, matrix_to_quaternion, parse_joint_state

FRAME_PARAMS = ('base_frame', 'camera_frame')
EXTRINSIC_PARAMS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
OPTIONS = {'config_file': '', 'joint_topic': '/joint_states', 'status_topic': '/head_tf/status',
           'stale_s': 0.5, 'still_ticks': 3.0, 'still_s': 0.3, 'status_hz': 5.0}


def check_value(name, value):
    """回 (float 值, None)；不是有限的數字時回 (None, 原因)。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None, f'{name} 必須是數字（{value!r}）'
    return float(value), None


class NeckTfPublisher(Node):
    def __init__(self):
        super().__init__('neck_tf_publisher')
        values, missing = {}, []
        for name in FRAME_PARAMS + EXTRINSIC_PARAMS:
            self.declare_parameter(name, None, ParameterDescriptor(dynamic_typing=True))
            value = self.get_parameter(name).value
            if value is None:
                missing.append(name)
            else:
                values[name] = value
        if missing:
            raise ValueError('缺少參數: ' + ', '.join(missing) + '（平常由 bottle_localization.launch.py 帶進來）')
        opts = {k: self.declare_parameter(k, v).value for k, v in OPTIONS.items()}
        self._frames = {name: str(values[name]) for name in FRAME_PARAMS}
        self._extrinsics = {}
        for name in EXTRINSIC_PARAMS:
            value, reason = check_value(name, values[name])
            if reason:
                raise ValueError(reason)
            self._extrinsics[name] = value

        self._config_path = opts['config_file'] or nm.default_config()
        self._config = nm.load_config(self._config_path)
        self.core = NeckTfCore(self._build_model(self._extrinsics), opts['stale_s'], opts['still_ticks'],
                               opts['still_s'])
        self._last_valid = None
        self._last_reasons = None

        self._broadcaster = TransformBroadcaster(self)
        self._status_pub = self.create_publisher(String, opts['status_topic'], 10)
        self.create_subscription(JointState, opts['joint_topic'], self._on_joint, 50)
        self.create_timer(1.0 / float(opts['status_hz']), self._on_timer)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        m = self.core.model
        ref = '、'.join(f'{k} = {v:.0f}' for k, v in m.reference_ticks.items())
        self.get_logger().info(
            f"動態 TF {self._frames['base_frame']} -> {self._frames['camera_frame']}；頭部模型 {self._config_path}；"
            f"基準刻度 {ref}；轉頭軸傾斜修正 roll {m.neck_tilt_deg[0]:+.2f}° pitch {m.neck_tilt_deg[1]:+.2f}°")
        self._log_extrinsics()

    def _build_model(self, e):
        R0 = rpy_matrix(e['roll'], e['pitch'], e['yaw'])
        return nm.NeckModel(self._config, R0, [e['x'], e['y'], e['z']])

    def _log_extrinsics(self):
        e = self._extrinsics
        self.get_logger().info(
            f"基準刻度時的外參：x {e['x']:.4f} y {e['y']:.4f} z {e['z']:.4f} m, roll {math.degrees(e['roll']):.2f} "
            f"pitch {math.degrees(e['pitch']):.2f} yaw {math.degrees(e['yaw']):.2f} 度")

    # ---------------- callback ----------------
    def _on_joint(self, msg):
        ticks, missing = parse_joint_state(msg.name, msg.position, self.core.model.motor_ids)
        if ticks is None:
            return
        ok, pose = self.core.update(ticks, missing, time.monotonic())
        if ok:
            stamp = msg.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                stamp = self.get_clock().now().to_msg()
            self._send(pose, stamp)
        self._report_change()

    def _send(self, pose, stamp):
        R, t = pose
        qx, qy, qz, qw = matrix_to_quaternion(R)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._frames['base_frame']
        tf.child_frame_id = self._frames['camera_frame']
        tf.transform.translation.x = float(t[0])
        tf.transform.translation.y = float(t[1])
        tf.transform.translation.z = float(t[2])
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._broadcaster.sendTransform(tf)

    def _on_timer(self):
        self._report_change()
        self._publish_status()

    def _report_change(self):
        """可不可用改變時寫 log、馬上發一筆狀態。"""
        status = self.core.status(time.monotonic())
        if status['valid'] == self._last_valid and status['reasons'] == self._last_reasons:
            return
        if status['valid']:
            self.get_logger().info('頭部刻度可用，開始發 TF：' + json.dumps(status['ticks']))
        else:
            self.get_logger().warn('停止發 TF：' + '；'.join(status['reasons']))
        self._last_valid, self._last_reasons = status['valid'], status['reasons']
        self._publish_status(status)

    def _publish_status(self, status=None):
        status = status or self.core.status(time.monotonic())
        self._status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def _on_set_parameters(self, params):
        """只接受外參 6 個值（和 camera_tf_publisher 一樣，給 bottle_tuner 用）；檢查都通過才一起套用。"""
        updates = {}
        for p in params:
            if p.name not in EXTRINSIC_PARAMS:
                return SetParametersResult(
                    successful=False,
                    reason=f'執行中只能修改 {", ".join(EXTRINSIC_PARAMS)}；{p.name} 請改設定檔後重新 launch')
            if p.type_ == Parameter.Type.NOT_SET:
                return SetParametersResult(successful=False, reason=f'{p.name} 不能刪除')
            value, reason = check_value(p.name, p.value)
            if reason:
                return SetParametersResult(successful=False, reason=reason)
            updates[p.name] = value
        new = dict(self._extrinsics, **updates)
        try:
            model = self._build_model(new)
        except ValueError as e:
            return SetParametersResult(successful=False, reason=str(e))
        self._extrinsics = new
        self.core.set_model(model)
        self._log_extrinsics()
        if not self.core.at_reference():
            self.get_logger().warn('頭現在不在基準刻度：外參是「基準刻度時」的值，要先把頭轉回基準刻度再調')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = NeckTfPublisher()
    except (ValueError, OSError, KeyError) as e:
        get_logger('neck_tf_publisher').fatal(f'啟動失敗: {e}')
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
