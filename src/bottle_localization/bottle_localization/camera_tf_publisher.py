#!/usr/bin/env python3
#coding=utf-8
"""相機外參的 TF：發 camera_frame -> base_frame（外參的反轉換），執行中可以修改。

外參（camera_extrinsics.yaml）是 base_link -> zed_camera_link，但 ZED wrapper 已經在發
odom -> zed_camera_link，zed_camera_link 不能再有第二個 parent，所以發反過來的那一個
（README「TF」一節）。查 base_link <-> 左眼 optical frame 的結果與直接發 base_link -> zed_camera_link 相同。

x、y、z、roll、pitch、yaw 可以在執行中修改（bottle_tuner 或 ros2 param set），
改了立刻重新發布 /tf_static。執行中改的值不會寫回 yaml，要存檔請用 bottle_tuner。
參數由 launch 從 robot_reference.yaml 與 camera_extrinsics.yaml 帶進來，程式裡不放預設值。
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from bottle_localization import geometry

FRAME_PARAMS = ('base_frame', 'camera_frame')
#執行中可以修改的外參（位置 m、角度 rad）
EXTRINSIC_PARAMS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


class CameraTfPublisher(Node):
#國科會計畫 - 藥罐 3D 定位：相機外參 TF
    def __init__(self):
        super().__init__('bottle_localization_camera_tf')
        values = {}
        missing = []
        for name in FRAME_PARAMS + EXTRINSIC_PARAMS:
            self.declare_parameter(name, None, ParameterDescriptor(dynamic_typing=True))
            value = self.get_parameter(name).value
            if value is None:
                missing.append(name)
            else:
                values[name] = value
        if missing:
            raise ValueError('缺少參數: ' + ', '.join(missing))
        self._frames = {name: str(values[name]) for name in FRAME_PARAMS}
        self._extrinsics = {}
        for name in EXTRINSIC_PARAMS:
            value, reason = self.check_value(name, values[name])
            if reason:
                raise ValueError(reason)
            self._extrinsics[name] = value

        self._broadcaster = StaticTransformBroadcaster(self)
        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.publish()

    @staticmethod
    def check_value(name, value):
        """回 (float 值, None)；不是有限的數字時回 (None, 原因)。"""
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            return None, f'{name} 必須是數字（{value!r}）'
        return float(value), None

    def on_set_parameters(self, params):
        """只接受外參 6 個值；檢查都通過才一起套用並重新發布。"""
        updates = {}
        for p in params:
            if p.name not in EXTRINSIC_PARAMS:
                return SetParametersResult(
                    successful=False,
                    reason=f'執行中只能修改 {", ".join(EXTRINSIC_PARAMS)}；{p.name} 請改設定檔後重新 launch')
            if p.type_ == Parameter.Type.NOT_SET:
                return SetParametersResult(successful=False, reason=f'{p.name} 不能刪除')
            value, reason = self.check_value(p.name, p.value)
            if reason:
                return SetParametersResult(successful=False, reason=reason)
            updates[p.name] = value
        self._extrinsics.update(updates)
        self.publish()
        return SetParametersResult(successful=True)

    def publish(self):
        e = self._extrinsics
        trans, (qx, qy, qz, qw) = geometry.inverse_extrinsics(
            e['x'], e['y'], e['z'], e['roll'], e['pitch'], e['yaw'])
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._frames['camera_frame']
        tf.child_frame_id = self._frames['base_frame']
        tf.transform.translation.x = float(trans[0])
        tf.transform.translation.y = float(trans[1])
        tf.transform.translation.z = float(trans[2])
        tf.transform.rotation.x = float(qx)
        tf.transform.rotation.y = float(qy)
        tf.transform.rotation.z = float(qz)
        tf.transform.rotation.w = float(qw)
        self._broadcaster.sendTransform(tf)
        self.get_logger().info(
            f"外參 {self._frames['base_frame']} -> {self._frames['camera_frame']}: "
            f"x {e['x']:.4f} y {e['y']:.4f} z {e['z']:.4f} m, "
            f"roll {math.degrees(e['roll']):.2f} pitch {math.degrees(e['pitch']):.2f} "
            f"yaw {math.degrees(e['yaw']):.2f} 度")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraTfPublisher()
    except ValueError as e:
        get_logger('bottle_localization_camera_tf').fatal(f"啟動失敗: {e}")
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
