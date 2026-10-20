#!/usr/bin/env python3
#coding=utf-8
"""把機器人內部的幾個 topic 轉到網路上，給桌機的 RViz 看。

機器人的 ~/.bashrc 設了 ROS_DOMAIN_ID=10、ROS_LOCALHOST_ONLY=1，ROS 2 不往網路送資料。
實測（Humble / Fast DDS）ROS_LOCALHOST_ONLY=1 與 =0 的節點彼此收不到，所以不能只把
bottle_localizer 改成對外 —— 它會收不到 YOLO 與 ZED。

本程式在同一個行程裡開兩個 context：
    輸入   ROS_LOCALHOST_ONLY=1、機器人的 domain：收機器人內部的 topic
    輸出   ROS_LOCALHOST_ONLY=0、out_domain：原封不動（不解序列化）重新發布到網路
只單向轉發，機器人其他節點與設定都不動。輸出的 domain 必須和輸入不同，避免轉出去的資料繞回來。

    ros2 run bottle_localization rviz_relay                  # 輸出到 domain 0
    ros2 run bottle_localization rviz_relay --point-cloud    # 連 ZED 點雲一起轉（很吃頻寬）
    ros2 run bottle_localization rviz_relay --out-domain 5

桌機：ROS_DOMAIN_ID 設成 out_domain（預設 0，即不設）、ROS_LOCALHOST_ONLY=0，再開 RViz。
"""
import argparse
import os
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray


def _qos(depth, reliable=True, latched=False):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=depth,
        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)


#(topic, 型別, QoS)。/tf_static 要 TRANSIENT_LOCAL，RViz 晚開也拿得到所有靜態 TF
TOPICS = [
    ('/tf', TFMessage, _qos(100)),
    ('/tf_static', TFMessage, _qos(100, latched=True)),
    ('/bottle_markers', MarkerArray, _qos(10)),
]
POINT_CLOUD = ('/zed/zed_node/point_cloud/cloud_registered', PointCloud2, _qos(1, reliable=False))


def make_context(localhost_only, domain_id):
    """建一個 context。rcl 在 init 時讀 ROS_LOCALHOST_ONLY，所以先設好環境變數再 init。"""
    os.environ['ROS_LOCALHOST_ONLY'] = '1' if localhost_only else '0'
    ctx = rclpy.Context()
    rclpy.init(args=None, context=ctx, domain_id=domain_id)
    return ctx


def main(args=None):
    parser = argparse.ArgumentParser(description='把機器人內部的 topic 轉到網路上給 RViz 看')
    parser.add_argument('--in-domain', type=int,
                        default=int(os.environ.get('ROS_DOMAIN_ID', '0') or 0),
                        help='機器人的 ROS_DOMAIN_ID（預設讀環境變數）')
    parser.add_argument('--out-domain', type=int, default=0,
                        help='轉出去的 domain，桌機要設成一樣（預設 0）')
    parser.add_argument('--point-cloud', action='store_true',
                        help='連 ZED 點雲一起轉（每張約 9 MB，Wi-Fi 會很卡）')
    opts = parser.parse_args(remove_ros_args(args if args is not None else sys.argv)[1:])
    if opts.in_domain == opts.out_domain:
        parser.error(f'--out-domain 不能和機器人的 domain（{opts.in_domain}）一樣，'
                     f'否則轉出去的資料可能繞回來')

    topics = TOPICS + ([POINT_CLOUD] if opts.point_cloud else [])
    ctx_in = make_context(True, opts.in_domain)
    ctx_out = make_context(False, opts.out_domain)
    node_in = rclpy.create_node('rviz_relay_in', context=ctx_in)
    node_out = rclpy.create_node('rviz_relay_out', context=ctx_out)

    counts = {topic: 0 for topic, _, _ in topics}
    for topic, msg_type, qos in topics:
        pub = node_out.create_publisher(msg_type, topic, qos)

        def forward(data, pub=pub, topic=topic):
            pub.publish(data)          #raw=True：收到的是序列化後的 bytes，原樣送出
            counts[topic] += 1
        node_in.create_subscription(msg_type, topic, forward, qos, raw=True)

    def report():
        node_out.get_logger().info(
            '已轉發 ' + '，'.join(f'{t} {n}' for t, n in counts.items()))
    node_out.create_timer(10.0, report)

    node_out.get_logger().info(
        f'domain {opts.in_domain}（只在本機）-> domain {opts.out_domain}（網路）：'
        + '、'.join(t for t, _, _ in topics))

    exec_in = SingleThreadedExecutor(context=ctx_in)
    exec_out = SingleThreadedExecutor(context=ctx_out)
    exec_in.add_node(node_in)
    exec_out.add_node(node_out)
    worker = threading.Thread(target=exec_in.spin, daemon=True)
    worker.start()
    try:
        exec_out.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for ctx in (ctx_in, ctx_out):
            if ctx.ok():
                rclpy.shutdown(context=ctx)


if __name__ == '__main__':
    main()
