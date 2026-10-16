"""只啟動 bottle_localizer 與相機外參的 static transform。

不啟動 ZED、YOLO 或深度節點 —— 那些由機器人既有的 camera.launch.py 與
bottle_detect_v2 負責（同一時間只能有一個程式開相機）。

    ros2 launch bottle_localization bottle_localization.launch.py

三個設定檔都可以用 launch 參數換掉:
    params_file:=...  dims_file:=...  extrinsics_file:=...

相機外參（camera_extrinsics.yaml）量的是 base_link -> zed_camera_link，
但 ZED wrapper 已經在發 odom -> zed_camera_link，zed_camera_link 不能再有第二個 parent。
所以這裡發的是反過來的 zed_camera_link -> base_link（README「TF」一節）。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from bottle_localization.geometry import invert_transform, matrix_to_quaternion, rpy_to_matrix


def _static_tf(context):
    path = LaunchConfiguration('extrinsics_file').perform(context)
    with open(path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    keys = ('parent_frame', 'child_frame', 'x', 'y', 'z', 'roll', 'pitch', 'yaw')
    missing = [k for k in keys if k not in cfg]
    if missing:
        raise RuntimeError(f'{path} 缺少欄位: {", ".join(missing)}')

    rot = rpy_to_matrix(float(cfg['roll']), float(cfg['pitch']), float(cfg['yaw']))
    trans = [float(cfg['x']), float(cfg['y']), float(cfg['z'])]
    inv_rot, inv_trans = invert_transform(rot, trans)
    qx, qy, qz, qw = matrix_to_quaternion(inv_rot)

    actions = []
    if not cfg.get('measured', False):
        actions.append(LogInfo(msg=(
            f'[bottle_localization] 警告：{path} 還是 placeholder（measured: false），'
            f'{cfg["parent_frame"]} 的位置不可信。量完外參後填入並把 measured 改成 true。')))

    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='bottle_localization_camera_tf',
        arguments=[
            '--x', repr(float(inv_trans[0])),
            '--y', repr(float(inv_trans[1])),
            '--z', repr(float(inv_trans[2])),
            '--qx', repr(qx), '--qy', repr(qy), '--qz', repr(qz), '--qw', repr(qw),
            # 反轉換：parent/child 對調
            '--frame-id', str(cfg['child_frame']),
            '--child-frame-id', str(cfg['parent_frame']),
        ],
        output='screen',
    ))
    return actions


def generate_launch_description():
    share = get_package_share_directory('bottle_localization')
    config = os.path.join(share, 'config')

    params_file = LaunchConfiguration('params_file')
    dims_file = LaunchConfiguration('dims_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(config, 'params.yaml'),
            description='bottle_localizer 的節點參數'),
        DeclareLaunchArgument(
            'dims_file', default_value=os.path.join(config, 'bottle_dims.yaml'),
            description='藥罐尺寸（公尺）'),
        DeclareLaunchArgument(
            'extrinsics_file', default_value=os.path.join(config, 'camera_extrinsics.yaml'),
            description='相機外參 base_link -> zed_camera_link'),

        OpaqueFunction(function=_static_tf),

        Node(
            package='bottle_localization',
            executable='bottle_localizer',
            name='bottle_localizer',
            output='screen',
            parameters=[params_file, {'bottle_dims_file': dims_file}],
        ),
    ])
