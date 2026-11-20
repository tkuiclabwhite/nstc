"""只啟動 bottle_localizer 與相機外參的 TF（camera_tf_publisher）。

不啟動 ZED、YOLO 或深度節點 —— 那些由機器人既有的 camera.launch.py 與
bottle_detect_v2 負責（同一時間只能有一個程式開相機）。

    ros2 launch bottle_localization bottle_localization.launch.py

四個設定檔都可以用 launch 參數換掉:
    params_file:=...  dims_file:=...  extrinsics_file:=...  reference_file:=...

看每個框被跳過的原因（合理性檢查、點數不足、碰到邊緣…）:
    log_level:=debug

執行中調整桌高、外參：另開一個終端機執行 bottle_tuner（README「bottle_tuner」）。
TUI 需要鍵盤輸入，launch 底下的程式收不到，所以不包在這裡。

相機外參的 TF（base_link -> zed_camera_link）用 camera_tf 選：
    camera_tf:=head    （預設）head_tf 的 neck_tf_publisher 依頭部馬達刻度（/joint_states 28/29/30）發動態 TF，
                       頭轉了藥罐位置照樣準；camera_extrinsics.yaml 是頭在基準刻度（head_tf/config/neck_head.yaml）時的外參。
                       超出允許範圍或 /joint_states 停了就不發 TF，bottle_localizer 會跳過那幾幀。狀態看 /head_tf/status。
    camera_tf:=static  camera_tf_publisher 發固定外參（頭部馬達沒開時用；頭轉了就不準）。
兩種節點名稱都是 bottle_localization_camera_tf，bottle_tuner 調外參照常可以用（head 模式要先把頭轉回基準刻度）。
ZED 不能發 odom -> zed_camera_link（nstc.launch.py 已設 publish_tf: false），否則 zed_camera_link 會有兩個 parent（README「TF」一節）。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

EXTRINSIC_KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')


def _load_yaml(context, arg, keys):
    path = LaunchConfiguration(arg).perform(context)
    with open(path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}
    missing = [k for k in keys if k not in cfg]
    if missing:
        raise RuntimeError(f'{path} 缺少欄位: {", ".join(missing)}')
    return path, cfg


def _nodes(context):
    ref_path, ref = _load_yaml(context, 'reference_file',
                               ('base_frame', 'reference_point', 'reference_height_cm'))
    ext_path, ext = _load_yaml(context, 'extrinsics_file', ('child_frame',) + EXTRINSIC_KEYS)
    point = ref['reference_point']
    if not isinstance(point, list) or len(point) != 3:
        raise RuntimeError(f'{ref_path} 的 reference_point 要是 [x, y, z]')

    actions = []
    if not ext.get('measured', False):
        actions.append(LogInfo(msg=(
            f'[bottle_localization] 警告：{ext_path} 還是 placeholder（measured: false），'
            f'{ref["base_frame"]} 的位置不可信。量完外參後填入並把 measured 改成 true。')))

    tf_params = dict({'base_frame': str(ref['base_frame']), 'camera_frame': str(ext['child_frame'])},
                     **{k: float(ext[k]) for k in EXTRINSIC_KEYS})
    camera_tf = LaunchConfiguration('camera_tf').perform(context)
    if camera_tf == 'head':
        actions.append(Node(
            package='head_tf',
            executable='neck_tf_publisher',
            name='bottle_localization_camera_tf',
            output='screen',
            parameters=[dict(tf_params, config_file=LaunchConfiguration('neck_config').perform(context))],
        ))
    elif camera_tf == 'static':
        actions.append(LogInfo(msg='[bottle_localization] camera_tf:=static：用固定外參，頭轉了藥罐位置就不準'))
        actions.append(Node(
            package='bottle_localization',
            executable='camera_tf_publisher',
            name='bottle_localization_camera_tf',
            output='screen',
            parameters=[tf_params],
        ))
    else:
        raise RuntimeError(f'camera_tf 只能是 head 或 static（{camera_tf!r}）')

    actions.append(Node(
        package='bottle_localization',
        executable='bottle_localizer',
        name='bottle_localizer',
        output='screen',
        parameters=[LaunchConfiguration('params_file').perform(context), {
            'bottle_dims_file': LaunchConfiguration('dims_file').perform(context),
            'target_frame': str(ref['base_frame']),
            'reference_height_cm': float(ref['reference_height_cm']),
            'reference_z': float(point[2]),
        }],
        #只調本節點的 logger，不會把 rclpy 自己的 debug 訊息也打開
        arguments=['--ros-args', '--log-level',
                   'bottle_localizer:=' + LaunchConfiguration('log_level').perform(context)],
    ))
    return actions


def generate_launch_description():
    config = os.path.join(get_package_share_directory('bottle_localization'), 'config')
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
        DeclareLaunchArgument(
            'reference_file', default_value=os.path.join(config, 'robot_reference.yaml'),
            description='base_link 名稱與量測基準點（換手臂時改這個檔）'),
        DeclareLaunchArgument(
            'camera_tf', default_value='head',
            description='相機外參的 TF：head = 依頭部馬達刻度的動態 TF（head_tf）、static = 固定外參'),
        DeclareLaunchArgument(
            'neck_config', default_value='',
            description='camera_tf:=head 用的頭部模型；空的 = head_tf 安裝的 config/neck_head.yaml'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='bottle_localizer 的 log 等級；debug 會印出每個框被跳過的原因'),
        OpaqueFunction(function=_nodes),
    ])
