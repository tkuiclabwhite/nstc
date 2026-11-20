"""辨識系統一次開：YOLO 藥罐偵測（med_detect bottle_detect_v2）+ 藥罐 3D 定位（bottle_localizer）+ 相機外參 TF。

    ros2 launch bottle_localization vision.launch.py

要先開 nstc.launch.py（ZED、馬達 driver、rosbridge、web_video_server 都在裡面，大家共用，這裡不開）。
監控畫面：瀏覽器開 web/VisionInterface.html（導覽列 Vision）。

參數（其他 bottle_localization.launch.py 的參數也可以直接加，例如 params_file:=、extrinsics_file:=）：
    camera_tf:=head     （預設）依頭部馬達刻度的動態 TF（head_tf）；static = 固定外參（頭部馬達沒開時）
    log_level:=debug    印出每個框被跳過的原因
    yolo:=false         不開 YOLO（YOLO 另外開，例如要看它的終端機輸出）
    yolo_engine:=<路徑>  換 YOLO 模型（預設 med_detect/Parameter/best(1).engine）

bottle_viewer、bottle_tuner 要鍵盤輸入，launch 底下收不到，要用的話另開終端機。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _yolo(context):
    if LaunchConfiguration('yolo').perform(context).lower() not in ('true', '1', 'yes'):
        return []
    engine = LaunchConfiguration('yolo_engine').perform(context)
    return [Node(
        package='med_detect',
        executable='bottle_detect_v2',
        output='screen',
        parameters=[{'engine_path': engine}] if engine else [],
    )]


def generate_launch_description():
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bottle_localization'), 'launch', 'bottle_localization.launch.py')),
        launch_arguments={'camera_tf': LaunchConfiguration('camera_tf'),
                          'log_level': LaunchConfiguration('log_level')}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('camera_tf', default_value='head',
                              description='相機外參的 TF：head = 依頭部馬達刻度的動態 TF、static = 固定外參'),
        DeclareLaunchArgument('log_level', default_value='info',
                              description='bottle_localizer 的 log 等級；debug 會印出每個框被跳過的原因'),
        DeclareLaunchArgument('yolo', default_value='true', description='要不要開 YOLO（bottle_detect_v2）'),
        DeclareLaunchArgument('yolo_engine', default_value='',
                              description='YOLO 模型路徑；空的 = bottle_detect_v2 預設的 Parameter/best(1).engine'),
        OpaqueFunction(function=_yolo),
        localization,
    ])
