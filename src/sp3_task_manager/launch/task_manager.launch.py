"""子三任務管理。

    ros2 launch sp3_task_manager task_manager.launch.py               # 接實機
    ros2 launch sp3_task_manager task_manager.launch.py dry_run:=true # 不接硬體，測流程與主控通訊
    ros2 launch sp3_task_manager task_manager.launch.py config:=/path/to/my.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory('sp3_task_manager'),
                               'config', 'task_manager.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='true = 不接硬體，各轉接層模擬成功'),
        DeclareLaunchArgument('config', default_value=default_cfg),
        Node(
            package='sp3_task_manager', executable='task_manager', name='sp3_task_manager',
            output='screen',
            parameters=[{
                'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
                'config': LaunchConfiguration('config'),
            }],
        ),
    ])
