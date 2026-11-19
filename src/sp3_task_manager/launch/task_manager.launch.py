"""子三任務管理。

    ros2 launch sp3_task_manager task_manager.launch.py               # 照設定檔 dry_run.modules
    ros2 launch sp3_task_manager task_manager.launch.py dry_run:=false # 全部接實機（不管設定檔）
    ros2 launch sp3_task_manager task_manager.launch.py dry_run:=true # 全部模擬，測流程與主控通訊
    ros2 launch sp3_task_manager task_manager.launch.py dry_run:=vision        # 假藥瓶，手臂和手真的動
    ros2 launch sp3_task_manager task_manager.launch.py dry_run:=vision,arm    # 只有手真的動
    ros2 launch sp3_task_manager task_manager.launch.py real_arm:=false     # 只看 MoveIt（模擬），不管實機跟隨、馬達
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
        DeclareLaunchArgument('dry_run', default_value='config',
                              description='要模擬哪幾段：true / false / vision,arm,hand 任意組合；'
                                          'config = 用設定檔 dry_run.modules'),
        DeclareLaunchArgument('real_arm', default_value='config',
                              description='true = 要實機跟隨；false = 只看 MoveIt（模擬）；config = 用設定檔 real_arm.required'),
        DeclareLaunchArgument('config', default_value=default_cfg),
        Node(
            package='sp3_task_manager', executable='task_manager', name='sp3_task_manager',
            output='screen',
            parameters=[{
                'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=str),
                'real_arm': ParameterValue(LaunchConfiguration('real_arm'), value_type=str),
                'config': LaunchConfiguration('config'),
            }],
        ),
    ])
