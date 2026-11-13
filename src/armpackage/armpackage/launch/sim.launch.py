"""
一個指令開全部：雙手 MoveIt（含 RViz）+ rosbridge + MoveIt 橋接 + 打開 RobotViewer.html

    ros2 launch armpackage sim.launch.py
    參數：rviz:=false（不開 RViz）　browser:=false（不自動開網頁）　port:=9090（rosbridge 的 port）
          rosbridge:=false（不開 rosbridge，用 camera.launch.py 開的 9090）　real:=true（開實機橋接，見 viewer.launch.py）

請在 Ubuntu 一般的終端機執行（VSCode 內建終端機的環境變數會讓 RViz 開不起來）
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit = os.path.join(get_package_share_directory("seven_axis_arm_moveit_config"), "launch", "demo.launch.py")
    viewer = os.path.join(get_package_share_directory("armpackage"), "launch", "viewer.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="false = 不開 RViz"),
        DeclareLaunchArgument("browser", default_value="true", description="false = 不自動開網頁"),
        DeclareLaunchArgument("port", default_value="9090", description="rosbridge 的 port"),
        DeclareLaunchArgument("rosbridge", default_value="true", description="false = 不開 rosbridge（用 camera.launch.py 開的 9090）"),
        DeclareLaunchArgument("real", default_value="false", description="true = 開實機橋接 real_arm_bridge.py"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(moveit),
                                 launch_arguments={"use_rviz": LaunchConfiguration("rviz")}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(viewer),
                                 launch_arguments={"browser": LaunchConfiguration("browser"),
                                                   "port": LaunchConfiguration("port"),
                                                   "rosbridge": LaunchConfiguration("rosbridge"),
                                                   "real": LaunchConfiguration("real")}.items()),
    ])
