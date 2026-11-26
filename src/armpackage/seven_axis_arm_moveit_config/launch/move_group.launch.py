"""move_group（demo.launch.py 會 include 這個檔）。

內容 = moveit_configs_utils.launches.generate_move_group_launch（Humble）原樣抄過來，只多一個 ros_arguments：
planning_scene_monitor 這個 logger 只印 ERROR 以上。

為什麼（2026-11-26 子三使用者要的）：ZED 發的 zed_camera_link → zed_camera_center → 左右眼 這幾個 TF，
要靠 bottle_localization 開的相機 TF（head_tf neck_tf_publisher）才會接到 base_link 底下。沒開影像（例如任務管理
dry_run:=vision）或頭部馬達沒上電時，相機那棵樹是斷開的，move_group 每次更新場景都對每個座標系印一次
「Unable to transform object from frame 'zed_...' to planning frame 'base_link'」（一次開機印了 2.8 萬次）。
MoveIt 用不到相機的座標系（障礙物都用 base_link 送），所以只關掉這個 logger 的 WARN；其他 logger 照常。
要恢復：把 QUIET_LOGGERS 清空，或改回原本的兩行（generate_move_group_launch(moveit_config)）。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node

QUIET_LOGGERS = ["moveit_ros.planning_scene_monitor.planning_scene_monitor"]


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("seven_axis_arm_urdf", package_name="seven_axis_arm_moveit_config").to_moveit_configs()

    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument(
        "capabilities", default_value=moveit_config.move_group_capabilities["capabilities"]))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument(
        "disable_capabilities", default_value=moveit_config.move_group_capabilities["disable_capabilities"]))
    # do not copy dynamics information from /joint_states to internal robot monitoring
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(LaunchConfiguration("capabilities"), value_type=str),
        "disable_capabilities": ParameterValue(LaunchConfiguration("disable_capabilities"), value_type=str),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    ros_arguments = []
    for name in QUIET_LOGGERS:
        ros_arguments += ["--log-level", f"{name}:=error"]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=[moveit_config.to_dict(), move_group_configuration],
        extra_debug_args=["--debug"],
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        ros_arguments=ros_arguments,
    )
    return ld
