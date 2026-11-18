"""開 ros2_control 的控制器：joint_state_broadcaster + MoveIt 用的手臂控制器（moveit_controllers.yaml 的 controller_names）。

MoveIt Setup Assistant 產生的版本（generate_spawn_controllers_launch）是每個控制器各開一個 spawner、同時跑。
開機負載重時 controller_manager 回 load_controller 太慢，spawner 等不到（預設 10 秒）就重送，
撞到 already loaded 退出，那個控制器就卡在 unconfigured（2026-11-17 16:39 right_arm_controller 發生過，
右手之後所有移動都 exec_failed / plan_failed）。
改成一個 spawner 依序開全部控制器（同一時間只有一個請求），等待時間拉長。

卡住時手動救：ros2 control set_controller_state right_arm_controller inactive，再 active
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

CM_TIMEOUT = "60"         # 等 controller_manager 起來最多幾秒
CALL_TIMEOUT = "30"       # 每個 load / configure / switch 請求等回覆最多幾秒（預設 10）


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("seven_axis_arm_urdf", package_name="seven_axis_arm_moveit_config").to_moveit_configs()
    controllers = ["joint_state_broadcaster"] + list(
        moveit_config.trajectory_execution.get("moveit_simple_controller_manager", {}).get("controller_names", []))
    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=controllers + ["--controller-manager-timeout", CM_TIMEOUT,
                                     "--service-call-timeout", CALL_TIMEOUT],
            output="screen",
        ),
    ])
