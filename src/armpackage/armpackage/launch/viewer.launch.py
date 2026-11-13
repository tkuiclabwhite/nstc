"""
網頁操作介面：rosbridge + MoveIt 橋接（moveit_bridge.py）+ 打開 RobotViewer.html
MoveIt 另外開的時候用這個；要一次全開請用 sim.launch.py

    ros2 launch armpackage viewer.launch.py
    參數：browser:=false（不自動開網頁；從 SSH 開的本來就不開）　port:=9090（rosbridge 的 port）
          rosbridge:=false（不開 rosbridge：跟 camera.launch.py 一起開時用它的）
          real:=true（開實機橋接 real_arm_bridge.py，開起來是關的，要另外送 /arm_real_enable）
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WEB_DIR = os.path.join(os.path.expanduser("~"), "nstc", "web")     # 同 handpackage/launch/hands.launch.py


def launch_setup(context):
    port = LaunchConfiguration("port").perform(context)
    html = os.path.join(WEB_DIR, "RobotViewer.html")
    actions = [Node(package="armpackage", executable="moveit_bridge.py", name="moveit_bridge", output="screen")]
    if LaunchConfiguration("rosbridge").perform(context) == "true":
        actions.append(Node(package="rosbridge_server", executable="rosbridge_websocket", name="rosbridge_websocket",
                            parameters=[{"port": int(port)}], output="log"))
    if LaunchConfiguration("real").perform(context) == "true":
        actions.append(Node(package="armpackage", executable="real_arm_bridge.py", name="real_arm_bridge",
                            output="screen"))
    ssh = os.environ.get("SSH_CONNECTION", "").split()     # 「client_ip client_port server_ip server_port」
    if LaunchConfiguration("browser").perform(context) == "true" and ssh:
        # 從自己的電腦 SSH 進來開的：網頁要開在自己電腦的瀏覽器（沒帶 ?ws= 會從 sftp 掛載的路徑抓 IP），這裡不開
        hint = "" if port == "9090" else f"，網址後面加 ?ws={ssh[2]}:{port}"
        actions.append(LogInfo(msg=f"從 SSH 開的，不自動開網頁。請在自己電腦的瀏覽器打開 AGX 上的 "
                                   f"{os.path.realpath(html)}（例如透過 sftp 掛載）{hint}"))
    elif LaunchConfiguration("browser").perform(context) == "true":
        # 等 rosbridge 開好再開網頁；網址帶 ?ws= 讓網頁自動連線（9090 只寫 localhost，網頁右上角的選單才對得上）
        ws = "localhost" if port == "9090" else f"localhost:{port}"
        url = f"file://{os.path.realpath(html)}?ws={ws}"
        actions.append(TimerAction(period=3.0, actions=[ExecuteProcess(cmd=["xdg-open", url], output="log")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="9090", description="rosbridge 的 port"),
        DeclareLaunchArgument("browser", default_value="true", description="false = 不自動開網頁"),
        DeclareLaunchArgument("rosbridge", default_value="true", description="false = 不開 rosbridge（用 camera.launch.py 開的 9090）"),
        DeclareLaunchArgument("real", default_value="false", description="true = 開實機橋接 real_arm_bridge.py"),
        OpaqueFunction(function=launch_setup),
    ])
