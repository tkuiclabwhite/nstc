"""nstc 系統 launch：ZED、手臂馬達驅動、雙手、影像處理、網頁通訊、手臂 MoveIt + 實機橋接。

由 ros2_adult 的 usb_cam/launch/camera.launch.py 精簡而來（2026-11-12），
拿掉步態、IMU、USB 相機、GPIO 開關、熱點、拍照與 http_server。
之後子三的任務管理套件建好後，把這支搬進該套件的 launch/。

用法（直接用檔案路徑執行）：
    ros2 launch ~/nstc/launch/nstc.launch.py
    ros2 launch ~/nstc/launch/nstc.launch.py hands:=left     # 只接左手
    ros2 launch ~/nstc/launch/nstc.launch.py hands:=none     # 不開手
    ros2 launch ~/nstc/launch/nstc.launch.py arm:=false      # 不開手臂 MoveIt / 橋接

手臂的實機跟隨開起來是關的，要在網頁 IK 面板按「實機跟隨」或送 /arm_real_enable 才會動。
self_check.py 要在開這支之前跑（driver_node 開著會佔住 U2D2）。

參數檔一律在 ~/nstc/config/（strategy.ini 指向 nstc/Parameter）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# nstc 只有一組參數：~/nstc/config/nstc/Parameter
LOCATION = 'nstc'


def generate_launch_description():
    hands_arg = DeclareLaunchArgument(
        'hands', default_value='both',
        description="要啟動哪幾隻手：'both'（預設）/ 'left' / 'right' / 'none'")
    hands = LaunchConfiguration('hands')
    arm_arg = DeclareLaunchArgument(
        'arm', default_value='true',
        description="'false' = 不開手臂 MoveIt、網頁橋接、實機橋接")

    # 1) ZED X Mini
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')),
        # publish_tf / publish_map_tf 關掉：不發 odom -> zed_camera_link、map -> odom。
        # zed_camera_link 由 head_tf 的動態 TF 接在 base_link 底下（world -> base_link -> zed_camera_link），
        # ZED 再發 odom 的話 zed_camera_link 會有兩個 parent。nstc 沒有東西用 odom/map 的 TF（2026-11-20）。
        launch_arguments={'camera_model': 'zedxm',
                          'publish_tf': 'false',
                          'publish_map_tf': 'false'}.items(),
    )

    # 2) 手臂馬達驅動（Dynamixel，原樣沿用 ros2_adult）與非 MoveIt 的手臂控制
    driver_node = Node(
        package='motor_control', executable='driver_node', name='dynamixel_driver',
        output='screen', parameters=[{'baudrate': 1000000}],
    )
    motion_node = Node(
        package='motionpackage', executable='motionpackage', name='motion_strategy',
        output='screen', parameters=[{'location': LOCATION}],
    )

    # 3) LinkerHand O6 雙手。web 固定 false：本系統不起 http_server
    hands_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('handpackage'), 'launch', 'hands.launch.py')),
        launch_arguments={'hands': hands, 'location': LOCATION, 'web': 'false'}.items(),
        condition=IfCondition(PythonExpression(["'", hands, "' in ('both', 'left', 'right')"])),
    )

    # 4) 影像處理（ImageProcessInterface 四個分頁各對應一個節點）
    image_node = Node(
        package='imageprocess', executable='image', name='image_node',
        output='screen', parameters=[{'camera_source': 'zed'}],
    )
    depth_process_node = Node(
        package='imageprocess', executable='depth_process_node', name='depth_process_node',
        output='screen',
    )
    overlap_node = Node(
        package='imageprocess', executable='overlap_node', name='overlap_node',
        output='screen',
    )
    camera_param_bridge_node = Node(
        package='imageprocess', executable='camera_param_bridge_node',
        name='camera_param_bridge_node', output='screen',
    )

    # 5) 網頁通訊：影像串流 8080、網頁按鈕 9090
    web_video = Node(package='web_video_server', executable='web_video_server',
                     name='web_video_server')
    rosbridge_node = Node(
        package='rosbridge_server', executable='rosbridge_websocket', name='rosbridge_websocket',
        parameters=[{'port': 9090, 'address': '0.0.0.0'}], output='screen',
    )

    # 6) 手臂：MoveIt + moveit_bridge + real_arm_bridge + 打開 RobotViewer.html
    #    RViz 不開；rosbridge 用上面 5) 開的 9090；real 開實機橋接（開起來是關的）
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('armpackage'), 'launch', 'sim.launch.py')),
        launch_arguments={'rviz': 'false', 'rosbridge': 'false', 'real': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('arm')),
    )

    return LaunchDescription([
        hands_arg,
        arm_arg,
        GroupAction(actions=[
            zed_launch, driver_node, motion_node, hands_launch,
            image_node, depth_process_node, overlap_node, camera_param_bridge_node,
            web_video, rosbridge_node, arm_launch,
        ]),
    ])
