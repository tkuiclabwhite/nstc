"""雙手 LinkerHand O6：兩個 SDK 節點 + handpackage。

這支可以單獨跑，不用起整台機器人。web:=true 會順便把 rosbridge(9090) 和
http_server(9999) 一起起來，所以一個指令就能用網頁操作：

    # 有手：一個指令搞定
    ros2 launch handpackage hands.launch.py web:=true

    # 沒手：fake:=true 用假 SDK 取代兩個真的 SDK 節點，一樣一個指令
    ros2 launch handpackage hands.launch.py web:=true fake:=true

    # 只接了一隻手：hands:=left 或 hands:=right，另一隻就不會被啟動
    ros2 launch handpackage hands.launch.py web:=true hands:=left

機器人上不要直接用這支，camera.launch.py 會把它 include 進去（並強制 web:=false，
因為 rosbridge 和 http_server 那邊已經起過了）：

    ros2 launch usb_cam camera.launch.py hands:=both
    ros2 launch usb_cam camera.launch.py hands:=left left_can:=can0

這支的每一個 argument 都會透出到 camera.launch.py 的指令列，所以 CAN 介面、
壓感、型號要覆蓋都直接在那邊加就好。

CAN 介面做成 launch argument 是因為編號會跑掉：172.17.121.11 上 can0/can1 是
AGX 內建 CAN（沒接東西），can2/can3 才是 PEAK USB-CAN。換機器要先用
`ip -br link show type can` 看清楚，再用 left_can:= / right_can:= 覆蓋。
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

HUROCUP_DIR = os.path.join(os.path.expanduser('~'), 'nstc', 'web')
# fake_hands.py 在原始碼樹裡（不是安裝過的 executable），所以要指到 src/
PKG_SRC_DIR = os.path.join(os.path.expanduser('~'), 'nstc', 'src', 'handpackage')


def _iface_is_up(iface):
    """用 IFF_UP 旗標判斷介面是否啟用。

    不看 operstate：CAN 介面啟用後 operstate 常常還是 "unknown"，
    但 /sys/class/net/<iface>/flags 的 bit0（IFF_UP）是可靠的。
    回傳 True / False / None（介面不存在）。
    """
    path = f'/sys/class/net/{iface}/flags'
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            return bool(int(f.read().strip(), 16) & 0x1)
    except Exception:
        return None


def _wanted(context):
    """這次要開哪幾隻手。"""
    w = LaunchConfiguration('hands').perform(context).strip().lower()
    return ['left', 'right'] if w == 'both' else ([w] if w in ('left', 'right') else [])


def _report_can(context, *_, **__):
    """啟動時先回報 CAN 狀態，並排一個 10 秒後的複檢。

    複檢必須在這裡排、而且只能帶「已經解析好的字串」進去。
    理由是 launch configuration 有 scope：camera.launch.py 用 GroupAction 包住
    全部動作，裡面再 IncludeLaunchDescription 這一支，兩層都會在 include 處理完
    之後把 scope pop 掉。TimerAction 的動作是 10 秒後才執行的，那時候
    LaunchConfiguration('fake') 已經查不到了，會丟
    「launch configuration 'fake' does not exist」—— 而 OpaqueFunction 丟例外
    會讓整個 LaunchService 關機，現象是「沒按 Ctrl+C 但整台機器人自己關掉」。
    單獨跑這一支沒有那兩層 scope，所以不會發生，很容易漏掉。
    """
    fake = LaunchConfiguration('fake').perform(context).lower() in ('true', '1')
    if fake:
        print('[hands] fake:=true，用假 SDK，不檢查 CAN')
        return []
    want = _wanted(context)
    if not want:
        print(f'[hands] hands:={LaunchConfiguration("hands").perform(context)}'
              f' —— 不啟動任何手（只起 handpackage）')
    elif len(want) < 2:
        print(f'[hands] hands:={LaunchConfiguration("hands").perform(context)}'
              f' —— 只啟動 {want[0]}，另一隻不開')
    # 這裡就把 side -> 介面名稱 解析完，後面複檢只用這份純字典
    ifaces = {side: LaunchConfiguration(f'{side}_can').perform(context)
              for side in want}
    for side, iface in ifaces.items():
        up = _iface_is_up(iface)
        if up is None:
            print(f'[hands] {side:5s} CAN {iface}：找不到這個介面！'
                  f' 先用 `ip -br link show type can` 確認編號')
        elif up:
            print(f'[hands] {side:5s} CAN {iface}：已啟用')
        else:
            print(f'[hands] {side:5s} CAN {iface}：目前是 DOWN，等 SDK 自己開')
    return [TimerAction(period=10.0, actions=[
        OpaqueFunction(function=lambda ctx, *a, **k: _recheck_can(ifaces))])]


def _recheck_can(ifaces):
    """啟動後再檢查一次。只吃解析好的 {side: iface}，不碰 LaunchConfiguration。

    SDK 的 LinkerHand/utils/open_can.py 是用 setting.yaml 裡的 PASSWORD 去
    `sudo -S ip link set ... up`，失敗是 `except: pass` 吞掉不報錯 ——
    現象會是「手完全不動，但畫面上一個錯誤訊息都沒有」。
    密碼不對（例如換了機器）就是這樣，所以這裡一定要吼出來。
    """
    for side, iface in ifaces.items():
        if _iface_is_up(iface) is not True:
            print('=' * 72)
            print(f'[hands] 警告：{side} 手的 CAN 介面 {iface} 到現在還沒啟用。')
            print('  手會完全不動，而且 SDK 不會印任何錯誤（它把例外吞掉了）。')
            print('  最常見原因：setting.yaml 的 PASSWORD 不是這台機器的 sudo 密碼。')
            print('    src/drivers/linkerhand-ros2-sdk/linker_hand_ros2_sdk/'
                  'linker_hand_ros2_sdk/LinkerHand/config/setting.yaml')
            print(f'  也可以先手動開：sudo ip link set {iface} up type can bitrate 1000000')
            print('=' * 72)
    return []


def _hand_node(side):
    """一個 SDK 節點 = 一隻手。

    remap /cb_hand_setting_cmd 是必要的，不是潔癖：上游 linker_hand.py 的
    hand_setting_cb() 沒有比對 params.hand_type 跟節點自己的 hand_type
    （兩個分支都是 `hand = self.api`），所以用全域 topic 的話，對一隻手下的
    設定指令兩隻手都會執行。詳見 src/drivers/linkerhand-ros2-sdk/UPSTREAM.md 地雷 3。
    """
    return Node(
        package='linker_hand_ros2_sdk',
        executable='linker_hand_sdk',
        name=f'linker_hand_sdk_{side}',
        output='screen',
        parameters=[{
            'hand_type': side,
            'hand_joint': LaunchConfiguration('hand_joint'),
            'is_touch': LaunchConfiguration(f'{side}_touch'),
            'can': LaunchConfiguration(f'{side}_can'),
            'modbus': LaunchConfiguration('modbus'),
        }],
        remappings=[('/cb_hand_setting_cmd', f'/cb_{side}_hand_setting_cmd')],
        # 兩個條件要一起看：fake:=true 時不起真 SDK（會跟假 SDK 互相蓋），
        # hands:=left/right 時只起那一隻（另一隻沒接的話節點會讀不到版本然後退出）
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('fake'), "'.lower() not in ('true','1')",
            " and '", LaunchConfiguration('hands'), "' in ('both','", side, "')",
        ])),
    )


def generate_launch_description():
    args = [
        DeclareLaunchArgument('left_can', default_value='can2',
                              description='左手的 CAN 介面（左手 CAN ID 0x28）'),
        DeclareLaunchArgument('right_can', default_value='can3',
                              description='右手的 CAN 介面（右手 CAN ID 0x27）'),
        DeclareLaunchArgument('left_touch', default_value='true',
                              description='左手是否有壓力感測器'),
        DeclareLaunchArgument('right_touch', default_value='true',
                              description='右手是否有壓力感測器'),
        DeclareLaunchArgument('hand_joint', default_value='O6',
                              description='手型號，大寫。O6/L6/L7/L10/L20/G20/L21/L25'),
        DeclareLaunchArgument('modbus', default_value='None',
                              description='走 RS485 時填 /dev/ttyUSB*，走 CAN 就是 None'),
        DeclareLaunchArgument('location', default_value='ar',
                              description='預設策略；實際值會被 strategy.ini 覆蓋'),
        DeclareLaunchArgument('hands', default_value='both',
                              description='要啟動哪幾隻手：both / left / right。'
                                          '只接一隻手時要指定，否則沒接的那隻會'
                                          '讀不到版本然後退出'),
        DeclareLaunchArgument('fake', default_value='false',
                              description='用 test/fake_hands.py 取代兩個真的 SDK 節點。'
                                          '沒有硬體時拿來看網頁、驗流程'),
        DeclareLaunchArgument('web', default_value='false',
                              description='順便起 rosbridge + http_server。'
                                          '單獨測手時設 true；機器人上設 false'
                                          '（camera.launch.py 已經有了）'),
    ]

    handpackage_node = Node(
        package='handpackage',
        executable='handpackage',
        name='handpackage',
        output='screen',
        parameters=[{'location': LaunchConfiguration('location')}],
    )

    # 只有 web:=true 才起。Port 9090 / 9999 與 camera.launch.py 完全一致，
    # 這樣網頁的 js/config.js 不用為了「單獨測手」另外改位址。
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090, 'address': '0.0.0.0'}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('web')),
    )
    http_server = ExecuteProcess(
        cmd=['python3', os.path.join(HUROCUP_DIR, 'http_server.py')],
        cwd=HUROCUP_DIR,
        output='screen',
        condition=IfCondition(LaunchConfiguration('web')),
    )

    # 假 SDK。它在 test/ 下面、不是安裝過的 entry point，所以用 ExecuteProcess
    # 直接跑原始碼路徑 —— 與 camera.launch.py 跑 http_server.py 同一個做法
    fake_hands = ExecuteProcess(
        cmd=['python3', os.path.join(PKG_SRC_DIR, 'test', 'fake_hands.py')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('fake')),
    )

    ld = LaunchDescription()
    for a in args:
        ld.add_action(a)
    # _report_can 會順便回傳那個 10 秒後的複檢 TimerAction（見它的 docstring）
    ld.add_action(OpaqueFunction(function=_report_can))
    ld.add_action(GroupAction(actions=[
        _hand_node('left'),
        _hand_node('right'),
        fake_hands,
        handpackage_node,
        rosbridge_node,
        http_server,
    ]))
    return ld
