# 啟動指令

只列開啟各個節點和常用工具的指令，參數和原理看各套件的 README。沒特別註明的都在機器人上執行，每個指令開一個終端機。

## 每個新終端機先執行

nstc 已經不靠 ros2_adult，不用再 source 它。`~/.bashrc` 裡如果有自動 source ros2_adult，也不影響（nstc 最後 source，同名套件以 nstc 為準）。

```bash
source /opt/ros/humble/setup.bash
source ~/nstc/install/setup.bash
```

## 系統啟動（nstc.launch.py）

ZED、手臂馬達 driver、motionpackage、雙手、影像處理 4 個節點、rosbridge 9090、web_video_server 8080、手臂 MoveIt + 橋接。
同一時間只能有一個程式開相機和 U2D2，不要再同時開 ros2_adult 的 `camera.launch.py`。

```bash
ros2 launch ~/nstc/launch/nstc.launch.py                 # 全部
ros2 launch ~/nstc/launch/nstc.launch.py hands:=left     # 只接左手（right 同理）
ros2 launch ~/nstc/launch/nstc.launch.py hands:=none     # 不開手
ros2 launch ~/nstc/launch/nstc.launch.py arm:=false      # 不開手臂 MoveIt / 橋接
```

網頁：在桌機用 sftp 掛載 `~/nstc/web/` 直接開 html（IP 自動判斷）。
Motion、ImageProcess、HandControl、HandMotion、HandTouch、RobotViewer。

## 藥罐定位（照順序開）

```bash
# 1. 系統（ZED 相機、馬達、手等），見上一節
ros2 launch ~/nstc/launch/nstc.launch.py

# 2. YOLO 藥罐偵測（b1～b4），發布 /med_detect/bottles
ros2 run med_detect bottle_detect_v2

# 3. 藥罐 3D 定位，發布 /bottle_detections_3d
ros2 launch bottle_localization bottle_localization.launch.py
```

## 查看與調整（上面三個在跑的時候）

```bash
# 即時查看輸出的座標
ros2 run bottle_localization bottle_viewer

# 執行中調整桌高與外參
ros2 run bottle_localization bottle_tuner

# YOLO 有框到、卻沒有輸出座標時，印出被跳過的原因（取代上面第 3 步）
ros2 launch bottle_localization bottle_localization.launch.py log_level:=debug
```

看 YOLO 畫面：在桌機的瀏覽器開 `http://172.17.121.10:8080/stream?topic=/med_detect/image`

## 在桌機用 RViz 看

```bash
# 機器人上：把 TF 和標記轉發到桌機
ros2 run bottle_localization rviz_relay

# 桌機上（ROS_DOMAIN_ID 不要設）
source /opt/ros/humble/setup.bash
rviz2 -d ~/nstc_ws/nstc/src/bottle_localization/rviz/bottle_localization.rviz
```

## 手臂（armpackage）

程式已搬進 `src/armpackage`（套件名稱 `armpackage`），說明還在 `~/arm_ws/README.md`。`~/arm_ws/install/setup.bash` 不要跟 nstc 一起 source，兩邊都有 `seven_axis_arm_urdf`、`seven_axis_arm_moveit_config`。

```bash
# 模擬：MoveIt + rosbridge + 網頁橋接。從 SSH 開的要加 rviz:=false
ros2 launch armpackage sim.launch.py

# 接實機：nstc.launch.py 已經包含手臂（等於下一行），不用另外開；arm:=false 不開手臂
ros2 launch ~/nstc/launch/nstc.launch.py
# 主系統用 ros2_adult 的 camera.launch.py 時：先開它（driver_node、motionpackage、rosbridge 9090），再開這個
ros2 launch armpackage sim.launch.py rviz:=false rosbridge:=false real:=true

# 開關實機跟隨（也可以在網頁「IK 目標」面板按「實機跟隨」）
ros2 topic pub --once /arm_real_enable std_msgs/msg/Bool "{data: true}"
```

網頁：在桌機的瀏覽器用 sftp 掛載打開 `~/nstc/web/RobotViewer.html`。實機每次停穩會記一行到 `~/.ros/arm_settle_log.csv`。

```bash
# 開機自我檢查（只讀）：要在 nstc.launch.py 開之前跑，driver_node 開著會佔住 U2D2
ros2 run armpackage self_check.py
ros2 run armpackage self_check.py config        # 只檢查設定檔，不碰馬達，隨時可以跑

# 順應控制（實驗中）：current_log 要 driver_node 開著；compliance 要 driver_node 關著、馬達扭力關著
ros2 run armpackage current_log.py
ros2 run armpackage compliance.py status
ros2 run armpackage compliance.py apply
ros2 run armpackage compliance.py restore

# 馬達方向校正：driver_node 開著、實機跟隨關著
ros2 run armpackage jog_motor.py check right    # 左手：check left

# 改了 URDF、STL 或網頁樣板（~/nstc/web/RobotViewer.template.html）之後，重新產生網頁
ros2 run armpackage build_viewer.py
```

## 子三任務管理（sp3_task_manager）

接主控的取放任務 `/manip/pick_place`（型別 `interfaces/action/ExecuteTaskPlan`），說明在 `src/sp3_task_manager/README.md`。
主控模擬器在 `src/drivers/ros2_task_plan_demo`。

### 不接硬體測流程（dry_run）

```bash
# 終端機 1：任務管理，各模組模擬成功
ros2 launch sp3_task_manager task_manager.launch.py dry_run:=true

# 終端機 2：主控模擬器（要有螢幕），按「送出請求」
ros2 run task_plan_demo client_gui
```

想看失敗時的回覆：改 `src/sp3_task_manager/config/task_manager.yaml` 的 `dry_run.fail`
（`detect` 找不到藥瓶、`arm` 手臂到不了、`grip` 沒夾到、`hold` 抬起後沒夾住；`''` 全部成功），重開終端機 1。

### 接實機

```bash
# 1. 系統（含手臂）
ros2 launch ~/nstc/launch/nstc.launch.py
# 2. YOLO 與藥罐定位
ros2 run med_detect bottle_detect_v2
ros2 launch bottle_localization bottle_localization.launch.py
# 3. 網頁 IK 面板打開「實機跟隨」（或下一行）
ros2 topic pub --once /arm_real_enable std_msgs/msg/Bool "{data: true}"
# 4. 任務管理
ros2 launch sp3_task_manager task_manager.launch.py
```

上實機前先填 `task_manager.yaml` 標【待量測】的放置點 `place_targets` 與工作範圍 `workspace`。

### 沒有螢幕時用指令送任務、看狀態

```bash
# 送任務（Ctrl+C 會取消）。keys 可用：task_id、bottle_class（必填 b1～b4）、arm（auto/left/right）、place_target、timeout_s
ros2 action send_goal --feedback /manip/pick_place interfaces/action/ExecuteTaskPlan \
  "{subtask: {parameter_keys: [bottle_class, arm, place_target], parameter_values: [b1, auto, tray_1]}}"

# 狀態（IDLE / BUSY / ERROR / ESTOP），1 Hz
ros2 topic echo /manip/status

# 急停／解除
ros2 topic pub --once /estop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /estop std_msgs/msg/Bool "{data: false}"
```

```bash
# 單元測試（不需要 ROS）
cd ~/nstc/src/sp3_task_manager && python3 -m pytest test -q
```

## 其他節點

```bash
# 藥物尺寸辨識
ros2 run med_detect med_detect

# 舊版 YOLO（不分種類，現在用 bottle_detect_v2）
ros2 run med_detect bottle_detect
```

## 外參與桌高校正

工具在 `src/bottle_localization/tools/`，沒有安裝成 ROS 節點，所以直接用 `python3` 執行。詳細步驟見 `src/bottle_localization/README.md` 的「準確度校正流程」。

```bash
cd ~/nstc/src/bottle_localization/tools

# 量相機相對桌面的 roll、pitch 和高度（換桌子時用）。ZED 要開著，畫面下方中間那塊只能有桌面
python3 table_plane.py

# 藥罐放在量好的位置，算出誤差和外參的修正量。藥罐定位要在跑
python3 position_check.py --label b2

# 兩瓶距離檢查：兩罐中間放一把尺，只檢查定位本身準不準。藥罐定位要在跑
python3 pair_check.py --labels b2 b3 --length <尺的全長 cm>
```

重新量外參時才用（例如換手臂、相機位置改變）：

```bash
# ZED IMU 的 roll、pitch，平均 5 秒（只當參考，角度以 table_plane.py 為準）
python3 zed_imu_average.py

# 從 ZED 的 3D 模型讀出鏡頭玻璃到螺絲孔的距離 a
python3 zed_stl_front.py

# 由量到的值算出 camera_extrinsics.yaml 要填的數字（任何電腦都能跑）
python3 extrinsics_calc.py --dx <cm> --dy <cm> --dz <cm> --pitch <度> --roll <度> --a <m>
```

## FoundationPose 試驗（實驗用，不影響上面的系統）

桌機上的指令都在 `~/nstc_ws/nstc/foundationpose_trial` 執行。第一次在新電腦上用，要先照該資料夾 README.md 的「建立工具用的 Python 環境」和「階段 2」建好環境。

### 即時看效果

```bash
# 第一次要先把工具傳到機器人（在桌機上）
ssh iclab@172.17.121.10 mkdir -p fp_bags
scp tools/fp_stream_server.py tools/fp_record.py iclab@172.17.121.10:~/fp_bags/

# 機器人上（藥罐定位的三個要先開）
python3 ~/fp_bags/fp_stream_server.py

# 桌機上
tools/fp_run.sh python tools/fp_live_view.py

# 沒有機器人時，用錄好的資料重播（兩個都在桌機上）
tools/fp_run.sh python tools/fp_stream_server.py --replay data/robot_inverted_01
tools/fp_run.sh python tools/fp_live_view.py --server 127.0.0.1
```

### 錄資料與離線分析

```bash
# 1. 錄 bag（機器人上，藥罐定位的三個要先開）
#    名稱要含 upright、inverted 或 tilted，彙總時用來判斷瓶蓋應該朝哪邊
cd ~/fp_bags && python3 fp_record.py robot_upright_02 --seconds 20

# 2. 傳回桌機
scp -r iclab@172.17.121.10:~/fp_bags/robot_upright_02 bags/

# 3. 匯出成 FoundationPose 的格式
env -u PYTHONPATH .venv/bin/python tools/export_rosbag.py bags/robot_upright_02 --out data

# 4. 跑 FoundationPose，每個類別一次，輸出姿態和疊圖影片 overlay.mp4
for l in b1 b2 b3 b4; do
    tools/fp_run.sh python tools/run_fp_scene.py --scene data/robot_upright_02/$l \
        --mesh meshes/shaped/$l.obj --out outputs/robot_upright_02_${l}_shaped
done

# 5. 彙總傾角、中心高度、瓶蓋方向
tools/fp_run.sh python tools/summarize_runs.py robot_upright_02

# 6. 和現有定位（/bottle_detections_3d）比較位置、頻率、延遲
env -u PYTHONPATH .venv/bin/python tools/compare_localizer.py robot_upright_02
```

### 其他

```bash
# 改了 config/bottle_shapes.yaml（藥罐尺寸）之後，重新產生 mesh
env -u PYTHONPATH .venv/bin/python tools/make_shaped_meshes.py

# 跑測試
env -u PYTHONPATH .venv/bin/python -m pytest tests/ -q
```

桌機的 `~/.bashrc` 會載入 ROS，它的 `PYTHONPATH` 會讓這些工具出錯，所以指令前要加 `env -u PYTHONPATH`；`tools/fp_run.sh` 已經處理好了。
`tools/` 裡沒列在這裡的是開發和驗證用的工具，用法見 README.md。

## 改了程式之後重新建置

```bash
# nstc 自己有 nstc_msgs，不用 source ros2_adult
source /opt/ros/humble/setup.bash
cd ~/nstc
colcon build --symlink-install
source install/setup.bash

# 只建改到的套件，例如任務管理
colcon build --symlink-install --packages-select sp3_task_manager
```
