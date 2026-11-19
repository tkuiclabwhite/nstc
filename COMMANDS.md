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

看 YOLO 畫面：在桌機的瀏覽器開 `http://172.17.121.11:8080/stream?topic=/med_detect/image`

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
ros2 topic pub --once -w 1 /arm_real_enable std_msgs/msg/Bool "{data: true}"
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

只模擬其中幾段、其他接實機：寫在 `task_manager.yaml` 的 `dry_run.modules`（例如 `[vision]`，`[]` = 全部接實機），
或 launch 時用 `dry_run:=` 臨時指定（逗號分隔，以 launch 為準）：

```bash
ros2 launch sp3_task_manager task_manager.launch.py dry_run:=vision       # 假藥瓶（dry_run.bottles：b1 右、b2 左，另兩瓶當障礙物），手臂和手真的動
ros2 launch sp3_task_manager task_manager.launch.py dry_run:=vision,arm   # 只有手真的開合、夾持
```

真的那幾段要的節點一樣要開（見下面「接實機」）。只模擬 vision 時手臂會真的移到假藥瓶的位置，先確定那裡是空的。

### 接實機

```bash
# 1. 系統（含手臂）
ros2 launch ~/nstc/launch/nstc.launch.py
# 2. YOLO 與藥罐定位
ros2 run med_detect bottle_detect_v2
ros2 launch bottle_localization bottle_localization.launch.py
# 3. 打開實機跟隨（沒開的話任務會回 MAN-E-007）
ros2 topic pub --once -w 1 /arm_real_enable std_msgs/msg/Bool "{data: true}"
# 4. 任務管理
ros2 launch sp3_task_manager task_manager.launch.py
```

- 上實機前先填 `task_manager.yaml` 標【待量測】的值（桌子 `table`、放置點 `place_targets`、工作範圍 `workspace`）。
- 任務管理會把桌子和其他藥瓶發到 `/arm_ik_obstacles`（來源 sp3）給 MoveIt 避開，跟網頁加的物體分開、不會互相蓋掉。
  夾住的藥瓶會掛到手上（`/arm_attach`）。armpackage 要用 2026-11-17 之後的版本（.11 上的），舊版不認這些格式。
- 急停或任務中斷時，任務管理會取消 MoveIt 軌跡（`/arm_stop`）並關掉實機跟隨；再跑任務前要重新執行第 3 步。
- 只測 MoveIt、不動實機：`ros2 launch sp3_task_manager task_manager.launch.py real_arm:=false`
  （不看實機跟隨、也不看馬達 driver，馬達可以不上電；可以再加 `dry_run:=hand` 讓手也模擬）。
- 任務第 0 步、第 6 步都回 **ready pose**：`task_manager.yaml` 的 `ready_pose`（桌面座標 左 (−50, 15, 20)、右 (50, 15, 20)，
  掌心朝下）。`via: path`（2026-11-19）：照**錄好的路線**兩隻手同時走（速度 `ready_pose.speed` 50，走的時候實機馬達速度也切到 50、
  走完切回 `grasp.arm_speed`），回站姿倒過來走；
  路線存在 `ready_pose.path_file`（`src/sp3_task_manager/config/ready_path.yaml`）。
  - 錄（網頁按鈕 2026-11-19 確認後拿掉了）：兩隻手在站姿時
    `ros2 topic pub --once -w 1 /manip/test_cmd std_msgs/msg/String "{data: '{\"cmd\": \"record_ready\"}'}"`：
    用原本的方法（`/arm_move` 左手再右手，MoveIt 規劃）舉到 ready，途中的路線存起來，蓋掉舊的。
    實機跟隨沒開時目標自己往上補 0.17×前（同 moveit_bridge 的高度補償），只看模擬時 ready 會比設定的高約 4.8 cm。
  - `ready_pose.mirror: right`：兩隻手都用右手的路線（URDF 左右鏡像，同一組關節角 = 左右相反的姿勢），錄的時候只舉右手。
    2026-11-19 左手自己錄的路線越過桌緣太低（手最低只比桌面高約 4 cm），實機拇指擦到桌角；右手的先舉到掌心高 113 才往前。
  - `ready_pose.adjust_cm`：只改某隻手的路線，不用重錄（掌心平移、姿態不變，逐點逆解）。`edge` 只在經過桌角那段、`end` 一直到 ready。
    現在左手 `edge: [-2, 0, 1]`（往後 2、往上 1）、`end: [0, 0, 1]`：實機左手比右手低約 1 cm，經過桌角時拇指擦到。
    套不上（逆解不到等）時不照路線走，結果會寫原因。
  - 還沒錄、設定檔 ready 的位置改了（要重錄）、路線會撞到（走之前每 1.5° 問 MoveIt）時，自動改回一隻一隻移（`via: move`），
    回站姿改回 moveit_bridge 的 `/arm_stand`；網頁測試結果會寫用了哪一種、為什麼。
  - 手不在站姿也不在 ready（例如放完藥瓶）：先每顆馬達直接轉到 ready，回站姿再接倒過來的路線。
  - 這兩段是任務管理直接交給 MoveIt（`/execute_trajectory`），不經 moveit_bridge；手臂端做好 `/arm_ready` 後可改 `via: arm_ready`。
  - 走之前的碰撞檢查：一開始（路程前 5%）就碰到的物體（例如站姿時左手拇指在桌子的加厚裡）先不算，分開後就算；路程前 20% 還沒分開就不走。
- 放置（2026-11-19 改）：`place_targets` 的「高」是放置面（tray 底面）離地多高，掌心高度依手上藥瓶的高度算，
  放開時瓶底在放置面上方 `grasp.place_gap_cm`（1 cm，實機）。放下那一步桌子上方加厚暫時改成 `grasp.place_table_margin_cm`（0.5），
  抬起後改回（實機跟隨開著時 moveit_bridge 也檢查實機的位置）。
- 某隻手另外補高度：`grasp.arm_z_offset_cm`（2026-11-19 尺量後兩隻手都是 0；ready 路線另外用 `ready_pose.adjust_cm`）。只看模擬時任務管理每一步移動都自己補 0.17×前（實機跟隨開著時 moveit_bridge 會補），模擬的位置、可及範圍跟開實機跟隨時一樣。

```bash
# 看任務管理送給 MoveIt 的障礙物、手臂每一步的結果（cmd / arm / reason）
ros2 topic echo /arm_ik_obstacles
ros2 topic echo /arm_ik_status
```

### 可及範圍與避障測試（reach_test）

不經任務管理，直接對 moveit_bridge 下移動指令看結果。要先開 moveit_bridge（第 1 步的 nstc.launch.py）；除了 scan，**任務管理不要開**。
先在實機跟隨**關著**時跑（只動模擬，在 RViz／網頁看）；實機跟隨開著時要加 `--real` 才會執行。

scan（量可及範圍，2026-11-19 改）：**任務管理可以開著**，開著時先請它照錄好的路線回 ready，沒開就用 /arm_move 把這隻手移到 ready
（`--start stand` 改回從站姿出發的舊做法）。每一列先回 ready 再往前試；`plan_failed`（到得了、這次沒找到路）先回 ready 再重試，
預設 2 次（`--retry`）。結果分開標：✓ 到了（+ = 重試幾次才到）、✗ 到不了（IK／關節極限）、C 目標會撞到、P 重試後還是找不到路。
最後印出「最近到了／最遠到了／IK 最遠」和可以直接貼進 `task_manager.yaml` 的 `workspace`；每一點存在 `~/reach_test_out/<時間>_scan_<手>/scan.csv`。
最後一點都還到得了會提醒加大 `--beyond`。
只舉測的那隻手（另一隻留在站姿），**測完會回站姿**：手舉著不動馬達會過熱（2026-11-19 右手過熱）。動實機要加 `--real`。

```bash
# 實際極限：每個左右位置從桌緣往前每 2 cm 試一次
ros2 run sp3_task_manager reach_test scan                       # 右手、從上方、掌心離桌 10 cm（b1 頂上 1 cm）
ros2 run sp3_task_manager reach_test scan --arm left
ros2 run sp3_task_manager reach_test scan --up 9                # 矮的藥瓶（b2、b4 約 7.8 cm 高）
ros2 run sp3_task_manager reach_test scan --dir front --fingers 90 --up 6   # 從前方（目前不用）

# 避障（從上方）：沒障礙、旁邊 15 cm、旁邊 7 cm 緊貼、前面加另一側、上方有板子、目標在桌面下，看 PASS / FAIL
ros2 run sp3_task_manager reach_test obstacles --pause          # 每一項停下來讓你看 RViz
ros2 run sp3_task_manager reach_test obstacles --arm left --front 15

# 確定模擬沒問題，再開實機跟隨跑實機（手臂會真的動，旁邊要有人顧急停）
ros2 run sp3_task_manager reach_test point 18.9 12 10 --real              # 移到一點停住，量掌心，Enter 回站姿
ros2 run sp3_task_manager reach_test point 0 10 10 30 20 10 --real       # 好幾點（每三個數一點：右 前 上）
ros2 run sp3_task_manager reach_test obstacles --real --pause --cases 1 2 4
```

```bash
# 影像對手（要先開 bottle_detect_v2、bottle_localization；任務管理不用開）：讀藥瓶位置，掌心停在瓶頂正上方 10 cm
ros2 run sp3_task_manager reach_test bottle --bottle b1 --arm left                     # 先只動模擬
ros2 run sp3_task_manager reach_test bottle --bottle b1 --arm left --real --speed 10   # 實機：量掌心有沒有在瓶子正上方
```
會先印影像算的瓶子位置（桌面座標），跟捲尺量的比 = 影像準不準；掌心對不對準瓶子 = 影像＋手臂一起準不準。

實機時 `obstacles --pause` 會先印出實體物品要擺在哪（桌面座標），擺好按 Enter 手臂才動，動完再停一次讓你量。
掌心點 = 手掌面上、左右置中、離手掌底部（接手腕那端）7 cm；量「右」離機器人中線、「前」離桌緣、「上」離桌面。

測試加的物體用場景來源 `reach_test`，測完會自動清掉；中途 Ctrl+C 也會清。

**不用 RViz 看結果**：`obstacles` 每一項都存一張圖（俯視、正視、側視：桌子、障礙物、目標掌心、手臂外形），
並印出「手離物體最近幾 cm」（含障礙物的餘裕，負的 = 插進去）。圖在 `~/reach_test_out/<時間>_obstacles_<手>/case<N>.png`，
輸出會印路徑。看圖：在 AGX 用 `eog <路徑>`，或拿回桌機 `scp iclab@172.17.121.11:<路徑> .`。

### 網頁送任務（暫代總計畫主控）

上方控制列：急停、解除急停、**Stand**（手放下）、**Ready pose**（兩隻手移到設定檔 `ready_pose` 的位置）。
Stand、Ready 都會先把桌子、小桌子放進場景再動。錄好路線後兩隻手一起照錄好的路線走（每次一樣）；沒錄時是 MoveIt 一般規劃
（左手再右手、每次路線不一樣），實機要放慢。
右上「測試」：不跑完整流程，**由任務管理執行**（要開著），帶同樣的場景（桌子、小桌子、其他藥瓶）與實機跟隨檢查：
移到座標（桌面 cm 或 base_link m）、移到影像看到的某瓶瓶頂上方。失敗時會把手臂端的原因翻成白話
（IK 到不了／找不到路／目標會撞到），結果也會寫「可及範圍表：範圍內／外」，順便驗證設定檔的可及範圍。
控制列「速度」（Dynamixel 單位 1～50，實機最多 50）：改了按 Enter 或點別處就送出，之後的任務、測試、Stand、Ready 都用它，
旁邊「（目前 N）」是任務管理現在用的值。重開任務管理會回到設定檔 `grasp.arm_speed`（30）。
RobotViewer IK 面板或 reach_test 改的速度，會在下一個任務／測試開始時被這個值蓋回。
「座標」區：要抓的藥瓶中心、最後的掌心目標、左右手掌心（模擬、實機），各列 base_link 與桌面座標。

`web/SP3TaskInterface.html`（導覽列 SP3Task）：送任務（task_id 空白自動產生 T-年月日-時分、timeout_s 預設 150）、看各階段進度與結果、
`/manip/status`、取消、急停／解除急停。要先開 nstc.launch（rosbridge 9090）與任務管理。
任務被拒絕（急停中、正在執行別的任務）時 rosbridge 不會回任何東西，網頁等 5 秒沒 feedback 就顯示「沒有回應」。
別的主控（client_gui）送的任務也看得到進度與結果（看 `/manip/status` 的 feedback、last_message），但不能從網頁取消。

RobotViewer 會把別的來源（sp3、reach_test）發到 `/arm_ik_obstacles` 的障礙物畫成半透明青色（只看，不能拖）。
任務管理每秒重發一次自己的障礙物，網頁晚開也看得到；reach_test 只在測試中發。
要抓的藥瓶畫成亮綠色（`/manip/target`，辨識到時出現、任務結束 10 秒後消失）；手臂模擬（dry_run 有 arm）時只有它，沒有障礙物。改了樣板要重新產生：
`ros2 run armpackage build_viewer.py`。

### 沒有螢幕時用指令送任務、看狀態

```bash
# 送任務（Ctrl+C 會取消）。keys 可用：task_id、bottle_class（必填 b1～b4）、arm（auto/left/right）、place_target、timeout_s
ros2 action send_goal --feedback /manip/pick_place interfaces/action/ExecuteTaskPlan \
  "{subtask: {parameter_keys: [bottle_class, arm, place_target], parameter_values: [b1, auto, tray_1]}}"

# 狀態（IDLE / BUSY / ERROR / ESTOP），1 Hz
ros2 topic echo /manip/status

# 急停／解除（-w 1：等訂閱的節點連上才送；只寫 --once 常常還沒連上就送出，訊息會掉）
ros2 topic pub --once -w 1 /estop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once -w 1 /estop std_msgs/msg/Bool "{data: false}"
```

```bash
# 單元測試（不需要 ROS）
cd ~/nstc/src/sp3_task_manager && python3 -m pytest test -q
# 有 pytest 外掛衝突時
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test -q
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
