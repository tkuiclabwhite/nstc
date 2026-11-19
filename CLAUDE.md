# nstc — 交接說明（給接手的 Claude 對話）

國科會醫院具身機器人計畫**子計畫三（操作 Manipulation）**的測試平台 ROS2 系統：雙七軸手臂（Dynamixel XM540 ×14）、
Linker Hand O6 ×2、ZED X Mini、頭部馬達 3 顆。負責人楊舒凱。回覆一律用**繁體中文**。

人看的指令在 `COMMANDS.md`；各套件細節看各自的 README。本檔記的是「為什麼長這樣、還有什麼沒做、哪些規矩」。

## 規矩（使用者定的）

- **不要替使用者 commit／push 本 repo**（GitHub tkuiclabwhite/nstc，他用 `git_sync.sh` 自己推）。
- 建置、安裝、長時間執行的指令**列給使用者自己打**；唯讀的查詢可以直接跑。
- `~/ros2_adult` 是大人型系統，**不改**（本 repo 2026-11-12 從它獨立出來，之後各自發展）。
- 密碼、token 不寫進任何文件或指令範例。`linkerhand-ros2-sdk` 的 `setting.yaml` 有 sudo 密碼，別外流。
- 原網頁檔是 CRLF，改完維持 CRLF。

## 機器

| 機器 | 用途 |
|---|---|
| 本機（桌機，RTX 3060） | 開發、Isaac；`~/nstc` 是 .11 的鏡像 |
| `172.17.121.11`（Jetson AGX Orin） | **nstc 以這台為準**（2026-11-13 使用者決定）；無線網卡另有 `192.168.1.200`（同一台） |
| `172.17.121.10` | 另一台測試機，也有 nstc 與 ros2_adult；網頁影像串流在 `:8080` |

登入資訊問使用者，不要寫進檔案。備份：`~/backup_from10_20261112`、`~/backup_from10_20261113/{ros2_adult,nstc}`、`~/nstc_old_20261113`（拆分前本機舊版）。

## 系統拆分（2026-11-12～13，已完成）

- 從 ros2_adult 搬來：motor_control、handpackage、linkerhand-ros2-sdk、zed-ros2-wrapper、imageprocess、motionpackage。
  不搬：usb_cam、walking、strategy、http_server、熱點、拍照。
- `tku_msgs` → `nstc_msgs`（挑 17 msg＋15 srv，py、package.xml、網頁 messageType 全改）。
- `config/` 取代 strategy 資料夾（`strategy.ini` → `nstc/Parameter`）；stand.ini／29.ini 手臂 1～14 全 2048（armpackage 的刻度換算依賴這點）。
- `src/drivers/`：**之後會被正式硬體取代的測試平台驅動**（motor_control、linkerhand-ros2-sdk、zed-ros2-wrapper）＋主控 demo。
  handpackage、motionpackage、imageprocess 使用者說留在 `src/` 根層。
- arm_ws 已併入 `src/armpackage`（URDF、MoveIt 設定、橋接）；手臂說明 README 還在舊 arm_ws，未搬。
- `launch/nstc.launch.py` 用路徑啟動：ZED、driver、motionpackage、雙手、影像 4 節點、rosbridge、web_video_server、手臂 MoveIt。
- `web/` 只留 5 頁＋RobotViewer。

## 子三任務管理 `src/sp3_task_manager`（2026-11-13）

對應子三文件 S3-01 的「任務管理」模組；介面照 **S3-05**（文件在 `~/Desktop/國科會計畫範例/SP3_文件工具包`，GitHub tkuiclabwhite/nstc-sp3-docs）。

- Action `/manip/pick_place`，型別**用主控 demo 的 `interfaces/action/ExecuteTaskPlan`**（套件名 `interfaces` 不能改，否則跟主控不通）。
  goal 是 keys／values 字串陣列，keys 照 S3-05 PickPlace：task_id、bottle_class（b1～b4）、arm（auto/left/right 或 0/1/2）、place_target、timeout_s（預設 30）。
- feedback 階段 0 辨識／1 規劃／2 接近／3 抓取／4 驗證／5 放置／6 退回；error_code uint8 1～8＝MAN-E-001～008，完整碼寫在 message 開頭。
- `/manip/status`（std_msgs/String JSON，1 Hz，IDLE／BUSY／ERROR／ESTOP）、`/estop`（std_msgs/Bool）。
- 所有測試平台介面包在 `adapters.py`（`/bottle_detections_3d`、armpackage 的 `/arm_move` 等、handpackage 的 `/hand/cmd`、`/hand/status`）；換正式硬體只改這檔。
- dry_run 可分段：設定檔 `dry_run.modules`（vision／arm／hand 任意組合），launch `dry_run:=` 有給就覆蓋（2026-11-17）。
  全部模擬時本機 7 種情境測過；**分段模擬與實機版還沒測**。
- 實機跟隨檢查（`/arm_real_status` 的 state／settled，`real_arm.required`）是 2026-11-17 別人在 .11 上加的。
- 信心值門檻、多幀平滑交給 bottle_localization，任務管理只等一筆新的偵測（2026-11-17 拿掉自己的平均）。
- 避障用**方案 A**（2026-11-17 使用者定）：任務管理把桌子（設定檔 `table`，暫時寫死）＋**其他**藥瓶整份發到
  `/arm_ik_obstacles`，要抓的那瓶不算障礙物。別人在 .11 寫的方案 B（safe_lift 過渡點）使用者沒同意，已拿掉。
  直線靠近的碰撞檢查 moveit_bridge 已有（moveit_bridge.py 約 878 行）。
  RobotViewer 網頁每 0.5 秒整份發一次 `/arm_ik_obstacles`（舊格式 = 來源 web），手臂端已改成依來源分開，不再互相蓋掉。
- 急停、移動中被中斷（取消／逾時／狀態中斷）時送 `/arm_real_enable false`（S3-05 §9 要求停止手臂）。
- 頭部：發 `nstc_msgs/HeadPackage` 到 `Head_Topic`（id 1 = pan 28 號、2 = tilt 29 號、3 = roll 30 號；position 刻度、speed），
  driver_node 收。轉頭後相機外參會失準，要等 head_tf 第 4 階段（動態 TF）。
- 單元測試 22 項：`cd src/sp3_task_manager && python3 -m pytest test -q`（本機有 pytest 外掛衝突時加 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`）。
  本機沒編 `interfaces`，節點跑不起來；要實測得在 .11 build。
- 與 S3-05 的差異（ExecuteTaskPlan、JSON 狀態、Bool 急停）是暫時做法；使用者說 ExecuteTaskPlan「只是 demo」，**S3-05 不改**。
  改這裡的名稱、欄位、階段、錯誤碼時，要提醒使用者文件那邊也要對。

## 主計畫主控 demo `src/drivers/ros2_task_plan_demo`

來源 owen-kdweng/ros2_task_plan_demo，**只改了** Action 名稱（`/manip/pick_place`）和 client_gui 的預設 keys／values，README 開頭有註記。
`ros2 run task_plan_demo client_gui` 當主控送任務。

## 同步 .11 的做法

改完**直接同步**，不用先檢查 .11 有沒有人改（使用者 2026-11-17 說「直接同步就好」）：
`rsync -av --relative --exclude __pycache__ --exclude .pytest_cache ./<檔案> iclab@172.17.121.11:~/nstc/`，
同步後在 .11 跑一次單元測試。（.11 上別人也會直接改，11/17 有人沒討論就加了方案 B，使用者知道。）

## 手臂負責人 2026-11-17 已改（moveit_bridge.py，.11 上，還沒推 GitHub）

`/arm_ik_obstacles` 支援 `{"source", "objects"}`（依來源分開存，舊格式 = web）、`/arm_attach`（attach／detach）、
`/arm_stop`（取消軌跡）、`/arm_ik_status` 結果加 `cmd`、`arm`、`reason`（bad_request／busy／unreachable／joint_limit／
collision／plan_failed／exec_failed／canceled／moveit_down／internal）。任務管理已照這些改（2026-11-17）：
- 只認 `cmd` 對得上的結果（`cmd = None` 是狀態通知）。
- reason → 錯誤碼在 adapters.py `ARM_REASON_CODE`：unreachable／joint_limit／bad_request = MAN-E-002，其他 = MAN-E-004。
- auto 選手：兩隻手位置一樣（front／top）就送 `arm: auto` 給 bridge 選，side 才自己照左右試；兩隻手都會先開掌。
- 夾住後（抬起前）attach `sp3_held`，放下、重試、失敗時 detach。
- 急停／中斷：`/arm_stop` + `/arm_real_enable false`。
- 還沒回答：重開實機跟隨時，是模擬回到實機姿勢還是實機衝向模擬。

## task_manager.yaml 逐項調整（2026-11-17 進行中）

順序 frames → table → workspace → place_targets → grasp → hand → retries／timeouts → obstacles → dry_run。
- frames：照 armpackage `MEAS_ORIGIN`（120.2），使用者定。離地 120.2（STL）vs 119.5（捲尺）未判定；
  可靠的是 ZED 量到 base_link 比桌面高 39.0 cm，所以 `table.top_cm` = 81.2（量測座標，不是捲尺離地高度）。
  判定方法：量右肩轉軸離地，CAD 預測 121.7，若約 121.0 就請手臂負責人改 MEAS_ORIGIN。
- **桌面座標**（使用者 2026-11-17 定）：原點 = 桌緣上正對 base_link 中線（base_link 前 16 cm）、桌面高；[右, 前, 上] cm。
  workspace、之後的 place_targets 都用這套。
- workspace：**改用 reach_test 實測表**（使用者 2026-11-17 選 A）。從上方、掌心離桌 10，離中線往自己那側
  0／10／18.9／30／40／50 → 最遠往前 16／22／24／24／22／14，左右手相同、內插；`front_min_cm` 2（前 0 在 18.9、30 joint_limit）。
  表外（跨到另一側）當搆不到——跨中線沒測過，要測用 `reach_test scan --arm left --rights -10 -20`（負的 = 跨過去）。
  歷史：先用左手指尖實量三點的肩膀球模型（扣指尖到掌心 10.8 cm），實測平均寬 3 cm、中線寬 7.5，已拿掉。
- table：edge 16（11/18 改 12.5）、top 81.2、depth 75、width 150、center 0（使用者 2026-11-17 給）。
- obstacles 尺寸照 bottle_localization 每類別的 bottle_dims（已是）。
- **測試工具 `reach_test`**（2026-11-17）：`ros2 run sp3_task_manager reach_test scan|obstacles`，直接對 moveit_bridge 測
  實際極限（跟球模型比）與 6 種避障擺法，場景來源 `sp3_test`（11/18 改名 `reach_test`：sp3_ 開頭會被任務管理當成自己的）。使用者要先跑這個確認極限與避障，再填 place_targets。
- place_targets：使用者說先一個就好（tray_1，位置還沒給）。
- **2026-11-17 reach_test 第一輪**：幾乎全是 collision，原因是手本身插進桌子，不是範圍。URDF 算手＋手腕相對掌心點的外形：
  手指方向 −14.9（含手腕）～+10.7 cm、左右 −4～+11（拇指側）、掌心法向 −4～+4.5。掌心最低離桌：front 手指朝上 14.9、
  手指橫放拇指朝下 11、拇指朝上約 5、top 4.5。**從前方抓桌上 8～9 cm 的藥瓶，手指一定要橫放（拇指朝上）或改 top**。
  → 調 `grasp` 時要加「掌心離桌高度」與 `fingers`，現在 grasp_plan 把掌心放在藥瓶中心高度（約 4 cm），front 一定撞。
  工具的 busy（送太快）已修，加了 `--fingers`。
- **從上方抓**（使用者 2026-11-17：本來就要從上方抓）：`grasp.approach: top`，reach_test 預設改 `--dir top --up 10`、
  避障案例改成從上方的版本。從前方測過手指 ±90、掌心離桌 6：右肩前方都到不了（沒再追，因為不用）。
- **reach_test 從上方第一輪（2026-11-17 17:0x）**：
  - 左手（掌心離桌 10）實際最遠前：右 0 → 16、−10 → 22、−18.9 → 24、−30 → 24、−40 → 22、−50 → 14；
    模型 23.5／26.5／27.5／26／22／15，平均寬 3.1 cm（中線差最多 7.5）。前 0 常 joint_limit（太靠身體）。
  - 右手幾乎全 exec_failed／plan_failed：**right_arm_controller 卡在 unconfigured**。launch.log：load_controller 的服務回應
    逾時沒送到 spawner（開機負載重），spawner 重載撞「already loaded」退出。Humble 啟動競爭，偶發（11/16 兩次都正常）。
    手動救：`ros2 control set_controller_state right_arm_controller inactive` 再 `active`。
    根治在手臂端（seven_axis_arm_moveit_config 的 MoveIt 產生的 spawn，每個控制器各一個 spawner 同時開）：
    建議改成一個 spawner 一次開全部控制器，或加長 timeout——要跟手臂負責人說。
  - reach_test 開頭已加控制器檢查（兩隻手不是 active 就停）。
  - 避障 3、5、6 PASS（緊貼一瓶、上方板子、桌面下都正確擋下）；1、2、4 失敗是右手控制器，要重測。
- **看結果不用 RViz**（使用者 2026-11-17：RViz 太麻煩——AGX 是 ROS_LOCALHOST_ONLY=1、桌機 domain 77，rviz_relay 只轉 TF／標記）：
  reach_test obstacles 每項存 PNG（`~/reach_test_out/…/caseN.png`）＋印手離各物體最近距離。`arm_shape.py` 從 URDF 碰撞模型
  算手臂外形（FK 跟 armpackage ChainFK 比過誤差 0），手指當伸直。**使用者跑完，我用 scp 抓圖回來看、跟他報告。**
  抓圖：`sshpass … rsync -a iclab@192.168.1.200:reach_test_out/ <scratchpad>/rt_out/`（scp 多來源會失敗）。
- **避障第二輪（2026-11-17 17:49，控制器都 active）**：右手 6/6；左手 4/6——第 1、2 項 plan_failed（試 4 秒），圖上左手停在站姿沒動；
  同一個目標第 4 項卻成功（手肘往外甩到右 −35 繞上來）→ **左手從站姿（手在桌面下、桌緣後）規劃到桌面上不穩定**，不是避障錯。
  旁邊 15 cm 的瓶：手離 +2.8 cm（含 1 cm 餘裕）；前方 15 cm 的瓶：+0.4～0.6 cm（手指朝前偏內約 30°，指尖快碰到）。
- reach_test 加了 `point`（移到一點停住讓人量，Enter 回站姿）、obstacles 會印實體物品擺放位置、`--pause` 時先停讓人擺好再動、
  `--cases` 挑項目。準備上實機（2026-11-17）：實機跟隨開著時 moveit_bridge 會把目標往上補 0.17×前（REAL_Z_COMP），
  要量的是實機掌心有沒有真的在目標高度。
- **point 模擬（2026-11-17 18:0x）**：右手 5 點全到（離桌 5.5／3.5／1.5 跟掌心高度一起降，手最低處在掌心下 4.5 cm）。
  左手同 5 點（鏡像）：1、3 到；2（−18.9,12,8）、4（−18.9,3,8）plan_failed；5（0,10,8）joint_limit。
  → **左手從站姿出發規劃約一半失敗**（含避障那輪共 9 次失敗 4 次；右手 11/11），**左右不完全對稱**（中線右手到、左手 joint_limit；
  scan 時左手中線到 16 是從旁邊的點挪過去的）。都要找手臂負責人。workspace 表左右共用，左手中線附近偏寬。
  使用者說右手那半實機有點問題，2026-11-17 實機先只測左手。
- `ros2 topic pub --once` 送 `/arm_real_enable` 沒生效（狀態還是 off、沒有「沒開：」）：還沒連上就送出、訊息掉了。
  COMMANDS.md 的 `/arm_real_enable`、`/estop` 都改成 `--once -w 1`。實機跟隨開著時打開的當下是模擬移到實機姿勢（不會衝）。
  實機跟隨一次管兩隻手（arm_config 的 calibrated），只開左手要把右手 calibrated 暫改 False（手臂負責人的檔，要使用者同意）。
- **實機第一次（2026-11-17 18:3x，左手 point −18.9,12,10）：剛從站姿往旁邊抬時拇指撞桌子，使用者斷電**（沒到終點，settle log 沒有目標紀錄）。
  REAL_Z_COMP 在站姿附近（前 ≈ 0）≈ 0，所以不是終點補償的問題。兩個假設：①模擬路徑本身離桌子太近 ②實機跟不上模擬、各關節直接往
  最新目標轉（抄捷徑），路線跟模擬不同。→ reach_test 加了**路徑記錄**：移動中記 /joint_states（模擬 = 關節名 rad、實機 = 馬達 ID 刻度
  經 arm_config.ticks_to_rad），印整段路徑離物體最近的時間與掌心位置，圖上畫掌心路線（綠 = 模擬、紫 = 實機）。
  **模擬結果（18:31）：模擬路徑本身穿過桌子 13.1 cm**（第 2.4 秒，掌心在 右 −27.9、前 4.7、上 −6.7）：站姿→預備點那段
  從桌面下往前掃進桌子再抬起。moveit_bridge `_direct` 每 1.5° 用 /check_state_validity 檢查，照理會擋 → 懷疑 MoveIt 場景沒有桌子
  （bridge 自己的清單有，所以「目標在桌子裡」擋得下）或碰撞設定排除。reach_test 已加：路徑插進物體時問 MoveIt 那一刻姿勢有沒有撞、
  列出 MoveIt 場景物體（/get_planning_scene WORLD_OBJECT_NAMES）。下垂（終點才補償）那則訊息先不送，原因不是它。
  第二次（18:35）同一點路線不同：先往旁邊、往後甩（掌心到 右 −55、前 −27），再在桌面上 ~5 cm 越過桌緣，最近 2 cm
  ＝使用者說的「側舉時撞到」。→ MoveIt 路線隨機：有時穿過桌子（手臂端漏檢，待查原因），有時只留 2 cm。
- **`table.margin_cm: 3`**（2026-11-17）：場景裡的桌子上、前後、左右各加厚 3 cm。終點的 _real_free 檢查比的是實機的手對加厚的桌子，
  從上方抓手最低在掌心下 4.5，最矮 b2 掌心 8.8 → 手最低 4.3，所以最多 4 cm。reach_test 印的 table 距離是對加厚後的桌子。
  加厚後同點跑 5 次（18:38）：3 次到、2 次 plan_failed；成功的路徑離加厚桌 +1.1／0.0／+0.4（離真桌 ≥ 3 cm），沒再穿過桌子。
- **穿過桌子的原因（推測，待手臂負責人確認）**：moveit_bridge `_sync_scene` 在 `/apply_planning_scene` 還沒 ready 時直接 return、
  不重試也不報錯 → 開機後第一次發的場景沒進 MoveIt；bridge 自己的清單有（終點檢查擋得下），但路徑檢查問 MoveIt 看不到桌子。
  實機撞桌那次也是 18:16 重開後的第一次測試。→ **任務管理 set_obstacles、reach_test scene 發完都問 /get_planning_scene
  確認物體真的在（少了或多了就重發），5 秒（`obstacles.scene_confirm_s`）還不行就不動手臂**（任務管理回 MAN-E-004）。
  package.xml 加 moveit_msgs。
- **實機第二次（18:50，加厚 3 cm＋場景確認）**：拇指撞桌改善很多，但側舉高度還是不夠（目測差 ~5 cm）。輸出資料夾是空的（中途停了）。
  推測：手臂往旁邊伸（掌心離肩膀遠）時機構下垂，馬達讀數看不到；REAL_Z_COMP 只看「前」、只補終點。要量：
  `reach_test point -50 -10 15 --arm left --real --speed 10`（桌緣後、桌面上 15，側伸）量實際高度。table.margin_cm 最多可到 4。
- **左肩轉軸實量離地 121.5 cm**（2026-11-17），模型 120.2 + 1.475 = 121.7 → **MEAS_ORIGIN 120.2 是對的**；bottle_localization
  robot_reference 的 119.5 偏低 0.5～0.7（不是使用者的套件，告知負責人）。桌高若 80.5，量測座標桌面應 80.5，模型 81.2 偏高＝保守。
- **給手臂負責人的清單（2026-11-17 整理給使用者轉）**：①下垂補償只在終點、只看前 → 要用到整段路徑、看離肩水平距離
  ②_sync_scene 服務沒 ready 直接 return 不重試 ③左手從站姿規劃約一半失敗（14 次失敗 6）、中線左右不對稱
  ④路徑往旁邊甩到右 −55～−67 ⑤right_arm_controller spawner 競爭。已確認不用改：MEAS_ORIGIN、重開跟隨不會衝。
- **完整流程模擬（2026-11-18）**：.11 上用 `~/sp3_flow_test.yaml`（主設定檔副本：required false、fail ''、tray_1 = [32.4, 25.0, 93.2]）
  ＋`dry_run:=vision,hand`，手臂在 MoveIt 裡動：**成功**（右手 b1 → tray_1，87 秒），第一次移到預抓位置失敗、重試一次後成功（原因待查）。
  踩到：①driver 啟動時馬達沒上電 → /joint_states 名稱空的 → MAN-E-005（要先上電再開 nstc.launch；required false 時是否跳過此檢查待決定）
  ②預設 timeout_s 30 不夠（只到預抓就 17 秒），送任務要給 150；預設值是 S3-05 的，要改得連文件一起改。
  網頁橘色半透明手臂 = MoveIt 模擬（moveit_bridge 轉 /ik_viewer/joint_state，只有手臂關節，手指不會動）；藍色 = /joint_commands。
- **子三任務網頁 `web/SP3TaskInterface.html`（2026-11-18，使用者要的，暫代總計畫主控）**：導覽列加了 SP3Task（js/nav.js）。
  不用 roslib：web/js/roslib 是 1.2.0 沒有 ROS 2 action；1.4.1 也沒有，2.x 改成 ES module，換掉會壞其他 5 頁 → 直接講 rosbridge 2.0 JSON。
  rosbridge 坑：①`send_action_goals_in_new_thread` 預設 false → 送 goal 的那條連線卡到任務結束，所以狀態／急停／取消走常駐連線、
  每個任務另開一條（取消用 goal id，rosbridge 的 client_handler_list 是類別共用的，跨連線可取消）
  ②goal 被拒絕或 server 沒開時 rosbridge 卡死在等結果、不回任何東西（那條 thread 洩漏）→ 網頁 5 秒沒 feedback 就判定沒被接受；
  送出前看 /manip/status 是 BUSY／ESTOP 就不給按。已測：訂閱 /manip/status OK、headless 載入無 JS 錯誤；**送任務、取消還沒實測**。
  使用者決定：不做原始 keys／values 欄位；急停不確認、解除要確認；總計畫 demo 不動。
- 網頁看得到 client_gui 送的任務：**rosbridge 訂不到 action 的 feedback topic**（它只在 `interfaces.msg` 找型別，
  `ExecuteTaskPlan_FeedbackMessage` 在 interfaces.action，log：Unable to import msg class）→ 任務管理把 `feedback`（最新 status 字串）、
  `last_message`（上一個 result.message）加進 /manip/status，每次 feedback 立刻發一筆（2026-11-18）。**S3-05 的狀態欄位要跟著對。**
  網頁用 /manip/status 看別人的任務（不能取消）。本機私有 rosbridge（domain 91、port 9097）＋headless Chrome（CDP）模擬狀態序列測過。
- 任務管理每秒重發自己的障礙物（adapters `_republish`）：/arm_ik_obstacles 不是 latched，RobotViewer 晚開會看不到。
  RobotViewer 顯示功能本機測過正常；使用者第一次沒看到 = 頁面開在任務發障礙物之後。
- **RobotViewer 顯示別的來源的障礙物（2026-11-18，使用者跟手臂負責人談過、同意改）**：樣板加 `startExtObstacles()`，訂 /arm_ik_obstacles，
  `{source, objects}` 且 source ≠ web 的畫成半透明青色；放在獨立的 `extObsRoot`（位姿抄 base_link），**不掛進機器人底下**
  （幽靈手臂 clone 整個 worldGroup 會被複製、網頁碰撞檢查也會掃到）。HUD 多一行。本機 headless 測過（WebGL 要 `--use-angle=swiftshader
  --enable-unsafe-swiftshader`）。RobotViewer.html 要使用者在 .11 跑 build_viewer.py 重新產生。夾在手上的（/arm_attach）沒畫。
- **手臂負責人 2026-11-18 的修改（已拉回本機）**：spawn_controllers 改成一個 spawner 依序開、call timeout 30（⑤）；
  每個動作前 `_ensure_scene` 確認場景真的在 MoveIt（②）；**真正穿過桌子的原因：Humble MoveIt 不檢查規劃出來的路線**，
  現在 `_plan_checked` 逐點查、撞到 -13 重規劃；移到預備點／回站姿多一種「先往正上方抬過物體（物體最高點 +10／18 cm）再直接轉」
  的路（④，也應該改善③）；結果 note 會寫「路線：…」。**①下垂補償（整段路徑、看側伸距離）沒改**，③左右不對稱沒處理。
- **要抓的藥瓶另外顯示（2026-11-18）**：任務管理發 `/manip/target`（JSON {class, objects:[stick]}，跟 /manip/status 一起每秒發，
  任務結束 10 秒後清空），RobotViewer 畫亮綠色；障礙物青色。不能借 /object_position（網頁會把 trackedProp 當自己的物體發給 MoveIt）。
  RobotViewer 兩種都「內容沒變不重畫」。本機私有 rosbridge＋CDP 測過。
- **固定物 `fixtures`（2026-11-18）**：使用者說手後面有黏在架子上的小桌子，從站姿要先側舉；手臂端新的「先往上抬」直直往上會撞它，
  場景裡卻沒有。task_manager.yaml 加 `fixtures`（桌面座標方塊 center/size，加厚 table.margin_cm），scene() 與 reach_test 都會放。
  **尺寸位置待使用者量**，目前空的。reach_test point 圖：紫色（實機馬達）停在站姿 = 實機跟隨關著，不是路徑差很多。
- 放置高度 bug（待使用者選放手高度）：place_targets 寫死掌心離桌 12，手上瓶子高 9.2（b1）時瓶底只離桌 1.8 < 桌子加厚 3 → collision。
  建議改成放置點 = 盤面、掌心 = 盤面 + 瓶高 + palm_gap + 放手高度（A：3.5 cm）。
- reach_test 加 `bottle` 模式（2026-11-18）：收 2 秒 /bottle_detections_3d 平均，掌心到瓶頂正上方 `--above`（預設 10）cm，
  其他看到的藥瓶當障礙物；印影像的桌面座標讓人跟捲尺比。使用者後來決定加小桌子：fixtures back_shelf（長 50 對稱、寬 20、離地 101～107、前緣離「base_link」5.5）。
  **URDF 的 base_link 不在立柱中間**：立柱在 base_link 後 1.4～4.3 cm，肩關節在 x = 0；使用者說的 base_link 應該是立柱前面，
  所以前緣放在 base_link 後 6.9（待確認）。站姿手臂在那個高度最後到後 6.2 → 小桌子只加厚 0.5（fixtures 可各自設 margin_cm）；
  起點就碰到的物體 moveit_bridge 會暫時不算，所以不能讓站姿碰到。
- **桌子、小桌子改以網頁三軸（= URDF base_link）為準（2026-11-18）**：base_link 在肩關節轉軸那條線上，在立柱前面 1.35 cm 的空中
  （量測座標「前 0」= 立柱前表面，MEAS_ORIGIN x −0.0135）。使用者對照三軸定：桌緣 base_link 前 **12**（原本 16；中途說過 12.5、13）、
  小桌子前緣後 **9**（後緣 29；前後不加厚，左右上下 1.5，fixtures margin_cm 可給 [前後, 左右, 上下]）。
  桌板 72～80 離地，試過 `table.bottom_cm: 72` 只畫桌板 → 站姿的手在桌板下，規劃讓手指從桌板下往前伸、實機撞桌 → **改回實心到地板**；大桌前後加厚 2（margin_cm [2, 3, 3]；0 時手貼桌緣邊舉邊往前，3 會讓站姿碰到）；桌板下面空了，站姿手在桌板下（掌心離地 67）。
  桌面座標原點跟著往後 4 → workspace「前」全部 +4、front_min 6；tray 的 pos_cm 是量測座標沒變（桌面座標前 19）；
  假藥瓶照桌面座標寫，實際往身體靠 4。站姿離加厚後的桌子 0.6、離小桌子 2.6（URDF 算）。
- **任務第 0 步、第 6 步改回 ready pose（2026-11-19 使用者：「直接改回 ready」，不要退回 stand 的備案）**：`arm.ready()`；
  手臂端還沒有 /arm_ready 前任務會在第 0 步 MAN-E-007。launch 加 `real_arm:=true/false`（覆蓋 real_arm.required；false 也不檢查
  driver /joint_states），**不再需要 ~/sp3_flow_test.yaml**；主設定檔 `dry_run.fail` 改回 ''。
- **SP3Task 網頁改版（2026-11-19，使用者要的）**：上方控制列（急停、Stand、Ready pose、狀態）；右上原本的狀態區改成「測試」：
  移到座標（桌面／base_link）、移到影像某瓶上方，**由任務管理執行**（使用者選，帶場景與實機檢查），`/manip/test_cmd` →
  `/manip/test_result`，解析在 `try_cmd.py`；失敗把 `[reason]` 翻白話（unreachable／joint_limit = 到不了、plan_failed = 找不到路）。
  「座標」區吃 `/manip/pose`（URDF FK 算掌心，`ArmShape(meshes=False)`；實機用 arm_config 換刻度），base_link 與桌面座標並列。
  Ready 按鈕送 `/arm_ready`（**格式我們先定：std_msgs/String {"arm": "both|left|right"}，結果 cmd = ready**），
  沒人訂閱就回「手臂端還沒有 /arm_ready」（使用者選）。測試結果附「可及範圍表：範圍內／外」= 驗證 workspace 表。
  本機：dry_run 實跑 test 指令＋headless 網頁對假任務管理測過；.11 上還沒實測。
  待做：reach_test scan 改從 ready 出發、plan_failed 重試、分開記「IK 到不了／找不到路」（等 ready 做好）。
- **SP3Task 速度欄位（2026-11-19，使用者要的；launch 參數不要）**：控制列「速度」（Dynamixel 單位 1～50）→ `/manip/speed`
  std_msgs/Float64 → 任務管理記在 `arm.speed`、馬上轉送 `/arm_speed_cmd`，之後的任務、測試、Stand、Ready 都用它；
  `/manip/status` 多 `speed` 欄位，網頁欄位跟著它。重開任務管理回到設定檔 `grasp.arm_speed`（30）。
  moveit_bridge 只有一個速度，最後送的人贏（RobotViewer、reach_test 送的會被下一個任務／測試蓋回）。
- **11/19 使用者說手臂負責人改好了，但 .11 與 GitHub 都沒有**：.11 跑的是 `~/nstc/src/armpackage`（不是 ros2_adult），
  moveit_bridge.py 最後改 11/18 16:58、沒有 /arm_ready；ros2_adult GitHub 最後 11/13。已請使用者叫他放上 .11 再看。
- **ready pose 位置定了（2026-11-19 使用者在網頁測試區抓的，「之後都用這組」）**：桌面座標 左 (−50, 15, 20)、右 (50, 15, 20)，
  掌心朝下（top）＝量測 前 28.4、右 ±50、高 101.2（先抓的 (±25, 15, 10) 實機太低 → (±35, 15, 15) → 錄路線實機測過後使用者再改成左右 50、高 +5）。寫在 task_manager.yaml `ready_pose`；`via: move` = ArmClient.ready
  用 /arm_move 依序移左、右手（已在那裡 = bridge 微調），`via: arm_ready` = 手臂端 /arm_ready（做好後切；它的 ready 要對到這兩點）。
  任務第 0、6 步、網頁 Ready 按鈕都走這個；網頁測試區 stand／ready 前先放場景。掌心離桌 20 → 手最低離桌約 15.5（URDF 算：手最低處在掌心後下方，b1 高 9.2）。
  .11 上第一次測時：使用者在 RobotViewer 填桌面座標（它是量測座標，高 = 離地）→ 離肩 111 cm，提醒過兩套座標的差別。
- **ready 路線錄起來、兩隻手同時走（2026-11-19 使用者：「存起來、同時送兩個路徑」，速度 stand↔ready 都 30）**：`via: path`（預設）。
  為什麼存整條路：只存終點關節角、每顆馬達直接轉，手指會掃進桌子。.11 log（14:26）：一隻手站姿→ready 15～19 秒，
  其中規劃只 2～4 秒，大部分是 MoveIt 繞路的執行＋最後 10 cm 慢速直線；「先往上抬」要抬到 118.5（小桌子 107＋1.5＋10）撞到 → 只剩 MoveIt。
  `ready_path.py`（純 numpy，有單元測試）＋ adapters `record_ready`／`_path_go`／`_path_hits`／`_exec_path`：
  錄 = 手在站姿、/arm_move 左再右、記 MoveIt /joint_states，左右各一段存 `config/ready_path.yaml`（設定檔 `path_file`）；
  走 = 兩段按進度同步成一條 14 軸軌跡，任務管理**直接**送 MoveIt `/execute_trajectory`（不經 moveit_bridge，同 bridge 回站姿的做法），
  走前每 1.5° `/check_state_validity`（同時最多 8 個請求：服務佇列只有 10，一次全送會被丟掉——本機測到）；回站姿倒過來；
  手在別處先直接轉到 ready。不能用（沒錄、位置改了、會撞）→ 退回 move／`/arm_stand`。實機跟隨沒開時錄的目標自己補 0.17×前（`real_z_comp`）。
  網頁控制列加「錄 ready 路線」（confirm）。本機假 MoveIt 整合測試 7 種情況過（scratchpad sim_ready_path.py）。
  .11 17:22 錄（只有模擬），實機「快很多」，但**左手越過桌緣時拇指擦到桌角**（右手沒事）：URDF 算左手自己的路線舉到掌心高 83 就往前，
  越過桌緣時手最低只比桌面高約 4 cm（模擬沒撞，實機左手側伸比模擬低）；右手的先舉到 113 才往前（31 cm）。
  URDF 左右鏡像（同一組關節角 = 鏡像姿勢，FK 確認；錄到的左右 ready 終點關節角一模一樣）→ 設定檔 `mirror: right` 兩隻手都用右手的路線，
  舊檔案不用重錄（load 時套用）；錄的時候只舉右手。實機左手側伸會往下垂（約 4 cm 以上），給手臂負責人下垂補償當資料。
- **ready (±50, 15, 20) 18:30 重錄（實機跟隨開著、只舉右手）**：實機左手拇指又輕碰桌子一下；實機右手 ready 比左手高 1 cm，
  使用者量右肩馬達比左肩高 0.5。到位紀錄：照馬達讀數算的掌心高 左 104.5、右 104.4（一樣）→ 差的 1 cm 是機構（安裝高度＋變形）。
  URDF：路線在 34% 拇指只比桌面高 1.8、在桌緣後 2.9（離真的桌子 3.2）。那段手臂幾乎伸直：往上 1 cm 要轉 10～13°、往後只要 2～5°。
  → `ready_pose.adjust_cm`（ready_path.adjust／apply_adjust，ArmClient._ready_rec 套用、快取）左手 edge [-2, 0, 1]、end [0, 0, 1]：
  經過桌角離真的桌子 3.2 → 5.5、ready 左手 +1 cm；往後 3 逆解不到。arm_shape 加讀 URDF 關節範圍。
  實機確認「轉的路線沒問題了」（2026-11-19）→ 網頁「錄 ready 路線」按鈕拿掉（record_ready 指令還在，要重錄用 ros2 topic pub）；
  `ready_pose.speed` 30 → 50（使用者），_exec_path 走的時候 /arm_speed_cmd 也送 50（實機馬達不能比軌跡慢，不然跟不上、抄近路），
  走完切回 grasp.arm_speed 30。URDF 算一趟 8.9 → 6.9 秒（加速度也跟著速度比例，所以只快 2 秒）。**速度 50 還沒上實機**。
  照路線走時用 `ready_pose.speed`（不是網頁速度欄位 /manip/speed），走完切回 arm.speed（網頁設的值）。
- **2026-11-19 19:11 另一邊同步到 .11 時用舊檔蓋掉** adapters.py、task_manager.py、SP3TaskInterface.html、COMMANDS.md、CLAUDE.md、README
  （git_sync 19:16 commit 7094796 也是舊的；那份有新的 /manip/speed 速度欄位）。19:3x 從對話紀錄重建我的版本，再把 /manip/speed 合併進去。
  **改 .11 前先確認對方沒有同時在同步同一批檔案**（rsync 會整個檔案蓋掉，不會合併）。
- **reach_test scan 改從 ready 出發（2026-11-19 使用者：「可以測手的極限範圍了」）**：任務管理開著就送 /manip/test_cmd ready
  （照錄好的路線），沒開用 /arm_move；每列先回 ready；plan_failed 回 ready 重試 `--retry`（預設 2）；分開標 ✓／✗ IK／C 撞／P 找不到路；
  印 workspace 表可直接貼、每點存 scan.csv。本機用假 moveit_bridge 回應測過邏輯，.11 還沒實測。
- **11/19 20:33 .11 log 看到的兩個問題 → 21:xx 已改（使用者：「tray 高度要根據瓶子」「你直接改吧」）**：
  ① 放 tray 撞 `sp3_table↔手上的 sp3_held`：原本 tray pos_cm 是掌心、離桌 12 寫死，瓶底 = 12 − 1 − 瓶高（b1 1.8 < 加厚 3）。
     **改**：place_targets 的「高」改成放置面（81.2 = 桌面，tray 厚度待量），掌心 = 放置面 + 瓶高 + palm_gap + `grasp.place_gap_cm`（1）。
     實機跟隨開著時 moveit_bridge 往上補 0.17×前（tray 前 32 → 5.5），模擬的瓶底高於加厚、實機剛好在目標（使用者確認補償後實機高度準）；
     實機跟隨沒開時任務管理自己補（`ArmClient.sim_z_comp`，用 ready_pose.real_z_comp），模擬才跟開實機時一樣。
     左手實機再低約 1 cm（機構）→ `grasp.arm_z_offset_cm: {left: 1}`，ArmClient.move 每個 /arm_move 都加（使用者 11/19 說好）；
     兩隻手補的不同時 auto 不交給 bridge，照 arm_order 一隻一隻試。ready 錄好的路線用自己的 adjust_cm，不重複。
  ② 錄好的 ready 路線被拒 `路線第 1/139 點會撞到：lh_thumb_distal↔html_sp3_table`：左手在站姿時拇指就在桌子加厚裡（只動右手時左手也被檢查）。
     跟 19:11 覆蓋無關（原本的 _path_hits 也沒處理）。**改**：`_path_hits` 先檢查起點，起點就碰到的那對先放行、分開後就不放行，
     只在路程前 `START_TOUCH_FRAC`（20%）內放行（同 moveit_bridge 的做法）。本機假 MoveIt 回應測過四種情況，.11 還沒實測。
  ③ `reach_test scan --arm left` 不給測，但右手沒加 --real 也跟著實機動：**bug**，原本固定等 1.5 秒，新節點還沒收到
     /arm_real_status 就當「關著」。改成等到收到為止（5 秒沒收到 = real_arm_bridge 沒開），測到一半被打開也會停。
- **reach_test scan 右手實測（11/19 21:01，掌心離桌 10，csv 在 .11 ~/reach_test_out/20261119_210151_scan_right）**：沒有 plan_failed。
  最遠（前，桌面座標）：右 0 ≥28、10 ≥34、18.9 34、30 34、40 32、50 ≥26（≥ = 最後一點都到了，還沒找到極限）；
  最近：右 0 → 0、10 → 4、18.9 → 8、30 → 6、40、50 → 0（joint_limit），近端不是單調的，front_min 一個數不夠。
  21:25 補 0、10、50（--beyond 20）、21:32 --up 18：最遠 0 → 28（up 18 時 ≥34）、10、18.9、30 → 34、40 → 32、50 → 26，兩個高度一樣。
  → **workspace 已更新**：reach_front_cm 照實測（取小的）、新增 `reach_near_cm` 表（近端 0/4/8/6/0/0，兩列中間取遠的、不內插），
  front_min_cm 只在沒表時用。左手 22:10 抽測 18.9、40：近 8／0、遠 34／32，跟右手一模一樣 → 兩隻手共用一張表確認。
- **11/19 22:20 dry_run:=vision,hand real_arm:=false 全流程**：Ready／Stand 照錄好的路線（兩隻手一起、倒著走）OK；
  b1、b3 右手→tray_1，b2、b4 左手→tray_2 全部成功（放置不再撞）。測試區 auto 右 30 前 32 失敗、卻回報左手「離肩膀太遠」：
  左手 +1 讓 auto 改成任務管理一隻一隻試，只回報最後一隻的錯。**改**：auto 只試可及範圍表搆得到的手（都搆不到才兩隻都試），
  都失敗時每隻手的原因都列出來。重測：右手 unreachable（離肩膀 56 > 55）。原因：scan 是開實機跟隨量的，bridge 把模擬目標補高
  0.17×前（98.9），只看模擬時沒補（91.2）→ **改**：sim_z_comp 從只用在放置改成 ArmClient.move 每一步都加（實機跟隨沒開時），
  模擬跟開實機時停在一樣的地方；record_ready 不再自己補（move 會補）。
  重測：第 7 項右手到了；Stand→Ready 照錄好的路線。
  另：22:27 那次 stand→ready 退回 move：`路線第 2/85 點會撞到 lh_thumb_distal↔桌子`——起點沒碰、第 2 點才碰（站姿拇指離加厚 0.6，
  差零點幾度）→ **改**：「起點就碰到」放寬成路程前 `START_WINDOW_FRAC`（5%）內碰到的，一樣要在 20% 前分開、分開後再碰就不走。
- **11/19 22:49 實機 dry_run:=vision,hand：放置三次都 collision**「右手從上方：實機的位置（比模擬低 5.5 cm）會撞到東西」。
  moveit_bridge 實機跟隨開著時**也檢查實機的位置**（我原本以為只檢查模擬），實機瓶底離桌 1 在桌子上方加厚 3 裡。
  **改**：放下那一步桌子上方加厚暫時改成 `grasp.place_table_margin_cm`（0.5），放開、抬起後改回（grasp_plan.scene 的 table_top_margin_cm）。
  22:58 重測：b1 放下 joint_limit（微調時實機位置撞 → 一般流程最後 10 cm 碰關節極限）、b3 collision、b2（左手）成功。
  推論（bridge 沒列碰到什麼）：實機位置檢查 = 場景物體上移 5.5，手最低處離桌約 6.5，tray（右 25 前 19）離假藥瓶 b1 只有 9.3 cm，
  手掌真的會壓到 b1、擦到 b3。**改**：tray 移到桌面座標 (±25, 28)（離 b1 17、b3 19 cm）。tray 真正位置還沒定。
  使用者量：b2 放下時實機掌心離桌 13.5（預期 9.8，高 3.7）→ 前 41（tray 原位前 32）0.17×前 補太多？給手臂負責人（下垂補償）。
  Ready／Stand 照錄好的路線 OK，測試區右 30 前 32 實機到了。
- **11/19 23:17 實機 tray (±25, 28)：b1～b4 四個任務都成功**。放下時照馬達讀數掌心比要的高 5.5～6.9（補償 +7、實機只低 1.1～1.5）。
- **11/19 尺量 4 點（測試區移到座標）**：右 25 前 28 上 11 → 14.3；左同 → 15.8；右 18.9 前 12 上 10 → 12.8；左同 → 14.5。
  實際下垂（補償 − 多出來的）：右 前41 3.7、前25 1.5；左（扣掉 +1）3.2、0.8。≈ 0.14×(前 − 14.5)，不是單純比例（或掌心點定義差固定值）。
  左手 +1 之後反而比右手高 1.5～1.7 → **arm_z_offset_cm 左手改回 0**（ready 路線的 adjust_cm +1 沒動）。
  補償太多 → 實機抓取掌心高約 3 cm、放下瓶子掉約 4 cm；不能從任務管理往下減（bridge 實機位置檢查會撞桌），要手臂端改。
- **給手臂負責人的第三批（11/19 晚整理，使用者轉）**：①下垂補償補太多（上面 4 點＋放置紀錄）②實機位置會撞時列出碰到哪兩個物體
  ③/arm_ready、側舉預備姿勢不用做了（任務管理錄好的路線取代）④not_ready 可不做 ⑤之前還沒做的：_obstacle_top 抬太高、
  起點碰到的物體整段不算、掌心偏移（等我們量）、_sync_scene 不重試、spawner 競爭。
- **scan 測完停在 ready 舉著 → 右手過熱（11/19）**：scan 改成只舉測的那隻手（test_cmd ready 帶 arm），測完回站姿（任務管理開著就照路線）。
- **2026-11-19 開會決定：stand／ready pose 兩種姿勢**：stand = 現在手放下的站姿；ready = 雙手舉到桌子上方準備。
  只有第一次跑任務前 stand → ready，任務之間都回 ready。側舉只用在 stand ↔ ready，其他移動照常規劃（moveit_bridge 兩種走法）。
  手臂負責人還沒動 11/18 的清單 → 依這個決定重新整理給他（11/19）。任務管理（主控）要怎麼改使用者等等再說，**先不改**。
- **2026-11-18 收工時已整理給手臂負責人（第二批，使用者轉）**：①「先往上抬」高度算太高（旁邊藥瓶、小桌子都算、圓柱多算半徑）
  ②加小桌子後只剩 MoveIt 隨機規劃 → 要固定「側舉預備姿勢」③站姿手指朝前、離真的桌角幾 cm，實機離開／回站姿手指撞桌
  ④起點碰到的物體整段不算 → 限制加厚 ⑤實機掌心偏移（前 +4、左不夠 2，影像準）待 `reach_test bottle --fingers 0/90` 分辨
  ⑥下垂補償還沒改（待子三多點量測）。也告知他 RobotViewer 樣板加了障礙物、目標顯示。
  子三待辦：--fingers 測偏移、多點下垂、放置高度（等使用者選 A）、retries.grasp 改 2？、告知 bottle_localization 119.5。
- **給手臂負責人的第二批（使用者說晚點一起整理，2026-11-18 累積）**：①「先往上抬」_obstacle_top 把旁邊的藥瓶、後面的小桌子都算進
  「路線下方」→ 要抬到 103～118.5 cm，左右手都抬不上去；直立圓柱最高點多算半徑 ②加了小桌子後「先往上抬」一定撞小桌，
  只剩 MoveIt 隨機規劃，左手約一半失敗（規劃結果無效、繞遠路 >30°、路線撞到）→ 希望固定的「側舉預備姿勢」（站姿每顆馬達直接轉過去、
  手往外抬高過桌面，回站姿反過來）③下垂補償（第一批 ①）還沒改。
- **RobotViewer 收不到障礙物的 QoS 坑（2026-11-18）**：rosbridge 訂閱時，若當下所有發布者都是 TRANSIENT_LOCAL 就用 TRANSIENT_LOCAL 訂
  （rosbridge 自己的發布者一律 TRANSIENT_LOCAL）。RobotViewer 自己就經 rosbridge 發 /arm_ik_obstacles，所以先開網頁再開 reach_test／
  任務管理時，網頁的訂閱是 TRANSIENT_LOCAL，VOLATILE 的發布者收不到（WARN：requesting incompatible QoS … DURABILITY）。
  → /arm_ik_obstacles、/manip/target 改用 `adapters.LATCHED_QOS`（TRANSIENT_LOCAL, depth 1）發；本機私有 rosbridge 重現並驗證過。
- 黃色手指要動（使用者問）：網頁幽靈手臂吃 `/ik_viewer/joint_state` 任何 URDF 關節名，模擬手時任務管理可以發手指角度到那裡（不用改網頁）——還沒做。
  網頁顯示 sp3 障礙物、真手時黃色手指跟著動：要改 RobotViewer，使用者要先跟手臂負責人討論。
- **假藥瓶改成每類一瓶（2026-11-18）**：`dry_run.bottles`（桌面座標 [右, 前]＋[直徑, 高]，立在桌面上）；要抓的是目標、其他的當障礙物。
  b1 右肩正前方、b2 左肩正前方（前 12）、b3／b4 各在外側 16 cm。取代舊的 bottle_base_link_m／bottle_size_m。
- **tray_2（左手）**：place_targets 改成 tray_1 = 桌面 右 25、前 15、掌心離桌 12（右手），tray_2 = 鏡像（左手），都還是暫定。
  任務管理選手時只留「藥瓶和放置點都搆得到」的手（抓起來不換手），沒有就 MAN-E-002。b1 → tray_2 會被擋。

## 還沒做的

1. task_manager.yaml 調完（見上一節）。`dry_run.fail` 已改回 `''`（2026-11-19）。
2. sp3_task_manager、ros2_task_plan_demo、COMMANDS.md、CLAUDE.md 尚未 commit（2026-11-17 15:08 本機已同步到 .11，由使用者 commit）。
3. 實機跑一次完整取放（流程見 COMMANDS.md「子三任務管理／接實機」）。
4. 使用者的「修改主計畫主控模擬」任務還沒開始（先問清楚要改什麼）。
5. `src/head_tf` 第 4 階段：動態 TF 接到 bottle_localization（改 launch 前先列出要改什麼給使用者看）；README 的建置段還寫 source ros2_adult，要更新。
6. 取放結果目前只在 result.message，沒有存檔（文件 S3-03「記錄取放結果」未完成）。
   使用者 2026-11-17 列為之後再做的：重試前先退開再辨識、放下後確認藥瓶放好、失敗或取消後的收尾。
7. bottle_localization、med_detect 不是使用者負責的，沒驗證過；URDF 原點對齊是 URDF 負責人的事，不管。
