# sp3_task_manager：子三任務管理

接主控的取放任務，依序呼叫藥瓶辨識、手臂、手，回報進度與結果。對應 S3-01 的「任務管理」模組。

## 對外介面（給主控）

| 名稱 | 型別 | 說明 |
|---|---|---|
| `/manip/pick_place` | Action `interfaces/action/ExecuteTaskPlan`（主控的型別） | 取放任務 |
| `/manip/target` | `std_msgs/String`（JSON），1 Hz | 要抓的藥瓶（給 RobotViewer 畫，不是障礙物）：`{"class", "objects": [stick]}`，空 = 沒有 |
| `/manip/status` | `std_msgs/String`（JSON），1 Hz＋每次 feedback | `state`：IDLE / BUSY / ERROR / ESTOP，另有 task_id、stage、last_error、feedback（最新一筆 feedback.status）、last_message（上一個任務的 result.message） |
| `/estop` | `std_msgs/Bool` | True = 急停（測試平台用，正式型別待定） |

### Goal：parameter_keys / parameter_values

| key | 必填 | 值 |
|---|---|---|
| `bottle_class` | ✓ | `b1`～`b4`（設定檔 `bottle_classes`） |
| `arm` |  | `auto`（預設）/ `left` / `right`，也接受 0 / 1 / 2 |
| `place_target` |  | 設定檔 `place_targets` 的名稱；不給 = 抓起來回站姿、不放 |
| `timeout_s` |  | 任務時限，預設 30 |
| `task_id` |  | `T-YYYYMMDD-NNNN`，不給就自動產生 |

不認得或重複的 key 會直接回失敗（MAN-E-002），不會默默忽略。忙碌中或急停中送來的任務會被拒絕（goal 不接受）。

### Feedback / Result

- `status`：`<階段號> <階段名>｜retry=<n>｜<說明>`，階段 0 辨識、1 規劃、2 接近、3 抓取、4 驗證、5 放置、6 退回（S3-05）
- `progress`：階段號 / 7
- `error_code`：0 成功；1～8 對應 MAN-E-001～008（逾時、資料超出範圍、找不到藥瓶、抓取失敗、關節狀態中斷、手部狀態中斷、驅動器錯誤、急停）。
  主控的 error_code 是 uint8，完整錯誤碼寫在 `message` 開頭，例如 `MAN-E-003 找不到藥瓶 …`
- 取消：接受，`success=False`、`error_code=0`、`message=已取消 …`

## 對內（測試平台）

全部包在 `adapters.py`，換正式硬體時只改這個檔。

| 轉接 | 用到的介面 |
|---|---|
| VisionClient | `/bottle_detections_3d`（bottle_localization，base_link，m） |
| ArmClient | `/arm_move`、`/arm_stand`、`/arm_speed_cmd`、`/arm_ik_obstacles`、`/arm_attach`、`/arm_stop`、`/arm_ik_status`（armpackage 的 moveit_bridge，只認 `cmd` 對得上的結果）；`/joint_states` 檢查 driver 還活著；`/arm_real_status`（real_arm_bridge）：實機跟隨不是 follow 就回 MAN-E-007，MoveIt 到了之後等 `settled` 才往下（設定檔 `real_arm.required: false` 可以只測 MoveIt） |
| 急停／中斷 | 急停時、手臂移動中被取消／逾時／狀態中斷時送 `/arm_stop`（取消 MoveIt 軌跡）＋`/arm_real_enable false`（實機停在最後的目標）；之後要重新打開實機跟隨 |
| HandClient | `/hand/cmd`、`/hand/status`（handpackage）；夾到「壓力到門檻」才算抓到 |

位置換算（base_link → 手臂量測座標）、抓取偏移、放置點、逾時、重試次數都在 `config/task_manager.yaml`。

障礙物（`table`、`obstacles`）：任務開始時把桌子（暫時寫死在設定檔）、找到藥瓶後再把**其他**藥瓶（要抓的那瓶不算）
發到 `/arm_ik_obstacles`（`{"source": "sp3", "objects": [...]}`，不會蓋掉網頁的物體），moveit_bridge 規劃、最後 10 cm 直線靠近都會檢查碰撞。
夾住後把藥瓶掛到手上（`/arm_attach`），放下、重試或任務失敗時拿掉。

可及範圍（`workspace`）：reach_test scan 實測的表（每個左右位置掌心最遠往前多少，左右手相同、中間內插），從上方抓用。
藥瓶不在任何一隻手的範圍內 = MAN-E-002；指定的手搆不到也是 MAN-E-002；auto 時只考慮搆得到的手。
桌面座標 [右, 前, 上]（cm）：原點 = 桌緣上、正對 base_link 左右中線的點，高度 = 桌面，拿捲尺量的值都用這套。

選手：主控指定 left / right 就用那隻；auto 時兩隻手的預抓、抓取位置一樣（`front`、`top`）就送 `arm: auto`，
由 moveit_bridge 依 IK 與碰撞挑（結果的 `arm` 欄位），`side` 才由任務管理照藥瓶左右依序試。

moveit_bridge 結果的 `reason` → 錯誤碼：`unreachable`、`joint_limit`、`bad_request` = MAN-E-002（不重試，auto 時換手）；
其他（`collision`、`plan_failed`、`exec_failed`…）= MAN-E-004（可重試）。

## 執行

```bash
cd ~/nstc && colcon build --symlink-install --packages-select interfaces task_plan_demo sp3_task_manager
source install/setup.bash

# 要模擬哪幾段寫在設定檔 dry_run.modules（vision / arm / hand 任意組合，[] = 全部接實機），
# launch 給 dry_run:= 就以 launch 為準。設定檔 dry_run.fail 可模擬各種失敗
ros2 launch sp3_task_manager task_manager.launch.py                    # 照設定檔
ros2 launch sp3_task_manager task_manager.launch.py dry_run:=true      # 全部模擬，測流程與主控通訊
ros2 launch sp3_task_manager task_manager.launch.py dry_run:=vision    # 假藥瓶，手臂和手真的動
ros2 run task_plan_demo client_gui          # 主控模擬器（src/drivers/ros2_task_plan_demo）

# 接實機：nstc.launch.py、arm_ws（sim.launch.py real:=true）、bottle_localization 都開著
ros2 launch sp3_task_manager task_manager.launch.py
```

可及範圍與避障測試（直接對 moveit_bridge，不經任務管理）：`ros2 run sp3_task_manager reach_test scan` / `obstacles`，用法見檔頭或 COMMANDS.md。

單元測試（不需要 ROS）：`cd src/sp3_task_manager && python3 -m pytest test -q`
