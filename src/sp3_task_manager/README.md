# sp3_task_manager：子三任務管理

接主控的取放任務，依序呼叫藥瓶辨識、手臂、手，回報進度與結果。對應 S3-01 的「任務管理」模組。

## 對外介面（給主控）

| 名稱 | 型別 | 說明 |
|---|---|---|
| `/manip/pick_place` | Action `interfaces/action/ExecuteTaskPlan`（主控的型別） | 取放任務 |
| `/manip/status` | `std_msgs/String`（JSON），1 Hz | `state`：IDLE / BUSY / ERROR / ESTOP，另有 task_id、stage、last_error |
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
| ArmClient | `/arm_move`、`/arm_stand`、`/arm_speed_cmd`、`/arm_ik_status`（arm_ws 的 moveit_bridge）；`/joint_states` 檢查 driver 還活著 |
| HandClient | `/hand/cmd`、`/hand/status`（handpackage）；夾到「壓力到門檻」才算抓到 |

位置換算（base_link → 手臂量測座標）、抓取偏移、放置點、逾時、重試次數都在 `config/task_manager.yaml`。

## 執行

```bash
cd ~/nstc && colcon build --symlink-install --packages-select interfaces task_plan_demo sp3_task_manager
source install/setup.bash

# 不接硬體，先測流程與主控通訊（設定檔 dry_run.fail 可模擬各種失敗）
ros2 launch sp3_task_manager task_manager.launch.py dry_run:=true
ros2 run task_plan_demo client_gui          # 主控模擬器（src/drivers/ros2_task_plan_demo）

# 接實機：nstc.launch.py、arm_ws（sim.launch.py real:=true）、bottle_localization 都開著
ros2 launch sp3_task_manager task_manager.launch.py
```

單元測試（不需要 ROS）：`cd src/sp3_task_manager && python3 -m pytest test -q`
