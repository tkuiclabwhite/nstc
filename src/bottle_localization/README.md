# bottle_localization

把 YOLO 節點（`bottle_detect_v2`）的 2D 偵測框，結合 ZED X Mini 的深度圖（`/zed/zed_node/depth/depth_registered`），
算出每個藥罐**瓶身中心**在 `base_link` 下的 3D 位置，發布給手臂抓取端。

精度目標：`base_link` 下絕對誤差 ≤ 1 cm。

> **上線前要確認的量測值**（少了節點或 launch 會報錯／警告）：
> 1. `config/bottle_dims.yaml`：四種藥罐的直徑與高度
> 2. `config/camera_extrinsics.yaml`：`base_link → zed_camera_link`
> 3. `config/params.yaml` 的 `table_height_cm`：桌面離地高度（cm），換桌子時要改
> 4. `config/robot_reference.yaml`：`base_link` 名稱與量測基準點（換手臂時要改）
>
> 執行中可以用 `bottle_tuner` 直接調桌高與外參並存回設定檔（見「執行中調整桌高與外參」）。
>
> 另外，抓取時數位變焦必須固定在 `params.yaml` 的 `zoom`（目前是 1.0），見「數位變焦」一節。

---

## 檔案

| 檔案 | 內容 |
|---|---|
| `bottle_localization/geometry.py` | 幾何計算，不依賴 ROS（內參換算、步驟 1–7、平滑、設定檔檢查） |
| `bottle_localization/bottle_localizer.py` | ROS 節點：訂閱、時間配對、TF 查詢、呼叫 `geometry.py`、發布；執行中可改桌高 |
| `bottle_localization/camera_tf_publisher.py` | ROS 節點：發相機外參的 TF（`zed_camera_link → base_link`），執行中可改外參 |
| `bottle_localization/bottle_tuner.py` | 終端機介面：執行中調桌高與外參、存回設定檔 |
| `bottle_localization/bottle_viewer.py` | 終端機即時顯示輸出的座標 |
| `bottle_localization/rviz_relay.py` | 把 TF 與 marker 轉到桌機的 RViz |
| `config/params.yaml` | 節點參數（節點不帶預設值，少一個就啟動失敗） |
| `config/bottle_dims.yaml` | 藥罐尺寸（公尺） |
| `config/camera_extrinsics.yaml` | 相機外參 `base_link → zed_camera_link` |
| `config/robot_reference.yaml` | `base_link` 名稱與量測基準點（換手臂時改這個檔） |
| `launch/bottle_localization.launch.py` | 只啟動 `bottle_localizer` 與 `camera_tf_publisher` |
| `tools/` | 校正與實測工具（不會被安裝，直接用 python3 執行，見「準確度校正流程」） |
| `test/` | pytest：單元測試、合成場景、工具與設定檔的檢查 |

---

## 建置與執行（Jetson，ROS 2 Humble）

```bash
# 依賴：vision_msgs 若沒裝
ros2 pkg prefix vision_msgs || sudo apt install ros-humble-vision-msgs

source /opt/ros/humble/setup.bash
source ~/ros2_adult/install/setup.bash      # 一定要先 source，原因見下方
cd ~/nstc
colcon build --symlink-install --packages-select bottle_localization
source install/setup.bash
```

本套件本身只用標準訊息，但**在 `~/nstc` 建置任何套件之前都要先 source `ros2_adult`**：
每次 `colcon build` 都會重新產生整個 workspace 的 `install/setup.bash`，把建置當下有 source 的
workspace 記進去。沒 source `ros2_adult` 就建置的話，之後 `source ~/nstc/install/setup.bash`
不會再載入 `ros2_adult`，`med_detect` 會找不到 `tku_msgs`。確認方式：
`grep COLCON_CURRENT_PREFIX= ~/nstc/install/setup.bash` 應該同時看到 `/opt/ros/humble` 與 `ros2_adult/install`。

`--symlink-install` 讓 `config/` 裡的 yaml 改完重新 launch 就生效，不用重建；`bottle_tuner` 存檔時也是直接改到 `src` 裡的檔案。
新增程式（`setup.py` 的 `console_scripts`）之後要重新 `colcon build` 一次，`ros2 run` 才找得到。

執行順序：

```bash
# 1. 機器人既有系統（ZED、depth_process_node 等），同一時間只能有一個程式開相機
ros2 launch usb_cam camera.launch.py
# 2. YOLO
ros2 run med_detect bottle_detect_v2
# 3. 本套件（不會啟動 ZED、YOLO 或深度節點）
ros2 launch bottle_localization bottle_localization.launch.py
```

四個設定檔都可以用 launch 參數換掉：`params_file:=`、`dims_file:=`、`extrinsics_file:=`、`reference_file:=`。

### 即時查看輸出的座標

```bash
ros2 run bottle_localization bottle_viewer                       # 另開一個終端機
ros2 run bottle_localization bottle_viewer --ros-args -p window:=5.0
```

終端機畫面每秒更新 5 次（`refresh_hz`），每個類別一行：

| 欄位 | 意思 |
|---|---|
| x / y / z | 最新的瓶身中心（`base_link`，公尺） |
| 信心 | 這一筆的 YOLO 信心值 |
| 多久前 | 最後一次出現在輸出裡是幾秒前 |
| 出現率 | 最近 `window` 秒（預設 3）的訊息裡，有幾成包含這個類別。偏低代表 YOLO 時有時無，或被本節點跳過 |
| 抖動 | 最近 `window` 秒內 x、y 的標準差（mm）。瓶子靜止時就是量測的重複性 |

### 執行中調整桌高與外參（`bottle_tuner`）

```bash
ros2 run bottle_localization bottle_tuner      # 另開一個終端機（SSH 也可以），launch 要先在跑
```

```
   參數                    目前生效         設定檔        步距
 > 桌高                    74.5 cm          74.5 cm       0.5 cm
   外參 x（往前）          88.0 mm        * 87.0 mm       1 mm
   ...
即時座標（base_link，m）
   b2     x +0.352   y -0.099   z -0.141    0.1 秒前
```

| 按鍵 | 動作 |
|---|---|
| ↑ ↓ | 選參數 |
| ← → | 減少／增加一個步距（外參 0.5／1／5 mm、0.05／0.1／0.5°，桌高 0.1／0.5／1 cm） |
| `[` `]` | 換小／大一級的步距 |
| Enter | 直接輸入數值（量到的桌高用這個） |
| `s` | 存回設定檔（只改那幾行的數字，註解保留，舊檔留一份 `.bak`） |
| `r` | 放棄修改，還原成設定檔裡的值 |
| `q` | 離開；有沒存的修改會問要存檔、還原，還是直接離開 |

- 調整**立刻生效**（不用重新 launch），`*` 表示已經生效但還沒存檔；要按 `s` 才會寫回設定檔，下次 launch 才會沿用。
- 外參 x、y、z 以 mm、角度以度顯示，存檔時換回 m、rad。各參數對輸出的影響：x、y 讓所有瓶子一起前後、左右移；
  yaw 讓瓶子繞相機轉（越遠偏越多）；pitch、roll 每 0.1° 大約讓瓶子前後、左右移 0.5 mm；z 對 x、y 幾乎沒有影響。
- 只有桌高和外參能在執行中改，其他參數（`ros2 param set` 也一樣）會被拒絕，請改設定檔後重新 launch。
- TUI 需要鍵盤輸入，launch 底下的程式收不到，所以不包在 launch 裡。
- launch 時用 `params_file:=`／`extrinsics_file:=` 換過檔案的話，`bottle_tuner` 也要用 `--params-file`／`--extrinsics-file` 指定同一個。

### 在桌機用 RViz 看

機器人的 `~/.bashrc` 設了 `ROS_DOMAIN_ID=10`、`ROS_LOCALHOST_ONLY=1`：ROS 2 只在機器人內部通訊，
桌機收不到。實測 Humble / Fast DDS 上 `ROS_LOCALHOST_ONLY=1` 與 `=0` 的節點彼此不通，
所以不能只把本套件改成對外。改用轉發程式，機器人其他設定不動：

source /opt/ros/humble/setup.bash
rviz2 -d /home/iclab/nstc_ws/nstc/src/bottle_localization/rviz/bottle_localization.rviz

```bash
# 機器人上（bottle_localization.launch.py 在跑的同時）
ros2 run bottle_localization rviz_relay                 # 轉 /tf、/tf_static、/bottle_markers 到 domain 0
ros2 run bottle_localization rviz_relay --point-cloud   # 連 ZED 點雲一起轉（很吃頻寬）

# 桌機上（ROS_DOMAIN_ID 不設 = 0）
source /opt/ros/humble/setup.bash
rviz2 -d <套件>/rviz/bottle_localization.rviz
```

轉發程式在同一個行程裡開兩個 context（一個只在本機、一個對外），單向原樣轉發；
輸出 domain 不能和機器人的一樣，避免資料繞回來。

YOLO 有框到、輸出卻沒有那個瓶子時，加 `log_level:=debug` 會印出每個框被跳過的原因
（`框寬不符`／`框高不符` 是步驟 7 的合理性檢查、`有效深度點少於` 是步驟 2、`碰到畫面邊緣` 是前置過濾）。

---

## 介面

### 輸入（2026-10 從原始碼查證）

| 資料 | Topic | 說明 |
|---|---|---|
| YOLO 偵測 | `/med_detect/bottles` | `std_msgs/String`（JSON）。`stamp` 在 JSON 裡，沿用原始影像的 `header.stamp`；`bbox = [xmin, ymin, xmax, ymax]`（整數、含端點、960×600 座標系）；`label` 是 `b1`–`b4` |
| 深度圖 | `/zed/zed_node/depth/depth_registered` | `sensor_msgs/Image` 32FC1，公尺，NaN／±inf = 無效。沿光軸的 Z（ZED `MEASURE::DEPTH`），和 YOLO 用的左眼影像是同一次擷取、同一個 stamp。也支援 `/depth_mm`（mono16 公釐，見下方） |
| 內參 | `/zed/zed_node/left/camera_info` | ZED 發布解析度（960×600）的 K |
| TF | — | `base_link` ↔ `zed_left_camera_optical_frame` |

### 輸出

| Topic | 型別 | 內容 |
|---|---|---|
| `/bottle_detections_3d` | `vision_msgs/Detection3DArray` | 給抓取端 |
| `/bottle_markers` | `visualization_msgs/MarkerArray` | RViz 檢查用：與瓶子同尺寸的圓柱 + 文字（每次先送 `DELETEALL`） |

每個 `Detection3D`：

| 欄位 | 值 |
|---|---|
| `header.frame_id` | `base_link`（`target_frame`） |
| `header.stamp` | 原始影像的 stamp（JSON 的 `stamp`） |
| `id`、`results[0].hypothesis.class_id` | 類別名稱（`b1`–`b4`） |
| `results[0].hypothesis.score` | 這一幀的 YOLO 信心值 |
| `results[0].pose.pose.position`、`bbox.center.position` | 瓶身中心（多幀平滑後） |
| `bbox.center.orientation` | 單位四元數 |
| `bbox.size` | (D, D, H) |

偵測結果處理完但沒有任何有效偵測時，照樣發一個空陣列（header 正確），表示目前看不到瓶子。
因時間配對失敗、TF 查詢失敗、還沒收到 `camera_info` 而跳過的那一筆，則不發布。

### 深度圖為什麼用 ZED wrapper 的，而不是 `/depth_mm`

2026-10 上機量測（`ros2 topic hz` / `ros2 topic delay`）：

| Topic | 頻率 | 說明 |
|---|---|---|
| `/depth_mm` | 約 21 Hz（ZED 是 30 Hz） | `depth_process_node` 處理不完，約三成的幀被丟掉；從擷取到送達約 146 ms |
| `/med_detect/bottles` | 約 17.8 Hz | 偵測結果約在擷取後 80 ms 到達 |

偵測結果到達時，同一幀的 `/depth_mm` 還要再晚約 66 ms 才到（或根本被丟掉），
時間差超過 `sync_tolerance` 而每一筆都被跳過。

`depth_process_node` 的輸入就是 wrapper 的 `depth_registered`，它只做數位變焦的裁切和公尺→公釐。
抓取時 zoom 固定 1.0，裁切等於沒做，所以 wrapper 的深度圖和 `/depth_mm` 逐像素對齊。
wrapper 同一次擷取同時發布左眼影像和深度圖，YOLO 還要推論，偵測結果到達時同一幀的深度一定已經在緩衝裡。

要改回 `/depth_mm`（例如抓取時必須用 zoom ≠ 1）：`depth_topic: /depth_mm`、`depth_is_zoomed: true`。

---

## 需要量測／填入的值

### `config/bottle_dims.yaml`（公尺）

key 必須和 YOLO 類別名稱完全一致：`b1`、`b2`、`b3`、`b4`（`best(1).onnx` 的 metadata）。
`diameter` 是瓶身直徑 D，`height` 是直立時桌面到瓶蓋頂的總高 H。
任何值 ≤ 0、缺少類別或多出 `class_names` 以外的類別時，節點啟動就報錯並列出問題。

### `config/camera_extrinsics.yaml`

`zed_camera_link` 在 `base_link` 下的位置（m）與姿態（rad，與 URDF／`static_transform_publisher` 相同的
roll → pitch → yaw 固定軸慣例）。x、y、z 往前、往左、往上為正；yaw 左右轉頭（往左為正）、
pitch 點頭（往下看為正）、roll 歪頭（左邊抬高為正）。`base_link` 的名稱來自 `robot_reference.yaml`。

- `zed_camera_link` 是 **ZED X Mini 底部的螺絲孔**（ZED URDF 的根）。
- 左眼光心在 `zed_camera_link` 的 (−0.010, +0.025, +0.016) m（ZED URDF 的 zedxm 數值，由 wrapper 發布，不用自己加）。
- 量完把 `measured` 改成 `true`，launch 就不再印警告。

### 桌高：`params.yaml` 的 `table_height_cm`

桌面離地高度（cm），用捲尺量。節點換算成桌面在 `base_link` 下的高度：

```
z_table = (table_height_cm − reference_height_cm) / 100 + reference_point.z
```

瓶身中心的 z 直接用 `z_table + H/2`，不用深度估計值。正式的桌子 80.5 cm（`z_table` −0.1203），
測試用的桌子 74.5 cm（−0.1803）。

### `config/robot_reference.yaml`（換手臂時改）

| 欄位 | 目前值 | 說明 |
|---|---|---|
| `base_frame` | `base_link` | 輸出座標系，也是外參的 parent |
| `reference_name` | 右肩關節點 | 量測工具的提示文字 |
| `reference_point` | [−0.01375, −0.0436, 0.01475] | 基準點在 `base_frame` 下的位置（m），抄自 URDF（`armurdf4_with_hand.urdf` 的 `RshoulderJoint`） |
| `reference_height_cm` | 94.0 | 基準點離地高度（cm），實際量 |

基準點是量外參（Δx、Δy、Δz）、實測瓶子位置時在機器人身上實際量的那一點。launch（`target_frame`、桌高換算）
和 `tools/` 的量測工具都讀這個檔，程式裡不寫死。

### 換手臂（`base_link` 改變）時

1. 改 `robot_reference.yaml`：新的 `base_frame` 名稱、基準點在新 `base_frame` 下的位置（從新的 URDF 抄）、基準點離地高度。
2. 照「準確度校正流程」第 2 步重量外參（Δx、Δy、Δz 從新的基準點量），`extrinsics_calc.py` 會讀新的基準點。
3. 重新 launch，用 `bottle_tuner` 或實測工具確認。

---

## 準確度校正流程

工具在 `tools/`（不會被安裝，直接用 python3 執行）。

1. **擺好抓取姿勢**：頭低到抓取角度（記下 `/joint_states` 的馬達 28、29 刻度，之後每次抓取都轉回這組）、
   軀幹直立、機器人不動。
2. **量外參**（都在抓取姿勢下）：
   - 基準點（`robot_reference.yaml`，目前是右肩關節點）→ 鏡頭中點（前面玻璃上兩顆鏡頭圓心連線的中點）沿 前／左／上 的距離 Δx、Δy、Δz（cm）
   - roll、pitch：**優先用 `python3 tools/table_plane.py`**，從深度圖擬合桌面，直接量相機相對桌面的角度（取樣區域要是空的桌面）。
     `tools/zed_imu_average.py` 讀的是 IMU 晶片自己的方向，和鏡頭之間有出廠安裝角度（實測 roll −0.96°、pitch −0.24°、yaw 0.54°，見 `/zed/zed_node/left_cam_imu_transform`），只適合粗略核對
   - `python3 tools/zed_stl_front.py`（先 source `ros2_adult`）：前面玻璃到螺絲孔的距離 a
   - `python3 tools/extrinsics_calc.py --dx … --dy … --dz … --pitch … --roll … --a …`，
     把印出來的 7 行貼進 `config/camera_extrinsics.yaml`，重新 launch
3. **桌高**：用捲尺量桌面離地高度，填 `params.yaml` 的 `table_height_cm`，或在 `bottle_tuner` 裡按 Enter 輸入後按 `s` 存檔。
   基準點離地高度（`robot_reference.yaml` 的 `reference_height_cm`）也要是實際量的。
   外參填好後，可用 RViz 的 Publish Point 點桌面核對。
4. **RViz 檢查**：點雲裡桌面應該是水平的、高度在 `z_table`；圓柱應立在桌上、和瓶子重疊。
5. **實測與微調**（`tools/position_check.py`）：
   - **先開啟兩個選用修正**。以實機的相機姿態（離桌面 35 cm、低頭 31°）模擬，外參完全正確時，
     照原規格的演算法本身在 28～40 cm 的近距離就有不規則的誤差（b2 最大 8 mm、b1 最大 14 mm），
     開啟選用修正後都在 2 mm 以內。演算法誤差每個點不一樣，會混進擬合、把外參帶歪。
   - **形狀檢查（`tools/pair_check.py`，不用量瓶子的位置）**：兩個不同類別的瓶子中間平放一把尺，
     尺的兩端貼著瓶身、對準兩瓶圓心，兩瓶圓心的真實距離 = 尺的全長 + 兩瓶半徑。
     在工作範圍內前後、左右、斜放各擺幾次，比較相機量到的兩瓶距離：
     `python3 tools/pair_check.py --labels b2 b3 --length <尺的全長cm>`
     外參的小誤差只會讓所有點一起平移、旋轉，不影響兩瓶距離，所以這一步只檢查深度與定位演算法。
     形狀不對（某個方向被拉長或壓縮）時，擬合外參也修不好，要先找原因。
   - 真值的量法（建議用 A）：
     - **A. 桌邊兩點量距離**：左右肩關節點各垂一條線，量線到桌邊的垂直距離 a_R、a_L，
       在桌邊標出兩個垂足 P0（右肩正前方）、P0_L（左肩正前方），量 P0 到 P0_L 的距離。
       之後瓶子隨便放，每個點用捲尺從 P0、P0_L 各量一次到瓶身的最短距離，工具加上半徑、
       算出瓶子圓心。桌子不用擺正，也不用在桌上畫點：
       `python3 tools/position_check.py --label b2 --edge <a_R> <a_L> <P0到P0_L>`
     - **B. 在桌上畫點**：桌子擺正後，以 P0 為基準畫好點，每個點輸入
       「瓶子圓心相對右肩關節點往前、往左的水平距離（cm）」（往前 = a + 離桌邊的距離）：
       `python3 tools/position_check.py --label b2`
   - 同一瓶（建議 b2）放 6～8 個點，前後左右分散、涵蓋實際抓取的範圍。工具每個點收 3 秒取平均、算誤差。
     真值和相機輸出差超過 `--max-diff`（預設 3 cm）的點會先問要不要記錄，多半是量錯或輸入錯。
   - 按 Enter 結束後，工具擬合 `真值 ≈ Rz(δ)·輸出 + d`，印出修正後的 `x`、`y`、`yaw`。
     合成測試（真值帶 ±2 mm 量測誤差、外參誤差 yaw ±2°、位置 ±1 cm）：用 b2 擬合後最大誤差 1.7～1.8 mm。
   - 用 `bottle_tuner` 填進外參並存檔（或改 `camera_extrinsics.yaml` 後重新 launch），在**不同的點**用新的 CSV 再測一次確認（`--out ~/check_after.csv`），
     其他瓶子也各放幾個點，確認整體誤差。
   - 已存的 CSV 可在任何電腦重新分析：`python3 tools/position_check.py --analyze points.csv --extrinsics config/camera_extrinsics.yaml`

## TF

ZED wrapper 預設會發布 `map → odom → zed_camera_link`（positional tracking 的 `publish_tf`，
`camera.launch.py` 沒有關掉），`zed_camera_link → zed_camera_center → zed_left_camera_frame →
zed_left_camera_optical_frame` 則由 ZED 的 URDF 靜態發布。

`zed_camera_link` 已經有 parent（`odom`），不能再發 `base_link → zed_camera_link`。
所以 `camera_tf_publisher` 把量到的 `base_link → zed_camera_link` **反過來**，發 `zed_camera_link → base_link`
（`/tf_static`；執行中外參改了會重新發布）：

```
map → odom → zed_camera_link ─┬→ zed_camera_center → zed_left_camera_frame → zed_left_camera_optical_frame
                              └→ base_link            （本套件，static）
```

`base_link ↔ zed_left_camera_optical_frame` 的查詢只經過 static transform，結果和直接發
`base_link → zed_camera_link` 完全相同，也不受 ZED 里程計漂移影響。

日後改用頭部馬達做動態 TF（`base_link → 頸部 → 頭 → zed_camera_link`）時，必須讓 ZED 不再發
`odom → zed_camera_link`（ZED launch 加 `publish_tf:=false`；策略 API 用的是 `/zed/zed_node/odom`
topic，不是 TF，不受影響），並移除本 launch 的 `camera_tf_publisher`。節點本身已經用影像 stamp 查 TF，不用改。

頭部馬達的回授（給日後的動態 TF）：`motor_control/driver_node` 以 50 Hz 發 `/joint_states`，
pan 是 `name = "28"`、tilt 是 `"29"`，`position` 是 Dynamixel 原始刻度（一圈 4096，約 0.088°/格），
不是弧度；策略 API 以 2048 為中心，但 2048 是否等於正前方／水平、正負方向，需要實測。

---

## 內參換算

### 解析度鏈（2026-10 從原始碼查證）

| 環節 | 尺寸 | 說明 |
|---|---|---|
| ZED X Mini 擷取 | 1920×1200 | `zedxm.yaml`：`grab_resolution: HD1200` |
| wrapper 發布 | 960×600 | `common_stereo.yaml`：`pub_downscale_factor: 2.0`。影像、深度、`camera_info` 都在這個解析度，**K 已是 960×600 的值** |
| `image.py` / `params_1.yaml` | — | 960×540 + 上下補 30 列黑邊只發生在 **USB 相機**模式；ZED 模式不補黑 |
| `depth_process_node` → `/depth_mm` | 960×600 | 只做數位變焦（中心裁切 + `INTER_NEAREST` 放大回原尺寸），不縮放、不補黑 |
| `bottle_detect_v2` | 960×600 | 訂閱未變焦的 `left/image_rect_color`，框換算到 960×600 時套用自己的 zoom 值 |

結論：YOLO 的框和 `/depth_mm` 在同一個 960×600 像素座標系上，**前提是兩個節點的 zoom 一樣**。
zoom = 1.0 時，wrapper 的 `depth_registered` 也是同一個座標系（`/depth_mm` 就是由它裁切而來）。

### 公式（`geometry.detection_intrinsics()`）

一般情況分兩段，順序與上游一致：

1. 等比縮放 s，再補黑邊（`bottle_detect.map_to_strategy()`）：

   ```
   fx₁ = s·fx        cx₁ = s·cx + letterbox_left
   fy₁ = s·fy        cy₁ = s·cy + letterbox_top
   ```

2. 數位變焦 z（`depth_process_node`）：中心裁成 `new_w = int(W/z)`、`new_h = int(H/z)`，
   裁切原點 `x0 = (W − new_w) // 2`、`y0 = (H − new_h) // 2`，再放大回 W×H。
   原圖像素 u 在放大後的位置是 `(u − x0)·W/new_w`，所以：

   ```
   fx' = fx₁·W/new_w     cx' = (cx₁ − x0)·W/new_w
   fy' = fy₁·H/new_h     cy' = (cy₁ − y0)·H/new_h
   ```

目前的設定（ZED X Mini）：`image_scale = 1.0`、`letterbox_top = letterbox_left = 0`、`zoom = 1.0`，
兩段都是恆等變換，**直接使用 `camera_info` 的 K**。節點啟動後第一筆偵測會在 log 印出實際使用的內參。

2026-10 上機實測（960×600）：`fx = fy = 364.87`、`cx = 472.73`、`cy = 308.39`。
舊程式 `med_detect.py` 的 `FOCAL_PX = 440` 偏了約 20%，主點也不在畫面正中央。

不使用舊程式 `med_detect.py` 寫死的 `FOCAL_PX = 440` 與畫面中心主點。

### 數位變焦

- `depth_process_node` 開機時從 `ZedCameraSet.ini` 讀 zoom；`bottle_detect_v2` 開機時固定是 1.0，
  只靠 `/Zoom_In_Topic` 更新（不讀 ini、topic 也不是 latched）。兩者可能不同步，
  不同步時框會落在深度圖上整塊錯的位置，而 JSON 沒有帶 zoom 值，本節點無法察覺。
- 抓取時固定 zoom：`params.yaml` 的 `zoom`、使用中策略的 `ZedCameraSet.ini` 的 `zoomin`、
  以及兩個上游節點實際在用的值必須一致（建議都是 1.0）。
- `zoom != 1` 時不支援黑邊（`letterbox_*` 必須是 0），啟動時會擋下來。
- `depth_is_zoomed: false`（ZED wrapper 的深度圖，沒有裁切）時，`zoom` 必須是 1.0，否則框和深度會對到不同的地方，啟動時會擋下來。

---

## 演算法（每一幀、每個偵測框）

1. **框內中央區域**：四邊各內縮 1/4，截斷在影像內。
2. **過濾雜點**：去掉 0 → 算 Z 中位數 `Z_med` → 只留 `|Z − Z_med| < r`（r = D/2）→ 少於 `min_valid_points` 就跳過。
3. **反投影**：`X = (u − cx')·Z/fx'`、`Y = (v − cy')·Z/fy'`，X、Y、Z 各取中位數得表面點 `P_s`（optical frame）。
4. **轉到 base_link**：`lookup_transform(target_frame, optical_frame, 影像 stamp, timeout)`；相機位置 `C` 是轉換的平移。
5. **水平半徑修正**：`d_h = (P_s − C).xy`，`center.xy = P_s.xy + r·d_h/‖d_h‖`（不沿完整視線方向推）。
6. **垂直位置**：`center.z = z_table + H/2`。
7. **合理性檢查**（`enable_size_check`，**已偏離原始規格**）：把直徑 D、高 H 的圓柱放在算出來的中心、
   投影回影像得到預期的框，和 YOLO 框的寬、高比；任一項相對誤差超過 `size_check_ratio` 就跳過
   （debug log 記錄是哪一項、預期幾 px、實際幾 px）。啟用 `enable_workspace_bounds` 時中心落在範圍外也跳過。

   原規格是 `Z_size = fx'·D/w` 對 `Z_med`、`h_expect = fy'·H/Z_med` 對 `h`，只在畫面中央成立。
   ZED 是廣角，瓶子放在畫面左右兩側時框會被透視拉寬：實測 b3 在偏離中心約 30° 的位置，
   原公式相對誤差 0.37，正確的瓶子被擋掉（只乘 1/cos 偏角也修不乾淨，因為相機是往下看的）。
   投影的做法在合成測試中，偏離 ±38° 內框寬、框高誤差都在 3% 以內；類別認錯（b3 當成 b1）時
   框寬誤差約 60%，照樣擋得下來。這一步需要中心，所以移到步驟 6 之後。
8. **多幀平滑**：每個類別保留 `smoothing_frames` 個 base_link 中心，發布 x、y 的中位數；
   超過 `smoothing_timeout` 沒有新的有效偵測就清空。同一類別多個框取信心值最高的並警告。

前置：信心值低於 `score_threshold`、類別不在 `bottle_dims.yaml`、框碰到畫面邊緣（`edge_margin`，扣掉黑邊）的框直接跳過。
時間配對：深度圖連同 `header.stamp` 存進長度 `depth_buffer_size` 的緩衝，偵測結果來時用 JSON 的 `stamp`
找最接近的一張；相差超過 `sync_tolerance` 就跳過並警告。

### 選用修正（不在原始規格內，預設關閉）

合成測試發現照原始規格有兩個系統性偏差，俯角越大越明顯：

| 偏差 | 原因 | 修正（參數） |
|---|---|---|
| 縱向（中心偏遠） | 相機往下看時框的中央區域會包含**瓶蓋頂面**。頂面點離相機的距離和正面差不多，步驟 2 濾不掉，但水平位置在正面後方，把 `P_s` 往後拉。D = 8.4 cm、俯角 45° 時中央區域有四成以上是頂面。 | `enable_top_surface_filter`：把點轉到 base_link，去掉高度 ≥ `z_table + H − top_surface_margin` 的點 |
| 橫向 | 相機往下看、瓶子不在畫面正中時，瓶子在影像裡是斜的，框的中央區域取到的弧段左右不對稱（俯角 45°、偏 15° 時中位角度偏 6°）。 | `enable_symmetric_arc`：以目前的中心估計，只留繞中軸「正對相機 ± 半寬」的點重算，迭代 `symmetric_arc_iterations` 次 |

兩個修正都只篩選步驟 2 留下的點，步驟 3–8 不變。剩下約 1 mm 的縱向偏差來自弧面取中位數（點在弧上，中位數略在弧內側）。

---

## 參數（`config/params.yaml`）

| 參數 | 目前值 | 說明 |
|---|---|---|
| `detection_topic` | `/med_detect/bottles` | |
| `depth_topic` | `/zed/zed_node/depth/depth_registered` | 支援 32FC1（公尺）、mono16／16UC1（公釐） |
| `depth_is_zoomed` | false | 深度圖是否已套用和框相同的數位變焦；wrapper 為 false、`/depth_mm` 為 true |
| `camera_info_topic` | `/zed/zed_node/left/camera_info` | |
| `class_names` | `[b1, b2, b3, b4]` | 必須都在 `bottle_dims.yaml` 裡 |
| `image_scale` | 1.0 | 原生 → 偵測座標系的縮放比 s |
| `letterbox_top` / `letterbox_left` | 0 / 0 | 黑邊列數／行數（視為上下、左右對稱） |
| `zoom` | 1.0 | 抓取時固定的數位變焦倍率 |
| `table_height_cm` | 80.5 | 桌面離地（cm），換算成 `z_table`；執行中可改（`bottle_tuner`） |
| `score_threshold` | 0.5 | `bottle_detect_v2` 本身已用 0.70 過濾 |
| `edge_margin` | 2 | 框離邊緣幾個像素內算碰到邊緣 |
| `min_valid_points` | 20 | |
| `sync_tolerance` | 0.1 | s。wrapper 深度圖比偵測結果晚到約 40 ms，0.05 會跳過約兩成（實測輸出 14.4 Hz／YOLO 17.8 Hz），暫時放寬 |
| `depth_buffer_size` | 10 | 30 fps 時涵蓋約 0.33 s，必須大於 YOLO 延遲（實測約 80 ms） |
| `tf_timeout` | 0.1 | s |
| `smoothing_frames` | 5 | |
| `smoothing_timeout` | 1.0 | s |
| `enable_size_check` / `size_check_ratio` | true / 0.3 | |
| `enable_workspace_bounds` | false | |
| `workspace_x_min` … `y_max` | 0.0 | m，base_link |
| `enable_top_surface_filter` / `top_surface_margin` | false / 0.01 | 選用修正 |
| `enable_symmetric_arc` / `symmetric_arc_iterations` / `symmetric_arc_trim_pct` | false / 3 / 2.0 | 選用修正 |
| `marker_rgba` / `marker_text_size` | [0.1, 0.8, 0.2, 0.6] / 0.03 | RViz |
| `warn_throttle_sec` | 2.0 | 同一個警告最快幾秒印一次 |

`bottle_dims_file`（`dims_file:=`）、`target_frame`、`reference_height_cm`、`reference_z`（`robot_reference.yaml`）由 launch 帶入，
不寫在 `params.yaml`。小數參數寫成整數（例如 `0`）也可以，節點會轉型。
執行中只有 `table_height_cm` 可以改，其他參數改了會被拒絕。

---

## 測試

```bash
cd ~/nstc/src/bottle_localization
python3 -m pytest test/ -v
python3 -m pytest test/ -v -s -k "noise_distribution or spec_table"   # 印出誤差表
```

不需要 ROS 也不需要相機。合成場景：桌面 + 直立圓柱（側面與頂面），光線追蹤產生深度圖（z-buffer 遮擋），
照 `/depth_mm` 的方式截斷成公釐（另有一項用 ZED wrapper 的 float32 + NaN 格式），框取物體可見像素的外接矩形。距離 45 cm，俯角 0/20/35/45°，
瓶子在畫面左／中／右（側向 ±12 cm），兩種尺寸（D 5.0 × H 9.0、D 8.4 × H 9.3 cm），有／無 2 cm 外的鄰瓶。
合成用的內參（fx = 360）是假設值。

### 無雜訊（48 種情況，水平誤差；z 由構造保證等於 `z_table + H/2`）

| 設定 | 中位數 | 最大 | 超過 2 mm |
|---|---|---|---|
| 照原始規格 | 1.55 mm | 9.07 mm（大罐、45°、右） | 18 / 48 |
| 開啟兩個選用修正 | 1.06 mm | 1.93 mm | 0 / 48 |

照原始規格時，相機水平（俯角 0）的所有情況都 < 2 mm；誤差隨俯角與瓶子大小增加。

### 加雜訊

雜訊模型：深度 1% 高斯（50 cm 時 5 mm）+ 5% 破洞 + 輪廓飛點 + 公釐截斷 + 框每邊 ±3 px。
48 組 × 20 次 × 5 幀。

| 設定 | 單幀 中位數 / p95 / 最大 | 5 幀中位數 中位數 / p95 / 最大 |
|---|---|---|
| 照原始規格 | 2.68 / 10.04 / 13.07 mm | 2.00 / 9.31 / 12.08 mm |
| 開啟兩個選用修正 | 1.67 / 3.95 / 6.48 mm | 1.20 / 2.82 / 4.49 mm |

這是對 ZED 雜訊的粗略假設，實機誤差一定更大，請以上機量測為準。

---

## 上機檢查

```bash
ros2 topic echo --once /zed/zed_node/left/camera_info        # 960x600、K、frame_id
ros2 topic echo --once /zed/zed_node/depth/depth_registered --no-arr   # 960x600、32FC1
ros2 topic hz /zed/zed_node/depth/depth_registered           # 應接近 30
ros2 topic echo /Zoom_In_Topic                               # 有沒有人發、值多少
ros2 run tf2_tools view_frames                               # 應看到 zed_camera_link → base_link
ros2 topic hz /med_detect/bottles
ros2 topic hz /bottle_detections_3d                          # 應接近 YOLO 的頻率
```

- 深度圖不補黑：`np.count_nonzero(depth[0:30, :])` 應該**不是 0**（舊程式第 6(c) 點的「應為 0」是 USB 模式）。
- 開機 log：`depth_process_node` 的 `[ZedCameraSet.ini] zoom=`、`bottle_detect_v2` 的 `模型:`、本節點的內參。
- 持續出現「偵測與深度時間差 … 跳過（偵測比最新深度新）」：深度圖比偵測結果晚到，先確認 `depth_topic` 是 wrapper 的深度圖（`/depth_mm` 會這樣，見「深度圖為什麼用 ZED wrapper 的」）。
- 持續出現「…（偵測比最新深度舊）」：YOLO 延遲大於深度緩衝涵蓋的時間，加大 `depth_buffer_size`。
- RViz：Fixed Frame 設 `base_link`，加 `/bottle_markers`，圓柱應和實際瓶子重疊。
- 用捲尺量幾個瓶子的實際中心，和 `/bottle_detections_3d` 比對（目標 ≤ 1 cm）。

---

## 已知限制

- **zoom 同步**：見「數位變焦」，本節點無法偵測上游兩個節點的 zoom 不一致。
- **時間配對**：wrapper 深度圖從擷取到送達約 124 ms，比偵測結果（約 80 ms）晚到，偵測到達時最新的深度圖是 1～2 幀前，所以 `sync_tolerance` 放寬到 0.1，配到的是 1～2 幀前的深度。頭固定、瓶子靜止時沒有影響；手臂伸進畫面等移動中的情況可能有影響，正規的修法是讓偵測結果等同一幀的深度圖（尚未實作）。
- **深度來源與 zoom**：用 wrapper 的深度圖時 zoom 只能是 1.0；抓取時必須用 zoom ≠ 1 的話，要改回 `/depth_mm`，並接受它的頻率與延遲（見上方量測）。
- **瓶子必須立在桌上**：z 直接用 `z_table + H/2`，不看深度。合理性檢查（步驟 7）把圓柱「立在桌面上」投影成預期的框，瓶子被拿起來、或放在別的高度時框對不上而被跳過，不會輸出錯誤的高度給抓取端。只抬高一兩公分時可能還在容許範圍內，此時 z 會差那一兩公分。
- **瓶子形狀**：演算法假設直立圓柱、瓶身直徑 D。瓶肩或瓶蓋比瓶身細時，若中央區域取到那些部位，中心會偏後；`enable_top_surface_filter` 只去掉最上面 `top_surface_margin` 的高度。
- **深度截斷**：只有改回 `/depth_mm` 時才有：它是公尺 × 1000 直接截斷成 uint16，深度最多偏小 1 mm。
- **相機正上方**：相機在瓶子正上方時水平方向無法決定，該框跳過。
