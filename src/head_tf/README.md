# head_tf：由頭部馬達刻度計算相機外參

`bottle_localization/config/camera_extrinsics.yaml` 是頭在某一組刻度時校正的，頭一轉外參就不對了。
這個套件的目標：讀頭部馬達刻度，算出當下的相機外參，發布成動態 TF。

2026-11-16 起 ZED 裝在**新平台**（seven_axis_arm_head_urdf，三軸脖子：28 轉頭、29 低頭、30 歪頭），
見下一節。之後的「為什麼要用刻度算」以下是**舊機器人**（兩軸頭，28 pan、29 tilt）時寫的，
收集工具的用法和輸出格式兩邊共用，舊機器人的模型（head_model.py、fit_head_model.py、sweep_01）留著當紀錄。

新平台進度：

| 階段 | 內容 | 狀態 |
|---|---|---|
| 1 | 確認刻度和 URDF 角度的對應（零點、正負號） | **完成**（2026-11-18，`config/neck_head.yaml`） |
| 2 | 頭部模型：URDF 連桿 + 由外參反推的相機固定位置 | **完成**（`head_tf/neck_model.py`） |
| 3 | 改 head_sweep 支援三軸、驗證工具 `check_neck_model.py` | 工具完成，**等機器人上收資料** |
| 4 | 動態 TF 節點，接到 bottle_localization | 還沒開始（改 launch 前會先列出要改什麼） |

## 新平台（三軸脖子）

### 模型

```
相機姿態(刻度) = FK(關節角) · M
關節角 = sign × (刻度 − 2048) × 360° / 4096
M = FK(基準刻度)⁻¹ · camera_extrinsics.yaml
```

- FK 是 URDF 從 `base_link` 經 neck_base → neckYaw → neckPitch → headRoll 到 `head_Link` 的連桿，數字抄在
  `config/neck_head.yaml`（測試會拿 URDF 原檔比對）。
- M 是相機固定在頭上的位置，用基準刻度時校正好的外參反推，所以**在基準刻度時結果和 camera_extrinsics.yaml 完全一樣**，
  頭轉了才用 URDF 算相對的移動。相機實際裝得和 URDF 設計值不同也沒關係。
- **重新校正外參時，要同時把 neck_head.yaml 的 `reference_ticks` 改成校正當時的刻度。**
- 零點的影響：歪頭固定時，歪頭、低頭的零點完全不影響；轉頭零點差 1°，允許範圍角落約差 0.16°、1 mm。

| 馬達 | URDF 關節 | 刻度變大 | sign | 基準刻度 | 允許範圍 |
|---|---|---|---|---|---|
| 28（HeadPackage id 1） | neckYawJoint | 往左轉 | +1 | 2047 | 1877～2218（±15°） |
| 29（id 2） | neckPitchJoint | 往上看 | −1 | 1708（低頭 29.9°） | 1594～1820（低頭 40°～20°） |
| 30（id 3） | headRollJoint | 往左歪 | −1 | 2024 | 2014～2034（固定） |

正負號是 2026-11-18 用眼睛看的；29 號用 table_plane 量到的相機 pitch 30.4° 對過（換算 29.9°）。
範圍是 2026-11-18 和使用者定的：抓取時頭不動，這是為了之後的場景先做。

檢查設定：`python3 -m head_tf.neck_model`（在 head_tf 資料夾）印出基準姿勢、相機裝在頭上的位置和 URDF 設計值的差、
允許範圍四個角的相機姿態。2026-11-18：位置差 3 mm 內、roll −1.86°（30 號零點或相機裝歪，歪頭固定所以不影響）、pitch +0.49°。

### 在機器人上收資料

```bash
# 桌機：把套件同步到 .11
rsync -av --exclude __pycache__ --exclude .pytest_cache ~/nstc_ws/nstc/src/head_tf iclab@172.17.121.11:~/nstc/src/

# .11：建置（nstc 有自己的 nstc_msgs，不用 source ros2_adult）
source /opt/ros/humble/setup.bash
cd ~/nstc && colcon build --symlink-install --packages-select head_tf && source install/setup.bash

# .11：nstc.launch.py 和 bottle_detect_v2 開著，任務管理不要開、網頁頭部拉條不要動
ros2 run head_tf head_sweep neck_01

# 桌機：傳回來
scp -r iclab@172.17.121.11:~/head_sweeps/neck_01 ~/nstc_ws/head_sweeps/
```

- 預設姿勢從 `neck_head.yaml` 算：轉頭 1877 / 2047 / 2218 × 低頭 1594 / 1651 / 1708 / 1764 / 1820，頭尾加量基準，共 17 個，約 2 分鐘。
  `--pan`、`--tilt`、`--reference` 可以改，但不能超出允許範圍。
- 開始前 30 號要在 2014～2034，不在就不跑（模型假設歪頭固定）。30 號不轉，只記錄；收集中變了超過 3 格會在結束時警告。
- 網頁（ImageProcessInterface）開著時 rosbridge 也算 `/Head_Topic` 的發布者，工具會拒絕；確定沒人會動頭就加 `--force`。
- 結束時的快速檢查在新平台應該是「tilt：刻度變大 → 往上看」「pan：刻度變大 → 往左轉」。

### 驗證（桌機）

```bash
cd ~/nstc_ws/nstc/src/head_tf
env -u PYTHONPATH ~/nstc_ws/nstc/foundationpose_trial/.venv/bin/python tools/check_neck_model.py ~/nstc_ws/head_sweeps/neck_01
```

設定檔用資料夾裡 `robot_config/` 的（收集當時機器人上的），沒有才用 repo 的。都和頭尾的基準姿勢比
（外參本身的誤差、IMU 裝歪、桌子微斜相減就消掉），印每個姿勢和整體的：

| 項目 | 看什麼 |
|---|---|
| 藥罐水平位置（動態 TF / 固定外參） | 同一個藥罐在各姿勢算出的位置和基準差幾 mm，直接是抓取會差多少。固定外參 = 沒有動態 TF 的對照 |
| IMU roll / pitch 殘差 | pitch 對不上 = 低頭角度不準；轉頭時 roll 跟著變 = 轉頭軸不鉛直（支架歪） |
| 桌面高度殘差 | 相機上下位置不準 |

結論用藥罐位置的 95 百分位：≤ 5 mm 直接用、≤ 10 mm 堪用、更大要修正。**模型完全正確也不會是 0**：
bottle_localization 的中心算法在藥罐位於畫面不同位置時本身有幾 mm 偏差，模擬中（轉頭 15°、低頭 ±10°）
95% 約 4 mm、最大約 6 mm，大瓶子在畫面邊緣最明顯。IMU 和桌面高度殘差不受這個影響。

## 為什麼要用刻度算，以及精度要求

用 `geometry.localize` 對 robot_upright_01 實測，外參有誤差時輸出位置偏多少：

| 外參誤差 | 輸出偏移 |
|---|---|
| pitch 1° | 約 5 mm |
| yaw 1° | 約 5 mm |
| roll 1° | 約 2.5 mm |
| 相機 x 或 y 差 1 cm | 10 mm |
| 相機 z | 不影響（z 由桌高算） |

所以角度要準到 0.5° 左右、水平位置 3 mm 左右。ZED 的 IMU 量不到左右轉（pan）和相機位置，只能當檢查用；
主要靠馬達刻度加上頭部模型，這也是其他人形機器人頭部相機的常見作法。

頭部模型（階段 3 擬合的形式）：

```
相機姿態(pan, tilt) = 繞 pan 軸轉 Δpan ∘ 繞 tilt 軸轉 Δtilt ∘ 基準姿態（現在的 camera_extrinsics.yaml）
Δ角度 = (刻度 − 基準刻度) × 360 / 4096 × 方向（±1）
```

pan 軸、tilt 軸的位置和方向先用捲尺量（`config/head_axes.yaml`：螺絲孔在 pan 軸前方 6.5 cm、在 tilt 軸前方 1 cm 上方 5 cm），再用 `head_sweep` 收的資料修正。模型和手臂分開：
之後 ZED 搬到新手臂平台（seven_axis_arm_urdf_v2）時，現在的 `base_link` 就是新 URDF 的 `Rarm_base_link`，
只要多一段固定平移 (0, −0.1454, 0)，頭的參數重新量、重新收資料就好。

## 刻度與角度（舊機器人）

馬達一圈 4096 格，1 格 = 0.088°，2048 是正中間。新平台見上面「新平台」。

| 馬達 | 基準 | 抓取時允許的範圍 | 收集工具預設量的刻度 |
|---|---|---|---|
| pan（28 號，HeadPackage id 1） | 2048 | 1934～2162（±10°），超出就要先轉正 | 1934、2048、2162 |
| tilt（29 號，HeadPackage id 2） | 2300 | 2186～2414（±10°） | 2186、2243、2300、2357、2414 |

工具本身最多只允許基準 ±341 格（±30°），打錯數字會被擋下來。哪個方向刻度變大，收完資料會自動判斷（見下面的快速檢查）。

## 建置（機器人上）

見上面「新平台 / 在機器人上收資料」。頭部指令用 `nstc_msgs/HeadPackage`（找不到才用舊機器人的 `tku_msgs`），
nstc 不用再 source ros2_adult。

## head_sweep：收集校正資料

### 開始前

1. 開 `nstc.launch.py`（ZED 和馬達 driver）和 `bottle_detect_v2`。`bottle_localization` 開著沒關係。
2. **任務管理（sp3_task_manager）不要開、網頁的頭部拉條不要動**：它們也會發 `/Head_Topic`，工具偵測到會拒絕執行。
3. 頭部扭力開著、歪頭（30 號）在基準刻度。整個過程**不要動機器人、桌子和瓶子**。
4. 桌上放 2～4 個藥罐，分散在畫面中間附近，頭轉到範圍邊緣時也要看得到。
5. 畫面下方那塊（深度圖高 60～98%、寬 20～80%）是桌面擬合區，瓶子的框會自動挖掉，但盡量不要有滑鼠、鍵盤、手臂或桌緣。

### 執行

```bash
ros2 run head_tf head_sweep neck_01                     # 預設：轉頭 3 個 x 低頭 5 個，頭尾加量基準，共 17 個姿勢，約 2 分鐘
ros2 run head_tf head_sweep neck_02 --tilt 1651 1708 1764 --note "瓶子 b1 b2 b4，桌高 80.5"
ros2 run head_tf head_sweep neck_03 --manual            # 工具不轉頭，自己轉好按 Enter 記錄，q 結束
ros2 run head_tf head_sweep --help                      # 全部參數
```

工具先列出計畫、目前刻度和看到的瓶子，按 Enter 才開始。每個姿勢：轉頭 → 等刻度停穩 → 再等 1.5 秒 → 記錄 2 秒。
轉速預設 40（約 9 rpm）；0 在 Dynamixel 是「不限速」，工具不允許。
**結束、出錯或按 Ctrl+C 時，頭會轉回開始時的刻度**（最多等 5 秒，這段時間再按 Ctrl+C 不理）。

每個姿勢印一行：

```
[ 5/17] pan 2162→2161 tilt 2300→2302 | 桌面 roll +0.70° pitch +31.00° 高 33.0 cm | IMU roll +0.70° pitch +31.00° | 瓶子 b1 b2 b3
```

`2162→2161` 是「下的指令→實際讀到的刻度」，分析都用實際刻度。行尾的 ⚠：

| 警告 | 意思 |
|---|---|
| 沒到目標刻度 | 5 秒內沒到目標 ±12 格（例如重力讓 tilt 差一點）。資料照樣能用 |
| 頭沒停穩 | 3 秒內刻度一直在變。看是不是有別的東西在動頭 |
| 桌面取樣區只有 xx% 在平面上 | 擬合區混到別的東西，看 png 是什麼 |
| 記錄時刻度在跳 | 記錄期間刻度標準差 > 1.5 格 |

頭完全不動（目標差 40 格以上卻沒動）時工具會停止，多半是扭力沒開或有別的程式在控制頭。

### 結束時的快速檢查

```
tilt：刻度變大 → 往上看（桌面 pitch 變化 / tilt 角度變化 = -1.00，17 個姿勢；理想值 ±1）
pan：刻度變大 → 往左轉（瓶子方位角變化 / pan 角度變化 = +0.90，51 筆；相機往下看，所以會比 1 小一點）
歪頭（30 號）：2024～2024（變化 0 格），沒有動
頭尾基準姿勢的差：IMU pitch +0.13° roll +0.07°，桌面高度 +0.4 mm（桌面角度 pitch +0.21°，只供參考），瓶子 b1 2.1 mm …
  頭尾一致，中間沒有東西動過
```

- **tilt 比例**應該接近 ±1。差很多代表 tilt 軸和相機的左右軸不平行，或桌面擬合有問題。
- **pan 方向**要有瓶子在基準姿勢和其他姿勢都被看到才算得出來。
- **頭尾差**：IMU 的 pitch 或 roll 超過 0.3°、桌面高度超過 3 mm、或瓶子超過 5 mm，代表中間有東西動過，這次資料要重收。
  桌面角度不拿來判斷：深度的小誤差在桌面角度上會放大，sweep_01 因此誤報過一次（當時擬合區也比較小）。

### 傳回桌機

```bash
mkdir -p ~/nstc_ws/head_sweeps
scp -r iclab@172.17.121.11:~/head_sweeps/neck_01 ~/nstc_ws/head_sweeps/
```

資料不要放進 `nstc/`（`git_sync.sh` 會整包加進 git），一次約 26 MB。

### 輸出格式

存在 `~/head_sweeps/<名稱>/`：

| 檔案 | 內容 |
|---|---|
| `meta.json` | 參數、相機內參（K、P、尺寸、frame）、開始時刻度、姿勢清單、狀態（finished / interrupted / error） |
| `postures.csv` | 每個姿勢一列，欄位見下 |
| `bottles.csv` | 每個姿勢每個瓶子一列 |
| `posture_XX.npz` | `depth`（深度中位數圖，公尺，float32）、`k`、`size`、`joints`（記錄期間每筆 t, pan, tilt）、`roll`（t, 30 號刻度；新平台）、`imu`（t, roll, pitch, ax, ay, az, wx, wy, wz）、`detections`（YOLO 原始 JSON） |
| `posture_XX.png` | 彩色影像（沒有 OpenCV 時改存在 npz 的 `color`） |
| `robot_config/` | 當下 bottle_localization 的 camera_extrinsics.yaml、robot_reference.yaml、params.yaml、bottle_dims.yaml（用 bottle_tuner 改過、沒存檔的值不在裡面），和 head_tf 的 neck_head.yaml |

`postures.csv` 主要欄位：

| 欄位 | 意思 |
|---|---|
| `kind` | `ref`（基準姿勢）、`grid`、`manual` |
| `pan_cmd`、`tilt_cmd` | 下的刻度（手動模式是 nan） |
| `pan_mean`、`tilt_mean`（和 std / min / max） | 記錄期間實際讀到的刻度 |
| `roll_mean`、`roll_min`、`roll_max` | 歪頭（30 號）的刻度（新平台；舊機器人是 nan） |
| `reached`、`still`、`move_s` | 有沒有到目標、有沒有停穩、轉頭花幾秒 |
| `table_roll_deg`、`table_pitch_deg`、`table_height_m` | 相機相對桌面的姿態和光心高度（同 table_plane.py 的慣例，往下看 pitch 為正） |
| `normal_x/y/z`、`plane_inliers`、`plane_rms_mm` | 桌面法向量（optical frame）、擬合品質 |
| `imu_roll_deg`、`imu_pitch_deg`、`acc_x/y/z`、`gyro_rms_dps` | ZED IMU 平均（同 zed_imu_average.py 的算法；IMU 晶片和鏡頭之間有出廠安裝角度，只看變化量） |

`bottles.csv`：框的中位數 `x1 y1 x2 y2`、表面點 `x y z`（optical frame，公尺）、`bearing_deg`（往右為正）、`elevation_deg`（往下為正）。
表面點和 bottle_localization 一樣是「框內縮 1/4、深度中位數 ±3 cm 的點取中位數」，在弧面上會落在瓶身正面後方 1～4 mm。

## 舊機器人階段 3：擬合頭部模型（桌機）

```bash
cd ~/nstc_ws/nstc/src/head_tf
env -u PYTHONPATH ~/nstc_ws/nstc/foundationpose_trial/.venv/bin/python tools/fit_head_model.py \
    ~/nstc_ws/head_sweeps/sweep_01 --out config/head_model.yaml
```

可以一次給好幾個資料夾（每次的瓶子位置、桌高各自擬合）。需要 scipy，所以在桌機跑；結果寫進 `config/head_model.yaml`，機器人只讀這個檔。

擬合用到：

- **IMU 的 roll / pitch**：相機姿態，最準（約 0.05°）。
- **瓶子的水平位置**：照 bottle_localization 的算法算，瓶子整個過程沒動，所以每個姿勢都要算出同一點。
- **相機光心離桌面的高度**：從存下來的深度圖重新擬合。
- **手量的轉軸位置**：只當很寬的起始值（±2 cm）。

### sweep_01 的結果（2026-11-10）

| 參數 | 擬合值 | 手量值 |
|---|---|---|
| pan 軸方向 | **往前傾 7.3°**（±0.1°），左右 0° | 假設垂直 |
| tilt 軸方向 | 幾乎水平（前後偏 0.4°） | 假設水平 |
| 螺絲孔在 pan 軸的 | 前方 8.4 cm、右方 0.2 cm | 前方 6.5 cm |
| 螺絲孔在 tilt 軸的 | 前方 8.1 cm、上方 3.2 cm | 前方 1 cm、上方 5 cm |
| 刻度比例（檢查用） | pan 0.995（±0.013）、tilt 1.009（±0.001） | 1 |

殘差：IMU 0.06°、桌面高度 1.4 mm、瓶子水平位置 1.4 mm。

驗證：同一個瓶子在 15 個非基準姿勢算出的水平位置，和基準姿勢差多少（mm）：

| 方法 | 中位數 | 95% | 最大 |
|---|---|---|---|
| 固定外參（現在的做法，頭一轉就錯） | 59.4 | 78.2 | 87.5 |
| 手量的轉軸、pan 當垂直 | 8.4 | 11.8 | 12.0 |
| 擬合的模型 | 1.4 | 2.6 | 3.5 |
| **擬合的模型（留一驗證）** | **1.5** | **2.6** | **3.6** |

留一驗證：每次拿掉一個姿勢擬合，再用那個模型算被拿掉的姿勢，比較接近實際使用時的誤差。

### 要知道的限制

- **pan 軸往前傾 7.3°**，所以頭左右轉時相機會跟著歪（±10° 時 roll 約 ±1.3°）。機器人站著時身體會微微往前傾（2026-11-10 確認），
  這 7.3° 至少有一部分是身體前傾。每次站的傾角如果不同，相機角度也會跟著變，要用另一天收的資料驗證，見下一點。
- **tilt 軸位置以擬合值為準**（2026-11-10 確認）：手量值和資料差很多，手量轉軸中心不容易準。
- **只有一次收集**：模型是用同一組資料擬合和驗證的。之後換一天、換瓶子位置再收一次，
  直接拿 head_model.yaml 去算新的資料（不重新擬合），才是真正的驗證。
- **tilt 軸的依據**：資料很明確，不用手量值也得到前方 8.3 cm、上方 3.1 cm。
  光看桌面高度就看得出來：頭從 tilt 2190 轉到 2415（約 20°），光心離桌面從約 365 mm 降到 333 mm；
  如果 tilt 軸只在相機後面 1 cm，高度只會變約 5 mm。
- **pan 軸的前後距離會被 bottle_localization 的算法偏差影響**：相機往下看，偏離畫面中央的直立瓶子會變斜，
  框跟著偏，算出的位置每偏 1° 方位角約橫移 0.16 mm。這和「頭左右轉時相機橫移」分不開，
  模擬測試裡這會讓 pan 軸的前後距離偏約 1 cm（對定位結果約 1.6 mm）。擬合的目標是讓 bottle_localization 的輸出在各個頭部姿勢一致，所以這個偏差一起被吸收。
- **只在收集過的範圍內保證**：pan 1934～2162、tilt 2186～2414。
- **基準姿態的絕對準度**還是看 `camera_extrinsics.yaml`。頭部模型只負責「頭轉了以後相對基準怎麼變」。

## 測試（桌機）

```bash
cd ~/nstc_ws/nstc/src/head_tf
env -u PYTHONPATH ~/nstc_ws/nstc/foundationpose_trial/.venv/bin/python -B -m pytest test/ -q -p no:cacheprovider
```

`test/sim_head.py` 是模擬器：已知的頭部模型、桌面、圓柱形瓶子，算出深度圖、YOLO 框和 IMU。測試確認：

- 每個姿勢的桌面 roll / pitch / 高度和模擬值相差 0.1° / 1 mm 以內。
- 方向判斷（含反向的頭）正確。
- 頭不動、刻度停止更新、重力讓 tilt 差幾格、中途移動瓶子、手動模式都有對應的處理。
- 頭部模型和模擬器（各自獨立寫的）算出的姿態完全相同。
- 擬合：量測沒有偏差時所有參數都擬合回真值（方向 0.05°、位置 1 mm 以內）；用 bottle_localization 的算法量瓶子時，
  定位漂移和用真值模型一樣好，只有 pan 軸前後距離會偏（見上面的限制）。

另外在桌機用真的 ROS topic 跑過整個工具：一個假機器人發 `/joint_states`、ZED 深度 / 內參 / 彩色 / IMU、YOLO，
頭部指令改走替代 topic，因為桌機沒有 tku_msgs。確認了這些：

- 17 個姿勢約 80 秒跑完，每個姿勢收到 100 筆刻度、9 張深度圖、200 筆 IMU。
- Ctrl+C、SIGTERM、還有別的程式在發 `/Head_Topic`、沒有 ZED、參數錯誤都處理正確。
- 中斷後頭都會轉回開始的刻度。

新平台（2026-11-18）：

- `test/test_neck_model.py`：URDF 連桿和設定檔一致、零位和 URDF README 的相機位置一致、基準刻度重現外參、
  正負號、轉頭 / 低頭繞正確的軸、零點的影響、範圍檢查、實際設定檔的一致性（左眼離桌 52.4 cm）。
- `test/sim_neck.py` + `test/test_check_neck_model.py`：相機姿態由「真的」脖子模型決定的模擬器跑完整收集流程，
  驗證工具在模型正確時判「可以直接用」（藥罐 95% 4 mm 內，固定外參差 20 公分），支架往前傾 2° 時 IMU roll 殘差抓得到；
  30 號有記錄、動了會警告；sweep_plan 的姿勢和範圍。
- 用真的 ROS topic 和假機器人（發 28、29、30）跑完 17 個姿勢、頭轉回原位，驗證工具讀得到輸出；30 號不在基準時拒絕執行。

## 檔案

| 檔案 | 內容 |
|---|---|
| `head_tf/sweep_core.py` | 純計算：姿勢清單、深度轉換、桌面擬合、瓶子表面點、IMU、快速檢查（不依賴 ROS） |
| `head_tf/sweep_runner.py` | 流程：轉頭、等停穩、記錄、存檔（不依賴 ROS，透過 io 介面） |
| `head_tf/head_sweep.py` | ROS 節點與指令列 |
| `head_tf/head_model.py` | 舊機器人的頭部模型：刻度 -> zed_camera_link 的姿態（純 numpy） |
| `head_tf/neck_model.py` | 新平台的頭部模型（URDF 三軸脖子）、`sweep_plan()`（head_sweep 的預設姿勢） |
| `tools/check_neck_model.py` | 新平台：用收集的資料驗證頭部模型（桌機，需要 scipy） |
| `config/neck_head.yaml` | 新平台：URDF 連桿、馬達對應、基準刻度、允許範圍 |
| `tools/fit_head_model.py` | 擬合頭部模型（桌機，需要 scipy） |
| `config/head_axes.yaml` | 舊機器人：手量的轉軸位置（擬合的起始值） |
| `config/head_model.yaml` | 舊機器人：擬合出來的頭部模型 |
| `test/` | 單元測試與模擬器 |
