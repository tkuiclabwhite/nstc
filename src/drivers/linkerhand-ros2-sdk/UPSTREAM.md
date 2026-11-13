# linkerhand-ros2-sdk（vendored 第三方套件）

LinkerHand 官方 ROS2 SDK，直接複製進 `ros2_adult/src/`（做法比照 `zed-ros2-wrapper`，
不是 git submodule）。我們的手部功能寫在 `src/handpackage`，這裡盡量不改上游程式，
只改設定檔，這樣日後要跟上游更新比較容易。

- 上游：<https://github.com/linker-bot/linkerhand-ros2-sdk>
- 對應 commit：`0035f23`（Update pressure_diagram.py）
- SDK 版本：`setting.yaml` 的 `VERSION: 3.2.5`（repo 根目錄另有 `release_3.2.3.txt`，上游自己就不一致）
- 這份是從 172.17.121.11（測試用 AGX）的 `~/linkerhand-ros2-sdk` 複製過來的，
  已經含有下面那幾處在實機上調好的設定。`build/` `install/` `log/` 沒有複製
  （那是 aarch64 上編的，裡面都是 .11 的絕對路徑）。

## 我們改過的地方（相對於上游 0035f23）

全部都是設定，沒有動邏輯：

| 檔案 | 改了什麼 |
|---|---|
| `linker_hand_ros2_sdk/linker_hand_ros2_sdk/LinkerHand/config/setting.yaml` | `PASSWORD` 改成實機 sudo 密碼 |
| `linker_hand_ros2_sdk/launch/linker_hand_double.launch.py` | `hand_joint` → `O6`，CAN → `can2`(左) / `can3`(右) |
| `linker_hand_ros2_sdk/launch/linker_hand.launch.py` | `hand_type` → `right`、`hand_joint` → `O6`、CAN → `can2` |
| `gui_control/launch/gui_control.launch.py` | `hand_joint` → `O6`，把右手那段 Node 解除註解（官方預設只開左手） |
| `linker_hand_ros2_sdk/linker_hand_ros2_sdk/linker_hand.py` | `main()` 加一個 `embedded_version is None` 的檢查（唯一一處動到邏輯，見地雷 8） |

註：我們實際啟動是用 `src/handpackage/launch/hands.launch.py`，上面那兩個
`linker_hand*.launch.py` 保留原狀只當參考，平常不會用到。

## 已知地雷（踩過才知道，別再踩）

1. **一定要用 `colcon build --symlink-install`。**
   `linker_hand_ros2_sdk/setup.py` 裡把 `LinkerHand/config` 的 `data_files` 整段註解掉了，
   `setting.yaml`、`O6_positions.yaml` 不會被安裝。程式是用 `os.path.dirname(__file__)`
   往上找設定檔，只有 symlink-install 時才找得到；用一般 build 會在讀 yaml 時掛掉。

2. **`PASSWORD` 錯了會靜默失敗。**
   `LinkerHand/utils/open_can.py` 用 `sudo -S` 把密碼餵進去開 CAN，
   失敗是 `except: pass` 吞掉，不會報錯 —— 現象是「手完全不動但沒有任何錯誤訊息」。
   換機器（例如同步到 172.17.121.10）時務必改 `setting.yaml` 的 `PASSWORD`。
   `hands.launch.py` 有加啟動檢查，CAN 沒 UP 會印警告。

3. **`/cb_hand_setting_cmd` 不分左右手（上游 bug）。**
   `linker_hand.py` 的 `hand_setting_cb()` 裡，`hand_type == "left"` 和 `== "right"`
   兩個分支都是 `hand = self.api`，而且從來沒把 `params.hand_type` 跟節點自己的
   `self.hand_type` 比對。結果雙手同時跑時，對一隻手下的設定指令（設速度、設扭矩）
   **兩隻手都會執行**。
   我們的對策是在 `hands.launch.py` 裡 remap 成 `/cb_left_hand_setting_cmd` /
   `/cb_right_hand_setting_cmd`，不動上游程式。跟上游更新時記得這個 remap 還要留著。

4. **O6 這些功能沒有，網頁上不要做。**
   - `set_enable` / `set_disable`（上電／釋放）：只有 L21/L24/L25 的 CAN driver 有實作
   - `clear_faults`：O6 是空函式 `pass`
   - 電流：`get_current()` 回傳全 `-1`，`set_current()` 只對 L20 有效
   - 速度回讀：`get_speed()` 讀回來是 0（driver 自己註解「L6暂不支持读取实时速度」），
     所以介面上要顯示「最後下達值」而不是回讀值
   - 弧度控制 topic：`doc/Topic-Reference.md` 寫了 `/cb_*_hand_control_cmd_arc`，
     但 `linker_hand.py` 根本沒訂這個 topic。要用弧度得自己換算
     （`LinkerHand/utils/mapping.py` 有 O6 的表，左右手相同：max `[0.58,1.36,1.6,1.6,1.6,1.6]`）

5. **`gui_control/config/constants.py` 的 O6 `joint_names_en` 順序是錯的。**
   它把 index 4 寫成 pinky、5 寫成 ring，但 driver 的 `get_finger_order()` 和同檔案的
   中文 `joint_names` 都是 4=無名指、5=小拇指。以 driver 為準：
   `["thumb_cmc_pitch","thumb_cmc_yaw","index_mcp_pitch","middle_mcp_pitch","ring_mcp_pitch","pinky_mcp_pitch"]`

6. **`LinkerHand/config/O6_positions.yaml` 是死檔。**
   `LoadWriteYaml` 只定義了 L7/L10/L20/L21/L25 的路徑，沒有 O6，所以這個檔案
   程式從來沒讀過。裡面左右手的拇指橫擺值不一樣，但那不代表實機需要鏡像轉換
   （`constants.py` 的 O6 預設動作本來就是左右手共用同一組數值）。

7. **手不回應查詢時，原本會吐一個看不懂的 TypeError。**
   `linker_hand.py` 的 `main()` 直接 `len(node.embedded_version)`，但
   `get_version()` 在手沒回應時會回 `None`，於是 crash 成
   `TypeError: object of type 'NoneType' has no len()`，指向 SDK 內部，
   完全看不出真正的問題在硬體。
   已在本地加上 `None` 檢查，改成印出可行動的訊息再乾淨退出。
   2026-10-08 在 172.17.121.11 上就是這個情形，實測確認過：
   CAN 介面 ERROR-ACTIVE、berr-counter 0、TX 37 筆全成功（代表封包有被 ACK），
   但 RX 永遠是 0；用 `cansend` 在 SDK 之外手工發 10 種查詢
   （0x01/0x02/0x05/0x33/0x35/0x36/0xB1/0x64/0xC0/0xC2）、兩個 ID（0x28/0x27）、
   兩條匯流排（can2/can3），手一筆都沒回 —— 所以是硬體側的問題，不是程式。
   自己驗的方法：
   ```bash
   candump can2            # 一個終端
   cansend can2 028#64     # 另一個終端，正常會看到手回一筆 028 的版本資料
   ```

8. **訂閱壓感會讓控制迴路變鈍。**
   `linker_hand.py` 的 `run()` 是 100 Hz timer 但裡面串了一堆 `time.sleep()`，
   矩陣壓感是用 `run_count` 輪流讀每根手指。只要有人訂 `matrix_touch` 或
   `matrix_touch_mass`（兩者共用同一個 gate），這些 CAN 讀取就會插進控制迴路。
   所以 `handpackage` 只在夾持模式啟用時才訂 `_mass`。

## 實機量到的規格（2026-10，172.17.121.11，雙 O6）

- `touch_type: 4`（SN 第 5 段 `F`）→ 每指矩陣 **10 列 × 4 行**，五指都有
- **沒有掌心感測器**：`palm_matrix` 回 `[-1]`，`palm_mass` 回 `[-1]`（注意是 list 不是數字）
- `matrix_touch_mass` 的 `unit` 標 `"g"` 但其實是 10×4=40 格 0~255 的總和，**上限 10200，不是公克**
- 所有 topic 發布率約 **55 Hz**，`matrix_touch` 的 JSON 約 **887 bytes**
- 嵌入式版本 `version: [3,1,3]`
- 可用的回讀：`motor_temperature`、`fault`、`torque`（`torque` driver 註解寫 not supported 但有數值會變動，含義待確認）
- 左手 CAN ID `0x28`、右手 `0x27`
- .11 上 `can0`/`can1` 是 AGX 內建 CAN（沒用到），`can2`/`can3` 才是 PEAK USB-CAN
  （`/dev/pcanusb32`、`/dev/pcanusb33`）。**換機器編號不保證一樣**，所以
  `hands.launch.py` 把 CAN 介面做成 launch argument

## 怎麼跟上游更新

```bash
# 另外 clone 一份乾淨的上游來比對（不要直接蓋）
git clone https://github.com/linker-bot/linkerhand-ros2-sdk.git /tmp/lh-upstream
diff -r --exclude=build --exclude=install --exclude=log --exclude=__pycache__ --exclude=.git \
     /tmp/lh-upstream  ~/ros2_adult/src/linkerhand-ros2-sdk
```

確認上游的改動不會影響上面第 3、4 點之後再合進來，然後更新本檔的 commit 編號。
