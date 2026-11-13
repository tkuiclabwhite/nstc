# handpackage 測試

全部不需要真的手，裸 Humble 就能跑。檔名刻意都不是 `test_*.py`，
`colcon test` / `pytest` 不會自動收集（它們需要另一個終端先把節點跑起來）。

兩邊的 `ROS_DOMAIN_ID` 要一致。想跟正在跑的真手隔開，兩邊都另外設一個：

```bash
export ROS_DOMAIN_ID=77
```

## ⚠️ 不要同時跑 fake_hands 和前三支測試

`smoke_hands` / `smoke_page_contract` / `smoke_grip` 自己就會發假的
`/cb_*_hand_state`、`/cb_*_hand_info`、`/cb_*_hand_matrix_touch_mass`。
如果 `fake_hands` 同時在跑，兩邊在同一個 topic 上互相蓋，測試會以很難理解的
方式失敗（位置對不上、online 判定跳動）。同理，接著真手時也不要跑這三支。

分組是：

* `smoke_hands` / `smoke_page_contract` / `smoke_grip` → 只跑 `handpackage`
* `smoke_fake_shapes` / `smoke_integration` → 要跑 `fake_hands`

## ⚠️ 每一支 smoke 都要用「剛啟動的」handpackage

節點有「相同值不重發」的去重（省 CAN 頻寬）。如果連續幾支測試共用同一個節點
行程，後面那支會看到前面留下的目標值，於是「送了指令卻沒有收到命令」——
失敗訊息完全看不出原因（實際遇過：`smoke_hands` 跑完後 `smoke_page_contract`
的 speed / preset both 會假性失敗）。每一支之間把節點關掉再開。

| 檔案 | 測什麼 | 需要先跑 |
|---|---|---|
| `unit_hand_motion.py` | `hand_motion.py` 的純邏輯：檔案讀寫、sector 快取、位元封包解析、播放器時序、單手切半、以及**網頁 Save → 寫檔 → Read → 網頁重建模型**的 offset 對位 | 不用，連 ROS 都不用 |
| `unit_play.py` | 在行程內直接驅動節點物件，測 `_apply_sector` / `_play_tick` / `_start_players` / `_cmd_play`：絕對與相對的套用、雙手獨立時間軸、單手播放、loop、播完回 manual、手動介入中斷 | 不用，自己建節點 |
| `smoke_hands.py` | handpackage 自己的行為：模式仲裁、雙手同步、`-1` 哨符、速度夾限、扭矩分手 topic、緊急停止、壞輸入不會讓節點死 | `handpackage` |
| `smoke_page_contract.py` | 把 `HandControlInterface.html` 裡每一個 `sendCmd` 的 payload 原樣重建丟給節點，確認介面合約沒斷 | `handpackage` |
| `smoke_history.py` | 執行歷史：Undo 要讓手真的回到執行前的位置、Redo、Clear、預設動作自帶速度 | `handpackage` |
| `smoke_grip.py` | 夾持迴路，壓力值由測試直接設定所以門檻時機完全可控。17 組情境 56 項檢查 | `handpackage` |
| `smoke_fake_shapes.py` | `fake_hands` 發的資料形狀跟實機一致（避免拿失真的假資料驗網頁） | `fake_hands` |
| `smoke_integration.py` | `fake_hands` + `handpackage` 真閉迴路：夾持會不會停在合理位置 | 兩個都要 |
| `probe_current.py` | **實機量測**，不是測試：O6 的電流回授（CAN frame 0x36）拿不拿得到、讀得多快、會不會隨負載變。順便印出 `effort` 其實是轉矩**上限**的回讀 | 真的手 + CAN，且**要先關掉** `handpackage` |

另外 `hurocup_interface/check_pages.py` 檢查三個手部頁面的結構（JS 括號平衡、
事件函式、寫死的 element id、class 有沒有 CSS、標籤平衡）。這台和機器人都沒有
JS 引擎可用，所以那是改完頁面後唯一能做的自動檢查：

```bash
python3 hurocup_interface/check_pages.py
```

`fake_hands.py` 不是測試，是假的 linker_hand_sdk —— 沒有硬體時用它把三個網頁
跑起來看。它也會吃 `control_cmd`，所以 slider 和夾持都能實際操作。

```bash
# 沒有手，但想看網頁
python3 src/handpackage/test/fake_hands.py      # 終端 1
ros2 run handpackage handpackage                # 終端 2
ros2 run rosbridge_server rosbridge_websocket   # 終端 3
python3 hurocup_interface/http_server.py        # 終端 4
# 瀏覽器開 http://localhost:9999/HandControlInterface.html
```
