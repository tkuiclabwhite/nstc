# drivers：測試平台的硬體驅動

這裡的套件只為了在測試平台上把硬體接起來，正式硬體到位後會被替換掉，
之後改成吃主控轉來的 topic（手臂關節、手部、相機）。src 底下其他套件會繼續留著。

| 套件 | 測試平台硬體 | 之後改成 |
|---|---|---|
| motor_control | Dynamixel XM540 手臂、頭部馬達（U2D2） | 子一、喜特的手臂驅動 |
| linkerhand-ros2-sdk | Linker Hand O6（CAN） | 喜特的手部控制板 |
| zed-ros2-wrapper | ZED X Mini | 總計畫提供的 RGB-D |

colcon 會往下找套件，放在這層資料夾不影響建置。
