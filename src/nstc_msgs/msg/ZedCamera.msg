# ZED 相機參數即時調整（對應 /zed/zed_node 的 video.* parameter）
# whitebalance_temperature 為 ROS parameter 原值 28~65（ZED SDK 內部 x100）
# denoising 為 ZED X 專屬參數，範圍 0~100
bool auto_exposure_gain
int32 exposure
int32 gain
bool auto_whitebalance
int32 whitebalance_temperature
int32 saturation
int32 denoising
int32 gamma
