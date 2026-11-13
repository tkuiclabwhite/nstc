cmake_minimum_required(VERSION 3.8)
project(nstc_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 從 ros2_adult 的 tku_msgs 只挑出 nstc 用到的訊息（2026-11-12）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ButtonColorForm.msg"
  "msg/DepthValue.msg"
  "msg/Dio.msg"
  "msg/DrawImage.msg"
  "msg/HSVValue.msg"
  "msg/HeadPackage.msg"
  "msg/InterfaceSend2Sector.msg"
  "msg/Location.msg"
  "msg/OpenCvOrder.msg"
  "msg/OverlapValue.msg"
  "msg/SaveMotion.msg"
  "msg/SensorPackage.msg"
  "msg/SingleMotorData.msg"
  "msg/YUVValue.msg"
  "msg/ZedCamera.msg"
  "msg/ZedCameraSave.msg"
  "msg/Zoom.msg"
  "srv/BuildModel.srv"
  "srv/BuildParam.srv"
  "srv/CheckSector.srv"
  "srv/DepthInfo.srv"
  "srv/HSVInfo.srv"
  "srv/OpenCvInfo.srv"
  "srv/OverlapInfo.srv"
  "srv/ReadMotion.srv"
  "srv/SaveHSV.srv"
  "srv/SaveOpenCv.srv"
  "srv/SaveParam.srv"
  "srv/SaveYUV.srv"
  "srv/TorqueState.srv"
  "srv/YUVInfo.srv"
  "srv/ZedCameraInfo.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
