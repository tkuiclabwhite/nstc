cmake_minimum_required(VERSION 3.8)
project(armpackage)

find_package(ament_cmake REQUIRED)

# 橋接節點和產生網頁的工具（ros2 run armpackage moveit_bridge.py）
install(PROGRAMS
  scripts/moveit_bridge.py
  scripts/real_arm_bridge.py
  scripts/build_viewer.py
  scripts/jog_motor.py
  scripts/self_check.py
  scripts/current_log.py
  scripts/compliance.py
  DESTINATION lib/${PROJECT_NAME})
# 橋接節點 import 的模組，放在同一個資料夾
install(FILES
  scripts/arm_config.py
  scripts/kinematics.py
  DESTINATION lib/${PROJECT_NAME})

# 網頁在 ~/nstc/web（跟其他網頁放一起），不裝進 share
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
