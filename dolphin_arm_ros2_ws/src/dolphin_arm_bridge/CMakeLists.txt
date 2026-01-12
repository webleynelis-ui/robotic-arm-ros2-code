cmake_minimum_required(VERSION 3.8)
project(dolphin_arm_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 手动指定 LibSerial 的 include 与 lib 路径
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

add_executable(serial_bridge.cpp src/serial_bridge.cpp)
add_executable(joystick_gripper.cpp src/joystick_gripper.cpp)
add_executable(joystick_arm.cpp src/joystick_arm.cpp)

ament_target_dependencies(
  serial_bridge.cpp
  rclcpp
  trajectory_msgs
  sensor_msgs
)

ament_target_dependencies(
  joystick_gripper.cpp
  rclcpp
  trajectory_msgs
  sensor_msgs
)
ament_target_dependencies(
  joystick_arm.cpp 
  rclcpp 
  trajectory_msgs
  sensor_msgs)

# 链接 LibSerial 库
target_link_libraries(serial_bridge.cpp serial)
target_link_libraries(joystick_gripper.cpp serial)
target_link_libraries(joystick_arm.cpp serial)

install(TARGETS serial_bridge.cpp
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS joystick_gripper.cpp
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS joystick_arm.cpp
  DESTINATION lib/${PROJECT_NAME})


ament_package()
