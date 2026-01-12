cmake_minimum_required(VERSION 3.8)
project(dolphin_arm_joy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(joy_inspector src/joy_inspector.cpp)
add_executable(joy_to_arm src/joy_to_arm.cpp)
add_executable(joy_to_gripper src/joy_to_gripper.cpp)
add_executable(joy_to_velocity src/joy_to_velocity.cpp)
target_include_directories(joy_inspector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(joy_inspector PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  joy_inspector
  "rclcpp"
  "sensor_msgs"
)
ament_target_dependencies(
  joy_to_arm 
  rclcpp sensor_msgs 
  trajectory_msgs
)
ament_target_dependencies(
  joy_to_gripper
  rclcpp sensor_msgs 
  trajectory_msgs
)

ament_target_dependencies(
  joy_to_velocity 
  rclcpp 
  sensor_msgs 
  trajectory_msgs
  geometry_msgs)

install(TARGETS 
  joy_inspector
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS 
joy_to_arm
DESTINATION 
lib/${PROJECT_NAME})

install(TARGETS 
joy_to_gripper
DESTINATION 
lib/${PROJECT_NAME})

install(TARGETS 
joy_to_velocity 
DESTINATION 
lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
