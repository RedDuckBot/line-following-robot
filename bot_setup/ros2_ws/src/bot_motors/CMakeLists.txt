cmake_minimum_required(VERSION 3.8)
project(bot_motors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(bot_interfaces REQUIRED)
find_package(fmt REQUIRED)

include_directories(
  include
)

add_executable(motor_controller_node
  src/motorControllerNode.cpp
  src/device.cpp
  src/digitalOutputDevice.cpp
  src/motorDriver.cpp
)

target_link_libraries(motor_controller_node
  pigpiod_if2
  fmt::fmt
)

#Link ROS2 packages
ament_target_dependencies(motor_controller_node
  rclcpp
  rclcpp_action
  bot_interfaces
)


install(TARGETS
  motor_controller_node
  DESTINATION lib/${PROJECT_NAME}/
)



ament_package()
