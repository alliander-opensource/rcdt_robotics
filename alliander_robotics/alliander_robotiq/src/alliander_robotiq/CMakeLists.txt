# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(alliander_robotiq)

# CMake dependencies:
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Other dependencies:
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(alliander_interfaces REQUIRED)

# Python package:
ament_python_install_package(${PROJECT_NAME})

# Python executables:
install(
  DIRECTORY src_py/
  DESTINATION lib/${PROJECT_NAME}
)

# C++ executables:
include_directories(include)
add_executable(gripper_controller
  src/gripper_controller.cpp
  src/finger_status.cpp
  src/gripper_status.cpp
  src/json_utils.cpp
  src/modbus_tcp_client.cpp
  src/modbus_controller.cpp
)
ament_target_dependencies(gripper_controller
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  alliander_interfaces
)

install(
  TARGETS gripper_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Shared folders:
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Default test:
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
