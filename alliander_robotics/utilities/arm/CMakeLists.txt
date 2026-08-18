# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(alliander_utilities_arm)

# CMake dependencies:
find_package(ament_cmake REQUIRED)

# Other dependencies:
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(alliander_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# C++ executable - Arm Wave demo:
include_directories(include)
add_executable(arm_wave_node
  src/arm_wave_node.cpp
  src/arm_wave.cpp
)
target_include_directories(arm_wave_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/arm_wave>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(arm_wave_node
  rclcpp
  rclcpp_action
  alliander_interfaces
  control_msgs
  trajectory_msgs
)

install(
  TARGETS arm_wave_node
  DESTINATION lib/${PROJECT_NAME}
)

# Default test:
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
