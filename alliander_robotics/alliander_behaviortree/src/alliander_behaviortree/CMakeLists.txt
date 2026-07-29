# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(alliander_behaviortree)

# CMake dependencies:
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)
find_package(alliander_interfaces REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

# C++ executables:
include_directories(include)
add_executable(bt_node
  src/bt_node.cpp
)
# target_include_directories(bt_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bt_node
  rclcpp
  rclcpp_action
  action_tutorials_interfaces
  alliander_interfaces
  behaviortree_ros2
)

install(
  TARGETS bt_node 
  DESTINATION lib/${PROJECT_NAME}
)

# Shared folders:
install(
  DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}
)

# Default test:
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
