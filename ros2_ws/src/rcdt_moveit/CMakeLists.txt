# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(rcdt_moveit)

# CMake dependencies:
find_package(ament_cmake REQUIRED)

# Other dependencies:
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rcdt_utilities_msgs REQUIRED)

# C++ executables:
include_directories(include)
add_executable(moveit_controller src/moveit_controller.cpp src/moveit_servo.cpp src/moveit_cpp.cpp src/planning_scene.cpp)
ament_target_dependencies(moveit_controller rclcpp geometry_msgs moveit_servo rcdt_utilities_msgs)

install(
  TARGETS moveit_controller
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
