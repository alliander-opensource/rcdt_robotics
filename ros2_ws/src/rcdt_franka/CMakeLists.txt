# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(rcdt_franka)

# CMake dependencies:
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(franka_msgs REQUIRED)

# C++ executables:
include_directories(include)
add_executable(open_gripper src/open_gripper.cpp)
ament_target_dependencies(open_gripper
  rclcpp
  rclcpp_action
  std_srvs
  franka_msgs
)

install(
  TARGETS open_gripper
  DESTINATION lib/${PROJECT_NAME}
)

# Python executables:
install(
  DIRECTORY src_py/
  DESTINATION lib/${PROJECT_NAME}
)

# Python package:
ament_python_install_package(${PROJECT_NAME})

# Shared folders:
install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

# Default test:
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
