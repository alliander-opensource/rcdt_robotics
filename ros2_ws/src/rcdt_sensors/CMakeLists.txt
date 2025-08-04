# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(rcdt_sensors)

# CMake dependencies:
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Other dependencies:
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(rcdt_utilities REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# C++ executables:
include_directories(include)
add_executable(combine_camera_topics src/combine_camera_topics.cpp)
ament_target_dependencies(combine_camera_topics
  rclcpp
  sensor_msgs
  realsense2_camera_msgs
)
add_executable(convert_32FC1_to_16UC1 src/convert_32FC1_to_16UC1.cpp)
ament_target_dependencies(convert_32FC1_to_16UC1
  rclcpp
  cv_bridge
  OpenCV
)

install(
  TARGETS combine_camera_topics convert_32FC1_to_16UC1
  DESTINATION lib/${PROJECT_NAME}
)

# Python executables:
install(
  DIRECTORY src_py/
  DESTINATION lib/${PROJECT_NAME}
)

# Shared folders:
install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
