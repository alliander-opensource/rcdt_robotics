# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(alliander_utilities_depth_camera)

# CMake dependencies:
find_package(ament_cmake REQUIRED)

# Other dependencies:
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)

# C++ executables:
include_directories(include)
add_executable(convert_32FC1_to_16UC1
  src/convert_32FC1_to_16UC1.cpp)
ament_target_dependencies(convert_32FC1_to_16UC1
  cv_bridge
)

install(
  TARGETS convert_32FC1_to_16UC1
  DESTINATION lib/${PROJECT_NAME}
)

# Default test:
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
