# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)

project(rcdt_detection)

find_package(rclpy REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install project files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install python package
ament_python_install_package(${PROJECT_NAME})

# Install nodes
install(PROGRAMS
  nodes/object_detection.py
  nodes/segment_image_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
