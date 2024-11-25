# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)

project(rcdt_franka)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install project files
install(
  DIRECTORY
  launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(PROGRAMS
  nodes/franka_gripper_simulation_node.py
  nodes/joy_to_gripper_node.py
  nodes/settings_setter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
