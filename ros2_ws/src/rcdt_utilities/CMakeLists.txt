# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)

project(rcdt_utilities)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_srvs REQUIRED)
find_package(rcdt_utilities_msgs REQUIRED)

find_package(rviz_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)

# Install project files
install(
  DIRECTORY
  launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install cpp:
add_executable(rviz_controller_node src/rviz_controller_node.cpp)
ament_target_dependencies(rviz_controller_node rclcpp std_srvs rcdt_utilities_msgs rviz_visual_tools tf2_eigen)

install(TARGETS
  rviz_controller_node
  DESTINATION lib/${PROJECT_NAME})

# Install python nodes:
install(PROGRAMS
  nodes/joy_to_twist_node.py
  nodes/moveit_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install python package
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
