# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(rcdt_utilities)

# CMake dependencies:
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Other dependencies:
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcdt_utilities_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit_servo REQUIRED)

# C++ executables:
add_executable(rviz_controller_node src/rviz_controller_node.cpp)
add_executable(moveit_controller src/moveit_controller.cpp)
ament_target_dependencies(rviz_controller_node rclcpp std_srvs rcdt_utilities_msgs rviz_visual_tools tf2_eigen)
ament_target_dependencies(moveit_controller moveit_servo)
install(
  TARGETS rviz_controller_node moveit_controller
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
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Default test:
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
