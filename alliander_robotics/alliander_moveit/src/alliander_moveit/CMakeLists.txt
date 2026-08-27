# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(alliander_moveit)

# CMake dependencies:
find_package(ament_cmake REQUIRED)

# Other dependencies:
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(alliander_interfaces REQUIRED)

# Python package:
ament_python_install_package(${PROJECT_NAME})

# Moveit Manager:
include_directories(include)
add_executable(moveit_manager src/moveit_manager.cpp)
ament_target_dependencies(moveit_manager
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  alliander_interfaces
)

install(
  TARGETS moveit_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Manipulate Pose:
add_executable(manipulate_pose 
  src/manipulate_pose.cpp
  src/main.cpp
  )
ament_target_dependencies(manipulate_pose
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  alliander_interfaces
)

install(
  TARGETS manipulate_pose
  DESTINATION lib/${PROJECT_NAME}
)

# Shared folders:
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Unit tests:
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(manipulate_pose_test
    tests/test_package.cpp
    src/manipulate_pose.cpp
  )
  ament_target_dependencies(manipulate_pose_test
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
    alliander_interfaces
  )
endif()

ament_package()
