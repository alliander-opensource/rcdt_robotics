# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)

project(rcdt_detection)

find_package(rclpy REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rcdt_utilities REQUIRED)

# Install project files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install python package
ament_python_install_package(${PROJECT_NAME})

# Install nodes
install(PROGRAMS
  nodes/define_centroid_node.py
  nodes/filter_masks_node.py
  nodes/get_rgbd_from_topic_node.py
  nodes/pose_from_pixel_node.py
  nodes/publish_image_node.py
  nodes/segment_image_node.py
  nodes/select_image_from_list_node.py
  nodes/split_rgbd_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
