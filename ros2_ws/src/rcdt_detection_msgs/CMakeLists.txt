# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)

project(rcdt_detection_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/DefineCentroid.srv
  srv/FilterMasks.srv
  srv/GetBoundingBox2D.srv
  srv/GetLargestContour.srv
  srv/GetMeanHue.srv
  srv/GetRectangleFactor.srv
  srv/GetRGBDFromTopic.srv
  srv/PoseFromPixel.srv
  srv/PublishImage.srv
  srv/SegmentImage.srv
  srv/SelectImageFromList.srv
  srv/SplitRGBD.srv
  DEPENDENCIES geometry_msgs sensor_msgs vision_msgs realsense2_camera_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
