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

# grab all .srv files in /srv, and pass them to rosidl_generate_interfaces
# Note that this is actually against cmake guidelines.
# because cmake is unable to identify when srv files have changed.
file(GLOB SRVS srv/*.srv*)

foreach(file IN LISTS SRVS)
    string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/" "" file_relative ${file})
    list(APPEND SRVS_STRIPPED ${file_relative})
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRVS_STRIPPED}
  DEPENDENCIES geometry_msgs sensor_msgs vision_msgs realsense2_camera_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
