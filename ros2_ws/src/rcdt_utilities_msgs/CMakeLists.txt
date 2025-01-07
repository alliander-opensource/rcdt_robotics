# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)

project(rcdt_utilities_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# grab all .srv files in /srv, and pass them to rosidl_generate_interfaces
# Note that this is actually against cmake guidelines 
# because cmake is unable to identify when srv files have changed.
file(GLOB SRVS CONFIGURE_DEPENDS srv/*.srv*)

foreach(file IN LISTS SRVS)
  string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/" "" file_relative ${file})
  list(APPEND SRVS_STRIPPED ${file_relative})
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRVS_STRIPPED}
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
