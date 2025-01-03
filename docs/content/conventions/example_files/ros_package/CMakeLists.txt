# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(ros_package)

# CMake dependencies:
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Other dependencies:
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# C++ executables:
add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node geometry_msgs vision_msgs)
install(
  TARGETS cpp_node
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
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Default test:
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()