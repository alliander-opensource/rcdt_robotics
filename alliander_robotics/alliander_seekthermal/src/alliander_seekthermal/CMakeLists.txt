# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(alliander_seekthermal)

# CMake dependencies:
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Other dependencies:
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

# Python package:
ament_python_install_package(${PROJECT_NAME})

# Python executables:
install(
  DIRECTORY src_py/
  DESTINATION lib/${PROJECT_NAME}
)

# Shared folders:
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
