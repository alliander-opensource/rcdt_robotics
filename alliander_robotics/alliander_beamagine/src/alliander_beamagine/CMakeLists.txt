# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(alliander_beamagine)

# CMake dependencies:
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Shared folders:
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
