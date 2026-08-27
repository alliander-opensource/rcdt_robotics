# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(alliander_robotiq)

# CMake dependencies:
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Other dependencies:
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(alliander_interfaces REQUIRED)

#########################
# C++ gripper_controller:
#########################

# Add include directories and source files:
include_directories(include)
add_executable(gripper_controller
  src/gripper_controller.cpp
  src/modbus_controller.cpp
)
ament_target_dependencies(gripper_controller
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  alliander_interfaces
)

# Add Glaze to enable struct to JSON serialization:
include(FetchContent)
FetchContent_Declare(
  glaze
  GIT_REPOSITORY https://github.com/stephenberry/glaze.git
  GIT_TAG v8.0.0
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(glaze)
target_link_libraries(gripper_controller glaze::glaze)

# Add Magic Enum to enable enum to string conversion:
find_package(magic_enum CONFIG REQUIRED)
target_link_libraries(gripper_controller magic_enum::magic_enum)

# Add Libmodbus to enable Modbus communication:
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)
target_include_directories(gripper_controller PRIVATE ${MODBUS_INCLUDE_DIRS})
target_link_libraries(gripper_controller ${MODBUS_LIBRARIES})

# Install targets:
install(
  TARGETS gripper_controller
  DESTINATION lib/${PROJECT_NAME}
)
#########################

# Shared folders:
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
