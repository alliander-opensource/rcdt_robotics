# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)
project(alliander_openvla)

# CMake dependencies:
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Other dependencies
# ADD OTHER DEPENDENCIES HERE AND IN package.xml??
# find_package(moveit_msgs REQUIRED)

# Python packages:
ament_python_install_package(${PROJECT_NAME}) # The alliander_openvla folder (PURE python files)
ament_python_install_package(src_py) # The src_py folder (containing ROS-Python helper files)

# Python executables:
install(
  DIRECTORY src_py/
  DESTINATION lib/${PROJECT_NAME}
)

# ament_target_dependencies(vla_node
#   moveit_msgs
# )

# Shared folders:
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Default test:
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
