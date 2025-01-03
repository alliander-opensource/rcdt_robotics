cmake_minimum_required(VERSION 3.5)
project(ros_package)

# CMake dependencies:
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Other dependencies:
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# Service definitions:
file(GLOB SRVS CONFIGURE_DEPENDS srv/*.srv*)

foreach(file IN LISTS SRVS)
  string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/" "" file_relative ${file})
  list(APPEND SRVS_STRIPPED ${file_relative})
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRVS_STRIPPED}
  DEPENDENCIES geometry_msgs vision_msgs
)

# Default test:
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()