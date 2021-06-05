cmake_minimum_required(VERSION 3.5)
project(realsense_hardware_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(librealsense2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(realsense_hardware_interface SHARED
  src/t265_hardware_interface.cpp
)
target_include_directories(
  realsense_hardware_interface
  PRIVATE
  include
)
ament_target_dependencies(realsense_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  ros2_control
  realtime_tools
  librealsense2
)
pluginlib_export_plugin_description_file(hardware_interface realsense_hardware_interface.xml)

ament_export_libraries(
  realsense_hardware_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  ros2_control
  realtime_tools
  librealsense2
)

install(
  TARGETS realsense_hardware_interface
  DESTINATION lib
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
