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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(librealsense_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(t265_hardware_interface SHARED
  src/t265_hardware_interface.cpp
)
target_include_directories(
  t265_hardware_interface
  PRIVATE
  include
)
ament_target_dependencies(t265_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  ros2_control
  realtime_tools
  librealsense_vendor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
