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

if($ENV{ROS_DISTRO} STREQUAL "galactic")
  add_definitions(-DGALACTIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realsense2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Poco REQUIRED Foundation)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(realsense_hardware_interface SHARED
  src/t265_hardware_interface.cpp
  src/util.cpp
)
target_include_directories(
  realsense_hardware_interface
  PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(realsense_hardware_interface
  ${realsense2_LIBRARY}
  Poco::Foundation
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(realsense_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  ros2_control
  realtime_tools
  nav_msgs
  tf2_msgs
  image_transport
  sensor_msgs
  cv_bridge
  Poco
  OpenCV
)
pluginlib_export_plugin_description_file(hardware_interface realsense_hardware_interface.xml)

add_library(realsense_controller_interface SHARED
  src/rs2_pose_publisher.cpp
  src/rs2_image_publisher.cpp
  src/rs2_imu_publisher.cpp
  src/util.cpp
)
target_include_directories(
  realsense_controller_interface
  PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(realsense_controller_interface
  ${realsense2_LIBRARY}
  Poco::Foundation
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(realsense_controller_interface
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  ros2_control
  realtime_tools
  nav_msgs
  tf2_msgs
  image_transport
  sensor_msgs
  cv_bridge
  Poco
  OpenCV
)
pluginlib_export_plugin_description_file(controller_interface realsense_controller_interface.xml)

ament_export_libraries(
  realsense_hardware_interface realsense_controller_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  ros2_control
  realtime_tools
  nav_msgs
  tf2_msgs
  image_transport
  sensor_msgs
  cv_bridge
  Poco
  OpenCV
)

install(
  TARGETS realsense_hardware_interface realsense_controller_interface
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
