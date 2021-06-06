// Copyright (c) 2021 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef REALSENSE_HARDWARE_INTERFACE__UTIL_HPP_
#define REALSENSE_HARDWARE_INTERFACE__UTIL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <librealsense2/rs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <string>

namespace realsense_hardware_interface
{
void toMsg(const rs2_vector & point, geometry_msgs::msg::Point & msg)
{
  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;
}

void toMsg(const rs2_quaternion & quat, geometry_msgs::msg::Quaternion & msg)
{
  msg.x = quat.x;
  msg.y = quat.y;
  msg.z = quat.z;
  msg.w = quat.w;
}

void toMsg(const rs2_vector & vector, geometry_msgs::msg::Vector3 & msg)
{
  msg.x = vector.x;
  msg.y = vector.y;
  msg.z = vector.z;
}

void toMsg(const rs2_vector & linear, const rs2_vector & angular, geometry_msgs::msg::Twist & msg)
{
  toMsg(linear, msg.linear);
  toMsg(angular, msg.linear);
}

void toMsg(
  const rs2_pose & pose,
  const std::string & realsense_frame,
  const std::string & odom_frame,
  const rclcpp::Time & time,
  nav_msgs::msg::Odometry & msg)
{
  msg.header.frame_id = odom_frame;
  msg.header.stamp = time;
  msg.child_frame_id = realsense_frame;
  toMsg(pose.translation, msg.pose.pose.position);
  toMsg(pose.rotation, msg.pose.pose.orientation);
  toMsg(pose.velocity, pose.angular_velocity, msg.twist.twist);
}

struct DoubleDataHandle
{
  const std::string & name;
  const double value;
  DoubleDataHandle(const std::string & name, double value)
  : name(name), value(value) {}
};

struct Rs2VectorHandle
{
  const std::string & name;
  const DoubleDataHandle x;
  const DoubleDataHandle y;
  const DoubleDataHandle z;
  Rs2VectorHandle(const std::string & name, double x, double y, double z)
  : name(name),
    x(name + "::x", x),
    y(name + "::y", y),
    z(name + "::z", z) {}
};

struct Rs2QuaternionHandle
{
  const std::string & name;
  const DoubleDataHandle x;
  const DoubleDataHandle y;
  const DoubleDataHandle z;
  const DoubleDataHandle w;
  Rs2QuaternionHandle(const std::string & name, double x, double y, double z, double w)
  : name(name),
    x(name + "::x", x),
    y(name + "::y", y),
    z(name + "::z", z),
    w(name + "::z", w) {}
};
}  // namespace realsense_hardware_interface

#endif  // REALSENSE_HARDWARE_INTERFACE__UTIL_HPP_
