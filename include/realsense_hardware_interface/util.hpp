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
  const std::string & sensor_name;
  const std::string & name;
  double value;
  DoubleDataHandle(const std::string & sensor_name, const std::string & name, double value)
  : sensor_name(sensor_name), name(name), value(value) {}
  void appendStateInterface(std::vector<hardware_interface::StateInterface> & interfaces)
  {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger(
        "realsemse_hardware_interface"), "exporting interface" << sensor_name << "/" << name);
    interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, name, &value));
  }
};

struct Rs2VectorHandle
{
  const std::string & sensor_name;
  const std::string & name;
  DoubleDataHandle x;
  DoubleDataHandle y;
  DoubleDataHandle z;
  Rs2VectorHandle(const std::string & sensor_name, const std::string & name, const rs2_vector & vec)
  : sensor_name(sensor_name),
    name(name),
    x(sensor_name, name + "::x", vec.x),
    y(sensor_name, name + "::y", vec.y),
    z(sensor_name, name + "::z", vec.z) {}
  void appendStateInterface(std::vector<hardware_interface::StateInterface> & interfaces)
  {
    x.appendStateInterface(interfaces);
    y.appendStateInterface(interfaces);
    z.appendStateInterface(interfaces);
  }
};

struct Rs2QuaternionHandle
{
  const std::string & sensor_name;
  const std::string & name;
  DoubleDataHandle x;
  DoubleDataHandle y;
  DoubleDataHandle z;
  DoubleDataHandle w;
  Rs2QuaternionHandle(
    const std::string & sensor_name, const std::string & name,
    const rs2_quaternion & quat)
  : sensor_name(sensor_name),
    name(name),
    x(sensor_name, name + "::x", quat.x),
    y(sensor_name, name + "::y", quat.y),
    z(sensor_name, name + "::z", quat.z),
    w(sensor_name, name + "::z", quat.w) {}
  void appendStateInterface(std::vector<hardware_interface::StateInterface> & interfaces)
  {
    x.appendStateInterface(interfaces);
    y.appendStateInterface(interfaces);
    z.appendStateInterface(interfaces);
    w.appendStateInterface(interfaces);
  }
};

struct Rs2PoseHandle
{
  const std::string & sensor_name;
  const std::string & name;
  Rs2VectorHandle translation;
  Rs2VectorHandle velocity;
  Rs2VectorHandle acceleration;
  Rs2QuaternionHandle rotation;
  Rs2VectorHandle angular_velocity;
  Rs2VectorHandle angular_acceleration;
  DoubleDataHandle tracker_confidence;
  DoubleDataHandle mapper_confidence;
  Rs2PoseHandle(const std::string & sensor_name, const std::string & name, const rs2_pose & pose)
  : sensor_name(sensor_name),
    name(name),
    translation(sensor_name, name + "::translation", pose.translation),
    velocity(sensor_name, name + "::velocity", pose.velocity),
    acceleration(sensor_name, name + "::acceleration", pose.acceleration),
    rotation(sensor_name, name + "::rotation", pose.rotation),
    angular_velocity(sensor_name, name + "::angular_velocity", pose.angular_velocity),
    angular_acceleration(sensor_name, name + "::angular_acceleration", pose.angular_acceleration),
    tracker_confidence(sensor_name, name + "::tracker_confidence", pose.tracker_confidence),
    mapper_confidence(sensor_name, name + "::mapper_confidence", pose.mapper_confidence) {}
  void appendStateInterface(std::vector<hardware_interface::StateInterface> & interfaces)
  {
    translation.appendStateInterface(interfaces);
    velocity.appendStateInterface(interfaces);
    acceleration.appendStateInterface(interfaces);
    rotation.appendStateInterface(interfaces);
    angular_velocity.appendStateInterface(interfaces);
    tracker_confidence.appendStateInterface(interfaces);
    mapper_confidence.appendStateInterface(interfaces);
  }
};
}  // namespace realsense_hardware_interface

#endif  // REALSENSE_HARDWARE_INTERFACE__UTIL_HPP_
