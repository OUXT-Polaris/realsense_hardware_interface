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

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <librealsense2/rs.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

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
  const rs2_pose & pose, const std::string & realsense_frame, const std::string & odom_frame,
  const rclcpp::Time & time, nav_msgs::msg::Odometry & msg)
{
  msg.header.frame_id = odom_frame;
  msg.header.stamp = time;
  msg.child_frame_id = realsense_frame;
  toMsg(pose.translation, msg.pose.pose.position);
  toMsg(pose.rotation, msg.pose.pose.orientation);
  toMsg(pose.velocity, pose.angular_velocity, msg.twist.twist);
}

double getState(
  const std::string & joint_name, const std::string & interface_name,
  const std::vector<hardware_interface::LoanedStateInterface> & interfaces)
{
  for (const auto & interface : interfaces) {
    if (interface.get_name() == joint_name && interface.get_interface_name() == interface_name) {
      return interface.get_value();
    }
  }
  throw std::runtime_error(
    "state interface : " + interface_name + " does not exist in : " + joint_name);
}

class DoubleDataHandle
{
public:
  const std::string sensor_name;
  const std::string name;

private:
  double value;

public:
  DoubleDataHandle() = delete;
  DoubleDataHandle(const std::string & sensor_name, const std::string & name, double value)
  : sensor_name(sensor_name), name(name), value(value)
  {
  }
  void appendStateInterface(std::vector<hardware_interface::StateInterface> & interfaces)
  {
    interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, name, &value));
  }
  void appendStateInterfaceNames(
    const std::string & joint_name, std::vector<std::string> & interface_names)
  {
    interface_names.emplace_back(joint_name + "/" + name);
  }
  void setValue(double val) { value = val; }
  void setValue(const std::vector<hardware_interface::LoanedStateInterface> & interface)
  {
    value = getState(sensor_name, name, interface);
  }
  double getValue() const { return value; }
};

class Rs2VectorHandle
{
public:
  const std::string sensor_name;
  const std::string name;

private:
  DoubleDataHandle x;
  DoubleDataHandle y;
  DoubleDataHandle z;

public:
  Rs2VectorHandle() = delete;
  Rs2VectorHandle(const std::string & sensor_name, const std::string & name, const rs2_vector & vec)
  : sensor_name(sensor_name),
    name(name),
    x(sensor_name, name + "::x", vec.x),
    y(sensor_name, name + "::y", vec.y),
    z(sensor_name, name + "::z", vec.z)
  {
  }
  void appendStateInterface(std::vector<hardware_interface::StateInterface> & interfaces)
  {
    x.appendStateInterface(interfaces);
    y.appendStateInterface(interfaces);
    z.appendStateInterface(interfaces);
  }
  void appendStateInterfaceNames(
    const std::string & joint_name, std::vector<std::string> & interface_names)
  {
    x.appendStateInterfaceNames(joint_name, interface_names);
    y.appendStateInterfaceNames(joint_name, interface_names);
    z.appendStateInterfaceNames(joint_name, interface_names);
  }
  void setValue(const rs2_vector & vec)
  {
    x.setValue(vec.x);
    y.setValue(vec.y);
    z.setValue(vec.z);
  }
  void setValue(const std::vector<hardware_interface::LoanedStateInterface> & interface)
  {
    x.setValue(interface);
    y.setValue(interface);
    z.setValue(interface);
  }
  const rs2_vector getValue() const
  {
    rs2_vector vec;
    vec.x = x.getValue();
    vec.y = y.getValue();
    vec.z = z.getValue();
    return vec;
  }
};

class Rs2QuaternionHandle
{
public:
  const std::string sensor_name;
  const std::string name;

private:
  DoubleDataHandle x;
  DoubleDataHandle y;
  DoubleDataHandle z;
  DoubleDataHandle w;

public:
  Rs2QuaternionHandle() = delete;
  Rs2QuaternionHandle(
    const std::string & sensor_name, const std::string & name, const rs2_quaternion & quat)
  : sensor_name(sensor_name),
    name(name),
    x(sensor_name, name + "::x", quat.x),
    y(sensor_name, name + "::y", quat.y),
    z(sensor_name, name + "::z", quat.z),
    w(sensor_name, name + "::z", quat.w)
  {
  }
  void appendStateInterface(std::vector<hardware_interface::StateInterface> & interfaces)
  {
    x.appendStateInterface(interfaces);
    y.appendStateInterface(interfaces);
    z.appendStateInterface(interfaces);
    w.appendStateInterface(interfaces);
  }
  void appendStateInterfaceNames(
    const std::string & joint_name, std::vector<std::string> & interface_names)
  {
    x.appendStateInterfaceNames(joint_name, interface_names);
    y.appendStateInterfaceNames(joint_name, interface_names);
    z.appendStateInterfaceNames(joint_name, interface_names);
    w.appendStateInterfaceNames(joint_name, interface_names);
  }
  void setValue(const rs2_quaternion & quat)
  {
    x.setValue(quat.x);
    y.setValue(quat.y);
    z.setValue(quat.z);
    w.setValue(quat.w);
  }
  void setValue(const std::vector<hardware_interface::LoanedStateInterface> & interface)
  {
    x.setValue(interface);
    y.setValue(interface);
    z.setValue(interface);
    w.setValue(interface);
  }
  const rs2_quaternion getValue() const
  {
    rs2_quaternion quat;
    quat.x = x.getValue();
    quat.y = y.getValue();
    quat.z = z.getValue();
    quat.w = w.getValue();
    return quat;
  }
};

class Rs2PoseHandle
{
public:
  const std::string sensor_name;
  const std::string name;

private:
  Rs2VectorHandle translation;
  Rs2VectorHandle velocity;
  Rs2VectorHandle acceleration;
  Rs2QuaternionHandle rotation;
  Rs2VectorHandle angular_velocity;
  Rs2VectorHandle angular_acceleration;
  DoubleDataHandle tracker_confidence;
  DoubleDataHandle mapper_confidence;

public:
  Rs2PoseHandle() = delete;
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
    mapper_confidence(sensor_name, name + "::mapper_confidence", pose.mapper_confidence)
  {
  }
  void appendStateInterface(std::vector<hardware_interface::StateInterface> & interfaces)
  {
    translation.appendStateInterface(interfaces);
    velocity.appendStateInterface(interfaces);
    acceleration.appendStateInterface(interfaces);
    rotation.appendStateInterface(interfaces);
    angular_velocity.appendStateInterface(interfaces);
    angular_acceleration.appendStateInterface(interfaces);
    tracker_confidence.appendStateInterface(interfaces);
    mapper_confidence.appendStateInterface(interfaces);
  }
  void appendStateInterfaceNames(
    const std::string & joint_name, std::vector<std::string> & interface_names)
  {
    translation.appendStateInterfaceNames(joint_name, interface_names);
    velocity.appendStateInterfaceNames(joint_name, interface_names);
    acceleration.appendStateInterfaceNames(joint_name, interface_names);
    rotation.appendStateInterfaceNames(joint_name, interface_names);
    angular_velocity.appendStateInterfaceNames(joint_name, interface_names);
    angular_acceleration.appendStateInterfaceNames(joint_name, interface_names);
    tracker_confidence.appendStateInterfaceNames(joint_name, interface_names);
    mapper_confidence.appendStateInterfaceNames(joint_name, interface_names);
  }
  void setValue(const rs2_pose & pose)
  {
    translation.setValue(pose.translation);
    velocity.setValue(pose.velocity);
    acceleration.setValue(pose.acceleration);
    rotation.setValue(pose.rotation);
    angular_velocity.setValue(pose.angular_velocity);
    angular_acceleration.setValue(pose.angular_acceleration);
    tracker_confidence.setValue(pose.tracker_confidence);
    mapper_confidence.setValue(pose.mapper_confidence);
  }
  void setValue(const std::vector<hardware_interface::LoanedStateInterface> & interface)
  {
    translation.setValue(interface);
    velocity.setValue(interface);
    acceleration.setValue(interface);
    rotation.setValue(interface);
    angular_velocity.setValue(interface);
    angular_acceleration.setValue(interface);
    tracker_confidence.setValue(interface);
    mapper_confidence.setValue(interface);
  }
  const rs2_pose getValue()
  {
    rs2_pose pose;
    pose.translation = translation.getValue();
    pose.velocity = velocity.getValue();
    pose.acceleration = acceleration.getValue();
    pose.rotation = rotation.getValue();
    pose.angular_velocity = angular_velocity.getValue();
    pose.angular_acceleration = angular_acceleration.getValue();
    pose.tracker_confidence = tracker_confidence.getValue();
    pose.mapper_confidence = mapper_confidence.getValue();
    return pose;
  }
};
}  // namespace realsense_hardware_interface

#endif  // REALSENSE_HARDWARE_INTERFACE__UTIL_HPP_
