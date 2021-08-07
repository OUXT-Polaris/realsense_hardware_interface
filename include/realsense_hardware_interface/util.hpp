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

#include <quaternion_operation/quaternion_operation.h>

#include <exception>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>  // Include OpenCV API
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector>

namespace realsense_hardware_interface
{
void toMsg(const rs2_vector & point, geometry_msgs::msg::Point & msg);
void toMsg(const rs2_quaternion & quat, geometry_msgs::msg::Quaternion & msg);
void toMsg(const rs2_vector & vector, geometry_msgs::msg::Vector3 & msg);
void toMsg(const rs2_vector & linear, const rs2_vector & angular, geometry_msgs::msg::Twist & msg);
void toMsg(
  const rs2_pose & pose, const std::string & realsense_frame, const std::string & odom_frame,
  const rclcpp::Time & time, nav_msgs::msg::Odometry & msg);
double getState(
  const std::string & joint_name, const std::string & interface_name,
  const std::vector<hardware_interface::LoanedStateInterface> & interfaces);

class DoubleDataHandle
{
public:
  const std::string sensor_name;
  const std::string name;

private:
  double value;

public:
  DoubleDataHandle() = delete;
  DoubleDataHandle(const std::string & sensor_name, const std::string & name, double value);
  void appendStateInterface(std::vector<hardware_interface::StateInterface> & interfaces);
  void appendStateInterfaceNames(
    const std::string & joint_name, std::vector<std::string> & interface_names);
  void setValue(double val);
  void setValue(const std::vector<hardware_interface::LoanedStateInterface> & interface);
  double getValue() const;
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
  Rs2VectorHandle(
    const std::string & sensor_name, const std::string & name, const rs2_vector & vec);
  void appendStateInterface(std::vector<hardware_interface::StateInterface> & interfaces);
  void appendStateInterfaceNames(
    const std::string & joint_name, std::vector<std::string> & interface_names);
  void setValue(const rs2_vector & vec);
  void setValue(const std::vector<hardware_interface::LoanedStateInterface> & interface);
  const rs2_vector getValue() const;
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
    const std::string & sensor_name, const std::string & name, const rs2_quaternion & quat);
  void appendStateInterface(std::vector<hardware_interface::StateInterface> & interfaces);
  void appendStateInterfaceNames(
    const std::string & joint_name, std::vector<std::string> & interface_names);
  void setValue(const rs2_quaternion & quat);
  void setValue(const std::vector<hardware_interface::LoanedStateInterface> & interface);
  const rs2_quaternion getValue() const;
};

struct rs2_imu
{
public:
  rs2_imu(
    const rs2_quaternion & orientation, const rs2_vector & angular_velocity,
    const rs2_vector & acceleration)
  : orientation(orientation), angular_velocity(angular_velocity), acceleration(acceleration)
  {
  }
  const rs2_quaternion orientation;
  const rs2_vector angular_velocity;
  const rs2_vector acceleration;
};

class Rs2ImuHandle
{
public:
  const std::string sensor_name;
  const std::string name;

private:
  Rs2QuaternionHandle orientation;
  Rs2VectorHandle angular_velocity;
  Rs2VectorHandle acceleration;

public:
  Rs2ImuHandle() = delete;
  Rs2ImuHandle(
    const std::string & sensor_name, const std::string & name, const rs2_pose & pose,
    const rs2_vector angular_velocity, const rs2_vector acceleration);
  void appendStateInterface(std::vector<hardware_interface::StateInterface> & interfaces);
  void appendStateInterfaceNames(
    const std::string & joint_name, std::vector<std::string> & interface_names);
  void setValue(
    const rs2_pose & pose, const rs2_vector angular_velocity, const rs2_vector acceleration);
  void setValue(const std::vector<hardware_interface::LoanedStateInterface> & interface);
  const rs2_imu getValue() const;
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
  Rs2PoseHandle(const std::string & sensor_name, const std::string & name, const rs2_pose & pose);
  void appendStateInterface(std::vector<hardware_interface::StateInterface> & interfaces);
  void appendStateInterfaceNames(
    const std::string & joint_name, std::vector<std::string> & interface_names);
  void setValue(const rs2_pose & pose);
  void setValue(const std::vector<hardware_interface::LoanedStateInterface> & interface);
  const rs2_pose getValue();
};

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

// Convert rs2::frame to cv::Mat
cv::Mat frameToMat(const rs2::frame & f);

// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat depthFrameToMeters(const rs2::depth_frame & f);

std::size_t getImageMatSize(const std::string & camera_type);

std::size_t getImageMatCols(const std::string & camera_type);

std::size_t getImageMatRows(const std::string & camera_type);

std::string getImageEncording(const std::string & camera_type);

void getRealsenseDeviceLiet();
const std::string getDeviceName(const rs2::device & dev);
}  // namespace realsense_hardware_interface

#endif  // REALSENSE_HARDWARE_INTERFACE__UTIL_HPP_
