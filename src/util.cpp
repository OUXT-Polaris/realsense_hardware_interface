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

#include <realsense_hardware_interface/util.hpp>

namespace realsense_hardware_interface
{
void toMsg(const rs2_vector & point, geometry_msgs::msg::Point & msg)
{
  msg.x = point.z * -1;
  msg.y = point.x * -1;
  msg.z = point.y;
}

void toMsg(const rs2_quaternion & quat, geometry_msgs::msg::Quaternion & msg)
{
  msg.x = quat.z * -1;
  msg.y = quat.x * -1;
  msg.z = quat.y;
  msg.w = quat.w;
}

void toMsg(const rs2_vector & vector, geometry_msgs::msg::Vector3 & msg)
{
  msg.x = vector.z * -1;
  msg.y = vector.x * -1;
  msg.z = vector.y;
}

void toMsg(const rs2_imu & imu, sensor_msgs::msg::Imu & msg)
{
  toMsg(imu.getOrientation(), msg.orientation);
  toMsg(imu.getAngularVelocity(), msg.angular_velocity);
  toMsg(imu.getAcceleration(), msg.linear_acceleration);
}

void toMsg(const rs2_vector & linear, const rs2_vector & angular, geometry_msgs::msg::Twist & msg)
{
  toMsg(linear, msg.linear);
  toMsg(angular, msg.angular);
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

DoubleDataHandle::DoubleDataHandle(
  const std::string & sensor_name, const std::string & name, double value)
: sensor_name(sensor_name), name(name), value(value)
{
}

void DoubleDataHandle::appendStateInterface(
  std::vector<hardware_interface::StateInterface> & interfaces)
{
  interfaces.emplace_back(hardware_interface::StateInterface(sensor_name, name, &value));
}

void DoubleDataHandle::appendStateInterfaceNames(
  const std::string & joint_name, std::vector<std::string> & interface_names)
{
  interface_names.emplace_back(joint_name + "/" + name);
}

void DoubleDataHandle::setValue(double val) { value = val; }
void DoubleDataHandle::setValue(
  const std::vector<hardware_interface::LoanedStateInterface> & interface)
{
  value = getState(sensor_name, name, interface);
}
double DoubleDataHandle::getValue() const { return value; }

Rs2VectorHandle::Rs2VectorHandle(
  const std::string & sensor_name, const std::string & name, const rs2_vector & vec)
: sensor_name(sensor_name),
  name(name),
  x(sensor_name, name + "::x", vec.x),
  y(sensor_name, name + "::y", vec.y),
  z(sensor_name, name + "::z", vec.z)
{
}

void Rs2VectorHandle::appendStateInterface(
  std::vector<hardware_interface::StateInterface> & interfaces)
{
  x.appendStateInterface(interfaces);
  y.appendStateInterface(interfaces);
  z.appendStateInterface(interfaces);
}
void Rs2VectorHandle::appendStateInterfaceNames(
  const std::string & joint_name, std::vector<std::string> & interface_names)
{
  x.appendStateInterfaceNames(joint_name, interface_names);
  y.appendStateInterfaceNames(joint_name, interface_names);
  z.appendStateInterfaceNames(joint_name, interface_names);
}
void Rs2VectorHandle::setValue(const rs2_vector & vec)
{
  x.setValue(vec.x);
  y.setValue(vec.y);
  z.setValue(vec.z);
}
void Rs2VectorHandle::setValue(
  const std::vector<hardware_interface::LoanedStateInterface> & interface)
{
  x.setValue(interface);
  y.setValue(interface);
  z.setValue(interface);
}
const rs2_vector Rs2VectorHandle::getValue() const
{
  rs2_vector vec;
  vec.x = x.getValue();
  vec.y = y.getValue();
  vec.z = z.getValue();
  return vec;
}

Rs2QuaternionHandle::Rs2QuaternionHandle(
  const std::string & sensor_name, const std::string & name, const rs2_quaternion & quat)
: sensor_name(sensor_name),
  name(name),
  x(sensor_name, name + "::x", quat.x),
  y(sensor_name, name + "::y", quat.y),
  z(sensor_name, name + "::z", quat.z),
  w(sensor_name, name + "::w", quat.w)
{
}
void Rs2QuaternionHandle::appendStateInterface(
  std::vector<hardware_interface::StateInterface> & interfaces)
{
  x.appendStateInterface(interfaces);
  y.appendStateInterface(interfaces);
  z.appendStateInterface(interfaces);
  w.appendStateInterface(interfaces);
}
void Rs2QuaternionHandle::appendStateInterfaceNames(
  const std::string & joint_name, std::vector<std::string> & interface_names)
{
  x.appendStateInterfaceNames(joint_name, interface_names);
  y.appendStateInterfaceNames(joint_name, interface_names);
  z.appendStateInterfaceNames(joint_name, interface_names);
  w.appendStateInterfaceNames(joint_name, interface_names);
}
void Rs2QuaternionHandle::setValue(const rs2_quaternion & quat)
{
  x.setValue(quat.x);
  y.setValue(quat.y);
  z.setValue(quat.z);
  w.setValue(quat.w);
}
void Rs2QuaternionHandle::setValue(
  const std::vector<hardware_interface::LoanedStateInterface> & interface)
{
  x.setValue(interface);
  y.setValue(interface);
  z.setValue(interface);
  w.setValue(interface);
}
const rs2_quaternion Rs2QuaternionHandle::getValue() const
{
  rs2_quaternion quat;
  quat.x = x.getValue();
  quat.y = y.getValue();
  quat.z = z.getValue();
  quat.w = w.getValue();
  return quat;
}

Rs2ImuHandle::Rs2ImuHandle(
  const std::string & sensor_name, const std::string & name, const rs2_pose & pose,
  const rs2_vector angular_velocity, const rs2_vector acceleration)
: sensor_name(sensor_name),
  name(name),
  orientation(sensor_name, name + "::orientation", pose.rotation),
  angular_velocity(sensor_name, name + "::angular_velocity", angular_velocity),
  acceleration(sensor_name, name + "::acceleration", acceleration)
{
}

void Rs2ImuHandle::appendStateInterface(
  std::vector<hardware_interface::StateInterface> & interfaces)
{
  orientation.appendStateInterface(interfaces);
  angular_velocity.appendStateInterface(interfaces);
  acceleration.appendStateInterface(interfaces);
}

void Rs2ImuHandle::appendStateInterfaceNames(
  const std::string & joint_name, std::vector<std::string> & interface_names)
{
  orientation.appendStateInterfaceNames(joint_name, interface_names);
  angular_velocity.appendStateInterfaceNames(joint_name, interface_names);
  acceleration.appendStateInterfaceNames(joint_name, interface_names);
}

const rs2_imu Rs2ImuHandle::getValue() const
{
  rs2_imu imu(orientation.getValue(), angular_velocity.getValue(), acceleration.getValue());
  return imu;
}

Rs2PoseHandle::Rs2PoseHandle(
  const std::string & sensor_name, const std::string & name, const rs2_pose & pose)
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

void Rs2PoseHandle::appendStateInterface(
  std::vector<hardware_interface::StateInterface> & interfaces)
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
void Rs2PoseHandle::appendStateInterfaceNames(
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

void Rs2PoseHandle::setValue(const rs2_pose & pose)
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

void Rs2PoseHandle::setValue(
  const std::vector<hardware_interface::LoanedStateInterface> & interface)
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
const rs2_pose Rs2PoseHandle::getValue()
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

// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

// Convert rs2::frame to cv::Mat
cv::Mat frameToMat(const rs2::frame & f)
{
  using namespace cv;
  using namespace rs2;

  auto vf = f.as<video_frame>();
  const int w = vf.get_width();
  const int h = vf.get_height();

  if (f.get_profile().format() == RS2_FORMAT_BGR8) {
    return Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_RGB8) {
    auto r_rgb = Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
    Mat r_bgr;
    cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
    return r_bgr;
  } else if (f.get_profile().format() == RS2_FORMAT_Z16) {
    return Mat(Size(w, h), CV_16UC1, (void *)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_Y8) {
    return Mat(Size(w, h), CV_8UC1, (void *)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32) {
    return Mat(Size(w, h), CV_32FC1, (void *)f.get_data(), Mat::AUTO_STEP);
  }

  throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat depthFrameToMeters(const rs2::depth_frame & f)
{
  cv::Mat dm = frameToMat(f);
  dm.convertTo(dm, CV_64F);
  dm = dm * f.get_units();
  return dm;
}

std::size_t getImageMatSize(const std::string & camera_type)
{
  return getImageMatCols(camera_type) * getImageMatRows(camera_type);
}

std::size_t getImageMatCols(const std::string & camera_type)
{
  if (camera_type == "t265_fisheye") {
    return 848;
  }
  throw std::runtime_error("camera_type : " + camera_type + " does not support.");
}

std::size_t getImageMatRows(const std::string & camera_type)
{
  if (camera_type == "t265_fisheye") {
    return 800;
  }
  throw std::runtime_error("camera_type : " + camera_type + " does not support.");
}

std::string getImageEncording(const std::string & camera_type)
{
  if (camera_type == "t265_fisheye") {
    return "mono8";
  }
  throw std::runtime_error("camera_type : " + camera_type + " does not support.");
}

void getRealsenseDeviceLiet()
{
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();
  rs2::device selected_device;
  if (devices.size() == 0) {
    throw std::runtime_error("failed to query realsense device");
  } else {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("realsense_hardware_interface"), "found realsense device");
    for (const auto & device : devices) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("realsense_hardware_interface"), getDeviceName(device));
    }
  }
}

const std::string getDeviceName(const rs2::device & dev)
{
  // Each device provides some information on itself, such as name:
  std::string name = "Unknown Device";
  if (dev.supports(RS2_CAMERA_INFO_NAME)) name = dev.get_info(RS2_CAMERA_INFO_NAME);

  // and the serial number of the device:
  std::string sn = "########";
  if (dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
    sn = std::string("#") + dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

  return name + " " + sn;
}

}  // namespace realsense_hardware_interface