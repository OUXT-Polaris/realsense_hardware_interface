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

#include <realsense_hardware_interface/rs2_imu_publisher.hpp>
#include <realsense_hardware_interface/rs2_pose_publisher.hpp>

namespace realsense_hardware_interface
{
controller_interface::return_type Rs2ImuPublisher::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  auto node = get_node();
  clock_ptr_ = node->get_clock();
  node->declare_parameter("joint", "");
  joint_ = node->get_parameter("joint").as_string();
  node->declare_parameter("imu_frame", "");
  imu_frame_ = node->get_parameter("imu_frame").as_string();
  node->declare_parameter("imu_topic", "imu");
  imu_topic_ = node->get_parameter("imu_topic").as_string();
  handle_ =
    std::make_shared<Rs2ImuHandle>(joint_, "rs2_imu", rs2_pose(), rs2_vector(), rs2_vector());
  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Rs2ImuPublisher::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SystemDefaultsQoS());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Rs2ImuPublisher::update()
{
  handle_->setValue(state_interfaces_);
  const auto rs2_imu = handle_->getValue();
  rclcpp::Time time = clock_ptr_->now();
  sensor_msgs::msg::Imu imu;
  toMsg(rs2_imu, imu);
  imu.header.frame_id = imu_frame_;
  imu.header.stamp = time;
  imu_pub_->publish(imu);
  return controller_interface::return_type::OK;
}
}  // namespace realsense_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  realsense_hardware_interface::Rs2ImuPublisher, controller_interface::ControllerInterface)
