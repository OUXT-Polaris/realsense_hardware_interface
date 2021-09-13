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

#ifndef REALSENSE_HARDWARE_INTERFACE__RS2_IMU_PUBLISHER_HPP_
#define REALSENSE_HARDWARE_INTERFACE__RS2_IMU_PUBLISHER_HPP_

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realsense_hardware_interface/util.hpp>
#include <realsense_hardware_interface/visibility_control.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace realsense_hardware_interface
{
class Rs2ImuPublisher : public controller_interface::ControllerInterface
{
public:
  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    std::vector<std::string> interface_names = {};
    handle_->appendStateInterfaceNames(joint_, interface_names);
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
  }

#if GALACTIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init()
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
#endif

  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override;

  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  controller_interface::return_type update() override;

private:
  std::string joint_;
  std::shared_ptr<Rs2ImuHandle> handle_;
  rclcpp::Clock::SharedPtr clock_ptr_;
  std::string imu_frame_;
  std::string imu_topic_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};
}  // namespace realsense_hardware_interface

#endif  // REALSENSE_HARDWARE_INTERFACE__IMU_PUBLISHER_HPP_
