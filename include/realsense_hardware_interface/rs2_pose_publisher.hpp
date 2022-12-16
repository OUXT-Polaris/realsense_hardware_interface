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

#ifndef REALSENSE_HARDWARE_INTERFACE__RS2_POSE_PUBLISHER_HPP_
#define REALSENSE_HARDWARE_INTERFACE__RS2_POSE_PUBLISHER_HPP_

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/controller_interface.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realsense_hardware_interface/util.hpp>
#include <realsense_hardware_interface/visibility_control.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <unordered_map>
#include <vector>

namespace realsense_hardware_interface
{
class Rs2PosePublisher : public controller_interface::ControllerInterface
{
public:
  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  controller_interface::return_type init(const std::string & controller_name, const std::string & namespace_ = "",
    const rclcpp::NodeOptions & node_options =
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)) override;

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

#if defined(GALACTIC) || defined(HUMBLE)
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
#if defined(GALACTIC) || defined(HUMBLE)
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
#else
  controller_interface::return_type update() override;
#endif

private:
  std::string joint_;
  std::string odom_frame_;
  std::string odom_topic_;
  bool publish_tf_;
  std::shared_ptr<Rs2PoseHandle> handle_;
  std::shared_ptr<rclcpp::Clock> clock_ptr_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_pub_realtime_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> tf_pub_realtime_;
};
}  // namespace realsense_hardware_interface

#endif  // REALSENSE_HARDWARE_INTERFACE__RS2_POSE_PUBLISHER_HPP_
