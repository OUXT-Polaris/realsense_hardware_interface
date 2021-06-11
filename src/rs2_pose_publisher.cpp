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

#include <realsense_hardware_interface/rs2_pose_publisher.hpp>

namespace realsense_hardware_interface
{
controller_interface::return_type Rs2PosePublisher::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  auto node = get_node();
  node->declare_parameter("joint");
  joint_ = node->get_parameter("joint").as_string();
  node->declare_parameter("sensor_name");
  sensor_name_ = node->get_parameter("sensor_name").as_string();
  handle_ = std::make_shared<Rs2PoseHandle>(joint_, "rs2_pose", rs2_pose());
  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Rs2PosePublisher::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Rs2PosePublisher::update()
{
  handle_->setValue(state_interfaces_);
  return controller_interface::return_type::OK;
}
}  // namespace realsense_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  realsense_hardware_interface::Rs2PosePublisher, controller_interface::ControllerInterface)
