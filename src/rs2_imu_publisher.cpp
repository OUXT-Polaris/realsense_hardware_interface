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
  /*
  clock_ptr_ = node->get_clock();
  node->declare_parameter("joint", "");
  joint_ = node->get_parameter("joint").as_string();
  node->declare_parameter("odom_frame", "");
  odom_frame_ = node->get_parameter("odom_frame").as_string();
  node->declare_parameter("odom_topic", "odom");
  odom_topic_ = node->get_parameter("odom_topic").as_string();
  node->declare_parameter("publish_tf", false);
  publish_tf_ = node->get_parameter("publish_tf").as_bool();
  handle_ = std::make_shared<Rs2PoseHandle>(joint_, "rs2_pose", rs2_pose());
  */
  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Rs2ImuPublisher::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  /*
  odom_pub_ =
    node->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SystemDefaultsQoS());
  odom_pub_realtime_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);
  if (publish_tf_) {
    tf_pub_ = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS());
    tf_pub_realtime_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(tf_pub_);
  }
  */
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Rs2ImuPublisher::update()
{
  /*
  handle_->setValue(state_interfaces_);
  const auto rs2_pose = handle_->getValue();
  rclcpp::Time time = clock_ptr_->now();
  nav_msgs::msg::Odometry odom;
  toMsg(rs2_pose, joint_, "odom", time, odom);
  if (odom_pub_realtime_->trylock()) {
    odom_pub_realtime_->msg_ = odom;
    odom_pub_realtime_->unlockAndPublish();
  }
  if (publish_tf_) {
    if (tf_pub_realtime_->trylock()) {
      tf2_msgs::msg::TFMessage tf;
      geometry_msgs::msg::TransformStamped transform;
      transform.child_frame_id = joint_;
      transform.header.frame_id = odom_frame_;
      transform.header.stamp = time;
      transform.transform.rotation = odom.pose.pose.orientation;
      transform.transform.translation.x = odom.pose.pose.position.x;
      transform.transform.translation.y = odom.pose.pose.position.y;
      transform.transform.translation.z = odom.pose.pose.position.z;
      tf.transforms.emplace_back(transform);
      tf_pub_realtime_->msg_ = tf;
      tf_pub_realtime_->unlockAndPublish();
    }
  }
  */
  return controller_interface::return_type::OK;
}
}  // namespace realsense_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  realsense_hardware_interface::Rs2ImuPublisher, controller_interface::ControllerInterface)
