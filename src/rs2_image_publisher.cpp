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

#include <realsense_hardware_interface/rs2_image_publisher.hpp>

namespace realsense_hardware_interface
{
controller_interface::return_type Rs2ImagePublisher::init(const std::string & controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }
  auto node = get_node();
  clock_ptr_ = node->get_clock();
  node->declare_parameter("joint", "");
  joint_ = node->get_parameter("joint").as_string();
  node->declare_parameter("optical_frame", "");
  optical_frame_ = node->get_parameter("optical_frame").as_string();
  node->declare_parameter("image_topic", "");
  image_topic_ = node->get_parameter("image_topic").as_string();
  node->declare_parameter("camera_type", "");
  camera_type_ = node->get_parameter("camera_type").as_string();
  node->declare_parameter("shared_memory_key", "");
  shared_memory_key_ = node->get_parameter("shared_memory_key").as_string();
  node->declare_parameter("publish_rate", 30.0);
  publish_rate_ = node->get_parameter("publish_rate").as_double();
  update_duration_ = 1.0 / publish_rate_;
  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Rs2ImagePublisher::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  configure_time_ = node->get_clock()->now().seconds();
  next_update_time_ = configure_time_ + update_duration_;
  image_memory_ptr_ = std::make_shared<Poco::SharedMemory>(
    shared_memory_key_, getImageMatSize(camera_type_), Poco::SharedMemory::AccessMode::AM_WRITE);
  image_pub_ =
    node->create_publisher<sensor_msgs::msg::Image>(image_topic_, rclcpp::SensorDataQoS());
  image_pub_realtime_ =
    std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>>(image_pub_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void Rs2ImagePublisher::publishImage()
{
  auto node = get_node();
  const auto now = node->get_clock()->now();
  const auto image = cv::Mat(
    cv::Size(getImageMatCols(camera_type_), getImageMatRows(camera_type_)), CV_8U,
    image_memory_ptr_->begin());
  std_msgs::msg::Header header;
  header.frame_id = optical_frame_;
  header.stamp = now;
  sensor_msgs::msg::Image::SharedPtr image_msg =
    cv_bridge::CvImage(header, "mono8", image).toImageMsg();
  if(image_pub_realtime_->trylock()) {
    image_pub_realtime_->msg_ = *image_msg;
    image_pub_realtime_->unlockAndPublish();
  }
  next_update_time_ = next_update_time_ + update_duration_;
}

controller_interface::return_type Rs2ImagePublisher::update()
{
  auto node = get_node();
  const auto now = node->get_clock()->now();
  if (std::fabs(now.seconds() - next_update_time_) < update_duration_ * 0.5) {
    publishImage();
    return controller_interface::return_type::OK;
  }
  return controller_interface::return_type::OK;
}
}  // namespace realsense_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  realsense_hardware_interface::Rs2ImagePublisher, controller_interface::ControllerInterface)