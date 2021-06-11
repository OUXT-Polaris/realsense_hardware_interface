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

#include <realsense_hardware_interface/t265_hardware_interface.hpp>

#include <vector>
#include <memory>

namespace realsense_hardware_interface
{
hardware_interface::return_type T265HardwareInterface::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }
  if (info.joints.size() != 1) {
    throw std::runtime_error("joint size should be 1");
  }
  joint_ = info.joints[0].name;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> T265HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces = {};
  pose_handle_ptr_ = std::make_shared<Rs2PoseHandle>(joint_, "rs2_pose", rs2_pose());
  pose_handle_ptr_->appendStateInterface(interfaces);
  for(const auto interface : interfaces) {
    std::cout << interface.get_name() << "/" << interface.get_interface_name() << std::endl;
  }
  return interfaces;
}

hardware_interface::return_type T265HardwareInterface::start()
{
  cfg_.enable_stream(RS2_STREAM_POSE);
  pipe_.start(cfg_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type T265HardwareInterface::stop()
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type T265HardwareInterface::read()
{
  rs2::frameset frameset;
  pipe_.poll_for_frames(&frameset);
  if (rs2::pose_frame pose_frame = frameset.first_or_default(RS2_STREAM_POSE)) {
    pose_handle_ptr_->setValue(pose_frame.get_pose_data());
  }
  return hardware_interface::return_type::OK;
}

}  // namespace realsense_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  realsense_hardware_interface::T265HardwareInterface, hardware_interface::SensorInterface)
