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

#ifndef REALSENSE_HARDWARE_INTERFACE__T265_HARDWARE_INTERFACE_HPP_
#define REALSENSE_HARDWARE_INTERFACE__T265_HARDWARE_INTERFACE_HPP_

#include <Poco/SharedMemory.h>

#if defined(GALACTIC) || defined(HUMBLE)
#include <hardware_interface/system_interface.hpp>
#else
#include <hardware_interface/base_interface.hpp>
#endif
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#if defined(GALACTIC) || defined(HUMBLE)
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#else
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#endif
#include <librealsense2/rs.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <realsense_hardware_interface/util.hpp>
#include <realsense_hardware_interface/visibility_control.hpp>
#include <string>
#include <vector>

namespace realsense_hardware_interface
{
class T265HardwareInterface
#if defined(GALACTIC) || defined(HUMBLE)
: public hardware_interface::SensorInterface
#else
: public hardware_interface::BaseInterface<hardware_interface::SensorInterface>
#endif
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(T265HardwareInterface)

#if defined(GALACTIC) || defined(HUMBLE)
  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
#else
  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;
#endif

  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

#ifndef GALACTIC
  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type start() override;

  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type stop() override;
#endif

  REALSENSE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

private:
  std::string serial_;
  std::string joint_;
  std::string left_image_key_;
  std::string right_image_key_;
  bool retrive_image_;
  rs2::pipeline pipe_;
  rs2::config cfg_;
  std::shared_ptr<Rs2PoseHandle> pose_handle_ptr_;
  std::shared_ptr<Rs2ImuHandle> imu_handle_ptr_;
  std::shared_ptr<rs2_imu> imu_;
  std::shared_ptr<Poco::SharedMemory> right_image_memory_ptr_;
  std::shared_ptr<Poco::SharedMemory> left_image_memory_ptr_;
  template <typename T>
  T getParameter(const std::string key, const hardware_interface::ComponentInfo & info) const
  {
    T param;
    getParameter(key, info, param);
    return param;
  }
  void getParameter(
    const std::string & key, const hardware_interface::ComponentInfo & info,
    std::string & parameter) const
  {
    try {
      parameter = info.parameters.at(key);
    } catch (std::out_of_range & e) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("REALSENSE_hardware_interface"),
        "parameter : " << key << " does not exist.");
    }
  }
  void getParameter(
    const std::string & key, const hardware_interface::ComponentInfo & info, int & parameter) const
  {
    std::string param_string;
    getParameter(key, info, param_string);
    parameter = std::stoi(param_string);
  }
  void getParameter(
    const std::string & key, const hardware_interface::ComponentInfo & info, bool & parameter) const
  {
    parameter = false;
    std::string param_string;
    getParameter(key, info, param_string);
    if (param_string == "true" || param_string == "True") {
      parameter = true;
    }
  }
  template <typename T>
  T getHardwareParameter(const std::string key) const
  {
    T param;
    getHardwareParameter(key, param);
    return param;
  }
  void getHardwareParameter(const std::string & key, std::string & parameter) const
  {
    try {
      parameter = info_.hardware_parameters.at(key);
    } catch (std::out_of_range & e) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("REALSENSE_hardware_interface"),
        "hardware parameter : " << key << " does not exist.");
    }
  }
  void getHardwareParameter(const std::string & key, int & parameter) const
  {
    std::string param_string;
    getHardwareParameter(key, param_string);
    parameter = std::stoi(param_string);
  }
  void getHardwareParameter(const std::string & key, bool & parameter) const
  {
    parameter = false;
    std::string param_string;
    getHardwareParameter(key, param_string);
    if (param_string == "true" || param_string == "True") {
      parameter = true;
    }
  }
};
}  // namespace realsense_hardware_interface

#endif  // REALSENSE_HARDWARE_INTERFACE__T265_HARDWARE_INTERFACE_HPP_
