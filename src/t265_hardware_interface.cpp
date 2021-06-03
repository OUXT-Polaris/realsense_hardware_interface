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

namespace realsense_hardware_interface
{
hardware_interface::return_type T265HardwareInterface::start()
{
  cfg_.enable_stream(RS2_STREAM_GYRO);
  cfg_.enable_stream(RS2_STREAM_ACCEL);
  cfg_.enable_stream(RS2_STREAM_POSE);
  return hardware_interface::return_type::OK;
}
}  // namespace realsense_hardware_interface