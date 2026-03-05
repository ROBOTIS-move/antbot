// Copyright 2026 ROBOTIS AI CO., LTD.
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
//
// Authors: Jaehun Park

#include "antbot_camera/camera_driver.hpp"

namespace antbot_camera
{

CameraDriver::CameraDriver(const std::string & position, rclcpp::Node * node)
: node_(node), position_(position)
{
}

DriverStatus CameraDriver::get_status() const
{
  return status_.load();
}

const FrameData & CameraDriver::get_frame_data() const
{
  return frame_data_;
}

std::string CameraDriver::get_position() const
{
  return position_;
}

void CameraDriver::set_status(DriverStatus s)
{
  status_.store(s);
}

}  // namespace antbot_camera
