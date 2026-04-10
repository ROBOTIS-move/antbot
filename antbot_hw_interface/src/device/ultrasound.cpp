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
// Authors: Daun Jeong

#include "antbot_hw_interface/device/ultrasound.hpp"

namespace antbot
{
namespace hw_interface
{
device::UltraSound::UltraSound(const std::string & name, const DeviceConfig & config)
: Device(name, config)
{
  ultrasound_items_ = {
    "UltraSonic_1",
    "UltraSonic_2"
  };

  msg_publisher_ =
    node_->create_publisher<std_msgs::msg::Float64MultiArray>("ultrasound", qos_);
  msg_.data.resize(ultrasound_items_.size(), 0.0);
}

void device::UltraSound::update(std::unordered_map<std::string, double> &)
{
  constexpr double ULTRASONIC_SCALE = 0.01;

  for (size_t i = 0; i < ultrasound_items_.size(); i++) {
    uint16_t raw_data = get_data<uint16_t>(ultrasound_items_[i]);
    msg_.data[i] = raw_data * ULTRASONIC_SCALE;
  }
}

void device::UltraSound::publish(const rclcpp::Time &)
{
  msg_publisher_->publish(msg_);
}
}  // namespace hw_interface
}  // namespace antbot
