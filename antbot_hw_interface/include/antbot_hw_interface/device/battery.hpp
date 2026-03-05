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

#ifndef ANTBOT_HW_INTERFACE__DEVICE__BATTERY_HPP_
#define ANTBOT_HW_INTERFACE__DEVICE__BATTERY_HPP_

#include <string>
#include <unordered_map>

#include "sensor_msgs/msg/battery_state.hpp"

#include "antbot_hw_interface/device/device.hpp"

namespace antbot
{
namespace hw_interface
{
namespace device
{
class Battery : public Device
{
public:
  explicit Battery(const std::string & name, const DeviceConfig & config);
  ~Battery() = default;

  void update(std::unordered_map<std::string, double> & state_map) override;
  void publish(const rclcpp::Time & time) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr msg_publisher_;

  sensor_msgs::msg::BatteryState msg_;
  float voltage_{0.0};
  float current_{0.0};
  float capacity_{0.0};
  float percentage_{0.0};
  float temperature_{0.0};
  bool is_charging_{false};
};
}  // namespace device
}  // namespace hw_interface
}  // namespace antbot
#endif  // ANTBOT_HW_INTERFACE__DEVICE__BATTERY_HPP_
