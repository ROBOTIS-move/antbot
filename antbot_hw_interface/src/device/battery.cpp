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

#include "antbot_hw_interface/device/battery.hpp"

namespace antbot
{
namespace hw_interface
{
device::Battery::Battery(const std::string & name, const DeviceConfig & config)
: Device(name, config)
{
  msg_publisher_ =
    node_->create_publisher<sensor_msgs::msg::BatteryState>("battery", qos_);
}

void device::Battery::update(std::unordered_map<std::string, double> &)
{
  current_ = get_data<int16_t>("Battery_Current") * 10.0f * 0.001f;  // mA (scale=10) → A
  voltage_ = get_data<int16_t>("Battery_Voltage") * 0.01f;  // V (scale=0.01)
  capacity_ = get_data<uint16_t>("Battery_Capacity") * 0.01f;  // Ah (scale=0.01)
  percentage_ = get_data<uint8_t>("Battery_Percentage") * 0.01f;  // % → ratio (0.0~1.0)
  temperature_ = static_cast<float>(get_data<int8_t>("BMS_Temperature"));  // °C
  constexpr uint8_t CHARGE = 1;
  is_charging_ = (get_data<uint8_t>("Battery_Is_Charging") == CHARGE);
}

void device::Battery::publish(const rclcpp::Time & time)
{
  if (is_charging_) {
    msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  } else {
    msg_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  }

  msg_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  msg_.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

  msg_.header.stamp = time;
  msg_.voltage = voltage_;
  msg_.current = current_;
  msg_.capacity = capacity_;
  msg_.percentage = percentage_;
  msg_.temperature = temperature_;
  msg_.present = true;

  msg_publisher_->publish(msg_);
}
}  // namespace hw_interface
}  // namespace antbot
