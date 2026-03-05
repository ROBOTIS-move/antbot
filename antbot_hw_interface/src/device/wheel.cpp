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

#include <algorithm>

#include "antbot_hw_interface/device/wheel.hpp"

namespace antbot
{
namespace hw_interface
{
device::Wheel::Wheel(const std::string & name, const DeviceConfig & config)
: Device(name, config)
{
  node_->declare_parameter("wheel.min_velocity", 0.0);
  min_velocity_ = node_->get_parameter("wheel.min_velocity").get_value<double>();
  node_->declare_parameter("wheel.max_velocity", 0.0);
  max_velocity_ = node_->get_parameter("wheel.max_velocity").get_value<double>();

  velocity_register_map_ = {
    {"M1_Present_RPM", "wheel_front_left_joint/velocity"},
    {"M2_Present_RPM", "wheel_front_right_joint/velocity"},
    {"M3_Present_RPM", "wheel_rear_left_joint/velocity"},
    {"M4_Present_RPM", "wheel_rear_right_joint/velocity"}
  };

  current_register_map_ = {
    {"M1_Present_Current", "wheel_front_left_joint/effort"},
    {"M2_Present_Current", "wheel_front_right_joint/effort"},
    {"M3_Present_Current", "wheel_rear_left_joint/effort"},
    {"M4_Present_Current", "wheel_rear_right_joint/effort"}
  };
}

void device::Wheel::update(std::unordered_map<std::string, double> & state_map)
{
  for (const auto & item : velocity_register_map_) {
    double rpm = static_cast<double>(get_data<int32_t>(item.first)) * constants::RCU_RPM_RESOLUTION;
    double linear_vel = constants::RPM_TO_RAD_PER_SEC * rpm;
    state_map[item.second] = linear_vel;
  }

  for (const auto & item : current_register_map_) {
    double current = static_cast<double>(
      get_data<int32_t>(item.first)) * constants::MILLIAMPERE_TO_AMPERE;
    state_map[item.second] = current;
  }
}

void device::Wheel::write_velocity(
  const double & cmd_front_left_velocity,
  const double & cmd_front_right_velocity,
  const double & cmd_rear_left_velocity,
  const double & cmd_rear_right_velocity)
{
  double front_left_vel = std::clamp(cmd_front_left_velocity, min_velocity_, max_velocity_);
  double front_right_vel = std::clamp(cmd_front_right_velocity, min_velocity_, max_velocity_);
  double rear_left_vel = std::clamp(cmd_rear_left_velocity, min_velocity_, max_velocity_);
  double rear_right_vel = std::clamp(cmd_rear_right_velocity, min_velocity_, max_velocity_);

  int32_t rpm[] = {
    static_cast<int32_t>(
      constants::RAD_PER_SEC_TO_RPM * front_left_vel) *
    constants::RCU_RPM_SCALE,
    static_cast<int32_t>(
      constants::RAD_PER_SEC_TO_RPM * front_right_vel) *
    constants::RCU_RPM_SCALE,
    static_cast<int32_t>(
      constants::RAD_PER_SEC_TO_RPM * rear_left_vel) *
    constants::RCU_RPM_SCALE,
    static_cast<int32_t>(
      constants::RAD_PER_SEC_TO_RPM * rear_right_vel) *
    constants::RCU_RPM_SCALE
  };

  std::string comm_message;
  bool success = communicator_->write_batch(
    "M1_Goal_RPM", rpm, velocity_register_map_.size(), &comm_message);

  if (!success) {
    RCLCPP_WARN_THROTTLE(
      Device::logger_,
      *node_->get_clock(),
      3000,
      "Failed to write rpm: %s",
      comm_message.c_str());
  }
}

void device::Wheel::write_acceleration(const double & acceleration)
{
  std::string comm_message = "";
  uint32_t accel = acceleration * constants::RAD_PER_SEC2_TO_REV_PER_MIN2 /
    constants::WHEEL_ACCEL_SCALE;
  bool success = write("Motor_Goal_Acceleration", accel, &comm_message);

  if (!success) {
    RCLCPP_WARN_THROTTLE(
      Device::logger_,
      *node_->get_clock(),
      3000,
      "Failed to write acceleration: %s",
      comm_message.c_str());
  }
}
}  // namespace hw_interface
}  // namespace antbot
