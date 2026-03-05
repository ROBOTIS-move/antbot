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

#include <cinttypes>

#include "antbot_hw_interface/device/encoder.hpp"

namespace antbot
{
namespace hw_interface
{
device::Encoder::Encoder(const std::string & name, const DeviceConfig & config)
: Device(name, config)
{
  position_joints_ = {
    "wheel_front_left_joint/position",
    "wheel_front_right_joint/position",
    "wheel_rear_left_joint/position",
    "wheel_rear_right_joint/position"
  };

  const size_t num_motors = position_joints_.size();
  RCLCPP_INFO(Device::logger_, "Encoder device initialized with %zu motors", num_motors);

  current_raw_tick_.resize(num_motors, 0);
  previous_raw_tick_.resize(num_motors, 0);
  current_tick_.resize(num_motors, 0);
  position_rad_.resize(num_motors, 0.0);
  was_rebooted_.resize(num_motors, 0);
}

void device::Encoder::update(std::unordered_map<std::string, double> & state_map)
{
  uint8_t motor_state = get_data<uint8_t>("Motor_State");
  bool is_motor_state_okay = (motor_state == constants::MOTOR_READY);

  uint8_t motor_reboot_check_num = get_data<uint8_t>("Motor_Reboot_Check");

  if (motor_reboot_check_num != constants::MOTOR_REBOOT_NORMAL) {
    for (size_t i = 0; i < position_joints_.size(); i++) {
      std::string item_position = "M" + std::to_string(i + 1) + "_Present_Position";
      current_raw_tick_[i] = get_data<int32_t>(item_position);

      was_rebooted_[i] = true;
      RCLCPP_WARN(
        Device::logger_,
        "Detected motor reboot. Keep motor(%zu) current_tick[%" PRId64 "]", i, current_tick_[i]);
    }
    return;
  }

  if (is_motor_state_okay) {
    for (size_t i = 0; i < position_joints_.size(); i++) {
      std::string item_position = "M" + std::to_string(i + 1) + "_Present_Position";
      current_raw_tick_[i] = get_data<int32_t>(item_position);

      // wrapping-aware tick difference calculation
      uint32_t curr_u32 = static_cast<uint32_t>(current_raw_tick_[i]);
      uint32_t prev_u32 = static_cast<uint32_t>(previous_raw_tick_[i]);
      int64_t delta = 0;

      if (was_rebooted_[i]) {
        was_rebooted_[i] = false;
        RCLCPP_INFO(
          Device::logger_,
          "Motor reboot complete. Resetting tick reference for motor %zu. current_raw: %d",
          i, current_raw_tick_[i]);
      } else {
        delta = static_cast<int64_t>(curr_u32) - static_cast<int64_t>(prev_u32);
      }

      if (delta > INT32_MAX) {
        delta -= static_cast<int64_t>(UINT32_MAX) + 1;
        RCLCPP_INFO(
          Device::logger_,
          "Detected [%zu]encoder underflow. "
          "current raw: %d(%u), previous raw: %d(%u), delta corrected to %" PRId64,
          i,
          current_raw_tick_[i],
          curr_u32,
          previous_raw_tick_[i],
          prev_u32,
          delta);
      } else if (delta < -static_cast<int64_t>(INT32_MAX)) {
        delta += static_cast<int64_t>(UINT32_MAX) + 1;
        RCLCPP_INFO(
          Device::logger_,
          "Detected [%zu]encoder overflow. "
          "current raw: %d(%u), previous raw: %d(%u), delta corrected to %" PRId64,
          i,
          current_raw_tick_[i],
          curr_u32,
          previous_raw_tick_[i],
          prev_u32,
          delta);
      }

      current_tick_[i] += delta;
      previous_raw_tick_[i] = current_raw_tick_[i];
      position_rad_[i] = static_cast<double>(current_tick_[i]) * constants::TICK_TO_RAD;
    }
  } else {
    RCLCPP_WARN_THROTTLE(
      Device::logger_,
      *node_->get_clock(),
      3000,
      "Failed to get present position, check motor state : %d",
      motor_state);
  }

  for (size_t i = 0; i < position_joints_.size(); ++i) {
    state_map[position_joints_[i]] = position_rad_[i];
  }
}
}  // namespace hw_interface
}  // namespace antbot
