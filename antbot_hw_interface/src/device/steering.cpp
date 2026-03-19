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

#include "antbot_hw_interface/device/steering.hpp"

namespace antbot
{
namespace hw_interface
{
device::Steering::Steering(const std::string & name, const DeviceConfig & config)
: device::Device(name, config)
{
  node_->declare_parameter("steering.gear_ratio", 0.0);
  gear_ratio_ = node_->get_parameter("steering.gear_ratio").get_value<double>();
  node_->declare_parameter("steering.front_left_min_degree", 0.0);
  front_left_min_rad_ = node_->get_parameter("steering.front_left_min_degree").
    get_value<double>() * constants::DEG_TO_RAD;
  node_->declare_parameter("steering.front_left_max_degree", 0.0);
  front_left_max_rad_ = node_->get_parameter("steering.front_left_max_degree").
    get_value<double>() * constants::DEG_TO_RAD;
  node_->declare_parameter("steering.front_right_min_degree", 0.0);
  front_right_min_rad_ = node_->get_parameter("steering.front_right_min_degree").
    get_value<double>() * constants::DEG_TO_RAD;
  node_->declare_parameter("steering.front_right_max_degree", 0.0);
  front_right_max_rad_ = node_->get_parameter("steering.front_right_max_degree").
    get_value<double>() * constants::DEG_TO_RAD;
  node_->declare_parameter("steering.rear_left_min_degree", 0.0);
  rear_left_min_rad_ = node_->get_parameter("steering.rear_left_min_degree").
    get_value<double>() * constants::DEG_TO_RAD;
  node_->declare_parameter("steering.rear_left_max_degree", 0.0);
  rear_left_max_rad_ = node_->get_parameter("steering.rear_left_max_degree").
    get_value<double>() * constants::DEG_TO_RAD;
  node_->declare_parameter("steering.rear_right_min_degree", 0.0);
  rear_right_min_rad_ = node_->get_parameter("steering.rear_right_min_degree").
    get_value<double>() * constants::DEG_TO_RAD;
  node_->declare_parameter("steering.rear_right_max_degree", 0.0);
  rear_right_max_rad_ = node_->get_parameter("steering.rear_right_max_degree").
    get_value<double>() * constants::DEG_TO_RAD;

  position_register_map_ = {
    {"S1_Present_Position", "steering_front_left_joint/position"},
    {"S2_Present_Position", "steering_front_right_joint/position"},
    {"S3_Present_Position", "steering_rear_left_joint/position"},
    {"S4_Present_Position", "steering_rear_right_joint/position"}
  };

  current_register_map_ = {
    {"S1_Present_Current", "steering_front_left_joint/effort"},
    {"S2_Present_Current", "steering_front_right_joint/effort"},
    {"S3_Present_Current", "steering_rear_left_joint/effort"},
    {"S4_Present_Current", "steering_rear_right_joint/effort"}
  };
}

void device::Steering::update(std::unordered_map<std::string, double> & state_map)
{
  for (const auto & item : position_register_map_) {
    double radian = static_cast<double>(
      get_data<int32_t>(item.first) - constants::STEERING_PULSE_OFFSET) *
      constants::STEERING_RAD_PER_PULSE;
    state_map[item.second] = radian / gear_ratio_;
  }

  for (const auto & item : current_register_map_) {
    double current = static_cast<double>(
      get_data<int32_t>(item.first)) * constants::MILLIAMPERE_TO_AMPERE;
    state_map[item.second] = current;
  }
}

void device::Steering::sync_commands_to_current_state(
  const std::unordered_map<std::string, double> & state_map,
  std::unordered_map<std::string, double> & command_map)
{
  for (const auto & item : position_register_map_) {
    auto it = state_map.find(item.second);
    if (it != state_map.end()) {
      command_map[item.second] = it->second;
    }
  }
}

void device::Steering::write_position(
  const double & cmd_front_left_pos,
  const double & cmd_front_right_pos,
  const double & cmd_rear_left_pos,
  const double & cmd_rear_right_pos)
{
  double front_left_pos = std::clamp(
    cmd_front_left_pos,
    front_left_min_rad_,
    front_left_max_rad_) * gear_ratio_;
  double front_right_pos = std::clamp(
    cmd_front_right_pos,
    front_right_min_rad_,
    front_right_max_rad_) * gear_ratio_;
  double rear_left_pos = std::clamp(
    cmd_rear_left_pos,
    rear_left_min_rad_,
    rear_left_max_rad_) * gear_ratio_;
  double rear_right_pos = std::clamp(
    cmd_rear_right_pos,
    rear_right_min_rad_,
    rear_right_max_rad_) * gear_ratio_;

  int32_t position[] = {
    static_cast<int32_t>(
      front_left_pos / constants::STEERING_RAD_PER_PULSE +
      constants::STEERING_PULSE_OFFSET),
    static_cast<int32_t>(
      front_right_pos / constants::STEERING_RAD_PER_PULSE +
      constants::STEERING_PULSE_OFFSET),
    static_cast<int32_t>(
      rear_left_pos / constants::STEERING_RAD_PER_PULSE +
      constants::STEERING_PULSE_OFFSET),
    static_cast<int32_t>(
      rear_right_pos / constants::STEERING_RAD_PER_PULSE +
      constants::STEERING_PULSE_OFFSET)
  };

  std::string comm_message;
  bool success = communicator_->write_batch(
    "S1_Goal_Position", position, position_register_map_.size(), &comm_message);

  if (!success) {
    RCLCPP_WARN_THROTTLE(
      Device::logger_,
      *node_->get_clock(),
      3000,
      "Failed to write steering position(radian): %s",
      comm_message.c_str());
  }
}

void device::Steering::write_acceleration(
  const double & cmd_front_left_acc,
  const double & cmd_front_right_acc,
  const double & cmd_rear_left_acc,
  const double & cmd_rear_right_acc)
{
  uint32_t acceleration[] = {
    static_cast<uint32_t>(
      cmd_front_left_acc *
      constants::RAD_PER_SEC2_TO_REV_PER_MIN2 /
      constants::STEERING_ACCEL_SCALE),
    static_cast<uint32_t>(
      cmd_front_right_acc *
      constants::RAD_PER_SEC2_TO_REV_PER_MIN2 /
      constants::STEERING_ACCEL_SCALE),
    static_cast<uint32_t>(
      cmd_rear_left_acc *
      constants::RAD_PER_SEC2_TO_REV_PER_MIN2 /
      constants::STEERING_ACCEL_SCALE),
    static_cast<uint32_t>(
      cmd_rear_right_acc *
      constants::RAD_PER_SEC2_TO_REV_PER_MIN2 /
      constants::STEERING_ACCEL_SCALE)
  };

  std::string comm_message;
  bool success = communicator_->write_batch(
    "S1_Profile_Acceleration", acceleration, position_register_map_.size(), &comm_message);

  if (!success) {
    RCLCPP_WARN_THROTTLE(
      Device::logger_,
      *node_->get_clock(),
      3000,
      "Failed to write steering acceleration: %s",
      comm_message.c_str());
  }
}

void device::Steering::write_velocity(
  const double & cmd_front_left_vel,
  const double & cmd_front_right_vel,
  const double & cmd_rear_left_vel,
  const double & cmd_rear_right_vel)
{
  uint32_t velocity[] = {
    static_cast<uint32_t>(
      cmd_front_left_vel * constants::RAD_PER_SEC_TO_RPM /
      constants::DXL_VELOCITY_UNIT),
    static_cast<uint32_t>(
      cmd_front_right_vel * constants::RAD_PER_SEC_TO_RPM /
      constants::DXL_VELOCITY_UNIT),
    static_cast<uint32_t>(
      cmd_rear_left_vel * constants::RAD_PER_SEC_TO_RPM /
      constants::DXL_VELOCITY_UNIT),
    static_cast<uint32_t>(
      cmd_rear_right_vel * constants::RAD_PER_SEC_TO_RPM /
      constants::DXL_VELOCITY_UNIT)
  };

  std::string comm_message;
  bool success = communicator_->write_batch(
    "S1_Profile_Velocity", velocity, position_register_map_.size(), &comm_message);

  if (!success) {
    RCLCPP_WARN_THROTTLE(
      Device::logger_,
      *node_->get_clock(),
      3000,
      "Failed to write steering velocity: %s",
      comm_message.c_str());
  }
}
}  // namespace hw_interface
}  // namespace antbot
