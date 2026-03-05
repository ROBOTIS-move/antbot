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

#ifndef ANTBOT_HW_INTERFACE__DEVICE__STEERING_HPP_
#define ANTBOT_HW_INTERFACE__DEVICE__STEERING_HPP_

#include <string>
#include <unordered_map>
#include <vector>
#include <utility>

#include "antbot_hw_interface/device/device.hpp"

namespace antbot
{
namespace hw_interface
{
namespace device
{
class Steering : public Device
{
public:
  explicit Steering(const std::string & name, const DeviceConfig & config);
  ~Steering() = default;

  void update(std::unordered_map<std::string, double> & state_map) override;

  void write_position(
    const double & cmd_front_left_pos,
    const double & cmd_front_right_pos,
    const double & cmd_rear_left_pos,
    const double & cmd_rear_right_pos);
  void write_acceleration(
    const double & cmd_front_left_acc,
    const double & cmd_front_right_acc,
    const double & cmd_rear_left_acc,
    const double & cmd_rear_right_acc);
  void write_velocity(
    const double & cmd_front_left_vel,
    const double & cmd_front_right_vel,
    const double & cmd_rear_left_vel,
    const double & cmd_rear_right_vel);

private:
  double gear_ratio_{0.0};
  double front_left_min_rad_{0.0};
  double front_left_max_rad_{0.0};
  double front_right_min_rad_{0.0};
  double front_right_max_rad_{0.0};
  double rear_left_min_rad_{0.0};
  double rear_left_max_rad_{0.0};
  double rear_right_min_rad_{0.0};
  double rear_right_max_rad_{0.0};

  std::vector<std::pair<std::string, std::string>> position_register_map_;
  std::vector<std::pair<std::string, std::string>> current_register_map_;
};
}  // namespace device
}  // namespace hw_interface
}  // namespace antbot
#endif  // ANTBOT_HW_INTERFACE__DEVICE__STEERING_HPP_
