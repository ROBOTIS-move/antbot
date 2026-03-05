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

#ifndef ANTBOT_HW_INTERFACE__DEVICE__HEADLIGHT_HPP_
#define ANTBOT_HW_INTERFACE__DEVICE__HEADLIGHT_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "std_srvs/srv/set_bool.hpp"

#include "antbot_hw_interface/device/device.hpp"

namespace antbot
{
namespace hw_interface
{
namespace device
{

class Headlight : public Device
{
public:
  explicit Headlight(const std::string & name, const DeviceConfig & config);
  ~Headlight() = default;

  void update(std::unordered_map<std::string, double> & state_map) override;
  void activate() override;
  void deactivate() override;

private:
  static constexpr int32_t HEADLIGHT_ON = 1;
  static constexpr int32_t HEADLIGHT_OFF = 0;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr operation_srv_;
};

}  // namespace device
}  // namespace hw_interface
}  // namespace antbot
#endif  // ANTBOT_HW_INTERFACE__DEVICE__HEADLIGHT_HPP_
