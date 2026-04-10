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

#ifndef ANTBOT_HW_INTERFACE__DEVICE__WIPER_HPP_
#define ANTBOT_HW_INTERFACE__DEVICE__WIPER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "antbot_interfaces/srv/wiper_operation.hpp"

#include "antbot_hw_interface/device/device.hpp"

namespace antbot
{
namespace hw_interface
{
namespace device
{

using WiperOperationSrv = antbot_interfaces::srv::WiperOperation;

class Wiper : public Device
{
public:
  explicit Wiper(const std::string & name, const DeviceConfig & config);
  ~Wiper() = default;

  void update(std::unordered_map<std::string, double> & state_map) override;
  void activate() override;
  void deactivate() override;

private:
  void command(
    const std::shared_ptr<WiperOperationSrv::Request> request,
    std::shared_ptr<WiperOperationSrv::Response> response);

  rclcpp::Service<WiperOperationSrv>::SharedPtr operation_srv_;
};

}  // namespace device
}  // namespace hw_interface
}  // namespace antbot
#endif  // ANTBOT_HW_INTERFACE__DEVICE__WIPER_HPP_
