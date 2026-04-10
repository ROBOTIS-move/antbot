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

#include "antbot_hw_interface/device/headlight.hpp"

namespace antbot
{
namespace hw_interface
{
device::Headlight::Headlight(const std::string & name, const DeviceConfig & config)
: Device(name, config)
{
}

void device::Headlight::update(std::unordered_map<std::string, double> &)
{
}

void device::Headlight::activate()
{
  operation_srv_ = node_->create_service<std_srvs::srv::SetBool>(
    "headlight/operation",
    [this](
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
    {
      int32_t operation = request->data ? HEADLIGHT_ON : HEADLIGHT_OFF;

      std::string comm_message;
      response->success = write(
        "Headlight_State", static_cast<uint32_t>(operation), &comm_message);

      if (response->success) {
        RCLCPP_INFO(
          logger_, "Headlight operation: %s",
          request->data ? "ON" : "OFF");
      } else {
        response->message = comm_message;
        RCLCPP_ERROR(logger_, "Headlight operation error: %s", comm_message.c_str());
      }
    });
}

void device::Headlight::deactivate()
{
  operation_srv_.reset();
}
}  // namespace hw_interface
}  // namespace antbot
