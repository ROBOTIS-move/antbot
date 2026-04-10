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

#include "antbot_hw_interface/device/wiper.hpp"

namespace antbot
{
namespace hw_interface
{
device::Wiper::Wiper(const std::string & name, const DeviceConfig & config)
: Device(name, config)
{
}

void device::Wiper::update(std::unordered_map<std::string, double> &)
{
}

void device::Wiper::activate()
{
  operation_srv_ = node_->create_service<WiperOperationSrv>(
    "wiper/operation",
    std::bind(
      &Wiper::command, this,
      std::placeholders::_1, std::placeholders::_2));
}

void device::Wiper::deactivate()
{
  operation_srv_.reset();
}

void device::Wiper::command(
  const std::shared_ptr<WiperOperationSrv::Request> request,
  std::shared_ptr<WiperOperationSrv::Response> response)
{
  if (request->mode != WiperOperationSrv::Request::OFF &&
    request->mode != WiperOperationSrv::Request::ONCE &&
    request->mode != WiperOperationSrv::Request::REPEAT)
  {
    response->success = false;
    response->message = "Invalid wiper mode";
    RCLCPP_WARN(logger_, "Invalid wiper mode: %d", request->mode);
    return;
  }

  // Convert cycle_time (sec) to control table unit (*10)
  uint8_t interval = static_cast<uint8_t>(request->cycle_time * 10.0f);

  // Write Wiper_Mode and Wiper_INT_Time together
  uint8_t values[2] = {request->mode, interval};
  std::string comm_message;
  response->success = communicator_->write_batch<uint8_t>(
    "Wiper_Mode", values, 2, &comm_message);

  if (response->success) {
    RCLCPP_INFO(
      logger_, "Wiper operation: mode=%d, cycle_time=%.1f sec",
      request->mode, request->cycle_time);
  } else {
    response->message = comm_message;
    RCLCPP_ERROR(logger_, "Wiper operation error: %s", comm_message.c_str());
  }
}
}  // namespace hw_interface
}  // namespace antbot
