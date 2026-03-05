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

#include "antbot_hw_interface/device/cargo.hpp"

namespace antbot
{
namespace hw_interface
{
device::Cargo::Cargo(const std::string & name, const DeviceConfig & config)
: Device(name, config)
{
  status_publisher_ =
    node_->create_publisher<CargoStatusMsg>("cargo/status", qos_);
}

void device::Cargo::update(std::unordered_map<std::string, double> &)
{
  constexpr uint8_t CLOSED = 0;
  uint8_t door_state = get_data<uint8_t>("Cargo_Door_State");
  if (door_state == CLOSED) {
    status_msg_.door_status = CargoStatusMsg::DOOR_STATUS_CLOSED;
  } else {
    status_msg_.door_status = CargoStatusMsg::DOOR_STATUS_OPENED;
  }

  uint8_t lock_state = get_data<uint8_t>("Cargo_Lock_State");
  constexpr uint8_t RAW_LOCKED = 1;
  constexpr uint8_t RAW_UNLOCKED = 2;
  if (lock_state == RAW_LOCKED) {
    status_msg_.lock_status = CargoStatusMsg::LOCK_STATUS_LOCKED;
  } else if (lock_state == RAW_UNLOCKED) {
    status_msg_.lock_status = CargoStatusMsg::LOCK_STATUS_UNLOCKED;
  } else {
    status_msg_.lock_status = CargoStatusMsg::LOCK_STATUS_NEUTRAL;
  }
}

void device::Cargo::publish(const rclcpp::Time &)
{
  status_publisher_->publish(status_msg_);
}

void device::Cargo::activate()
{
  command_service_ = node_->create_service<CargoCommandSrv>(
    "cargo/command",
    std::bind(
      &Cargo::cargo_command_callback, this,
      std::placeholders::_1, std::placeholders::_2));
}

void device::Cargo::deactivate()
{
  command_service_.reset();
}

void device::Cargo::cargo_command_callback(
  const std::shared_ptr<CargoCommandSrv::Request> request,
  std::shared_ptr<CargoCommandSrv::Response> response)
{
  if (operation_map_.count(request->operation) == 0) {
    response->success = false;
    response->message = "Invalid operation";
    RCLCPP_WARN(logger_, "Invalid cargo operation: %d", request->operation);
    return;
  }

  std::string comm_message;
  uint32_t value = static_cast<uint32_t>(operation_map_[request->operation]);
  response->success = write("Cargo_Command", value, &comm_message);
  if (!response->success) {
    response->message = comm_message;
    RCLCPP_WARN(logger_, "Failed to command cargo: %s", comm_message.c_str());
  } else {
    RCLCPP_INFO(logger_, "Cargo command succeeded: %d", request->operation);
  }
}
}  // namespace hw_interface
}  // namespace antbot
