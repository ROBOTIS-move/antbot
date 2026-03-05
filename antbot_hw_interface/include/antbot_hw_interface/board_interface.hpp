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

#ifndef ANTBOT_HW_INTERFACE__BOARD_INTERFACE_HPP_
#define ANTBOT_HW_INTERFACE__BOARD_INTERFACE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"

#include "antbot_interfaces/srv/direct_read.hpp"
#include "antbot_interfaces/srv/direct_write.hpp"
#include "antbot_libs/communicator.hpp"
#include "antbot_libs/node_thread.hpp"
#include "antbot_hw_interface/device/device.hpp"
#include "antbot_hw_interface/device/steering.hpp"
#include "antbot_hw_interface/device/wheel.hpp"

namespace antbot
{
namespace hw_interface
{

class BoardInterface : public hardware_interface::SystemInterface
{
public:
  BoardInterface() = default;
  ~BoardInterface();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  bool connect_to_board();

  rclcpp::Logger logger_{rclcpp::get_logger("board_hw_interface")};
  rclcpp::Node::SharedPtr node_;

  std::string serial_port_;
  uint16_t board_id_{0};
  uint32_t baud_rate_{0};
  float protocol_version_{0.0f};
  std::string control_table_path_;

  std::shared_ptr<antbot::libs::Communicator> communicator_;

  std::vector<std::unique_ptr<device::Device>> device_list_;
  device::Wheel * wheel_device_{nullptr};
  device::Steering * steering_device_{nullptr};

  std::unordered_map<std::string, double> command_map_;
  std::unordered_map<std::string, double> state_map_;
  std::unique_ptr<antbot::libs::NodeThread> node_thread_;

  rclcpp::Service<antbot_interfaces::srv::DirectRead>::SharedPtr direct_read_srv_;
  rclcpp::Service<antbot_interfaces::srv::DirectWrite>::SharedPtr direct_write_srv_;
};
}  // namespace hw_interface
}  // namespace antbot
#endif  // ANTBOT_HW_INTERFACE__BOARD_INTERFACE_HPP_
