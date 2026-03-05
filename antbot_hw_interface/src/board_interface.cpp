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

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "antbot_hw_interface/board_interface.hpp"
#include "antbot_hw_interface/device/battery.hpp"
#include "antbot_hw_interface/device/cargo.hpp"
#include "antbot_hw_interface/device/encoder.hpp"
#include "antbot_hw_interface/device/headlight.hpp"
#include "antbot_hw_interface/device/ultrasound.hpp"
#include "antbot_hw_interface/device/wiper.hpp"

namespace antbot
{
namespace hw_interface
{
BoardInterface::~BoardInterface()
{
  if (wheel_device_ != nullptr) {
    wheel_device_->write_velocity(0, 0, 0, 0);
    wheel_device_ = nullptr;
  }
  steering_device_ = nullptr;

  communicator_.reset();
}

hardware_interface::CallbackReturn BoardInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ = rclcpp::Node::make_shared("board_hw_interface");

  node_->declare_parameter("serial_port", "");
  serial_port_ = node_->get_parameter("serial_port").get_value<std::string>();
  node_->declare_parameter("board_id", 0);
  board_id_ = node_->get_parameter("board_id").get_value<uint16_t>();
  node_->declare_parameter("baud_rate", 0);
  baud_rate_ = node_->get_parameter("baud_rate").get_value<uint32_t>();
  node_->declare_parameter("protocol_version", 0.0f);
  protocol_version_ = node_->get_parameter("protocol_version").get_value<float>();
  node_->declare_parameter("control_table_path", "");
  control_table_path_ = node_->get_parameter("control_table_path").get_value<std::string>();

  node_thread_ = std::make_unique<libs::NodeThread>(node_->get_node_base_interface());

  if (!connect_to_board()) {
    RCLCPP_ERROR(logger_, "Failed to connect to board");
    return hardware_interface::CallbackReturn::ERROR;
  }

  device::DeviceConfig config(node_, communicator_);

  wheel_device_ = device::add_device_with_return<device::Wheel>(
    "wheel", config, device_list_);
  steering_device_ = device::add_device_with_return<device::Steering>(
    "steering", config, device_list_);
  device::add_device<device::Battery>("battery", config, device_list_);
  device::add_device<device::Cargo>("cargo", config, device_list_);
  device::add_device<device::Encoder>("encoder", config, device_list_);
  device::add_device<device::UltraSound>("ultrasound", config, device_list_);
  device::add_device<device::Headlight>("headlight", config, device_list_);
  device::add_device<device::Wiper>("wiper", config, device_list_);

  direct_write_srv_ = node_->create_service<antbot_interfaces::srv::DirectWrite>(
    "hw/direct_write",
    [this](
      const std::shared_ptr<antbot_interfaces::srv::DirectWrite::Request> request,
      std::shared_ptr<antbot_interfaces::srv::DirectWrite::Response> response) -> void
    {
      response->success = communicator_->write(
        request->item_name,
        static_cast<uint32_t>(request->data),
        &response->message);
    });

  direct_read_srv_ = node_->create_service<antbot_interfaces::srv::DirectRead>(
    "hw/direct_read",
    [this](
      const std::shared_ptr<antbot_interfaces::srv::DirectRead::Request> request,
      std::shared_ptr<antbot_interfaces::srv::DirectRead::Response> response) -> void
    {
      response->data = communicator_->get_data<int32_t>(request->item_name);
      auto * item = communicator_->get_control_item(request->item_name);
      if (item) {
        response->message = "addr[" + std::to_string(item->address) + "]";
      } else {
        response->message = "item not found";
      }
    });

  RCLCPP_INFO(logger_, "Initialized board hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BoardInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  for (auto & pair : state_map_) {
    pair.second = 0.0;
  }
  for (auto & pair : command_map_) {
    pair.second = 0.0;
  }

  RCLCPP_INFO(logger_, "Configured board hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BoardInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  for (const auto & device : device_list_) {
    device->activate();
  }

  RCLCPP_INFO(logger_, "Activated board hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BoardInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  for (const auto & device : device_list_) {
    device->deactivate();
  }

  RCLCPP_INFO(logger_, "Deactivated board hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
BoardInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (const hardware_interface::ComponentInfo & sensor : info_.sensors) {
    for (const hardware_interface::InterfaceInfo & i : sensor.state_interfaces) {
      std::string key_name = sensor.name + "/" + i.name;
      state_map_[key_name] = 0.0;
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(sensor.name, i.name, &state_map_[key_name]));
      RCLCPP_DEBUG(logger_, "sensor state: %s", key_name.c_str());
    }
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    for (const hardware_interface::InterfaceInfo & i : joint.state_interfaces) {
      std::string key_name = joint.name + "/" + i.name;
      state_map_[key_name] = 0.0;
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(joint.name, i.name, &state_map_[key_name]));
      RCLCPP_DEBUG(logger_, "joint state: %s", key_name.c_str());
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
BoardInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    for (const hardware_interface::InterfaceInfo & i : joint.command_interfaces) {
      std::string key_name = joint.name + "/" + i.name;
      command_map_[key_name] = 0.0;
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint.name, i.name, &command_map_[key_name]));
      RCLCPP_DEBUG(logger_, "joint command: %s", key_name.c_str());
    }
  }

  return command_interfaces;
}

hardware_interface::return_type BoardInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  int result = 0;
  if (!communicator_->read_control_table(&result)) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *node_->get_clock(), 3000,
      "Failed to read control table: %d", result);
  }

  for (const auto & device : device_list_) {
    device->update(state_map_);
    device->publish(time);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BoardInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (wheel_device_ != nullptr) {
    wheel_device_->write_velocity(
      command_map_.at("wheel_front_left_joint/velocity"),
      command_map_.at("wheel_front_right_joint/velocity"),
      command_map_.at("wheel_rear_left_joint/velocity"),
      command_map_.at("wheel_rear_right_joint/velocity"));
    wheel_device_->write_acceleration(command_map_.at("wheel_front_left_joint/acceleration"));
  }

  if (steering_device_ != nullptr) {
    steering_device_->write_position(
      command_map_.at("steering_front_left_joint/position"),
      command_map_.at("steering_front_right_joint/position"),
      command_map_.at("steering_rear_left_joint/position"),
      command_map_.at("steering_rear_right_joint/position"));
    steering_device_->write_acceleration(
      command_map_.at("steering_front_left_joint/acceleration"),
      command_map_.at("steering_front_right_joint/acceleration"),
      command_map_.at("steering_rear_left_joint/acceleration"),
      command_map_.at("steering_rear_right_joint/acceleration"));
    steering_device_->write_velocity(
      command_map_.at("steering_front_left_joint/velocity"),
      command_map_.at("steering_front_right_joint/velocity"),
      command_map_.at("steering_rear_left_joint/velocity"),
      command_map_.at("steering_rear_right_joint/velocity"));
  }

  uint8_t motor_reboot_check_num =
    communicator_->get_data<uint8_t>("Motor_Reboot_Check");
  if (motor_reboot_check_num != constants::MOTOR_REBOOT_NORMAL) {
    std::string comm_message;
    bool success = communicator_->write(
      "Motor_Reboot_Check", motor_reboot_check_num, &comm_message);
    if (!success) {
      RCLCPP_WARN(
        logger_,
        "Failed to write Motor_Reboot_Check(num: %d): %s",
        motor_reboot_check_num,
        comm_message.c_str());
    }
  }

  return hardware_interface::return_type::OK;
}

bool BoardInterface::connect_to_board()
{
  communicator_ = libs::create_communicator(
    serial_port_, baud_rate_, protocol_version_,
    static_cast<uint8_t>(board_id_), control_table_path_);

  if (!communicator_) {
    RCLCPP_ERROR(logger_, "Failed to connect to board");
    return false;
  }

  if (!communicator_->read_control_table()) {
    RCLCPP_ERROR(logger_, "Failed to read initial control table");
    return false;
  }

  RCLCPP_INFO(
    logger_, "  Model Number: %d", communicator_->get_model_number());
  RCLCPP_INFO(
    logger_, "  F/W ver: %d.%d",
    communicator_->get_firmware_major_version(),
    communicator_->get_firmware_minor_version());

  return true;
}
}  // namespace hw_interface
}  // namespace antbot

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  antbot::hw_interface::BoardInterface,
  hardware_interface::SystemInterface
)
