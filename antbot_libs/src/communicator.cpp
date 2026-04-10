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
// Authors: Darby Lim, Daun Jeong

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "antbot_libs/communicator.hpp"

namespace antbot
{
namespace libs
{
Communicator::Communicator(
  dynamixel::PortHandler * port_handler,
  dynamixel::PacketHandler * packet_handler)
{
  port_handler_ = port_handler;
  packet_handler_ = packet_handler;
}

Communicator::~Communicator()
{
  if (port_handler_ != nullptr) {
    port_handler_->closePort();
    delete port_handler_;
    port_handler_ = nullptr;
  }
  // packet_handler_ is a SDK singleton — do not delete
}

bool Communicator::init(uint8_t id, const std::string & control_table_xml_path)
{
  device_id_ = id;

  if (!init_control_table(control_table_xml_path)) {
    RCLCPP_ERROR(logger_, "Failed to initialize control table for ID: %d", id);
    return false;
  }

  data_.assign(READ_BUFFER_SIZE, 0);

  return true;
}

bool Communicator::init_control_table(const std::string & xml_file_path)
{
  auto parser = std::make_unique<ControlTableParser>();

  if (parser->load_xml_file(xml_file_path.c_str())) {
    RCLCPP_INFO(logger_, "Loaded control table XML for ID: %d", device_id_);
  } else {
    RCLCPP_ERROR(logger_, "Failed to load xml file for ID: %d", device_id_);
    return false;
  }

  if (parser->parse_control_table()) {
    RCLCPP_INFO(logger_, "Parsed control table for ID: %d", device_id_);
  } else {
    RCLCPP_ERROR(logger_, "Failed to parse control table for ID: %d", device_id_);
    return false;
  }

  auto table = parser->get_control_table();
  if (table.empty()) {
    RCLCPP_ERROR(logger_, "Couldn't get control table for ID: %d", device_id_);
    return false;
  }

  control_table_ = table;
  min_address_ = parser->parse_min_address();
  max_address_ = parser->parse_max_address();

  return true;
}

bool Communicator::is_connected_to_device()
{
  auto it = control_table_.find("ID");
  if (it == control_table_.end()) {
    RCLCPP_ERROR(logger_, "Control table missing 'ID' item");
    return false;
  }

  const ControlTableParser::ControlItem * control_item = &it->second;

  uint8_t read_id;
  const char * log = nullptr;
  bool ret = this->read(control_item->address, control_item->length, &read_id, &log);

  if (ret == false) {
    RCLCPP_ERROR(logger_, "Failed to read device id: %s", log);
    return false;
  }

  RCLCPP_INFO(logger_, "Connected device id: %d", read_id);
  return device_id_ == read_id;
}

uint16_t Communicator::get_model_number()
{
  return get_data<uint16_t>("Model_Number");
}

uint8_t Communicator::get_firmware_major_version()
{
  return get_data<uint8_t>("Firmware_Version_Major");
}

uint8_t Communicator::get_firmware_minor_version()
{
  return get_data<uint8_t>("Firmware_Version_Minor");
}

bool Communicator::find_control_item(const std::string & item_name)
{
  return control_table_.find(item_name) != control_table_.end();
}

const ControlTableParser::ControlItem * Communicator::get_control_item(
  const std::string & item_name)
{
  if (control_table_.find(item_name) == control_table_.end()) {return nullptr;}
  return &control_table_[item_name];
}

bool Communicator::write(
  const std::string & item_name,
  const uint32_t & data,
  std::string * msg)
{
  bool ret = false;
  if (!find_control_item(item_name)) {
    if (msg) {*msg = "Failed to find item name: " + item_name;}
    return ret;
  }

  const ControlTableParser::ControlItem * control_item = &control_table_[item_name];
  constexpr int64_t READ_ONLY = 1u;
  constexpr int64_t READ_WRITE = 3u;

  uint32_t casted_data = static_cast<uint32_t>(data);
  if (control_item->rw == READ_WRITE) {
    ret = write(
      control_item->address, control_item->length,
      reinterpret_cast<uint8_t *>(&casted_data), msg);
  } else if (control_item->rw == READ_ONLY) {
    if (msg) {*msg = "This item is read only";}
  } else {
    if (msg) {*msg = "This item's rw value is invalid";}
  }

  return ret;
}

bool Communicator::write(
  const uint16_t & address,
  const uint16_t & length,
  uint8_t * data,
  std::string * msg)
{
  const char * log = nullptr;
  bool ret = write(address, length, data, &log);

  if (msg) {
    if (ret == true) {
      *msg = "Succeeded to write data";
    } else {
      *msg = "Failed to write data" + std::string(log ? log : "");
    }
  }

  return ret;
}

bool Communicator::write(
  const uint16_t & address,
  const uint16_t & length,
  uint8_t * data,
  const char ** log,
  int * comm_result)
{
  std::lock_guard<std::mutex> lock(sdk_handler_mutex_);

  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  dxl_comm_result = packet_handler_->writeTxRx(
    port_handler_,
    device_id_,
    address,
    length,
    data,
    &dxl_error);

  if (comm_result != nullptr) {
    *comm_result = dxl_comm_result;
  }
  if (dxl_comm_result != COMM_SUCCESS) {
    if (log != nullptr) {
      *log = packet_handler_->getTxRxResult(dxl_comm_result);
    }
    return false;
  } else if (dxl_error != 0) {
    if (log != nullptr) {
      *log = packet_handler_->getRxPacketError(dxl_error);
    }
    return false;
  }

  return true;
}

bool Communicator::read_control_table(int * comm_result)
{
  return read_control_table(min_address_, max_address_, comm_result);
}

bool Communicator::read_control_table(
  const uint16_t & min_address,
  const uint16_t & max_address,
  int * comm_result)
{
  const char * log = nullptr;
  int result = 0;
  uint16_t length = max_address - min_address;
  if (length > READ_BUFFER_SIZE) {
    RCLCPP_ERROR(
      logger_, "Read length(%d) exceeds buffer size(%d)", length, READ_BUFFER_SIZE);
    return false;
  }
  bool ret = this->read(
    min_address,
    length,
    &data_buffer_[0],
    &log,
    &result);

  if (comm_result != nullptr) {
    *comm_result = result;
  }
  if (ret == true) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (data_.size() < length) {
      RCLCPP_ERROR(logger_, "Check data size for ID: %d", device_id_);
      return false;
    }
    std::copy(data_buffer_, data_buffer_ + length, data_.begin());
  }

  return ret;
}

bool Communicator::read_control_table(
  const uint16_t & address,
  const uint16_t & length,
  uint8_t * buffer,
  int * comm_result)
{
  const char * log = nullptr;
  int result = 0;
  bool ret = this->read(
    address,
    length,
    buffer,
    &log,
    &result);

  if (comm_result != nullptr) {
    *comm_result = result;
  }

  return ret;
}

bool Communicator::read(
  const uint16_t & address,
  const uint16_t & length,
  uint8_t * data_basket,
  const char ** log,
  int * comm_result)
{
  std::lock_guard<std::mutex> lock(sdk_handler_mutex_);

  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  dxl_comm_result = packet_handler_->readTxRx(
    port_handler_,
    device_id_,
    address,
    length,
    data_basket,
    &dxl_error);

  if (comm_result != nullptr) {
    *comm_result = dxl_comm_result;
  }
  if (dxl_comm_result != COMM_SUCCESS) {
    if (log != nullptr) {
      *log = packet_handler_->getTxRxResult(dxl_comm_result);
    }
    return false;
  } else if (dxl_error != 0) {
    if (log != nullptr) {
      *log = packet_handler_->getRxPacketError(dxl_error);
    }
    return false;
  }

  return true;
}
std::shared_ptr<Communicator> create_communicator(
  const std::string & port,
  uint32_t baud_rate,
  float protocol_version,
  uint8_t device_id,
  const std::string & control_table_path)
{
  auto logger = rclcpp::get_logger("create_communicator");

  auto * port_handler = dynamixel::PortHandler::getPortHandler(port.c_str());
  auto * packet_handler = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  if (!port_handler->openPort()) {
    RCLCPP_ERROR(logger, "Failed to open port: %s", port.c_str());
    delete port_handler;
    return nullptr;
  }

  if (!port_handler->setBaudRate(baud_rate)) {
    RCLCPP_ERROR(logger, "Failed to set baud rate: %d", baud_rate);
    port_handler->closePort();
    delete port_handler;
    return nullptr;
  }

  // Communicator takes ownership of port_handler (cleaned up in its destructor).
  // packet_handler is a SDK singleton and must not be deleted.
  auto communicator = std::make_shared<Communicator>(port_handler, packet_handler);

  if (!communicator->init(device_id, control_table_path)) {
    RCLCPP_ERROR(logger, "Failed to init communicator for ID: %d", device_id);
    return nullptr;
  }

  if (!communicator->is_connected_to_device()) {
    RCLCPP_ERROR(logger, "Device not responding at ID: %d", device_id);
    return nullptr;
  }

  RCLCPP_INFO(logger, "Connected to device (ID: %d) on port: %s", device_id, port.c_str());
  return communicator;
}

}  // namespace libs
}  // namespace antbot
