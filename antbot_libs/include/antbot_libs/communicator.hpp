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

#ifndef ANTBOT_LIBS__COMMUNICATOR_HPP_
#define ANTBOT_LIBS__COMMUNICATOR_HPP_

#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "antbot_libs/control_table_parser.hpp"

namespace antbot
{
namespace libs
{

constexpr uint16_t READ_BUFFER_SIZE = 2048;

class Communicator
{
public:
  Communicator(
    dynamixel::PortHandler * port_handler,
    dynamixel::PacketHandler * packet_handler);
  virtual ~Communicator();

  /// Initialize the communicator: load control table XML and allocate data buffer.
  bool init(uint8_t id, const std::string & control_table_xml_path);

  /// Read data from the internal buffer by control table item name.
  template<typename DataByteT>
  DataByteT get_data(const std::string & name)
  {
    DataByteT data = 0;

    if (control_table_.empty()) {
      RCLCPP_ERROR(logger_, "Control table not initialized");
      return data;
    }

    if (!find_control_item(name)) {
      RCLCPP_ERROR(logger_, "Failed to find item name[%s]", name.c_str());
      return data;
    }

    uint16_t address = control_table_[name].address;
    uint16_t length = control_table_[name].length;

    if (address < min_address_ || address + length > max_address_) {
      RCLCPP_ERROR(
        logger_,
        "Error in checking address: item_name[%s] | address[%d]", name.c_str(), address);
      return data;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    uint16_t offset = address - min_address_;
    uint8_t * p_data = reinterpret_cast<uint8_t *>(&data);

    switch (length) {
      case 1:
        p_data[0] = data_[offset + 0];
        break;
      case 2:
        p_data[0] = data_[offset + 0];
        p_data[1] = data_[offset + 1];
        break;
      case 4:
        p_data[0] = data_[offset + 0];
        p_data[1] = data_[offset + 1];
        p_data[2] = data_[offset + 2];
        p_data[3] = data_[offset + 3];
        break;
      default:
        p_data[0] = data_[offset + 0];
        break;
    }

    return data;
  }

  /// Read entire control table from the device into the internal buffer.
  bool read_control_table(int * comm_result = nullptr);

  /// Read a specific address range from the device into the internal buffer.
  bool read_control_table(
    const uint16_t & min_address,
    const uint16_t & max_address,
    int * comm_result = nullptr);

  /// Read raw bytes from the device at a specific address.
  bool read_control_table(
    const uint16_t & address,
    const uint16_t & length,
    uint8_t * buffer,
    int * comm_result = nullptr);

  /// Write a value to a control table item by name.
  bool write(
    const std::string & item_name,
    const uint32_t & data,
    std::string * msg);

  /// Write raw bytes to a specific address.
  bool write(
    const uint16_t & address,
    const uint16_t & length,
    uint8_t * data,
    std::string * msg);

  /// Write consecutive control table items starting from first_item_name.
  /// Values are serialized as contiguous bytes using memcpy.
  template<typename T>
  bool write_batch(
    const std::string & first_item_name,
    const T * values,
    uint8_t count,
    std::string * msg)
  {
    const ControlTableParser::ControlItem * item = get_control_item(first_item_name);
    if (item == nullptr) {
      if (msg) {*msg = "Failed to find item: " + first_item_name;}
      return false;
    }

    const uint16_t total_length = item->length * count;
    std::vector<uint8_t> buffer(total_length);
    for (uint8_t i = 0; i < count; i++) {
      std::memcpy(&buffer[i * sizeof(T)], &values[i], sizeof(T));
    }
    return write(item->address, total_length, buffer.data(), msg);
  }

  /// Check if the device is connected by reading its ID register.
  bool is_connected_to_device();

  /// Get the model number from the internal data buffer.
  uint16_t get_model_number();

  /// Get firmware version from the internal data buffer.
  uint8_t get_firmware_major_version();
  uint8_t get_firmware_minor_version();

  /// Check if a control table item exists.
  bool find_control_item(const std::string & item_name);

  /// Get a pointer to a control table item.
  const ControlTableParser::ControlItem * get_control_item(const std::string & item_name);

private:
  bool init_control_table(const std::string & xml_file_path);

  bool read(
    const uint16_t & address,
    const uint16_t & length,
    uint8_t * data,
    const char ** log = nullptr,
    int * comm_result = nullptr);

  bool write(
    const uint16_t & address,
    const uint16_t & length,
    uint8_t * data,
    const char ** log = nullptr,
    int * comm_result = nullptr);

  rclcpp::Logger logger_{rclcpp::get_logger("Communicator")};

  std::mutex sdk_handler_mutex_;
  std::mutex data_mutex_;

  dynamixel::PortHandler * port_handler_;
  dynamixel::PacketHandler * packet_handler_;

  uint8_t device_id_{0};

  // Data storage
  std::vector<uint8_t> data_;
  uint8_t data_buffer_[READ_BUFFER_SIZE];

  // Control table
  std::unordered_map<std::string, ControlTableParser::ControlItem> control_table_;
  uint16_t min_address_{0};
  uint16_t max_address_{255};
};

/// Open port, set baud rate, create and initialize a Communicator, verify connection.
/// Communicator owns port_handler and will clean it up on destruction.
std::shared_ptr<Communicator> create_communicator(
  const std::string & port,
  uint32_t baud_rate,
  float protocol_version,
  uint8_t device_id,
  const std::string & control_table_path);

}  // namespace libs
}  // namespace antbot
#endif  // ANTBOT_LIBS__COMMUNICATOR_HPP_
