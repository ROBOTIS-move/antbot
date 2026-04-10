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

#ifndef ANTBOT_HW_INTERFACE__DEVICE__DEVICE_HPP_
#define ANTBOT_HW_INTERFACE__DEVICE__DEVICE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"

#include "antbot_libs/communicator.hpp"
#include "antbot_libs/constants.hpp"

namespace antbot
{
namespace hw_interface
{
namespace device
{

struct DeviceConfig
{
  std::shared_ptr<rclcpp::Node> node = nullptr;
  std::shared_ptr<antbot::libs::Communicator> communicator = nullptr;

  DeviceConfig(
    const std::shared_ptr<rclcpp::Node> & n,
    const std::shared_ptr<antbot::libs::Communicator> & comm)
  : node(n), communicator(comm) {}
};

class Device
{
public:
  explicit Device(const std::string & name, const DeviceConfig & config)
  : name_(name),
    node_(config.node),
    communicator_(config.communicator),
    logger_(rclcpp::get_logger(name))
  {
  }
  virtual ~Device() = default;

  virtual void update(std::unordered_map<std::string, double> & state_map) = 0;
  virtual void publish(const rclcpp::Time & time) {(void)time;}
  virtual void activate() {}
  virtual void deactivate() {}

  template<typename DataByteT>
  DataByteT get_data(const std::string & name)
  {
    if (communicator_) {
      return communicator_->get_data<DataByteT>(name);
    }
    return DataByteT();
  }

  bool write(
    const std::string & item_name,
    const uint32_t & value,
    std::string * msg)
  {
    if (communicator_) {
      return communicator_->write(item_name, value, msg);
    }
    return false;
  }

protected:
  std::string name_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<antbot::libs::Communicator> communicator_;
  rclcpp::Logger logger_;
  rclcpp::QoS qos_{constants::SensorDataQoS()};
};

template<typename DeviceType>
void add_device(
  const std::string & name,
  const DeviceConfig & config,
  std::vector<std::unique_ptr<Device>> & device_list)
{
  device_list.push_back(std::make_unique<DeviceType>(name, config));
}

template<typename DeviceType>
DeviceType * add_device_with_return(
  const std::string & name,
  const DeviceConfig & config,
  std::vector<std::unique_ptr<Device>> & device_list)
{
  auto device = std::make_unique<DeviceType>(name, config);
  auto * raw_ptr = device.get();
  device_list.push_back(std::move(device));
  return raw_ptr;
}

}  // namespace device
}  // namespace hw_interface
}  // namespace antbot
#endif  // ANTBOT_HW_INTERFACE__DEVICE__DEVICE_HPP_
