// Copyright 2019 Intel Corporation
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

#ifndef ANTBOT_LIBS__NODE_THREAD_HPP_
#define ANTBOT_LIBS__NODE_THREAD_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace antbot
{
namespace libs
{
/// @class antbot::libs::NodeThread
/// @brief A background thread to process node/executor callbacks
class NodeThread
{
public:
  /// @brief Construct with a node base interface to spin in a background thread
  /// @param node_base Interface to Node to spin in thread
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base);

  /// @brief Construct with an executor to spin in a background thread
  /// @param executor Interface to executor to spin in thread
  explicit NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);

  /// @brief Construct with a node pointer to spin in a background thread
  /// @param node Node pointer to spin in thread
  template<typename NodeT>
  explicit NodeThread(NodeT node)
  : NodeThread(node->get_node_base_interface())
  {}

  /// @brief Destructor
  ~NodeThread();

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::Executor::SharedPtr executor_;
};

}  // namespace libs
}  // namespace antbot

#endif  // ANTBOT_LIBS__NODE_THREAD_HPP_
