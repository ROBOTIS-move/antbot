/*********************************************************************************************************************
Copyright (c) 2023 Vanjee
All rights reserved

By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install, copy or
use the software.

License Agreement
For Vanjee LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the names of the Vanjee, nor Suteng Innovation Technology, nor the
names of other contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <vanjee_driver/msg/device_ctrl_msg.hpp>

#include "source/source.hpp"
#ifdef ROS_FOUND
#include <ros/ros.h>
#include <vanjee_lidar_sdk/DeviceCtrl.h>
namespace vanjee {
namespace lidar {
inline std::shared_ptr<DeviceCtrl> getRosMsg(const vanjee_lidar_sdk::DeviceCtrl::ConstPtr &msg) {
  std::shared_ptr<DeviceCtrl> vanjee_msg = std::shared_ptr<DeviceCtrl>(new DeviceCtrl());
  vanjee_msg->seq = msg->header.seq;
  vanjee_msg->timestamp = msg->header.stamp.toSec();
  vanjee_msg->cmd_id = msg->cmd_id;
  vanjee_msg->cmd_param = msg->cmd_param;
  vanjee_msg->cmd_state = msg->cmd_state;

  return vanjee_msg;
}
/// @brief 'SourceDeviceCtrlRos' subscribe device control state through ROS topic
/// '/vanjee_device_ctrl_state'
class SourceDeviceCtrlRos : public SourceDeviceCtrl {
 private:
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Subscriber device_ctrl_sub_;
  std::string frame_id_;

 public:
  virtual ~SourceDeviceCtrlRos() = default;
  /// @brief Initialize the instance of 'SourceDeviceCtrlRos'
  virtual void init(const YAML::Node &config);

  /// @brief Subscribe device ctrl state through ROS topic '/vanjee_device_ctrl_state'
  void recvDeviceCtrl(const vanjee_lidar_sdk::DeviceCtrl::ConstPtr &msg);
};

inline void SourceDeviceCtrlRos::init(const YAML::Node &config) {
  yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar");
  std::string ros_recv_topic;
  yamlRead<std::string>(config["ros"], "ros_recv_device_ctrl_cmd_topic", ros_recv_topic, "vanjee_lidar_device_ctrl_cmd");

  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  device_ctrl_sub_ = nh_->subscribe<vanjee_lidar_sdk::DeviceCtrl>(ros_recv_topic, 10, &SourceDeviceCtrlRos::recvDeviceCtrl, this);
}
inline void SourceDeviceCtrlRos::recvDeviceCtrl(const vanjee_lidar_sdk::DeviceCtrl::ConstPtr &msg) {
  cached_message_.push(getRosMsg(msg));
}

}  // namespace lidar

}  // namespace vanjee

#endif
#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>

#include "source/source_ros_msg_delegate.hpp"
#include "vanjee_lidar_msg/msg/device_ctrl.hpp"
namespace vanjee {
namespace lidar {
inline std::shared_ptr<DeviceCtrl> getRosMsg(const vanjee_lidar_msg::msg::DeviceCtrl::SharedPtr msg) {
  std::shared_ptr<DeviceCtrl> vanjee_msg = std::shared_ptr<DeviceCtrl>(new DeviceCtrl());
  vanjee_msg->timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  vanjee_msg->cmd_id = msg->cmd_id;
  vanjee_msg->cmd_param = msg->cmd_param;
  vanjee_msg->cmd_state = msg->cmd_state;
  return vanjee_msg;
}

/// @brief 'SourceDeviceCtrlRos' subscribe device ctrl state through ROS topic
/// '/vanjee_device_ctrl_state'
class SourceDeviceCtrlRos : virtual public SourceDeviceCtrl {
 private:
  using DeviceCtrlMsgSubPtr = std::shared_ptr<VanjeeLidarSdkSubscribeRosMsg<vanjee_lidar_msg::msg::DeviceCtrl, vanjee::lidar::DeviceCtrl>>;
  DeviceCtrlMsgSubPtr device_ctrl_msg_sub_ptr_;
  std::string frame_id_;

 public:
  virtual ~SourceDeviceCtrlRos() = default;
  /// @brief Initialize the instance of 'SourceDeviceCtrlRos'
  virtual void init(const YAML::Node &config);

  /// @brief subscribe device ctrl state through ROS topic '/vanjee_device_ctrl_state'
  void recvDeviceCtrl(const vanjee_lidar_msg::msg::DeviceCtrl::SharedPtr msg);
};

inline void SourceDeviceCtrlRos::init(const YAML::Node &config) {
  yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar");
  std::string ros_recv_topic;
  yamlRead<std::string>(config["ros"], "ros_recv_device_ctrl_cmd_topic", ros_recv_topic, "vanjee_lidar_device_ctrl_cmd");
  device_ctrl_msg_sub_ptr_ = VanjeeLidarSdkNode::CreateInstance()->GetDeviceCtrlMsgSubscriber(ros_recv_topic);
  device_ctrl_msg_sub_ptr_->SubscribeTopic(std::bind(&SourceDeviceCtrlRos::recvDeviceCtrl, this, std::placeholders::_1));
}
inline void SourceDeviceCtrlRos::recvDeviceCtrl(const vanjee_lidar_msg::msg::DeviceCtrl::SharedPtr msg) {
  cached_message_.push(getRosMsg(msg));
}

}  // namespace lidar

}  // namespace vanjee

#endif