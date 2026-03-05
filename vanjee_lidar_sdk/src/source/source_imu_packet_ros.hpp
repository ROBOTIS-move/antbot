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
#include <source/source.hpp>

#ifdef ROS_FOUND
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
namespace vanjee {
namespace lidar {
inline sensor_msgs::Imu toRosMsg(const ImuPacket &msg, const std::string &frame_id) {
  sensor_msgs::Imu ros_msg;

  ros_msg.orientation.x = msg.orientation[1];
  ros_msg.orientation.y = msg.orientation[2];
  ros_msg.orientation.z = msg.orientation[3];
  ros_msg.orientation.w = msg.orientation[0];

  ros_msg.angular_velocity.x = msg.angular_voc[0];
  ros_msg.angular_velocity.y = msg.angular_voc[1];
  ros_msg.angular_velocity.z = msg.angular_voc[2];

  ros_msg.linear_acceleration.x = msg.linear_acce[0];
  ros_msg.linear_acceleration.y = msg.linear_acce[1];
  ros_msg.linear_acceleration.z = msg.linear_acce[2];

  for (int i = 0; i < 9; i++) {
    ros_msg.orientation_covariance[i] = msg.orientation_covariance[i];
    ros_msg.angular_velocity_covariance[i] = msg.angular_voc_covariance[i];
    ros_msg.linear_acceleration_covariance[i] = msg.linear_acce_covariance[i];
  }

  ros_msg.header.seq = msg.seq;
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(msg.timestamp);
  ros_msg.header.frame_id = frame_id;

  return ros_msg;
}

class DestinationImuPacketRos : public DestinationImuPacket {
 private:
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Publisher imu_pkt_pub_;
  std::string frame_id_;

 public:
  virtual void init(const YAML::Node &config);
  virtual void sendImuPacket(const ImuPacket &msg);
  virtual ~DestinationImuPacketRos() = default;
};
inline void DestinationImuPacketRos::init(const YAML::Node &config) {
  yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar_imu");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_imu_packet_topic", ros_send_topic, "vanjee_lidar_imu_packets");

  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  imu_pkt_pub_ = nh_->advertise<sensor_msgs::Imu>(ros_send_topic, 10);
}

inline void DestinationImuPacketRos::sendImuPacket(const ImuPacket &msg) {
  imu_pkt_pub_.publish(toRosMsg(msg, frame_id_));
}

}  // namespace lidar
}  // namespace vanjee

#endif

#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "source/source_ros_msg_delegate.hpp"
namespace vanjee {
namespace lidar {

inline sensor_msgs::msg::Imu toRosMsg(const ImuPacket &msg, const std::string &frame_id) {
  sensor_msgs::msg::Imu ros_msg;

  ros_msg.orientation.x = msg.orientation[1];
  ros_msg.orientation.y = msg.orientation[2];
  ros_msg.orientation.z = msg.orientation[3];
  ros_msg.orientation.w = msg.orientation[0];

  ros_msg.angular_velocity.x = msg.angular_voc[0];
  ros_msg.angular_velocity.y = msg.angular_voc[1];
  ros_msg.angular_velocity.z = msg.angular_voc[2];

  ros_msg.linear_acceleration.x = msg.linear_acce[0];
  ros_msg.linear_acceleration.y = msg.linear_acce[1];
  ros_msg.linear_acceleration.z = msg.linear_acce[2];

  for (int i = 0; i < 9; i++) {
    ros_msg.orientation_covariance[i] = msg.orientation_covariance[i];
    ros_msg.angular_velocity_covariance[i] = msg.angular_voc_covariance[i];
    ros_msg.linear_acceleration_covariance[i] = msg.linear_acce_covariance[i];
  }

  ros_msg.header.stamp.sec = (uint32_t)floor(msg.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)round((msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  ros_msg.header.frame_id = frame_id;

  return ros_msg;
}

class DestinationImuPacketRos : public DestinationImuPacket {
 private:
  using ImuMsgPubPtr = std::shared_ptr<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::Imu>>;
  ImuMsgPubPtr imu_msg_pub_ptr_;
  std::string frame_id_;

 public:
  virtual void init(const YAML::Node &config);
  virtual void sendImuPacket(const ImuPacket &msg);
  virtual ~DestinationImuPacketRos() = default;
};

inline void DestinationImuPacketRos::init(const YAML::Node &config) {
  yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_imu_packet_topic", ros_send_topic, "vanjee_lidar_imu_packets");
  imu_msg_pub_ptr_ = VanjeeLidarSdkNode::CreateInstance()->GetImuMsgPublisher(ros_send_topic);
}

inline void DestinationImuPacketRos::sendImuPacket(const ImuPacket &msg) {
  sensor_msgs::msg::Imu imu = toRosMsg(msg, frame_id_);
  imu_msg_pub_ptr_->PublishMsg(imu);
}
}  // namespace lidar
}  // namespace vanjee
#endif