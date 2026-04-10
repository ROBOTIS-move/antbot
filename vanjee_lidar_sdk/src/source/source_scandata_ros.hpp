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
#include "source/source.hpp"
#ifdef ROS_FOUND
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
namespace vanjee {
namespace lidar {
inline sensor_msgs::LaserScan toRosMsg(const ScanData &vanjee_msg, const std::string &frame_id) {
  sensor_msgs::LaserScan ros_msg;

  ros_msg.angle_min = vanjee_msg.angle_min / 180 * 3.1415926;
  ros_msg.angle_max = vanjee_msg.angle_max / 180 * 3.1415926;
  ros_msg.angle_increment = vanjee_msg.angle_increment / 180 * 3.1415926;
  ros_msg.time_increment = vanjee_msg.time_increment;
  ros_msg.scan_time = vanjee_msg.scan_time;
  ros_msg.range_min = vanjee_msg.range_min;
  ros_msg.range_max = vanjee_msg.range_max;

  int pointNum = (vanjee_msg.angle_max - vanjee_msg.angle_min) / vanjee_msg.angle_increment;
  for (int i = 0; i < pointNum; i++) {
    ros_msg.ranges.emplace_back(vanjee_msg.ranges[i]);
    ros_msg.intensities.emplace_back(vanjee_msg.intensities[i]);
  }

  ros_msg.header.seq = vanjee_msg.seq;
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(vanjee_msg.timestamp);
  ros_msg.header.frame_id = frame_id;

  return ros_msg;
}
/// @brief 'DestinationScanDataRos' publish point cloud through ROS topic
/// '/vanjee_lidar scan'
class DestinationScanDataRos : public DestinationScanData {
 private:
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher scan_data_pub_;
  std::string frame_id_;

 public:
  /// @brief Initialize the instance of 'DestinationPacketRos'
  virtual void init(const YAML::Node &config);
  /// @brief Publish point cloud through ROS topic '/vanjee_lidar_scan'
  virtual void sendScanData(const ScanData &msg);
  virtual ~DestinationScanDataRos() = default;
};

inline void DestinationScanDataRos::init(const YAML::Node &config) {
  yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar");
  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_laser_scan_topic", ros_send_topic, "vanjee_scan");

  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  scan_data_pub_ = nh_->advertise<sensor_msgs::LaserScan>(ros_send_topic, 10);
}
inline void DestinationScanDataRos::sendScanData(const ScanData &msg) {
  scan_data_pub_.publish(toRosMsg(msg, frame_id_));
}
}  // namespace lidar

}  // namespace vanjee

#endif
#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "source/source_ros_msg_delegate.hpp"
namespace vanjee {
namespace lidar {
inline sensor_msgs::msg::LaserScan toRosMsg(const ScanData &vanjee_msg, const std::string &frame_id) {
  sensor_msgs::msg::LaserScan ros_msg;

  ros_msg.angle_min = vanjee_msg.angle_min / 180 * 3.1415926;
  ros_msg.angle_max = vanjee_msg.angle_max / 180 * 3.1415926;
  ros_msg.angle_increment = vanjee_msg.angle_increment / 180 * 3.1415926;
  ros_msg.time_increment = vanjee_msg.time_increment;
  ros_msg.scan_time = vanjee_msg.scan_time;
  ros_msg.range_min = vanjee_msg.range_min;
  ros_msg.range_max = vanjee_msg.range_max;

  int pointNum = (vanjee_msg.angle_max - vanjee_msg.angle_min) / vanjee_msg.angle_increment;
  for (int i = 0; i < pointNum; i++) {
    ros_msg.ranges.emplace_back(vanjee_msg.ranges[i]);
    ros_msg.intensities.emplace_back(vanjee_msg.intensities[i]);
  }

  ros_msg.header.stamp.sec = (long)vanjee_msg.timestamp;
  ros_msg.header.stamp.nanosec = (long)((vanjee_msg.timestamp - (long)vanjee_msg.timestamp) * 1000000000);
  ros_msg.header.frame_id = frame_id;

  return ros_msg;
}
/// @brief 'DestinationScanDataRos' publish point cloud through ROS topic
/// '/vanjee_lidar_scan'
class DestinationScanDataRos : virtual public DestinationScanData {
 private:
  using ScanDataMsgPubPtr = std::shared_ptr<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::LaserScan>>;
  ScanDataMsgPubPtr scan_data_msg_pub_ptr_;
  std::string frame_id_;

 public:
  /// @brief Initialize the instance of 'DestinationPacketRos'
  virtual void init(const YAML::Node &config);
  /// @brief Publish point cloud through ROS topic '/vanjee_lidar_scan'
  virtual void sendScanData(const ScanData &msg);
  virtual ~DestinationScanDataRos() = default;
};

inline void DestinationScanDataRos::init(const YAML::Node &config) {
  yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar");
  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_laser_scan_topic", ros_send_topic, "vanjee_scan");
  scan_data_msg_pub_ptr_ = VanjeeLidarSdkNode::CreateInstance()->GetLaserScanMsgPublisher(ros_send_topic);
}
inline void DestinationScanDataRos::sendScanData(const ScanData &msg) {
  sensor_msgs::msg::LaserScan laser_scan = toRosMsg(msg, frame_id_);
  scan_data_msg_pub_ptr_->PublishMsg(laser_scan);
}
}  // namespace lidar

}  // namespace vanjee

#endif