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

#include "manager/node_manager.hpp"

#include "source/source_device_ctrl_ros.hpp"
#include "source/source_driver.hpp"
#include "source/source_imu_packet_ros.hpp"
#include "source/source_pointcloud_ros.hpp"
#include "source/source_recv_device_ctrl_ros.hpp"
#include "source/source_scandata_ros.hpp"

namespace vanjee {
namespace lidar {

void NodeManager::init(const YAML::Node &config) {
  YAML::Node common_config = yamlSubNodeAbort(config, "common");
  int msg_source = 0;
  yamlRead<int>(common_config, "msg_source", msg_source, 0);
  bool send_point_cloud_ros;
  yamlRead<bool>(common_config, "send_point_cloud_ros", send_point_cloud_ros, false);
  bool send_imu_packet_ros;
  yamlRead<bool>(common_config, "send_imu_packet_ros", send_imu_packet_ros, false);
  bool send_laser_scan_ros;
  yamlRead<bool>(common_config, "send_laser_scan_ros", send_laser_scan_ros, false);
  bool send_device_ctrl_state_ros;
  yamlRead<bool>(common_config, "send_device_ctrl_state_ros", send_device_ctrl_state_ros, false);
  bool recv_device_ctrl_cmd_ros;
  yamlRead<bool>(common_config, "recv_device_ctrl_cmd_ros", recv_device_ctrl_cmd_ros, false);

  YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");
  for (uint8_t i = 0; i < lidar_config.size(); i++) {
    lidar_config[i]["driver"]["point_cloud_enable"] = send_point_cloud_ros;
    if (!send_imu_packet_ros) {
      lidar_config[i]["driver"]["imu_enable"] = -1;
    } else if (msg_source == 1) {
      lidar_config[i]["driver"]["imu_enable"] = 1;
    } else {
      lidar_config[i]["driver"]["imu_enable"] = 0;
    }
    lidar_config[i]["driver"]["laser_scan_enable"] = send_laser_scan_ros;
    lidar_config[i]["driver"]["device_ctrl_state_enable"] = send_device_ctrl_state_ros;
    lidar_config[i]["driver"]["device_ctrl_cmd_enable"] = recv_device_ctrl_cmd_ros;

    std::shared_ptr<Source> source;
    switch (msg_source) {
      case SourceType::MSG_FROM_LIDAR:

        WJ_INFO << "------------------------------------------------------" << WJ_REND;
        WJ_INFO << "Receive Packets From : Online LiDAR" << WJ_REND;
        WJ_INFO << "Msop Port: " << lidar_config[i]["driver"]["host_msop_port"].as<uint16_t>() << WJ_REND;
        WJ_INFO << "------------------------------------------------------" << WJ_REND;

        source = std::make_shared<SourceDriver>(SourceType::MSG_FROM_LIDAR);
        source->send_point_cloud_ros_ = send_point_cloud_ros;
        source->send_imu_packet_ros_ = send_imu_packet_ros;
        source->send_laser_scan_ros_ = send_laser_scan_ros;
        source->send_device_ctrl_state_ros_ = send_device_ctrl_state_ros;
        source->recv_device_ctrl_cmd_ros_ = recv_device_ctrl_cmd_ros;
        source->init(lidar_config[i]);
        break;

      case SourceType::MSG_FROM_PCAP:

        WJ_INFO << "------------------------------------------------------" << WJ_REND;
        WJ_INFO << "Receive Packets From : Pcap" << WJ_REND;
        WJ_INFO << "Msop Port: " << lidar_config[i]["driver"]["host_msop_port"].as<uint16_t>() << WJ_REND;
        WJ_INFO << "------------------------------------------------------" << WJ_REND;

        source = std::make_shared<SourceDriver>(SourceType::MSG_FROM_PCAP);
        source->send_point_cloud_ros_ = send_point_cloud_ros;
        source->send_imu_packet_ros_ = send_imu_packet_ros;
        source->send_laser_scan_ros_ = send_laser_scan_ros;
        source->send_device_ctrl_state_ros_ = send_device_ctrl_state_ros;
        source->recv_device_ctrl_cmd_ros_ = recv_device_ctrl_cmd_ros;
        source->init(lidar_config[i]);
        break;

      default:
        WJ_ERROR << "Unsupported LiDAR message source:" << msg_source << "." << WJ_REND;
        exit(-1);
    }

    if (send_point_cloud_ros) {
      WJ_DEBUG << "------------------------------------------------------" << WJ_REND;
      WJ_DEBUG << "Send PointCloud To : ROS" << WJ_REND;
      WJ_DEBUG << "PointCloud Topic: " << lidar_config[i]["ros"]["ros_send_point_cloud_topic"].as<std::string>() << WJ_REND;
      WJ_DEBUG << "------------------------------------------------------" << WJ_REND;

      std::shared_ptr<DestinationPointCloud> dst = std::make_shared<DestinationPointCloudRos>();
      dst->init(lidar_config[i]);
      source->regPointCloudCallback(dst);
    }

    if (send_imu_packet_ros) {
      try {
        string ros_topic_str = lidar_config[i]["ros"]["ros_send_imu_packet_topic"].as<std::string>();

        WJ_DEBUG << "------------------------------------------------------" << WJ_REND;
        WJ_DEBUG << "Send ImuPackets To : ROS" << WJ_REND;
        WJ_DEBUG << "ImuPacket Topic: " << ros_topic_str << WJ_REND;
        WJ_DEBUG << "------------------------------------------------------" << WJ_REND;

        std::shared_ptr<DestinationImuPacket> dst = std::make_shared<DestinationImuPacketRos>();
        dst->init(lidar_config[i]);
        source->regImuPacketCallback(dst);
      } catch (...) {
        WJ_WARNING << "ros_send_imu_packet_topic is null" << WJ_REND;
      }
    }

    if (send_laser_scan_ros) {
      try {
        string ros_topic_str = lidar_config[i]["ros"]["ros_send_laser_scan_topic"].as<std::string>();

        WJ_DEBUG << "------------------------------------------------------" << WJ_REND;
        WJ_DEBUG << "Send LaserScan To : ROS" << WJ_REND;
        WJ_DEBUG << "LaserScan Topic: " << ros_topic_str << WJ_REND;
        WJ_DEBUG << "------------------------------------------------------" << WJ_REND;

        std::shared_ptr<DestinationScanDataRos> dst = std::make_shared<DestinationScanDataRos>();
        dst->init(lidar_config[i]);
        source->regScanDataCallback(dst);
      } catch (...) {
        WJ_WARNING << "ros_send_laser_scan_topic is null" << WJ_REND;
      }
    }

    if (send_device_ctrl_state_ros) {
      try {
        string ros_topic_str = lidar_config[i]["ros"]["ros_send_device_ctrl_state_topic"].as<std::string>();

        WJ_DEBUG << "------------------------------------------------------" << WJ_REND;
        WJ_DEBUG << "Send DeviceCtrl Command Feedback To : ROS" << WJ_REND;
        WJ_DEBUG << "DeviceCtrl State Topic: " << ros_topic_str << WJ_REND;
        WJ_DEBUG << "------------------------------------------------------" << WJ_REND;

        std::shared_ptr<DestinationDeviceCtrlRos> dst = std::make_shared<DestinationDeviceCtrlRos>();
        dst->init(lidar_config[i]);
        source->regDeviceCtrlCallback(dst);
      } catch (...) {
        WJ_WARNING << "ros_send_device_ctrl_state_topic is null" << WJ_REND;
      }
    }

    if (recv_device_ctrl_cmd_ros) {
      try {
        string ros_topic_str = lidar_config[i]["ros"]["ros_recv_device_ctrl_cmd_topic"].as<std::string>();

        WJ_DEBUG << "------------------------------------------------------" << WJ_REND;
        WJ_DEBUG << "Recv DeviceCtrl Command From : ROS" << WJ_REND;
        WJ_DEBUG << "DeviceCtrl Command Topic: " << ros_topic_str << WJ_REND;
        WJ_DEBUG << "------------------------------------------------------" << WJ_REND;

        std::shared_ptr<SourceDeviceCtrlRos> dst = std::make_shared<SourceDeviceCtrlRos>();
        dst->init(lidar_config[i]);
        source->setDeviceCtrlPtr(dst);
      } catch (...) {
        WJ_WARNING << "ros_recv_device_ctrl_cmd_topic is null" << WJ_REND;
      }
    }
    sources_.emplace_back(source);
  }
}
void NodeManager::start() {
  for (auto &iter : sources_) {
    if (iter != nullptr) {
      iter->start();
    }
  }
}

void NodeManager::stop() {
  for (auto &iter : sources_) {
    if (iter != nullptr) {
      iter->stop();
    }
  }
}

NodeManager::~NodeManager() {
  stop();
}

}  // namespace lidar

}  // namespace vanjee
