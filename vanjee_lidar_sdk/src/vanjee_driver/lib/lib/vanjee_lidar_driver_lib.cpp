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

#include "vanjee_lidar_driver_lib.hpp"

using namespace vanjee::lidar;

bool VanjeeLidarDriverLib::init(const WJDriverParam &param, const std::function<void(std::shared_ptr<PointCloudClient>)> &cb_put_cloud,
                                const std::function<void(std::shared_ptr<ImuPacketClient>)> &cb_put_imu_pkt,
                                const std::function<void(std::shared_ptr<ScanDataClient>)> &cb_put_scan_data,
                                const std::function<void(std::shared_ptr<DeviceCtrlClient>)> &cb_put_device_ctrl_state) {
  point_cloud_callback_ = cb_put_cloud;
  imu_pack_callback_ = cb_put_imu_pkt;
  laser_scan_callback_ = cb_put_scan_data;
  device_ctrl_callback_ = cb_put_device_ctrl_state;

  WJ_INFO << "------------------------------------------------------" << WJ_REND;
  WJ_INFO << "                      " << name_ << WJ_REND;
  WJ_INFO << "------------------------------------------------------" << WJ_REND;
  param.print();

  // @brief these two callback,allocatePointCloudMemoryCallback and
  // pointCloudCallback, have to be implemented,and as the parameter of
  // regPointCloudCallback function.
  driver_.regPointCloudCallback(std::bind(&VanjeeLidarDriverLib::allocatePointCloudMemoryCallback, this),
                                std::bind(&VanjeeLidarDriverLib::pointCloudCallback, this, std::placeholders::_1));
  // @brief these two callback,allocateImuPacketMemoryCallback and
  // imuPacketCallback, have to be implemented,and as the parameter of
  // regImuPacketCallback function.
  driver_.regImuPacketCallback(std::bind(&VanjeeLidarDriverLib::allocateImuPacketMemoryCallback, this),
                               std::bind(&VanjeeLidarDriverLib::imuPacketCallback, this, std::placeholders::_1));
  // @brief these two callback,allocateLaserScanMemoryCallback and
  // laserScanCallback, have to be implemented,and as the parameter of
  // regScanDataCallback function.
  driver_.regScanDataCallback(std::bind(&VanjeeLidarDriverLib::allocateLaserScanMemoryCallback, this),
                              std::bind(&VanjeeLidarDriverLib::laserScanCallback, this, std::placeholders::_1));
  // @brief exceptionCallback have to be implemented,and as the parameter of
  // regExceptionCallback function.
  driver_.regExceptionCallback(std::bind(&VanjeeLidarDriverLib::exceptionCallback, this, std::placeholders::_1));
  // @brief these two callback,allocateDeviceCtrlMemoryCallback and
  // deviceCtrlCallback, have to be implemented,and as the parameter of
  // regDeviceCtrlCallback function.
  driver_.regDeviceCtrlCallback(std::bind(&VanjeeLidarDriverLib::allocateDeviceCtrlMemoryCallback, this),
                                std::bind(&VanjeeLidarDriverLib::deviceCtrlCallback, this, std::placeholders::_1));
  if (!driver_.init(param)) {
    WJ_ERROR << name_ << ": Failed to initialize driver." << WJ_REND;
    return false;
  }

  return true;
}

bool VanjeeLidarDriverLib::start() {
  to_exit_driver_ = false;
  point_cloud_thread_ = std::thread(&VanjeeLidarDriverLib::processPointCloud, this);
  imu_thread_ = std::thread(&VanjeeLidarDriverLib::processImu, this);
  laser_scan_thread_ = std::thread(&VanjeeLidarDriverLib::processLaserScan, this);
  device_ctrl_thread_ = std::thread(&VanjeeLidarDriverLib::processDeviceCtrl, this);
  device_ctrl_cmd_thread_ = std::thread(&VanjeeLidarDriverLib::processDeviceCtrlCmd, this);
  driver_.start();
  WJ_DEBUG << name_ << ": Started driver." << WJ_REND;

  return true;
}

void VanjeeLidarDriverLib::stop() {
  driver_.stop();
  to_exit_driver_ = true;
  point_cloud_thread_.join();
  imu_thread_.join();
  laser_scan_thread_.join();
  device_ctrl_thread_.join();
  device_ctrl_cmd_thread_.join();
}

void VanjeeLidarDriverLib::deviceCtrlApi(DeviceCtrl device_ctrl) {
  driver_.deviceCtrlApi(device_ctrl);
}

std::shared_ptr<PointCloudMsg> VanjeeLidarDriverLib::allocatePointCloudMemoryCallback() {
  std::shared_ptr<PointCloudClient> msg = free_cloud_queue_.pop();
  if (msg.get() != NULL) {
    return std::make_shared<PointCloudMsg>(msg->point_cloud_msg_);
  }

  return std::make_shared<PointCloudMsg>();
}

void VanjeeLidarDriverLib::pointCloudCallback(std::shared_ptr<PointCloudMsg> msg) {
  std::shared_ptr<PointCloudClient> msg_client = std::make_shared<PointCloudClient>();
  msg_client->lidar_id_ = lidar_id_;
  msg_client->point_cloud_msg_ = *msg;
  stuffed_cloud_queue_.push(msg_client);
}

std::shared_ptr<ImuPacket> VanjeeLidarDriverLib::allocateImuPacketMemoryCallback() {
  std::shared_ptr<ImuPacketClient> msg = free_imu_queue_.pop();
  if (msg.get() != NULL) {
    std::shared_ptr<ImuPacket> msg_ret = std::make_shared<ImuPacket>();
    msg_ret->timestamp = msg->timestamp;
    msg_ret->seq = msg->seq;
    msg_ret->orientation = msg->orientation;
    msg_ret->orientation_covariance = msg->orientation_covariance;
    msg_ret->angular_voc = msg->angular_voc;
    msg_ret->angular_voc_covariance = msg->angular_voc_covariance;
    msg_ret->linear_acce = msg->linear_acce;
    msg_ret->linear_acce_covariance = msg->linear_acce_covariance;
    return msg_ret;
  }

  return std::make_shared<ImuPacket>();
}

void VanjeeLidarDriverLib::imuPacketCallback(std::shared_ptr<ImuPacket> msg) {
  std::shared_ptr<ImuPacketClient> msg_client = std::make_shared<ImuPacketClient>();
  msg_client->timestamp = msg->timestamp;
  msg_client->seq = msg->seq;
  msg_client->orientation = msg->orientation;
  msg_client->orientation_covariance = msg->orientation_covariance;
  msg_client->angular_voc = msg->angular_voc;
  msg_client->angular_voc_covariance = msg->angular_voc_covariance;
  msg_client->linear_acce = msg->linear_acce;
  msg_client->linear_acce_covariance = msg->linear_acce_covariance;

  msg_client->lidar_id_ = lidar_id_;
  stuffed_imu_queue_.push(msg_client);
}

std::shared_ptr<ScanData> VanjeeLidarDriverLib::allocateLaserScanMemoryCallback() {
  std::shared_ptr<ScanDataClient> msg = free_scan_data_queue_.pop();
  if (msg.get() != NULL) {
    std::shared_ptr<ScanData> msg_ret = std::make_shared<ScanData>();
    msg_ret->seq = msg->seq;
    msg_ret->timestamp = msg->timestamp;
    msg_ret->angle_min = msg->angle_min;
    msg_ret->angle_max = msg->angle_max;
    msg_ret->angle_increment = msg->angle_increment;
    msg_ret->time_increment = msg->time_increment;
    msg_ret->scan_time = msg->scan_time;
    msg_ret->range_min = msg->range_min;
    msg_ret->range_max = msg->range_max;
    msg_ret->ranges = msg->ranges;
    msg_ret->intensities = msg->intensities;
    return msg_ret;
  }

  return std::make_shared<ScanData>();
}

void VanjeeLidarDriverLib::laserScanCallback(std::shared_ptr<ScanData> msg) {
  std::shared_ptr<ScanDataClient> msg_client = std::make_shared<ScanDataClient>();
  msg_client->seq = msg->seq;
  msg_client->timestamp = msg->timestamp;
  msg_client->angle_min = msg->angle_min;
  msg_client->angle_max = msg->angle_max;
  msg_client->angle_increment = msg->angle_increment;
  msg_client->time_increment = msg->time_increment;
  msg_client->scan_time = msg->scan_time;
  msg_client->range_min = msg->range_min;
  msg_client->range_max = msg->range_max;
  msg_client->ranges = msg->ranges;
  msg_client->intensities = msg->intensities;

  msg_client->lidar_id_ = lidar_id_;
  stuffed_scan_data_queue_.push(msg_client);
}

std::shared_ptr<DeviceCtrl> VanjeeLidarDriverLib::allocateDeviceCtrlMemoryCallback() {
  std::shared_ptr<DeviceCtrlClient> msg = free_device_ctrl_queue_.pop();
  if (msg.get() != NULL) {
    std::shared_ptr<DeviceCtrl> msg_ret = std::make_shared<DeviceCtrl>();
    msg_ret->seq = msg->seq;
    msg_ret->timestamp = msg->timestamp;
    msg_ret->cmd_id = msg->cmd_id;
    msg_ret->cmd_param = msg->cmd_param;
    msg_ret->cmd_state = msg->cmd_state;
    return msg_ret;
  }

  return std::make_shared<DeviceCtrl>();
}

void VanjeeLidarDriverLib::deviceCtrlCallback(std::shared_ptr<DeviceCtrl> msg) {
  std::shared_ptr<DeviceCtrlClient> msg_client = std::make_shared<DeviceCtrlClient>();
  msg_client->seq = msg->seq;
  msg_client->timestamp = msg->timestamp;
  msg_client->cmd_id = msg->cmd_id;
  msg_client->cmd_param = msg->cmd_param;
  msg_client->cmd_state = msg->cmd_state;

  msg_client->lidar_id_ = lidar_id_;
  stuffed_device_ctrl_queue_.push(msg_client);
}

void VanjeeLidarDriverLib::exceptionCallback(const Error &code) {
  WJ_WARNING << name_ << ": " << code.toString() << WJ_REND;
}

void VanjeeLidarDriverLib::processPointCloud() {
  while (!to_exit_driver_) {
    std::shared_ptr<PointCloudClient> msg = stuffed_cloud_queue_.popWait();
    if (msg.get() == NULL) {
      continue;
    }
    point_cloud_callback_(msg);
    free_cloud_queue_.push(msg);
  }
}

void VanjeeLidarDriverLib::processImu() {
  while (!to_exit_driver_) {
    std::shared_ptr<ImuPacketClient> msg = stuffed_imu_queue_.popWait();
    if (msg.get() == NULL) {
      continue;
    }
    imu_pack_callback_(msg);
    free_imu_queue_.push(msg);
  }
}

void VanjeeLidarDriverLib::processLaserScan() {
  while (!to_exit_driver_) {
    std::shared_ptr<ScanDataClient> msg = stuffed_scan_data_queue_.popWait();
    if (msg.get() == NULL) {
      continue;
    }
    laser_scan_callback_(msg);
    free_scan_data_queue_.push(msg);
  }
}

void VanjeeLidarDriverLib::processDeviceCtrl() {
  while (!to_exit_driver_) {
    std::shared_ptr<DeviceCtrlClient> msg = stuffed_device_ctrl_queue_.popWait();
    if (msg.get() == NULL) {
      continue;
    }
    device_ctrl_callback_(msg);
    free_device_ctrl_queue_.push(msg);
  }
}

void VanjeeLidarDriverLib::processDeviceCtrlCmd() {
  while (!to_exit_driver_) {
    std::shared_ptr<DeviceCtrlClient> msg = device_ctrl_param_.popWait();
    if (msg.get() == NULL) {
      continue;
    }
    if (msg->lidar_id_ == lidar_id_) {
      std::shared_ptr<DeviceCtrl> msg_ret = std::make_shared<DeviceCtrl>();
      msg_ret->seq = msg->seq;
      msg_ret->timestamp = msg->timestamp;
      msg_ret->cmd_id = msg->cmd_id;
      msg_ret->cmd_param = msg->cmd_param;
      msg_ret->cmd_state = msg->cmd_state;
      deviceCtrlApi(*msg_ret);
      // std::cout << "lidar [" << (uint16_t)msg->lidar_id_ << "] seq: " << msg->seq << ", cmd_id: " << msg->cmd_id << ", cmd_param: "
      //           << msg->cmd_param << ", cmd_state: " << (uint16_t)msg->cmd_state << std::endl;
    } else {
      device_ctrl_param_.push(msg);
    }
  }
}

bool vanjeeLidarDriverLibStop() {
  if (driver_status_) {
    to_exit_driver_ = true;
    while (driver_status_) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return true;
  } else {
    WJ_WARNING << "Please start the driver first!" << WJ_REND;
    return false;
  }
}

int vanjeeLidarDriverLibStart(std::string config_path, const std::function<void(std::shared_ptr<PointCloudClient>)> &cb_put_cloud,
                              const std::function<void(std::shared_ptr<ImuPacketClient>)> &cb_put_imu_pkt,
                              const std::function<void(std::shared_ptr<ScanDataClient>)> &cb_put_scan_data,
                              const std::function<void(std::shared_ptr<DeviceCtrlClient>)> &cb_put_device_ctrl_state) {
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path);
  } catch (...) {
    WJ_ERROR << "The format of config file " << config_path << " is wrong. Please check (e.g. indentation)." << WJ_REND;
    return -1;
  }

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
  std::vector<VanjeeLidarDriverLib> client(lidar_config.size());
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

    YAML::Node driver_config = yamlSubNodeAbort(lidar_config[i], "driver");
    vanjee::lidar::WJDriverParam driver_param;
    yamlRead<uint16_t>(driver_config, "connect_type", driver_param.input_param.connect_type, 1);
    yamlRead<uint16_t>(driver_config, "host_msop_port", driver_param.input_param.host_msop_port, 3001);
    yamlRead<uint16_t>(driver_config, "lidar_msop_port", driver_param.input_param.lidar_msop_port, 3333);
    yamlRead<std::string>(driver_config, "host_address", driver_param.input_param.host_address, "0.0.0.0");
    yamlRead<std::string>(driver_config, "group_address", driver_param.input_param.group_address, "0.0.0.0");
    yamlRead<std::string>(driver_config, "lidar_address", driver_param.input_param.lidar_address, "0.0.0.0");
    yamlRead<bool>(driver_config, "use_vlan", driver_param.input_param.use_vlan, false);
    yamlRead<std::string>(driver_config, "pcap_path", driver_param.input_param.pcap_path, "");
    yamlRead<bool>(driver_config, "pcap_repeat", driver_param.input_param.pcap_repeat, true);
    yamlRead<float>(driver_config, "pcap_rate", driver_param.input_param.pcap_rate, 1);
    yamlRead<uint16_t>(driver_config, "use_layer_bytes", driver_param.input_param.user_layer_bytes, 0);
    yamlRead<uint16_t>(driver_config, "tail_layer_bytes", driver_param.input_param.tail_layer_bytes, 0);
    std::string lidar_type;
    yamlReadAbort<std::string>(driver_config, "lidar_type", lidar_type);
    driver_param.lidar_type = strToLidarType(lidar_type);
    yamlRead<bool>(driver_config, "wait_for_difop", driver_param.decoder_param.wait_for_difop, false);
    yamlRead<bool>(driver_config, "use_lidar_clock", driver_param.decoder_param.use_lidar_clock, false);
    yamlRead<float>(driver_config, "min_distance", driver_param.decoder_param.min_distance, 0.2);
    yamlRead<float>(driver_config, "max_distance", driver_param.decoder_param.max_distance, 200);
    yamlRead<float>(driver_config, "start_angle", driver_param.decoder_param.start_angle, 0);
    yamlRead<float>(driver_config, "end_angle", driver_param.decoder_param.end_angle, 360);
    yamlRead<bool>(driver_config, "dense_points", driver_param.decoder_param.dense_points, false);
    yamlRead<bool>(driver_config, "ts_first_point", driver_param.decoder_param.ts_first_point, false);
    yamlRead<bool>(driver_config, "config_from_file", driver_param.decoder_param.config_from_file, true);
    yamlRead<uint16_t>(driver_config, "rpm", driver_param.decoder_param.rpm, 1200);
    yamlRead<std::string>(driver_config, "angle_path_ver", driver_param.decoder_param.angle_path_ver, "");
    yamlRead<std::string>(driver_config, "angle_path_hor", driver_param.decoder_param.angle_path_hor, "");
    yamlRead<bool>(driver_config, "point_cloud_enable", driver_param.decoder_param.point_cloud_enable, false);
    yamlRead<int16_t>(driver_config, "imu_enable", driver_param.decoder_param.imu_enable, 1);
    yamlRead<bool>(driver_config, "laser_scan_enable", driver_param.decoder_param.laser_scan_enable, false);
    yamlRead<bool>(driver_config, "device_ctrl_state_enable", driver_param.decoder_param.device_ctrl_state_enable, false);
    yamlRead<bool>(driver_config, "device_ctrl_cmd_enable", driver_param.decoder_param.device_ctrl_cmd_enable, false);
    yamlRead<int16_t>(driver_config, "imu_enable", driver_param.decoder_param.imu_enable, 1);
    yamlRead<std::string>(driver_config, "hide_points_range", driver_param.decoder_param.hide_points_range, "");
    yamlRead<uint16_t>(driver_config, "publish_mode", driver_param.decoder_param.publish_mode, 0);

    yamlRead<float>(driver_config, "x", driver_param.decoder_param.transform_param.x, 0);
    yamlRead<float>(driver_config, "y", driver_param.decoder_param.transform_param.y, 0);
    yamlRead<float>(driver_config, "z", driver_param.decoder_param.transform_param.z, 0);
    yamlRead<float>(driver_config, "roll", driver_param.decoder_param.transform_param.roll, 0);
    yamlRead<float>(driver_config, "pitch", driver_param.decoder_param.transform_param.pitch, 0);
    yamlRead<float>(driver_config, "yaw", driver_param.decoder_param.transform_param.yaw, 0);
    switch (msg_source) {
      case 1:
        driver_param.input_type = InputType::ONLINE_LIDAR;
        break;
      case 2:
        driver_param.input_type = InputType::PCAP_FILE;
        break;
      default:
        break;
    }
    client[i].name_ = "lidar_" + std::to_string(i + 1);
    client[i].lidar_id_ = i + 1;
    if (!client[i].init(driver_param, cb_put_cloud, cb_put_imu_pkt, cb_put_scan_data, cb_put_device_ctrl_state)) {
      WJ_ERROR << "Driver Initialize Error...." << WJ_REND;
      return -1;
    }
  }
  for (size_t i = 0; i < client.size(); i++) {
    client[i].start();
  }
  driver_status_ = true;
#ifdef ORDERLY_EXIT
  std::this_thread::sleep_for(std::chrono::seconds(10));

  for (size_t i = 0; i < client.size(); i++) {
    client[i].stop();
  }
#else
  while (true) {
    if (to_exit_driver_) {
      for (size_t i = 0; i < client.size(); i++) {
        client[i].stop();
      }
      break;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  driver_status_ = false;
#endif

  return 0;
}

void vanjeeLidarDriverDeviceCtrlApi(DeviceCtrlClient &device_ctrl) {
  std::shared_ptr<DeviceCtrlClient> device_ctrl_ptr = std::make_shared<DeviceCtrlClient>(device_ctrl);
  device_ctrl_param_.push(device_ctrl_ptr);
}
