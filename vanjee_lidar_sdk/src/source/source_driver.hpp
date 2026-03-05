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

#include <vanjee_driver/api/lidar_driver.hpp>
#include <vanjee_driver/utility/sync_queue.hpp>

#include "source/source.hpp"

namespace vanjee {
namespace lidar {
class SourceDriver : public Source {
 public:
  virtual void init(const YAML::Node &config);
  virtual void start();
  virtual void stop();
  virtual ~SourceDriver();

  SourceDriver(SourceType src_type);

 protected:
  std::shared_ptr<LidarPointCloudMsg> getPointCloud(void);
  void putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg);
  std::shared_ptr<ImuPacket> getImuPacket(void);
  void putImuPacket(std::shared_ptr<ImuPacket> msg);
  void putException(const lidar::Error &msg);
  std::shared_ptr<ScanData> getScanData(void);
  void putScanData(std::shared_ptr<ScanData> msg);
  std::shared_ptr<DeviceCtrl> getDeviceCtrl(void);
  void putDeviceCtrl(std::shared_ptr<DeviceCtrl> msg);
  void processPointCloud();
  void processImuPacket();
  void processScanData();
  void publishDeviceCtrlCmd();
  void subscribeDeviceCtrlState();

  std::shared_ptr<lidar::LidarDriver<LidarPointCloudMsg>> driver_ptr_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> free_point_cloud_queue_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> point_cloud_queue_;

  SyncQueue<std::shared_ptr<ImuPacket>> free_imu_packet_queue_;
  SyncQueue<std::shared_ptr<ImuPacket>> imu_packet_queue_;

  SyncQueue<std::shared_ptr<ScanData>> free_scan_data_queue_;
  SyncQueue<std::shared_ptr<ScanData>> scan_data_queue_;

  SyncQueue<std::shared_ptr<DeviceCtrl>> free_device_ctrl_queue_;
  SyncQueue<std::shared_ptr<DeviceCtrl>> device_ctrl_queue_;

  std::thread point_cloud_process_thread_;
  std::thread imu_packet_process_thread_;
  std::thread scan_data_process_thread_;
  std::thread pub_device_ctrl_state_thread_;
  std::thread sub_device_ctrl_cmd_thread_;
  bool to_exit_process_;
};
SourceDriver::SourceDriver(SourceType src_type) : Source(src_type), to_exit_process_(false) {
}

inline void SourceDriver::init(const YAML::Node &config) {
  YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
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
  yamlRead<bool>(driver_config, "use_offset_timestamp", driver_param.decoder_param.use_offset_timestamp, true);
  yamlRead<bool>(driver_config, "config_from_file", driver_param.decoder_param.config_from_file, true);
  yamlRead<uint16_t>(driver_config, "rpm", driver_param.decoder_param.rpm, 1200);
  yamlRead<std::string>(driver_config, "angle_path_ver", driver_param.decoder_param.angle_path_ver, "");
  yamlRead<std::string>(driver_config, "angle_path_hor", driver_param.decoder_param.angle_path_hor, "");
  yamlRead<std::string>(driver_config, "imu_param_path", driver_param.decoder_param.imu_param_path, "");
  yamlRead<bool>(driver_config, "point_cloud_enable", driver_param.decoder_param.point_cloud_enable, false);
  yamlRead<int16_t>(driver_config, "imu_enable", driver_param.decoder_param.imu_enable, 1);
  yamlRead<bool>(driver_config, "laser_scan_enable", driver_param.decoder_param.laser_scan_enable, false);
  yamlRead<bool>(driver_config, "device_ctrl_state_enable", driver_param.decoder_param.device_ctrl_state_enable, false);
  yamlRead<bool>(driver_config, "device_ctrl_cmd_enable", driver_param.decoder_param.device_ctrl_cmd_enable, false);
  yamlRead<std::string>(driver_config, "hide_points_range", driver_param.decoder_param.hide_points_range, "");
  yamlRead<uint16_t>(driver_config, "publish_mode", driver_param.decoder_param.publish_mode, 0);

  yamlRead<float>(driver_config, "x", driver_param.decoder_param.transform_param.x, 0);
  yamlRead<float>(driver_config, "y", driver_param.decoder_param.transform_param.y, 0);
  yamlRead<float>(driver_config, "z", driver_param.decoder_param.transform_param.z, 0);
  yamlRead<float>(driver_config, "roll", driver_param.decoder_param.transform_param.roll, 0);
  yamlRead<float>(driver_config, "pitch", driver_param.decoder_param.transform_param.pitch, 0);
  yamlRead<float>(driver_config, "yaw", driver_param.decoder_param.transform_param.yaw, 0);

  switch (src_type_) {
    case SourceType::MSG_FROM_LIDAR:
      driver_param.input_type = InputType::ONLINE_LIDAR;
      break;
    case SourceType::MSG_FROM_PCAP:
      driver_param.input_type = InputType::PCAP_FILE;
      break;
    default:
      break;
  }

  driver_param.print();

  driver_ptr_.reset(new lidar::LidarDriver<LidarPointCloudMsg>());
  driver_ptr_->regPointCloudCallback(std::bind(&SourceDriver::getPointCloud, this),
                                     std::bind(&SourceDriver::putPointCloud, this, std::placeholders::_1));
  driver_ptr_->regExceptionCallback(std::bind(&SourceDriver::putException, this, std::placeholders::_1));
  if (this->send_point_cloud_ros_)
    point_cloud_process_thread_ = std::thread(std::bind(&SourceDriver::processPointCloud, this));

  driver_ptr_->regImuPacketCallback(std::bind(&SourceDriver::getImuPacket, this),
                                    std::bind(&SourceDriver::putImuPacket, this, std::placeholders::_1));
  if (this->send_imu_packet_ros_)
    imu_packet_process_thread_ = std::thread(std::bind(&SourceDriver::processImuPacket, this));

  driver_ptr_->regScanDataCallback(std::bind(&SourceDriver::getScanData, this), std::bind(&SourceDriver::putScanData, this, std::placeholders::_1));
  if (this->send_laser_scan_ros_)
    scan_data_process_thread_ = std::thread(std::bind(&SourceDriver::processScanData, this));

  driver_ptr_->regDeviceCtrlCallback(std::bind(&SourceDriver::getDeviceCtrl, this),
                                     std::bind(&SourceDriver::putDeviceCtrl, this, std::placeholders::_1));

  if (this->send_device_ctrl_state_ros_)
    pub_device_ctrl_state_thread_ = std::thread(std::bind(&SourceDriver::publishDeviceCtrlCmd, this));

  if (!driver_ptr_->init(driver_param)) {
    WJ_ERROR << "Driver Initialize Error...." << WJ_REND;
    exit(-1);
  }
}

inline void SourceDriver::start() {
  driver_ptr_->start();
  if (this->recv_device_ctrl_cmd_ros_ && device_ctrl_ptr_ != nullptr)
    sub_device_ctrl_cmd_thread_ = std::thread(std::bind(&SourceDriver::subscribeDeviceCtrlState, this));
}

inline SourceDriver::~SourceDriver() {
}

inline void SourceDriver::stop() {
  driver_ptr_->stop();

  to_exit_process_ = true;
  if (this->send_point_cloud_ros_)
    point_cloud_process_thread_.join();
  if (this->send_imu_packet_ros_)
    imu_packet_process_thread_.join();
  if (this->send_laser_scan_ros_)
    scan_data_process_thread_.join();
  if (this->send_device_ctrl_state_ros_)
    pub_device_ctrl_state_thread_.join();
  if (this->recv_device_ctrl_cmd_ros_ && device_ctrl_ptr_ != nullptr)
    sub_device_ctrl_cmd_thread_.join();
}

/// @brief Provide idle point cloud instances to member 'driver_ptr_'
inline std::shared_ptr<LidarPointCloudMsg> SourceDriver::getPointCloud(void) {
  std::shared_ptr<LidarPointCloudMsg> point_cloud = free_point_cloud_queue_.pop();
  if (point_cloud.get() != NULL) {
    return point_cloud;
  }

  return std::make_shared<LidarPointCloudMsg>();
}

inline std::shared_ptr<ImuPacket> SourceDriver::getImuPacket() {
  std::shared_ptr<ImuPacket> pkt = free_imu_packet_queue_.pop();
  if (pkt.get() != NULL) {
    return pkt;
  }

  return std::make_shared<ImuPacket>();
}

inline std::shared_ptr<ScanData> SourceDriver::getScanData() {
  std::shared_ptr<ScanData> pkt = free_scan_data_queue_.pop();
  if (pkt.get() != NULL) {
    return pkt;
  }

  return std::make_shared<ScanData>();
}

inline std::shared_ptr<DeviceCtrl> SourceDriver::getDeviceCtrl() {
  std::shared_ptr<DeviceCtrl> pkt = free_device_ctrl_queue_.pop();
  if (pkt.get() != NULL) {
    return pkt;
  }

  return std::make_shared<DeviceCtrl>();
}

/// @brief Obtain a filled point cloud from member 'driver_ptr_'
void SourceDriver::putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg) {
  point_cloud_queue_.push(msg);
}

void SourceDriver::putImuPacket(std::shared_ptr<ImuPacket> msg) {
  imu_packet_queue_.push(msg);
}

/// @brief Get filled ScanData from member 'driver_ptr_'
void SourceDriver::putScanData(std::shared_ptr<ScanData> msg) {
  scan_data_queue_.push(msg);
}

void SourceDriver::putDeviceCtrl(std::shared_ptr<DeviceCtrl> msg) {
  device_ctrl_queue_.push(msg);
}

void SourceDriver::processImuPacket() {
  while (!to_exit_process_) {
    std::shared_ptr<ImuPacket> msg = imu_packet_queue_.popWait(1000);
    if (msg.get() == NULL) {
      continue;
    }

    sendImuPacket(*msg);

    free_imu_packet_queue_.push(msg);
  }
}

void SourceDriver::processPointCloud() {
  while (!to_exit_process_) {
    /// @brief Retrieve a point cloud instance from the pending point cloud
    /// queue 'point_cloud_queue'
    std::shared_ptr<LidarPointCloudMsg> msg = point_cloud_queue_.popWait(1000);
    if (msg.get() == NULL) {
      continue;
    }

    /// @brief Call 'sendPointCloud()', where the DestinationPointCloud instance
    /// in member 'pc_cb_vec_ []' is called to send the point cloud
    sendPointCloud(msg);

    /// @brief After processing, put it back into the idle queue
    /// 'free_point_cloud_queue' for next use
    free_point_cloud_queue_.push(msg);
  }
}

void SourceDriver::processScanData() {
  while (!to_exit_process_) {
    /// @brief Retrieve a scan data instance from the pending scan data
    /// queue 'scan_data_queue'
    std::shared_ptr<ScanData> msg = scan_data_queue_.popWait(1000);
    if (msg.get() == NULL) {
      continue;
    }
    /// @brief Call 'sendScanData()', where the DestinationScanData instance in
    /// member 'pc_cb_vec_ []' is called to send the point cloud
    sendScanData(*msg);

    /// @brief After processing, put it back into the idle queue
    /// 'free_stcan_data_queue' for next use
    free_scan_data_queue_.push(msg);
  }
}

void SourceDriver::publishDeviceCtrlCmd() {
  while (!to_exit_process_) {
    /// @brief Retrieve a point cloud instance from the pending device control
    /// queue 'device_ctrl_queue_'
    std::shared_ptr<DeviceCtrl> msg = device_ctrl_queue_.popWait(1000);
    if (msg.get() == NULL) {
      continue;
    }
    /// @brief Call 'sendDeviceCtrl()', where the DestinationDeviceCtrl instance in
    /// member 'pc_cb_vec_ []' is called to send the device control
    sendDeviceCtrlState(*msg);

    /// @brief After processing, put it back into the idle queue
    /// 'free_device_ctrl_queue_' for next use
    free_device_ctrl_queue_.push(msg);
  }
}

void SourceDriver::subscribeDeviceCtrlState() {
  while (!to_exit_process_) {
    std::shared_ptr<DeviceCtrl> msg = device_ctrl_ptr_->cached_message_.popWait(1000);
    if (msg.get() == NULL) {
      continue;
    }
    driver_ptr_->deviceCtrlApi(*msg);
  }
}

/// @brief prompt
inline void SourceDriver::putException(const lidar::Error &msg) {
  switch (msg.error_code_type) {
    case lidar::ErrCodeType::INFO_CODE:
      WJ_INFO << msg.toString() << WJ_REND;
      break;
    case lidar::ErrCodeType::WARNING_CODE:
      WJ_WARNING << msg.toString() << WJ_REND;
      break;
    case lidar::ErrCodeType::ERROR_CODE:
      WJ_ERROR << msg.toString() << WJ_REND;
      break;
  }
}
}  // namespace lidar

}  // namespace vanjee
