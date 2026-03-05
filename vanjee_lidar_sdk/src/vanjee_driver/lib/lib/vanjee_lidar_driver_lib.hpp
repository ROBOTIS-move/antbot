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

#include "yaml_reader.hpp"

#ifdef ENABLE_PCL_POINTCLOUD
#include <vanjee_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <vanjee_driver/msg/point_cloud_msg.hpp>
#endif

using namespace vanjee::lidar;

typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

class PointCloudClient {
 public:
  uint8_t lidar_id_;
  PointCloudMsg point_cloud_msg_;
};

class ImuPacketClient : public ImuPacket {
 public:
  uint8_t lidar_id_;
};

class ScanDataClient : public ScanData {
 public:
  uint8_t lidar_id_;
};

class DeviceCtrlClient : public DeviceCtrl {
 public:
  uint8_t lidar_id_;
};

class VanjeeLidarDriverLib {
 public:
  std::string name_;
  uint8_t lidar_id_;

  bool init(const WJDriverParam &param, const std::function<void(std::shared_ptr<PointCloudClient>)> &cb_put_cloud,
            const std::function<void(std::shared_ptr<ImuPacketClient>)> &cb_put_imu_pkt,
            const std::function<void(std::shared_ptr<ScanDataClient>)> &cb_put_scan_data,
            const std::function<void(std::shared_ptr<DeviceCtrlClient>)> &cb_put_device_ctrl_state);

  bool start();

  void stop();

  void deviceCtrlApi(DeviceCtrl device_ctrl);

 protected:
  LidarDriver<PointCloudMsg> driver_;
  SyncQueue<std::shared_ptr<PointCloudClient>> free_cloud_queue_;
  SyncQueue<std::shared_ptr<PointCloudClient>> stuffed_cloud_queue_;
  SyncQueue<std::shared_ptr<ImuPacketClient>> free_imu_queue_;
  SyncQueue<std::shared_ptr<ImuPacketClient>> stuffed_imu_queue_;
  SyncQueue<std::shared_ptr<ScanDataClient>> free_scan_data_queue_;
  SyncQueue<std::shared_ptr<ScanDataClient>> stuffed_scan_data_queue_;
  SyncQueue<std::shared_ptr<DeviceCtrlClient>> free_device_ctrl_queue_;
  SyncQueue<std::shared_ptr<DeviceCtrlClient>> stuffed_device_ctrl_queue_;

  std::function<void(std::shared_ptr<PointCloudClient>)> point_cloud_callback_;
  std::function<void(std::shared_ptr<ImuPacketClient>)> imu_pack_callback_;
  std::function<void(std::shared_ptr<ScanDataClient>)> laser_scan_callback_;
  std::function<void(std::shared_ptr<DeviceCtrlClient>)> device_ctrl_callback_;

  std::thread point_cloud_thread_;
  std::thread imu_thread_;
  std::thread laser_scan_thread_;
  std::thread device_ctrl_thread_;
  std::thread device_ctrl_cmd_thread_;

 protected:
  // @brief allocate memory for point cloud，sdk would call this callback to get
  // the memory for point cloud storage.
  std::shared_ptr<PointCloudMsg> allocatePointCloudMemoryCallback();
  // @brief this callback which would return the point cloud to queue
  void pointCloudCallback(std::shared_ptr<PointCloudMsg> msg);

  // @brief allocate memory for imu，sdk would call this callback to get the
  // memory for imu storage.
  std::shared_ptr<ImuPacket> allocateImuPacketMemoryCallback();
  // @brief this callback which would return the imu to queue
  void imuPacketCallback(std::shared_ptr<ImuPacket> msg);

  // @brief allocate memory for laserScan，sdk would call this callback to get
  // the memory for laserScan storage.
  std::shared_ptr<ScanData> allocateLaserScanMemoryCallback();
  // @brief this callback which would return the laserScan to queue
  void laserScanCallback(std::shared_ptr<ScanData> msg);

  // @brief allocate memory for device ctrl would call this callback to get
  // the memory for device ctrol storage.
  std::shared_ptr<DeviceCtrl> allocateDeviceCtrlMemoryCallback();
  // @brief this callback which would return the device ctrl to queue
  void deviceCtrlCallback(std::shared_ptr<DeviceCtrl> msg);

  // @brief this callback which would return the exception prompt information to
  // terminal
  void exceptionCallback(const Error &code);

  void processPointCloud();
  void processImu();
  void processLaserScan();
  void processDeviceCtrl();
  void processDeviceCtrlCmd();
};

SyncQueue<std::shared_ptr<DeviceCtrlClient>> device_ctrl_param_;

bool to_exit_driver_ = false;
bool driver_status_ = false;

void vanjeeLidarDriverDeviceCtrlApi(DeviceCtrlClient &device_ctrl);

bool vanjeeLidarDriverLibStop();

int vanjeeLidarDriverLibStart(std::string config_path, const std::function<void(std::shared_ptr<PointCloudClient>)> &cb_put_cloud,
                              const std::function<void(std::shared_ptr<ImuPacketClient>)> &cb_put_imu_pkt,
                              const std::function<void(std::shared_ptr<ScanDataClient>)> &cb_put_scan_data,
                              const std::function<void(std::shared_ptr<DeviceCtrlClient>)> &cb_put_device_ctrl_data);
