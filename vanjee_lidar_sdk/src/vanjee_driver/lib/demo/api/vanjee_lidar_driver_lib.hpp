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

#ifdef ENABLE_PCL_POINTCLOUD
#include "pcl_point_cloud_msg.hpp"
#else
#include "point_cloud_msg.hpp"
#endif
#include "device_ctrl_msg.hpp"
#include "imu_packet.hpp"
#include "scan_data_msg.hpp"

using namespace vanjee::lidar;

typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

class PointCloudClient {
 public:
  uint8_t lidar_id_;
  PointCloudMsg point_cloud_msg_;
};

class ImuPacketClient : public vanjee::lidar::ImuPacket {
 public:
  uint8_t lidar_id_;
};

class ScanDataClient : public vanjee::lidar::ScanData {
 public:
  uint8_t lidar_id_;
};

class DeviceCtrlClient : public vanjee::lidar::DeviceCtrl {
 public:
  uint8_t lidar_id_;
};

extern void vanjeeLidarDriverDeviceCtrlApi(DeviceCtrlClient &deviceCtrl);

extern int vanjeeLidarDriverLibStart(std::string config_path, const std::function<void(std::shared_ptr<PointCloudClient>)> &cb_put_cloud,
                                     const std::function<void(std::shared_ptr<ImuPacketClient>)> &cb_put_imu_pkt,
                                     const std::function<void(std::shared_ptr<ScanDataClient>)> &cb_put_scan_data,
                                     const std::function<void(std::shared_ptr<DeviceCtrlClient>)> &cb_put_device_ctrl_state);
extern bool vanjeeLidarDriverLibStop();
