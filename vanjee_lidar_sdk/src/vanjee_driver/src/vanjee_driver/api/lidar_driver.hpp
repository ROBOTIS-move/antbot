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

#include <vanjee_driver/driver/lidar_driver_impl.hpp>
#include <vanjee_driver/msg/device_ctrl_msg.hpp>
#include <vanjee_driver/msg/imu_packet.hpp>
#include <vanjee_driver/msg/scan_data_msg.hpp>

namespace vanjee {
namespace lidar {
std::stringstream getDriverVersion();

/**
 * @brief 激光雷达驱动接口类
 */
template <typename T_PointCloud>
class LidarDriver {
 private:
  std::shared_ptr<LidarDriverImpl<T_PointCloud>> driver_ptr_;

 public:
  /**
   * @brief 构造函数，初始化驱动程序指针
   */
  LidarDriver() : driver_ptr_(std::make_shared<LidarDriverImpl<T_PointCloud>>()) {
  }
  /**
   * @brief 向驱动注册激光雷达点云回调函数。点云就绪后，将调用此函数
   * @param callback 回调函数
   */
  inline void regPointCloudCallback(const std::function<std::shared_ptr<T_PointCloud>(void)> &cb_get_cloud,
                                    const std::function<void(std::shared_ptr<T_PointCloud>)> &cb_put_cloud) {
    driver_ptr_->regPointCloudCallback(cb_get_cloud, cb_put_cloud);
  }

  inline void regImuPacketCallback(const std::function<std::shared_ptr<ImuPacket>(void)> &cb_get_imu_pkt,
                                   const std::function<void(std::shared_ptr<ImuPacket>)> &cb_put_imu_pkt) {
    driver_ptr_->regImuPacketCallback(cb_get_imu_pkt, cb_put_imu_pkt);
  }

  inline void regScanDataCallback(const std::function<std::shared_ptr<ScanData>(void)> &cb_get_scan_data,
                                  const std::function<void(std::shared_ptr<ScanData>)> &cb_put_scan_data) {
    driver_ptr_->regScanDataCallback(cb_get_scan_data, cb_put_scan_data);
  }

  inline void deviceCtrlApi(const DeviceCtrl &device_ctrl_cmd) {
    driver_ptr_->deviceCtrlCmdInsert(device_ctrl_cmd);
  }

  inline void regDeviceCtrlCallback(const std::function<std::shared_ptr<DeviceCtrl>(void)> &cb_get_device_ctrl_state,
                                    const std::function<void(std::shared_ptr<DeviceCtrl>)> &cb_put_device_ctrl_state) {
    driver_ptr_->regDeviceCtrlCallback(cb_get_device_ctrl_state, cb_put_device_ctrl_state);
  }
  /**
   * @brief 向驱动程序注册异常消息回调函数。发生错误时，将调用此函数
   * @param callback 回调函数
   */
  inline void regExceptionCallback(const std::function<void(const Error &)> &cb_excep) {
    driver_ptr_->regExceptionCallback(cb_excep);
  }
  /**
   * @brief
   * 初始化函数，用于设置参数和实例对象，用于从在线激光雷达或pcap获取数据包
   * @param param 配置参数
   * @return 成功 返回 true; 否则 返回 false
   */
  inline bool init(const WJDriverParam &param) {
    return driver_ptr_->init(param);
  }
  /**
   * @brief 开始接收数据包的线程
   * @return 成功 返回 true; 失败 返回 false
   */
  inline bool start() {
    return driver_ptr_->start();
  }
  /**
   * @brief 停止所有线程
   */
  inline void stop() {
    driver_ptr_->stop();
  }
};

}  // namespace lidar

}  // namespace vanjee
