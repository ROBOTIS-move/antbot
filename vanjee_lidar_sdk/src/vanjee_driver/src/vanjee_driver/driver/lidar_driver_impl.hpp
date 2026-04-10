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
#include <chrono>
#include <sstream>

#include <vanjee_driver/common/error_code.hpp>
#include <vanjee_driver/driver/decoder/decoder_factory.hpp>
#include <vanjee_driver/driver/difop/difop_factory.hpp>
#include <vanjee_driver/driver/input/input_factory.hpp>
#include <vanjee_driver/macro/version.hpp>
#include <vanjee_driver/msg/device_ctrl_msg.hpp>
#include <vanjee_driver/msg/imu_packet.hpp>
#include <vanjee_driver/msg/scan_data_msg.hpp>
#include <vanjee_driver/utility/buffer.hpp>
#include <vanjee_driver/utility/sync_queue.hpp>

namespace vanjee {
namespace lidar {
inline std::stringstream getDriverVersion() {
  std::stringstream stream;
  stream << VANJEE_LIDAR_VERSION_MAJOR << "." << VANJEE_LIDAR_VERSION_MINOR << "." << VANJEE_LIDAR_VERSION_PATCH;
  return stream;
}
template <typename T_PointCloud>
class LidarDriverImpl {
 public:
  LidarDriverImpl();
  ~LidarDriverImpl();
  void regPointCloudCallback(const std::function<std::shared_ptr<T_PointCloud>(void)> &cb_get_cloud,
                             const std::function<void(std::shared_ptr<T_PointCloud>)> &cb_put_cloud);
  void regImuPacketCallback(const std::function<std::shared_ptr<ImuPacket>(void)> &cb_get_imu_pkt,
                            const std::function<void(std::shared_ptr<ImuPacket>)> &cb_put_imu_pkt);
  void regScanDataCallback(const std::function<std::shared_ptr<ScanData>(void)> &cb_get_scan_data,
                           const std::function<void(std::shared_ptr<ScanData>)> &cb_put_scan_data);
  void regDeviceCtrlCallback(const std::function<std::shared_ptr<DeviceCtrl>(void)> &cb_get_device_ctrl_state,
                             const std::function<void(std::shared_ptr<DeviceCtrl>)> &cb_put_device_ctrl_state);
  void regExceptionCallback(const std::function<void(const Error &)> &cb_excep);
  void deviceCtrlCmdInsert(const DeviceCtrl &device_ctrl);

  bool init(const WJDriverParam &param);
  bool start();
  void stop();

 private:
  void runExceptionCallback(const Error &error);

  std::shared_ptr<Buffer> packetGet(size_t size);
  void packetPut(std::shared_ptr<Buffer> pkt, bool stuffed);

  void processPacket();

  std::shared_ptr<T_PointCloud> getPointCloud();
  std::shared_ptr<ImuPacket> getImuPacket();
  std::shared_ptr<ScanData> getScanData();
  std::shared_ptr<DeviceCtrl> getDeviceCtrl();
  void splitFrame(uint16_t height, double ts);
  void imuPacketCallback();
  void scanDataCallback(double ts);
  void deviceCtrlCallback(double ts);
  void setPointCloudHeader(std::shared_ptr<T_PointCloud> msg, uint16_t height, double chan_ts);
  void setScanDataHeader(std::shared_ptr<ScanData> msg, double ts);
  void setDeviceCtrlHeader(std::shared_ptr<DeviceCtrl> msg, double ts);

 private:
  WJDriverParam driver_param_;
  std::function<std::shared_ptr<T_PointCloud>(void)> cb_get_cloud_;
  std::function<void(std::shared_ptr<T_PointCloud>)> cb_put_cloud_;
  std::function<std::shared_ptr<ImuPacket>(void)> cb_get_imu_pkt_;
  std::function<void(std::shared_ptr<ImuPacket>)> cb_put_imu_pkt_;
  std::function<std::shared_ptr<ScanData>(void)> cb_get_scan_data_;
  std::function<void(std::shared_ptr<ScanData>)> cb_put_scan_data_;
  std::function<std::shared_ptr<DeviceCtrl>(void)> cb_get_device_ctrl_state_;
  std::function<void(std::shared_ptr<DeviceCtrl>)> cb_put_device_ctrl_state_;
  std::function<void(const Error &)> cb_excep_;

  std::shared_ptr<Input> input_ptr_;
  std::shared_ptr<Decoder<T_PointCloud>> decoder_ptr_;
  std::shared_ptr<DifopBase> difop_ptr_;
  SyncQueue<std::shared_ptr<Buffer>> free_pkt_queue_;
  SyncQueue<std::shared_ptr<Buffer>> pkt_queue_;
  std::thread handle_thread_;
  uint32_t pkt_seq_;
  uint32_t imu_seq_;
  uint32_t point_cloud_seq_;
  uint32_t scan_data_seq_;
  uint32_t device_ctrl_seq_;
  bool to_exit_handle_;
  bool init_flag_;
  bool start_flag_;
};
template <typename T_PointCloud>
inline LidarDriverImpl<T_PointCloud>::LidarDriverImpl()
    : pkt_seq_(0), point_cloud_seq_(0), imu_seq_(0), scan_data_seq_(0), device_ctrl_seq_(0), init_flag_(false), start_flag_(false) {
}

template <typename T_PointCloud>
inline LidarDriverImpl<T_PointCloud>::~LidarDriverImpl() {
}

template <typename T_PointCloud>
std::shared_ptr<T_PointCloud> LidarDriverImpl<T_PointCloud>::getPointCloud() {
  while (1) {
    std::shared_ptr<T_PointCloud> cloud = cb_get_cloud_();
    if (cloud) {
      cloud->points.resize(0);
      return cloud;
    }

    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_POINTCLOUDNULL)), 1);
  }
}

template <typename T_PointCloud>
std::shared_ptr<ImuPacket> LidarDriverImpl<T_PointCloud>::getImuPacket() {
  while (1) {
    std::shared_ptr<ImuPacket> imu_packet = cb_get_imu_pkt_();
    if (imu_packet) {
      return imu_packet;
    }

    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_IMUPACKETNULL)), 1);
  }
}

template <typename T_PointCloud>
std::shared_ptr<ScanData> LidarDriverImpl<T_PointCloud>::getScanData() {
  while (1) {
    std::shared_ptr<ScanData> scan_data = cb_get_scan_data_();
    if (scan_data) {
      scan_data->ranges.clear();
      scan_data->intensities.clear();
      return scan_data;
    }

    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_LASERSCANPACKETNULL)), 1);
  }
}

template <typename T_PointCloud>
std::shared_ptr<DeviceCtrl> LidarDriverImpl<T_PointCloud>::getDeviceCtrl() {
  while (1) {
    std::shared_ptr<DeviceCtrl> device_ctrl = cb_get_device_ctrl_state_();
    if (device_ctrl) {
      return device_ctrl;
    }

    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_DEVICECONTROLPACKETNULL)), 1);
  }
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::regPointCloudCallback(const std::function<std::shared_ptr<T_PointCloud>(void)> &cb_get_cloud,
                                                          const std::function<void(std::shared_ptr<T_PointCloud>)> &cb_put_cloud) {
  cb_get_cloud_ = cb_get_cloud;
  cb_put_cloud_ = cb_put_cloud;
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::regImuPacketCallback(const std::function<std::shared_ptr<ImuPacket>(void)> &cb_get_imu_pkt,
                                                         const std::function<void(std::shared_ptr<ImuPacket>)> &cb_put_imu_pkt) {
  cb_get_imu_pkt_ = cb_get_imu_pkt;
  cb_put_imu_pkt_ = cb_put_imu_pkt;
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::regScanDataCallback(const std::function<std::shared_ptr<ScanData>(void)> &cb_get_scan_data,
                                                        const std::function<void(std::shared_ptr<ScanData>)> &cb_put_scan_data) {
  cb_get_scan_data_ = cb_get_scan_data;
  cb_put_scan_data_ = cb_put_scan_data;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regExceptionCallback(const std::function<void(const Error &)> &cb_excep) {
  cb_excep_ = cb_excep;
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::regDeviceCtrlCallback(const std::function<std::shared_ptr<DeviceCtrl>(void)> &cb_get_device_ctrl_state,
                                                          const std::function<void(std::shared_ptr<DeviceCtrl>)> &cb_put_device_ctrl_state) {
  cb_get_device_ctrl_state_ = cb_get_device_ctrl_state;
  cb_put_device_ctrl_state_ = cb_put_device_ctrl_state;
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::deviceCtrlCmdInsert(const DeviceCtrl &device_ctrl_cmd) {
  if (start_flag_) {
    decoder_ptr_->device_ctrl_->cmd_id = device_ctrl_cmd.cmd_id;
    decoder_ptr_->device_ctrl_->cmd_param = device_ctrl_cmd.cmd_param;
    difop_ptr_->addItem2GetDifoCtrlDataMapPtr(device_ctrl_cmd);
  }
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::init(const WJDriverParam &param) {
  if (init_flag_) {
    return true;
  }
  driver_param_ = param;
  decoder_ptr_ = DecoderFactory<T_PointCloud>::createDecoder(param.lidar_type, param.decoder_param);
  decoder_ptr_->point_cloud_ = getPointCloud();
  decoder_ptr_->imu_packet_ = getImuPacket();
  decoder_ptr_->scan_data_ = getScanData();
  decoder_ptr_->device_ctrl_ = getDeviceCtrl();
  decoder_ptr_->regCallback(std::bind(&LidarDriverImpl<T_PointCloud>::runExceptionCallback, this, std::placeholders::_1),
                            std::bind(&LidarDriverImpl<T_PointCloud>::splitFrame, this, std::placeholders::_1, std::placeholders::_2),
                            std::bind(&LidarDriverImpl<T_PointCloud>::imuPacketCallback, this),
                            std::bind(&LidarDriverImpl<T_PointCloud>::scanDataCallback, this, std::placeholders::_1),
                            std::bind(&LidarDriverImpl<T_PointCloud>::deviceCtrlCallback, this, std::placeholders::_1));

  double packet_duration = decoder_ptr_->getPacketDuration();

  input_ptr_ = InputFactory::createInput(param.input_type, param.input_param, packet_duration);
  input_ptr_->regCallback(std::bind(&LidarDriverImpl<T_PointCloud>::runExceptionCallback, this, std::placeholders::_1),
                          std::bind(&LidarDriverImpl<T_PointCloud>::packetGet, this, std::placeholders::_1),
                          std::bind(&LidarDriverImpl<T_PointCloud>::packetPut, this, std::placeholders::_1, std::placeholders::_2));

  if (driver_param_.input_type == InputType::ONLINE_LIDAR) {
    difop_ptr_ = DifopFactory::createDifop(param);
    difop_ptr_->regCallback(std::bind(&Input::send_, input_ptr_, std::placeholders::_1, std::placeholders::_2),
                            std::bind(&Decoder<T_PointCloud>::processDifopPkt, decoder_ptr_, std::placeholders::_1));
    difop_ptr_->initGetDifoCtrlDataMapPtr();
    decoder_ptr_->regGetDifoCtrlDataInterface(difop_ptr_->getDifoCtrlData_map_ptr_);

    difop_ptr_->start(driver_param_.input_param.connect_type);
  }

  if (!input_ptr_->init()) {
    goto failInputInit;
  }

  init_flag_ = true;

  return true;

failInputInit:
  if (driver_param_.input_type == InputType::ONLINE_LIDAR) {
    difop_ptr_->stop();
  }
  input_ptr_.reset();
  decoder_ptr_.reset();
  return false;
}
template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::start() {
  if (start_flag_) {
    return true;
  }

  if (!init_flag_) {
    return false;
  }

  to_exit_handle_ = false;
  handle_thread_ = std::thread(std::bind(&LidarDriverImpl<T_PointCloud>::processPacket, this));

  input_ptr_->start();

  start_flag_ = true;
  return true;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::stop() {
  if (!start_flag_) {
    return;
  }

  input_ptr_->stop();
  if (driver_param_.input_type == InputType::ONLINE_LIDAR) {
    difop_ptr_->stop();
  }

  to_exit_handle_ = true;
  handle_thread_.join();
  start_flag_ = false;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::runExceptionCallback(const Error &error) {
  if (cb_excep_) {
    cb_excep_(error);
  }
}
/// @brief Obtain an idle packet instance from an idle packet queue
template <typename T_PointCloud>
inline std::shared_ptr<Buffer> LidarDriverImpl<T_PointCloud>::packetGet(size_t size) {
  std::shared_ptr<Buffer> pkt = free_pkt_queue_.pop();
  if (pkt.get() != NULL) {
    return pkt;
  }

  return std::make_shared<Buffer>(size);
}
/// @brief Build a filled instance and transfer the filled packet instance to
/// the 'pkt_queue_' queue for processing
template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::packetPut(std::shared_ptr<Buffer> pkt, bool stuffed) {
  constexpr static int64_t PACKET_POOL_MAX = 10240;
  if (!stuffed) {
    free_pkt_queue_.push(pkt);
    return;
  }

  size_t sz = pkt_queue_.push(pkt);
  if (sz > PACKET_POOL_MAX) {
    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_PKTBUFOVERFLOW)), 1);
    pkt_queue_.clear();
  }
}

/// @brief Retrieve a packet instance, process it, and then put it back into the
/// free queue 'free_pkt_queue_' after processing
template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::processPacket() {
  while (!to_exit_handle_) {
    std::shared_ptr<Buffer> pkt = pkt_queue_.popWait(50000);

    if (pkt.get() == NULL) {
      continue;
    }

    bool pkt_to_split = decoder_ptr_->processMsopPkt(pkt->data(), pkt->dataSize());

    if (driver_param_.input_type == InputType::ONLINE_LIDAR && difop_ptr_->processDifoPktFlag())
      difop_ptr_->dataEnqueue(pkt->getIp(), pkt->getBuf());

    free_pkt_queue_.push(pkt);
  }
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::imuPacketCallback() {
  std::shared_ptr<ImuPacket> pkt = decoder_ptr_->imu_packet_;
  pkt->seq = imu_seq_++;
  if (pkt->timestamp > 1) {
#ifdef ENABLE_GRAVITY_ACCELERATION_REMOVE
    double G = 9.81;
    std::vector<double> gravity_acce_xyz;
    gravity_acce_xyz.resize(3);
    gravity_acce_xyz[0] = 2 * (pkt->orientation[1] * pkt->orientation[3] - pkt->orientation[0] * pkt->orientation[2]) * G;
    gravity_acce_xyz[1] = 2 * (pkt->orientation[2] * pkt->orientation[3] + pkt->orientation[0] * pkt->orientation[1]) * G;
    gravity_acce_xyz[2] = (pkt->orientation[0] * pkt->orientation[0] - pkt->orientation[1] * pkt->orientation[1] -
                           pkt->orientation[2] * pkt->orientation[2] + pkt->orientation[3] * pkt->orientation[3]) *
                          G;

    pkt->linear_acce[0] -= gravity_acce_xyz[0];
    pkt->linear_acce[1] -= gravity_acce_xyz[1];
    pkt->linear_acce[2] -= gravity_acce_xyz[2];
#endif
    cb_put_imu_pkt_(pkt);
    decoder_ptr_->imu_packet_ = getImuPacket();
  } else {
    runExceptionCallback(Error(ERRCODE_WRONGIMUPACKET));
  }
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::scanDataCallback(double ts) {
  std::shared_ptr<ScanData> pkt = decoder_ptr_->scan_data_;
  if (pkt->ranges.size() > 0) {
    setScanDataHeader(pkt, ts);
    cb_put_scan_data_(pkt);
    decoder_ptr_->scan_data_ = getScanData();
  } else {
    runExceptionCallback(Error(ERRCODE_ZEROPOINTS));
  }
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::deviceCtrlCallback(double ts) {
  std::shared_ptr<DeviceCtrl> pkt = decoder_ptr_->device_ctrl_;
  if (pkt->cmd_state != 0) {
    setDeviceCtrlHeader(pkt, ts);
    cb_put_device_ctrl_state_(pkt);
    decoder_ptr_->device_ctrl_ = getDeviceCtrl();
  } else {
    runExceptionCallback(Error(ERRCODE_ZEROPOINTS));
  }
}

/// @brief Point cloud split frame function
template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::splitFrame(uint16_t height, double ts) {
  std::shared_ptr<T_PointCloud> cloud = decoder_ptr_->point_cloud_;
  if (cloud->points.size() > 0) {
    setPointCloudHeader(cloud, height, ts);
#ifdef POINT_TYPE_XYZIRT
    if (!decoder_ptr_->param_.use_offset_timestamp) {
      for (int i = 0; i < cloud->points.size(); i++) {
        cloud->points[i].timestamp += ts;
      }
    }
#endif
    cb_put_cloud_(cloud);
    decoder_ptr_->point_cloud_ = getPointCloud();
  } else {
    runExceptionCallback(Error(ERRCODE_ZEROPOINTS));
  }
}
/// @brief Set Point cloud head
template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::setPointCloudHeader(std::shared_ptr<T_PointCloud> msg, uint16_t height, double ts) {
  msg->seq = point_cloud_seq_++;
  msg->timestamp = ts;
  msg->is_dense = driver_param_.decoder_param.dense_points;
  msg->height = height;
  msg->width = (uint32_t)msg->points.size() / msg->height;
}

/// @brief Set scandata head
template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::setScanDataHeader(std::shared_ptr<ScanData> msg, double ts) {
  msg->seq = scan_data_seq_++;
  msg->timestamp = ts;
}

/// @brief Set device ctrl state head
template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::setDeviceCtrlHeader(std::shared_ptr<DeviceCtrl> msg, double ts) {
  msg->seq = device_ctrl_seq_++;
  msg->timestamp = ts;
}

}  // namespace lidar

}  // namespace vanjee
