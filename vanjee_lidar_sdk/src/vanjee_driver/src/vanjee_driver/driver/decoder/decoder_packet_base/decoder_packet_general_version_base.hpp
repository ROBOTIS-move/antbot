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

#include <string.h>

#include <cmath>
#include <mutex>
using namespace std;
#include <vanjee_driver/driver/decoder/decoder_packet_base/imu/imuParamGet.hpp>
#include <vanjee_driver/driver/decoder/member_checker.hpp>

#include "vanjee_driver/common/wj_log.hpp"

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct _ProtocolVersion {
  uint8_t main_version_id_;
  uint8_t sub_version_id_;
  uint16_t getVersionId() {
    return (main_version_id_ << 8) + sub_version_id_;
  }
} ProtocolVersion;

typedef struct _DataBlockInfo {
  uint8_t data_block_packet_type_;
  uint16_t data_block_num_in_packet_;
  uint16_t valid_data_block_num_in_packet_;
  uint16_t row_channel_num_in_data_block_;
  uint16_t col_channel_num_in_data_block_;
  uint8_t distance_resolution_;
} DataBlockInfo;

typedef struct _TimestampInfo {
  uint8_t clock_source_;
  uint32_t second_;
  uint32_t nanosecond_;
} TimestampInfo;

typedef struct _VanjeeLidarPointCloudPacketBaseHeader {
  uint8_t header_[2];
  uint16_t len_;
  uint8_t lidar_type_[2];
  ProtocolVersion protocol_version_;
  uint16_t getLidarType() {
    return (lidar_type_[0] << 8) + lidar_type_[1];
  }
} VanjeeLidarPointCloudPacketBaseHeader;

typedef struct _VanjeeLidarPointCloudPacketHeaderV1_0 {
  VanjeeLidarPointCloudPacketBaseHeader vanjee_lidar_point_cloud_packet_base_header_;
  uint16_t circle_id_;
  uint16_t total_pkt_num_;
  uint16_t pkt_id_;
  uint16_t operate_frequency_;
  uint8_t echo_num_;
  uint16_t col_channel_num_;
  uint16_t row_channel_num_;
  uint8_t reserved_field_1_[8];
  DataBlockInfo data_block_info_;
  TimestampInfo timestamp_info_;
  uint8_t reserved_field_2_[16];
  uint16_t info_flag_;
} VanjeeLidarPointCloudPacketHeaderV1_0;

typedef struct _ImuDataInfo {
  int32_t imu_data_timestamp_;
  int32_t imu_linear_acce_x_;
  int32_t imu_linear_acce_y_;
  int32_t imu_linear_acce_z_;
  int32_t imu_angle_voc_x_;
  int32_t imu_angle_voc_y_;
  int32_t imu_angle_voc_z_;

  void imuDataInfoInit() {
    imu_data_timestamp_ = -1;
    imu_linear_acce_x_ = imu_linear_acce_y_ = imu_linear_acce_z_ = -1;
    imu_angle_voc_x_ = imu_angle_voc_y_ = imu_angle_voc_z_ = -1;
  }
} ImuDataInfo;

typedef struct _FunctionSafetyInfo {
  uint8_t lidar_running_state_;
  uint16_t error_code_id_;
  uint8_t fault_detection_module_counter_;
} FunctionSafetyInfo;

typedef struct _CyberSecurityInfo {
  uint8_t digital_signature_field_[32];
} CyberSecurityInfo;

typedef struct _CheckInfo {
  uint8_t check_[2];
} CheckInfo;

typedef struct _TailInfo {
  uint8_t tail_[2];
} TailInfo;

typedef struct _PointInfo {
  uint32_t distance_;
  double azimuth_;
  double elevation_;
  double reflectivity_;
  uint8_t confidence_;
  uint16_t ring_;
  double timestamp_;
  uint32_t id_;
} PointInfo;

typedef struct _DataBlockAngleAndTimestampInfo {
  uint16_t protocol_version_;
  double first_point_ts_;
  uint16_t circle_id_;
  uint16_t total_pkt_num_;
  uint16_t packet_id_;
  uint16_t operate_frequency_;
  uint8_t echo_num_;
  uint16_t col_channel_num_;
  uint16_t row_channel_num_;
  uint8_t data_block_index_in_packet_;
  DataBlockInfo data_block_info_;
  uint8_t reserved_field_1_[8];
  uint8_t reserved_field_2_[16];
  std::vector<PointInfo> point_info_vector_;

  void init(uint16_t protocol_version, double pkt_ts, uint16_t circle_id, uint16_t total_pkt_num, uint16_t packet_id, uint16_t operate_frequency,
            uint8_t echo_num, uint16_t col_channel_num, uint16_t row_channel_num, uint8_t data_block_index_in_packet, DataBlockInfo& data_block_info,
            uint8_t* reserved_field_1, uint8_t* reserved_field_2) {
    protocol_version_ = protocol_version;
    first_point_ts_ = pkt_ts;
    circle_id_ = circle_id;
    total_pkt_num_ = total_pkt_num;
    packet_id_ = packet_id;
    operate_frequency_ = operate_frequency;
    echo_num_ = echo_num;
    col_channel_num_ = col_channel_num;
    row_channel_num_ = row_channel_num;
    data_block_index_in_packet_ = data_block_index_in_packet;
    memcpy((void*)&data_block_info_, (void*)&data_block_info, sizeof(DataBlockInfo));
    memcpy(reserved_field_1_, reserved_field_1, sizeof(reserved_field_1_));
    memcpy(reserved_field_2_, reserved_field_2, sizeof(reserved_field_2_));

    point_info_vector_.resize(0);
  }
} DataBlockAngleAndTimestampInfo;
#pragma pack(pop)
template <typename T_PointCloud, typename T_DataBlock, typename T_DataUnit>
class DecoderPacketGeneralVersionBase {
 public:
  std::shared_ptr<ImuParamGet> imu_params_get_;
  Decoder<T_PointCloud>* decoder_ptr_;
  std::vector<double> all_points_luminous_moment_;
  uint32_t pre_circle_id_;
  double pre_imu_timestamp_;
  uint8_t fault_detection_module_counter_;
  double first_point_ts_ = 0.0;

 public:
  DecoderPacketGeneralVersionBase(Decoder<T_PointCloud>* decoder_ptr) {
    imu_params_get_ = std::make_shared<ImuParamGet>();
    decoder_ptr_ = decoder_ptr;
  }

  virtual bool decoderPacket(const uint8_t* buf, size_t size,
                             std::function<void(DataBlockAngleAndTimestampInfo&, T_DataBlock*)> update_angle_and_timestamp_info_callback,
                             std::function<void(T_DataUnit*, PointInfo&)> decoder_data_unit_callback,
                             std::function<void()> point_cloud_algorithm_callbck) {
    VanjeeLidarPointCloudPacketBaseHeader& vanjee_lidar_point_cloud_packet_base_header = *(VanjeeLidarPointCloudPacketBaseHeader*)buf;
    return decoderPointCloudPacketProtocolVersionV1_0(buf, size, update_angle_and_timestamp_info_callback, decoder_data_unit_callback,
                                                      point_cloud_algorithm_callbck);
  }

  bool decoderPointCloudPacketProtocolVersionV1_0(
      const uint8_t* buf, uint16_t size, std::function<void(DataBlockAngleAndTimestampInfo&, T_DataBlock*)> update_angle_and_timestamp_info_callback,
      std::function<void(T_DataUnit*, PointInfo&)> decoder_data_unit_callback, std::function<void()> point_cloud_algorithm_callbck) {
    bool ret = false;
    uint16_t offset = 0;
    VanjeeLidarPointCloudPacketHeaderV1_0& vanjee_lidar_point_cloud_packet_header = *((VanjeeLidarPointCloudPacketHeaderV1_0*)(buf + offset));

    if (!protocolCheck(buf, size, vanjee_lidar_point_cloud_packet_header))
      return false;

    double pkt_ts = 0.0, host_pkt_ts = 0.0, lidar_pkt_ts = 0.0;
    host_pkt_ts = getTimeHost() * 1e-6;
    lidar_pkt_ts =
        vanjee_lidar_point_cloud_packet_header.timestamp_info_.second_ + vanjee_lidar_point_cloud_packet_header.timestamp_info_.nanosecond_ * 1e-9;
    if (!decoder_ptr_->param_.use_lidar_clock) {
      pkt_ts = host_pkt_ts;
    } else {
      pkt_ts = lidar_pkt_ts;
      if (pkt_ts < 3600) {
        pkt_ts == 3600;
      }
    }

    functionalSafetyDecoder(buf, size, vanjee_lidar_point_cloud_packet_header);
    if (decoder_ptr_->param_.imu_enable != -1) {
      sendImuData(buf, size, pkt_ts, lidar_pkt_ts, vanjee_lidar_point_cloud_packet_header);
    }

    uint32_t loss_circles_num = (vanjee_lidar_point_cloud_packet_header.circle_id_ + 0x10000 - pre_circle_id_) % 0x10000;
    pre_circle_id_ = vanjee_lidar_point_cloud_packet_header.circle_id_;
    if (loss_circles_num > 1)
      decoder_ptr_->point_cloud_->points.clear();

    if ((loss_circles_num == 1 && decoder_ptr_->point_cloud_->points.size() != 0)) {
      decoder_ptr_->first_point_ts_ = first_point_ts_;
      decoder_ptr_->last_point_ts_ = decoder_ptr_->first_point_ts_ + all_points_luminous_moment_[all_points_luminous_moment_.size() - 1];
      if (decoder_ptr_->param_.point_cloud_enable) {
        if (point_cloud_algorithm_callbck != nullptr)
          point_cloud_algorithm_callbck();
        transformPointCloud();
        decoder_ptr_->cb_split_frame_(vanjee_lidar_point_cloud_packet_header.row_channel_num_, decoder_ptr_->cloudTs());
        ret = true;
      }

      if (decoder_ptr_->param_.laser_scan_enable) {
        decoder_ptr_->scan_data_->ranges.clear();
        decoder_ptr_->scan_data_->intensities.clear();
      }
    }

    offset += sizeof(VanjeeLidarPointCloudPacketHeaderV1_0);
    uint16_t data_block_num_in_packet = vanjee_lidar_point_cloud_packet_header.data_block_info_.data_block_num_in_packet_;
    DataBlockAngleAndTimestampInfo data_block_angle_and_timestamp_info;
    for (uint16_t i = 0; i < data_block_num_in_packet; i++) {
      if (i >= vanjee_lidar_point_cloud_packet_header.data_block_info_.valid_data_block_num_in_packet_)
        break;

      data_block_angle_and_timestamp_info.init(
          vanjee_lidar_point_cloud_packet_header.vanjee_lidar_point_cloud_packet_base_header_.protocol_version_.getVersionId(), pkt_ts,
          vanjee_lidar_point_cloud_packet_header.circle_id_, vanjee_lidar_point_cloud_packet_header.total_pkt_num_,
          vanjee_lidar_point_cloud_packet_header.pkt_id_, vanjee_lidar_point_cloud_packet_header.operate_frequency_,
          vanjee_lidar_point_cloud_packet_header.echo_num_, vanjee_lidar_point_cloud_packet_header.col_channel_num_,
          vanjee_lidar_point_cloud_packet_header.row_channel_num_, i, vanjee_lidar_point_cloud_packet_header.data_block_info_,
          vanjee_lidar_point_cloud_packet_header.reserved_field_1_, vanjee_lidar_point_cloud_packet_header.reserved_field_2_);
      T_DataBlock* data_block_ptr = (T_DataBlock*)(buf + offset);
      update_angle_and_timestamp_info_callback(data_block_angle_and_timestamp_info, data_block_ptr);
      decoderDataBlock(data_block_ptr, data_block_angle_and_timestamp_info, decoder_data_unit_callback, point_cloud_algorithm_callbck);

      offset += sizeof(T_DataBlock);
    }
    first_point_ts_ = data_block_angle_and_timestamp_info.first_point_ts_;
    if (vanjee_lidar_point_cloud_packet_header.pkt_id_ == vanjee_lidar_point_cloud_packet_header.total_pkt_num_) {
      decoder_ptr_->first_point_ts_ = first_point_ts_;
      decoder_ptr_->last_point_ts_ = decoder_ptr_->first_point_ts_ + all_points_luminous_moment_[all_points_luminous_moment_.size() - 1];

      if (this->decoder_ptr_->param_.point_cloud_enable) {
        if (point_cloud_algorithm_callbck != nullptr)
          point_cloud_algorithm_callbck();
        transformPointCloud();
        decoder_ptr_->cb_split_frame_(vanjee_lidar_point_cloud_packet_header.row_channel_num_, decoder_ptr_->cloudTs());
        ret = true;
      }

      if (this->decoder_ptr_->param_.laser_scan_enable) {
        if (decoder_ptr_->scan_data_->ranges.size() == vanjee_lidar_point_cloud_packet_header.col_channel_num_) {
          decoder_ptr_->cb_scan_data_(decoder_ptr_->cloudTs());
          ret = true;
        }
        decoder_ptr_->scan_data_->ranges.clear();
        decoder_ptr_->scan_data_->intensities.clear();
      }
    }

    return ret;
  }

  bool protocolCheck(const uint8_t* buf, uint16_t size, VanjeeLidarPointCloudPacketHeaderV1_0& vanjee_lidar_point_cloud_packet_header) {
    bool ret = true;
    // 0：exclude cyber security info field；1：include cyber security info field,but data is invalidate;
    // 2:include cyber security info field ,and data is valid;s
    uint8_t cyber_security_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 4) & 0x03;
    // 0：exclude check info field；1：include check info field,but data is invalidate;2：crc16;3：bcc;
    uint8_t check_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 6) & 0x03;

    TailInfo& tailInfo = *(TailInfo*)(buf + size - 2);
    if (tailInfo.tail_[0] != 0xee || tailInfo.tail_[1] != 0xee) {
      WJ_ERROR << "Tail: Frame tail err" << WJ_REND;
      ret = false;
    }

    if (check_flag == 2) {
      CheckInfo& checkInfo = *(CheckInfo*)(buf + size - 4);
      if (decoder_ptr_->checkCRC16(buf, 2, size - 4) != (uint16_t)(checkInfo.check_[0] | (checkInfo.check_[1] << 8))) {
        WJ_ERROR << "Data check: Check err" << WJ_REND;
        ret = false;
      }
    } else if (check_flag == 3) {
      CheckInfo& checkInfo = *(CheckInfo*)(buf + size - 4);
      if (decoder_ptr_->checkBCC(buf, 2, size - 4) != checkInfo.check_[1]) {
        WJ_ERROR << "Data check: Check err" << WJ_REND;
        ret = false;
      }
    }
    return ret;
  }

  void functionalSafetyDecoder(const uint8_t* buf, uint16_t size, VanjeeLidarPointCloudPacketHeaderV1_0& vanjee_lidar_point_cloud_packet_header) {
    // 0：exclude functional safety info field；1：include functional safety info field,but data is invalidate;
    // 2:include functional safety info field, and data is valid;
    uint8_t function_safety_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 2) & 0x03;
    if (function_safety_flag == 2) {
      // 0：exclude cyber security info field；1：include cyber security info field,but data is invalidate;
      // 2:include cyber security info field ,and data is valid;
      uint8_t cyber_security_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 4) & 0x03;
      // 0：exclude check info field；1：include check info field,but data is invalidate;2：crc16;3：bcc;
      uint8_t check_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 6) & 0x03;
      uint16_t offset = sizeof(TailInfo) + 1;
      if (check_flag != 0)
        offset += sizeof(CheckInfo);
      if (cyber_security_flag != 0)
        offset += sizeof(CyberSecurityInfo);
      offset += sizeof(FunctionSafetyInfo);
      FunctionSafetyInfo& functionSafetyInfo = *(FunctionSafetyInfo*)(buf + size - offset);
      if (fault_detection_module_counter_ != 0 && functionSafetyInfo.fault_detection_module_counter_ == fault_detection_module_counter_) {
        WJ_ERROR << "Functional safety: fault detection module error" << WJ_REND;
        return;
      }
      std::string err_code_str = "";
      for (int i = 0; i < 16; i++) {
        if (((functionSafetyInfo.error_code_id_ >> i) & 0x01) == 1)
          err_code_str += " " + std::to_string(i);
      }
      if (err_code_str.length() > 1) {
        WJ_ERROR << "Functional safety: fault code:" << err_code_str << WJ_REND;
      }
    }
  }

  void sendImuData(const uint8_t* buf, uint16_t size, double timestamp, double lidar_timestamp,
                   VanjeeLidarPointCloudPacketHeaderV1_0& vanjee_lidar_point_cloud_packet_header) {
    // 0：exclude imu info field ;1: include imu info field,but data is invalidate;
    // 2:include imu info field,and data is valid;
    uint8_t imu_flag = vanjee_lidar_point_cloud_packet_header.info_flag_ & 0x03;

    if (imu_flag == 2) {
      // 0：exclude functional safety info field；1：include functional safety info field,but data is invalidate;
      // 2:include functional safety info field ,and data is valid;
      uint8_t function_safety_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 2) & 0x03;
      // 0：exclude cyber security info field；1：include cyber security info field,but data is invalidate;
      // 2:include cyber security info field ,and data is valid;
      uint8_t cyber_security_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 4) & 0x03;
      // 0：exclude check info field；1：include check info field,but data is invalidate;2：crc16;3：bcc;
      uint8_t check_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 6) & 0x03;
      uint16_t offset = sizeof(TailInfo);
      if (check_flag != 0)
        offset += sizeof(CheckInfo);
      if (cyber_security_flag != 0)
        offset += sizeof(CyberSecurityInfo);
      if (function_safety_flag != 0)
        offset += sizeof(FunctionSafetyInfo);
      offset += sizeof(ImuDataInfo);

      ImuDataInfo& imuDataInfo = *(ImuDataInfo*)(buf + size - offset);
      double imu_timestamp = lidar_timestamp + (double)imuDataInfo.imu_data_timestamp_ * 1e-9;
      if (abs(imu_timestamp - pre_imu_timestamp_) > 1e-6) {
        bool ret = imu_params_get_->imuGet(imuDataInfo.imu_angle_voc_x_, imuDataInfo.imu_angle_voc_y_, imuDataInfo.imu_angle_voc_z_,
                                           imuDataInfo.imu_linear_acce_x_, imuDataInfo.imu_linear_acce_y_, imuDataInfo.imu_linear_acce_z_,
                                           imu_timestamp, -100.0, false, false, false, false, true);
        if (ret) {
          decoder_ptr_->imu_packet_->timestamp = timestamp + (double)imuDataInfo.imu_data_timestamp_ * 1e-9;
          decoder_ptr_->imu_packet_->angular_voc[0] = imu_params_get_->imu_result_stu_.x_angle;
          decoder_ptr_->imu_packet_->angular_voc[1] = imu_params_get_->imu_result_stu_.y_angle;
          decoder_ptr_->imu_packet_->angular_voc[2] = imu_params_get_->imu_result_stu_.z_angle;

          decoder_ptr_->imu_packet_->linear_acce[0] = imu_params_get_->imu_result_stu_.x_acc;
          decoder_ptr_->imu_packet_->linear_acce[1] = imu_params_get_->imu_result_stu_.y_acc;
          decoder_ptr_->imu_packet_->linear_acce[2] = imu_params_get_->imu_result_stu_.z_acc;

          decoder_ptr_->imu_packet_->orientation[0] = imu_params_get_->imu_result_stu_.q0;
          decoder_ptr_->imu_packet_->orientation[1] = imu_params_get_->imu_result_stu_.q1;
          decoder_ptr_->imu_packet_->orientation[2] = imu_params_get_->imu_result_stu_.q2;
          decoder_ptr_->imu_packet_->orientation[3] = imu_params_get_->imu_result_stu_.q3;

          decoder_ptr_->cb_imu_pkt_();
        }
        pre_imu_timestamp_ = imu_timestamp;
      }
    }
  }

  void decoderDataBlock(T_DataBlock* data_block_ptr, DataBlockAngleAndTimestampInfo& data_block_angle_and_timestamp_info,
                        std::function<void(T_DataUnit*, PointInfo&)> decoder_data_unit_callback,
                        std::function<void()> point_cloud_algorithm_callbck) {
    uint16_t data_unit_num = data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ *
                             data_block_angle_and_timestamp_info.data_block_info_.col_channel_num_in_data_block_;
    if (data_block_angle_and_timestamp_info.point_info_vector_.size() == 0)
      return;
    for (uint16_t i = 0; i < data_unit_num; i++) {
      T_DataUnit* data_unit_ptr = reinterpret_cast<T_DataUnit*>(reinterpret_cast<char*>(data_block_ptr) + 5 + i * sizeof(T_DataUnit));
      decoder_data_unit_callback(data_unit_ptr, data_block_angle_and_timestamp_info.point_info_vector_[i]);

      typename T_PointCloud::PointT point =
          getPoint(data_block_angle_and_timestamp_info.point_info_vector_[i],
                   data_block_angle_and_timestamp_info.data_block_info_.distance_resolution_, point_cloud_algorithm_callbck);
      if (decoder_ptr_->param_.point_cloud_enable)
        decoder_ptr_->point_cloud_->points.emplace_back(point);
    }
  }

  typename T_PointCloud::PointT getPoint(PointInfo& point_info, uint8_t distance_resolution, std::function<void()> point_cloud_algorithm_callbck) {
    typename T_PointCloud::PointT point;
    double distance = point_info.distance_ * distance_resolution * 1e-3;
    uint32_t azimuth = (uint32_t)(point_info.azimuth_ * 1e3 + 360000) % 360000;
    uint32_t elevation = (uint32_t)(point_info.elevation_ * 1e3 + 360000) % 360000;
    uint8_t reflectivity = point_info.reflectivity_;
    uint16_t ring = point_info.ring_;
    double timestamp = point_info.timestamp_;

    if (point_cloud_algorithm_callbck == nullptr) {
      if (decoder_ptr_->param_.start_angle < decoder_ptr_->param_.end_angle &&
          (azimuth < decoder_ptr_->param_.start_angle * 1000 || azimuth > decoder_ptr_->param_.end_angle * 1000)) {
        distance = 0;
      } else if (decoder_ptr_->param_.start_angle > decoder_ptr_->param_.end_angle &&
                 (azimuth > decoder_ptr_->param_.end_angle * 1000 && azimuth < decoder_ptr_->param_.start_angle * 1000)) {
        distance = 0;
      }

      if (decoder_ptr_->hide_range_params_.size() > 0 && distance != 0 &&
          decoder_ptr_->isValueInRange(ring, azimuth / 1000.0, distance, decoder_ptr_->hide_range_params_)) {
        distance = 0;
      }
    }

    if (decoder_ptr_->param_.point_cloud_enable) {
      if (!decoder_ptr_->distance_section_.in(distance)) {
        setX(point, 0);
        setY(point, 0);
        setZ(point, 0);
        setIntensity(point, 0);
      } else {
        float dxy = distance * decoder_ptr_->trigon_.cos(elevation);
        float x = dxy * decoder_ptr_->trigon_.cos(azimuth);
        float y = dxy * decoder_ptr_->trigon_.sin(azimuth);
        float z = distance * decoder_ptr_->trigon_.sin(elevation);
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, reflectivity);
      }
      setTimestamp(point, timestamp);
      setRing(point, ring);
#ifdef ENABLE_GTEST
      setPointId(point, point_info.id_);
      setHorAngle(point, point_info.azimuth_);
      setVerAngle(point, point_info.elevation_);
      setDistance(point, distance);
#endif
    }

    if (decoder_ptr_->param_.laser_scan_enable && elevation == 0) {
      if (decoder_ptr_->distance_section_.in(distance)) {
        decoder_ptr_->scan_data_->ranges.emplace_back(distance);
        decoder_ptr_->scan_data_->intensities.emplace_back(reflectivity);
      } else {
        if (!decoder_ptr_->param_.dense_points) {
          decoder_ptr_->scan_data_->ranges.emplace_back(NAN);
          decoder_ptr_->scan_data_->intensities.emplace_back(NAN);
        } else {
          decoder_ptr_->scan_data_->ranges.emplace_back(0);
          decoder_ptr_->scan_data_->intensities.emplace_back(0);
        }
      }
    }

    return point;
  }

  void transformPointCloud() {
    for (int i = 0; i < decoder_ptr_->point_cloud_->points.size(); i++) {
      if (decoder_ptr_->point_cloud_->points[i].x == 0 && decoder_ptr_->point_cloud_->points[i].y == 0 &&
          decoder_ptr_->point_cloud_->points[i].z == 0) {
        if (!decoder_ptr_->param_.dense_points) {
          decoder_ptr_->point_cloud_->points[i].x = NAN;
          decoder_ptr_->point_cloud_->points[i].y = NAN;
          decoder_ptr_->point_cloud_->points[i].z = NAN;
        }
        continue;
      }

      decoder_ptr_->transformPoint(decoder_ptr_->point_cloud_->points[i].x, decoder_ptr_->point_cloud_->points[i].y,
                                   decoder_ptr_->point_cloud_->points[i].z);
    }
  }
};
}  // namespace lidar
}  // namespace vanjee
