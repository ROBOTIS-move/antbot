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
#include "vanjee_driver/driver/decoder/decoder_packet_base/decoder_packet_general_version_base.hpp"
using namespace std;

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)

typedef struct _CheckInfoV1_1 {
  uint32_t check_[2];
} CheckInfoV1_1;

#pragma pack(pop)
template <typename T_PointCloud, typename T_DataBlock, typename T_DataUnit>
class DecoderPacketGeneralVersionV1_1 : public DecoderPacketGeneralVersionBase<T_PointCloud, T_DataBlock, T_DataUnit> {
 public:
  DecoderPacketGeneralVersionV1_1(Decoder<T_PointCloud>* decoder_ptr)
      : DecoderPacketGeneralVersionBase<T_PointCloud, T_DataBlock, T_DataUnit>(decoder_ptr) {
  }

  virtual bool decoderPacket(const uint8_t* buf, size_t size,
                             std::function<void(DataBlockAngleAndTimestampInfo&, T_DataBlock*)> update_angle_and_timestamp_info_callback,
                             std::function<void(T_DataUnit*, PointInfo&)> decoder_data_unit_callback,
                             std::function<void()> point_cloud_algorithm_callbck) {
    VanjeeLidarPointCloudPacketBaseHeader& vanjee_lidar_point_cloud_packet_base_header = *(VanjeeLidarPointCloudPacketBaseHeader*)buf;
    return decoderPointCloudPacketProtocolVersionV1_1(buf, size, update_angle_and_timestamp_info_callback, decoder_data_unit_callback,
                                                      point_cloud_algorithm_callbck);
  }

  bool decoderPointCloudPacketProtocolVersionV1_1(
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
    if (!this->decoder_ptr_->param_.use_lidar_clock) {
      pkt_ts = host_pkt_ts;
    } else {
      pkt_ts = lidar_pkt_ts;
      if (pkt_ts < 3600) {
        pkt_ts == 3600;
      }
    }

    functionalSafetyDecoder(buf, size, vanjee_lidar_point_cloud_packet_header);
    if (this->decoder_ptr_->param_.imu_enable != -1) {
      sendImuData(buf, size, pkt_ts, lidar_pkt_ts, vanjee_lidar_point_cloud_packet_header);
    }

    uint32_t loss_circles_num = (vanjee_lidar_point_cloud_packet_header.circle_id_ + 0x10000 - this->pre_circle_id_) % 0x10000;
    this->pre_circle_id_ = vanjee_lidar_point_cloud_packet_header.circle_id_;
    if (loss_circles_num > 1)
      this->decoder_ptr_->point_cloud_->points.clear();

    if ((loss_circles_num == 1 && this->decoder_ptr_->point_cloud_->points.size() != 0)) {
      if (this->decoder_ptr_->param_.point_cloud_enable) {
        this->decoder_ptr_->first_point_ts_ = this->first_point_ts_;
        this->decoder_ptr_->last_point_ts_ =
            this->decoder_ptr_->first_point_ts_ + this->all_points_luminous_moment_[this->all_points_luminous_moment_.size() - 1];
        if (point_cloud_algorithm_callbck != nullptr)
          point_cloud_algorithm_callbck();
        this->transformPointCloud();
        this->decoder_ptr_->cb_split_frame_(vanjee_lidar_point_cloud_packet_header.row_channel_num_, this->decoder_ptr_->cloudTs());
        ret = true;
      }

      if (this->decoder_ptr_->param_.laser_scan_enable) {
        this->decoder_ptr_->scan_data_->ranges.clear();
        this->decoder_ptr_->scan_data_->intensities.clear();
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
      this->decoderDataBlock(data_block_ptr, data_block_angle_and_timestamp_info, decoder_data_unit_callback, point_cloud_algorithm_callbck);

      offset += sizeof(T_DataBlock);
    }
    this->first_point_ts_ = data_block_angle_and_timestamp_info.first_point_ts_;
    if (vanjee_lidar_point_cloud_packet_header.pkt_id_ == vanjee_lidar_point_cloud_packet_header.total_pkt_num_) {
      this->decoder_ptr_->first_point_ts_ = this->first_point_ts_;
      this->decoder_ptr_->last_point_ts_ =
          this->decoder_ptr_->first_point_ts_ + this->all_points_luminous_moment_[this->all_points_luminous_moment_.size() - 1];

      if (this->decoder_ptr_->param_.point_cloud_enable) {
        if (point_cloud_algorithm_callbck != nullptr)
          point_cloud_algorithm_callbck();
        this->transformPointCloud();
        this->decoder_ptr_->cb_split_frame_(vanjee_lidar_point_cloud_packet_header.row_channel_num_, this->decoder_ptr_->cloudTs());
        ret = true;
      }

      if (this->decoder_ptr_->param_.laser_scan_enable) {
        if (this->decoder_ptr_->scan_data_->ranges.size() == vanjee_lidar_point_cloud_packet_header.col_channel_num_) {
          this->decoder_ptr_->cb_scan_data_(this->decoder_ptr_->cloudTs());
          ret = true;
        }
        this->decoder_ptr_->scan_data_->ranges.clear();
        this->decoder_ptr_->scan_data_->intensities.clear();
      }
    }

    return ret;
  }

  bool protocolCheck(const uint8_t* buf, uint16_t size, VanjeeLidarPointCloudPacketHeaderV1_0& vanjee_lidar_point_cloud_packet_header) {
    bool ret = true;
    // 0：exclude cyber security info field；1：include cyber security info field,but data is invalidate;
    // 2:include cyber security info field ,and data is valid;
    uint8_t cyber_security_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 4) & 0x03;
    // 0：exclude check info field；1：include check info field,but data is invalidate;
    // 2：crc16;3：bcc,4：crc32,5：crc64;
    uint8_t check_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 6) & 0x0f;

    TailInfo& tailInfo = *(TailInfo*)(buf + size - 2);
    if (tailInfo.tail_[0] != 0xee || tailInfo.tail_[1] != 0xee) {
      WJ_ERROR << "Tail: Frame tail err" << WJ_REND;
      ret = false;
    }

    if (check_flag == 2) {
      CheckInfoV1_1& checkInfo = *(CheckInfoV1_1*)(buf + size - 10);
      if (this->decoder_ptr_->checkCRC16(buf, 2, size - 10) != (uint16_t)checkInfo.check_[0]) {
        WJ_ERROR << "Data check: Check err" << WJ_REND;
        ret = false;
      }
    } else if (check_flag == 3) {
      CheckInfoV1_1& checkInfo = *(CheckInfoV1_1*)(buf + size - 10);
      if (this->decoder_ptr_->checkBCC(buf, 2, size - 10) != (uint8_t)checkInfo.check_[0]) {
        WJ_ERROR << "Data check: Check err" << WJ_REND;
        ret = false;
      }
    } else if (check_flag == 4) {
      CheckInfoV1_1& checkInfo = *(CheckInfoV1_1*)(buf + size - 10);
      if (this->decoder_ptr_->checkCRC32(buf, 2, size - 10) != checkInfo.check_[0]) {
        WJ_ERROR << "Data check: Check err" << WJ_REND;
        ret = false;
      }
    }
    return ret;
  }

  void functionalSafetyDecoder(const uint8_t* buf, uint16_t size, VanjeeLidarPointCloudPacketHeaderV1_0& vanjee_lidar_point_cloud_packet_header) {
    // 0：exclude functional safety info field；1：include functional safety info field,but data is invalidate;
    // 2:include functional safety info field ,and data is valid;
    uint8_t function_safety_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 2) & 0x03;
    if (function_safety_flag == 2) {
      // 0：exclude cyber security info field；1：include cyber security info field,but data is invalidate;
      // 2:include cyber security info field ,and data is valid;
      uint8_t cyber_security_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 4) & 0x03;
      // 0：exclude check info field；1：include check info field,but data is invalidate;
      // 2：crc16;3：bcc,4：crc32,5：crc64;
      uint8_t check_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 6) & 0x0f;
      uint16_t offset = sizeof(TailInfo) + 1;
      if (check_flag != 0)
        offset += sizeof(CheckInfoV1_1);
      if (cyber_security_flag != 0)
        offset += sizeof(CyberSecurityInfo);
      offset += sizeof(FunctionSafetyInfo);
      FunctionSafetyInfo& functionSafetyInfo = *(FunctionSafetyInfo*)(buf + size - offset);
      if (this->fault_detection_module_counter_ != 0 && functionSafetyInfo.fault_detection_module_counter_ == this->fault_detection_module_counter_) {
        WJ_ERROR << "Functional safety: fault detection module error" << WJ_REND;
        return;
      }

      std::string err_code_str = "";
      uint8_t lidar_stat = functionSafetyInfo.lidar_running_state_ & 0x0f;
      if (lidar_stat != 0) {
        uint8_t fault_id = ((functionSafetyInfo.lidar_running_state_ >> 4) & 0x0f);
        err_code_str += "fault id: " + std::to_string((uint16_t)(fault_id + 1)) + ", ";
        err_code_str += "error code: " + std::to_string(functionSafetyInfo.error_code_id_);
        err_code_str += "error name: [ ";
        for (int i = 0; i < 16; i++) {
          if (((functionSafetyInfo.error_code_id_ >> i) & 0x01) == 1) {
            err_code_str += this->decoder_ptr_->error_code_[fault_id][i] + " ";
          }
        }
        err_code_str += "]";
      }

      if (err_code_str.length() > 1) {
        if (lidar_stat == 1)
          WJ_WARNING << "Functional safety [WARNNING]: " << err_code_str << WJ_REND;
        else
          WJ_ERROR << "Functional safety [ERROR]: " << err_code_str << WJ_REND;
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
      // 0：exclude check info field；1：include check info field,but data is invalidate;
      // 2：crc16;3：bcc,4：crc32,5：crc64
      uint8_t check_flag = (vanjee_lidar_point_cloud_packet_header.info_flag_ >> 6) & 0x0f;
      uint16_t offset = sizeof(TailInfo);
      if (check_flag != 0)
        offset += sizeof(CheckInfoV1_1);
      if (cyber_security_flag != 0)
        offset += sizeof(CyberSecurityInfo);
      if (function_safety_flag != 0)
        offset += sizeof(FunctionSafetyInfo);
      offset += sizeof(ImuDataInfo);

      ImuDataInfo& imuDataInfo = *(ImuDataInfo*)(buf + size - offset);
      double imu_timestamp = lidar_timestamp + (double)imuDataInfo.imu_data_timestamp_ * 1e-9;
      if (abs(imu_timestamp - this->pre_imu_timestamp_) > 1e-6) {
        bool ret = this->imu_params_get_->imuGet(imuDataInfo.imu_angle_voc_x_, imuDataInfo.imu_angle_voc_y_, imuDataInfo.imu_angle_voc_z_,
                                                 imuDataInfo.imu_linear_acce_x_, imuDataInfo.imu_linear_acce_y_, imuDataInfo.imu_linear_acce_z_,
                                                 imu_timestamp, -100.0, false, false, false, false, true);
        if (ret) {
          this->decoder_ptr_->imu_packet_->timestamp = timestamp + (double)imuDataInfo.imu_data_timestamp_ * 1e-9;
          this->decoder_ptr_->imu_packet_->angular_voc[0] = this->imu_params_get_->imu_result_stu_.x_angle;
          this->decoder_ptr_->imu_packet_->angular_voc[1] = this->imu_params_get_->imu_result_stu_.y_angle;
          this->decoder_ptr_->imu_packet_->angular_voc[2] = this->imu_params_get_->imu_result_stu_.z_angle;

          this->decoder_ptr_->imu_packet_->linear_acce[0] = this->imu_params_get_->imu_result_stu_.x_acc;
          this->decoder_ptr_->imu_packet_->linear_acce[1] = this->imu_params_get_->imu_result_stu_.y_acc;
          this->decoder_ptr_->imu_packet_->linear_acce[2] = this->imu_params_get_->imu_result_stu_.z_acc;

          this->decoder_ptr_->imu_packet_->orientation[0] = this->imu_params_get_->imu_result_stu_.q0;
          this->decoder_ptr_->imu_packet_->orientation[1] = this->imu_params_get_->imu_result_stu_.q1;
          this->decoder_ptr_->imu_packet_->orientation[2] = this->imu_params_get_->imu_result_stu_.q2;
          this->decoder_ptr_->imu_packet_->orientation[3] = this->imu_params_get_->imu_result_stu_.q3;

          this->decoder_ptr_->cb_imu_pkt_();
        }
        this->pre_imu_timestamp_ = imu_timestamp;
      }
    }
  }
};
}  // namespace lidar
}  // namespace vanjee
