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
#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/decoder/decoder_packet_base/decoder_packet_base.hpp>
#include <vanjee_driver/driver/decoder/wlr719e/protocol/frames/cmd_repository_719e.hpp>
#include <vanjee_driver/driver/decoder/wlr719e/protocol/frames/protocol_protocol_version_set.hpp>
#include <vanjee_driver/driver/decoder/wlr719e/protocol/frames/protocol_scan_data_get.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {

#pragma pack(push, 1)

typedef struct _Vanjee719EDataUnit {
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t Confidence;
} Vanjee719EDataUnit;

typedef struct _Vanjee719EDataBlock {
  uint8_t start_flag;
  int16_t azimuth;
  uint8_t remain3[2];
  Vanjee719EDataUnit channel[4];
} Vanjee719EDataBlock;

#pragma pack(pop)

template <typename T_PointCloud>
class DecoderVanjee719E : public DecoderMech<T_PointCloud> {
 private:
  std::vector<double> all_points_luminous_moment_719e;         // Cache 4 channels with a circular
                                                               // point cloud time difference
  const double luminous_period_of_ld_ = 0.000027777;           // Time interval at adjacent horizontal angles
  const double luminous_period_of_adjacent_ld_ = 0.000006944;  // Time interval between adjacent vertical angles within the
                                                               // group

  int32_t azimuth_cur_ = -1.0;
  int32_t pre_frame_id_ = -1;
  uint8_t pre_bank_id_ = 0;
  uint32_t point_id_offset_ = 0;
  int32_t pre_point_id_offset_ = -1;

  std::vector<int32_t> lidar_ver_angle_ = std::vector<int32_t>{355500, 0, 4500, 9000};

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam &getConstParam(uint8_t mode);
  ChanAngles chan_angles_;

  uint16_t pre_operate_frequency_ = 0;
  void initLdLuminousMoment(uint16_t operate_frequency, uint32_t point_offset);

  DecoderPacketBase<T_PointCloud, Vanjee719EDataBlock, Vanjee719EDataUnit> *decoder_packet_base_ = nullptr;
  void updateAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo &data_block_angle_and_timestamp_info, Vanjee719EDataBlock *data_block);
  void decoderDataUnitCallback(Vanjee719EDataUnit *data_unit, PointInfo &point_info);

 public:
  constexpr static double FRAME_DURATION = 0.05;
  constexpr static uint32_t SINGLE_PKT_NUM = 28;
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee719E() {  // = default;
    delete decoder_packet_base_;
  }
  explicit DecoderVanjee719E(const WJDecoderParam &param);
  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  bool decodeMsopPktBaseProtocol(const uint8_t *pkt, size_t size);
};

template <typename T_PointCloud>
inline WJDecoderMechConstParam &DecoderVanjee719E<T_PointCloud>::getConstParam(uint8_t mode) {
  // WJDecoderConstParam
  // WJDecoderMechConstParam
  static WJDecoderMechConstParam param = {
      1384  // msop len
      ,
      4  // laser number
      ,
      450  // blocks per packet
      ,
      4  // channels per block
      ,
      0.2f  // distance min
      ,
      50.0f  // distance max
      ,
      0.001f  // distance resolution
      ,
      80.0f  // initial value of temperature
  };
  param.BLOCK_DURATION = 0.2 / 360;
  return param;
}

template <typename T_PointCloud>
inline DecoderVanjee719E<T_PointCloud>::DecoderVanjee719E(const WJDecoderParam &param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param), chan_angles_(this->const_param_.LASER_NUM) {
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);
  decoder_packet_base_ = new DecoderPacketBase<T_PointCloud, Vanjee719EDataBlock, Vanjee719EDataUnit>(this);
}

template <typename T_PointCloud>
void DecoderVanjee719E<T_PointCloud>::initLdLuminousMoment(uint16_t operate_frequency, uint32_t point_offset) {
  if (operate_frequency != pre_operate_frequency_ || point_offset != pre_point_id_offset_) {
    uint16_t col_num = 1800 - (point_offset / 2);
    uint32_t point_num = col_num * 4;
    double offset = 0;
    all_points_luminous_moment_719e.resize(point_num);  // 7200

    for (uint16_t col = 0; col < col_num; col++) {  // 1800
      for (uint8_t row = 0; row < 4; row++) {
        offset = row * luminous_period_of_adjacent_ld_;
        all_points_luminous_moment_719e[col * 4 + row] = col * luminous_period_of_ld_ + offset;
      }
    }
    decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_ = all_points_luminous_moment_719e;
    pre_operate_frequency_ = operate_frequency;
    pre_point_id_offset_ = point_offset;
  }

  if (this->param_.laser_scan_enable) {
    double angle_offset = point_offset / 4 * 0.2;
    this->scan_data_->angle_min = 180 - angle_offset;
    this->scan_data_->angle_max = -180 + angle_offset;
    this->scan_data_->angle_increment = -360.0 / 1800;
    this->scan_data_->scan_time = 1.0 / operate_frequency;
    if (this->param_.ts_first_point)
      this->scan_data_->time_increment = this->scan_data_->scan_time / 1800;
    else
      this->scan_data_->time_increment = -this->scan_data_->scan_time / 1800;
    this->scan_data_->range_min = this->param_.min_distance;
    this->scan_data_->range_max = this->param_.max_distance;
  }
}

template <typename T_PointCloud>
inline bool DecoderVanjee719E<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size) {
  bool ret = false;
  uint16_t l_pktheader = pkt[0] << 8 | pkt[1];
  switch (l_pktheader) {
    case 0xFFCC: {
      if (size == 1400) {
        ret = decodeMsopPktBaseProtocol(pkt, size);
      }
    } break;

    default:
      break;
  }

  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee719E<T_PointCloud>::updateAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo &data_block_angle_and_timestamp_info,
                                                                          Vanjee719EDataBlock *data_block) {
  int32_t hor_angle = ((data_block->azimuth * 10) + 360000) % 360000;
  uint16_t hor_resolution =
      (uint16_t)(data_block_angle_and_timestamp_info.reserved_field_1_[0] + (data_block_angle_and_timestamp_info.reserved_field_1_[1] << 8));
  int32_t col_index = hor_angle / hor_resolution;

  if (point_id_offset_ != pre_point_id_offset_) {
    uint32_t point_num =
        ((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ * 4) +
        (data_block_angle_and_timestamp_info.data_block_index_in_packet_ * 4);
    uint32_t point_id = col_index * data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_;
    point_id_offset_ = point_id - point_num;
  }

  initLdLuminousMoment((uint16_t)(data_block_angle_and_timestamp_info.operate_frequency_ * 0.01), point_id_offset_);

  if (data_block_angle_and_timestamp_info.data_block_index_in_packet_ ==
      data_block_angle_and_timestamp_info.data_block_info_.valid_data_block_num_in_packet_ - 1) {
    uint32_t point_id = (col_index + 1) * data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ - 1 - point_id_offset_;
    if (!this->param_.use_lidar_clock) {
      data_block_angle_and_timestamp_info.first_point_ts_ -=
          decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id];
    } else {
      data_block_angle_and_timestamp_info.first_point_ts_ -=
          ((25567LL * 24 * 3600) + decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id]);
      if (data_block_angle_and_timestamp_info.first_point_ts_ < 0) {
        data_block_angle_and_timestamp_info.first_point_ts_ = 0;
      }
    }
  }

  if (data_block_angle_and_timestamp_info.data_block_info_.data_block_packet_type_ == 1) {
    for (int chan_id = 0; chan_id < data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_; chan_id++) {
      int32_t row_index = chan_id;
      uint32_t point_id =
          col_index * data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ + row_index - point_id_offset_;
      double timestamp_point = 0.0;
      if (this->param_.ts_first_point)
        timestamp_point = decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id];
      else
        timestamp_point =
            decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id] -
            decoder_packet_base_->decoder_packet_general_version_base_ptr_
                ->all_points_luminous_moment_[decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_.size() - 1];
      PointInfo point_info;
      // point_info.distance_ = 0;
      point_info.azimuth_ = ((540000 - hor_angle) % 360000) * 1e-3;
      point_info.elevation_ = lidar_ver_angle_[row_index] * 1e-3;
      // point_info.reflectivity_ = 0;
      point_info.ring_ = chan_id + this->first_line_id_;
      point_info.timestamp_ = timestamp_point;
      point_info.id_ = point_id;
      data_block_angle_and_timestamp_info.point_info_vector_.emplace_back(point_info);
    }
  }
}

template <typename T_PointCloud>
void DecoderVanjee719E<T_PointCloud>::decoderDataUnitCallback(Vanjee719EDataUnit *data_unit, PointInfo &point_info) {
  point_info.distance_ = data_unit->distance;
  // point_info.azimuth_ = point_info.azimuth_;
  // point_info.elevation_ = point_info.elevation_;
  point_info.reflectivity_ = data_unit->reflectivity;
  // point_info.ring_ = point_info.ring_;
  // point_info.timestamp_ = point_info.timestamp_;
  // point_info.id_ = point_info.id_;
}

template <typename T_PointCloud>
inline bool DecoderVanjee719E<T_PointCloud>::decodeMsopPktBaseProtocol(const uint8_t *pkt, size_t size) {
  return decoder_packet_base_->decoderPacket(
      pkt, size, std::bind(&DecoderVanjee719E::updateAngleAndTimestampInfoCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DecoderVanjee719E::decoderDataUnitCallback, this, std::placeholders::_1, std::placeholders::_2));
}

template <typename T_PointCloud>
void DecoderVanjee719E<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<ProtocolAbstract719E> p;
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

  if (*sp_cmd == *(CmdRepository719E::CreateInstance()->Sp_ScanDataGet)) {
    p = std::make_shared<Protocol_ScanDataGet719E>();
  } else {
    return;
  }
  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Params_ScanData719E)) {
    std::shared_ptr<Params_ScanData719E> param = std::dynamic_pointer_cast<Params_ScanData719E>(params);
    if (param->data_get_flag) {
      // (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      Decoder<T_PointCloud>::angles_ready_ = true;
      WJ_INFOL << "get wlr719e scan data succ" << WJ_REND;
    } else {
      WJ_WARNING << "get wlr719e scan data fail" << WJ_REND;
    }

  } else {
    WJ_WARNING << "Unknown Params Type..." << WJ_REND;
  }
}

}  // namespace lidar
}  // namespace vanjee