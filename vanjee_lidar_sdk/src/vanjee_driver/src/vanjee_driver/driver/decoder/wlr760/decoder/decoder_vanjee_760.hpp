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
#include <vanjee_driver/driver/decoder/wlr760/algorithm/filterate.hpp>
#include <vanjee_driver/driver/decoder/wlr760/decoder/one_point_info_manage.hpp>
#include <vanjee_driver/driver/decoder/wlr760/protocol/frames/cmd_repository_760.hpp>
#include <vanjee_driver/driver/decoder/wlr760/protocol/frames/protocol_protocol_version_set.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)

typedef struct _Vanjee760ChannelData {
  uint16_t distance;
  uint8_t reflectivity;
} Vanjee760ChannelData;

typedef struct _Vanjee760Block {
  uint8_t data_header;
  uint16_t azimuth;
  uint8_t rotate_mirror_id;
  int16_t ver_offset_angle;
  int16_t hor_offset_angle;
  Vanjee760ChannelData channel[192];
} Vanjee760Block;

typedef struct _Vanjee760DifopPkt {
  uint8_t param_dentify_code[4];
  uint16_t circle_id;
  uint16_t pkt_id;
  uint8_t device_type;
  uint8_t return_wave_num;
  uint16_t rpm;
  uint32_t second;
  uint32_t microsecond;
  uint16_t frame_len;
  uint8_t remain[4];
  uint16_t tail;
} Vanjee760DifopPkt;

typedef struct _Vanjee760MsopPkt {
  uint16_t header;
  Vanjee760Block blocks[2];
  Vanjee760DifopPkt difop;
} Vanjee760MsopPkt;

typedef struct _Vanjee760ChannelDataPulseWidth {
  uint16_t distance;
  uint8_t reflectivity;
  uint16_t low_thres_pulse_width;
  uint16_t high_thres_pulse_width;
} Vanjee760ChannelDataPulseWidth;

typedef struct _Vanjee760BlockPulseWidth {
  uint16_t azimuth;
  uint8_t rotate_mirror_id;
  int16_t ver_offset_angle;
  int16_t hor_offset_angle;
  Vanjee760ChannelDataPulseWidth channel[192];
} Vanjee760BlockPulseWidth;

typedef struct _Vanjee760MsopPktPulseWidth {
  uint16_t header;
  Vanjee760BlockPulseWidth block;
  Vanjee760DifopPkt difop;
} Vanjee760MsopPktPulseWidth;

typedef struct _Vanjee760DataUnit {
  uint16_t distance;
  uint8_t reflectivity;
} Vanjee760DataUnit;

typedef struct _Vanjee760DataBlock {
  uint8_t start_flag;
  uint16_t azimuth_offset;
  uint16_t elevation_offset;
  Vanjee760DataUnit channel[192];
} Vanjee760DataBlock;

typedef struct _Vanjee760DataBlockV1_1 {
  uint8_t start_flag;
  uint16_t azimuth_offset;
  uint16_t elevation_offset;
  Vanjee760DataUnit channel[192];
  uint8_t confidence[72];
} Vanjee760DataBlockV1_1;

#pragma pack(pop)

template <typename T_PointCloud>
class DecoderVanjee760 : public DecoderMech<T_PointCloud> {
 private:
  bool init_flag = false;
  std::vector<double> all_points_luminous_moment_760_120degree_;
  std::vector<double> all_points_luminous_moment_760_140degree_;
  const double luminous_period_of_ld_ = 0.0000925;
  bool algorithm_enable_;

  int32_t pre_frame_id_ = -1;
  uint8_t publish_mode_ = 0;
  uint16_t pre_operate_frequency_ = 0;
  uint32_t one_point_info_tmp_index_ = 0;
  uint16_t hor_resolution_ = 15;
  int32_t max_hor_angle_ = 14040;

  std::vector<int32_t> ver_angle_type1_ = std::vector<int32_t>{
      12500,  12369,  12238,  12107,  11976,  11846,  11715,  11584,  11453,  11322,  11191,  11060,  10929,  10798,  10668,  10537,  10406,  10275,
      10144,  10013,  9882,   9751,   9620,   9490,   9359,   9228,   9097,   8966,   8835,   8704,   8573,   8442,   8312,   8181,   8050,   7919,
      7788,   7657,   7526,   7395,   7264,   7134,   7003,   6872,   6741,   6610,   6479,   6348,   6217,   6086,   5955,   5825,   5694,   5563,
      5432,   5301,   5170,   5039,   4908,   4777,   4647,   4516,   4385,   4254,   4123,   3992,   3861,   3730,   3599,   3469,   3338,   3207,
      3076,   2945,   2814,   2683,   2552,   2421,   2291,   2160,   2029,   1898,   1767,   1636,   1505,   1374,   1243,   1113,   982,    851,
      720,    589,    458,    327,    196,    65,     -65,    -196,   -327,   -458,   -589,   -720,   -851,   -982,   -1113,  -1243,  -1374,  -1505,
      -1636,  -1767,  -1898,  -2029,  -2160,  -2291,  -2421,  -2552,  -2683,  -2814,  -2945,  -3076,  -3207,  -3338,  -3469,  -3599,  -3730,  -3861,
      -3992,  -4123,  -4254,  -4385,  -4516,  -4647,  -4777,  -4908,  -5039,  -5170,  -5301,  -5432,  -5563,  -5694,  -5825,  -5955,  -6086,  -6217,
      -6348,  -6479,  -6610,  -6741,  -6872,  -7003,  -7134,  -7264,  -7395,  -7526,  -7657,  -7788,  -7919,  -8050,  -8181,  -8312,  -8442,  -8573,
      -8704,  -8835,  -8966,  -9097,  -9228,  -9359,  -9490,  -9620,  -9751,  -9882,  -10013, -10144, -10275, -10406, -10537, -10668, -10798, -10929,
      -11060, -11191, -11322, -11453, -11584, -11715, -11846, -11976, -12107, -12238, -12369, -12500};

  std::vector<int32_t> ver_angle_type2_ = std::vector<int32_t>{
      12500, 12237, 11974,  11711,  11447,  11184,  10921,  10658,  10395,  10132,  9868,   9605,  9342,  9079,  8816,  8553,  8289,  8026,
      7763,  7500,  7237,   6974,   6711,   6447,   6250,   6184,   6118,   5987,   5921,   5855,  5724,  5658,  5592,  5461,  5395,  5329,
      5197,  5132,  5066,   4934,   4868,   4803,   4671,   4605,   4539,   4408,   4342,   4276,  4145,  4079,  4013,  3882,  3816,  3750,
      3618,  3553,  3487,   3355,   3289,   3224,   3092,   3026,   2961,   2829,   2763,   2697,  2566,  2500,  2434,  2303,  2237,  2171,
      2039,  1974,  1908,   1776,   1711,   1645,   1513,   1447,   1382,   1250,   1184,   1118,  987,   921,   855,   724,   658,   592,
      461,   395,   329,    197,    132,    66,     -66,    -132,   -197,   -329,   -395,   -461,  -592,  -658,  -724,  -855,  -921,  -987,
      -1118, -1184, -1250,  -1382,  -1447,  -1513,  -1645,  -1711,  -1776,  -1908,  -1974,  -2039, -2171, -2237, -2303, -2434, -2500, -2566,
      -2697, -2763, -2829,  -2961,  -3026,  -3092,  -3224,  -3289,  -3355,  -3487,  -3553,  -3618, -3750, -3816, -3882, -4013, -4079, -4145,
      -4276, -4342, -4408,  -4539,  -4605,  -4671,  -4803,  -4868,  -4934,  -5066,  -5132,  -5197, -5329, -5395, -5461, -5592, -5658, -5724,
      -5855, -5921, -5987,  -6118,  -6184,  -6250,  -6447,  -6711,  -6974,  -7237,  -7500,  -7763, -8026, -8289, -8553, -8816, -9079, -9342,
      -9605, -9868, -10132, -10395, -10658, -10921, -11184, -11447, -11711, -11974, -12237, -12500};

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam& getConstParam(uint8_t mode);
  void initLdLuminousMoment(void);
  int32_t getTimeSequenceOffsetHorAngle(float speed, int line);
  OnePointInfoManage760* m_qmap_one_circle_point_ = new OnePointInfoManage760();

  DecoderPacketBase<T_PointCloud, Vanjee760DataBlock, Vanjee760DataUnit>* decoder_packet_base_ = nullptr;
  DecoderPacketBase<T_PointCloud, Vanjee760DataBlockV1_1, Vanjee760DataUnit>* decoder_packet_v1_1_ = nullptr;

  void updateAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo& data_block_angle_and_timestamp_info, Vanjee760DataBlock* data_block);
  void updateAngleAndTimestampInfoCallbackV1_1(DataBlockAngleAndTimestampInfo& data_block_angle_and_timestamp_info,
                                               Vanjee760DataBlockV1_1* data_block);
  void decoderDataUnitCallback(Vanjee760DataUnit* data_unit, PointInfo& point_info);
  void pointCloudAlgorithmCallbck();

  void decoderPacketBaseInit(uint16_t protocol_version);
  bool decodeMsopPktBaseProtocol(const uint8_t* pkt, size_t size);
  bool decodeMsopPktProtocolV1_1(const uint8_t* pkt, size_t size);

 public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 400;
  uint32 getOnePointInfoKey(const OnePointInfo760& val, uint16_t hor_resolution);
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee760() {  // = default;
    delete decoder_packet_base_;
    delete decoder_packet_v1_1_;
  }
  explicit DecoderVanjee760(const WJDecoderParam& param);
};

template <typename T_PointCloud>
void DecoderVanjee760<T_PointCloud>::initLdLuminousMoment() {
  double point_ts_offset = luminous_period_of_ld_ / 192;
  double glowing_moments[192] = {0};

  for (int i = 0; i < 192; i++) {
    glowing_moments[i] = point_ts_offset * i;
  }

  all_points_luminous_moment_760_120degree_.resize(153600);
  all_points_luminous_moment_760_140degree_.resize(179712);
  for (uint16_t col = 0; col < 936; col++) {
    for (uint8_t row = 0; row < 192; row++) {
      if (col < 800) {
        all_points_luminous_moment_760_120degree_[col * 192 + row] = col * luminous_period_of_ld_ + glowing_moments[row];
      }
      all_points_luminous_moment_760_140degree_[col * 192 + row] = col * luminous_period_of_ld_ + glowing_moments[row];
    }
  }
}

template <typename T_PointCloud>
inline WJDecoderMechConstParam& DecoderVanjee760<T_PointCloud>::getConstParam(uint8_t mode) {
  static WJDecoderMechConstParam param = {
      1186  // msop len
      ,
      192  // laser number
      ,
      2  // blocks per packet
      ,
      192  // channels per block
      ,
      1.5f  // distance min
      ,
      260.0f  // distance max
      ,
      0.004f  // distance resolution
      ,
      80.0f  // initial value of temperature
  };
  param.BLOCK_DURATION = 0.1 / 800;
  return param;
}

template <typename T_PointCloud>
inline DecoderVanjee760<T_PointCloud>::DecoderVanjee760(const WJDecoderParam& param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param) {
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  publish_mode_ = param.publish_mode;
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);

  // Filterate760::GetInstance()->init(192);
  algorithm_enable_ = true;
  Filterate760::GetInstance()->setAlgorithmEnb(algorithm_enable_);
  Filterate760::GetInstance()->settripleThreeFilterEnb(true);
  Filterate760::GetInstance()->setReplenishPointEnb(true);
  Filterate760::GetInstance()->psCrosstalkFilterEnb(true);
  m_qmap_one_circle_point_->init(192);
  m_qmap_one_circle_point_->clear();

  initLdLuminousMoment();
}

template <typename T_PointCloud>
inline bool DecoderVanjee760<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size) {
  bool ret = false;
  uint16_t pkt_header = pkt[0] << 8 | pkt[1];
  uint16_t protocol_version = pkt[6] << 8 | pkt[7];
  decoderPacketBaseInit(protocol_version);
  switch (pkt_header) {
    case 0xFFCC: {
      switch (protocol_version) {
        case 0x0100:
          ret = decodeMsopPktBaseProtocol(pkt, size);
          break;

        case 0x0101:
          ret = decodeMsopPktProtocolV1_1(pkt, size);
          break;

        default:
          break;
      }
    } break;

    default:
      break;
  }
  return ret;
}

template <typename T_PointCloud>
inline void DecoderVanjee760<T_PointCloud>::decoderPacketBaseInit(uint16_t protocol_version) {
  if (!init_flag) {
    switch (protocol_version) {
      case 0x0100:
        decoder_packet_base_ = new DecoderPacketBase<T_PointCloud, Vanjee760DataBlock, Vanjee760DataUnit>(this);
        init_flag = true;
        break;

      case 0x0101:
        decoder_packet_v1_1_ = new DecoderPacketBase<T_PointCloud, Vanjee760DataBlockV1_1, Vanjee760DataUnit>(this);
        init_flag = true;
        break;

      default:
        break;
    }
  }
}

template <typename T_PointCloud>
inline int32_t DecoderVanjee760<T_PointCloud>::getTimeSequenceOffsetHorAngle(float speed, int line) {
  // 459*2
  // int group = 0;
  // if((line >= 1 && line <= 12) || (line >= 97 && line <= 132))
  // {
  //     group = 3;
  // }
  // else if((line >= 13 && line <= 24) || (line >= 133 && line <= 168))
  // {
  //     group = 2;
  // }
  // else if((line >= 25 && line <= 60) || (line >= 169 && line <= 180))
  // {
  //     group = 1;
  // }
  // else if((line >= 61 && line <= 96) || (line >= 181 && line <= 192))
  // {
  //     group = 0;
  // }
  // int32_t h_offset = (int32_t)(speed * 360.0 / 60000 * 27.767 * group);

  // 459*1
  int32_t h_offset = 0;
  if ((line >= 25 && line <= 48) || (line >= 169 && line <= 192)) {
    h_offset = speed * 126 * 1e-3;  // speed * 6 * 7 * 3 * 1e-3;   //18.9e-3
  } else if ((line >= 73 && line <= 96) || (line >= 121 && line <= 144)) {
    h_offset = speed * 252 * 1e-3;  // speed * 6 * 14 * 3 * 1e-3;    //37.8e-3
  } else if ((line >= 49 && line <= 72) || (line >= 97 && line <= 120)) {
    h_offset = speed * 216 * 1e-3;  // speed * 6 * 12 * 3 * 1e-3;    //32.4e-3
  } else if ((line >= 1 && line <= 24) || (line >= 145 && line <= 168)) {
    h_offset = speed * 342 * 1e-3;  // speed * 6 * 19 * 3 * 1e-3;    //51.3e-3
  }
  return h_offset;
}

template <typename T_PointCloud>
inline uint32 DecoderVanjee760<T_PointCloud>::getOnePointInfoKey(const OnePointInfo760& val, uint16_t hor_resolution) {
  if (val.echo_type == 1) {
    return ((val.line_id - 1) * MAX_COL_NUM_760) + (val.hor_azimuth / hor_resolution);
  } else if (val.echo_type == 2) {
    return ((val.line_id - 1) * MAX_COL_NUM_760) + (val.hor_azimuth / hor_resolution) + 192 * MAX_COL_NUM_760;
  } else {
    return 0;
  }
}

template <typename T_PointCloud>
void DecoderVanjee760<T_PointCloud>::updateAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo& data_block_angle_and_timestamp_info,
                                                                         Vanjee760DataBlock* data_block) {
  if (data_block_angle_and_timestamp_info.operate_frequency_ / 100 != pre_operate_frequency_) {
    if (data_block_angle_and_timestamp_info.total_pkt_num_ * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ /
            data_block_angle_and_timestamp_info.echo_num_ ==
        800) {
      decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_ = all_points_luminous_moment_760_120degree_;
    } else {
      decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_ = all_points_luminous_moment_760_140degree_;
    }
  }

  if (data_block_angle_and_timestamp_info.data_block_index_in_packet_ ==
      data_block_angle_and_timestamp_info.data_block_info_.valid_data_block_num_in_packet_ - 1) {
    uint32_t point_id = 0;
    if (!this->param_.use_lidar_clock) {
      point_id =
          (((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ +
            data_block_angle_and_timestamp_info.data_block_info_.valid_data_block_num_in_packet_) /
           data_block_angle_and_timestamp_info.echo_num_) *
              data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ -
          1;
    } else {
      point_id =
          (((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_) /
           data_block_angle_and_timestamp_info.echo_num_) *
          data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_;
    }
    data_block_angle_and_timestamp_info.first_point_ts_ -=
        decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id];
  }

  uint16_t hor_resolution =
      (uint16_t)(data_block_angle_and_timestamp_info.reserved_field_1_[0] + (data_block_angle_and_timestamp_info.reserved_field_1_[1] << 8));
  uint8_t mirror_id = data_block_angle_and_timestamp_info.reserved_field_1_[4];
  uint16_t rpm =
      (uint16_t)(data_block_angle_and_timestamp_info.reserved_field_1_[5] + (data_block_angle_and_timestamp_info.reserved_field_1_[6] << 8));

  if (hor_resolution_ != hor_resolution / 10)
    hor_resolution_ = hor_resolution / 10;
  if (max_hor_angle_ != hor_resolution * data_block_angle_and_timestamp_info.col_channel_num_ / 10)
    max_hor_angle_ = hor_resolution * data_block_angle_and_timestamp_info.col_channel_num_ / 10;

  uint8_t cur_echo = 1;
  if (data_block_angle_and_timestamp_info.echo_num_ == 2) {
    cur_echo =
        ((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ +
         data_block_angle_and_timestamp_info.data_block_index_in_packet_) %
            2 +
        1;
  }

  if (data_block_angle_and_timestamp_info.echo_num_ == 2 &&
      ((this->param_.publish_mode == 0 && cur_echo == 2) || (this->param_.publish_mode == 1 && cur_echo == 1))) {
    return;
  }

  if (data_block_angle_and_timestamp_info.data_block_info_.data_block_packet_type_ == 1) {
    pre_operate_frequency_ = data_block_angle_and_timestamp_info.operate_frequency_ / 100;
    one_point_info_tmp_index_ = 0;
    if (this->point_cloud_->points.size() == 0)
      m_qmap_one_circle_point_->clear();
    for (int chan_id = 0; chan_id < data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_; chan_id++) {
      int32_t col_index =
          ((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ +
           data_block_angle_and_timestamp_info.data_block_index_in_packet_) /
          data_block_angle_and_timestamp_info.echo_num_;
      int32_t row_index = chan_id;
      uint32_t point_id = col_index * data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ + row_index;
      double timestamp_point = 0.0;
      if (this->param_.ts_first_point)
        timestamp_point = decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id];
      else
        timestamp_point =
            decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id] -
            decoder_packet_base_->decoder_packet_general_version_base_ptr_
                ->all_points_luminous_moment_[decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_.size() - 1];

      int32_t rpm_offset_hor_angle = getTimeSequenceOffsetHorAngle(rpm / 100.0, chan_id + 1);
      int32_t hor_angle = (60000 - (hor_resolution * col_index + data_block->azimuth_offset + rpm_offset_hor_angle) + 360000) % 360000;
      int32_t ver_angle = (ver_angle_type1_[row_index] + data_block->elevation_offset + 360000) % 360000;

      Filterate760::GetInstance()->offset_hor_angle_[mirror_id - 1][data_block_angle_and_timestamp_info.packet_id_ - 1] = data_block->azimuth_offset;
      Filterate760::GetInstance()->offset_ver_angle_[mirror_id - 1][data_block_angle_and_timestamp_info.packet_id_ - 1] =
          data_block->elevation_offset;

      PointInfo point_info;
      // point_info.distance_ = 0;
      point_info.azimuth_ = (double)hor_angle * 1e-3;
      point_info.elevation_ = (double)ver_angle * 1e-3;
      // point_info.reflectivity_ = 0;
      point_info.ring_ = chan_id + this->first_line_id_;
      point_info.timestamp_ = timestamp_point;
      point_info.id_ = point_id;
      data_block_angle_and_timestamp_info.point_info_vector_.emplace_back(point_info);

      OnePointInfo760 one_point_info_tmp;
      one_point_info_tmp.timestamp = timestamp_point;
      one_point_info_tmp.rotating_mirror_id = mirror_id;
      one_point_info_tmp.motor_speed = rpm;
      one_point_info_tmp.intensity = 0;
      // one_point_info_tmp.reflectivity = this->point_cloud_->points[i];
      one_point_info_tmp.distance = data_block_angle_and_timestamp_info.data_block_info_.distance_resolution_;
      one_point_info_tmp.hor_azimuth = hor_resolution * col_index / 10;
      one_point_info_tmp.ver_azimuth = (int16_t)(ver_angle_type1_[row_index] * 100);
      one_point_info_tmp.hor_angle = (double)hor_angle * 1e-3;
      one_point_info_tmp.ver_angle = (double)ver_angle * 1e-3;
      one_point_info_tmp.hor_angle_index = (uint16_t)col_index;
      one_point_info_tmp.echo_type = cur_echo;
      // one_point_info_tmp.y = this->point_cloud_->points[i].y;
      // one_point_info_tmp.x = this->point_cloud_->points[i].x;
      // one_point_info_tmp.z = this->point_cloud_->points[i].z;
      one_point_info_tmp.line_id = chan_id + this->first_line_id_;
      one_point_info_tmp.device_model = 1;

      one_point_info_tmp.index = getOnePointInfoKey(one_point_info_tmp, hor_resolution_);
      m_qmap_one_circle_point_->insert(one_point_info_tmp.index, std::move(one_point_info_tmp));
      one_point_info_tmp_index_++;
    }
  }
}

template <typename T_PointCloud>
void DecoderVanjee760<T_PointCloud>::decoderDataUnitCallback(Vanjee760DataUnit* data_unit, PointInfo& point_info) {
  point_info.distance_ = data_unit->distance;
  // point_info.azimuth_ = point_info.azimuth_;
  // point_info.elevation_ = point_info.elevation_;
  point_info.reflectivity_ = data_unit->reflectivity;
  // point_info.ring_ = point_info.ring_;
  // point_info.timestamp_ = point_info.timestamp_;
  // point_info.id_ = point_info.id_;

  OnePointInfo760& tem = m_qmap_one_circle_point_->getDataIt(m_qmap_one_circle_point_->index_record_size_ - one_point_info_tmp_index_);
  tem.reflectivity = data_unit->reflectivity;
  tem.distance *= data_unit->distance * 1e-3;
  one_point_info_tmp_index_--;
}

template <typename T_PointCloud>
void DecoderVanjee760<T_PointCloud>::pointCloudAlgorithmCallbck() {
  for (int i = 0; i < this->point_cloud_->points.size(); i++) {
    OnePointInfo760& tem = m_qmap_one_circle_point_->getDataIt(i);
    tem.x = this->point_cloud_->points[i].x;
    tem.y = this->point_cloud_->points[i].y;
    tem.z = this->point_cloud_->points[i].z;
  }
  int publish_mode = this->param_.publish_mode >= 2 ? 2 : this->param_.publish_mode;

  Filterate760::GetInstance()->filter(*m_qmap_one_circle_point_, pre_operate_frequency_, true, publish_mode, hor_resolution_, max_hor_angle_);

  this->point_cloud_->points.clear();
  for (size_t i = 0; i < m_qmap_one_circle_point_->index_record_size_; i++) {
    OnePointInfo760& tem = m_qmap_one_circle_point_->getDataIt(i);

    if (this->param_.start_angle < this->param_.end_angle && (tem.hor_angle < this->param_.start_angle || tem.hor_angle > this->param_.end_angle)) {
      tem.distance = 0;
    } else if (this->param_.start_angle > this->param_.end_angle &&
               (tem.hor_angle > this->param_.end_angle && tem.hor_angle < this->param_.start_angle)) {
      tem.distance = 0;
    }

    if (this->hide_range_params_.size() > 0 && tem.distance != 0 &&
        this->isValueInRange(tem.line_id, tem.hor_angle, tem.distance, this->hide_range_params_)) {
      tem.distance = 0;
    }

    typename T_PointCloud::PointT point;
    if (this->distance_section_.in(tem.distance) && tem.non_filterable != 6) {
      setX(point, tem.x);
      setY(point, tem.y);
      setZ(point, tem.z);

      setIntensity(point, tem.reflectivity);
    } else {
      if (!this->param_.dense_points) {
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
      } else {
        setX(point, 0);
        setY(point, 0);
        setZ(point, 0);
      }
      setIntensity(point, 0);
    }
    setTimestamp(point, tem.timestamp);
    setRing(point, tem.line_id);
#ifdef ENABLE_GTEST
    setPointId(point, tem.hor_angle_index * ver_angle_type1_.size() + tem.line_id - this->first_line_id_);
    setHorAngle(point, tem.hor_angle);
    setVerAngle(point, tem.ver_angle);
    setDistance(point, tem.distance);
#endif
    this->point_cloud_->points.emplace_back(std::move(point));
  }
  m_qmap_one_circle_point_->clear();
}

template <typename T_PointCloud>
inline bool DecoderVanjee760<T_PointCloud>::decodeMsopPktBaseProtocol(const uint8_t* pkt, size_t size) {
  std::function<void()> pointcloud_algorithm_callback = nullptr;
  if (algorithm_enable_)
    pointcloud_algorithm_callback = std::bind(&DecoderVanjee760::pointCloudAlgorithmCallbck, this);

  return decoder_packet_base_->decoderPacket(
      pkt, size, std::bind(&DecoderVanjee760::updateAngleAndTimestampInfoCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DecoderVanjee760::decoderDataUnitCallback, this, std::placeholders::_1, std::placeholders::_2), pointcloud_algorithm_callback);
}

template <typename T_PointCloud>
void DecoderVanjee760<T_PointCloud>::updateAngleAndTimestampInfoCallbackV1_1(DataBlockAngleAndTimestampInfo& data_block_angle_and_timestamp_info,
                                                                             Vanjee760DataBlockV1_1* data_block) {
  if (data_block_angle_and_timestamp_info.operate_frequency_ / 100 != pre_operate_frequency_) {
    if (data_block_angle_and_timestamp_info.total_pkt_num_ * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ /
            data_block_angle_and_timestamp_info.echo_num_ ==
        800) {
      decoder_packet_v1_1_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_ = all_points_luminous_moment_760_120degree_;
    } else {
      decoder_packet_v1_1_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_ = all_points_luminous_moment_760_140degree_;
    }
  }

  if (data_block_angle_and_timestamp_info.data_block_index_in_packet_ ==
      data_block_angle_and_timestamp_info.data_block_info_.valid_data_block_num_in_packet_ - 1) {
    uint32_t point_id = 0;
    if (!this->param_.use_lidar_clock) {
      point_id =
          (((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ +
            data_block_angle_and_timestamp_info.data_block_info_.valid_data_block_num_in_packet_) /
           data_block_angle_and_timestamp_info.echo_num_) *
              data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ -
          1;
    } else {
      point_id =
          (((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_) /
           data_block_angle_and_timestamp_info.echo_num_) *
          data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_;
    }
    data_block_angle_and_timestamp_info.first_point_ts_ -=
        decoder_packet_v1_1_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id];
  }

  uint16_t hor_resolution =
      (uint16_t)(data_block_angle_and_timestamp_info.reserved_field_1_[0] + (data_block_angle_and_timestamp_info.reserved_field_1_[1] << 8));
  uint8_t mirror_id = data_block_angle_and_timestamp_info.reserved_field_1_[4];
  uint16_t rpm =
      (uint16_t)(data_block_angle_and_timestamp_info.reserved_field_1_[5] + (data_block_angle_and_timestamp_info.reserved_field_1_[6] << 8));

  if (hor_resolution_ != hor_resolution / 10)
    hor_resolution_ = hor_resolution / 10;
  if (max_hor_angle_ != hor_resolution * data_block_angle_and_timestamp_info.col_channel_num_ / 10)
    max_hor_angle_ = hor_resolution * data_block_angle_and_timestamp_info.col_channel_num_ / 10;

  uint8_t cur_echo = 1;
  if (data_block_angle_and_timestamp_info.echo_num_ == 2) {
    cur_echo =
        ((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ +
         data_block_angle_and_timestamp_info.data_block_index_in_packet_) %
            2 +
        1;
  }

  if (data_block_angle_and_timestamp_info.echo_num_ == 2 &&
      ((this->param_.publish_mode == 0 && cur_echo == 2) || (this->param_.publish_mode == 1 && cur_echo == 1))) {
    return;
  }

  uint8_t confidence_[192];
  for (int i = 0; i < sizeof(confidence_); i++) {
    int byte_index = (i * 3) / 8;
    int bit_offset = (i * 3) % 8;
    uint8_t value = (data_block->confidence[byte_index] >> bit_offset) & 0x07;
    if (bit_offset > 5) {
      value |= (data_block->confidence[byte_index + 1] << (8 - bit_offset)) & 0x07;
    }
    confidence_[i] = value & 0x03;
  }

  if (data_block_angle_and_timestamp_info.data_block_info_.data_block_packet_type_ == 1) {
    pre_operate_frequency_ = data_block_angle_and_timestamp_info.operate_frequency_ / 100;
    one_point_info_tmp_index_ = 0;
    if (this->point_cloud_->points.size() == 0)
      m_qmap_one_circle_point_->clear();
    for (int chan_id = 0; chan_id < data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_; chan_id++) {
      int32_t col_index =
          ((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ +
           data_block_angle_and_timestamp_info.data_block_index_in_packet_) /
          data_block_angle_and_timestamp_info.echo_num_;
      int32_t row_index = chan_id;
      uint32_t point_id = col_index * data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ + row_index;
      double timestamp_point = 0.0;
      if (this->param_.ts_first_point)
        timestamp_point = decoder_packet_v1_1_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id];
      else
        timestamp_point =
            decoder_packet_v1_1_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id] -
            decoder_packet_v1_1_->decoder_packet_general_version_base_ptr_
                ->all_points_luminous_moment_[decoder_packet_v1_1_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_.size() - 1];

      int32_t rpm_offset_hor_angle = getTimeSequenceOffsetHorAngle(rpm / 100.0, chan_id + 1);
      int32_t hor_angle = (60000 - (hor_resolution * col_index + data_block->azimuth_offset + rpm_offset_hor_angle) + 360000) % 360000;
      int32_t ver_angle = (ver_angle_type1_[row_index] + data_block->elevation_offset + 360000) % 360000;

      Filterate760::GetInstance()->offset_hor_angle_[mirror_id - 1][data_block_angle_and_timestamp_info.packet_id_ - 1] = data_block->azimuth_offset;
      Filterate760::GetInstance()->offset_ver_angle_[mirror_id - 1][data_block_angle_and_timestamp_info.packet_id_ - 1] =
          data_block->elevation_offset;

      PointInfo point_info;
      // point_info.distance_ = 0;
      point_info.azimuth_ = (double)hor_angle * 1e-3;
      point_info.elevation_ = (double)ver_angle * 1e-3;
      // point_info.reflectivity_ = 0;
      point_info.confidence_ = confidence_[chan_id];
      point_info.ring_ = chan_id + this->first_line_id_;
      point_info.timestamp_ = timestamp_point;
      point_info.id_ = point_id;
      data_block_angle_and_timestamp_info.point_info_vector_.emplace_back(point_info);

      OnePointInfo760 one_point_info_tmp;
      one_point_info_tmp.timestamp = timestamp_point;
      one_point_info_tmp.rotating_mirror_id = mirror_id;
      one_point_info_tmp.motor_speed = rpm;
      one_point_info_tmp.intensity = 0;
      // one_point_info_tmp.reflectivity = this->point_cloud_->points[i];
      one_point_info_tmp.confidence = confidence_[chan_id];
      one_point_info_tmp.distance = data_block_angle_and_timestamp_info.data_block_info_.distance_resolution_;
      one_point_info_tmp.hor_azimuth = hor_resolution * col_index / 10;
      one_point_info_tmp.ver_azimuth = (int16_t)(ver_angle_type1_[row_index] * 100);
      one_point_info_tmp.hor_angle = (double)hor_angle * 1e-3;
      one_point_info_tmp.ver_angle = (double)ver_angle * 1e-3;
      one_point_info_tmp.hor_angle_index = (uint16_t)col_index;
      one_point_info_tmp.echo_type = cur_echo;
      // one_point_info_tmp.y = this->point_cloud_->points[i].y;
      // one_point_info_tmp.x = this->point_cloud_->points[i].x;
      // one_point_info_tmp.z = this->point_cloud_->points[i].z;
      one_point_info_tmp.line_id = chan_id + this->first_line_id_;
      one_point_info_tmp.device_model = 1;

      one_point_info_tmp.index = getOnePointInfoKey(one_point_info_tmp, hor_resolution_);
      m_qmap_one_circle_point_->insert(one_point_info_tmp.index, std::move(one_point_info_tmp));
      one_point_info_tmp_index_++;
    }
  }
}

template <typename T_PointCloud>
inline bool DecoderVanjee760<T_PointCloud>::decodeMsopPktProtocolV1_1(const uint8_t* pkt, size_t size) {
  std::function<void()> pointcloud_algorithm_callback = nullptr;
  if (algorithm_enable_)
    pointcloud_algorithm_callback = std::bind(&DecoderVanjee760::pointCloudAlgorithmCallbck, this);

  return decoder_packet_v1_1_->decoderPacket(
      pkt, size, std::bind(&DecoderVanjee760::updateAngleAndTimestampInfoCallbackV1_1, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DecoderVanjee760::decoderDataUnitCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DecoderVanjee760::pointCloudAlgorithmCallbck, this));
}

template <typename T_PointCloud>
void DecoderVanjee760<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);
  std::shared_ptr<ProtocolAbstract> p;
}
}  // namespace lidar

}  // namespace vanjee
