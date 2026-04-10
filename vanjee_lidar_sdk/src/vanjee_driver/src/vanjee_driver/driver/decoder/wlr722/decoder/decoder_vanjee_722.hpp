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
#include <vanjee_driver/driver/decoder/decoder_packet_base/imu/imuParamGet.hpp>
#include <vanjee_driver/driver/decoder/imu_calibration_param.hpp>
#include <vanjee_driver/driver/decoder/wlr722/protocol/frames/cmd_repository_722.hpp>
#include <vanjee_driver/driver/decoder/wlr722/protocol/frames/protocol_imu_packet_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722/protocol/frames/protocol_imu_temp_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722/protocol/frames/protocol_imuaddpa_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722/protocol/frames/protocol_imulinepa_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722/protocol/frames/protocol_ld_eccentricity_param_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722/protocol/frames/protocol_ldangle_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722/protocol/frames/protocol_ldoffset_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722/protocol/frames/protocol_ldvalue_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722/protocol/frames/protocol_protocol_version_set.hpp>
#include <vanjee_driver/driver/decoder/wlr722/protocol/frames/protocol_work_mode_get.hpp>
#include <vanjee_driver/driver/decoder/wlr722/protocol/frames/protocol_work_mode_set.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct _Vanjee722Channel {
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t remain;
} Vanjee722Channel;

typedef struct _Vanjee722BlockChannel16 {
  uint16_t azimuth;
  Vanjee722Channel channel[16];
} Vanjee722BlockChannel16;

typedef struct _Vanjee722Difop {
  uint8_t mac_id[2];
  uint16_t circle_id;
  uint8_t data_time[6];
  uint8_t timestamp[4];
  int16_t imu_linear_acce_x;
  int16_t imu_linear_acce_y;
  int16_t imu_linear_acce_z;
  int16_t imu_angle_voc_x;
  int16_t imu_angle_voc_y;
  int16_t imu_angle_voc_z;
  uint8_t remain1[11];
  uint8_t return_wave_mode;
  uint16_t rpm;
  uint8_t device_id[3];
  uint8_t versoin;
  uint8_t remain2[12];
  uint16_t frame_id;
  uint8_t tail[2];
} Vanjee722Difop;

typedef struct _Vanjee722MsopPktChannel16 {
  uint8_t head[2];
  uint8_t channel_num;
  uint8_t return_wave_num;
  uint8_t block_num;
  Vanjee722BlockChannel16 blocks[10];
  Vanjee722Difop difop;
} Vanjee722MsopPktChannel16;

typedef struct _Vanjee722BlockChannel32 {
  uint16_t azimuth;
  Vanjee722Channel channel[32];
} Vanjee722BlockChannel32;

typedef struct _Vanjee722MsopPktChannel32 {
  uint8_t head[2];
  uint8_t channel_num;
  uint8_t return_wave_num;
  uint8_t block_num;
  Vanjee722BlockChannel32 blocks[10];
  Vanjee722Difop difop;
} Vanjee722MsopPktChannel32;

#pragma pack(pop)

#pragma pack(push, 1)

typedef struct _Vanjee722DataUnit {
  uint16_t distance;
  uint8_t reflectivity;
  uint8_t Confidence;
} Vanjee722DataUnit;

typedef struct _Vanjee722DataBlock {
  uint8_t start_flag;
  uint16_t azimuth_offset;
  uint16_t elevation_offset;
  Vanjee722DataUnit channel[32];
} Vanjee722DataBlock;

#pragma pack(pop)

template <typename T_PointCloud>
class DecoderVanjee722 : public DecoderMech<T_PointCloud> {
 private:
  std::vector<double> all_points_luminous_moment_722_16_;
  std::vector<std::vector<double>> all_points_luminous_moment_722_32_;
  const double luminous_period_of_ld_16_ = 8.33e-5;
  const double luminous_period_of_ld_32_ = 1.666e-4;
  const double luminous_period_of_adjacent_ld_ = 5e-6;
  std::vector<uint32_t> all_points_luminous_moment_size_;
  int32_t azimuth_trans_pre_ = -1.0;

  int32_t pre_frame_id_ = -1;
  int32_t pre_circle_id_ = -1;
  uint8_t publish_mode_ = 0;

  int32_t pre_pkt_hor_angle_ = -1;
  double pre_pkt_time_ = -1;
  int32_t resolution_num_offset_ = -1;

  uint16_t pre_operate_frequency_ = 0;
  int32_t pre_pkt_resolution_ = -1;
  bool angle_param_get_flag_ = false;
  bool eccentricity_param_get_flag_ = false;
  std::vector<int32_t> eccentricity_angles_;
  std::vector<int32_t> eccentricity_angles_real_;

  uint8_t protocol_versoin_ = 0;
  double pre_imu_timestamp_ = 0.0;
  uint32_t start_angle_ = 0;
  uint32_t end_angle_ = 360000;

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam &getConstParam(uint8_t mode);
  void initLdLuminousMoment(void);
  bool lidarParamGet(uint32_t angle_index, uint32_t hor_angle, uint32_t resolution, uint8_t block_num, double time);

  DecoderPacketBase<T_PointCloud, Vanjee722DataBlock, Vanjee722DataUnit> *decoder_packet_base_ = nullptr;
  void updateAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo &data_block_angle_and_timestamp_info, Vanjee722DataBlock *data_block);
  void decoderDataUnitCallback(Vanjee722DataUnit *data_unit, PointInfo &point_info);

 public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 100;
  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  virtual bool decodeMsopPktChannel16(const uint8_t *pkt, size_t size);
  virtual bool decodeMsopPktChannel32(const uint8_t *pkt, size_t size);
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee722() = default;
  explicit DecoderVanjee722(const WJDecoderParam &param);

  bool decodeMsopPktBaseProtocol(const uint8_t *pkt, size_t size);

  void SendImuData(Vanjee722Difop difop, double temperature, double timestamp, double lidar_timestamp);

 public:
  std::shared_ptr<ImuParamGet> m_imu_params_get_;
  double imu_temperature_;
  bool imu_ready_;
  bool point_cloud_ready_;
};

template <typename T_PointCloud>
bool DecoderVanjee722<T_PointCloud>::lidarParamGet(uint32_t angle_index, uint32_t hor_angle, uint32_t resolution, uint8_t block_num, double time) {
  if (pre_pkt_hor_angle_ > 0 && (hor_angle + 36000 - pre_pkt_hor_angle_) % 36000 == resolution * block_num) {
    if (pre_pkt_time_ > 0 && fabs(time - pre_pkt_time_) > 0.001) {
      resolution_num_offset_ = (hor_angle / resolution) % angle_index;
      return true;
    }
  }
  pre_pkt_hor_angle_ = hor_angle;
  pre_pkt_time_ = time;
  return false;
}

template <typename T_PointCloud>
void DecoderVanjee722<T_PointCloud>::initLdLuminousMoment() {
  double offset = 0;
  all_points_luminous_moment_722_16_.resize(19200);
  all_points_luminous_moment_722_32_.resize(2);
  all_points_luminous_moment_722_32_[0].resize(38400);
  all_points_luminous_moment_722_32_[1].resize(19200);
  all_points_luminous_moment_size_.resize(2);
  all_points_luminous_moment_size_[0] = 38400;
  all_points_luminous_moment_size_[1] = 19200;
  for (uint16_t col = 0; col < 1200; col++) {
    for (uint8_t row = 0; row < 16; row++) {
      all_points_luminous_moment_722_16_[col * 16 + row] = col * luminous_period_of_ld_16_ + row * luminous_period_of_adjacent_ld_;
    }
  }
  for (uint16_t col = 0; col < 1200; col++) {
    for (uint8_t row = 0; row < 32; row++) {
      all_points_luminous_moment_722_32_[0][col * 32 + row] = (col * luminous_period_of_ld_32_ + row * luminous_period_of_adjacent_ld_) / 2;
      if (col < 600) {
        all_points_luminous_moment_722_32_[1][col * 32 + row] = col * luminous_period_of_ld_32_ + row * luminous_period_of_adjacent_ld_;
      }
    }
  }
}

template <typename T_PointCloud>
inline WJDecoderMechConstParam &DecoderVanjee722<T_PointCloud>::getConstParam(uint8_t mode) {
  // WJ_INFOL << "publish_mode ============mode=================" << mode <<
  // WJ_REND;
  uint16_t msop_len = 1365;
  uint16_t laser_num = 32;
  uint16_t block_num = 10;
  uint16_t chan_num = 32;
  float distance_min = 0.2f;
  float distance_max = 70.0f;
  float distance_resolution = 0.004f;
  float init_temperature = 80.0f;

  static WJDecoderMechConstParam param = {
      msop_len  /// msop len
      ,
      laser_num  /// laser number
      ,
      block_num  /// blocks per packet
      ,
      chan_num  /// channels per block
      ,
      distance_min  /// distance min
      ,
      distance_max  /// distance max
      ,
      distance_resolution  /// distance resolution
      ,
      init_temperature  /// initial value of temperature
  };
  param.BLOCK_DURATION = 0.1 / 360;
  return param;
}

template <typename T_PointCloud>
inline DecoderVanjee722<T_PointCloud>::DecoderVanjee722(const WJDecoderParam &param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param) {
  decoder_packet_base_ = new DecoderPacketBase<T_PointCloud, Vanjee722DataBlock, Vanjee722DataUnit>(this);
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  publish_mode_ = param.publish_mode;
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);
  m_imu_params_get_ = std::make_shared<ImuParamGet>(270);
  imu_temperature_ = -100.0;
  imu_ready_ = false;
  point_cloud_ready_ = false;
  if (param.imu_enable == -1) {
    point_cloud_ready_ = true;
  } else if (param.imu_enable == 0) {
    imu_temperature_ = 40.0;
    point_cloud_ready_ = true;
  } else {
    WJ_INFO << "Waiting for IMU calibration..." << WJ_REND;
  }
  start_angle_ = this->param_.start_angle * 1000;
  end_angle_ = this->param_.end_angle * 1000;

  if (this->param_.config_from_file) {
    this->chan_angles_.loadFromFile(this->param_.angle_path_ver);

    this->chan_angles_.loadFromFile(this->param_.angle_path_hor, 1200);
    for (int i = 0; i < 1200; i++) {
      eccentricity_angles_real_.push_back(this->chan_angles_.eccentricityAdjust(i, 100));
    }

    this->imu_calibration_param_.loadFromFile(param.imu_param_path);

    m_imu_params_get_->setImuTempCalibrationParams(this->imu_calibration_param_.x_axis_temp_k, this->imu_calibration_param_.x_axis_temp_b,
                                                   this->imu_calibration_param_.y_axis_temp_k, this->imu_calibration_param_.y_axis_temp_b,
                                                   this->imu_calibration_param_.z_axis_temp_k, this->imu_calibration_param_.z_axis_temp_b);

    m_imu_params_get_->setImuAcceCalibrationParams(this->imu_calibration_param_.x_axis_acc_k, this->imu_calibration_param_.x_axis_acc_b,
                                                   this->imu_calibration_param_.y_axis_acc_k, this->imu_calibration_param_.y_axis_acc_b,
                                                   this->imu_calibration_param_.z_axis_acc_k, this->imu_calibration_param_.z_axis_acc_b);
  }
  initLdLuminousMoment();
}

template <typename T_PointCloud>
inline bool DecoderVanjee722<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size) {
  bool ret = false;
  uint16_t l_pktheader = pkt[0] << 8 | pkt[1];
  switch (l_pktheader) {
    case 0xFFEE: {
      if (size == sizeof(Vanjee722MsopPktChannel16))
        ret = decodeMsopPktChannel16(pkt, size);
      else if (size == sizeof(Vanjee722MsopPktChannel32))
        ret = decodeMsopPktChannel32(pkt, size);
    } break;
    case 0xFFDD: {
      if (size == sizeof(Vanjee722MsopPktChannel16))
        ret = decodeMsopPktChannel16(pkt, size);
      else if (size == sizeof(Vanjee722MsopPktChannel32))
        ret = decodeMsopPktChannel32(pkt, size);
    } break;
    case 0xFFCC: {
      ret = decodeMsopPktBaseProtocol(pkt, size);
      protocol_versoin_ = 2;
    } break;
    default:
      break;
  }

  return ret;
}

template <typename T_PointCloud>
bool DecoderVanjee722<T_PointCloud>::decodeMsopPktChannel16(const uint8_t *pkt, size_t size) {
  if (size != sizeof(Vanjee722MsopPktChannel16))
    return false;
  auto &packet = *(Vanjee722MsopPktChannel16 *)pkt;
  bool ret = false;
  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;

  uint16_t frame_id = ntohs(packet.difop.frame_id);
  uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_frame_id_ = frame_id;

  pkt_host_ts = getTimeHost() * 1e-6;
  std::tm stm;
  memset(&stm, 0, sizeof(stm));
  stm.tm_year = packet.difop.data_time[5] + 100;
  stm.tm_mon = packet.difop.data_time[4] - 1;
  stm.tm_mday = packet.difop.data_time[3];
  stm.tm_hour = packet.difop.data_time[2];
  stm.tm_min = packet.difop.data_time[1];
  stm.tm_sec = packet.difop.data_time[0];
  double nsec = (packet.difop.timestamp[0] + (packet.difop.timestamp[1] << 8) + (packet.difop.timestamp[2] << 16) +
                 ((packet.difop.timestamp[3] & 0x0F) << 24)) *
                1e-8;
  pkt_lidar_ts = std::mktime(&stm) + nsec;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else {
    pkt_ts = pkt_lidar_ts;
    if (pkt_ts < 3600) {
      pkt_ts = 3600;
    }
  }

  int32_t resolution = 30;
  double last_point_to_first_point_time = 0;
  if (packet.blocks[1].azimuth - packet.blocks[0].azimuth == 0) {
    resolution = (packet.blocks[2].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  } else {
    resolution = (packet.blocks[1].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  }

  if (resolution != pre_pkt_resolution_) {
    resolution_num_offset_ = -1;
    pre_pkt_hor_angle_ = -1;
    pre_pkt_time_ = -1;
  }

  uint32_t angle_index = 150;
  if (packet.difop.versoin == 1) {
    angle_index = 30;
  }

  pre_pkt_resolution_ = resolution;
  if (resolution_num_offset_ < 0 && !lidarParamGet(angle_index, packet.blocks[0].azimuth, resolution, packet.block_num, pkt_lidar_ts)) {
    return false;
  }

  if (resolution != 30) {
    return ret;
  }

  if (pkt_lidar_ts != this->prev_pkt_ts_) {
    SendImuData(packet.difop, imu_temperature_, pkt_ts, pkt_lidar_ts);
  }

  if (!imu_ready_ && !point_cloud_ready_) {
    this->prev_pkt_ts_ = pkt_lidar_ts;
    return ret;
  } else if (!point_cloud_ready_) {
    point_cloud_ready_ = true;
  }

  uint32_t loss_circles_num = (ntohs(packet.difop.circle_id) + 65536 - pre_circle_id_) % 65536;
  if (loss_circles_num > 1) {
    this->point_cloud_->points.clear();
  }
  uint16_t block_num = packet.block_num * packet.return_wave_num;
  for (uint16_t blk = 0; blk < block_num; blk++) {
    if (packet.return_wave_num == 2 && publish_mode_ == 0 && (blk % 2 == 1)) {
      continue;
    } else if (packet.return_wave_num == 2 && publish_mode_ == 1 && (blk % 2 == 0)) {
      continue;
    }

    const Vanjee722BlockChannel16 &block = packet.blocks[blk];
    int32_t azimuth = block.azimuth % 36000;
    int32_t azimuth_trans = azimuth;  //(block.azimuth + resolution) % 36000;

    if ((this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans != 0) ||
        (loss_circles_num == 1 && blk == 0 && (packet.blocks[0].azimuth % 36000) != 0 &&
         packet.blocks[0].azimuth < packet.blocks[block_num - 1].azimuth && this->point_cloud_->points.size() != 0)) {
      int32_t time_reload_num = 0;
      int32_t time_change_num = (azimuth / resolution - resolution_num_offset_);
      if (time_change_num >= 0) {
        time_reload_num = time_change_num / angle_index;
        if (time_change_num % angle_index >= 0) {
          time_reload_num += 1;
        }
      }
      uint32_t point_gap_num = (time_reload_num * angle_index + resolution_num_offset_) * packet.channel_num;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_722_16_[point_gap_num];
      this->first_point_ts_ = this->last_point_ts_ - all_points_luminous_moment_722_16_[all_points_luminous_moment_722_16_.size() - 1];
      this->cb_split_frame_(packet.channel_num, this->cloudTs());
      ret = true;
    }

    {
      double timestamp_point;
      for (uint16_t chan = 0; chan < packet.channel_num; chan++) {
        float x, y, z, xy;

        uint32_t point_id = azimuth / resolution * packet.channel_num + chan;
        if (this->param_.ts_first_point == true) {
          timestamp_point = all_points_luminous_moment_722_16_[point_id];
        } else {
          timestamp_point =
              all_points_luminous_moment_722_16_[point_id] - all_points_luminous_moment_722_16_[all_points_luminous_moment_722_16_.size() - 1];
        }

        const Vanjee722Channel &channel = block.channel[chan];

        float distance = channel.distance * this->const_param_.DISTANCE_RES;
        int32_t angle_vert = this->chan_angles_.vertAdjust(chan + 8);
        int32_t angle_horiz_final = (this->chan_angles_.horizAdjust(chan + 8, azimuth * 10, RotateDirection::clockwise) +
                                     this->chan_angles_.eccentricityAdjust(((azimuth + 36000) % 36000) / resolution, 100)) %
                                    360000;

        if (angle_horiz_final < 0) {
          angle_horiz_final += 360000;
        }

        int32_t angle_horiz_mask = (360000 - angle_horiz_final) % 360000;
        if (this->param_.start_angle < this->param_.end_angle) {
          if (angle_horiz_mask < this->param_.start_angle * 1000 || angle_horiz_mask > this->param_.end_angle * 1000) {
            distance = 0;
          }
        } else {
          if (angle_horiz_mask > this->param_.end_angle * 1000 && angle_horiz_mask < this->param_.start_angle * 1000) {
            distance = 0;
          }
        }

        if (this->hide_range_params_.size() > 0 && distance != 0 &&
            this->isValueInRange(chan + this->first_line_id_, angle_horiz_mask / 1000.0, distance, this->hide_range_params_)) {
          distance = 0;
        }

        int32_t azimuth_index = angle_horiz_final;
        int32_t verticalVal_722 = angle_vert;

        if (this->distance_section_.in(distance)) {
          xy = distance * COS(verticalVal_722);
          x = xy * COS(azimuth_index);
          y = -xy * SIN(azimuth_index);
          z = distance * SIN(verticalVal_722);
          this->transformPoint(x, y, z);

          typename T_PointCloud::PointT point;
          setX(point, x);
          setY(point, y);
          setZ(point, z);
          setIntensity(point, channel.reflectivity);
          setTimestamp(point, timestamp_point);
          setRing(point, chan + this->first_line_id_);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, azimuth_index / 1000.0);
          setVerAngle(point, verticalVal_722 / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        } else {
          typename T_PointCloud::PointT point;
          if (!this->param_.dense_points) {
            setX(point, NAN);
            setY(point, NAN);
            setZ(point, NAN);
          } else {
            float x_zero = 0;
            float y_zero = 0;
            float z_zero = 0;
            this->transformPoint(x_zero, y_zero, z_zero);

            setX(point, x_zero);
            setY(point, y_zero);
            setZ(point, z_zero);
          }
          setIntensity(point, 0);
          setTimestamp(point, timestamp_point);
          setRing(point, chan + this->first_line_id_);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, azimuth_index / 1000.0);
          setVerAngle(point, verticalVal_722 / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        }
      }
    }
    if (azimuth_trans == 0 && (packet.return_wave_num == 1 || (packet.return_wave_num == 2 && publish_mode_ != 2) ||
                               (packet.return_wave_num == 2 && publish_mode_ == 2 && azimuth_trans_pre_ == 0))) {
      uint32_t point_gap_num = resolution_num_offset_ * 32;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_722_16_[point_gap_num];
      this->first_point_ts_ = this->last_point_ts_ - all_points_luminous_moment_722_16_[all_points_luminous_moment_722_16_.size() - 1];
      this->cb_split_frame_(packet.channel_num, this->cloudTs());
      ret = true;
    }
    azimuth_trans_pre_ = azimuth_trans;
  }
  pre_circle_id_ = ntohs(packet.difop.circle_id);
  this->prev_pkt_ts_ = pkt_lidar_ts;
  return ret;
}

template <typename T_PointCloud>
bool DecoderVanjee722<T_PointCloud>::decodeMsopPktChannel32(const uint8_t *pkt, size_t size) {
  if (size != sizeof(Vanjee722MsopPktChannel32))
    return false;
  auto &packet = *(Vanjee722MsopPktChannel32 *)pkt;
  bool ret = false;
  double pkt_ts = 0;
  double pkt_host_ts = 0;
  double pkt_lidar_ts = 0;

  uint16_t frame_id = ntohs(packet.difop.frame_id);
  uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_frame_id_ = frame_id;

  pkt_host_ts = getTimeHost() * 1e-6;
  std::tm stm;
  memset(&stm, 0, sizeof(stm));
  stm.tm_year = packet.difop.data_time[5] + 100;
  stm.tm_mon = packet.difop.data_time[4] - 1;
  stm.tm_mday = packet.difop.data_time[3];
  stm.tm_hour = packet.difop.data_time[2];
  stm.tm_min = packet.difop.data_time[1];
  stm.tm_sec = packet.difop.data_time[0];
  double nsec = (packet.difop.timestamp[0] + (packet.difop.timestamp[1] << 8) + (packet.difop.timestamp[2] << 16) +
                 ((packet.difop.timestamp[3] & 0x0F) << 24)) *
                1e-8;
  pkt_lidar_ts = std::mktime(&stm) + nsec;

  if (!this->param_.use_lidar_clock)
    pkt_ts = pkt_host_ts;
  else {
    pkt_ts = pkt_lidar_ts;
    if (pkt_ts < 3600) {
      pkt_ts = 3600;
    }
  }

  int32_t resolution = 60;
  if (packet.blocks[1].azimuth - packet.blocks[0].azimuth == 0) {
    resolution = (packet.blocks[2].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  } else {
    resolution = (packet.blocks[1].azimuth - packet.blocks[0].azimuth + 36000) % 36000;
  }

  if (resolution != pre_pkt_resolution_) {
    resolution_num_offset_ = -1;
    pre_pkt_hor_angle_ = -1;
    pre_pkt_time_ = -1;
  }

  uint32_t angle_index = 150;
  if (packet.difop.versoin >= 1) {
    angle_index = 30;
    if (resolution == 30)
      angle_index = 60;
  }
  protocol_versoin_ = packet.difop.versoin;

  pre_pkt_resolution_ = resolution;
  if (resolution_num_offset_ < 0 && !lidarParamGet(angle_index, packet.blocks[0].azimuth, resolution, packet.block_num, pkt_lidar_ts)) {
    return false;
  }

  if (resolution != 60 && resolution != 30)
    return false;

  uint8_t resolurion_index = resolution / 30 - 1;

  if (packet.difop.versoin < 2 && pkt_lidar_ts != this->prev_pkt_ts_) {
    SendImuData(packet.difop, imu_temperature_, pkt_ts, pkt_lidar_ts);
  }

  if (!imu_ready_ && !point_cloud_ready_) {
    this->prev_pkt_ts_ = pkt_lidar_ts;
    return ret;
  } else if (!point_cloud_ready_) {
    point_cloud_ready_ = true;
  }

  uint32_t loss_circles_num = (ntohs(packet.difop.circle_id) + 65536 - pre_circle_id_) % 65536;
  if (loss_circles_num > 1) {
    this->point_cloud_->points.clear();
  }
  uint16_t block_num = packet.block_num * packet.return_wave_num;
  for (uint16_t blk = 0; blk < block_num; blk++) {
    if (packet.return_wave_num == 2 && publish_mode_ == 0 && (blk % 2 == 1)) {
      continue;
    } else if (packet.return_wave_num == 2 && publish_mode_ == 1 && (blk % 2 == 0)) {
      continue;
    }

    const Vanjee722BlockChannel32 &block = packet.blocks[blk];
    int32_t azimuth = block.azimuth % 36000;
    int32_t azimuth_trans = (block.azimuth + resolution) % 36000;

    if ((this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans != 0) ||
        (loss_circles_num == 1 && blk == 0 && azimuth != 0 && azimuth < (packet.blocks[block_num - 1].azimuth % 36000) &&
         this->point_cloud_->points.size() != 0)) {
      int32_t time_reload_num = 0;
      int32_t time_change_num = (azimuth / resolution - resolution_num_offset_);
      if (time_change_num >= 0) {
        time_reload_num = time_change_num / angle_index;
        if (time_change_num % angle_index >= 0) {
          time_reload_num += 1;
        }
      }
      uint32_t point_gap_num = (time_reload_num * angle_index + resolution_num_offset_) * 32;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_722_32_[resolurion_index][point_gap_num];
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_722_32_[resolurion_index][all_points_luminous_moment_size_[resolurion_index] - 1];

      this->cb_split_frame_(packet.channel_num, this->cloudTs());
      ret = true;
    }

    {
      double timestamp_point;
      uint32_t point_id_first = (azimuth / resolution) * packet.channel_num;
      uint32_t col_index_eccentricity = azimuth / 30;
      int32_t azimuth_10 = azimuth * 10;
      for (uint16_t chan = 0; chan < packet.channel_num; chan++) {
        float x, y, z, xy;

        uint32_t point_id = point_id_first + chan;
        if (this->param_.ts_first_point == true) {
          timestamp_point = all_points_luminous_moment_722_32_[resolurion_index][point_id];
        } else {
          timestamp_point = all_points_luminous_moment_722_32_[resolurion_index][point_id] -
                            all_points_luminous_moment_722_32_[resolurion_index][all_points_luminous_moment_size_[resolurion_index] - 1];
        }

        const Vanjee722Channel &channel = block.channel[chan];

        float distance = channel.distance * this->const_param_.DISTANCE_RES;
        int32_t verticalVal_722 = this->chan_angles_.vertAdjust(chan);
        int32_t azimuth_index =
            (this->chan_angles_.horizAdjust(chan, azimuth_10, RotateDirection::clockwise) + eccentricity_angles_real_[col_index_eccentricity]) %
            360000;

        int32_t angle_horiz_mask = 360000 - azimuth_index;
        if (start_angle_ < end_angle_) {
          if (angle_horiz_mask < start_angle_ || angle_horiz_mask > end_angle_) {
            distance = 0;
          }
        } else {
          if (angle_horiz_mask > end_angle_ && angle_horiz_mask < start_angle_) {
            distance = 0;
          }
        }

        if (this->hide_range_flag_ && distance != 0 &&
            this->isValueInRange(chan + this->first_line_id_, angle_horiz_mask / 1000.0, distance, this->hide_range_params_)) {
          distance = 0;
        }

        if (this->distance_section_.in(distance)) {
          xy = distance * COS(verticalVal_722);
          x = xy * COS(azimuth_index);
          y = -xy * SIN(azimuth_index);
          z = distance * SIN(verticalVal_722);
          this->transformPoint(x, y, z);

          typename T_PointCloud::PointT point;
          setX(point, x);
          setY(point, y);
          setZ(point, z);
          setIntensity(point, channel.reflectivity);
          setTimestamp(point, timestamp_point);
          setRing(point, chan + this->first_line_id_);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, azimuth_index / 1000.0);
          setVerAngle(point, verticalVal_722 / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        } else {
          typename T_PointCloud::PointT point;
          if (!this->param_.dense_points) {
            setX(point, NAN);
            setY(point, NAN);
            setZ(point, NAN);
          } else {
            float x_zero = 0;
            float y_zero = 0;
            float z_zero = 0;
            this->transformPoint(x_zero, y_zero, z_zero);

            setX(point, x_zero);
            setY(point, y_zero);
            setZ(point, z_zero);
          }
          setIntensity(point, 0);
          setTimestamp(point, timestamp_point);
          setRing(point, chan + this->first_line_id_);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, azimuth_index / 1000.0);
          setVerAngle(point, verticalVal_722 / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        }
      }
    }
    if (azimuth_trans == 0 && (packet.return_wave_num == 1 || (packet.return_wave_num == 2 && publish_mode_ != 2) ||
                               (packet.return_wave_num == 2 && publish_mode_ == 2 && azimuth_trans_pre_ == 0))) {
      uint32_t point_gap_num = resolution_num_offset_ * 32;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_722_32_[resolurion_index][point_gap_num];
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_722_32_[resolurion_index][all_points_luminous_moment_size_[resolurion_index] - 1];

      this->cb_split_frame_(packet.channel_num, this->cloudTs());
      ret = true;
    }
    azimuth_trans_pre_ = azimuth_trans;
  }
  pre_circle_id_ = ntohs(packet.difop.circle_id);
  this->prev_pkt_ts_ = pkt_lidar_ts;
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee722<T_PointCloud>::updateAngleAndTimestampInfoCallback(DataBlockAngleAndTimestampInfo &data_block_angle_and_timestamp_info,
                                                                         Vanjee722DataBlock *data_block) {
  if (data_block_angle_and_timestamp_info.operate_frequency_ / 100 != pre_operate_frequency_) {
    int resolution_index =
        (data_block_angle_and_timestamp_info.reserved_field_1_[0] | (data_block_angle_and_timestamp_info.reserved_field_1_[1] << 8)) / 300 - 1;
    decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_ =
        all_points_luminous_moment_722_32_[resolution_index];
  }

  if (data_block_angle_and_timestamp_info.data_block_index_in_packet_ ==
      (data_block_angle_and_timestamp_info.data_block_info_.valid_data_block_num_in_packet_ - 1)) {
    if (!this->param_.use_lidar_clock) {
      uint32_t point_id =
          (((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ +
            data_block_angle_and_timestamp_info.data_block_info_.valid_data_block_num_in_packet_) /
           data_block_angle_and_timestamp_info.echo_num_) *
              data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ -
          1;
      data_block_angle_and_timestamp_info.first_point_ts_ -=
          decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[point_id];
    } else {
      uint32_t offset_point_num =
          ((data_block_angle_and_timestamp_info.packet_id_ - 1) / (3 * data_block_angle_and_timestamp_info.echo_num_)) * 30 * 32;
      data_block_angle_and_timestamp_info.first_point_ts_ -=
          decoder_packet_base_->decoder_packet_general_version_base_ptr_->all_points_luminous_moment_[offset_point_num];
    }
  }

  if (data_block_angle_and_timestamp_info.data_block_info_.data_block_packet_type_ == 1) {
    if (data_block_angle_and_timestamp_info.echo_num_ == 2) {
      if (publish_mode_ == 0 && (data_block_angle_and_timestamp_info.data_block_index_in_packet_ % 2) == 1) {
        return;
      } else if (publish_mode_ == 1 && (data_block_angle_and_timestamp_info.data_block_index_in_packet_ % 2) == 0) {
        return;
      }
    }
    pre_operate_frequency_ = data_block_angle_and_timestamp_info.operate_frequency_ / 100;
    for (int chan_id = 0; chan_id < data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_; chan_id++) {
      int32_t col_index =
          ((data_block_angle_and_timestamp_info.packet_id_ - 1) * data_block_angle_and_timestamp_info.data_block_info_.data_block_num_in_packet_ +
           data_block_angle_and_timestamp_info.data_block_index_in_packet_) /
          data_block_angle_and_timestamp_info.echo_num_;
      int32_t row_index = chan_id;
      uint32_t point_id = col_index * data_block_angle_and_timestamp_info.data_block_info_.row_channel_num_in_data_block_ + row_index;
      uint16_t hor_resolution =
          (uint16_t)(data_block_angle_and_timestamp_info.reserved_field_1_[0] + (data_block_angle_and_timestamp_info.reserved_field_1_[1] << 8));
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
      point_info.azimuth_ = ((360000 -
                              (this->chan_angles_.horizAdjust(chan_id, col_index * hor_resolution, RotateDirection::clockwise) % 360000 +
                               this->chan_angles_.eccentricityAdjust(((col_index * hor_resolution + 360000) % 360000) / 300, 100)) +
                              360000) %
                             360000) /
                            1000.0;
      point_info.elevation_ = this->chan_angles_.vertAdjust(chan_id) / 1000.0;
      // point_info.reflectivity_ = 0;
      point_info.ring_ = chan_id + this->first_line_id_;
      point_info.timestamp_ = timestamp_point;
      point_info.id_ = point_id;
      data_block_angle_and_timestamp_info.point_info_vector_.emplace_back(point_info);
    }
  }
}

template <typename T_PointCloud>
void DecoderVanjee722<T_PointCloud>::decoderDataUnitCallback(Vanjee722DataUnit *data_unit, PointInfo &point_info) {
  point_info.distance_ = data_unit->distance;
  // point_info.azimuth_ = point_info.azimuth_;
  // point_info.elevation_ = point_info.elevation_;
  point_info.reflectivity_ = data_unit->reflectivity;
  // point_info.ring_ = point_info.ring_;
  // point_info.timestamp_ = point_info.timestamp_;
  // point_info.id_ = point_info.id_;
}

template <typename T_PointCloud>
inline bool DecoderVanjee722<T_PointCloud>::decodeMsopPktBaseProtocol(const uint8_t *pkt, size_t size) {
  return decoder_packet_base_->decoderPacket(
      pkt, size, std::bind(&DecoderVanjee722::updateAngleAndTimestampInfoCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DecoderVanjee722::decoderDataUnitCallback, this, std::placeholders::_1, std::placeholders::_2));
}

template <typename T_PointCloud>
void DecoderVanjee722<T_PointCloud>::SendImuData(Vanjee722Difop difop, double temperature, double timestamp, double lidar_timestamp) {
  if (this->param_.imu_enable == -1)
    return;
  imu_ready_ = m_imu_params_get_->imuGet(difop.imu_angle_voc_x, difop.imu_angle_voc_y, difop.imu_angle_voc_z, difop.imu_linear_acce_x,
                                         difop.imu_linear_acce_y, difop.imu_linear_acce_z, lidar_timestamp, temperature);
  if (imu_ready_) {
    this->imu_packet_->timestamp = timestamp;
    this->imu_packet_->angular_voc[0] = m_imu_params_get_->imu_result_stu_.x_angle;
    this->imu_packet_->angular_voc[1] = m_imu_params_get_->imu_result_stu_.y_angle;
    this->imu_packet_->angular_voc[2] = m_imu_params_get_->imu_result_stu_.z_angle;

    this->imu_packet_->linear_acce[0] = m_imu_params_get_->imu_result_stu_.x_acc;
    this->imu_packet_->linear_acce[1] = m_imu_params_get_->imu_result_stu_.y_acc;
    this->imu_packet_->linear_acce[2] = m_imu_params_get_->imu_result_stu_.z_acc;

    this->imu_packet_->orientation[0] = m_imu_params_get_->imu_result_stu_.q0;
    this->imu_packet_->orientation[1] = m_imu_params_get_->imu_result_stu_.q1;
    this->imu_packet_->orientation[2] = m_imu_params_get_->imu_result_stu_.q2;
    this->imu_packet_->orientation[3] = m_imu_params_get_->imu_result_stu_.q3;

    this->cb_imu_pkt_();
  }
}

template <typename T_PointCloud>
void DecoderVanjee722<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<ProtocolAbstract722> p;
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

  if (*sp_cmd == *(CmdRepository722::CreateInstance()->sp_ld_value_get_)) {
    p = std::make_shared<Protocol_LDValueGet722>();
  } else if (*sp_cmd == *(CmdRepository722::CreateInstance()->sp_ld_eccentricity_param_get_)) {
    p = std::make_shared<Protocol_LDEccentricityParamGet722>();
  } else if (*sp_cmd == *(CmdRepository722::CreateInstance()->sp_imu_line_Param_get_)) {
    p = std::make_shared<Protocol_ImuLineGet722>();
  } else if (*sp_cmd == *(CmdRepository722::CreateInstance()->sp_imu_add_Param_get_)) {
    p = std::make_shared<Protocol_ImuAddGet722>();
  } else if (*sp_cmd == *(CmdRepository722::CreateInstance()->sp_temperature_param_get_)) {
    p = std::make_shared<Protocol_ImuTempGet722>();
  } else if (*sp_cmd == *(CmdRepository722::CreateInstance()->sp_get_work_mode_)) {
    p = std::make_shared<Protocol_WorkModeGet722>();
  } else if (*sp_cmd == *(CmdRepository722::CreateInstance()->sp_set_work_mode_)) {
    p = std::make_shared<Protocol_WorkModeSet722>();
  } else if (*sp_cmd == *(CmdRepository722::CreateInstance()->sp_get_imu_packet_)) {
    p = std::make_shared<Protocol_ImuPacketGet722>();
  } else {
    return;
  }
  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Params_LDValue722)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    if (!angle_param_get_flag_) {
      std::shared_ptr<Params_LDValue722> param = std::dynamic_pointer_cast<Params_LDValue722>(params);
      std::vector<double> vert_angles;
      std::vector<double> offset_angles;
      for (int num_of_lines = 0; num_of_lines < param->num_of_lines_; num_of_lines++) {
        vert_angles.push_back((double)(param->ver_angle_[num_of_lines] / 1000.0));
        offset_angles.push_back((double)(param->offset_angle_[num_of_lines] / 1000.0));
      }

      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        this->chan_angles_.loadFromLiDAR(this->param_.angle_path_ver, param->num_of_lines_, vert_angles, offset_angles);
        WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }

      angle_param_get_flag_ = true;
    }

    if (angle_param_get_flag_ && eccentricity_param_get_flag_) {
      Decoder<T_PointCloud>::angles_ready_ = true;
    }
  } else if (typeid(*params) == typeid(Params_LDEccentricityParamGet722)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }
    if (!eccentricity_param_get_flag_) {
      std::shared_ptr<Params_LDEccentricityParamGet722> param = std::dynamic_pointer_cast<Params_LDEccentricityParamGet722>(params);
      // std::vector<int32_t> eccentricity_angles_;
      if ((eccentricity_angles_.size() == 0 && param->frame_id_ == 1) ||
          (eccentricity_angles_.size() == param->frame_len_ && param->frame_id_ == 2)) {
        for (int num_of_angle = 0; num_of_angle < param->frame_len_; num_of_angle++) {
          eccentricity_angles_.push_back((int32_t)(param->eccentricity_angle_[num_of_angle]));
          eccentricity_angles_real_.push_back((int32_t)(param->eccentricity_angle_[num_of_angle] * 100));
        }
        if (param->frame_id_ == 2)
          eccentricity_param_get_flag_ = true;
      }

      if (eccentricity_param_get_flag_ && Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        this->chan_angles_.loadFromLiDAR(this->param_.angle_path_hor, param->frame_len_ * param->frame_id_, eccentricity_angles_);
        WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[0x0001].setStopFlag(true);
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[0x0002].setStopFlag(true);
      }
    }

    if (angle_param_get_flag_ && eccentricity_param_get_flag_) {
      Decoder<T_PointCloud>::angles_ready_ = true;
    }
  } else if (typeid(*params) == typeid(Params_IMULine722)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    std::shared_ptr<Params_IMULine722> param = std::dynamic_pointer_cast<Params_IMULine722>(params);

    if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
      WJ_INFOL << "Get LiDAR<IMU> angular_vel data..." << WJ_REND;
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    }

    this->imu_calibration_param_.loadFromLiDAR(this->param_.imu_param_path, this->imu_calibration_param_.TEMP_PARAM, param->x_k_, param->x_b_,
                                               param->y_k_, param->y_b_, param->z_k_, param->z_b_);

    WJ_INFO << param->x_k_ << "," << param->x_b_ << "," << param->y_k_ << "," << param->y_b_ << "," << param->z_k_ << "," << param->z_b_ << WJ_REND;

    m_imu_params_get_->setImuTempCalibrationParams(param->x_k_, param->x_b_, param->y_k_, param->y_b_, param->z_k_, param->z_b_);

  } else if (typeid(*params) == typeid(Params_IMUAdd722)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    std::shared_ptr<Params_IMUAdd722> param = std::dynamic_pointer_cast<Params_IMUAdd722>(params);

    if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
      WJ_INFOL << "Get LiDAR<IMU> linear_acc data..." << WJ_REND;
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    }

    this->imu_calibration_param_.loadFromLiDAR(this->param_.imu_param_path, this->imu_calibration_param_.ACC_PARAM, param->x_k_, param->x_b_,
                                               param->y_k_, param->y_b_, param->z_k_, param->z_b_);

    WJ_INFO << param->x_k_ << "," << param->x_b_ << "," << param->y_k_ << "," << param->y_b_ << "," << param->z_k_ << "," << param->z_b_ << WJ_REND;

    m_imu_params_get_->setImuAcceCalibrationParams(param->x_k_, param->x_b_, param->y_k_, param->y_b_, param->z_k_, param->z_b_);
  } else if (typeid(*params) == typeid(Params_LiDARRunStatus722)) {
    std::shared_ptr<Params_LiDARRunStatus722> param = std::dynamic_pointer_cast<Params_LiDARRunStatus722>(params);

    // WJ_INFOL << "imu_temp:" << param->imu_temp_ << WJ_REND;

    imu_temperature_ = param->imu_temp_ / 100.0f;
  } else if (typeid(*params) == typeid(Params_WorkModeGet722)) {
    std::shared_ptr<Params_WorkModeGet722> param = std::dynamic_pointer_cast<Params_WorkModeGet722>(params);
    (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);

  } else if (typeid(*params) == typeid(Params_WorkModeSet722)) {
    std::shared_ptr<Params_WorkModeSet722> param = std::dynamic_pointer_cast<Params_WorkModeSet722>(params);
    if (this->param_.device_ctrl_state_enable && param->flag_ == 1) {
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      this->device_ctrl_->cmd_id = 1;
      this->device_ctrl_->cmd_state = 1;
      this->cb_device_ctrl_state_(pre_pkt_time_ < 0 ? 0 : pre_pkt_time_);
    }

  } else if (typeid(*params) == typeid(Params_ImuPacketGet722)) {
    std::shared_ptr<Params_ImuPacketGet722> param = std::dynamic_pointer_cast<Params_ImuPacketGet722>(params);
    if (protocol_versoin_ < 2 || this->param_.imu_enable == -1)
      return;
    double pkt_ts = 0.0;
    double imu_timestamp = param->imu_sec_ + (param->imu_nsec_ * 1e-9);
    if (this->param_.use_lidar_clock)
      pkt_ts = imu_timestamp;
    else
      pkt_ts = getTimeHost() * 1e-6;

    if (pre_imu_timestamp_ > 0 && imu_timestamp > pre_imu_timestamp_) {
      imu_ready_ =
          m_imu_params_get_->imuGet(param->imu_angle_voc_x_, param->imu_angle_voc_y_, param->imu_angle_voc_z_, param->imu_linear_acce_x_,
                                    param->imu_linear_acce_y_, param->imu_linear_acce_z_, imu_timestamp, -100.0, false, false, false, false, true);
      if (imu_ready_) {
        this->imu_packet_->timestamp = pkt_ts;
        this->imu_packet_->angular_voc[0] = m_imu_params_get_->imu_result_stu_.x_angle;
        this->imu_packet_->angular_voc[1] = m_imu_params_get_->imu_result_stu_.y_angle;
        this->imu_packet_->angular_voc[2] = m_imu_params_get_->imu_result_stu_.z_angle;

        this->imu_packet_->linear_acce[0] = m_imu_params_get_->imu_result_stu_.x_acc;
        this->imu_packet_->linear_acce[1] = m_imu_params_get_->imu_result_stu_.y_acc;
        this->imu_packet_->linear_acce[2] = m_imu_params_get_->imu_result_stu_.z_acc;

        this->imu_packet_->orientation[0] = m_imu_params_get_->imu_result_stu_.q0;
        this->imu_packet_->orientation[1] = m_imu_params_get_->imu_result_stu_.q1;
        this->imu_packet_->orientation[2] = m_imu_params_get_->imu_result_stu_.q2;
        this->imu_packet_->orientation[3] = m_imu_params_get_->imu_result_stu_.q3;

        this->cb_imu_pkt_();
      }
    }
    pre_imu_timestamp_ = imu_timestamp;

  } else {
    WJ_WARNING << "Unknown Params Type..." << WJ_REND;
  }
}

}  // namespace lidar

}  // namespace vanjee
