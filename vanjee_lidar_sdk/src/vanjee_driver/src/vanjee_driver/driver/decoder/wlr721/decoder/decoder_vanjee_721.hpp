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
#include <vanjee_driver/driver/decoder/wlr721/protocol/frames/cmd_repository_721.hpp>
#include <vanjee_driver/driver/decoder/wlr721/protocol/frames/protocol_ldangle_get_721.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct _Vanjee721Channel {
  uint8_t distance[2];
  uint8_t intensity;
  uint8_t reflectivity;
} Vanjee721Channel;

typedef struct _Vanjee721Block {
  uint8_t header[2];
  uint8_t rotation[2];
  Vanjee721Channel channel[64];
} Vanjee721Block;

typedef struct _Vanjee721Difop {
  uint8_t id[4];
  uint8_t circle_id[2];
  uint8_t frame_id[2];
  uint8_t channel_num;
  uint8_t echo_mode;
  uint16_t rpm;
  uint32_t second;
  uint32_t microsecond;
  uint8_t reserved[4];
} Vanjee721Difop;

typedef struct _Vanjee721MsopPktSingle {
  Vanjee721Block blocks[5];
  Vanjee721Difop difop;
} Vanjee721MsopPktSingle;

typedef struct _Vanjee721MsopPktDouble {
  Vanjee721Block blocks[4];
  Vanjee721Difop difop;
} Vanjee721MsopPktDouble;
#pragma pack(pop)

template <typename T_PointCloud>
class DecoderVanjee721 : public DecoderMech<T_PointCloud> {
 private:
  std::vector<std::vector<double>> all_points_luminous_moment_721_;  // Cache 64 channels, one circle point
                                                                     // cloud time difference
  const double luminous_period_of_ld_ = 0.000166655;                 // Time interval at adjacent horizontal angles
  const double luminous_period_of_adjacent_ld_ = 0.000001130;        // Time interval between adjacent vertical angles within the
                                                                     // group

  int32_t azimuth_trans_pre_ = -1.0;

  int32_t pre_frame_id_ = -1;
  uint8_t publish_mode_ = 0;

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam& getConstParam(uint8_t mode);
  void initLdLuminousMoment(void);

 public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 120;

  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee721() = default;
  explicit DecoderVanjee721(const WJDecoderParam& param);

  bool decodeMsopPktSingleEcho(const uint8_t* pkt, size_t size);
  bool decodeMsopPktDoubleEcho(const uint8_t* pkt, size_t size);
};

template <typename T_PointCloud>
void DecoderVanjee721<T_PointCloud>::initLdLuminousMoment() {
  double offset = 0;
  all_points_luminous_moment_721_.resize(3);
  all_points_luminous_moment_721_[0].resize(115200);
  all_points_luminous_moment_721_[1].resize(76800);
  all_points_luminous_moment_721_[2].resize(38400);
  for (uint16_t col = 0; col < 1800; col++) {
    for (uint8_t row = 0; row < 64; row++) {
      offset = (row - ((row / 4) * 4) + 1) * luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ * (row / 4) / 16;

      if (col < 600) {
        for (int i = 0; i < 3; i++) all_points_luminous_moment_721_[i][col * 64 + row] = col * luminous_period_of_ld_ + offset;
      } else if (col >= 600 && col < 1200) {
        for (int i = 0; i < 2; i++) all_points_luminous_moment_721_[i][col * 64 + row] = col * luminous_period_of_ld_ + offset;
      } else {
        all_points_luminous_moment_721_[0][col * 64 + row] = col * luminous_period_of_ld_ + offset;
      }
    }
  }
}

template <typename T_PointCloud>
inline WJDecoderMechConstParam& DecoderVanjee721<T_PointCloud>::getConstParam(uint8_t mode) {
  switch (mode) {
    case 1: {
      static WJDecoderMechConstParam param = {1324, 64, 5, 64, 0.3f, 45.0f, 0.004f, 80.0f};
      param.BLOCK_DURATION = 0.1 / 360;
      return param;
    } break;

    case 2:
    default: {
      static WJDecoderMechConstParam param = {1064, 64, 4, 64, 0.3f, 45.0f, 0.004f, 80.0f};
      param.BLOCK_DURATION = 0.1 / 360;
      return param;
    } break;
  }
}

template <typename T_PointCloud>
inline DecoderVanjee721<T_PointCloud>::DecoderVanjee721(const WJDecoderParam& param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param) {
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  publish_mode_ = param.publish_mode;
  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);
  if (this->param_.config_from_file) {
    this->chan_angles_.loadFromFile(this->param_.angle_path_ver);
  }
  initLdLuminousMoment();
}

template <typename T_PointCloud>
inline bool DecoderVanjee721<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size) {
  bool ret = false;
  switch (size) {
    case 1324: {
      ret = decodeMsopPktSingleEcho(pkt, size);
    } break;

    case 1064: {
      ret = decodeMsopPktDoubleEcho(pkt, size);
    } break;
  }
  return ret;
}

template <typename T_PointCloud>
inline bool DecoderVanjee721<T_PointCloud>::decodeMsopPktSingleEcho(const uint8_t* pkt, size_t size) {
  const Vanjee721MsopPktSingle& packet = *(Vanjee721MsopPktSingle*)pkt;
  bool ret = false;
  double pkt_ts = 0;

  uint16_t frame_id = (packet.difop.frame_id[0] << 8) | packet.difop.frame_id[1];
  uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_frame_id_ = frame_id;

  if (!this->param_.use_lidar_clock)
    pkt_ts = getTimeHost() * 1e-6;
  else {
    pkt_ts = packet.difop.second + ((double)(packet.difop.microsecond & 0x0fffffff) * 1e-6);
    if (pkt_ts < 3600)
      pkt_ts = 3600;
  }

  int32_t resolution = 200;
  uint8_t resolution_index = 0;
  double last_point_to_first_point_time = 0;
  resolution = (((packet.blocks[1].rotation[0] << 8 | packet.blocks[1].rotation[1]) -
                 (packet.blocks[0].rotation[0] << 8 | packet.blocks[0].rotation[1]) + 36000) %
                36000) *
               10;

  if (resolution == 200) {
    resolution_index = 0;
    last_point_to_first_point_time = (1.0 / 3.3) - all_points_luminous_moment_721_[resolution_index][115199];
  } else if (resolution == 300) {
    resolution_index = 1;
    last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_721_[resolution_index][76799];
  } else if (resolution == 600) {
    resolution_index = 2;
    last_point_to_first_point_time = (1.0 / 10.0) - all_points_luminous_moment_721_[resolution_index][38399];
  } else {
    return ret;
  }

  for (uint16_t blk = 0; blk < 5; blk++) {
    const Vanjee721Block& block = packet.blocks[blk];
    int32_t azimuth = (block.rotation[0] << 8 | block.rotation[1]) * 10;
    int32_t azimuth_trans = (azimuth + resolution) % 360000;

    if (this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans != 0) {
      int32_t time_reload_num = (azimuth_trans / resolution) / 5;
      if (((azimuth_trans / resolution) % 5) > 0)
        time_reload_num += 1;
      int32_t point_gap_num = time_reload_num * 5 * 64;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_721_[resolution_index][point_gap_num] - last_point_to_first_point_time;
      ;
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_721_[resolution_index][all_points_luminous_moment_721_[resolution_index].size() - 1];
      this->cb_split_frame_(64, this->cloudTs());
      ret = true;
    }

    double timestamp_point;
    for (uint16_t chan = 0; chan < 64; chan++) {
      float x, y, z, xy;

      uint32_t point_id = azimuth / resolution * 64 + chan;
      if (this->param_.ts_first_point == true) {
        timestamp_point = all_points_luminous_moment_721_[resolution_index][point_id];
      } else {
        timestamp_point = all_points_luminous_moment_721_[resolution_index][point_id] -
                          all_points_luminous_moment_721_[resolution_index][all_points_luminous_moment_721_[resolution_index].size() - 1];
      }

      const Vanjee721Channel& channel = block.channel[chan];
      float tem_distance = channel.distance[0] << 8 | channel.distance[1];
      float distance = tem_distance * this->const_param_.DISTANCE_RES;
      int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
      int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, azimuth) % 360000;
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
      int32_t verticalVal_721 = angle_vert;

      if (this->distance_section_.in(distance)) {
        xy = distance * COS(verticalVal_721);
        x = xy * COS(azimuth_index);
        y = -xy * SIN(azimuth_index);
        z = distance * SIN(verticalVal_721);
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
        setVerAngle(point, verticalVal_721 / 1000.0);
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
          setX(point, 0);
          setY(point, 0);
          setZ(point, 0);
        }

        setIntensity(point, 0);
        setTimestamp(point, timestamp_point);
        setRing(point, chan + this->first_line_id_);
#ifdef ENABLE_GTEST
        setPointId(point, point_id);
        setHorAngle(point, azimuth_index / 1000.0);
        setVerAngle(point, verticalVal_721 / 1000.0);
        setDistance(point, distance);
#endif
        this->point_cloud_->points.emplace_back(point);
      }
    }
    if (azimuth_trans == 0) {
      this->last_point_ts_ = pkt_ts;
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_721_[resolution_index][all_points_luminous_moment_721_[resolution_index].size() - 1];
      this->cb_split_frame_(64, this->cloudTs());
      ret = true;
    }
  }
  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

template <typename T_PointCloud>
inline bool DecoderVanjee721<T_PointCloud>::decodeMsopPktDoubleEcho(const uint8_t* pkt, size_t size) {
  const Vanjee721MsopPktDouble& packet = *(Vanjee721MsopPktDouble*)pkt;
  bool ret = false;
  double pkt_ts = 0;

  uint16_t frame_id = (packet.difop.frame_id[0] << 8) | packet.difop.frame_id[1];
  uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_frame_id_ = frame_id;

  if (!this->param_.use_lidar_clock)
    pkt_ts = getTimeHost() * 1e-6;
  else {
    pkt_ts = packet.difop.second + ((double)(packet.difop.microsecond & 0x0fffffff) * 1e-6);
    if (pkt_ts < 3600)
      pkt_ts = 3600;
  }

  int32_t resolution = 20;
  uint8_t resolution_index = 0;
  double last_point_to_first_point_time = 0;
  resolution = (((packet.blocks[2].rotation[0] << 8 | packet.blocks[2].rotation[1]) -
                 (packet.blocks[0].rotation[0] << 8 | packet.blocks[0].rotation[1]) + 36000) %
                36000) *
               10;

  if (resolution == 200) {
    resolution_index = 0;
    last_point_to_first_point_time = (1.0 / 3.3) - all_points_luminous_moment_721_[resolution_index][115199];
  } else if (resolution == 300) {
    resolution_index = 1;
    last_point_to_first_point_time = (1.0 / 5.0) - all_points_luminous_moment_721_[resolution_index][76799];
  } else if (resolution == 600) {
    resolution_index = 2;
    last_point_to_first_point_time = (1.0 / 10.0) - all_points_luminous_moment_721_[resolution_index][38399];
  } else {
    return ret;
  }

  for (uint16_t blk = 0; blk < 4; blk++) {
    if ((packet.difop.echo_mode & 0xf0) != 0 && publish_mode_ == 0 && (blk % 2 == 1)) {
      continue;
    } else if ((packet.difop.echo_mode & 0xf0) != 0 && publish_mode_ == 1 && (blk % 2 == 0)) {
      continue;
    }

    const Vanjee721Block& block = packet.blocks[blk];
    int32_t azimuth = block.rotation[0] << 8 | block.rotation[1];
    int32_t azimuth_trans = (azimuth + resolution) % 360000;

    if (this->split_strategy_->newBlock(azimuth_trans) && azimuth_trans != 0) {
      int32_t time_reload_num = (azimuth_trans / resolution) / 2;
      if (((azimuth_trans / resolution) % 2) > 0)
        time_reload_num += 1;
      int32_t point_gap_num = time_reload_num * 5 * 64;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_721_[resolution_index][point_gap_num] - last_point_to_first_point_time;
      ;
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_721_[resolution_index][all_points_luminous_moment_721_[resolution_index].size() - 1];
      this->cb_split_frame_(64, this->cloudTs());
      ret = true;
    }
    {
      double timestamp_point;
      for (uint16_t chan = 0; chan < 64; chan++) {
        float x, y, z, xy;

        uint32_t point_id = azimuth / resolution * 64 + chan;
        if (this->param_.ts_first_point == true) {
          timestamp_point = all_points_luminous_moment_721_[resolution_index][point_id];
        } else {
          timestamp_point = all_points_luminous_moment_721_[resolution_index][point_id] -
                            all_points_luminous_moment_721_[resolution_index][all_points_luminous_moment_721_[resolution_index].size() - 1];
        }

        const Vanjee721Channel& channel = block.channel[chan];
        float tem_distance = channel.distance[0] << 8 | channel.distance[1];
        float distance = tem_distance * this->const_param_.DISTANCE_RES;
        int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
        int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, azimuth * 10) % 360000;
        if (angle_horiz_final < 0) {
          angle_horiz_final += 360000;
        }
        if (this->param_.start_angle < this->param_.end_angle) {
          if (angle_horiz_final < this->param_.start_angle * 1000 || angle_horiz_final > this->param_.end_angle * 1000) {
            distance = 0;
          }
        } else {
          if (angle_horiz_final > this->param_.end_angle * 1000 && angle_horiz_final < this->param_.start_angle * 1000) {
            distance = 0;
          }
        }

        if (this->hide_range_params_.size() > 0 && distance != 0 &&
            this->isValueInRange(chan + this->first_line_id_, angle_horiz_final / 1000.0, distance, this->hide_range_params_)) {
          distance = 0;
        }

        int32_t azimuth_index = angle_horiz_final;
        int32_t verticalVal_721 = angle_vert;

        if (this->distance_section_.in(distance)) {
          xy = distance * COS(verticalVal_721);
          x = xy * SIN(azimuth_index);
          y = xy * (COS(azimuth_index));
          z = distance * SIN(verticalVal_721);
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
          setVerAngle(point, verticalVal_721 / 1000.0);
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
            setX(point, 0);
            setY(point, 0);
            setZ(point, 0);
          }

          setIntensity(point, 0);
          setTimestamp(point, timestamp_point);
          setRing(point, chan + this->first_line_id_);
#ifdef ENABLE_GTEST
          setPointId(point, point_id);
          setHorAngle(point, azimuth_index / 1000.0);
          setVerAngle(point, verticalVal_721 / 1000.0);
          setDistance(point, distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        }
      }
    }
    if (azimuth_trans == 0 && (publish_mode_ != 2 || (publish_mode_ == 2 && azimuth_trans_pre_ == 0))) {
      this->last_point_ts_ = pkt_ts;
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_721_[resolution_index][all_points_luminous_moment_721_[resolution_index].size() - 1];
      this->cb_split_frame_(64, this->cloudTs());
      ret = true;
    }
    azimuth_trans_pre_ = azimuth_trans;
  }
  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee721<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<ProtocolAbstract> p;
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

  if (*sp_cmd == *(CmdRepository721::CreateInstance()->Sp_LDAngleGet)) {
    p = std::make_shared<Protocol_LDAngleGet721>();
  } else {
  }

  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Params_LDAngle721)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    std::shared_ptr<Params_LDAngle721> param = std::dynamic_pointer_cast<Params_LDAngle721>(params);

    std::vector<double> vert_angles;
    std::vector<double> horiz_angles;

    for (int NumOfLines = 0; NumOfLines < param->NumOfLines; NumOfLines++) {
      vert_angles.push_back((double)(param->VerAngle[NumOfLines] / 1000.0));
      horiz_angles.push_back((double)(param->HorAngle[NumOfLines] / 1000.0));
    }

    this->chan_angles_.loadFromLiDAR(this->param_.angle_path_ver, param->NumOfLines, vert_angles, horiz_angles);

    if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
      WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      Decoder<T_PointCloud>::angles_ready_ = true;
    }
  } else {
  }
}

}  // namespace lidar

}  // namespace vanjee
