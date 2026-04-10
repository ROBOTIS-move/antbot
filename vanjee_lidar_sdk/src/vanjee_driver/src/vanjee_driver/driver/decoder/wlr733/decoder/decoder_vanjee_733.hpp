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
#include <vanjee_driver/driver/decoder/wlr733/protocol/frames/cmd_repository_733.hpp>
#include <vanjee_driver/driver/decoder/wlr733/protocol/frames/protocol_ldangle_get_733.hpp>
#include <vanjee_driver/driver/decoder/wlr733/protocol/frames/protocol_motor_params_get_733.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct _Vanjee733Channel {
  uint16_t distance;
  uint8_t intensity;
  uint8_t reflectivity;

} Vanjee733Channel;

typedef struct _Vanjee733Block {
  uint16_t header;
  uint16_t rotation;
  Vanjee733Channel channel[64];
} Vanjee733Block;

typedef struct _Vanjee733DifopPkt {
  uint8_t time_syn_state;
  uint8_t time_syn_source;
  uint8_t time_syn_error;
  uint8_t protocol_ver;
  uint8_t echo_mode;
  uint8_t pc_msg;
  uint8_t reserved[18];
} Vanjee733DifopPkt;

typedef struct _Vanjee733MsopPkt {
  Vanjee733Block blocks[4];
  uint8_t status[16];
  uint16_t circle_id;
  uint16_t frame_id;
  uint32_t second;
  uint32_t microsecond;
  Vanjee733DifopPkt difop;
} Vanjee733MsopPkt;
#pragma pack(pop)

template <typename T_PointCloud>
class DecoderVanjee733 : public DecoderMech<T_PointCloud> {
 private:
  std::vector<std::vector<double>> all_points_luminous_moment_733_;  // Cache 64 channels, one circle point
                                                                     // cloud time difference
  const double luminous_period_of_ld_ = 0.00005555;                  // Time interval at adjacent horizontal angles

  int32_t azimuth_trans_pre_ = -1.0;

  int32_t pre_circle_id_ = -1;
  ChanAngles chan_angles_;
  uint8_t publish_mode_ = 0;

  uint32_t laser_degree_per_sec_ = 0;
  std::vector<double> channel_light_moment_733_ = std::vector<double>{
      0.2,  3.2,  6.2,  9.2,  12.2, 15.2, 1.7,  4.7,  7.7,  10.7, 13.7, 16.7, 18.2, 21.2, 24.5, 27.2, 29.9, 32.6, 19.7, 22.7, 26,  28.7,
      31.4, 34.1, 35.3, 37.5, 39.6, 41.7, 43.6, 45.5, 36.5, 38.7, 40.8, 42.7, 44.6, 46.5, 47.3, 48.9, 50.5, 48.1, 49.7, 51.3, 2.3, 5.3,
      8.3,  0.8,  3.8,  6.8,  11.3, 14.3, 17.3, 9.8,  12.8, 15.8, 18.8, 25.1, 27.8, 30.5, 33.2, 20.3, 26.6, 29.3, 32,   34.7};

  bool angle_get_flag_ = false;
  bool motor_params_get_flag_ = false;
  RotateDirection rotate_direction_ = RotateDirection::anticlockwise;

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam &getConstParam(uint8_t mode);

 public:
  constexpr static double FRAME_DURATION = 0.1;
  constexpr static uint32_t SINGLE_PKT_NUM = 450;
  void initLdLuminousMoment(void);

  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee733() = default;
  explicit DecoderVanjee733(const WJDecoderParam &param);
};

template <typename T_PointCloud>
void DecoderVanjee733<T_PointCloud>::initLdLuminousMoment() {
  double glowing_moments[64] = {
      0.0000001,  0.0000073,  0.00001465, 0.00002185, 0.00001645, 0.00002365, 0.0000001,  0.0000073,  0.00001465, 0.00002185, 0.00001645,
      0.00002365, 0.0000019,  0.0000091,  0.0000037,  0.0000109,  0.00001825, 0.00002545, 0.0000019,  0.0000091,  0.0000037,  0.0000109,
      0.00001825, 0.00002545, 0.0000055,  0.0000127,  0.00002005, 0.00002725, 0.0000292,  0.000034,   0.0000055,  0.0000127,  0.00002005,
      0.00002725, 0.0000292,  0.000034,   0.00003715, 0.0000308,  0.0000324,  0.00003855, 0.00003575, 0.0000324,  0.00003995, 0.00003855,
      0.00003575, 0.00003995, 0.00003715, 0.0000308,  0.0000457,  0.0000429,  0.0000443,  0.0000415,  0.0000429,  0.0000443,  0.0000457,
      0.0000415,  0.00004725, 0.00004865, 0.00005005, 0.00005145, 0.00004725, 0.00004865, 0.00005005, 0.00005145};
  all_points_luminous_moment_733_.resize(4);
  all_points_luminous_moment_733_[0].resize(230400);
  all_points_luminous_moment_733_[1].resize(115200);
  all_points_luminous_moment_733_[2].resize(76800);
  all_points_luminous_moment_733_[3].resize(57600);
  for (uint16_t col = 0; col < 3600; col++) {
    for (uint8_t row = 0; row < 64; row++) {
      if (col < 900) {
        for (int i = 0; i < 4; i++) all_points_luminous_moment_733_[i][col * 64 + row] = col * luminous_period_of_ld_ + glowing_moments[row];
      } else if (col >= 900 && col < 1200) {
        for (int i = 0; i < 3; i++) all_points_luminous_moment_733_[i][col * 64 + row] = col * luminous_period_of_ld_ + glowing_moments[row];
      } else if (col >= 1200 && col < 1800) {
        for (int i = 0; i < 2; i++) all_points_luminous_moment_733_[i][col * 64 + row] = col * luminous_period_of_ld_ + glowing_moments[row];
      } else {
        all_points_luminous_moment_733_[0][col * 64 + row] = col * luminous_period_of_ld_ + glowing_moments[row];
      }
    }
  }
}

template <typename T_PointCloud>
inline WJDecoderMechConstParam &DecoderVanjee733<T_PointCloud>::getConstParam(uint8_t mode) {
  // WJDecoderConstParam
  // WJDecoderMechConstParam
  static WJDecoderMechConstParam param = {
      1092  // msop len
      ,
      64  // laser number
      ,
      4  // blocks per packet
      ,
      64  // channels per block
      ,
      0.3f  // distance min
      ,
      200.0f  // distance max
      ,
      0.004f  // distance resolution
      ,
      80.0f  // initial value of temperature
  };
  param.BLOCK_DURATION = 0.1 / 360;
  return param;
}

template <typename T_PointCloud>
inline DecoderVanjee733<T_PointCloud>::DecoderVanjee733(const WJDecoderParam &param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param), chan_angles_(this->const_param_.LASER_NUM) {
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
inline bool DecoderVanjee733<T_PointCloud>::decodeMsopPkt(const uint8_t *packet, size_t size) {
  if (size != sizeof(Vanjee733MsopPkt))
    return false;
  const Vanjee733MsopPkt &pkt = *(Vanjee733MsopPkt *)packet;
  bool ret = false;
  double pkt_ts = 0;

  int16_t near_packe_no_gap = 1;
  if ((pkt.difop.echo_mode & 0x80) == 0x80)
    near_packe_no_gap = 2;
  int32_t loss_circles_num = (pkt.circle_id + 65536 - pre_circle_id_) % 65536;
  if (loss_circles_num > near_packe_no_gap && pre_circle_id_ >= 0) {
    WJ_WARNING << "loss " << (loss_circles_num / near_packe_no_gap - 1) << " circle" << WJ_REND;
    this->point_cloud_->points.clear();
  }
  pre_circle_id_ = pkt.circle_id;

  if (!this->param_.use_lidar_clock)
    pkt_ts = getTimeHost() * 1e-6;
  else {
    pkt_ts = pkt.second + pkt.microsecond * 1e-6;
    if (pkt_ts < 3600) {
      pkt_ts = 3600;
    }
  }

  int32_t resolution = 100;
  uint8_t resolution_index = 0;
  if ((pkt.difop.echo_mode & 0x80) == 0x80) {
    resolution = ((pkt.blocks[2].rotation - pkt.blocks[0].rotation + 36000) % 36000) * 10;
  } else {
    resolution = ((pkt.blocks[1].rotation - pkt.blocks[0].rotation + 36000) % 36000) * 10;
  }

  if (resolution == 100) {
    resolution_index = 0;
    laser_degree_per_sec_ = 1800;
  } else if (resolution == 200) {
    resolution_index = 1;
    laser_degree_per_sec_ = 3600;
  } else if (resolution == 300) {
    resolution_index = 2;
    laser_degree_per_sec_ = 5400;
  } else if (resolution == 400) {
    resolution_index = 3;
    laser_degree_per_sec_ = 7200;
  } else {
    return ret;
  }

  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++) {
    if (publish_mode_ == 0 && (pkt.difop.echo_mode & 0x80) == 0x80 && (blk % 2) == 1) {
      continue;
    } else if (publish_mode_ == 1 && (pkt.difop.echo_mode & 0x80) == 0x80 && (blk % 2) == 0) {
      continue;
    }

    const Vanjee733Block &block = pkt.blocks[blk];
    int32_t azimuth_trans = block.rotation * 10;  //(block.rotation * 10 + resolution) % 360000;

    if ((this->split_strategy_->newBlock(azimuth_trans) || (loss_circles_num == 1 && blk == 0 && azimuth_trans != resolution)) &&
        this->point_cloud_->points.size() != 0) {
      int32_t point_gap_num = (azimuth_trans / resolution - 1) * 64;
      this->last_point_ts_ = pkt_ts - all_points_luminous_moment_733_[resolution_index][point_gap_num];
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_733_[resolution_index][all_points_luminous_moment_733_[resolution_index].size() - 1];

      this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
      ret = true;
    }

    double timestamp_point;
    for (uint16_t chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++) {
      float xy, x, y, z;
      const Vanjee733Channel &channel = block.channel[chan];

      float distance = channel.distance * this->const_param_.DISTANCE_RES;
      int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
      int32_t azimuth = block.rotation * 10;
      int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, azimuth, rotate_direction_);
      angle_horiz_final += laser_degree_per_sec_ * (channel_light_moment_733_[chan] - channel_light_moment_733_[0]) * 1e-3;

      int32_t azimuth_index = (angle_horiz_final + 180000 + 360000) % 360000;
      if (rotate_direction_ == RotateDirection::clockwise) {
        azimuth_index = (360000 - azimuth_index) % 360000;
      }
      int32_t verticalVal_733 = angle_vert;
      verticalVal_733 = ((verticalVal_733) + 360000) % 360000;

      uint32_t point_id = (azimuth / resolution - 1) * 64 + chan;
      if (this->param_.ts_first_point == true) {
        timestamp_point = all_points_luminous_moment_733_[resolution_index][point_id];
      } else {
        timestamp_point = all_points_luminous_moment_733_[resolution_index][point_id] -
                          all_points_luminous_moment_733_[resolution_index][all_points_luminous_moment_733_[resolution_index].size() - 1];
      }

      int32_t angle_horiz_mask = azimuth_index;
      if (this->param_.start_angle < this->param_.end_angle) {
        if (angle_horiz_mask < (this->param_.start_angle * 1000) || angle_horiz_mask > (this->param_.end_angle * 1000)) {
          distance = 0;
        }
      } else {
        if (angle_horiz_mask < (this->param_.start_angle * 1000) && angle_horiz_mask > (this->param_.end_angle * 1000)) {
          distance = 0;
        }
      }

      if (this->hide_range_params_.size() > 0 && distance != 0 &&
          this->isValueInRange(chan + this->first_line_id_, angle_horiz_mask / 1000.0, distance, this->hide_range_params_)) {
        distance = 0;
      }

      if (this->distance_section_.in(distance)) {
        xy = distance * COS(verticalVal_733);
        x = xy * COS(azimuth_index);
        y = xy * SIN(azimuth_index);
        z = distance * SIN(verticalVal_733);
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
        setVerAngle(point, verticalVal_733 / 1000.0);
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
        setVerAngle(point, verticalVal_733 / 1000.0);
        setDistance(point, distance);
#endif
        this->point_cloud_->points.emplace_back(point);
      }
    }
    if (azimuth_trans == 360000 && ((pkt.difop.echo_mode & 0x80) != 0x80 || ((pkt.difop.echo_mode & 0x80) == 0x80 && publish_mode_ != 2) ||
                                    ((pkt.difop.echo_mode & 0x80) == 0x80 && publish_mode_ == 2 && azimuth_trans_pre_ == 360000))) {
      this->last_point_ts_ = pkt_ts;
      this->first_point_ts_ =
          this->last_point_ts_ - all_points_luminous_moment_733_[resolution_index][all_points_luminous_moment_733_[resolution_index].size() - 1];
      this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
      ret = true;
    }
    azimuth_trans_pre_ = azimuth_trans;
  }
  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee733<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);
  std::shared_ptr<ProtocolAbstract> p;

  if (*sp_cmd == *(CmdRepository733::CreateInstance()->Sp_LDAngleGet)) {
    p = std::make_shared<Protocol_LDAngleGet733>();
  } else if (*sp_cmd == *(CmdRepository733::CreateInstance()->Sp_MotorParamsGet)) {
    p = std::make_shared<Protocol_MotorParamsGet733>();
  } else {
    return;
  }

  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;

  if (typeid(*params) == typeid(Params_LDAngle733)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    std::shared_ptr<Params_LDAngle733> param = std::dynamic_pointer_cast<Params_LDAngle733>(params);

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
      angle_get_flag_ = true;
      if (angle_get_flag_ && motor_params_get_flag_) {
        Decoder<T_PointCloud>::angles_ready_ = true;
      }
    }
  } else if (typeid(*params) == typeid(Params_MotorParams733)) {
    if (!this->param_.wait_for_difop) {
      if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
        (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      }
      return;
    }

    std::shared_ptr<Params_MotorParams733> param = std::dynamic_pointer_cast<Params_MotorParams733>(params);
    if (param->rotate_direction_ == 0) {
      rotate_direction_ = RotateDirection::anticlockwise;
    } else {
      rotate_direction_ = RotateDirection::clockwise;
    }
    if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr) {
      WJ_INFOL << "Get mortor rotate direction..." << WJ_REND;
      (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      motor_params_get_flag_ = true;
      if (angle_get_flag_ && motor_params_get_flag_) {
        Decoder<T_PointCloud>::angles_ready_ = true;
      }
    }
  } else {
  }
  // if(*sp_cmd == *(CmdRepository733::CreateInstance()->Sp_LDAngleGet))
  // {
  //   Params_LDAngle733 params_LDAngle733;
  //   params_LDAngle733.Load(*protocol);
  //   std::vector<int32_t> vert_angles;
  //   std::vector<int32_t> horiz_angles;
  //   for(int i=0;i<64;i++)
  //   {
  //     vert_angles.emplace_back(params_LDAngle733.VerAngle[i]);
  //     horiz_angles.emplace_back(params_LDAngle733.HorAngle[i]);
  //   }
  //   this->chan_angles_.loadFromLiDAR(this->param_.angle_path_ver ,
  //   param->NumOfLines , vert_angles,horiz_angles);

  //   if(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
  //   {
  //     WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
  //     (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
  //     Decoder<T_PointCloud>::angles_ready_ = true;
  //   }
  //}
}
}  // namespace lidar

}  // namespace vanjee
