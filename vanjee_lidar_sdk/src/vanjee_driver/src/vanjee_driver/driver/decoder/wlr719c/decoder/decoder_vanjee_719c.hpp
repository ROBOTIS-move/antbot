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
#include <vanjee_driver/driver/decoder/wlr719c/protocol/frames/cmd_repository_719c.hpp>
#include <vanjee_driver/driver/decoder/wlr719c/protocol/frames/protocol_heartbeat_tcp.hpp>
#include <vanjee_driver/driver/decoder/wlr719c/protocol/frames/protocol_heartbeat_udp.hpp>
#include <vanjee_driver/driver/decoder/wlr719c/protocol/frames/protocol_scan_data_get.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct _Vanjee719CBlock {
  uint8_t intensity;
  uint16_t distance;
} Vanjee719CBlock;

typedef struct _Vanjee719C20MsopPkt {
  uint8_t header[2];
  uint16_t frame_len;
  uint16_t frame_id;
  uint32_t timestamp;
  uint8_t check_type;
  uint8_t frame_type;
  uint16_t device_type;
  uint16_t remain1;
  uint8_t main_cmd;
  uint8_t sub_cmd;
  uint8_t bank_id;
  uint16_t motor_speed;
  Vanjee719CBlock blocks[300];
  uint32_t second;
  uint32_t subsecond;
  uint8_t remain2;
  uint16_t check;
  uint8_t tail[2];

  void ToLittleEndian() {
    frame_len = ntohs(frame_len);
    frame_id = ntohs(frame_id);
    device_type = ntohs(device_type);
    remain1 = ntohs(remain1);
    motor_speed = ntohs(motor_speed);
    for (int i = 0; i < 300; i++) blocks[i].distance = ntohs(blocks[i].distance);
    second = ntohl(second);
    subsecond = ntohl(subsecond);
    check = ntohs(check);
  }
} Vanjee719C20MsopPkt;

typedef struct _Vanjee719C10And20MsopPkt {
  uint8_t header[2];
  uint16_t frame_len;
  uint16_t frame_id;
  uint32_t timestamp;
  uint8_t check_type;
  uint8_t frame_type;
  uint16_t device_type;
  uint16_t remain1;
  uint8_t main_cmd;
  uint8_t sub_cmd;
  uint8_t bank_id;
  uint16_t motor_speed;
  Vanjee719CBlock blocks[450];
  uint32_t second;
  uint32_t subsecond;
  uint8_t remain2;
  uint16_t check;
  uint8_t tail[2];

  void ToLittleEndian() {
    frame_len = ntohs(frame_len);
    frame_id = ntohs(frame_id);
    device_type = ntohs(device_type);
    remain1 = ntohs(remain1);
    motor_speed = ntohs(motor_speed);
    for (int i = 0; i < 450; i++) blocks[i].distance = ntohs(blocks[i].distance);
    second = ntohl(second);
    subsecond = ntohl(subsecond);
    check = ntohs(check);
  }
} Vanjee719C10And20MsopPkt;
#pragma pack(pop)

template <typename T_PointCloud>
class DecoderVanjee719C : public DecoderMech<T_PointCloud> {
 private:
  std::vector<std::vector<double>> all_points_luminous_moment_719c_;  // Cache 4 channels with a circular
                                                                      // point cloud time difference
  const double luminous_period_of_ld_ = 0.000055555;                  // Time interval at adjacent horizontal angles
  const double luminous_period_of_adjacent_ld_ = 0.000013888;         // Time interval between adjacent vertical angles within the
                                                                      // group

  int32_t azimuth_cur_ = -1.0;
  int32_t pre_frame_id_ = -1;
  uint8_t pre_bank_id_ = 0;

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam &getConstParam(uint8_t mode);
  ChanAngles chan_angles_;

  std::vector<uint8_t> buf_cache_;

  void initLdLuminousMoment(void);

 public:
  constexpr static double FRAME_DURATION = 0.05;
  constexpr static uint32_t SINGLE_PKT_NUM = 28;
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee719C() = default;
  explicit DecoderVanjee719C(const WJDecoderParam &param);
  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  bool decodeMsopPkt_20HzOld(const uint8_t *pkt, size_t size);
  bool decodeMsopPkt_10HzAnd20Hz(const uint8_t *pkt, size_t size);
};

template <typename T_PointCloud>
inline WJDecoderMechConstParam &DecoderVanjee719C<T_PointCloud>::getConstParam(uint8_t mode) {
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
inline DecoderVanjee719C<T_PointCloud>::DecoderVanjee719C(const WJDecoderParam &param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param), chan_angles_(this->const_param_.LASER_NUM) {
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);
  initLdLuminousMoment();
}

template <typename T_PointCloud>
void DecoderVanjee719C<T_PointCloud>::initLdLuminousMoment() {
  double offset = 0;
  all_points_luminous_moment_719c_.resize(2);
  all_points_luminous_moment_719c_[0].resize(7200);
  all_points_luminous_moment_719c_[1].resize(3600);

  for (uint16_t col = 0; col < 1800; col++) {
    for (uint8_t row = 0; row < 4; row++) {
      offset = row * luminous_period_of_adjacent_ld_;

      if (col < 900) {
        for (int i = 0; i < 2; i++) all_points_luminous_moment_719c_[i][col * 4 + row] = col * luminous_period_of_ld_ + offset;
      } else {
        all_points_luminous_moment_719c_[0][col * 4 + row] = col * luminous_period_of_ld_ + offset;
      }
    }
  }
}

template <typename T_PointCloud>
inline bool DecoderVanjee719C<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size) {
  bool ret = false;
  std::vector<uint8_t> data;
  if (buf_cache_.size() > 0) {
    std::copy(buf_cache_.begin(), buf_cache_.end(), std::back_inserter(data));
    std::copy(pkt, pkt + size, std::back_inserter(data));
  } else {
    std::copy(pkt, pkt + size, std::back_inserter(data));
  }

  buf_cache_.clear();
  buf_cache_.shrink_to_fit();

  uint32 index_last = 0;
  for (size_t i = 0; i < data.size(); i++) {
    if (data.size() - i < 4)
      break;
    if (!(data[i] == 0xff && data[i + 1] == 0xaa)) {
      index_last = i + 1;
      continue;
    }

    uint16_t frameLen = ((data[i + 2] << 8) | data[i + 3]) + 4;

    if (i + frameLen > data.size())
      break;

    if (!(data[i + frameLen - 2] == 0xee && data[i + frameLen - 1] == 0xee)) {
      index_last = i + 1;
      continue;
    }

    if (frameLen == 934) {
      uint8_t pkt[934];
      memcpy(pkt, &data[i], 934);
      if (decodeMsopPkt_20HzOld(pkt, 934)) {
        ret = true;
        index_last = i + frameLen;
        break;
      }
    } else if (frameLen == 1384) {
      uint8_t pkt[1384];
      memcpy(pkt, &data[i], 1384);
      if (decodeMsopPkt_10HzAnd20Hz(pkt, 1384)) {
        ret = true;
        index_last = i + frameLen;
        break;
      }
    }
    i += frameLen - 1;
    index_last = i + 1;
  }

  if (index_last < data.size()) {
    buf_cache_.assign(data.begin() + index_last, data.end());
  }

  return ret;
}

template <typename T_PointCloud>
inline bool DecoderVanjee719C<T_PointCloud>::decodeMsopPkt_20HzOld(const uint8_t *packet, size_t size) {
  Vanjee719C20MsopPkt &pkt = (*(Vanjee719C20MsopPkt *)packet);
  pkt.ToLittleEndian();

  bool ret = false;
  double pkt_ts = 0;

  int32_t loss_packets_num = (pkt.frame_id + 65536 - pre_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_frame_id_ = pkt.frame_id;

  if (!this->param_.use_lidar_clock)
    pkt_ts = getTimeHost() * 1e-6;
  else {
    double gapTime1900_1970 = (25567LL * 24 * 3600);
    pkt_ts = (double)pkt.second - gapTime1900_1970 + ((double)pkt.subsecond * 0.23283 * 1e-9);
    if (pkt_ts < 3600)
      pkt_ts = 3600;
  }

  uint8_t line_num = 4;
  int32_t resolution_index = 1;
  uint16_t point_num = 3600;
  uint8_t bank_num = 12;

  bool publish_flag = false;
  if (pre_bank_id_ != 0) {
    if (loss_packets_num >= 2 * bank_num) {
      this->scan_data_->ranges.clear();
      this->scan_data_->intensities.clear();
      this->point_cloud_->points.clear();
      publish_flag = true;
    } else if (loss_packets_num > bank_num && loss_packets_num < 2 * bank_num) {
      if (pkt.bank_id <= loss_packets_num - bank_num) {
        this->scan_data_->ranges.clear();
        this->scan_data_->intensities.clear();
        this->point_cloud_->points.clear();
      }
      publish_flag = true;
    } else if (loss_packets_num > 1 && loss_packets_num <= bank_num && pkt.bank_id <= loss_packets_num - 1) {
      publish_flag = true;
    }
  }
  pre_bank_id_ = pkt.bank_id;

  azimuth_cur_ = (pkt.bank_id - 1) * (300 * 360000 / point_num);

  if ((this->split_strategy_->newBlock((int64_t)pkt.bank_id % bank_num) && (int64_t)pkt.bank_id != bank_num) || publish_flag) {
    // pointcloud
    if (!this->param_.use_lidar_clock)
      this->first_point_ts_ = pkt_ts - all_points_luminous_moment_719c_[resolution_index][300 * pkt.bank_id - 1] - 0.05;
    else
      this->first_point_ts_ = this->prev_pkt_ts_;
    this->last_point_ts_ =
        this->first_point_ts_ + all_points_luminous_moment_719c_[resolution_index][all_points_luminous_moment_719c_[resolution_index].size() - 1];

    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
    ret = true;
  }

  double timestamp_point;
  for (uint16_t point_index = 0; point_index < 300; point_index++) {
    uint16_t chan_id = (point_index % line_num) + this->first_line_id_;
    int32_t azimuth = (azimuth_cur_ + (point_index / line_num) * (360000 * line_num / point_num)) % 360000;

    uint32_t point_id = 300 * (pkt.bank_id - 1) + point_index;
    if (this->param_.ts_first_point == true) {
      timestamp_point = all_points_luminous_moment_719c_[resolution_index][point_id];
    } else {
      timestamp_point = all_points_luminous_moment_719c_[resolution_index][point_id] -
                        all_points_luminous_moment_719c_[resolution_index][all_points_luminous_moment_719c_[resolution_index].size() - 1];
    }

    float x, y, z, xy;
    float distance = pkt.blocks[point_index].distance / 1000.0;
    float intensity = pkt.blocks[point_index].intensity & 0x7f;
    int32_t angle_vert[4] = {350000, 355000, 0, 300};
    int32_t angle_horiz_final = azimuth;

    if (angle_horiz_final < 0) {
      angle_horiz_final += 360000;
    }

    int32_t azimuth_index = (angle_horiz_final + 180000) % 360000;

    int32_t angle_horiz_mask = (360000 - azimuth_index) % 360000;
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
        this->isValueInRange(chan_id, angle_horiz_mask / 1000.0, distance, this->hide_range_params_)) {
      distance = 0;
    }

    int32_t verticalVal_719c = angle_vert[point_index % line_num];

    typename T_PointCloud::PointT point;
    if (this->distance_section_.in(distance)) {
      xy = distance * COS(verticalVal_719c);
      x = xy * COS(azimuth_index);
      y = -xy * SIN(azimuth_index);
      z = distance * SIN(verticalVal_719c);
      this->transformPoint(x, y, z);

      setX(point, x);
      setY(point, y);
      setZ(point, z);
      setIntensity(point, intensity);
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
      setIntensity(point, 0.0);
    }
    setTimestamp(point, timestamp_point);
    setRing(point, chan_id);
#ifdef ENABLE_GTEST
    setPointId(point, point_id);
    setHorAngle(point, azimuth_index / 1000.0);
    setVerAngle(point, verticalVal_719c / 1000.0);
    setDistance(point, distance);
#endif
    this->point_cloud_->points.emplace_back(point);
  }

  if ((int64_t)pkt.bank_id == bank_num) {
    // pointcloud
    if (!this->param_.use_lidar_clock)
      this->first_point_ts_ = pkt_ts - all_points_luminous_moment_719c_[resolution_index][300 * pkt.bank_id - 1];
    else
      this->first_point_ts_ = pkt_ts;
    this->last_point_ts_ =
        this->first_point_ts_ + all_points_luminous_moment_719c_[resolution_index][all_points_luminous_moment_719c_[resolution_index].size() - 1];

    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
    ret = true;
  }

  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

template <typename T_PointCloud>
inline bool DecoderVanjee719C<T_PointCloud>::decodeMsopPkt_10HzAnd20Hz(const uint8_t *packet, size_t size) {
  Vanjee719C10And20MsopPkt &pkt = (*(Vanjee719C10And20MsopPkt *)packet);
  pkt.ToLittleEndian();

  bool ret = false;
  double pkt_ts = 0;

  int32_t loss_packets_num = (pkt.frame_id + 65536 - pre_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_frame_id_ = pkt.frame_id;

  if (!this->param_.use_lidar_clock)
    pkt_ts = getTimeHost() * 1e-6;
  else {
    double gapTime1900_1970 = (25567LL * 24 * 3600);
    pkt_ts = (double)pkt.second - gapTime1900_1970 + ((double)pkt.subsecond * 0.23283 * 1e-9);
    if (pkt_ts < 3600)
      pkt_ts = 3600;
  }

  uint8_t line_num = 4;
  int32_t resolution_index = 0;
  uint8_t frequency = 10;
  uint16_t point_num = 7200;
  uint8_t bank_num = 12;
  if (pkt.sub_cmd == 0x03 || pkt.sub_cmd == 0x04) {
    bank_num = 16;
    point_num = 7200;
    frequency = 10;
    resolution_index = 0;
  } else if (pkt.sub_cmd == 0x05 || pkt.sub_cmd == 0x06) {
    bank_num = 8;
    point_num = 3600;
    frequency = 20;
    resolution_index = 1;
  } else {
    return ret;
  }

  bool publish_flag = false;
  if (pre_bank_id_ != 0) {
    if (loss_packets_num >= 2 * bank_num) {
      this->scan_data_->ranges.clear();
      this->scan_data_->intensities.clear();
      this->point_cloud_->points.clear();
      publish_flag = true;
    } else if (loss_packets_num > bank_num && loss_packets_num < 2 * bank_num) {
      if (pkt.bank_id <= loss_packets_num - bank_num) {
        this->scan_data_->ranges.clear();
        this->scan_data_->intensities.clear();
        this->point_cloud_->points.clear();
      }
      publish_flag = true;
    } else if (loss_packets_num > 1 && loss_packets_num <= bank_num && pkt.bank_id <= loss_packets_num - 1) {
      publish_flag = true;
    }
  }
  pre_bank_id_ = pkt.bank_id;

  int32_t first_pack_angle = ((int32_t)(450 / line_num) + 1) * (360000 / (point_num / line_num));
  int32_t second_pack_angle = (int32_t)(450 / line_num) * (360000 / (point_num / line_num));
  int divid_bank_num = pkt.bank_id / 2;
  if (pkt.bank_id == 1)
    azimuth_cur_ = 0;
  else {
    if (pkt.bank_id % 2 == 1)
      azimuth_cur_ = divid_bank_num * first_pack_angle + divid_bank_num * second_pack_angle;
    else
      azimuth_cur_ = (divid_bank_num - 1) * first_pack_angle + divid_bank_num * second_pack_angle;
  }

  if ((this->split_strategy_->newBlock((int64_t)pkt.bank_id % bank_num) && (int64_t)pkt.bank_id != bank_num) || publish_flag) {
    // pointcloud
    if (!this->param_.use_lidar_clock)
      this->first_point_ts_ = pkt_ts - all_points_luminous_moment_719c_[resolution_index][450 * pkt.bank_id - 1] - (1.0 / frequency);
    else
      this->first_point_ts_ = this->prev_pkt_ts_;
    this->last_point_ts_ =
        this->first_point_ts_ + all_points_luminous_moment_719c_[resolution_index][all_points_luminous_moment_719c_[resolution_index].size() - 1];

    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
    ret = true;
  }

  double timestamp_point;
  for (uint16_t point_index = 0; point_index < 450; point_index++) {
    uint16_t chan_id = 0;
    if (pkt.bank_id % 2 == 1)
      chan_id = (point_index % line_num) + this->first_line_id_;
    else
      chan_id = ((point_index + 2) % line_num) + this->first_line_id_;

    int32_t azimuth = 0;
    if (pkt.bank_id % 2 == 1)
      azimuth = (azimuth_cur_ + (point_index / line_num) * (360000 * line_num / point_num)) % 360000;
    else
      azimuth = (azimuth_cur_ + ((point_index + 2) / line_num) * (360000 * line_num / point_num)) % 360000;

    uint32_t point_id = 450 * (pkt.bank_id - 1) + point_index;
    if (this->param_.ts_first_point == true) {
      timestamp_point = all_points_luminous_moment_719c_[resolution_index][point_id];
    } else {
      timestamp_point = all_points_luminous_moment_719c_[resolution_index][point_id] -
                        all_points_luminous_moment_719c_[resolution_index][all_points_luminous_moment_719c_[resolution_index].size() - 1];
    }

    float x, y, z, xy;
    float distance = pkt.blocks[point_index].distance / 1000.0;
    float intensity = pkt.blocks[point_index].intensity & 0x7f;
    int32_t angle_vert[4] = {350000, 355000, 0, 300};
    int32_t angle_horiz_final = azimuth;

    if (angle_horiz_final < 0) {
      angle_horiz_final += 360000;
    }

    int32_t azimuth_index = (angle_horiz_final + 180000) % 360000;

    int32_t angle_horiz_mask = (360000 - azimuth_index) % 360000;
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
        this->isValueInRange(chan_id, angle_horiz_mask / 1000.0, distance, this->hide_range_params_)) {
      distance = 0;
    }

    int32_t verticalVal_719c = 0;
    if (pkt.bank_id % 2 == 1)
      verticalVal_719c = angle_vert[point_index % line_num];
    else
      verticalVal_719c = angle_vert[(point_index + 2) % line_num];

    if (this->distance_section_.in(distance)) {
      xy = distance * COS(verticalVal_719c);
      x = xy * COS(azimuth_index);
      y = -xy * SIN(azimuth_index);
      z = distance * SIN(verticalVal_719c);
      this->transformPoint(x, y, z);

      typename T_PointCloud::PointT point;
      setX(point, x);
      setY(point, y);
      setZ(point, z);
      setIntensity(point, intensity);
      setTimestamp(point, timestamp_point);
      setRing(point, chan_id);
#ifdef ENABLE_GTEST
      setPointId(point, point_id);
      setHorAngle(point, azimuth_index / 1000.0);
      setVerAngle(point, verticalVal_719c / 1000.0);
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

      setIntensity(point, 0.0);
      setTimestamp(point, timestamp_point);
      setRing(point, chan_id);
#ifdef ENABLE_GTEST
      setPointId(point, point_id);
      setHorAngle(point, azimuth_index / 1000.0);
      setVerAngle(point, verticalVal_719c / 1000.0);
      setDistance(point, distance);
#endif
      this->point_cloud_->points.emplace_back(point);
    }
  }

  if ((int64_t)pkt.bank_id == bank_num) {
    // pointcloud
    if (!this->param_.use_lidar_clock)
      this->first_point_ts_ = pkt_ts - all_points_luminous_moment_719c_[resolution_index][450 * pkt.bank_id - 1];
    else
      this->first_point_ts_ = pkt_ts;
    this->last_point_ts_ =
        this->first_point_ts_ + all_points_luminous_moment_719c_[resolution_index][all_points_luminous_moment_719c_[resolution_index].size() - 1];

    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
    ret = true;
  }

  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee719C<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<ProtocolAbstract719C> p;
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

  if (*sp_cmd == *(CmdRepository719C::CreateInstance()->Sp_ScanDataGet)) {
    p = std::make_shared<Protocol_ScanDataGet719C>();
  } else if (*sp_cmd == *(CmdRepository719C::CreateInstance()->Sp_HeartBeat_Tcp)) {
    p = std::make_shared<Protocol_HeartBeat719CTcp>();
  } else if (*sp_cmd == *(CmdRepository719C::CreateInstance()->Sp_HeartBeat_Udp)) {
    p = std::make_shared<Protocol_HeartBeat719CUdp>();
  } else {
    return;
  }
  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Params_ScanData719C)) {
    std::shared_ptr<Params_ScanData719C> param = std::dynamic_pointer_cast<Params_ScanData719C>(params);
    // if (param->data_get_flag)
    // {
    //     WJ_INFOL << "get wlr719 scan data succ" << WJ_REND;
    //     (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
    // }
    // else
    // {
    //     WJ_INFOL << "get wlr719 scan data fail" << WJ_REND;
    // }

  } else if (typeid(*params) == typeid(Params_HeartBeat719CTcp)) {
    std::shared_ptr<Params_HeartBeat719CTcp> param = std::dynamic_pointer_cast<Params_HeartBeat719CTcp>(params);
    // if (param->heartbeat_flag_)
    // {
    //     WJ_INFOL << "get wlr719 heartbeat succ" << WJ_REND;
    // }
    // else
    // {
    //     WJ_INFOL << "get wlr719 scan data fail" << WJ_REND;
    // }
  } else if (typeid(*params) == typeid(Params_HeartBeat719CUdp)) {
    std::shared_ptr<Params_HeartBeat719CUdp> param = std::dynamic_pointer_cast<Params_HeartBeat719CUdp>(params);
    // if (param->heartbeat_flag_)
    // {
    //     WJ_INFOL << "get wlr719 heartbeat succ" << WJ_REND;
    // }
    // else
    // {
    //     WJ_INFOL << "get wlr719 scan data fail" << WJ_REND;
    // }
  } else {
    WJ_WARNING << "Unknown Params Type..." << WJ_REND;
  }
}

}  // namespace lidar
}  // namespace vanjee