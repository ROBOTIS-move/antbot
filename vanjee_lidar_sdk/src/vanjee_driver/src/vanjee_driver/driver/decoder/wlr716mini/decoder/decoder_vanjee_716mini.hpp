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
#include <vanjee_driver/driver/decoder/wlr716mini/protocol/frames/cmd_repository_716mini.hpp>
#include <vanjee_driver/driver/decoder/wlr716mini/protocol/frames/protocol_device_operate_params_get.hpp>
#include <vanjee_driver/driver/decoder/wlr716mini/protocol/frames/protocol_scan_data_get.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

namespace vanjee {
namespace lidar {

enum ProtocolType716mini { NON = 0, UNIVERSAL = 1, HIK };

#pragma pack(push, 1)

typedef struct _Vanjee716miniMsopPkt {
  uint8_t header[2];
  uint16_t frame_len;
  uint16_t frame_id;
  uint32_t timestamp;
  uint8_t check_type;
  uint8_t frame_type;
  uint16_t device_type;
  uint32_t second;
  uint32_t subsecond;
  uint8_t main_cmd;
  uint8_t sub_cmd;
  uint8_t cmd_param1;
  uint8_t cmd_param2;

  uint16_t device_status;
  uint16_t watchdog_reset_num;
  uint16_t software_reset_num;
  uint16_t loss_elec_reset_num;
  uint16_t detected_encoder_groove_count_per_circle;
  uint32_t zero_point_pulsewidth;
  uint16_t zero_point_distance;
  uint16_t motor_speed;
  uint16_t tcp_crc_error;
  uint8_t critical;
  uint16_t link_disconnect_num;
  uint16_t reconnect_num;
  uint16_t disconnect_num;
  uint16_t clog_num;
  uint16_t resend_failure_num;
  uint16_t heartbeat_disconnect_num;
  uint16_t keepalive_disconnect_num;
  uint16_t error_disconnect_num;
  uint16_t connect_reset_num;
  uint8_t intensity_measurement_method;
  uint8_t intensity_measurement_method_of_cur_pkt;
  uint8_t remain[4];
  uint8_t event_switch_mode;
  uint8_t active_event_id;
  uint8_t input_IO_value;
  uint8_t zone_output_value;
  uint32_t circle_id;
  uint8_t frequency;
  uint8_t total_pkts_num;
  uint8_t pkt_id;
  uint8_t pkt_type;
  uint16_t points_num;

  void ToLittleEndian() {
    frame_len = ntohs(frame_len);
    frame_id = ntohs(frame_id);
    timestamp = ntohl(timestamp);
    device_type = ntohs(device_type);
    second = ntohl(second);
    subsecond = ntohl(subsecond);

    device_status = ntohs(device_status);
    watchdog_reset_num = ntohs(watchdog_reset_num);
    software_reset_num = ntohs(software_reset_num);
    loss_elec_reset_num = ntohs(loss_elec_reset_num);
    detected_encoder_groove_count_per_circle = ntohs(detected_encoder_groove_count_per_circle);
    zero_point_pulsewidth = ntohl(zero_point_pulsewidth);
    zero_point_distance = ntohs(zero_point_distance);
    motor_speed = ntohs(motor_speed);
    tcp_crc_error = ntohs(tcp_crc_error);
    link_disconnect_num = ntohs(link_disconnect_num);
    reconnect_num = ntohs(reconnect_num);
    disconnect_num = ntohs(disconnect_num);
    clog_num = ntohs(clog_num);
    resend_failure_num = ntohs(resend_failure_num);
    heartbeat_disconnect_num = ntohs(heartbeat_disconnect_num);
    keepalive_disconnect_num = ntohs(keepalive_disconnect_num);
    error_disconnect_num = ntohs(error_disconnect_num);
    connect_reset_num = ntohs(connect_reset_num);
    circle_id = ntohl(circle_id);
    points_num = ntohs(points_num);
  }
} Vanjee716miniMsopPkt;
#pragma pack(pop)

typedef struct _Vanjee716miniPointDXYZIRT {
  float the_1st_echo_distance;
  float the_1st_echo_x;
  float the_1st_echo_y;
  float the_1st_echo_z;
  float the_2nd_echo_distance;
  float the_2nd_echo_x;
  float the_2nd_echo_y;
  float the_2nd_echo_z;
  float intensity;
  double timestamp;
  int ring;
} Vanjee716miniPointDXYZIRT;

typedef struct _Vanjee716miniPacket {
  double timestamp;
  uint32_t circle_id;
  uint8_t frequency;
  uint8_t total_pkts_num;
  uint8_t pkt_id;
  uint8_t pkt_type;
  uint16_t points_num;
  std::vector<uint8_t> pkt_data;
} Vanjee716miniPacket;

template <typename T_PointCloud>
class DecoderVanjee716Mini : public DecoderMech<T_PointCloud> {
 private:
  std::vector<double> all_points_luminous_moment_716mini_;  // Cache a circle of point cloud
                                                            // time difference

  bool scan_data_recv_flag_ = false;
  double pkt_ts_ = 0;
  uint32_t pkt_id_mask_ = 0;
  uint32_t circle_id_of_pre_pkt_ = 0;
  int32_t pre_frame_id_ = -1;
  uint32_t pre_point_cloud_size_ = 0;
  int32_t pre_frequency_ = -1;

  uint8_t protocol_type_ = 0;

  std::vector<uint8_t> buf_cache_;

  std::shared_ptr<SplitStrategy> split_strategy_;
  static WJDecoderMechConstParam &getConstParam(uint8_t mode);
  ChanAngles chan_angles_;

  std::vector<Vanjee716miniPacket> point_cloud_packet;
  std::vector<Vanjee716miniPointDXYZIRT> point_cloud_value_;
  void initLdLuminousMoment(uint32_t point_cloud_size, double circle_time);
  bool lidarParamGet(uint8_t frequency, uint8_t pkt_type, uint8_t total_pkts_num, uint16_t points_num, uint32_t &point_cloud_size,
                     double &circle_time, double &resolution);
  void setPointsValue(const uint8_t *points_buf, uint8_t pkt_type, uint8_t pkg_no, uint8_t pkt_num, uint16_t points_num, double resolution);

 public:
  constexpr static double FRAME_DURATION = 0.066666667;
  constexpr static uint32_t SINGLE_PKT_NUM = 4;
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
  virtual ~DecoderVanjee716Mini() = default;
  explicit DecoderVanjee716Mini(const WJDecoderParam &param);
  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
  bool decodeMsopPkt_1(const uint8_t *pkt, size_t size);
};

template <typename T_PointCloud>
inline WJDecoderMechConstParam &DecoderVanjee716Mini<T_PointCloud>::getConstParam(uint8_t mode) {
  // WJDecoderConstParam
  // WJDecoderMechConstParam
  static WJDecoderMechConstParam param = {
      1289  // msop len
      ,
      1  // laser number
      ,
      500  // blocks per packet
      ,
      1  // channels per block
      ,
      0.1f  // distance min
      ,
      30.0f  // distance max
      ,
      0.001f  // distance resolution
      ,
      80.0f  // initial value of temperature
  };
  param.BLOCK_DURATION = 0.1 / 360;
  return param;
}

template <typename T_PointCloud>
inline DecoderVanjee716Mini<T_PointCloud>::DecoderVanjee716Mini(const WJDecoderParam &param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param), chan_angles_(this->const_param_.LASER_NUM) {
  if (param.max_distance < param.min_distance)
    WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;

  this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
  split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);
}

template <typename T_PointCloud>
void DecoderVanjee716Mini<T_PointCloud>::initLdLuminousMoment(uint32_t point_cloud_size, double circle_time) {
  // time of 270 degree
  double luminous_period_of_ld = circle_time * 0.75 / point_cloud_size;
  all_points_luminous_moment_716mini_.resize(point_cloud_size);

  for (size_t col = 0; col < point_cloud_size; col++) {
    all_points_luminous_moment_716mini_[col] = col * luminous_period_of_ld;
  }
}

template <typename T_PointCloud>
bool DecoderVanjee716Mini<T_PointCloud>::lidarParamGet(uint8_t frequency, uint8_t pkt_type, uint8_t total_pkts_num, uint16_t points_num,
                                                       uint32_t &point_cloud_size, double &circle_time, double &resolution) {
  if (pkt_type == 0)
    point_cloud_size = (total_pkts_num - 1) * 600 + points_num;
  else if (pkt_type == 1)
    point_cloud_size = (total_pkts_num / 2 - 1) * 600 + points_num;
  else
    return false;

  resolution = 270.0 / (point_cloud_size - 1);

  if (frequency == 0)
    circle_time = 0.1;
  else if (frequency == 1)
    circle_time = 0.0666666666;
  else if (frequency == 2)
    circle_time = 0.04;
  else if (frequency == 3)
    circle_time = 0.028571428;
  else if (frequency == 4)
    circle_time = 0.333333333;
  else
    circle_time = 1.0 / frequency;

  return true;
}

template <typename T_PointCloud>
inline bool DecoderVanjee716Mini<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size) {
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

  uint32 indexLast = 0;
  for (size_t i = 0; i < data.size(); i++) {
    if (data.size() - i < 4)
      break;
    if (!(data[i] == 0xff && data[i + 1] == 0xaa)) {
      indexLast = i + 1;
      continue;
    }

    uint16_t frameLen = ((data[i + 2] << 8) | data[i + 3]) + 4;

    if (i + frameLen > data.size())
      break;

    if (!(data[i + frameLen - 2] == 0xee && data[i + frameLen - 1] == 0xee)) {
      indexLast = i + 1;
      continue;
    }

    if (frameLen <= data.size() - i) {
      if (decodeMsopPkt_1(&data[i], frameLen)) {
        ret = true;
        indexLast = i + frameLen;
        break;
      }
    }

    i += frameLen - 1;
    indexLast = i + 1;
  }

  if (indexLast < data.size()) {
    buf_cache_.assign(data.begin() + indexLast, data.end());
  }

  return ret;
}

template <typename T_PointCloud>
inline void DecoderVanjee716Mini<T_PointCloud>::setPointsValue(const uint8_t *points_buf, uint8_t pkt_type, uint8_t pkg_no, uint8_t pkt_num,
                                                               uint16_t points_num, double resolution) {
  if (point_cloud_value_.size() == 0) {
    point_cloud_value_.resize(all_points_luminous_moment_716mini_.size());
  }
  uint32_t point_id = 0;
  if (pkt_type == 0) {
    double timestamp_point;
    for (size_t i = 0; i < points_num; i++) {
      if (protocol_type_ == (uint8_t)ProtocolType716mini::HIK) {
        point_id = (pkg_no / 2) * 600 + i;
      } else {
        point_id = (pkg_no - 1) * 600 + i;
      }
      if (this->param_.ts_first_point == true) {
        timestamp_point = all_points_luminous_moment_716mini_[point_id];
      } else {
        timestamp_point =
            all_points_luminous_moment_716mini_[point_id] - all_points_luminous_moment_716mini_[all_points_luminous_moment_716mini_.size() - 1];
      }

      int32_t angle = (int32_t)((225 + point_id * resolution) * 1000) % 360000;
      float distance = ((points_buf[2 * i] << 8) + points_buf[2 * i + 1]) / 1000.0;
      if (this->param_.start_angle < this->param_.end_angle) {
        if (angle < this->param_.start_angle * 1000 || angle > this->param_.end_angle * 1000) {
          distance = 0;
        }
      } else {
        if (angle > this->param_.end_angle * 1000 && angle < this->param_.start_angle * 1000) {
          distance = 0;
        }
      }

      if (this->hide_range_params_.size() > 0 && distance != 0 &&
          this->isValueInRange(this->first_line_id_, angle / 1000.0, distance, this->hide_range_params_)) {
        distance = 0;
      }

      point_cloud_value_[point_id].the_1st_echo_distance = distance;
      point_cloud_value_[point_id].the_1st_echo_x = distance * COS(angle);
      point_cloud_value_[point_id].the_1st_echo_y = distance * SIN(angle);
      point_cloud_value_[point_id].the_1st_echo_z = 0.0;
      // point_cloud_value_[point_id].the_2nd_echo_distance = 0.0;
      // point_cloud_value_[point_id].the_2nd_echo_x = 0.0;
      // point_cloud_value_[point_id].the_2nd_echo_y = 0.0;
      // point_cloud_value_[point_id].the_2nd_echo_z = 0.0;
      // point_cloud_value_[point_id].intensity = 0;
      point_cloud_value_[point_id].timestamp = timestamp_point;
      point_cloud_value_[point_id].ring = this->first_line_id_;
    }
  } else if (pkt_type == 1) {
    for (size_t i = 0; i < points_num; i++) {
      if (protocol_type_ == (uint8_t)ProtocolType716mini::HIK) {
        point_id = (pkg_no / 2 - 1) * 600 + i;
      } else {
        point_id = (pkg_no - (pkt_num / 2) - 1) * 600 + i;
      }
      point_cloud_value_[point_id].intensity = (points_buf[2 * i] << 8) + points_buf[2 * i + 1];
    }
  }
}

template <typename T_PointCloud>
inline bool DecoderVanjee716Mini<T_PointCloud>::decodeMsopPkt_1(const uint8_t *packet, size_t size) {
  bool ret = false;
  Vanjee716miniMsopPkt &pkt = (*(Vanjee716miniMsopPkt *)packet);
  pkt.ToLittleEndian();

  if (pkt.main_cmd != 0x02 || pkt.sub_cmd != 0x02)
    return false;
  if (protocol_type_ == (uint8_t)ProtocolType716mini::NON) {
    if (pkt.pkt_id == 2 && pkt.pkt_type == 0) {
      protocol_type_ = (uint8_t)ProtocolType716mini::UNIVERSAL;
    } else if (pkt.pkt_id == 2 && pkt.pkt_type == 1) {
      protocol_type_ = (uint8_t)ProtocolType716mini::HIK;
    } else if (pkt.pkt_id != 1) {
      return false;
    }
  }

  int32_t loss_packets_num = (pkt.frame_id + 65536 - pre_frame_id_) % 65536;
  if (loss_packets_num > 1 && pre_frame_id_ >= 0)
    WJ_WARNING << "loss " << (loss_packets_num - 1) << " packets" << WJ_REND;
  pre_frame_id_ = pkt.frame_id;

  double pkt_ts = 0.0;
  if (!this->param_.use_lidar_clock)
    pkt_ts = getTimeHost() * 1e-6;
  else {
    double gapTime1900_1970 = (25567LL * 24 * 3600);
    pkt_ts = (double)pkt.second + ((double)pkt.subsecond * 0.23283 * 1e-9) - gapTime1900_1970;
    if (pkt_ts < 3600) {
      pkt_ts == 3600;
    }
  }

  if (this->param_.device_ctrl_state_enable && pkt.device_status == 0x0001 || pkt.device_status == 0x0002) {
    this->device_ctrl_->cmd_id = 0x0100;
    this->device_ctrl_->cmd_param = pkt.device_status;
    this->device_ctrl_->cmd_state = (uint8_t)pkt.device_status;
    this->cb_device_ctrl_state_(pkt_ts);
  }

  if (pkt.circle_id != circle_id_of_pre_pkt_) {
    if (pkt.circle_id > circle_id_of_pre_pkt_)
      point_cloud_packet.clear();
    else {
      if (circle_id_of_pre_pkt_ - pkt.circle_id < 10)
        return ret;
      else
        point_cloud_packet.clear();
    }
  }

  Vanjee716miniPacket vanjee716miniPacket;
  vanjee716miniPacket.timestamp = pkt_ts;
  vanjee716miniPacket.circle_id = pkt.circle_id;
  vanjee716miniPacket.frequency = pkt.frequency;
  vanjee716miniPacket.total_pkts_num = pkt.total_pkts_num;
  vanjee716miniPacket.pkt_id = pkt.pkt_id;
  vanjee716miniPacket.pkt_type = pkt.pkt_type;
  vanjee716miniPacket.points_num = pkt.points_num;
  vanjee716miniPacket.pkt_data.resize(size);
  std::copy(packet, packet + size, vanjee716miniPacket.pkt_data.begin());

  if (point_cloud_packet.size() >= pkt.total_pkts_num)
    point_cloud_packet.clear();
  point_cloud_packet.push_back(vanjee716miniPacket);

  if (point_cloud_packet.size() == pkt.total_pkts_num) {
    uint32_t all_pkt_mask = 0;
    for (size_t i = 0; i < pkt.total_pkts_num; i++) all_pkt_mask |= (0x01 << i);
    pkt_id_mask_ = 0;
    point_cloud_value_.clear();

    uint32_t point_cloud_size = 0;
    double resolution = -1;
    double circle_time = 0.0;

    pkt_ts_ = point_cloud_packet[0].timestamp;
    std::sort(point_cloud_packet.begin(), point_cloud_packet.end(), [](const Vanjee716miniPacket &a, const Vanjee716miniPacket &b) {
      return a.pkt_id < b.pkt_id;  // Sort in ascending order
    });

    if (lidarParamGet(point_cloud_packet[point_cloud_packet.size() - 1].frequency, point_cloud_packet[point_cloud_packet.size() - 1].pkt_type,
                      point_cloud_packet[point_cloud_packet.size() - 1].total_pkts_num, point_cloud_packet[point_cloud_packet.size() - 1].points_num,
                      point_cloud_size, circle_time, resolution)) {
      if (point_cloud_packet[point_cloud_packet.size() - 1].frequency != pre_frequency_ || point_cloud_size != pre_point_cloud_size_)
        initLdLuminousMoment(point_cloud_size, circle_time);
    } else
      return false;
    pre_point_cloud_size_ = point_cloud_size;
    pre_frequency_ = point_cloud_packet[point_cloud_packet.size() - 1].frequency;

    for (size_t i = 0; i < point_cloud_packet.size(); i++) {
      setPointsValue(&point_cloud_packet[i].pkt_data[85], point_cloud_packet[i].pkt_type, point_cloud_packet[i].pkt_id,
                     point_cloud_packet[i].total_pkts_num, point_cloud_packet[i].points_num, resolution);
      pkt_id_mask_ = pkt_id_mask_ | (0x01 << (point_cloud_packet[i].pkt_id - 1));
    }

    if ((pkt_id_mask_ & all_pkt_mask) == all_pkt_mask) {
      this->first_point_ts_ = pkt_ts_ - all_points_luminous_moment_716mini_[all_points_luminous_moment_716mini_.size() - 1];
      this->last_point_ts_ = pkt_ts_;

      if (this->param_.point_cloud_enable) {
        for (size_t i = 0; i < point_cloud_value_.size(); i++) {
          typename T_PointCloud::PointT point;
          if (this->distance_section_.in(point_cloud_value_[i].the_1st_echo_distance) && point_cloud_value_[i].the_1st_echo_distance >= 0.01) {
            float x = point_cloud_value_[i].the_1st_echo_x;
            float y = point_cloud_value_[i].the_1st_echo_y;
            float z = point_cloud_value_[i].the_1st_echo_z;
            this->transformPoint(x, y, z);

            setX(point, x);
            setY(point, y);
            setZ(point, z);
            setIntensity(point, point_cloud_value_[i].intensity);
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
          setTimestamp(point, point_cloud_value_[i].timestamp);
          setRing(point, point_cloud_value_[i].ring);
#ifdef ENABLE_GTEST
          setPointId(point, i);
          setHorAngle(point, ((int32_t)(225 + (i * resolution) * 1000) % 360000) / 1000.0);
          setVerAngle(point, 0.0);
          setDistance(point, point_cloud_value_[i].the_1st_echo_distance);
#endif
          this->point_cloud_->points.emplace_back(point);
        }
        this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
      }

      if (this->param_.laser_scan_enable) {
        for (size_t i = 0; i < point_cloud_value_.size(); i++) {
          if (this->distance_section_.in(point_cloud_value_[i].the_1st_echo_distance) && point_cloud_value_[i].the_1st_echo_distance >= 0.01) {
            this->scan_data_->ranges.emplace_back(point_cloud_value_[i].the_1st_echo_distance);
            this->scan_data_->intensities.emplace_back(point_cloud_value_[i].intensity);
          } else {
            typename T_PointCloud::PointT point;
            if (!this->param_.dense_points) {
              this->scan_data_->ranges.emplace_back(NAN);
            } else {
              this->scan_data_->ranges.emplace_back(0);
            }
            this->scan_data_->intensities.emplace_back(0);
          }
        }
        this->scan_data_->angle_min = -135;
        this->scan_data_->angle_max = 135;
        this->scan_data_->angle_increment = resolution;
        this->scan_data_->time_increment = all_points_luminous_moment_716mini_[1] - all_points_luminous_moment_716mini_[0];
        this->scan_data_->scan_time = circle_time;
        this->scan_data_->range_min = this->param_.min_distance;
        this->scan_data_->range_max = this->param_.max_distance;

        this->cb_scan_data_(this->cloudTs());

        this->scan_data_->ranges.clear();
        this->scan_data_->intensities.clear();
      }
      ret = true;
    }
  }
  circle_id_of_pre_pkt_ = pkt.circle_id;
  return ret;
}

template <typename T_PointCloud>
void DecoderVanjee716Mini<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
  std::shared_ptr<ProtocolAbstract716Mini> p;
  std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

  if (*sp_cmd == *(CmdRepository716Mini::CreateInstance()->sp_scan_data_get_)) {
    p = std::make_shared<Protocol_ScanDataGet716Mini>();
  } else {
    return;
  }
  p->Load(*protocol);

  std::shared_ptr<ParamsAbstract> params = p->Params;
  if (typeid(*params) == typeid(Params_ScanData716Mini)) {
    std::shared_ptr<Params_ScanData716Mini> param = std::dynamic_pointer_cast<Params_ScanData716Mini>(params);
    if (param->data_get_flag_ && !scan_data_recv_flag_) {
      WJ_INFOL << "get wlr716mini scan data succ !" << WJ_REND;
      // (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      scan_data_recv_flag_ = true;
      Decoder<T_PointCloud>::angles_ready_ = true;
    }
  } else {
    WJ_WARNING << "Unknown Params Type..." << WJ_REND;
  }
}

}  // namespace lidar
}  // namespace vanjee