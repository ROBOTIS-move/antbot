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
#include <vanjee_driver/common/error_code.hpp>
#include <vanjee_driver/driver/decoder/basic_attr.hpp>
#include <vanjee_driver/driver/decoder/chan_angles.hpp>
#include <vanjee_driver/driver/decoder/imu_calibration_param.hpp>
#include <vanjee_driver/driver/decoder/member_checker.hpp>
#include <vanjee_driver/driver/decoder/section.hpp>
#include <vanjee_driver/driver/decoder/split_strategy.hpp>
#include <vanjee_driver/driver/decoder/trigon.hpp>
#include <vanjee_driver/driver/difop/difop_base.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>
#include <vanjee_driver/driver/driver_param.hpp>
#include <vanjee_driver/msg/imu_packet.hpp>
#include <vanjee_driver/msg/scan_data_msg.hpp>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#ifdef ENABLE_TRANSFORM
#include <Eigen/Dense>
#endif

#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>

namespace vanjee {
namespace lidar {
#pragma pack(push, 1)
typedef struct {
  uint16_t start_angle;
  uint16_t end_angle;
} WJFOV;
typedef struct {
  uint8_t sync_mode;
  uint8_t sync_sts;
  WJTimestampUTC timestamp;
} WJTimeInfo;

#pragma pack(pop)

enum WJEchoMode { ECHO_SINGLE = 0, ECHO_DUAL };

struct WJDecoderConstParam {
  uint16_t MSOP_LEN;

  uint16_t LASER_NUM;
  uint16_t BLOCKS_PER_PKT;
  uint16_t CHANNELS_PER_BLOCK;

  double DISTANCE_MIN;
  double DISTANCE_MAX;
  double DISTANCE_RES;
  double TEMPERATURE_RES;
};

typedef struct _HideAngleParams {
  float min_angle;
  float max_angle;
} HideAngleParams;

typedef struct _HideDistanceParams {
  float min_distance;
  float max_distance;
} HideDistanceParams;

typedef struct _HideRangeParams {
  uint16_t hide_point_channel_id;
  std::vector<HideAngleParams> hide_point_angle;
  std::vector<HideDistanceParams> hide_point_distance;
} HideRangeParams;

#define INIT_ONLY_ONCE()         \
  static bool init_flag = false; \
  if (init_flag)                 \
    return param;                \
  init_flag = true;

/// @brief Analyze the MSOP/DIFOP packet of the lidar to obtain a point cloud.
template <typename T_PointCloud>
class Decoder {
 public:
#ifdef ENABLE_TRANSFORM
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
  virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size) = 0;
  virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol) {
    WJ_INFO << "Default processDifop." << WJ_REND;
  };
  virtual ~Decoder() = default;
  bool processMsopPkt(const uint8_t *pkt, size_t size);

  explicit Decoder(const WJDecoderConstParam &const_param, const WJDecoderParam &param);
  /// @brief Calculate the time interval for data blocks
  double getPacketDuration();
  double prevPktTs();
  void transformPoint(float &x, float &y, float &z);

  std::vector<std::string> split(const std::string &s, char delimiter);
  void hidePiontsParamsLoad();
  bool isValueInRange(uint16_t channel, float angle, float distance, const std::vector<HideRangeParams> &ranges);

  void regCallback(const std::function<void(const Error &)> &cb_excep, const std::function<void(uint16_t, double)> &cb_split_frame,
                   const std::function<void(void)> &cb_imu_pkt = nullptr, const std::function<void(double)> &cb_scan_data = nullptr,
                   const std::function<void(double)> &cb_device_ctrl_state = nullptr);

  void regGetDifoCtrlDataInterface(std::shared_ptr<std::map<uint16, GetDifoCtrlClass>> get_difo_ctrl_map_ptr);

  std::shared_ptr<T_PointCloud> point_cloud_;
  std::shared_ptr<ImuPacket> imu_packet_;
  std::shared_ptr<ScanData> scan_data_;
  std::shared_ptr<DeviceCtrl> device_ctrl_;

 public:
  double cloudTs();
  uint8_t checkBCC(const uint8_t *pkt, size_t start_index, size_t end_index);
  uint16_t checkCRC16(const uint8_t *pkt, size_t start_index, size_t end_index);
  uint32_t checkCRC32(const uint8_t *pkt, size_t start_index, size_t end_index);

  WJDecoderConstParam const_param_;
  WJDecoderParam param_;
  Imu_Calibration_Param imu_calibration_param_;
  std::function<void(uint16_t, double)> cb_split_frame_;
  std::function<void(void)> cb_imu_pkt_;
  std::function<void(double)> cb_scan_data_;
  std::function<void(const Error &)> cb_excep_;
  std::function<void(double)> cb_device_ctrl_state_;
  std::shared_ptr<std::map<uint16, GetDifoCtrlClass>> get_difo_ctrl_map_ptr_;

#ifdef ENABLE_TRANSFORM
  Eigen::Matrix4d trans_;
#endif

  Trigon trigon_;
#define SIN(angle) this->trigon_.sin(angle)
#define COS(angle) this->trigon_.cos(angle)

  double packet_duration_;
  DistanceSection distance_section_;
  Projection projection;

  WJEchoMode echo_mode_;
  bool angles_ready_;
  double prev_pkt_ts_;
  double first_point_ts_;
  double last_point_ts_;
  uint16_t first_line_id_;

  bool hide_range_flag_;
  std::vector<HideRangeParams> hide_range_params_;

  // id 0: 0B1111111111111111
  // fpga1与fpga2通讯 顶部电源2 顶部电源1 fpga1 LD电压 fpga1阈值电压 无线供电 OSSD与GND短路
  // FPGA与MCU2通讯 FPGA与MCU1通讯 MCU1与MCU2通讯 MCU2时钟 MCU1时钟 普通输出 OSSD与VCC短路 输入IO 底部电源
  // id 1: 0B1111111111111111
  // OSSD_IAD过流保护故障 ARM程序执行顺序错误 ARM滴答计时器中断故障 ARM SPI中断故障 ARM SPI中断故障 ARM2 CPU/FLASH/SRAM异常 ARM1
  // CPU/FLASH/SRAM异常 至少90°区域内未检测到任何值 APD高压异常 EDM故障 光罩严重污染 RS485通讯 光通讯 电机 APD接收 温度采集
  std::vector<std::vector<std::string>> error_code_ = std::vector<std::vector<std::string>>{
      {"底部电源", "输入IO", "OSSD与VCC短路", "普通输出", "MCU1时钟", "MCU2时钟", "MCU1与MCU2通讯", "FPGA与MCU1通讯", "FPGA与MCU2通讯",
       "OSSD与GND短路", "无线供电", "fpga1 阈值电压", "fpga1 LD电压", "顶部电源1", "顶部电源2", "fpga1与fpga2通讯"},
      {"温度采集", "APD接收", "电机", "光通讯", "RS485通讯", "光罩严重污染", "EDM故障", "APD高压异常", "至少90°区域内未检测到任何值",
       "ARM1 CPU/FLASH/SRAM异常", "ARM2 CPU/FLASH/SRAM异常", "ARM SPI中断故障", "ARM USART中断故障", "ARM滴答计时器中断故障", "ARM程序执行顺序错误",
       "OSSD_IAD过流保护故障"}};
};

template <typename T_PointCloud>
void Decoder<T_PointCloud>::regGetDifoCtrlDataInterface(std::shared_ptr<std::map<uint16, GetDifoCtrlClass>> get_difo_ctrl_map_ptr) {
  get_difo_ctrl_map_ptr_ = get_difo_ctrl_map_ptr;
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::regCallback(const std::function<void(const Error &)> &cb_excep,
                                               const std::function<void(uint16_t, double)> &cb_split_frame,
                                               const std::function<void(void)> &cb_imu_pkt, const std::function<void(double)> &cb_scan_data,
                                               const std::function<void(double)> &cb_device_ctrl_state) {
  cb_excep_ = cb_excep;
  cb_split_frame_ = cb_split_frame;
  cb_imu_pkt_ = cb_imu_pkt;
  cb_scan_data_ = cb_scan_data;
  cb_device_ctrl_state_ = cb_device_ctrl_state;
}

template <typename T_PointCloud>
inline Decoder<T_PointCloud>::Decoder(const WJDecoderConstParam &const_param, const WJDecoderParam &param)
    : const_param_(const_param),
      param_(param),
      packet_duration_(0),
      distance_section_(const_param.DISTANCE_MIN, const_param.DISTANCE_MAX, param.min_distance, param.max_distance),
      echo_mode_(ECHO_SINGLE),
      angles_ready_(false),
      prev_pkt_ts_(0.0),
      first_point_ts_(0.0),
      last_point_ts_(0.0),
      first_line_id_(1),
      hide_range_flag_(false) {
  hidePiontsParamsLoad();
#ifdef ENABLE_TRANSFORM
  Eigen::AngleAxisd current_rotation_x(DEGREE_TO_RADIAN(param_.transform_param.roll), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd current_rotation_y(DEGREE_TO_RADIAN(param_.transform_param.pitch), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd current_rotation_z(DEGREE_TO_RADIAN(param_.transform_param.yaw), Eigen::Vector3d::UnitZ());
  Eigen::Translation3d current_translation(param_.transform_param.x, param_.transform_param.y, param_.transform_param.z);
  trans_ = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();
#endif
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::getPacketDuration() {
  return packet_duration_;
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::prevPktTs() {
  return prev_pkt_ts_;
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::cloudTs() {
  double ret_point_ts = param_.ts_first_point ? first_point_ts_ : last_point_ts_;
  return (ret_point_ts < 0 ? 0 : ret_point_ts);
}

/// @brief Perform coordinate transformation on points. It is based on the
/// third-party open-source library Eigen
template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::transformPoint(float &x, float &y, float &z) {
#ifdef ENABLE_TRANSFORM
  if (param_.transform_param.x != 0 || param_.transform_param.y != 0 || param_.transform_param.z != 0 || param_.transform_param.roll != 0 ||
      param_.transform_param.pitch != 0 || param_.transform_param.yaw != 0) {
    Eigen::Vector4d target_ori(x, y, z, 1);
    Eigen::Vector4d target_rotate = trans_ * target_ori;
    x = target_rotate(0);
    y = target_rotate(1);
    z = target_rotate(2);
  }
#endif
}

template <typename T_PointCloud>
std::vector<std::string> Decoder<T_PointCloud>::split(const std::string &s, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);

  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::hidePiontsParamsLoad() {
  if (param_.hide_points_range != "") {
    std::string hide_points_range_str = param_.hide_points_range;
    hide_points_range_str.erase(std::remove(hide_points_range_str.begin(), hide_points_range_str.end(), ' '), hide_points_range_str.end());

    std::vector<std::string> vec_str = split(hide_points_range_str, ';');
    for (size_t i = 0; i < vec_str.size(); i++) {
      std::vector<std::string> group_str = split(vec_str[i], ',');
      if (group_str.size() != 3) {
        WJ_WARNING << "group " << (i + 1) << " : format warning!" << WJ_REND;
        continue;
      }
      std::vector<std::string> channel_group_str = split(group_str[0], '/');
      std::vector<std::string> angle_group_str = split(group_str[1], '/');
      std::vector<std::string> distance_group_str = split(group_str[2], '/');

      std::vector<std::string> param_channel_str;
      std::vector<std::string> param_angle_str;
      std::vector<std::string> param_distance_str;

      HideRangeParams hide_range_param;

      for (size_t j = 0; j < angle_group_str.size(); j++) {
        HideAngleParams hide_angle_params;
        param_angle_str = split(angle_group_str[j], '-');

        if (param_angle_str.size() == 1) {
          hide_angle_params.min_angle = std::stof(param_angle_str[0]);
          hide_angle_params.max_angle = std::stof(param_angle_str[0]);
        } else if (param_angle_str.size() == 2) {
          hide_angle_params.min_angle = std::stof(param_angle_str[0]);
          hide_angle_params.max_angle = std::stof(param_angle_str[1]);
        } else {
          WJ_WARNING << "group " << (i + 1) << " : angle format warning!" << WJ_REND;
          continue;
        }
        hide_range_param.hide_point_angle.push_back(hide_angle_params);
      }

      for (size_t j = 0; j < distance_group_str.size(); j++) {
        HideDistanceParams hide_distance_params;
        param_distance_str = split(distance_group_str[j], '-');

        if (param_distance_str.size() == 1) {
          hide_distance_params.min_distance = std::stof(param_distance_str[0]);
          hide_distance_params.max_distance = std::stof(param_distance_str[0]);
        } else if (param_distance_str.size() == 2) {
          hide_distance_params.min_distance = std::stof(param_distance_str[0]);
          hide_distance_params.max_distance = std::stof(param_distance_str[1]);
        } else {
          WJ_WARNING << "group " << (i + 1) << " : distance format warning!" << WJ_REND;
          continue;
        }
        hide_range_param.hide_point_distance.push_back(hide_distance_params);
      }

      std::vector<uint16_t> channel_id;
      for (size_t j = 0; j < channel_group_str.size(); j++) {
        param_channel_str = split(channel_group_str[j], '-');

        if (param_channel_str.size() == 1) {
          channel_id.push_back(std::stoi(param_channel_str[0]));
        } else if (param_distance_str.size() == 2) {
          for (int k = std::stoi(param_channel_str[0]); k <= std::stoi(param_channel_str[1]); k++) {
            channel_id.push_back(k);
          }
        } else {
          WJ_WARNING << "group " << (i + 1) << " : channel format warning!" << WJ_REND;
          continue;
        }
      }

      for (size_t channel_index = 0; channel_index < channel_id.size(); channel_index++) {
        HideRangeParams hide_range_param_temp;
        hide_range_param_temp = hide_range_param;
        hide_range_param_temp.hide_point_channel_id = channel_id[channel_index];
        hide_range_params_.push_back(hide_range_param_temp);
      }
    }
    if (hide_range_params_.size() > 0) {
      hide_range_flag_ = true;
    }
  }
}

template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::isValueInRange(uint16_t channel, float angle, float distance, const std::vector<HideRangeParams> &ranges) {
  std::vector<int> channel_index;
  for (size_t i = 0; i < ranges.size(); i++) {
    if (channel == ranges[i].hide_point_channel_id) {
      channel_index.push_back(i);
    }
  }

  for (size_t i = 0; i < channel_index.size(); i++) {
    bool angle_flag = false;
    bool distance_flag = false;

    for (size_t j = 0; j < ranges[channel_index[i]].hide_point_angle.size(); j++) {
      if (angle >= ranges[channel_index[i]].hide_point_angle[j].min_angle && angle <= ranges[channel_index[i]].hide_point_angle[j].max_angle) {
        angle_flag = true;
        break;
      }
    }
    for (size_t j = 0; j < ranges[channel_index[i]].hide_point_distance.size(); j++) {
      if (distance >= ranges[channel_index[i]].hide_point_distance[j].min_distance &&
          distance <= ranges[channel_index[i]].hide_point_distance[j].max_distance) {
        distance_flag = true;
        break;
      }
    }
    if (angle_flag && distance_flag)
      return true;
  }
  return false;
}

template <typename T_PointCloud>
inline uint8_t Decoder<T_PointCloud>::checkBCC(const uint8_t *pkt, size_t start_index, size_t end_index) {
  uint8 bcc = 0x00;
  for (size_t i = start_index; i < end_index; i++) {
    bcc ^= pkt[i];
  }
  return bcc;
}

template <typename T_PointCloud>
inline uint16_t Decoder<T_PointCloud>::checkCRC16(const uint8_t *pkt, size_t start_index, size_t end_index) {
  static const uint16_t crctab[256] = {
      0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210,
      0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401,
      0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6,
      0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
      0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC,
      0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD,
      0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A,
      0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
      0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB,
      0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8,
      0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9,
      0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
      0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07,
      0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74,
      0x2E93, 0x3EB2, 0x0ED1, 0x1EF0};
  uint16_t crc16 = 0x00;

  for (size_t i = start_index; i < end_index; i++) {
    crc16 = (uint16)(crctab[(crc16 >> 8) ^ pkt[i]] ^ (crc16 << 8));
  }

  return crc16;
}

template <typename T_PointCloud>
inline uint32_t Decoder<T_PointCloud>::checkCRC32(const uint8_t *pkt, size_t start_index, size_t end_index) {
  static const uint32_t crctab[256] = {
      0x00000000, 0xf26b8303, 0xe13b70f7, 0x1350f3f4, 0xc79a971f, 0x35f1141c, 0x26a1e7e8, 0xd4ca64eb, 0x8ad958cf, 0x78b2dbcc, 0x6be22838, 0x9989ab3b,
      0x4d43cfd0, 0xbf284cd3, 0xac78bf27, 0x5e133c24, 0x105ec76f, 0xe235446c, 0xf165b798, 0x030e349b, 0xd7c45070, 0x25afd373, 0x36ff2087, 0xc494a384,
      0x9a879fa0, 0x68ec1ca3, 0x7bbcef57, 0x89d76c54, 0x5d1d08bf, 0xaf768bbc, 0xbc267848, 0x4e4dfb4b, 0x20bd8ede, 0xd2d60ddd, 0xc186fe29, 0x33ed7d2a,
      0xe72719c1, 0x154c9ac2, 0x061c6936, 0xf477ea35, 0xaa64d611, 0x580f5512, 0x4b5fa6e6, 0xb93425e5, 0x6dfe410e, 0x9f95c20d, 0x8cc531f9, 0x7eaeb2fa,
      0x30e349b1, 0xc288cab2, 0xd1d83946, 0x23b3ba45, 0xf779deae, 0x05125dad, 0x1642ae59, 0xe4292d5a, 0xba3a117e, 0x4851927d, 0x5b016189, 0xa96ae28a,
      0x7da08661, 0x8fcb0562, 0x9c9bf696, 0x6ef07595, 0x417b1dbc, 0xb3109ebf, 0xa0406d4b, 0x522bee48, 0x86e18aa3, 0x748a09a0, 0x67dafa54, 0x95b17957,
      0xcba24573, 0x39c9c670, 0x2a993584, 0xd8f2b687, 0x0c38d26c, 0xfe53516f, 0xed03a29b, 0x1f682198, 0x5125dad3, 0xa34e59d0, 0xb01eaa24, 0x42752927,
      0x96bf4dcc, 0x64d4cecf, 0x77843d3b, 0x85efbe38, 0xdbfc821c, 0x2997011f, 0x3ac7f2eb, 0xc8ac71e8, 0x1c661503, 0xee0d9600, 0xfd5d65f4, 0x0f36e6f7,
      0x61c69362, 0x93ad1061, 0x80fde395, 0x72966096, 0xa65c047d, 0x5437877e, 0x4767748a, 0xb50cf789, 0xeb1fcbad, 0x197448ae, 0x0a24bb5a, 0xf84f3859,
      0x2c855cb2, 0xdeeedfb1, 0xcdbe2c45, 0x3fd5af46, 0x7198540d, 0x83f3d70e, 0x90a324fa, 0x62c8a7f9, 0xb602c312, 0x44694011, 0x5739b3e5, 0xa55230e6,
      0xfb410cc2, 0x092a8fc1, 0x1a7a7c35, 0xe811ff36, 0x3cdb9bdd, 0xceb018de, 0xdde0eb2a, 0x2f8b6829, 0x82f63b78, 0x709db87b, 0x63cd4b8f, 0x91a6c88c,
      0x456cac67, 0xb7072f64, 0xa457dc90, 0x563c5f93, 0x082f63b7, 0xfa44e0b4, 0xe9141340, 0x1b7f9043, 0xcfb5f4a8, 0x3dde77ab, 0x2e8e845f, 0xdce5075c,
      0x92a8fc17, 0x60c37f14, 0x73938ce0, 0x81f80fe3, 0x55326b08, 0xa759e80b, 0xb4091bff, 0x466298fc, 0x1871a4d8, 0xea1a27db, 0xf94ad42f, 0x0b21572c,
      0xdfeb33c7, 0x2d80b0c4, 0x3ed04330, 0xccbbc033, 0xa24bb5a6, 0x502036a5, 0x4370c551, 0xb11b4652, 0x65d122b9, 0x97baa1ba, 0x84ea524e, 0x7681d14d,
      0x2892ed69, 0xdaf96e6a, 0xc9a99d9e, 0x3bc21e9d, 0xef087a76, 0x1d63f975, 0x0e330a81, 0xfc588982, 0xb21572c9, 0x407ef1ca, 0x532e023e, 0xa145813d,
      0x758fe5d6, 0x87e466d5, 0x94b49521, 0x66df1622, 0x38cc2a06, 0xcaa7a905, 0xd9f75af1, 0x2b9cd9f2, 0xff56bd19, 0x0d3d3e1a, 0x1e6dcdee, 0xec064eed,
      0xc38d26c4, 0x31e6a5c7, 0x22b65633, 0xd0ddd530, 0x0417b1db, 0xf67c32d8, 0xe52cc12c, 0x1747422f, 0x49547e0b, 0xbb3ffd08, 0xa86f0efc, 0x5a048dff,
      0x8ecee914, 0x7ca56a17, 0x6ff599e3, 0x9d9e1ae0, 0xd3d3e1ab, 0x21b862a8, 0x32e8915c, 0xc083125f, 0x144976b4, 0xe622f5b7, 0xf5720643, 0x07198540,
      0x590ab964, 0xab613a67, 0xb831c993, 0x4a5a4a90, 0x9e902e7b, 0x6cfbad78, 0x7fab5e8c, 0x8dc0dd8f, 0xe330a81a, 0x115b2b19, 0x020bd8ed, 0xf0605bee,
      0x24aa3f05, 0xd6c1bc06, 0xc5914ff2, 0x37faccf1, 0x69e9f0d5, 0x9b8273d6, 0x88d28022, 0x7ab90321, 0xae7367ca, 0x5c18e4c9, 0x4f48173d, 0xbd23943e,
      0xf36e6f75, 0x0105ec76, 0x12551f82, 0xe03e9c81, 0x34f4f86a, 0xc69f7b69, 0xd5cf889d, 0x27a40b9e, 0x79b737ba, 0x8bdcb4b9, 0x988c474d, 0x6ae7c44e,
      0xbe2da0a5, 0x4c4623a6, 0x5f16d052, 0xad7d5351};
  uint32_t crc32 = 0x00;

  for (size_t i = start_index; i < end_index; i++) {
    uint8_t index = (uint8_t)((crc32 ^ pkt[i]) & 0xff);
    crc32 = (crctab[index] ^ (crc32 >> 8)) & 0xffffffff;  //(uint16)(crctab[(crc16 >> 8) ^ pkt[i]] ^ (crc16 << 8));
  }
  return crc32;
}

/// @brief MSOP Packet Handler
/// @brief This is a purely virtual function, provided by the derived classes of
/// each lidar to provide its own implementation
template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::processMsopPkt(const uint8_t *pkt, size_t size) {
  constexpr static int CLOUD_POINT_MAX = 1000000;
  if (this->point_cloud_ && (this->point_cloud_->points.size() > CLOUD_POINT_MAX)) {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_CLOUDOVERFLOW)), 1);
  }
  if (param_.wait_for_difop && !angles_ready_) {
    DELAY_LIMIT_CALL(cb_excep_(Error(ERRCODE_NODIFOPRECV)), 1);
    return false;
  }
  // if (size != this->const_param_.MSOP_LEN)
  // {
  //     LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGMSOPLEN)), 1);
  //     return false;
  // }
  // /// @brief Packet的标志字节是否匹配。
  return decodeMsopPkt(pkt, size);
}

}  // namespace lidar
}  // namespace vanjee