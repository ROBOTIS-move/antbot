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

#include <math.h>

#include <iostream>
#include <memory>
#include <vector>

#include <vanjee_driver/driver/decoder/decoder_packet_base/imu/complementary_filter.hpp>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/common/wj_log.hpp"

#define HDL_Grabber_toRadians(x) ((x)*M_PI / 180.0)

namespace vanjee {
namespace lidar {
class ImuParamGet {
 public:
  double x_ang_k_;
  double x_ang_b_;
  double y_ang_k_;
  double y_ang_b_;
  double z_ang_k_;
  double z_ang_b_;

  double x_acc_k_;
  double x_acc_b_;
  double y_acc_k_;
  double y_acc_b_;
  double z_acc_k_;
  double z_acc_b_;

  double x_zero_param_;
  double y_zero_param_;
  double z_zero_param_;

  double rotate_b_[2][2];

  int32_t me_zero_param_time_;
  int32_t aim_me_zero_param_time_;

  double dere_time_pre_;

  struct imuGetResultPa {
    double q0;
    double q1;
    double q2;
    double q3;

    double x_angle;
    double y_angle;
    double z_angle;

    double x_acc;
    double y_acc;
    double z_acc;

    void init() {
      q0 = q1 = q2 = q3 = 0;
      x_angle = y_angle = z_angle = 0;
      x_acc = y_acc = z_acc = 0;
    }
  };

  imuGetResultPa imu_result_stu_;

  // std::shared_ptr<imu_tools::ComplementaryFilter> m_filter_;
  imu_tools::ComplementaryFilter m_filter_;

  ImuParamGet(double axisoffset = 0.0) {
    x_ang_k_ = y_ang_k_ = z_ang_k_ = 1;
    x_ang_b_ = y_ang_b_ = z_ang_b_ = 0;

    x_acc_k_ = y_acc_k_ = z_acc_k_ = 1;
    x_acc_b_ = y_acc_b_ = z_acc_b_ = 0;

    x_zero_param_ = y_zero_param_ = z_zero_param_ = 0;

    rotate_b_[0][0] = cos(HDL_Grabber_toRadians(axisoffset));
    rotate_b_[0][1] = -sin(HDL_Grabber_toRadians(axisoffset));
    rotate_b_[1][0] = sin(HDL_Grabber_toRadians(axisoffset));
    rotate_b_[1][1] = cos(HDL_Grabber_toRadians(axisoffset));

    me_zero_param_time_ = 0;
    aim_me_zero_param_time_ = 1000;

    dere_time_pre_ = 0;

    imu_result_stu_.init();
  }

  ~ImuParamGet() {
  }

  inline void setImuTempCalibrationParams(double x_k, double x_b, double y_k, double y_b, double z_k, double z_b) {
    x_ang_k_ = x_k;
    x_ang_b_ = x_b;
    y_ang_k_ = y_k;
    y_ang_b_ = y_b;
    z_ang_k_ = z_k;
    z_ang_b_ = z_b;
  }

  inline void setImuAcceCalibrationParams(double x_k, double x_b, double y_k, double y_b, double z_k, double z_b) {
    x_acc_k_ = x_k;
    x_acc_b_ = x_b;
    y_acc_k_ = y_k;
    y_acc_b_ = y_b;
    z_acc_k_ = z_k;
    z_acc_b_ = z_b;
  }

  inline void imuAngCorrbyTemp(double &x_ang, double &y_ang, double &z_ang, double imuTempreture) {
    x_ang -= x_ang_k_ * (imuTempreture - 40) + x_ang_b_;
    y_ang -= y_ang_k_ * (imuTempreture - 40) + y_ang_b_;
    z_ang -= z_ang_k_ * (imuTempreture - 40) + z_ang_b_;
  }

  inline void imuAccCorrbyKB(double &x_acc, double &y_acc, double &z_acc) {
    x_acc = x_acc_k_ * x_acc - x_acc_b_;
    y_acc = y_acc_k_ * y_acc - y_acc_b_;
    z_acc = z_acc_k_ * z_acc - z_acc_b_;
  }

  inline void rotateImu(double &x_acc, double &y_acc, double &z_acc, double &x_ang, double &y_ang, double &z_ang) {
    double rotate_acc[2] = {x_acc, y_acc};
    double rotate_Ang[2] = {x_ang, y_ang};

    x_acc = rotate_acc[0] * rotate_b_[0][0] + rotate_acc[1] * rotate_b_[1][0];
    y_acc = rotate_acc[0] * rotate_b_[0][1] + rotate_acc[1] * rotate_b_[1][1];

    x_ang = rotate_Ang[0] * rotate_b_[0][0] + rotate_Ang[1] * rotate_b_[1][0];
    y_ang = rotate_Ang[0] * rotate_b_[0][1] + rotate_Ang[1] * rotate_b_[1][1];
  }

  int32_t imuGetZeroPa(double x_ang, double y_ang, double z_ang) {
    x_zero_param_ += x_ang;
    y_zero_param_ += y_ang;
    z_zero_param_ += z_ang;

    return me_zero_param_time_++;
  }

  bool imuGet(double x_ang, double y_ang, double z_ang, double x_acc, double y_acc, double z_acc, double time, double temperature = -100.0,
              bool temperature_enable = true, bool old_coefficient_flag = true, bool calibration_param_enable = true, bool rotate_enbale = true,
              bool zero_calibrate_enbale = true) {
    if (temperature_enable) {
      if (temperature <= -100.0)
        return false;
      imuAngCorrbyTemp(x_ang, y_ang, z_ang, temperature);
    }

    if (old_coefficient_flag) {
      x_acc = x_acc / 1000 * 0.061 * 9.81;
      y_acc = y_acc / 1000 * 0.061 * 9.81;
      z_acc = z_acc / 1000 * 0.061 * 9.81;

      x_ang = x_ang / 1000 * 8.75 * 0.0174533;
      y_ang = y_ang / 1000 * 8.75 * 0.0174533;
      z_ang = z_ang / 1000 * 8.75 * 0.0174533;
    } else {
      x_acc = x_acc * 1e-6 * 9.81;
      y_acc = y_acc * 1e-6 * 9.81;
      z_acc = z_acc * 1e-6 * 9.81;

      x_ang = x_ang * 1e-3 * 0.0174533;
      y_ang = y_ang * 1e-3 * 0.0174533;
      z_ang = z_ang * 1e-3 * 0.0174533;
    }

    if (calibration_param_enable)
      imuAccCorrbyKB(x_acc, y_acc, z_acc);

    if (rotate_enbale)
      rotateImu(x_acc, y_acc, z_acc, x_ang, y_ang, z_ang);

    double l_dertatime;
    if (dere_time_pre_ < time) {
      l_dertatime = time - dere_time_pre_;
    } else {
      dere_time_pre_ = time;
      return false;
    }
    dere_time_pre_ = time;

    if (zero_calibrate_enbale) {
      if (me_zero_param_time_ < aim_me_zero_param_time_ - 5) {
        imuGetZeroPa(x_ang, y_ang, z_ang);
        return false;
      } else if (me_zero_param_time_ >= aim_me_zero_param_time_ - 5 && me_zero_param_time_ < aim_me_zero_param_time_) {
        imuGetZeroPa(x_ang, y_ang, z_ang);
        double x_angle_offset_ = x_zero_param_ / me_zero_param_time_;
        double y_angle_offset_ = y_zero_param_ / me_zero_param_time_;
        double z_angle_offset_ = z_zero_param_ / me_zero_param_time_;

        if (std::abs(x_ang - x_angle_offset_) > 0.008 || std::abs(y_ang - y_angle_offset_) > 0.008 || std::abs(z_ang - z_angle_offset_) > 0.008) {
          WJ_WARNING << "Do not move during IMU calibration" << WJ_REND;
          WJ_INFO << "Waiting for IMU calibration..." << WJ_REND;
          x_zero_param_ = 0.0;
          y_zero_param_ = 0.0;
          z_zero_param_ = 0.0;
          me_zero_param_time_ = 0;
        }
        return false;
      } else if (me_zero_param_time_ == aim_me_zero_param_time_) {
        x_zero_param_ /= aim_me_zero_param_time_;
        y_zero_param_ /= aim_me_zero_param_time_;
        z_zero_param_ /= aim_me_zero_param_time_;

        WJ_INFO << "Begin publish Imu" << WJ_REND;
        me_zero_param_time_++;
      }
    }

    x_ang = x_ang - x_zero_param_;
    y_ang = y_ang - y_zero_param_;
    z_ang = z_ang - z_zero_param_;

    imu_result_stu_.x_angle = x_ang;
    imu_result_stu_.y_angle = y_ang;
    imu_result_stu_.z_angle = z_ang;

    imu_result_stu_.x_acc = x_acc;
    imu_result_stu_.y_acc = y_acc;
    imu_result_stu_.z_acc = z_acc;

    static double q0 = 0;
    static double q1 = 0;
    static double q2 = 0;
    static double q3 = 0;

    m_filter_.update(x_acc, y_acc, z_acc, x_ang, y_ang, z_ang, l_dertatime, q0, q1, q2, q3);
    m_filter_.getOrientation(q0, q1, q2, q3);

    imu_result_stu_.q0 = q0;
    imu_result_stu_.q1 = q1;
    imu_result_stu_.q2 = q2;
    imu_result_stu_.q3 = q3;

    return true;
  }
};

}  // namespace lidar
}  // namespace vanjee