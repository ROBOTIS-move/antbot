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

#include "vanjee_driver/common/super_header.hpp"

namespace vanjee {
namespace lidar {
class Imu_Calibration_Param {
 public:
  enum PARAM_TYPE { TEMP_PARAM = 1, ACC_PARAM = 2 };

 public:
  Imu_Calibration_Param();
  int loadFromFile(const std::string &imu_param_path);
  int loadFromLiDAR(const std::string &imu_param_path, PARAM_TYPE param_type, float32 x_k_, float32 x_b_, float32 y_k_, float32 y_b_, float32 z_k_,
                    float32 z_b_);
  void getFloatFromString(std::string str, float32 &k_, float32 &b_);
  bool param_is_ready();

 public:
  float32 x_axis_temp_k;
  float32 x_axis_temp_b;
  float32 y_axis_temp_k;
  float32 y_axis_temp_b;
  float32 z_axis_temp_k;
  float32 z_axis_temp_b;

  float32 x_axis_acc_k;
  float32 x_axis_acc_b;
  float32 y_axis_acc_k;
  float32 y_axis_acc_b;
  float32 z_axis_acc_k;
  float32 z_axis_acc_b;

 private:
  bool lidar_online_param_is_ready_;
  bool temp_param_is_ready_;
  bool acc_param_is_ready_;
};

Imu_Calibration_Param::Imu_Calibration_Param() : temp_param_is_ready_(false), acc_param_is_ready_(false) {
}

bool Imu_Calibration_Param::param_is_ready() {
  return (temp_param_is_ready_ && acc_param_is_ready_);
}

int Imu_Calibration_Param::loadFromLiDAR(const std::string &imu_param_path, PARAM_TYPE param_type, float32 x_k_, float32 x_b_, float32 y_k_,
                                         float32 y_b_, float32 z_k_, float32 z_b_) {
  std::ofstream fd;
  if (!lidar_online_param_is_ready_) {
    temp_param_is_ready_ = false;
    acc_param_is_ready_ = false;
    lidar_online_param_is_ready_ = true;
  }

  if (!temp_param_is_ready_ && !acc_param_is_ready_) {
    fd.open(imu_param_path.c_str(), std::ios::out);
  } else {
    fd.open(imu_param_path.c_str(), std::ios::app);
  }

  if (!fd.is_open()) {
    WJ_WARNING << "fail to open imu_param file:" << imu_param_path << WJ_REND;
    return -1;
  }

  if (param_type == TEMP_PARAM) {
    fd << "lin,X," << std::to_string(x_k_) << "," << std::to_string(x_b_) << "\n";
    fd << "lin,Y," << std::to_string(y_k_) << "," << std::to_string(y_b_) << "\n";
    fd << "lin,Z," << std::to_string(z_k_) << "," << std::to_string(z_b_) << "\n";

    temp_param_is_ready_ = true;
  } else if (param_type == ACC_PARAM) {
    fd << "acc,X," << std::to_string(x_k_) << "," << std::to_string(x_b_) << "\n";
    fd << "acc,Y," << std::to_string(y_k_) << "," << std::to_string(y_b_) << "\n";
    fd << "acc,Z," << std::to_string(z_k_) << "," << std::to_string(z_b_) << "\n";

    acc_param_is_ready_ = true;
  }
  fd.close();
  return 0;
}

void Imu_Calibration_Param::getFloatFromString(std::string str, float32 &k_, float32 &b_) {
  size_t pos = std::string::npos;
  pos = str.find(",");
  if (pos != std::string::npos) {
    k_ = std::stof(str.substr(0, pos));
    b_ = std::stof(str.substr(pos + 1));
  }
}

int Imu_Calibration_Param::loadFromFile(const std::string &imu_param_path) {
  std::ifstream fd(imu_param_path.c_str(), std::ios::in);
  if (!fd.is_open()) {
    WJ_ERROR << "fail to open imu_param file:" << imu_param_path << WJ_REND;
    return -1;
  }

  std::string line;
  while (std::getline(fd, line)) {
    std::string head = line.substr(0, 6);
    std::string content = line.substr(6, line.length() - 6);

    if (head.substr(0, 3) == "lin") {
      temp_param_is_ready_ = true;
    } else if (head.substr(0, 3) == "acc") {
      acc_param_is_ready_ = true;
    } else {
      continue;
    }

    if (head == "lin,X,") {
      getFloatFromString(content, x_axis_temp_k, x_axis_temp_b);
    } else if (head == "lin,Y,") {
      getFloatFromString(content, y_axis_temp_k, y_axis_temp_b);
    } else if (head == "lin,Z,") {
      getFloatFromString(content, z_axis_temp_k, z_axis_temp_b);
    } else if (head == "acc,X,") {
      getFloatFromString(content, x_axis_acc_k, x_axis_acc_b);
    } else if (head == "acc,Y,") {
      getFloatFromString(content, y_axis_acc_k, y_axis_acc_b);
    } else if (head == "acc,Z,") {
      getFloatFromString(content, z_axis_acc_k, z_axis_acc_b);
    } else {
      WJ_WARNING << "ERROR IMU_PARAM..." << WJ_REND;
    }
  }

  fd.close();
  return 0;
}
}  // namespace lidar
}  // namespace vanjee