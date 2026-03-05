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

#include <cmath>

#include <vanjee_driver/common/wj_common.hpp>

namespace vanjee {
namespace lidar {
/// @brief The granularity of calculating the 'sin' and 'cos' values within the
/// angle range of (0, 360) is 0.001 degrees
class Trigon {
 private:
  float *sins_;
  float *coss_;
  float *tans_;
  float *o_sins_;
  float *o_coss_;
  float *o_tans_;

 public:
  constexpr static int64_t ANGLE_MIN = -180000;
  constexpr static int64_t ANGLE_MAX = 360000;

  Trigon() {
    int64_t range = ANGLE_MAX - ANGLE_MIN;
    o_sins_ = (float *)malloc(range * sizeof(float));
    o_coss_ = (float *)malloc(range * sizeof(float));
    o_tans_ = (float *)malloc(range * sizeof(float));
    for (int64_t i = ANGLE_MIN, j = 0; i < ANGLE_MAX; i++, j++) {
      double rad = DEGREE_TO_RADIAN(static_cast<double>(i) * 0.001);
      o_sins_[j] = (float)std::sin(rad);
      o_coss_[j] = (float)std::cos(rad);
      o_tans_[j] = (float)std::tan(rad);
    }
    sins_ = o_sins_ - ANGLE_MIN;
    coss_ = o_coss_ - ANGLE_MIN;
    tans_ = o_tans_ - ANGLE_MIN;
  }
  ~Trigon() {
    free(o_sins_);
    free(o_coss_);
    free(o_tans_);
  }

  float sin(int64_t angle) {
    if (angle < ANGLE_MIN || angle >= ANGLE_MAX) {
      angle = 0;
    }
    return sins_[angle];
  }
  float cos(int64_t angle) {
    if (angle < ANGLE_MIN || angle >= ANGLE_MAX) {
      angle = 0;
    }
    return coss_[angle];
  }
  float tan(int64_t angle) {
    if (angle < ANGLE_MIN || angle >= ANGLE_MAX) {
      angle = 0;
    }
    return tans_[angle];
  }
  void print() {
    for (int64_t i = -1800; i < -1790; i++) {
      std::cout << sins_[i] << "\t" << coss_[i] << std::endl;
    }
  }
};

}  // namespace lidar

}  // namespace vanjee
