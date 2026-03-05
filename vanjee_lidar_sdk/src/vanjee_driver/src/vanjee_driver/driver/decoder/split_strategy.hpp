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
#include <stdint.h>

#include <cmath>
namespace vanjee {
namespace lidar {
/// @brief Frame division mode interface of mechanical lidar
class SplitStrategy {
 public:
  /// @brief Subcontracting virtual functions，call SplitStrategy::newBlock()
  virtual bool newBlock(int64_t value, int8_t mirror_id = -1) = 0;
  virtual ~SplitStrategy() = default;
};

/// @brief Subcontract with azimuth
class SplitStrategyByAngle : public SplitStrategy {
 private:
  const int8_t split_mirror_id_;
  int64_t prev_angle_;

 public:
  SplitStrategyByAngle(int8_t split_mirror_id) : split_mirror_id_(split_mirror_id), prev_angle_(0) {
  }
  virtual ~SplitStrategyByAngle() = default;
  virtual bool newBlock(int64_t angle, int8_t mirror_id = -1) {
    if ((mirror_id == -1 || mirror_id == split_mirror_id_) && angle < prev_angle_) {
      prev_angle_ = angle;
      return true;
    } else {
      prev_angle_ = angle;
      return false;
    }
  }
};

/// @brief Subcontract with block number
class SplitStrategyByBlock : public SplitStrategy {
 private:
  const int8_t split_mirror_id_;
  int64_t prev_block_;

 public:
  SplitStrategyByBlock(int8_t split_mirror_id) : split_mirror_id_(split_mirror_id), prev_block_(0) {
  }
  virtual ~SplitStrategyByBlock() = default;
  virtual bool newBlock(int64_t block, int8_t mirror_id = -1) {
    if ((mirror_id == -1 || mirror_id == split_mirror_id_) && block < prev_block_) {
      prev_block_ = block;
      return true;
    } else {
      prev_block_ = block;
      return false;
    }
  }
};

}  // namespace lidar

}  // namespace vanjee
