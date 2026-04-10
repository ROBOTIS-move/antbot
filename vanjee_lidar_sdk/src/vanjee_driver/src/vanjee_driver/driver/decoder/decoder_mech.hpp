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

#include <vanjee_driver/driver/decoder/decoder.hpp>

namespace vanjee {
namespace lidar {

struct WJDecoderMechConstParam {
  WJDecoderConstParam base;

  float RX;
  float RY;
  float RZ;

  double BLOCK_DURATION;
  double CHAN_TSS[128];
  float CHAN_AZIS[128];
};
/// @brief DecoderMech originated from Decoder
template <typename T_PointCloud>
class DecoderMech : public Decoder<T_PointCloud> {
 public:
  constexpr static int32_t WJ_ONE_ROUND = 36000;

  virtual ~DecoderMech() = default;

  explicit DecoderMech(const WJDecoderMechConstParam &const_param, const WJDecoderParam &param);

  void print();

#ifndef UNIT_TEST
 protected:
#endif

  WJDecoderMechConstParam mech_const_param_;
  ChanAngles chan_angles_;
  AzimuthSection scan_section_;
  std::shared_ptr<SplitStrategy> split_strategy_;

  uint16_t rps_;
  uint16_t blks_per_frame_;
  uint16_t split_blks_per_frame_;
  uint16_t block_az_diff_;

  double fov_blind_ts_diff_;
};

template <typename T_PointCloud>
inline DecoderMech<T_PointCloud>::DecoderMech(const WJDecoderMechConstParam &const_param, const WJDecoderParam &param)
    : Decoder<T_PointCloud>(const_param.base, param),
      mech_const_param_(const_param),
      chan_angles_(this->const_param_.LASER_NUM),
      scan_section_((int32_t)(this->param_.start_angle * 1000), (int32_t)(this->param_.end_angle * 1000)),
      rps_(this->param_.rpm),
      blks_per_frame_((uint16_t)(1 / (10 * this->mech_const_param_.BLOCK_DURATION))),
      split_blks_per_frame_(blks_per_frame_),
      block_az_diff_(20),
      fov_blind_ts_diff_(0.0) {
  // this->packet_duration_ =
  // this->mech_const_param_.BLOCK_DURATION * this->const_param_.BLOCKS_PER_PKT;
}

template <typename T_PointCloud>
inline void DecoderMech<T_PointCloud>::print() {
  std::cout << "-----------------------------------------" << std::endl
            << "rps:\t\t\t" << this->rps_ << std::endl
            << "echo_mode:\t\t" << this->echo_mode_ << std::endl
            << "blks_per_frame:\t\t" << this->blks_per_frame_ << std::endl
            << "split_blks_per_frame:\t" << this->split_blks_per_frame_ << std::endl
            << "block_az_diff:\t\t" << this->block_az_diff_ << std::endl
            << "fov_blind_ts_diff:\t" << this->fov_blind_ts_diff_ << std::endl
            << "angle_from_file:\t" << this->param_.config_from_file << std::endl
            << "angles_ready:\t\t" << this->angles_ready_ << std::endl;
}
}  // namespace lidar

}  // namespace vanjee
