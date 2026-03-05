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

#include <vanjee_driver/driver/difop/cmd_class.hpp>

namespace vanjee {
namespace lidar {
class CmdRepository722 {
 public:
  const std::shared_ptr<CmdClass> sp_ld_angle_get_ = std::make_shared<CmdClass>(0x06, 0x03);
  const std::shared_ptr<CmdClass> sp_ld_offset_get_ = std::make_shared<CmdClass>(0x05, 0x37);
  const std::shared_ptr<CmdClass> sp_ld_value_get_ = std::make_shared<CmdClass>(0x05, 0x14);
  const std::shared_ptr<CmdClass> sp_ld_eccentricity_param_get_ = std::make_shared<CmdClass>(0x05, 0x1a);
  const std::shared_ptr<CmdClass> sp_imu_line_Param_get_ = std::make_shared<CmdClass>(0x06, 0x12);
  const std::shared_ptr<CmdClass> sp_imu_add_Param_get_ = std::make_shared<CmdClass>(0x06, 0x14);
  const std::shared_ptr<CmdClass> sp_temperature_param_get_ = std::make_shared<CmdClass>(0x04, 0x02);
  const std::shared_ptr<CmdClass> set_protocol_version_cmd_id_ptr_ = std::make_shared<CmdClass>(0xff, 0x00);
  const std::shared_ptr<CmdClass> sp_get_work_mode_ = std::make_shared<CmdClass>(0x06, 0x16);
  const std::shared_ptr<CmdClass> sp_set_work_mode_ = std::make_shared<CmdClass>(0x06, 0x17);
  const std::shared_ptr<CmdClass> sp_get_imu_packet_ = std::make_shared<CmdClass>(0x20, 0x01);

  static CmdRepository722* CreateInstance() {
    if (p_CmdRepository722 == nullptr)
      p_CmdRepository722 = new CmdRepository722();

    return p_CmdRepository722;
  }

 private:
  static CmdRepository722* p_CmdRepository722;
  CmdRepository722() {
  }
  CmdRepository722(const CmdRepository722&) = delete;
  CmdRepository722& operator=(const CmdRepository722&) = delete;
};

CmdRepository722* CmdRepository722::p_CmdRepository722 = nullptr;
}  // namespace lidar
}  // namespace vanjee