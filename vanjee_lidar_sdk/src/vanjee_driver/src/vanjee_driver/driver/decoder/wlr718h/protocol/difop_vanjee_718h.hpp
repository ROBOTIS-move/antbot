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
#include <memory>
#include <vector>

#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/cmd_repository_718h.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_scan_data_get.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_topboard_operate_params_get.hpp>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"

namespace vanjee {
namespace lidar {
class DifopVanjee718H : public DifopBase {
 public:
  virtual void initGetDifoCtrlDataMapPtr();
};

void DifopVanjee718H::initGetDifoCtrlDataMapPtr() {
  getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16, GetDifoCtrlClass>>();

  const uint8 arr[] = {0x01, 0x00, 0x00, 0x00};
  std::shared_ptr<std::vector<uint8>> content = std::make_shared<std::vector<uint8>>();
  content->insert(content->end(), arr, arr + sizeof(arr) / sizeof(uint8));

  GetDifoCtrlClass getDifoCtrlData_ScanDataGet(*(std::make_shared<Protocol_ScanDataGet718H>()->GetRequest(content)), false, 3000);
  (*getDifoCtrlData_map_ptr_).emplace(CmdRepository718H::CreateInstance()->sp_scan_data_get_->GetCmdKey(), getDifoCtrlData_ScanDataGet);

  // GetDifoCtrlClass
  // getDifoCtrlData_TopBoardOperateGet(*(std::make_shared<Protocol_TopBoardOperateParamsGet718H>()->GetRequest()),
  // false, 3000);
  // (*getDifoCtrlData_map_ptr_).emplace(CmdRepository718H::CreateInstance()->sp_top_board_operate_params_get_->GetCmdKey(),getDifoCtrlData_TopBoardOperateGet);
}
}  // namespace lidar
}  // namespace vanjee