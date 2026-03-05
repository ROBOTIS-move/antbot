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

#include "protocol_abstract_719c.hpp"
#include "vanjee_driver/driver/decoder/wlr719c/protocol/params/params_scan_data.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee {
namespace lidar {
class Protocol_ScanDataGet719C : public ProtocolAbstract719C {
 public:
  Protocol_ScanDataGet719C() : ProtocolAbstract719C(CmdRepository719C::CreateInstance()->Sp_ScanDataGet, std::make_shared<Params_ScanData719C>()) {
  }

  std::shared_ptr<std::vector<uint8>> GetRequest(std::shared_ptr<std::vector<uint8>> content = nullptr) {
    if (content == nullptr) {
      const uint8 arr[] = {0x00, 0x00, 0x00, 0x00};
      content = std::make_shared<std::vector<uint8>>();
      content->insert(content->end(), arr, arr + sizeof(arr) / sizeof(uint8));
    }

    return (std::make_shared<ProtocolBase>(ByteVector({0x00, 0x00}), ByteVector({0x00, 0x00, 0x00, 0x00}), CheckType, Type, DeviceType, Remain,
                                           Sp_Cmd->MainCmd, Sp_Cmd->SubCmd, ByteVector({0x00, 0x01}), *content))
        ->GetBytes();
  }
};
}  // namespace lidar
}  // namespace vanjee