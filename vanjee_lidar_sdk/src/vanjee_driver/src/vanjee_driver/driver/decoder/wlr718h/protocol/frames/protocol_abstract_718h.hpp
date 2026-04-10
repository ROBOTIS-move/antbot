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

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"

namespace vanjee {
namespace lidar {
class ProtocolAbstract718H : public ProtocolAbstract {
 public:
  ProtocolAbstract718H(const std::shared_ptr<CmdClass> sp_cmd, std::shared_ptr<ParamsAbstract> content, uint16 idx = 0, uint32 timestamp = 0,
                       uint8 checkType = 1, uint8 type = 1)
      : ProtocolAbstract(checkType, type, {0x00, 0x0E}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, sp_cmd, {0x00, 0x00}, content) {
  }

  virtual bool Load(ProtocolBase protocol) {
    CheckType = protocol.CheckType;
    Type = protocol.Type;
    Sp_Cmd.reset(new CmdClass(protocol.MainCmd, protocol.SubCmd));
    CmdParams = protocol.CmdParams;
    Params->Load(protocol);
    return true;
  }

  virtual std::shared_ptr<std::vector<uint8>> GetRequest(std::shared_ptr<std::vector<uint8>> content = nullptr) override {
    if (content == nullptr) {
      const uint8 arr[] = {0x00, 0x00, 0x00, 0x00};
      content = std::make_shared<std::vector<uint8>>();
      content->insert(content->end(), arr, arr + sizeof(arr) / sizeof(uint8));
    }

    return (std::make_shared<ProtocolBase>(ByteVector({0x00, 0x00}), ByteVector({0x00, 0x00, 0x00, 0x00}), CheckType, Type, DeviceType, Remain,
                                           Sp_Cmd->MainCmd, Sp_Cmd->SubCmd, CmdParams, *content))
        ->GetBytes();
  }

  virtual std::shared_ptr<std::vector<uint8>> SetRequest() override {
    return (std::make_shared<ProtocolBase>(ByteVector({0x00, 0x00}), ByteVector({0x00, 0x00, 0x00, 0x00}), CheckType, Type, DeviceType, Remain,
                                           Sp_Cmd->MainCmd, Sp_Cmd->SubCmd, CmdParams, *Params->GetBytes()))
        ->GetBytes();
  }
};
}  // namespace lidar
}  // namespace vanjee