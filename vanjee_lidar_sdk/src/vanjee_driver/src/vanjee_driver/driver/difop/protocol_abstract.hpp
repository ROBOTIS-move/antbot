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

#include "cmd_class.hpp"
#include "params_abstract.hpp"
#include "protocol_base.hpp"
#include "vanjee_driver/common/super_header.hpp"

namespace vanjee {
namespace lidar {
class ProtocolAbstract {
 public:
  uint16 Idx;
  uint32 Timestamp;
  uint8 CheckType;
  uint8 Type;
  ByteVector DeviceType;
  ByteVector Remain;
  std::shared_ptr<CmdClass> Sp_Cmd;
  ByteVector CmdParams;
  std::shared_ptr<ParamsAbstract> Params;

 public:
  ProtocolAbstract(uint16 idx, uint32 timestamp, uint8 checkType, uint8 type, const ByteVector &deviceType, const ByteVector &remain,
                   std::shared_ptr<CmdClass> sp_cmd, const ByteVector &cmdParams, std::shared_ptr<ParamsAbstract> content) {
    Idx = idx;
    Timestamp = timestamp;
    CheckType = checkType;
    Type = type;
    DeviceType = deviceType.size() == 2 ? deviceType : ByteVector({0x00, 0x00});
    Remain = remain.size() == 8 ? remain : ByteVector({0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    Sp_Cmd = sp_cmd;
    CmdParams = cmdParams;
    Params = content;
  }

  ProtocolAbstract(uint8 checkType, uint8 type, const ByteVector &deviceType, const ByteVector &remain, std::shared_ptr<CmdClass> sp_cmd,
                   const ByteVector &cmdParams, std::shared_ptr<ParamsAbstract> content) {
    CheckType = checkType;
    Type = type;
    DeviceType = deviceType;
    Remain = remain;
    Sp_Cmd = sp_cmd;
    CmdParams = cmdParams;
    Params = content;
  }

  virtual bool Load(ProtocolBase protocol) {
    Idx = *reinterpret_cast<uint16 *>(protocol.Idx.data());
    Timestamp = *reinterpret_cast<uint32 *>(protocol.Timestamp.data());
    CheckType = protocol.CheckType;
    Type = protocol.Type;
    DeviceType = protocol.DeviceType;
    Sp_Cmd.reset(new CmdClass(protocol.MainCmd, protocol.SubCmd));
    CmdParams = protocol.CmdParams;
    Params->Load(protocol);
    return true;
  }

  virtual std::shared_ptr<std::vector<uint8>> GetRequest(std::shared_ptr<std::vector<uint8>> content = nullptr) {
    if (content == nullptr) {
      std::array<uint8, 4> arr = {0x00, 0x00, 0x00, 0x00};
      content = std::make_shared<std::vector<uint8>>(arr.begin(), arr.begin() + sizeof(arr) / sizeof(uint8));
    }

    ProtocolBase pb(Idx, Timestamp, CheckType, Type, DeviceType, Remain, Sp_Cmd->MainCmd, Sp_Cmd->SubCmd, CmdParams, *content);
    return pb.GetBytes();
  }

  virtual std::shared_ptr<std::vector<uint8>> SetRequest() {
    ProtocolBase pb(Idx, Timestamp, CheckType, Type, DeviceType, Remain, Sp_Cmd->MainCmd, Sp_Cmd->SubCmd, CmdParams, *Params->GetBytes());

    return pb.GetBytes();
  }
};
}  // namespace lidar
}  // namespace vanjee