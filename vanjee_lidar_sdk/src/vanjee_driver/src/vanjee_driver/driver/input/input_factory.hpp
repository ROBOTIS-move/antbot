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
#include <vanjee_driver/driver/input/input.hpp>
#include <vanjee_driver/driver/input/input_socket.hpp>

#ifndef DISABLE_PCAP_PARSE
#include <vanjee_driver/driver/input/input_pcap.hpp>
#endif

namespace vanjee {
namespace lidar {
class InputFactory {
 public:
  static std::shared_ptr<Input> createInput(InputType type, const WJInputParam &param, double sec_to_delay);
};

inline std::shared_ptr<Input> InputFactory::createInput(InputType type, const WJInputParam &param, double sec_to_delay) {
  std::shared_ptr<Input> input;
  switch (type) {
    case InputType::ONLINE_LIDAR: {
      if (param.connect_type == 2)
        input = std::make_shared<InputTcpSocket>(param);
      else
        input = std::make_shared<InputUdpSocket>(param);
    } break;

#ifndef DISABLE_PCAP_PARSE
    case InputType::PCAP_FILE: {
      input = std::make_shared<InputPcap>(param, sec_to_delay);
    } break;

#endif
    default:
      WJ_ERROR << "Wrong Input Type " << type << "." << WJ_REND;
      if (type == InputType::PCAP_FILE) {
        WJ_ERROR << "To use InputType::PCAP_FILE, please do not specify the "
                    "make option DISABLE_PCAP_PARSE."
                 << WJ_REND;
      }
      exit(-1);
  }
  return input;
}
}  // namespace lidar
}  // namespace vanjee
