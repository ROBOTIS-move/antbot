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

#include "vanjee_driver/driver/decoder/wlr716mini/protocol/difop_vanjee_716mini.hpp"
#include "vanjee_driver/driver/decoder/wlr718h/protocol/difop_vanjee_718h.hpp"
#include "vanjee_driver/driver/decoder/wlr719/protocol/difop_vanjee_719.hpp"
#include "vanjee_driver/driver/decoder/wlr719c/protocol/difop_vanjee_719c.hpp"
#include "vanjee_driver/driver/decoder/wlr719e/protocol/difop_vanjee_719e.hpp"
#include "vanjee_driver/driver/decoder/wlr720/protocol/difop_vanjee_720.hpp"
#include "vanjee_driver/driver/decoder/wlr720_32/protocol/difop_vanjee_720_32.hpp"
#include "vanjee_driver/driver/decoder/wlr721/protocol/difop_vanjee_721.hpp"
#include "vanjee_driver/driver/decoder/wlr722/protocol/difop_vanjee_722.hpp"
#include "vanjee_driver/driver/decoder/wlr733/protocol/difop_vanjee_733.hpp"
#include "vanjee_driver/driver/decoder/wlr750/protocol/difop_vanjee_750.hpp"
#include "vanjee_driver/driver/decoder/wlr760/protocol/difop_vanjee_760.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"
#include "vanjee_driver/driver/difop/protocol_base.hpp"

namespace vanjee {
namespace lidar {
class DifopFactory {
 public:
  static std::shared_ptr<DifopBase> createDifop(WJDriverParam param);
};

std::shared_ptr<DifopBase> DifopFactory::createDifop(WJDriverParam param) {
  std::shared_ptr<DifopBase> ret_ptr;
  ProtocolBase pb;

  switch (param.lidar_type) {
    case LidarType::vanjee_716mini:
      ret_ptr = std::make_shared<DifopVanjee716Mini>();
      break;
    case LidarType::vanjee_718h:
      ret_ptr = std::make_shared<DifopVanjee718H>();
      break;
    case LidarType::vanjee_719:
      ret_ptr = std::make_shared<DifopVanjee719>(param.input_param.connect_type);
      break;
    case LidarType::vanjee_719c:
      ret_ptr = std::make_shared<DifopVanjee719C>(param.input_param.connect_type);
      break;
    case LidarType::vanjee_719e:
      ret_ptr = std::make_shared<DifopVanjee719E>();
      break;
    case LidarType::vanjee_720:
    case LidarType::vanjee_720_16:
      ret_ptr = std::make_shared<DifopVanjee720>();
      break;
    case LidarType::vanjee_720_32:
      ret_ptr = std::make_shared<DifopVanjee720_32>();
      break;
    case LidarType::vanjee_721:
      ret_ptr = std::make_shared<DifopVanjee721>();
      break;
    case LidarType::vanjee_722:
      ret_ptr = std::make_shared<DifopVanjee722>();
      break;
    case LidarType::vanjee_733:
      ret_ptr = std::make_shared<DifopVanjee733>();
      pb.Idx.resize(4);
      ret_ptr->setOrgProtocolBase(pb);
      ProtocolBase::ByteOrder = 2;
      break;
    case LidarType::vanjee_750:
      ret_ptr = std::make_shared<DifopVanjee750>();
      break;
    case LidarType::vanjee_760:
      ret_ptr = std::make_shared<DifopVanjee760>();
      break;

    default:
      exit(-1);
  }

  return ret_ptr;
}
}  // namespace lidar
}  // namespace vanjee