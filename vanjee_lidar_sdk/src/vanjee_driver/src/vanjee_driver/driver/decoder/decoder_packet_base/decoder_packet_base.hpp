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

#include <cstdint>
#include <vector>

#include <vanjee_driver/driver/decoder/wlr719e/protocol/frames/cmd_repository_719e.hpp>
#include <vanjee_driver/driver/decoder/wlr722/protocol/frames/cmd_repository_722.hpp>
#include <vanjee_driver/driver/decoder/wlr750/protocol/frames/cmd_repository_750.hpp>
#include <vanjee_driver/driver/decoder/wlr760/protocol/frames/cmd_repository_760.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>

// #include "vanjee_driver/driver/decoder/decoder_packet_base/decoder_packet_general_version_base.hpp"
#include "vanjee_driver/driver/decoder/decoder_packet_base/decoder_packet_general_version_v1_1.hpp"

using namespace vanjee::lidar;

namespace vanjee {
namespace lidar {
template <typename T_PointCloud, typename T_DataBlock, typename T_DataUnit>
class DecoderPacketBase {
 public:
  std::shared_ptr<DecoderPacketGeneralVersionBase<T_PointCloud, T_DataBlock, T_DataUnit>> decoder_packet_general_version_base_ptr_ = nullptr;

 private:
  Decoder<T_PointCloud>* decoder_ptr_ = nullptr;
  const std::vector<uint16_t> device_type_vector_ = {0x190E, 0x0100, 0x0101, 0x0102, 0x0200, 0x0201};

 public:
  DecoderPacketBase(Decoder<T_PointCloud>* decoder_ptr) {
    decoder_ptr_ = decoder_ptr;
  }

  bool decoderPacket(const uint8_t* buf, size_t size,
                     std::function<void(DataBlockAngleAndTimestampInfo&, T_DataBlock*)> update_angle_and_timestamp_info_callback,
                     std::function<void(T_DataUnit*, PointInfo&)> decoder_data_unit_callback,
                     std::function<void()> point_cloud_algorithm_callbck = nullptr) {
    bool ret = false;
    VanjeeLidarPointCloudPacketBaseHeader& vanjee_lidar_point_cloud_packet_base_header = *(VanjeeLidarPointCloudPacketBaseHeader*)buf;
    if (!(vanjee_lidar_point_cloud_packet_base_header.header_[0] == 0xFF && vanjee_lidar_point_cloud_packet_base_header.header_[1] == 0xCC))
      return false;
    uint16_t lidar_type = vanjee_lidar_point_cloud_packet_base_header.getLidarType();
    uint16_t protocol_version = vanjee_lidar_point_cloud_packet_base_header.protocol_version_.getVersionId();

    switch (lidar_type) {
      case 0x190E:
        ret = decoderWlr719ePacket(buf, size, protocol_version, update_angle_and_timestamp_info_callback, decoder_data_unit_callback,
                                   point_cloud_algorithm_callbck);
        break;

      case 0x000B:
        ret = decoderWlr722Packet(buf, size, protocol_version, update_angle_and_timestamp_info_callback, decoder_data_unit_callback,
                                  point_cloud_algorithm_callbck);
        break;

      case 0x0101:
        ret = decoderWlr750bPacket(buf, size, protocol_version, update_angle_and_timestamp_info_callback, decoder_data_unit_callback,
                                   point_cloud_algorithm_callbck);
        break;

      case 0x0201:
        ret = decoderWlr760Packet(buf, size, protocol_version, update_angle_and_timestamp_info_callback, decoder_data_unit_callback,
                                  point_cloud_algorithm_callbck);
        break;

      default:
        break;
    }
    return ret;
  }

  bool decoderWlr719ePacket(const uint8_t* buf, size_t size, uint16_t protocol_version,
                            std::function<void(DataBlockAngleAndTimestampInfo&, T_DataBlock*)> update_angle_and_timestamp_info_callback,
                            std::function<void(T_DataUnit*, PointInfo&)> decoder_data_unit_callback,
                            std::function<void()> point_cloud_algorithm_callbck) {
    std::vector<uint16_t> wlr719e_protocol_version_list = {0x0101};
    bool ret = false;
    if (protocol_version > wlr719e_protocol_version_list[wlr719e_protocol_version_list.size() - 1]) {
      // protocol handshake
      if (decoder_ptr_->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
        (*(decoder_ptr_
               ->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[(CmdRepository719E::CreateInstance()->set_protocol_version_cmd_id_ptr_)->GetCmdKey()]
            .setStopFlag(false);
      return false;
    } else {
      if (decoder_ptr_->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
        (*(decoder_ptr_
               ->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[(CmdRepository719E::CreateInstance()->set_protocol_version_cmd_id_ptr_)->GetCmdKey()]
            .setStopFlag(true);
    }

    switch (protocol_version) {
      case 0x0101:
        if (decoder_packet_general_version_base_ptr_ == nullptr)
          decoder_packet_general_version_base_ptr_ =
              std::make_shared<DecoderPacketGeneralVersionV1_1<T_PointCloud, T_DataBlock, T_DataUnit>>(decoder_ptr_);
        ret = decoder_packet_general_version_base_ptr_->decoderPacket(buf, size, update_angle_and_timestamp_info_callback, decoder_data_unit_callback,
                                                                      point_cloud_algorithm_callbck);
        break;

      default:
        break;
    }
    return ret;
  }

  bool decoderWlr722Packet(const uint8_t* buf, size_t size, uint16_t protocol_version,
                           std::function<void(DataBlockAngleAndTimestampInfo&, T_DataBlock*)> update_angle_and_timestamp_info_callback,
                           std::function<void(T_DataUnit*, PointInfo&)> decoder_data_unit_callback,
                           std::function<void()> point_cloud_algorithm_callbck) {
    std::vector<uint16_t> wlr722_protocol_version_list = {0x0100};
    bool ret = false;
    if (protocol_version > wlr722_protocol_version_list[wlr722_protocol_version_list.size() - 1]) {
      // protocol handshake
      if (decoder_ptr_->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
        (*(decoder_ptr_
               ->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[(CmdRepository722::CreateInstance()->set_protocol_version_cmd_id_ptr_)->GetCmdKey()]
            .setStopFlag(false);
      return false;
    } else {
      if (decoder_ptr_->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
        (*(decoder_ptr_
               ->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[(CmdRepository722::CreateInstance()->set_protocol_version_cmd_id_ptr_)->GetCmdKey()]
            .setStopFlag(true);
    }

    switch (protocol_version) {
      case 0x0100:
        if (decoder_packet_general_version_base_ptr_ == nullptr)
          decoder_packet_general_version_base_ptr_ =
              std::make_shared<DecoderPacketGeneralVersionBase<T_PointCloud, T_DataBlock, T_DataUnit>>(decoder_ptr_);
        ret = decoder_packet_general_version_base_ptr_->decoderPacket(buf, size, update_angle_and_timestamp_info_callback, decoder_data_unit_callback,
                                                                      point_cloud_algorithm_callbck);
        break;

      default:
        break;
    }
    return ret;
  }

  bool decoderWlr750bPacket(const uint8_t* buf, size_t size, uint16_t protocol_version,
                            std::function<void(DataBlockAngleAndTimestampInfo&, T_DataBlock*)> update_angle_and_timestamp_info_callback,
                            std::function<void(T_DataUnit*, PointInfo&)> decoder_data_unit_callback,
                            std::function<void()> point_cloud_algorithm_callbck) {
    std::vector<uint16_t> wlr750b_protocol_version_list = {0x0100, 0x0102};
    bool ret = false;
    if (protocol_version > wlr750b_protocol_version_list[wlr750b_protocol_version_list.size() - 1]) {
      // protocol handshake
      if (decoder_ptr_->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
        (*(decoder_ptr_
               ->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[(CmdRepository750::CreateInstance()->set_protocol_version_cmd_id_ptr_)->GetCmdKey()]
            .setStopFlag(false);
      return false;
    } else {
      if (decoder_ptr_->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
        (*(decoder_ptr_
               ->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[(CmdRepository750::CreateInstance()->set_protocol_version_cmd_id_ptr_)->GetCmdKey()]
            .setStopFlag(true);
    }

    switch (protocol_version) {
      case 0x0100:
      case 0x0102:
        if (decoder_packet_general_version_base_ptr_ == nullptr)
          decoder_packet_general_version_base_ptr_ =
              std::make_shared<DecoderPacketGeneralVersionBase<T_PointCloud, T_DataBlock, T_DataUnit>>(decoder_ptr_);
        ret = decoder_packet_general_version_base_ptr_->decoderPacket(buf, size, update_angle_and_timestamp_info_callback, decoder_data_unit_callback,
                                                                      point_cloud_algorithm_callbck);
        break;

      default:
        break;
    }
    return ret;
  }

  bool decoderWlr760Packet(const uint8_t* buf, size_t size, uint16_t protocol_version,
                           std::function<void(DataBlockAngleAndTimestampInfo&, T_DataBlock*)> update_angle_and_timestamp_info_callback,
                           std::function<void(T_DataUnit*, PointInfo&)> decoder_data_unit_callback,
                           std::function<void()> point_cloud_algorithm_callbck) {
    std::vector<uint16_t> wlr760_protocol_version_list = {0x0100, 0x0101};
    bool ret = false;
    if (protocol_version > wlr760_protocol_version_list[wlr760_protocol_version_list.size() - 1]) {
      // protocol handshake
      if (decoder_ptr_->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
        (*(decoder_ptr_
               ->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[CmdRepository760::CreateInstance()->set_protocol_version_cmd_id_ptr_->GetCmdKey()]
            .setStopFlag(false);
      return false;
    } else {
      if (decoder_ptr_->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
        (*(decoder_ptr_
               ->Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[(CmdRepository760::CreateInstance()->set_protocol_version_cmd_id_ptr_)->GetCmdKey()]
            .setStopFlag(true);
    }

    switch (protocol_version) {
      case 0x0100:
      case 0x0101:
        if (decoder_packet_general_version_base_ptr_ == nullptr)
          decoder_packet_general_version_base_ptr_ =
              std::make_shared<DecoderPacketGeneralVersionBase<T_PointCloud, T_DataBlock, T_DataUnit>>(decoder_ptr_);
        ret = decoder_packet_general_version_base_ptr_->decoderPacket(buf, size, update_angle_and_timestamp_info_callback, decoder_data_unit_callback,
                                                                      point_cloud_algorithm_callbck);
        break;

      default:
        break;
    }
    return ret;
  }
};
}  // namespace lidar
}  // namespace vanjee
