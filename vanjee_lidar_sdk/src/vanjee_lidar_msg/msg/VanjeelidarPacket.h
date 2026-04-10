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

#ifndef VANJEE_LIDAR_MSG_MESSAGE_VANJEELIDARPACKET_H
#define VANJEE_LIDAR_MSG_MESSAGE_VANJEELIDARPACKET_H

#include <map>
#include <string>
#include <vector>

#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>
#include <ros/serialization.h>
#include <ros/types.h>
#include <std_msgs/Header.h>

namespace vanjee_lidar_msg {
template <class ContainerAllocator>
struct VanjeelidarPacket_ {
  typedef VanjeelidarPacket_<ContainerAllocator> Type;

  VanjeelidarPacket_() : header(), is_difop(0), is_frame_begin(0), data() {
  }
  VanjeelidarPacket_(const ContainerAllocator& _alloc) : header(_alloc), is_difop(0), is_frame_begin(0), data(_alloc) {
    (void)_alloc;
  }

  typedef ::std_msgs::Header_<ContainerAllocator> _header_type;
  _header_type header;

  typedef uint8_t _is_difop_type;
  _is_difop_type is_difop;

  typedef uint8_t _is_frame_begin_type;
  _is_frame_begin_type is_frame_begin;

  typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other> _data_type;
  _data_type data;

  typedef boost::shared_ptr<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>> Ptr;
  typedef boost::shared_ptr<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator> const> ConstPtr;
};

typedef ::vanjee_lidar_msg::VanjeelidarPacket_<std::allocator<void>> VanjeelidarPacket;

typedef boost::shared_ptr<::vanjee_lidar_msg::VanjeelidarPacket> VanjeelidarPacketPtr;
typedef boost::shared_ptr<::vanjee_lidar_msg::VanjeelidarPacket const> VanjeelidarPacketConstPtr;

template <typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>& v) {
  ros::message_operations::Printer<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>>::stream(s, "", v);
  return s;
}

template <typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator1>& lhs,
                const ::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator2>& rhs) {
  return lhs.header == rhs.header && lhs.is_difop == rhs.is_difop && lhs.is_frame_begin == rhs.is_frame_begin && lhs.data == rhs.data;
}

template <typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator1>& lhs,
                const ::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator2>& rhs) {
  return !(lhs == rhs);
}

}  // namespace vanjee_lidar_msg

namespace ros {
namespace message_traits {

template <class ContainerAllocator>
struct IsMessage<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>> : TrueType {};

template <class ContainerAllocator>
struct IsMessage<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator> const> : TrueType {};

template <class ContainerAllocator>
struct IsFixedSize<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>> : FalseType {};

template <class ContainerAllocator>
struct IsFixedSize<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator> const> : FalseType {};

template <class ContainerAllocator>
struct HasHeader<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>> : TrueType {};

template <class ContainerAllocator>
struct HasHeader<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator> const> : TrueType {};

template <class ContainerAllocator>
struct MD5Sum<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>> {
  static const char* value() {
    return "4b1cc155a9097c0cb935a7abf46d6eef";
  }

  static const char* value(const ::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>&) {
    return value();
  }
  static const uint64_t static_value1 = 0x4b1cc155a9097c0cULL;
  static const uint64_t static_value2 = 0xb935a7abf46d6eefULL;
};

template <class ContainerAllocator>
struct DataType<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>> {
  static const char* value() {
    return "vanjee_lidar_msg/VanjeelidarPacket";
  }

  static const char* value(const ::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>&) {
    return value();
  }
};

template <class ContainerAllocator>
struct Definition<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>> {
  static const char* value() {
    return "std_msgs/Header header \n"
           "uint8 is_difop\n"
           "uint8 is_frame_begin\n"
           "uint8[] data\n"
           "==================================================================="
           "=============\n"
           "MSG: std_msgs/Header\n"
           "# Standard metadata for higher-level stamped data types.\n"
           "# This is generally used to communicate timestamped data \n"
           "# in a particular coordinate frame.\n"
           "# \n"
           "# sequence ID: consecutively increasing ID \n"
           "uint32 seq\n"
           "#Two-integer timestamp that is expressed as:\n"
           "# * stamp.sec: seconds (stamp_secs) since epoch (in Python the "
           "variable is called 'secs')\n"
           "# * stamp.nsec: nanoseconds since stamp_secs (in Python the "
           "variable is called 'nsecs')\n"
           "# time-handling sugar is provided by the client library\n"
           "time stamp\n"
           "#Frame this data is associated with\n"
           "string frame_id\n";
  }

  static const char* value(const ::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>&) {
    return value();
  }
};

}  // namespace message_traits
}  // namespace ros

namespace ros {
namespace serialization {

template <class ContainerAllocator>
struct Serializer<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>> {
  template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m) {
    stream.next(m.header);
    stream.next(m.is_difop);
    stream.next(m.is_frame_begin);
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};

}  // namespace serialization
}  // namespace ros

namespace ros {
namespace message_operations {

template <class ContainerAllocator>
struct Printer<::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>> {
  template <typename Stream>
  static void stream(Stream& s, const std::string& indent, const ::vanjee_lidar_msg::VanjeelidarPacket_<ContainerAllocator>& v) {
    s << indent << "header: ";
    s << std::endl;
    Printer<::std_msgs::Header_<ContainerAllocator>>::stream(s, indent + "  ", v.header);
    s << indent << "is_difop: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_difop);
    s << indent << "is_frame_begin: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_frame_begin);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i) {
      s << indent << "  data[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.data[i]);
    }
  }
};

}  // namespace message_operations
}  // namespace ros

#endif
