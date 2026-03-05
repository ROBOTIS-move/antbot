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
#include "source/source.hpp"
#ifdef ROS_FOUND
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
namespace vanjee {
namespace lidar {
inline sensor_msgs::PointCloud2 toRosMsgLight(const LidarPointCloudMsg &vanjee_msg, const std::string &frame_id) {
  sensor_msgs::PointCloud2 ros_msg;
  uint16_t offset = 0;

  ros_msg.width = vanjee_msg.height;
  ros_msg.height = vanjee_msg.width;

  sensor_msgs::PointField x_field, y_field, z_field;
  x_field.name = "x";
  x_field.offset = offset;
  x_field.datatype = sensor_msgs::PointField::FLOAT32;
  x_field.count = 1;  // 为每个点的x坐标
  offset += x_field.count * sizeOfPointField(sensor_msgs::PointField::FLOAT32);
  ros_msg.fields.push_back(x_field);

  y_field.name = "y";
  y_field.offset = offset;
  y_field.datatype = sensor_msgs::PointField::FLOAT32;
  y_field.count = 1;  // 为每个点的y坐标
  offset += y_field.count * sizeOfPointField(sensor_msgs::PointField::FLOAT32);
  ros_msg.fields.push_back(y_field);

  z_field.name = "z";
  z_field.offset = offset;
  z_field.datatype = sensor_msgs::PointField::FLOAT32;
  z_field.count = 1;  // 为每个点的z坐标
  offset += z_field.count * sizeOfPointField(sensor_msgs::PointField::FLOAT32);
  ros_msg.fields.push_back(z_field);

#ifdef POINT_TYPE_XYZI
  sensor_msgs::PointField intensity_field;

  intensity_field.name = "intensity";
  intensity_field.offset = offset;
  intensity_field.datatype = sensor_msgs::PointField::FLOAT32;
  intensity_field.count = 1;  // 为每个点的intensity
  offset += intensity_field.count * sizeOfPointField(sensor_msgs::PointField::FLOAT32);
  ros_msg.fields.push_back(intensity_field);
#endif

#ifdef POINT_TYPE_XYZIRT
  sensor_msgs::PointField intensity_field, ring_field, timestamp_field;

  intensity_field.name = "intensity";
  intensity_field.offset = offset;
  intensity_field.datatype = sensor_msgs::PointField::FLOAT32;
  intensity_field.count = 1;  // 为每个点的intensity
  offset += intensity_field.count * sizeOfPointField(sensor_msgs::PointField::FLOAT32);
  ros_msg.fields.push_back(intensity_field);

  ring_field.name = "ring";
  ring_field.offset = offset;
  ring_field.datatype = sensor_msgs::PointField::UINT16;
  ring_field.count = 1;  // 为每个点的通道号
  offset += ring_field.count * sizeOfPointField(sensor_msgs::PointField::UINT16);
  ros_msg.fields.push_back(ring_field);

  timestamp_field.name = "timestamp";
  timestamp_field.offset = offset;
  timestamp_field.datatype = sensor_msgs::PointField::FLOAT64;
  timestamp_field.count = 1;  // 为每个点的时间戳
  offset += timestamp_field.count * sizeOfPointField(sensor_msgs::PointField::FLOAT64);
  ros_msg.fields.push_back(timestamp_field);
#endif

  ros_msg.is_dense = vanjee_msg.is_dense;
  ros_msg.is_bigendian = false;
  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.point_step * ros_msg.width;

  ros_msg.header.seq = vanjee_msg.seq;
  ros_msg.header.frame_id = frame_id;
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(vanjee_msg.timestamp);
  uint32_t size = ros_msg.row_step * ros_msg.height;
  ros_msg.data.resize(size);
  std::memcpy(ros_msg.data.data(), vanjee_msg.points.data(), size);

  return ros_msg;
}

inline sensor_msgs::PointCloud2 toRosMsg(const LidarPointCloudMsg &vanjee_msg, const std::string &frame_id, bool send_by_rows) {
  if (send_by_rows) {
    sensor_msgs::PointCloud2 ros_msg;

    int fields = 4;
#ifdef POINT_TYPE_XYZIRT
    fields = 6;
#endif

    ros_msg.fields.clear();
    ros_msg.fields.reserve(fields);

    ros_msg.width = vanjee_msg.width;
    ros_msg.height = vanjee_msg.height;

    int offset = 0;
    offset = addPointField(ros_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(ros_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(ros_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
#ifdef POINT_TYPE_XYZI
    offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
#endif
#ifdef POINT_TYPE_XYZIRT
    offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
    offset = addPointField(ros_msg, "ring", 1, sensor_msgs::PointField::UINT16, offset);
    offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::PointField::FLOAT64, offset);
#endif

    ros_msg.point_step = offset;
    ros_msg.row_step = ros_msg.width * ros_msg.point_step;
    ros_msg.is_dense = vanjee_msg.is_dense;
    ros_msg.data.resize(ros_msg.point_step * vanjee_msg.points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
#ifdef POINT_TYPE_XYZI
    sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
#endif
#ifdef POINT_TYPE_XYZIRT
    sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
    sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
#endif

    for (size_t i = 0; i < vanjee_msg.height; i++) {
      for (size_t j = 0; j < vanjee_msg.width; j++) {
        const LidarPointCloudMsg::PointT &point = vanjee_msg.points[i + j * vanjee_msg.height];

        *iter_x_ = point.x;
        *iter_y_ = point.y;
        *iter_z_ = point.z;

        ++iter_x_;
        ++iter_y_;
        ++iter_z_;
#ifdef POINT_TYPE_XYZI
        *iter_intensity_ = point.intensity;
        ++iter_intensity_;
#endif

#ifdef POINT_TYPE_XYZIRT
        *iter_intensity_ = point.intensity;
        *iter_ring_ = point.ring;
        *iter_timestamp_ = point.timestamp;

        ++iter_intensity_;
        ++iter_ring_;
        ++iter_timestamp_;
#endif
      }
    }

    ros_msg.header.seq = vanjee_msg.seq;
    ros_msg.header.stamp = ros_msg.header.stamp.fromSec(vanjee_msg.timestamp);
    ros_msg.header.frame_id = frame_id;

    return ros_msg;
  } else {
    return toRosMsgLight(vanjee_msg, frame_id);
  }
}

/// @brief DestinationPointCloudRos publish point clouds through ROS topic
/// '/vanjee_lidar points'
class DestinationPointCloudRos : public DestinationPointCloud {
 private:
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher pub_;
  std::string frame_id_;
  bool send_by_rows_;

 public:
  /// @brief Initialize the 'DestinationPacketRos' instance
  virtual void init(const YAML::Node &config);
  /// @brief Publish point cloud through ROS topic '/vanjee_lidar points'
  virtual void sendPointCloud(const LidarPointCloudMsg &msg);
  virtual ~DestinationPointCloudRos() = default;
};

inline void DestinationPointCloudRos::init(const YAML::Node &config) {
  yamlRead<bool>(config["ros"], "ros_send_by_rows", send_by_rows_, false);

  bool dense_points;
  yamlRead<bool>(config["driver"], "dense_points", dense_points, false);
  yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar");
  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_topic, "vanjee_lidar_points");

  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  pub_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_topic, 10);
}
inline void DestinationPointCloudRos::sendPointCloud(const LidarPointCloudMsg &msg) {
  pub_.publish(toRosMsg(msg, frame_id_, send_by_rows_));
}
}  // namespace lidar

}  // namespace vanjee

#endif
#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "source/source_ros_msg_delegate.hpp"
namespace vanjee {
namespace lidar {
inline sensor_msgs::msg::PointCloud2 toRosMsgLight(const LidarPointCloudMsg &vanjee_msg, const std::string &frame_id) {
  sensor_msgs::msg::PointCloud2 ros_msg;
  uint16_t offset = 0;

  ros_msg.width = vanjee_msg.height;
  ros_msg.height = vanjee_msg.width;

  sensor_msgs::msg::PointField x_field, y_field, z_field;
  x_field.name = "x";
  x_field.offset = offset;
  x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  x_field.count = 1;  // 为每个点的x坐标
  offset += x_field.count * sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32);
  ros_msg.fields.push_back(x_field);

  y_field.name = "y";
  y_field.offset = offset;
  y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  y_field.count = 1;  // 为每个点的y坐标
  offset += y_field.count * sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32);
  ros_msg.fields.push_back(y_field);

  z_field.name = "z";
  z_field.offset = offset;
  z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  z_field.count = 1;  // 为每个点的z坐标
  offset += z_field.count * sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32);
  ros_msg.fields.push_back(z_field);

#ifdef POINT_TYPE_XYZI
  sensor_msgs::msg::PointField intensity_field;

  intensity_field.name = "intensity";
  intensity_field.offset = offset;
  intensity_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  intensity_field.count = 1;  // 为每个点的intensity
  offset += intensity_field.count * sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32);
  ros_msg.fields.push_back(intensity_field);
#endif

#ifdef POINT_TYPE_XYZIRT
  sensor_msgs::msg::PointField intensity_field, ring_field, timestamp_field;

  intensity_field.name = "intensity";
  intensity_field.offset = offset;
  intensity_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  intensity_field.count = 1;  // 为每个点的intensity
  offset += intensity_field.count * sizeOfPointField(sensor_msgs::msg::PointField::FLOAT32);
  ros_msg.fields.push_back(intensity_field);

  ring_field.name = "ring";
  ring_field.offset = offset;
  ring_field.datatype = sensor_msgs::msg::PointField::UINT16;
  ring_field.count = 1;  // 为每个点的通道号
  offset += ring_field.count * sizeOfPointField(sensor_msgs::msg::PointField::UINT16);
  ros_msg.fields.push_back(ring_field);

  timestamp_field.name = "timestamp";
  timestamp_field.offset = offset;
  timestamp_field.datatype = sensor_msgs::msg::PointField::FLOAT64;
  timestamp_field.count = 1;  // 为每个点的时间戳
  offset += timestamp_field.count * sizeOfPointField(sensor_msgs::msg::PointField::FLOAT64);
  ros_msg.fields.push_back(timestamp_field);
#endif

  ros_msg.is_dense = vanjee_msg.is_dense;
  ros_msg.is_bigendian = false;
  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.point_step * ros_msg.width;

  ros_msg.header.frame_id = frame_id;
  ros_msg.header.stamp.sec = (uint32_t)floor(vanjee_msg.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)round((vanjee_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  uint32_t size = ros_msg.row_step * ros_msg.height;
  ros_msg.data.resize(size);
  std::memcpy(ros_msg.data.data(), vanjee_msg.points.data(), size);

  return ros_msg;
}

inline sensor_msgs::msg::PointCloud2 toRosMsg(const LidarPointCloudMsg &vanjee_msg, const std::string &frame_id, bool send_by_rows) {
  if (send_by_rows) {
    sensor_msgs::msg::PointCloud2 ros_msg;

    int fields = 4;
#ifdef POINT_TYPE_XYZIRT
    fields = 6;
#endif
    ros_msg.fields.clear();
    ros_msg.fields.reserve(fields);

    ros_msg.width = vanjee_msg.width;
    ros_msg.height = vanjee_msg.height;

    int offset = 0;
    offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
    offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
    offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
#ifdef POINT_TYPE_XYZI
    offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
#endif
#ifdef POINT_TYPE_XYZIRT
    offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
    offset = addPointField(ros_msg, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
    offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);
#endif

    ros_msg.point_step = offset;
    ros_msg.row_step = ros_msg.width * ros_msg.point_step;
    ros_msg.is_dense = vanjee_msg.is_dense;
    ros_msg.data.resize(ros_msg.point_step * vanjee_msg.points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
#ifdef POINT_TYPE_XYZI
    sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
#endif
#ifdef POINT_TYPE_XYZIRT
    sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
    sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
#endif

    for (size_t i = 0; i < vanjee_msg.height; i++) {
      for (size_t j = 0; j < vanjee_msg.width; j++) {
        const LidarPointCloudMsg::PointT &point = vanjee_msg.points[i + j * vanjee_msg.height];

        *iter_x_ = point.x;
        *iter_y_ = point.y;
        *iter_z_ = point.z;

        ++iter_x_;
        ++iter_y_;
        ++iter_z_;
#ifdef POINT_TYPE_XYZI
        *iter_intensity_ = point.intensity;
        ++iter_intensity_;
#endif

#ifdef POINT_TYPE_XYZIRT
        *iter_intensity_ = point.intensity;
        *iter_ring_ = point.ring;
        *iter_timestamp_ = point.timestamp;

        ++iter_intensity_;
        ++iter_ring_;
        ++iter_timestamp_;
#endif
      }
    }

    ros_msg.header.stamp.sec = (uint32_t)floor(vanjee_msg.timestamp);
    ros_msg.header.stamp.nanosec = (uint32_t)round((vanjee_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
    ros_msg.header.frame_id = frame_id;

    return ros_msg;
  } else {
    return toRosMsgLight(vanjee_msg, frame_id);
  }
}

/// @brief DestinationPointCloudRos publish point clouds through ROS topic
/// '/vanjee_lidar points'
class DestinationPointCloudRos : virtual public DestinationPointCloud {
 private:
  using PointCloud2MsgPubPtr = std::shared_ptr<VanjeeLidarSdkPublishRosMsg<sensor_msgs::msg::PointCloud2>>;
  PointCloud2MsgPubPtr pointcloud2_msg_pub_ptr_;
  std::string frame_id_;
  bool send_by_rows_;

 public:
  /// @brief Initialize the 'DestinationPacketRos' instance
  virtual void init(const YAML::Node &config);
  /// @brief Publish point cloud through ROS topic '/vanjee_lidar points'
  virtual void sendPointCloud(const LidarPointCloudMsg &msg);
  virtual ~DestinationPointCloudRos() = default;
};

inline void DestinationPointCloudRos::init(const YAML::Node &config) {
  yamlRead<bool>(config["ros"], "ros_send_by_rows", send_by_rows_, false);
  yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar");
  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_topic, "vanjee_lidar_points");

  pointcloud2_msg_pub_ptr_ = VanjeeLidarSdkNode::CreateInstance()->GetPointCloud2MsgPublisher(ros_send_topic);
}
inline void DestinationPointCloudRos::sendPointCloud(const LidarPointCloudMsg &msg) {
  sensor_msgs::msg::PointCloud2 point_cloud2 = toRosMsg(msg, frame_id_, send_by_rows_);
  pointcloud2_msg_pub_ptr_->PublishMsg(point_cloud2);
}
}  // namespace lidar

}  // namespace vanjee

#endif