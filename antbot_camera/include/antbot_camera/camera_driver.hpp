// Copyright 2026 ROBOTIS AI CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Jaehun Park

#ifndef ANTBOT_CAMERA__CAMERA_DRIVER_HPP_
#define ANTBOT_CAMERA__CAMERA_DRIVER_HPP_

#include <atomic>
#include <mutex>
#include <string>

#include "opencv2/core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace antbot_camera
{

enum class DriverStatus { UNINITIALIZED, OK, ERROR };

struct FrameData
{
  cv::Mat color;                              // BGR8
  cv::Mat depth;                              // 16UC1 mm (Gemini336L only)
  sensor_msgs::msg::CameraInfo color_info;
  sensor_msgs::msg::CameraInfo depth_info;    // Gemini336L only
  std::string frame_id;
};

class CameraDriver
{
public:
  CameraDriver(const std::string & position, rclcpp::Node * node);
  virtual ~CameraDriver() = default;

  virtual bool initialize() = 0;
  virtual bool update() = 0;
  virtual void release() = 0;

  DriverStatus get_status() const;
  const FrameData & get_frame_data() const;
  std::string get_position() const;

protected:
  void set_status(DriverStatus s);
  rclcpp::Node * node_;
  FrameData frame_data_;
  std::string position_;
  mutable std::mutex frame_mutex_;

private:
  std::atomic<DriverStatus> status_{DriverStatus::UNINITIALIZED};
};

}  // namespace antbot_camera
#endif  // ANTBOT_CAMERA__CAMERA_DRIVER_HPP_
