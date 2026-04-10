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

#ifndef ANTBOT_CAMERA__USB_DRIVER_HPP_
#define ANTBOT_CAMERA__USB_DRIVER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"

#include "antbot_camera/camera_driver.hpp"

namespace antbot_camera
{

class UsbDriver : public CameraDriver
{
public:
  UsbDriver(
    const std::string & position,
    rclcpp::Node * node,
    const cv::Size & image_size,
    int fps,
    const std::string & device_path,
    const std::string & frame_id,
    int rotation);
  ~UsbDriver() override;

  bool initialize() override;
  bool update() override;
  void release() override;

private:
  cv::Size image_size_;
  int fps_;
  std::string device_path_;
  std::string frame_id_;
  int rotation_;  // -1 for no rotation, or cv::ROTATE_* enum

  std::unique_ptr<cv::VideoCapture> video_capture_;
  cv::Mat color_image_;
  std::mutex grab_mutex_;
};

}  // namespace antbot_camera
#endif  // ANTBOT_CAMERA__USB_DRIVER_HPP_
