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

#include "antbot_camera/usb_driver.hpp"

namespace antbot_camera
{

UsbDriver::UsbDriver(
  const std::string & position,
  rclcpp::Node * node,
  const cv::Size & image_size,
  int fps,
  const std::string & device_path,
  const std::string & frame_id,
  int rotation)
: CameraDriver(position, node),
  image_size_(image_size),
  fps_(fps),
  device_path_(device_path),
  frame_id_(frame_id),
  rotation_(rotation)
{
  frame_data_.frame_id = frame_id;
}

UsbDriver::~UsbDriver()
{
  release();
}

bool UsbDriver::initialize()
{
  video_capture_ = std::make_unique<cv::VideoCapture>(device_path_, cv::CAP_V4L);
  if (!video_capture_->isOpened()) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to open video capture device: %s (%s)",
      position_.c_str(), device_path_.c_str());
    set_status(DriverStatus::ERROR);
    return false;
  }
  video_capture_->set(cv::CAP_PROP_FRAME_WIDTH, image_size_.width);
  video_capture_->set(cv::CAP_PROP_FRAME_HEIGHT, image_size_.height);
  video_capture_->set(cv::CAP_PROP_FPS, fps_);

  color_image_ = cv::Mat::zeros(image_size_.height, image_size_.width, CV_8UC3);

  RCLCPP_INFO(
    node_->get_logger(), "Initialized usb camera: %s (%s)",
    position_.c_str(), device_path_.c_str());
  set_status(DriverStatus::OK);
  return true;
}

bool UsbDriver::update()
{
  if (get_status() != DriverStatus::OK) {
    return false;
  }

  if (!video_capture_ || !video_capture_->isOpened()) {
    set_status(DriverStatus::ERROR);
    return false;
  }

  if (!video_capture_->grab()) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to grab frame from usb camera: %s", device_path_.c_str());
    set_status(DriverStatus::ERROR);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(grab_mutex_);
    video_capture_->retrieve(color_image_);
    if (rotation_ >= 0) {
      cv::rotate(color_image_, color_image_, rotation_);
    }
  }

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    color_image_.copyTo(frame_data_.color);
  }
  return true;
}

void UsbDriver::release()
{
  if (video_capture_ && video_capture_->isOpened()) {
    video_capture_->release();
  }
  set_status(DriverStatus::UNINITIALIZED);
}

}  // namespace antbot_camera
