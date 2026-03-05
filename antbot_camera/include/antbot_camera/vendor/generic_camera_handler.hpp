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

#ifndef ANTBOT_CAMERA__VENDOR__GENERIC_CAMERA_HANDLER_HPP_
#define ANTBOT_CAMERA__VENDOR__GENERIC_CAMERA_HANDLER_HPP_

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "opencv2/core/types.hpp"
#include "rclcpp/rclcpp.hpp"

#include "antbot_camera/vendor/camera_vendor_handler.hpp"

namespace antbot_camera
{
namespace vendor
{

class GenericCameraHandler : public CameraVendorHandler
{
public:
  explicit GenericCameraHandler(const std::string & port_root);
  virtual ~GenericCameraHandler() = default;

  int get_vendor_device_id(const std::string & port_id) override;
  bool supports_rom_mode() const override {return false;}
  bool supports_framerate_setting() const override {return true;}
  bool read_calibration_data(
    const std::string & port_id,
    int internal_id,
    const std::string & distortion_model,
    const cv::Size & image_size,
    CalibrationData & data) override;
  std::string get_device_path(const std::string & port_id) override;
  void cleanup() override;

private:
  std::string port_root_;
  rclcpp::Logger logger_;
};

}  // namespace vendor
}  // namespace antbot_camera
#endif  // ANTBOT_CAMERA__VENDOR__GENERIC_CAMERA_HANDLER_HPP_
