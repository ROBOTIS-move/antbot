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

#ifndef ANTBOT_CAMERA__VENDOR__CAMERA_VENDOR_HANDLER_HPP_
#define ANTBOT_CAMERA__VENDOR__CAMERA_VENDOR_HANDLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "opencv2/core/types.hpp"

namespace antbot_camera
{
namespace vendor
{

struct CalibrationData
{
  std::vector<double> intrinsic;
  bool valid = false;
};

class CameraVendorHandler
{
public:
  virtual ~CameraVendorHandler() = default;

  virtual int get_vendor_device_id(const std::string & port_id) = 0;
  virtual bool supports_rom_mode() const = 0;
  virtual bool supports_framerate_setting() const = 0;
  virtual bool read_calibration_data(
    const std::string & port_id,
    int internal_id,
    const std::string & distortion_model,
    const cv::Size & image_size,
    CalibrationData & data) = 0;
  virtual std::string get_device_path(const std::string & port_id) = 0;
  virtual void cleanup() = 0;
};

std::shared_ptr<CameraVendorHandler> create_vendor_handler(
  const std::string & vendor_name,
  const std::string & port_root = "/sys/class/video4linux");

}  // namespace vendor
}  // namespace antbot_camera
#endif  // ANTBOT_CAMERA__VENDOR__CAMERA_VENDOR_HANDLER_HPP_
