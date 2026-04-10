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

#include "antbot_camera/vendor/generic_camera_handler.hpp"

namespace antbot_camera
{
namespace vendor
{

GenericCameraHandler::GenericCameraHandler(const std::string & port_root)
: port_root_(port_root),
  logger_(rclcpp::get_logger("GenericCameraHandler"))
{
}

int GenericCameraHandler::get_vendor_device_id(const std::string & port_id)
{
  RCLCPP_INFO(logger_, "Generic camera handler prepared for port: %s", port_id.c_str());
  return 0;
}

bool GenericCameraHandler::read_calibration_data(
  const std::string & /* port_id */,
  int /* internal_id */,
  const std::string & /* distortion_model */,
  const cv::Size & /* image_size */,
  CalibrationData & data)
{
  data.valid = false;
  RCLCPP_INFO(logger_, "Generic camera does not support ROM calibration");
  return false;
}

std::string GenericCameraHandler::get_device_path(const std::string & port_id)
{
  try {
    std::vector<std::string> entries;
    for (const auto & entry : std::filesystem::directory_iterator(port_root_)) {
      entries.push_back(entry.path().filename().string());
    }
    std::sort(entries.begin(), entries.end());

    for (const auto & sub_dir : entries) {
      std::string device_path = port_root_ + "/" + sub_dir + "/name";
      std::ifstream camera_port_info_file(device_path);
      if (camera_port_info_file.is_open()) {
        std::string content;
        std::getline(camera_port_info_file, content);
        camera_port_info_file.close();

        if (content.find(port_id) != std::string::npos) {
          RCLCPP_INFO(
            logger_, "Found generic camera: %s at /dev/%s", content.c_str(), sub_dir.c_str());
          return "/dev/" + sub_dir;
        }
      }
    }
  } catch (const std::filesystem::filesystem_error & e) {
    RCLCPP_ERROR(logger_, "Filesystem error: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception: %s", e.what());
  }
  RCLCPP_WARN(logger_, "Generic camera not found for port_id: %s", port_id.c_str());
  return "Unknown";
}

void GenericCameraHandler::cleanup()
{
  RCLCPP_INFO(logger_, "Generic camera handler cleanup");
}

}  // namespace vendor
}  // namespace antbot_camera
