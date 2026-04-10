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

#include "antbot_camera/vendor/novitec_camera_handler.hpp"

namespace antbot_camera
{
namespace vendor
{

NovitecCameraHandler::NovitecCameraHandler(const std::string & port_root)
: port_root_(port_root),
  logger_(rclcpp::get_logger("NovitecCameraHandler"))
{
  rectrl_.open();

  int fw_ver = 0;
  if (rectrl_.getBridgeboardFirmwareVersion(fw_ver) != 0) {
    RCLCPP_ERROR(logger_, "Failed to communicate with Novitec SDK after open");
  } else {
    RCLCPP_INFO(logger_, "Successfully initialized Novitec SDK (Bridge FW: %d)", fw_ver);
  }
}

NovitecCameraHandler::~NovitecCameraHandler()
{
  cleanup();
}

void NovitecCameraHandler::cleanup()
{
  rectrl_.close();
}

int NovitecCameraHandler::get_vendor_device_id(const std::string & port_id)
{
  if (port_id.empty()) {
    RCLCPP_ERROR(logger_, "Port ID is empty");
    return -1;
  }
  char last_char = port_id.back();
  if (last_char < 'a' || last_char > 'z') {
    RCLCPP_ERROR(
      logger_, "Invalid port ID format: %s (Last char must be a-z)", port_id.c_str());
    return -1;
  }
  int camera_id = last_char - 'a' + 1;

  RCLCPP_INFO(
    logger_, "Novitec device prepared. Port: %s, Camera ID: %d", port_id.c_str(), camera_id);
  return camera_id;
}

int NovitecCameraHandler::get_bridge_firmware_version(int & version)
{
  return rectrl_.getBridgeboardFirmwareVersion(version);
}

int NovitecCameraHandler::get_camera_firmware_version(RECTRL_CAM_ID cam_id, int & version)
{
  return rectrl_.getCameraFirmwareVersion(cam_id, version);
}

int NovitecCameraHandler::get_serial_number(RECTRL_CAM_ID cam_id, std::string & serial_number)
{
  return rectrl_.getSerialNumber(cam_id, serial_number);
}

int NovitecCameraHandler::set_property(
  RECTRL_CAM_ID cam_id, RECTRL_PROPERTY property, int value)
{
  return rectrl_.set(cam_id, property, value);
}

int NovitecCameraHandler::set_test_pattern(RECTRL_CAM_ID cam_id, int pattern_type)
{
  return rectrl_.setTestPattern(cam_id, pattern_type);
}

int NovitecCameraHandler::get_sensor_temperature(RECTRL_CAM_ID cam_id, double & temperature)
{
  return rectrl_.getSensorTemperature(cam_id, temperature);
}

int NovitecCameraHandler::get_sensor_status(RECTRL_CAM_ID cam_id, int & status)
{
  return rectrl_.getSensorStatus(cam_id, status);
}

int NovitecCameraHandler::enable_master_sync_mode(bool enable, RECTRL_CAM_ID master_cam_id)
{
  return rectrl_.enableMasterSyncMode(enable, master_cam_id);
}

int NovitecCameraHandler::set_sync_delay(int delay)
{
  return rectrl_.setSyncDelay(delay);
}

bool NovitecCameraHandler::read_calibration_data(
  const std::string & port_id,
  int internal_id,
  const std::string & distortion_model,
  const cv::Size & image_size,
  CalibrationData & data)
{
  if (internal_id == -1) {
    RCLCPP_ERROR(
      logger_, "Invalid camera ID. Port: %s, ID: %d", port_id.c_str(), internal_id);
    return false;
  }

  int repeat_num = 2;
  ROM_READ_STATE state = read_parameters_from_rom(internal_id, distortion_model, data, false);

  if (state == ROM_READ_STATE::ROM_INVALID_DATA) {
    RCLCPP_INFO(logger_, "Invalid read data, retry ROM read %d times", repeat_num);
    bool failure = true;
    for (int i = 0; i < repeat_num; i++) {
      RCLCPP_INFO(logger_, "Repeat count: %d", i);
      if (this->read_parameters_from_rom(
          internal_id, distortion_model, data, false) == ROM_READ_STATE::ROM_SUCCESS)
      {
        state = ROM_READ_STATE::ROM_SUCCESS;
        failure = false;
        break;
      }
    }
    if (failure) {
      RCLCPP_ERROR(logger_, "Failed to read ROM data after %d retries", repeat_num);
      print_vector_data("Last parsed intrinsic parameters", data.intrinsic);
      const int read_size = get_rom_read_size(distortion_model);
      validate_calibration_data(data.intrinsic, read_size, distortion_model, true);
      return false;
    }
  }

  if (state == ROM_READ_STATE::ROM_SUCCESS ||
    state == ROM_READ_STATE::ROM_NOT_SUPPORTED_FW_VER ||
    state == ROM_READ_STATE::ROM_INVALID_FW_VER)
  {
    print_vector_data("ROM calibration intrinsic parameters", data.intrinsic);

    // Scale intrinsic parameters if resolution differs from 1920x1080
    if (image_size.width != 1920 || image_size.height != 1080) {
      double scale_x = static_cast<double>(image_size.width) / 1920.0;
      double scale_y = static_cast<double>(image_size.height) / 1080.0;

      data.intrinsic[0] *= scale_x;   // fx
      data.intrinsic[1] *= scale_y;   // fy
      data.intrinsic[2] *= scale_x;   // ppx
      data.intrinsic[3] *= scale_y;   // ppy

      RCLCPP_INFO(
        logger_, "Scaled intrinsics for resolution %dx%d. Scale X: %f, Scale Y: %f",
        image_size.width, image_size.height, scale_x, scale_y);
    }
    return true;
  }

  return false;
}

ROM_READ_STATE NovitecCameraHandler::read_parameters_from_rom(
  const int cam_id,
  const std::string & distortion_model,
  CalibrationData & data,
  bool verbose)
{
  // Read firmware versions
  int cam_fw_ver = 0;
  int bridge_fw_ver = 0;
  if (rectrl_.getCameraFirmwareVersion(static_cast<RECTRL_CAM_ID>(cam_id), cam_fw_ver) != 0 ||
    rectrl_.getBridgeboardFirmwareVersion(bridge_fw_ver) != 0)
  {
    RCLCPP_ERROR(logger_, "Failed to read firmware versions");
    return ROM_READ_STATE::ROM_INVALID_FW_VER;
  }

  if (verbose) {
    RCLCPP_INFO(logger_, "Bridge board firmware version: %d", bridge_fw_ver);
    RCLCPP_INFO(logger_, "Camera board (%d) firmware version: %d", cam_id, cam_fw_ver);
  }

  const int read_size = get_rom_read_size(distortion_model);
  const int calib_data_size = read_size / 4;
  std::vector<float> calib_data(calib_data_size);
  std::vector<double> intrinsic;

  char * rom_calib_data = new char[read_size];
  const int result = rectrl_.readCalibrationData(
    static_cast<RECTRL_CAM_ID>(cam_id), rom_calib_data, read_size);
  if (result != 0) {
    RCLCPP_ERROR(logger_, "ROM read result: %d", result);
  }

  for (int i = 0; i < calib_data_size; i++) {
    memcpy(&calib_data[i], &rom_calib_data[i * 4], sizeof(float));
  }
  delete[] rom_calib_data;

  if (verbose) {
    print_vector_data("Raw ROM calibration data", calib_data);
  }

  parse_calibration_data(calib_data, read_size, intrinsic);

  if (verbose) {
    print_vector_data("Parsed intrinsic parameters", intrinsic);
  }

  data.intrinsic = intrinsic;

  if (!validate_calibration_data(intrinsic, read_size, distortion_model, verbose)) {
    RCLCPP_ERROR(logger_, "Invalid calibration data from ROM");
    return ROM_READ_STATE::ROM_INVALID_DATA;
  }

  data.valid = true;
  return ROM_READ_STATE::ROM_SUCCESS;
}

std::string NovitecCameraHandler::get_device_path(const std::string & port_id)
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

        std::istringstream string_stream(content);
        std::vector<std::string> port_infos;
        std::string port_info;
        while (string_stream >> port_info) {
          port_infos.push_back(port_info);
        }

        if (port_infos.size() >= 2) {
          std::string port_type = port_infos.front();
          std::string target_port = port_infos.back();

          if (port_type.find("vi-output") != std::string::npos && target_port == port_id) {
            return "/dev/" + sub_dir;
          }
        }
      }
    }
  } catch (const std::filesystem::filesystem_error & e) {
    RCLCPP_ERROR(logger_, "Filesystem error: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception: %s", e.what());
  }
  return "Unknown";
}

int NovitecCameraHandler::get_rom_read_size(const std::string & distortion_model) const
{
  // G5 ROM read sizes based on distortion model
  if (distortion_model == "rational_polynomial") {
    return 48;
  } else if (distortion_model == "equidistant") {
    return 32;
  }
  return 40;  // Default fisheye
}

template<typename T>
void NovitecCameraHandler::print_vector_data(
  const std::string & label,
  const std::vector<T> & data)
{
  std::stringstream stream;
  stream << "[";
  for (size_t i = 0; i < data.size(); i++) {
    stream << data[i];
    if (i < data.size() - 1) {
      stream << ", ";
    }
  }
  stream << "]";
  RCLCPP_INFO(logger_, "%s: %s", label.c_str(), stream.str().c_str());
}

void NovitecCameraHandler::parse_calibration_data(
  const std::vector<float> & calib_data,
  int read_size,
  std::vector<double> & intrinsic)
{
  // G5 format: intrinsic only (no extrinsic in ROM for G5)
  intrinsic.resize(12, 0.0);
  intrinsic[0] = calib_data[0];   // fx
  intrinsic[1] = calib_data[1];   // fy
  intrinsic[2] = calib_data[2];   // ppx
  intrinsic[3] = calib_data[3];   // ppy

  if (read_size == 48) {   // Rational polynomial: 8 distortion coefficients
    intrinsic[4] = calib_data[4];    // k1
    intrinsic[5] = calib_data[5];    // k2
    intrinsic[6] = calib_data[6];    // p1
    intrinsic[7] = calib_data[7];    // p2
    intrinsic[8] = calib_data[8];    // k3
    intrinsic[9] = calib_data[9];    // k4
    intrinsic[10] = calib_data[10];  // k5
    intrinsic[11] = calib_data[11];  // k6
  } else {   // Fisheye or Equidistant: 4 distortion coefficients
    intrinsic[4] = calib_data[4];   // k1
    intrinsic[5] = calib_data[5];   // k2
    intrinsic[6] = calib_data[6];   // k3
    intrinsic[7] = calib_data[7];   // k4
  }
}

bool NovitecCameraHandler::validate_calibration_data(
  const std::vector<double> & intrinsic,
  int read_size,
  const std::string & distortion_model,
  bool verbose)
{
  (void) distortion_model;

  std::vector<std::pair<double, double>> intrinsics_range;
  // Common intrinsic parameter ranges (fx, fy, ppx, ppy)
  intrinsics_range.emplace_back(700.0, 1100.0);   // fx
  intrinsics_range.emplace_back(700.0, 1100.0);   // fy
  intrinsics_range.emplace_back(800.0, 1100.0);   // ppx
  intrinsics_range.emplace_back(400.0, 800.0);    // ppy

  if (read_size == 48) {
    for (int i = 0; i < 8; i++) {
      intrinsics_range.emplace_back(-1.5, 1.5);
    }
  } else if (read_size == 32) {
    for (int i = 0; i < 4; i++) {
      intrinsics_range.emplace_back(-1.5, 1.5);
    }
  } else {
    for (int i = 0; i < 4; i++) {
      intrinsics_range.emplace_back(-1.5, 1.5);
    }
  }

  size_t num_intrinsic_to_validate = (read_size == 48) ? 12 : 8;
  std::vector<double> intrinsic_to_validate(
    intrinsic.begin(),
    intrinsic.begin() + static_cast<int64_t>(
      std::min(num_intrinsic_to_validate, intrinsic.size())));

  bool is_valid = true;
  for (size_t i = 0; i < intrinsic_to_validate.size(); ++i) {
    if (i >= intrinsics_range.size()) {
      break;
    }
    if (intrinsic_to_validate[i] < intrinsics_range[i].first ||
      intrinsic_to_validate[i] > intrinsics_range[i].second)
    {
      if (verbose) {
        RCLCPP_ERROR(
          logger_, "Intrinsic parameter[%zu] out of range: val=%f, range=[%f, %f]",
          i, intrinsic_to_validate[i], intrinsics_range[i].first, intrinsics_range[i].second);
      }
      is_valid = false;
    }
  }

  return is_valid;
}

}  // namespace vendor
}  // namespace antbot_camera
