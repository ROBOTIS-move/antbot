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

#ifndef ANTBOT_CAMERA__VENDOR__NOVITEC_CAMERA_HANDLER_HPP_
#define ANTBOT_CAMERA__VENDOR__NOVITEC_CAMERA_HANDLER_HPP_

#include <dlfcn.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "opencv2/core/types.hpp"
#include "rclcpp/rclcpp.hpp"

#include "antbot_camera/vendor/camera_vendor_handler.hpp"

// RECTRL C API declarations (loaded dynamically via dlopen)
extern "C" {
#define RECTRL_SUCCESS                     0
#define RECTRL_ERROR_FAIL                  -1
#define RECTRL_ERROR_NOT_OPENED            -2
#define RECTRL_ERROR_INVALID_PROPERTY      -100
#define RECTRL_ERROR_INVALID_VALUE         -101

typedef enum
{
  CAM0 = 1, CAM1, CAM2, CAM3, CAM4, CAM5,
  CAM_ALL = 0
} RECTRL_CAM_ID;

typedef enum
{
  NO_PATTERN = 0,
  SOLID_COLOR,
  VERTIAL_COLOR_BARS,
  FADE_TO_GRAY_COLOR_BARS,
} RECTRL_TEST_PATTERN_TYPE;

typedef enum
{
  BRIGHTNESS = 1,
  SHUTTER,
  AGC,
  AWB_MODE,
  AWB_MANUAL_CTEMP,
  AWB_MANUAL_RGAIN,
  AWB_MANUAL_BGAIN,
  COLOR_GAIN,
  COLOR_TONE,
  DNR,
  SHARPNESS,
  MIRROR,
  FLIP,
  SHADING_MODE,
  SHADING_WEIGHT,
  SHADING_DET,
} RECTRL_PROPERTY;

typedef enum
{
  MODE_720_30P = 0,
  MODE_1080_30P,
} RECTRL_VIDEO_FORMAT;

int RECTRL_Open();
int RECTRL_Close();

int RECTRL_GetBridgeboardFirmwareVersion(int * version);
int RECTRL_GetCameraFirmwareVersion(RECTRL_CAM_ID camID, int * version);
int RECTRL_GetSerialNumber(RECTRL_CAM_ID camID, char * serialNumber);

int RECTRL_Set(RECTRL_CAM_ID camID, RECTRL_PROPERTY property, int value);
int RECTRL_SetTestPattern(RECTRL_CAM_ID camID, int patternType);
int RECTRL_GetSensorTemperature(RECTRL_CAM_ID camID, double * temperature);
int RECTRL_GetSensorStatus(RECTRL_CAM_ID camID, int * status);

int RECTRL_SensorRead(RECTRL_CAM_ID camID, uint16_t address, uint16_t * value);
int RECTRL_SensorWrite(RECTRL_CAM_ID camID, uint16_t address, uint16_t value);

int RECTRL_EnableMasterSyncMode(bool enable, RECTRL_CAM_ID masterCamID);
int RECTRL_SetSyncDelay(int delay);

int RECTRL_WriteCalibrationData(RECTRL_CAM_ID camID, char * data, int size);
int RECTRL_ReadCalibrationData(RECTRL_CAM_ID camID, char * data, int size);

int RECTRL_SetVideoFormat(RECTRL_VIDEO_FORMAT videoFormat);

int RECTRL_OSDCenter(RECTRL_CAM_ID camID);
int RECTRL_OSDUp(RECTRL_CAM_ID camID);
int RECTRL_OSDDown(RECTRL_CAM_ID camID);
int RECTRL_OSDLeft(RECTRL_CAM_ID camID);
int RECTRL_OSDRight(RECTRL_CAM_ID camID);
int RECTRL_OSDStop(RECTRL_CAM_ID camID);
}

class RECTRL
{
public:
  RECTRL();
  ~RECTRL();

  void open();
  void close();

  int getBridgeboardFirmwareVersion(int & version);
  int getCameraFirmwareVersion(RECTRL_CAM_ID camID, int & version);
  int getSerialNumber(RECTRL_CAM_ID camID, std::string & serialNumber);

  int set(RECTRL_CAM_ID camID, RECTRL_PROPERTY property, int value);
  int setTestPattern(RECTRL_CAM_ID camID, int patternType);
  int getSensorTemperature(RECTRL_CAM_ID camID, double & temperature);
  int getSensorStatus(RECTRL_CAM_ID camID, int & status);

  int sensorRead(RECTRL_CAM_ID camID, uint16_t address, uint16_t & value);
  int sensorWrite(RECTRL_CAM_ID camID, uint16_t address, uint16_t value);

  int enableMasterSyncMode(bool enable, RECTRL_CAM_ID masterCamID);
  int setSyncDelay(int delay);

  int writeCalibrationData(RECTRL_CAM_ID camID, char * data, int size);
  int readCalibrationData(RECTRL_CAM_ID camID, char * data, int size);

  int setVideoFormat(RECTRL_VIDEO_FORMAT videoFormat);

  int osdCenter(RECTRL_CAM_ID camID);
  int osdUp(RECTRL_CAM_ID camID);
  int osdDown(RECTRL_CAM_ID camID);
  int osdLeft(RECTRL_CAM_ID camID);
  int osdRight(RECTRL_CAM_ID camID);
  int osdStop(RECTRL_CAM_ID camID);
};

namespace antbot_camera
{
namespace vendor
{

typedef enum
{
  ROM_INVALID_DATA = -3,
  ROM_INVALID_FW_VER,
  ROM_NOT_SUPPORTED_FW_VER,
  ROM_SUCCESS
} ROM_READ_STATE;

class NovitecCameraHandler : public CameraVendorHandler
{
public:
  explicit NovitecCameraHandler(const std::string & port_root);
  virtual ~NovitecCameraHandler();

  int get_vendor_device_id(const std::string & port_id) override;
  bool supports_rom_mode() const override {return true;}
  bool supports_framerate_setting() const override {return false;}
  bool read_calibration_data(
    const std::string & port_id,
    int internal_id,
    const std::string & distortion_model,
    const cv::Size & image_size,
    CalibrationData & data) override;
  std::string get_device_path(const std::string & port_id) override;
  void cleanup() override;

  int get_bridge_firmware_version(int & version);
  int get_camera_firmware_version(RECTRL_CAM_ID cam_id, int & version);
  int get_serial_number(RECTRL_CAM_ID cam_id, std::string & serial_number);
  int set_property(RECTRL_CAM_ID cam_id, RECTRL_PROPERTY property, int value);
  int set_test_pattern(RECTRL_CAM_ID cam_id, int pattern_type);
  int get_sensor_temperature(RECTRL_CAM_ID cam_id, double & temperature);
  int get_sensor_status(RECTRL_CAM_ID cam_id, int & status);
  int enable_master_sync_mode(bool enable, RECTRL_CAM_ID master_cam_id);
  int set_sync_delay(int delay);

private:
  ROM_READ_STATE read_parameters_from_rom(
    const int cam_id,
    const std::string & distortion_model,
    CalibrationData & data,
    bool verbose);

  int get_rom_read_size(const std::string & distortion_model) const;

  template<typename T>
  void print_vector_data(const std::string & label, const std::vector<T> & data);

  void parse_calibration_data(
    const std::vector<float> & calib_data,
    int read_size,
    std::vector<double> & intrinsic);

  bool validate_calibration_data(
    const std::vector<double> & intrinsic,
    int read_size,
    const std::string & distortion_model,
    bool verbose);

  RECTRL rectrl_;
  std::string port_root_;
  rclcpp::Logger logger_;
};

}  // namespace vendor
}  // namespace antbot_camera
#endif  // ANTBOT_CAMERA__VENDOR__NOVITEC_CAMERA_HANDLER_HPP_
