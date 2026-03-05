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

RECTRL::RECTRL() {}
RECTRL::~RECTRL() {close();}

void RECTRL::open() {RECTRL_Open();}
void RECTRL::close() {RECTRL_Close();}

int RECTRL::getBridgeboardFirmwareVersion(int & version)
{
  return RECTRL_GetBridgeboardFirmwareVersion(&version);
}

int RECTRL::getCameraFirmwareVersion(RECTRL_CAM_ID camID, int & version)
{
  return RECTRL_GetCameraFirmwareVersion(camID, &version);
}

int RECTRL::getSerialNumber(RECTRL_CAM_ID camID, std::string & serialNumber)
{
  char buffer[256] = {0};
  int result = RECTRL_GetSerialNumber(camID, buffer);
  serialNumber = std::string(buffer);
  return result;
}

int RECTRL::set(RECTRL_CAM_ID camID, RECTRL_PROPERTY property, int value)
{
  return RECTRL_Set(camID, property, value);
}

int RECTRL::setTestPattern(RECTRL_CAM_ID camID, int patternType)
{
  return RECTRL_SetTestPattern(camID, patternType);
}

int RECTRL::getSensorTemperature(RECTRL_CAM_ID camID, double & temperature)
{
  return RECTRL_GetSensorTemperature(camID, &temperature);
}

int RECTRL::getSensorStatus(RECTRL_CAM_ID camID, int & status)
{
  return RECTRL_GetSensorStatus(camID, &status);
}

int RECTRL::sensorRead(RECTRL_CAM_ID camID, uint16_t address, uint16_t & value)
{
  return RECTRL_SensorRead(camID, address, &value);
}

int RECTRL::sensorWrite(RECTRL_CAM_ID camID, uint16_t address, uint16_t value)
{
  return RECTRL_SensorWrite(camID, address, value);
}

int RECTRL::enableMasterSyncMode(bool enable, RECTRL_CAM_ID masterCamID)
{
  return RECTRL_EnableMasterSyncMode(enable, masterCamID);
}

int RECTRL::setSyncDelay(int delay)
{
  return RECTRL_SetSyncDelay(delay);
}

int RECTRL::writeCalibrationData(RECTRL_CAM_ID camID, char * data, int size)
{
  return RECTRL_WriteCalibrationData(camID, data, size);
}

int RECTRL::readCalibrationData(RECTRL_CAM_ID camID, char * data, int size)
{
  return RECTRL_ReadCalibrationData(camID, data, size);
}

int RECTRL::setVideoFormat(RECTRL_VIDEO_FORMAT videoFormat)
{
  return RECTRL_SetVideoFormat(videoFormat);
}

int RECTRL::osdCenter(RECTRL_CAM_ID camID) {return RECTRL_OSDCenter(camID);}
int RECTRL::osdUp(RECTRL_CAM_ID camID) {return RECTRL_OSDUp(camID);}
int RECTRL::osdDown(RECTRL_CAM_ID camID) {return RECTRL_OSDDown(camID);}
int RECTRL::osdLeft(RECTRL_CAM_ID camID) {return RECTRL_OSDLeft(camID);}
int RECTRL::osdRight(RECTRL_CAM_ID camID) {return RECTRL_OSDRight(camID);}
int RECTRL::osdStop(RECTRL_CAM_ID camID) {return RECTRL_OSDStop(camID);}
