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

#include <functional>
#include <iostream>
#include <thread>

#include "vanjee_lidar_driver_lib.hpp"

// @brief this callback which would return the point cloud to your own
// application from sdk
void pointCloudCallback(std::shared_ptr<PointCloudClient> msg) {
#if false
  std::cout << "lidar[" << (uint16_t)msg->lidar_id_ << "] msg: " << msg->point_cloud_msg_.seq << " , timestamp: " << std::to_string(msg->point_cloud_msg_.timestamp)
             << " , point cloud size: " << msg->point_cloud_msg_.points.size() << std::endl;
  // for (auto it = msg->point_cloud_msg_.points.begin(); it != msg->point_cloud_msg_.points.end(); it++)
  // {
  //   std::cout << std::fixed << std::setprecision(3) 
  //             << "(" << it->x << ", " << it->y << ", " << it->z << ", " << (int)it->intensity << ")" 
  //             << std::endl;
  // }
#endif
}

// @brief this callback which would return the imu to your own application from
// sdk
void imuPacketCallback(std::shared_ptr<ImuPacketClient> msg) {
#if false
  std::cout << "lidar[" << (uint16_t)msg->lidar_id_ << "] timestamp: " << std::to_string(msg->timestamp) << ", seq: " << msg->seq << std::endl;
  std::cout << "x_acc: " << msg->linear_acce[0] << ", y_acc: " << msg->linear_acce[1]<< ", z_acc: " << msg->linear_acce[2] << std::endl;
  std::cout << "angular_voc_x: " << msg->angular_voc[0] << ", angular_voc_y: " << msg->angular_voc[1]<< ", angular_voc_z: " << msg->angular_voc[2] <<std::endl;
#endif
}

// @brief this callback which would return the laserScan to your own application
// from sdk
void laserScanCallback(std::shared_ptr<ScanDataClient> msg) {
#if false
    std::cout << "lidar[" << (uint16_t)msg->lidar_id_ << "] timestamp: " << std::to_string(msg->timestamp) 
              << ", seq: " << msg->seq << ", data_size: " << msg->ranges.size() << std::endl;
#endif
}

// @brief this callback which would return the device ctrl state to your own application
// from sdk
void deviceCtrlStateCallback(std::shared_ptr<DeviceCtrlClient> msg) {
#if false
  std::cout << "lidar[" << (uint16_t)msg->lidar_id_ << "] seq: " << msg->seq << ", timestamp: " << std::to_string(msg->timestamp) 
            << ", cmd_id: " << msg->cmd_id << ", cmd_param: " << msg->cmd_param << ", cmd_state: " << (uint16_t)msg->cmd_state << std::endl;
#endif
}

bool to_exit_keyboard_detect_process = false;
bool to_exit_process = false;

void getKeyboard(void) {
  std::string input;
  while (!to_exit_keyboard_detect_process) {
    std::getline(std::cin, input);
    if (!input.empty()) {
      std::cout << "detect keyboard input: " << input << std::endl;
      to_exit_keyboard_detect_process = true;
      to_exit_process = true;

      if (vanjeeLidarDriverLibStop()) {
        std::cout << "vanjee driver exit!" << std::endl;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void deviceCtrlCmd(void) {
  // 设备控制参数，详情见 /src/vanjee_lidar_sdk/doc/intro/03_device_ctrl_intro_CN.md
  uint8_t lidar_id = 1;    // 雷达id 起始id为1
  uint32_t seq = 0;        // 序列号
  uint16_t cmd_id = 0;     // 设备操作码
  uint16_t cmd_param = 0;  // 设置操作参数
  uint8_t cmd_state = 0;   // 设备操作状态
  while (!to_exit_process) {
    DeviceCtrlClient device_ctrl;
    device_ctrl.lidar_id_ = lidar_id;
    device_ctrl.seq = seq++;
    device_ctrl.cmd_id = cmd_id;
    device_ctrl.cmd_param = cmd_param;
    device_ctrl.cmd_state = cmd_state;
    vanjeeLidarDriverDeviceCtrlApi(device_ctrl);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

int main() {
  std::thread detect_handle_thread = std::thread(getKeyboard);
  // std::thread device_ctrl_handle_thread = std::thread(deviceCtrlCmd);
  std::string config_path =
      "/home/vanjee/ROS/01vanjee_sdk_dev/00temp_workspace/src/vanjee_lidar_sdk/src/vanjee_driver/lib/demo/"
      "config/config.yaml";  // absolute path of config file
  vanjeeLidarDriverLibStart(config_path, pointCloudCallback, imuPacketCallback, laserScanCallback, deviceCtrlStateCallback);
  // device_ctrl_handle_thread.join();
  detect_handle_thread.join();
  return 0;
}
