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

#include <vanjee_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <vanjee_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <vanjee_driver/msg/point_cloud_msg.hpp>
#endif

using namespace vanjee::lidar;

typedef PointXYZIRT PointT;
typedef PointCloudT<PointT> PointCloudMsg;

class PointCloudClient {
 public:
  uint8_t lidar_id_;
  PointCloudMsg point_cloud_msg_;
};

class ImuPacketClient : public vanjee::lidar::ImuPacket {
 public:
  uint8_t lidar_id_;
};

class ScanDataClient : public vanjee::lidar::ScanData {
 public:
  uint8_t lidar_id_;
};

class DeviceCtrlClient : public vanjee::lidar::DeviceCtrl {
 public:
  uint8_t lidar_id_;
};

SyncQueue<std::shared_ptr<DeviceCtrlClient>> device_ctrl_param_;

// @brief this function which would return the point cloud to your own
// application from sdk
void pointCloudHandler(std::shared_ptr<PointCloudClient> msg) {
#if false
  WJ_MSG << "lidar[" << (uint16_t)msg->lidar_id_ << "] msg: " << msg->point_cloud_msg_.seq << ", timestamp: " 
          << std::to_string(msg->point_cloud_msg_.timestamp) << ", point cloud size: " << msg->point_cloud_msg_.points.size() << WJ_REND;
  // for (auto it = msg->point_cloud_msg_.points.begin(); it != msg->point_cloud_msg_.points.end(); it++)
  // {
  //   std::cout << std::fixed << std::setprecision(3)
  //             << "(" << it->x << ", " << it->y << ", " << it->z << ", " 
  //             << (int)it->intensity << ", " << (int)it->ring<< ", " << (int)it->timestamp << ")"
  //             << std::endl;
  // }
#endif
}

// @brief this function which would return the imu to your own application from
// sdk
void imuPacketHandler(std::shared_ptr<ImuPacketClient> msg) {
#if false
  WJ_MSG << "lidar[" << (uint16_t)msg->lidar_id_ << "] seq: " << msg->seq << ", timestamp: " << std::to_string(msg->timestamp) << WJ_REND;
  WJ_MSG << "x_acc: " << msg->linear_acce[0] << ", y_acc: " << msg->linear_acce[1]<< ", z_acc: " << msg->linear_acce[2] << WJ_REND;
  WJ_MSG << "angular_voc_x: " << msg->angular_voc[0] << ", angular_voc_y: " << msg->angular_voc[1]<< ", angular_voc_z: " << msg->angular_voc[2] <<WJ_REND;
#endif
}

// @brief this function which would return the laser scan to your own
// application from sdk
void laserScanHandler(std::shared_ptr<ScanDataClient> msg) {
#if false
    WJ_MSG << "lidar[" << (uint16_t)msg->lidar_id_ << "] seq: " << msg->seq << ", timestamp: " << std::to_string(msg->timestamp) 
            << ", data_size: " << msg->ranges.size() << WJ_REND;
#endif
}

// @brief this function which would return the device control state to your own
// application from sdk
void deviceCtrlStateHandler(std::shared_ptr<DeviceCtrlClient> msg) {
#if false
  WJ_MSG << "lidar[" << (uint16_t)msg->lidar_id_ << "] seq: " << msg->seq << ", timestamp: " << std::to_string(msg->timestamp) 
          << ", cmd_id: " << msg->cmd_id << ", cmd_param: " << msg->cmd_param << ", cmd_state: " << (uint16_t)msg->cmd_state << WJ_REND;
#endif
}

class DriverClient {
 public:
  uint8_t lidar_id_;

  DriverClient(const std::string name) : name_(name) {
  }

  bool init(const WJDriverParam& param) {
    WJ_INFO << "------------------------------------------------------" << WJ_REND;
    WJ_INFO << "                      " << name_ << WJ_REND;
    WJ_INFO << "------------------------------------------------------" << WJ_REND;
    param.print();
    // @brief these two callback,allocatePointCloudMemoryCallback and
    // pointCloudCallback, have to be implemented,and as the parameter of
    // regPointCloudCallback function.
    driver_.regPointCloudCallback(std::bind(&DriverClient::allocatePointCloudMemoryCallback, this),
                                  std::bind(&DriverClient::pointCloudCallback, this, std::placeholders::_1));
    // @brief exceptionCallback have to be implemented,and as the parameter of
    // regExceptionCallback function.
    driver_.regExceptionCallback(std::bind(&DriverClient::exceptionCallback, this, std::placeholders::_1));
    // @brief these two callback,allocateImuPacketMemoryCallback and
    // imuPacketCallback, have to be implemented,and as the parameter of
    // regImuPacketCallback function.s
    driver_.regImuPacketCallback(std::bind(&DriverClient::allocateImuPacketMemoryCallback, this),
                                 std::bind(&DriverClient::imuPacketCallback, this, std::placeholders::_1));
    // @brief these two callback,allocateLaserScanMemoryCallback and
    // laserScanCallback, have to be implemented,and as the parameter of
    // regScanDataCallback function.
    driver_.regScanDataCallback(std::bind(&DriverClient::allocateLaserScanMemoryCallback, this),
                                std::bind(&DriverClient::laserScanCallback, this, std::placeholders::_1));
    // @brief these two callback,allocateDeviceCtrlMemoryCallback and
    // deviceCtrlCallback, have to be implemented,and as the parameter of
    // regDeviceCtrlCallback function.
    driver_.regDeviceCtrlCallback(std::bind(&DriverClient::allocateDeviceCtrlMemoryCallback, this),
                                  std::bind(&DriverClient::deviceCtrlCallback, this, std::placeholders::_1));

    if (!driver_.init(param)) {
      WJ_ERROR << name_ << ": Failed to initialize driver." << WJ_REND;
      return false;
    }
    return true;
  }

  bool start() {
    to_exit_process_ = false;
    point_cloud_thread_ = std::thread(&DriverClient::processPointCloud, this);
    imu_thread_ = std::thread(&DriverClient::processImu, this);
    laser_scan_thread_ = std::thread(&DriverClient::processLaserScan, this);
    device_ctrl_thread_ = std::thread(&DriverClient::processDeviceCtrlState, this);
    device_ctrl_cmd_thread_ = std::thread(&DriverClient::processDeviceCtrlCmd, this);
    driver_.start();
    WJ_DEBUG << name_ << ": Started driver." << WJ_REND;

    return true;
  }

  void stop() {
    driver_.stop();
    to_exit_process_ = true;
    point_cloud_thread_.join();
    imu_thread_.join();
    laser_scan_thread_.join();
    device_ctrl_thread_.join();
    device_ctrl_cmd_thread_.join();
  }

 protected:
  // @brief allocate memory for point cloud，sdk would call this callback to get
  // the memory for point cloud storage.
  std::shared_ptr<PointCloudMsg> allocatePointCloudMemoryCallback(void) {
    std::shared_ptr<PointCloudClient> msg = free_cloud_queue_.pop();
    if (msg.get() != NULL) {
      return std::make_shared<PointCloudMsg>(msg->point_cloud_msg_);
    }

    return std::make_shared<PointCloudMsg>();
  }

  // @brief this callback which would return the point cloud to queue
  void pointCloudCallback(std::shared_ptr<PointCloudMsg> msg) {
    std::shared_ptr<PointCloudClient> msg_client = std::make_shared<PointCloudClient>();
    msg_client->lidar_id_ = lidar_id_;
    msg_client->point_cloud_msg_ = *msg;
    stuffed_cloud_queue_.push(msg_client);
  }

  // @brief allocate memory for imu，sdk would call this callback to get the
  // memory for imu storage.
  std::shared_ptr<ImuPacket> allocateImuPacketMemoryCallback() {
    std::shared_ptr<ImuPacketClient> msg = free_imu_queue_.pop();
    if (msg.get() != NULL) {
      std::shared_ptr<ImuPacket> msg_ret = std::make_shared<ImuPacket>();
      msg_ret->timestamp = msg->timestamp;
      msg_ret->seq = msg->seq;
      msg_ret->orientation = msg->orientation;
      msg_ret->orientation_covariance = msg->orientation_covariance;
      msg_ret->angular_voc = msg->angular_voc;
      msg_ret->angular_voc_covariance = msg->angular_voc_covariance;
      msg_ret->linear_acce = msg->linear_acce;
      msg_ret->linear_acce_covariance = msg->linear_acce_covariance;
      return msg_ret;
    }

    return std::make_shared<ImuPacket>();
  }

  // @brief this callback which would return the imu to queue
  void imuPacketCallback(std::shared_ptr<ImuPacket> msg) {
    std::shared_ptr<ImuPacketClient> msg_client = std::make_shared<ImuPacketClient>();
    msg_client->timestamp = msg->timestamp;
    msg_client->seq = msg->seq;
    msg_client->orientation = msg->orientation;
    msg_client->orientation_covariance = msg->orientation_covariance;
    msg_client->angular_voc = msg->angular_voc;
    msg_client->angular_voc_covariance = msg->angular_voc_covariance;
    msg_client->linear_acce = msg->linear_acce;
    msg_client->linear_acce_covariance = msg->linear_acce_covariance;

    msg_client->lidar_id_ = lidar_id_;
    stuffed_imu_queue_.push(msg_client);
  }

  // @brief allocate memory for laserScan，sdk would call this callback to get
  // the memory for laserScan storage.
  std::shared_ptr<ScanData> allocateLaserScanMemoryCallback() {
    std::shared_ptr<ScanDataClient> msg = free_scan_data_queue_.pop();
    if (msg.get() != NULL) {
      std::shared_ptr<ScanData> msg_ret = std::make_shared<ScanData>();
      msg_ret->seq = msg->seq;
      msg_ret->timestamp = msg->timestamp;
      msg_ret->angle_min = msg->angle_min;
      msg_ret->angle_max = msg->angle_max;
      msg_ret->angle_increment = msg->angle_increment;
      msg_ret->time_increment = msg->time_increment;
      msg_ret->scan_time = msg->scan_time;
      msg_ret->range_min = msg->range_min;
      msg_ret->range_max = msg->range_max;
      msg_ret->ranges = msg->ranges;
      msg_ret->intensities = msg->intensities;
      return msg_ret;
    }

    return std::make_shared<ScanData>();
  }

  // @brief this callback which would return the laserScan to queue
  void laserScanCallback(std::shared_ptr<ScanData> msg) {
    std::shared_ptr<ScanDataClient> msg_client = std::make_shared<ScanDataClient>();
    msg_client->seq = msg->seq;
    msg_client->timestamp = msg->timestamp;
    msg_client->angle_min = msg->angle_min;
    msg_client->angle_max = msg->angle_max;
    msg_client->angle_increment = msg->angle_increment;
    msg_client->time_increment = msg->time_increment;
    msg_client->scan_time = msg->scan_time;
    msg_client->range_min = msg->range_min;
    msg_client->range_max = msg->range_max;
    msg_client->ranges = msg->ranges;
    msg_client->intensities = msg->intensities;

    msg_client->lidar_id_ = lidar_id_;
    stuffed_scan_data_queue_.push(msg_client);
  }

  // @brief allocate memory for device control state，sdk would call this callback to get the
  // memory for device contrl state storage.
  std::shared_ptr<DeviceCtrl> allocateDeviceCtrlMemoryCallback() {
    std::shared_ptr<DeviceCtrlClient> msg = free_device_ctrl_queue_.pop();
    if (msg.get() != NULL) {
      std::shared_ptr<DeviceCtrl> msg_ret = std::make_shared<DeviceCtrl>();
      msg_ret->seq = msg->seq;
      msg_ret->timestamp = msg->timestamp;
      msg_ret->cmd_id = msg->cmd_id;
      msg_ret->cmd_param = msg->cmd_param;
      msg_ret->cmd_state = msg->cmd_state;
      return msg_ret;
    }

    return std::make_shared<DeviceCtrl>();
  }

  // @brief this callback which would return the device control to queue
  void deviceCtrlCallback(std::shared_ptr<DeviceCtrl> msg) {
    std::shared_ptr<DeviceCtrlClient> msg_client = std::make_shared<DeviceCtrlClient>();
    msg_client->seq = msg->seq;
    msg_client->timestamp = msg->timestamp;
    msg_client->cmd_id = msg->cmd_id;
    msg_client->cmd_param = msg->cmd_param;
    msg_client->cmd_state = msg->cmd_state;

    msg_client->lidar_id_ = lidar_id_;
    stuffed_device_ctrl_queue_.push(msg_client);
  }

  // @brief this callback which would return the exception prompt information to
  // terminal
  void exceptionCallback(const Error& code) {
    WJ_WARNING << code.toString() << WJ_REND;
  }

  void processPointCloud() {
    while (!to_exit_process_) {
      std::shared_ptr<PointCloudClient> msg = stuffed_cloud_queue_.popWait();
      if (msg.get() == NULL) {
        continue;
      }
      pointCloudHandler(msg);
      free_cloud_queue_.push(msg);
    }
  }

  void processImu() {
    while (!to_exit_process_) {
      std::shared_ptr<ImuPacketClient> msg = stuffed_imu_queue_.popWait();
      if (msg.get() == NULL) {
        continue;
      }
      imuPacketHandler(msg);
      free_imu_queue_.push(msg);
    }
  }

  void processLaserScan() {
    while (!to_exit_process_) {
      std::shared_ptr<ScanDataClient> msg = stuffed_scan_data_queue_.popWait();
      if (msg.get() == NULL) {
        continue;
      }
      laserScanHandler(msg);
      free_scan_data_queue_.push(msg);
    }
  }

  void processDeviceCtrlState(void) {
    while (!to_exit_process_) {
      std::shared_ptr<DeviceCtrlClient> msg = stuffed_device_ctrl_queue_.popWait();
      if (msg.get() == NULL) {
        continue;
      }
      deviceCtrlStateHandler(msg);
      free_device_ctrl_queue_.push(msg);
    }
  }

  void processDeviceCtrlCmd() {
    while (!to_exit_process_) {
      std::shared_ptr<DeviceCtrlClient> msg = device_ctrl_param_.popWait();
      if (msg.get() == NULL) {
        continue;
      }
      if (msg->lidar_id_ == lidar_id_) {
        std::shared_ptr<DeviceCtrl> msg_ret = std::make_shared<DeviceCtrl>();
        msg_ret->seq = msg->seq;
        msg_ret->timestamp = msg->timestamp;
        msg_ret->cmd_id = msg->cmd_id;
        msg_ret->cmd_param = msg->cmd_param;
        msg_ret->cmd_state = msg->cmd_state;
        driver_.deviceCtrlApi(*msg_ret);
        // std::cout << "lidar [" << (uint16_t)msg->lidar_id_ << "] seq: " << msg->seq << ", cmd_id: " << msg->cmd_id << ", cmd_param: " <<
        // msg->cmd_param
        //           << ", cmd_state: " << (uint16_t)msg->cmd_state << std::endl;
      } else {
        device_ctrl_param_.push(msg);
      }
    }
  }

 protected:
  std::string name_;
  LidarDriver<PointCloudMsg> driver_;

  SyncQueue<std::shared_ptr<PointCloudClient>> free_cloud_queue_;
  SyncQueue<std::shared_ptr<PointCloudClient>> stuffed_cloud_queue_;
  SyncQueue<std::shared_ptr<ImuPacketClient>> free_imu_queue_;
  SyncQueue<std::shared_ptr<ImuPacketClient>> stuffed_imu_queue_;
  SyncQueue<std::shared_ptr<ScanDataClient>> free_scan_data_queue_;
  SyncQueue<std::shared_ptr<ScanDataClient>> stuffed_scan_data_queue_;
  SyncQueue<std::shared_ptr<DeviceCtrlClient>> free_device_ctrl_queue_;
  SyncQueue<std::shared_ptr<DeviceCtrlClient>> stuffed_device_ctrl_queue_;

  bool to_exit_process_;
  std::thread point_cloud_thread_;
  std::thread imu_thread_;
  std::thread laser_scan_thread_;
  std::thread device_ctrl_thread_;
  std::thread device_ctrl_cmd_thread_;
};

bool to_exit_keyboard_detect_process_ = false;

void getKeyboard(void) {
  std::string input;
  while (!to_exit_keyboard_detect_process_) {
    std::getline(std::cin, input);
    if (!input.empty()) {
      WJ_INFO << "detect keyboard input: " << input << WJ_REND;
      to_exit_keyboard_detect_process_ = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void deviceCtrlCmd(void) {
  // 设备控制参数，详情见 /src/vanjee_lidar_sdk/doc/intro/03_device_ctrl_intro_CN.md
  uint8_t lidar_id = 2;    // 雷达id 起始id为1
  uint32_t seq = 0;        // 序列号
  uint16_t cmd_id = 0;     // 设备操作码
  uint16_t cmd_param = 0;  // 设置操作参数
  uint8_t cmd_state = 0;   // 设备操作状态
  while (!to_exit_keyboard_detect_process_) {
    std::shared_ptr<DeviceCtrlClient> msg_client = std::make_shared<DeviceCtrlClient>();
    msg_client->lidar_id_ = lidar_id;
    msg_client->seq = seq++;
    msg_client->cmd_id = cmd_id;
    msg_client->cmd_param = cmd_param;
    msg_client->cmd_state = cmd_state;
    device_ctrl_param_.push(msg_client);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

int main(int argc, char* argv[]) {
  WJDriverParam param_left;
  param_left.lidar_type = LidarType::vanjee_720_16;
  param_left.input_type = InputType::ONLINE_LIDAR;
  param_left.input_param.host_msop_port = 3001;
  param_left.input_param.lidar_msop_port = 3333;
  param_left.input_param.host_address = "192.168.2.88";
  param_left.input_param.lidar_address = "192.168.2.86";
  param_left.input_param.group_address = "0.0.0.0";
  param_left.decoder_param.config_from_file = false;
  param_left.decoder_param.angle_path_ver = ".../src/vanjee_lidar_sdk/param/Vanjee_720_16.csv";
  param_left.decoder_param.imu_param_path = ".../src/vanjee_lidar_sdk/param/vanjee_720_imu_param.csv";
  param_left.decoder_param.wait_for_difop = true;
  param_left.decoder_param.point_cloud_enable = true;  // true: enable PointCloud2 msg; false: disable PointCloud2 msg
  param_left.decoder_param.imu_enable = 1;             // -1: disable IMU ; 0: enable IMU but calibrate IMU using default
                                                       // parameters ; 1: enable IMU;
  param_left.decoder_param.hide_points_range = "";     // For details about the configuration format, refer to the following
                                                       // documents: "./doc/intro/02_parameter_intro_CN.md"

  DriverClient client_left("lidar_1 ");
  client_left.lidar_id_ = 1;
  if (!client_left.init(param_left)) {
    return -1;
  }

  WJDriverParam param_right;
  param_right.lidar_type = LidarType::vanjee_719;
  param_right.input_type = InputType::ONLINE_LIDAR;
  param_right.input_param.host_msop_port = 3002;
  param_right.input_param.lidar_msop_port = 6050;
  param_right.input_param.host_address = "192.168.2.88";
  param_right.input_param.lidar_address = "192.168.2.85";
  param_right.decoder_param.config_from_file = false;
  param_right.decoder_param.wait_for_difop = false;

  param_right.decoder_param.point_cloud_enable = true;         // true: enable PointCloud2 msg; false: disable PointCloud2 msg
  param_right.decoder_param.laser_scan_enable = false;         // true: enable LaserScan msg; false: disable LaserScan msg
  param_right.decoder_param.device_ctrl_state_enable = false;  // true: enable Device Control State msg; false: disable Device Control State msg

  DriverClient client_right("lidar_2");
  client_right.lidar_id_ = 2;
  if (!client_right.init(param_right)) {
    return -1;
  }

  client_left.start();
  client_right.start();

#ifdef ORDERLY_EXIT
  std::this_thread::sleep_for(std::chrono::seconds(10));

  client_left.stop();
  client_right.stop();
#else
  // std::thread device_ctrl_handle_thread = std::thread(deviceCtrlCmd);
  std::thread detect_handle_thread = std::thread(getKeyboard);
  WJ_MSG << "Enter any character and press enter to exit! " << WJ_REND;
  while (true) {
    if (to_exit_keyboard_detect_process_) {
      client_left.stop();
      client_right.stop();
      // device_ctrl_handle_thread.join();
      detect_handle_thread.join();
      break;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#endif

  return 0;
}
