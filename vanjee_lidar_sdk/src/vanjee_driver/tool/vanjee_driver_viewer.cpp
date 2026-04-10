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

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vanjee_driver/api/lidar_driver.hpp>
#include <vanjee_driver/msg/pcl_point_cloud_msg.hpp>

using namespace vanjee::lidar;
using namespace pcl::visualization;

typedef PointCloudT<PointXYZI> PointCloudMsg;

std::shared_ptr<PCLVisualizer> pcl_viewer;
std::mutex mtx_viewer;

bool checkKeywordExist(int argc, const char* const* argv, const char* str) {
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], str) == 0) {
      return true;
    }
  }
  return false;
}

bool parseArgument(int argc, const char* const* argv, const char* str, std::string& val) {
  int index = -1;

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], str) == 0) {
      index = i + 1;
    }
  }

  if (index > 0 && index < argc) {
    val = argv[index];
    return true;
  }

  return false;
}

void parseParam(int argc, char* argv[], WJDriverParam& param) {
  std::string result_str;

  if (parseArgument(argc, argv, "-x", result_str)) {
    param.decoder_param.transform_param.x = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-y", result_str)) {
    param.decoder_param.transform_param.y = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-z", result_str)) {
    param.decoder_param.transform_param.z = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-roll", result_str)) {
    param.decoder_param.transform_param.roll = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-pitch", result_str)) {
    param.decoder_param.transform_param.pitch = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-yaw", result_str)) {
    param.decoder_param.transform_param.yaw = std::stof(result_str);
  }
}

void printHelpMenu() {
  WJ_MSG << "Arguments: " << WJ_REND;
  WJ_MSG << "  -x      = Transformation parameter, unit: m " << WJ_REND;
  WJ_MSG << "  -y      = Transformation parameter, unit: m " << WJ_REND;
  WJ_MSG << "  -z      = Transformation parameter, unit: m " << WJ_REND;
  WJ_MSG << "  -roll   = Transformation parameter, unit: degree " << WJ_REND;
  WJ_MSG << "  -pitch  = Transformation parameter, unit: degree " << WJ_REND;
  WJ_MSG << "  -yaw    = Transformation parameter, unit: degree " << WJ_REND;
}

// @brief this callback which would return the exception prompt information to
// your own application from sdk
void exceptionCallback(const Error& code) {
  WJ_WARNING << code.toString() << WJ_REND;
}

// @brief allocate memory for point cloud，sdk would call this callback to get
// the memory for point cloud storage.
std::shared_ptr<PointCloudMsg> allocatePointCloudMemoryCallback(void) {
  return std::make_shared<PointCloudMsg>();
}

// @brief this callback which would return the point cloud to your own
// application from sdk
void pointCloudCallback(std::shared_ptr<PointCloudMsg> msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_pointcloud->points.swap(msg->points);
  pcl_pointcloud->height = msg->height;
  pcl_pointcloud->width = msg->width;
  pcl_pointcloud->is_dense = msg->is_dense;

  PointCloudColorHandlerGenericField<pcl::PointXYZI> point_color_handle(pcl_pointcloud, "intensity");
  {
    const std::lock_guard<std::mutex> lock(mtx_viewer);
    // pcl_viewer->updatePointCloud<pcl::PointXYZI>(pcl_pointcloud,
    // point_color_handle, "vanjeelidar");
    pcl_viewer->removePointCloud("vanjeelidar");
    pcl_viewer->addPointCloud<pcl::PointXYZI>(pcl_pointcloud, point_color_handle, "vanjeelidar");
  }
}

// @brief allocate memory for imu，sdk would call this callback to get the
// memory for imu storage.
std::shared_ptr<ImuPacket> allocateImuPacketMemoryCallback() {
  return std::make_shared<ImuPacket>();
}

// @brief this callback which would return the imu to your own application from
// sdk
void imuPacketCallback(std::shared_ptr<ImuPacket> msg) {
#if 0
  WJ_MSG << "seq: " << msg->seq << ", timestamp: " << std::to_string(msg->timestamp) << WJ_REND;
  WJ_MSG << "x_acc: " << msg->linear_acce[0] << ", y_acc: " << msg->linear_acce[1]<< ", z_acc: " << msg->linear_acce[2] << WJ_REND;
  WJ_MSG << "angular_voc_x: " << msg->angular_voc[0] << ", angular_voc_y: " << msg->angular_voc[1]<< ", angular_voc_z: " << msg->angular_voc[2] <<WJ_REND;
#endif
}

// @brief allocate memory for laserScan，sdk would call this callback to get the
// memory for laserScan storage.
std::shared_ptr<ScanData> allocateLaserScanMemoryCallback() {
  return std::make_shared<ScanData>();
}

// @brief this callback which would return the laserScan to your own application
// from sdk
void laserScanCallback(std::shared_ptr<ScanData> msg) {
#if 0
    WJ_MSG << "seq: " << msg->seq << ", timestamp: " << std::to_string(msg->timestamp) << "data_size: " << msg->ranges.size() << WJ_REND;
#endif
}

int main(int argc, char* argv[]) {
  WJ_TITLE << "------------------------------------------------------" << WJ_REND;
  WJ_TITLE << "            Vanjee_Driver Viewer Version: v" << VANJEE_LIDAR_VERSION_MAJOR << "." << VANJEE_LIDAR_VERSION_MINOR << "."
           << VANJEE_LIDAR_VERSION_PATCH << WJ_REND;
  WJ_TITLE << "------------------------------------------------------" << WJ_REND;

  if (argc < 2) {
    printHelpMenu();
    // return 0;
  }

  if (checkKeywordExist(argc, argv, "-h") || checkKeywordExist(argc, argv, "--help")) {
    printHelpMenu();
    // return 0;
  }

  WJDriverParam param;

  param.input_type = InputType::PCAP_FILE;
  param.lidar_type = LidarType::vanjee_720_16;
  param.input_param.host_msop_port = 3001;
  param.input_param.lidar_msop_port = 3333;
  param.input_param.host_address = "192.168.2.88";
  param.input_param.lidar_address = "192.168.2.86";
  param.input_param.pcap_path = "";
  param.input_param.pcap_rate = 10;
  param.decoder_param.angle_path_ver = ".../src/vanjee_lidar_sdk/param/Vanjee_720_16.csv";
  param.decoder_param.imu_param_path = ".../src/vanjee_lidar_sdk/param/vanjee_720_imu_param.csv";
  param.decoder_param.config_from_file = true;
  param.decoder_param.wait_for_difop = false;
  param.decoder_param.imu_enable = 1;  // -1: disable IMU ; 0: enable IMU but calibrate IMU using default
                                       // parameters ; 1: enable IMU;

  parseParam(argc, argv, param);
  param.print();

  pcl_viewer = std::make_shared<PCLVisualizer>("VanjeePointCloudViewer");
  pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);
  pcl_viewer->addCoordinateSystem(1.0);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_viewer->addPointCloud<pcl::PointXYZI>(pcl_pointcloud, "vanjeelidar");
  pcl_viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "vanjeelidar");

  LidarDriver<PointCloudMsg> driver;
  // @brief these two callback,allocatePointCloudMemoryCallback and
  // pointCloudCallback, have to be implemented,and as the parameter of
  // regPointCloudCallback function.
  driver.regPointCloudCallback(allocatePointCloudMemoryCallback, pointCloudCallback);
  // @brief exceptionCallback have to be implemented,and as the parameter of
  // regExceptionCallback function.
  driver.regExceptionCallback(exceptionCallback);
  // @brief these two callback,allocateImuPacketMemoryCallback and
  // imuPacketCallback, have to be implemented,and as the parameter of
  // regImuPacketCallback function.
  driver.regImuPacketCallback(allocateImuPacketMemoryCallback, imuPacketCallback);
  // @brief these two callback,allocateLaserScanMemoryCallback and
  // laserScanCallback, have to be implemented,and as the parameter of
  // regScanDataCallback function.
  driver.regScanDataCallback(allocateLaserScanMemoryCallback, laserScanCallback);

  if (!driver.init(param)) {
    WJ_ERROR << "Driver Initialize Error..." << WJ_REND;
    return -1;
  }

  WJ_INFO << "Vanjee Lidar-Driver Viewer start......" << WJ_REND;

  driver.start();

  while (!pcl_viewer->wasStopped()) {
    {
      const std::lock_guard<std::mutex> lock(mtx_viewer);
      pcl_viewer->spinOnce();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
