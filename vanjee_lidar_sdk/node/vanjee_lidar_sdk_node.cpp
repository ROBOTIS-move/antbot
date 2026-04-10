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

#include <signal.h>

#include <vanjee_driver/macro/version.hpp>

#include "manager/node_manager.hpp"
#ifdef ROS_FOUND
#include <ros/package.h>
#include <ros/ros.h>
#elif ROS2_FOUND
#include <rclcpp/rclcpp.hpp>

#include "source/source_ros_msg_delegate.hpp"
#endif

using namespace vanjee::lidar;
static void sigHandler(int sig) {
  WJ_MSG << "Vanjee-LiDAR-Driver is stopping....." << WJ_REND;
#ifdef ROS_FOUND
  ros::shutdown();
#elif ROS2_FOUND
  rclcpp::shutdown();
#endif
}
int main(int argc, char **argv) {
  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);

  WJ_TITLE << "********************************************************" << WJ_REND;
  WJ_TITLE << "**********                                    **********" << WJ_REND;
  WJ_TITLE << "**********  Vanjee_Lidar_SDK Version: v" << VANJEE_LIDAR_VERSION_MAJOR << "." << VANJEE_LIDAR_VERSION_MINOR << "."
           << VANJEE_LIDAR_VERSION_PATCH << "  **********" << WJ_REND;
  WJ_TITLE << "**********                                    **********" << WJ_REND;
  WJ_TITLE << "********************************************************" << WJ_REND;
#ifdef ROS_FOUND
  ros::init(argc, argv, "vanjee_lidar_sdk_node", ros::init_options::NoSigintHandler);
#elif ROS2_FOUND

  rclcpp::init(argc, argv);
#endif
  std::string config_path;
#ifdef RUN_IN_ROS_WORKSPACE
  config_path = ros::package::getPath("vanjee_lidar_sdk");
#else
  config_path = (std::string)PROJECT_PATH;
#endif
  config_path += "/config/config.yaml";
#ifdef ROS_FOUND

  ros::NodeHandle priv_hh("~");
  std::string path;
  priv_hh.param("config_path", path, std::string(""));
  if (!path.empty()) {
    config_path = path;
  }
#elif ROS2_FOUND
  auto node = VanjeeLidarSdkNode::CreateInstance();
  node->declare_parameter<std::string>("config_path", "");
  std::string path;
  node->get_parameter("config_path", path);
  if (!path.empty()) {
    config_path = path;
  }
#endif
  YAML::Node config;
  try {
    config = YAML::LoadFile(config_path);
  } catch (...) {
    WJ_ERROR << "The format of config file " << config_path << " is wrong. Please check (e.g. indentation)." << WJ_REND;
    return -1;
  }
  std::shared_ptr<NodeManager> demo_ptr = std::make_shared<NodeManager>();
  demo_ptr->init(config);
  demo_ptr->start();
  WJ_MSG << "Vanjee-LiDAR-Driver is running....." << WJ_REND;

#ifdef ROS_FOUND
  ros::spin();
#elif ROS2_FOUND
  rclcpp::spin(VanjeeLidarSdkNode::CreateInstance());
#endif

  return 0;
}
