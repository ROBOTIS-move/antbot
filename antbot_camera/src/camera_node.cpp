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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "antbot_camera/camera_driver.hpp"
#include "antbot_camera/v4l2_driver.hpp"
#include "antbot_camera/usb_driver.hpp"
#include "antbot_camera/vendor/camera_vendor_handler.hpp"

namespace antbot_camera
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options)
  : Node("antbot_camera_node", options)
  {
    load_parameters();
    create_drivers();
    create_all_publishers();

    // Use the minimum timer period (highest fps among all cameras)
    auto period = std::chrono::milliseconds(33);  // ~30Hz max
    timer_ = this->create_wall_timer(period, std::bind(&CameraNode::timer_callback, this));
    RCLCPP_INFO(
      this->get_logger(), "CameraNode initialized with %zu camera types",
      camera_list_.size());
  }

  ~CameraNode()
  {
    timer_->cancel();
    for (auto & [type, position_map] : drivers_) {
      for (auto & [position, driver] : position_map) {
        driver->release();
      }
    }
  }

private:
  struct PublisherSet
  {
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub;
  };

  void load_parameters()
  {
    this->declare_parameter<std::vector<std::string>>(
      "camera.camera_list", std::vector<std::string>());
    camera_list_ = this->get_parameter("camera.camera_list").as_string_array();

    // Declare V4L2 parameters
    this->declare_parameter<int>("camera.v4l2_driver.resolution.width", 640);
    this->declare_parameter<int>("camera.v4l2_driver.resolution.height", 360);
    this->declare_parameter<int>("camera.v4l2_driver.default_fps", 15);
    this->declare_parameter<bool>("camera.v4l2_driver.use_rom_mode", false);
    this->declare_parameter<std::string>(
      "camera.v4l2_driver.port_info.root", "/sys/class/video4linux");
    this->declare_parameter<std::vector<std::string>>(
      "camera.v4l2_driver.camera_position_list", std::vector<std::string>());

    // Declare USB camera parameters
    this->declare_parameter<int>("camera.usb_camera.resolution.width", 640);
    this->declare_parameter<int>("camera.usb_camera.resolution.height", 480);
    this->declare_parameter<int>("camera.usb_camera.default_fps", 10);
    this->declare_parameter<std::vector<std::string>>(
      "camera.usb_camera.camera_position_list", std::vector<std::string>());
  }

  void create_drivers()
  {
    for (const auto & camera_type : camera_list_) {
      if (camera_type == "v4l2_driver") {
        create_v4l2_drivers();
      } else if (camera_type == "usb_camera") {
        create_usb_drivers();
      } else {
        RCLCPP_WARN(this->get_logger(), "Unknown camera type: %s", camera_type.c_str());
      }
    }
  }

  void create_v4l2_drivers()
  {
    int width = this->get_parameter("camera.v4l2_driver.resolution.width").as_int();
    int height = this->get_parameter("camera.v4l2_driver.resolution.height").as_int();
    int default_fps = this->get_parameter("camera.v4l2_driver.default_fps").as_int();
    bool use_rom_mode = this->get_parameter("camera.v4l2_driver.use_rom_mode").as_bool();
    std::string port_root = this->get_parameter(
      "camera.v4l2_driver.port_info.root").as_string();
    auto position_list = this->get_parameter(
      "camera.v4l2_driver.camera_position_list").as_string_array();

    // Create default Novitec vendor handler for the whole set
    auto novitec_handler = vendor::create_vendor_handler("novitec", port_root);

    for (const auto & pos : position_list) {
      // Declare per-position parameters
      this->declare_parameter<std::string>("camera.v4l2_driver." + pos + ".port", "");
      this->declare_parameter<std::string>("camera.v4l2_driver." + pos + ".fourcc_type", "UYVY");
      this->declare_parameter<std::string>(
        "camera.v4l2_driver." + pos + ".distortion_model", "equidistant");
      this->declare_parameter<std::string>(
        "camera.v4l2_driver." + pos + ".frame_id", pos + "_camera_optical_frame");
      this->declare_parameter<std::string>(
        "camera.v4l2_driver." + pos + ".vendor", "novitec");
      this->declare_parameter<int>(
        "camera.v4l2_driver." + pos + ".grab_width", 1920);
      this->declare_parameter<int>(
        "camera.v4l2_driver." + pos + ".grab_height", 1080);
      this->declare_parameter<int>(
        "camera.v4l2_driver." + pos + ".fps", default_fps);

      std::string port = this->get_parameter("camera.v4l2_driver." + pos + ".port").as_string();
      std::string fourcc = this->get_parameter(
        "camera.v4l2_driver." + pos + ".fourcc_type").as_string();
      std::string distortion_model = this->get_parameter(
        "camera.v4l2_driver." + pos + ".distortion_model").as_string();
      std::string frame_id = this->get_parameter(
        "camera.v4l2_driver." + pos + ".frame_id").as_string();
      std::string vendor_name = this->get_parameter(
        "camera.v4l2_driver." + pos + ".vendor").as_string();
      int grab_width = this->get_parameter(
        "camera.v4l2_driver." + pos + ".grab_width").as_int();
      int grab_height = this->get_parameter(
        "camera.v4l2_driver." + pos + ".grab_height").as_int();
      int fps = this->get_parameter(
        "camera.v4l2_driver." + pos + ".fps").as_int();

      // Use appropriate vendor handler
      std::shared_ptr<vendor::CameraVendorHandler> handler;
      if (vendor_name == "Generic" || vendor_name == "generic") {
        handler = vendor::create_vendor_handler("generic", port_root);
      } else {
        handler = novitec_handler;
      }

      auto driver = std::make_shared<V4l2Driver>(
        pos, this, cv::Size(width, height), fps, port, frame_id,
        fourcc, use_rom_mode, cv::Size(grab_width, grab_height),
        distortion_model, handler);

      if (driver->initialize()) {
        drivers_["v4l2_driver"][pos] = driver;
        RCLCPP_INFO(this->get_logger(), "V4L2 driver created for: %s", pos.c_str());
      } else {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to initialize V4L2 driver for: %s", pos.c_str());
        drivers_["v4l2_driver"][pos] = driver;  // Keep for retry
      }
    }
  }

  void create_usb_drivers()
  {
    int width = this->get_parameter("camera.usb_camera.resolution.width").as_int();
    int height = this->get_parameter("camera.usb_camera.resolution.height").as_int();
    int default_fps = this->get_parameter("camera.usb_camera.default_fps").as_int();
    auto position_list = this->get_parameter(
      "camera.usb_camera.camera_position_list").as_string_array();

    for (const auto & pos : position_list) {
      this->declare_parameter<std::string>(
        "camera.usb_camera." + pos + ".device_name", "");
      this->declare_parameter<std::string>(
        "camera.usb_camera." + pos + ".rotation", "");
      this->declare_parameter<std::string>(
        "camera.usb_camera." + pos + ".frame_id", pos + "_camera_optical_frame");
      this->declare_parameter<int>(
        "camera.usb_camera." + pos + ".fps", default_fps);

      std::string device_name = this->get_parameter(
        "camera.usb_camera." + pos + ".device_name").as_string();
      std::string rotation_str = this->get_parameter(
        "camera.usb_camera." + pos + ".rotation").as_string();
      std::string frame_id = this->get_parameter(
        "camera.usb_camera." + pos + ".frame_id").as_string();
      int fps = this->get_parameter(
        "camera.usb_camera." + pos + ".fps").as_int();

      int rotation = -1;
      if (rotation_str == "90") {
        rotation = cv::ROTATE_90_CLOCKWISE;
      } else if (rotation_str == "180") {
        rotation = cv::ROTATE_180;
      } else if (rotation_str == "270") {
        rotation = cv::ROTATE_90_COUNTERCLOCKWISE;
      }

      // Find device path by device_name
      std::string device_path;
      if (!device_name.empty()) {
        // Search /sys/class/video4linux for the device
        std::string port_root = "/sys/class/video4linux";
        auto generic_handler = vendor::create_vendor_handler("generic", port_root);
        device_path = generic_handler->get_device_path(device_name);
      }

      if (device_path.empty() || device_path == "Unknown") {
        RCLCPP_ERROR(
          this->get_logger(), "USB camera device not found: %s", device_name.c_str());
        continue;
      }

      auto driver = std::make_shared<UsbDriver>(
        pos, this, cv::Size(width, height), fps, device_path, frame_id, rotation);

      if (driver->initialize()) {
        drivers_["usb_camera"][pos] = driver;
        RCLCPP_INFO(this->get_logger(), "USB driver created for: %s", pos.c_str());
      } else {
        RCLCPP_ERROR(
          this->get_logger(), "Failed to initialize USB driver for: %s", pos.c_str());
        drivers_["usb_camera"][pos] = driver;
      }
    }
  }

  void create_all_publishers()
  {
    auto qos = rclcpp::SensorDataQoS();

    for (auto & [type, position_map] : drivers_) {
      for (auto & [position, driver] : position_map) {
        PublisherSet pubs;

        std::string prefix = "/sensor/camera/" + type + "/" + position + "/";
        pubs.image_pub = this->create_publisher<sensor_msgs::msg::Image>(
          prefix + "image_raw", qos);
        pubs.info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(
          prefix + "camera_info", qos);

        publishers_[type + "/" + position] = pubs;
      }
    }
  }

  void timer_callback()
  {
    for (auto & [type, position_map] : drivers_) {
      for (auto & [position, driver] : position_map) {
        if (driver->get_status() == DriverStatus::ERROR) {
          driver->release();
          driver->initialize();
          continue;
        }
        if (driver->get_status() == DriverStatus::UNINITIALIZED) {
          continue;
        }

        if (!driver->update()) {
          continue;
        }

        const auto & frame = driver->get_frame_data();
        const std::string key = type + "/" + position;
        auto it = publishers_.find(key);
        if (it == publishers_.end()) {
          continue;
        }
        auto & pubs = it->second;
        auto stamp = this->now();

        if (!frame.color.empty() && pubs.image_pub) {
          auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame.color).toImageMsg();
          msg->header.stamp = stamp;
          msg->header.frame_id = frame.frame_id;
          pubs.image_pub->publish(*msg);

          if (pubs.info_pub) {
            auto info = frame.color_info;
            info.header.stamp = stamp;
            info.header.frame_id = frame.frame_id;
            pubs.info_pub->publish(info);
          }
        }

        if (!frame.depth.empty() && pubs.depth_image_pub) {
          auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "16UC1", frame.depth).toImageMsg();
          msg->header.stamp = stamp;
          msg->header.frame_id = frame.frame_id;
          pubs.depth_image_pub->publish(*msg);

          if (pubs.depth_info_pub) {
            auto info = frame.depth_info;
            info.header.stamp = stamp;
            info.header.frame_id = frame.frame_id;
            pubs.depth_info_pub->publish(info);
          }
        }
      }
    }
  }

  std::map<std::string, std::map<std::string, std::shared_ptr<CameraDriver>>> drivers_;
  std::map<std::string, PublisherSet> publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> camera_list_;
};

}  // namespace antbot_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<antbot_camera::CameraNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
