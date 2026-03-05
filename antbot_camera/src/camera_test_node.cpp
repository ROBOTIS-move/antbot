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

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace antbot_camera
{

class CameraTestNode : public rclcpp::Node
{
public:
  CameraTestNode()
  : Node("camera_test_node")
  {
    load_parameters();
    setup_subscriptions();
    RCLCPP_INFO(
      this->get_logger(), "CameraTestNode initialized — subscribing to %zu camera(s)",
      subscriptions_.size());
    RCLCPP_INFO(this->get_logger(), "Output directory: %s", output_dir_.c_str());
  }

private:
  struct CameraSubscription
  {
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
    sensor_msgs::msg::CameraInfo latest_info;
    bool info_received = false;
    rclcpp::Time last_save_time{0, 0, RCL_ROS_TIME};
    int frame_count = 0;
    rclcpp::Time fps_window_start{0, 0, RCL_ROS_TIME};
    double current_fps = 0.0;
  };

  void load_parameters()
  {
    // Test node parameters
    this->declare_parameter<std::string>("test.output_dir", "/tmp/antbot_camera_test");
    this->declare_parameter<double>("test.save_interval_sec", 1.0);
    output_dir_ = this->get_parameter("test.output_dir").as_string();
    save_interval_sec_ = this->get_parameter("test.save_interval_sec").as_double();

    // Create output directory
    std::filesystem::create_directories(output_dir_);

    // Read camera_list from the same config as camera_node
    this->declare_parameter<std::vector<std::string>>(
      "camera.camera_list", std::vector<std::string>());
    camera_list_ = this->get_parameter("camera.camera_list").as_string_array();

    // Declare position list parameters for each camera type
    for (const auto & type : camera_list_) {
      this->declare_parameter<std::vector<std::string>>(
        "camera." + type + ".camera_position_list", std::vector<std::string>());
    }
  }

  void setup_subscriptions()
  {
    auto qos = rclcpp::SensorDataQoS();

    for (const auto & type : camera_list_) {
      auto position_list = this->get_parameter(
        "camera." + type + ".camera_position_list").as_string_array();

      for (const auto & pos : position_list) {
        if (type == "gemini336l") {
          // Gemini336L publishes color and depth as separate topics
          setup_single_subscription(type, pos + "/color", qos);
          setup_single_subscription(type, pos + "/depth", qos);
        } else {
          setup_single_subscription(type, pos, qos);
        }
      }
    }
  }

  void setup_single_subscription(
    const std::string & type,
    const std::string & position,
    const rclcpp::QoS & qos)
  {
    std::string key = type + "/" + position;
    std::string topic_prefix = "/sensor/camera/" + key + "/";

    auto & sub = subscriptions_[key];

    sub.info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      topic_prefix + "camera_info", qos,
      [this, key](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        on_camera_info(key, msg);
      });

    sub.image_sub = this->create_subscription<sensor_msgs::msg::Image>(
      topic_prefix + "image_raw", qos,
      [this, key](const sensor_msgs::msg::Image::SharedPtr msg) {
        on_image(key, msg);
      });

    RCLCPP_INFO(
      this->get_logger(), "Subscribed: %simage_raw, %scamera_info",
      topic_prefix.c_str(), topic_prefix.c_str());
  }

  void on_camera_info(
    const std::string & key,
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    auto & sub = subscriptions_[key];
    sub.latest_info = *msg;
    sub.info_received = true;
  }

  void on_image(
    const std::string & key,
    const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto & sub = subscriptions_[key];

    // Update FPS calculation
    auto now = this->now();
    sub.frame_count++;
    if (sub.fps_window_start.nanoseconds() == 0) {
      sub.fps_window_start = now;
    } else {
      double elapsed = (now - sub.fps_window_start).seconds();
      if (elapsed >= 1.0) {
        sub.current_fps = sub.frame_count / elapsed;
        sub.frame_count = 0;
        sub.fps_window_start = now;
      }
    }

    // Check save interval
    if (sub.last_save_time.nanoseconds() != 0) {
      double since_last = (now - sub.last_save_time).seconds();
      if (since_last < save_interval_sec_) {
        return;
      }
    }
    sub.last_save_time = now;

    // Convert to cv::Mat
    cv::Mat display_image;
    try {
      if (msg->encoding == "16UC1") {
        // Depth image: convert to colormap for visualization
        auto cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        cv::Mat & depth = cv_ptr->image;

        double min_val, max_val;
        cv::minMaxLoc(depth, &min_val, &max_val);
        if (max_val > 0) {
          cv::Mat depth_8u;
          depth.convertTo(depth_8u, CV_8UC1, 255.0 / max_val);
          cv::applyColorMap(depth_8u, display_image, cv::COLORMAP_JET);
        } else {
          display_image = cv::Mat::zeros(depth.size(), CV_8UC3);
        }
      } else {
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        display_image = cv_ptr->image;
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "[%s] cv_bridge error: %s", key.c_str(), e.what());
      return;
    }

    // Overlay camera info
    overlay_info(display_image, key, sub, msg->header);

    // Save image
    save_image(display_image, key, msg->header.stamp);
  }

  void overlay_info(
    cv::Mat & image,
    const std::string & key,
    const CameraSubscription & sub,
    const std_msgs::msg::Header & header)
  {
    int y = 20;
    const int line_height = 18;
    const double font_scale = 0.45;
    const int thickness = 1;
    const auto font = cv::FONT_HERSHEY_SIMPLEX;
    const auto color = cv::Scalar(0, 255, 0);
    const auto bg_color = cv::Scalar(0, 0, 0);

    auto put_text = [&](const std::string & text) {
        // Draw background for readability
        int baseline = 0;
        auto text_size = cv::getTextSize(text, font, font_scale, thickness, &baseline);
        cv::rectangle(
          image,
          cv::Point(4, y - text_size.height - 2),
          cv::Point(4 + text_size.width + 4, y + baseline + 2),
          bg_color, cv::FILLED);
        cv::putText(image, text, cv::Point(6, y), font, font_scale, color, thickness);
        y += line_height;
      };

    put_text("Topic: " + key);
    put_text("frame_id: " + header.frame_id);

    // Timestamp
    int64_t stamp_sec = header.stamp.sec;
    uint32_t stamp_nsec = header.stamp.nanosec;
    put_text(
      "stamp: " + std::to_string(stamp_sec) + "." +
      std::to_string(stamp_nsec / 1000000));

    put_text("Resolution: " + std::to_string(image.cols) + "x" + std::to_string(image.rows));

    // FPS
    char fps_buf[32];
    std::snprintf(fps_buf, sizeof(fps_buf), "FPS: %.1f", sub.current_fps);
    put_text(std::string(fps_buf));

    // CameraInfo details
    if (sub.info_received) {
      const auto & info = sub.latest_info;
      put_text("distortion_model: " + info.distortion_model);

      if (info.k.size() >= 5) {
        char k_buf[128];
        std::snprintf(
          k_buf, sizeof(k_buf),
          "K: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
          info.k[0], info.k[4], info.k[2], info.k[5]);
        put_text(std::string(k_buf));
      }
    } else {
      put_text("CameraInfo: waiting...");
    }
  }

  void save_image(
    const cv::Mat & image,
    const std::string & key,
    const builtin_interfaces::msg::Time & stamp)
  {
    // Replace '/' with '_' for filename
    std::string filename_key = key;
    std::replace(filename_key.begin(), filename_key.end(), '/', '_');

    std::string filename = output_dir_ + "/" + filename_key + "_" +
      std::to_string(stamp.sec) + ".jpg";

    if (cv::imwrite(filename, image)) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Saved: %s", filename.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Failed to save: %s", filename.c_str());
    }
  }

  std::map<std::string, CameraSubscription> subscriptions_;
  std::vector<std::string> camera_list_;
  std::string output_dir_;
  double save_interval_sec_;
};

}  // namespace antbot_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<antbot_camera::CameraTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
