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
// Authors: Jeonggeun Lim, Donghoon Oh, Daun Jeong

#ifndef ANTBOT_IMU__IMU_NODE_HPP_
#define ANTBOT_IMU__IMU_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "antbot_libs/communicator.hpp"

namespace antbot
{
namespace imu
{
class ImuNode : public rclcpp::Node
{
public:
  explicit ImuNode(const rclcpp::NodeOptions & options);
  virtual ~ImuNode();

private:
  struct Attitude
  {
    double roll{0.0};
    double pitch{0.0};
    double yaw{0.0};
  };

  struct Filter
  {
    Attitude current;
    Attitude last;
  };

  struct Sensor
  {
    Attitude accel;
    Attitude gyro;
    Attitude last_accel;
    Attitude last_gyro;
  };

  bool initialize();
  bool connect_to_board();
  bool read_control_table();

  void initialize_parameters();
  void create_interfaces();

  void start_calibration_thread();
  void stop_calibration_thread();
  void calibration_thread_func();

  void publish_timer_callback();

  void get_imu_data();
  sensor_msgs::msg::Imu::UniquePtr convert_imu_msg();
  void initialize_filters(const sensor_msgs::msg::Imu::UniquePtr & imu);
  void get_attitude_from_complementary_filter(
    const sensor_msgs::msg::Imu::UniquePtr & imu);

  void get_attitude_from_accel(
    const sensor_msgs::msg::Imu::UniquePtr & imu, Attitude & attitude);
  void get_attitude_from_gyro(
    const sensor_msgs::msg::Imu::UniquePtr & imu, Attitude & attitude);

  double low_pass_filter(
    const double & cutoff,
    const double & publish_rate,
    const double & input,
    const double & last_output);

  double high_pass_filter(
    const double & cutoff,
    const double & publish_rate,
    const double & input,
    const double & last_input,
    const double & last_output);

  std::shared_ptr<antbot::libs::Communicator> communicator_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::thread calibration_thread_;
  std::atomic_bool calibration_running_{false};
  std::atomic_bool is_calibrated_{false};
  std::atomic_bool moving_{false};
  std::atomic_bool odom_received_{false};

  std::string port_;
  uint8_t device_id_{0};
  uint32_t baud_rate_{0};
  float protocol_version_{0.0};
  std::string frame_id_;
  std::string control_table_path_;

  double filter_cutoff_hz_{10.0};
  double publish_rate_{100.0};
  int calibration_num_{350};
  double accel_scale_{0.00006103515625};
  double gyro_scale_{0.0609756098};
  const double gravity_{9.80665};

  double accel_x_{0.0};
  double accel_y_{0.0};
  double accel_z_{0.0};
  double gyro_x_{0.0};
  double gyro_y_{0.0};
  double gyro_z_{0.0};

  double avg_accel_magnitude_{0.0};
  double avg_gyro_x_{0.0};
  double avg_gyro_y_{0.0};
  double avg_gyro_z_{0.0};

  const double linear_vel_thres_{0.01};
  const double angular_vel_thres_{0.01};
  const double accel_norm_thres_{0.1};
  const double gyro_z_bias_thres_{0.05};

  bool need_init_{true};

  Filter low_pass_;
  Filter high_pass_;
  Sensor sensor_;
};
}  // namespace imu
}  // namespace antbot
#endif  // ANTBOT_IMU__IMU_NODE_HPP_
