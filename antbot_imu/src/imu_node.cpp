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

#include <cmath>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "antbot_imu/imu_node.hpp"
#include "antbot_libs/constants.hpp"

namespace antbot
{
namespace imu
{
ImuNode::ImuNode(const rclcpp::NodeOptions & options)
: Node("imu_node", options)
{
  initialize_parameters();

  if (!initialize()) {
    throw std::runtime_error("Failed to initialize IMU node");
  }

  RCLCPP_INFO(this->get_logger(), "Initialized IMU node");
}

ImuNode::~ImuNode()
{
  stop_calibration_thread();

  publish_timer_.reset();
  imu_pub_.reset();
  odom_sub_.reset();
  communicator_.reset();
}

void ImuNode::initialize_parameters()
{
  this->declare_parameter("imu.frame_id", "imu_link");
  this->declare_parameter("imu.filter_cutoff_hz", 10.0);
  this->declare_parameter("imu.publish_rate", 100.0);
  this->declare_parameter("imu.calibration_num", 350);
  this->declare_parameter("imu.scale.acceleration", 0.00006103515625);
  this->declare_parameter("imu.scale.angular_vel", 0.0609756098);
  this->declare_parameter("imu_board.port", "");
  this->declare_parameter("imu_board.id", 0);
  this->declare_parameter("imu_board.baud_rate", 0);
  this->declare_parameter("imu_board.protocol_version", 0.0);
  this->declare_parameter("control_table_path", "");

  frame_id_ = this->get_parameter("imu.frame_id").as_string();
  filter_cutoff_hz_ = this->get_parameter("imu.filter_cutoff_hz").as_double();
  publish_rate_ = this->get_parameter("imu.publish_rate").as_double();
  calibration_num_ = this->get_parameter("imu.calibration_num").as_int();
  accel_scale_ = this->get_parameter("imu.scale.acceleration").as_double();
  gyro_scale_ = this->get_parameter("imu.scale.angular_vel").as_double();

  port_ = this->get_parameter("imu_board.port").as_string();
  device_id_ = static_cast<uint8_t>(this->get_parameter("imu_board.id").as_int());
  baud_rate_ = static_cast<uint32_t>(this->get_parameter("imu_board.baud_rate").as_int());
  protocol_version_ = static_cast<float>(
    this->get_parameter("imu_board.protocol_version").as_double());
  control_table_path_ = this->get_parameter("control_table_path").as_string();
}

bool ImuNode::initialize()
{
  if (!connect_to_board()) {
    return false;
  }

  create_interfaces();

  start_calibration_thread();

  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<uint32_t>(1000.0 / publish_rate_)),
    std::bind(&ImuNode::publish_timer_callback, this));

  return true;
}

bool ImuNode::connect_to_board()
{
  communicator_ = antbot::libs::create_communicator(
    port_, baud_rate_, protocol_version_, device_id_, control_table_path_);

  if (!communicator_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to IMU board");
    return false;
  }

  if (!this->read_control_table()) {
    return false;
  }

  return true;
}

bool ImuNode::read_control_table()
{
  if (communicator_ == nullptr) {
    return false;
  }

  int result = 0;
  if (!communicator_->read_control_table(&result)) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Failed to read control table: %d", result);
    return false;
  }
  return true;
}

void ImuNode::create_interfaces()
{
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "imu/accel_gyro", constants::SensorDataQoS());

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    constants::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::SharedPtr msg)
    {
      if (!odom_received_.load()) {
        odom_received_.store(true);
      }

      if ((std::fabs(msg->twist.twist.linear.x) > linear_vel_thres_) ||
      (std::fabs(msg->twist.twist.angular.z) > angular_vel_thres_))
      {
        moving_.store(true);
      } else {
        moving_.store(false);
      }
    }
  );
}

void ImuNode::start_calibration_thread()
{
  if (calibration_running_.load()) {
    return;
  }

  calibration_running_.store(true);
  calibration_thread_ = std::thread(&ImuNode::calibration_thread_func, this);
}

void ImuNode::stop_calibration_thread()
{
  calibration_running_.store(false);
  if (calibration_thread_.joinable()) {
    calibration_thread_.join();
  }
}

void ImuNode::calibration_thread_func()
{
  RCLCPP_INFO(this->get_logger(), "Starting IMU calibration");

  auto interval = std::chrono::milliseconds(
    static_cast<uint32_t>(1000.0 / publish_rate_));

  int sample_count = 0;
  int retry_count = 0;

  while (calibration_running_.load() && rclcpp::ok()) {
    auto start = std::chrono::steady_clock::now();

    if (!read_control_table()) {
      continue;
    } else {
      get_imu_data();

      if (!odom_received_.load()) {
        // Wait for encoder velocity
      } else if (moving_.load()) {
        if (sample_count > 0) {
          avg_accel_magnitude_ = 0.0;
          avg_gyro_x_ = 0.0;
          avg_gyro_y_ = 0.0;
          avg_gyro_z_ = 0.0;
          sample_count = 0;
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(),
            1000, "Robot moved during calibration, resetting collected samples");
        }
      } else if (sample_count < calibration_num_) {
        avg_accel_magnitude_ +=
          std::sqrt(std::pow(accel_x_, 2) + std::pow(accel_y_, 2) + std::pow(accel_z_, 2));
        avg_gyro_x_ += gyro_x_;
        avg_gyro_y_ += gyro_y_;
        avg_gyro_z_ += gyro_z_;
        sample_count++;
      } else {
        avg_accel_magnitude_ /= static_cast<double>(sample_count);
        avg_gyro_x_ /= static_cast<double>(sample_count);
        avg_gyro_y_ /= static_cast<double>(sample_count);
        avg_gyro_z_ /= static_cast<double>(sample_count);

        RCLCPP_INFO(this->get_logger(), "IMU calibration completed");
        RCLCPP_INFO(
          this->get_logger(),
          "acc: %1.5f %1.5f %1.5f, ang: %1.5f %1.5f %1.5f",
          accel_x_ * gravity_ / avg_accel_magnitude_,
          accel_y_ * gravity_ / avg_accel_magnitude_,
          accel_z_ * gravity_ / avg_accel_magnitude_,
          gyro_x_ - avg_gyro_x_,
          gyro_y_ - avg_gyro_y_,
          gyro_z_ - avg_gyro_z_);

        if (std::fabs(avg_gyro_z_) > gyro_z_bias_thres_ && retry_count < 3) {
          avg_accel_magnitude_ = 0.0;
          avg_gyro_x_ = 0.0;
          avg_gyro_y_ = 0.0;
          avg_gyro_z_ = 0.0;
          sample_count = 0;
          retry_count++;
          RCLCPP_WARN(
            this->get_logger(), "Invalid calibration data, restarting calibration");
        } else {
          is_calibrated_.store(true);
          calibration_running_.store(false);
          break;
        }
      }
    }

    auto elapsed = std::chrono::steady_clock::now() - start;
    auto sleep_time = interval - elapsed;
    if (sleep_time > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleep_time);
    }
  }
}

void ImuNode::publish_timer_callback()
{
  if (is_calibrated_.load()) {
    if (!read_control_table()) {
      return;
    }

    get_imu_data();

    sensor_msgs::msg::Imu::UniquePtr imu_msg = this->convert_imu_msg();
    if (imu_msg) {
      if (need_init_) {
        initialize_filters(imu_msg);
        need_init_ = false;
        return;
      }

      get_attitude_from_complementary_filter(imu_msg);
      imu_pub_->publish(std::move(imu_msg));
    }
  }
}

void ImuNode::get_imu_data()
{
  accel_x_ =
    static_cast<double>(
    communicator_->get_data<int16_t>("IMU_Acc_X")) * accel_scale_ * gravity_;
  accel_y_ =
    static_cast<double>(
    communicator_->get_data<int16_t>("IMU_Acc_Y")) * accel_scale_ * gravity_;
  accel_z_ =
    static_cast<double>(
    communicator_->get_data<int16_t>("IMU_Acc_Z")) * accel_scale_ * gravity_;

  gyro_x_ =
    static_cast<double>(
    communicator_->get_data<int16_t>("IMU_Gyro_X")) * gyro_scale_ * constants::DEG_TO_RAD;
  gyro_y_ =
    static_cast<double>(
    communicator_->get_data<int16_t>("IMU_Gyro_Y")) * gyro_scale_ * constants::DEG_TO_RAD;
  gyro_z_ =
    static_cast<double>(
    communicator_->get_data<int16_t>("IMU_Gyro_Z")) * gyro_scale_ * constants::DEG_TO_RAD;
}

sensor_msgs::msg::Imu::UniquePtr ImuNode::convert_imu_msg()
{
  sensor_msgs::msg::Imu::UniquePtr msg = std::make_unique<sensor_msgs::msg::Imu>();
  msg->header.frame_id = frame_id_;
  msg->header.stamp = this->now();

  if (std::fabs(avg_accel_magnitude_) > 1e-6) {
    msg->linear_acceleration.x = accel_x_ * gravity_ / avg_accel_magnitude_;
    msg->linear_acceleration.y = accel_y_ * gravity_ / avg_accel_magnitude_;
    msg->linear_acceleration.z = accel_z_ * gravity_ / avg_accel_magnitude_;
  } else {
    msg->linear_acceleration.x = accel_x_;
    msg->linear_acceleration.y = accel_y_;
    msg->linear_acceleration.z = accel_z_;
  }

  msg->angular_velocity.x = gyro_x_ - avg_gyro_x_;
  msg->angular_velocity.y = gyro_y_ - avg_gyro_y_;
  msg->angular_velocity.z = gyro_z_ - avg_gyro_z_;

  return msg;
}

void ImuNode::initialize_filters(const sensor_msgs::msg::Imu::UniquePtr & imu)
{
  get_attitude_from_accel(imu, sensor_.accel);
  get_attitude_from_gyro(imu, sensor_.gyro);

  low_pass_.last.roll = sensor_.accel.roll;
  low_pass_.last.pitch = sensor_.accel.pitch;
  high_pass_.last.roll = 0.0;
  high_pass_.last.pitch = 0.0;
  sensor_.last_gyro = sensor_.gyro;
}

void ImuNode::get_attitude_from_complementary_filter(
  const sensor_msgs::msg::Imu::UniquePtr & imu)
{
  get_attitude_from_accel(imu, sensor_.accel);
  get_attitude_from_gyro(imu, sensor_.gyro);

  low_pass_.current.roll =
    low_pass_filter(
    filter_cutoff_hz_, publish_rate_, sensor_.accel.roll, low_pass_.last.roll);
  low_pass_.current.pitch =
    low_pass_filter(
    filter_cutoff_hz_, publish_rate_, sensor_.accel.pitch, low_pass_.last.pitch);

  high_pass_.current.roll =
    high_pass_filter(
    filter_cutoff_hz_, publish_rate_, sensor_.gyro.roll,
    sensor_.last_gyro.roll, high_pass_.last.roll);
  high_pass_.current.pitch =
    high_pass_filter(
    filter_cutoff_hz_, publish_rate_, sensor_.gyro.pitch,
    sensor_.last_gyro.pitch, high_pass_.last.pitch);

  sensor_.last_accel = sensor_.accel;
  sensor_.last_gyro = sensor_.gyro;

  low_pass_.last = low_pass_.current;
  high_pass_.last = high_pass_.current;

  tf2::Quaternion q;
  q.setRPY(
    low_pass_.current.roll + high_pass_.current.roll,
    low_pass_.current.pitch + high_pass_.current.pitch,
    0.0);

  imu->orientation.x = q.x();
  imu->orientation.y = q.y();
  imu->orientation.z = q.z();
  imu->orientation.w = q.w();
}

double ImuNode::low_pass_filter(
  const double & cutoff,
  const double & publish_rate,
  const double & input,
  const double & last_output)
{
  double output = 0.0;
  double delta = 1.0 / (publish_rate);
  double tau = 1.0 / (2.0 * M_PI * cutoff);

  output = (tau * last_output + delta * input) / (tau + delta);

  return output;
}

double ImuNode::high_pass_filter(
  const double & cutoff,
  const double & publish_rate,
  const double & input,
  const double & last_input,
  const double & last_output)
{
  double delta = 1.0 / (publish_rate);
  double tau = 1.0 / (2.0 * M_PI * cutoff);
  double coeff = tau / (tau + delta);

  double output = coeff * (last_output + input - last_input);

  return output;
}

void ImuNode::get_attitude_from_gyro(
  const sensor_msgs::msg::Imu::UniquePtr & imu,
  Attitude & attitude)
{
  const double reset_angle = constants::DEG_TO_RAD;

  attitude.roll += imu->angular_velocity.x / publish_rate_;
  attitude.pitch += imu->angular_velocity.y / publish_rate_;

  if (std::abs(sensor_.accel.roll) < reset_angle || !moving_.load()) {
    attitude.roll = 0.0;
  }

  if (std::abs(sensor_.accel.pitch) < reset_angle || !moving_.load()) {
    attitude.pitch = 0.0;
  }
}

void ImuNode::get_attitude_from_accel(
  const sensor_msgs::msg::Imu::UniquePtr & imu,
  Attitude & attitude)
{
  double numerator =
    std::sqrt(std::pow(imu->linear_acceleration.y, 2) + std::pow(imu->linear_acceleration.z, 2));

  if (std::abs(numerator) < accel_norm_thres_) {
    attitude.pitch = M_PI_2;
  } else {
    attitude.pitch = std::atan2(-imu->linear_acceleration.x, numerator);
  }

  numerator =
    std::sqrt(std::pow(imu->linear_acceleration.x, 2) + std::pow(imu->linear_acceleration.z, 2));

  if (std::abs(numerator) < accel_norm_thres_) {
    attitude.roll = M_PI_2;
  } else {
    attitude.roll = std::atan2(imu->linear_acceleration.y, numerator);
  }
}
}  // namespace imu
}  // namespace antbot
