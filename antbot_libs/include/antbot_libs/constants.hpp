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
// Authors: Daun Jeong

#ifndef ANTBOT_LIBS__CONSTANTS_HPP_
#define ANTBOT_LIBS__CONSTANTS_HPP_

#include <cmath>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"

namespace antbot
{
namespace constants
{
inline rclcpp::QoS SensorDataQoS()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

constexpr double RAD_PER_SEC_TO_RPM = 60.0 / (2 * M_PI);  // rad/sec -> rev/min
constexpr double RPM_TO_RAD_PER_SEC = (2 * M_PI) / 60.0;  // rev/min -> rad/sec
constexpr double DEG_TO_RAD = M_PI / 180.0;  // degree -> radian
constexpr double RAD_TO_DEG = 180.0 / M_PI;  // radian -> degree

constexpr double REV_PER_MIN2_TO_RAD_PER_SEC2 = M_PI / 1800.0;  // rev/min^2 -> rad/sec^2
constexpr double RAD_PER_SEC2_TO_REV_PER_MIN2 = 1800.0 / M_PI;  // rad/sec^2 -> rev/min^2

constexpr double MILLIAMPERE_TO_AMPERE = 0.001;  // mA -> A

constexpr double STEERING_RAD_PER_PULSE = M_PI / 2048;
constexpr int32_t STEERING_PULSE_OFFSET = 2048;

// 12-bit encoder with 4x interpolation: (2 * PI) / (4096 * 4)
constexpr double TICK_TO_RAD = M_PI / 8192.0;

// RCU transmits RPM as (actual_rpm * 100) for 0.01 RPM resolution
constexpr double RCU_RPM_RESOLUTION = 0.01;
constexpr int32_t RCU_RPM_SCALE = 100;

// Wheel motor acceleration scale
// RCU unit = 53.644... rev/min^2 per LSB
constexpr double WHEEL_ACCEL_SCALE = 53.64418029785156;

// Steering profile acceleration scale
// 214.577 rev/min^2 per LSB (from Dynamixel protocol)
constexpr double STEERING_ACCEL_SCALE = 214.577;

// Steering profile velocity scale: Dynamixel velocity register unit
// 0.229 rev/min per LSB (from Dynamixel protocol)
constexpr double DXL_VELOCITY_UNIT = 0.229;

constexpr uint8_t MOTOR_IDLE = 0;
constexpr uint8_t MOTOR_READY_ENTER = 1;
constexpr uint8_t MOTOR_READY = 2;
constexpr uint8_t MOTOR_RUNNING = 3;
constexpr uint8_t MOTOR_FAULT = 4;
constexpr uint8_t MOTOR_BRAKE = 5;

constexpr uint8_t EMERGENCY_NORMAL = 0;
constexpr uint8_t EMERGENCY_ESTOP = 1;

constexpr uint8_t MOTOR_REBOOT_NORMAL = 0;

}  // namespace constants
}  // namespace antbot
#endif  // ANTBOT_LIBS__CONSTANTS_HPP_
