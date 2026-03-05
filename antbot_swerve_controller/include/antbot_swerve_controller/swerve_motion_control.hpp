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

#ifndef ANTBOT_SWERVE_CONTROLLER__SWERVE_MOTION_CONTROL_HPP_
#define ANTBOT_SWERVE_CONTROLLER__SWERVE_MOTION_CONTROL_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <stdexcept>
#include <numeric>
#include <map>

// ROS 2 message type and time-related dependencies
#include "geometry_msgs/msg/twist.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "rclcpp/rclcpp.hpp"

namespace antbot
{
namespace swerve_drive_controller
{

// Added for compatibility, as M_PI is defined in <cmath> in C++17 and later.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum class RobotState
{
  IDLE,
  CONTINUOUS,
  DISCONTINUOUS_RE_ALIGNMENT   // Discontinuous motion requiring re-alignment
};

enum class TargetDrivingType
{
  STOPPED,
  LINEAR_DRIVING,
  L_TURNING,
  R_TURNING,
  DIAGONAL_DRIVING,
  SPINNING,
  RE_ALIGNING,   // Transitional state
  FAULT   // Invalid state
};

struct PlannedMotion
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  RobotState type = RobotState::CONTINUOUS;
  std::vector<double> steering_peak_velocities;
  std::vector<double> steering_accelerations;
};

// Simple structure for 2D coordinates
struct Point
{
  double x, y;
};

/**
 * @brief A class that generates and manages trapezoidal velocity profiles
 * (for steering control only).
 */
struct TrapezoidalProfile
{
  double start_state = 0.0, end_state = 0.0;
  double accel_time = 0.0, coast_time = 0.0, decel_time = 0.0;
  double total_time = 0.0, peak_velocity = 0.0, acceleration = 0.0;
  double direction = 1.0;
  bool valid = false;

  TrapezoidalProfile() = default;

  TrapezoidalProfile(double start, double end, double max_v, double max_a);

  TrapezoidalProfile(
    double start, double end, double max_v_abs, double max_a_abs,
    double desired_duration);

  bool isValid() const;
  double getPositionAt(double t) const;
  double getVelocityAt(double t) const;
  double getAccelerationAt(double t) const;
};

/**
 * @brief A class that generates and manages Swerve Drive motion profiling plans.
 */
class SwerveMotionControl
{
public:
  SwerveMotionControl();

  void configure(
    size_t num_modules,
    double wheel_radius,
    const std::vector<Point> & module_positions,
    const std::vector<double> & steering_to_wheel_offset,
    const std::vector<double> & module_angle_offsets,
    const std::vector<std::string> & joint_names,
    const double steering_max_vel,
    const double steering_max_accel,
    const double drive_max_accel,
    const std::vector<double> & module_steering_limit_lower,
    const std::vector<double> & module_steering_limit_upper,
    int non_coaxial_ik_iterations,
    double realigning_angle_threshold,
    double discontinuous_motion_steering_tolerance,
    double velocity_deadband
  );

  PlannedMotion plan(
    const geometry_msgs::msg::Twist & desired_setpoint,
    const std::vector<double> & current_steering_angles,
    const std::vector<double> & current_wheel_velocities_rad_per_sec);

  TargetDrivingType determine_target_mode(
    const geometry_msgs::msg::Twist & setpoint) const;

  TargetDrivingType getCurrentMode() const {return current_mode_;}
  const PlannedMotion & getPlannedMotion() const {return last_planned_motion_;}

  void initialize_transition_matrix();

  bool is_angle_within_limits(double angle, double lower_limit, double upper_limit);

  bool is_trajectory_valid(const trajectory_msgs::msg::JointTrajectory & trajectory);

private:
  double normalize_angle(double angle_rad);
  double normalize_angle_positive(double angle);
  double shortest_angular_distance(double from, double to);

  bool handle_realignment_in_progress(
    const std::vector<double> & current_steering_angles,
    const std::vector<double> & current_wheel_velocities_rad_per_sec,
    PlannedMotion & motion_out);

  void compute_inverse_kinematics(
    const geometry_msgs::msg::Twist & desired_setpoint,
    const std::vector<double> & current_steering_angles,
    TargetDrivingType target_mode,
    std::vector<double> & desired_steering_angles,
    std::vector<double> & desired_wheel_speeds_rad_per_sec);

  bool check_mode_transition_needed(
    TargetDrivingType target_mode,
    const std::vector<double> & current_steering_angles,
    const std::vector<double> & desired_steering_angles);

  PlannedMotion generate_realignment_motion(
    TargetDrivingType target_mode,
    const std::vector<double> & current_steering_angles,
    const std::vector<double> & desired_steering_angles,
    const std::vector<double> & current_wheel_velocities_rad_per_sec);

  PlannedMotion generate_continuous_motion(
    TargetDrivingType target_mode,
    const std::vector<double> & current_steering_angles,
    std::vector<double> & desired_steering_angles,
    std::vector<double> & desired_wheel_speeds_rad_per_sec,
    const std::vector<double> & current_wheel_velocities_rad_per_sec);

  trajectory_msgs::msg::JointTrajectory generate_trajectory_message(
    const std::vector<TrapezoidalProfile> & steering_profiles,
    const std::vector<double> & target_drive_speeds,
    double T_sync,
    const std::vector<double> & initial_drive_speeds
  );

  size_t num_modules_;
  double wheel_radius_;
  std::vector<Point> module_positions_;
  std::vector<double> module_angle_offsets_;
  std::vector<double> steering_to_wheel_offset_;
  std::vector<std::string> joint_names_;

  TargetDrivingType target_mode_after_realignment_;
  std::vector<double> realignment_target_angles_;
  TargetDrivingType current_mode_ = TargetDrivingType::STOPPED;
  PlannedMotion last_planned_motion_;

  double steering_max_vel_;
  double steering_max_accel_;
  double drive_max_accel_;

  std::map<TargetDrivingType, std::map<TargetDrivingType, bool>> needs_realignment_matrix_;

  std::vector<double> steering_limits_lower_;
  std::vector<double> steering_limits_upper_;

  std::vector<bool> is_valid_steering_;

  int non_coaxial_ik_iterations_;
  double realigning_angle_threshold_;
  double discontinuous_motion_steering_tolerance_;
  double velocity_deadband_;
  double trajectory_delay_time_;

  TargetDrivingType pending_target_mode_ = TargetDrivingType::STOPPED;
  int mode_transition_count_ = 0;
  static constexpr int MODE_TRANSITION_THRESHOLD = 5;

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::Time realignment_start_time_{0, 0, RCL_STEADY_TIME};
  bool realignment_watchdog_triggered_ = false;
  static constexpr double REALIGNMENT_TIMEOUT_SEC = 2.0;
  static constexpr int MAX_REALIGNMENT_RETRIES = 1;
  int realignment_retry_count_ = 0;

  const std::vector<TargetDrivingType> modes = {
    TargetDrivingType::STOPPED,
    TargetDrivingType::LINEAR_DRIVING,
    TargetDrivingType::L_TURNING,
    TargetDrivingType::R_TURNING,
    TargetDrivingType::DIAGONAL_DRIVING,
    TargetDrivingType::SPINNING
  };
};

}  // namespace swerve_drive_controller
}  // namespace antbot
#endif  // ANTBOT_SWERVE_CONTROLLER__SWERVE_MOTION_CONTROL_HPP_
