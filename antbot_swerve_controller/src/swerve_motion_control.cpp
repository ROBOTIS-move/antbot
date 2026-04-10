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

#include "antbot_swerve_controller/swerve_motion_control.hpp"

namespace antbot
{
namespace swerve_drive_controller
{

// TrapezoidalProfile implementations
TrapezoidalProfile::TrapezoidalProfile(double start, double end, double max_v, double max_a)
{
  if (max_v <= 0 || max_a <= 0) {
    valid = false;
    return;
  }

  start_state = start;
  end_state = end;
  direction = (end > start) ? 1.0 : -1.0;
  double distance = std::abs(end - start);

  if (distance < 1e-9) {
    total_time = 0.0;
    valid = true;
    return;
  }

  double time_to_reach_max_v = max_v / max_a;
  double distance_at_max_v_accel = max_a * time_to_reach_max_v * time_to_reach_max_v;

  if (distance > distance_at_max_v_accel) {
    acceleration = max_a;
    peak_velocity = max_v;
    accel_time = decel_time = time_to_reach_max_v;
    coast_time = (distance - distance_at_max_v_accel) / peak_velocity;
    total_time = accel_time + coast_time + decel_time;
  } else {
    acceleration = max_a;
    accel_time = decel_time = std::sqrt(distance / max_a);
    peak_velocity = acceleration * accel_time;
    coast_time = 0.0;
    total_time = accel_time + decel_time;
  }
  valid = true;
}

TrapezoidalProfile::TrapezoidalProfile(
  double start, double end, double max_v_abs, double max_a_abs,
  double desired_duration)
{
  start_state = start;
  end_state = end;
  total_time = desired_duration;
  direction = (end > start) ? 1.0 : -1.0;
  double distance = std::abs(end - start);
  valid = false;

  if (max_v_abs <= 0 || max_a_abs <= 0) {
    return;
  }

  if (distance < 1e-9) {
    acceleration = peak_velocity = accel_time = coast_time = decel_time = 0;
    valid = true;
    return;
  }

  double min_time_needed = std::sqrt(4.0 * distance / max_a_abs);
  if (desired_duration < min_time_needed - 1e-6) {
    RCLCPP_WARN(
      rclcpp::get_logger("TrapezoidalProfile"),
      "Invalid profile request: Desired duration (%.4fs) is physically too short. "
      "Minimum required time is %.4fs.", desired_duration, min_time_needed);
    return;
  }

  double peak_v_for_triangle = 2.0 * distance / desired_duration;

  double min_time_for_trapezoid_at_max_v = 2.0 * max_v_abs / max_a_abs;

  if (peak_v_for_triangle > max_v_abs && desired_duration > min_time_for_trapezoid_at_max_v) {
    acceleration = max_a_abs;
    peak_velocity = max_v_abs;
    accel_time = decel_time = max_v_abs / max_a_abs;
    coast_time = desired_duration - accel_time - decel_time;
  } else {
    coast_time = 0.0;
    accel_time = decel_time = desired_duration / 2.0;
    acceleration = 4.0 * distance / (desired_duration * desired_duration);
    peak_velocity = acceleration * accel_time;
  }

  if (acceleration > max_a_abs + 1e-6 || peak_velocity > max_v_abs + 1e-6) {
    RCLCPP_WARN(
      rclcpp::get_logger("TrapezoidalProfile"),
      "Profile calculation error: Required accel(%.2f) or velocity(%.2f) exceeds limits.",
      acceleration, peak_velocity);
    return;
  }

  valid = true;
}

bool TrapezoidalProfile::isValid() const
{
  return valid;
}

double TrapezoidalProfile::getPositionAt(double t) const
{
  if (!valid) {
    return start_state;
  }
  t = std::clamp(t, 0.0, total_time);

  if (t < accel_time) {
    return start_state + direction * (0.5 * acceleration * t * t);
  } else if (t < accel_time + coast_time) {
    double coast_start_pos = start_state + direction *
      (0.5 * acceleration * accel_time * accel_time);
    return coast_start_pos + direction * (peak_velocity * (t - accel_time));
  } else {
    double decel_start_t = t - accel_time - coast_time;
    double coast_end_pos = start_state + direction *
      (0.5 * acceleration * accel_time * accel_time + peak_velocity * coast_time);
    return coast_end_pos + direction *
           (peak_velocity * decel_start_t - 0.5 * acceleration * decel_start_t * decel_start_t);
  }
}

double TrapezoidalProfile::getVelocityAt(double t) const
{
  if (!valid) {
    return 0.0;
  }
  t = std::clamp(t, 0.0, total_time);

  if (t < accel_time) {
    return direction * acceleration * t;
  } else if (t < accel_time + coast_time) {
    return direction * peak_velocity;
  } else {
    double time_into_decel = t - accel_time - coast_time;
    return direction * (peak_velocity - acceleration * time_into_decel);
  }
}

double TrapezoidalProfile::getAccelerationAt(double t) const
{
  if (!valid) {
    return 0.0;
  }
  t = std::clamp(t, 0.0, total_time);

  if (t < accel_time) {
    return direction * acceleration;
  } else if (t < accel_time + coast_time) {
    return 0.0;
  } else {
    return -direction * acceleration;
  }
}

// SwerveMotionControl implementations
SwerveMotionControl::SwerveMotionControl()
: num_modules_(0) {}

void SwerveMotionControl::configure(
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
)
{
  num_modules_ = num_modules;
  wheel_radius_ = wheel_radius;
  module_positions_ = module_positions;
  steering_to_wheel_offset_ = steering_to_wheel_offset;
  module_angle_offsets_ = module_angle_offsets;
  joint_names_ = joint_names;

  steering_max_vel_ = steering_max_vel;
  steering_max_accel_ = steering_max_accel;
  drive_max_accel_ = drive_max_accel;

  steering_limits_lower_ = module_steering_limit_lower;
  steering_limits_upper_ = module_steering_limit_upper;

  non_coaxial_ik_iterations_ = non_coaxial_ik_iterations;
  realigning_angle_threshold_ = realigning_angle_threshold;
  discontinuous_motion_steering_tolerance_ = discontinuous_motion_steering_tolerance;
  velocity_deadband_ = velocity_deadband;

  initialize_transition_matrix();

  is_valid_steering_.reserve(num_modules_);
}

PlannedMotion SwerveMotionControl::plan(
  const geometry_msgs::msg::Twist & desired_setpoint,
  const std::vector<double> & current_steering_angles,
  const std::vector<double> & current_wheel_velocities_rad_per_sec)
{
  TargetDrivingType target_mode = determine_target_mode(desired_setpoint);

  RCLCPP_DEBUG(
    rclcpp::get_logger("SwerveMotionControl"),
    "Planning motion with %zu modules, desired setpoint: vx=%.2f, vy=%.2f, wz=%.2f, target_mode=%d",
    num_modules_, desired_setpoint.linear.x, desired_setpoint.linear.y,
    desired_setpoint.angular.z, static_cast<int>(target_mode));

  if (target_mode == TargetDrivingType::STOPPED &&
    current_mode_ != TargetDrivingType::STOPPED &&
    current_mode_ != TargetDrivingType::RE_ALIGNING)
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger("SwerveMotionControl"),
      "[MODE SYNC] Transitioning to STOPPED mode from mode %d",
      static_cast<int>(current_mode_));
    current_mode_ = TargetDrivingType::STOPPED;
    mode_transition_count_ = 0;
    pending_target_mode_ = TargetDrivingType::STOPPED;
  }

  PlannedMotion realign_motion;
  if (handle_realignment_in_progress(
      current_steering_angles, current_wheel_velocities_rad_per_sec, realign_motion))
  {
    return realign_motion;
  }

  std::vector<double> desired_steering_angles(num_modules_);
  std::vector<double> desired_wheel_speeds_rad_per_sec(num_modules_);

  compute_inverse_kinematics(
    desired_setpoint, current_steering_angles, target_mode,
    desired_steering_angles, desired_wheel_speeds_rad_per_sec);

  PlannedMotion planned_motion;

  if (check_mode_transition_needed(target_mode, current_steering_angles, desired_steering_angles)) {
    planned_motion = generate_realignment_motion(
      target_mode, current_steering_angles, desired_steering_angles,
      current_wheel_velocities_rad_per_sec);
  } else {
    planned_motion = generate_continuous_motion(
      target_mode, current_steering_angles, desired_steering_angles,
      desired_wheel_speeds_rad_per_sec, current_wheel_velocities_rad_per_sec);
  }

  if (!is_trajectory_valid(planned_motion.trajectory)) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("SwerveMotionControl"),
      "Generated trajectory violates steering joint limits!");
  }

  last_planned_motion_ = planned_motion;
  return planned_motion;
}

bool SwerveMotionControl::handle_realignment_in_progress(
  const std::vector<double> & current_steering_angles,
  const std::vector<double> & current_wheel_velocities_rad_per_sec,
  PlannedMotion & motion_out)
{
  if (current_mode_ != TargetDrivingType::RE_ALIGNING) {
    return false;
  }

  double elapsed_sec = (steady_clock_.now() - realignment_start_time_).seconds();
  if (elapsed_sec > REALIGNMENT_TIMEOUT_SEC) {
    realignment_retry_count_++;

    if (realignment_retry_count_ >= MAX_REALIGNMENT_RETRIES) {
      RCLCPP_WARN(
        rclcpp::get_logger("SwerveMotionControl"),
        "[WATCHDOG] RE_ALIGNMENT timeout (%.1fs > %.1fs), retry %d/%d exceeded. "
        "Force recovery to target mode: %d",
        elapsed_sec, REALIGNMENT_TIMEOUT_SEC,
        realignment_retry_count_, MAX_REALIGNMENT_RETRIES,
        static_cast<int>(target_mode_after_realignment_));

      current_mode_ = target_mode_after_realignment_;
      realignment_retry_count_ = 0;
      realignment_watchdog_triggered_ = true;
      mode_transition_count_ = 0;
      return false;
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger("SwerveMotionControl"),
        "[WATCHDOG] RE_ALIGNMENT timeout (%.1fs), retry %d/%d. Resetting timer.",
        elapsed_sec, realignment_retry_count_, MAX_REALIGNMENT_RETRIES);
      realignment_start_time_ = steady_clock_.now();
    }
  }

  bool all_aligned = true;
  for (size_t i = 0; i < num_modules_; ++i) {
    double error =
      std::abs(normalize_angle(current_steering_angles[i] - realignment_target_angles_[i]));
    if (error > realigning_angle_threshold_) {
      all_aligned = false;
      RCLCPP_DEBUG(
        rclcpp::get_logger("SwerveMotionControl"),
        "Steering of module %zu not aligned: error=%.2f deg",
        i, error * 180.0 / M_PI);
    }
  }

  if (all_aligned) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("SwerveMotionControl"),
      "Re-alignment complete. Transitioning to target mode: %d",
      static_cast<int>(target_mode_after_realignment_));
    current_mode_ = target_mode_after_realignment_;
    mode_transition_count_ = 0;
    pending_target_mode_ = target_mode_after_realignment_;
    realignment_retry_count_ = 0;
    return false;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("SwerveMotionControl"),
    "Currently re-aligning, waiting for completion...");

  motion_out.type = RobotState::DISCONTINUOUS_RE_ALIGNMENT;

  std::vector<double> zero_drive_speeds(num_modules_, 0.0);
  double T_sync = 0.0;
  for (size_t i = 0; i < num_modules_; ++i) {
    TrapezoidalProfile steer_prof(
      current_steering_angles[i], realignment_target_angles_[i],
      steering_max_vel_, steering_max_accel_);
    if (steer_prof.isValid()) {
      T_sync = std::max(T_sync, steer_prof.total_time);
    }
  }

  T_sync = std::max(T_sync, 0.05);

  std::vector<TrapezoidalProfile> steering_profiles;
  for (size_t i = 0; i < num_modules_; ++i) {
    steering_profiles.emplace_back(
      current_steering_angles[i], realignment_target_angles_[i],
      steering_max_vel_, steering_max_accel_, T_sync);
  }

  motion_out.steering_peak_velocities.resize(num_modules_);
  motion_out.steering_accelerations.resize(num_modules_);
  for (size_t i = 0; i < num_modules_; ++i) {
    double peak_vel = steering_profiles[i].peak_velocity;
    double accel = steering_profiles[i].acceleration;
    motion_out.steering_peak_velocities[i] = (peak_vel > 1e-6) ? peak_vel : steering_max_vel_;
    motion_out.steering_accelerations[i] = (accel > 1e-6) ? accel : steering_max_accel_;
  }

  motion_out.trajectory = generate_trajectory_message(
    steering_profiles, zero_drive_speeds, T_sync, current_wheel_velocities_rad_per_sec);

  last_planned_motion_ = motion_out;
  return true;
}

void SwerveMotionControl::compute_inverse_kinematics(
  const geometry_msgs::msg::Twist & desired_setpoint,
  const std::vector<double> & current_steering_angles,
  TargetDrivingType target_mode,
  std::vector<double> & desired_steering_angles,
  std::vector<double> & desired_wheel_speeds_rad_per_sec)
{
  for (size_t i = 0; i < num_modules_; ++i) {
    const double pivot_vel_x =
      desired_setpoint.linear.x - desired_setpoint.angular.z * module_positions_[i].y;
    const double pivot_vel_y =
      desired_setpoint.linear.y + desired_setpoint.angular.z * module_positions_[i].x;

    double refined_steering_angle_robot = std::atan2(pivot_vel_y, pivot_vel_x + 1e-9);

    if (non_coaxial_ik_iterations_ > 0) {
      for (int j = 0; j < non_coaxial_ik_iterations_; ++j) {
        const double v_offset_x = -desired_setpoint.angular.z * steering_to_wheel_offset_[i] *
          std::cos(refined_steering_angle_robot);
        const double v_offset_y = desired_setpoint.angular.z * steering_to_wheel_offset_[i] *
          std::sin(refined_steering_angle_robot);
        const double contact_vel_x = pivot_vel_x + v_offset_x;
        const double contact_vel_y = pivot_vel_y + v_offset_y;
        refined_steering_angle_robot = std::atan2(contact_vel_y, contact_vel_x + 1e-9);
      }
    }

    const double final_contact_vel_x = pivot_vel_x +
      (-desired_setpoint.angular.z * steering_to_wheel_offset_[i] *
      std::cos(refined_steering_angle_robot));
    const double final_contact_vel_y = pivot_vel_y +
      (desired_setpoint.angular.z * steering_to_wheel_offset_[i] *
      std::sin(refined_steering_angle_robot));

    const double target_wheel_speed_mps = std::sqrt(
      final_contact_vel_x * final_contact_vel_x + final_contact_vel_y * final_contact_vel_y);
    double primary_angle_joint =
      normalize_angle(refined_steering_angle_robot - module_angle_offsets_[i]);
    double primary_wheel_speed = (target_wheel_speed_mps / wheel_radius_);

    if (target_mode == TargetDrivingType::STOPPED) {
      primary_angle_joint = current_steering_angles[i];
      primary_wheel_speed = 0.0;
    }

    double flipped_angle_joint = normalize_angle(primary_angle_joint + M_PI);
    double flipped_wheel_speed = -primary_wheel_speed;

    bool is_primary_valid = is_angle_within_limits(
      primary_angle_joint, steering_limits_lower_[i], steering_limits_upper_[i]);
    bool is_flipped_valid = is_angle_within_limits(
      flipped_angle_joint, steering_limits_lower_[i], steering_limits_upper_[i]);

    if (is_primary_valid && is_flipped_valid) {
      double primary_diff =
        std::abs(normalize_angle(primary_angle_joint - current_steering_angles[i]));
      double flipped_diff =
        std::abs(normalize_angle(flipped_angle_joint - current_steering_angles[i]));

      if (primary_diff <= flipped_diff) {
        desired_steering_angles[i] = primary_angle_joint;
        desired_wheel_speeds_rad_per_sec[i] = primary_wheel_speed;
        if (i < is_valid_steering_.size() && !is_valid_steering_[i]) {
          desired_steering_angles[i] = flipped_angle_joint;
          desired_wheel_speeds_rad_per_sec[i] = flipped_wheel_speed;
        }
      } else {
        desired_steering_angles[i] = flipped_angle_joint;
        desired_wheel_speeds_rad_per_sec[i] = flipped_wheel_speed;
        if (i < is_valid_steering_.size() && !is_valid_steering_[i]) {
          desired_steering_angles[i] = primary_angle_joint;
          desired_wheel_speeds_rad_per_sec[i] = primary_wheel_speed;
        }
      }
    } else if (is_primary_valid) {
      desired_steering_angles[i] = primary_angle_joint;
      desired_wheel_speeds_rad_per_sec[i] = primary_wheel_speed;
    } else if (is_flipped_valid) {
      desired_steering_angles[i] = flipped_angle_joint;
      desired_wheel_speeds_rad_per_sec[i] = flipped_wheel_speed;
    } else {
      double lower_diff = std::abs(
        normalize_angle(primary_angle_joint - steering_limits_lower_[i]));
      double upper_diff = std::abs(
        normalize_angle(primary_angle_joint - steering_limits_upper_[i]));

      constexpr double ANGLE_CLAMP_EPSILON = 1e-6;
      double clamped_angle_joint = (lower_diff <= upper_diff) ?
        steering_limits_lower_[i] + ANGLE_CLAMP_EPSILON :
        steering_limits_upper_[i] - ANGLE_CLAMP_EPSILON;

      desired_steering_angles[i] = clamped_angle_joint;

      double clamped_angle_robot = clamped_angle_joint + module_angle_offsets_[i];
      double delta_angle = normalize_angle(primary_angle_joint - clamped_angle_robot);
      double projected_speed_mps = target_wheel_speed_mps * std::cos(delta_angle);
      desired_wheel_speeds_rad_per_sec[i] = projected_speed_mps / wheel_radius_;
    }
  }
}

bool SwerveMotionControl::check_mode_transition_needed(
  TargetDrivingType target_mode,
  const std::vector<double> & current_steering_angles,
  const std::vector<double> & desired_steering_angles)
{
  if (current_mode_ == TargetDrivingType::RE_ALIGNING) {
    return true;
  }

  bool transition_candidate = false;

  if (needs_realignment_matrix_.count(current_mode_) &&
    needs_realignment_matrix_.at(current_mode_).count(target_mode))
  {
    transition_candidate = needs_realignment_matrix_.at(current_mode_).at(target_mode);
  }

  if (!transition_candidate && current_mode_ != target_mode) {
    double max_required_angle_change = 0.0;
    for (size_t i = 0; i < num_modules_; ++i) {
      double angle_change =
        std::abs(normalize_angle(desired_steering_angles[i] - current_steering_angles[i]));
      if (angle_change > max_required_angle_change) {
        max_required_angle_change = angle_change;
      }
    }

    if (max_required_angle_change > discontinuous_motion_steering_tolerance_) {
      transition_candidate = true;
    }
  }

  if (transition_candidate) {
    auto is_same_mode_group = [](TargetDrivingType a, TargetDrivingType b) -> bool {
        auto is_driving_mode = [](TargetDrivingType m) {
            return m == TargetDrivingType::LINEAR_DRIVING ||
                   m == TargetDrivingType::L_TURNING ||
                   m == TargetDrivingType::R_TURNING ||
                   m == TargetDrivingType::DIAGONAL_DRIVING;
          };
        if (is_driving_mode(a) && is_driving_mode(b)) {
          return true;
        }
        return a == b;
      };

    if (is_same_mode_group(target_mode, pending_target_mode_)) {
      mode_transition_count_++;
      pending_target_mode_ = target_mode;
    } else {
      pending_target_mode_ = target_mode;
      mode_transition_count_ = 1;
    }

    if (mode_transition_count_ >= MODE_TRANSITION_THRESHOLD) {
      mode_transition_count_ = 0;
      return true;
    }
  } else {
    if (mode_transition_count_ > 0) {
      mode_transition_count_--;
    }
  }

  return false;
}

PlannedMotion SwerveMotionControl::generate_realignment_motion(
  TargetDrivingType target_mode,
  const std::vector<double> & current_steering_angles,
  const std::vector<double> & desired_steering_angles,
  const std::vector<double> & current_wheel_velocities_rad_per_sec)
{
  PlannedMotion planned_motion;
  planned_motion.type = RobotState::DISCONTINUOUS_RE_ALIGNMENT;

  RCLCPP_DEBUG(
    rclcpp::get_logger("SwerveMotionControl"),
    "Mode change required! From %d to %d. Starting re-alignment.",
    static_cast<int>(current_mode_), static_cast<int>(target_mode));

  current_mode_ = TargetDrivingType::RE_ALIGNING;
  target_mode_after_realignment_ = target_mode;
  realignment_target_angles_ = desired_steering_angles;

  realignment_start_time_ = steady_clock_.now();
  realignment_watchdog_triggered_ = false;

  std::vector<double> zero_drive_speeds(num_modules_, 0.0);
  double T_sync = 0.0;
  for (size_t i = 0; i < num_modules_; ++i) {
    TrapezoidalProfile steer_prof(
      current_steering_angles[i], desired_steering_angles[i],
      steering_max_vel_, steering_max_accel_);
    if (steer_prof.isValid()) {
      T_sync = std::max(T_sync, steer_prof.total_time);
    } else {
      PlannedMotion empty_motion;
      empty_motion.type = RobotState::IDLE;
      return empty_motion;
    }
  }

  std::vector<TrapezoidalProfile> steering_profiles;
  for (size_t i = 0; i < num_modules_; ++i) {
    steering_profiles.emplace_back(
      current_steering_angles[i], desired_steering_angles[i],
      steering_max_vel_, steering_max_accel_, T_sync);
  }

  planned_motion.steering_peak_velocities.resize(num_modules_);
  planned_motion.steering_accelerations.resize(num_modules_);
  for (size_t i = 0; i < num_modules_; ++i) {
    double peak_vel = steering_profiles[i].peak_velocity;
    double accel = steering_profiles[i].acceleration;
    planned_motion.steering_peak_velocities[i] = (peak_vel > 1e-6) ? peak_vel : steering_max_vel_;
    planned_motion.steering_accelerations[i] = (accel > 1e-6) ? accel : steering_max_accel_;
  }

  planned_motion.trajectory = generate_trajectory_message(
    steering_profiles, zero_drive_speeds, T_sync, current_wheel_velocities_rad_per_sec);

  return planned_motion;
}

PlannedMotion SwerveMotionControl::generate_continuous_motion(
  TargetDrivingType target_mode,
  const std::vector<double> & current_steering_angles,
  std::vector<double> & desired_steering_angles,
  std::vector<double> & desired_wheel_speeds_rad_per_sec,
  const std::vector<double> & current_wheel_velocities_rad_per_sec)
{
  PlannedMotion planned_motion;
  planned_motion.type = RobotState::CONTINUOUS;

  bool use_current_steering = false;
  if (mode_transition_count_ == 0) {
    current_mode_ = target_mode;
  } else {
    use_current_steering = true;
  }

  if (use_current_steering) {
    for (size_t i = 0; i < num_modules_; ++i) {
      desired_steering_angles[i] = current_steering_angles[i];
      desired_wheel_speeds_rad_per_sec[i] = 0.0;
    }
  }

  double T_sync = 0.0;
  if (current_mode_ == TargetDrivingType::STOPPED) {
    for (size_t i = 0; i < num_modules_; ++i) {
      double time_needed = std::abs(current_wheel_velocities_rad_per_sec[i]) / drive_max_accel_;
      T_sync = std::max(T_sync, time_needed);
      desired_wheel_speeds_rad_per_sec[i] = 0.0;
      if (current_wheel_velocities_rad_per_sec[i] < velocity_deadband_) {
        T_sync = std::max(T_sync, 0.1);
      }
    }
  } else {
    double T_sync_steering = 0.0;
    for (size_t i = 0; i < num_modules_; ++i) {
      TrapezoidalProfile steer_prof(
        current_steering_angles[i], desired_steering_angles[i],
        steering_max_vel_, steering_max_accel_);
      if (steer_prof.isValid()) {
        T_sync_steering = std::max(T_sync_steering, steer_prof.total_time);
      }
    }
    double T_sync_drive = 0.0;
    for (size_t i = 0; i < num_modules_; ++i) {
      double vel_change = std::abs(
        desired_wheel_speeds_rad_per_sec[i] - current_wheel_velocities_rad_per_sec[i]);
      double time_needed = vel_change / drive_max_accel_;
      T_sync_drive = std::max(T_sync_drive, time_needed);
    }
    T_sync = std::max(T_sync_steering, T_sync_drive);
  }

  constexpr double MIN_PROFILE_TIME = 0.02;
  bool use_immediate_target = (T_sync < MIN_PROFILE_TIME);

  std::vector<TrapezoidalProfile> steering_profiles;
  for (size_t i = 0; i < num_modules_; ++i) {
    if (use_immediate_target) {
      steering_profiles.emplace_back(
        desired_steering_angles[i], desired_steering_angles[i],
        steering_max_vel_, steering_max_accel_);
    } else {
      double distance = std::abs(desired_steering_angles[i] - current_steering_angles[i]);
      double min_physical_time = (distance > 1e-9) ?
        std::sqrt(4.0 * distance / steering_max_accel_) : 0.0;

      if (T_sync < min_physical_time) {
        steering_profiles.emplace_back(
          desired_steering_angles[i], desired_steering_angles[i],
          steering_max_vel_, steering_max_accel_);
      } else {
        steering_profiles.emplace_back(
          current_steering_angles[i], desired_steering_angles[i],
          steering_max_vel_, steering_max_accel_, T_sync);
      }
    }
  }

  if (use_immediate_target) {
    T_sync = MIN_PROFILE_TIME;
  }

  planned_motion.steering_peak_velocities.resize(num_modules_);
  planned_motion.steering_accelerations.resize(num_modules_);
  for (size_t i = 0; i < num_modules_; ++i) {
    double peak_vel = steering_profiles[i].peak_velocity;
    double accel = steering_profiles[i].acceleration;
    planned_motion.steering_peak_velocities[i] = (peak_vel > 1e-6) ? peak_vel : steering_max_vel_;
    planned_motion.steering_accelerations[i] = (accel > 1e-6) ? accel : steering_max_accel_;
  }


  planned_motion.trajectory = generate_trajectory_message(
    steering_profiles, desired_wheel_speeds_rad_per_sec, T_sync,
    current_wheel_velocities_rad_per_sec);

  return planned_motion;
}

TargetDrivingType SwerveMotionControl::determine_target_mode(
  const geometry_msgs::msg::Twist & setpoint) const
{
  if (std::abs(setpoint.linear.x) < velocity_deadband_ &&
    std::abs(setpoint.linear.y) < velocity_deadband_ &&
    std::abs(setpoint.angular.z) < velocity_deadband_)
  {
    return TargetDrivingType::STOPPED;
  }

  if (std::abs(setpoint.linear.x) < velocity_deadband_ &&
    std::abs(setpoint.linear.y) < velocity_deadband_ &&
    std::abs(setpoint.angular.z) >= velocity_deadband_)
  {
    return TargetDrivingType::SPINNING;
  }

  if (std::abs(setpoint.linear.x) >= velocity_deadband_ &&
    std::abs(setpoint.linear.y) >= velocity_deadband_ &&
    std::abs(setpoint.angular.z) < velocity_deadband_)
  {
    return TargetDrivingType::DIAGONAL_DRIVING;
  }

  if (setpoint.angular.z <= -velocity_deadband_ &&
    std::abs(setpoint.linear.x) > velocity_deadband_ &&
    std::abs(setpoint.linear.y) < velocity_deadband_)
  {
    return TargetDrivingType::R_TURNING;
  }

  if (setpoint.angular.z >= velocity_deadband_ &&
    std::abs(setpoint.linear.x) > velocity_deadband_ &&
    std::abs(setpoint.linear.y) < velocity_deadband_)
  {
    return TargetDrivingType::L_TURNING;
  }

  return TargetDrivingType::LINEAR_DRIVING;
}

void SwerveMotionControl::initialize_transition_matrix()
{
  for (const auto & from : modes) {
    for (const auto & to : modes) {
      needs_realignment_matrix_[from][to] = false;
    }
  }

  needs_realignment_matrix_[TargetDrivingType::STOPPED][TargetDrivingType::LINEAR_DRIVING] = true;
  needs_realignment_matrix_[TargetDrivingType::STOPPED][TargetDrivingType::L_TURNING] = true;
  needs_realignment_matrix_[TargetDrivingType::STOPPED][TargetDrivingType::R_TURNING] = true;
  needs_realignment_matrix_[TargetDrivingType::STOPPED][TargetDrivingType::DIAGONAL_DRIVING] =
    true;
  needs_realignment_matrix_[TargetDrivingType::STOPPED][TargetDrivingType::SPINNING] = true;

  for (const auto & mode : modes) {
    if (mode != TargetDrivingType::SPINNING) {
      needs_realignment_matrix_[mode][TargetDrivingType::SPINNING] = true;
      needs_realignment_matrix_[TargetDrivingType::SPINNING][mode] = true;
    }
  }
  needs_realignment_matrix_[TargetDrivingType::SPINNING][TargetDrivingType::STOPPED] = true;

  std::vector<TargetDrivingType> translational_modes = {
    TargetDrivingType::DIAGONAL_DRIVING
  };
  std::vector<TargetDrivingType> turning_modes = {
    TargetDrivingType::L_TURNING, TargetDrivingType::R_TURNING
  };

  for (const auto & from : translational_modes) {
    for (const auto & to : turning_modes) {
      needs_realignment_matrix_[from][to] = false;
      needs_realignment_matrix_[to][from] = true;
    }
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("SwerveMotionControl"),
    "Robust transition matrix initialized.");
}

bool SwerveMotionControl::is_angle_within_limits(
  double angle, double lower_limit, double upper_limit)
{
  double norm_angle = normalize_angle(angle);
  if (lower_limit <= upper_limit) {
    return norm_angle >= lower_limit && norm_angle <= upper_limit;
  } else {
    return norm_angle >= lower_limit || norm_angle <= upper_limit;
  }
}

bool SwerveMotionControl::is_trajectory_valid(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.points.empty()) {
    return true;
  }

  is_valid_steering_.clear();

  bool is_trajectory_all_valid = true;

  for (size_t i = 0; i < num_modules_; ++i) {
    bool is_module_trajectory_valid = true;

    for (const auto & point : trajectory.points) {
      if (i >= point.positions.size()) {
        continue;
      }
      const double steering_angle = point.positions[i];
      const double lower_limit = steering_limits_lower_[i];
      const double upper_limit = steering_limits_upper_[i];

      if (!is_angle_within_limits(steering_angle, lower_limit, upper_limit)) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("SwerveMotionControl"),
          "Module %zu steering angle %.2f [deg] exceeds joint limits [%.2f, %.2f] "
          "at trajectory point",
          i, steering_angle * 180.0 / M_PI, lower_limit * 180.0 / M_PI, upper_limit * 180.0 / M_PI);
        is_module_trajectory_valid = false;
        is_trajectory_all_valid = false;
      }
    }

    is_valid_steering_.push_back(is_module_trajectory_valid);
  }

  return is_trajectory_all_valid;
}

double SwerveMotionControl::normalize_angle(double angle_rad)
{
  double remainder = std::fmod(angle_rad + M_PI, 2.0 * M_PI);
  if (remainder < 0.0) {
    remainder += 2.0 * M_PI;
  }
  return remainder - M_PI;
}

double SwerveMotionControl::normalize_angle_positive(double angle)
{
  return std::fmod(std::fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

double SwerveMotionControl::shortest_angular_distance(double from, double to)
{
  double result = normalize_angle_positive(to) - normalize_angle_positive(from);
  if (result > M_PI) {
    result -= 2.0 * M_PI;
  } else if (result < -M_PI) {
    result += 2.0 * M_PI;
  }
  return result;
}

trajectory_msgs::msg::JointTrajectory SwerveMotionControl::generate_trajectory_message(
  const std::vector<TrapezoidalProfile> & steering_profiles,
  const std::vector<double> & target_drive_speeds,
  double T_sync,
  const std::vector<double> & initial_drive_speeds
)
{
  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.joint_names = joint_names_;

  const int NUM_SAMPLES = 50;
  double dt = (NUM_SAMPLES > 0 && T_sync > 1e-6) ? T_sync / NUM_SAMPLES : 0.0;

  std::vector<double> current_drive_velocities = initial_drive_speeds;
  std::vector<double> current_drive_accelerations(num_modules_, 0.0);

  for (int i = 0; i <= NUM_SAMPLES; ++i) {
    double t = dt * i;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(t);

    for (size_t j = 0; j < num_modules_; ++j) {
      if (j < steering_profiles.size()) {
        point.positions.push_back(steering_profiles[j].getPositionAt(t));
        point.velocities.push_back(steering_profiles[j].getVelocityAt(t));
        point.accelerations.push_back(steering_profiles[j].getAccelerationAt(t));
      } else {
        point.positions.push_back(0.0);
        point.velocities.push_back(0.0);
        point.accelerations.push_back(0.0);
      }
    }

    if (i > 0) {
      for (size_t j = 0; j < num_modules_; ++j) {
        if (j >= target_drive_speeds.size() || j >= current_drive_velocities.size()) {
          continue;
        }
        double vel_error = target_drive_speeds[j] - current_drive_velocities[j];
        double max_accel_step = drive_max_accel_ * dt;

        if (std::abs(vel_error) <= max_accel_step) {
          current_drive_velocities[j] = target_drive_speeds[j];
          current_drive_accelerations[j] = vel_error / dt;
        } else {
          double accel_direction = (vel_error > 0) ? 1.0 : -1.0;
          current_drive_velocities[j] += accel_direction * max_accel_step;
          current_drive_accelerations[j] = accel_direction * drive_max_accel_;
        }
      }
    }

    for (size_t j = 0; j < num_modules_; ++j) {
      if (j < current_drive_velocities.size()) {
        point.positions.push_back(current_drive_velocities[j]);
        point.velocities.push_back(current_drive_accelerations[j]);
        point.accelerations.push_back(0.0);
      } else {
        point.positions.push_back(0.0);
        point.velocities.push_back(0.0);
        point.accelerations.push_back(0.0);
      }
    }

    trajectory_msg.points.push_back(point);
  }

  return trajectory_msg;
}

}  // namespace swerve_drive_controller
}  // namespace antbot
