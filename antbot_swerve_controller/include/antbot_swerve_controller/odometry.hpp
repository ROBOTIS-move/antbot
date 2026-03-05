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

/*
 * Author: Geonhee Lee
 */

#ifndef ANTBOT_SWERVE_CONTROLLER__ODOMETRY_HPP_
#define ANTBOT_SWERVE_CONTROLLER__ODOMETRY_HPP_

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <array>
#include <string>

#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"

namespace antbot
{
namespace swerve_drive_controller
{
enum class OdomSolverMethod
{
  PSEUDO_INVERSE,
  QR_DECOMPOSITION,
  SVD
};

enum class OdomIntegrationMethod
{
  EULER_METHOD,
  RK2_METHOD,
  RK4_METHOD,
  ANALYTIC_SWERVE_METHOD
};

class Odometry
{
public:
  explicit Odometry(size_t velocity_rolling_window_size = 1);

  void init(
    const rclcpp::Time & time, const std::array<double, 3> & base_frame_offset = {0.0, 0.0,
      0.0});

  bool update(
    const std::vector<double> & steering_positions,
    const std::vector<double> & wheel_velocities,
    const double dt);
  bool update(double target_vx, double target_vy, double target_w, const double dt);

  double getX() const {return position_x_odom_;}
  double getY() const {return position_y_odom_;}
  double getYaw() const {return orientation_yaw_odom_;}
  double getVx() const {return velocity_in_base_frame_linear_x_;}
  double getVy() const {return velocity_in_base_frame_linear_y_;}
  double getWz() const {return velocity_in_base_frame_angular_z_;}

  void setModuleParams(
    const std::vector<double> & module_x_offsets,
    const std::vector<double> & module_y_offsets,
    const std::vector<double> & steering_to_wheel_offsets,
    const double wheel_radius);

  void setSolverMethod(OdomSolverMethod method);
  void setIntegrationMethod(OdomIntegrationMethod method);

  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);
  void resetAccumulators();
  Eigen::Vector3d updateFromVelocity(double linear_x, double linear_y, double angular_z);


  void integratePose(double v_x_body, double v_y_body, double omega_body, double dt_sec);

  // Integrate methods
  void integrateEuler(double v_x_body, double v_y_body, double omega_body, double dt_sec);
  void integrateRK2(double v_x_body, double v_y_body, double omega_body, double dt_sec);
  void integrateRK4(double v_x_body, double v_y_body, double omega_body, double dt_sec);
  void integrateAnalyticSwerve(double v_x_body, double v_y_body, double omega_body, double dt_sec);
  struct StateDerivative
  {
    double dx_dt_world;
    double dy_dt_world;
    double dtheta_dt;
  };
  StateDerivative calculateWorldDerivatives(
    double current_heading_world, double v_x_body,
    double v_y_body, double omega_body);

private:
// \note The versions conditioning is added here to support the source-compatibility with Humble
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

  rclcpp::Time timestamp_;
  std::array<double, 3> base_frame_offset_;

  double position_x_odom_;
  double position_y_odom_;
  double orientation_yaw_odom_;

  double velocity_in_base_frame_linear_x_;
  double velocity_in_base_frame_linear_y_;
  double velocity_in_base_frame_angular_z_;

  size_t num_modules_ = 0;
  std::vector<double> module_x_offsets_;
  std::vector<double> module_y_offsets_;
  std::vector<double> steering_to_wheel_offsets_;
  double wheel_radius_ = 0.0;

  OdomSolverMethod solver_method_;
  OdomIntegrationMethod integration_method_name_;

  static double normalize_angle(double angle_rad);

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_x_accumulator_;
  RollingMeanAccumulator linear_y_accumulator_;
  RollingMeanAccumulator angular_z_accumulator_;
};
}  // namespace swerve_drive_controller
}  // namespace antbot
#endif  // ANTBOT_SWERVE_CONTROLLER__ODOMETRY_HPP_
