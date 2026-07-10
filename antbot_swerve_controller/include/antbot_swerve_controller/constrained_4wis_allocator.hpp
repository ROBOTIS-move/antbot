// Copyright 2026 ANTBot
//
// Constrained 4WIS-4WID per-tick allocator (pure C++, ROS-independent).
//
// One continuous closed-form control law replaces the old mode/FSM + stop-and-
// realign planner.  Per module i, for a body twist reference u = [vx, vy, w]:
//
//   v_A,i   = [vx - w*y_i, vy + w*x_i]                (steering-axis velocity)
//   beta_i  = atan2(v_Ay, v_Ax) ,  candidates {beta, beta+-pi}  (flip)
//   d*_i    = nearest limit-valid candidate (else nearest, clamped)
//   nu_i    = sat(kp * wrap(d*_i - d_i), +-nu_max)    (steering velocity cmd)
//   d_cmd_i = clamp(d_i + nu_i*dt, d_lo, d_hi)        (steering position cmd)
//   phi_dot_i = ( u(d_cmd_i) . v_A,i - c_i*(w + nu_i) ) / r      (wheel velocity)
//
// ALPHA IS REMOVED (per the antbot continuous-3dof plan, v1): there is no
// feasibility gate / alignment slow-down / common wheel-speed scale.  Steering
// is rate-limited and clamped; wheel speed is clamped per wheel (best-effort).
// Feasibility is guaranteed upstream (navigation); this class reports a
// `feasible` flag and clamp diagnostics so that contract can be monitored.
//
// Reference (executable spec, minus alpha):
//   colcon_ws/src/gaemi_simulations/scripts/g50_constrained_4wis.py
//   Constrained4WISController.update()  (module order FL, FR, RL, RR)

#ifndef ANTBOT_SWERVE_CONTROLLER__CONSTRAINED_4WIS_ALLOCATOR_HPP_
#define ANTBOT_SWERVE_CONTROLLER__CONSTRAINED_4WIS_ALLOCATOR_HPP_

#include <array>

namespace antbot_swerve_controller
{

/// Steering-axis geometry of one module (base_link frame).
struct ModuleGeometry
{
  double x{0.0};  ///< steering-axis x [m]
  double y{0.0};  ///< steering-axis y [m]
  double c{0.0};  ///< signed lateral offset steering-axis -> wheel (left +, right -) [m]
};

/// Allocator tuning + limits.
struct AllocatorConfig
{
  double steer_min{-0.9599};       ///< hardware min steering angle [rad] (~-55 deg)
  double steer_max{0.9599};        ///< hardware max steering angle [rad] (~+55 deg)
  double soft_margin{0.0};         ///< feasibility/command soft margin [rad]
  double steer_gain{4.0};          ///< kp for steering velocity [1/s]
  double steer_rate_limit{3.0};    ///< |nu| max [rad/s]
  double wheel_speed_limit{20.0};  ///< per-wheel |phi_dot| max [rad/s]
  double wheel_radius{0.103};      ///< r [m]
  double low_speed_eps{1e-3};      ///< |v_A| below which a module holds its angle
};

/// One control step's commands + diagnostics.
struct AllocatorResult
{
  std::array<double, 4> steer_position{};   ///< d_cmd [rad], clamped
  std::array<double, 4> steer_velocity{};   ///< nu [rad/s]
  std::array<double, 4> wheel_velocity{};   ///< phi_dot [rad/s], clamped
  std::array<double, 4> wheel_velocity_raw{};  ///< phi_dot before per-wheel clamp
  std::array<double, 4> target{};           ///< chosen target steering angle [rad]
  std::array<double, 4> clamp_error{};      ///< wrap(target - selected branch) [rad]
  std::array<int, 4> drive_sign{};          ///< +1 primary / -1 flipped (diagnostic)
  bool feasible{true};                      ///< all modules had a limit-valid candidate
  bool wheel_clamp_active{false};           ///< any wheel speed was clamped
};

/// Stateless constrained 4WIS-4WID allocator.
class Constrained4WisAllocator
{
public:
  Constrained4WisAllocator() = default;
  Constrained4WisAllocator(
    const std::array<ModuleGeometry, 4> & modules, const AllocatorConfig & config);

  /// Compute per-module steering + wheel commands for a body twist reference.
  /// @param steering current measured steering angles [rad], order FL,FR,RL,RR
  AllocatorResult update(
    const std::array<double, 4> & steering,
    double vx, double vy, double w, double dt) const;

  const AllocatorConfig & config() const { return config_; }
  const std::array<ModuleGeometry, 4> & modules() const { return modules_; }

  /// Wrap an angle to (-pi, pi] using the reference model's convention.
  static double wrapPi(double a);

private:
  std::array<ModuleGeometry, 4> modules_{};
  AllocatorConfig config_{};
};

}  // namespace antbot_swerve_controller

#endif  // ANTBOT_SWERVE_CONTROLLER__CONSTRAINED_4WIS_ALLOCATOR_HPP_
