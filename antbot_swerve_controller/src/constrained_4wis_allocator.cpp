// Copyright 2026 ANTBot
#include "antbot_swerve_controller/constrained_4wis_allocator.hpp"

#include <algorithm>
#include <cmath>

namespace antbot_swerve_controller
{

double Constrained4WisAllocator::wrapPi(double a)
{
  // Matches the reference model: (a + pi) mod 2pi - pi, in (-pi, pi].
  double m = std::fmod(a + M_PI, 2.0 * M_PI);
  if (m < 0.0) {m += 2.0 * M_PI;}
  return m - M_PI;
}

Constrained4WisAllocator::Constrained4WisAllocator(
  const std::array<ModuleGeometry, 4> & modules, const AllocatorConfig & config)
: modules_(modules), config_(config)
{
}

AllocatorResult Constrained4WisAllocator::update(
  const std::array<double, 4> & steering,
  double vx, double vy, double w, double dt) const
{
  const auto & cfg = config_;
  const double d_lo = cfg.steer_min + cfg.soft_margin;
  const double d_hi = cfg.steer_max - cfg.soft_margin;

  AllocatorResult r;
  r.feasible = true;

  std::array<double, 4> branch{};

  // --- 1) per-module target steering angle + flip-aware feasibility ---
  for (int i = 0; i < 4; ++i) {
    const double x_i = modules_[i].x;
    const double y_i = modules_[i].y;
    const double v_ax = vx - w * y_i;
    const double v_ay = vy + w * x_i;

    double br;
    int sign;
    bool feas_i;

    if (std::hypot(v_ax, v_ay) < cfg.low_speed_eps) {
      // module barely moves -> hold current angle (avoid atan2(0,0))
      br = steering[i];
      sign = 1;
      feas_i = true;
    } else {
      const double beta = std::atan2(v_ay, v_ax);
      const double cand0 = wrapPi(beta);
      const double cand1 = wrapPi(beta + M_PI);
      const bool valid0 = (cand0 >= d_lo) && (cand0 <= d_hi);
      const bool valid1 = (cand1 >= d_lo) && (cand1 <= d_hi);
      const double d0 = std::abs(wrapPi(cand0 - steering[i]));
      const double d1 = std::abs(wrapPi(cand1 - steering[i]));

      if (valid0 && valid1) {
        if (d0 <= d1) {br = cand0; sign = 1;} else {br = cand1; sign = -1;}
        feas_i = true;
      } else if (valid0) {
        br = cand0; sign = 1; feas_i = true;
      } else if (valid1) {
        br = cand1; sign = -1; feas_i = true;
      } else {
        // direction infeasible for this module; keep nearest candidate (clamped).
        feas_i = false;
        if (d0 <= d1) {br = cand0; sign = 1;} else {br = cand1; sign = -1;}
      }
    }

    branch[i] = br;
    r.drive_sign[i] = sign;
    if (!feas_i) {r.feasible = false;}
    r.target[i] = std::clamp(br, d_lo, d_hi);
    r.clamp_error[i] = wrapPi(r.target[i] - br);
  }

  // --- 2) steering velocity (rate-limited P) + position command (clamped) ---
  for (int i = 0; i < 4; ++i) {
    const double err = wrapPi(r.target[i] - steering[i]);
    const double nu = std::clamp(cfg.steer_gain * err, -cfg.steer_rate_limit, cfg.steer_rate_limit);
    r.steer_velocity[i] = nu;
    r.steer_position[i] = std::clamp(steering[i] + nu * dt, d_lo, d_hi);
  }

  // --- 3) wheel velocity (no alpha; lateral-offset scrub baked in) + clamp ---
  for (int i = 0; i < 4; ++i) {
    const double x_i = modules_[i].x;
    const double y_i = modules_[i].y;
    const double c_i = modules_[i].c;
    const double v_ax = vx - w * y_i;
    const double v_ay = vy + w * x_i;
    const double raw =
      (std::cos(r.steer_position[i]) * v_ax + std::sin(r.steer_position[i]) * v_ay -
      c_i * (w + r.steer_velocity[i])) / cfg.wheel_radius;
    r.wheel_velocity_raw[i] = raw;
    const double clamped = std::clamp(raw, -cfg.wheel_speed_limit, cfg.wheel_speed_limit);
    r.wheel_velocity[i] = clamped;
    if (clamped != raw) {r.wheel_clamp_active = true;}
  }

  return r;
}

}  // namespace antbot_swerve_controller
