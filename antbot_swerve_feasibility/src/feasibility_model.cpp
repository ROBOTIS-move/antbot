// Copyright 2026 ANTBot
#include "antbot_swerve_feasibility/feasibility_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace antbot_swerve_feasibility
{

FeasibilityModel::FeasibilityModel(
  const std::array<ModuleGeometry, 4> & modules,
  double steer_min, double steer_max, double margin)
: modules_(modules),
  steer_min_(steer_min),
  steer_max_(steer_max),
  soft_margin_(margin)
{
  const double d_min = steer_min_ + soft_margin_;
  const double d_max = steer_max_ - soft_margin_;
  gamma_ = 0.5 * (d_max - d_min);
  theta_ = 0.5 * (d_min + d_max);
  cos2_gamma_ = std::cos(gamma_) * std::cos(gamma_);
  q_ = Eigen::Vector2d(std::cos(theta_), std::sin(theta_));

  // Core2 = q q^T - cos^2(gamma) I2 ; M_i = J_i^T Core2 J_i, J_i = [[1,0,-y],[0,1,x]]
  const Eigen::Matrix2d core = q_ * q_.transpose() - cos2_gamma_ * Eigen::Matrix2d::Identity();
  for (int i = 0; i < 4; ++i) {
    Eigen::Matrix<double, 2, 3> J;
    J << 1.0, 0.0, -modules_[i].y,
      0.0, 1.0, modules_[i].x;
    M_[i] = J.transpose() * core * J;
  }

  // Symmetric rectangular layout detection (enables analytic helpers).
  const double eps = 1e-9;
  const double ax = std::abs(modules_[0].x);
  const double ay = std::abs(modules_[0].y);
  bool sym = std::abs(theta_) < eps;
  for (int i = 0; i < 4 && sym; ++i) {
    if (std::abs(std::abs(modules_[i].x) - ax) > eps) {sym = false;}
    if (std::abs(std::abs(modules_[i].y) - ay) > eps) {sym = false;}
  }
  symmetric_ = sym;
  sx_ = sym ? ax : 0.0;
  sy_ = sym ? ay : 0.0;
}

Eigen::Vector2d FeasibilityModel::moduleVelocity(int i, const Twist2D & u) const
{
  return Eigen::Vector2d(u.vx - u.w * modules_[i].y, u.vy + u.w * modules_[i].x);
}

double FeasibilityModel::moduleMargin(int i, const Twist2D & u) const
{
  const Eigen::Vector3d uu(u.vx, u.vy, u.w);
  return uu.dot(M_[i] * uu);
}

std::array<double, 4> FeasibilityModel::moduleMargins(const Twist2D & u) const
{
  std::array<double, 4> out{};
  for (int i = 0; i < 4; ++i) {out[i] = moduleMargin(i, u);}
  return out;
}

double FeasibilityModel::margin(const Twist2D & u) const
{
  double g = moduleMargin(0, u);
  for (int i = 1; i < 4; ++i) {g = std::min(g, moduleMargin(i, u));}
  return g;
}

double FeasibilityModel::moduleMarginNorm(int i, const Twist2D & u) const
{
  const Eigen::Vector2d v = moduleVelocity(i, u);
  const double speed2 = v.squaredNorm();
  return moduleMargin(i, u) / (speed2 + kSpeedFloor * kSpeedFloor);
}

double FeasibilityModel::marginNorm(const Twist2D & u) const
{
  double g = moduleMarginNorm(0, u);
  for (int i = 1; i < 4; ++i) {g = std::min(g, moduleMarginNorm(i, u));}
  return g;
}

double FeasibilityModel::smoothAlpha(const Twist2D & u, double gate_width) const
{
  const double s = std::clamp(marginNorm(u) / std::max(gate_width, 1e-12), 0.0, 1.0);
  return s * s * (3.0 - 2.0 * s);
}

double FeasibilityModel::radiusOfW(double w) const
{
  return std::abs(w) * (sx_ * std::cos(gamma_) + sy_ * std::sin(gamma_));
}

double FeasibilityModel::apexVx(double w) const
{
  const double s = std::sin(gamma_);
  if (std::abs(s) < 1e-12) {return 0.0;}
  return radiusOfW(w) / s;
}

double FeasibilityModel::feasibleAbsVy(double vx, double w) const
{
  const double cg = std::cos(gamma_);
  if (std::abs(cg) < 1e-12) {return -1.0;}
  return std::tan(gamma_) * std::abs(vx) - radiusOfW(w) / cg;
}

template<typename MakeTwist>
double FeasibilityModel::maxFeasibleScale(MakeTwist make) const
{
  double lo = 0.0;  // assumed feasible
  double hi = 1.0;  // assumed infeasible
  for (int it = 0; it < 50; ++it) {
    const double mid = 0.5 * (lo + hi);
    if (feasible(make(mid))) {lo = mid;} else {hi = mid;}
  }
  return lo;
}

ProjectResult FeasibilityModel::coneProject(const Twist2D & u, ProjectPolicy policy) const
{
  if (feasible(u)) {
    return ProjectResult{u, false, true, 1.0, 1.0};
  }

  auto tryReduceVy = [&](std::optional<ProjectResult> & out) {
    if (!feasible(Twist2D{u.vx, 0.0, u.w})) {return;}
    const double s = maxFeasibleScale([&](double t) {return Twist2D{u.vx, t * u.vy, u.w};});
    out = ProjectResult{Twist2D{u.vx, s * u.vy, u.w}, true, true, s, 1.0};
  };
  auto tryReduceW = [&](std::optional<ProjectResult> & out) {
    if (!feasible(Twist2D{u.vx, u.vy, 0.0})) {return;}
    const double s = maxFeasibleScale([&](double t) {return Twist2D{u.vx, u.vy, t * u.w};});
    out = ProjectResult{Twist2D{u.vx, u.vy, s * u.w}, true, true, 1.0, s};
  };
  auto tryZeroVyReduceW = [&](std::optional<ProjectResult> & out) {
    if (!feasible(Twist2D{u.vx, 0.0, 0.0})) {return;}
    const double s = maxFeasibleScale([&](double t) {return Twist2D{u.vx, 0.0, t * u.w};});
    out = ProjectResult{Twist2D{u.vx, 0.0, s * u.w}, true, true, 0.0, s};
  };

  std::optional<ProjectResult> vy_cand, w_cand;
  tryReduceVy(vy_cand);
  tryReduceW(w_cand);

  auto changeMag = [&](const ProjectResult & r) {
    const double dvy = u.vy - r.twist.vy;
    const double dw = u.w - r.twist.w;
    return dvy * dvy + dw * dw;
  };

  if (policy == ProjectPolicy::CrabPreferred) {
    if (vy_cand) {return *vy_cand;}
    if (w_cand) {return *w_cand;}
  } else if (policy == ProjectPolicy::ArcPreferred) {
    if (w_cand) {return *w_cand;}
    if (vy_cand) {return *vy_cand;}
  } else {  // MinChange
    if (vy_cand && w_cand) {
      return changeMag(*vy_cand) <= changeMag(*w_cand) ? *vy_cand : *w_cand;
    }
    if (vy_cand) {return *vy_cand;}
    if (w_cand) {return *w_cand;}
  }

  std::optional<ProjectResult> fwd_cand;
  tryZeroVyReduceW(fwd_cand);
  if (fwd_cand) {return *fwd_cand;}

  return ProjectResult{Twist2D{0.0, 0.0, 0.0}, true, true, 0.0, 0.0};  // stop
}

Twist2D FeasibilityModel::speedScale(const Twist2D & u, const ActuatorLimits & lim) const
{
  double max_v = 0.0;
  for (int i = 0; i < 4; ++i) {
    max_v = std::max(max_v, moduleVelocity(i, u).norm());
  }
  if (max_v < 1e-12) {return u;}
  const double allowed = lim.wheel_speed_max * lim.wheel_radius;
  const double s = std::min(1.0, allowed / max_v);
  return Twist2D{s * u.vx, s * u.vy, s * u.w};
}

std::optional<double> FeasibilityModel::feasibleOmega(
  double vx, double vy, double w_ref, double w_search_max, int steps) const
{
  if (feasible(Twist2D{vx, vy, w_ref})) {return w_ref;}
  std::optional<double> best;
  double best_d = std::numeric_limits<double>::infinity();
  for (int k = 0; k < steps; ++k) {
    const double w = -w_search_max + 2.0 * w_search_max * k / std::max(1, steps - 1);
    if (feasible(Twist2D{vx, vy, w}) && std::abs(w - w_ref) < best_d) {
      best = w;
      best_d = std::abs(w - w_ref);
    }
  }
  return best;
}

}  // namespace antbot_swerve_feasibility
