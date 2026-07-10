// Copyright 2026 ANTBot
//
// Constrained 4WIS-4WID steering-feasibility model (portable, ROS-independent).
//
// Given a body twist u = [vx, vy, w] and the per-module steering-axis geometry
// plus a symmetric steering-angle limit, this answers: "can all four modules
// steer to realize this twist (primary or +-pi flip) within their angle
// limits?"  The core is the quadratic-form reduction of the reference model:
//
//   v_A,i = [vx - w*y_i, vy + w*x_i]              (module contact velocity)
//   g_i(u) = (q . v_A,i)^2 - cos^2(gamma) ||v_A,i||^2 = u^T M_i u
//   M_i    = J_i^T (q q^T - cos^2(gamma) I_2) J_i,  J_i = [[1,0,-y_i],[0,1,x_i]]
//   G(u)   = min_i u^T M_i u ,   feasible  <=>  G(u) >= 0
//
// with q = [cos(theta), sin(theta)], theta = (d_min+d_max)/2 the cone center and
// gamma = (d_max-d_min)/2 the half-width (after any soft margin).  G is a
// homogeneous degree-2 form, so the feasible set is a cone from the origin
// (feasibility is a property of DIRECTION, invariant to uniform scaling).
//
// The analytic helpers (r(w), apexVx, feasibleAbsVy) are the closed form of the
// same set for a symmetric rectangular layout (theta == 0); they are convenience
// functions for navigation-side reasoning and are unit-tested to agree with G().
//
// Reference: colcon_ws/src/gaemi_simulations/scripts/g50_feasibility_continuous.py
//            g50_constrained_4wis.py (module order FL, FR, RL, RR).

#ifndef ANTBOT_SWERVE_FEASIBILITY__FEASIBILITY_MODEL_HPP_
#define ANTBOT_SWERVE_FEASIBILITY__FEASIBILITY_MODEL_HPP_

#include <array>
#include <optional>

#include <Eigen/Dense>

namespace antbot_swerve_feasibility
{

/// Steering-axis geometry of one module (base_link frame).
struct ModuleGeometry
{
  double x{0.0};  ///< steering-axis x [m]
  double y{0.0};  ///< steering-axis y [m]
  double c{0.0};  ///< signed lateral offset steering-axis -> wheel (left +, right -) [m]
};

/// Planar body twist.
struct Twist2D
{
  double vx{0.0};
  double vy{0.0};
  double w{0.0};
};

/// Actuator limits used by speedScale().
struct ActuatorLimits
{
  double wheel_speed_max{20.0};  ///< per-wheel |phi_dot| limit [rad/s]
  double wheel_radius{0.103};    ///< r [m]
};

/// How coneProject() trades off which component to shrink when infeasible.
enum class ProjectPolicy
{
  CrabPreferred,  ///< keep vx & w, shrink |vy| first (preserve arc/curvature)
  ArcPreferred,   ///< keep vx & vy, shrink |w| first (preserve crab direction)
  MinChange       ///< pick whichever single-axis reduction changes the twist least
};

/// Result of coneProject().
struct ProjectResult
{
  Twist2D twist;          ///< feasible twist to command
  bool changed{false};    ///< true if the input had to be modified
  bool feasible{false};   ///< true if the returned twist is feasible (always true unless input was NaN)
  double vy_scale{1.0};   ///< applied scale on vy in [0,1]
  double w_scale{1.0};    ///< applied scale on w in [0,1]
};

/// Constrained 4WIS-4WID feasibility model.
class FeasibilityModel
{
public:
  FeasibilityModel() = default;

  /// @param modules   steering-axis geometry, order FL, FR, RL, RR
  /// @param steer_min minimum steering angle [rad] (e.g. -0.9599 for -55 deg)
  /// @param steer_max maximum steering angle [rad]
  /// @param margin    soft feasibility margin [rad] subtracted from each side
  FeasibilityModel(
    const std::array<ModuleGeometry, 4> & modules,
    double steer_min, double steer_max, double margin = 0.0);

  // --- Idea #1: quadratic-form feasibility (authoritative) ---

  /// Contact velocity of module i for twist u:  [vx - w*y_i, vy + w*x_i].
  Eigen::Vector2d moduleVelocity(int i, const Twist2D & u) const;

  /// Raw per-module margin g_i = u^T M_i u (>= 0 feasible for that module).
  double moduleMargin(int i, const Twist2D & u) const;
  std::array<double, 4> moduleMargins(const Twist2D & u) const;

  /// G(u) = min_i u^T M_i u.  G >= 0  <=>  feasible.
  double margin(const Twist2D & u) const;
  bool feasible(const Twist2D & u) const { return margin(u) >= 0.0; }

  /// Normalized margin g_i / (||v_A,i||^2 + floor^2); dimensionless, same sign.
  double moduleMarginNorm(int i, const Twist2D & u) const;
  double marginNorm(const Twist2D & u) const;

  /// C1 smoothstep gate in [0,1] from the normalized margin (0 = boundary).
  double smoothAlpha(const Twist2D & u, double gate_width = 0.08) const;

  // --- Idea #2: analytic helpers (symmetric theta == 0 layout) ---

  bool symmetric() const { return symmetric_; }
  double gamma() const { return gamma_; }
  double theta() const { return theta_; }
  double halfWheelbase() const { return sx_; }  ///< |x_i| (symmetric only)
  double halfSteerWidth() const { return sy_; }  ///< |y_i| (symmetric only)

  /// Radius of the origin-centred circle tangent to the wedge edges:
  ///   r(w) = |w| * (sx*cos(gamma) + sy*sin(gamma))  (grows with |w|).
  /// This is the infeasible gap around the origin at yaw-rate w.
  double radiusOfW(double w) const;

  /// Smallest |vx| that admits any feasible vy at this w: r(w)/sin(gamma).
  double apexVx(double w) const;

  /// Upper bound on |vy| that is feasible at fixed (vx, w):
  ///   |vy| <= tan(gamma)*|vx| - r(w)/cos(gamma).
  /// Returns a negative number when no vy is feasible (|vx| < apexVx).
  double feasibleAbsVy(double vx, double w) const;

  // --- projection / scaling for the navigation limiter ---

  /// Project an infeasible twist onto the feasible cone with minimal direction
  /// change, per @p policy.  Uses G()-based bisection (authoritative), so the
  /// result is always feasible.  A feasible input is returned unchanged.
  ProjectResult coneProject(
    const Twist2D & u, ProjectPolicy policy = ProjectPolicy::CrabPreferred) const;

  /// Uniformly scale u so no wheel exceeds the speed limit.  Uniform scaling
  /// preserves direction and curvature (the "diff-drive-like" reduction) and,
  /// by cone homogeneity, preserves feasibility.  Approximates wheel speed by
  /// ||v_A,i||/r (ignores the steering-rate feed-forward the controller adds).
  Twist2D speedScale(const Twist2D & u, const ActuatorLimits & lim) const;

  /// Nearest yaw-rate to w_ref that keeps (vx, vy) feasible; std::nullopt if the
  /// translation direction itself is infeasible for every w in the search range.
  std::optional<double> feasibleOmega(
    double vx, double vy, double w_ref,
    double w_search_max = 3.0, int steps = 241) const;

  const std::array<ModuleGeometry, 4> & modules() const { return modules_; }

private:
  /// Largest s in [0,1] with feasible(make(s)); assumes make(0) feasible.
  template<typename MakeTwist>
  double maxFeasibleScale(MakeTwist make) const;

  std::array<ModuleGeometry, 4> modules_{};
  std::array<Eigen::Matrix3d, 4> M_{};

  double steer_min_{0.0};
  double steer_max_{0.0};
  double soft_margin_{0.0};
  double gamma_{0.0};
  double theta_{0.0};
  double cos2_gamma_{0.0};
  Eigen::Vector2d q_{1.0, 0.0};

  bool symmetric_{false};
  double sx_{0.0};
  double sy_{0.0};

  static constexpr double kSpeedFloor = 1e-9;
};

}  // namespace antbot_swerve_feasibility

#endif  // ANTBOT_SWERVE_FEASIBILITY__FEASIBILITY_MODEL_HPP_
