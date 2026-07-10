// Copyright 2026 ANTBot
// Unit tests for FeasibilityModel, cross-checked against the reference Python
// (g50_feasibility_continuous.py) via the auto-generated oracle_data.hpp.
#include <array>
#include <cmath>
#include <map>

#include <gtest/gtest.h>

#include "antbot_swerve_feasibility/feasibility_model.hpp"
#include "oracle_data.hpp"

using antbot_swerve_feasibility::ActuatorLimits;
using antbot_swerve_feasibility::FeasibilityModel;
using antbot_swerve_feasibility::ModuleGeometry;
using antbot_swerve_feasibility::ProjectPolicy;
using antbot_swerve_feasibility::Twist2D;

namespace
{

constexpr double kSx = 0.265;
constexpr double kSy = 0.2005;
constexpr double kC = 0.0555;

// antbot geometry, order FL, FR, RL, RR.
std::array<ModuleGeometry, 4> antbotModules()
{
  return {{
    {+kSx, +kSy, +kC},   // FL
    {+kSx, -kSy, -kC},   // FR
    {-kSx, +kSy, +kC},   // RL
    {-kSx, -kSy, -kC},   // RR
  }};
}

FeasibilityModel makeModel(double limit_deg, double margin_deg)
{
  const double lim = limit_deg * M_PI / 180.0;
  const double marg = margin_deg * M_PI / 180.0;
  return FeasibilityModel(antbotModules(), -lim, lim, marg);
}

// Cache one model per (limit, margin) so the oracle sweep is cheap.
const FeasibilityModel & cachedModel(double limit_deg, double margin_deg)
{
  static std::map<std::pair<double, double>, FeasibilityModel> cache;
  const auto key = std::make_pair(limit_deg, margin_deg);
  auto it = cache.find(key);
  if (it == cache.end()) {
    it = cache.emplace(key, makeModel(limit_deg, margin_deg)).first;
  }
  return it->second;
}

}  // namespace

// Every module margin, the aggregate G, and the feasible flag must match the
// reference Python to double precision.
TEST(OracleMatch, MatchesReferencePython)
{
  ASSERT_GT(oracle_rows().size(), 100u);
  for (const auto & row : oracle_rows()) {
    const auto & model = cachedModel(row.steer_limit_deg, row.margin_deg);
    const Twist2D u{row.vx, row.vy, row.w};

    const auto mm = model.moduleMargins(u);
    for (int i = 0; i < 4; ++i) {
      EXPECT_NEAR(mm[i], row.g[i], 1e-9)
        << "module " << i << " lim=" << row.steer_limit_deg << " marg=" << row.margin_deg
        << " u=(" << row.vx << "," << row.vy << "," << row.w << ")";
    }
    EXPECT_NEAR(model.margin(u), row.G, 1e-9);
    // Boolean feasibility only where the sign is unambiguous (away from G==0).
    if (std::abs(row.G) > 1e-9) {
      EXPECT_EQ(model.feasible(u), row.feasible)
        << "G=" << row.G << " u=(" << row.vx << "," << row.vy << "," << row.w << ")";
    }
  }
}

// G is homogeneous degree 2: feasibility is a property of direction only.
TEST(Homogeneity, DegreeTwoScaling)
{
  const auto & model = cachedModel(55.0, 0.0);
  const std::array<Twist2D, 4> pts{{
    {0.8, 0.4, 0.3}, {0.2, -0.5, 0.9}, {1.0, 0.0, -0.7}, {0.3, 0.3, 0.3}}};
  for (const auto & u : pts) {
    const double g = model.margin(u);
    for (double a : {0.25, 0.5, 2.0, 3.5}) {
      const Twist2D ua{a * u.vx, a * u.vy, a * u.w};
      EXPECT_NEAR(model.margin(ua), a * a * g, 1e-9);
      // Scaling never changes feasibility.
      EXPECT_EQ(model.feasible(ua), model.feasible(u));
    }
  }
}

TEST(Scenarios, KnownDirections)
{
  const auto & m = cachedModel(55.0, 0.0);
  EXPECT_TRUE(m.feasible({0.5, 0.0, 0.0}));    // forward
  EXPECT_TRUE(m.feasible({-0.5, 0.0, 0.0}));   // backward (flip)
  EXPECT_TRUE(m.feasible({0.8, 0.4, 0.0}));    // diagonal ~26.6 deg
  EXPECT_FALSE(m.feasible({0.0, 0.5, 0.0}));   // pure lateral -> 90 deg, infeasible
  // Pure spin needs atan2(sx,sy) = 52.9 deg: feasible at 55 deg, infeasible at 30 deg.
  EXPECT_TRUE(m.feasible({0.0, 0.0, 0.5}));
  EXPECT_FALSE(cachedModel(30.0, 0.0).feasible({0.0, 0.0, 0.5}));
}

// Analytic idea-#2 helpers must agree with the authoritative quadratic form.
TEST(AnalyticVsG, FeasibleVyBoundaryMatches)
{
  const auto & m = cachedModel(55.0, 0.0);
  ASSERT_TRUE(m.symmetric());
  // r grows with |w|.
  EXPECT_DOUBLE_EQ(m.radiusOfW(0.0), 0.0);
  EXPECT_GT(m.radiusOfW(1.0), m.radiusOfW(0.5));
  EXPECT_GT(m.radiusOfW(0.5), 0.0);

  for (double vx : {0.5, 0.8, 1.2, 1.5}) {
    for (double w : {0.0, 0.3, 0.6}) {
      for (double sgn : {+1.0, -1.0}) {
        const double bound = m.feasibleAbsVy(sgn * vx, w);
        if (bound <= 0.05) {continue;}   // only test a safely-positive boundary
        EXPECT_TRUE(m.feasible({sgn * vx, sgn * 0.0 + 0.98 * bound, w}));
        EXPECT_FALSE(m.feasible({sgn * vx, 1.02 * bound, w}));
        EXPECT_TRUE(m.feasible({sgn * vx, -0.98 * bound, w}));
      }
    }
  }
}

TEST(AnalyticVsG, ApexVxIsFeasibilityThreshold)
{
  const auto & m = cachedModel(55.0, 0.0);
  for (double w : {0.3, 0.6, 1.0}) {
    const double apex = m.apexVx(w);
    ASSERT_GT(apex, 0.0);
    // Just inside apex, vy==0 arc is feasible; just below, infeasible.
    EXPECT_TRUE(m.feasible({1.02 * apex, 0.0, w}));
    EXPECT_FALSE(m.feasible({0.98 * apex, 0.0, w}));
  }
}

TEST(ConeProject, AlwaysReturnsFeasible)
{
  const auto & m = cachedModel(55.0, 0.0);
  const std::array<Twist2D, 5> infeasibles{{
    {0.2, 0.9, 0.0},   // too much crab
    {0.3, 0.2, 1.4},   // arc too tight
    {0.0, 0.5, 0.0},   // pure lateral
    {0.1, -0.8, 0.6},  // mixed
    {0.05, 0.0, 1.2},  // near-spin, small vx
  }};
  for (const auto & u : infeasibles) {
    ASSERT_FALSE(m.feasible(u));
    for (auto policy : {ProjectPolicy::CrabPreferred, ProjectPolicy::ArcPreferred,
        ProjectPolicy::MinChange})
    {
      const auto r = m.coneProject(u, policy);
      EXPECT_TRUE(r.changed);
      EXPECT_TRUE(m.feasible(r.twist))
        << "policy result infeasible for u=(" << u.vx << "," << u.vy << "," << u.w << ")";
      // Reductions stay within [0,1] and never amplify a component.
      EXPECT_GE(r.vy_scale, 0.0);
      EXPECT_LE(r.vy_scale, 1.0);
      EXPECT_GE(r.w_scale, 0.0);
      EXPECT_LE(r.w_scale, 1.0);
    }
  }
  // A feasible input is returned unchanged.
  const Twist2D good{0.5, 0.1, 0.2};
  ASSERT_TRUE(m.feasible(good));
  const auto r = m.coneProject(good);
  EXPECT_FALSE(r.changed);
  EXPECT_DOUBLE_EQ(r.twist.vx, good.vx);
  EXPECT_DOUBLE_EQ(r.twist.vy, good.vy);
  EXPECT_DOUBLE_EQ(r.twist.w, good.w);
}

TEST(ConeProject, CrabPreferredKeepsArc)
{
  const auto & m = cachedModel(55.0, 0.0);
  const Twist2D u{0.8, 1.3, 0.3};   // vx & w feasible as an arc, vy above the ~0.98 bound
  ASSERT_FALSE(m.feasible(u));
  const auto r = m.coneProject(u, ProjectPolicy::CrabPreferred);
  EXPECT_TRUE(m.feasible(r.twist));
  EXPECT_DOUBLE_EQ(r.twist.vx, u.vx);   // vx preserved
  EXPECT_DOUBLE_EQ(r.twist.w, u.w);     // w (arc) preserved
  EXPECT_LT(std::abs(r.twist.vy), std::abs(u.vy));  // vy shrunk
}

TEST(SpeedScale, PreservesDirectionAndRespectsLimit)
{
  const auto & m = cachedModel(55.0, 0.0);
  ActuatorLimits lim;
  lim.wheel_speed_max = 20.0;
  lim.wheel_radius = 0.103;
  const Twist2D u{3.0, 1.0, 2.5};   // deliberately fast
  const Twist2D s = m.speedScale(u, lim);
  // Same direction (curvature preserved): u x s == 0 componentwise ratios equal.
  ASSERT_GT(std::hypot(u.vx, u.vy), 1e-9);
  const double ratio = s.vx / u.vx;
  EXPECT_NEAR(s.vy / u.vy, ratio, 1e-9);
  EXPECT_NEAR(s.w / u.w, ratio, 1e-9);
  EXPECT_GT(ratio, 0.0);
  EXPECT_LE(ratio, 1.0);
  // No wheel exceeds the limit (contact-speed proxy).
  double max_v = 0.0;
  for (int i = 0; i < 4; ++i) {max_v = std::max(max_v, m.moduleVelocity(i, s).norm());}
  EXPECT_LE(max_v, lim.wheel_speed_max * lim.wheel_radius + 1e-9);
}

TEST(FeasibleOmega, RelaxesToNearestFeasible)
{
  const auto & m = cachedModel(55.0, 0.0);
  // Feasible request returns itself.
  const auto keep = m.feasibleOmega(0.6, 0.0, 0.3);
  ASSERT_TRUE(keep.has_value());
  EXPECT_DOUBLE_EQ(*keep, 0.3);
  // Infeasible arc: reduce |w| to a feasible one.
  const Twist2D tight{0.3, 0.0, 1.5};
  ASSERT_FALSE(m.feasible(tight));
  const auto relaxed = m.feasibleOmega(0.3, 0.0, 1.5);
  ASSERT_TRUE(relaxed.has_value());
  EXPECT_TRUE(m.feasible({0.3, 0.0, *relaxed}));
  EXPECT_LT(std::abs(*relaxed), 1.5);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
