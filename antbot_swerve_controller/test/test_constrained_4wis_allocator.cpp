// Copyright 2026 ANTBot
// Unit tests for Constrained4WisAllocator, cross-checked against the reference
// Python (g50_constrained_4wis.py, alpha removed) via allocator_oracle_data.hpp.
#include <array>
#include <cmath>
#include <map>

#include <gtest/gtest.h>

#include "antbot_swerve_controller/constrained_4wis_allocator.hpp"
#include "allocator_oracle_data.hpp"

using antbot_swerve_controller::AllocatorConfig;
using antbot_swerve_controller::Constrained4WisAllocator;
using antbot_swerve_controller::ModuleGeometry;

namespace
{

constexpr double kSx = 0.265;
constexpr double kSy = 0.2005;
constexpr double kC = 0.0555;
constexpr double kR = 0.103;

std::array<ModuleGeometry, 4> antbotModules()
{
  return {{
    {+kSx, +kSy, +kC},   // FL
    {+kSx, -kSy, -kC},   // FR
    {-kSx, +kSy, +kC},   // RL
    {-kSx, -kSy, -kC},   // RR
  }};
}

// Build an allocator matching the oracle generator (kp=4, rate=3), with the
// wheel-speed limit set huge so wheel_velocity_raw is un-clamped for comparison.
Constrained4WisAllocator makeAllocator(double limit_deg, double margin_deg, double wheel_limit)
{
  AllocatorConfig cfg;
  cfg.steer_min = -limit_deg * M_PI / 180.0;
  cfg.steer_max = limit_deg * M_PI / 180.0;
  cfg.soft_margin = margin_deg * M_PI / 180.0;
  cfg.steer_gain = 4.0;
  cfg.steer_rate_limit = 3.0;
  cfg.wheel_speed_limit = wheel_limit;
  cfg.wheel_radius = kR;
  cfg.low_speed_eps = 1e-3;
  return Constrained4WisAllocator(antbotModules(), cfg);
}

}  // namespace

TEST(AllocatorOracle, MatchesReferencePython)
{
  ASSERT_GT(alloc_oracle_rows().size(), 50u);
  for (const auto & row : alloc_oracle_rows()) {
    const auto alloc = makeAllocator(row.steer_limit_deg, row.margin_deg, 1e9);
    const std::array<double, 4> steering{
      row.steering[0], row.steering[1], row.steering[2], row.steering[3]};
    const auto r = alloc.update(steering, row.vx, row.vy, row.w, 0.02);

    for (int i = 0; i < 4; ++i) {
      EXPECT_NEAR(r.target[i], row.target[i], 1e-9) << "target " << i;
      EXPECT_NEAR(r.steer_velocity[i], row.delta_dot[i], 1e-9) << "delta_dot " << i;
      EXPECT_NEAR(r.steer_position[i], row.steer_cmd[i], 1e-9) << "steer_cmd " << i;
      EXPECT_NEAR(r.wheel_velocity_raw[i], row.wheel_raw[i], 1e-9) << "wheel_raw " << i;
    }
    EXPECT_EQ(r.feasible, row.feasible)
      << "u=(" << row.vx << "," << row.vy << "," << row.w << ") lim=" << row.steer_limit_deg;
  }
}

TEST(AllocatorScenarios, ForwardAndBackward)
{
  const auto a = makeAllocator(55.0, 0.0, 20.0);
  const std::array<double, 4> zero{0, 0, 0, 0};

  // Forward: all modules point straight, equal positive wheel speeds.
  const auto f = a.update(zero, 0.5, 0.0, 0.0, 0.02);
  EXPECT_TRUE(f.feasible);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(f.target[i], 0.0, 1e-9);
    EXPECT_GT(f.wheel_velocity[i], 0.0);
  }
  EXPECT_NEAR(f.wheel_velocity[0], f.wheel_velocity[1], 1e-9);

  // Backward: flip branch keeps steering ~0 (no 180 turn) and drives wheels reverse.
  const auto b = a.update(zero, -0.5, 0.0, 0.0, 0.02);
  EXPECT_TRUE(b.feasible);
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(b.target[i], 0.0, 1e-9);
    EXPECT_EQ(b.drive_sign[i], -1);
    EXPECT_LT(b.wheel_velocity[i], 0.0);
  }
}

TEST(AllocatorScenarios, PureLateralIsInfeasibleAndClamped)
{
  const auto a = makeAllocator(55.0, 0.0, 20.0);
  const std::array<double, 4> zero{0, 0, 0, 0};
  const auto r = a.update(zero, 0.0, 0.6, 0.0, 0.02);
  EXPECT_FALSE(r.feasible);   // needs 90 deg > 55 deg limit
  for (int i = 0; i < 4; ++i) {
    // target clamped inside the limit; clamp_error records the shortfall.
    EXPECT_LE(std::abs(r.target[i]), 55.0 * M_PI / 180.0 + 1e-9);
    EXPECT_GT(std::abs(r.clamp_error[i]), 1e-6);
  }
}

TEST(AllocatorScenarios, LowSpeedHoldsCurrentAngle)
{
  const auto a = makeAllocator(55.0, 0.0, 20.0);
  const std::array<double, 4> held{0.3, -0.2, 0.4, -0.1};
  const auto r = a.update(held, 0.0, 0.0, 0.0, 0.02);   // |v_A|=0 everywhere
  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(r.target[i], held[i], 1e-12);
    EXPECT_NEAR(r.steer_velocity[i], 0.0, 1e-12);
    EXPECT_NEAR(r.wheel_velocity[i], 0.0, 1e-12);
  }
}

TEST(AllocatorWheelClamp, RespectsLimitAndFlags)
{
  const auto a = makeAllocator(55.0, 0.0, 20.0);
  const std::array<double, 4> zero{0, 0, 0, 0};
  const auto r = a.update(zero, 5.0, 0.0, 0.0, 0.02);   // 5 m/s -> ~48.5 rad/s > 20
  EXPECT_TRUE(r.wheel_clamp_active);
  for (int i = 0; i < 4; ++i) {
    EXPECT_LE(std::abs(r.wheel_velocity[i]), 20.0 + 1e-9);
  }
}

TEST(AllocatorSteering, RateLimitedTowardTarget)
{
  const auto a = makeAllocator(55.0, 0.0, 20.0);
  // Large steering error should saturate the steering-velocity rate limit (3 rad/s).
  const std::array<double, 4> zero{0, 0, 0, 0};
  const auto r = a.update(zero, 0.0, 0.0, 1.0, 0.02);   // spin -> targets ~ +-52.9 deg
  for (int i = 0; i < 4; ++i) {
    EXPECT_LE(std::abs(r.steer_velocity[i]), 3.0 + 1e-12);
    // step moves toward target but not past it
    EXPECT_LE(std::abs(r.steer_position[i]), std::abs(r.target[i]) + 1e-9);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
