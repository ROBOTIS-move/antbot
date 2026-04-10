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

#ifndef ANTBOT_SWERVE_CONTROLLER__SPEED_LIMITER_HPP_
#define ANTBOT_SWERVE_CONTROLLER__SPEED_LIMITER_HPP_

#include <cmath>

namespace antbot
{
namespace swerve_drive_controller
{
class SpeedLimiter
{
public:
  SpeedLimiter(
    bool has_velocity_limits = false, bool has_acceleration_limits = false,
    bool has_jerk_limits = false, double min_velocity = NAN, double max_velocity = NAN,
    double min_acceleration = NAN, double max_acceleration = NAN, double min_jerk = NAN,
    double max_jerk = NAN);

  double limit(double & v, double v0, double v1, double dt);
  double limit_velocity(double & v);
  double limit_acceleration(double & v, double v0, double dt);
  double limit_jerk(double & v, double v0, double v1, double dt);

private:
  bool has_velocity_limits_;
  bool has_acceleration_limits_;
  bool has_jerk_limits_;

  double min_velocity_;
  double max_velocity_;

  double min_acceleration_;
  double max_acceleration_;

  double min_jerk_;
  double max_jerk_;
};

}   // namespace swerve_drive_controller
}  // namespace antbot

#endif  // ANTBOT_SWERVE_CONTROLLER__SPEED_LIMITER_HPP_
