# antbot_swerve_controller

A **ros2_control** controller for non-coaxial swerve-drive mobile robots with limited steering range. Each swerve module has an offset between the steering axis and the wheel contact point, and operates within per-module steering angle limits. The controller handles re-alignment and mode transitions to operate within the constrained steering range.

The controller accepts `geometry_msgs/msg/Twist` commands and translates them into per-module steering angles and wheel velocities through inverse kinematics, optional motion profiling, and real-time-safe command dispatch.

> **Platform:** ROS 2 Humble &nbsp;|&nbsp; **License:** Apache-2.0 &nbsp;|&nbsp; **C++ Standard:** C++17

---

## Features

| Category | Description |
|----------|-------------|
| **Swerve Kinematics** | Full inverse-kinematics for N-module swerve drives with non-coaxial steering support (configurable steering-to-wheel Y offset). |
| **Synchronized Motion Profiling** | Trapezoidal velocity profiles with global synchronization across all modules, ensuring smooth coordinated motion during mode transitions. |
| **Steering Optimization** | Direction-flip logic minimises steering rotation; per-module angle limits are enforced at every cycle. |
| **Steering Scrub Compensation** | Feed-forward compensation for wheel scrub caused by non-coaxial steering rotation. |
| **Speed Limiting** | Per-axis velocity, acceleration, and jerk limits on the commanded twist. |
| **Wheel Saturation Scaling** | When any wheel exceeds its speed limit, all wheels are scaled proportionally to preserve the commanded trajectory shape. |
| **Odometry** | Multiple solvers (SVD, QR, pseudo-inverse) and integration methods (Euler, RK2, RK4, analytic swerve). Publishes both `nav_msgs/Odometry` and TF. |
| **Direct Joint Commands** | Bypass twist control and command individual steering/wheel joints directly via `sensor_msgs/JointState`. |
| **Real-time Safety** | All publishers use `realtime_tools` buffers; no heap allocation in the update loop. |
| **Lifecycle Support** | Full `ros2_control` lifecycle (unconfigured &rarr; inactive &rarr; active). |

---

## Architecture

```
                          ┌──────────────────────────────────┐
                          │    antbot_swerve_controller      │
  /cmd_vel ──────────────►│                                  │
  (geometry_msgs/Twist)   │  ┌────────────────────────────┐  │
                          │  │  Speed Limiter              │  │──► steering_joint/position
                          │  │  (vel / acc / jerk limits)  │  │──► steering_joint/velocity  (opt)
                          │  └──────────┬─────────────────┘  │──► steering_joint/acceleration (opt)
                          │             ▼                    │
                          │  ┌────────────────────────────┐  │──► wheel_joint/velocity
                          │  │  Swerve Motion Control     │  │──► wheel_joint/acceleration
                          │  │  (IK + motion profiling)   │  │
                          │  └──────────┬─────────────────┘  │──► /odom
                          │             ▼                    │──► /tf
                          │  ┌────────────────────────────┐  │──► ~/planned_trajectory
                          │  │  Odometry                  │  │──► ~/joint_commanders
                          │  │  (SVD / QR / pseudo-inv)   │  │
                          │  └────────────────────────────┘  │
                          └──────────────────────────────────┘
```

---

## Quick Start

### 1. Register the controller in your robot's `ros2_control` configuration

```yaml
controller_manager:
  ros__parameters:
    update_rate: 20  # Hz
    antbot_swerve_controller:
      type: antbot_swerve_controller/SwerveDriveController
```

### 2. Launch and activate the controller

```bash
# Load
ros2 control load_controller antbot_swerve_controller

# Configure + Activate
ros2 control set_controller_state antbot_swerve_controller active
```

### 3. Send velocity commands

```bash
# Drive forward at 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Strafe right at 0.3 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: -0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate in place at 1.0 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

---

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command (configurable via `cmd_vel_topic`). |
| `~/direct_joint_commands` | `sensor_msgs/msg/JointState` | Direct per-joint steering position and wheel velocity commands. |

### Published Topics

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `~/odom` | `nav_msgs/msg/Odometry` | Default | Robot odometry (pose + twist) |
| `/tf` | `tf2_msgs/msg/TFMessage` | Default | `odom` &rarr; `base_link` transform (toggleable) |
| `~/planned_trajectory` | `trajectory_msgs/msg/JointTrajectory` | Default | Planned motion profile for visualisation/debugging |
| `~/joint_commanders` | `sensor_msgs/msg/JointState` | Default | Commanded steering positions and wheel velocities |
| `~/limited_cmd_vel` | `geometry_msgs/msg/Twist` | Default | Post-limiter velocity (when `publish_limited_velocity: true`) |

---

## Hardware Interfaces

### Command Interfaces

| Interface | Type | Required |
|-----------|------|----------|
| `<steering_joint>/position` | `double` | **Yes** |
| `<steering_joint>/velocity` | `double` | Optional (`steering.use_velocity_command`) |
| `<steering_joint>/acceleration` | `double` | Optional (`steering.use_acceleration_command`) |
| `<wheel_joint>/velocity` | `double` | **Yes** |
| `<wheel_joint>/acceleration` | `double` | **Yes** (always claimed; sends `wheel.max_acceleration` when `wheel.use_acceleration_command: false`) |

### State Interfaces

| Interface | Type | Required |
|-----------|------|----------|
| `<steering_joint>/position` | `double` | **Yes** |
| `<wheel_joint>/velocity` | `double` | **Yes** |

---

## Parameters

### Robot Geometry

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `steering_joint_names` | `string[]` | `[]` | Steering joint names (one per module) |
| `wheel_joint_names` | `string[]` | `[]` | Wheel joint names (one per module) |
| `wheel_radius` | `double` | `0.103` | Wheel radius [m] |
| `module_x_offsets` | `double[]` | `[]` | X-offset from base_link to each steering joint [m] |
| `module_y_offsets` | `double[]` | `[]` | Y-offset from base_link to each steering joint [m] |
| `module_angle_offsets` | `double[]` | `[0, ...]` | Initial angle offset for each steering joint [rad] |
| `steering_to_wheel_y_offsets` | `double[]` | `[0, ...]` | Non-coaxial Y-offset between steering axis and wheel contact [m] |
| `module_steering_limit_lower` | `double[]` | `[-1.047, ...]` | Lower steering angle limit per module [rad] |
| `module_steering_limit_upper` | `double[]` | `[1.047, ...]` | Upper steering angle limit per module [rad] |
| `module_wheel_speed_limit_lower` | `double[]` | `[-50, ...]` | Lower wheel speed limit per module [rad/s] |
| `module_wheel_speed_limit_upper` | `double[]` | `[50, ...]` | Upper wheel speed limit per module [rad/s] |

### Motion Control

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enabled_synchronized_setpoint` | `bool` | `false` | Enable chassis-level synchronized setpoint |
| `trajectory_delay_time` | `double` | `0.50` | Look-ahead time offset for trajectory tracking [s] |
| `enabled_steering_flip` | `bool` | `true` | Optimise steering by flipping direction when shorter |
| `steering_alignment_angle_error_threshold` | `double` | `6.29` | Wheel stops if steering error exceeds this [rad] |
| `enabled_wheel_saturation_scaling` | `bool` | `true` | Scale all wheels proportionally when one saturates |
| `realigning_angle_threshold` | `double` | `0.087` | Angle tolerance for re-alignment completion [rad] |
| `discontinuous_motion_steering_tolerance` | `double` | `1.0` | Threshold for detecting discontinuous mode change [rad] |
| `velocity_deadband` | `double` | `0.01` | Ignore velocities below this value [m/s, rad/s] |
| `non_coaxial_ik_iterations` | `int` | `0` | Iterative IK refinement steps for non-coaxial geometry |

### Steering Profile

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `steering.use_velocity_command` | `bool` | `true` | Send velocity feedforward to steering joint |
| `steering.use_acceleration_command` | `bool` | `true` | Send acceleration feedforward to steering joint |
| `steering.max_velocity` | `double` | `5.0` | Max steering angular velocity [rad/s] |
| `steering.max_acceleration` | `double` | `10.0` | Max steering angular acceleration [rad/s^2] |
| `enable_steering_scrub_compensator` | `bool` | `true` | Enable non-coaxial scrub feed-forward |
| `steering_scrub_compensator_scale_factor` | `double` | `1.0` | Scale factor for scrub compensation |

### Wheel Profile

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wheel.use_acceleration_command` | `bool` | `true` | Use dynamic acceleration (emergency stop aware); when `false`, always sends `wheel.max_acceleration` |
| `wheel.max_acceleration` | `double` | `18.73` | Max wheel acceleration [rad/s^2] |
| `wheel.emergency_stop_acceleration` | `double` | `0.0` | Wheel decel on emergency stop (0 = instant) [rad/s^2] |

### Speed Limits (cmd_vel)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enabled_speed_limits` | `bool` | `false` | Enable chassis-level speed limiting |
| `publish_limited_velocity` | `bool` | `true` | Publish the post-limiter velocity |
| `linear.x.max_velocity` | `double` | `2.0` | [m/s] |
| `linear.x.max_acceleration` | `double` | `1.0` | [m/s^2] |
| `linear.y.max_velocity` | `double` | `1.5` | [m/s] |
| `angular.z.max_velocity` | `double` | `2.0` | [rad/s] |
| `angular.z.max_acceleration` | `double` | `1.0` | [rad/s^2] |

> Full per-axis `min/max` velocity, acceleration, and jerk limits are available. See `config/swerve_drive_controller_parameter.yaml` for the complete list.

### Odometry

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `odom_solver_method` | `string` | `"svd"` | `"svd"`, `"qr"`, or `"pseudo_inverse"` |
| `odom_integration_method` | `string` | `"rk4"` | `"euler"`, `"rk2"`, `"rk4"`, or `"analytic_swerve"` |
| `odom_frame_id` | `string` | `"odom"` | Odometry frame ID |
| `base_frame_id` | `string` | `"base_link"` | Robot base frame ID |
| `enable_odom_tf` | `bool` | `true` | Publish odom &rarr; base_link TF |
| `pose_covariance_diagonal` | `double[6]` | `[0.001, ...]` | Diagonal of pose covariance matrix |
| `twist_covariance_diagonal` | `double[6]` | `[0.001, ...]` | Diagonal of twist covariance matrix |
| `velocity_rolling_window_size` | `int` | `1` | Rolling window size for velocity estimation |

### Command Interface

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cmd_vel_topic` | `string` | `"/cmd_vel"` | Velocity command topic name |
| `use_stamped_cmd_vel` | `bool` | `false` | Accept `TwistStamped` instead of `Twist` |
| `cmd_vel_timeout` | `double` | `0.50` | Halt if no command received for this duration [s] |
| `enable_direct_joint_commands` | `bool` | `true` | Enable direct joint command interface |
| `direct_joint_command_topic` | `string` | `"~/direct_joint_commands"` | Direct joint command topic |
| `direct_joint_command_timeout_sec` | `double` | `5.0` | Timeout for direct joint commands [s] |

---

## Configuration Example

A complete example for a 4-module swerve robot:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 20
    antbot_swerve_controller:
      type: antbot_swerve_controller/SwerveDriveController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

antbot_swerve_controller:
  ros__parameters:
    steering:
      use_velocity_command: true
      use_acceleration_command: true
      max_velocity: 5.0       # rad/s
      max_acceleration: 10.0  # rad/s^2

    wheel:
      use_acceleration_command: true
      max_acceleration: 18.7254  # rad/s^2

    # --- Odometry ---
    odom_solver_method: "svd"
    odom_integration_method: "rk4"
    enable_odom_tf: true

    # --- Timeout ---
    cmd_vel_timeout: 0.50
```

---

## License

Copyright 2026 ROBOTIS AI CO., LTD.

Licensed under the Apache License, Version 2.0. See [LICENSE](http://www.apache.org/licenses/LICENSE-2.0) for details.
