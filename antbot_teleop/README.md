# antbot_teleop

Teleoperation package for the ANTBot swerve-drive robot. Provides keyboard and PS4 joystick control nodes. All nodes publish `geometry_msgs/msg/Twist` on `/cmd_vel`.

## Nodes

### teleop_keyboard

Terminal-based keyboard control. The robot moves only while a key is held and stops on release.

```
   q    w    e
   a         d
        x

w/x : forward / backward     (linear.x)
a/d : strafe left / right    (linear.y)
q/e       : rotate CCW / CW        (angular.z)
1~9       : speed level (1=slow, 9=max)
ESC/Ctrl+C: quit
```

> **Note:** Requires a terminal (TTY) for keyboard input. Always run directly in a terminal, not via a launch file.

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_vel` | `1.0` | Maximum linear velocity [m/s] |
| `max_angular_vel` | `1.0` | Maximum angular velocity [rad/s] |
| `speed_level` | `3` | Initial speed level (1~9) |
| `publish_rate` | `10.0` | Publish rate [Hz] |

### teleop_joystick

Joystick control with automatic controller detection (DualSense) and geometry-based angular velocity limiting. Prevents steering angles from exceeding hardware limits by computing maximum angular velocity from robot geometry.

**Supported controllers:** PS5 DualSense (tested), PS4 DualShock 4 (untested) via USB connection. The node auto-detects the controller type via USB product ID, with axis-based fallback detection. Hot-plug is supported — swapping controllers mid-session is automatically handled.

```
[Left Stick]                        [Triggers]
  Y-axis : forward / backward        L2 : in-place rotate CCW
  X-axis : curve turning              R2 : in-place rotate CW
           (while moving)

[Buttons]
  Triangle : speed level UP (+1)     L1 : headlight toggle (ON/OFF)
  Cross    : speed level DOWN (-1)   R1 : wiper toggle (REPEAT/OFF)
  Square   : cargo lock              Circle : cargo unlock
```

**Two driving modes** (mutually exclusive):
- **Curve driving** (`|vx| >= 0.05 m/s`): Left stick X controls angular velocity, constrained by `w_max = min(|vx| / R_min, W_ABS_MAX)`. Steering inverts automatically when reversing.
- **In-place rotation** (`|vx| < 0.05 m/s`): L2/R2 triggers control spin. `wz = max_spin_vel * speed_ratio * (L2 - R2)`.

**Service calls:** Square/Circle buttons call `cargo/command` (`CargoCommand`). L1 toggles `headlight/operation` (`SetBool`). R1 toggles `wiper/operation` (`WiperOperation`).

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_vel` | `1.0` | Maximum linear velocity [m/s] |
| `max_spin_vel` | `1.0` | Maximum in-place rotation speed [rad/s] |
| `speed_level` | `3` | Initial speed level (1~9) |
| `deadzone` | `0.1` | Stick/trigger deadzone threshold |
| `module_x` | `0.265` | Swerve module X-offset from base center [m] |
| `module_y` | `0.256` | Swerve module Y-offset from base center [m] |
| `steering_limit_deg` | `60.0` | Hardware steering limit [deg] |
| `safety_factor` | `0.95` | Use 95% of steering limit (3 deg margin) |
| `w_abs_max` | `2.0` | Absolute max angular velocity [rad/s] |

## Usage

```bash
# Keyboard teleop (run in terminal)
ros2 run antbot_teleop teleop_keyboard

# Joystick teleop (DualSense via USB)
ros2 launch antbot_teleop teleop_joy.launch.py
```

Override parameters via command line:
```bash
ros2 run antbot_teleop teleop_keyboard --ros-args -p max_linear_vel:=0.5
ros2 run antbot_teleop teleop_joystick --ros-args -p max_linear_vel:=0.5 -p deadzone:=0.15
```

## Dependencies

| Dependency | Description |
|-----------|-------------|
| `rclpy` | ROS 2 Python client library |
| `geometry_msgs` | Twist message type |
| `sensor_msgs` | Joy message type (joystick) |
| `antbot_interfaces` | CargoCommand, WiperOperation service types |
| `std_srvs` | SetBool service type (headlight) |
| `joy` | Joystick driver node (runtime) |

## Build

```bash
colcon build --symlink-install --packages-select antbot_teleop
```

## License

Apache License 2.0 — Copyright 2026 ROBOTIS AI CO., LTD.
