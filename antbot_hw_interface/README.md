# antbot_hw_interface

A [ros2_control](https://control.ros.org/) `SystemInterface` hardware plugin for the ANTBot 4-wheel independent swerve-drive delivery robot.

This plugin communicates with the ANT-RCU (Robot Control Unit) board over Dynamixel Protocol 2.0.

## Features

- **4-wheel swerve drive** — Independent velocity control for 4 wheel motors (M1-M4) and position control for 4 steering motors (S1-S4)
- **Encoder feedback** — 12-bit encoders with 4x interpolation, multi-turn position tracking
- **Battery monitoring** — Publishes `sensor_msgs/BatteryState` (voltage, current, SoC, temperature)
- **Ultrasound sensors** — Publishes distance measurements as `std_msgs/Float64MultiArray`
- **Cargo door control** — Lock/unlock cargo via `cargo/command` service
- **Headlight control** — On/off via `headlight/operation` service
- **Wiper control** — Off/once/repeat modes via `wiper/operation` service

## Interfaces

### Command Interfaces

| Joint | Interface | Unit |
|-------|-----------|------|
| `wheel_{front,rear}_{left,right}_joint` | `velocity` | rad/s |
| `wheel_{front,rear}_{left,right}_joint` | `acceleration` | rad/s² |
| `steering_{front,rear}_{left,right}_joint` | `position` | rad |
| `steering_{front,rear}_{left,right}_joint` | `velocity` | rad/s |
| `steering_{front,rear}_{left,right}_joint` | `acceleration` | rad/s² |

### State Interfaces

| Joint | Interface | Unit |
|-------|-----------|------|
| `wheel_*_joint` | `velocity` | rad/s |
| `wheel_*_joint` | `effort` | A |
| `wheel_*_joint` | `position` | rad (cumulative) |
| `steering_*_joint` | `position` | rad |
| `steering_*_joint` | `effort` | A |

Battery state is published as `sensor_msgs/BatteryState` via a dedicated ROS 2 topic (voltage, current, SoC, temperature).

Ultrasound distance data is published as `std_msgs/Float64MultiArray` on the `sensor/ultrasound` topic.

### Services

| Service | Type | Description |
|---------|------|-------------|
| `cargo/command` | `antbot_interfaces/CargoCommand` | Lock (`OPERATION_LOCK=0`) or unlock (`OPERATION_UNLOCK=1`) the cargo door |
| `headlight/operation` | `std_srvs/SetBool` | Turn headlight on (`true`) or off (`false`) |
| `wiper/operation` | `antbot_interfaces/WiperOperation` | Set wiper mode: `OFF=0`, `ONCE=1`, `REPEAT=2` (with configurable `cycle_time`) |

## Control Table

The ANT-RCU register map is defined in [config/control_table.xml](config/control_table.xml).

## Package Structure

```
antbot_hw_interface/
├── include/antbot_hw_interface/
│   ├── board_interface.hpp       # SystemInterface implementation
│   └── device/
│       ├── device.hpp            # Device base class
│       ├── wheel.hpp             # Wheel motor control
│       ├── steering.hpp          # Steering motor control
│       ├── encoder.hpp           # Encoder position tracking
│       ├── battery.hpp           # Battery state monitoring
│       ├── ultrasound.hpp        # Ultrasound distance sensing
│       ├── cargo.hpp             # Cargo door lock/unlock control
│       ├── headlight.hpp         # Headlight on/off control
│       └── wiper.hpp             # Wiper motor control
├── src/                          # Implementation files
├── config/
│   ├── control_table.xml         # ANT-RCU register map
│   └── board_params.yaml         # Default parameters
└── antbot_hw_interface.xml       # pluginlib plugin descriptor
```

## Dependencies

| Dependency | Description |
|-----------|-------------|
| `antbot_libs` | Dynamixel communication and control table parsing |
| `hardware_interface` | ros2_control base classes |
| `pluginlib` | Dynamic plugin loading |
| `rclcpp` / `rclcpp_lifecycle` | ROS 2 C++ client |
| `dynamixel_sdk` | ROBOTIS Dynamixel SDK |
| `sensor_msgs` | BatteryState message |
| `std_msgs` | Float64MultiArray message |
| `std_srvs` | SetBool service (headlight) |
| `antbot_interfaces` | CargoCommand, WiperOperation services |

## Build

```bash
colcon build --symlink-install --packages-up-to antbot_hw_interface
```

## License

Apache License 2.0 — Copyright 2026 ROBOTIS AI CO., LTD.
