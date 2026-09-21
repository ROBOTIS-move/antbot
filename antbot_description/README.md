# antbot_description

URDF/xacro robot description for the ANTBot 4-Wheel Independent Swerve Drive delivery robot.

## Robot Model

### Joints

| Joint | Type | Axis | Limits |
|-------|------|------|--------|
| `steering_{corner}_joint` | revolute | Z | -90° ~ +90° |
| `wheel_{corner}_joint` | continuous | Y | — |

where `{corner}` = `front_left`, `front_right`, `rear_left`, `rear_right`

### Sensor Frames

The model defines TF frames for the following sensors:

- **Cameras** — stereo depth (front), 4x mono (left/front/right/back)
- **IMU** — 6-axis inertial measurement unit
- **GNSS** — u-blox GPS receiver
- **Magnetometer**
- **LiDAR** — 2D front/back, 3D top
- **Charging coil** — wireless charging contact

## Xacro Structure

| File | Description |
|------|-------------|
| `urdf/antbot.xacro` | Top-level robot definition, sensor mounting, and parameters |
| `urdf/base.xacro` | Base link with mesh and inertia |
| `urdf/wheel.xacro` | Swerve wheel module macro (steering + drive) |
| `urdf/ros2_control.xacro` | ros2_control hardware interface definition |
| `urdf/sensors.xacro` | Sensor frame mounting macros |

## Sensor Calibration

`~/ANTBOT/calibration.yaml` stores per-robot sensor extrinsic calibration data measured during production. `antbot_bringup` passes this file to `antbot.xacro` through the `calibration_yaml_path` xacro argument when it exists.

| Fields | Meaning | Unit |
|--------|---------|------|
| `tx`, `ty`, `tz` | Sensor X, Y, Z position relative to `base_link` | meters |
| `rx`, `ry`, `rz` | Sensor roll, pitch, yaw relative to `base_link` | radians |

The `CalibratedSensors` macro replaces the default joint origin with these values. If no file is supplied or a sensor entry is absent, the corresponding URDF defaults are used. Each provided entry must contain all six fields.

Production calibration entries are `camera_front_extrinsic`, `camera_left_extrinsic`, `camera_right_extrinsic`, `lidar_2d_front_extrinsic`, and `lidar_2d_back_extrinsic`. Camera keys refer to the mono camera mounting frames; optical frame transforms are applied separately.

See the wiki for production data examples: [English](../docs/wiki/src/content/docs/en/hardware/sensor-coordinates.mdx#sensor-calibration) / [한국어](../docs/wiki/src/content/docs/hardware/sensor-coordinates.mdx#센서-캘리브레이션).

## Visualization

> **Note:** `description.launch.py` previews the default model and does not automatically load `~/ANTBOT/calibration.yaml`. Calibration is loaded by `antbot_bringup/robot_state_publisher.launch.py` during robot bringup.

```bash
# View in RViz with interactive joint sliders
ros2 launch antbot_description description.launch.py

# Launch arguments
ros2 launch antbot_description description.launch.py use_rviz:=true use_joint_state_publisher_gui:=true
```

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation clock |
| `use_joint_state_publisher` | `false` | Enable joint_state_publisher |
| `use_joint_state_publisher_gui` | `true` | Enable GUI joint sliders |
| `use_rviz` | `true` | Launch RViz |

## Meshes

| File | Description |
|------|-------------|
| `meshes/p38f1_body_asm.stl` | Robot body/chassis |
| `meshes/inwheel_mdh150_2_1.stl` | In-wheel motor assembly |

## Dependencies

| Dependency | Description |
|-----------|-------------|
| `urdf` | URDF parser |
| `xacro` | Xacro macro processor |
| `robot_state_publisher` | URDF → TF broadcast |
| `joint_state_publisher` | Joint state publishing |
| `joint_state_publisher_gui` | Interactive joint visualization |

## Build

```bash
colcon build --symlink-install --packages-select antbot_description
```

## License

Apache License 2.0 — Copyright 2026 ROBOTIS AI CO., LTD.
