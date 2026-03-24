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
- **LiDAR** — 2D front/back (COIN D4), 3D top (Vanjee WLR-722)
- **Charging coil** — wireless charging contact

## Xacro Structure

| File | Description |
|------|-------------|
| `urdf/antbot.xacro` | Top-level robot definition, sensor mounting, and parameters |
| `urdf/base.xacro` | Base link with mesh and inertia |
| `urdf/wheel.xacro` | Swerve wheel module macro (steering + drive) |
| `urdf/ros2_control.xacro` | ros2_control hardware interface definition |
| `urdf/sensors.xacro` | Sensor frame mounting macros |

## Visualization

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
