# antbot_bringup

Bringup launch files for the ANTBot swerve-drive delivery robot. Provides modular launch configurations for bringing up all hardware drivers, controllers, and sensors.

## Launch Files

### Full System

```bash
# Start all hardware and sensors
ros2 launch antbot_bringup bringup.launch.py
```

### Individual Components

| Launch File | Description |
|-------------|-------------|
| `bringup.launch.py` | Full system — includes all components below |
| `robot_state_publisher.launch.py` | URDF → TF broadcast via `robot_state_publisher` |
| `controller.launch.py` | ros2_control node + joint_state_broadcaster + swerve_drive_controller |
| `lidar_2d.launch.py` | Dual 2D LiDAR (USB serial) |
| `lidar_3d.launch.py` | 3D LiDAR (Ethernet) |
| `view.launch.py` | RViz2 visualization with `rviz/antbot.rviz` config |

### Startup Sequence (`bringup.launch.py`)

```
bringup.launch.py
├── robot_state_publisher.launch.py  (URDF → /tf, /tf_static)
├── controller.launch.py             (ros2_control + swerve drive)
├── antbot_imu / imu.launch.py       (IMU sensor)
├── lidar_2d.launch.py               (2x 2D LiDAR)
├── lidar_3d.launch.py               (3D LiDAR)
├── ublox_gps / ublox_gps_node       (GNSS/GPS)
└── antbot_camera / camera.launch.py (Camera)
```

## Configuration

| File | Description |
|------|-------------|
| `~/ANTBOT/calibration.yaml` | Per-robot sensor extrinsic calibration data measured during production |
| `config/lidar_3d.yaml` | 3D LiDAR settings (UDP, 192.168.6.x subnet) |
| `rviz/antbot.rviz` | RViz2 visualization settings for `view.launch.py` |

Sensor-specific parameters (board, IMU, swerve controller) are loaded from their respective packages.

### Sensor Calibration

`robot_state_publisher.launch.py` automatically reads `~/ANTBOT/calibration.yaml` from the launching user's home directory and passes it to `antbot_description`. This also applies during full system bringup.

The file defines sensor positions (`tx`, `ty`, `tz`, in meters) and orientations (`rx`, `ry`, `rz`, as roll, pitch, yaw in radians) relative to `base_link`. These values replace the URDF defaults. If the file or a sensor entry is absent, the corresponding default coordinates are used. Each provided sensor entry requires all six fields.

See the wiki for the file format and production data examples: [English](../docs/wiki/src/content/docs/en/hardware/sensor-coordinates.mdx#sensor-calibration) / [한국어](../docs/wiki/src/content/docs/hardware/sensor-coordinates.mdx#센서-캘리브레이션).

## Dependencies

| Dependency | Description |
|-----------|-------------|
| `antbot_description` | URDF/xacro robot model |
| `antbot_hw_interface` | ros2_control hardware plugin |
| `antbot_imu` | IMU driver node |
| `antbot_swerve_controller` | Swerve drive controller |
| `controller_manager` | ros2_control controller manager |
| `coin_d4_driver` | 2D LiDAR driver |
| `vanjee_lidar_sdk` | 3D LiDAR driver |
| `ublox_gps` | u-blox GNSS/GPS driver |

## Build

```bash
colcon build --symlink-install --packages-select antbot_bringup
```

## License

Apache License 2.0 — Copyright 2026 ROBOTIS AI CO., LTD.
