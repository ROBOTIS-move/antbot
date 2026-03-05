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
| `lidar_2d.launch.py` | Dual COIN D4 2D LiDAR (USB serial) |
| `lidar_3d.launch.py` | Vanjee WLR-722 3D LiDAR (Ethernet) |
| `view.launch.py` | RViz2 visualization with `rviz/antbot.rviz` config |

### Startup Sequence (`bringup.launch.py`)

```
bringup.launch.py
├── robot_state_publisher.launch.py  (URDF → /tf, /tf_static)
├── controller.launch.py             (ros2_control + swerve drive)
├── antbot_imu / imu.launch.py       (IMU sensor)
├── lidar_2d.launch.py               (2x COIN D4)
├── lidar_3d.launch.py               (Vanjee WLR-722)
├── ublox_gps / ublox_gps_node       (GNSS/GPS)
└── antbot_camera / camera.launch.py (Camera)
```

## Configuration

| File | Description |
|------|-------------|
| `config/lidar_3d.yaml` | Vanjee WLR-722 3D LiDAR settings (UDP, 192.168.6.x subnet) |
| `rviz/antbot.rviz` | RViz2 visualization settings for `view.launch.py` |

Sensor-specific parameters (board, IMU, swerve controller) are loaded from their respective packages.

## Dependencies

| Dependency | Description |
|-----------|-------------|
| `antbot_description` | URDF/xacro robot model |
| `antbot_hw_interface` | ros2_control hardware plugin |
| `antbot_imu` | IMU driver node |
| `antbot_swerve_controller` | Swerve drive controller |
| `controller_manager` | ros2_control controller manager |
| `coin_d4_driver` | COIN D4 2D LiDAR driver |
| `vanjee_lidar_sdk` | Vanjee 3D LiDAR driver |
| `ublox_gps` | u-blox GNSS/GPS driver |

## Build

```bash
colcon build --symlink-install --packages-select antbot_bringup
```

## License

Apache License 2.0 — Copyright 2026 ROBOTIS AI CO., LTD.
