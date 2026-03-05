# antbot_imu

ROS 2 node for the ANTBot onboard IMU sensor. Reads 6-axis inertial data (accelerometer + gyroscope) from the IMU board via Dynamixel Protocol 2.0, applies a complementary filter for orientation estimation, and publishes `sensor_msgs/Imu` messages.

## Features

- **6-axis IMU** — 3-axis accelerometer + 3-axis gyroscope
- **Complementary filter** — Fuses accelerometer (low-pass) and gyroscope (high-pass) for Roll/Pitch estimation
- **Auto-calibration** — Computes gyroscope bias and gravity reference from static samples at startup
- **Motion-aware calibration** — Pauses sample collection while the robot is moving (via odom subscription)

## Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `imu/accel_gyro` | `sensor_msgs/Imu` | 200 Hz (configurable) | Orientation (quaternion), angular velocity, linear acceleration |

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `odom` | `nav_msgs/Odometry` | Odometry for motion detection during calibration |

## Parameters

Configured via [config/imu.yaml](config/imu.yaml):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `imu_board.port` | `/dev/ttyUSB1` | Serial port for the IMU board |
| `imu_board.id` | `200` | Dynamixel device ID |
| `imu_board.baud_rate` | `4000000` | Serial baud rate (bps) |
| `imu_board.protocol_version` | `2.0` | Dynamixel protocol version |
| `imu.frame_id` | `imu_link` | TF frame ID for IMU messages |
| `imu.publish_rate` | `200.0` | Publish rate in Hz |
| `imu.calibration_num` | `350` | Number of samples for calibration |
| `imu.filter_cutoff_hz` | `10.0` | Complementary filter cutoff frequency (Hz) |
| `imu.scale.acceleration` | `0.00006103515625` | Accelerometer scale (g/LSB) |
| `imu.scale.angular_vel` | `0.0609756098` | Gyroscope scale (deg/s per LSB) |
| `control_table_path` | *(set by launch)* | Path to IMU control table XML |

## Usage

### Launch

```bash
ros2 launch antbot_imu imu.launch.py
```

**Launch arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `control_table_path` | `<pkg_share>/config/control_table.xml` | IMU control table XML path |
| `imu_param` | `<pkg_share>/config/imu.yaml` | Parameter file path |

### Verify output

```bash
ros2 topic echo /imu_node/imu/accel_gyro
```

## Calibration

On startup, the node runs a background calibration thread that:

1. Waits for odometry data to confirm the robot is stationary
2. Collects `calibration_num` (default: 350) static IMU samples
3. Computes average gyroscope bias (X, Y, Z) and gravity reference magnitude
4. If gyro Z bias exceeds threshold, retries calibration (up to 3 times)

If the robot moves during calibration (detected via odometry), collected samples are discarded and collection restarts.

## Control Table

The IMU board register map is defined in [config/control_table.xml](config/control_table.xml).

## Package Structure

```
antbot_imu/
├── include/antbot_imu/
│   └── imu_node.hpp          # ImuNode class definition
├── src/
│   ├── imu_node.cpp           # ImuNode implementation
│   └── main.cpp               # Entry point
├── launch/
│   └── imu.launch.py          # Launch file
└── config/
    ├── imu.yaml               # Default parameters
    └── control_table.xml      # IMU register map
```

## Dependencies

| Dependency | Description |
|-----------|-------------|
| `antbot_libs` | Dynamixel communication and control table parsing |
| `rclcpp` | ROS 2 C++ client library |
| `sensor_msgs` | Imu message type |
| `nav_msgs` | Odometry for motion detection |
| `tf2` | Quaternion conversion |
| `dynamixel_sdk` | ROBOTIS Dynamixel SDK |

## Build

```bash
colcon build --symlink-install --packages-up-to antbot_imu
```

## License

Apache License 2.0 — Copyright 2026 ROBOTIS AI CO., LTD.
