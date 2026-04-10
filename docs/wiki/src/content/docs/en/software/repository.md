---
title: 3.1 Open Source Repository
description: AntBot GitHub Open Source Repository
sidebar:
  order: 1
---

The ROS 2 drivers and control packages for operating AntBot are open-sourced under the **Apache License 2.0**.

## GitHub Repository

- **Repository:** [github.com/ROBOTIS-move/antbot](https://github.com/ROBOTIS-move/antbot)
- **License:** Apache License 2.0
- **Tech Stack:** ROS 2 Humble, C++17, Python 3, Ubuntu 22.04

## Repository Structure

```bash
antbot/
|-- antbot/                        # Meta package
|-- antbot_bringup/                # Launch files + configuration
|-- antbot_description/            # URDF / Xacro robot model
|-- antbot_swerve_controller/      # Swerve drive controller
|-- antbot_hw_interface/           # ros2_control hardware interface
|-- antbot_libs/                   # Shared C++ libraries
|-- antbot_interfaces/             # Custom message/service definitions
|-- antbot_camera/                 # Multi-camera driver
|-- antbot_imu/                    # IMU driver
|-- antbot_teleop/                 # Keyboard/joystick teleoperation
|-- vanjee_lidar_sdk/              # Vanjee 3D LiDAR driver (external)
|-- vanjee_lidar_msg/              # LiDAR message definitions (external)
|-- setting.sh                     # Dependency installation script
+-- additional_repos.repos         # External repository list (for vcs import)
```
