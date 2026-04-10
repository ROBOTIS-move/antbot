# antbot_navigation

Nav2 navigation stack integration for AntBot swerve-drive robot.

AntBot can perform autonomous navigation using Nav2, the official ROS 2 navigation framework.
To run in simulation, [antbot_gazebo](../antbot_gazebo/README.md) setup must be completed first.

## Prerequisites

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-mppi-controller

colcon build --symlink-install --packages-select antbot_navigation
source install/setup.bash
```

## Launch Types

| Launch file | Purpose | Features |
|-------------|---------|----------|
| `navigation.launch.py` | Autonomous driving with saved map | AMCL + full Nav2 stack |
| `slam.launch.py` | Build map while navigating | SLAM Toolbox (no pre-built map needed) |
| `localization.launch.py` | Localization only | No path planning |

All launch files accept a `mode` argument to switch between simulation and real robot:

```bash
ros2 launch antbot_navigation slam.launch.py mode:=sim    # Simulation
ros2 launch antbot_navigation slam.launch.py mode:=real   # Real robot
```

## Simulation Mode

**Terminal 1** — Gazebo simulation:
```bash
ros2 launch antbot_gazebo gazebo.launch.py world:=depot
```

**Terminal 2** — Nav2 navigation:
```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim world:=depot
```

**Terminal 3** — RViz visualization:
```bash
rviz2 -d $(ros2 pkg prefix antbot_navigation --share)/rviz/navigation.rviz \
  --ros-args -p use_sim_time:=true
```

Save map: `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map`

Navigate with saved map:
```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim map:=/path/to/my_map.yaml
```

## Real Robot Mode

**Terminal 1** — AntBot bringup:
```bash
ros2 launch antbot_bringup bringup.launch.py
```

**Terminal 2** — Nav2 navigation:
```bash
ros2 launch antbot_navigation navigation.launch.py mode:=real
```

**Terminal 3** — RViz visualization (from remote PC):
```bash
rviz2 -d $(ros2 pkg prefix antbot_navigation --share)/rviz/navigation.rviz
```

## Sim / Real Mode

| Setting | `mode:=sim` | `mode:=real` |
|---------|-------------|--------------|
| Config directory | `config/sim/` | `config/real/` |
| `use_sim_time` | `true` | `false` |
| MPPI `vx_max` | 1.0 m/s | 2.0 m/s |
| Velocity smoother | [1.5, 0.15, 1.5] | [1.0, 0.10, 1.0] |
| EKF IMU topic | `/imu/data` | `/imu/accel_gyro` |
| EKF process noise | Low (ideal sensors) | Higher (real noise) |
| MPPI `batch_size` | 2000 | 1500 (Jetson Orin) |

## Package Structure

```
antbot_navigation/
├── config/
│   ├── sim/                  # Gazebo simulation configs
│   │   ├── nav2_params.yaml
│   │   ├── ekf.yaml
│   │   └── slam_toolbox_params.yaml
│   └── real/                 # Real robot configs
│       ├── nav2_params.yaml
│       ├── ekf.yaml
│       └── slam_toolbox_params.yaml
├── launch/
│   ├── slam.launch.py
│   ├── navigation.launch.py
│   └── localization.launch.py
├── maps/
│   └── depot_sim.yaml
└── rviz/
    └── navigation.rviz
```

## AntBot vs Standard diff-drive

| Aspect | Standard diff-drive | AntBot swerve |
|--------|---------------------|---------------|
| Controller | DWB | **MPPI** (rollout-based optimization) |
| Motion Model (AMCL) | DifferentialMotionModel | **OmniMotionModel** |
| Velocity DOF | vx, wz (2DOF) | **vx, vy, wz (3DOF)** |
| Costmap Inflation | ~0.3m | **0.75m** (steering overshoot margin) |
| LiDAR | Single | **Dual 2D** (front + back) |
| Odometry TF | Direct publish | **EKF sensor fusion** (collision protection) |

## System Architecture

### TF Tree

| Transform | Publisher | Input | Output |
|-----------|----------|-------|--------|
| `map → odom` | AMCL or SLAM Toolbox | LiDAR scans (`/scan_0`) + map data | Global position correction |
| `odom → base_link` | EKF (Nav2 mode) or swerve controller (standalone) | Wheel odometry (`/odom`) + IMU | Robot displacement estimation |
| `base_link → *` | robot_state_publisher | URDF model | Sensor/wheel frame positions |

### odom TF Handover

> **Warning**: If both swerve controller and EKF publish `odom→base_link` TF simultaneously, jitter occurs.
> Navigation launch auto-disables the controller's TF publish, and Nav2 nodes start with an 8-second delay to allow EKF to establish TF first.
>
> Manual override: `ros2 param set /antbot_swerve_controller enable_odom_tf false`

## Parameter Tuning

Config files: `config/{sim,real}/`

| Component | Role | Config File |
|-----------|------|-------------|
| **MPPI Controller** | Generates candidate paths and selects the optimal one | `nav2_params.yaml` |
| **AMCL** | Compares LiDAR scans against the map for localization | `nav2_params.yaml` |
| **EKF** | Fuses wheel odometry and IMU for position accuracy | `ekf.yaml` |
| **SLAM Toolbox** | Builds a map in real-time while estimating position | `slam_toolbox_params.yaml` |
| **Costmap** | Builds an obstacle grid from sensor data | `nav2_params.yaml` |

### MPPI Controller

MPPI simulates thousands of candidate paths and selects the highest-scoring one. AntBot's steering is limited to **±60°**, so direction changes use a **rotate-in-place → drive-forward** pattern.

**Velocity Parameters**

| Parameter | Sim | Real | Description |
|-----------|-----|------|-------------|
| `vx_max` | 1.0 | 2.0 | Max forward speed (m/s) |
| `vy_max` | 0.1 | 0.5 | Max lateral speed (m/s). Low = forward-driving behavior |
| `wz_max` | 1.5 | 2.0 | Max rotation speed (rad/s) |
| `batch_size` | 2000 | 1500 | Candidate paths per cycle |
| `time_steps` | 56 | 56 | Lookahead steps (× model_dt 0.05s = 2.8s horizon) |

**MPPI Critics — Swerve Direction Control**

| Critic | Sim | Real | Description |
|--------|-----|------|-------------|
| `PreferForwardCritic` | 15.0 | 5.0 | Encourages forward-facing movement |
| `PathAngleCritic` | 15.0 | 5.0 | Aligns robot heading with path direction |
| `TwirlingCritic` | 10.0 | 5.0 | Suppresses unnecessary spinning |

Sim weights are higher because frictionless simulation causes the robot to slide easily. Real robot tire-ground friction naturally encourages straight driving.

**MPPI Critics — Path Following & Safety**

| Critic | Weight | Description |
|--------|--------|-------------|
| `GoalCritic` | 5.0 | Rewards paths closer to the goal |
| `PathAlignCritic` | 10.0 | Evaluates alignment with global path |
| `PathFollowCritic` | 5.0 | Prevents deviation from global path |
| `ObstaclesCritic` | — | Penalizes near-obstacle paths (`collision_cost: 10000`) |
| `ConstraintCritic` | 4.0 | Penalizes velocity/acceleration limit violations |

### AMCL

```yaml
amcl:
  robot_model_type: "nav2_amcl::OmniMotionModel"
  scan_topic: /scan_0
```

> **Important**: `OmniMotionModel` is required. `DifferentialMotionModel` cannot recognize lateral movement (vy), degrading localization accuracy on swerve robots.

### EKF Sensor Fusion

| Setting | Sim | Real | Description |
|---------|-----|------|-------------|
| Odometry topic | `/odom` | `/odom` | Velocity from wheels (vx, vy, vyaw) |
| IMU topic | `/imu/data` | `/imu/accel_gyro` | Gyro/accelerometer (yaw, vyaw) |
| Process noise | Low | High | Real sensors have more noise |
| `odom0_rejection_threshold` | 2.0 | 1.5 | Ignores odometry spikes (e.g. wall collision) |

### SLAM Toolbox

| Parameter | Sim | Real | Description |
|-----------|-----|------|-------------|
| `max_laser_range` | 20.0 | 12.0 | Max LiDAR range (m) |
| `minimum_travel_distance` | 0.5 | 0.3 | Min movement before adding scan (m) |
| `minimum_travel_heading` | 0.5 | 0.4 | Min rotation before adding scan (rad) |
| `resolution` | 0.05 | 0.05 | Map grid resolution (m/pixel) |
| `map_update_interval` | 5.0 | 5.0 | Map update period (seconds) |

### Costmap

Dual 2D LiDARs — front (`/scan_0`) and rear (`/scan_1`) — for 360° obstacle detection.

- **Robot footprint**: 0.70m × 0.60m
- **Inflation radius**: 0.75m (steering overshoot margin)

| Aspect | Local Costmap | Global Costmap |
|--------|---------------|----------------|
| Reference frame | `odom` | `map` |
| Size | 5m × 5m | Full map |
| Update rate | 5 Hz | 1 Hz |
| Layers | obstacle ×2 + inflation | static + obstacle ×2 + inflation |

## Sensor Topics

| Topic | Type | Source |
|-------|------|--------|
| `/scan_0` | `LaserScan` | Front 2D LiDAR |
| `/scan_1` | `LaserScan` | Back 2D LiDAR |
| `/odom` | `Odometry` | Swerve controller |
| `/imu/accel_gyro` | `Imu` | IMU sensor |

## Troubleshooting

- **"Failed to create plan"** — Robot is inside an obstacle on the costmap. Use **2D Pose Estimate** in RViz, or clear costmap: `ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap`
- **Wall collision / avoidance failure** — Increase `inflation_radius` (0.75 → 1.0), decrease `cost_scaling_factor` (1.5 → 1.0), increase `ObstaclesCritic.collision_margin_distance` (0.1 → 0.2)
- **Excessive spinning in place** — Increase `TwirlingCritic` weight (10.0 → 15.0), decrease `wz_max` (1.5 → 1.0)
- **Missing map frame / TF timeout** — Gazebo restart resets sim time. Always restart Gazebo and Navigation together.
- **Odom drift after collision** — EKF `odom0_rejection_threshold` filters spikes. Re-localize if needed.
- **TF jitter** — Navigation launch auto-disables swerve controller's odom TF (EKF takes over). Manual: `ros2 param set /antbot_swerve_controller enable_odom_tf false`

### Diagnostic Commands

```bash
ros2 topic hz /scan_0                                      # LiDAR rate (expected ~10Hz)
ros2 topic hz /odometry/filtered                           # EKF output (expected ~50Hz)
ros2 run tf2_ros tf2_echo map odom                         # map → odom TF status
ros2 param get /antbot_swerve_controller enable_odom_tf    # odom TF publish status
ros2 lifecycle get /controller_server                      # Nav2 controller lifecycle
ros2 control list_controllers                              # ros2_control status
```
