# antbot_navigation

ROS 2 Nav2 navigation package for AntBot. This package supports saved-map navigation, SLAM-based map creation, and localization-only launches. It keeps physical robot settings (`mode:=real`) and simulation settings (`mode:=sim`) separated inside the same package.

AntBot uses a 4-wheel independent swerve-drive layout, but steering angle limits mean it does not behave like a fully holonomic robot in practice. The real robot prioritizes stability with RPP (Regulated Pure Pursuit), while simulation uses MPPI to test rollout-based path following.

## Prerequisites

The Nav2, SLAM Toolbox, robot localization, MPPI, and RPP controller dependencies are declared in `package.xml`, so they can be installed with rosdep from the workspace root.

```bash
rosdep install --from-paths src --ignore-src -r -y
```

To install them manually, use:

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-mppi-controller \
  ros-humble-nav2-regulated-pure-pursuit-controller
```

Then build the package:

```bash
colcon build --symlink-install --packages-select antbot_navigation
source install/setup.bash
```

All launch files use the `mode` argument to select the configuration directory and time source.

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=real map:=/path/to/map.yaml
ros2 launch antbot_navigation navigation.launch.py mode:=sim map:=/path/to/map.yaml
```

## Real Mode

This is the default mode for operating the physical robot. It favors simplicity and stability over sensor fusion, and directly uses the front LiDAR and swerve controller odometry.

### Real Launches

Saved-map navigation:

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=real map:=/path/to/map.yaml
```

Create a map with SLAM:

```bash
ros2 launch antbot_navigation slam.launch.py mode:=real
```

`slam.launch.py` does not include `localization.launch.py`. Instead of running `map_server` and AMCL against a saved map, `slam_toolbox` builds the map from LiDAR and `/odom` while publishing the `map -> odom` TF.

Localization only:

```bash
ros2 launch antbot_navigation localization.launch.py mode:=real map:=/path/to/map.yaml
```

Save a map created by SLAM:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Real Architecture

- **Controller**: RPP. If the heading error is large, the robot stops and rotates in place; otherwise, it follows the lookahead point.
- **Command DOF**: Real driving mostly avoids `vy` and uses `vx` and `wz`.
- **Odometry**: The swerve controller directly publishes `/odom` and the `odom -> base_link` TF.
- **EKF**: Not used by the current real launch.
- **LiDAR**: Uses only the front LiDAR (`/scan_0`).
- **Scan fix relay**: Normalizes the COIN D4 driver's variable-length LaserScan output to `/scan_0_fixed`.

The TF chain is:

```text
map -> odom -> base_link -> sensor_frames
```

| Transform | Publisher | Description |
|-----------|-----------|-------------|
| `map -> odom` | AMCL or SLAM Toolbox | Corrects the global pose by matching the map and LiDAR scan |
| `odom -> base_link` | Swerve controller | Robot motion estimated from wheel odometry |
| `base_link -> *` | `robot_state_publisher` | URDF-based sensor and wheel frames |

### Real Key Settings

The main configuration file is `config/real/nav2_params.yaml`.

| Item | Value |
|------|-------|
| Config directory | `config/real/` |
| `use_sim_time` | `false` |
| Controller plugin | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| AMCL motion model | `nav2_amcl::OmniMotionModel` |
| Odometry topic | `/odom` |
| LiDAR topic | `/scan_0_fixed` |
| Velocity smoother | `[0.30, 0.02, 0.3]` |
| Costmap inflation | `0.5m` |

Key RPP parameters:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `desired_linear_vel` | `0.30` | Target linear velocity |
| `lookahead_dist` | `0.6` | Default lookahead distance |
| `min_lookahead_dist` | `0.4` | Minimum lookahead distance kept at low speed |
| `max_lookahead_dist` | `1.0` | Maximum lookahead distance |
| `rotate_to_heading_angular_vel` | `0.5` | In-place rotation speed |
| `rotate_to_heading_min_angle` | `0.785` | Rotate in place when heading error is about 45 degrees or greater |
| `regulated_linear_scaling_min_speed` | `0.10` | Minimum speed during curvature-based slowdown |
| `max_angular_accel` | `0.5` | Maximum angular acceleration |

If `rotate_to_heading_min_angle` is too low, the robot may stop and rotate even on gentle curves. The current value is about 45 degrees, so small curves are followed while driving and only larger heading changes trigger in-place rotation.

### Real Sensor Topics

| Topic | Source | Usage |
|-------|--------|-------|
| `/scan_0` | Raw front COIN D4 LiDAR | Corrected by `scan_fix_relay` because the number of ranges can vary per frame |
| `/scan_0_fixed` | Corrected fixed 400-point scan | Real AMCL, SLAM, and costmap input |
| `/odom` | Swerve controller | Nav2 odometry and TF reference |
| `/imu/accel_gyro` | Physical IMU | Not used by the current real launch |

`scan_fix_relay.py` republishes the variable-length range array from `/scan_0` as `/scan_0_fixed`, a fixed 400-point LaserScan. The raw scan can vary in point count from frame to frame, and its angle metadata may not match the actual range count, so real-mode AMCL, SLAM, and costmaps use the corrected topic.

### Real Costmap

| Item | Local Costmap | Global Costmap |
|------|---------------|----------------|
| Frame | `odom` | `map` |
| Size | `5m x 5m`, rolling window | Full map |
| Update frequency | `5Hz` | `1Hz` |
| Layers | front obstacle + inflation | static + front obstacle + inflation |
| Footprint | `0.70m x 0.60m` | `0.70m x 0.60m` |

## Sim Mode

This is the Gazebo-based test mode. It uses more aggressive speeds than the real robot, a dual-LiDAR costmap, and MPPI as the controller.

### Sim Launches

Run the Gazebo simulation in terminal 1.

```bash
ros2 launch antbot_gazebo gazebo.launch.py world:=depot
```

Run the saved-map Nav2 stack in terminal 2.

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim map:=/path/to/depot_sim.yaml
```

Run RViz in terminal 3.

```bash
rviz2 -d $(ros2 pkg prefix antbot_navigation --share)/rviz/navigation.rviz \
  --ros-args -p use_sim_time:=true
```

Create a map with SLAM:

```bash
ros2 launch antbot_navigation slam.launch.py mode:=sim
```

`slam.launch.py` does not include `localization.launch.py`. Instead of AMCL localization against a saved map, `slam_toolbox` creates the map in real time while publishing the `map -> odom` TF.

Localization only:

```bash
ros2 launch antbot_navigation localization.launch.py mode:=sim map:=/path/to/map.yaml
```

Gazebo usually takes about 8-15 seconds to open and load the ros2_control controllers. In RViz, set the initial pose with `2D Pose Estimate`, then choose a target with `Nav2 Goal`.

### Sim Architecture

- **Controller**: MPPI. It samples multiple candidate trajectories and selects the best command using critics.
- **Command DOF**: It can generate `vx`, `vy`, and `wz`, but `vy_max` is kept low to encourage forward driving and turning instead of crab-walking.
- **LiDAR**: Uses `/scan_0` and `/scan_1` in the costmap obstacle layers.
- **Costmap**: Uses larger inflation than real mode to account for steering overshoot and a wider turning radius.

Based on the current launch files, `config/sim/ekf.yaml` exists but `localization.launch.py` does not start an EKF node. To re-enable EKF, also revisit the `robot_localization/ekf_node` launch and which node publishes the `odom -> base_link` TF.

### Sim Key Settings

The main configuration file is `config/sim/nav2_params.yaml`.

| Item | Value |
|------|-------|
| Config directory | `config/sim/` |
| `use_sim_time` | `true` |
| Controller plugin | `nav2_mppi_controller::MPPIController` |
| MPPI motion model | `Omni` |
| AMCL motion model | `nav2_amcl::OmniMotionModel` |
| Velocity smoother | `[1.5, 0.15, 1.5]` |
| Costmap inflation | `0.75m` |

Key MPPI parameters:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `vx_max` | `1.0` | Maximum forward velocity |
| `vy_max` | `0.1` | Lateral velocity limit. This mostly prevents crab-walking and encourages forward driving and turning |
| `wz_max` | `1.5` | Maximum angular velocity |
| `batch_size` | `2000` | Number of candidate trajectories evaluated at once |
| `time_steps` | `56` | Predicts 2.8 seconds ahead with `model_dt: 0.05` |
| `motion_model` | `Omni` | Internal Nav2 controller motion model |

Main critics:

| Critic | Weight | Purpose |
|--------|--------|---------|
| `PreferForwardCritic` | `15.0` | Prefers forward driving over reverse or lateral motion |
| `PathAngleCritic` | `15.0` | Aligns robot travel direction with the global path direction |
| `TwirlingCritic` | `10.0` | Suppresses unnecessary in-place rotation |
| `PathAlignCritic` | `10.0` | Scores alignment between candidate trajectories and the global path |
| `PathFollowCritic` | `5.0` | Penalizes deviation from the global path |
| `ObstaclesCritic` | `collision_cost: 10000.0` | Rejects collision trajectories |
| `ConstraintCritic` | `4.0` | Penalizes velocity and acceleration limit violations |

### Sim Sensor Topics

| Topic | Source | Usage |
|-------|--------|-------|
| `/scan_0` | Front LiDAR | Sim AMCL, SLAM, and costmap |
| `/scan_1` | Rear LiDAR | Sim costmap obstacle layer |
| `/odom` | Swerve controller odometry | Velocity smoother feedback |
| `/odometry/filtered` | Reserved topic for EKF output | Still present in the Sim BT navigator configuration |
| `/imu/data` | Simulated IMU | EKF input if EKF is used |

### Sim Costmap

| Item | Local Costmap | Global Costmap |
|------|---------------|----------------|
| Frame | `odom` | `map` |
| Size | `5m x 5m`, rolling window | Full map |
| Update frequency | `5Hz` | `1Hz` |
| Layers | front obstacle + back obstacle + inflation | static + front obstacle + back obstacle + inflation |
| Footprint | `0.70m x 0.60m` | `0.70m x 0.60m` |

## Common Usage

### Navigation Goal

For a single goal in RViz, set the initial pose with `2D Pose Estimate`, then click `Nav2 Goal`.

You can also send a goal from the CLI.

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0}}}}"
```

For multiple waypoints, add `Panels > Add New Panel > nav2_rviz_plugins/Navigation2`, enable Waypoint Mode, and select multiple goals.

### World and Map

`navigation.launch.py` can take a direct `map:=` argument. To auto-select a map from a world name such as `world:=depot`, register the world name and map file in `maps/worlds.yaml`.

```yaml
worlds:
  depot:
    sdf: depot.sdf
    map: depot_sim.yaml
```

After registration, launch with:

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim world:=depot
```

When adding a new world, place the `.sdf`, `.pgm`, and `.yaml` map files in `maps/`, then register them in `worlds.yaml`.

## Real / Sim Comparison

| Setting | `mode:=real` | `mode:=sim` |
|---------|--------------|-------------|
| Config directory | `config/real/` | `config/sim/` |
| `use_sim_time` | `false` | `true` |
| Controller | RPP | MPPI |
| Controller plugin | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | `nav2_mppi_controller::MPPIController` |
| Motion model | AMCL `OmniMotionModel` | MPPI `Omni`, AMCL `OmniMotionModel` |
| Command DOF | mostly `vx`, `wz` | `vx`, `vy`, `wz` |
| Max linear speed | RPP `desired_linear_vel: 0.30` | MPPI `vx_max: 1.0` |
| Velocity smoother | `[0.30, 0.02, 0.3]` | `[1.5, 0.15, 1.5]` |
| Odometry | swerve controller `/odom` | `/odom`, with some `/odometry/filtered` configuration still present |
| LiDAR | `/scan_0` -> `/scan_0_fixed` | `/scan_0`, `/scan_1` |
| Costmap inflation | `0.5m` | `0.75m` |

AMCL uses `nav2_amcl::OmniMotionModel` in both modes. AntBot's structure needs to account for holonomic motion, so the diff-drive `DifferentialMotionModel` does not represent lateral motion components well enough.

The DWB controller is not used in the current default configuration. AntBot is prone to `wz` oscillation and `vx` stalling with DWB, so real mode defaults to RPP and sim mode defaults to MPPI.

## Package Structure

```text
antbot_navigation/
├── config/
│   ├── real/
│   │   ├── nav2_params.yaml
│   │   ├── ekf.yaml
│   │   └── slam_toolbox_params.yaml
│   └── sim/
│       ├── nav2_params.yaml
│       ├── ekf.yaml
│       └── slam_toolbox_params.yaml
├── launch/
│   ├── slam.launch.py
│   ├── navigation.launch.py
│   └── localization.launch.py
├── maps/
│   ├── depot.sdf
│   ├── depot_sim.pgm
│   ├── depot_sim.yaml
│   └── worlds.yaml
├── scripts/
│   └── scan_fix_relay.py
└── rviz/
    └── navigation.rviz
```

## Troubleshooting

### `Failed to create plan`

This can happen when the robot's initial pose is wrong or the robot footprint overlaps an obstacle in the costmap.

- Reset the initial pose in RViz with `2D Pose Estimate`.
- Clear the global costmap.

```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
```

### Wall Collisions or Obstacle Avoidance Failures

This usually means the safety margin around obstacles is too small.

- Increase `inflation_radius`: real `0.5 -> 0.75`, sim `0.75 -> 1.0`
- Decrease `cost_scaling_factor`: for example, `1.5 -> 1.0`
- When using MPPI, increase `ObstaclesCritic.collision_margin_distance`: for example, `0.1 -> 0.2`

### Real: Robot Stops on Curves

With `use_rotate_to_heading: true`, RPP switches to stop-and-rotate behavior when the heading error exceeds `rotate_to_heading_min_angle`.

- Do not set `rotate_to_heading_min_angle` too low. The current recommended value is `0.785` rad.
- If `min_lookahead_dist` is too small, the robot can repeatedly stop, shrink the lookahead, and interpret the curve as sharper.
- Increase `rotate_to_heading_angular_vel` to finish in-place rotation faster.

### Sim: Excessive In-Place Rotation

- Increase the `TwirlingCritic` weight: `10.0 -> 15.0`
- Decrease `wz_max`: `1.5 -> 1.0`
- Tune the `PreferForwardCritic` weight

### TF Timeout or Missing `map` Frame

When Gazebo restarts, sim time resets to 0 and can invalidate the TF buffer. Restart Gazebo and Navigation together when this happens.

### Diagnostic Commands

```bash
# Check LiDAR receive rate
ros2 topic hz /scan_0

# Check corrected scan in real mode
ros2 topic hz /scan_0_fixed

# Check map -> odom TF
ros2 run tf2_ros tf2_echo map odom

# Check odom -> base_link TF
ros2 run tf2_ros tf2_echo odom base_link

# Check swerve controller odom TF publishing state
ros2 param get /antbot_swerve_controller enable_odom_tf

# Check Nav2 controller lifecycle state
ros2 lifecycle get /controller_server

# Check ros2_control controller state
ros2 control list_controllers
```
