# antbot_gazebo

Ignition Gazebo simulation package for AntBot swerve-drive robot.

## Prerequisites

```bash
sudo apt install ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-xacro ros-humble-robot-state-publisher
```

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to antbot_gazebo antbot_teleop
source install/setup.bash
```

## Quick Start

**Terminal 1** — Launch Gazebo simulation:
```bash
ros2 launch antbot_gazebo gazebo.launch.py
```

**Terminal 2** — Keyboard teleop:
```bash
ros2 run antbot_teleop teleop_keyboard
```

Specify a world by name (resolved via `config/worlds.yaml`) or full path:
```bash
ros2 launch antbot_gazebo gazebo.launch.py world:=depot
ros2 launch antbot_gazebo gazebo.launch.py world:=/path/to/world.sdf
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `empty` | World name (from `worlds.yaml`) or full path to SDF file |

## Package Structure

```
antbot_gazebo/
├── config/
│   ├── swerve_controller_gazebo.yaml   # Sim-specific controller params
│   └── worlds.yaml                     # World name → SDF path mapping
├── launch/
│   └── gazebo.launch.py                # Gazebo + robot spawn + controllers
├── urdf/
│   ├── antbot_sim.xacro                # Top-level sim URDF
│   ├── gazebo_plugins.xacro            # Sensor plugins (LiDAR, IMU)
│   └── ros2_control_gazebo.xacro       # IgnitionSystem hardware interface
└── worlds/
    ├── empty.sdf                       # Empty world (default)
    └── depot.sdf                       # Warehouse-style world
```

## Simulation vs Real Hardware

| Aspect | Real HW | Gazebo |
|--------|---------|--------|
| Hardware interface | BoardInterface | **IgnitionSystem** |
| Acceleration command | Supported | **Not supported** (velocity/position only) |
| 2D LiDAR | 2D LiDAR sensor | **gpu_lidar** (ogre2 required) |
| IMU | antbot_imu node | **Gazebo IMU plugin** |
| Control rate | 20 Hz | **100 Hz** |
| Scrub compensation | Required | **Not needed** |
| IK iterations | 0 | **3** (55mm offset correction) |
| Odom integration | rk4 | **analytic_swerve** |
| Odom smoothing | window: 1 | **window: 10** |

## Launch Sequence

```
gazebo.launch.py
  ├── Environment variables (IGN_GAZEBO_RESOURCE_PATH, PLUGIN_PATH)
  ├── xacro → URDF generation
  ├── worlds.yaml → resolve world name to SDF path
  ├── ign gazebo -r world.sdf
  ├── Robot spawn (x=0, y=0, z=0.15)
  ├── robot_state_publisher
  ├── [spawn_robot exit wait (OnProcessExit)]
  ├── controller_manager service wait
  ├── joint_state_broadcaster spawn
  ├── swerve_drive_controller spawn (after JSB completes)
  └── ros_gz_bridge (sensor topic bridging)
```

## URDF Structure

```
antbot_sim.xacro (simulation entry point)
  ├── antbot_description/  (shared — same as real HW)
  │    ├── sensors.xacro, base.xacro, wheel.xacro
  └── antbot_gazebo/  (sim-specific)
       ├── ros2_control_gazebo.xacro   IgnitionSystem
       └── gazebo_plugins.xacro        friction + sensors
```

### Hardware Interface

```
IgnitionSystem (ign_ros2_control)
  ├── Wheels (x4):    velocity cmd → velocity/position state
  └── Steering (x4):  position cmd → position/velocity state
```

> **Note**: IgnitionSystem does not support `acceleration` or `effort` command interfaces.
> Controller config must set `use_acceleration_command: false`.

### Friction Model

| Part | Friction coefficient | Description |
|------|---------------------|-------------|
| base_link | 0.2 | Low friction (sliding) |
| wheel (x4) | 1.8 | High friction (traction) |
| steering (x4) | 0.0 | No friction (free rotation) |

## Simulated Sensors

| Sensor | Plugin | Topic | Samples | Range | Noise |
|--------|--------|-------|---------|-------|-------|
| Front 2D LiDAR | `gpu_lidar` | `/scan_0` | 720 | 0.6-20m | stddev 0.008 m |
| Back 2D LiDAR | `gpu_lidar` | `/scan_1` | 720 | 0.6-20m | stddev 0.008 m |
| IMU | `imu_sensor` | `/imu/data` | — | — | angular 0.0003 rad/s, linear 0.02 m/s² |

> **Note**: `gpu_lidar` requires the **ogre2** render engine. Set `<render_engine>ogre2</render_engine>` in your world SDF.

## Controller Configuration

Config file: `config/swerve_controller_gazebo.yaml`

| Parameter | Sim value | HW value | Reason |
|-----------|-----------|----------|--------|
| `non_coaxial_ik_iterations` | **3** | 0 | Gazebo exposes 55mm offset error |
| `enable_steering_scrub_compensator` | **false** | true | No real scrub in Gazebo |
| `velocity_rolling_window_size` | **10** | 1 | Smooth sim encoder noise |
| `odom_integration_method` | **analytic_swerve** | rk4 | Exact for piecewise-constant |
| `use_acceleration_command` | **false** | true | IgnitionSystem limitation |

## Custom Worlds

### worlds.yaml

Simulation (SDF) and navigation (map) use separate `worlds.yaml` files:

| File | Purpose | Mapping |
|------|---------|---------|
| `antbot_gazebo/config/worlds.yaml` | Gazebo simulation | World name → SDF file |
| `antbot_navigation/maps/worlds.yaml` | Nav2 navigation | World name → map file |

```yaml
# antbot_gazebo/config/worlds.yaml
worlds:
  empty:
    sdf: empty.sdf
  depot:
    sdf: depot.sdf        # relative to antbot_gazebo/worlds/
```

### Registering a New World

1. Place the SDF file in `antbot_gazebo/worlds/`
2. Add an entry to `antbot_gazebo/config/worlds.yaml`
3. If Nav2 navigation is needed:
   - Build a map with SLAM: `ros2 launch antbot_navigation slam.launch.py mode:=sim`
   - Save it: `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_world`
   - Copy `.pgm` and `.yaml` to `antbot_navigation/maps/`
   - Register in `antbot_navigation/maps/worlds.yaml`

### World SDF Requirements

- `render_engine: ogre2` (required for gpu_lidar)
- Physics, Sensors, UserCommands, SceneBroadcaster plugins
- ground_plane + sun (lighting)

## Troubleshooting

- **gpu_lidar returns only range_min** — Check that `render_engine` is `ogre2`. Using `ogre` causes all rays to return minimum values.
- **Controller activation failure** — Run `ros2 control list_controllers`. If stuck in `unconfigured`, check Gazebo logs for `gz_ros2_control` plugin errors.
- **Odometry drift** — `non_coaxial_ik_iterations` set to 0 causes drift from the 55mm steering-wheel offset. Set to 2-3 in simulation.
