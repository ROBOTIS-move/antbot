# antbot_navigation

AntBot용 ROS 2 Nav2 네비게이션 패키지입니다. 저장된 맵 기반 자율주행, SLAM 기반 맵 생성, 위치 추정 전용 실행을 지원하며, 실제 로봇(`mode:=real`)과 시뮬레이션(`mode:=sim`) 설정을 한 패키지 안에서 분리해 관리합니다.

AntBot은 4-wheel independent swerve-drive 구조이지만 스티어링 각도 제한 때문에 완전한 횡이동 로봇처럼 동작하지 않습니다. 실제 로봇에서는 안정성을 우선해 RPP(Regulated Pure Pursuit)를 사용하고, 시뮬레이션에서는 MPPI를 사용해 rollout 기반 경로 추종을 테스트합니다.

## Prerequisites

Nav2, SLAM Toolbox, robot localization, MPPI, RPP 컨트롤러 패키지가 필요합니다.

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-nav2-mppi-controller \
  ros-humble-nav2-regulated-pure-pursuit-controller

colcon build --symlink-install --packages-select antbot_navigation
source install/setup.bash
```

모든 launch 파일은 `mode` 인자로 설정 디렉토리와 시간 기준을 선택합니다.

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=real map:=/path/to/map.yaml
ros2 launch antbot_navigation navigation.launch.py mode:=sim map:=/path/to/map.yaml
```

## Real Mode

실제 로봇 운용을 위한 기본 모드입니다. 센서 퓨전보다 단순성과 안정성을 우선하며, 전방 LiDAR와 swerve controller odometry를 직접 사용합니다.

### Real 실행

저장된 맵 기반 자율주행:

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=real map:=/path/to/map.yaml
```

SLAM으로 맵 생성:

```bash
ros2 launch antbot_navigation slam.launch.py mode:=real
```

`slam.launch.py`는 `localization.launch.py`를 함께 실행하지 않습니다. 저장된 맵을 읽는 `map_server`와 AMCL 대신, `slam_toolbox`가 LiDAR와 `/odom`을 사용해 맵을 만들면서 `map -> odom` TF를 발행합니다.

위치 추정 전용:

```bash
ros2 launch antbot_navigation localization.launch.py mode:=real map:=/path/to/map.yaml
```

SLAM으로 생성한 맵 저장:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Real 아키텍처

- **Controller**: RPP. Heading 오차가 크면 정지 후 제자리 회전하고, 그 외에는 lookahead point를 따라 주행합니다.
- **Command DOF**: 실제 주행에서는 `vy`를 거의 사용하지 않고 `vx`, `wz` 중심으로 이동합니다.
- **Odometry**: swerve controller가 `/odom`과 `odom -> base_link` TF를 직접 발행합니다.
- **EKF**: 현재 real launch에서는 사용하지 않습니다.
- **LiDAR**: 전방 LiDAR(`/scan_0`)만 사용합니다.
- **Scan fix relay**: COIN D4 드라이버의 가변 길이 LaserScan을 `/scan_0_fixed`로 정규화합니다.

TF 체인은 다음과 같습니다.

```text
map -> odom -> base_link -> sensor_frames
```

| Transform | Publisher | Description |
|-----------|-----------|-------------|
| `map -> odom` | AMCL 또는 SLAM Toolbox | 맵과 LiDAR 스캔을 비교해 글로벌 위치 보정 |
| `odom -> base_link` | Swerve controller | 바퀴 오도메트리 기반 로봇 이동량 |
| `base_link -> *` | `robot_state_publisher` | URDF 기반 센서/바퀴 프레임 |

### Real 핵심 설정

주요 설정 파일은 `config/real/nav2_params.yaml`입니다.

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

RPP 핵심 파라미터:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `desired_linear_vel` | `0.30` | 목표 선속도 |
| `lookahead_dist` | `0.6` | 기본 lookahead 거리 |
| `min_lookahead_dist` | `0.4` | 저속에서도 유지하는 최소 lookahead |
| `max_lookahead_dist` | `1.0` | 최대 lookahead 거리 |
| `rotate_to_heading_angular_vel` | `0.5` | 제자리 회전 속도 |
| `rotate_to_heading_min_angle` | `0.785` | 약 45도 이상 heading 오차에서 제자리 회전 |
| `regulated_linear_scaling_min_speed` | `0.10` | 곡률 기반 감속 시 최소 속도 |
| `max_angular_accel` | `0.5` | 최대 각가속도 |

`rotate_to_heading_min_angle`이 너무 낮으면 완만한 커브에서도 멈춰서 회전하려고 합니다. 현재 값은 약 45도로, 작은 커브는 주행하면서 따라가고 큰 방향 전환만 제자리 회전하도록 맞춰져 있습니다.

### Real 센서 토픽

| Topic | Source | Usage |
|-------|--------|-------|
| `/scan_0` | 전방 COIN D4 LiDAR 원본 | 원본 scan의 range 개수가 프레임마다 달라 `scan_fix_relay`로 보정 |
| `/scan_0_fixed` | 고정 400포인트 보정 scan | Real AMCL/SLAM/costmap 입력 |
| `/odom` | swerve controller | Nav2 odom, TF 기준 |
| `/imu/accel_gyro` | 실제 IMU | 현재 real launch에서는 사용 안 함 |

`scan_fix_relay.py`는 `/scan_0`의 가변 길이 range 배열을 고정 400포인트 LaserScan인 `/scan_0_fixed`로 재발행합니다. 원본 scan은 프레임마다 포인트 개수가 달라지고 angle metadata와 실제 range 개수가 맞지 않을 수 있어, Real 모드의 AMCL, SLAM, costmap은 보정된 토픽을 사용합니다.

### Real costmap

| Item | Local Costmap | Global Costmap |
|------|---------------|----------------|
| Frame | `odom` | `map` |
| Size | `5m x 5m`, rolling window | 전체 맵 |
| Update frequency | `5Hz` | `1Hz` |
| Layers | front obstacle + inflation | static + front obstacle + inflation |
| Footprint | `0.70m x 0.60m` | `0.70m x 0.60m` |

## Sim Mode

Gazebo 기반 테스트 모드입니다. 실제 로봇보다 공격적인 속도와 듀얼 LiDAR costmap을 사용하며, 컨트롤러는 MPPI로 설정되어 있습니다.

### Sim 실행

터미널 1에서 Gazebo 시뮬레이션을 실행합니다.

```bash
ros2 launch antbot_gazebo gazebo.launch.py world:=depot
```

터미널 2에서 저장된 맵 기반 Nav2 스택을 실행합니다.

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim map:=/path/to/depot_sim.yaml
```

터미널 3에서 RViz를 실행합니다.

```bash
rviz2 -d $(ros2 pkg prefix antbot_navigation --share)/rviz/navigation.rviz \
  --ros-args -p use_sim_time:=true
```

SLAM으로 맵 생성:

```bash
ros2 launch antbot_navigation slam.launch.py mode:=sim
```

`slam.launch.py`는 `localization.launch.py`를 함께 실행하지 않습니다. 저장된 맵 기반 AMCL 위치 추정이 아니라, `slam_toolbox`가 실시간으로 맵을 생성하면서 `map -> odom` TF를 발행합니다.

위치 추정 전용:

```bash
ros2 launch antbot_navigation localization.launch.py mode:=sim map:=/path/to/map.yaml
```

Gazebo 창이 뜨고 ros2_control 컨트롤러가 로드될 때까지 보통 8-15초 정도 걸립니다. RViz에서 `2D Pose Estimate`로 초기 위치를 맞춘 다음 `Nav2 Goal`로 목표 지점을 지정합니다.

### Sim 아키텍처

- **Controller**: MPPI. 여러 후보 궤적을 샘플링하고 critic으로 평가해 가장 좋은 명령을 선택합니다.
- **Command DOF**: `vx`, `vy`, `wz`를 모두 생성할 수 있지만, `vy_max`를 낮게 둬 직진/회전 위주로 움직이게 합니다.
- **LiDAR**: `/scan_0`, `/scan_1`을 costmap obstacle layer에서 사용합니다.
- **Costmap**: 조향 오버슈트와 넓은 회전 반경을 고려해 real보다 큰 inflation을 사용합니다.

현재 launch 파일 기준으로 `config/sim/ekf.yaml`은 존재하지만 `localization.launch.py`에서 EKF 노드를 실행하지 않습니다. EKF를 다시 사용하려면 `robot_localization/ekf_node` 실행과 `odom -> base_link` TF 발행 주체를 함께 정리해야 합니다.

### Sim 핵심 설정

주요 설정 파일은 `config/sim/nav2_params.yaml`입니다.

| Item | Value |
|------|-------|
| Config directory | `config/sim/` |
| `use_sim_time` | `true` |
| Controller plugin | `nav2_mppi_controller::MPPIController` |
| MPPI motion model | `Omni` |
| AMCL motion model | `nav2_amcl::OmniMotionModel` |
| Velocity smoother | `[1.5, 0.15, 1.5]` |
| Costmap inflation | `0.75m` |

MPPI 핵심 파라미터:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `vx_max` | `1.0` | 전진 최대 속도 |
| `vy_max` | `0.1` | 횡방향 속도 제한. crab-walking을 거의 막고 직진/회전 위주로 유도 |
| `wz_max` | `1.5` | 최대 회전 속도 |
| `batch_size` | `2000` | 한 번에 평가하는 후보 궤적 수 |
| `time_steps` | `56` | `model_dt: 0.05` 기준 2.8초 미래 예측 |
| `motion_model` | `Omni` | Nav2 컨트롤러 내부 motion model |

주요 critic:

| Critic | Weight | Purpose |
|--------|--------|---------|
| `PreferForwardCritic` | `15.0` | 후진과 횡이동보다 전방 주행을 선호 |
| `PathAngleCritic` | `15.0` | 로봇 진행 방향과 글로벌 경로 방향 정렬 |
| `TwirlingCritic` | `10.0` | 불필요한 제자리 회전 억제 |
| `PathAlignCritic` | `10.0` | 글로벌 경로와 후보 궤적의 정렬도 평가 |
| `PathFollowCritic` | `5.0` | 글로벌 경로 이탈 억제 |
| `ObstaclesCritic` | `collision_cost: 10000.0` | 충돌 궤적 차단 |
| `ConstraintCritic` | `4.0` | 속도/가속도 제한 위반 페널티 |

### Sim 센서 토픽

| Topic | Source | Usage |
|-------|--------|-------|
| `/scan_0` | 전방 LiDAR | Sim AMCL/SLAM/costmap |
| `/scan_1` | 후방 LiDAR | Sim costmap obstacle layer |
| `/odom` | swerve controller odom | velocity smoother feedback |
| `/odometry/filtered` | EKF 출력으로 예약된 토픽 | Sim BT navigator 설정에 남아 있음 |
| `/imu/data` | 시뮬레이션 IMU | EKF 사용 시 입력 |

### Sim costmap

| Item | Local Costmap | Global Costmap |
|------|---------------|----------------|
| Frame | `odom` | `map` |
| Size | `5m x 5m`, rolling window | 전체 맵 |
| Update frequency | `5Hz` | `1Hz` |
| Layers | front obstacle + back obstacle + inflation | static + front obstacle + back obstacle + inflation |
| Footprint | `0.70m x 0.60m` | `0.70m x 0.60m` |

## Common Usage

### Navigation goal

RViz에서 단일 목표를 줄 때는 `2D Pose Estimate`로 초기 위치를 설정한 뒤 `Nav2 Goal`을 클릭합니다.

CLI로 목표를 보낼 수도 있습니다.

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0}}}}"
```

다중 웨이포인트를 사용하려면 `Panels > Add New Panel > nav2_rviz_plugins/Navigation2`를 추가한 뒤 Waypoint Mode를 켜고 여러 목표를 선택합니다.

### World and map

`navigation.launch.py`는 `map:=` 인자를 직접 받을 수 있습니다. `world:=depot`처럼 월드 이름으로 맵을 자동 선택하려면 `maps/worlds.yaml`에 월드 이름과 맵 파일을 등록해야 합니다.

```yaml
worlds:
  depot:
    sdf: depot.sdf
    map: depot_sim.yaml
```

등록 후에는 다음처럼 실행할 수 있습니다.

```bash
ros2 launch antbot_navigation navigation.launch.py mode:=sim world:=depot
```

새 월드를 추가할 때는 `.sdf`, `.pgm`, `.yaml` 맵 파일을 `maps/`에 넣고 `worlds.yaml`에 등록합니다.

## Real / Sim Comparison

| Setting | `mode:=real` | `mode:=sim` |
|---------|--------------|-------------|
| Config directory | `config/real/` | `config/sim/` |
| `use_sim_time` | `false` | `true` |
| Controller | RPP | MPPI |
| Controller plugin | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | `nav2_mppi_controller::MPPIController` |
| Motion model | AMCL `OmniMotionModel` | MPPI `Omni`, AMCL `OmniMotionModel` |
| Command DOF | `vx`, `wz` 중심 | `vx`, `vy`, `wz` |
| Max linear speed | RPP `desired_linear_vel: 0.30` | MPPI `vx_max: 1.0` |
| Velocity smoother | `[0.30, 0.02, 0.3]` | `[1.5, 0.15, 1.5]` |
| Odometry | swerve controller `/odom` | `/odom`, `/odometry/filtered` 설정 일부 존재 |
| LiDAR | `/scan_0` -> `/scan_0_fixed` | `/scan_0`, `/scan_1` |
| Costmap inflation | `0.5m` | `0.75m` |

AMCL은 두 모드 모두 `nav2_amcl::OmniMotionModel`을 사용합니다. AntBot은 구조상 holonomic motion을 고려해야 하므로 일반 diff-drive용 `DifferentialMotionModel`을 쓰면 좌우 이동 성분을 제대로 반영하지 못합니다.

DWB 컨트롤러는 현재 기본 설정에서 사용하지 않습니다. AntBot에서는 `wz` 진동과 `vx` 정체가 발생하기 쉬워, real은 RPP, sim은 MPPI 구성을 기본으로 둡니다.

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
│   └── depot_sim.yaml
├── scripts/
│   ├── scan_fix_relay.py
│   ├── cmd_vel_logger.py
│   └── laserscan_merger.py
└── rviz/
    └── navigation.rviz
```

## Troubleshooting

### `Failed to create plan`

로봇 초기 위치가 틀렸거나 costmap 상에서 로봇 footprint가 장애물과 겹칠 때 발생합니다.

- RViz에서 `2D Pose Estimate`로 초기 위치를 다시 지정합니다.
- global costmap을 초기화합니다.

```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
```

### 벽 충돌 또는 장애물 회피 실패

장애물과의 안전 여유가 부족한 경우입니다.

- `inflation_radius` 증가: real `0.5 -> 0.75`, sim `0.75 -> 1.0`
- `cost_scaling_factor` 감소: 예) `1.5 -> 1.0`
- MPPI 사용 시 `ObstaclesCritic.collision_margin_distance` 증가: 예) `0.1 -> 0.2`

### Real: 커브에서 주행이 멈추는 경우

`use_rotate_to_heading: true` 상태에서 heading 오차가 `rotate_to_heading_min_angle`보다 커지면 RPP가 정지 후 제자리 회전으로 전환합니다.

- `rotate_to_heading_min_angle`을 너무 낮게 두지 않습니다. 현재 권장값은 `0.785` rad입니다.
- `min_lookahead_dist`를 너무 작게 두면 정지, lookahead 축소, 더 급한 커브 인식이 반복될 수 있습니다.
- `rotate_to_heading_angular_vel`을 높이면 제자리 회전을 더 빨리 끝낼 수 있습니다.

### Sim: 제자리 회전이 과도한 경우

- `TwirlingCritic` 가중치 증가: `10.0 -> 15.0`
- `wz_max` 감소: `1.5 -> 1.0`
- `PreferForwardCritic` 가중치 조정

### TF 타임아웃 또는 `map` 프레임 누락

Gazebo를 재시작하면 sim time이 0으로 리셋되어 TF 버퍼가 깨질 수 있습니다. Gazebo와 Navigation은 함께 재시작하는 편이 안전합니다.

### 진단 명령어

```bash
# LiDAR 데이터 수신 주기 확인
ros2 topic hz /scan_0

# Real 모드 보정 scan 확인
ros2 topic hz /scan_0_fixed

# map -> odom TF 확인
ros2 run tf2_ros tf2_echo map odom

# odom -> base_link TF 확인
ros2 run tf2_ros tf2_echo odom base_link

# swerve controller의 odom TF 발행 상태 확인
ros2 param get /antbot_swerve_controller enable_odom_tf

# Nav2 controller lifecycle 상태 확인
ros2 lifecycle get /controller_server

# ros2_control controller 상태 확인
ros2 control list_controllers

# cmd_vel 로깅
ros2 run antbot_navigation cmd_vel_logger.py
```
