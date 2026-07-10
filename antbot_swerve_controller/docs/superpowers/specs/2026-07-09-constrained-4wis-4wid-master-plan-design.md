# Constrained 4WIS-4WID + Feasibility + MPPI 마스터 기획

- 작성일: 2026-07-09
- 대상 플랫폼: **antbot** (= gaemi **G50**, 동일 물리 플랫폼으로 확인됨)
- 범위: `antbot_swerve_controller` 재작성 + navigation feasibility(cmd_vel 리미터 + Nav2 MPPI critic) + 공통 feasibility 라이브러리
- 상태: 마스터 기획(전체 4개 컴포넌트) + 첫 상세 스펙 대상 = **[0] 공통 feasibility 라이브러리**
- 선행 문서: `antbot_swerve_controller/docs/continuous_3dof_controller_plan.html` (사용자 작성 P0–P6 컨트롤러 설계)
- 참조 구현(실행 가능 스펙): `colcon_ws/src/gaemi_simulations/scripts/g50_constrained_4wis.py`, `g50_feasibility_continuous.py`, `g50_constrained_4wis_sim.py`

---

## 0. 배경과 목표

### 0.1 현재 문제
현재 `antbot_swerve_controller`(`SwerveDriveController` + `SwerveMotionControl`)는 **모드 기반 FSM**이다.

- `TargetDrivingType { STOPPED, LINEAR_DRIVING, L_TURNING, R_TURNING, DIAGONAL_DRIVING, SPINNING, RE_ALIGNING, FAULT }`
- 모드 전환 시 `needs_realignment_matrix_`/`generate_realignment_motion()`으로 **바퀴를 완전히 정지시키고 조향만 재정렬**한 뒤 주행 재개 → **불연속**.
- 특히 `SPINNING`(제자리 회전)은 진입/이탈 모든 전환이 realignment를 강제 → **잦은 제자리 회전/멈칫거림**.
- IK 자체는 3DOF(vx,vy,wz)를 쓰지만 모드 라벨이 명령 공간을 쪼개고, `swerve_sim.py` 등은 2DOF로 퇴화. 실질 3DOF 연속 제어가 어려움.

### 0.2 목표
1. **연속 단일 함수** 컨트롤러(constrained 4WIS-4WID per-tick allocator)로 교체. 모드/재정렬 FSM 제거.
2. **alpha 제거** (사용자 지시). feasibility 책임은 상류(navigation)로 이전.
3. **v_y(크랩)와 arc(선회) 모두** 부드럽게 사용. 순수 횡이동(vx=0)은 물리적으로 불가함을 시스템 전체가 인지.
4. navigation이 **feasible한 (vx,vy,w) 명령만** 내리도록:
   - **아이디어 #1**(신뢰): 2차형식 feasibility 함수 `G(u)=min_i uᵀMᵢu` (전 영역 해석적).
   - **아이디어 #2**(참고): 조향각 제한 · w에 따른 쐐기/전환 해석식 (r(γ,w), feasible v_y 등).
5. **MPPI**가 "모든 timestep이 feasible한 궤적 중 최선"을 선택하도록 수정.

### 0.3 확정된 정체성
antbot과 gaemi G50은 **동일 물리 플랫폼**이다(wheelbase 0.53, wheel track 0.512, wheel_radius 0.103 동일; gaemi changelog상 조향한계를 55°→53°로 축소한 이력). 따라서 gaemi의 실기 파라미터/파이썬 시뮬레이션을 antbot의 **신뢰 가능한 참조**로 사용하되, **antbot 저장소를 canonical**로 삼는다.

---

## 1. 확정된 시스템 사실 / 파라미터

> 출처는 코드 검토(antbot_description URDF, board_params.yaml, gaemi hardware.yaml, 참조 파이썬)로 교차확인.

### 1.1 기하 (핵심 정정 포함)
- 모듈 순서: **FL, FR, RL, RR** (전 코드 공통).
- **조향축(steering axis) 위치** `a_i = [x_i, y_i]`:
  - `x_i = ±0.265 m` (wheelbase 0.530/2) — 유지.
  - `y_i = ±0.2005 m` (**steering_width 0.401/2**) — **정정 대상**.
- **바퀴 접지점 위치** `p_i = a_i + c_i·n_i`, 횡 오프셋 `c_i = ±0.0555 m` (left +, right −; `wheel_offset=(0.512−0.401)/2`).
  - 결과적으로 바퀴 접지 y = 0.2005 + 0.0555 = **±0.256 m** (wheel track 0.512/2).
- **정정 필요**: 현재 `swerve_drive_controller.yaml`의 `module_y_offsets = ±0.256`은 *바퀴 접지 y*이며, IK/feasibility가 요구하는 *조향축 y*(±0.2005)와 다르다. `module_y_offsets = ±0.2005`로 수정하고 `steering_to_wheel_y_offsets = ±0.0555`는 그대로 둔다(현재는 0.0555를 이중 계상).
- `wheel_radius r = 0.103 m` — 유지.

### 1.2 한계값
- **조향각 한계**: 실기 `board_params.yaml` = **±55° (±0.9599 rad)**, `steering.cpp`에서 clamp·gear_ratio 2.43 적용. gaemi 최신 hardware는 ±53°(±0.925 rad). URDF ±90°, gazebo ±60°은 느슨한 sim 값.
  - **결정(잠정)**: 하드웨어 clamp = **±55°**, feasibility 판정 γ = **53°**(≈ 55° − 2° soft margin)로 보수적 사용. **→ 실기 조향한계·영점 최종 확인 필요(리스크 R1).**
- **바퀴 속도 한계**: 실기 `board_params.yaml` = **±20 rad/s (≈2.06 m/s)**. (참조 파이썬은 30 rad/s를 쓰므로 실기 20으로 조정.)
- **조향 속도 한계**: URDF 6.5 rad/s, 컨트롤러 config `steering.max_velocity 5.0`. → 조향 rate limit `nu_max ≤ 5.0` (참조 3.0 사용 가능).
- **바디 속도(참고)**: nav swerve_limits(gaemi) vx_max 1.5, vy_max 0.5, wz_max 1.5. antbot MPPI(sim) vx_max 1.0, vy_max 0.1(크랩 억제됨), wz_max 1.5. → **크랩 사용을 위해 [B]/[C]에서 vy_max 상향 필요**.

### 1.3 인터페이스 / 실행 환경
- 컨트롤러: ros2_control `controller_interface::ControllerInterface` 플러그인.
  - 입력: `geometry_msgs/Twist` on `/cmd_vel` (linear.x=vx, linear.y=vy, angular.z=wz; **linear.z<0 = 비상정지 플래그**).
  - 출력(모듈별): 조향 **position**(필수, opt velocity/accel), 바퀴 **velocity**(필수, opt accel).
  - 상태: 조향 position, 바퀴 velocity.
  - 주기: 실기 20 Hz, gazebo 100 Hz.
- Navigation: **stock Nav2 (Humble)**, `antbot_navigation`은 config/launch 전용 패키지(포크 아님).
  - **sim 모드 = `nav2_mppi_controller::MPPIController`** (batch 2000, time_steps 56, motion_model "Omni", critic 10개). → **[C]는 새 MPPI가 아니라 critic 플러그인**.
  - real 모드 = `nav2_regulated_pure_pursuit_controller`(RPP, vy=0).
  - `controller_server → /cmd_vel → swerve controller` **직결**(중간 노드 없음). `velocity_smoother` 블록은 config에만 있고 미기동. → **[B] 삽입용 빈 지점 존재**.
  - footprint 0.70×0.60 사각형.

---

## 2. 핵심 수식 (feasibility)

기준 좌표: +X 전방, +Y 좌, yaw CCW(+). 모듈 순서 FL,FR,RL,RR.

### 2.1 모듈 운동학
```
v_A,i = [ vx − w·y_i ,  vy + w·x_i ]        # 조향축 i 지점의 바디 twist 속도
u_i   = [ cos d_i , sin d_i ]               # 구름(rolling) 방향 단위벡터
n_i   = [ −sin d_i , cos d_i ]              # 횡(no-slip) 방향
p_i   = a_i + c_i·n_i                        # 바퀴 접지점
```

### 2.2 조향 IK + flip
```
beta_i  = atan2( vy + w·x_i ,  vx − w·y_i )
cand0   = wrap_pi(beta_i) ,  cand1 = wrap_pi(beta_i + pi)   # +pi = 같은 구름선, 역회전(바퀴속도 부호 반전)
d*_i    = argmin_{φ∈{cand0,cand1}} | wrap_pi(φ − d_i) |     # 현재각에 가까운 유효 후보
저속(|v_A,i|<eps) → 현재 조향각 유지 (atan2 chatter 방지)
```

### 2.3 아이디어 #1 — 2차형식 feasibility (전 영역, 신뢰)
조향 구간 `[d_min, d_max]`에 대해 `θ = ½(d_min+d_max)`(중심), `γ = ½(d_max−d_min)`(반각), `q = [cosθ, sinθ]`.
대칭 한계(±L)이면 `θ=0, q=[1,0], γ=L`.

```
J_i = [[1, 0, −y_i],
       [0, 1,  x_i]]                         # v_A,i = J_i · u,  u=[vx,vy,w]

g_i(u) = (q·v_A,i)² − cos²γ · ||v_A,i||²
       = uᵀ M_i u ,   M_i = J_iᵀ (q qᵀ − cos²γ · I₂) J_i      # 상수 3×3 대칭행렬 (모듈당 1개)

G(u) = min_{i∈{FL,FR,RL,RR}} uᵀ M_i u
feasible ⇔ G(u) ≥ 0
```
- 로봇당 **4개 상수 행렬**만 사전계산하면 됨. `G`는 2차 동차 → `G(αu)=α²G(u)` → **feasible 집합은 원점 꼭짓점 cone(스케일 불변, 방향 성질)**.
- 부드러운(C¹) 버전: `g_i_norm = g_i/(||v_A,i||²+ε)`, `α_soft = smoothstep(G_norm/gate_width)`, `gate_width=0.08`.

### 2.4 아이디어 #2 — 해석적 쐐기/전환 (θ=0 대칭 가정, sx=0.265, sy=0.2005)
```
feasible ⇔ sinγ·vx − cosγ·|vy| ≥ r      또는     −sinγ·vx − cosγ·|vy| ≥ r
r          = w·( sx·cosγ + sy·sinγ )       # 원점 중심, 쐐기 경계에 접하는 원 = 'infeasible 간격' 반지름
apex_vx    = r / sinγ = w·( sy + sx/tanγ )  # 쐐기 꼭짓점(원점에 가장 가까운 feasible vx)
feasible v_y (vx 고정): |vy| ≤ tanγ·|vx| − r/cosγ = tanγ·|vx| − w·( sx + sy·tanγ )   (단 |vx| ≥ apex_vx)
```
**정정(사용자 정리 대비):**
- `r`은 w에 **비례해 커진다**(≈0.32·w). "w↑ 하면 내접원이 줄어든다"는 반대. `r`은 원점 주변 *infeasible 간격* 반지름이며, 커질수록 feasible가 줄어드는 것과 동치.
- 초록 영역은 **무한대 쐐기**(플롯 박스에 잘려 삼각형처럼 보임). 물리적으로 "줄어드는 내접원"은 없음.
- "w·vx 동일 비율 감소로 feasible 진입"은 **불가**(cone은 스케일 불변). feasible 진입은 **방향 변경**으로만: `v_y 축소` 또는 `w 축소(곡률↑)`. 동일 비율 균일 스케일은 *속도/구동 한계*(바퀴 saturation)용 도구이며 그때 곡률을 보존.

### 2.5 v_y(크랩) + arc(선회) 사용 범위
- **대각/크랩**: 쐐기 내(`|vy| ≤ tanγ·vx − r/cosγ`)면 feasible. 예: vx=0.8, vy=0.4 → 26.6° 조향(feasible).
- **arc(선회)**: vy=0, w≠0 → ICR 기반 **최소 회전반경** 제약. `apex_vx = r/sinγ`가 실질 최소 곡률 경계.
- **제자리 회전**: vx=vy=0, w≠0 → 필요 조향각 `atan2(sx,sy)=52.9°`. ±53° 한계에서 **마진 0에 가까움**(margin>0이면 infeasible). → 저속·연속 회전은 되지만 마진 관리 필요.
- **순수 횡이동**: vx=0, vy≠0 → 90° 필요 → **영구 infeasible**(시스템 전체가 인지, planner가 회피).

---

## 3. 아키텍처 — 4개 컴포넌트

```
          ┌──────────────────────────────────────────────────────┐
          │ [0] feasibility 라이브러리 (portable C++/Eigen, ROS 무관) │
          │   • G(u)=min_i uᵀMᵢu (아이디어 #1, 4개 행렬)             │
          │   • r(γ,w), feasibleVy(vx,w), coneProject, speedScale  │
          │     (아이디어 #2)   ← g50_*.py 오라클로 유닛테스트        │
          └───────────────┬──────────────────────┬─────────────────┘
        ┌─────────────────┘                      └──────────────────┐
   [A] antbot_swerve_controller            [B] cmd_vel feasibility 리미터 (nav)
   Constrained4WisAllocator                독립 노드: /cmd_vel_raw → project +
   • FSM/RE_ALIGNING/alpha 제거              curvature-preserve speedScale → /cmd_vel
   • y=0.2005, c=0.0555, 실기 한계           → [C]가 가정하는 "상류 feasible" 계약 보장
   • 8×3 odometry + 진단(clamp_error)                     │
   • control_model feature flag             [C] Nav2 MPPI FeasibilityCritic
        (사용자 P0–P6 정식화)                 배리어비용 β·Σ max(0,−G) + 최종 투영([B])
                                            → 가장 좋은 *feasible* 궤적 선택
```

- 의존: `[B]`, `[C]`는 `[0]`에 의존. `[A]`는 `[0]` 없이도 동작(clamp-only)하나 동일 기하/수식 상수를 공유하면 좋음.
- **빌드 순서: 0 → A → B → C** (0의 API 확정 후 A는 병렬 가능).

---

## 4. [0] 공통 feasibility 라이브러리 — **첫 상세 스펙 대상**

### 4.1 원칙
- 순수 C++17 + Eigen, **ROS/Nav2 의존 없음** → 컨트롤러/노드/critic 모두 링크. 헤더+정적 라이브러리 또는 헤더온리.
- 참조 파이썬(`g50_feasibility_continuous.py`)의 **수치 오라클과 대조 가능**해야 함(핵심 검증 수단).

### 4.2 API (초안)
```cpp
struct ModuleGeom { double x, y, c; };            // 조향축 x,y + 부호있는 횡오프셋 c
struct FeasibilityModel {
  ModuleGeom mod[4];
  double gamma, theta;                            // 조향 cone 반각/중심 (대칭이면 theta=0)
  // 사전계산
  void build();                                   // M_i (Eigen::Matrix3d) 4개
  // 아이디어 #1
  double G(double vx,double vy,double w) const;   // min_i uᵀ M_i u
  bool   feasible(double vx,double vy,double w,double margin=0) const;
  std::array<double,4> moduleMargins(double vx,double vy,double w) const;
  // 아이디어 #2 (theta=0 대칭 전용 해석식; 일반각은 #1로 폴백)
  double r_of_w(double w) const;                  // w·(sx cosγ + sy sinγ)
  double apexVx(double w) const;                  // r/sinγ
  std::pair<double,double> feasibleVyRange(double vx,double w) const;
  // 투영/스케일
  Twist  coneProject(Twist u, ProjectPolicy p) const;   // v_y 축소 우선 → 필요시 w 축소
  Twist  speedScale (Twist u, const ActuatorLimits&) const; // 균일 스케일(곡률 보존)
  std::optional<double> feasibleOmega(double vx,double vy,double w_ref) const; // (vx,vy 고정, w만)
};
```

### 4.3 coneProject 정책 (아이디어 #2 반영, 크랩+arc 지원)
입력 `u=(vx,vy,w)`가 infeasible일 때, **명령 의도별 최소 방향변경**:
1. **크랩 우선 정책**(대각 유지): `vx, w` 고정, `|vy|`를 `feasibleVyRange(vx,w)` 상한까지 축소.
2. **arc 우선 정책**(선회 유지): `vy` 고정(주로 0), `|w|`를 축소하여 회전반경↑ (또는 `feasibleOmega`).
3. **혼합/일반**: `G(u)<0`이면 cone 표면으로의 최소거리 투영(수치). 정책 플래그로 선택.
- 반환에 **적용 방식·투영량**을 함께 담아 진단 발행.

### 4.4 speedScale (곡률 보존)
`coneProject` 후에도 바퀴속도/축가속 한계 초과 시 `u ← s·u (s∈(0,1])` 균일 스케일. cone 불변이므로 feasibility 유지, **곡률(arc) 보존** — 사용자의 "diff drive처럼" 요구가 여기에 대응.

### 4.5 검증(유닛테스트, 사용자 테스트 전 자동)
- `g50_feasibility_continuous.py`와 **20000 랜덤 (vx,vy,w) 대조**(margin 목표 ~1e-12).
- `run_check()` 시나리오 값 재현: 전진 feasible, 대각30° feasible, 제자리회전(마진0 근처), 순수횡 infeasible, arc R fit.
- 오라클 값은 파이썬을 한 번 실행해 CSV로 덤프 후 C++ gtest 고정값으로 사용.

> **[0]의 상세 구현 스펙 + 태스크 분해는 이 마스터 기획 승인 후 별도 스펙 문서로 작성 → writing-plans로 진행.**

---

## 5. [A] antbot_swerve_controller 재작성 (사용자 P0–P6 정식화)

### 5.1 Constrained4WisAllocator (순수 C++, 테스트 가능)
`update(steering[4], u_ref=(vx,vy,w), dt) → { steering_pos[4], steering_vel[4]?, wheel_vel[4], diag }`
```
per module i:
  v_A,i  = [vx − w·y_i, vy + w·x_i]
  beta   = atan2(v_A,y, v_A,x);  후보 {wrap_pi(beta), wrap_pi(beta+pi)}; 현재각 근접 선택 = d_branch
  저속 hold: |v_A,i|<low_speed_threshold → d_branch = 현재각
  d*_i   = clamp(d_branch, d_min+m, d_max−m)                # m = steering.soft_margin
  nu_i   = sat(k_d · wrap_pi(d*_i − d_i), ±nu_max)          # 조향 속도(P + rate limit)
  d_cmd  = clamp(d_i + nu_i·dt, d_min+m, d_max−m)           # 조향 position 명령
  phi_dot_i = ( u(d_cmd)·v_A,i − c_i·(w + nu_i) ) / r        # 바퀴속도 (−c_i(w+nu)=스크럽 보정 내장)
  phi_dot_i = clamp(phi_dot_i, ±phi_dot_max)                # per-wheel 포화
diag: clamp_error[], target_branch[], cmd_vel_executed, wheel_clamp_active, steering_margin
```

### 5.2 alpha 제거의 3역할 처리 (명시)
참조의 `alpha = min(1, α_dir, α_align, α_wheel)` 3역할을 각각:
- **α_dir (feasibility gate)** 제거 → **상류([B]/[C])가 보장**. 컨트롤러는 `clamp-only best_effort`(문서 기본): 후보가 한계 밖이면 정지 대신 soft limit으로 clamp하고 주행. 대신 **clamp_error/cmd_vel_executed 진단 필수**(경로 왜곡 감시).
- **α_align (정렬 감속)** 제거 → 조향 전이 중 소량 스크럽 허용. rate-limited 조향으로 완화, 상류가 부드러운 명령을 준다는 전제.
- **α_wheel (공통 스케일)** 제거 → per-wheel clamp(문서 기본). (참고: 공통 스케일은 [B]의 speedScale이 곡률 보존 형태로 수행하므로 컨트롤러단 불필요. 필요 시 옵션 복원 가능.)

### 5.3 파라미터 변경
- 수정: `module_y_offsets ±0.256 → ±0.2005`; 조향한계 실기값 + `steering.soft_margin`.
- 유지: `steering_to_wheel_y_offsets ±0.0555`, `wheel_radius 0.103`, `module_x_offsets ±0.265`.
- 신규: `control_model {legacy|constrained_4wis_4wid}`, `steering.kp(k_d)`, `steering.soft_margin(m)`, `steering.low_speed_threshold`, `steering.clamp_policy(best_effort)`, `diagnostics.publish_clamp_error`.
- 상수 초기값: `k_d=4.0`, `nu_max=3.0`(≤5.0), `phi_dot_max=20 rad/s`(**실기 20**, 참조 30 아님), `low_speed_eps=1e-3`.

### 5.4 8×3 odometry
```
rolling row : [ cos d_i, sin d_i, x_i sin d_i − y_i cos d_i − c_i ]·[vx,vy,w]ᵀ = r·phi_dot_i + c_i·d_dot_i
lateral row : [ −sin d_i, cos d_i, x_i cos d_i + y_i sin d_i ]·[vx,vy,w]ᵀ = 0
```
4모듈×2 = 8행, 3미지수 → SVD 최소자승. position-only 조향이면 `d_dot_i`는 측정 조향각의 저역통과 차분.

### 5.5 제거 목록
`TargetDrivingType` FSM, `RobotState`, `needs_realignment_matrix_`, `generate_realignment_motion()`, `SwerveMotionControl` 전체, JointTrajectory 실행/`planned_trajectory` 발행, `enable_steering_scrub_compensator`(−c_i(w+nu)로 흡수), `non_coaxial_ik_iterations`, `realigning_angle_threshold`, `discontinuous_motion_steering_tolerance`, `trajectory_delay_time`.

### 5.6 마이그레이션 (사용자 문서 P0–P6)
- **P0** 기하 파라미터 정정(module_y ±0.2005, sim/real 분리).
- **P1** `Constrained4WisAllocator` 순수 C++ + 유닛테스트(참조 오라클 값). (branch select/clamp/steering cmd/wheel cmd)
- **P2** `SwerveDriveController::update()`에 `control_model` feature flag로 연결(legacy 병행).
- **P3** 8×3 odometry + 조향속도 차분 필터.
- **P4** 진단 발행(clamp_error/steering_margin/wheel_clamp_active).
- **P5** legacy FSM/trajectory 삭제.
- **P6** navigation 연동([B]/[C]로 이관 — 아래).

---

## 6. [B] cmd_vel feasibility 리미터 (navigation)

### 6.1 배치
- **독립 ROS2 노드**(권장): `controller_server`를 `/cmd_vel_raw`로 remap → limiter 노드가 구독 → 처리 후 `/cmd_vel` 발행. sim(MPPI)·real(RPP) 양쪽에 동일 적용.
- 대안: dormant Nav2 `velocity_smoother` 부활(축별 accel만 → feasibility 불가, 부적합) / Nav2 컨트롤러 플러그인 래퍼.

### 6.2 처리 파이프라인 ([0] 사용)
```
u_raw → coneProject(정책: 크랩/arc)     # 방향 최소변경으로 feasible cone 진입
      → speedScale(actuator limits)     # 곡률 보존 균일 스케일 (사용자 "diff drive처럼")
      → rate limit / deadband
      → /cmd_vel
```
- 실기 조향한계 + margin으로 γ 설정. **크랩 사용 위해 vy 상한을 쐐기 상한까지 허용**.
- [A]의 best_effort 안전망과 이중화(정상 시 [B]가 이미 feasible 보장).
- 진단: 투영량/스케일량 발행 → planner 피드백(P6) 소스.

---

## 7. [C] Nav2 MPPI FeasibilityCritic

### 7.1 기본안 (권장 시작점 = (나)-①)
- **커스텀 Nav2 MPPI critic 플러그인** 추가:
  ```
  cost += β · Σ_t max(0, −G(V_{k,t}))     # 소프트 배리어; 비볼록 안전; batch 유지
  ```
  참조 문서 근거: feasible 집합이 **비볼록**이라 rejection sampling 금지(두 feasible 샘플 평균이 infeasible 가능) → **soft barrier**로 가중치만 0쪽으로.
- **최종 명령은 [B]에서 투영**(가중평균의 비볼록 누출 보정 = 문서의 "final projection").
- 기존 Omni 모션모델 유지 + **vy_max 상향**(크랩). 기존 ConstraintCritic 등과 병행.
- 신규 파라미터: `β`, `gate`, (샘플 K/호라이즌 T/온도 λ는 참조에 없어 신규 튜닝).

### 7.2 업그레이드 (phase-2, 옵션 = (나)-②)
- MPPI 모션모델을 **constrained-swerve 플랜트**로 교체 → 모든 롤아웃이 feasible-by-construction, infeasible 방향은 정체(낮은 보상→낮은 가중치). 문서 방식 완전 구현. 플러그인 심층 포크, 유지비↑. [C] 상세 스펙에서 재확정.

---

## 8. 빌드 순서 / 페이즈 요약

| 단계 | 산출물 | 검증 |
|---|---|---|
| **Phase 0** | [0] feasibility 라이브러리 + gtest | 파이썬 오라클 20000 대조 |
| Phase A1 | `Constrained4WisAllocator` 순수 C++ + gtest | 참조 오라클 시나리오 값 |
| Phase A2 | `update()` feature flag 연결 | legacy 병행 회귀 |
| Phase A3 | 8×3 odometry | FK 대조 |
| Phase A4 | 진단 발행 | 토픽 확인 |
| Phase A5 | legacy 삭제 | 빌드/회귀 |
| Phase B | cmd_vel 리미터 노드 + remap | 단위/재생 테스트 |
| Phase C | MPPI FeasibilityCritic (+opt 모션모델) | 롤아웃 feasible율 |

각 컴포넌트는 이 마스터 기획 승인 후 **개별 상세 스펙 → writing-plans**로 확장. 첫 상세 스펙 = **[0]**.

---

## 9. 테스트 / 검증 (통합·실기 검증은 사용자 수행)

- **유닛(자동)**: [0] 라이브러리, `Constrained4WisAllocator`를 참조 파이썬 오라클과 대조.
- **통합(사용자)**: `antbot_gazebo` sim에서 MPPI 경로주행 — 크랩·arc 연속성, 제자리회전 감소, clamp_error 최소 확인.
- **실기(사용자)**: `control_model` feature flag로 점진 전환, 진단 모니터링.

---

## 10. 리스크 / 미해결

- **R1** 조향한계 실기 확정: antbot ±55°(board_params) vs gaemi 최신 ±53°. + 영점 보정. → 하드웨어 확인.
- **R2** 바퀴속도 한계 20(실기) vs 30(참조): 20 사용.
- **R3** best_effort clamp 시 경로 왜곡/슬립: 진단으로 감시, 필요 시 컨트롤러 reject 정책 옵션.
- **R4** MPPI K/T/λ/β 튜닝값 미정(참조에 없음): [C]에서 신규 설계.
- **R5** 순수 횡이동 불가 — planner/BT가 인지해야(경로가 크랩만으로 목표 도달 시도 금지).
- **R6** 조향 속도 인터페이스 유무 → position-only면 d_dot 필터 품질이 odometry/스크럽 보정에 영향.
- **R7** [B] 삽입용 `/cmd_vel` remap이 다른 노드에 영향 없는지 확인.

---

## 11. 파라미터 표 (초기값)

| 파라미터 | 값 | 비고 |
|---|---|---|
| module_x_offsets | ±0.265 m | 유지 |
| **module_y_offsets** | **±0.2005 m** | 0.256에서 정정(조향축) |
| steering_to_wheel_y_offsets (c) | ±0.0555 m | left+, right− |
| wheel_radius (r) | 0.103 m | 유지 |
| 조향 hardware clamp | ±55° (±0.9599 rad) | 실기, R1 확인 |
| feasibility γ | 53° | soft margin ≈2° |
| wheel_speed_limit (phi_dot_max) | 20 rad/s | 실기 |
| steer rate limit (nu_max) | 3.0 rad/s | ≤5.0 |
| steer gain (k_d) | 4.0 1/s | 초기 |
| low_speed_threshold | 1e-3 | atan2 chatter 방지 |
| MPPI vy_max | 상향(예 0.5) | 크랩 사용 |
| MPPI barrier β / gate_width | 튜닝 / 0.08 | [C] |
