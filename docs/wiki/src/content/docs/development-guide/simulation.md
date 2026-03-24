---
title: 5.4 시뮬레이션 환경 구축
description: AntBot 시뮬레이션 환경 (URDF 시각화)
sidebar:
  order: 4
---

이 섹션에서는 실물 로봇 없이 AntBot 소프트웨어를 테스트할 수 있는 시뮬레이션 환경을 다룰 예정입니다.

:::note
본 가이드는 현재 작성 중이며, 향후 업데이트에서 제공됩니다. 진행 상황은 [GitHub 저장소](https://github.com/ROBOTIS-move/antbot)에서 확인하세요.
:::

### 예정 내용

- Gazebo 기반 물리 시뮬레이션
- URDF/Xacro 모델 시각화
- 센서 시뮬레이션 (LiDAR, 카메라, IMU)
- Nav2 연동 테스트

현재 URDF 모델만 확인하려면 다음 명령을 실행하세요:

```bash
ros2 launch antbot_description description.launch.py
```

설치 및 빌드 방법은 [소프트웨어 환경 구축](/antbot/software/environment-setup/)을 참조하세요.
