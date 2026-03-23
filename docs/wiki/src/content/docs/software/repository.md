---
title: 3.1 오픈소스 저장소 안내
description: AntBot GitHub 오픈소스 저장소
sidebar:
  order: 1
---

AntBot의 구동을 위한 ROS 2 드라이버 및 제어 패키지는 **Apache License 2.0**으로 오픈소스 공개되어 있습니다.

## GitHub 저장소

- **Repository:** [github.com/ROBOTIS-move/antbot](https://github.com/ROBOTIS-move/antbot)
- **라이선스:** Apache License 2.0
- **기술 스택:** ROS 2 Humble, C++17, Python 3, Ubuntu 22.04

## 저장소 구성

```bash
antbot/
|-- antbot/                        # 메타 패키지
|-- antbot_bringup/                # Launch 파일 + 설정
|-- antbot_description/            # URDF / Xacro 로봇 모델
|-- antbot_swerve_controller/      # 스워브 드라이브 컨트롤러
|-- antbot_hw_interface/           # ros2_control 하드웨어 인터페이스
|-- antbot_libs/                   # 공유 C++ 라이브러리
|-- antbot_interfaces/             # 커스텀 메시지/서비스 정의
|-- antbot_camera/                 # 멀티 카메라 드라이버
|-- antbot_imu/                    # IMU 드라이버
|-- antbot_teleop/                 # 키보드/조이스틱 텔레오퍼레이션
|-- vanjee_lidar_sdk/              # Vanjee 3D LiDAR 드라이버 (외부)
|-- vanjee_lidar_msg/              # LiDAR 메시지 정의 (외부)
|-- setting.sh                     # 의존성 설치 스크립트
+-- additional_repos.repos         # 외부 저장소 목록 (vcs import용)
```
