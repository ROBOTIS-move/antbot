---
title: 5.3 Nav2 연동
description: AntBot Nav2 네비게이션 가이드
sidebar:
  order: 3
---

이 섹션에서는 AntBot과 [Nav2](https://docs.nav2.org/) 네비게이션 스택의 연동 방법을 다룰 예정입니다.

:::note
본 가이드는 현재 작성 중이며, 향후 업데이트에서 제공됩니다. 진행 상황은 [GitHub 저장소](https://github.com/ROBOTIS-move/antbot)에서 확인하세요.
:::

### 예정 내용

- AMCL 기반 로컬라이제이션 설정
- Nav2 파라미터 튜닝
- 웨이포인트 네비게이션
- 커스텀 코스트맵 플러그인 연동

현재 AntBot의 `/cmd_vel` 토픽과 `/odom` 토픽 명세는 [주요 ROS 토픽/서비스](/antbot/development-guide/ros-topics/)에서 확인할 수 있습니다.
