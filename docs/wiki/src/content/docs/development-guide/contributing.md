---
title: 5.5 기여 가이드
description: AntBot 프로젝트 기여 방법 (브랜치 전략, PR 규칙, 코딩 표준)
sidebar:
  order: 5
---

AntBot에 관심을 가져주셔서 감사합니다! 이 페이지는 본 프로젝트의 브랜치 전략, 개발 워크플로우, 컨벤션을 안내합니다.

전체 기여 가이드는 저장소의 [`CONTRIBUTING_ko.md`](https://github.com/ROBOTIS-move/antbot/blob/main/CONTRIBUTING_ko.md)에서도 확인할 수 있습니다.

---

## 브랜치 전략

| 브랜치 | 용도 | 생명주기 |
|--------|------|----------|
| `main` | 기본 브랜치. 안정적이고 배포 가능한 코드 | 영구 |
| `feature-*` | 기능 개발 및 버그 수정 | `main`에서 분기, 병합 후 삭제 |

- **`main`**이 단일 기준점입니다. 모든 개발 브랜치는 `main`에서 생성하고, `main`으로 병합합니다.
- `main` 브랜치에 직접 push는 **금지**됩니다. 모든 변경은 Pull Request를 통해야 합니다.

### 워크플로우

```
(1) 분기:   main → feature-*
(2) 개발:   브랜치에서 커밋 및 푸시
(3) 병합:   PR 생성 → 코드 리뷰 → main에 병합
(4) 릴리즈: main에서 태그 생성 → GitHub Release
```

---

## Pull Request 규칙

- **리뷰어**: 최소 **1명**의 승인이 필요합니다.
- **병합 담당**: PR 작성자가 승인 후 병합합니다.
- **CI 통과 필수**: 모든 린트 검사(cppcheck, cpplint, uncrustify, flake8, pep257, lint_cmake, xmllint, copyright)를 통과해야 합니다.
- **Assignees & Labels**: PR에 본인을 지정하고 관련 라벨을 추가합니다.
- 병합된 브랜치는 **자동 삭제**됩니다.

---

## 코딩 표준

본 프로젝트는 [ROS 2 코드 스타일](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)을 따릅니다:

- **C++**: Google C++ Style (cpplint & uncrustify로 검사)
- **Python**: PEP 8 (flake8) + PEP 257 docstrings
- **CMake**: lint_cmake 규칙
- **XML**: xmllint 검증
- 모든 소스 파일에 **Apache License 2.0** 저작권 헤더를 포함해야 합니다
- 모든 파일은 **UTF-8** 인코딩을 사용해야 합니다

---

## 버전 관리

[Semantic Versioning](https://semver.org/) (`x.y.z`)을 따릅니다:

| 변경 수준 | 범프 | 예시 |
|-----------|------|------|
| 버그 수정만 포함 | **z** 증가 | 1.1.0 → 1.1.1 |
| 신규 기능 포함 | **y** 증가, z 초기화 | 1.1.1 → 1.2.0 |
| 호환성이 깨지는 변경 | **x** 증가, y·z 초기화 | 1.2.0 → 2.0.0 |

버전은 개별 PR이 아닌, **릴리즈 시점에만** 올립니다. 변경 대상: `package.xml`의 `<version>` 태그 및 `setup.py`의 `version` 항목.

---

## Developer Certificate of Origin (DCO)

본 프로젝트에 기여함으로써 [Developer Certificate of Origin](https://developercertificate.org/)에 동의하게 됩니다. 커밋 시 `-s` 플래그를 사용하여 서명하세요:

```bash
git commit -s -m "Add new feature"
```

---

## 라이선스

본 프로젝트는 [Apache License 2.0](/antbot/license/)으로 배포됩니다. 모든 기여는 이 라이선스와 호환되어야 합니다.
