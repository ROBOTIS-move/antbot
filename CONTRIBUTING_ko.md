# AntBot 기여 가이드

AntBot에 관심을 가져주셔서 감사합니다! 이 문서는 본 프로젝트의 브랜치 전략, 개발 워크플로우, 컨벤션을 설명합니다.

## 브랜치 전략

### 브랜치 종류

| 브랜치 | 용도 | 생명주기 |
|--------|------|----------|
| `main` | 기본 브랜치. 안정적이고 배포 가능한 코드 | 영구 |
| `feature-*` | 기능 개발 및 버그 수정 (예: `feature-navigation`, `feature-fix-imu-crash`) | `main`에서 분기, 병합 후 삭제 |

- **`main`**이 단일 기준점입니다. 모든 개발 브랜치는 `main`에서 생성하고, `main`으로 병합합니다.
- `main` 브랜치에 직접 push는 **금지**됩니다. 모든 변경은 Pull Request를 통해야 합니다.

### 워크플로우

```
(1) 분기:   main → feature-*
(2) 개발:   브랜치에서 커밋 및 푸시
(3) 병합:   PR 생성 → 코드 리뷰 → main에 병합
(4) 릴리즈: main에서 태그 생성 → GitHub Release
```

## Pull Request 규칙

- **리뷰어**: 최소 **1명**의 승인이 필요합니다.
- **병합 담당**: PR 작성자가 승인 후 병합합니다.
- **CI 통과 필수**: 모든 린트 검사(cppcheck, cpplint, uncrustify, flake8, pep257, lint_cmake, xmllint, copyright)를 통과해야 합니다.
- **Assignees & Labels**: PR에 본인을 지정하고 관련 라벨을 추가합니다.
- 병합된 브랜치는 **자동 삭제**됩니다.

## 버전 관리

[Semantic Versioning](https://semver.org/) (`x.y.z`)을 따릅니다:

- **x (major)**: 이전 버전과 호환되지 않는 변경
- **y (minor)**: 하위 호환되는 신규 기능 추가
- **z (patch)**: 버그 수정 및 간단한 수정

### 버전 범프 시점

버전은 개별 PR이 아닌, **릴리즈 시점에만** 올립니다. 릴리즈 담당자가 마지막 릴리즈 이후 누적된 변경 사항을 기준으로 버전을 결정합니다:

- 버그 수정만 포함 → **z** 증가 (예: 1.1.0 → 1.1.1)
- 신규 기능 포함 → **y** 증가, z 초기화 (예: 1.1.1 → 1.2.0)
- 호환성이 깨지는 변경 포함 → **x** 증가, y와 z 초기화 (예: 1.2.0 → 2.0.0)

### 변경 대상

다음 항목을 모두 함께 업데이트해야 합니다:

- `package.xml` — `<version>` 태그 (저장소 내 모든 패키지)
- `setup.py` — `version` 항목 (Python 패키지)

저장소 내 모든 패키지는 동일한 버전 번호를 사용합니다.

## 릴리즈 프로세스

릴리즈는 정해진 주기 없이, 의미 있는 변경 사항(신규 기능, 버그 수정)이 `main`에 병합되어 배포 준비가 되면 **수시로** 진행합니다.

### 절차

1. 각 패키지의 `CHANGELOG.rst` 업데이트
2. `package.xml`과 `setup.py`의 버전 범프 ([버전 관리](#버전-관리) 참고)
3. `main`에 커밋 (예: `Bump version to x.y.z`)
4. GitHub Release와 태그 생성 (예: `v1.0.1`)

## 코딩 표준

본 프로젝트는 [ROS 2 코드 스타일](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)을 따릅니다:

- **C++**: Google C++ Style (cpplint & uncrustify로 검사)
- **Python**: PEP 8 (flake8으로 검사) + PEP 257 docstrings
- **CMake**: lint_cmake 규칙
- **XML**: xmllint 검증
- 모든 소스 파일에 저작권 헤더(Apache License 2.0)를 포함해야 합니다
- 모든 파일은 **UTF-8** 인코딩을 사용해야 합니다

## Developer Certificate of Origin (DCO)

본 프로젝트에 기여함으로써 [Developer Certificate of Origin](https://developercertificate.org/)에 동의하게 됩니다. 커밋 시 `-s` 플래그를 사용하여 서명하세요:

```bash
git commit -s -m "Add new feature"
```

이렇게 하면 커밋 메시지에 `Signed-off-by` 라인이 추가됩니다.

## 라이선스

본 프로젝트는 [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0)으로 배포됩니다. 모든 기여는 이 라이선스와 호환되어야 합니다.
