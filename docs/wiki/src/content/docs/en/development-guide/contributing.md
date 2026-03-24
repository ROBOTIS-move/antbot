---
title: 5.5 Contributing Guide
description: How to contribute to AntBot (branch strategy, PR rules, coding standards)
sidebar:
  order: 5
---

Thank you for your interest in contributing to AntBot! This page describes the branch strategy, development workflow, and conventions for this project.

The full contributing guide is also available at [`CONTRIBUTING.md`](https://github.com/ROBOTIS-move/antbot/blob/main/CONTRIBUTING.md) in the repository.

---

## Branch Strategy

| Branch | Purpose | Lifecycle |
|--------|---------|-----------|
| `main` | Default branch. Stable, release-ready code | Permanent |
| `feature-*` | Feature development and bug fixes | Created from `main`, deleted after merge |

- **`main`** is the single source of truth. All development branches are created from and merged back into `main`.
- Direct pushes to `main` are **not allowed**. All changes must go through a Pull Request.

### Workflow

```
(1) Branch:  main → feature-*
(2) Develop: Commit and push to your branch
(3) Merge:   Open PR → Code review → Merge into main
(4) Release: Tag on main → GitHub Release
```

---

## Pull Request Rules

- **Reviewers**: At least **1 reviewer** must approve before merge.
- **Merge responsibility**: The PR author merges after approval.
- **CI must pass**: All lint checks (cppcheck, cpplint, uncrustify, flake8, pep257, lint_cmake, xmllint, copyright) must pass.
- **Assignees & Labels**: Assign yourself and add relevant labels to the PR.
- Merged branches are **automatically deleted**.

---

## Coding Standards

This project follows the [ROS 2 Code Style](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html):

- **C++**: Google C++ Style (enforced by cpplint & uncrustify)
- **Python**: PEP 8 (enforced by flake8) + PEP 257 docstrings
- **CMake**: lint_cmake rules
- **XML**: xmllint validation
- All source files must include an **Apache License 2.0** copyright header
- All files must be **UTF-8** encoded

---

## Versioning

We follow [Semantic Versioning](https://semver.org/) (`x.y.z`):

| Change level | Bump | Example |
|-------------|------|---------|
| Bug fixes only | **z** increase | 1.1.0 → 1.1.1 |
| New features | **y** increase, reset z | 1.1.1 → 1.2.0 |
| Breaking changes | **x** increase, reset y·z | 1.2.0 → 2.0.0 |

Version is bumped **only at release time**, not on individual PRs. Update targets: `<version>` tag in `package.xml` and `version` field in `setup.py`.

---

## Developer Certificate of Origin (DCO)

By contributing to this project, you agree to the [Developer Certificate of Origin](https://developercertificate.org/). Sign off your commits using the `-s` flag:

```bash
git commit -s -m "Add new feature"
```

---

## License

This project is licensed under the [Apache License 2.0](/antbot/en/license/). All contributions must be compatible with this license.
