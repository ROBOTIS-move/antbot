# Contributing to AntBot

Thank you for your interest in contributing to AntBot! This document describes the branch strategy, development workflow, and conventions for this project.

## Branch Strategy

### Branch Types

| Branch | Purpose | Lifecycle |
|--------|---------|-----------|
| `main` | Default branch. Stable, release-ready code | Permanent |
| `feature-*` | Feature development and bug fixes (e.g., `feature-navigation`, `feature-fix-imu-crash`) | Created from `main`, deleted after merge |

- **`main`** is the single source of truth. All development branches are created from and merged back into `main`.
- Direct pushes to `main` are **not allowed**. All changes must go through a Pull Request.

### Workflow

```
(1) Branch:  main → feature-*
(2) Develop: Commit and push to your branch
(3) Merge:   Open PR → Code review → Merge into main
(4) Release: Tag on main → GitHub Release
```

## Pull Request Rules

- **Reviewers**: At least **1 reviewer** must approve before merge.
- **Merge responsibility**: The PR author merges after approval.
- **CI must pass**: All lint checks (cppcheck, cpplint, uncrustify, flake8, pep257, lint_cmake, xmllint, copyright) must pass.
- **Assignees & Labels**: Assign yourself and add relevant labels to the PR.
- Merged branches are **automatically deleted**.

## Versioning

We follow [Semantic Versioning](https://semver.org/) (`x.y.z`):

- **x (major)**: Breaking changes, incompatible with previous versions
- **y (minor)**: New features, backward compatible
- **z (patch)**: Bug fixes and minor corrections

### When to bump

Version is bumped **only at release time**, not on individual PRs. The release author decides the version based on the accumulated changes since the last release:

- Bug fixes only → bump **z** (e.g., 1.1.0 → 1.1.1)
- New features included → bump **y**, reset z (e.g., 1.1.1 → 1.2.0)
- Breaking changes included → bump **x**, reset y and z (e.g., 1.2.0 → 2.0.0)

### What to update

All of the following must be updated together:

- `package.xml` — `<version>` tag (for every package in the repository)
- `setup.py` — `version` field (for Python packages)

All packages in this repository share the same version number.

## Release Process

Releases are made **on demand** — whenever a meaningful set of changes (new features, bug fixes) has been merged into `main` and is ready to ship. There is no fixed schedule.

### Steps

1. Update `CHANGELOG.rst` for each package
2. Bump version in `package.xml` and `setup.py` (see [Versioning](#versioning) for which level to bump)
3. Commit to `main` with a message like: `Bump version to x.y.z`
4. Create a GitHub Release with a new tag (e.g., `v1.0.1`)

## Coding Standards

This project follows the [ROS 2 Code Style](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html):

- **C++**: Google C++ Style (enforced by cpplint & uncrustify)
- **Python**: PEP 8 (enforced by flake8) + PEP 257 docstrings
- **CMake**: lint_cmake rules
- **XML**: xmllint validation
- All source files must include a copyright header (Apache License 2.0)
- All files must be **UTF-8** encoded

## Developer Certificate of Origin (DCO)

By contributing to this project, you agree to the [Developer Certificate of Origin](https://developercertificate.org/). Sign off your commits using the `-s` flag:

```bash
git commit -s -m "Add new feature"
```

This adds a `Signed-off-by` line to your commit message.

## License

This project is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0). All contributions must be compatible with this license.
