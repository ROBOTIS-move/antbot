#!/usr/bin/env bash
#
# bump_version.sh - Bump version of all antbot packages
#
# Usage:
#   ./scripts/bump_version.sh <version>
#
# Example:
#   ./scripts/bump_version.sh 1.1.0

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# vanjee packages are third-party; exclude from version bumping
EXCLUDE_DIRS=("vanjee_lidar_sdk" "vanjee_lidar_msg")

VERSION="${1:-}"

if [[ -z "${VERSION}" ]]; then
  echo "Usage: $0 <version>"
  echo "Example: $0 1.1.0"
  exit 1
fi

if [[ ! "${VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "Error: Version must be in semver format (e.g. 1.2.3)"
  exit 1
fi

is_excluded() {
  local dir_name
  dir_name="$(basename "$1")"
  for exclude in "${EXCLUDE_DIRS[@]}"; do
    if [[ "${dir_name}" == "${exclude}" ]]; then
      return 0
    fi
  done
  return 1
}

echo "Bumping version to ${VERSION} ..."
echo ""

# Update package.xml files
shopt -s nullglob
updated=0
for pkg_xml in "${REPO_ROOT}"/*/package.xml; do
  [[ -f "${pkg_xml}" ]] || continue

  pkg_dir="$(dirname "${pkg_xml}")"
  if is_excluded "${pkg_dir}"; then
    echo "  skip: $(basename "${pkg_dir}")/package.xml (third-party)"
    continue
  fi

  old_version=$(sed -n 's/.*<version>\(.*\)<\/version>.*/\1/p' "${pkg_xml}" | head -1)
  if [[ -z "${old_version}" ]]; then
    echo "  skip: $(basename "${pkg_dir}")/package.xml (no version tag found)"
    continue
  fi
  sed -i "0,/<version>${old_version}<\/version>/s//<version>${VERSION}<\/version>/" "${pkg_xml}"
  echo "  updated: $(basename "${pkg_dir}")/package.xml (${old_version} -> ${VERSION})"
  updated=$((updated + 1))
done

# Update CITATION.cff
citation="${REPO_ROOT}/CITATION.cff"
if [[ -f "${citation}" ]]; then
  if grep -q '^version:' "${citation}"; then
    sed -i "s/^version:.*/version: ${VERSION}/" "${citation}"
  else
    sed -i "/^license:/a version: ${VERSION}" "${citation}"
  fi
  echo "  updated: CITATION.cff"
fi

echo ""
echo "Done. ${updated} package(s) updated to ${VERSION}."
