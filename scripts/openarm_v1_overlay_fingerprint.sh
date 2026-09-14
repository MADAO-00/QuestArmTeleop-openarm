#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-/workspace}"
PACKAGE_ROOT="src/oculus_reader"
OPENARM_MODEL_ROOT="assets/openarm_mujoco/v1"

[[ "${WORKSPACE_ROOT}" == /* \
    && -d "${WORKSPACE_ROOT}/${PACKAGE_ROOT}" \
    && -d "${WORKSPACE_ROOT}/${OPENARM_MODEL_ROOT}" ]] \
  || { echo "OpenArm overlay fingerprint source is unavailable." >&2; exit 1; }

cd "${WORKSPACE_ROOT}"
{
  find "${PACKAGE_ROOT}" -type f \
    ! -path '*/__pycache__/*' \
    ! -name '*.pyc' \
    ! -path '*/APK/*' \
    -print0
  # The IK configuration loads scene.xml from this external tree.  Hash the
  # complete official model release, including recursively included MJCF and
  # every referenced collision/visual mesh, so the reviewed real release
  # cannot silently run with different kinematics or geometry.
  find "${OPENARM_MODEL_ROOT}" -type f -print0
  printf '%s\0' \
    scripts/rebuild_openarm_v1_overlay_in_container.sh \
    scripts/source_openarm_v1_overlay.sh \
    scripts/openarm_v1_overlay_fingerprint.sh
} | LC_ALL=C sort -z \
  | xargs -0 sha256sum \
  | sha256sum \
  | awk '{print $1}'
