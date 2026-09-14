#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This script must be sourced: source ${BASH_SOURCE[0]}" >&2
  exit 2
fi

_openarm_v1_had_nounset=0
case $- in *u*) _openarm_v1_had_nounset=1; set +u ;; esac

ROS_DISTRO="${ROS_DISTRO:-humble}"
QUESTARMTELEOP_DIR="${QUESTARMTELEOP_DIR:-/workspace}"
OPENARM_WS="${OPENARM_WS:-/opt/openarm_ws}"
OPENARM_V1_OVERLAY_PREFIX="${OPENARM_V1_OVERLAY_PREFIX:-/workspace/.openarm_v1_overlay/install}"
OPENARM_VT_PYTHON="${OPENARM_VT_PYTHON:-/root/miniforge3/envs/vt/bin/python}"
OPENARM_V1_ROS_SETUP="${OPENARM_V1_ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
# The standalone package always uses only the reviewed ROS/OpenArm underlays.
OPENARM_V1_SKIP_LEGACY_OVERLAY=1

_openarm_v1_source_environment() {
  [[ -f "${OPENARM_V1_ROS_SETUP}" ]] \
    || { echo "ROS ${ROS_DISTRO} setup was not found." >&2; return 1; }
  [[ -f "${OPENARM_WS}/install/setup.bash" ]] \
    || { echo "Official OpenArm underlay was not found at ${OPENARM_WS}/install." >&2; return 1; }
  [[ -x "${OPENARM_VT_PYTHON}" ]] \
    || { echo "OpenArm Python was not found at ${OPENARM_VT_PYTHON}." >&2; return 1; }

  unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
  source "${OPENARM_V1_ROS_SETUP}" || return
  source "${OPENARM_WS}/install/setup.bash" || return
  # An explicit interpreter path works for venv without a shell activation hook.
  export PATH="$(dirname -- "${OPENARM_VT_PYTHON}"):${PATH}"
  if [[ -f "${OPENARM_V1_OVERLAY_PREFIX}/local_setup.bash" ]]; then
    source "${OPENARM_V1_OVERLAY_PREFIX}/local_setup.bash" || return
  fi
  export QUESTARMTELEOP_DIR OPENARM_WS OPENARM_V1_OVERLAY_PREFIX OPENARM_VT_PYTHON
  export OPENARM_V1_SKIP_LEGACY_OVERLAY
}

_openarm_v1_status=0
_openarm_v1_source_environment || _openarm_v1_status=$?
unset -f _openarm_v1_source_environment
if [[ "${_openarm_v1_had_nounset}" == 1 ]]; then set -u; fi
unset _openarm_v1_had_nounset
return "${_openarm_v1_status}"
