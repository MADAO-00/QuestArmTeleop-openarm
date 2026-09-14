#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
QUESTARMTELEOP_DIR="${QUESTARMTELEOP_DIR:-/workspace}"
OPENARM_WS="${OPENARM_WS:-/opt/openarm_ws}"
OPENARM_V1_OVERLAY_ROOT="${OPENARM_V1_OVERLAY_ROOT:-/workspace/.openarm_v1_overlay}"
OPENARM_V1_OVERLAY_PACKAGES="${OPENARM_V1_OVERLAY_PACKAGES:-oculus_reader}"
OPENARM_V1_SOURCE_SCRIPT="${OPENARM_V1_SOURCE_SCRIPT:-/workspace/scripts/source_openarm_v1_overlay.sh}"
OPENARM_V1_FINGERPRINT_SCRIPT="${OPENARM_V1_FINGERPRINT_SCRIPT:-/workspace/scripts/openarm_v1_overlay_fingerprint.sh}"
OPENARM_V1_FINGERPRINT_STAMP="${OPENARM_V1_FINGERPRINT_STAMP:-${OPENARM_V1_OVERLAY_ROOT}/source.sha256}"
OPENARM_V1_SYMLINK_INSTALL="${OPENARM_V1_SYMLINK_INSTALL:-1}"
OPENARM_V1_SKIP_LEGACY_OVERLAY="${OPENARM_V1_SKIP_LEGACY_OVERLAY:-1}"
OPENARM_V1_ROS_SETUP="${OPENARM_V1_ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"

[[ -f "${OPENARM_V1_ROS_SETUP}" ]] \
  || { echo "ROS ${ROS_DISTRO} setup was not found." >&2; exit 1; }
[[ -f "${OPENARM_WS}/install/setup.bash" ]] \
  || { echo "Official OpenArm underlay was not found at ${OPENARM_WS}/install." >&2; exit 1; }
[[ -d "${QUESTARMTELEOP_DIR}/src" ]] \
  || { echo "QuestArmTeleop source tree was not found at ${QUESTARMTELEOP_DIR}/src." >&2; exit 1; }
[[ -f "${OPENARM_V1_SOURCE_SCRIPT}" ]] \
  || { echo "OpenArm source helper was not found at ${OPENARM_V1_SOURCE_SCRIPT}." >&2; exit 1; }
[[ -x "${OPENARM_V1_FINGERPRINT_SCRIPT}" ]] \
  || { echo "OpenArm fingerprint helper was not found at ${OPENARM_V1_FINGERPRINT_SCRIPT}." >&2; exit 1; }
[[ "${OPENARM_V1_SKIP_LEGACY_OVERLAY}" == "0" \
    || "${OPENARM_V1_SKIP_LEGACY_OVERLAY}" == "1" ]] \
  || { echo "OPENARM_V1_SKIP_LEGACY_OVERLAY must be 0 or 1." >&2; exit 1; }

had_nounset=0
case $- in
  *u*)
    had_nounset=1
    set +u
    ;;
esac

# Do not source the previous custom overlay while rebuilding it.
OPENARM_V1_OVERLAY_PREFIX=/nonexistent-openarm-v1-overlay \
  source "${OPENARM_V1_SOURCE_SCRIPT}"

if [[ "${had_nounset}" == "1" ]]; then
  set -u
fi

read -r -a selected_packages <<< "${OPENARM_V1_OVERLAY_PACKAGES}"
[[ "${#selected_packages[@]}" -gt 0 ]] \
  || { echo "OPENARM_V1_OVERLAY_PACKAGES is empty." >&2; exit 1; }
[[ "${OPENARM_V1_SYMLINK_INSTALL}" == "0" || "${OPENARM_V1_SYMLINK_INSTALL}" == "1" ]] \
  || { echo "OPENARM_V1_SYMLINK_INSTALL must be 0 or 1." >&2; exit 1; }

colcon_install_args=()
if [[ "${OPENARM_V1_SYMLINK_INSTALL}" == "1" ]]; then
  colcon_install_args+=(--symlink-install)
fi

cd "${QUESTARMTELEOP_DIR}"
colcon --log-base "${OPENARM_V1_OVERLAY_ROOT}/log" build \
  --base-paths src \
  --packages-select "${selected_packages[@]}" \
  "${colcon_install_args[@]}" \
  --build-base "${OPENARM_V1_OVERLAY_ROOT}/build" \
  --install-base "${OPENARM_V1_OVERLAY_ROOT}/install" \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

[[ -f "${OPENARM_V1_OVERLAY_ROOT}/install/setup.bash" \
    && -f "${OPENARM_V1_OVERLAY_ROOT}/install/local_setup.bash" ]] \
  || { echo "Dedicated OpenArm overlay setup files were not generated." >&2; exit 1; }

if [[ "${OPENARM_V1_SKIP_LEGACY_OVERLAY}" == "1" ]]; then
  setup_chain_scripts=()
  for setup_chain_script in "${OPENARM_V1_OVERLAY_ROOT}/install"/setup.*; do
    [[ -f "${setup_chain_script}" ]] || continue
    setup_chain_scripts+=("${setup_chain_script}")
  done
  forbidden_prefixes=(
    /opt/piper_ros/install
    /workspace/install
    "${QUESTARMTELEOP_DIR}/install"
  )
  for forbidden_prefix in "${forbidden_prefixes[@]}"; do
    if contaminated_setup="$(
      grep -F -l -- "${forbidden_prefix}" "${setup_chain_scripts[@]}"
    )"; then
      echo "Generated OpenArm setup chain contains forbidden legacy prefix ${forbidden_prefix}: ${contaminated_setup}" >&2
      exit 1
    fi
  done
fi

OPENARM_V1_OVERLAY_PREFIX="${OPENARM_V1_OVERLAY_ROOT}/install" \
  source "${OPENARM_V1_SOURCE_SCRIPT}"

# A global ``pip check`` is intentionally not used after sourcing ROS.  At that
# point PYTHONPATH exposes apt-managed ROS distributions to the conda
# interpreter, and their optional metadata can produce unrelated false errors.
# Validate only the Python packages this overlay actually executes instead.
python -c \
  "import importlib.metadata as m; import rclpy; from openarm_control import Kinematics; assert m.version('openarm-control') == '0.2.0'; assert m.version('openarm-mujoco') == '2.0.1'; print('OpenArm Python runtime OK')"
mkdir -p "${OPENARM_V1_OVERLAY_ROOT}/test-ros-log"
ROS_LOG_DIR="${OPENARM_V1_OVERLAY_ROOT}/test-ros-log" \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPYCACHEPREFIX="${OPENARM_V1_OVERLAY_ROOT}/pycache" \
  /usr/bin/python3 -m pytest -q --assert=plain -p no:cacheprovider \
  "${QUESTARMTELEOP_DIR}/src/oculus_reader/test/test_openarm_teleop_core.py" \
  "${QUESTARMTELEOP_DIR}/src/oculus_reader/test/test_openarm_launch_contract.py"
ros2 pkg prefix openarm_bringup >/dev/null
for package_name in "${selected_packages[@]}"; do
  package_prefix="$(ros2 pkg prefix "${package_name}")"
  expected_prefix="${OPENARM_V1_OVERLAY_ROOT}/install/${package_name}"
  [[ "$(readlink -f -- "${package_prefix}")" == "$(readlink -f -- "${expected_prefix}")" ]] \
    || { echo "${package_name} resolved outside the dedicated OpenArm overlay: ${package_prefix}" >&2; exit 1; }
done

current_fingerprint="$(WORKSPACE_ROOT=/workspace bash "${OPENARM_V1_FINGERPRINT_SCRIPT}")"
temporary_stamp="$(mktemp "${OPENARM_V1_OVERLAY_ROOT}/.source.sha256.XXXXXX")"
printf '%s\n' "${current_fingerprint}" >"${temporary_stamp}"
chmod 0644 "${temporary_stamp}"
mv -f -- "${temporary_stamp}" "${OPENARM_V1_FINGERPRINT_STAMP}"

echo "OpenArm v1 QuestArm overlay is ready at ${OPENARM_V1_OVERLAY_ROOT}/install"
