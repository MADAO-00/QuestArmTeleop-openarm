#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
IMAGE_NAME="${IMAGE_NAME:-questarm-openarm-v1:humble-standalone-1.0}"
CONTAINER_NAME="${CONTAINER_NAME:-questarm-openarm-v1}"
OPENARM_MODE="${OPENARM_MODE:-fake}"
OPENARM_BUILD_IMAGE="${OPENARM_BUILD_IMAGE:-0}"
OPENARM_BUILD_ONLY="${OPENARM_BUILD_ONLY:-0}"
OPENARM_DOCKER_BUILD_NETWORK="${OPENARM_DOCKER_BUILD_NETWORK:-host}"
OPENARM_ENABLE_QUEST_USB="${OPENARM_ENABLE_QUEST_USB:-0}"
OPENARM_NO_TTY="${OPENARM_NO_TTY:-0}"
OPENARM_AUTO_CONFIGURE_CAN="${OPENARM_AUTO_CONFIGURE_CAN:-1}"
OPENARM_REQUIRE_QUEST="${OPENARM_REQUIRE_QUEST:-1}"
DOCKER_CMD=(docker)
RUN_ARGS=()
RUNTIME_IMAGE="${IMAGE_NAME}"
REAL_LOCK_FILE="${SCRIPT_DIR}/.cache/openarm-vr/real-can0-can1.lock"
REAL_PREFLIGHT_SCRIPT="${SCRIPT_DIR}/scripts/preflight_openarm_v1_ros_real.py"
REAL_CAN_CONFIG_SCRIPT="${SCRIPT_DIR}/scripts/configure_openarm_v1_canfd.sh"
REAL_MOTOR_PREFLIGHT_SCRIPT="${SCRIPT_DIR}/scripts/openarm_can_no_motion_preflight.py"
REAL_GATE_CONTAINER_PATH=/run/openarm-real/preflight.gate
REAL_GATE_DIR=
REAL_GATE_FILE=
REAL_GATE_TOKEN=
REAL_MOTOR_SAFETY_ATTEMPTED=0
REAL_OVERLAY_ROOT="${SCRIPT_DIR}/.openarm_v1_real_overlay_v4"
REAL_OVERLAY_CONTAINER_ROOT=/workspace/.openarm_v1_real_overlay_v4
REAL_SAFETY_IMAGE_DIR=/opt/questarm/openarm-v1-safety
REAL_BASE_IMAGE_NAME=questarm-openarm-base:humble
REAL_BASE_IMAGE_ID=
REAL_SAFETY_SCHEMA=questarm-openarm-v1-real-safety-v7
REAL_ROS_DOMAIN_ID=91
REAL_SAFETY_MARKER_SHA256=53701b4b2323beccb3eca617f39b481cf9d54c48170f3a6e6f137a2d5c3958a3
REAL_SAFETY_VERIFIER_SHA256=6b110d24027588f0398c4fdf39e46525359c5138d5266ec6b10201cad0bcf25a
REAL_HOST_PREFLIGHT_SHA256=5d541310fee80b4af4f6d64d72a0812eeeb1f614c725d897c8facc2ca9e2c1ce
REAL_CAN_CONFIG_SHA256=1cd3cd66044256205f0fd080310460bb27cb32ea6e670c7cf35e9ca7d1250bb2
REAL_MOTOR_SAFETY_SHA256=f712c10adf890026960a70184635d1b3822e904435fe268984109db000dc908e
REAL_OVERLAY_FINGERPRINT_HELPER_SHA256=34f65c3c823e6cad8c31dc86d8d5c42abfb1b15244dd7a7a36ccf6fcf14b59a3
REAL_OVERLAY_REBUILD_SHA256=1c855cc7a2f1d35928056e0d1bd7565e168e43324bb7f1fa911c97a6b46cb8f9
REAL_OVERLAY_SOURCE_SHA256=609a22a53b8a08892076f9a885edadeb55cfeb8c8efa0d1ce74bf8e240788e47
REAL_IK_SAFETY_PATCH_SHA256=602ef06a63840be85a6a063202fd28da9ab3a134882259faeaac6f42b0aee93b
REAL_IK_SAFETY_VERIFIER_SHA256=8aa467dacd8c6d2871e7cf665d378b6e8919606719fe90e3f54287ea15df5238
REAL_PATCHED_KINEMATICS_SHA256=aa0ca15e2b45486c5e62093acee3b26985743834cf57b10bf6aeae6547a77d1d
REAL_OVERLAY_RELEASE_FINGERPRINT=bac1199b1d32f57900c6e5981ddf2c0db035f1c091063d3c43608db8c468c33a
REAL_QUEST_TELEOP_APK_SHA256=6ddd90d8bced3a9533ae36099c950238fdb3ffd5feb5a6daa0afd63ac484cdb0
REAL_SAFETY_ACK_PHRASE=I_CONFIRM_OPENARM_ARMS_SUPPORTED_AREA_CLEAR_ESTOP_READY_GRIPS_RELEASED

die() {
  echo "ERROR: $*" >&2
  exit 1
}

if [[ "${OPENARM_MODE}" == "real" && "$#" -gt 0 ]]; then
  die "Real mode accepts no custom command; it only runs the fixed gated OpenArm teleop launch."
fi

cleanup_real_runtime() {
  local cleanup_status=0

  if [[ "${OPENARM_MODE}" == "real" && "${REAL_MOTOR_SAFETY_ATTEMPTED}" == "1" ]]; then
    echo "Running final whole-robot ID1-8 torque-disable on can0/can1 (ID8 is the gripper motor)."
    if ! run_motor_safety_helper disable-only; then
      echo "ERROR: final whole-robot ID1-8 disable could not be confirmed." >&2
      cleanup_status=1
    fi
  fi

  if [[ -n "${REAL_GATE_FILE}" && -f "${REAL_GATE_FILE}" ]]; then
    rm -f -- "${REAL_GATE_FILE}" || cleanup_status=1
  fi
  if [[ -n "${REAL_GATE_DIR}" && -d "${REAL_GATE_DIR}" ]]; then
    case "$(basename -- "${REAL_GATE_DIR}")" in
      questarm-openarm-v1-real.*)
        rmdir -- "${REAL_GATE_DIR}" || cleanup_status=1
        ;;
      *)
        echo "ERROR: refusing to remove unexpected gate directory: ${REAL_GATE_DIR}" >&2
        cleanup_status=1
        ;;
    esac
  fi
  return "${cleanup_status}"
}

acquire_real_lock() {
  local lock_dir
  local lock_status
  local reserved_name

  [[ "${OPENARM_AUTO_CONFIGURE_CAN}" == "0" || "${OPENARM_AUTO_CONFIGURE_CAN}" == "1" ]] \
    || die "OPENARM_AUTO_CONFIGURE_CAN must be 0 or 1."
  [[ "${OPENARM_REAL_PREFLIGHT_ONLY:-0}" == "0" || "${OPENARM_REAL_PREFLIGHT_ONLY:-0}" == "1" ]] \
    || die "OPENARM_REAL_PREFLIGHT_ONLY must be 0 or 1."
  [[ "${OPENARM_REQUIRE_QUEST}" == "0" || "${OPENARM_REQUIRE_QUEST}" == "1" ]] \
    || die "OPENARM_REQUIRE_QUEST must be 0 or 1."
  [[ "${OPENARM_NO_TTY}" == "0" || "${OPENARM_NO_TTY}" == "1" ]] \
    || die "OPENARM_NO_TTY must be 0 or 1."
  if [[ "${OPENARM_REQUIRE_QUEST}" == "0" && "${OPENARM_REAL_PREFLIGHT_ONLY:-0}" != "1" ]]; then
    die "OPENARM_REQUIRE_QUEST=0 is allowed only with OPENARM_REAL_PREFLIGHT_ONLY=1."
  fi
  [[ "${OPENARM_ENABLE_QUEST_USB}" == "0" ]] \
    || die "Real mode does not map USB devices. Use the host ADB proxy."
  for reserved_name in \
    OPENARM_REAL_PREFLIGHT_PASSED \
    OPENARM_REAL_PREFLIGHT_GATE \
    OPENARM_REAL_PREFLIGHT_TOKEN \
    OPENARM_V1_SKIP_LEGACY_OVERLAY \
    ROS_DOMAIN_ID \
    ROS_LOCALHOST_ONLY; do
    [[ ! -v "${reserved_name}" ]] \
      || die "${reserved_name} is reserved for the locked host preflight."
  done

  command -v flock >/dev/null 2>&1 \
    || die "flock is required for the whole-robot real-mode interlock."
  lock_dir="$(dirname -- "${REAL_LOCK_FILE}")"
  install -d -m 0775 "${lock_dir}"
  exec 9>"${REAL_LOCK_FILE}"
  set +e
  flock --exclusive --nonblock --conflict-exit-code 75 9
  lock_status=$?
  set -e
  if [[ "${lock_status}" -ne 0 ]]; then
    echo "ERROR: another OpenArm real-hardware process holds ${REAL_LOCK_FILE}." >&2
    exit "${lock_status}"
  fi
  echo "Acquired whole-robot lock: ${REAL_LOCK_FILE}"
}

ensure_docker_access() {
  if docker info >/dev/null 2>&1; then
    return
  fi

  command -v sudo >/dev/null 2>&1 \
    || die "Docker daemon is unavailable and sudo is not installed."
  sudo -v
  DOCKER_CMD=(sudo docker)
}

resolve_standalone_base() {
  local base_schema
  REAL_BASE_IMAGE_ID="$("${DOCKER_CMD[@]}" image inspect --format '{{.Id}}' "${REAL_BASE_IMAGE_NAME}")" \
    || die "Standalone base image is missing; build with OPENARM_BUILD_IMAGE=1 OPENARM_BUILD_ONLY=1 ./run_openarm_v1.sh"
  [[ "${REAL_BASE_IMAGE_ID}" =~ ^sha256:[0-9a-f]{64}$ ]] \
    || die "Cannot resolve an immutable standalone base image ID."
  base_schema="$("${DOCKER_CMD[@]}" image inspect \
    --format '{{ index .Config.Labels "org.questarm.openarm-base.schema" }}' "${REAL_BASE_IMAGE_ID}")"
  [[ "${base_schema}" == questarm-openarm-base-humble-v1 ]] \
    || die "Standalone OpenArm base schema does not match."
}

verify_pinned_base_layers() {
  local image_ref=$1
  local index
  local base_layers=()
  local image_layers=()

  mapfile -t base_layers < <(
    "${DOCKER_CMD[@]}" image inspect --format '{{range .RootFS.Layers}}{{println .}}{{end}}' \
      "${REAL_BASE_IMAGE_ID}" | sed '/^$/d'
  )
  mapfile -t image_layers < <(
    "${DOCKER_CMD[@]}" image inspect --format '{{range .RootFS.Layers}}{{println .}}{{end}}' \
      "${image_ref}" | sed '/^$/d'
  )
  [[ "${#base_layers[@]}" -gt 0 \
      && "${#image_layers[@]}" -ge "${#base_layers[@]}" ]] \
    || { echo "ERROR: cannot prove pinned base layers for ${image_ref}." >&2; return 1; }
  for index in "${!base_layers[@]}"; do
    [[ "${image_layers[index]}" == "${base_layers[index]}" ]] \
      || { echo "ERROR: ${image_ref} does not derive from the pinned base image layers." >&2; return 1; }
  done
}

build_openarm_image() {
  local base_build_ref
  local build_context
  local build_status=0
  build_context="$(mktemp -d /tmp/questarm-openarm-v1-build.XXXXXX)"

  cleanup_openarm_build_context() {
    case "${build_context}" in
      /tmp/questarm-openarm-v1-build.*)
        rm -rf -- "${build_context}"
        ;;
      *)
        echo "Refusing to remove unexpected build context: ${build_context}" >&2
        ;;
    esac
  }
  trap cleanup_openarm_build_context EXIT

  cp -- "${SCRIPT_DIR}/Dockerfile.openarm-base" "${build_context}/Dockerfile.openarm-base"
  cp -- "${SCRIPT_DIR}/requirements.txt" "${build_context}/requirements.txt"
  "${DOCKER_CMD[@]}" build \
    --network "${OPENARM_DOCKER_BUILD_NETWORK}" \
    --file "${build_context}/Dockerfile.openarm-base" \
    --tag "${REAL_BASE_IMAGE_NAME}" "${build_context}"
  resolve_standalone_base
  base_build_ref="questarm-openarm-base:build-${REAL_BASE_IMAGE_ID#sha256:}"
  "${DOCKER_CMD[@]}" tag "${REAL_BASE_IMAGE_ID}" "${base_build_ref}"
  [[ "$("${DOCKER_CMD[@]}" image inspect --format '{{.Id}}' "${base_build_ref}")" == "${REAL_BASE_IMAGE_ID}" ]] \
    || die "Immutable base build tag does not match its image ID."

  cp -- "${SCRIPT_DIR}/Dockerfile.openarm-v1" \
    "${build_context}/Dockerfile.openarm-v1"
  cp -- "${SCRIPT_DIR}/openarm_v1.repos" \
    "${build_context}/openarm_v1.repos"
  cp -- "${SCRIPT_DIR}/requirements-openarm-humble.txt" \
    "${build_context}/requirements-openarm-humble.txt"
  mkdir -p "${build_context}/patches"
  mkdir -p "${build_context}/scripts"
  for patch_name in \
    openarm_can-feedback-status.patch \
    openarm_can-strict-feedback.patch \
    openarm_ros2-safe-activation.patch \
    openarm_ros2-feedback-deadline.patch \
    openarm_ros2-runtime-bootstrap.patch \
    openarm_ros2-gripper-safety.patch \
    openarm_ros2-runtime-torque-debounce.patch \
    openarm_ros2-gripper-directional-velocity.patch \
    openarm_ros2-runtime-gripper-velocity-policy.patch \
    openarm_ros2-v1-velocity-limits.patch \
    openarm_description-v1-hand.patch \
    openarm_control-constrained-ik.patch \
    verify_openarm_control_constrained_ik.py \
    verify_openarm_v1_safety_patches.sh; do
    cp -- "${SCRIPT_DIR}/patches/${patch_name}" \
      "${build_context}/patches/${patch_name}"
  done
  for script_name in \
    preflight_openarm_v1_ros_real.py \
    configure_openarm_v1_canfd.sh \
    openarm_can_no_motion_preflight.py \
    openarm_v1_overlay_fingerprint.sh \
    rebuild_openarm_v1_overlay_in_container.sh \
    source_openarm_v1_overlay.sh; do
    cp -- "${SCRIPT_DIR}/scripts/${script_name}" \
      "${build_context}/scripts/${script_name}"
  done

  echo "Building ${IMAGE_NAME} from isolated context ${build_context}"
  echo "Docker build network: ${OPENARM_DOCKER_BUILD_NETWORK}"
  "${DOCKER_CMD[@]}" build \
    --network "${OPENARM_DOCKER_BUILD_NETWORK}" \
    --pull=false \
    --build-arg "OPENARM_BASE_IMAGE=${base_build_ref}" \
    --build-arg "OPENARM_BASE_IMAGE_ID=${REAL_BASE_IMAGE_ID}" \
    --file "${build_context}/Dockerfile.openarm-v1" \
    --tag "${IMAGE_NAME}" \
    "${build_context}" || build_status=$?
  if [[ "${build_status}" -eq 0 ]]; then
    verify_pinned_base_layers "${IMAGE_NAME}" || build_status=$?
  fi
  cleanup_openarm_build_context
  trap - EXIT
  return "${build_status}"
}

run_can_configuration_helper() {
  "${DOCKER_CMD[@]}" run \
    --rm \
    --pull never \
    --network host \
    --cap-drop ALL \
    --cap-add NET_ADMIN \
    --cap-add NET_RAW \
    --security-opt no-new-privileges=true \
    --read-only \
    -e PYTHONDONTWRITEBYTECODE=1 \
    --entrypoint /bin/bash \
    "${RUNTIME_IMAGE}" \
    "${REAL_SAFETY_IMAGE_DIR}/configure_openarm_v1_canfd.sh"
}

run_readonly_can_preflight_helper() {
  local preflight_mode=${1:---identity-only}
  local preflight_args=()
  [[ "${preflight_mode}" == "--identity-only" || "${preflight_mode}" == "--full" ]] \
    || die "invalid read-only CAN preflight mode: ${preflight_mode}"
  if [[ "${preflight_mode}" == "--identity-only" ]]; then
    preflight_args+=(--identity-only)
  fi
  "${DOCKER_CMD[@]}" run \
    --rm \
    --pull never \
    --network host \
    --cap-drop ALL \
    --security-opt no-new-privileges=true \
    --read-only \
    -e PYTHONDONTWRITEBYTECODE=1 \
    --entrypoint /usr/bin/python3 \
    "${RUNTIME_IMAGE}" \
    "${REAL_SAFETY_IMAGE_DIR}/preflight_openarm_v1_ros_real.py" \
      "${preflight_args[@]}"
}

verify_real_image_safety() {
  local actual_base_label
  local actual_entrypoint
  local actual_label
  local actual_ik_patch_sha256
  local actual_ik_verifier_sha256
  local actual_verifier_sha256

  [[ -x "${SCRIPT_DIR}/patches/verify_openarm_v1_safety_patches.sh" ]] \
    || die "missing OpenArm image safety verifier"
  actual_verifier_sha256="$(sha256sum "${SCRIPT_DIR}/patches/verify_openarm_v1_safety_patches.sh" | awk '{print $1}')"
  [[ "${actual_verifier_sha256}" == "${REAL_SAFETY_VERIFIER_SHA256}" ]] \
    || die "host OpenArm image safety verifier checksum mismatch"
  actual_ik_patch_sha256="$(sha256sum "${SCRIPT_DIR}/patches/openarm_control-constrained-ik.patch" | awk '{print $1}')"
  [[ "${actual_ik_patch_sha256}" == "${REAL_IK_SAFETY_PATCH_SHA256}" ]] \
    || die "host constrained-only OpenArm IK patch checksum mismatch"
  actual_ik_verifier_sha256="$(sha256sum "${SCRIPT_DIR}/patches/verify_openarm_control_constrained_ik.py" | awk '{print $1}')"
  [[ "${actual_ik_verifier_sha256}" == "${REAL_IK_SAFETY_VERIFIER_SHA256}" ]] \
    || die "host constrained-only OpenArm IK verifier checksum mismatch"
  actual_label="$("${DOCKER_CMD[@]}" image inspect \
    --format '{{ index .Config.Labels "org.questarm.openarm-v1.real-safety.schema" }}' \
    "${RUNTIME_IMAGE}")"
  [[ "${actual_label}" == "${REAL_SAFETY_SCHEMA}" ]] \
    || die "image '${IMAGE_NAME}' lacks the required real-safety schema label; rebuild with OPENARM_BUILD_IMAGE=1 OPENARM_BUILD_ONLY=1 ./run_openarm_v1.sh"
  actual_base_label="$("${DOCKER_CMD[@]}" image inspect \
    --format '{{ index .Config.Labels "org.questarm.openarm-v1.base-image-id" }}' \
    "${RUNTIME_IMAGE}")"
  [[ "${actual_base_label}" == "${REAL_BASE_IMAGE_ID}" ]] \
    || die "image '${IMAGE_NAME}' does not attest the pinned base image"
  verify_pinned_base_layers "${RUNTIME_IMAGE}" \
    || die "image '${IMAGE_NAME}' does not derive from the pinned base image"
  actual_entrypoint="$("${DOCKER_CMD[@]}" image inspect \
    --format '{{json .Config.Entrypoint}}' "${RUNTIME_IMAGE}")"
  [[ "${actual_entrypoint}" == "null" || "${actual_entrypoint}" == "[]" ]] \
    || die "image '${IMAGE_NAME}' inherits an unreviewed ENTRYPOINT"
  "${DOCKER_CMD[@]}" run \
    --rm \
    --pull never \
    --network none \
    --cap-drop ALL \
    --security-opt no-new-privileges=true \
    --read-only \
    -e "OPENARM_EXPECTED_BASE_IMAGE_ID=${REAL_BASE_IMAGE_ID}" \
    -e "OPENARM_EXPECTED_SAFETY_MARKER_SHA256=${REAL_SAFETY_MARKER_SHA256}" \
    -e "OPENARM_EXPECTED_PREFLIGHT_SHA256=${REAL_HOST_PREFLIGHT_SHA256}" \
    -e "OPENARM_EXPECTED_CAN_CONFIG_SHA256=${REAL_CAN_CONFIG_SHA256}" \
    -e "OPENARM_EXPECTED_MOTOR_SAFETY_SHA256=${REAL_MOTOR_SAFETY_SHA256}" \
    -e "OPENARM_EXPECTED_FINGERPRINT_HELPER_SHA256=${REAL_OVERLAY_FINGERPRINT_HELPER_SHA256}" \
    -e "OPENARM_EXPECTED_OVERLAY_REBUILD_SHA256=${REAL_OVERLAY_REBUILD_SHA256}" \
    -e "OPENARM_EXPECTED_OVERLAY_SOURCE_SHA256=${REAL_OVERLAY_SOURCE_SHA256}" \
    -e "OPENARM_EXPECTED_IK_PATCH_SHA256=${REAL_IK_SAFETY_PATCH_SHA256}" \
    -e "OPENARM_EXPECTED_IK_VERIFIER_SHA256=${REAL_IK_SAFETY_VERIFIER_SHA256}" \
    -e "OPENARM_EXPECTED_PATCHED_KINEMATICS_SHA256=${REAL_PATCHED_KINEMATICS_SHA256}" \
    --mount "type=bind,src=${SCRIPT_DIR}/patches,dst=/safety-patches,readonly" \
    --entrypoint /bin/bash \
    "${RUNTIME_IMAGE}" \
    -lc '
set -euo pipefail
marker=/opt/openarm_ws/.questarm-real-safety-v7
[[ -f "${marker}" && ! -L "${marker}" ]]
[[ "$(stat -c %a "${marker}")" == 444 ]]
[[ "$(stat -c %u:%g "${marker}")" == 0:0 ]]
[[ "$(grep -c '^base_image_id=' "${marker}")" == 1 ]]
grep -Fx "base_image_id=${OPENARM_EXPECTED_BASE_IMAGE_ID}" "${marker}" >/dev/null
marker_checksum="$(sed -E '"'"'s/^base_image_id=sha256:[0-9a-f]{64}$/base_image_id=BUILD_BASE_IMAGE_ID/'"'"' "${marker}" | sha256sum)"
actual_marker_sha256="${marker_checksum%% *}"
[[ "${actual_marker_sha256}" == "${OPENARM_EXPECTED_SAFETY_MARKER_SHA256}" ]]
[[ "$(sha256sum /opt/questarm/openarm-v1-safety/preflight_openarm_v1_ros_real.py | awk '\''{print $1}'\'')" == "${OPENARM_EXPECTED_PREFLIGHT_SHA256}" ]]
[[ "$(sha256sum /opt/questarm/openarm-v1-safety/configure_openarm_v1_canfd.sh | awk '\''{print $1}'\'')" == "${OPENARM_EXPECTED_CAN_CONFIG_SHA256}" ]]
[[ "$(sha256sum /opt/questarm/openarm-v1-safety/openarm_can_no_motion_preflight.py | awk '\''{print $1}'\'')" == "${OPENARM_EXPECTED_MOTOR_SAFETY_SHA256}" ]]
[[ "$(sha256sum /opt/questarm/openarm-v1-safety/openarm_v1_overlay_fingerprint.sh | awk '\''{print $1}'\'')" == "${OPENARM_EXPECTED_FINGERPRINT_HELPER_SHA256}" ]]
[[ "$(sha256sum /opt/questarm/openarm-v1-safety/rebuild_openarm_v1_overlay_in_container.sh | awk '\''{print $1}'\'')" == "${OPENARM_EXPECTED_OVERLAY_REBUILD_SHA256}" ]]
[[ "$(sha256sum /opt/questarm/openarm-v1-safety/source_openarm_v1_overlay.sh | awk '\''{print $1}'\'')" == "${OPENARM_EXPECTED_OVERLAY_SOURCE_SHA256}" ]]
[[ "$(sha256sum /safety-patches/openarm_control-constrained-ik.patch | awk '\''{print $1}'\'')" == "${OPENARM_EXPECTED_IK_PATCH_SHA256}" ]]
[[ "$(sha256sum /safety-patches/verify_openarm_control_constrained_ik.py | awk '\''{print $1}'\'')" == "${OPENARM_EXPECTED_IK_VERIFIER_SHA256}" ]]
kinematics_path=/root/miniforge3/envs/vt/lib/python3.10/site-packages/openarm_control/kinematics.py
[[ "$(sha256sum "${kinematics_path}" | awk '\''{print $1}'\'')" == "${OPENARM_EXPECTED_PATCHED_KINEMATICS_SHA256}" ]]
! grep -F "limits=[]" "${kinematics_path}"
grep -F "Warning: constrained IK solver failed. Skipping step." "${kinematics_path}" >/dev/null
/root/miniforge3/envs/vt/bin/python /safety-patches/verify_openarm_control_constrained_ik.py
[[ "$(stat -c %a /opt/questarm/openarm-v1-safety/preflight_openarm_v1_ros_real.py)" == 555 ]]
[[ "$(stat -c %a /opt/questarm/openarm-v1-safety/configure_openarm_v1_canfd.sh)" == 555 ]]
[[ "$(stat -c %a /opt/questarm/openarm-v1-safety/openarm_can_no_motion_preflight.py)" == 444 ]]
exec bash /safety-patches/verify_openarm_v1_safety_patches.sh \
  /opt/openarm_ws/src/openarm_can \
  /opt/openarm_ws/src/openarm_ros2 \
  /opt/openarm_ws/src/openarm_description
' \
    || die "image '${IMAGE_NAME}' failed real-safety marker/static attestation; rebuild with OPENARM_BUILD_IMAGE=1 OPENARM_BUILD_ONLY=1 ./run_openarm_v1.sh"
}

verify_real_host_sources() {
  local actual
  local checksum
  local expected
  local relative
  local specification
  local specifications=(
    "${REAL_HOST_PREFLIGHT_SHA256} scripts/preflight_openarm_v1_ros_real.py"
    "${REAL_CAN_CONFIG_SHA256} scripts/configure_openarm_v1_canfd.sh"
    "${REAL_MOTOR_SAFETY_SHA256} scripts/openarm_can_no_motion_preflight.py"
    "${REAL_OVERLAY_FINGERPRINT_HELPER_SHA256} scripts/openarm_v1_overlay_fingerprint.sh"
    "${REAL_OVERLAY_REBUILD_SHA256} scripts/rebuild_openarm_v1_overlay_in_container.sh"
    "${REAL_OVERLAY_SOURCE_SHA256} scripts/source_openarm_v1_overlay.sh"
  )

  for specification in "${specifications[@]}"; do
    expected="${specification%% *}"
    relative="${specification#* }"
    [[ -f "${SCRIPT_DIR}/${relative}" && ! -L "${SCRIPT_DIR}/${relative}" ]] \
      || die "real safety source must be a regular non-symlink file: ${relative}"
    checksum="$(sha256sum -- "${SCRIPT_DIR}/${relative}")"
    actual="${checksum%% *}"
    [[ "${actual}" == "${expected}" ]] \
      || die "real safety source checksum mismatch: ${relative}"
  done

  actual="$(
    WORKSPACE_ROOT="${SCRIPT_DIR}" \
      bash "${SCRIPT_DIR}/scripts/openarm_v1_overlay_fingerprint.sh"
  )"
  [[ "${actual}" == "${REAL_OVERLAY_RELEASE_FINGERPRINT}" ]] \
    || die "OpenArm real overlay source does not match the reviewed release fingerprint"
}

verify_real_overlay_release() {
  local destination
  local relative
  local safety_files=(
    launch/teleop_double_openarm_v1.launch.py
    config/openarm_v1_bimanual_controllers.yaml
    config/openarm_v1_teleop.yaml
    scripts/pub_pose_openarm_v1.py
    scripts/pub_delta_pose_openarm_v1.py
    scripts/openarm_bimanual_ik_node.py
    scripts/openarm_command_guard_node.py
    scripts/openarm_teleop_core.py
    scripts/oculus_reader.py
    scripts/transformations.py
    scripts/buttons_parser.py
    scripts/FPS_counter.py
  )

  [[ -f "${REAL_OVERLAY_ROOT}/install/setup.bash" \
      && -f "${REAL_OVERLAY_ROOT}/install/local_setup.bash" ]] || return 1
  [[ -f "${REAL_OVERLAY_ROOT}/source.sha256" \
      && "$(wc -c <"${REAL_OVERLAY_ROOT}/source.sha256")" == 65 \
      && "$(<"${REAL_OVERLAY_ROOT}/source.sha256")" == "${REAL_OVERLAY_RELEASE_FINGERPRINT}" ]] \
    || return 1
  [[ -z "$(find "${REAL_OVERLAY_ROOT}/install" -type l -print -quit)" ]] \
    || return 1
  ! grep -R -F -e /opt/piper_ros/install -e /workspace/install \
      "${REAL_OVERLAY_ROOT}/install/setup.bash" \
      "${REAL_OVERLAY_ROOT}/install/setup.sh" >/dev/null \
    || return 1

  for relative in "${safety_files[@]}"; do
    case "${relative}" in
      launch/*|config/*)
        destination="${REAL_OVERLAY_ROOT}/install/oculus_reader/share/oculus_reader/${relative}"
        ;;
      scripts/*)
        destination="${REAL_OVERLAY_ROOT}/install/oculus_reader/lib/oculus_reader/${relative#scripts/}"
        ;;
    esac
    [[ -f "${destination}" && ! -L "${destination}" ]] || return 1
    cmp -s \
      "${SCRIPT_DIR}/src/oculus_reader/${relative}" \
      "${destination}" \
      || return 1
  done
  destination="${REAL_OVERLAY_ROOT}/install/oculus_reader/share/oculus_reader/package.xml"
  [[ -f "${destination}" && ! -L "${destination}" ]] || return 1
  cmp -s \
    "${SCRIPT_DIR}/src/oculus_reader/package.xml" \
    "${destination}" || return 1
  # The launch loads the installed model; attest those bytes as well as source.
  diff -qr -- "${SCRIPT_DIR}/assets/openarm_mujoco" \
    "${REAL_OVERLAY_ROOT}/install/oculus_reader/share/oculus_reader/assets/openarm_mujoco" \
    >/dev/null || return 1
}

ensure_real_overlay_release() {
  if [[ -e "${REAL_OVERLAY_ROOT}/install/setup.bash" ]]; then
    verify_real_overlay_release \
      || die "existing real OpenArm overlay is not the reviewed non-symlink release"
    return
  fi

  install -d -m 0755 "${REAL_OVERLAY_ROOT}"
  echo "Building reviewed non-symlink OpenArm real overlay without network or CAN capabilities."
  "${DOCKER_CMD[@]}" run \
    --rm \
    --pull never \
    --network none \
    --cap-drop ALL \
    --cap-add DAC_OVERRIDE \
    --security-opt no-new-privileges=true \
    --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,exec,size=2g,mode=1777 \
    -e ROS_DISTRO="${ROS_DISTRO:-humble}" \
    -e "OPENARM_V1_OVERLAY_ROOT=${REAL_OVERLAY_CONTAINER_ROOT}" \
    -e OPENARM_V1_SYMLINK_INSTALL=0 \
    -e OPENARM_V1_SKIP_LEGACY_OVERLAY=1 \
    -e "OPENARM_V1_SOURCE_SCRIPT=${REAL_SAFETY_IMAGE_DIR}/source_openarm_v1_overlay.sh" \
    -e "OPENARM_V1_FINGERPRINT_SCRIPT=${REAL_SAFETY_IMAGE_DIR}/openarm_v1_overlay_fingerprint.sh" \
    --mount "type=bind,src=${SCRIPT_DIR},dst=/workspace,readonly" \
    --mount "type=bind,src=${REAL_OVERLAY_ROOT},dst=${REAL_OVERLAY_CONTAINER_ROOT}" \
    -w /workspace \
    --entrypoint /bin/bash \
    "${RUNTIME_IMAGE}" \
    -lc 'exec bash /opt/questarm/openarm-v1-safety/rebuild_openarm_v1_overlay_in_container.sh'
  verify_real_overlay_release \
    || die "new real OpenArm overlay failed the non-symlink release verification"
}

run_motor_safety_helper() {
  local helper_mode=$1

  [[ "${helper_mode}" == "preflight" || "${helper_mode}" == "disable-only" ]] \
    || die "invalid motor safety helper mode: ${helper_mode}"
  "${DOCKER_CMD[@]}" run \
    --rm \
    --pull never \
    --network host \
    --cap-drop ALL \
    --cap-add NET_RAW \
    --security-opt no-new-privileges=true \
    --read-only \
    -e PYTHONDONTWRITEBYTECODE=1 \
    --entrypoint /usr/bin/python3 \
    "${RUNTIME_IMAGE}" \
    "${REAL_SAFETY_IMAGE_DIR}/openarm_can_no_motion_preflight.py" \
      --mode "${helper_mode}"
}

create_real_preflight_gate() {
  local runtime_base

  runtime_base="${XDG_RUNTIME_DIR:-/tmp}"
  [[ "${runtime_base}" == /* && -d "${runtime_base}" && -w "${runtime_base}" ]] \
    || die "real preflight gate runtime directory is not a writable absolute directory: ${runtime_base}"
  REAL_GATE_DIR="$(mktemp -d "${runtime_base%/}/questarm-openarm-v1-real.XXXXXX")"
  chmod 0700 "${REAL_GATE_DIR}"
  REAL_GATE_FILE="${REAL_GATE_DIR}/preflight.gate"
  REAL_GATE_TOKEN="$(od -An -N32 -tx1 /dev/urandom | tr -d ' \n')"
  [[ "${REAL_GATE_TOKEN}" =~ ^[0-9a-f]{64}$ ]] \
    || die "failed to generate the real preflight nonce"
  (umask 077; printf '%s\n' "${REAL_GATE_TOKEN}" >"${REAL_GATE_FILE}")
  # The host creates this as the desktop user, while the capability-dropped
  # control container reads the exact file bind mount as UID 0 without
  # CAP_DAC_OVERRIDE. 0444 keeps the nonce immutable inside the read-only mount
  # and readable across that UID boundary; the parent remains private (0700).
  chmod 0444 "${REAL_GATE_FILE}"
}

require_host_quest() (
  # The whole-robot lock must remain owned by the parent runner, but adb may
  # start a detached host server that outlives this invocation. Close only this
  # subshell's copy so neither adb nor any of its descendants can retain FD 9.
  exec 9>&-
  local adb_output
  local installed_apk_sha256
  local package_path
  local quest_serial
  local quest_state
  local remote_package_path
  local quest_rows=()

  [[ "${OPENARM_REQUIRE_QUEST}" == "1" ]] || return
  command -v adb >/dev/null 2>&1 \
    || die "OPENARM_REQUIRE_QUEST=1 but host adb is unavailable"
  adb_output="$(adb devices)" \
    || die "host adb devices failed"
  # Count every USB-eligible ADB row, not only authorized rows.  Otherwise an
  # authorized Quest plus an earlier unauthorized/offline device could pass
  # this gate while pure-python-adb later selects the wrong first device.
  mapfile -t quest_rows < <(
    awk 'NR > 1 && NF >= 2 { serial = $1; if (gsub(/\./, ".", serial) < 3) print $1 "\t" $2 }' \
      <<<"${adb_output}"
  )
  [[ "${#quest_rows[@]}" -eq 1 ]] \
    || die "real launch requires exactly one USB-eligible Quest ADB entry across all states; found ${#quest_rows[@]}. Use OPENARM_REAL_PREFLIGHT_ONLY=1 until Quest is ready"
  IFS=$'\t' read -r quest_serial quest_state <<<"${quest_rows[0]}"
  [[ "${quest_state}" == "device" ]] \
    || die "the only USB-eligible Quest ADB entry is not authorized and ready: serial=${quest_serial}, state=${quest_state}"
  package_path="$(
    adb -s "${quest_serial}" shell pm path com.rail.oculus.teleop 2>/dev/null \
      | tr -d '\r'
  )" \
    || die "cannot query com.rail.oculus.teleop on Quest ${quest_serial}"
  [[ -n "${package_path}" && "${package_path}" == package:* ]] \
    || die "Quest ${quest_serial} is missing com.rail.oculus.teleop; install it first with: adb -s ${quest_serial} install -r ${SCRIPT_DIR}/src/oculus_reader/APK/teleop-debug.apk"
  [[ "${package_path}" != *$'\n'* ]] \
    || die "Quest ${quest_serial} returned multiple base APK paths for com.rail.oculus.teleop"
  remote_package_path="${package_path#package:}"
  installed_apk_sha256="$(
    adb -s "${quest_serial}" shell sha256sum "${remote_package_path}" 2>/dev/null \
      | tr -d '\r' \
      | awk 'NR == 1 { print $1 }'
  )" \
    || die "cannot checksum com.rail.oculus.teleop on Quest ${quest_serial}"
  [[ "${installed_apk_sha256}" == "${REAL_QUEST_TELEOP_APK_SHA256}" ]] \
    || die "Quest ${quest_serial} has an incompatible teleop APK (sha256=${installed_apk_sha256:-unavailable}); replace it with the pinned Quest 3/OpenXR APK: adb -s ${quest_serial} uninstall com.rail.oculus.teleop && adb -s ${quest_serial} install ${SCRIPT_DIR}/src/oculus_reader/APK/teleop-debug.apk"
  echo "Host Quest ADB readiness passed: serial=${quest_serial}, pinned Quest 3/OpenXR teleop APK verified."
)

confirm_real_safety_ack() {
  local entered_ack

  if [[ "${OPENARM_NO_TTY}" == "1" ]]; then
    [[ "${OPENARM_REAL_SAFETY_ACK:-}" == "${REAL_SAFETY_ACK_PHRASE}" ]] \
      || die "non-interactive real mode requires OPENARM_REAL_SAFETY_ACK=${REAL_SAFETY_ACK_PHRASE}"
    return
  fi

  [[ -t 0 && -t 1 ]] \
    || die "interactive safety confirmation needs a TTY; otherwise set OPENARM_NO_TTY=1 and the explicit safety ACK"
  echo "Real OpenArm safety confirmation required before whole-robot ID1-8 torque-disable/enable on both CAN buses:"
  echo "  - both arms are physically supported; arm motors ID1-7 can be safely torque-disabled/enabled"
  echo "  - both grippers are clear and supported; gripper motor ID8 can be safely torque-disabled/enabled"
  echo "  - the full dual-arm-and-gripper workspace is clear and the emergency stop is reachable"
  echo "  - both Quest Grip/deadman and Index Trigger/gripper controls are released"
  printf '确认以上全部安全条件后，直接按 Enter 继续；输入任何其他内容后按 Enter 取消。\n> '
  IFS= read -r entered_ack \
    || die "real safety confirmation cancelled; no CAN helper was started"
  [[ -z "${entered_ack}" ]] \
    || die "real safety confirmation cancelled; no CAN helper was started"
}

prepare_real_host() {
  [[ -x "${REAL_PREFLIGHT_SCRIPT}" ]] \
    || die "missing executable host preflight: ${REAL_PREFLIGHT_SCRIPT}"
  [[ -x "${REAL_CAN_CONFIG_SCRIPT}" ]] \
    || die "missing executable CAN-FD configurator: ${REAL_CAN_CONFIG_SCRIPT}"
  [[ -x "${REAL_MOTOR_PREFLIGHT_SCRIPT}" ]] \
    || die "missing executable motor preflight: ${REAL_MOTOR_PREFLIGHT_SCRIPT}"

  # These files are mounted into containers that receive CAN capabilities.
  # Pin them, plus the complete safety-critical ROS overlay release, before
  # the first host identity script or SocketCAN helper is executed.
  verify_real_host_sources

  # Never expose CAN to an image that still contains upstream activation-time
  # return-to-zero behavior or lacks the reviewed ID8 gripper safety path.
  verify_real_image_safety
  ensure_real_overlay_release

  # A normal launch must prove the exact host-ADB/installed-app path before any
  # helper sees SocketCAN.  Preflight-only intentionally audits the robot with
  # no Quest attached and never creates a launch gate.
  if [[ "${OPENARM_REAL_PREFLIGHT_ONLY:-0}" != "1" ]]; then
    require_host_quest
  fi
  confirm_real_safety_ack

  # The audited helper is baked into the immutable image.  It receives no
  # capabilities and proves both PEAK channel identities and receiver
  # ownership before any CAN-capable process is created.
  run_readonly_can_preflight_helper --identity-only

  if [[ "${OPENARM_AUTO_CONFIGURE_CAN}" == "1" ]]; then
    run_can_configuration_helper
  fi

  # Recheck the complete link contract, then use a separate NET_RAW-only
  # helper for the no-motion whole-robot ID1-8 motor audit on both CAN buses.
  run_readonly_can_preflight_helper --full
  REAL_MOTOR_SAFETY_ATTEMPTED=1
  run_motor_safety_helper preflight
  run_readonly_can_preflight_helper --full

  if [[ "${OPENARM_REAL_PREFLIGHT_ONLY:-0}" == "1" ]]; then
    echo "OpenArm real preflight-only mode passed; ROS was not started and no gate was created."
    return
  fi
  create_real_preflight_gate
}

prepare_mode_args() {
  local network_mode

  case "${OPENARM_MODE}" in
    fake)
      network_mode="${OPENARM_NETWORK_MODE:-bridge}"
      [[ "${network_mode}" != "host" ]] \
        || die "Host networking is disabled in the fake-only wrapper because it can expose configured host SocketCAN interfaces."
      RUN_ARGS+=(--network "${network_mode}")
      RUN_ARGS+=(--cap-drop NET_ADMIN --cap-drop NET_RAW)
      RUN_ARGS+=(-e OPENARM_HARDWARE_MODE=fake)
      RUN_ARGS+=(-e OPENARM_USE_FAKE_HARDWARE=true)
      ;;
    real)
      network_mode="${OPENARM_NETWORK_MODE:-host}"
      [[ "${network_mode}" == "host" ]] \
        || die "Real mode requires OPENARM_NETWORK_MODE=host for SocketCAN."
      [[ -n "${REAL_GATE_FILE}" && -f "${REAL_GATE_FILE}" ]] \
        || die "real preflight gate is missing"
      RUN_ARGS+=(--network host)
      RUN_ARGS+=(--cap-drop ALL --cap-add NET_RAW)
      RUN_ARGS+=(--security-opt no-new-privileges=true)
      RUN_ARGS+=(--pull never)
      RUN_ARGS+=(-e OPENARM_HARDWARE_MODE=real)
      RUN_ARGS+=(-e OPENARM_USE_FAKE_HARDWARE=false)
      RUN_ARGS+=(-e OPENARM_CAN_LEFT=can0)
      RUN_ARGS+=(-e OPENARM_CAN_RIGHT=can1)
      RUN_ARGS+=(-e OPENARM_CAN_BUS_INFO=3-1:1.0)
      RUN_ARGS+=(-e OPENARM_CAN_LEFT_DEV_ID=0)
      RUN_ARGS+=(-e OPENARM_CAN_RIGHT_DEV_ID=1)
      RUN_ARGS+=(-e OPENARM_CAN_DRIVER=peak_usb)
      RUN_ARGS+=(-e OPENARM_CAN_ARBITRATION_BITRATE=1000000)
      RUN_ARGS+=(-e OPENARM_CAN_DATA_BITRATE=5000000)
      RUN_ARGS+=(-e "ROS_DOMAIN_ID=${REAL_ROS_DOMAIN_ID}")
      RUN_ARGS+=(-e ROS_LOCALHOST_ONLY=1)
      RUN_ARGS+=(-e OPENARM_V1_SKIP_LEGACY_OVERLAY=1)
      RUN_ARGS+=(-e OPENARM_REAL_PREFLIGHT_PASSED=1)
      RUN_ARGS+=(-e "OPENARM_REAL_PREFLIGHT_GATE=${REAL_GATE_CONTAINER_PATH}")
      RUN_ARGS+=(-e "OPENARM_REAL_PREFLIGHT_TOKEN=${REAL_GATE_TOKEN}")
      RUN_ARGS+=(--mount "type=bind,src=${REAL_GATE_FILE},dst=${REAL_GATE_CONTAINER_PATH},readonly")
      ;;
    *)
      die "OPENARM_MODE must be 'fake' or 'real', got '${OPENARM_MODE}'."
      ;;
  esac
}

prepare_optional_usb_args() {
  if [[ "${OPENARM_ENABLE_QUEST_USB}" != "1" ]]; then
    return
  fi

  [[ -d /dev/bus/usb ]] \
    || die "OPENARM_ENABLE_QUEST_USB=1, but /dev/bus/usb is unavailable."
  RUN_ARGS+=(-v /dev/bus/usb:/dev/bus/usb)
  RUN_ARGS+=(--device-cgroup-rule "c 189:* rwm")
  echo "Quest USB forwarding explicitly enabled."
}

prepare_display_args() {
  if [[ -z "${DISPLAY:-}" ]] || [[ ! -d /tmp/.X11-unix ]]; then
    return
  fi

  RUN_ARGS+=(-e "DISPLAY=${DISPLAY}")
  RUN_ARGS+=(-e QT_X11_NO_MITSHM=1)
  RUN_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)

  if [[ -n "${XAUTHORITY:-}" ]] && [[ -f "${XAUTHORITY}" ]]; then
    RUN_ARGS+=(-e XAUTHORITY=/tmp/.docker.xauth)
    RUN_ARGS+=(-v "${XAUTHORITY}:/tmp/.docker.xauth:ro")
  elif [[ -f "${HOME}/.Xauthority" ]]; then
    RUN_ARGS+=(-e XAUTHORITY=/tmp/.docker.xauth)
    RUN_ARGS+=(-v "${HOME}/.Xauthority:/tmp/.docker.xauth:ro")
  fi
}

ensure_docker_access

if [[ "${OPENARM_BUILD_IMAGE}" == "1" ]]; then
  build_openarm_image
fi

if [[ "${OPENARM_BUILD_ONLY}" == "1" ]]; then
  [[ "${OPENARM_BUILD_IMAGE}" == "1" ]] \
    || die "OPENARM_BUILD_ONLY=1 requires OPENARM_BUILD_IMAGE=1."
  exit 0
fi

finish_real_runtime() {
  local status=$?
  trap - EXIT
  if ! cleanup_real_runtime; then
    [[ "${status}" -ne 0 ]] || status=1
  fi
  exit "${status}"
}

if [[ "${OPENARM_MODE}" == "real" ]]; then
  acquire_real_lock
  trap finish_real_runtime EXIT
fi

"${DOCKER_CMD[@]}" image inspect "${IMAGE_NAME}" >/dev/null 2>&1 \
  || die "Image '${IMAGE_NAME}' is missing. Build it with: OPENARM_BUILD_IMAGE=1 OPENARM_BUILD_ONLY=1 ./run_openarm_v1.sh"

if [[ "${OPENARM_MODE}" == "real" ]]; then
  RUNTIME_IMAGE="$("${DOCKER_CMD[@]}" image inspect --format '{{.Id}}' "${IMAGE_NAME}")"
  [[ "${RUNTIME_IMAGE}" =~ ^sha256:[0-9a-f]{64}$ ]] \
    || die "cannot resolve immutable image ID for '${IMAGE_NAME}'"
  resolve_standalone_base
  prepare_real_host
  if [[ "${OPENARM_REAL_PREFLIGHT_ONLY:-0}" == "1" ]]; then
    exit 0
  fi
fi

if [[ "${OPENARM_NO_TTY}" == "1" ]]; then
  RUN_ARGS+=(--rm)
else
  RUN_ARGS+=(--rm -it)
fi
RUN_ARGS+=(--name "${CONTAINER_NAME}")
if [[ "${OPENARM_MODE}" == "real" ]]; then
  RUN_ARGS+=(-v "${SCRIPT_DIR}:/workspace:ro")
  RUN_ARGS+=(-e "OPENARM_V1_OVERLAY_PREFIX=${REAL_OVERLAY_CONTAINER_ROOT}/install")
else
  RUN_ARGS+=(-v "${SCRIPT_DIR}:/workspace")
fi
RUN_ARGS+=(-w /workspace)
RUN_ARGS+=(-e "TZ=${TZ:-Asia/Shanghai}")
RUN_ARGS+=(-e "ROS_DISTRO=${ROS_DISTRO:-humble}")

for env_name in RMW_IMPLEMENTATION; do
  if [[ -n "${!env_name:-}" ]]; then
    RUN_ARGS+=(-e "${env_name}=${!env_name}")
  fi
done

# The baked base image contains localhost proxy variables. They are useful for
# the host-network image build but invalid inside the default bridge runtime.
RUN_ARGS+=(-e HTTP_PROXY= -e HTTPS_PROXY= -e http_proxy= -e https_proxy=)

prepare_mode_args
prepare_optional_usb_args
prepare_display_args
RUN_ARGS+=(--entrypoint /bin/bash)

echo "Starting ${CONTAINER_NAME} from ${IMAGE_NAME} in ${OPENARM_MODE} mode."
if [[ "${OPENARM_MODE}" == "fake" ]]; then
  echo "Fake mode does not expose host devices, CAN capabilities, or host networking by default."
elif (( $# == 0 )); then
  echo "Real OpenArm whole-robot launch: arm motors ID1-7 plus gripper motor ID8 are enabled on can0/can1."
  echo "Quest Grip/deadman gates arm motion only; each Index Trigger independently closes its gripper, and released Triggers command guarded opening."
  set -- ros2 launch oculus_reader teleop_double_openarm_v1.launch.py \
    use_fake_hardware:=false \
    left_can_interface:=can0 \
    right_can_interface:=can1 \
    enable_home:=false
fi

if [[ "${OPENARM_MODE}" == "real" ]]; then
  # Recheck immediately before exec so edits after the initial preflight cannot
  # replace the reviewed launch, controllers, or CAN-capable helper scripts.
  verify_real_host_sources
  verify_real_overlay_release \
    || die "real OpenArm overlay changed after preflight"
fi

"${DOCKER_CMD[@]}" run "${RUN_ARGS[@]}" "${RUNTIME_IMAGE}" \
  -lc '
set -euo pipefail
if [[ "${OPENARM_HARDWARE_MODE:-}" == real ]]; then
  [[ "${ROS_DOMAIN_ID:-}" == 91 ]] \
    || { echo "Invalid real ROS domain." >&2; exit 70; }
  [[ "${OPENARM_V1_SKIP_LEGACY_OVERLAY:-}" == 1 ]] \
    || { echo "Legacy overlay isolation is not active." >&2; exit 70; }
  [[ "${OPENARM_REAL_PREFLIGHT_PASSED:-}" == 1 ]] \
    || { echo "Missing real preflight marker." >&2; exit 70; }
  [[ "${OPENARM_REAL_PREFLIGHT_GATE:-}" == /run/openarm-real/preflight.gate ]] \
    || { echo "Invalid real preflight gate path." >&2; exit 70; }
  [[ "${OPENARM_REAL_PREFLIGHT_TOKEN:-}" =~ ^[0-9a-f]{64}$ ]] \
    || { echo "Invalid real preflight nonce." >&2; exit 70; }
  [[ -f "${OPENARM_REAL_PREFLIGHT_GATE}" && ! -L "${OPENARM_REAL_PREFLIGHT_GATE}" ]] \
    || { echo "Real preflight gate is not a regular file." >&2; exit 70; }
  [[ "$(stat -c %a "${OPENARM_REAL_PREFLIGHT_GATE}")" == 444 ]] \
    || { echo "Real preflight gate mode is not 0444." >&2; exit 70; }
  [[ "$(wc -c <"${OPENARM_REAL_PREFLIGHT_GATE}")" == 65 ]] \
    || { echo "Real preflight gate length is invalid." >&2; exit 70; }
  [[ "$(<"${OPENARM_REAL_PREFLIGHT_GATE}")" == "${OPENARM_REAL_PREFLIGHT_TOKEN}" ]] \
    || { echo "Real preflight nonce does not match its gate." >&2; exit 70; }
  /usr/bin/python3 /opt/questarm/openarm-v1-safety/preflight_openarm_v1_ros_real.py
  /usr/bin/python3 /opt/questarm/openarm-v1-safety/openarm_can_no_motion_preflight.py --mode socket-smoke
fi
current_overlay_fingerprint="$(WORKSPACE_ROOT=/workspace bash /opt/questarm/openarm-v1-safety/openarm_v1_overlay_fingerprint.sh)"
if [[ "${OPENARM_HARDWARE_MODE:-}" == real ]]; then
  installed_overlay_fingerprint="$(cat /workspace/.openarm_v1_real_overlay_v4/source.sha256 2>/dev/null || true)"
  [[ -f /workspace/.openarm_v1_real_overlay_v4/install/local_setup.bash \
      && "${installed_overlay_fingerprint}" == "${current_overlay_fingerprint}" \
      && -z "$(find /workspace/.openarm_v1_real_overlay_v4/install -type l -print -quit)" \
      && -z "$(grep -R -F -l -e /opt/piper_ros/install -e /workspace/install /workspace/.openarm_v1_real_overlay_v4/install/setup.bash /workspace/.openarm_v1_real_overlay_v4/install/setup.sh 2>/dev/null)" ]] \
    || { echo "Reviewed non-symlink real overlay is missing or stale." >&2; exit 70; }
else
  installed_overlay_fingerprint="$(cat /workspace/.openarm_v1_overlay/source.sha256 2>/dev/null || true)"
  if [[ ! -f /workspace/.openarm_v1_overlay/install/setup.bash \
        || "${installed_overlay_fingerprint}" != "${current_overlay_fingerprint}" ]]; then
    echo "OpenArm QuestArm overlay is missing or stale; rebuilding it from the mounted source tree."
    bash /workspace/scripts/rebuild_openarm_v1_overlay_in_container.sh
  fi
fi
if [[ "${OPENARM_HARDWARE_MODE:-}" == real ]]; then
  source /opt/questarm/openarm-v1-safety/source_openarm_v1_overlay.sh
else
  source /workspace/scripts/source_openarm_v1_overlay.sh
fi
if [[ "${OPENARM_HARDWARE_MODE:-}" == real ]]; then
  actual_oculus_prefix="$(ros2 pkg prefix oculus_reader)"
  expected_oculus_prefix="${OPENARM_V1_OVERLAY_PREFIX}/oculus_reader"
  [[ "$(readlink -f -- "${actual_oculus_prefix}")" == "$(readlink -f -- "${expected_oculus_prefix}")" ]] \
    || { echo "oculus_reader resolved outside the reviewed real overlay: ${actual_oculus_prefix}" >&2; exit 70; }
fi
if (( $# > 0 )); then
  exec "$@"
fi
exec bash --noprofile --norc -i
' -- "$@"
