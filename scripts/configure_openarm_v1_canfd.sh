#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
PREFLIGHT="${SCRIPT_DIR}/preflight_openarm_v1_ros_real.py"
INTERFACES=(can0 can1)
ARBITRATION_BITRATE=1000000
DATA_BITRATE=5000000
CONFIGURATION_COMPLETE=0
SUDO_CMD=()

die() {
  echo "configure_openarm_v1_canfd.sh: ERROR: $*" >&2
  exit 1
}

fail_closed() {
  local status=${1:-1}
  [[ "${status}" -ne 0 ]] || status=1
  if [[ "${CONFIGURATION_COMPLETE}" != "1" ]]; then
    echo "CAN-FD configuration did not complete; forcing can0 and can1 DOWN." >&2
    for interface in "${INTERFACES[@]}"; do
      "${SUDO_CMD[@]}" ip link set dev "${interface}" down >/dev/null 2>&1 || true
    done
  fi
  exit "${status}"
}

[[ -x "${PREFLIGHT}" ]] || die "missing executable preflight: ${PREFLIGHT}"
command -v ip >/dev/null 2>&1 || die "ip is required"
command -v ethtool >/dev/null 2>&1 || die "ethtool is required"

# Verify the immutable USB/PEAK channel identity and that no CAN receiver owns
# either bus before changing link state.  This stage never changes an interface.
python3 "${PREFLIGHT}" --identity-only

if (( EUID != 0 )); then
  command -v sudo >/dev/null 2>&1 || die "sudo is required to configure CAN-FD"
  sudo -n true >/dev/null 2>&1 \
    || die "passwordless/cached sudo is required for non-interactive CAN setup"
  SUDO_CMD=(sudo -n)
fi

trap 'fail_closed $?' ERR
trap 'fail_closed 130' INT
trap 'fail_closed 143' TERM

for interface in "${INTERFACES[@]}"; do
  "${SUDO_CMD[@]}" ip link set dev "${interface}" down
  "${SUDO_CMD[@]}" ip link set dev "${interface}" type can \
    bitrate "${ARBITRATION_BITRATE}" \
    dbitrate "${DATA_BITRATE}" \
    fd on \
    restart-ms 100
  "${SUDO_CMD[@]}" ip link set dev "${interface}" up
done

# Fail closed unless both configured links now pass the complete read-only audit.
python3 "${PREFLIGHT}"
CONFIGURATION_COMPLETE=1
trap - ERR INT TERM

echo "Configured OpenArm v1 CAN-FD: left=can0, right=can1, arbitration=1M, data=5M."
