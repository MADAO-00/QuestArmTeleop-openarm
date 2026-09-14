#!/usr/bin/env bash
set -euo pipefail

[[ "${OPENARM_HARDWARE_MODE:-}" == "fake" ]] \
  || { echo "This smoke test must run through run_openarm_v1.sh in fake mode." >&2; exit 2; }

QUESTARMTELEOP_DIR="${QUESTARMTELEOP_DIR:-/workspace}"
source /workspace/scripts/source_openarm_v1_overlay.sh

smoke_tmp="$(mktemp -d /tmp/openarm-v1-fake-smoke.XXXXXX)"
launch_pid=""

cleanup() {
  local status=$?
  set +e
  if [[ -n "${launch_pid}" ]]; then
    kill -INT "${launch_pid}" 2>/dev/null || true
  fi
  if [[ "${status}" -ne 0 && -f "${smoke_tmp}/launch.log" ]]; then
    echo "OpenArm fake launch log (failure tail):" >&2
    sed -r 's/\x1B\[[0-9;]*[mK]//g' "${smoke_tmp}/launch.log" \
      | tail -200 >&2
  fi
  case "${smoke_tmp}" in
    /tmp/openarm-v1-fake-smoke.*) rm -rf -- "${smoke_tmp}" ;;
    *) echo "Refusing to remove unexpected smoke path: ${smoke_tmp}" >&2 ;;
  esac
  return "${status}"
}
trap cleanup EXIT

stop_launch_and_verify_clean_exit() {
  local launch_status
  local process_count
  local process_name
  local escaped_name
  local required_count
  local sanitized_log="${smoke_tmp}/launch.clean.log"

  kill -INT "${launch_pid}"
  set +e
  wait "${launch_pid}"
  launch_status=$?
  set -e
  launch_pid=""

  sed -r 's/\x1B\[[0-9;]*[mK]//g' "${smoke_tmp}/launch.log" >"${sanitized_log}"
  if grep -E \
      'Traceback|RCLError|rcl_shutdown already called|process has died' \
      "${sanitized_log}" >/dev/null; then
    echo "OpenArm fake launch produced an unclean SIGINT exit:" >&2
    tail -200 "${sanitized_log}" >&2
    return 1
  fi

  # start_quest_publisher=false intentionally omits pub_pose_openarm_v1.py in
  # fake mode because that node owns the physical Quest/ADB connection.  Its
  # identical shutdown contract is covered by the package source test; here we
  # require every Python runtime process that this fake launch actually starts.
  for process_name in \
      pub_delta_pose_openarm_v1.py \
      openarm_bimanual_ik_node.py \
      openarm_command_guard_node.py; do
    required_count=1
    [[ "${process_name}" == "pub_delta_pose_openarm_v1.py" ]] && required_count=2
    escaped_name="${process_name//./\\.}"
    process_count="$(
      grep -E -c \
        "^\\[INFO\\] \\[${escaped_name}-[0-9]+\\]: process has finished cleanly( |$)" \
        "${sanitized_log}" || true
    )"
    if (( process_count != required_count )); then
      echo "Missing clean-exit confirmation for ${process_name}: " \
        "expected ${required_count}, got ${process_count}; launch_status=${launch_status}" >&2
      tail -200 "${sanitized_log}" >&2
      return 1
    fi
  done
  echo "PASS OpenArm fake SIGINT shutdown: all four launched Python processes exited cleanly (launch_status=${launch_status})."
}

# Non-interactive Bash starts asynchronous children with SIGINT ignored. Reset
# both shutdown signals before exec so this smoke test exercises the same ROS
# signal path as a foreground launch stopped from the user's terminal.
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -c '
import os
import signal
import sys

signal.signal(signal.SIGINT, signal.SIG_DFL)
signal.signal(signal.SIGTERM, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])
' ros2 launch oculus_reader \
  teleop_double_openarm_v1.launch.py \
  start_quest_publisher:=false >"${smoke_tmp}/launch.log" 2>&1 &
launch_pid=$!

PYTHONDONTWRITEBYTECODE=1 python \
  "${QUESTARMTELEOP_DIR}/src/oculus_reader/test/openarm_fake_input_smoke.py"

# Keep the proof concise while retaining the complete launch log until cleanup.
sed -r 's/\x1B\[[0-9;]*[mK]//g' "${smoke_tmp}/launch.log" \
  | grep -E \
      'Configured and activated.*forward_position_controller|Official OpenArm bimanual IK ready|command guard re-armed|startup/rearm gripper opening complete|guard HOLD -> TELEOP' \
  | tail -20

stop_launch_and_verify_clean_exit
echo "OpenArm v1 fake-hardware ROS smoke test passed."
