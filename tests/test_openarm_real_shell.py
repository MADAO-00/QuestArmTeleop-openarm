from __future__ import annotations

import hashlib
import errno
import math
import os
import pty
import re
import signal
import shutil
import subprocess
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "run_openarm_v1.sh"
CONFIGURATOR = ROOT / "scripts" / "configure_openarm_v1_canfd.sh"
DOCKERFILE = ROOT / "Dockerfile.openarm-v1"
RUNTIME_BOOTSTRAP_PATCH = ROOT / "patches" / "openarm_ros2-runtime-bootstrap.patch"
VELOCITY_LIMITS_PATCH = ROOT / "patches" / "openarm_ros2-v1-velocity-limits.patch"
TORQUE_DEBOUNCE_PATCH = (
    ROOT / "patches" / "openarm_ros2-runtime-torque-debounce.patch"
)
GRIPPER_DIRECTIONAL_VELOCITY_PATCH = (
    ROOT / "patches" / "openarm_ros2-gripper-directional-velocity.patch"
)
RUNTIME_GRIPPER_VELOCITY_PATCH = (
    ROOT / "patches" / "openarm_ros2-runtime-gripper-velocity-policy.patch"
)
CONSTRAINED_IK_PATCH = ROOT / "patches" / "openarm_control-constrained-ik.patch"
CONSTRAINED_IK_VERIFIER = (
    ROOT / "patches" / "verify_openarm_control_constrained_ik.py"
)
SAFETY_VERIFIER = ROOT / "patches" / "verify_openarm_v1_safety_patches.sh"
QUEST_TELEOP_APK = (
    ROOT / "src" / "oculus_reader" / "APK" / "teleop-debug.apk"
)
QUEST_TELEOP_APK_SHA256 = (
    "6ddd90d8bced3a9533ae36099c950238fdb3ffd5feb5a6daa0afd63ac484cdb0"
)


def test_runner_pins_the_quest3_openxr_apk() -> None:
    runner = RUNNER.read_text(encoding="utf-8")

    assert hashlib.sha256(QUEST_TELEOP_APK.read_bytes()).hexdigest() == (
        QUEST_TELEOP_APK_SHA256
    )
    assert (
        f"REAL_QUEST_TELEOP_APK_SHA256={QUEST_TELEOP_APK_SHA256}" in runner
    )


def test_host_adb_path_closes_the_inherited_whole_robot_lock_fd() -> None:
    runner = RUNNER.read_text(encoding="utf-8")
    function_start = runner.index("require_host_quest() (")
    function_end = runner.index("\n)\n\nconfirm_real_safety_ack()", function_start)
    function_body = runner[function_start:function_end]
    outside_function = runner[:function_start] + runner[function_end:]

    close_fd = function_body.index("exec 9>&-")
    first_adb = function_body.index('adb_output="$(adb devices)"')
    assert close_fd < first_adb
    assert len(re.findall(r"(?m)^\s*adb -s ", function_body)) == 2
    assert 'adb_output="$(adb devices)"' not in outside_function
    assert not re.search(r"(?m)^\s*adb -s ", outside_function)


def test_runtime_bootstrap_patch_and_marker_are_attested() -> None:
    dockerfile = DOCKERFILE.read_text(encoding="utf-8")
    runner = RUNNER.read_text(encoding="utf-8")
    patch_sha256 = hashlib.sha256(RUNTIME_BOOTSTRAP_PATCH.read_bytes()).hexdigest()
    velocity_patch_sha256 = hashlib.sha256(VELOCITY_LIMITS_PATCH.read_bytes()).hexdigest()
    torque_debounce_patch_sha256 = hashlib.sha256(
        TORQUE_DEBOUNCE_PATCH.read_bytes()
    ).hexdigest()
    gripper_directional_patch_sha256 = hashlib.sha256(
        GRIPPER_DIRECTIONAL_VELOCITY_PATCH.read_bytes()
    ).hexdigest()
    runtime_gripper_velocity_patch_sha256 = hashlib.sha256(
        RUNTIME_GRIPPER_VELOCITY_PATCH.read_bytes()
    ).hexdigest()
    constrained_ik_patch_sha256 = hashlib.sha256(
        CONSTRAINED_IK_PATCH.read_bytes()
    ).hexdigest()
    constrained_ik_verifier_sha256 = hashlib.sha256(
        CONSTRAINED_IK_VERIFIER.read_bytes()
    ).hexdigest()
    verifier_sha256 = hashlib.sha256(SAFETY_VERIFIER.read_bytes()).hexdigest()

    assert "COPY patches/openarm_ros2-runtime-bootstrap.patch" in dockerfile
    assert dockerfile.count(
        "/tmp/openarm-v1-build/patches/openarm_ros2-runtime-bootstrap.patch"
    ) >= 4
    assert patch_sha256 in dockerfile
    assert "COPY patches/openarm_ros2-v1-velocity-limits.patch" in dockerfile
    assert velocity_patch_sha256 in dockerfile
    assert "openarm_ros2-v1-velocity-limits.patch" in runner
    assert "COPY patches/openarm_ros2-runtime-torque-debounce.patch" in dockerfile
    assert torque_debounce_patch_sha256 in dockerfile
    assert "openarm_ros2-runtime-torque-debounce.patch" in runner
    assert (
        "COPY patches/openarm_ros2-gripper-directional-velocity.patch" in dockerfile
    )
    assert gripper_directional_patch_sha256 in dockerfile
    assert "openarm_ros2-gripper-directional-velocity.patch" in runner
    assert (
        "COPY patches/openarm_ros2-runtime-gripper-velocity-policy.patch"
        in dockerfile
    )
    assert runtime_gripper_velocity_patch_sha256 in dockerfile
    assert "openarm_ros2-runtime-gripper-velocity-policy.patch" in runner
    assert "COPY patches/openarm_control-constrained-ik.patch" in dockerfile
    assert constrained_ik_patch_sha256 in dockerfile
    assert f"REAL_IK_SAFETY_PATCH_SHA256={constrained_ik_patch_sha256}" in runner
    assert "COPY patches/verify_openarm_control_constrained_ik.py" in dockerfile
    assert constrained_ik_verifier_sha256 in dockerfile
    assert (
        f"REAL_IK_SAFETY_VERIFIER_SHA256={constrained_ik_verifier_sha256}"
        in runner
    )
    assert "850a6d70231d74ecf01a3d8466b37a500dc7a596b4715f8187d245b94e9d17bd" in dockerfile
    assert "aa0ca15e2b45486c5e62093acee3b26985743834cf57b10bf6aeae6547a77d1d" in dockerfile
    assert "REAL_PATCHED_KINEMATICS_SHA256=" in runner
    assert "! grep -F \"limits=[]\"" in runner
    assert verifier_sha256 in dockerfile
    assert "openarm_ros2-runtime-bootstrap.patch" in runner
    assert f"REAL_SAFETY_VERIFIER_SHA256={verifier_sha256}" in runner

    rows = dockerfile.splitlines()
    marker_start = next(i for i, row in enumerate(rows) if row.startswith("RUN printf "))
    marker_values: list[str] = []
    for row in rows[marker_start + 1 :]:
        if row.lstrip().startswith("> "):
            break
        if '"base_image_id=${OPENARM_BASE_IMAGE_ID}"' in row:
            marker_values.append("base_image_id=BUILD_BASE_IMAGE_ID")
            continue
        match = re.match(r"\s*'([^']*)'\s*\\$", row)
        if match:
            marker_values.append(match.group(1))
    marker_payload = ("\n".join(marker_values) + "\n").encode("utf-8")
    marker_sha256 = hashlib.sha256(marker_payload).hexdigest()
    assert f"REAL_SAFETY_MARKER_SHA256={marker_sha256}" in runner
    assert "IMAGE_NAME=\"${IMAGE_NAME:-questarm-openarm-v1:humble-standalone-1.0}\"" in runner
    assert "REAL_SAFETY_SCHEMA=questarm-openarm-v1-real-safety-v7" in runner
    assert ".questarm-real-safety-v7" in dockerfile
    assert ".questarm-real-safety-v7" in runner


def test_whole_robot_feedback_gross_overspeed_and_command_velocity_contracts_are_distinct() -> None:
    velocity_patch = VELOCITY_LIMITS_PATCH.read_text(encoding="utf-8")
    runtime_gripper_patch = RUNTIME_GRIPPER_VELOCITY_PATCH.read_text(
        encoding="utf-8"
    )
    velocity_postimage = "\n".join(
        line[1:]
        for line in velocity_patch.splitlines()
        if line.startswith((" ", "+")) and not line.startswith("+++")
    )
    runtime_gripper_postimage = "\n".join(
        line[1:]
        for line in runtime_gripper_patch.splitlines()
        if line.startswith((" ", "+")) and not line.startswith("+++")
    )

    def added_array(symbol: str) -> tuple[str, ...]:
        match = re.search(
            rf"constexpr std::array<double, 7> {symbol} = \{{([^}}]+)\}};",
            velocity_postimage,
            re.DOTALL,
        )
        assert match is not None, f"missing patched array {symbol}"
        return tuple(value.strip() for value in match.group(1).split(","))

    def patched_scalar(source: str, symbol: str) -> str:
        match = re.search(
            rf"constexpr double {symbol} = ([0-9]+(?:\.[0-9]+)?);",
            source,
        )
        assert match is not None, f"missing patched scalar {symbol}"
        return match.group(1)

    feedback_limits = added_array("MAX_ABS_FEEDBACK_VELOCITY")
    command_limits = added_array("MAX_COMMAND_VELOCITY")
    gripper_feedback_limit = patched_scalar(
        velocity_postimage, "MAX_ABS_GRIPPER_FEEDBACK_VELOCITY"
    )
    runtime_gripper_feedback_limit = patched_scalar(
        runtime_gripper_postimage, "MAX_ABS_GRIPPER_FEEDBACK_VELOCITY"
    )
    runtime_gripper_hard_limit = patched_scalar(
        runtime_gripper_postimage, "RUNTIME_GRIPPER_VELOCITY_HARD_LIMIT"
    )

    assert feedback_limits == (
        "15.079644737",
        "15.079644737",
        "4.900884539",
        "4.900884539",
        "18.849555921",
        "18.849555921",
        "18.849555921",
    )
    assert gripper_feedback_limit == "18.849555921"
    assert runtime_gripper_feedback_limit == gripper_feedback_limit
    assert runtime_gripper_hard_limit == gripper_feedback_limit
    assert (
        "constexpr double MAX_ABS_GRIPPER_FEEDBACK_VELOCITY = 1.05;"
        not in velocity_postimage + runtime_gripper_postimage
    )
    assert (
        "constexpr double RUNTIME_GRIPPER_VELOCITY_HARD_LIMIT = 1.20;"
        not in runtime_gripper_postimage
    )
    assert (
        "1.57, 1.57, 3.14, 3.14, 12.6, 12.6, 12.6"
        not in velocity_patch
    )
    assert command_limits == (
        "1.413",
        "1.413",
        "2.826",
        "2.826",
        "11.34",
        "11.34",
        "11.34",
    )

    # The feedback gate is a project-selected gross-overspeed ceiling derived
    # from each exact OpenArm v1 BOM motor's published 24 V no-load speed.  The
    # decimal literals are truncated downward to nine places so they cannot
    # accidentally exceed 90% through ordinary rounding.
    manufacturer_no_load_rpm = (160, 160, 52, 52, 200, 200, 200, 200)
    exact_limits = tuple(
        0.90 * rpm * 2.0 * math.pi / 60.0
        for rpm in manufacturer_no_load_rpm
    )
    truncated_limits = tuple(
        math.floor(value * 1_000_000_000) / 1_000_000_000
        for value in exact_limits
    )
    actual_limits = tuple(float(value) for value in feedback_limits) + (
        float(gripper_feedback_limit),
    )

    assert actual_limits == truncated_limits
    assert all(
        0.0 <= exact - actual < 1e-9
        for actual, exact in zip(actual_limits, exact_limits, strict=True)
    )


def write_executable(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")
    path.chmod(0o755)


def link_or_copy(source: str, destination: str) -> str:
    """Avoid copying mesh bytes when tests share a filesystem with the source."""
    try:
        os.link(source, destination)
    except OSError as error:
        if error.errno not in (errno.EXDEV, errno.EPERM, errno.EROFS):
            raise
        shutil.copy2(source, destination)
    return destination


def test_canfd_configurator_uses_only_fixed_can0_can1_with_1m_5m(tmp_path: Path) -> None:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    log = tmp_path / "tools.log"
    write_executable(
        fake_bin / "python3",
        "#!/usr/bin/env bash\nprintf 'python:%s\\n' \"$*\" >>\"$MOCK_LOG\"\n",
    )
    write_executable(fake_bin / "ethtool", "#!/usr/bin/env bash\nexit 0\n")
    write_executable(
        fake_bin / "ip",
        "#!/usr/bin/env bash\nprintf 'ip:%s\\n' \"$*\" >>\"$MOCK_LOG\"\n",
    )
    write_executable(
        fake_bin / "sudo",
        """#!/usr/bin/env bash
set -e
[[ ${1:-} == -n ]] && shift
if [[ ${1:-} == true ]]; then
  exit 0
fi
exec "$@"
""",
    )
    env = os.environ.copy()
    env.update({"PATH": f"{fake_bin}:{env['PATH']}", "MOCK_LOG": str(log)})

    result = subprocess.run(
        ["bash", str(CONFIGURATOR)],
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    events = log.read_text(encoding="utf-8").splitlines()
    ip_events = [event for event in events if event.startswith("ip:")]
    assert ip_events == [
        "ip:link set dev can0 down",
        "ip:link set dev can0 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100",
        "ip:link set dev can0 up",
        "ip:link set dev can1 down",
        "ip:link set dev can1 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100",
        "ip:link set dev can1 up",
    ]
    assert events[0].endswith("--identity-only")
    assert events[-1].startswith("python:") and not events[-1].endswith("--identity-only")


def isolated_runner(tmp_path: Path) -> Path:
    project = tmp_path / "project"
    scripts = project / "scripts"
    scripts.mkdir(parents=True)
    patches = project / "patches"
    patches.mkdir()
    package_parent = project / "src"
    package_parent.mkdir(parents=True)
    shutil.copytree(
        ROOT / "src" / "oculus_reader",
        package_parent / "oculus_reader",
        ignore=shutil.ignore_patterns("APK", "__pycache__", "*.pyc"),
    )
    model_source = (
        ROOT
        / "assets/openarm_mujoco"
    )
    model_destination = (
        project
        / "assets/openarm_mujoco"
    )
    model_destination.parent.mkdir(parents=True)
    shutil.copytree(model_source, model_destination, copy_function=link_or_copy)
    shutil.copy2(RUNNER, project / RUNNER.name)
    for name in (
        "preflight_openarm_v1_ros_real.py",
        "configure_openarm_v1_canfd.sh",
        "openarm_can_no_motion_preflight.py",
        "openarm_v1_overlay_fingerprint.sh",
        "rebuild_openarm_v1_overlay_in_container.sh",
        "source_openarm_v1_overlay.sh",
    ):
        shutil.copy2(ROOT / "scripts" / name, scripts / name)
    shutil.copy2(
        ROOT / "patches" / "verify_openarm_v1_safety_patches.sh",
        patches / "verify_openarm_v1_safety_patches.sh",
    )
    shutil.copy2(
        CONSTRAINED_IK_PATCH,
        patches / CONSTRAINED_IK_PATCH.name,
    )
    shutil.copy2(
        CONSTRAINED_IK_VERIFIER,
        patches / CONSTRAINED_IK_VERIFIER.name,
    )

    overlay = project / ".openarm_v1_real_overlay_v4" / "install"
    package_source = package_parent / "oculus_reader"
    (overlay / "oculus_reader/share/oculus_reader/launch").mkdir(parents=True)
    (overlay / "oculus_reader/share/oculus_reader/config").mkdir(parents=True)
    (overlay / "oculus_reader/lib/oculus_reader").mkdir(parents=True)
    installed_models = overlay / "oculus_reader/share/oculus_reader/assets/openarm_mujoco"
    installed_models.parent.mkdir(parents=True)
    shutil.copytree(model_source, installed_models, copy_function=link_or_copy)
    (overlay / "setup.bash").write_text("# test overlay\n", encoding="utf-8")
    (overlay / "setup.sh").write_text("# test overlay\n", encoding="utf-8")
    (overlay / "local_setup.bash").write_text("# test overlay\n", encoding="utf-8")
    for relative in (
        "launch/teleop_double_openarm_v1.launch.py",
        "config/openarm_v1_bimanual_controllers.yaml",
        "config/openarm_v1_teleop.yaml",
    ):
        shutil.copy2(
            package_source / relative,
            overlay / "oculus_reader/share/oculus_reader" / relative,
        )
    for name in (
        "pub_pose_openarm_v1.py",
        "pub_delta_pose_openarm_v1.py",
        "openarm_bimanual_ik_node.py",
        "openarm_command_guard_node.py",
        "openarm_teleop_core.py",
        "oculus_reader.py",
        "transformations.py",
        "buttons_parser.py",
        "FPS_counter.py",
    ):
        shutil.copy2(
            package_source / "scripts" / name,
            overlay / "oculus_reader/lib/oculus_reader" / name,
        )
    shutil.copy2(
        package_source / "package.xml",
        overlay / "oculus_reader/share/oculus_reader/package.xml",
    )
    fingerprint = subprocess.run(
        ["bash", str(scripts / "openarm_v1_overlay_fingerprint.sh")],
        env={**os.environ, "WORKSPACE_ROOT": str(project)},
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    (project / ".openarm_v1_real_overlay_v4/source.sha256").write_text(
        fingerprint + "\n",
        encoding="ascii",
    )
    return project / RUNNER.name


def fake_runner_tools(tmp_path: Path) -> tuple[Path, Path]:
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    log = tmp_path / "runner.log"
    write_executable(
        fake_bin / "docker",
        r'''#!/usr/bin/env bash
set -euo pipefail
if [[ ${1:-} == info ]]; then
  printf 'docker:info\n' >>"$MOCK_LOG"
  exit 0
fi
if [[ ${1:-} == image && ${2:-} == inspect ]]; then
  printf 'docker:image-inspect\n' >>"$MOCK_LOG"
  if [[ $* == *RootFS.Layers* ]]; then
    printf 'sha256:base-layer-one\nsha256:base-layer-two\n'
  elif [[ $* == *'{{.Id}}'* ]]; then
    printf 'sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n'
  elif [[ $* == *org.questarm.openarm-base.schema* ]]; then
    printf '%s\n' "${MOCK_BASE_SCHEMA:-questarm-openarm-base-humble-v1}"
  elif [[ $* == *org.questarm.openarm-v1.real-safety.schema* ]]; then
    printf 'questarm-openarm-v1-real-safety-v7\n'
  elif [[ $* == *org.questarm.openarm-v1.base-image-id* ]]; then
    printf 'sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n'
  elif [[ $* == *Config.Entrypoint* ]]; then
    printf 'null\n'
  fi
  exit 0
fi
if [[ ${1:-} != run ]]; then
  exit 90
fi
kind=control
mode=
previous=
for argument in "$@"; do
  case "$argument" in
    *verify_openarm_v1_safety_patches.sh*) kind=verify ;;
    */configure_openarm_v1_canfd.sh) kind=config ;;
    */preflight_openarm_v1_ros_real.py) kind=readonly ;;
    */openarm_can_no_motion_preflight.py) kind=motor ;;
  esac
  if [[ "$previous" == --mode ]]; then
    mode=$argument
  fi
  previous=$argument
done
if [[ $* == *OPENARM_HARDWARE_MODE=real* ]]; then
  kind=control
fi
printf 'docker-run:%s:%s ' "$kind" "$mode" >>"$MOCK_LOG"
printf '<%q>' "$@" >>"$MOCK_LOG"
printf '\n' >>"$MOCK_LOG"
if [[ "$kind" == motor && "$mode" == preflight && ${MOCK_FAIL_MOTOR_PREFLIGHT:-0} == 1 ]]; then
  exit 42
fi
exit 0
''',
    )
    write_executable(
        fake_bin / "python3",
        "#!/usr/bin/env bash\nprintf 'python:%s\\n' \"$*\" >>\"$MOCK_LOG\"\n",
    )
    write_executable(
        fake_bin / "sudo",
        "#!/usr/bin/env bash\n[[ ${1:-} == -n && ${2:-} == true ]] && exit 1\nexit 91\n",
    )
    write_executable(
        fake_bin / "flock",
        "#!/usr/bin/env bash\nprintf 'flock:%s\\n' \"$*\" >>\"$MOCK_LOG\"\nexit \"${MOCK_FLOCK_STATUS:-0}\"\n",
    )
    write_executable(
        fake_bin / "adb",
        """#!/usr/bin/env bash
if [[ ${1:-} == devices ]]; then
	case ${MOCK_ADB_STATE:-device} in
	  device) printf 'List of devices attached\nquest-test\tdevice\n' ;;
	  multiple) printf 'List of devices attached\nquest-one\tdevice\nquest-two\tdevice\n' ;;
	  mixed) printf 'List of devices attached\nquest-authorized\tdevice\nquest-unauthorized\tunauthorized\n' ;;
	  *) printf 'List of devices attached\n\n' ;;
  esac
  exit 0
fi
if [[ ${1:-} == -s && ${3:-} == shell && ${4:-} == pm && ${5:-} == path ]]; then
  if [[ ${MOCK_APK_STATE:-installed} == installed ]]; then
    printf 'package:/data/app/com.rail.oculus.teleop/base.apk\n'
  fi
  exit 0
fi
if [[ ${1:-} == -s && ${3:-} == shell && ${4:-} == sha256sum ]]; then
  if [[ ${MOCK_APK_SHA_STATE:-current} == current ]]; then
    printf '6ddd90d8bced3a9533ae36099c950238fdb3ffd5feb5a6daa0afd63ac484cdb0  %s\n' "${5:-}"
  else
    printf '97b49f94682a732e14d131d50bcc7885e183d73b977e19a1752c28502519cd4b  %s\n' "${5:-}"
  fi
  exit 0
fi
exit 92
""",
    )
    return fake_bin, log


def runner_environment(tmp_path: Path, fake_bin: Path, log: Path) -> dict[str, str]:
    runtime = tmp_path / "runtime"
    runtime.mkdir()
    env = os.environ.copy()
    env.update(
        {
            "PATH": f"{fake_bin}:{env['PATH']}",
            "MOCK_LOG": str(log),
            "XDG_RUNTIME_DIR": str(runtime),
            "OPENARM_MODE": "real",
            "OPENARM_NO_TTY": "1",
            "OPENARM_REAL_SAFETY_ACK": (
                "I_CONFIRM_OPENARM_ARMS_SUPPORTED_AREA_CLEAR_"
                "ESTOP_READY_GRIPS_RELEASED"
            ),
        }
    )
    for name in (
        "DISPLAY",
        "XAUTHORITY",
        "OPENARM_REAL_PREFLIGHT_PASSED",
        "OPENARM_REAL_PREFLIGHT_GATE",
        "OPENARM_REAL_PREFLIGHT_TOKEN",
        "ROS_LOCALHOST_ONLY",
    ):
        env.pop(name, None)
    return env


def test_detached_adb_server_cannot_retain_the_whole_robot_lock(
    tmp_path: Path,
) -> None:
    """Exercise the real runner with fake ADB and a real host flock.

    The fake ADB client leaves a live background process behind, just like an
    on-demand ADB server. It must stay alive without inheriting descriptor 9,
    and the lock must be immediately acquirable after the runner exits.
    """

    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    real_flock = shutil.which("flock")
    assert real_flock is not None
    write_executable(
        fake_bin / "flock",
        "#!/usr/bin/env bash\n"
        "printf 'flock:%s\\n' \"$*\" >>\"$MOCK_LOG\"\n"
        f'exec "{real_flock}" "$@"\n',
    )

    daemon_pid_file = tmp_path / "fake-adb-server.pid"
    daemon_fd_state = tmp_path / "fake-adb-server-fd9.txt"
    write_executable(
        fake_bin / "adb",
        r'''#!/usr/bin/env bash
set -euo pipefail
if [[ ! -e "$MOCK_ADB_DAEMON_PID" ]]; then
  (
    if [[ -e /proc/self/fd/9 ]]; then
      printf 'present\n'
    else
      printf 'absent\n'
    fi >"$MOCK_ADB_FD_STATE"
    trap '' HUP
    exec sleep 30
  ) </dev/null >/dev/null 2>&1 &
  printf '%s\n' "$!" >"$MOCK_ADB_DAEMON_PID"
fi
if [[ ${1:-} == devices ]]; then
  printf 'List of devices attached\nquest-test\tdevice\n'
  exit 0
fi
if [[ ${1:-} == -s && ${3:-} == shell && ${4:-} == pm && ${5:-} == path ]]; then
  printf 'package:/data/app/com.rail.oculus.teleop/base.apk\n'
  exit 0
fi
if [[ ${1:-} == -s && ${3:-} == shell && ${4:-} == sha256sum ]]; then
  printf '6ddd90d8bced3a9533ae36099c950238fdb3ffd5feb5a6daa0afd63ac484cdb0  %s\n' "${5:-}"
  exit 0
fi
exit 92
''',
    )
    env = runner_environment(tmp_path, fake_bin, log)
    env.update(
        {
            "MOCK_ADB_DAEMON_PID": str(daemon_pid_file),
            "MOCK_ADB_FD_STATE": str(daemon_fd_state),
        }
    )
    daemon_pid: int | None = None
    try:
        result = subprocess.run(
            ["bash", str(runner)],
            env=env,
            cwd=runner.parent,
            check=False,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stderr

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not daemon_fd_state.exists():
            time.sleep(0.01)
        assert daemon_pid_file.is_file()
        assert daemon_fd_state.read_text(encoding="ascii").strip() == "absent"
        daemon_pid = int(daemon_pid_file.read_text(encoding="ascii").strip())
        os.kill(daemon_pid, 0)

        lock_file = (
            runner.parent
            / ".cache/openarm-vr/real-can0-can1.lock"
        )
        competitor = subprocess.run(
            [
                real_flock,
                "--exclusive",
                "--nonblock",
                "--conflict-exit-code",
                "75",
                str(lock_file),
                "true",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        assert competitor.returncode == 0, competitor.stderr
    finally:
        if daemon_pid is None and daemon_pid_file.is_file():
            daemon_pid = int(daemon_pid_file.read_text(encoding="ascii").strip())
        if daemon_pid is not None:
            try:
                os.kill(daemon_pid, signal.SIGTERM)
            except ProcessLookupError:
                pass


def test_real_runner_capabilities_gate_default_launch_and_cleanup(tmp_path: Path) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "arm motors ID1-7 plus gripper motor ID8 are enabled" in result.stdout
    assert "each Index Trigger independently closes its gripper" in result.stdout
    assert "released Triggers command guarded opening" in result.stdout
    events = log.read_text(encoding="utf-8").splitlines()
    run_events = [event for event in events if event.startswith("docker-run:")]
    assert [event.split(":", 2)[1] for event in run_events] == [
        "verify",
        "readonly",
        "config",
        "readonly",
        "motor",
        "readonly",
        "control",
        "motor",
    ]
    assert run_events[4].startswith("docker-run:motor:preflight")
    assert run_events[-1].startswith("docker-run:motor:disable-only")

    verifier = run_events[0]
    assert "<--network><none>" in verifier
    assert "NET_RAW" not in verifier and "NET_ADMIN" not in verifier
    assert "OPENARM_EXPECTED_SAFETY_MARKER_SHA256=" in verifier
    assert ".questarm-real-safety-v7" in verifier
    assert "<--entrypoint></bin/bash>" in verifier

    config = run_events[2]
    assert "<--cap-add><NET_ADMIN>" in config
    assert "<--cap-add><NET_RAW>" in config

    control = run_events[6]
    assert "<--network><host>" in control
    assert "<--cap-drop><ALL>" in control
    assert "<--cap-add><NET_RAW>" in control
    assert "NET_ADMIN" not in control
    assert "--device" not in control and "/dev/bus/usb" not in control
    assert "OPENARM_HARDWARE_MODE=real" in control
    assert "ROS_DOMAIN_ID=91" in control
    assert "ROS_LOCALHOST_ONLY=1" in control
    assert "OPENARM_V1_SKIP_LEGACY_OVERLAY=1" in control
    assert f"<{runner.parent}:/workspace:ro>" in control
    assert "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" in control
    runner_source = runner.read_text(encoding="utf-8")
    assert "questarm-openarm-v1:humble-standalone-1.0" in runner_source
    assert ".openarm_v1_real_overlay_v4" in runner_source
    assert "config/openarm_v1_bimanual_controllers.yaml" in runner_source
    assert "openarm_v1_bimanual_arm_only_controllers.yaml" not in runner_source
    assert "whole-robot ID1-8" in runner_source
    assert "ID8 is the gripper motor" in runner_source
    assert "ID8 remains untouched" not in runner_source
    assert "before arm-only ID1-7" not in runner_source
    assert "Quest Grip/deadman and Index Trigger/gripper controls are released" in runner_source
    assert "OPENARM_REAL_PREFLIGHT_PASSED=1" in control
    assert "OPENARM_REAL_PREFLIGHT_GATE=/run/openarm-real/preflight.gate" in control
    assert 'chmod 0444 "${REAL_GATE_FILE}"' in runner.read_text(encoding="utf-8")
    assert 'stat -c %a "${OPENARM_REAL_PREFLIGHT_GATE}")" == 444' in runner.read_text(
        encoding="utf-8"
    )
    assert "teleop_double_openarm_v1.launch.py" in control
    assert "use_fake_hardware:=false" in control
    assert "left_can_interface:=can0" in control
    assert "right_can_interface:=can1" in control
    assert "enable_home:=false" in control
    assert "socket-smoke" in control
    assert "<--entrypoint></bin/bash>" in control
    assert "/opt/questarm/openarm-v1-safety/preflight_openarm_v1_ros_real.py" in control

    assert (runner.parent / ".cache/openarm-vr/real-can0-can1.lock").is_file()
    assert any(event.startswith("flock:--exclusive --nonblock") for event in events)
    assert not list((tmp_path / "runtime").iterdir())


def test_real_preflight_only_never_creates_control_container(tmp_path: Path) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env["OPENARM_REAL_PREFLIGHT_ONLY"] = "1"

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    events = log.read_text(encoding="utf-8").splitlines()
    run_events = [event for event in events if event.startswith("docker-run:")]
    assert all("docker-run:control:" not in event for event in run_events)
    assert run_events[-1].startswith("docker-run:motor:disable-only")
    assert "no gate was created" in result.stdout
    assert not list((tmp_path / "runtime").iterdir())


def test_full_real_cannot_disable_quest_gate(tmp_path: Path) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env["OPENARM_REQUIRE_QUEST"] = "0"

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "allowed only with OPENARM_REAL_PREFLIGHT_ONLY=1" in result.stderr
    assert not any(
        event.startswith("docker-run:")
        for event in log.read_text(encoding="utf-8").splitlines()
    )


def test_real_lock_conflict_preserves_exit_75_and_runs_no_helper(tmp_path: Path) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env["MOCK_FLOCK_STATUS"] = "75"

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 75
    assert "another OpenArm real-hardware process" in result.stderr
    assert not any(
        event.startswith("docker-run:")
        for event in log.read_text(encoding="utf-8").splitlines()
    )


def test_partial_motor_preflight_failure_still_runs_exit_disable(tmp_path: Path) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env["MOCK_FAIL_MOTOR_PREFLIGHT"] = "1"

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 42
    run_events = [
        event
        for event in log.read_text(encoding="utf-8").splitlines()
        if event.startswith("docker-run:")
    ]
    assert run_events[-2].startswith("docker-run:motor:preflight")
    assert run_events[-1].startswith("docker-run:motor:disable-only")


def test_real_runner_rejects_external_ros_localhost_override(tmp_path: Path) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env["ROS_LOCALHOST_ONLY"] = "0"

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "ROS_LOCALHOST_ONLY is reserved" in result.stderr
    assert not any(
        event.startswith("docker-run:")
        for event in log.read_text(encoding="utf-8").splitlines()
    )


def test_real_runner_rejects_external_ros_domain_override(tmp_path: Path) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env["ROS_DOMAIN_ID"] = "0"

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "ROS_DOMAIN_ID is reserved" in result.stderr
    assert not any(
        event.startswith("docker-run:")
        for event in log.read_text(encoding="utf-8").splitlines()
    )


def test_tampered_can_helper_is_rejected_before_any_capable_container(
    tmp_path: Path,
) -> None:
    runner = isolated_runner(tmp_path)
    configurator = runner.parent / "scripts/configure_openarm_v1_canfd.sh"
    configurator.write_text(
        configurator.read_text(encoding="utf-8") + "\n# tampered\n",
        encoding="utf-8",
    )
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "real safety source checksum mismatch" in result.stderr
    assert not any(
        event.startswith("docker-run:")
        for event in log.read_text(encoding="utf-8").splitlines()
    )


def test_tampered_official_ik_model_is_rejected_before_any_container(
    tmp_path: Path,
) -> None:
    runner = isolated_runner(tmp_path)
    scene = (
        runner.parent
        / "assets/openarm_mujoco/v1/scene.xml"
    )
    scene_contents = scene.read_text(encoding="utf-8")
    # The isolated model tree uses hard links to avoid copying 44 MB per test;
    # break this test's link before intentionally changing its fixture.
    scene.unlink()
    scene.write_text(scene_contents + "\n<!-- tampered -->\n", encoding="utf-8")
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "overlay source does not match" in result.stderr
    assert not any(
        event.startswith("docker-run:")
        for event in log.read_text(encoding="utf-8").splitlines()
    )


def test_tampered_installed_ik_model_is_rejected_before_can(
    tmp_path: Path,
) -> None:
    runner = isolated_runner(tmp_path)
    scene = (
        runner.parent / ".openarm_v1_real_overlay_v4/install/oculus_reader"
        / "share/oculus_reader/assets/openarm_mujoco/v1/scene.xml"
    )
    original_scene = scene.read_text(encoding="utf-8")
    # Both fixture trees may be hard-linked to the repository. Break only the
    # installed fixture's link so this tests installed-only tampering.
    scene.unlink()
    scene.write_text(original_scene + "\n<!-- changed installed model -->\n", encoding="utf-8")
    assert (runner.parent / "assets/openarm_mujoco/v1/scene.xml").read_text(
        encoding="utf-8"
    ) == original_scene
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)

    result = subprocess.run(
        ["bash", str(runner)], env=env, cwd=runner.parent,
        check=False, capture_output=True, text=True,
    )

    assert result.returncode != 0
    assert "not the reviewed non-symlink release" in result.stderr
    run_events = [
        event for event in log.read_text(encoding="utf-8").splitlines()
        if event.startswith("docker-run:")
    ]
    assert [event.split(":", 2)[1] for event in run_events] == ["verify"]
    assert "<--network><none>" in run_events[0]
    assert "NET_RAW" not in run_events[0] and "NET_ADMIN" not in run_events[0]


def test_wrong_standalone_base_schema_is_rejected_before_any_container(
    tmp_path: Path,
) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env["MOCK_BASE_SCHEMA"] = "unreviewed-base"

    result = subprocess.run(
        ["bash", str(runner)], env=env, cwd=runner.parent,
        check=False, capture_output=True, text=True,
    )

    assert result.returncode != 0
    assert "Standalone OpenArm base schema does not match" in result.stderr
    assert not any(
        event.startswith("docker-run:")
        for event in log.read_text(encoding="utf-8").splitlines()
    )


def test_noninteractive_real_requires_physical_safety_ack_before_can(
    tmp_path: Path,
) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env.pop("OPENARM_REAL_SAFETY_ACK")

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "non-interactive real mode requires OPENARM_REAL_SAFETY_ACK" in result.stderr
    run_events = [
        event
        for event in log.read_text(encoding="utf-8").splitlines()
        if event.startswith("docker-run:")
    ]
    assert [event.split(":", 2)[1] for event in run_events] == ["verify"]


def _run_interactive_runner(
    runner: Path, env: dict[str, str], entered_text: bytes
) -> tuple[int, str]:
    master_fd, slave_fd = pty.openpty()
    try:
        process = subprocess.Popen(
            ["bash", str(runner)],
            env=env,
            cwd=runner.parent,
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=subprocess.PIPE,
        )
        os.close(slave_fd)
        slave_fd = -1
        os.write(master_fd, entered_text)
        _, stderr = process.communicate(timeout=30)
        return process.returncode, stderr.decode("utf-8", errors="replace")
    finally:
        if slave_fd >= 0:
            os.close(slave_fd)
        os.close(master_fd)


def test_interactive_real_accepts_empty_enter_before_can(tmp_path: Path) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env["OPENARM_NO_TTY"] = "0"
    env.pop("OPENARM_REAL_SAFETY_ACK")

    returncode, stderr = _run_interactive_runner(runner, env, b"\n")

    assert returncode == 0, stderr
    run_events = [
        event
        for event in log.read_text(encoding="utf-8").splitlines()
        if event.startswith("docker-run:")
    ]
    assert any(event.startswith("docker-run:config:") for event in run_events)
    assert any(event.startswith("docker-run:control:") for event in run_events)


def test_interactive_real_rejects_nonempty_input_before_can(tmp_path: Path) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env["OPENARM_NO_TTY"] = "0"
    env.pop("OPENARM_REAL_SAFETY_ACK")

    returncode, stderr = _run_interactive_runner(runner, env, b"cancel\n")

    assert returncode != 0
    assert "real safety confirmation cancelled" in stderr
    run_events = [
        event
        for event in log.read_text(encoding="utf-8").splitlines()
        if event.startswith("docker-run:")
    ]
    assert [event.split(":", 2)[1] for event in run_events] == ["verify"]


def test_real_runner_rejects_custom_command_before_docker_or_can(tmp_path: Path) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)

    result = subprocess.run(
        ["bash", str(runner), "bash"],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "accepts no custom command" in result.stderr
    assert not log.exists()


def test_missing_quest_blocks_before_any_can_helper(tmp_path: Path) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env["MOCK_ADB_STATE"] = "none"

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "exactly one USB-eligible Quest ADB entry across all states; found 0" in result.stderr
    run_events = [
        event
        for event in log.read_text(encoding="utf-8").splitlines()
        if event.startswith("docker-run:")
    ]
    assert [event.split(":", 2)[1] for event in run_events] == ["verify"]
    assert not list((tmp_path / "runtime").iterdir())


def test_authorized_plus_unauthorized_quest_blocks_before_any_can_helper(
    tmp_path: Path,
) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env["MOCK_ADB_STATE"] = "mixed"

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "across all states; found 2" in result.stderr
    run_events = [
        event
        for event in log.read_text(encoding="utf-8").splitlines()
        if event.startswith("docker-run:")
    ]
    assert [event.split(":", 2)[1] for event in run_events] == ["verify"]
    assert not list((tmp_path / "runtime").iterdir())


def test_missing_quest_apk_blocks_before_control_and_prints_install_command(
    tmp_path: Path,
) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env["MOCK_APK_STATE"] = "missing"

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "missing com.rail.oculus.teleop" in result.stderr
    assert "adb -s quest-test install -r" in result.stderr
    run_events = [
        event
        for event in log.read_text(encoding="utf-8").splitlines()
        if event.startswith("docker-run:")
    ]
    assert [event.split(":", 2)[1] for event in run_events] == ["verify"]


def test_incompatible_quest_apk_blocks_before_control_and_prints_migration_command(
    tmp_path: Path,
) -> None:
    runner = isolated_runner(tmp_path)
    fake_bin, log = fake_runner_tools(tmp_path)
    env = runner_environment(tmp_path, fake_bin, log)
    env["MOCK_APK_SHA_STATE"] = "old"

    result = subprocess.run(
        ["bash", str(runner)],
        env=env,
        cwd=runner.parent,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "incompatible teleop APK" in result.stderr
    assert "uninstall com.rail.oculus.teleop" in result.stderr
    assert "install " in result.stderr
    run_events = [
        event
        for event in log.read_text(encoding="utf-8").splitlines()
        if event.startswith("docker-run:")
    ]
    assert [event.split(":", 2)[1] for event in run_events] == ["verify"]
