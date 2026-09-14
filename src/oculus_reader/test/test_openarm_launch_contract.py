"""Static and launch-unit contracts for the gated OpenArm v1 bringup.

These tests never open a CAN socket and never start a ROS process.
"""

from __future__ import annotations

import ast
import importlib.util
from pathlib import Path
import re
import socket
import sys
import threading
import time
from types import ModuleType, SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from launch import LaunchContext
from launch.events import Shutdown
import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE / "launch" / "teleop_double_openarm_v1.launch.py"
CONTROLLERS_FILE = (
    PACKAGE / "config" / "openarm_v1_bimanual_controllers.yaml"
)
IK_NODE = PACKAGE / "scripts" / "openarm_bimanual_ik_node.py"
COMMAND_GUARD_NODE = PACKAGE / "scripts" / "openarm_command_guard_node.py"
TELEOP_CORE = PACKAGE / "scripts" / "openarm_teleop_core.py"
DELTA_NODE = PACKAGE / "scripts" / "pub_delta_pose_openarm_v1.py"
POSE_NODE = PACKAGE / "scripts" / "pub_pose_openarm_v1.py"
OPENARM_RUNTIME_PYTHON_NODES = (
    POSE_NODE,
    DELTA_NODE,
    IK_NODE,
    COMMAND_GUARD_NODE,
)
TELEOP_CONFIG = PACKAGE / "config" / "openarm_v1_teleop.yaml"
FAKE_INPUT_SMOKE = PACKAGE / "test" / "openarm_fake_input_smoke.py"
RUNTIME_GRIPPER_VELOCITY_PATCH = (
    PACKAGE.parents[1]
    / "patches"
    / "openarm_ros2-runtime-gripper-velocity-policy.patch"
)
OFFICIAL_BIMANUAL_MODEL = (
    PACKAGE.parents[1]
    / "assets"
    / "openarm_mujoco"
    / "v1"
    / "openarm_bimanual.xml"
)

scripts_path = str(PACKAGE / "scripts")
if scripts_path not in sys.path:
    sys.path.insert(0, scripts_path)

from openarm_teleop_core import (  # noqa: E402
    ATOMIC_IK_COMMAND_DOF,
    ATOMIC_TARGET_DOF,
    GUARD_ACCEPTANCE_DOF,
    GUARD_ANCHOR_DOF,
    GRIPPER_CLOSE_VELOCITY_M_S,
    GRIPPER_OPEN_POSITION_M,
    GRIPPER_OPEN_VELOCITY_M_S,
    LEFT_JOINT_NAMES,
    MAX_EXACT_FLOAT64_INT,
    OPENARM_LEFT_JOINT_HARD_LOWER_RAD,
    OPENARM_LEFT_JOINT_HARD_UPPER_RAD,
    OPENARM_LEFT_JOINT_SOFT_LOWER_RAD,
    OPENARM_LEFT_JOINT_SOFT_UPPER_RAD,
    OPENARM_RIGHT_JOINT_HARD_LOWER_RAD,
    OPENARM_RIGHT_JOINT_HARD_UPPER_RAD,
    OPENARM_RIGHT_JOINT_SOFT_LOWER_RAD,
    OPENARM_RIGHT_JOINT_SOFT_UPPER_RAD,
    OPENARM_TELEOP_SOFT_LIMIT_RATIO,
    RIGHT_JOINT_NAMES,
    TARGET_HISTORY_CAPACITY,
    shape_gripper_command,
)


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("openarm_v1_safe_launch", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_delta_module():
    scripts = str(PACKAGE / "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location("openarm_v1_delta_pose", DELTA_NODE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_pose_module():
    scripts = str(PACKAGE / "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    # The production node runs in the image's vt environment, while package
    # contract tests deliberately use /usr/bin/python3.  A nonfunctional stub
    # keeps this shutdown-only test independent of ADB and guarantees that an
    # accidental real reader construction fails immediately.
    if "ppadb" not in sys.modules and importlib.util.find_spec("ppadb") is None:
        ppadb_module = ModuleType("ppadb")
        ppadb_client_module = ModuleType("ppadb.client")

        class NoAdbClient:
            def __init__(self, *args, **kwargs):
                raise AssertionError("shutdown contract test must not access ADB")

        ppadb_client_module.Client = NoAdbClient
        ppadb_module.client = ppadb_client_module
        sys.modules.setdefault("ppadb", ppadb_module)
        sys.modules.setdefault("ppadb.client", ppadb_client_module)
    spec = importlib.util.spec_from_file_location("openarm_v1_pose", POSE_NODE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _transform(rotation, translation):
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def _assigned_numpy_array(node_source: Path, attribute: str) -> np.ndarray:
    """Read a literal ndarray assigned by production without constructing a ROS node."""

    tree = ast.parse(node_source.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(
            isinstance(target, ast.Attribute) and target.attr == attribute
            for target in node.targets
        ):
            continue
        value = node.value
        if (
            isinstance(value, ast.Call)
            and isinstance(value.func, ast.Attribute)
            and value.func.attr == "array"
            and value.args
        ):
            return np.asarray(ast.literal_eval(value.args[0]), dtype=float)
    raise AssertionError(f"missing literal ndarray assignment for {attribute}")


def _correct_openxr_pose(module, transform: np.ndarray) -> np.ndarray:
    openxr_to_ros = _assigned_numpy_array(POSE_NODE, "_openxr_to_ros")
    publisher = SimpleNamespace(
        _openxr_to_ros=openxr_to_ros,
        _ros_to_openxr=np.linalg.inv(openxr_to_ros),
    )
    return module.OpenArmOculusPublisher._correct_to_arm(publisher, transform)


@pytest.mark.parametrize("node_source", OPENARM_RUNTIME_PYTHON_NODES)
def test_openarm_runtime_nodes_shutdown_rclpy_idempotently(node_source):
    source = node_source.read_text(encoding="utf-8")
    main_source = source.split("def main", 1)[1]

    assert "from rclpy.executors import ExternalShutdownException" in source
    assert "except (KeyboardInterrupt, ExternalShutdownException):" in main_source
    assert "except RuntimeError as exc:" in main_source
    assert "is_rclpy_humble_shutdown_take_race(" in main_source
    assert "context_ok=rclpy.ok()" in main_source
    assert "if not shutdown_take_race:\n                raise" in main_source
    assert "finally:\n        try:" in main_source
    assert "finally:\n            rclpy.try_shutdown()" in main_source
    assert "rclpy.try_shutdown()" in main_source
    assert "rclpy.shutdown()" not in main_source


class _BlockingQuestFile:
    def __init__(self, *, wake_on_shutdown):
        self.entered_readline = threading.Event()
        self.release_readline = threading.Event()
        self.wake_on_shutdown = wake_on_shutdown
        self.closed = False

    def readline(self):
        self.entered_readline.set()
        self.release_readline.wait(timeout=2.0)
        return ""

    def close(self):
        self.closed = True


class _BlockingQuestSocket:
    def __init__(self, file_obj):
        self.file_obj = file_obj
        self.shutdown_calls = []

    def makefile(self):
        return self.file_obj

    def shutdown(self, how):
        self.shutdown_calls.append(how)
        if self.file_obj.wake_on_shutdown:
            self.file_obj.release_readline.set()


class _BlockingQuestConnection:
    def __init__(self, file_obj):
        self.socket = _BlockingQuestSocket(file_obj)
        self.close_count = 0

    def close(self):
        self.close_count += 1
        if self.socket.file_obj.wake_on_shutdown:
            self.socket.file_obj.release_readline.set()


class _BlockingQuestDevice:
    def __init__(self, connection):
        self.connection = connection
        self.commands = []

    def shell(self, command, handler=None):
        self.commands.append(command)
        if handler is not None:
            handler(self.connection)


def _make_reader_without_adb(module, *, wake_on_shutdown):
    file_obj = _BlockingQuestFile(wake_on_shutdown=wake_on_shutdown)
    connection = _BlockingQuestConnection(file_obj)
    reader = module.TimestampedOculusReader.__new__(module.TimestampedOculusReader)
    reader._snapshot_received_ns = 0
    reader._snapshot_sequence = 0
    reader._stream_lock = threading.Lock()
    reader._stream_connection = None
    reader._stream_file = None
    reader._lock = threading.Lock()
    reader.last_transforms = {}
    reader.last_buttons = {}
    reader.print_FPS = False
    reader.running = False
    reader.device = _BlockingQuestDevice(connection)
    return reader, connection, file_obj


def test_openarm_quest_reader_stop_wakes_blocking_readline_and_joins():
    module = _load_pose_module()
    reader, connection, file_obj = _make_reader_without_adb(
        module, wake_on_shutdown=True
    )

    reader.run()
    assert reader.thread.daemon
    assert file_obj.entered_readline.wait(timeout=1.0)

    started = time.monotonic()
    reader.stop()
    elapsed = time.monotonic() - started

    assert elapsed < reader.STOP_JOIN_TIMEOUT_SEC
    assert not reader.thread.is_alive()
    assert connection.socket.shutdown_calls
    assert all(call == socket.SHUT_RDWR for call in connection.socket.shutdown_calls)
    assert connection.close_count >= 1
    assert file_obj.closed


def test_openarm_quest_reader_stop_has_bounded_join_for_stubborn_stream():
    module = _load_pose_module()
    reader, connection, file_obj = _make_reader_without_adb(
        module, wake_on_shutdown=False
    )
    reader.STOP_JOIN_TIMEOUT_SEC = 0.02
    reader.run()
    assert file_obj.entered_readline.wait(timeout=1.0)

    started = time.monotonic()
    reader.stop()
    elapsed = time.monotonic() - started

    assert elapsed < 0.25
    assert reader.thread.daemon
    assert reader.thread.is_alive()
    assert connection.socket.shutdown_calls
    assert all(call == socket.SHUT_RDWR for call in connection.socket.shutdown_calls)
    assert connection.close_count == 1

    # Release the synthetic stubborn stream so this test leaves no live thread.
    file_obj.release_readline.set()
    reader.thread.join(timeout=1.0)
    assert not reader.thread.is_alive()


def test_openarm_quest_reader_swallows_expected_stop_makefile_race():
    module = _load_pose_module()
    reader, connection, _ = _make_reader_without_adb(
        module, wake_on_shutdown=True
    )
    makefile_entered = threading.Event()
    makefile_released = threading.Event()
    errors = []

    class RacingSocket:
        def __init__(self):
            self.shutdown_calls = []

        def makefile(self):
            makefile_entered.set()
            makefile_released.wait(timeout=1.0)
            raise OSError("synthetic close during makefile")

        def shutdown(self, how):
            self.shutdown_calls.append(how)
            makefile_released.set()

    connection.socket = RacingSocket()
    reader.running = True

    def invoke_reader():
        try:
            reader.read_logcat_by_line(connection)
        except BaseException as exc:  # record any thread escape for assertion
            errors.append(exc)

    reader.thread = threading.Thread(target=invoke_reader, daemon=True)
    reader.thread.start()
    assert makefile_entered.wait(timeout=1.0)

    reader.stop()

    assert not reader.thread.is_alive()
    assert errors == []
    assert connection.socket.shutdown_calls
    assert all(call == socket.SHUT_RDWR for call in connection.socket.shutdown_calls)
    assert connection.close_count >= 1


def test_openarm_pose_main_try_shutdown_survives_destroy_failure(monkeypatch):
    module = _load_pose_module()
    calls = []

    class FailingDestroyNode:
        def destroy_node(self):
            calls.append("destroy")
            raise RuntimeError("synthetic destroy failure")

    node = FailingDestroyNode()
    monkeypatch.setattr(module.rclpy, "init", lambda args=None: calls.append("init"))
    monkeypatch.setattr(module, "OpenArmOculusPublisher", lambda: node)

    def interrupt_spin(actual_node):
        assert actual_node is node
        calls.append("spin")
        raise KeyboardInterrupt

    monkeypatch.setattr(module.rclpy, "spin", interrupt_spin)
    monkeypatch.setattr(module.rclpy, "try_shutdown", lambda: calls.append("shutdown"))

    with pytest.raises(RuntimeError, match="synthetic destroy failure"):
        module.main()

    assert calls == ["init", "spin", "destroy", "shutdown"]


@pytest.fixture()
def launch_module():
    return _load_launch_module()


def test_qrafty_openxr_to_ros_matrix_is_exact_proper_rotation():
    actual = _assigned_numpy_array(POSE_NODE, "_openxr_to_ros")
    expected = np.array(
        (
            (0.0, 0.0, -1.0, 0.0),
            (-1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
    )
    assert np.array_equal(actual, expected)
    assert np.allclose(actual.T @ actual, np.eye(4))
    assert np.isclose(np.linalg.det(actual[:3, :3]), 1.0)


@pytest.mark.parametrize(
    ("raw_translation", "expected_flu_translation"),
    (
        ((0.10, 0.0, 0.0), (0.0, -0.10, 0.0)),
        ((0.0, 0.10, 0.0), (0.0, 0.0, 0.10)),
        ((0.0, 0.0, 0.10), (-0.10, 0.0, 0.0)),
    ),
)
def test_pub_pose_qrafty_conversion_maps_each_translation_axis_once(
    raw_translation,
    expected_flu_translation,
):
    module = _load_pose_module()
    raw_pose = _transform(np.eye(3), raw_translation)

    corrected = _correct_openxr_pose(module, raw_pose)

    assert np.allclose(corrected[:3, 3], expected_flu_translation)
    assert np.allclose(corrected[:3, :3], np.eye(3))


@pytest.mark.parametrize(
    ("raw_axis", "expected_flu_axis"),
    (
        ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
        ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)),
    ),
)
def test_pub_pose_qrafty_conversion_maps_each_rotation_axis_by_conjugation(
    raw_axis,
    expected_flu_axis,
):
    module = _load_pose_module()
    angle = 0.31
    raw_pose = _transform(
        Rotation.from_rotvec(angle * np.asarray(raw_axis)).as_matrix(),
        (0.0, 0.0, 0.0),
    )

    corrected = _correct_openxr_pose(module, raw_pose)
    expected_rotation = Rotation.from_rotvec(
        angle * np.asarray(expected_flu_axis)
    ).as_matrix()

    assert np.allclose(corrected[:3, :3], expected_rotation)
    assert np.allclose(corrected[:3, 3], np.zeros(3))


def test_pub_pose_qrafty_conversion_is_pure_a_t_a_inverse_for_mixed_pose():
    module = _load_pose_module()
    openxr_to_ros = _assigned_numpy_array(POSE_NODE, "_openxr_to_ros")
    raw_pose = _transform(
        Rotation.from_euler("xyz", (0.41, -0.27, 0.63)).as_matrix(),
        (0.17, -0.29, 0.38),
    )

    corrected = _correct_openxr_pose(module, raw_pose)

    assert np.allclose(
        corrected,
        openxr_to_ros @ raw_pose @ openxr_to_ros.T,
    )


@pytest.mark.parametrize(
    "invalid_pose",
    (
        np.eye(3),
        np.full((4, 4), float("nan")),
        np.vstack((np.eye(3, 4), np.array((0.0, 0.0, 1.0, 0.0)))),
    ),
)
def test_pub_pose_qrafty_conversion_rejects_invalid_transform(invalid_pose):
    module = _load_pose_module()
    with pytest.raises(ValueError, match="Quest transform"):
        _correct_openxr_pose(module, invalid_pose)


def test_raw_openxr_to_tcp_composition_applies_qrafty_axis_change_exactly_once():
    pose_module = _load_pose_module()
    delta_module = _load_delta_module()
    openxr_to_ros = _assigned_numpy_array(POSE_NODE, "_openxr_to_ros")
    axis_change = openxr_to_ros[:3, :3]
    zero_tcp = _transform(
        Rotation.from_euler("xyz", (0.37, -0.22, 0.51)).as_matrix(),
        (0.28, -0.19, 0.64),
    )
    raw_start = _transform(
        Rotation.from_euler("xyz", (-0.31, 0.46, -0.18)).as_matrix(),
        (0.91, -0.72, 1.34),
    )
    raw_current = _transform(
        Rotation.from_euler("xyz", (0.29, -0.14, 0.43)).as_matrix(),
        (1.08, -0.51, 1.12),
    )

    normalized_start = _correct_openxr_pose(pose_module, raw_start)
    normalized_current = _correct_openxr_pose(pose_module, raw_current)
    target = delta_module.compose_relative_target(
        zero_tcp,
        normalized_start,
        normalized_current,
    )

    assert np.allclose(
        target[:3, 3],
        zero_tcp[:3, 3]
        + axis_change @ (raw_current[:3, 3] - raw_start[:3, 3]),
    )
    assert np.allclose(
        target[:3, :3],
        axis_change
        @ (raw_current[:3, :3] @ raw_start[:3, :3].T)
        @ axis_change.T
        @ zero_tcp[:3, :3],
    )


@pytest.mark.parametrize(
    "handle_translation",
    (
        (0.10, 0.0, 0.0),
        (0.0, 0.10, 0.0),
        (0.0, 0.0, 0.10),
    ),
)
def test_relative_target_applies_each_normalized_translation_in_world_frame(
    handle_translation,
):
    module = _load_delta_module()
    zero_tcp = _transform(
        Rotation.from_euler("xyz", (0.35, -0.20, 0.55)).as_matrix(),
        (0.30, -0.25, 0.60),
    )
    start_handle = _transform(
        Rotation.from_euler("xyz", (-0.40, 0.25, -0.15)).as_matrix(),
        (1.0, 2.0, 3.0),
    )
    current_handle = start_handle.copy()
    current_handle[:3, 3] += np.asarray(handle_translation)

    target = module.compose_relative_target(zero_tcp, start_handle, current_handle)

    assert np.allclose(
        target[:3, 3] - zero_tcp[:3, 3],
        handle_translation,
    )
    assert np.allclose(target[:3, :3], zero_tcp[:3, :3])


def test_relative_target_uses_qrafty_world_translation_and_spatial_rotation_order():
    module = _load_delta_module()
    zero_tcp = _transform(
        Rotation.from_euler("xyz", (0.20, -0.30, 0.40)).as_matrix(),
        (0.35, -0.25, 0.70),
    )
    start_handle = _transform(
        Rotation.from_euler("xyz", (-0.15, 0.25, 0.10)).as_matrix(),
        (1.0, 2.0, 3.0),
    )
    current_handle = _transform(
        Rotation.from_euler("xyz", (0.45, -0.10, 0.30)).as_matrix(),
        (1.12, 1.82, 3.27),
    )

    target = module.compose_relative_target(zero_tcp, start_handle, current_handle)

    assert np.allclose(
        target[:3, 3],
        zero_tcp[:3, 3] + current_handle[:3, 3] - start_handle[:3, 3],
    )
    assert np.allclose(
        target[:3, :3],
        current_handle[:3, :3]
        @ start_handle[:3, :3].T
        @ zero_tcp[:3, :3],
    )


@pytest.mark.parametrize(
    "spatial_delta_rpy",
    (
        (0.30, 0.0, 0.0),
        (0.0, 0.25, 0.0),
        (0.0, 0.0, -0.35),
        (0.30, 0.25, -0.35),
    ),
)
def test_relative_target_left_multiplies_each_spatial_rotation_and_mixed_rotation(
    spatial_delta_rpy,
):
    module = _load_delta_module()
    zero_tcp = _transform(
        Rotation.from_euler("xyz", (0.20, -0.30, 0.40)).as_matrix(),
        (0.35, -0.25, 0.70),
    )
    start_handle = _transform(
        Rotation.from_euler("xyz", (-0.45, 0.35, 0.20)).as_matrix(),
        (1.0, 2.0, 3.0),
    )
    spatial_delta = Rotation.from_euler("xyz", spatial_delta_rpy).as_matrix()
    current_handle = start_handle.copy()
    current_handle[:3, :3] = spatial_delta @ start_handle[:3, :3]

    target = module.compose_relative_target(zero_tcp, start_handle, current_handle)

    assert np.allclose(target[:3, :3], spatial_delta @ zero_tcp[:3, :3])
    assert np.allclose(target[:3, 3], zero_tcp[:3, 3])


def test_relative_translation_is_independent_of_latched_tcp_and_handle_orientation():
    module = _load_delta_module()
    displacement = np.array((0.13, -0.08, 0.21))
    results = []
    for tcp_rpy, handle_rpy in (
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((0.80, -0.35, 1.10), (-1.20, 0.45, -0.60)),
    ):
        zero_tcp = _transform(
            Rotation.from_euler("xyz", tcp_rpy).as_matrix(),
            (0.2, -0.1, 0.5),
        )
        start_handle = _transform(
            Rotation.from_euler("xyz", handle_rpy).as_matrix(), (1.0, 2.0, 3.0)
        )
        current_handle = start_handle.copy()
        current_handle[:3, 3] += displacement
        target = module.compose_relative_target(zero_tcp, start_handle, current_handle)
        actual_displacement = target[:3, 3] - zero_tcp[:3, 3]
        assert np.allclose(actual_displacement, displacement)
        results.append(actual_displacement)

    assert np.allclose(results[0], results[1])


def test_relative_target_reanchor_has_no_pose_jump():
    module = _load_delta_module()
    zero_tcp = _transform(
        Rotation.from_euler("xyz", (0.30, 0.10, -0.20)).as_matrix(),
        (0.40, 0.20, 0.60),
    )
    start_handle = _transform(
        Rotation.from_euler("xyz", (-0.20, 0.40, 0.15)).as_matrix(),
        (-0.30, 0.80, 1.10),
    )
    target = module.compose_relative_target(zero_tcp, start_handle, start_handle)
    assert np.allclose(target, zero_tcp)


@pytest.mark.parametrize(
    "invalid_matrix",
    (
        np.eye(3),
        np.full((4, 4), float("nan")),
        np.vstack((np.eye(3, 4), np.array((0.0, 0.0, 1.0, 0.0)))),
    ),
)
def test_relative_target_rejects_invalid_transform(invalid_matrix):
    module = _load_delta_module()
    with pytest.raises(ValueError, match="relative pose"):
        module.compose_relative_target(np.eye(4), np.eye(4), invalid_matrix)


def test_qrafty_pose_contract_removes_legacy_j_b_second_a_and_tcp_local_paths():
    pose_source = POSE_NODE.read_text(encoding="utf-8")
    delta_source = DELTA_NODE.read_text(encoding="utf-8")
    config = yaml.safe_load(TELEOP_CONFIG.read_text(encoding="utf-8"))

    pose_correction = pose_source.split("def _correct_to_arm", 1)[1].split(
        "@staticmethod", 1
    )[0]
    assert "self._ros_to_openxr = np.linalg.inv(self._openxr_to_ros)" in pose_source
    assert "self._openxr_to_ros @ transform @ self._ros_to_openxr" in pose_correction
    assert "_controller_rotation" not in pose_source
    assert "_ros_to_arm" not in pose_source
    assert "ros_to_arm_rpy" not in pose_source
    assert "ros_to_arm_xyz" not in pose_source
    assert re.search(r"(?m)^\s*(?:J|B)\s*=", pose_source) is None

    relative_composition = delta_source.split("def compose_relative_target", 1)[1].split(
        "class OpenArmDeltaPosePublisher", 1
    )[0]
    assert "target[:3, 3] = zero_tcp[:3, 3] + delta_position" in relative_composition
    assert (
        "relative_rotation = current_handle[:3, :3] @ start_handle[:3, :3].T"
        in relative_composition
    )
    assert (
        "target[:3, :3] = relative_rotation @ zero_tcp[:3, :3]"
        in relative_composition
    )
    assert "VR_CONTROLLER_TO_TCP" not in delta_source
    assert "zero_tcp[:3, :3] @ mapped_position" not in delta_source
    assert "start_handle[:3, :3].T @ current_handle[:3, :3]" not in delta_source
    assert "zero_tcp[:3, :3] @ mapped_rotation" not in delta_source
    assert re.search(r"(?m)^\s*(?:J|B)\s*=", delta_source) is None

    pose_config = config["pub_pose_openarm_v1_node"]["ros__parameters"]
    assert "ros_to_arm_rpy" not in pose_config
    assert "ros_to_arm_xyz" not in pose_config
    for side in ("left", "right"):
        parameters = config[f"{side}_pub_delta_pose_openarm_v1"]["ros__parameters"]
        assert "relative_translation_scale_xyz" not in parameters


def _real_environment(module, token: str) -> dict[str, str]:
    return {
        "OPENARM_HARDWARE_MODE": "real",
        "OPENARM_REAL_PREFLIGHT_PASSED": "1",
        "OPENARM_REAL_PREFLIGHT_GATE": module.REAL_PREFLIGHT_GATE,
        "OPENARM_REAL_PREFLIGHT_TOKEN": token,
    }


def test_real_gate_accepts_only_exact_runner_nonce(tmp_path, monkeypatch, launch_module):
    token = "a" * 64
    gate = tmp_path / "preflight.gate"
    gate.write_bytes(token.encode("ascii") + b"\n")
    gate.chmod(0o444)
    monkeypatch.setattr(launch_module, "REAL_PREFLIGHT_GATE", str(gate))

    launch_module._require_real_runner_gate(_real_environment(launch_module, token))


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("OPENARM_HARDWARE_MODE", "fake"),
        ("OPENARM_REAL_PREFLIGHT_PASSED", "true"),
        ("OPENARM_REAL_PREFLIGHT_GATE", "/tmp/preflight.gate"),
        ("OPENARM_REAL_PREFLIGHT_TOKEN", "A" * 64),
        ("OPENARM_REAL_PREFLIGHT_TOKEN", "a" * 63),
    ],
)
def test_real_gate_rejects_each_invalid_environment_field(
    tmp_path, monkeypatch, launch_module, field, value
):
    token = "a" * 64
    gate = tmp_path / "preflight.gate"
    gate.write_bytes(token.encode("ascii") + b"\n")
    gate.chmod(0o444)
    monkeypatch.setattr(launch_module, "REAL_PREFLIGHT_GATE", str(gate))
    env = _real_environment(launch_module, token)
    env[field] = value
    with pytest.raises(RuntimeError):
        launch_module._require_real_runner_gate(env)


@pytest.mark.parametrize(
    ("content_suffix", "mode"),
    [
        (b"", 0o444),
        (b"\nextra\n", 0o444),
        (b"\n", 0o644),
        (b"\n", 0o400),
    ],
)
def test_real_gate_rejects_non_exact_or_wrong_mode(
    tmp_path, monkeypatch, launch_module, content_suffix, mode
):
    token = "b" * 64
    gate = tmp_path / "preflight.gate"
    gate.write_bytes(token.encode("ascii") + content_suffix)
    gate.chmod(mode)
    monkeypatch.setattr(launch_module, "REAL_PREFLIGHT_GATE", str(gate))
    with pytest.raises(RuntimeError):
        launch_module._require_real_runner_gate(_real_environment(launch_module, token))


def test_fake_gate_needs_no_real_runner_environment(monkeypatch, launch_module):
    for name in (
        "OPENARM_HARDWARE_MODE",
        "OPENARM_REAL_PREFLIGHT_PASSED",
        "OPENARM_REAL_PREFLIGHT_GATE",
        "OPENARM_REAL_PREFLIGHT_TOKEN",
    ):
        monkeypatch.delenv(name, raising=False)
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "use_fake_hardware": "true",
            "left_can_interface": "can0",
            "right_can_interface": "can1",
            "enable_home": "false",
            "start_quest_publisher": "true",
        }
    )
    assert launch_module._require_safe_configuration(context) == []


def test_fake_mode_allows_synthetic_quest_publisher(launch_module):
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "use_fake_hardware": "true",
            "left_can_interface": "can0",
            "right_can_interface": "can1",
            "enable_home": "false",
            "start_quest_publisher": "false",
        }
    )
    assert launch_module._require_safe_configuration(context) == []


def test_can_mapping_is_not_overridable(launch_module):
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "use_fake_hardware": "true",
            "left_can_interface": "can1",
            "right_can_interface": "can0",
            "enable_home": "false",
            "start_quest_publisher": "true",
        }
    )
    with pytest.raises(RuntimeError, match="left=can0 and right=can1"):
        launch_module._require_safe_configuration(context)


def test_real_mode_rejects_uncommissioned_home(
    tmp_path, monkeypatch, launch_module
):
    token = "c" * 64
    gate = tmp_path / "preflight.gate"
    gate.write_bytes(token.encode("ascii") + b"\n")
    gate.chmod(0o444)
    monkeypatch.setattr(launch_module, "REAL_PREFLIGHT_GATE", str(gate))
    for name, value in _real_environment(launch_module, token).items():
        monkeypatch.setenv(name, value)

    context = LaunchContext()
    context.launch_configurations.update(
        {
            "use_fake_hardware": "false",
            "left_can_interface": "can0",
            "right_can_interface": "can1",
            "enable_home": "true",
            "start_quest_publisher": "true",
        }
    )
    with pytest.raises(RuntimeError, match="HOME is not commissioned"):
        launch_module._require_safe_configuration(context)


def test_real_mode_cannot_disable_reviewed_quest_publisher(
    tmp_path, monkeypatch, launch_module
):
    token = "d" * 64
    gate = tmp_path / "preflight.gate"
    gate.write_bytes(token.encode("ascii") + b"\n")
    gate.chmod(0o444)
    monkeypatch.setattr(launch_module, "REAL_PREFLIGHT_GATE", str(gate))
    for name, value in _real_environment(launch_module, token).items():
        monkeypatch.setenv(name, value)

    context = LaunchContext()
    context.launch_configurations.update(
        {
            "use_fake_hardware": "false",
            "left_can_interface": "can0",
            "right_can_interface": "can1",
            "enable_home": "false",
            "start_quest_publisher": "false",
        }
    )
    with pytest.raises(RuntimeError, match="must start its reviewed Quest publisher"):
        launch_module._require_safe_configuration(context)


def test_controller_file_has_two_guarded_arms_and_two_guarded_grippers():
    config = yaml.safe_load(CONTROLLERS_FILE.read_text(encoding="utf-8"))
    manager = config["controller_manager"]["ros__parameters"]
    assert set(manager) == {
        "update_rate",
        "joint_state_broadcaster",
        "left_forward_position_controller",
        "right_forward_position_controller",
        "left_gripper_forward_position_controller",
        "right_gripper_forward_position_controller",
    }
    # OpenArm v1's official MoveIt configuration is 100 Hz.  Running the
    # eight-motor (arm + gripper) buses at the former 750 Hz starved the
    # last-sent ID8 feedback and tripped the 20 ms hardware watchdog.
    assert manager["update_rate"] == 100
    for side in ("left", "right"):
        controller = config[f"{side}_forward_position_controller"]["ros__parameters"]
        assert controller["joints"] == [
            f"openarm_{side}_joint{joint}" for joint in range(1, 8)
        ]
        assert controller["interface_name"] == "position"
        gripper = config[
            f"{side}_gripper_forward_position_controller"
        ]["ros__parameters"]
        assert gripper["joints"] == [f"openarm_{side}_finger_joint1"]
        assert gripper["interface_name"] == "position"


def test_teleop_uses_ninety_percent_of_official_openarm_control_velocity_caps():
    config = yaml.safe_load(TELEOP_CONFIG.read_text(encoding="utf-8"))
    expected_velocity = [1.413, 1.413, 2.826, 2.826, 11.34, 11.34, 11.34]
    expected_tracking = [0.12, 0.12, 0.12, 0.12, 0.24, 0.24, 0.24]

    ik = config["openarm_bimanual_ik"]["ros__parameters"]
    guard = config["openarm_command_guard"]["ros__parameters"]
    assert ik["solve_rate_hz"] == 100.0
    assert guard["control_rate_hz"] == 100.0
    for side in ("left", "right"):
        assert ik[f"{side}_max_velocity_rad_s"] == pytest.approx(expected_velocity)
        assert guard[f"{side}_max_velocity_rad_s"] == pytest.approx(expected_velocity)
    assert ik["max_tracking_error_rad"] == pytest.approx(expected_tracking)
    assert guard["max_tracking_error_rad"] == pytest.approx(expected_tracking)
    assert ik["max_tracking_error_rad"] == pytest.approx(
        guard["max_tracking_error_rad"]
    )
    assert all(
        2.0 * velocity / guard["control_rate_hz"] < tracking
        for velocity, tracking in zip(expected_velocity, expected_tracking)
    )


def test_launch_uses_local_guarded_bringup_and_enables_official_id8():
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert "IncludeLaunchDescription" not in source
    assert '"hand": "true"' in source
    assert "openarm_v1_bimanual_controllers.yaml" in source
    assert "left_gripper_forward_position_controller" in source
    assert "right_gripper_forward_position_controller" in source
    assert 'condition=IfCondition(start_quest_publisher)' in source
    assert 'LaunchConfiguration("start_quest_publisher")' in source
    assert "context.perform_substitution(controllers_file)" in source
    assert "parameters=[robot_description_param, controllers_file_str]" in source
    # Do not inject a process-wide ``__node`` remap into ros2_control_node.
    # Controllers are dynamically loaded in that process; the remap would rename
    # every controller to controller_manager and prevent their per-node YAML
    # sections (including the mandatory joint list) from matching.
    controller_manager_block = source.split("controller_manager = Node(", 1)[1].split(
        ")\n", 1
    )[0]
    assert 'name="controller_manager"' not in controller_manager_block


def test_arm_ik_and_guard_preserve_both_atomic_safety_references():
    ik_source = IK_NODE.read_text(encoding="utf-8")
    assert "solver_state = np.asarray(" in ik_source
    assert "build_driver_state(" in ik_source
    assert "build_atomic_ik_command(" in ik_source
    assert 'solver_bases = {"right": right_solver_base, "left": left_solver_base}' in ik_source
    pending_publish = ik_source.split("def _publish_pending_ik", 1)[1].split(
        "def _publish_sync_wait", 1
    )[0]
    assert "build_atomic_ik_command(" in pending_publish
    assert "intent_tokens," in pending_publish
    assert "select_solver_base(" in ik_source
    assert "decode_intent_token(" in ik_source
    assert "self._reset_bimanual_intent_session(now_ns)" in ik_source
    assert "validate_solver_candidate_step(" in ik_source
    assert "decision.accepted" in ik_source
    assert "decision.allowed" not in ik_source
    assert "_solver_shadow" not in ik_source
    assert "self._accepted_command" in ik_source
    assert "self._pending_ik" in ik_source
    assert "self._pending_acknowledged" in ik_source
    assert "self._pending_started_ns" in ik_source
    assert "self._guard_ack_fault" in ik_source
    assert "self._guard_anchor" in ik_source
    assert "self._guard_anchor_received_ns" in ik_source
    assert "self._publish_sync_wait(active_sides)" in ik_source
    assert "self._accepted_command[side] is not None" in ik_source
    assert "self._teleop_engaged_ns" in ik_source
    assert "def _guard_anchor_callback" in ik_source
    assert "is_post_engagement(" in ik_source
    assert "validate_guard_anchor(" in ik_source
    assert 'controller_anchor=self._guard_anchor["right"]' in ik_source
    assert 'controller_anchor=self._guard_anchor["left"]' in ik_source
    assert "validate_guard_acceptance_progress(" in ik_source
    assert '"right_guard_anchor_topic", "/openarm/guard_anchor/right"' in ik_source
    assert '"left_guard_anchor_topic", "/openarm/guard_anchor/left"' in ik_source
    assert '"right_sync_wait_topic", "/openarm/ik_sync_wait/right"' in ik_source
    assert '"left_sync_wait_topic", "/openarm/ik_sync_wait/left"' in ik_source
    assert 'self.declare_parameter("guard_anchor_timeout_sec", 0.30)' in ik_source
    assert 'self.declare_parameter("max_tracking_error_rad", DEFAULT_TRACKING_LIMITS)' in ik_source
    assert "parse_guard_acceptance(msg.data)" in ik_source
    assert "exact_echo" in ik_source
    assert "if self._pending_acknowledged[side]:" in ik_source
    pending_loop = ik_source.split("if pending_sides:", 1)[1].split(
        "selected_targets: dict[str, TargetHistorySample] = {}", 1
    )[0]
    assert "for side in pending_sides:" in pending_loop
    assert "for side in unacknowledged_sides:" in pending_loop
    assert "IK guard acknowledgement timed out" in pending_loop
    resend_gate = pending_loop.index("if unacknowledged_sides:")
    resend_publish = pending_loop.index("self._publish_pending_ik(side)", resend_gate)
    resend_return = pending_loop.index("return", resend_publish)
    pending_clear = pending_loop.index("self._pending_ik[side] = None", resend_return)
    assert pending_loop.count("self._publish_pending_ik(side)") == 1
    assert resend_gate < resend_publish < resend_return < pending_clear
    assert "feedback_state = np.asarray(feedback_driver_state, dtype=np.float32)" in ik_source
    solver_validation = ik_source.index("decision = validate_solver_candidate_step(")
    solver_soft_limit = ik_source.index("candidate = apply_joint_soft_limits(")
    bounded_validation = ik_source.index(
        "bounded_decision = validate_solver_candidate_step("
    )
    solver_commit = ik_source.index("self._pending_ik[side] = (")
    solver_publish = ik_source.rindex("self._publish_pending_ik(side)")
    assert (
        solver_validation
        < solver_soft_limit
        < bounded_validation
        < solver_commit
        < solver_publish
    )
    runtime_limits = ik_source.index(
        "self._verify_official_hard_joint_limits(setup)"
    )
    official_kinematics = ik_source.index("return Kinematics(setup, params)")
    assert runtime_limits < official_kinematics
    assert "bool(setup.model.jnt_limited[joint.id])" in ik_source
    assert "setup.model.jnt_limited[joint.id] =" not in ik_source
    assert "joint.range[:] =" not in ik_source
    guard_ack = ik_source.split("def _guard_acceptance_callback", 1)[1].split(
        "def _effective_mode", 1
    )[0]
    assert "self._hard_lower[side], self._hard_upper[side]" in guard_ack
    assert "self._lower[side], self._upper[side]" not in guard_ack

    guard_source = COMMAND_GUARD_NODE.read_text(encoding="utf-8")
    assert "candidate, solver_base, intent_tokens = parse_atomic_ik_command(msg.data)" in guard_source
    process_side = guard_source.split("def _process_side", 1)[1].split(
        "def _control_loop", 1
    )[0]
    solver_check = process_side.index("solver_decision = validate_solver_candidate_step(")
    controller_limit = process_side.index("arm_candidate = self._shape_arm_target(")
    final_validation = process_side.index("arm_decision = validate_guard_command(")
    assert solver_check < controller_limit < final_validation
    controller_publish = process_side.index("self.command_pub[side].publish(")
    acceptance_publish = process_side.index("self.guard_acceptance_pub[side].publish(")
    assert final_validation < controller_publish < acceptance_publish
    assert "build_guard_acceptance(" in process_side
    assert "generation != self.last_handled_ik_generation[side]" in process_side
    assert "payload == self.last_processed_ik_payload[side]" in process_side
    assert "retry_acceptance = cached_acceptance" in process_side
    assert "elif retry_acceptance is not None:" in process_side
    assert process_side.count("self.command_pub[side].publish(") == 1
    assert "shape_monotonic_guard_command(" in guard_source
    assert "monotonic_base=solver_base" in process_side
    assert "validate_guard_acceptance_progress(" in guard_source
    acceptance_validation = process_side.index(
        "acceptance_decision = validate_guard_acceptance_progress("
    )
    assert final_validation < acceptance_validation < controller_publish
    assert "reanchor_step_to_feedback" not in guard_source
    assert "self.guard_anchor_pub" in guard_source
    assert "self.ik_sync_wait_ns" in guard_source
    assert "self.ik_sync_wait_generated_ns" in guard_source
    assert "self.ik_sync_wait_started_ns" in guard_source
    assert "self.teleop_established_ns" in guard_source
    assert "def _ik_sync_wait_callback" in guard_source
    assert "sync_wait_fresh" in process_side
    assert "UInt64(data=generated_ns)" in ik_source
    assert "UInt64(data=0)" in ik_source
    assert "def _publish_controller_anchor" in guard_source
    assert "self.guard_anchor_pub[side].publish(" in guard_source
    assert '"right_guard_anchor_topic", "/openarm/guard_anchor/right"' in guard_source
    assert '"left_guard_anchor_topic", "/openarm/guard_anchor/left"' in guard_source
    assert 'self.declare_parameter("gripper_open_position_m", GRIPPER_OPEN_POSITION_M)' in guard_source
    assert '"gripper_open_velocity_m_s", GRIPPER_OPEN_VELOCITY_M_S' in guard_source
    assert '"gripper_close_velocity_m_s", GRIPPER_CLOSE_VELOCITY_M_S' in guard_source
    assert 'self.declare_parameter("gripper_reversal_settle_samples", 3)' in guard_source
    assert '"gripper_reversal_feedback_motion_tolerance_m", 0.00002' in guard_source
    assert 'self.declare_parameter("right_gripper_rearm_open_tolerance_m", 0.0045)' in guard_source
    assert 'self.declare_parameter("left_gripper_rearm_open_tolerance_m", 0.0025)' in guard_source
    assert "self.teleop_established" in guard_source
    assert "guard_teleop_ik_mode(" in process_side
    assert "teleop_engage_grace_sec" not in guard_source

    rearm = guard_source.split("def _try_rearm", 1)[1].split(
        "def _tracking_ok", 1
    )[0]
    assert "if self.last_command[side] is None:" in rearm
    assert "self.last_command[side] = self.feedback[side]" in rearm
    assert "if self.last_gripper_command[side] is None:" in rearm
    assert "self.last_gripper_command[side] = self.gripper_feedback[side]" in rearm
    assert "normalized_gripper >= self.gripper_rearm_open_intent_min" in rearm
    assert rearm.index("if not self._tracking_ok(side):") < rearm.index(
        "self.fault_latched[side] = False"
    )

    gripper_shape = guard_source.split("def _shape_gripper_target", 1)[1].split(
        "def _process_side", 1
    )[0]
    assert "interlock.should_hold(" in gripper_shape
    assert "shape_gripper_reversal_brake_command(" in gripper_shape
    assert "shape_gripper_command(" in gripper_shape
    assert gripper_shape.index("interlock.should_hold(") < gripper_shape.index(
        "shape_gripper_command("
    )
    assert gripper_shape.index("shape_gripper_command(") < gripper_shape.index(
        "decision = validate_gripper_command("
    )
    joint_callback = guard_source.split("def _joint_callback", 1)[1].split(
        "def _ik_callback", 1
    )[0]
    assert joint_callback.index("if gripper_name in position_by_name:") < joint_callback.index(
        "self.gripper_feedback_generation[side] += 1"
    )
    latch_fault = guard_source.split("def _latch_fault", 1)[1].split(
        "def _publish_effective_mode", 1
    )[0]
    assert "self.gripper_reversal_interlock[side].reset_pending()" in latch_fault
    gripper_publish = process_side.index("self.gripper_command_pub[side].publish(")
    reversal_commit = process_side.index(
        "self.gripper_reversal_interlock[side].commit_follow_command("
    )
    gripper_last_assignment = process_side.index(
        "self.last_gripper_command[side] = gripper_command"
    )
    assert gripper_publish < reversal_commit < gripper_last_assignment
    assert "and not gripper_reversal_brake:" in process_side
    assert "intent_deadband=self.gripper_reversal_catchup_tolerance" in process_side
    assert "self._shape_gripper_target(" in process_side
    assert "self.gripper_opening_after_rearm[side]" in process_side
    assert "self.gripper_open_position" in gripper_shape
    assert "self.gripper_upper_limit" in gripper_shape
    assert "self.gripper_open_position" in process_side
    assert "normalized_gripper = 1.0" in process_side
    assert "self.gripper_open_settle_started_ns[side]" in process_side
    assert "self.gripper_rearm_open_settle_sec" in process_side
    assert process_side.index("endpoint_ready = (") < process_side.index(
        ") = update_gripper_rearm_settle("
    )
    assert process_side.index(") = update_gripper_rearm_settle(") < process_side.index(
        "new Trigger presses are enabled"
    )
    assert "gripper did not reach the startup/rearm open endpoint" in process_side
    assert "gripper_engage_intent" not in guard_source
    assert "gripper_trigger_armed" not in guard_source
    assert "trigger_intent_changed(" not in guard_source

    delta_source = DELTA_NODE.read_text(encoding="utf-8")
    delta_loop = delta_source.split("def _control_loop", 1)[1].split(
        "def main", 1
    )[0]
    gripper_publish = delta_loop.index(
        "self.gripper_pub.publish(Float32(data=1.0 - self._trigger_value()))"
    )
    grip_branch = delta_loop.index("if hold_pressed:")
    assert gripper_publish < grip_branch
    assert "self._trigger_value() <= self.trigger_release_threshold" in delta_loop

    config = yaml.safe_load(TELEOP_CONFIG.read_text(encoding="utf-8"))
    ik_config = config["openarm_bimanual_ik"]["ros__parameters"]
    guard_config = config["openarm_command_guard"]["ros__parameters"]
    soft_limits = {
        "left_lower_limits": OPENARM_LEFT_JOINT_SOFT_LOWER_RAD,
        "left_upper_limits": OPENARM_LEFT_JOINT_SOFT_UPPER_RAD,
        "right_lower_limits": OPENARM_RIGHT_JOINT_SOFT_LOWER_RAD,
        "right_upper_limits": OPENARM_RIGHT_JOINT_SOFT_UPPER_RAD,
    }
    hard_limits = {
        "left_lower_limits": OPENARM_LEFT_JOINT_HARD_LOWER_RAD,
        "left_upper_limits": OPENARM_LEFT_JOINT_HARD_UPPER_RAD,
        "right_lower_limits": OPENARM_RIGHT_JOINT_HARD_LOWER_RAD,
        "right_upper_limits": OPENARM_RIGHT_JOINT_HARD_UPPER_RAD,
    }
    for side in ("left", "right"):
        assert ik_config[f"{side}_lower_limits"] == pytest.approx(
            soft_limits[f"{side}_lower_limits"], abs=1.1e-9
        )
        assert ik_config[f"{side}_upper_limits"] == pytest.approx(
            soft_limits[f"{side}_upper_limits"], abs=1.1e-9
        )
        assert guard_config[f"{side}_lower_limits"] == pytest.approx(
            hard_limits[f"{side}_lower_limits"], abs=1e-15
        )
        assert guard_config[f"{side}_upper_limits"] == pytest.approx(
            hard_limits[f"{side}_upper_limits"], abs=1e-15
        )
        assert all(
            soft >= hard
            for soft, hard in zip(
                ik_config[f"{side}_lower_limits"],
                guard_config[f"{side}_lower_limits"],
            )
        )
        assert all(
            soft <= hard
            for soft, hard in zip(
                ik_config[f"{side}_upper_limits"],
                guard_config[f"{side}_upper_limits"],
            )
        )
        assert ik_config[f"{side}_guard_acceptance_topic"] == guard_config[
            f"{side}_guard_acceptance_topic"
        ]
        assert ik_config[f"{side}_guard_anchor_topic"] == guard_config[
            f"{side}_guard_anchor_topic"
        ]
        assert ik_config[f"{side}_sync_wait_topic"] == guard_config[
            f"{side}_sync_wait_topic"
        ]
    assert tuple(ik_config["left_joint_names"]) == LEFT_JOINT_NAMES
    assert tuple(ik_config["right_joint_names"]) == RIGHT_JOINT_NAMES
    assert tuple(guard_config["left_joint_names"]) == LEFT_JOINT_NAMES
    assert tuple(guard_config["right_joint_names"]) == RIGHT_JOINT_NAMES
    assert guard_config["gripper_rearm_open_intent_min"] == pytest.approx(0.98)
    assert guard_config["right_gripper_rearm_open_tolerance_m"] == pytest.approx(
        0.0045
    )
    assert guard_config["left_gripper_rearm_open_tolerance_m"] == pytest.approx(
        0.0025
    )
    assert guard_config["gripper_rearm_open_settle_sec"] == pytest.approx(0.25)
    assert guard_config["gripper_lower_limit_m"] == pytest.approx(0.0)
    assert GRIPPER_OPEN_POSITION_M == pytest.approx(0.040)
    assert guard_config["gripper_open_position_m"] == pytest.approx(
        GRIPPER_OPEN_POSITION_M
    )
    assert guard_config["gripper_upper_limit_m"] == pytest.approx(0.044)
    assert (
        guard_config["gripper_lower_limit_m"]
        < guard_config["gripper_open_position_m"]
        < guard_config["gripper_upper_limit_m"]
    )
    assert ik_config["guard_ack_timeout_sec"] == pytest.approx(0.25)
    assert ik_config["guard_ack_timeout_sec"] < guard_config["ik_timeout_sec"]
    assert "teleop_engage_grace_sec" not in guard_config
    assert guard_config["sync_wait_max_sec"] == pytest.approx(0.75)
    assert guard_config["sync_wait_max_sec"] > guard_config["ik_timeout_sec"]
    assert ik_config["guard_anchor_timeout_sec"] == pytest.approx(0.30)
    assert ik_config["max_tracking_error_rad"] == pytest.approx(
        guard_config["max_tracking_error_rad"]
    )
    assert guard_config["gripper_rearm_open_timeout_sec"] == pytest.approx(5.0)
    assert (
        guard_config["gripper_open_position_m"]
        / guard_config["gripper_open_velocity_m_s"]
        + guard_config["gripper_rearm_open_settle_sec"]
        < guard_config["gripper_rearm_open_timeout_sec"]
    )
    assert GRIPPER_OPEN_VELOCITY_M_S == pytest.approx(0.500)
    assert GRIPPER_CLOSE_VELOCITY_M_S == pytest.approx(0.500)
    assert GRIPPER_OPEN_VELOCITY_M_S == pytest.approx(
        GRIPPER_CLOSE_VELOCITY_M_S
    )
    assert guard_config["gripper_open_velocity_m_s"] == pytest.approx(
        GRIPPER_OPEN_VELOCITY_M_S
    )
    assert guard_config["gripper_close_velocity_m_s"] == pytest.approx(
        GRIPPER_CLOSE_VELOCITY_M_S
    )
    assert guard_config["gripper_max_tracking_error_m"] == pytest.approx(0.00504)
    assert guard_config["gripper_reversal_settle_samples"] == 3
    assert guard_config[
        "gripper_reversal_feedback_motion_tolerance_m"
    ] == pytest.approx(0.00002)
    assert guard_config["control_rate_hz"] == pytest.approx(100.0)
    assert (
        guard_config["gripper_reversal_settle_samples"]
        / guard_config["control_rate_hz"]
        > 0.020
    )
    assert (
        guard_config["gripper_open_velocity_m_s"]
        / guard_config["control_rate_hz"]
        == pytest.approx(0.005)
    )
    assert (
        guard_config["gripper_close_velocity_m_s"]
        / guard_config["control_rate_hz"]
        == pytest.approx(0.005)
    )
    assert (
        guard_config["gripper_open_position_m"]
        / guard_config["gripper_close_velocity_m_s"]
        == pytest.approx(0.08)
    )
    assert (
        guard_config["gripper_open_position_m"]
        / guard_config["gripper_open_velocity_m_s"]
        == pytest.approx(0.08)
    )
    assert "gripper_max_velocity_m_s" not in guard_config
    assert (
        guard_config["gripper_open_position_m"]
        - guard_config["right_gripper_rearm_open_tolerance_m"]
        == pytest.approx(0.0355)
    )
    assert (
        guard_config["gripper_open_position_m"]
        - guard_config["left_gripper_rearm_open_tolerance_m"]
        == pytest.approx(0.0375)
    )
    assert all(
        guard_config[f"{side}_gripper_rearm_open_tolerance_m"]
        < guard_config["gripper_max_tracking_error_m"]
        for side in ("right", "left")
    )
    assert (
        guard_config["gripper_max_tracking_error_m"]
        - guard_config["right_gripper_rearm_open_tolerance_m"]
        == pytest.approx(0.00054)
    )
    right_rearm_floor = (
        guard_config["gripper_open_position_m"]
        - guard_config["right_gripper_rearm_open_tolerance_m"]
    )
    for measured_endpoint in (0.035703, 0.035767):
        assert measured_endpoint >= right_rearm_floor
        assert (
            guard_config["gripper_open_position_m"] - measured_endpoint
            < guard_config["gripper_max_tracking_error_m"]
        )
    assert guard_config["gripper_max_tracking_error_m"] > max(
        guard_config["gripper_open_velocity_m_s"],
        guard_config["gripper_close_velocity_m_s"],
    ) / guard_config["control_rate_hz"]
    assert (
        guard_config["gripper_max_tracking_error_m"]
        - max(
            guard_config["gripper_open_velocity_m_s"],
            guard_config["gripper_close_velocity_m_s"],
        )
        / guard_config["control_rate_hz"]
        == pytest.approx(0.00004)
    )
    assert (
        guard_config["gripper_reversal_feedback_motion_tolerance_m"]
        <= guard_config["gripper_max_tracking_error_m"]
        - max(
            guard_config["gripper_open_velocity_m_s"],
            guard_config["gripper_close_velocity_m_s"],
        )
        / guard_config["control_rate_hz"]
    )
    assert "gripper_trigger_activation_delta" not in guard_config


def test_gripper_adaptive_tracking_reserve_holds_until_feedback_catches_up():
    config = yaml.safe_load(TELEOP_CONFIG.read_text(encoding="utf-8"))
    guard = config["openarm_command_guard"]["ros__parameters"]
    open_step = guard["gripper_open_velocity_m_s"] / guard["control_rate_hz"]
    close_step = guard["gripper_close_velocity_m_s"] / guard["control_rate_hz"]
    tracking = guard["gripper_max_tracking_error_m"]

    assert open_step == pytest.approx(0.005)
    assert close_step == pytest.approx(0.005)
    assert tracking == pytest.approx(0.00504)

    opening_first = shape_gripper_command(
        current_command=0.010,
        target=GRIPPER_OPEN_POSITION_M,
        feedback=0.010,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking,
    )
    assert opening_first == pytest.approx(0.015)
    opening_held = shape_gripper_command(
        current_command=opening_first,
        target=GRIPPER_OPEN_POSITION_M,
        feedback=0.010,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking,
    )
    assert opening_held == pytest.approx(opening_first)
    assert shape_gripper_command(
        current_command=opening_held,
        target=GRIPPER_OPEN_POSITION_M,
        feedback=opening_held,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking,
    ) == pytest.approx(0.020)

    closing_first = shape_gripper_command(
        current_command=0.030,
        target=0.0,
        feedback=0.030,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking,
    )
    assert closing_first == pytest.approx(0.025)
    closing_held = shape_gripper_command(
        current_command=closing_first,
        target=0.0,
        feedback=0.030,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking,
    )
    assert closing_held == pytest.approx(closing_first)
    assert shape_gripper_command(
        current_command=closing_held,
        target=0.0,
        feedback=closing_held,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking,
    ) == pytest.approx(0.020)


def test_fake_smoke_validates_proportional_trigger_targets_and_safe_slew():
    smoke_source = FAKE_INPUT_SMOKE.read_text(encoding="utf-8")

    assert "GRIPPER_OPEN_STEP_M = GRIPPER_OPEN_VELOCITY_M_S /" in smoke_source
    assert "GRIPPER_CLOSE_STEP_M = GRIPPER_CLOSE_VELOCITY_M_S /" in smoke_source
    assert "FULL_CLOSE_MIN_NS" not in smoke_source
    assert "FULL_CLOSE_MAX_NS" not in smoke_source
    assert "FULL_CLOSE_TEST_TIMEOUT_NS = 5_000_000_000" in smoke_source
    assert '"right": GRIPPER_OPEN_POSITION_M * (1.0 - 0.25)' in smoke_source
    assert '"left": GRIPPER_OPEN_POSITION_M * (1.0 - 0.75)' in smoke_source
    assert "GRIPPER_COMMAND_QOS_DEPTH = 256" in smoke_source
    assert "GRIPPER_COMMAND_QOS_DEPTH," in smoke_source
    assert 'self._enter_phase("full_trigger_close")' in smoke_source
    assert 'self._enter_phase("full_trigger_reopen")' in smoke_source
    assert "command > previous + GRIPPER_COMMAND_STEP_TOLERANCE_M" in smoke_source
    assert "GRIPPER_CLOSE_STEP_M + GRIPPER_COMMAND_STEP_TOLERANCE_M" in smoke_source
    assert "command < previous - GRIPPER_COMMAND_STEP_TOLERANCE_M" in smoke_source
    assert "GRIPPER_OPEN_STEP_M + GRIPPER_COMMAND_STEP_TOLERANCE_M" in smoke_source
    assert "self.gripper_feedback[side] <= GRIPPER_FOLLOW_TOLERANCE_M" in smoke_source
    assert (
        "self.gripper_feedback[side]\n"
        "                >= GRIPPER_OPEN_POSITION_M - GRIPPER_FOLLOW_TOLERANCE_M"
        in smoke_source
    )
    assert "self.full_close_verified" in smoke_source
    assert "self.full_reopen_verified" in smoke_source


def test_hardware_runtime_gripper_velocity_gross_gate_is_single_sample_strict():
    patch_source = RUNTIME_GRIPPER_VELOCITY_PATCH.read_text(encoding="utf-8")

    assert (
        "constexpr double MAX_ABS_GRIPPER_FEEDBACK_VELOCITY = 18.849555921;"
        in patch_source
    )
    assert (
        "constexpr double RUNTIME_GRIPPER_VELOCITY_HARD_LIMIT = 18.849555921;"
        in patch_source
    )
    assert "MAX_ABS_GRIPPER_FEEDBACK_VELOCITY = 1.05" not in patch_source
    assert "RUNTIME_GRIPPER_VELOCITY_HARD_LIMIT = 1.20" not in patch_source
    assert (
        "constexpr uint64_t RUNTIME_GRIPPER_VELOCITY_MIN_FRESH_SAMPLES = 3;"
        in patch_source
    )
    assert (
        "RUNTIME_GRIPPER_VELOCITY_MIN_DURATION =\n"
        "+    std::chrono::milliseconds(20)" in patch_source
    )
    assert "struct RuntimeGripperVelocityExcursionState" in patch_source
    assert "bool over_limit_run = false;" in patch_source
    assert "uint64_t fresh_samples = 0;" in patch_source
    assert "double peak_abs_velocity = 0.0;" in patch_source

    runtime_start = patch_source.index("bool OpenArmHW::collect_runtime_feedback()")
    runtime_end = patch_source.index(
        "bool OpenArmHW::best_effort_disable()", runtime_start
    )
    runtime_source = patch_source[runtime_start:runtime_end]
    assert "bool gripper_fresh = false;" in runtime_source
    ordered_runtime_contract = (
        "feedback_stale",
        "!feedback_values_except_velocity_valid(gripper)",
        "MAX_ABS_GRIPPER_FEEDBACK_TORQUE",
        "if (velocity > RUNTIME_GRIPPER_VELOCITY_HARD_LIMIT)",
        "valid = false",
        "else if (gripper_fresh)",
        "velocity <= MAX_ABS_GRIPPER_FEEDBACK_VELOCITY",
        "excursion = RuntimeGripperVelocityExcursionState{}",
        "else if (!excursion.over_limit_run)",
        "excursion.fresh_samples = 1",
        "++excursion.fresh_samples",
        "RUNTIME_GRIPPER_VELOCITY_MIN_FRESH_SAMPLES",
        "RUNTIME_GRIPPER_VELOCITY_MIN_DURATION",
        "valid = false",
    )
    cursor = -1
    for token in ordered_runtime_contract:
        cursor = runtime_source.index(token, cursor + 1)

    assert "RUNTIME_GRIPPER_VELOCITY_MAX_PENDING_DURATION" not in patch_source
    for diagnostic in (
        "Runtime gripper velocity excursion pending side=%s can=%s",
        "Runtime gripper velocity excursion recovered side=%s",
        "Runtime gripper velocity trip reason=instant_hard side=%s",
        "Runtime gripper velocity trip reason=persistent side=%s",
    ):
        assert diagnostic in runtime_source
    assert runtime_source.count('"can=%s can_id=0x%02x') >= 3
    for diagnostic_field in (
        "can_id=0x%02x",
        "fresh_samples=%llu",
        "elapsed_ms=%.3f",
        "velocity_limit_ok=%s",
        "velocity_gate_ok=%s",
        "velocity_pending=%s",
    ):
        assert diagnostic_field in runtime_source
    assert runtime_source.count("gripper_velocity_policy_ok = false;") == 2
    assert (
        "gripper_velocity_policy_ok,\n"
        "+                      runtime_gripper_velocity_excursion_.over_limit_run,"
        in runtime_source
    )
    assert patch_source.count(
        "reset_runtime_gripper_velocity_excursion();"
    ) >= 5


def test_dual_intent_tokens_bind_mode_anchor_ik_ack_and_commit():
    core_source = TELEOP_CORE.read_text(encoding="utf-8")
    delta_source = DELTA_NODE.read_text(encoding="utf-8")
    ik_source = IK_NODE.read_text(encoding="utf-8")
    guard_source = COMMAND_GUARD_NODE.read_text(encoding="utf-8")

    assert GUARD_ANCHOR_DOF == 9
    assert ATOMIC_TARGET_DOF == 10
    assert ATOMIC_IK_COMMAND_DOF == 16
    assert GUARD_ACCEPTANCE_DOF == 23
    assert MAX_EXACT_FLOAT64_INT == (1 << 53) - 1
    assert "_intent_token_from_float64" in core_source
    assert "numeric.is_integer()" in core_source

    assert "self.mode_pub.publish(UInt64(data=self._intent_token))" in delta_source
    assert "transition_intent_token(self._intent_token, mode)" in delta_source
    assert "def _mode_callback(self, side: str, msg: UInt64)" in ik_source
    assert "def _mode_callback(self, side: str, msg: UInt64)" in guard_source
    assert "UInt8" not in ik_source
    for source in (delta_source, ik_source, guard_source):
        # Integer depth uses rclpy's reliable/volatile sensor-process default;
        # replay across a critical-node restart must never be enabled.
        assert "TRANSIENT_LOCAL" not in source

    ik_mode_callback = ik_source.split("def _mode_callback", 1)[1].split(
        "def _guard_anchor_callback", 1
    )[0]
    assert "decode_intent_token(raw_token)" in ik_mode_callback
    assert "token_changed = self._mode_token[side] != raw_token" in ik_mode_callback
    assert "self._mode_received_ns[side] = now_ns" in ik_mode_callback
    assert "if token_changed:" in ik_mode_callback
    assert "self._reset_bimanual_intent_session(now_ns)" in ik_mode_callback
    assert "OverflowError" not in ik_mode_callback

    ik_global_reset = ik_source.split("def _reset_bimanual_intent_session", 1)[1].split(
        "def _publish_pending_ik", 1
    )[0]
    assert "self._reset_solver_handshake()" in ik_global_reset
    assert "self._target[side] = None" in ik_global_reset
    assert "self._target_received_ns[side] = None" in ik_global_reset
    assert "self._target_epoch_ns[side] = None" in ik_global_reset
    assert "self._target_history[side].clear()" in ik_global_reset
    assert "self._last_selected_target_epoch_ns[side] = None" in ik_global_reset
    assert "self._guard_anchor[side] = None" in ik_global_reset
    assert "self._guard_anchor_received_ns[side] = None" in ik_global_reset
    assert "self._guard_anchor_tokens[side] = None" in ik_global_reset
    assert "self._sync_wait_active[side] = False" in ik_global_reset
    assert "now_ns if self._mode[side] == SideMode.TELEOP else None" in ik_global_reset

    ik_anchor_callback = ik_source.split("def _guard_anchor_callback", 1)[1].split(
        "def _guard_acceptance_callback", 1
    )[0]
    assert "anchor, intent_tokens = parse_guard_anchor(msg.data)" in ik_anchor_callback
    anchor_pair_check = ik_anchor_callback.index("intent_tokens != current_tokens")
    anchor_store = ik_anchor_callback.index("self._guard_anchor[side] = anchor")
    assert anchor_pair_check < ik_anchor_callback.index("return", anchor_pair_check) < anchor_store
    assert "pending_token_sets != {intent_tokens}" in ik_anchor_callback
    assert "self._reset_bimanual_intent_session(now_ns)" in ik_anchor_callback
    assert "self._guard_anchor_tokens[side] = intent_tokens" in ik_anchor_callback

    ik_ack_callback = ik_source.split("def _guard_acceptance_callback", 1)[1].split(
        "def _effective_mode", 1
    )[0]
    assert "acknowledged_tokens" in ik_ack_callback
    assert "pending_candidate, pending_base, pending_tokens = pending" in ik_ack_callback
    assert "acknowledged_tokens != current_tokens" in ik_ack_callback
    assert "acknowledged_tokens != pending_tokens" in ik_ack_callback
    token_check = ik_ack_callback.index("acknowledged_tokens != current_tokens")
    accepted_commit = ik_ack_callback.index("self._accepted_command[side] =")
    assert token_check < ik_ack_callback.index("return", token_check) < accepted_commit

    ik_solve = ik_source.split("def _solve_once", 1)[1].split("def main", 1)[0]
    assert "intent_tokens = self._current_intent_tokens()" in ik_solve
    assert "self._guard_anchor_tokens[side] != intent_tokens" in ik_solve
    assert "self._pending_ik[side][2] != intent_tokens" in ik_solve
    pending_install = ik_solve.index("self._pending_ik[side] = (")
    pending_tokens = ik_solve.index("intent_tokens,", pending_install)
    pending_publish = ik_solve.rindex("self._publish_pending_ik(side)")
    assert pending_install < pending_tokens < pending_publish

    guard_ik_callback = guard_source.split("def _ik_callback", 1)[1].split(
        "def _ik_sync_wait_callback", 1
    )[0]
    stale_check = guard_ik_callback.index(
        "intent_tokens != self._current_intent_tokens()"
    )
    guard_ik_store = guard_ik_callback.index("self.ik[side] = candidate")
    assert stale_check < guard_ik_callback.index("return", stale_check) < guard_ik_store
    assert guard_ik_callback.index("self.ik_ns[side] =") > stale_check
    assert guard_ik_callback.index("self.ik_generation[side] += 1") > stale_check

    guard_mode_callback = guard_source.split("def _mode_callback", 1)[1].split(
        "def _current_intent_tokens", 1
    )[0]
    assert "decode_intent_token(raw_token)" in guard_mode_callback
    assert "OverflowError" not in guard_mode_callback
    assert "raw_token != self.current_intent_token[side]" in guard_mode_callback
    assert "self._invalidate_bimanual_ik_state(now_ns)" in guard_mode_callback
    guard_global_reset = guard_source.split("def _invalidate_bimanual_ik_state", 1)[
        1
    ].split("def _reset_ik_acceptance_cache", 1)[0]
    for field in (
        "ik",
        "ik_solver_base",
        "ik_intent_tokens",
        "ik_ns",
        "ik_sync_wait_ns",
        "ik_sync_wait_generated_ns",
        "ik_sync_wait_started_ns",
        "teleop_established_ns",
    ):
        assert f"self.{field}[selected] = None" in guard_global_reset

    guard_process = guard_source.split("def _process_side", 1)[1].split(
        "def _control_loop", 1
    )[0]
    final_token_check = guard_process.index(
        "if guard_echo is not None and not self._teleop_tokens_are_current("
    )
    controller_publish = guard_process.index("self.command_pub[side].publish(")
    assert final_token_check < guard_process.index("return", final_token_check) < controller_publish
    assert "echoed_candidate, echoed_base, echoed_tokens = guard_echo" in guard_process
    assert "echoed_tokens," in guard_process
    retry_check = guard_process.index(
        "if not self._teleop_tokens_are_current(side, retry_intent_tokens):"
    )
    retry_publish = guard_process.index(
        "self.guard_acceptance_pub[side].publish(", retry_check
    )
    assert retry_check < guard_process.index("return", retry_check) < retry_publish


def test_atomic_target_binds_pose_epoch_and_current_side_intent():
    core_source = TELEOP_CORE.read_text(encoding="utf-8")
    delta_source = DELTA_NODE.read_text(encoding="utf-8")
    ik_source = IK_NODE.read_text(encoding="utf-8")
    config = yaml.safe_load(TELEOP_CONFIG.read_text(encoding="utf-8"))
    ik_config = config["openarm_bimanual_ik"]["ros__parameters"]

    assert ATOMIC_TARGET_DOF == 10
    assert "def build_atomic_target(" in core_source
    assert "def parse_atomic_target(" in core_source
    assert "payload[ARM_DOF + 2]" in core_source
    assert "atomic target intent token must encode TELEOP mode" in core_source

    delta_loop = delta_source.split("def _control_loop", 1)[1].split(
        "def main", 1
    )[0]
    mode_publish = delta_loop.index("self._publish_mode(SideMode.TELEOP)")
    atomic_build = delta_loop.index("atomic_target = build_atomic_target(")
    debug_publish = delta_loop.index("self.target_pub.publish(target)")
    atomic_publish = delta_loop.index("self.target_intent_pub.publish(")
    assert mode_publish < atomic_build < atomic_publish
    assert mode_publish < debug_publish
    assert "self._intent_token," in delta_loop[atomic_build:atomic_publish]

    for side in ("left", "right"):
        delta_config = config[f"{side}_pub_delta_pose_openarm_v1"][
            "ros__parameters"
        ]
        assert delta_config["target_intent_topic"] == ik_config[
            f"{side}_target_intent_topic"
        ]
        assert delta_config["target_pose_topic"] == ik_config[
            f"{side}_target_pose_topic"
        ]
        assert (
            f'"{side}_target_intent_topic", "/openarm/target_intent/{side}"'
            in ik_source
        )

    # PoseStamped targets remain available from delta for RViz, but the IK
    # control path subscribes only to the token-bound Float64 payload.
    assert "PoseStamped, str(self.get_parameter(\"target_pose_topic\").value), 1" in delta_source
    assert "self.target_pub.publish(target)" in delta_source
    assert "create_subscription(\n            PoseStamped" not in ik_source
    assert "str(self.get_parameter(\"right_target_intent_topic\").value)" in ik_source
    assert "str(self.get_parameter(\"left_target_intent_topic\").value)" in ik_source

    target_callback = ik_source.split("def _target_callback", 1)[1].split(
        "def _mode_callback", 1
    )[0]
    parse_target = target_callback.index(
        "target, epoch_ns, raw_token = parse_atomic_target(msg.data)"
    )
    token_check = target_callback.index("raw_token != self._mode_token[side]")
    mode_check = target_callback.index("self._mode[side] != SideMode.TELEOP")
    accepted_store = target_callback.index("self._target[side] = target")
    assert parse_target < token_check < mode_check < accepted_store
    assert target_callback.index("return", mode_check) < accepted_store
    assert "receipt_ns = time.monotonic_ns()" in target_callback
    assert "self._target_received_ns[side] = receipt_ns" in target_callback
    assert "self._target_epoch_ns[side] = epoch_ns" in target_callback
    assert "OverflowError" not in target_callback


def test_late_peer_uses_fixed_history_cohort_barrier_without_relaxing_timeouts():
    ik_source = IK_NODE.read_text(encoding="utf-8")
    config_source = TELEOP_CONFIG.read_text(encoding="utf-8")

    assert TARGET_HISTORY_CAPACITY == 32
    assert "from collections import deque" in ik_source
    assert "deque(maxlen=TARGET_HISTORY_CAPACITY)" in ik_source
    assert "target_history_capacity" not in config_source

    target_callback = ik_source.split("def _target_callback", 1)[1].split(
        "def _mode_callback", 1
    )[0]
    pair_snapshot = target_callback.index(
        "current_tokens = self._current_intent_tokens()"
    )
    token_check = target_callback.index("raw_token != self._mode_token[side]")
    mismatch_return = target_callback.index("return", token_check)
    sample_build = target_callback.index("TargetHistorySample(")
    history_append = target_callback.index("self._target_history[side].append(")
    assert pair_snapshot < token_check < mismatch_return < history_append < sample_build
    assert "intent_tokens=current_tokens" in target_callback

    effective_mode = ik_source.split("def _effective_mode", 1)[1].split(
        "def _publish_tcp", 1
    )[0]
    assert "self._mode_received_ns[side]" in effective_mode
    assert "return self._mode[side]" in effective_mode
    assert "_target_received_ns" not in effective_mode

    latest_sample = ik_source.split("def _latest_current_target_sample", 1)[1].split(
        "def _reset_bimanual_intent_session", 1
    )[0]
    assert "sample.intent_tokens == intent_tokens" in latest_sample
    assert "is_fresh(" in latest_sample
    assert "sample.receipt_ns" in latest_sample
    assert "default=None" in latest_sample

    solve = ik_source.split("def _solve_once", 1)[1].split("def main", 1)[0]
    cohort = solve.index("active_sides = tuple(")
    no_active = solve.index("if not active_sides:")
    fresh_clock = solve.index("fresh_now_ns = time.monotonic_ns()")
    fresh_feedback_barrier = solve.index("fresh_joint_feedback = all(")
    fresh_mode_barrier = solve.index("fresh_mode_cohort = (")
    fresh_barrier = solve.index("fresh_current_targets = {")
    missing_target_wait = solve.index(
        "if (\n                any(sample is None for sample in "
        "fresh_current_targets.values())"
    )
    pending_branch = solve.index("pending_sides = tuple(")
    pair_select = solve.index("select_coherent_target_history_pair(")
    pair_wait = solve.index("if selected_pair is None:")
    anchor_base_snapshot = solve.index("anchor_based_sides = tuple(")
    solver_sync = solve.index("self.kinematics.sync(solver_state)")
    assert (
        cohort
        < no_active
        < fresh_clock
        < fresh_feedback_barrier
        < fresh_mode_barrier
        < fresh_barrier
        < missing_target_wait
        < pending_branch
        < pair_select
        < pair_wait
        < anchor_base_snapshot
        < solver_sync
    )
    missing_block = solve[missing_target_wait:pending_branch]
    assert "self._reset_solver_handshake()" in missing_block
    assert "self._publish_sync_wait(())" in missing_block
    assert "return" in missing_block
    assert "_publish_pending_ik" not in missing_block
    assert "_last_selected_target_epoch_ns" not in missing_block
    assert "fresh_now_ns," in solve[fresh_barrier:missing_target_wait]
    pre_pending_freshness = solve[fresh_feedback_barrier:missing_target_wait]
    assert "self._current_intent_tokens() == intent_tokens" in pre_pending_freshness
    assert "self._mode[side] == SideMode.TELEOP" in pre_pending_freshness
    assert "self._mode_received_ns[side]" in pre_pending_freshness
    assert "self._joint_received_ns[side]" in pre_pending_freshness
    assert 'for side in ("right", "left")' in pre_pending_freshness
    assert pre_pending_freshness.count("fresh_now_ns,") >= 2
    assert "or not fresh_mode_cohort" in missing_block
    assert "or not fresh_joint_feedback" in missing_block
    pending_block = solve[pending_branch:pair_select]
    assert "fresh_now_ns - started_ns" in pending_block
    assert "now_ns=fresh_now_ns" in solve[pair_select:pair_wait]
    pair_wait_block = solve[pair_wait:solve.index("else:", pair_wait)]
    assert "self._publish_sync_wait(active_sides)" in pair_wait_block
    assert "last_selected_epoch_ns=(" in solve[pair_select:pair_wait]
    assert "enforce_watermark=True" in solve
    assert "selected_target.pose" in solve
    assert "teleop_targets_are_coherent" not in ik_source

    candidate_validated = solve.index("validated_candidates[side] = candidate")
    commit_clock = solve.index("commit_ns = time.monotonic_ns()")
    commit_gate = solve.index("intent_tokens_still_current =")
    target_only_decision = solve.index(
        "target_only_expiry = target_only_commit_expiry_allows_sync_wait("
    )
    target_only_branch = solve.index("if target_only_expiry:")
    commit_failure = solve.index("if not (", target_only_branch)
    pending_install = solve.index("self._pending_ik[side] = (")
    watermark_advance = solve.index(
        "self._last_selected_target_epoch_ns[side] = ("
    )
    handoff = solve.index("self._handoff_sync_wait_to_pending_ik()")
    pending_publish = solve.rindex("self._publish_pending_ik(side)")
    assert (
        candidate_validated
        < commit_clock
        < commit_gate
        < target_only_decision
        < target_only_branch
        < commit_failure
        < pending_install
        < watermark_advance
        < handoff
        < pending_publish
    )
    commit_checks = solve[commit_gate:target_only_branch]
    assert "current_commit_tokens == intent_tokens" in commit_checks
    assert "selected_targets[side].intent_tokens == intent_tokens" in commit_checks
    assert "selected_targets[side].receipt_ns" in commit_checks
    current_stream_check = commit_checks.split(
        "current_target_streams_still_fresh =", 1
    )[1].split("modes_still_current =", 1)[0]
    assert "self._latest_current_target_sample(" in current_stream_check
    assert "intent_tokens," in current_stream_check
    assert "enforce_watermark=False" in current_stream_check
    assert "is not None" in current_stream_check
    assert "self._mode[side] == SideMode.TELEOP" in commit_checks
    assert "self._mode_received_ns[side]" in commit_checks
    assert "self._joint_received_ns[side]" in commit_checks
    assert 'for side in ("right", "left")' in commit_checks
    assert "self._guard_anchor_tokens[side] == intent_tokens" in commit_checks
    assert "self._guard_anchor_received_ns[side]" in commit_checks
    assert "self._teleop_engaged_ns[side]" in commit_checks
    assert "self.guard_anchor_timeout_sec" in commit_checks
    assert "for side in anchor_based_sides" in commit_checks
    anchor_snapshot = solve[anchor_base_snapshot:solver_sync]
    assert "if self._accepted_command[side] is None" in anchor_snapshot
    assert commit_checks.count("commit_ns,") >= 5
    helper_call = solve[target_only_decision:target_only_branch]
    for keyword in (
        "intent_tokens_current=intent_tokens_still_current",
        "selected_targets_token_bound=selected_targets_token_bound",
        "selected_targets_fresh=selected_targets_still_fresh",
        "current_target_streams_fresh=current_target_streams_still_fresh",
        "modes_current=modes_still_current",
        "feedback_fresh=feedback_still_fresh",
        "anchors_current=anchors_still_current",
    ):
        assert keyword in helper_call

    target_only_block = solve[target_only_branch:commit_failure]
    assert "self._apply_commit_gate_failure(" in target_only_block
    assert "target_only_expiry=True" in target_only_block
    assert "active_sides=active_sides" in target_only_block
    assert "return" in target_only_block
    assert "self._pending_ik[side] = (" not in target_only_block
    assert "self._last_selected_target_epoch_ns[side] = (" not in target_only_block
    assert "self._publish_pending_ik(side)" not in target_only_block

    commit_failure_block = solve[commit_failure:pending_install]
    assert "self._apply_commit_gate_failure(" in commit_failure_block
    assert "target_only_expiry=False" in commit_failure_block
    assert "active_sides=active_sides" in commit_failure_block
    assert "return" in commit_failure_block
    assert "self._pending_ik[side] = (" not in commit_failure_block
    assert "self._last_selected_target_epoch_ns[side] = (" not in commit_failure_block
    assert "self._publish_pending_ik(side)" not in commit_failure_block

    commit_failure_helper = ik_source.split(
        "def _apply_commit_gate_failure(", 1
    )[1].split("def _current_intent_tokens", 1)[0]
    helper_target_only = commit_failure_helper.split(
        "if target_only_expiry:", 1
    )[1].split("self._reset_solver_handshake()", 1)[0]
    assert "self._reset_pending_handshake()" in helper_target_only
    assert "self._publish_sync_wait(active_sides)" in helper_target_only
    assert "self._publish_sync_wait(())" not in helper_target_only
    assert "return" in helper_target_only
    helper_full_failure = commit_failure_helper.split(
        "self._reset_solver_handshake()", 1
    )[1]
    assert "self._publish_sync_wait(())" in helper_full_failure
    assert "self._publish_sync_wait(active_sides)" not in helper_full_failure
    for forbidden in (
        "self._pending_ik[side] = (",
        "self._pending_started_ns[side] =",
        "self._last_selected_target_epoch_ns[side] = (",
        "self._handoff_sync_wait_to_pending_ik()",
        "self._publish_pending_ik(side)",
    ):
        assert forbidden not in commit_failure_helper

    core_source = TELEOP_CORE.read_text(encoding="utf-8")
    target_only_helper = core_source.split(
        "def target_only_commit_expiry_allows_sync_wait(", 1
    )[1].split("\ndef ", 1)[0]
    assert "not bool(selected_targets_fresh)" in target_only_helper
    for required in (
        "intent_tokens_current",
        "selected_targets_token_bound",
        "current_target_streams_fresh",
        "modes_current",
        "feedback_fresh",
        "anchors_current",
    ):
        assert f"bool({required})" in target_only_helper
    pending_install_block = solve[pending_install:watermark_advance]
    assert "self._pending_started_ns[side] = commit_ns" in pending_install_block
    assert "self._pending_started_ns[side] = now_ns" not in pending_install_block
    # Waiting and handled failure paths cannot consume source epochs. The only
    # non-reset watermark assignment is next to a successfully installed round.
    assert solve.count("self._last_selected_target_epoch_ns[side] = (") == 1


def test_sync_wait_solver_handoff_avoids_zero_window_and_fails_closed():
    ik_source = IK_NODE.read_text(encoding="utf-8")
    handoff = ik_source.split("def _handoff_sync_wait_to_pending_ik", 1)[1].split(
        "def _solve_once", 1
    )[0]
    assert "self._sync_wait_active[side] = False" in handoff
    assert ".publish(" not in handoff
    assert "_publish_sync_wait" not in handoff

    solve = ik_source.split("def _solve_once", 1)[1].split("def main", 1)[0]
    solver_preamble = solve.split(
        "# Preserve the last synchronization heartbeat", 1
    )[1].split("if not self.kinematics.ready():", 1)[0]
    assert "_publish_sync_wait" not in solver_preamble

    def failure_block(start: str, end: str, source: str = solve) -> str:
        return source.split(start, 1)[1].split(end, 1)[0]

    failure_blocks = (
        failure_block("if not self.kinematics.ready():", "result = self.kinematics.solve()"),
        failure_block("except (RuntimeError, ValueError) as exc:", "if result is None:"),
        failure_block("if result is None:", "try:\n            right_result"),
        failure_block(
            "try:\n            right_result, left_result = split_driver_state(result)",
            'results = {"right": right_result, "left": left_result}',
        ),
        failure_block(
            "if not decision.accepted or decision.command is None:",
            "try:\n                candidate = apply_joint_soft_limits(",
        ),
        failure_block(
            "candidate = apply_joint_soft_limits(",
            "bounded_decision = validate_solver_candidate_step(",
        ),
        failure_block(
            "if not bounded_decision.accepted or bounded_decision.command is None:",
            "candidate = bounded_decision.command",
        ),
    )
    for block in failure_blocks:
        assert "self._publish_sync_wait(())" in block
        assert "return" in block

    pending_install = solve.index("self._pending_ik[side] = (")
    handoff_call = solve.index("self._handoff_sync_wait_to_pending_ik()")
    pending_publish = solve.rindex("self._publish_pending_ik(side)")
    assert pending_install < handoff_call < pending_publish


def test_guard_arming_timeout_and_sync_wait_priority_are_fail_closed():
    core_source = TELEOP_CORE.read_text(encoding="utf-8")
    guard_source = COMMAND_GUARD_NODE.read_text(encoding="utf-8")
    config = yaml.safe_load(TELEOP_CONFIG.read_text(encoding="utf-8"))
    guard_config = config["openarm_command_guard"]["ros__parameters"]

    arming_timeout = guard_config["teleop_arming_timeout_sec"]
    assert arming_timeout == pytest.approx(0.75)
    assert arming_timeout > 0.0
    assert 'self.declare_parameter("teleop_arming_timeout_sec", 0.75)' in guard_source
    positive_values = guard_source.split("positive_values = (", 1)[1].split(
        ")\n", 1
    )[0]
    assert "self.teleop_arming_timeout_sec" in positive_values

    mode_decision = core_source.split("def guard_teleop_ik_mode(", 1)[1].split(
        "\ndef ", 1
    )[0]
    sync_wait_priority = mode_decision.index("if bool(sync_wait_fresh):")
    fresh_ik_priority = mode_decision.index(
        "if bool(ik_after_engagement) and bool(ik_fresh):"
    )
    assert sync_wait_priority < fresh_ik_priority

    process_side = guard_source.split("def _process_side", 1)[1].split(
        "def _control_loop", 1
    )[0]
    stale_fault = process_side.split("elif ik_mode == SideMode.FAULT:", 1)[1].split(
        "\n            else:\n                candidate", 1
    )[0]
    assert "not self.teleop_established[side]" in stale_fault
    assert "and not arming_within_deadline" in stale_fault
    assert "first post-engagement IK command missed the" in stale_fault
    assert 'reason = "IK command stale"' in stale_fault
    assert "self._latch_fault(side, reason)" in stale_fault

    first_commit = process_side.split(
        "if arm_decision is not None and arm_decision.command is not None:", 1
    )[1].split("elif retry_acceptance is not None:", 1)[0]
    deadline_recheck = first_commit.index("if not teleop_arming_within_deadline(")
    deadline_fault = first_commit.index(
        "first post-engagement IK command missed the", deadline_recheck
    )
    arm_publish = first_commit.index("self.command_pub[side].publish(")
    assert deadline_recheck < deadline_fault < arm_publish


def test_guard_sync_wait_episode_resets_only_after_successful_ack():
    guard_source = COMMAND_GUARD_NODE.read_text(encoding="utf-8")

    ik_callback = guard_source.split("def _ik_callback", 1)[1].split(
        "def _ik_sync_wait_callback", 1
    )[0]
    assert "self.ik_sync_wait_ns[side] = None" in ik_callback
    assert "self.ik_sync_wait_generated_ns[side] = None" in ik_callback
    assert "self.ik_sync_wait_started_ns[side]" not in ik_callback

    sync_wait_callback = guard_source.split("def _ik_sync_wait_callback", 1)[1].split(
        "def _gripper_intent_callback", 1
    )[0]
    assert "generated_ns > 0" in sync_wait_callback
    invalid_lease = sync_wait_callback.split("if not valid_current_lease:", 1)[1].split(
        "if self.ik_sync_wait_started_ns[side] is None:", 1
    )[0]
    assert "self.ik_sync_wait_ns[side] = None" in invalid_lease
    assert "self.ik_sync_wait_generated_ns[side] = None" in invalid_lease
    assert "self.ik_sync_wait_started_ns[side]" not in invalid_lease
    assert sync_wait_callback.count("self.ik_sync_wait_started_ns[side] =") == 1

    process_side = guard_source.split("def _process_side", 1)[1].split(
        "def _control_loop", 1
    )[0]
    normal_commit = process_side.split(
        "if arm_decision is not None and arm_decision.command is not None:", 1
    )[1].split("elif retry_acceptance is not None:", 1)[0]
    normal_ack = normal_commit.split("if guard_echo is not None:", 1)[1]
    normal_ack_publish = normal_ack.index("self.guard_acceptance_pub[side].publish(")
    normal_clear_positions = [
        normal_ack.index(f"self.{field}[side] = None")
        for field in (
            "ik_sync_wait_ns",
            "ik_sync_wait_generated_ns",
            "ik_sync_wait_started_ns",
        )
    ]
    assert all(normal_ack_publish < position for position in normal_clear_positions)

    retry_ack = process_side.split("elif retry_acceptance is not None:", 1)[1].split(
        "self.gripper_command_pub[side].publish(", 1
    )[0]
    retry_ack_publish = retry_ack.index("self.guard_acceptance_pub[side].publish(")
    retry_generation_handled = retry_ack.index(
        "self.last_handled_ik_generation[side] = self.ik_generation[side]"
    )
    retry_clear_positions = [
        retry_ack.index(f"self.{field}[side] = None")
        for field in (
            "ik_sync_wait_ns",
            "ik_sync_wait_generated_ns",
            "ik_sync_wait_started_ns",
        )
    ]
    assert retry_ack_publish < retry_generation_handled
    assert all(retry_generation_handled < position for position in retry_clear_positions)


def test_hard_limits_match_pinned_official_model_and_soft_limits_are_95_percent():
    model = ET.parse(OFFICIAL_BIMANUAL_MODEL)
    model_ranges = {
        joint.attrib["name"]: tuple(float(value) for value in joint.attrib["range"].split())
        for joint in model.getroot().iter("joint")
        if joint.attrib.get("name") in LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES
    }
    expected_hard = {
        **{
            name: bounds
            for name, bounds in zip(
                LEFT_JOINT_NAMES,
                zip(
                    OPENARM_LEFT_JOINT_HARD_LOWER_RAD,
                    OPENARM_LEFT_JOINT_HARD_UPPER_RAD,
                ),
            )
        },
        **{
            name: bounds
            for name, bounds in zip(
                RIGHT_JOINT_NAMES,
                zip(
                    OPENARM_RIGHT_JOINT_HARD_LOWER_RAD,
                    OPENARM_RIGHT_JOINT_HARD_UPPER_RAD,
                ),
            )
        },
    }
    assert model_ranges == expected_hard

    for hard, soft in (
        (OPENARM_LEFT_JOINT_HARD_LOWER_RAD, OPENARM_LEFT_JOINT_SOFT_LOWER_RAD),
        (OPENARM_LEFT_JOINT_HARD_UPPER_RAD, OPENARM_LEFT_JOINT_SOFT_UPPER_RAD),
        (OPENARM_RIGHT_JOINT_HARD_LOWER_RAD, OPENARM_RIGHT_JOINT_SOFT_LOWER_RAD),
        (OPENARM_RIGHT_JOINT_HARD_UPPER_RAD, OPENARM_RIGHT_JOINT_SOFT_UPPER_RAD),
    ):
        assert soft == pytest.approx(
            tuple(value * OPENARM_TELEOP_SOFT_LIMIT_RATIO for value in hard),
            abs=0.0,
        )


@pytest.mark.parametrize(
    "label",
    (
        "controller_manager",
        "pub_pose",
        "left_delta",
        "right_delta",
        "bimanual_ik",
        "command_guard",
    ),
)
def test_each_safety_critical_process_exit_requests_whole_launch_shutdown(
    launch_module, label
):
    actions = launch_module._shutdown_when_process_exits(label)(
        SimpleNamespace(returncode=0), None
    )
    assert len(actions) == 1
    assert isinstance(actions[0].event, Shutdown)


def test_runtime_safety_node_set_is_complete(launch_module):
    assert launch_module.SAFETY_CRITICAL_RUNTIME_NODES == (
        "pub_pose",
        "left_delta",
        "right_delta",
        "bimanual_ik",
        "command_guard",
    )


def test_spawner_success_is_allowed_but_failure_requests_shutdown(launch_module):
    handler = launch_module._shutdown_when_process_fails("dual-arm spawner")
    assert handler(SimpleNamespace(returncode=0), None) == []
    actions = handler(SimpleNamespace(returncode=1), None)
    assert any(
        getattr(action, "event", None).__class__ is Shutdown for action in actions
    )
