"""Host-side tests for the OpenArm ROS-independent contracts."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import math
import sys
import types

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
COMMAND_GUARD_NODE = SCRIPTS / "openarm_command_guard_node.py"
BIMANUAL_IK_NODE = SCRIPTS / "openarm_bimanual_ik_node.py"
DELTA_POSE_NODE = SCRIPTS / "pub_delta_pose_openarm_v1.py"
TELEOP_CONFIG = Path(__file__).resolve().parents[1] / "config" / "openarm_v1_teleop.yaml"
sys.path.insert(0, str(SCRIPTS))

from openarm_teleop_core import (  # noqa: E402
    ATOMIC_TARGET_DOF,
    LEFT_JOINT_NAMES,
    OPENARM_LEFT_JOINT_HARD_LOWER_RAD,
    OPENARM_LEFT_JOINT_HARD_UPPER_RAD,
    OPENARM_LEFT_JOINT_SOFT_LOWER_RAD,
    OPENARM_LEFT_JOINT_SOFT_UPPER_RAD,
    OPENARM_RIGHT_JOINT_HARD_LOWER_RAD,
    OPENARM_RIGHT_JOINT_HARD_UPPER_RAD,
    OPENARM_RIGHT_JOINT_SOFT_LOWER_RAD,
    OPENARM_RIGHT_JOINT_SOFT_UPPER_RAD,
    GRIPPER_CLOSE_VELOCITY_M_S,
    GRIPPER_OPEN_POSITION_M,
    GRIPPER_OPEN_VELOCITY_M_S,
    GripperReversalInterlock,
    MAX_EXACT_FLOAT64_INT,
    MAX_INTENT_EPOCH,
    OPENARM_TELEOP_SOFT_LIMIT_RATIO,
    RIGHT_JOINT_NAMES,
    ReleaseToRearmLatch,
    SideMode,
    TARGET_HISTORY_CAPACITY,
    TargetHistorySample,
    apply_joint_soft_limits,
    build_atomic_ik_command,
    build_atomic_target,
    build_driver_state,
    build_guard_anchor,
    build_guard_acceptance,
    decode_intent_token,
    encode_intent_token,
    gripper_target_from_normalized,
    guard_teleop_ik_mode,
    is_fresh,
    is_post_engagement,
    is_rclpy_humble_shutdown_take_race,
    map_named_positions,
    parse_atomic_ik_command,
    parse_atomic_target,
    parse_guard_anchor,
    parse_guard_acceptance,
    project_joint_positions,
    select_solver_base,
    select_solver_target,
    select_coherent_target_history_pair,
    solver_handshake_reset_required,
    shape_arm_command,
    shape_monotonic_guard_command,
    shape_gripper_command,
    shape_gripper_reversal_brake_command,
    side_control_defaults,
    split_driver_state,
    step_gripper_toward,
    step_toward,
    target_only_commit_expiry_allows_sync_wait,
    teleop_arming_within_deadline,
    teleop_targets_are_coherent,
    transition_intent_token,
    update_gripper_rearm_settle,
    update_grip_deadman,
    validate_gripper_command,
    validate_guard_acceptance_progress,
    validate_guard_anchor,
    validate_guard_command,
    validate_solver_candidate_step,
    xyzw_pose_to_openarm,
)


def test_openarm_teleop_soft_limits_are_exactly_95_percent_of_hard_limits():
    assert OPENARM_TELEOP_SOFT_LIMIT_RATIO == 0.95
    for hard, soft in (
        (OPENARM_LEFT_JOINT_HARD_LOWER_RAD, OPENARM_LEFT_JOINT_SOFT_LOWER_RAD),
        (OPENARM_LEFT_JOINT_HARD_UPPER_RAD, OPENARM_LEFT_JOINT_SOFT_UPPER_RAD),
        (OPENARM_RIGHT_JOINT_HARD_LOWER_RAD, OPENARM_RIGHT_JOINT_SOFT_LOWER_RAD),
        (OPENARM_RIGHT_JOINT_HARD_UPPER_RAD, OPENARM_RIGHT_JOINT_SOFT_UPPER_RAD),
    ):
        assert soft == tuple(value * 0.95 for value in hard)


def test_quest_button_contract_is_side_specific():
    assert side_control_defaults("left") == ("LG", "LJ", "leftTrig")
    assert side_control_defaults("right") == ("RG", "RJ", "rightTrig")
    with pytest.raises(ValueError):
        side_control_defaults("center")


def test_continuous_grip_deadman_uses_hysteresis_and_boolean_fallback():
    common = {"press_threshold": 0.55, "release_threshold": 0.35}
    assert not update_grip_deadman(
        raw_pressed=False, analog_value=0.54, previous=False, **common
    )
    assert update_grip_deadman(
        raw_pressed=False, analog_value=0.56, previous=False, **common
    )
    assert update_grip_deadman(
        raw_pressed=False, analog_value=0.40, previous=True, **common
    )
    assert not update_grip_deadman(
        raw_pressed=False, analog_value=0.34, previous=True, **common
    )
    assert update_grip_deadman(
        raw_pressed=True, analog_value=0.0, previous=False, **common
    )
    # A malformed continuous channel cannot retain a previous engagement.
    assert not update_grip_deadman(
        raw_pressed=False, analog_value=math.nan, previous=True, **common
    )
    with pytest.raises(ValueError, match="thresholds"):
        update_grip_deadman(
            raw_pressed=False,
            analog_value=0.0,
            previous=False,
            press_threshold=0.2,
            release_threshold=0.3,
        )


def test_named_feedback_is_reordered_and_requires_all_seven_axes():
    names = list(reversed(RIGHT_JOINT_NAMES))
    positions = list(range(7))
    assert map_named_positions(names, positions, RIGHT_JOINT_NAMES) == tuple(reversed(range(7)))
    with pytest.raises(ValueError, match="missing"):
        map_named_positions(names[:-1], positions[:-1], RIGHT_JOINT_NAMES)


def test_ros_xyzw_is_normalized_and_converted_to_official_wxyz():
    converted = xyzw_pose_to_openarm((1.0, 2.0, 3.0, 0.0, 0.0, 2.0, 2.0))
    assert converted[:3] == (1.0, 2.0, 3.0)
    assert converted[3:] == pytest.approx((math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)))
    with pytest.raises(ValueError, match="zero norm"):
        xyzw_pose_to_openarm((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))


def test_atomic_target_normalizes_pose_and_roundtrips_exact_stamp_and_token():
    token = encode_intent_token(7, SideMode.TELEOP)
    payload = build_atomic_target(
        (1.0, 2.0, 3.0, 0.0, 0.0, 2.0, 2.0),
        1_786_000_000,
        123_456_789,
        token,
    )
    assert len(payload) == ATOMIC_TARGET_DOF == 10
    assert payload[:3] == (1.0, 2.0, 3.0)
    assert payload[3:7] == pytest.approx(
        (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    )
    assert payload[7:] == (1_786_000_000.0, 123_456_789.0, float(token))

    official_pose, epoch_ns, parsed_token = parse_atomic_target(payload)
    assert official_pose == pytest.approx(
        (1.0, 2.0, 3.0, math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    )
    assert epoch_ns == 1_786_000_000_123_456_789
    assert parsed_token == token


def test_atomic_target_stamp_components_retain_exact_float64_boundaries():
    token = encode_intent_token(1, SideMode.TELEOP)
    payload = build_atomic_target(
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        MAX_EXACT_FLOAT64_INT,
        999_999_999,
        token,
    )
    assert int(payload[7]) == MAX_EXACT_FLOAT64_INT
    assert int(payload[8]) == 999_999_999
    _, epoch_ns, parsed_token = parse_atomic_target(payload)
    assert epoch_ns == MAX_EXACT_FLOAT64_INT * 1_000_000_000 + 999_999_999
    assert parsed_token == token


@pytest.mark.parametrize(
    ("stamp_sec", "stamp_nanosec", "error"),
    (
        (-1, 0, ValueError),
        (0, 0, ValueError),
        (1.0, 0, ValueError),
        (True, 0, ValueError),
        (0, -1, ValueError),
        (0, 1_000_000_000, ValueError),
        (0, 1.0, ValueError),
        (MAX_EXACT_FLOAT64_INT + 1, 0, OverflowError),
    ),
)
def test_atomic_target_builder_rejects_invalid_source_stamps(
    stamp_sec, stamp_nanosec, error
):
    with pytest.raises(error):
        build_atomic_target(
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            stamp_sec,
            stamp_nanosec,
            encode_intent_token(1, SideMode.TELEOP),
        )


def test_atomic_target_strictly_rejects_bad_pose_stamp_and_mode_token():
    teleop_token = encode_intent_token(1, SideMode.TELEOP)
    valid = build_atomic_target(
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), 1, 2, teleop_token
    )

    for malformed in (
        valid[:7] + (1.5,) + valid[8:],
        valid[:8] + (2.5,) + valid[9:],
        valid[:7] + (-1.0, 2.0) + valid[9:],
        valid[:7] + (0.0, 0.0) + valid[9:],
        valid[:8] + (1_000_000_000.0,) + valid[9:],
        valid[:-1] + (float(encode_intent_token(1, SideMode.HOLD)),),
        valid[:-1] + (256.5,),
        valid[:-1] + (0.0,),
    ):
        with pytest.raises(ValueError):
            parse_atomic_target(malformed)

    with pytest.raises(OverflowError):
        parse_atomic_target(valid[:7] + (float(1 << 53),) + valid[8:])
    with pytest.raises(OverflowError):
        parse_atomic_target(valid[:-1] + (float(1 << 53),))
    with pytest.raises(ValueError, match="TELEOP"):
        build_atomic_target(
            valid[:7], 1, 2, encode_intent_token(1, SideMode.HOLD)
        )
    with pytest.raises(ValueError, match="non-finite"):
        build_atomic_target(
            (math.nan, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            1,
            2,
            teleop_token,
        )
    with pytest.raises(ValueError, match="zero norm"):
        parse_atomic_target((0.0,) * 7 + valid[7:])
    with pytest.raises(ValueError, match="exactly 10"):
        parse_atomic_target(valid[:-1])


def _target_history_sample(epoch_ns, receipt_ns, tokens, pose_x=0.0):
    return TargetHistorySample(
        pose=(pose_x, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        receipt_ns=receipt_ns,
        source_epoch_ns=epoch_ns,
        intent_tokens=tokens,
    )


def test_target_history_selects_an_older_coherent_pair_when_latest_latest_skews():
    tokens = (
        encode_intent_token(7, SideMode.TELEOP),
        encode_intent_token(11, SideMode.TELEOP),
    )
    right_history = (
        _target_history_sample(100_000_000, 850_000_000, tokens, 1.0),
        _target_history_sample(200_000_000, 990_000_000, tokens, 2.0),
    )
    left_history = (
        _target_history_sample(110_000_000, 860_000_000, tokens, -1.0),
        _target_history_sample(260_000_000, 995_000_000, tokens, -2.0),
    )

    selected = select_coherent_target_history_pair(
        right_history,
        left_history,
        now_ns=1_000_000_000,
        timeout_sec=0.30,
        max_skew_ns=30_000_000,
        current_intent_tokens=tokens,
    )

    assert selected is not None
    assert (selected[0].source_epoch_ns, selected[1].source_epoch_ns) == (
        100_000_000,
        110_000_000,
    )
    assert selected[0].pose[0] == 1.0
    assert selected[1].pose[0] == -1.0


def test_target_history_selects_newest_pair_without_regressing_either_watermark():
    tokens = (
        encode_intent_token(3, SideMode.TELEOP),
        encode_intent_token(5, SideMode.TELEOP),
    )
    right_history = tuple(
        _target_history_sample(epoch, 900_000_000 + index, tokens)
        for index, epoch in enumerate((100, 200, 300))
    )
    left_history = tuple(
        _target_history_sample(epoch, 910_000_000 + index, tokens)
        for index, epoch in enumerate((110, 210, 290))
    )

    selected = select_coherent_target_history_pair(
        right_history,
        left_history,
        now_ns=1_000_000_000,
        timeout_sec=0.30,
        max_skew_ns=30,
        current_intent_tokens=tokens,
    )
    assert selected is not None
    assert (selected[0].source_epoch_ns, selected[1].source_epoch_ns) == (300, 290)

    # Equality is explicitly reusable; advancing either side's watermark past
    # the chosen epoch forbids a historical regression and leaves no pair.
    assert select_coherent_target_history_pair(
        right_history,
        left_history,
        now_ns=1_000_000_000,
        timeout_sec=0.30,
        max_skew_ns=30,
        current_intent_tokens=tokens,
        last_selected_epoch_ns=(300, 290),
    ) == selected
    assert select_coherent_target_history_pair(
        right_history,
        left_history,
        now_ns=1_000_000_000,
        timeout_sec=0.30,
        max_skew_ns=30,
        current_intent_tokens=tokens,
        last_selected_epoch_ns=(301, 290),
    ) is None


def test_target_history_filters_stale_receipts_future_receipts_and_old_token_pairs():
    current_tokens = (
        encode_intent_token(9, SideMode.TELEOP),
        encode_intent_token(13, SideMode.TELEOP),
    )
    old_tokens = (
        encode_intent_token(8, SideMode.TELEOP),
        current_tokens[1],
    )
    now_ns = 1_000_000_000
    right_history = (
        _target_history_sample(900, 600_000_000, current_tokens),
        _target_history_sample(1_000, now_ns + 1, current_tokens),
        _target_history_sample(2_000, 999_000_000, old_tokens),
    )
    left_history = (
        _target_history_sample(2_010, 999_000_000, old_tokens),
        _target_history_sample(1_010, 999_000_000, current_tokens),
    )

    assert select_coherent_target_history_pair(
        right_history,
        left_history,
        now_ns=now_ns,
        timeout_sec=0.30,
        max_skew_ns=30,
        current_intent_tokens=current_tokens,
    ) is None


@pytest.mark.parametrize(
    "kwargs",
    (
        {"now_ns": 0},
        {"now_ns": 1.0},
        {"timeout_sec": 0.0},
        {"timeout_sec": math.nan},
        {"timeout_sec": True},
        {"max_skew_ns": -1},
        {"max_skew_ns": 1.0},
        {"last_selected_epoch_ns": (0,)},
        {"last_selected_epoch_ns": (-1, 0)},
        {"last_selected_epoch_ns": (True, 0)},
    ),
)
def test_target_history_selector_rejects_invalid_parameters(kwargs):
    tokens = (
        encode_intent_token(1, SideMode.TELEOP),
        encode_intent_token(2, SideMode.TELEOP),
    )
    sample = _target_history_sample(100, 100, tokens)
    parameters = {
        "now_ns": 100,
        "timeout_sec": 0.30,
        "max_skew_ns": 30,
        "current_intent_tokens": tokens,
        "last_selected_epoch_ns": (None, None),
    }
    parameters.update(kwargs)
    with pytest.raises(ValueError):
        select_coherent_target_history_pair((sample,), (sample,), **parameters)


def test_target_history_sample_and_capacity_are_strictly_validated():
    tokens = (
        encode_intent_token(1, SideMode.TELEOP),
        encode_intent_token(2, SideMode.TELEOP),
    )
    valid = _target_history_sample(100, 100, tokens)
    assert valid.pose == (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)

    for invalid in (
        {"pose": (math.nan,) + valid.pose[1:]},
        {"receipt_ns": 0},
        {"receipt_ns": 1.0},
        {"source_epoch_ns": 0},
        {"source_epoch_ns": True},
        {"intent_tokens": (0, tokens[1])},
    ):
        values = {
            "pose": valid.pose,
            "receipt_ns": valid.receipt_ns,
            "source_epoch_ns": valid.source_epoch_ns,
            "intent_tokens": valid.intent_tokens,
        }
        values.update(invalid)
        with pytest.raises(ValueError):
            TargetHistorySample(**values)

    oversized = (valid,) * (TARGET_HISTORY_CAPACITY + 1)
    with pytest.raises(ValueError, match="fixed"):
        select_coherent_target_history_pair(
            oversized,
            (valid,),
            now_ns=100,
            timeout_sec=0.30,
            max_skew_ns=30,
            current_intent_tokens=tokens,
        )
    with pytest.raises(ValueError, match="sized"):
        select_coherent_target_history_pair(
            (entry for entry in (valid,)),
            (valid,),
            now_ns=100,
            timeout_sec=0.30,
            max_skew_ns=30,
            current_intent_tokens=tokens,
        )


def test_only_selected_coherent_target_expiry_allows_bounded_sync_wait():
    predicates = {
        "intent_tokens_current": True,
        "selected_targets_token_bound": True,
        "selected_targets_fresh": False,
        "current_target_streams_fresh": True,
        "modes_current": True,
        "feedback_fresh": True,
        "anchors_current": True,
    }
    recoverable_sync_wait = target_only_commit_expiry_allows_sync_wait(**predicates)
    assert recoverable_sync_wait
    assert (
        guard_teleop_ik_mode(True, True, False, recoverable_sync_wait, False)
        == SideMode.HOLD
    )
    assert not target_only_commit_expiry_allows_sync_wait(
        **{**predicates, "selected_targets_fresh": True}
    )


@pytest.mark.parametrize(
    "failed_prerequisite",
    (
        "intent_tokens_current",
        "selected_targets_token_bound",
        "current_target_streams_fresh",
        "modes_current",
        "feedback_fresh",
        "anchors_current",
    ),
)
def test_authority_or_live_input_loss_forces_full_commit_reset(failed_prerequisite):
    predicates = {
        "intent_tokens_current": True,
        "selected_targets_token_bound": True,
        "selected_targets_fresh": False,
        "current_target_streams_fresh": True,
        "modes_current": True,
        "feedback_fresh": True,
        "anchors_current": True,
    }
    predicates[failed_prerequisite] = False
    assert not target_only_commit_expiry_allows_sync_wait(**predicates)


def test_target_only_commit_branch_preserves_accepted_base_and_holds():
    source = BIMANUAL_IK_NODE.read_text(encoding="utf-8")
    action = source.split("def _apply_commit_gate_failure", 1)[1].split(
        "def _current_intent_tokens", 1
    )[0]
    target_action, full_action = action.split(
        "self._reset_solver_handshake()", 1
    )
    assert "self._reset_pending_handshake()" in target_action
    assert "self._publish_sync_wait(active_sides)" in target_action
    assert "self._publish_sync_wait(())" in full_action

    commit = source.split("target_only_expiry =", 1)[1].split(
        "# All active sides pass", 1
    )[0]
    target_only, full_failure = commit.split(
        "if not (", 1
    )
    assert "target_only_expiry=True" in target_only
    assert "target_only_expiry=False" in full_failure


def _load_stubbed_bimanual_ik_module(monkeypatch):
    """Load the production IK class without requiring ROS or NumPy on the host."""

    class StubNode:
        pass

    class StubMessage:
        def __init__(self, **fields):
            for name, value in fields.items():
                setattr(self, name, value)

    numpy_module = types.ModuleType("numpy")
    rclpy_module = types.ModuleType("rclpy")
    rclpy_module.__path__ = []
    executors_module = types.ModuleType("rclpy.executors")
    executors_module.ExternalShutdownException = type(
        "ExternalShutdownException", (Exception,), {}
    )
    node_module = types.ModuleType("rclpy.node")
    node_module.Node = StubNode

    stub_packages = {
        "geometry_msgs": ("PoseStamped",),
        "sensor_msgs": ("JointState",),
        "std_msgs": ("Float64MultiArray", "UInt64"),
    }
    monkeypatch.setitem(sys.modules, "numpy", numpy_module)
    monkeypatch.setitem(sys.modules, "rclpy", rclpy_module)
    monkeypatch.setitem(sys.modules, "rclpy.executors", executors_module)
    monkeypatch.setitem(sys.modules, "rclpy.node", node_module)
    for package_name, message_names in stub_packages.items():
        package = types.ModuleType(package_name)
        package.__path__ = []
        message_module = types.ModuleType(f"{package_name}.msg")
        for message_name in message_names:
            setattr(message_module, message_name, StubMessage)
        package.msg = message_module
        monkeypatch.setitem(sys.modules, package_name, package)
        monkeypatch.setitem(sys.modules, f"{package_name}.msg", message_module)

    module_name = "_openarm_bimanual_ik_commit_action_test"
    specification = importlib.util.spec_from_file_location(module_name, BIMANUAL_IK_NODE)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    monkeypatch.setitem(sys.modules, module_name, module)
    specification.loader.exec_module(module)
    return module


class _SyncWaitRecorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _stub_commit_action_node(module, *, sync_wait_active):
    node = module.OpenArmBimanualIK.__new__(module.OpenArmBimanualIK)
    node._accepted_command = {
        "right": (0.1,) * 7,
        "left": (-0.1,) * 7,
    }
    node._pending_ik = {
        "right": ((0.2,) * 7, (0.1,) * 7, (257, 257)),
        "left": ((-0.2,) * 7, (-0.1,) * 7, (257, 257)),
    }
    node._pending_acknowledged = {"right": True, "left": False}
    node._pending_started_ns = {"right": 11, "left": 12}
    node._guard_ack_fault = {"right": True, "left": True}
    node._sync_wait_active = {
        "right": sync_wait_active,
        "left": sync_wait_active,
    }
    node._last_selected_target_epoch_ns = {"right": 101, "left": 111}
    node.sync_wait_pub = {
        "right": _SyncWaitRecorder(),
        "left": _SyncWaitRecorder(),
    }
    return node


def test_target_only_commit_action_preserves_accepted_and_publishes_positive_hold(
    monkeypatch,
):
    module = _load_stubbed_bimanual_ik_module(monkeypatch)
    node = _stub_commit_action_node(module, sync_wait_active=False)
    accepted_before = dict(node._accepted_command)
    watermark_before = dict(node._last_selected_target_epoch_ns)

    node._apply_commit_gate_failure(
        target_only_expiry=True,
        active_sides=("right", "left"),
    )

    assert node._accepted_command == accepted_before
    assert node._pending_ik == {"right": None, "left": None}
    assert node._pending_acknowledged == {"right": False, "left": False}
    assert node._pending_started_ns == {"right": None, "left": None}
    assert node._guard_ack_fault == {"right": False, "left": False}
    assert node._last_selected_target_epoch_ns == watermark_before
    assert node._sync_wait_active == {"right": True, "left": True}
    for side in ("right", "left"):
        messages = node.sync_wait_pub[side].messages
        assert len(messages) == 1
        assert messages[0].data > 0


@pytest.mark.parametrize(
    "failed_prerequisite",
    (
        "intent_tokens_current",
        "selected_targets_token_bound",
        "current_target_streams_fresh",
        "modes_current",
        "feedback_fresh",
        "anchors_current",
    ),
)
def test_any_non_target_only_commit_failure_clears_accepted_and_publishes_zero(
    monkeypatch, failed_prerequisite
):
    module = _load_stubbed_bimanual_ik_module(monkeypatch)
    predicates = {
        "intent_tokens_current": True,
        "selected_targets_token_bound": True,
        "selected_targets_fresh": False,
        "current_target_streams_fresh": True,
        "modes_current": True,
        "feedback_fresh": True,
        "anchors_current": True,
    }
    predicates[failed_prerequisite] = False
    target_only_expiry = target_only_commit_expiry_allows_sync_wait(**predicates)
    assert not target_only_expiry
    node = _stub_commit_action_node(module, sync_wait_active=True)
    watermark_before = dict(node._last_selected_target_epoch_ns)

    node._apply_commit_gate_failure(
        target_only_expiry=target_only_expiry,
        active_sides=("right", "left"),
    )

    assert node._accepted_command == {"right": None, "left": None}
    assert node._pending_ik == {"right": None, "left": None}
    assert node._pending_acknowledged == {"right": False, "left": False}
    assert node._pending_started_ns == {"right": None, "left": None}
    assert node._guard_ack_fault == {"right": False, "left": False}
    assert node._last_selected_target_epoch_ns == watermark_before
    assert node._sync_wait_active == {"right": False, "left": False}
    for side in ("right", "left"):
        messages = node.sync_wait_pub[side].messages
        assert len(messages) == 1
        assert messages[0].data == 0


def test_official_driver_contract_is_right8_then_left8():
    state = build_driver_state(range(7), 70.0, range(10, 17), 170.0)
    assert state == tuple(range(7)) + (70.0,) + tuple(range(10, 17)) + (170.0,)
    right, left = split_driver_state(state)
    assert right == tuple(range(7)) + (70.0,)
    assert left == tuple(range(10, 17)) + (170.0,)


def test_intent_token_roundtrips_at_the_exact_float64_boundary():
    for mode in SideMode:
        token = encode_intent_token(MAX_INTENT_EPOCH, mode)
        assert token == (MAX_INTENT_EPOCH << 8) | int(mode)
        assert 0 < token <= MAX_EXACT_FLOAT64_INT
        assert int(float(token)) == token
        assert decode_intent_token(token) == (MAX_INTENT_EPOCH, mode)


@pytest.mark.parametrize("epoch", (0, -1, 1.0, True))
def test_intent_token_rejects_invalid_epochs(epoch):
    with pytest.raises(ValueError):
        encode_intent_token(epoch, SideMode.HOLD)


def test_intent_token_epoch_overflow_fails_closed():
    with pytest.raises(OverflowError):
        encode_intent_token(MAX_INTENT_EPOCH + 1, SideMode.HOLD)
    terminal = encode_intent_token(MAX_INTENT_EPOCH, SideMode.HOLD)
    with pytest.raises(OverflowError):
        transition_intent_token(terminal, SideMode.TELEOP)


@pytest.mark.parametrize("mode", (-1, 4, 255, 1.0, True))
def test_intent_token_rejects_invalid_modes(mode):
    with pytest.raises(ValueError):
        encode_intent_token(1, mode)


@pytest.mark.parametrize(
    "token",
    (
        0,
        -1,
        1.5,
        True,
        (1 << 8) | 4,
    ),
)
def test_intent_token_decode_rejects_zero_fractional_or_illegal_mode(token):
    with pytest.raises(ValueError):
        decode_intent_token(token)
    with pytest.raises(OverflowError):
        decode_intent_token(MAX_EXACT_FLOAT64_INT + 1)


def test_intent_epoch_advances_only_on_teleop_boundaries():
    hold_1 = encode_intent_token(1, SideMode.HOLD)
    assert transition_intent_token(hold_1, SideMode.HOLD) == hold_1

    fault_1 = transition_intent_token(hold_1, SideMode.FAULT)
    assert decode_intent_token(fault_1) == (1, SideMode.FAULT)

    teleop_2 = transition_intent_token(fault_1, SideMode.TELEOP)
    assert decode_intent_token(teleop_2) == (2, SideMode.TELEOP)
    assert transition_intent_token(teleop_2, SideMode.TELEOP) == teleop_2

    # Even if a subscriber misses this HOLD publication, the publisher's local
    # token advances across both edges; the next TELEOP cannot equal the old one.
    missed_hold_3 = transition_intent_token(teleop_2, SideMode.HOLD)
    teleop_4 = transition_intent_token(missed_hold_3, SideMode.TELEOP)
    assert decode_intent_token(missed_hold_3) == (3, SideMode.HOLD)
    assert decode_intent_token(teleop_4) == (4, SideMode.TELEOP)
    assert teleop_4 != teleop_2


def test_token_bound_payloads_roundtrip_without_touching_joint_values():
    candidate = tuple(index / 100.0 for index in range(7))
    solver_base = tuple(-index / 100.0 for index in range(7))
    accepted = tuple(index / 200.0 for index in range(7))
    command = tuple(index / 300.0 for index in range(7))
    tokens = (
        encode_intent_token(17, SideMode.TELEOP),
        encode_intent_token(23, SideMode.HOLD),
    )

    anchor_payload = build_guard_anchor(command, tokens)
    assert len(anchor_payload) == 9
    assert anchor_payload[:7] == command
    assert anchor_payload[-2:] == tuple(float(token) for token in tokens)
    assert parse_guard_anchor(anchor_payload) == (command, tokens)

    ik_payload = build_atomic_ik_command(candidate, solver_base, tokens)
    assert len(ik_payload) == 16
    assert ik_payload[:14] == candidate + solver_base
    assert ik_payload[-2:] == tuple(float(token) for token in tokens)
    assert parse_atomic_ik_command(ik_payload) == (candidate, solver_base, tokens)

    acceptance_payload = build_guard_acceptance(
        accepted, candidate, solver_base, tokens
    )
    assert len(acceptance_payload) == 23
    assert acceptance_payload[:21] == accepted + candidate + solver_base
    assert acceptance_payload[-2:] == tuple(float(token) for token in tokens)
    assert parse_guard_acceptance(acceptance_payload) == (
        accepted,
        candidate,
        solver_base,
        tokens,
    )


def test_token_bound_payloads_reject_wrong_lengths_and_nonfinite_joint_data():
    tokens = (
        encode_intent_token(1, SideMode.TELEOP),
        encode_intent_token(1, SideMode.HOLD),
    )
    candidate = tuple(index / 100.0 for index in range(7))
    solver_base = tuple(-index / 100.0 for index in range(7))
    ik_payload = build_atomic_ik_command(candidate, solver_base, tokens)

    with pytest.raises(ValueError, match="exactly 16"):
        parse_atomic_ik_command(candidate)
    with pytest.raises(ValueError, match="non-finite"):
        parse_atomic_ik_command(
            ik_payload[:13] + (math.nan,) + ik_payload[14:]
        )

    command = (0.0,) * 7
    accepted = (0.0,) * 7
    for payload, parser in (
        (build_guard_anchor(command, tokens), parse_guard_anchor),
        (ik_payload, parse_atomic_ik_command),
        (
            build_guard_acceptance(accepted, candidate, solver_base, tokens),
            parse_guard_acceptance,
        ),
    ):
        with pytest.raises(ValueError, match="exactly"):
            parser(payload[:-1])

    with pytest.raises(ValueError, match="integer"):
        build_guard_anchor(command, (float(tokens[0]), tokens[1]))


@pytest.mark.parametrize(
    "invalid_token",
    (
        256.5,
        float(1 << 53),
        float((1 << 8) | 4),
        0.0,
    ),
)
def test_all_float64_payload_parsers_strictly_reject_invalid_tokens(invalid_token):
    tokens = (
        encode_intent_token(1, SideMode.TELEOP),
        encode_intent_token(1, SideMode.HOLD),
    )
    command = tuple(index / 300.0 for index in range(7))
    candidate = tuple(index / 100.0 for index in range(7))
    solver_base = tuple(-index / 100.0 for index in range(7))
    accepted = tuple(index / 200.0 for index in range(7))
    payloads_and_parsers = (
        (build_guard_anchor(command, tokens), parse_guard_anchor),
        (
            build_atomic_ik_command(candidate, solver_base, tokens),
            parse_atomic_ik_command,
        ),
        (
            build_guard_acceptance(accepted, candidate, solver_base, tokens),
            parse_guard_acceptance,
        ),
    )
    for payload, parser in payloads_and_parsers:
        malformed = payload[:-1] + (invalid_token,)
        with pytest.raises((ValueError, OverflowError)):
            parser(malformed)


def test_delta_mode_publisher_uses_one_persistent_uint64_intent_token():
    source = DELTA_POSE_NODE.read_text(encoding="utf-8")
    assert "Float64MultiArray" in source
    assert "UInt64" in source
    assert "UInt8" not in source
    assert "self._intent_token = encode_intent_token(1, self._last_mode)" in source
    publish_mode = source.split("def _publish_mode", 1)[1].split("def _fail", 1)[0]
    transition = publish_mode.index(
        "self._intent_token = transition_intent_token(self._intent_token, mode)"
    )
    publish = publish_mode.index(
        "self.mode_pub.publish(UInt64(data=self._intent_token))"
    )
    assert transition < publish


def test_delta_publishes_mode_before_same_stamp_atomic_target_and_keeps_debug_pose():
    source = DELTA_POSE_NODE.read_text(encoding="utf-8")
    assert '"target_intent_topic", f"/openarm/target_intent/{self.side}"' in source
    assert "self.target_intent_pub = self.create_publisher(" in source
    teleop_branch = source.split("if hold_pressed:", 1)[1].split(
        "if self._teleop_active:", 1
    )[0]
    mode_publish = teleop_branch.index("self._publish_mode(SideMode.TELEOP)")
    atomic_build = teleop_branch.index("atomic_target = build_atomic_target(")
    debug_publish = teleop_branch.index("self.target_pub.publish(target)")
    atomic_publish = teleop_branch.index("self.target_intent_pub.publish(")
    assert mode_publish < atomic_build < debug_publish < atomic_publish
    assert teleop_branch.count("self._publish_mode(SideMode.TELEOP)") == 1
    assert "target.header.stamp.sec" in teleop_branch
    assert "target.header.stamp.nanosec" in teleop_branch
    assert "self._intent_token" in teleop_branch

    config = TELEOP_CONFIG.read_text(encoding="utf-8")
    assert config.count("/openarm/target_intent/left") == 2
    assert config.count("/openarm/target_intent/right") == 2
    assert "/openarm/target_pose/left" in config
    assert "/openarm/target_pose/right" in config


def test_stale_data_fails_closed():
    now = 2_000_000_000
    assert is_fresh(now, 1_900_000_000, 0.2)
    assert not is_fresh(now, 1_700_000_000, 0.2)
    assert not is_fresh(now, None, 0.2)
    assert not is_fresh(now, now + 1, 0.2)


def test_deadman_engagement_rejects_previous_ik_sample():
    assert not is_post_engagement(999, 1_000)
    assert is_post_engagement(1_000, 1_000)
    assert is_post_engagement(1_001, 1_000)
    assert not is_post_engagement(None, 1_000)


@pytest.mark.parametrize(
    ("elapsed_ns", "expected"),
    (
        (749_999_999, True),
        (750_000_000, True),
        (750_000_001, False),
    ),
)
def test_teleop_arming_deadline_is_inclusive(elapsed_ns, expected):
    engage_ns = 1_000_000_000
    assert (
        teleop_arming_within_deadline(
            engage_ns + elapsed_ns,
            engage_ns,
            0.75,
        )
        is expected
    )


@pytest.mark.parametrize("engagement_ns", (None, 0, 1_000_000_001))
def test_teleop_arming_deadline_rejects_missing_or_future_grip_edge(engagement_ns):
    assert not teleop_arming_within_deadline(1_000_000_000, engagement_ns, 0.75)


def test_guard_waits_only_within_deadline_for_first_post_engagement_ik():
    # Before this engagement has committed an IK command, asynchronous mode,
    # anchor, and IK callbacks may arrive in either order.  The arm must stay
    # in HOLD only while the bounded arming deadline remains live.
    assert guard_teleop_ik_mode(False, False, False, False, True) == SideMode.HOLD
    assert guard_teleop_ik_mode(False, True, False, False, True) == SideMode.HOLD
    assert guard_teleop_ik_mode(False, True, True, False, True) == SideMode.TELEOP

    # Even a fresh late sample cannot silently establish motion after the
    # deadline.  FAULT then requires the existing release/re-arm sequence.
    assert guard_teleop_ik_mode(False, False, False, False, False) == SideMode.FAULT
    assert guard_teleop_ik_mode(False, True, True, False, False) == SideMode.FAULT

    # Once TELEOP is established, the same missing/stale IK condition is a
    # real runtime outage unless the live IK node explicitly reports that it
    # is safely synchronizing with the peer side.
    assert guard_teleop_ik_mode(True, False, True, False, False) == SideMode.FAULT
    assert guard_teleop_ik_mode(True, True, False, False, False) == SideMode.FAULT
    assert guard_teleop_ik_mode(True, True, True, False, False) == SideMode.TELEOP


@pytest.mark.parametrize("teleop_established", (False, True))
@pytest.mark.parametrize("ik_after_engagement", (False, True))
@pytest.mark.parametrize("ik_fresh", (False, True))
@pytest.mark.parametrize("arming_within_deadline", (False, True))
def test_valid_sync_wait_has_absolute_priority_over_ik_and_arming_state(
    teleop_established,
    ik_after_engagement,
    ik_fresh,
    arming_within_deadline,
):
    assert (
        guard_teleop_ik_mode(
            teleop_established,
            ik_after_engagement,
            ik_fresh,
            True,
            arming_within_deadline,
        )
        == SideMode.HOLD
    )


def test_fresh_ik_resumes_after_its_sync_wait_lease_is_cleared():
    established_fresh_ik = (True, True, True)
    assert (
        guard_teleop_ik_mode(*established_fresh_ik, True, False)
        == SideMode.HOLD
    )
    # This is the pure decision transition exercised after _ik_callback has
    # cleared the current sync-wait lease.
    assert (
        guard_teleop_ik_mode(*established_fresh_ik, False, False)
        == SideMode.TELEOP
    )


def test_sync_wait_budget_resets_only_after_a_successful_guard_acceptance():
    source = COMMAND_GUARD_NODE.read_text(encoding="utf-8")
    ik_callback = source.split("def _ik_callback", 1)[1].split(
        "def _ik_sync_wait_callback", 1
    )[0]
    assert "self.ik_sync_wait_ns[side] = None" in ik_callback
    assert "self.ik_sync_wait_generated_ns[side] = None" in ik_callback
    assert "self.ik_sync_wait_started_ns[side] = None" not in ik_callback

    sync_callback = source.split("def _ik_sync_wait_callback", 1)[1].split(
        "def _gripper_intent_callback", 1
    )[0]
    invalid_lease = sync_callback.split("if not valid_current_lease:", 1)[1].split(
        "if self.ik_sync_wait_started_ns[side] is None:", 1
    )[0]
    assert "self.ik_sync_wait_ns[side] = None" in invalid_lease
    assert "self.ik_sync_wait_generated_ns[side] = None" in invalid_lease
    assert "self.ik_sync_wait_started_ns[side] = None" not in invalid_lease
    assert sync_callback.count(
        "self.ik_sync_wait_started_ns[side] = now_ns"
    ) == 1

    commit = source.split(
        "# Commit only after both independently prepared commands have passed.", 1
    )[1].split("self.gripper_command_pub[side].publish(", 1)[0]
    new_command_acceptance, cached_retry_acceptance = commit.split(
        "elif retry_acceptance is not None:", 1
    )
    for successful_acceptance in (
        new_command_acceptance,
        cached_retry_acceptance,
    ):
        acceptance_publish = successful_acceptance.index(
            "self.guard_acceptance_pub[side].publish("
        )
        for state in (
            "self.ik_sync_wait_ns[side] = None",
            "self.ik_sync_wait_generated_ns[side] = None",
            "self.ik_sync_wait_started_ns[side] = None",
        ):
            assert successful_acceptance.index(state) > acceptance_publish


def test_sync_wait_solver_handoff_has_no_cross_topic_zero_window():
    source = BIMANUAL_IK_NODE.read_text(encoding="utf-8")
    solve_once = source.split("def _solve_once", 1)[1].split("\ndef main", 1)[0]

    # Recovery from a coherent wait enters the solver without sending zero or
    # another True heartbeat. The last positive lease remains bounded by the
    # guard's unchanged IK timeout.
    handoff_to_solver = solve_once.split(
        "# Preserve the last synchronization heartbeat", 1
    )[1].split("right_solver_joints = select_solver_base(", 1)[0]
    assert "_publish_sync_wait(" not in handoff_to_solver

    local_handoff = source.split(
        "def _handoff_sync_wait_to_pending_ik", 1
    )[1].split("def _solve_once", 1)[0]
    assert 'for side in ("right", "left"):' in local_handoff
    assert "self._sync_wait_active[side] = False" in local_handoff
    assert ".publish(" not in local_handoff
    assert "_publish_sync_wait(" not in local_handoff

    candidate_validation = solve_once.index(
        "bounded_decision = validate_solver_candidate_step("
    )
    local_handoff_call = solve_once.index(
        "self._handoff_sync_wait_to_pending_ik()"
    )
    pending_publish = solve_once.rindex("self._publish_pending_ik(side)")
    assert candidate_validation < local_handoff_call < pending_publish

    pending_path = solve_once.split("if pending_sides:", 1)[1].split(
        "# A fresh Grip engagement", 1
    )[0]
    assert "_publish_sync_wait(active_sides)" not in pending_path


def test_every_handled_solver_failure_explicitly_withdraws_sync_wait():
    source = BIMANUAL_IK_NODE.read_text(encoding="utf-8")
    solve_once = source.split("def _solve_once", 1)[1].split("\ndef main", 1)[0]

    failure_blocks = (
        solve_once.split("if not self.kinematics.ready():", 1)[1].split(
            "result = self.kinematics.solve()", 1
        )[0],
        solve_once.split("except (RuntimeError, ValueError) as exc:", 1)[1].split(
            "if result is None:", 1
        )[0],
        solve_once.split("if result is None:", 1)[1].split("try:", 1)[0],
        solve_once.split("except (TypeError, ValueError) as exc:", 1)[1].split(
            'results = {"right": right_result', 1
        )[0],
        solve_once.split(
            "if not decision.accepted or decision.command is None:", 1
        )[1].split("try:", 1)[0],
        solve_once.split("except (TypeError, ValueError) as exc:", 2)[2].split(
            "bounded_decision = validate_solver_candidate_step(", 1
        )[0],
        solve_once.split(
            "if not bounded_decision.accepted or bounded_decision.command is None:",
            1,
        )[1].split("candidate = bounded_decision.command", 1)[0],
    )
    for failure_block in failure_blocks:
        assert "self._publish_sync_wait(())" in failure_block


def test_only_the_pinned_humble_post_shutdown_take_race_is_suppressed():
    observed = RuntimeError(
        "Unable to convert call argument to Python object "
        "(compile in debug mode for details)"
    )
    real_failure = RuntimeError("real callback failure")

    assert is_rclpy_humble_shutdown_take_race(observed, context_ok=False)
    assert not is_rclpy_humble_shutdown_take_race(observed, context_ok=True)
    assert not is_rclpy_humble_shutdown_take_race(real_failure, context_ok=False)
    assert not is_rclpy_humble_shutdown_take_race(real_failure, context_ok=True)


def test_non_teleop_side_uses_current_fk_target_each_solver_cycle():
    target = (1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0)
    hold = (4.0, 5.0, 6.0, 1.0, 0.0, 0.0, 0.0)
    assert select_solver_target(SideMode.TELEOP, target, hold) == target
    for mode in (SideMode.HOLD, SideMode.HOME, SideMode.FAULT):
        assert select_solver_target(mode, None, hold) == hold


def test_solver_advances_only_from_the_guard_accepted_command():
    feedback = (0.0,) * 7
    first_base = select_solver_base(
        mode=SideMode.TELEOP,
        feedback=feedback,
        accepted_command=None,
    )
    assert first_base == feedback

    # The official candidate itself is never the next base. If guard
    # backpressure accepted only 0.001, the solver advances from exactly 0.001.
    guard_accepted = (0.001,) * 7
    second_base = select_solver_base(
        mode=SideMode.TELEOP,
        feedback=feedback,
        accepted_command=guard_accepted,
    )
    assert second_base == guard_accepted

    # A new TELEOP engagement must start from the final guard's retained
    # controller command, not from lagging measured feedback.  This is the
    # exact condition that previously produced non-monotonic/unsafe ACKs.
    retained_controller_command = (0.25,) * 7
    reengagement_base = select_solver_base(
        mode=SideMode.TELEOP,
        feedback=feedback,
        accepted_command=None,
        controller_anchor=retained_controller_command,
    )
    assert reengagement_base == retained_controller_command

    # Any non-TELEOP cycle ignores an old acceptance and uses measurement.
    assert select_solver_base(
        mode=SideMode.HOLD,
        feedback=feedback,
        accepted_command=guard_accepted,
    ) == feedback


def test_fast_release_and_repress_invalidates_solver_handshake_between_ticks():
    # Both raw mode callbacks may run before the next IK timer callback.  The
    # release edge must discard the old pending/accepted handshake immediately.
    accepted = (0.25,) * 7
    mode = SideMode.TELEOP
    new_mode = SideMode.HOLD
    if solver_handshake_reset_required(mode, new_mode):
        accepted = None
    mode = new_mode
    assert accepted is None

    new_mode = SideMode.TELEOP
    assert solver_handshake_reset_required(mode, new_mode)
    assert not solver_handshake_reset_required(new_mode, SideMode.TELEOP)


def test_guard_and_ik_share_the_same_monotonic_acceptance_contract():
    max_step = (0.01413,) * 7
    feedback_base = (0.437743187,) + (0.0,) * 6
    retained_command = (0.476835936,) + (0.0,) * 6
    stale_base_candidate = (0.451873187,) + (0.0,) * 6

    # Reproduce the real log exactly: shaping from the retained command while
    # solving from feedback yields 0.462705936, which is neither one solver
    # step from feedback nor on feedback->candidate.
    poisoned_acceptance = shape_arm_command(
        current_command=retained_command,
        target=stale_base_candidate,
        feedback=feedback_base,
        max_step=max_step,
        max_tracking_error=(0.12,) * 7,
    )
    assert poisoned_acceptance[0] == pytest.approx(0.462705936)
    rejected = validate_guard_acceptance_progress(
        accepted_command=poisoned_acceptance,
        candidate=stale_base_candidate,
        solver_base=feedback_base,
        max_step=max_step,
    )
    assert not rejected.accepted
    assert "solver step" in rejected.reason

    # With a post-engagement guard anchor, the same safe controller step is a
    # valid monotonic step from the exact retained controller command.
    anchored_candidate = (retained_command[0] - max_step[0],) + (0.0,) * 6
    accepted = shape_arm_command(
        current_command=retained_command,
        target=anchored_candidate,
        feedback=feedback_base,
        max_step=max_step,
        max_tracking_error=(0.12,) * 7,
    )
    approved = validate_guard_acceptance_progress(
        accepted_command=accepted,
        candidate=anchored_candidate,
        solver_base=retained_command,
        max_step=max_step,
    )
    assert approved.accepted
    assert approved.command == pytest.approx(anchored_candidate)


def test_guard_tracking_backpressure_holds_on_the_solver_segment():
    max_step = (0.01413,) * 7
    feedback = (0.0,) * 7
    solver_base = (0.119,) + (0.0,) * 6
    outward_candidate = (solver_base[0] + max_step[0],) + (0.0,) * 6

    # The generic tracking shaper is allowed to pull a lagging position command
    # back toward feedback.  An IK acknowledgement cannot report that recovery
    # as progress because it lies behind the exact solver base.
    generic = shape_arm_command(
        current_command=solver_base,
        target=outward_candidate,
        feedback=feedback,
        max_step=max_step,
        max_tracking_error=(0.12,) * 7,
    )
    assert generic[0] < solver_base[0]

    # The final guard instead holds the already-issued controller command until
    # feedback catches up.  The ACK remains a zero-progress point on the
    # solver_base->candidate segment and can be consumed by IK safely.
    guarded = shape_monotonic_guard_command(
        current_command=solver_base,
        target=outward_candidate,
        feedback=feedback,
        max_step=max_step,
        max_tracking_error=(0.12,) * 7,
    )
    assert guarded == pytest.approx(solver_base)
    assert validate_guard_acceptance_progress(
        accepted_command=guarded,
        candidate=outward_candidate,
        solver_base=solver_base,
        max_step=max_step,
    ).accepted

    # The controller retains float64 while the official solver consumes
    # float32.  Project against the exact base carried in the IK payload so a
    # few nanoradians of representation difference cannot poison the ACK.
    retained_float64 = (solver_base[0] + 3e-9,) + (0.0,) * 6
    canonical_hold = shape_monotonic_guard_command(
        current_command=retained_float64,
        monotonic_base=solver_base,
        target=outward_candidate,
        feedback=feedback,
        max_step=max_step,
        max_tracking_error=(0.12,) * 7,
    )
    assert canonical_hold == pytest.approx(solver_base, abs=1e-15)
    assert validate_guard_acceptance_progress(
        accepted_command=canonical_hold,
        candidate=outward_candidate,
        solver_base=solver_base,
        max_step=max_step,
    ).accepted


def test_guard_anchor_is_hard_bounded_and_tracking_bounded():
    feedback = (0.0,) * 7
    lower = (-1.0,) * 7
    upper = (1.0,) * 7
    tracking = (0.12,) * 7

    accepted = validate_guard_anchor(
        anchor=(0.10,) * 7,
        feedback=feedback,
        lower_limits=lower,
        upper_limits=upper,
        max_tracking_error=tracking,
    )
    assert accepted.accepted
    assert accepted.command == pytest.approx((0.10,) * 7)

    assert not validate_guard_anchor(
        anchor=(0.13,) + (0.0,) * 6,
        feedback=feedback,
        lower_limits=lower,
        upper_limits=upper,
        max_tracking_error=tracking,
    ).accepted
    assert not validate_guard_anchor(
        anchor=(1.01,) + (0.0,) * 6,
        feedback=feedback,
        lower_limits=lower,
        upper_limits=upper,
        max_tracking_error=tracking,
    ).accepted


def test_dual_teleop_requires_bounded_nonzero_quest_epochs_but_single_does_not():
    assert teleop_targets_are_coherent(SideMode.TELEOP, SideMode.HOLD, 10, None)
    assert not teleop_targets_are_coherent(SideMode.TELEOP, SideMode.TELEOP, 10, 11)
    assert not teleop_targets_are_coherent(SideMode.TELEOP, SideMode.TELEOP, None, None)
    assert teleop_targets_are_coherent(SideMode.TELEOP, SideMode.TELEOP, 10, 10)
    assert teleop_targets_are_coherent(
        SideMode.TELEOP,
        SideMode.TELEOP,
        1_000_000_000,
        1_020_000_000,
        30_000_000,
    )
    assert not teleop_targets_are_coherent(
        SideMode.TELEOP,
        SideMode.TELEOP,
        1_000_000_000,
        1_040_000_000,
        30_000_000,
    )


def test_fault_latch_requires_healthy_continuous_release_before_rearm():
    latch = ReleaseToRearmLatch(0.5)
    latch.trip()
    assert latch.update(1_000_000_000, controls_released=False, inputs_ready=True)
    assert latch.update(2_000_000_000, controls_released=True, inputs_ready=True)
    # Missing input resets the release interval.
    assert latch.update(2_400_000_000, controls_released=True, inputs_ready=False)
    assert latch.update(3_000_000_000, controls_released=True, inputs_ready=True)
    assert latch.update(3_499_999_999, controls_released=True, inputs_ready=True)
    assert not latch.update(3_500_000_000, controls_released=True, inputs_ready=True)
    assert not latch.latched


def test_home_step_is_componentwise_velocity_bounded():
    stepped = step_toward([0.0] * 7, [1.0, -1.0, 0.1, 0.0, 2.0, -2.0, 3.0], [0.2] * 7)
    assert stepped == pytest.approx((0.2, -0.2, 0.1, 0.0, 0.2, -0.2, 0.2))


def test_arm_tracking_backpressure_reserves_one_full_guard_step():
    step = [0.004] * 7
    tracking_limit = 0.12
    soft_limit = tracking_limit - step[0]

    shaped = shape_arm_command(
        current_command=[0.119] * 7,
        target=[1.0] * 7,
        feedback=[0.0] * 7,
        max_step=step,
        max_tracking_error=tracking_limit,
    )
    assert shaped == pytest.approx([soft_limit] * 7)

    resumed = shape_arm_command(
        current_command=[soft_limit] * 7,
        target=[1.0] * 7,
        feedback=[0.01] * 7,
        max_step=step,
        max_tracking_error=tracking_limit,
    )
    assert resumed == pytest.approx([0.12] * 7)

    recovering = shape_arm_command(
        current_command=[soft_limit] * 7,
        target=[0.0] * 7,
        feedback=[0.0] * 7,
        max_step=step,
        max_tracking_error=tracking_limit,
    )
    assert recovering == pytest.approx([soft_limit - step[0]] * 7)

    with pytest.raises(ValueError, match="tracking hard gate"):
        shape_arm_command(
            current_command=[tracking_limit + 1e-6] * 7,
            target=[1.0] * 7,
            feedback=[0.0] * 7,
            max_step=step,
            max_tracking_error=tracking_limit,
        )


def test_arm_tracking_backpressure_accepts_per_axis_hard_gates():
    steps = [0.01413, 0.01413, 0.02826, 0.02826, 0.1134, 0.1134, 0.1134]
    tracking = [0.12, 0.12, 0.12, 0.12, 0.24, 0.24, 0.24]

    shaped = shape_arm_command(
        current_command=[0.0] * 7,
        target=steps,
        feedback=[0.0] * 7,
        max_step=steps,
        max_tracking_error=tracking,
    )

    assert shaped == pytest.approx(steps)


@pytest.mark.parametrize(
    ("trigger", "expected_position_m"),
    (
        (0.00, 0.04000),
        (0.25, 0.03000),
        (0.50, 0.02000),
        (0.75, 0.01000),
        (1.00, 0.00000),
    ),
)
def test_trigger_travel_maps_linearly_to_runtime_gripper_stroke(
    trigger: float,
    expected_position_m: float,
):
    assert GRIPPER_OPEN_POSITION_M == pytest.approx(0.040)
    normalized_opening = 1.0 - trigger
    assert gripper_target_from_normalized(
        normalized_opening,
        0.0,
        GRIPPER_OPEN_POSITION_M,
    ) == pytest.approx(expected_position_m)


def test_gripper_intent_rejects_out_of_range_values_and_preserves_directional_rate_limits():
    with pytest.raises(ValueError, match=r"\[0, 1\]"):
        gripper_target_from_normalized(1.01)

    assert GRIPPER_OPEN_VELOCITY_M_S == pytest.approx(0.5000)
    assert GRIPPER_CLOSE_VELOCITY_M_S == pytest.approx(0.5000)
    open_step = GRIPPER_OPEN_VELOCITY_M_S / 100.0
    close_step = GRIPPER_CLOSE_VELOCITY_M_S / 100.0
    assert step_gripper_toward(
        0.0, 0.040, open_step, close_step
    ) == pytest.approx(open_step)
    assert step_gripper_toward(
        0.040, 0.0, open_step, close_step
    ) == pytest.approx(
        0.040 - close_step
    )


def test_real_gripper_rearm_state_can_retreat_to_conservative_open_endpoint():
    open_step = GRIPPER_OPEN_VELOCITY_M_S / 100.0
    close_step = GRIPPER_CLOSE_VELOCITY_M_S / 100.0
    feedback = 0.041
    command = 0.0435
    for _ in range(100):
        command = shape_gripper_command(
            current_command=command,
            target=GRIPPER_OPEN_POSITION_M,
            feedback=feedback,
            max_open_step=open_step,
            max_close_step=close_step,
            max_tracking_error=0.00504,
        )
        # Model a real plant that remains slightly beyond the commissioned
        # runtime-open endpoint. It is still inside the unchanged 0..0.044 hard
        # feedback range and must not prevent Trigger arming.
        if command == pytest.approx(GRIPPER_OPEN_POSITION_M, abs=1e-12):
            break
    assert command == pytest.approx(GRIPPER_OPEN_POSITION_M)
    assert feedback >= GRIPPER_OPEN_POSITION_M - 0.0025


def test_gripper_rearm_endpoint_requires_one_continuous_settle_window():
    settled, started_ns = update_gripper_rearm_settle(
        now_ns=1_000_000_000,
        endpoint_ready=True,
        settle_started_ns=None,
        settle_sec=0.25,
    )
    assert not settled
    assert started_ns == 1_000_000_000

    settled, started_ns = update_gripper_rearm_settle(
        now_ns=1_249_999_999,
        endpoint_ready=True,
        settle_started_ns=started_ns,
        settle_sec=0.25,
    )
    assert not settled
    assert started_ns == 1_000_000_000

    settled, started_ns = update_gripper_rearm_settle(
        now_ns=1_250_000_000,
        endpoint_ready=True,
        settle_started_ns=started_ns,
        settle_sec=0.25,
    )
    assert settled
    assert started_ns == 1_000_000_000

    settled, started_ns = update_gripper_rearm_settle(
        now_ns=1_260_000_000,
        endpoint_ready=False,
        settle_started_ns=started_ns,
        settle_sec=0.25,
    )
    assert not settled
    assert started_ns is None

    # An endpoint wobble before the deadline must discard all previously
    # accumulated ready time.  A new ready sample starts a new complete 250 ms
    # window; the two windows may never be added together.
    settled, started_ns = update_gripper_rearm_settle(
        now_ns=2_000_000_000,
        endpoint_ready=True,
        settle_started_ns=None,
        settle_sec=0.25,
    )
    assert not settled
    assert started_ns == 2_000_000_000

    settled, started_ns = update_gripper_rearm_settle(
        now_ns=2_100_000_000,
        endpoint_ready=True,
        settle_started_ns=started_ns,
        settle_sec=0.25,
    )
    assert not settled
    assert started_ns == 2_000_000_000

    settled, started_ns = update_gripper_rearm_settle(
        now_ns=2_100_000_001,
        endpoint_ready=False,
        settle_started_ns=started_ns,
        settle_sec=0.25,
    )
    assert not settled
    assert started_ns is None

    settled, started_ns = update_gripper_rearm_settle(
        now_ns=2_200_000_000,
        endpoint_ready=True,
        settle_started_ns=started_ns,
        settle_sec=0.25,
    )
    assert not settled
    assert started_ns == 2_200_000_000

    settled, started_ns = update_gripper_rearm_settle(
        now_ns=2_449_999_999,
        endpoint_ready=True,
        settle_started_ns=started_ns,
        settle_sec=0.25,
    )
    assert not settled
    assert started_ns == 2_200_000_000

    settled, started_ns = update_gripper_rearm_settle(
        now_ns=2_450_000_000,
        endpoint_ready=True,
        settle_started_ns=started_ns,
        settle_sec=0.25,
    )
    assert settled
    assert started_ns == 2_200_000_000

    with pytest.raises(ValueError, match="duration"):
        update_gripper_rearm_settle(
            now_ns=1,
            endpoint_ready=True,
            settle_started_ns=None,
            settle_sec=0.0,
        )


def test_only_numeric_ik_limit_residue_is_projected_onto_the_work_area():
    lower = [-1.0] * 7
    upper = [1.0] * 7
    projected = project_joint_positions(
        [-1.0 - 5e-7, -1.0, -0.5, 0.0, 0.5, 1.0, 1.0 + 5e-7],
        lower,
        upper,
    )
    assert projected == pytest.approx([-1.0, -1.0, -0.5, 0.0, 0.5, 1.0, 1.0])

    with pytest.raises(ValueError, match="exceeds configured work area"):
        project_joint_positions([-1.2] + [0.0] * 6, lower, upper)

    with pytest.raises(ValueError, match="lower joint limit"):
        project_joint_positions([0.0] * 7, [0.0] * 7, [0.0] * 7)
    with pytest.raises(ValueError, match="finite"):
        project_joint_positions([math.nan] + [0.0] * 6, lower, upper)


def test_joint_soft_limits_saturate_candidates_when_reference_is_inside():
    lower = [-1.0] * 7
    upper = [1.0] * 7
    reference = [0.0] * 7
    candidate = [-1.2, -1.0, -0.5, 0.0, 0.5, 1.0, 1.2]

    assert apply_joint_soft_limits(
        candidate, reference, lower, upper
    ) == pytest.approx((-1.0, -1.0, -0.5, 0.0, 0.5, 1.0, 1.0))


def test_joint_soft_limits_allow_velocity_bounded_recovery_without_boundary_jump():
    lower = [-1.0] * 7
    upper = [1.0] * 7
    reference = [-1.10, -1.10, 1.10, 1.10, -1.10, 1.10, 0.0]
    candidate = [-1.20, -1.05, 1.20, 1.05, 1.20, -1.20, 0.25]

    bounded = apply_joint_soft_limits(candidate, reference, lower, upper)

    # Further outward motion is held at the current legal hard-range position.
    assert bounded[0] == pytest.approx(reference[0])
    assert bounded[2] == pytest.approx(reference[2])
    # Inward recovery remains gradual instead of jumping to the soft boundary.
    assert bounded[1] == pytest.approx(candidate[1])
    assert bounded[1] < lower[1]
    assert bounded[3] == pytest.approx(candidate[3])
    assert bounded[3] > upper[3]
    # A candidate that crosses the whole envelope saturates at its far boundary.
    assert bounded[4] == pytest.approx(upper[4])
    assert bounded[5] == pytest.approx(lower[5])
    assert bounded[6] == pytest.approx(candidate[6])
    for output, base, raw in zip(bounded, reference, candidate):
        assert min(base, raw) <= output <= max(base, raw)


def test_joint_soft_limits_reject_malformed_inputs():
    lower = [-1.0] * 7
    upper = [1.0] * 7
    with pytest.raises(ValueError, match="lower joint soft limit"):
        apply_joint_soft_limits(
            [0.0] * 7, [0.0] * 7, [0.0] * 7, [0.0] * 7
        )
    with pytest.raises(ValueError, match="finite"):
        apply_joint_soft_limits(
            [math.nan] + [0.0] * 6, [0.0] * 7, lower, upper
        )
    with pytest.raises(ValueError, match="exactly 7"):
        apply_joint_soft_limits([0.0] * 6, [0.0] * 7, lower, upper)


def test_gripper_tracking_backpressure_adapts_reserve_at_100_hz():
    open_step = GRIPPER_OPEN_VELOCITY_M_S / 100.0
    close_step = GRIPPER_CLOSE_VELOCITY_M_S / 100.0
    tracking_limit = 0.00504
    open_reserve = min(open_step, tracking_limit - open_step)
    close_reserve = min(close_step, tracking_limit - close_step)
    opening_feedback = 0.010
    opening_soft_upper = opening_feedback + tracking_limit - open_reserve
    closing_feedback = 0.030
    closing_soft_lower = closing_feedback - tracking_limit + close_reserve

    # Caught-up feedback permits one exact 5 mm step in either direction.
    opening_first = shape_gripper_command(
        current_command=opening_feedback,
        target=0.044,
        feedback=opening_feedback,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking_limit,
    )
    assert opening_first == pytest.approx(opening_feedback + open_step)
    assert opening_first == pytest.approx(opening_soft_upper)

    closing_first = shape_gripper_command(
        current_command=closing_feedback,
        target=0.0,
        feedback=closing_feedback,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking_limit,
    )
    assert closing_first == pytest.approx(closing_feedback - close_step)
    assert closing_first == pytest.approx(closing_soft_lower)

    # Until feedback catches the retained command, the next outward step holds.
    opening_hold = shape_gripper_command(
        current_command=opening_first,
        target=0.044,
        feedback=opening_feedback,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking_limit,
    )
    assert opening_hold == pytest.approx(opening_first)

    closing_hold = shape_gripper_command(
        current_command=closing_first,
        target=0.0,
        feedback=closing_feedback,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking_limit,
    )
    assert closing_hold == pytest.approx(closing_first)

    # Once feedback catches up, another exact 5 mm step is available.
    opening_resumed = shape_gripper_command(
        current_command=opening_first,
        target=0.044,
        feedback=opening_first,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking_limit,
    )
    assert opening_resumed == pytest.approx(opening_first + open_step)

    closing_resumed = shape_gripper_command(
        current_command=closing_first,
        target=0.0,
        feedback=closing_first,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking_limit,
    )
    assert closing_resumed == pytest.approx(closing_first - close_step)

    # A legal hard-gate-edge command outside the adaptive soft envelope retreats
    # deterministically to the boundary in the actual recovery direction.
    opening_hard_edge = math.nextafter(
        opening_feedback + tracking_limit, opening_feedback
    )
    opening_recovery = shape_gripper_command(
        current_command=opening_hard_edge,
        target=0.044,
        feedback=opening_feedback,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking_limit,
    )
    assert opening_recovery == pytest.approx(opening_soft_upper)

    closing_hard_edge = math.nextafter(
        closing_feedback - tracking_limit, closing_feedback
    )
    closing_recovery = shape_gripper_command(
        current_command=closing_hard_edge,
        target=0.0,
        feedback=closing_feedback,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=tracking_limit,
    )
    assert closing_recovery == pytest.approx(closing_soft_lower)

    with pytest.raises(ValueError, match="tracking hard gate"):
        shape_gripper_command(
            current_command=tracking_limit + 1e-6,
            target=0.044,
            feedback=0.0,
            max_open_step=open_step,
            max_close_step=close_step,
            max_tracking_error=tracking_limit,
        )
    with pytest.raises(ValueError, match="tracking limit must exceed"):
        shape_gripper_command(
            current_command=0.0,
            target=0.044,
            feedback=0.0,
            max_open_step=tracking_limit,
            max_close_step=close_step,
            max_tracking_error=tracking_limit,
        )


def test_gripper_reversal_interlock_brakes_real_race_until_feedback_settles():
    interlock = GripperReversalInterlock(min_settled_samples=3)
    tracking_limit = 0.00504
    step = GRIPPER_OPEN_VELOCITY_M_S / 100.0
    catchup_tolerance = tracking_limit - step
    previous_command = 0.014270
    current_command = 0.009270

    interlock.commit_follow_command(
        previous_command,
        current_command,
        target=0.0,
        feedback=previous_command,
        intent_deadband=catchup_tolerance,
    )
    assert interlock.active_direction == -1

    # This is the measured real-hardware failure shape: Trigger reverses to
    # opening while feedback is still overshooting the previous close command.
    assert interlock.should_hold(
        current_command=current_command,
        target=0.034716,
        feedback=0.008647,
        feedback_generation=1,
        catchup_tolerance=catchup_tolerance,
    )
    assert interlock.pending_direction == 1

    # A timer tick without a new JointState sample cannot satisfy the interlock.
    assert interlock.should_hold(
        current_command=current_command,
        target=0.034716,
        feedback=0.008647,
        feedback_generation=1,
        catchup_tolerance=catchup_tolerance,
    )
    assert interlock.settled_fresh_samples == 0

    brake_command = shape_gripper_reversal_brake_command(
        current_command=current_command,
        feedback=0.008647,
        max_open_step=step,
        max_close_step=step,
        max_tracking_error=tracking_limit,
    )
    assert brake_command == pytest.approx(0.008647)
    # Even an accidental follow-commit call cannot classify alignment as the
    # pending opening direction; the guard also skips this call in production.
    interlock.commit_follow_command(
        current_command,
        brake_command,
        target=0.034716,
        feedback=0.008647,
        intent_deadband=catchup_tolerance,
    )
    assert interlock.active_direction == -1
    assert interlock.pending_direction == 1

    # Continued old-direction motion resets settling even on a fresh sample.
    assert interlock.should_hold(
        current_command=brake_command,
        target=0.034716,
        feedback=0.008500,
        feedback_generation=2,
        catchup_tolerance=catchup_tolerance,
    )
    assert interlock.settled_fresh_samples == 0

    brake_command = shape_gripper_reversal_brake_command(
        current_command=brake_command,
        feedback=0.008500,
        max_open_step=step,
        max_close_step=step,
        max_tracking_error=tracking_limit,
    )
    assert brake_command == pytest.approx(0.008500)
    # Three fresh samples must be close to the aligned command and no longer move
    # in the old direction before a full 5 mm reverse step is released.
    assert interlock.should_hold(
        current_command=brake_command,
        target=0.034716,
        feedback=0.008500,
        feedback_generation=3,
        catchup_tolerance=catchup_tolerance,
    )
    assert interlock.settled_fresh_samples == 1
    assert interlock.should_hold(
        current_command=brake_command,
        target=0.034716,
        feedback=0.008500,
        feedback_generation=3,
        catchup_tolerance=catchup_tolerance,
    )
    assert interlock.settled_fresh_samples == 1
    assert interlock.should_hold(
        current_command=brake_command,
        target=0.034716,
        feedback=0.008500,
        feedback_generation=4,
        catchup_tolerance=catchup_tolerance,
    )
    assert not interlock.should_hold(
        current_command=brake_command,
        target=0.034716,
        feedback=0.008500,
        feedback_generation=5,
        catchup_tolerance=catchup_tolerance,
    )
    assert interlock.active_direction == -1
    assert interlock.pending_direction == 1

    reversed_command = shape_gripper_command(
        current_command=brake_command,
        target=0.034716,
        feedback=0.008500,
        max_open_step=step,
        max_close_step=step,
        max_tracking_error=tracking_limit,
    )
    assert reversed_command == pytest.approx(0.013500)
    assert reversed_command - brake_command == pytest.approx(step)
    assert reversed_command - 0.008500 < tracking_limit
    interlock.commit_follow_command(
        brake_command,
        reversed_command,
        target=0.034716,
        feedback=0.008500,
        intent_deadband=catchup_tolerance,
    )
    assert interlock.active_direction == 1
    assert interlock.pending_direction == 0


def test_gripper_reversal_interlock_handles_real_open_endpoint_without_deadlock():
    interlock = GripperReversalInterlock(min_settled_samples=3)
    tolerance = 0.00004
    interlock.commit_follow_command(
        0.035,
        0.040,
        target=0.040,
        feedback=0.035,
        intent_deadband=tolerance,
    )
    assert interlock.active_direction == 1

    # The first real log target was still 23 um above feedback.  It is neutral,
    # not a physical close reversal, even though it is below the leading command.
    assert not interlock.should_hold(
        current_command=0.040,
        target=0.039365,
        feedback=0.039342,
        feedback_generation=1,
        catchup_tolerance=tolerance,
    )
    assert interlock.active_direction == 1
    assert interlock.pending_direction == 0
    recovered_command = shape_gripper_command(
        current_command=0.040,
        target=0.039365,
        feedback=0.039342,
        max_open_step=0.005,
        max_close_step=0.005,
        max_tracking_error=0.00504,
    )
    assert recovered_command == pytest.approx(0.039365)
    interlock.commit_follow_command(
        0.040,
        recovered_command,
        target=0.039365,
        feedback=0.039342,
        intent_deadband=tolerance,
    )
    assert interlock.active_direction == 1

    # A true closing target first aligns the recovered command to the measured
    # 39.342 mm endpoint, then releases after three fresh static samples.
    assert interlock.should_hold(
        current_command=recovered_command,
        target=0.027878,
        feedback=0.039342,
        feedback_generation=2,
        catchup_tolerance=tolerance,
    )
    brake_command = shape_gripper_reversal_brake_command(
        current_command=recovered_command,
        feedback=0.039342,
        max_open_step=0.005,
        max_close_step=0.005,
        max_tracking_error=0.00504,
    )
    assert brake_command == pytest.approx(0.039342)
    interlock.commit_follow_command(
        recovered_command,
        brake_command,
        target=0.027878,
        feedback=0.039342,
        intent_deadband=tolerance,
    )
    assert interlock.active_direction == 1
    assert interlock.pending_direction == -1
    assert interlock.should_hold(
        current_command=brake_command,
        target=0.027878,
        feedback=0.039342,
        feedback_generation=3,
        catchup_tolerance=tolerance,
    )
    assert interlock.should_hold(
        current_command=brake_command,
        target=0.027878,
        feedback=0.039342,
        feedback_generation=4,
        catchup_tolerance=tolerance,
    )
    assert not interlock.should_hold(
        current_command=brake_command,
        target=0.027878,
        feedback=0.039342,
        feedback_generation=5,
        catchup_tolerance=tolerance,
    )
    close_command = shape_gripper_command(
        current_command=brake_command,
        target=0.027878,
        feedback=0.039342,
        max_open_step=0.005,
        max_close_step=0.005,
        max_tracking_error=0.00504,
    )
    assert close_command == pytest.approx(0.034342)
    assert brake_command - close_command == pytest.approx(0.005)
    assert abs(close_command - 0.039342) < 0.00504
    interlock.commit_follow_command(
        brake_command,
        close_command,
        target=0.027878,
        feedback=0.039342,
        intent_deadband=tolerance,
    )
    assert interlock.active_direction == -1
    assert interlock.pending_direction == 0


@pytest.mark.parametrize(
    ("active_direction", "current", "feedback", "target"),
    (
        (1, 0.040000, 0.039342, 0.0),
        (-1, 0.009270, 0.008647, 0.040),
    ),
)
def test_gripper_reversal_interlock_accepts_lsb_jitter_but_rejects_old_drift(
    active_direction: int,
    current: float,
    feedback: float,
    target: float,
):
    motion_tolerance = 0.00002
    catchup_tolerance = 0.00004
    old_direction = active_direction
    pending_direction = -active_direction
    interlock = GripperReversalInterlock(
        min_settled_samples=3,
        feedback_motion_tolerance=motion_tolerance,
        active_direction=active_direction,
    )
    assert interlock.should_hold(
        current_command=current,
        target=target,
        feedback=feedback,
        feedback_generation=1,
        catchup_tolerance=catchup_tolerance,
    )
    assert interlock.pending_direction == pending_direction
    aligned = shape_gripper_reversal_brake_command(
        current_command=current,
        feedback=feedback,
        max_open_step=0.005,
        max_close_step=0.005,
        max_tracking_error=0.00504,
    )

    # One DM4310 position LSB maps to roughly 16 um at the ROS gripper joint.
    # Alternating one-LSB noise stays inside the reviewed 20 um tolerance and
    # must not cause a permanent 1/0 settle loop.
    latest_feedback = feedback
    for generation, offset in enumerate(
        (old_direction * 0.000016, 0.0, old_direction * 0.000016),
        start=2,
    ):
        latest_feedback = feedback + offset
        hold = interlock.should_hold(
            current_command=aligned,
            target=target,
            feedback=latest_feedback,
            feedback_generation=generation,
            catchup_tolerance=catchup_tolerance,
        )
        if generation < 4:
            assert hold
            aligned = shape_gripper_reversal_brake_command(
                current_command=aligned,
                feedback=latest_feedback,
                max_open_step=0.005,
                max_close_step=0.005,
                max_tracking_error=0.00504,
            )
        else:
            assert not hold
    assert interlock.settled_fresh_samples == 3

    reversed_command = shape_gripper_command(
        current_command=aligned,
        target=target,
        feedback=latest_feedback,
        max_open_step=0.005,
        max_close_step=0.005,
        max_tracking_error=0.00504,
    )
    # One further maximum tolerated old-direction sample in the async publish
    # window still remains strictly inside the unchanged 5.04 mm hard gate.
    adversarial_feedback = latest_feedback + old_direction * motion_tolerance
    assert abs(reversed_command - adversarial_feedback) <= 0.00502 + 1e-12
    assert abs(reversed_command - adversarial_feedback) < 0.00504

    drifting = GripperReversalInterlock(
        min_settled_samples=3,
        feedback_motion_tolerance=motion_tolerance,
        active_direction=active_direction,
    )
    assert drifting.should_hold(
        current_command=current,
        target=target,
        feedback=feedback,
        feedback_generation=1,
        catchup_tolerance=catchup_tolerance,
    )
    drift_command = feedback
    assert drifting.should_hold(
        current_command=drift_command,
        target=target,
        feedback=feedback + old_direction * 0.000016,
        feedback_generation=2,
        catchup_tolerance=catchup_tolerance,
    )
    drift_command = feedback + old_direction * 0.000016
    assert drifting.should_hold(
        current_command=drift_command,
        target=target,
        feedback=feedback + old_direction * 0.000032,
        feedback_generation=3,
        catchup_tolerance=catchup_tolerance,
    )
    assert drifting.settled_fresh_samples == 0


def test_gripper_reversal_interlock_cancels_when_intent_returns_to_active_direction():
    interlock = GripperReversalInterlock(min_settled_samples=2)
    interlock.commit_follow_command(
        0.020,
        0.015,
        target=0.0,
        feedback=0.020,
        intent_deadband=0.00004,
    )
    assert interlock.should_hold(
        current_command=0.015,
        target=0.040,
        feedback=0.014,
        feedback_generation=1,
        catchup_tolerance=0.00004,
    )
    assert not interlock.should_hold(
        current_command=0.015,
        target=0.0,
        feedback=0.014,
        feedback_generation=2,
        catchup_tolerance=0.00004,
    )
    assert interlock.active_direction == -1
    assert interlock.pending_direction == 0


def test_gripper_reversal_interlock_does_not_release_outside_catchup_reserve():
    interlock = GripperReversalInterlock(min_settled_samples=2)
    interlock.commit_follow_command(
        0.020,
        0.015,
        target=0.0,
        feedback=0.020,
        intent_deadband=0.00004,
    )
    assert interlock.should_hold(
        current_command=0.015,
        target=0.040,
        feedback=0.01490,
        feedback_generation=1,
        catchup_tolerance=0.00004,
    )
    assert interlock.should_hold(
        current_command=0.015,
        target=0.040,
        feedback=0.014959,
        feedback_generation=2,
        catchup_tolerance=0.00004,
    )
    assert interlock.settled_fresh_samples == 0


def test_gripper_reversal_interlock_allows_startup_before_a_direction_is_published():
    interlock = GripperReversalInterlock(min_settled_samples=2)
    assert not interlock.should_hold(
        current_command=0.010,
        target=0.040,
        feedback=0.010,
        feedback_generation=1,
        catchup_tolerance=0.00004,
    )
    assert interlock.active_direction == 0
    assert interlock.pending_direction == 0


def test_gripper_reversal_interlock_fault_reset_keeps_only_published_direction():
    interlock = GripperReversalInterlock(min_settled_samples=2)
    interlock.commit_follow_command(
        0.020,
        0.015,
        target=0.0,
        feedback=0.020,
        intent_deadband=0.00004,
    )
    assert interlock.should_hold(
        current_command=0.015,
        target=0.040,
        feedback=0.01499,
        feedback_generation=1,
        catchup_tolerance=0.00004,
    )
    assert interlock.should_hold(
        current_command=0.015,
        target=0.040,
        feedback=0.015,
        feedback_generation=2,
        catchup_tolerance=0.00004,
    )
    assert interlock.settled_fresh_samples == 1

    interlock.reset_pending()
    assert interlock.active_direction == -1
    assert interlock.pending_direction == 0
    assert interlock.settled_fresh_samples == 0
    assert interlock.settle_origin_feedback is None
    assert interlock.last_feedback is None
    assert interlock.last_feedback_generation is None
    interlock.commit_follow_command(
        0.015,
        0.015,
        target=0.040,
        feedback=0.015,
        intent_deadband=0.00004,
    )
    assert interlock.active_direction == -1


def test_gripper_reversal_interlock_rearms_after_fault_without_losing_direction_history():
    interlock = GripperReversalInterlock(
        min_settled_samples=3,
        feedback_motion_tolerance=0.00002,
    )
    tolerance = 0.00004
    interlock.commit_follow_command(
        0.020,
        0.015,
        target=0.0,
        feedback=0.020,
        intent_deadband=tolerance,
    )
    assert interlock.active_direction == -1

    # A fault interrupts a pending opening reversal after one settled sample.
    assert interlock.should_hold(
        current_command=0.015,
        target=0.040,
        feedback=0.01499,
        feedback_generation=1,
        catchup_tolerance=tolerance,
    )
    assert interlock.should_hold(
        current_command=0.01499,
        target=0.040,
        feedback=0.01499,
        feedback_generation=2,
        catchup_tolerance=tolerance,
    )
    assert interlock.settled_fresh_samples == 1
    interlock.reset_pending()
    assert interlock.active_direction == -1
    assert interlock.pending_direction == 0
    assert interlock.settled_fresh_samples == 0

    # Rearm's forced opening must establish entirely new evidence instead of
    # inheriting the pre-fault sample, then commit the published opening step.
    assert interlock.should_hold(
        current_command=0.01499,
        target=0.040,
        feedback=0.01499,
        feedback_generation=3,
        catchup_tolerance=tolerance,
    )
    for generation in (4, 5):
        assert interlock.should_hold(
            current_command=0.01499,
            target=0.040,
            feedback=0.01499,
            feedback_generation=generation,
            catchup_tolerance=tolerance,
        )
    assert not interlock.should_hold(
        current_command=0.01499,
        target=0.040,
        feedback=0.01499,
        feedback_generation=6,
        catchup_tolerance=tolerance,
    )
    opening_command = shape_gripper_command(
        current_command=0.01499,
        target=0.040,
        feedback=0.01499,
        max_open_step=0.005,
        max_close_step=0.005,
        max_tracking_error=0.00504,
    )
    interlock.commit_follow_command(
        0.01499,
        opening_command,
        target=0.040,
        feedback=0.01499,
        intent_deadband=tolerance,
    )
    assert interlock.active_direction == 1
    assert interlock.pending_direction == 0

    # The next genuine close request is still recognized as a new reversal.
    assert interlock.should_hold(
        current_command=opening_command,
        target=0.0,
        feedback=opening_command,
        feedback_generation=7,
        catchup_tolerance=tolerance,
    )
    assert interlock.active_direction == 1
    assert interlock.pending_direction == -1


def test_gripper_reversal_interlock_is_symmetric_for_open_to_close():
    interlock = GripperReversalInterlock(min_settled_samples=2)
    interlock.commit_follow_command(
        0.020,
        0.025,
        target=0.040,
        feedback=0.020,
        intent_deadband=0.00004,
    )
    assert interlock.active_direction == 1
    assert interlock.should_hold(
        current_command=0.025,
        target=0.0,
        feedback=0.0255,
        feedback_generation=1,
        catchup_tolerance=0.00004,
    )
    assert interlock.pending_direction == -1
    assert interlock.should_hold(
        current_command=0.025,
        target=0.0,
        feedback=0.0257,
        feedback_generation=2,
        catchup_tolerance=0.00004,
    )
    assert interlock.settled_fresh_samples == 0
    assert interlock.should_hold(
        current_command=0.025,
        target=0.0,
        feedback=0.02502,
        feedback_generation=3,
        catchup_tolerance=0.00004,
    )
    assert not interlock.should_hold(
        current_command=0.025,
        target=0.0,
        feedback=0.025,
        feedback_generation=4,
        catchup_tolerance=0.00004,
    )


@pytest.mark.parametrize(
    ("current", "feedback", "expected"),
    (
        (0.009270, 0.008647, 0.008647),
        (0.010000, 0.004970, 0.005000),
        (0.030000, 0.035030, 0.035000),
    ),
)
def test_gripper_reversal_brake_moves_toward_feedback_without_crossing_it(
    current: float,
    feedback: float,
    expected: float,
):
    command = shape_gripper_reversal_brake_command(
        current_command=current,
        feedback=feedback,
        max_open_step=0.005,
        max_close_step=0.005,
        max_tracking_error=0.00504,
    )
    assert command == pytest.approx(expected)
    assert abs(command - feedback) < 0.00504
    assert abs(command - current) <= 0.005
    assert (feedback - current) * (command - feedback) <= 1e-12


def test_gripper_guard_rejects_limits_tracking_and_unbounded_steps():
    open_step = GRIPPER_OPEN_VELOCITY_M_S / 100.0
    close_step = GRIPPER_CLOSE_VELOCITY_M_S / 100.0
    accepted = validate_gripper_command(
        candidate=open_step,
        feedback=0.0,
        previous_command=0.0,
        lower_limit=0.0,
        upper_limit=0.044,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=0.00504,
    )
    assert accepted.accepted

    outside = validate_gripper_command(
        candidate=0.045,
        feedback=0.0,
        previous_command=0.0,
        lower_limit=0.0,
        upper_limit=0.044,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=0.00504,
    )
    assert not outside.accepted and "limits" in outside.reason

    tracking = validate_gripper_command(
        candidate=0.0101,
        feedback=0.0101,
        previous_command=0.0,
        lower_limit=0.0,
        upper_limit=0.044,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=0.00504,
    )
    assert not tracking.accepted and "tracking" in tracking.reason

    too_fast = validate_gripper_command(
        candidate=open_step + 0.00002,
        feedback=0.0,
        previous_command=0.0,
        lower_limit=0.0,
        upper_limit=0.044,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=0.00504,
    )
    assert not too_fast.accepted and "step" in too_fast.reason

    accepted_close = validate_gripper_command(
        candidate=0.020 - close_step,
        feedback=0.020,
        previous_command=0.020,
        lower_limit=0.0,
        upper_limit=0.044,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=0.00504,
    )
    assert accepted_close.accepted

    too_fast_close = validate_gripper_command(
        candidate=0.020 - close_step - 0.00002,
        feedback=0.020,
        previous_command=0.020,
        lower_limit=0.0,
        upper_limit=0.044,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=0.00504,
    )
    assert not too_fast_close.accepted and "step" in too_fast_close.reason

    # Even when the previous command remains legal, the next legal-rate step
    # must not cross the hardware's command-to-feedback tracking gate.
    candidate_tracking = validate_gripper_command(
        candidate=0.005 + open_step,
        feedback=0.0,
        previous_command=0.005,
        lower_limit=0.0,
        upper_limit=0.044,
        max_open_step=open_step,
        max_close_step=close_step,
        max_tracking_error=0.00504,
    )
    assert not candidate_tracking.accepted
    assert "candidate tracking" in candidate_tracking.reason


def test_gripper_fault_reentry_must_step_from_retained_controller_command():
    open_step = GRIPPER_OPEN_VELOCITY_M_S / 100.0
    close_step = GRIPPER_CLOSE_VELOCITY_M_S / 100.0
    hardware_burst_capacity = GRIPPER_CLOSE_VELOCITY_M_S * 0.020
    hardware_burst_tolerance = 1e-12
    retained_command = 0.020
    lagged_feedback_at_rearm = retained_command - 0.00504

    continuous = step_gripper_toward(
        retained_command, 0.0, open_step, close_step
    )
    assert abs(continuous - retained_command) == pytest.approx(close_step)
    assert (
        abs(continuous - retained_command)
        <= hardware_burst_capacity + hardware_burst_tolerance
    )

    # Re-anchoring software state to lagged feedback without publishing that
    # anchor would make the next command discontinuous from what the controller
    # and hardware still retain, reproducing the real token-bucket rejection.
    discontinuous = step_gripper_toward(
        lagged_feedback_at_rearm, 0.0, open_step, close_step
    )
    assert (
        abs(discontinuous - retained_command)
        > hardware_burst_capacity + hardware_burst_tolerance
    )


def test_guard_rejects_joint_limit_step_tracking_and_nonfinite_failures():
    lower = [-1.0] * 7
    upper = [1.0] * 7
    step = [0.01] * 7
    feedback = [0.0] * 7

    accepted = validate_guard_command(
        [0.005] * 7, feedback, None, lower, upper, step, 0.2
    )
    assert accepted.accepted

    float32_boundary = validate_guard_command(
        [0.010000029] * 7, feedback, None, lower, upper, step, 0.2
    )
    assert float32_boundary.accepted
    assert float32_boundary.command == pytest.approx([0.01] * 7, abs=1e-12)

    meaningful_step_violation = validate_guard_command(
        [0.01002] * 7, feedback, None, lower, upper, step, 0.2
    )
    assert not meaningful_step_violation.accepted
    assert "candidate=" in meaningful_step_violation.reason

    joint_limit_roundoff = validate_guard_command(
        [-1.0000004] + [0.0] * 6, [-1.0] + [0.0] * 6, None, lower, upper, step, 0.2
    )
    assert joint_limit_roundoff.accepted
    assert joint_limit_roundoff.command is not None
    assert joint_limit_roundoff.command[0] == -1.0

    meaningful_limit_violation = validate_guard_command(
        [-1.00001] + [0.0] * 6, [-1.0] + [0.0] * 6, None, lower, upper, step, 0.2
    )
    assert not meaningful_limit_violation.accepted

    too_large = validate_guard_command(
        [0.02] * 7, feedback, None, lower, upper, step, 0.2
    )
    assert not too_large.accepted and "step" in too_large.reason

    outside = validate_guard_command(
        [2.0] + [0.0] * 6, feedback, None, lower, upper, step, 0.2
    )
    assert not outside.accepted and "limits" in outside.reason

    tracking = validate_guard_command(
        [0.0] * 7, [0.3] + [0.0] * 6, [0.0] * 7, lower, upper, step, 0.2
    )
    assert not tracking.accepted and "tracking" in tracking.reason

    nonfinite = validate_guard_command(
        [math.nan] + [0.0] * 6, feedback, None, lower, upper, step, 0.2
    )
    assert not nonfinite.accepted and "non-finite" in nonfinite.reason


def test_solver_delta_is_checked_before_controller_delta_is_saturated():
    lower = [-1.0] * 7
    upper = [1.0] * 7
    step = [0.01] * 7
    previous = [0.0] * 7
    feedback = [0.005] * 7
    candidate = [0.015] * 7
    solver_base = [0.005] * 7

    solver_decision = validate_solver_candidate_step(candidate, solver_base, step)
    assert solver_decision.accepted
    assert solver_decision.command == pytest.approx(candidate)

    controller_bounded = step_toward(previous, solver_decision.command, step)
    assert controller_bounded == pytest.approx([0.01] * 7)
    final = validate_guard_command(
        controller_bounded, feedback, previous, lower, upper, step, 0.2
    )
    assert final.accepted
    assert final.command == pytest.approx([0.01] * 7)

    # A meaningful raw solver excess is rejected even though blindly applying
    # the controller-side saturation could have produced a bounded command.
    unsafe_raw = validate_solver_candidate_step(
        [0.01502] * 7, solver_base, step
    )
    assert not unsafe_raw.accepted
    assert "solver_base=" in unsafe_raw.reason


def test_solver_delta_clamps_only_existing_float32_numeric_tolerance():
    step = [0.01] * 7
    base = [0.25] * 7
    float32_residue = validate_solver_candidate_step(
        [0.260005] * 7, base, step
    )
    assert float32_residue.accepted
    assert float32_residue.command == pytest.approx([0.26] * 7, abs=1e-12)

    meaningful_excess = validate_solver_candidate_step(
        [0.26002] * 7, base, step
    )
    assert not meaningful_excess.accepted


def test_guard_never_reanchors_an_existing_command_to_feedback():
    decision = validate_guard_command(
        [0.015] * 7,
        [0.005] * 7,
        [0.0] * 7,
        [-1.0] * 7,
        [1.0] * 7,
        [0.01] * 7,
        0.2,
    )
    assert not decision.accepted
    assert "step" in decision.reason


def test_left_and_right_joint_sets_are_complete_and_disjoint():
    assert len(RIGHT_JOINT_NAMES) == len(LEFT_JOINT_NAMES) == 7
    assert not set(RIGHT_JOINT_NAMES) & set(LEFT_JOINT_NAMES)
