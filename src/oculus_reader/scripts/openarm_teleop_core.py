#!/usr/bin/env python3
"""ROS-independent safety and data-contract helpers for OpenArm teleoperation.

This module deliberately has no ROS, MuJoCo, or ``openarm_control`` imports so
the safety rules can be unit-tested in the host Python environment.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Iterable, Sequence


ARM_DOF = 7
DRIVER_DOF = 8
BIMANUAL_DRIVER_DOF = 16
INTENT_TOKEN_COUNT = 2
ATOMIC_TARGET_DOF = ARM_DOF + 3
GUARD_ANCHOR_DOF = ARM_DOF + INTENT_TOKEN_COUNT
ATOMIC_IK_COMMAND_DOF = ARM_DOF * 2 + INTENT_TOKEN_COUNT
GUARD_ACCEPTANCE_DOF = ARM_DOF * 3 + INTENT_TOKEN_COUNT
TARGET_HISTORY_CAPACITY = 32
INTENT_TOKEN_MODE_BITS = 8
MAX_EXACT_FLOAT64_INT = (1 << 53) - 1
MAX_INTENT_EPOCH = MAX_EXACT_FLOAT64_INT >> INTENT_TOKEN_MODE_BITS
JOINT_STEP_NUMERIC_TOLERANCE_RAD = 1e-5
JOINT_LIMIT_NUMERIC_TOLERANCE_RAD = 1e-6
GRIPPER_MIN_POSITION_M = 0.0
GRIPPER_MAX_POSITION_M = 0.044
# This robot's commissioned runtime-open target is 0.040 m while retaining the
# official 0.044 m hard envelope as independent command/feedback protection.
GRIPPER_OPEN_POSITION_M = 0.040
# Startup/re-arm opening and Trigger-driven motion use the same commissioned
# slew limit; the Trigger itself always selects a continuous position target.
GRIPPER_OPEN_VELOCITY_M_S = 0.5000
GRIPPER_CLOSE_VELOCITY_M_S = 0.5000
RCLPY_HUMBLE_SHUTDOWN_TAKE_RACE_MESSAGE = (
    "Unable to convert call argument to Python object "
    "(compile in debug mode for details)"
)

# Pinned ``openarm-control==0.2.0`` defines these per-arm-joint IK velocity
# caps in ``openarm_control.config.ARM_JOINT_VELOCITY_LIMITS_RAD_S``.  The VR
# command chain deliberately uses 90% of those official software caps; the
# much larger URDF/CAN ranges are not teleoperation command limits.
OPENARM_CONTROL_MAX_JOINT_VELOCITY_RAD_S = (
    1.57,
    1.57,
    3.14,
    3.14,
    12.6,
    12.6,
    12.6,
)
OPENARM_TELEOP_VELOCITY_LIMIT_RATIO = 0.90
OPENARM_TELEOP_MAX_JOINT_VELOCITY_RAD_S = tuple(
    value * OPENARM_TELEOP_VELOCITY_LIMIT_RATIO
    for value in OPENARM_CONTROL_MAX_JOINT_VELOCITY_RAD_S
)
OPENARM_TELEOP_CONTROL_RATE_HZ = 100.0
# The existing command shaper reserves one complete next-cycle step inside
# each hard tracking gate.  At 100 Hz, 0.24 rad is the smallest reviewed round
# value that keeps two J5-J7 90%-cap steps (2 * 0.1134 rad) inside that gate;
# the slower J1-J4 retain their original 0.12 rad protection.
OPENARM_TELEOP_TRACKING_ERROR_RAD = (
    0.12,
    0.12,
    0.12,
    0.12,
    0.24,
    0.24,
    0.24,
)

# Exact joint ranges from the pinned official bimanual OpenArm v1 MJCF/URDF.
# These are the final hard position limits consumed by the ROS command guard
# and the ros2_control hardware safety patch.  Keep the upstream decimal
# values instead of reconstructing them from rounded degree labels.
OPENARM_LEFT_JOINT_HARD_LOWER_RAD = (
    -3.490659,
    -3.3161253267948965,
    -1.570796,
    0.0,
    -1.570796,
    -0.785398,
    -1.570796,
)
OPENARM_LEFT_JOINT_HARD_UPPER_RAD = (
    1.3962629999999998,
    0.17453267320510335,
    1.570796,
    2.443461,
    1.570796,
    0.785398,
    1.570796,
)
OPENARM_RIGHT_JOINT_HARD_LOWER_RAD = (
    -1.396263,
    -0.17453267320510335,
    -1.570796,
    0.0,
    -1.570796,
    -0.785398,
    -1.570796,
)
OPENARM_RIGHT_JOINT_HARD_UPPER_RAD = (
    3.490659,
    3.3161253267948965,
    1.570796,
    2.443461,
    1.570796,
    0.785398,
    1.570796,
)

# Every official bimanual arm joint has ref/q0=0.  Scale both hard endpoints
# toward that zero by exactly 95% to form the IK-only teleoperation work area.
# The solver saturates at this soft boundary; only the official 100% range is a
# hard command/feedback fault boundary.
OPENARM_TELEOP_SOFT_LIMIT_RATIO = 0.95
OPENARM_LEFT_JOINT_SOFT_LOWER_RAD = tuple(
    value * OPENARM_TELEOP_SOFT_LIMIT_RATIO
    for value in OPENARM_LEFT_JOINT_HARD_LOWER_RAD
)
OPENARM_LEFT_JOINT_SOFT_UPPER_RAD = tuple(
    value * OPENARM_TELEOP_SOFT_LIMIT_RATIO
    for value in OPENARM_LEFT_JOINT_HARD_UPPER_RAD
)
OPENARM_RIGHT_JOINT_SOFT_LOWER_RAD = tuple(
    value * OPENARM_TELEOP_SOFT_LIMIT_RATIO
    for value in OPENARM_RIGHT_JOINT_HARD_LOWER_RAD
)
OPENARM_RIGHT_JOINT_SOFT_UPPER_RAD = tuple(
    value * OPENARM_TELEOP_SOFT_LIMIT_RATIO
    for value in OPENARM_RIGHT_JOINT_HARD_UPPER_RAD
)

RIGHT_JOINT_NAMES = tuple(f"openarm_right_joint{i}" for i in range(1, ARM_DOF + 1))
LEFT_JOINT_NAMES = tuple(f"openarm_left_joint{i}" for i in range(1, ARM_DOF + 1))


def side_control_defaults(side: str) -> tuple[str, str, str]:
    """Return ``(deadman, home, trigger)`` for a Quest controller side."""

    normalized = str(side).strip().lower()
    if normalized == "left":
        return "LG", "LJ", "leftTrig"
    if normalized == "right":
        return "RG", "RJ", "rightTrig"
    raise ValueError("side must be 'left' or 'right'")


def update_grip_deadman(
    *,
    raw_pressed: bool,
    analog_value: float,
    previous: bool,
    press_threshold: float,
    release_threshold: float,
) -> bool:
    """Fuse the Quest boolean Grip token with its continuous squeeze value.

    Some Quest/OpenXR combinations emit a short boolean ``LG``/``RG`` pulse
    even while the continuous ``leftGrip``/``rightGrip`` value remains held.
    Hysteresis keeps the deadman stable without ever latching it across a fully
    released or malformed analog sample.  The boolean path remains a backwards
    compatible, one-packet assertion path.
    """

    press = float(press_threshold)
    release = float(release_threshold)
    if (
        not math.isfinite(press)
        or not math.isfinite(release)
        or not 0.0 <= release < press <= 1.0
    ):
        raise ValueError("Grip thresholds must satisfy 0 <= release < press <= 1")
    try:
        analog = float(analog_value)
    except (TypeError, ValueError):
        analog = math.nan
    if not math.isfinite(analog) or not 0.0 <= analog <= 1.0:
        return bool(raw_pressed)
    if bool(raw_pressed) or analog >= press:
        return True
    if analog <= release:
        return False
    return bool(previous)


class SideMode(IntEnum):
    """Per-arm command mode exchanged between the OpenArm ROS nodes."""

    HOLD = 0
    TELEOP = 1
    HOME = 2
    FAULT = 3


def _strict_side_mode(value: SideMode | int, label: str) -> SideMode:
    if isinstance(value, SideMode):
        return value
    if type(value) is not int:
        raise ValueError(f"{label} must be an integer SideMode value")
    try:
        return SideMode(value)
    except ValueError as exc:
        raise ValueError(f"{label} is not a valid SideMode value") from exc


def encode_intent_token(epoch: int, mode: SideMode | int) -> int:
    """Encode one exact session/mode token without floating-point arithmetic."""

    if type(epoch) is not int:
        raise ValueError("intent epoch must be an integer")
    if epoch <= 0:
        raise ValueError("intent epoch must be positive")
    if epoch > MAX_INTENT_EPOCH:
        raise OverflowError("intent epoch exceeds the exact Float64 token range")
    selected_mode = _strict_side_mode(mode, "intent mode")
    token = (epoch << INTENT_TOKEN_MODE_BITS) | int(selected_mode)
    if token <= 0 or token > MAX_EXACT_FLOAT64_INT:
        raise OverflowError("intent token exceeds the exact Float64 integer range")
    return token


def decode_intent_token(token: int) -> tuple[int, SideMode]:
    """Decode and strictly validate one raw integer intent token."""

    if type(token) is not int:
        raise ValueError("intent token must be an integer")
    if token <= 0:
        raise ValueError("intent token must be positive")
    if token > MAX_EXACT_FLOAT64_INT:
        raise OverflowError("intent token exceeds the exact Float64 integer range")
    epoch = token >> INTENT_TOKEN_MODE_BITS
    if epoch <= 0 or epoch > MAX_INTENT_EPOCH:
        raise ValueError("intent token carries an invalid epoch")
    mode = _strict_side_mode(
        token & ((1 << INTENT_TOKEN_MODE_BITS) - 1),
        "intent token mode",
    )
    return epoch, mode


def transition_intent_token(token: int, new_mode: SideMode | int) -> int:
    """Advance epoch exactly once on every edge into or out of TELEOP.

    Repeated publications of the same mode retain the same raw token. A mode
    change entirely among HOLD/HOME/FAULT retains the epoch but still changes
    the low-byte mode encoded in the raw token.
    """

    epoch, previous_mode = decode_intent_token(token)
    selected_mode = _strict_side_mode(new_mode, "new intent mode")
    teleop_edge = previous_mode != selected_mode and (
        previous_mode == SideMode.TELEOP or selected_mode == SideMode.TELEOP
    )
    if teleop_edge:
        if epoch >= MAX_INTENT_EPOCH:
            raise OverflowError("intent epoch exhausted on TELEOP boundary")
        epoch += 1
    return encode_intent_token(epoch, selected_mode)


@dataclass(frozen=True)
class GuardDecision:
    """Result of validating a seven-joint command."""

    accepted: bool
    command: tuple[float, ...] | None
    reason: str


@dataclass
class ReleaseToRearmLatch:
    """Latch a fault until inputs are healthy and controls stay released."""

    release_sec: float
    latched: bool = False
    _release_started_ns: int | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.release_sec) or self.release_sec < 0.0:
            raise ValueError("release_sec must be finite and non-negative")

    def trip(self) -> None:
        self.latched = True
        self._release_started_ns = None

    def update(self, now_ns: int, controls_released: bool, inputs_ready: bool) -> bool:
        """Update and return the latch state.

        Any pressed control or missing input resets the release timer. A zero
        release duration still requires observing one healthy released sample.
        """

        if not self.latched:
            return False
        if not controls_released or not inputs_ready:
            self._release_started_ns = None
            return True
        if self._release_started_ns is None:
            self._release_started_ns = int(now_ns)
            if self.release_sec > 0.0:
                return True
        elapsed_ns = int(now_ns) - self._release_started_ns
        if elapsed_ns < int(self.release_sec * 1_000_000_000):
            return True
        self.latched = False
        self._release_started_ns = None
        return False


@dataclass
class GripperReversalInterlock:
    """Brake direction reversals until fresh feedback has stopped overshooting."""

    min_settled_samples: int = 3
    feedback_motion_tolerance: float = 0.00002
    active_direction: int = 0
    pending_direction: int = 0
    settled_fresh_samples: int = 0
    settle_origin_feedback: float | None = None
    last_feedback: float | None = None
    last_feedback_generation: int | None = None

    def __post_init__(self) -> None:
        if (
            isinstance(self.min_settled_samples, bool)
            or not isinstance(self.min_settled_samples, int)
            or self.min_settled_samples < 1
        ):
            raise ValueError("min_settled_samples must be a positive integer")
        self.feedback_motion_tolerance = float(self.feedback_motion_tolerance)
        if (
            not math.isfinite(self.feedback_motion_tolerance)
            or self.feedback_motion_tolerance <= 0.0
        ):
            raise ValueError("feedback_motion_tolerance must be positive")

    @staticmethod
    def _direction(delta: float, deadband: float = 1e-12) -> int:
        if delta > deadband:
            return 1
        if delta < -deadband:
            return -1
        return 0

    def _record_feedback(self, feedback: float, generation: int) -> None:
        self.last_feedback = feedback
        self.last_feedback_generation = generation

    def _cancel_pending(self) -> None:
        self.pending_direction = 0
        self.settled_fresh_samples = 0
        self.settle_origin_feedback = None

    def reset_pending(self) -> None:
        """Discard reversal evidence while retaining the last published direction."""

        self._cancel_pending()
        self.last_feedback = None
        self.last_feedback_generation = None

    def commit_follow_command(
        self,
        previous_command: float,
        command: float,
        *,
        target: float,
        feedback: float,
        intent_deadband: float,
    ) -> None:
        """Record only a published command that truly drives toward the target."""

        previous = float(previous_command)
        committed = float(command)
        requested = float(target)
        measured = float(feedback)
        deadband = float(intent_deadband)
        if not all(
            math.isfinite(value)
            for value in (previous, committed, requested, measured, deadband)
        ):
            raise ValueError("gripper follow inputs must be finite")
        if deadband <= 0.0:
            raise ValueError("intent_deadband must be positive")
        command_direction = self._direction(committed - previous)
        intent_direction = self._direction(requested - measured, deadband)
        if command_direction == 0 or command_direction != intent_direction:
            return
        # A tracking-recovery command can have the same delta as the requested
        # direction while remaining on the wrong side of measured position.
        # It is not evidence that the motor was commanded into the new motion.
        if intent_direction * (committed - measured) <= 1e-12:
            return
        self.active_direction = intent_direction
        if self.pending_direction == intent_direction:
            self._cancel_pending()

    def should_hold(
        self,
        *,
        current_command: float,
        target: float,
        feedback: float,
        feedback_generation: int,
        catchup_tolerance: float,
    ) -> bool:
        """Return whether an opposite-direction command must remain held."""

        current = float(current_command)
        requested = float(target)
        measured = float(feedback)
        tolerance = float(catchup_tolerance)
        if not all(math.isfinite(value) for value in (current, requested, measured)):
            raise ValueError("gripper reversal inputs must be finite")
        if not math.isfinite(tolerance) or tolerance <= 0.0:
            raise ValueError("catchup_tolerance must be finite and positive")
        if (
            isinstance(feedback_generation, bool)
            or not isinstance(feedback_generation, int)
            or feedback_generation < 0
        ):
            raise ValueError("feedback_generation must be a non-negative integer")
        if (
            self.last_feedback_generation is not None
            and feedback_generation < self.last_feedback_generation
        ):
            raise ValueError("feedback_generation must be monotonic")

        fresh_feedback = feedback_generation != self.last_feedback_generation
        # Direction is physical only relative to measured position.  Comparing
        # against a leading command misclassifies safe command alignment as a
        # motor reversal.  The tracking reserve is also the neutral deadband.
        desired_direction = self._direction(requested - measured, tolerance)
        if desired_direction == 0:
            self._cancel_pending()
            if fresh_feedback:
                self._record_feedback(measured, feedback_generation)
            return False

        if self.active_direction == 0:
            self._cancel_pending()
            if fresh_feedback:
                self._record_feedback(measured, feedback_generation)
            return False

        if self.pending_direction == 0:
            if desired_direction == self.active_direction:
                if fresh_feedback:
                    self._record_feedback(measured, feedback_generation)
                return False
            self.pending_direction = desired_direction
            self.settled_fresh_samples = 0
            self.settle_origin_feedback = measured
            if fresh_feedback:
                self._record_feedback(measured, feedback_generation)
            return True

        if desired_direction == self.active_direction:
            self._cancel_pending()
            if fresh_feedback:
                self._record_feedback(measured, feedback_generation)
            return False
        if desired_direction != self.pending_direction:
            self.pending_direction = desired_direction
            self.settled_fresh_samples = 0
            self.settle_origin_feedback = measured
            if fresh_feedback:
                self._record_feedback(measured, feedback_generation)
            return True
        if not fresh_feedback:
            return self.settled_fresh_samples < self.min_settled_samples

        previous_feedback = self.last_feedback
        origin_feedback = self.settle_origin_feedback
        step_has_stopped_old_direction = (
            previous_feedback is not None
            and self.pending_direction * (measured - previous_feedback)
            >= -self.feedback_motion_tolerance
        )
        window_has_stopped_old_direction = (
            origin_feedback is not None
            and self.pending_direction * (measured - origin_feedback)
            >= -self.feedback_motion_tolerance
        )
        caught_up = abs(measured - current) <= tolerance + 1e-12
        if (
            step_has_stopped_old_direction
            and window_has_stopped_old_direction
            and caught_up
        ):
            self.settled_fresh_samples += 1
        else:
            self.settled_fresh_samples = 0
            self.settle_origin_feedback = measured
        self._record_feedback(measured, feedback_generation)
        return self.settled_fresh_samples < self.min_settled_samples


def _float_tuple(values: Iterable[float], expected: int, label: str) -> tuple[float, ...]:
    converted = tuple(float(value) for value in values)
    if len(converted) != expected:
        raise ValueError(f"{label} must contain exactly {expected} values")
    if not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{label} contains a non-finite value")
    return converted


def _tracking_error_tuple(
    values: float | Sequence[float], label: str = "max_tracking_error"
) -> tuple[float, ...]:
    """Accept a legacy scalar or a reviewed per-axis tracking-error vector."""

    try:
        scalar = float(values)
    except (TypeError, ValueError):
        return _float_tuple(values, ARM_DOF, label)
    return (scalar,) * ARM_DOF


def validate_joint_names(names: Sequence[str], label: str = "joint_names") -> tuple[str, ...]:
    """Validate a complete, unambiguous seven-axis joint-name list."""

    normalized = tuple(str(name).strip() for name in names)
    if len(normalized) != ARM_DOF:
        raise ValueError(f"{label} must contain exactly {ARM_DOF} names")
    if any(not name for name in normalized):
        raise ValueError(f"{label} contains an empty name")
    if len(set(normalized)) != len(normalized):
        raise ValueError(f"{label} contains duplicate names")
    return normalized


def map_named_positions(
    names: Sequence[str], positions: Sequence[float], required_names: Sequence[str]
) -> tuple[float, ...]:
    """Map JointState-like arrays by name, never by incoming array order."""

    required = validate_joint_names(required_names, "required_names")
    if len(names) != len(set(names)):
        raise ValueError("joint feedback contains duplicate names")
    mapping = {
        str(name): float(positions[index])
        for index, name in enumerate(names)
        if index < len(positions)
    }
    missing = [name for name in required if name not in mapping]
    if missing:
        raise ValueError(f"joint feedback is missing: {', '.join(missing)}")
    return _float_tuple((mapping[name] for name in required), ARM_DOF, "joint feedback")


def xyzw_pose_to_openarm(pose_xyzw: Sequence[float]) -> tuple[float, ...]:
    """Convert ``[px,py,pz,qx,qy,qz,qw]`` to official OpenArm ``[p,qw,qx,qy,qz]``.

    The quaternion is normalized. A zero or non-finite quaternion is rejected
    instead of being silently forwarded into the IK solver.
    """

    pose = _float_tuple(pose_xyzw, 7, "pose_xyzw")
    px, py, pz, qx, qy, qz, qw = pose
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("pose quaternion has zero norm")
    return (px, py, pz, qw / norm, qx / norm, qy / norm, qz / norm)


def build_driver_state(
    right_arm: Sequence[float],
    right_gripper: float,
    left_arm: Sequence[float],
    left_gripper: float,
) -> tuple[float, ...]:
    """Build the official 16-value state: ``right[8] + left[8]``."""

    right = _float_tuple(right_arm, ARM_DOF, "right_arm")
    left = _float_tuple(left_arm, ARM_DOF, "left_arm")
    grippers = _float_tuple((right_gripper, left_gripper), 2, "grippers")
    return right + (grippers[0],) + left + (grippers[1],)


def split_driver_state(values: Sequence[float]) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Split an official IK result into right and left eight-value segments."""

    state = _float_tuple(values, BIMANUAL_DRIVER_DOF, "driver_state")
    return state[:DRIVER_DOF], state[DRIVER_DOF:]


def _intent_token_pair(tokens: Sequence[int], label: str) -> tuple[int, int]:
    try:
        pair = tuple(tokens)
    except TypeError as exc:
        raise ValueError(f"{label} must contain exactly two intent tokens") from exc
    if len(pair) != INTENT_TOKEN_COUNT:
        raise ValueError(f"{label} must contain exactly two intent tokens")
    right_token, left_token = pair
    decode_intent_token(right_token)
    decode_intent_token(left_token)
    return right_token, left_token


@dataclass(frozen=True)
class TargetHistorySample:
    """One bounded, token-pair-local atomic target observation.

    ``source_epoch_ns`` preserves the Quest packet epoch carried on the wire,
    while ``receipt_ns`` is the IK process's monotonic receipt time used only
    for freshness.  Binding the complete right/left token pair at receipt lets
    the consumer discard the whole history on either side's intent edge.
    """

    pose: tuple[float, ...]
    receipt_ns: int
    source_epoch_ns: int
    intent_tokens: tuple[int, int]

    def __post_init__(self) -> None:
        canonical_pose = _float_tuple(self.pose, ARM_DOF, "target history pose")
        for value, label in (
            (self.receipt_ns, "target history receipt_ns"),
            (self.source_epoch_ns, "target history source_epoch_ns"),
        ):
            if type(value) is not int or value <= 0:
                raise ValueError(f"{label} must be a positive integer")
        canonical_tokens = _intent_token_pair(
            self.intent_tokens, "target history intent tokens"
        )
        object.__setattr__(self, "pose", canonical_pose)
        object.__setattr__(self, "intent_tokens", canonical_tokens)


def _bounded_target_history(
    history: Sequence[TargetHistorySample], label: str
) -> tuple[TargetHistorySample, ...]:
    """Canonicalize one fixed-capacity history without consuming generators."""

    try:
        count = len(history)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a sized target history") from exc
    if count < 0 or count > TARGET_HISTORY_CAPACITY:
        raise ValueError(
            f"{label} exceeds the fixed {TARGET_HISTORY_CAPACITY}-sample capacity"
        )
    try:
        entries = tuple(history)
    except TypeError as exc:
        raise ValueError(f"{label} must be iterable") from exc
    if len(entries) != count:
        raise ValueError(f"{label} changed while it was being inspected")

    canonical = []
    for entry in entries:
        if not isinstance(entry, TargetHistorySample):
            raise ValueError(f"{label} contains a non-TargetHistorySample entry")
        # Revalidate instead of trusting a forged/bypassed dataclass instance.
        canonical.append(
            TargetHistorySample(
                pose=entry.pose,
                receipt_ns=entry.receipt_ns,
                source_epoch_ns=entry.source_epoch_ns,
                intent_tokens=entry.intent_tokens,
            )
        )
    return tuple(canonical)


def select_coherent_target_history_pair(
    right_history: Sequence[TargetHistorySample],
    left_history: Sequence[TargetHistorySample],
    *,
    now_ns: int,
    timeout_sec: float,
    max_skew_ns: int,
    current_intent_tokens: Sequence[int],
    last_selected_epoch_ns: Sequence[int | None] = (None, None),
) -> tuple[TargetHistorySample, TargetHistorySample] | None:
    """Select the newest fresh, non-regressing dual-target source pair.

    Histories are deliberately small and fixed-capacity.  A sample is eligible
    only when its locally bound dual-token pair is current, its receipt is
    fresh, and its source epoch does not precede that side's last selected
    epoch.  Equality is allowed so a stable Quest packet may be reused.  Of all
    pairs inside ``max_skew_ns``, prefer the newest common source epoch, then
    the newer peer epoch and monotonic receipts.  A valid-but-unpairable set
    returns ``None``; malformed parameters raise and therefore fail closed.
    """

    if type(now_ns) is not int or now_ns <= 0:
        raise ValueError("target history now_ns must be a positive integer")
    if isinstance(timeout_sec, bool):
        raise ValueError("target history timeout_sec must be finite and positive")
    try:
        timeout = float(timeout_sec)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "target history timeout_sec must be finite and positive"
        ) from exc
    if not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("target history timeout_sec must be finite and positive")
    if type(max_skew_ns) is not int or max_skew_ns < 0:
        raise ValueError("target history max_skew_ns must be a non-negative integer")

    intent_tokens = _intent_token_pair(
        current_intent_tokens, "current target history intent tokens"
    )
    try:
        last_epochs = tuple(last_selected_epoch_ns)
    except TypeError as exc:
        raise ValueError(
            "last_selected_epoch_ns must contain exactly two epochs"
        ) from exc
    if len(last_epochs) != INTENT_TOKEN_COUNT:
        raise ValueError("last_selected_epoch_ns must contain exactly two epochs")
    canonical_last_epochs = []
    for epoch in last_epochs:
        if epoch is None:
            canonical_last_epochs.append(0)
        elif type(epoch) is int and epoch >= 0:
            canonical_last_epochs.append(epoch)
        else:
            raise ValueError(
                "last_selected_epoch_ns values must be non-negative integers or None"
            )

    histories = (
        _bounded_target_history(right_history, "right target history"),
        _bounded_target_history(left_history, "left target history"),
    )
    eligible: list[tuple[TargetHistorySample, ...]] = []
    for side_index, history in enumerate(histories):
        minimum_epoch = canonical_last_epochs[side_index]
        eligible.append(
            tuple(
                sample
                for sample in history
                if sample.intent_tokens == intent_tokens
                and sample.source_epoch_ns >= minimum_epoch
                and is_fresh(now_ns, sample.receipt_ns, timeout)
            )
        )
    if not eligible[0] or not eligible[1]:
        return None

    candidates = (
        (right, left)
        for right in eligible[0]
        for left in eligible[1]
        if abs(right.source_epoch_ns - left.source_epoch_ns) <= max_skew_ns
    )
    return max(
        candidates,
        key=lambda pair: (
            min(pair[0].source_epoch_ns, pair[1].source_epoch_ns),
            max(pair[0].source_epoch_ns, pair[1].source_epoch_ns),
            min(pair[0].receipt_ns, pair[1].receipt_ns),
            max(pair[0].receipt_ns, pair[1].receipt_ns),
        ),
        default=None,
    )


def _intent_token_from_float64(value: float, label: str) -> int:
    token = _exact_integer_from_float64(value, label)
    decode_intent_token(token)
    return token


def _exact_integer_from_float64(value: float, label: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{label} must be an exact integer")
    if type(value) is int:
        integer = value
    else:
        try:
            numeric = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{label} must be an exact integer") from exc
        if not math.isfinite(numeric) or not numeric.is_integer():
            raise ValueError(f"{label} must be an exact integer")
        integer = int(numeric)
    if abs(integer) > MAX_EXACT_FLOAT64_INT:
        raise OverflowError(f"{label} exceeds the exact Float64 integer range")
    return integer


def _parse_intent_token_pair(
    values: Sequence[float], label: str
) -> tuple[int, int]:
    if len(values) != INTENT_TOKEN_COUNT:
        raise ValueError(f"{label} must contain exactly two intent tokens")
    return (
        _intent_token_from_float64(values[0], f"{label} right token"),
        _intent_token_from_float64(values[1], f"{label} left token"),
    )


def build_atomic_target(
    pose_xyzw: Sequence[float],
    stamp_sec: int,
    stamp_nanosec: int,
    intent_token: int,
) -> tuple[float, ...]:
    """Build normalized ``pose_xyzw[7] + sec + nanosec + TELEOP token``."""

    official_pose = xyzw_pose_to_openarm(pose_xyzw)
    for value, label in (
        (stamp_sec, "target stamp sec"),
        (stamp_nanosec, "target stamp nanosec"),
    ):
        if type(value) is not int:
            raise ValueError(f"{label} must be an integer")
        if abs(value) > MAX_EXACT_FLOAT64_INT:
            raise OverflowError(f"{label} exceeds the exact Float64 integer range")
    if stamp_sec < 0:
        raise ValueError("target stamp sec must be non-negative")
    if not 0 <= stamp_nanosec < 1_000_000_000:
        raise ValueError("target stamp nanosec must be in [0, 1000000000)")
    epoch_ns = stamp_sec * 1_000_000_000 + stamp_nanosec
    if epoch_ns <= 0:
        raise ValueError("target source epoch must be positive")
    _, mode = decode_intent_token(intent_token)
    if mode != SideMode.TELEOP:
        raise ValueError("atomic target intent token must encode TELEOP mode")

    # ``xyzw_pose_to_openarm`` performs the strict finite/non-zero quaternion
    # validation and normalization. Convert its canonical wxyz result back to
    # the ROS xyzw wire layout without recomputing the quaternion.
    normalized_xyzw = (
        official_pose[:3] + official_pose[4:] + (official_pose[3],)
    )
    return normalized_xyzw + (
        float(stamp_sec),
        float(stamp_nanosec),
        float(intent_token),
    )


def parse_atomic_target(
    values: Sequence[float],
) -> tuple[tuple[float, ...], int, int]:
    """Parse a 10-value target into official wxyz pose, source epoch, token."""

    payload = tuple(values)
    if len(payload) != ATOMIC_TARGET_DOF:
        raise ValueError(
            f"atomic target must contain exactly {ATOMIC_TARGET_DOF} values"
        )
    official_pose = xyzw_pose_to_openarm(payload[:ARM_DOF])
    stamp_sec = _exact_integer_from_float64(payload[ARM_DOF], "target stamp sec")
    stamp_nanosec = _exact_integer_from_float64(
        payload[ARM_DOF + 1], "target stamp nanosec"
    )
    if stamp_sec < 0:
        raise ValueError("target stamp sec must be non-negative")
    if not 0 <= stamp_nanosec < 1_000_000_000:
        raise ValueError("target stamp nanosec must be in [0, 1000000000)")
    epoch_ns = stamp_sec * 1_000_000_000 + stamp_nanosec
    if epoch_ns <= 0:
        raise ValueError("target source epoch must be positive")
    intent_token = _intent_token_from_float64(
        payload[ARM_DOF + 2], "atomic target intent token"
    )
    _, mode = decode_intent_token(intent_token)
    if mode != SideMode.TELEOP:
        raise ValueError("atomic target intent token must encode TELEOP mode")
    return official_pose, epoch_ns, intent_token


def build_guard_anchor(
    command: Sequence[float], intent_tokens: Sequence[int]
) -> tuple[float, ...]:
    """Build ``command[7] + (right_token, left_token)`` atomically."""

    canonical_command = _float_tuple(command, ARM_DOF, "guard controller anchor")
    tokens = _intent_token_pair(intent_tokens, "guard anchor intent tokens")
    return canonical_command + tuple(float(token) for token in tokens)


def parse_guard_anchor(
    values: Sequence[float],
) -> tuple[tuple[float, ...], tuple[int, int]]:
    """Parse a token-bound final-guard controller anchor."""

    payload = tuple(values)
    if len(payload) != GUARD_ANCHOR_DOF:
        raise ValueError(f"guard anchor must contain exactly {GUARD_ANCHOR_DOF} values")
    command = _float_tuple(payload[:ARM_DOF], ARM_DOF, "guard controller anchor")
    tokens = _parse_intent_token_pair(payload[ARM_DOF:], "guard anchor")
    return command, tokens


def build_atomic_ik_command(
    candidate: Sequence[float],
    solver_base: Sequence[float],
    intent_tokens: Sequence[int],
) -> tuple[float, ...]:
    """Build ``candidate[7] + solver_base[7] + right/left tokens``.

    ``solver_base`` is the exact seven-axis state from which the official IK
    cycle produced ``candidate``.  Keeping both halves in one ROS message
    prevents asynchronous joint feedback from changing the safety reference;
    the trailing exact tokens bind it to both intent sessions.
    """

    command = _float_tuple(candidate, ARM_DOF, "IK candidate")
    base = _float_tuple(solver_base, ARM_DOF, "IK solver base")
    tokens = _intent_token_pair(intent_tokens, "atomic IK intent tokens")
    return command + base + tuple(float(token) for token in tokens)


def parse_atomic_ik_command(
    values: Sequence[float],
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[int, int]]:
    """Parse the strict token-bound atomic IK contract."""

    payload = tuple(values)
    if len(payload) != ATOMIC_IK_COMMAND_DOF:
        raise ValueError(
            f"atomic IK command must contain exactly {ATOMIC_IK_COMMAND_DOF} values"
        )
    command = _float_tuple(payload[:ARM_DOF], ARM_DOF, "IK candidate")
    base = _float_tuple(
        payload[ARM_DOF : 2 * ARM_DOF], ARM_DOF, "IK solver base"
    )
    tokens = _parse_intent_token_pair(payload[2 * ARM_DOF :], "atomic IK command")
    return command, base, tokens


def build_guard_acceptance(
    accepted_command: Sequence[float],
    candidate: Sequence[float],
    solver_base: Sequence[float],
    intent_tokens: Sequence[int],
) -> tuple[float, ...]:
    """Atomically acknowledge the exact IK step accepted by the final guard.

    The echoed ``candidate``, ``solver_base``, and exact right/left intent
    tokens bind the accepted controller command to one outstanding IK payload.
    The solver may advance only after receiving this complete acknowledgement.
    """

    accepted = _float_tuple(accepted_command, ARM_DOF, "guard accepted command")
    command = _float_tuple(candidate, ARM_DOF, "guard echoed IK candidate")
    base = _float_tuple(solver_base, ARM_DOF, "guard echoed IK solver base")
    tokens = _intent_token_pair(intent_tokens, "guard acceptance intent tokens")
    return accepted + command + base + tuple(float(token) for token in tokens)


def parse_guard_acceptance(
    values: Sequence[float],
) -> tuple[
    tuple[float, ...],
    tuple[float, ...],
    tuple[float, ...],
    tuple[int, int],
]:
    """Parse the 23-value, dual-token guard acknowledgement atomically."""

    payload = tuple(values)
    if len(payload) != GUARD_ACCEPTANCE_DOF:
        raise ValueError(
            f"guard acceptance must contain exactly {GUARD_ACCEPTANCE_DOF} values"
        )
    accepted = _float_tuple(
        payload[:ARM_DOF], ARM_DOF, "guard accepted command"
    )
    command = _float_tuple(
        payload[ARM_DOF : 2 * ARM_DOF], ARM_DOF, "guard echoed IK candidate"
    )
    base = _float_tuple(
        payload[2 * ARM_DOF : 3 * ARM_DOF],
        ARM_DOF,
        "guard echoed IK solver base",
    )
    tokens = _parse_intent_token_pair(
        payload[3 * ARM_DOF :], "guard acceptance"
    )
    return (
        accepted,
        command,
        base,
        tokens,
    )


def is_fresh(now_ns: int, received_ns: int | None, timeout_sec: float) -> bool:
    """Return whether a monotonic receipt timestamp is still usable."""

    if received_ns is None or received_ns <= 0 or timeout_sec <= 0.0:
        return False
    age_ns = int(now_ns) - int(received_ns)
    return 0 <= age_ns <= int(timeout_sec * 1_000_000_000)


def target_only_commit_expiry_allows_sync_wait(
    *,
    intent_tokens_current: bool,
    selected_targets_token_bound: bool,
    selected_targets_fresh: bool,
    current_target_streams_fresh: bool,
    modes_current: bool,
    feedback_fresh: bool,
    anchors_current: bool,
) -> bool:
    """Allow HOLD only when an old selected pair's freshness alone expired.

    The selected pair must still be bound to the current intent pair, and each
    live target stream must independently contain a fresh exact-pair sample.
    Token, mode, feedback, anchor, or whole-stream loss therefore never enters
    this recovery path.
    """

    return (
        bool(intent_tokens_current)
        and bool(selected_targets_token_bound)
        and not bool(selected_targets_fresh)
        and bool(current_target_streams_fresh)
        and bool(modes_current)
        and bool(feedback_fresh)
        and bool(anchors_current)
    )


def is_post_engagement(received_ns: int | None, engagement_ns: int | None) -> bool:
    """Reject commands produced before the current deadman engagement."""

    return (
        received_ns is not None
        and engagement_ns is not None
        and received_ns >= engagement_ns
        and engagement_ns > 0
    )


def teleop_arming_within_deadline(
    now_ns: int,
    engagement_ns: int | None,
    timeout_sec: float,
) -> bool:
    """Bound the first committed IK command to its current Grip edge.

    The deadline is inclusive: a commit at exactly ``timeout_sec`` remains
    valid, while even one nanosecond beyond it is late.  Missing, future, or
    otherwise invalid engagement timestamps fail closed.
    """

    timeout = float(timeout_sec)
    if (
        engagement_ns is None
        or engagement_ns <= 0
        or not math.isfinite(timeout)
        or timeout <= 0.0
    ):
        return False
    age_ns = int(now_ns) - int(engagement_ns)
    return 0 <= age_ns <= int(timeout * 1_000_000_000)


def guard_teleop_ik_mode(
    teleop_established: bool,
    ik_after_engagement: bool,
    ik_fresh: bool,
    sync_wait_fresh: bool,
    arming_within_deadline: bool,
) -> SideMode:
    """Choose HOLD/TELEOP/FAULT for the guard's two-stage IK handshake.

    Cross-process mode, anchor, and IK callbacks may arrive in different
    orders.  Before this deadman engagement has committed its first validated
    IK command, missing IK therefore means safe HOLD only inside a bounded
    arming window.  Once established, the same condition is a fail-closed
    runtime FAULT.  A causally validated sync-wait lease always wins over a
    fresh IK sample, guaranteeing that synchronization never emits an arm
    command or acknowledgement.
    """

    if bool(sync_wait_fresh):
        return SideMode.HOLD
    if not bool(teleop_established) and not bool(arming_within_deadline):
        return SideMode.FAULT
    if bool(ik_after_engagement) and bool(ik_fresh):
        return SideMode.TELEOP
    if not bool(teleop_established):
        return SideMode.HOLD
    return SideMode.FAULT


def is_rclpy_humble_shutdown_take_race(
    error: BaseException,
    *,
    context_ok: bool,
) -> bool:
    """Recognize the pinned Humble subscription-take teardown race only.

    A generic ``RuntimeError`` must still terminate the safety process even if
    shutdown happens concurrently; otherwise its original cause is hidden.
    """

    return (
        not bool(context_ok)
        and type(error) is RuntimeError
        and str(error) == RCLPY_HUMBLE_SHUTDOWN_TAKE_RACE_MESSAGE
    )


def select_solver_target(
    mode: SideMode | int,
    teleop_target: Sequence[float] | None,
    hold_target: Sequence[float],
) -> tuple[float, ...]:
    """Choose a target for the shared bimanual solver.

    The official solver requires a target for both arms on every solve. Only a
    TELEOP side consumes its incoming target; HOLD, HOME, and FAULT use current
    FK as an internal hold target. The command guard still suppresses solver
    output for every non-TELEOP side.
    """

    requested = SideMode(int(mode))
    if requested == SideMode.TELEOP:
        if teleop_target is None:
            raise ValueError("TELEOP mode has no target")
        return _float_tuple(teleop_target, 7, "teleop_target")
    return _float_tuple(hold_target, 7, "hold_target")


def teleop_targets_are_coherent(
    right_mode: SideMode | int,
    left_mode: SideMode | int,
    right_epoch_ns: int | None,
    left_epoch_ns: int | None,
    max_skew_ns: int = 0,
) -> bool:
    """Require non-zero, tightly bounded Quest epochs only for dual TELEOP.

    Left and right intent nodes use independent ROS timers, so requiring exact
    stamp equality can starve forever when their phases straddle consecutive
    Quest packets.  A small explicit bound admits only neighboring source
    packets while still rejecting stale or arbitrarily mixed bimanual targets.
    """

    right = SideMode(int(right_mode))
    left = SideMode(int(left_mode))
    if right != SideMode.TELEOP or left != SideMode.TELEOP:
        return True
    if right_epoch_ns is None or left_epoch_ns is None:
        return False
    try:
        allowed_skew_ns = int(max_skew_ns)
    except (TypeError, ValueError):
        return False
    return (
        right_epoch_ns > 0
        and left_epoch_ns > 0
        and allowed_skew_ns >= 0
        and abs(int(right_epoch_ns) - int(left_epoch_ns)) <= allowed_skew_ns
    )


def step_toward(
    current: Sequence[float], target: Sequence[float], max_step: Sequence[float]
) -> tuple[float, ...]:
    """Take a component-wise bounded joint step."""

    current_values = _float_tuple(current, ARM_DOF, "current")
    target_values = _float_tuple(target, ARM_DOF, "target")
    step_values = _float_tuple(max_step, ARM_DOF, "max_step")
    if any(value <= 0.0 for value in step_values):
        raise ValueError("max_step values must be positive")
    result = []
    for value, goal, step in zip(current_values, target_values, step_values):
        delta = max(-step, min(goal - value, step))
        result.append(value + delta)
    return tuple(result)


def project_joint_positions(
    candidate: Sequence[float],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    max_projection_rad: float = JOINT_LIMIT_NUMERIC_TOLERANCE_RAD,
) -> tuple[float, ...]:
    """Project only numeric boundary residue onto the configured work area.

    Call this only after the raw official-IK step has passed its independent
    velocity check. A material violation is rejected rather than hidden by
    clamping; projection is not a substitute for constrained IK and does not
    make out-of-range measured feedback acceptable.
    """

    values = _float_tuple(candidate, ARM_DOF, "joint target")
    lower = _float_tuple(lower_limits, ARM_DOF, "lower joint limits")
    upper = _float_tuple(upper_limits, ARM_DOF, "upper joint limits")
    tolerance = float(max_projection_rad)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("max joint projection must be finite and non-negative")
    if any(lo >= hi for lo, hi in zip(lower, upper)):
        raise ValueError("each lower joint limit must be below its upper limit")
    projected = tuple(
        min(max(value, lo), hi) for value, lo, hi in zip(values, lower, upper)
    )
    for index, (raw, bounded) in enumerate(zip(values, projected)):
        excess = abs(raw - bounded)
        if excess > tolerance:
            raise ValueError(
                f"joint {index + 1} exceeds configured work area by "
                f"{excess:.9g}rad (numeric allowance {tolerance:.9g}rad)"
            )
    return projected


def apply_joint_soft_limits(
    candidate: Sequence[float],
    reference: Sequence[float],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
) -> tuple[float, ...]:
    """Saturate a velocity-checked command at the teleoperation soft limits.

    When ``reference`` is already inside the soft envelope, this is an ordinary
    clamp.  A real arm may, however, start legally inside the official hard
    range while lying just outside the narrower 95% teleoperation envelope. In
    that case the function never commands farther outward and never jumps to a
    boundary: it permits only the candidate's normal, velocity-bounded recovery
    toward the soft range.  Every returned component therefore remains on the
    closed segment between ``reference`` and ``candidate``.

    Call this only after validating the raw official-IK step against
    ``reference``.  The final command guard and hardware layer still enforce the
    independent 100% official hard limits.
    """

    values = _float_tuple(candidate, ARM_DOF, "joint target")
    current = _float_tuple(reference, ARM_DOF, "joint soft-limit reference")
    lower = _float_tuple(lower_limits, ARM_DOF, "lower joint soft limits")
    upper = _float_tuple(upper_limits, ARM_DOF, "upper joint soft limits")
    if any(lo >= hi for lo, hi in zip(lower, upper)):
        raise ValueError("each lower joint soft limit must be below its upper limit")

    bounded = []
    for value, base, lo, hi in zip(values, current, lower, upper):
        if base < lo:
            # Already below the soft range: hold or move inward, never farther
            # outward and never teleport directly to the boundary.
            permitted_lower, permitted_upper = base, hi
        elif base > hi:
            permitted_lower, permitted_upper = lo, base
        else:
            permitted_lower, permitted_upper = lo, hi
        bounded.append(min(max(value, permitted_lower), permitted_upper))
    return tuple(bounded)


def select_solver_base(
    *,
    mode: SideMode,
    feedback: Sequence[float],
    accepted_command: Sequence[float] | None,
    controller_anchor: Sequence[float] | None = None,
) -> tuple[float, ...]:
    """Select the official IK state for one control cycle.

    A continuously engaged TELEOP session may advance only from the exact
    controller command acknowledged by the final guard.  A fresh engagement
    starts from a post-engagement anchor published by that same final guard,
    rather than from feedback that can legitimately lag the retained position
    controller command.  This keeps the solver and command shaper on one exact
    base while preserving command continuity across deadman release/re-engage.
    """

    measured = _float_tuple(feedback, ARM_DOF, "solver feedback")
    try:
        current = SideMode(int(mode))
    except (TypeError, ValueError) as exc:
        raise ValueError("solver mode must be a valid SideMode value") from exc
    if current == SideMode.TELEOP and accepted_command is not None:
        return _float_tuple(accepted_command, ARM_DOF, "guard accepted command")
    if current == SideMode.TELEOP and controller_anchor is not None:
        return _float_tuple(controller_anchor, ARM_DOF, "guard controller anchor")
    return measured


def solver_handshake_reset_required(
    previous_mode: SideMode,
    new_mode: SideMode,
) -> bool:
    """Return whether a raw mode edge invalidates one side's IK handshake.

    Mode callbacks can observe a Grip release and re-press between two IK
    timer ticks. Reset on either TELEOP edge so a new engagement cannot inherit
    a pending payload or accepted command from the previous engagement.
    """

    return previous_mode != new_mode and (
        previous_mode == SideMode.TELEOP or new_mode == SideMode.TELEOP
    )


def shape_arm_command(
    *,
    current_command: Sequence[float],
    target: Sequence[float],
    feedback: Sequence[float],
    max_step: Sequence[float],
    max_tracking_error: float | Sequence[float],
) -> tuple[float, ...]:
    """Rate-limit an arm command inside a feedback-relative soft envelope.

    The normal controller step is intersected with ``feedback +/-
    (tracking_limit - one_step)`` for each joint.  Reserving one complete
    control-cycle step keeps the 100 Hz guard from handing the 100 Hz hardware
    loop a command that crosses the per-axis tracking hard gate.  Motion back
    toward feedback remains available, so backpressure clears as soon as the
    motor catches up.
    """

    current = _float_tuple(current_command, ARM_DOF, "current arm command")
    goal = _float_tuple(target, ARM_DOF, "arm target")
    measured = _float_tuple(feedback, ARM_DOF, "arm feedback")
    step = _float_tuple(max_step, ARM_DOF, "max_step")
    hard_limits = _tracking_error_tuple(max_tracking_error)
    if any(not math.isfinite(value) or value <= 0.0 for value in hard_limits):
        raise ValueError("arm tracking hard gates must be finite and positive")
    if any(
        value <= 0.0 or value >= hard_limit
        for value, hard_limit in zip(step, hard_limits)
    ):
        raise ValueError("each arm max_step must be positive and below the tracking gate")

    desired = step_toward(current, goal, step)
    shaped = []
    for index, (command, requested, actual, allowed_step, hard_limit) in enumerate(
        zip(current, desired, measured, step, hard_limits)
    ):
        if abs(command - actual) > hard_limit:
            raise ValueError(
                f"joint {index + 1} current command exceeds the tracking hard gate"
            )
        soft_limit = hard_limit - allowed_step
        lower = max(command - allowed_step, actual - soft_limit)
        upper = min(command + allowed_step, actual + soft_limit)
        if lower > upper:
            raise ValueError(
                f"joint {index + 1} has no safe command/tracking interval"
            )
        shaped.append(max(lower, min(requested, upper)))
    return tuple(shaped)


def shape_monotonic_guard_command(
    *,
    current_command: Sequence[float],
    monotonic_base: Sequence[float] | None = None,
    target: Sequence[float],
    feedback: Sequence[float],
    max_step: Sequence[float],
    max_tracking_error: float | Sequence[float],
) -> tuple[float, ...]:
    """Shape a controller command without leaving the requested IK segment.

    ``shape_arm_command`` may deliberately move a lagging command back toward
    feedback when its tracking reserve is exhausted.  That behavior is useful
    in isolation, but it cannot be acknowledged as progress for an atomic IK
    round because it lies behind the solver base.  The final command guard uses
    this variant: it projects the shaped value onto the closed solver-base ->
    ``target`` segment.  ``monotonic_base`` can carry the exact float32 base
    from the atomic IK payload when it differs by a few representation bits
    from the guard's float64 retained command.  When outward progress is not
    yet safe, that canonical base is held until feedback catches up.
    """

    current = _float_tuple(current_command, ARM_DOF, "current arm command")
    goal = _float_tuple(target, ARM_DOF, "arm target")
    segment_start = (
        current
        if monotonic_base is None
        else _float_tuple(monotonic_base, ARM_DOF, "monotonic solver base")
    )
    shaped = shape_arm_command(
        current_command=current,
        target=goal,
        feedback=feedback,
        max_step=max_step,
        max_tracking_error=max_tracking_error,
    )
    return tuple(
        min(max(value, min(start, end)), max(start, end))
        for value, start, end in zip(shaped, segment_start, goal)
    )


def validate_solver_candidate_step(
    candidate: Sequence[float],
    solver_base: Sequence[float],
    max_step: Sequence[float],
) -> GuardDecision:
    """Validate raw official-IK motion against its exact solver base.

    The solver operates in float32, so the existing representation-scale
    tolerance is accepted only by clamping back to the exact configured step.
    Any larger excess is a hard fault and must not be hidden by downstream
    rate limiting against the controller's previous command.
    """

    try:
        command = _float_tuple(candidate, ARM_DOF, "IK candidate")
        base = _float_tuple(solver_base, ARM_DOF, "IK solver base")
        step = _float_tuple(max_step, ARM_DOF, "max_step")
    except (TypeError, ValueError) as exc:
        return GuardDecision(False, None, str(exc))
    if any(value <= 0.0 for value in step):
        return GuardDecision(False, None, "max_step values must be positive")

    bounded = list(command)
    for index, (value, reference, allowed) in enumerate(zip(command, base, step)):
        requested_step = abs(value - reference)
        if requested_step <= allowed:
            continue
        if requested_step - allowed <= JOINT_STEP_NUMERIC_TOLERANCE_RAD:
            bounded[index] = reference + math.copysign(allowed, value - reference)
            continue
        return GuardDecision(
            False,
            None,
            f"joint {index + 1} solver step {requested_step:.9g} exceeds limit "
            f"{allowed:.9g} (candidate={value:.9g}, solver_base={reference:.9g})",
        )
    return GuardDecision(True, tuple(bounded), "ok")


def validate_guard_acceptance_progress(
    *,
    accepted_command: Sequence[float],
    candidate: Sequence[float],
    solver_base: Sequence[float],
    max_step: Sequence[float],
) -> GuardDecision:
    """Validate one guard ACK against the exact pending solver segment.

    Both the final command guard and the IK consumer use this function.  The
    guard therefore cannot publish/cache an acknowledgement that the IK must
    later reject as over-step or non-monotonic.
    """

    decision = validate_solver_candidate_step(
        accepted_command,
        solver_base,
        max_step,
    )
    if not decision.accepted or decision.command is None:
        return decision
    try:
        target = _float_tuple(candidate, ARM_DOF, "IK candidate")
        base = _float_tuple(solver_base, ARM_DOF, "IK solver base")
    except (TypeError, ValueError) as exc:
        return GuardDecision(False, None, str(exc))
    segment_tolerance = 1e-9
    for index, (value, start, goal) in enumerate(
        zip(decision.command, base, target)
    ):
        if (
            value < min(start, goal) - segment_tolerance
            or value > max(start, goal) + segment_tolerance
        ):
            return GuardDecision(
                False,
                None,
                f"joint {index + 1} guard acceptance is outside the "
                "solver_base-to-candidate segment",
            )
    return GuardDecision(True, decision.command, "ok")


def validate_guard_anchor(
    *,
    anchor: Sequence[float],
    feedback: Sequence[float],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    max_tracking_error: float | Sequence[float],
) -> GuardDecision:
    """Validate a final-guard controller anchor for fresh TELEOP takeover."""

    try:
        command = _float_tuple(anchor, ARM_DOF, "guard controller anchor")
        measured = _float_tuple(feedback, ARM_DOF, "feedback")
        lower = _float_tuple(lower_limits, ARM_DOF, "lower_limits")
        upper = _float_tuple(upper_limits, ARM_DOF, "upper_limits")
        tracking = _tracking_error_tuple(max_tracking_error)
    except (TypeError, ValueError) as exc:
        return GuardDecision(False, None, str(exc))
    if any(lo >= hi for lo, hi in zip(lower, upper)):
        return GuardDecision(False, None, "each lower limit must be below its upper limit")
    if any(value <= 0.0 or not math.isfinite(value) for value in tracking):
        return GuardDecision(False, None, "tracking limits must be finite and positive")
    for index, (value, actual, lo, hi, tracking_limit) in enumerate(
        zip(command, measured, lower, upper, tracking)
    ):
        if value < lo or value > hi:
            return GuardDecision(
                False,
                None,
                f"joint {index + 1} guard anchor exceeds configured hard limits",
            )
        if abs(value - actual) > tracking_limit:
            return GuardDecision(
                False,
                None,
                f"joint {index + 1} guard anchor exceeds the tracking hard gate",
            )
    return GuardDecision(True, command, "ok")


def gripper_target_from_normalized(
    normalized_opening: float,
    lower: float = GRIPPER_MIN_POSITION_M,
    upper: float = GRIPPER_MAX_POSITION_M,
) -> float:
    """Map Quest ``0=closed, 1=open`` intent to the official ROS joint."""

    value = float(normalized_opening)
    lo = float(lower)
    hi = float(upper)
    if not all(math.isfinite(item) for item in (value, lo, hi)):
        raise ValueError("gripper intent and limits must be finite")
    if lo >= hi:
        raise ValueError("gripper lower limit must be below upper limit")
    if not 0.0 <= value <= 1.0:
        raise ValueError("normalized gripper intent must be in [0, 1]")
    return lo + value * (hi - lo)


def update_gripper_rearm_settle(
    *,
    now_ns: int,
    endpoint_ready: bool,
    settle_started_ns: int | None,
    settle_sec: float,
) -> tuple[bool, int | None]:
    """Track a continuously-ready startup/re-arm gripper endpoint window."""

    if not isinstance(now_ns, int) or now_ns < 0:
        raise ValueError("gripper rearm settle time must be a non-negative integer")
    duration_sec = float(settle_sec)
    if not math.isfinite(duration_sec) or duration_sec <= 0.0:
        raise ValueError("gripper rearm settle duration must be positive")
    if settle_started_ns is not None and (
        not isinstance(settle_started_ns, int)
        or settle_started_ns < 0
        or settle_started_ns > now_ns
    ):
        raise ValueError("gripper rearm settle start time is invalid")
    if not endpoint_ready:
        return False, None
    started_ns = now_ns if settle_started_ns is None else settle_started_ns
    settled = now_ns - started_ns >= int(duration_sec * 1_000_000_000)
    return settled, started_ns


def step_gripper_toward(
    current: float,
    target: float,
    max_open_step: float,
    max_close_step: float,
) -> float:
    """Apply direction-specific ID8 limits in the ROS prismatic coordinate.

    Increasing position opens the gripper; decreasing position closes it.
    """

    values = tuple(
        float(value)
        for value in (current, target, max_open_step, max_close_step)
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("gripper step values must be finite")
    current_value, target_value, open_allowed, close_allowed = values
    if open_allowed <= 0.0 or close_allowed <= 0.0:
        raise ValueError("gripper open/close max steps must be positive")
    allowed = open_allowed if target_value > current_value else close_allowed
    return current_value + max(-allowed, min(target_value - current_value, allowed))


def shape_gripper_command(
    *,
    current_command: float,
    target: float,
    feedback: float,
    max_open_step: float,
    max_close_step: float,
    max_tracking_error: float,
) -> float:
    """Direction-rate-limit ID8 and reserve the next cycle's tracking room.

    The hardware checks command-to-feedback tracking at a much higher rate than
    the ROS guard.  Motion away from feedback keeps an adaptive same-direction
    reserve: the smaller of one command step and the room left beyond that step
    inside the unchanged hard gate.  This permits one full high-rate step from
    caught-up feedback, then holds until feedback follows.  If an existing legal
    command is already outside that soft envelope, the command retreats toward
    feedback at the rate of the actual recovery direction.
    """

    current, goal, measured, open_allowed, close_allowed, tracking_limit = tuple(
        float(value)
        for value in (
            current_command,
            target,
            feedback,
            max_open_step,
            max_close_step,
            max_tracking_error,
        )
    )
    if not all(
        math.isfinite(value)
        for value in (
            current,
            goal,
            measured,
            open_allowed,
            close_allowed,
            tracking_limit,
        )
    ):
        raise ValueError("gripper shaping values must be finite")
    if open_allowed <= 0.0 or close_allowed <= 0.0:
        raise ValueError("gripper open/close max steps must be positive")
    if tracking_limit <= max(open_allowed, close_allowed):
        raise ValueError("gripper tracking limit must exceed both command steps")
    if abs(current - measured) > tracking_limit:
        raise ValueError("current gripper command exceeds the tracking hard gate")

    open_reserve = min(open_allowed, tracking_limit - open_allowed)
    close_reserve = min(close_allowed, tracking_limit - close_allowed)
    if goal > current:
        normal = min(goal, current + open_allowed)
        opening_soft_upper = measured + tracking_limit - open_reserve
        if current <= opening_soft_upper:
            return min(normal, opening_soft_upper)
        # Opening would enlarge an already near-limit positive tracking error.
        return max(current - close_allowed, opening_soft_upper)
    if goal < current:
        normal = max(goal, current - close_allowed)
        closing_soft_lower = measured - tracking_limit + close_reserve
        if current >= closing_soft_lower:
            return max(normal, closing_soft_lower)
        # Closing would enlarge an already near-limit negative tracking error.
        return min(current + open_allowed, closing_soft_lower)
    return current


def shape_gripper_reversal_brake_command(
    *,
    current_command: float,
    feedback: float,
    max_open_step: float,
    max_close_step: float,
    max_tracking_error: float,
) -> float:
    """Rate-limit a reversal command back to feedback without crossing it."""

    current = float(current_command)
    measured = float(feedback)
    return shape_gripper_command(
        current_command=current,
        target=measured,
        feedback=measured,
        max_open_step=max_open_step,
        max_close_step=max_close_step,
        max_tracking_error=max_tracking_error,
    )


def validate_gripper_command(
    *,
    candidate: float,
    feedback: float,
    previous_command: float | None,
    lower_limit: float,
    upper_limit: float,
    max_open_step: float,
    max_close_step: float,
    max_tracking_error: float,
) -> GuardDecision:
    """Fail-closed validation for one OpenArm v1 ID8 ROS joint command."""

    values = tuple(
        float(value)
        for value in (
            candidate,
            feedback,
            lower_limit,
            upper_limit,
            max_open_step,
            max_close_step,
            max_tracking_error,
        )
    )
    if not all(math.isfinite(value) for value in values):
        return GuardDecision(False, None, "gripper command contract contains non-finite data")
    (
        command,
        measured,
        lower,
        upper,
        open_allowed,
        close_allowed,
        tracking_limit,
    ) = values
    if (
        lower >= upper
        or open_allowed <= 0.0
        or close_allowed <= 0.0
        or tracking_limit <= 0.0
    ):
        return GuardDecision(False, None, "gripper limits must be finite and positive")
    if command < lower or command > upper:
        return GuardDecision(False, None, "gripper command exceeds configured limits")
    if measured < lower or measured > upper:
        return GuardDecision(False, None, "gripper feedback exceeds configured limits")
    previous = measured if previous_command is None else float(previous_command)
    if not math.isfinite(previous):
        return GuardDecision(False, None, "previous gripper command is non-finite")
    if previous_command is not None and abs(measured - previous) > tracking_limit:
        return GuardDecision(False, None, "gripper tracking error exceeds configured limit")
    if abs(command - measured) > tracking_limit:
        return GuardDecision(
            False,
            None,
            "gripper candidate tracking error exceeds configured limit",
        )
    requested_delta = command - previous
    requested_step = abs(requested_delta)
    allowed = open_allowed if requested_delta > 0.0 else close_allowed
    numeric_tolerance = max(1e-12, allowed * 1e-6)
    if requested_step > allowed + numeric_tolerance:
        return GuardDecision(
            False,
            None,
            f"gripper step {requested_step:.9g} exceeds limit {allowed:.9g}",
        )
    if requested_step > allowed:
        command = previous + math.copysign(allowed, command - previous)
    return GuardDecision(True, (command,), "ok")


def validate_guard_command(
    candidate: Sequence[float],
    feedback: Sequence[float],
    previous_command: Sequence[float] | None,
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    max_step: Sequence[float],
    max_tracking_error: float | Sequence[float],
) -> GuardDecision:
    """Fail-closed validation for a forward-position command."""

    try:
        command = _float_tuple(candidate, ARM_DOF, "candidate")
        measured = _float_tuple(feedback, ARM_DOF, "feedback")
        lower = _float_tuple(lower_limits, ARM_DOF, "lower_limits")
        upper = _float_tuple(upper_limits, ARM_DOF, "upper_limits")
        step = _float_tuple(max_step, ARM_DOF, "max_step")
        tracking_limits = _tracking_error_tuple(max_tracking_error)
        previous = (
            None
            if previous_command is None
            else _float_tuple(previous_command, ARM_DOF, "previous_command")
        )
    except (TypeError, ValueError) as exc:
        return GuardDecision(False, None, str(exc))

    if any(value <= 0.0 or not math.isfinite(value) for value in tracking_limits):
        return GuardDecision(
            False, None, "max_tracking_error values must be finite and positive"
        )
    if any(lo >= hi for lo, hi in zip(lower, upper)):
        return GuardDecision(False, None, "each lower limit must be below its upper limit")
    if any(value <= 0.0 for value in step):
        return GuardDecision(False, None, "max_step values must be positive")

    bounded_command = list(command)
    for index, (value, lo, hi) in enumerate(zip(command, lower, upper)):
        # The official solver works in float32 and can return sub-microradian
        # residue just beyond an active joint bound. Clamp only representation
        # noise; any mechanically meaningful violation remains a hard fault.
        limit_tolerance = 1e-6
        if value < lo - limit_tolerance or value > hi + limit_tolerance:
            return GuardDecision(
                False,
                None,
                f"joint {index + 1} value {value:.9g} exceeds configured limits "
                f"[{lo:.9g}, {hi:.9g}]",
            )
        bounded_command[index] = min(max(value, lo), hi)
    command = tuple(bounded_command)

    reference = measured if previous is None else previous
    if previous is not None:
        for index, (actual, expected, tracking_limit) in enumerate(
            zip(measured, previous, tracking_limits)
        ):
            if abs(actual - expected) > tracking_limit:
                return GuardDecision(
                    False, None, f"joint {index + 1} tracking error exceeds limit"
                )

    bounded_step_command = list(command)
    for index, (value, base, allowed) in enumerate(zip(command, reference, step)):
        requested_step = abs(value - base)
        if requested_step <= allowed:
            continue

        # openarm-control solves in float32 and its QP can overshoot an active
        # velocity boundary by a few microradians.  Never publish that excess:
        # representation-scale residue is clamped back to the exact configured
        # step, while every mechanically meaningful excess remains a hard fault.
        if requested_step - allowed <= JOINT_STEP_NUMERIC_TOLERANCE_RAD:
            bounded_step_command[index] = base + math.copysign(allowed, value - base)
            continue
        return GuardDecision(
            False,
            None,
            f"joint {index + 1} step {requested_step:.9g} exceeds limit "
            f"{allowed:.9g} (candidate={value:.9g}, reference={base:.9g})",
        )

    return GuardDecision(True, tuple(bounded_step_command), "ok")
