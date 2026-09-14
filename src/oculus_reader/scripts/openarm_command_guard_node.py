#!/usr/bin/env python3
"""Final fail-closed command gate for OpenArm ros2_control position controllers."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64MultiArray, UInt64, UInt8

from openarm_teleop_core import (
    GRIPPER_CLOSE_VELOCITY_M_S,
    GRIPPER_OPEN_POSITION_M,
    GRIPPER_OPEN_VELOCITY_M_S,
    GripperReversalInterlock,
    LEFT_JOINT_NAMES,
    OPENARM_LEFT_JOINT_HARD_LOWER_RAD,
    OPENARM_LEFT_JOINT_HARD_UPPER_RAD,
    OPENARM_RIGHT_JOINT_HARD_LOWER_RAD,
    OPENARM_RIGHT_JOINT_HARD_UPPER_RAD,
    OPENARM_TELEOP_CONTROL_RATE_HZ,
    OPENARM_TELEOP_MAX_JOINT_VELOCITY_RAD_S,
    OPENARM_TELEOP_TRACKING_ERROR_RAD,
    RIGHT_JOINT_NAMES,
    SideMode,
    build_guard_anchor,
    build_guard_acceptance,
    decode_intent_token,
    gripper_target_from_normalized,
    guard_teleop_ik_mode,
    is_fresh,
    is_post_engagement,
    is_rclpy_humble_shutdown_take_race,
    map_named_positions,
    parse_atomic_ik_command,
    shape_monotonic_guard_command,
    shape_gripper_command,
    shape_gripper_reversal_brake_command,
    step_gripper_toward,
    step_toward,
    teleop_arming_within_deadline,
    update_gripper_rearm_settle,
    validate_gripper_command,
    validate_guard_acceptance_progress,
    validate_guard_command,
    validate_joint_names,
    validate_solver_candidate_step,
)


DEFAULT_VELOCITY_CAPS = list(OPENARM_TELEOP_MAX_JOINT_VELOCITY_RAD_S)
DEFAULT_TRACKING_LIMITS = list(OPENARM_TELEOP_TRACKING_ERROR_RAD)
DEFAULT_LEFT_LOWER = list(OPENARM_LEFT_JOINT_HARD_LOWER_RAD)
DEFAULT_LEFT_UPPER = list(OPENARM_LEFT_JOINT_HARD_UPPER_RAD)
DEFAULT_RIGHT_LOWER = list(OPENARM_RIGHT_JOINT_HARD_LOWER_RAD)
DEFAULT_RIGHT_UPPER = list(OPENARM_RIGHT_JOINT_HARD_UPPER_RAD)


class OpenArmCommandGuard(Node):
    """The only node authorized to publish arm controller commands."""

    def __init__(self):
        super().__init__("openarm_command_guard")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("right_ik_topic", "/openarm/ik/right")
        self.declare_parameter("left_ik_topic", "/openarm/ik/left")
        self.declare_parameter("right_gripper_intent_topic", "/openarm/gripper_intent/right")
        self.declare_parameter("left_gripper_intent_topic", "/openarm/gripper_intent/left")
        self.declare_parameter("right_mode_topic", "/openarm/mode/right")
        self.declare_parameter("left_mode_topic", "/openarm/mode/left")
        self.declare_parameter(
            "right_command_topic", "/right_forward_position_controller/commands"
        )
        self.declare_parameter(
            "left_command_topic", "/left_forward_position_controller/commands"
        )
        self.declare_parameter(
            "right_gripper_command_topic",
            "/right_gripper_forward_position_controller/commands",
        )
        self.declare_parameter(
            "left_gripper_command_topic",
            "/left_gripper_forward_position_controller/commands",
        )
        self.declare_parameter("right_guard_mode_topic", "/openarm/guard_mode/right")
        self.declare_parameter("left_guard_mode_topic", "/openarm/guard_mode/left")
        self.declare_parameter(
            "right_guard_acceptance_topic", "/openarm/guard_acceptance/right"
        )
        self.declare_parameter(
            "left_guard_acceptance_topic", "/openarm/guard_acceptance/left"
        )
        self.declare_parameter("right_guard_anchor_topic", "/openarm/guard_anchor/right")
        self.declare_parameter("left_guard_anchor_topic", "/openarm/guard_anchor/left")
        self.declare_parameter("right_sync_wait_topic", "/openarm/ik_sync_wait/right")
        self.declare_parameter("left_sync_wait_topic", "/openarm/ik_sync_wait/left")
        self.declare_parameter("right_joint_names", list(RIGHT_JOINT_NAMES))
        self.declare_parameter("left_joint_names", list(LEFT_JOINT_NAMES))
        self.declare_parameter("right_gripper_joint_name", "openarm_right_finger_joint1")
        self.declare_parameter("left_gripper_joint_name", "openarm_left_finger_joint1")
        self.declare_parameter("right_home_positions", [0.0] * 7)
        self.declare_parameter("left_home_positions", [0.0] * 7)
        self.declare_parameter("right_lower_limits", DEFAULT_RIGHT_LOWER)
        self.declare_parameter("right_upper_limits", DEFAULT_RIGHT_UPPER)
        self.declare_parameter("left_lower_limits", DEFAULT_LEFT_LOWER)
        self.declare_parameter("left_upper_limits", DEFAULT_LEFT_UPPER)
        self.declare_parameter("right_max_velocity_rad_s", DEFAULT_VELOCITY_CAPS)
        self.declare_parameter("left_max_velocity_rad_s", DEFAULT_VELOCITY_CAPS)
        self.declare_parameter("control_rate_hz", OPENARM_TELEOP_CONTROL_RATE_HZ)
        self.declare_parameter("feedback_timeout_sec", 0.25)
        self.declare_parameter("ik_timeout_sec", 0.15)
        self.declare_parameter("sync_wait_max_sec", 0.75)
        self.declare_parameter("teleop_arming_timeout_sec", 0.75)
        self.declare_parameter("mode_timeout_sec", 0.25)
        self.declare_parameter("max_tracking_error_rad", DEFAULT_TRACKING_LIMITS)
        self.declare_parameter("fault_rearm_hold_sec", 0.35)
        self.declare_parameter("home_tolerance_rad", 0.02)
        self.declare_parameter("gripper_lower_limit_m", 0.0)
        self.declare_parameter("gripper_upper_limit_m", 0.044)
        self.declare_parameter("gripper_open_position_m", GRIPPER_OPEN_POSITION_M)
        self.declare_parameter(
            "gripper_open_velocity_m_s", GRIPPER_OPEN_VELOCITY_M_S
        )
        self.declare_parameter(
            "gripper_close_velocity_m_s", GRIPPER_CLOSE_VELOCITY_M_S
        )
        self.declare_parameter("gripper_max_tracking_error_m", 0.00504)
        self.declare_parameter("gripper_reversal_settle_samples", 3)
        self.declare_parameter(
            "gripper_reversal_feedback_motion_tolerance_m", 0.00002
        )
        self.declare_parameter("gripper_intent_timeout_sec", 0.30)
        self.declare_parameter("gripper_rearm_open_intent_min", 0.98)
        self.declare_parameter("right_gripper_rearm_open_tolerance_m", 0.0045)
        self.declare_parameter("left_gripper_rearm_open_tolerance_m", 0.0025)
        self.declare_parameter("gripper_rearm_open_settle_sec", 0.25)
        self.declare_parameter("gripper_rearm_open_timeout_sec", 5.0)

        self.control_rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.feedback_timeout_sec = float(self.get_parameter("feedback_timeout_sec").value)
        self.ik_timeout_sec = float(self.get_parameter("ik_timeout_sec").value)
        self.sync_wait_max_sec = float(
            self.get_parameter("sync_wait_max_sec").value
        )
        self.teleop_arming_timeout_sec = float(
            self.get_parameter("teleop_arming_timeout_sec").value
        )
        self.mode_timeout_sec = float(self.get_parameter("mode_timeout_sec").value)
        self.max_tracking_error_rad = self._seven_values("max_tracking_error_rad")
        self.fault_rearm_hold_sec = max(
            0.0, float(self.get_parameter("fault_rearm_hold_sec").value)
        )
        self.home_tolerance_rad = float(self.get_parameter("home_tolerance_rad").value)
        self.gripper_lower_limit = float(
            self.get_parameter("gripper_lower_limit_m").value
        )
        self.gripper_upper_limit = float(
            self.get_parameter("gripper_upper_limit_m").value
        )
        self.gripper_open_position = float(
            self.get_parameter("gripper_open_position_m").value
        )
        self.gripper_open_velocity = float(
            self.get_parameter("gripper_open_velocity_m_s").value
        )
        self.gripper_close_velocity = float(
            self.get_parameter("gripper_close_velocity_m_s").value
        )
        self.gripper_max_tracking_error = float(
            self.get_parameter("gripper_max_tracking_error_m").value
        )
        self.gripper_reversal_settle_samples = self.get_parameter(
            "gripper_reversal_settle_samples"
        ).value
        if (
            isinstance(self.gripper_reversal_settle_samples, bool)
            or not isinstance(self.gripper_reversal_settle_samples, int)
            or self.gripper_reversal_settle_samples < 1
        ):
            raise ValueError("gripper reversal settle samples must be a positive integer")
        self.gripper_reversal_feedback_motion_tolerance = float(
            self.get_parameter(
                "gripper_reversal_feedback_motion_tolerance_m"
            ).value
        )
        self.gripper_intent_timeout_sec = float(
            self.get_parameter("gripper_intent_timeout_sec").value
        )
        self.gripper_rearm_open_intent_min = float(
            self.get_parameter("gripper_rearm_open_intent_min").value
        )
        self.gripper_rearm_open_tolerance = {
            side: float(
                self.get_parameter(f"{side}_gripper_rearm_open_tolerance_m").value
            )
            for side in ("right", "left")
        }
        self.gripper_rearm_open_settle_sec = float(
            self.get_parameter("gripper_rearm_open_settle_sec").value
        )
        self.gripper_rearm_open_timeout_sec = float(
            self.get_parameter("gripper_rearm_open_timeout_sec").value
        )
        positive_values = (
            self.feedback_timeout_sec,
            self.ik_timeout_sec,
            self.sync_wait_max_sec,
            self.teleop_arming_timeout_sec,
            self.mode_timeout_sec,
            min(self.max_tracking_error_rad),
            self.home_tolerance_rad,
            self.gripper_open_velocity,
            self.gripper_close_velocity,
            self.gripper_max_tracking_error,
            self.gripper_reversal_feedback_motion_tolerance,
            self.gripper_intent_timeout_sec,
            self.gripper_rearm_open_settle_sec,
            self.gripper_rearm_open_timeout_sec,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive_values):
            raise ValueError("guard timeouts, tracking limit, and home tolerance must be positive")
        if not (
            math.isfinite(self.gripper_lower_limit)
            and math.isfinite(self.gripper_upper_limit)
            and self.gripper_lower_limit < self.gripper_upper_limit
        ):
            raise ValueError("gripper limits must be finite and ordered")
        if not (
            math.isfinite(self.gripper_open_position)
            and self.gripper_lower_limit < self.gripper_open_position
            <= self.gripper_upper_limit
        ):
            raise ValueError(
                "gripper runtime open position must be inside the hard limits"
            )
        if not (
            math.isfinite(self.gripper_rearm_open_intent_min)
            and 0.0 < self.gripper_rearm_open_intent_min <= 1.0
        ):
            raise ValueError("gripper rearm open intent must be in (0, 1]")
        for side, tolerance in self.gripper_rearm_open_tolerance.items():
            if not (
                math.isfinite(tolerance)
                and 0.0 < tolerance < self.gripper_max_tracking_error
                and self.gripper_open_position - tolerance
                > self.gripper_lower_limit
            ):
                raise ValueError(
                    f"{side} gripper rearm open tolerance is invalid"
                )
        self.gripper_open_max_step = (
            self.gripper_open_velocity / self.control_rate_hz
        )
        self.gripper_close_max_step = (
            self.gripper_close_velocity / self.control_rate_hz
        )
        if self.gripper_max_tracking_error <= max(
            self.gripper_open_max_step, self.gripper_close_max_step
        ):
            raise ValueError("gripper tracking limit must exceed both command steps")
        self.gripper_reversal_catchup_tolerance = (
            self.gripper_max_tracking_error
            - max(self.gripper_open_max_step, self.gripper_close_max_step)
        )
        if (
            self.gripper_reversal_feedback_motion_tolerance
            > self.gripper_reversal_catchup_tolerance
        ):
            raise ValueError(
                "gripper reversal feedback motion tolerance must not exceed "
                "the tracking reserve"
            )

        self.joint_names = {
            "right": validate_joint_names(
                list(self.get_parameter("right_joint_names").value), "right_joint_names"
            ),
            "left": validate_joint_names(
                list(self.get_parameter("left_joint_names").value), "left_joint_names"
            ),
        }
        self.home = {
            "right": self._seven_values("right_home_positions"),
            "left": self._seven_values("left_home_positions"),
        }
        self.gripper_joint_name = {
            side: str(self.get_parameter(f"{side}_gripper_joint_name").value).strip()
            for side in ("right", "left")
        }
        if any(not name for name in self.gripper_joint_name.values()):
            raise ValueError("gripper joint names must be non-empty")
        if len(set(self.gripper_joint_name.values())) != 2:
            raise ValueError("left/right gripper joint names must be distinct")
        self.lower = {
            "right": self._seven_values("right_lower_limits"),
            "left": self._seven_values("left_lower_limits"),
        }
        self.upper = {
            "right": self._seven_values("right_upper_limits"),
            "left": self._seven_values("left_upper_limits"),
        }
        self.velocity = {
            "right": self._seven_values("right_max_velocity_rad_s"),
            "left": self._seven_values("left_max_velocity_rad_s"),
        }
        self.max_step = {
            side: tuple(value / self.control_rate_hz for value in self.velocity[side])
            for side in ("right", "left")
        }
        for side in ("right", "left"):
            if any(value <= 0.0 for value in self.velocity[side]):
                raise ValueError(f"{side}_max_velocity_rad_s values must be positive")
            if any(
                value >= tracking_limit
                for value, tracking_limit in zip(
                    self.max_step[side], self.max_tracking_error_rad
                )
            ):
                raise ValueError(
                    f"{side} command steps must stay below the tracking hard gate"
                )
            if any(lo >= hi for lo, hi in zip(self.lower[side], self.upper[side])):
                raise ValueError(f"{side} lower limits must be below upper limits")
            if any(
                target < lo or target > hi
                for target, lo, hi in zip(
                    self.home[side], self.lower[side], self.upper[side]
                )
            ):
                raise ValueError(f"{side} home position exceeds configured joint limits")

        self.feedback: dict[str, tuple[float, ...] | None] = {"right": None, "left": None}
        self.feedback_ns: dict[str, int | None] = {"right": None, "left": None}
        self.ik: dict[str, tuple[float, ...] | None] = {"right": None, "left": None}
        self.ik_solver_base: dict[str, tuple[float, ...] | None] = {
            "right": None,
            "left": None,
        }
        self.ik_intent_tokens: dict[str, tuple[int, int] | None] = {
            "right": None,
            "left": None,
        }
        self.ik_ns: dict[str, int | None] = {"right": None, "left": None}
        self.ik_sync_wait_ns: dict[str, int | None] = {"right": None, "left": None}
        self.ik_sync_wait_generated_ns: dict[str, int | None] = {
            "right": None,
            "left": None,
        }
        self.ik_sync_wait_started_ns: dict[str, int | None] = {
            "right": None,
            "left": None,
        }
        self.ik_generation = {"right": 0, "left": 0}
        self.last_handled_ik_generation = {"right": 0, "left": 0}
        self.last_processed_ik_payload: dict[
            str,
            tuple[tuple[float, ...], tuple[float, ...], tuple[int, int]] | None,
        ] = {"right": None, "left": None}
        self.last_guard_acceptance: dict[str, tuple[float, ...] | None] = {
            "right": None,
            "left": None,
        }
        self.gripper_feedback: dict[str, float | None] = {"right": None, "left": None}
        self.gripper_feedback_ns: dict[str, int | None] = {"right": None, "left": None}
        self.gripper_feedback_generation = {"right": 0, "left": 0}
        self.gripper_reversal_interlock = {
            side: GripperReversalInterlock(
                min_settled_samples=self.gripper_reversal_settle_samples,
                feedback_motion_tolerance=(
                    self.gripper_reversal_feedback_motion_tolerance
                ),
            )
            for side in ("right", "left")
        }
        self.gripper_intent: dict[str, float | None] = {"right": None, "left": None}
        self.gripper_intent_ns: dict[str, int | None] = {"right": None, "left": None}
        self.requested_mode = {"right": SideMode.FAULT, "left": SideMode.FAULT}
        self.current_intent_token: dict[str, int | None] = {
            "right": None,
            "left": None,
        }
        self.mode_ns: dict[str, int | None] = {"right": None, "left": None}
        self.teleop_requested_ns: dict[str, int | None] = {"right": None, "left": None}
        self.teleop_established = {"right": False, "left": False}
        self.teleop_established_ns: dict[str, int | None] = {
            "right": None,
            "left": None,
        }
        self.arm_backpressure_active = {"right": False, "left": False}
        self.gripper_backpressure_active = {"right": False, "left": False}
        self.last_command: dict[str, tuple[float, ...] | None] = {"right": None, "left": None}
        self.last_gripper_command: dict[str, float | None] = {"right": None, "left": None}
        self.gripper_opening_after_rearm = {"right": True, "left": True}
        self.gripper_opening_started_ns: dict[str, int | None] = {
            "right": None,
            "left": None,
        }
        self.gripper_open_settle_started_ns: dict[str, int | None] = {
            "right": None,
            "left": None,
        }
        self.fault_latched = {"right": True, "left": True}
        self.fault_reason = {"right": "startup", "left": "startup"}
        self.hold_rearm_started_ns: dict[str, int | None] = {"right": None, "left": None}
        self.last_effective_mode = {"right": SideMode.FAULT, "left": SideMode.FAULT}

        self.command_pub = {
            "right": self.create_publisher(
                Float64MultiArray, str(self.get_parameter("right_command_topic").value), 1
            ),
            "left": self.create_publisher(
                Float64MultiArray, str(self.get_parameter("left_command_topic").value), 1
            ),
        }
        self.gripper_command_pub = {
            side: self.create_publisher(
                Float64MultiArray,
                str(self.get_parameter(f"{side}_gripper_command_topic").value),
                1,
            )
            for side in ("right", "left")
        }
        self.guard_mode_pub = {
            "right": self.create_publisher(
                UInt8, str(self.get_parameter("right_guard_mode_topic").value), 1
            ),
            "left": self.create_publisher(
                UInt8, str(self.get_parameter("left_guard_mode_topic").value), 1
            ),
        }
        self.guard_acceptance_pub = {
            side: self.create_publisher(
                Float64MultiArray,
                str(self.get_parameter(f"{side}_guard_acceptance_topic").value),
                1,
            )
            for side in ("right", "left")
        }
        self.guard_anchor_pub = {
            side: self.create_publisher(
                Float64MultiArray,
                str(self.get_parameter(f"{side}_guard_anchor_topic").value),
                1,
            )
            for side in ("right", "left")
        }

        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._joint_callback,
            1,
        )
        for side in ("right", "left"):
            self.create_subscription(
                Float64MultiArray,
                str(self.get_parameter(f"{side}_ik_topic").value),
                lambda msg, selected=side: self._ik_callback(selected, msg),
                1,
            )
            self.create_subscription(
                Float32,
                str(self.get_parameter(f"{side}_gripper_intent_topic").value),
                lambda msg, selected=side: self._gripper_intent_callback(selected, msg),
                1,
            )
            self.create_subscription(
                UInt64,
                str(self.get_parameter(f"{side}_mode_topic").value),
                lambda msg, selected=side: self._mode_callback(selected, msg),
                1,
            )
            self.create_subscription(
                UInt64,
                str(self.get_parameter(f"{side}_sync_wait_topic").value),
                lambda msg, selected=side: self._ik_sync_wait_callback(selected, msg),
                1,
            )

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.get_logger().info(
            "OpenArm command guard ready in latched FAULT; release both Grip and "
            f"Index Trigger controls for {self.fault_rearm_hold_sec:.2f}s to arm HOLD"
        )

    def _seven_values(self, parameter: str) -> tuple[float, ...]:
        values = tuple(float(value) for value in self.get_parameter(parameter).value)
        if len(values) != 7 or not all(math.isfinite(value) for value in values):
            raise ValueError(f"{parameter} must contain seven finite values")
        return values

    def _joint_callback(self, msg: JointState) -> None:
        now_ns = time.monotonic_ns()
        if len(msg.name) != len(set(msg.name)):
            return
        position_by_name = {
            str(name): float(msg.position[index])
            for index, name in enumerate(msg.name)
            if index < len(msg.position) and math.isfinite(float(msg.position[index]))
        }
        for side in ("right", "left"):
            try:
                self.feedback[side] = map_named_positions(
                    msg.name, msg.position, self.joint_names[side]
                )
                self.feedback_ns[side] = now_ns
            except ValueError:
                pass
            gripper_name = self.gripper_joint_name[side]
            if gripper_name in position_by_name:
                self.gripper_feedback[side] = position_by_name[gripper_name]
                self.gripper_feedback_ns[side] = now_ns
                self.gripper_feedback_generation[side] += 1

    def _ik_callback(self, side: str, msg: Float64MultiArray) -> None:
        try:
            candidate, solver_base, intent_tokens = parse_atomic_ik_command(msg.data)
            if (
                intent_tokens != self._current_intent_tokens()
                or self.requested_mode[side] != SideMode.TELEOP
            ):
                # A valid but stale solve is ordinary cross-topic reordering.
                # It must not refresh IK freshness, generations, caches, or the
                # bounded synchronization lease for the current intent pair.
                return
            self.ik[side] = candidate
            self.ik_solver_base[side] = solver_base
            self.ik_intent_tokens[side] = intent_tokens
            self.ik_ns[side] = time.monotonic_ns()
            self.ik_sync_wait_ns[side] = None
            self.ik_sync_wait_generated_ns[side] = None
            self.ik_generation[side] += 1
        except (TypeError, ValueError) as exc:
            self.ik[side] = None
            self.ik_solver_base[side] = None
            self.ik_intent_tokens[side] = None
            self.ik_ns[side] = None
            self._latch_fault(side, f"invalid IK message: {exc}")

    def _ik_sync_wait_callback(self, side: str, msg: UInt64) -> None:
        generated_ns = int(msg.data)
        now_ns = time.monotonic_ns()
        valid_current_lease = (
            generated_ns > 0
            and generated_ns <= now_ns
            and self.requested_mode[side] == SideMode.TELEOP
            and self.teleop_established[side]
            and is_post_engagement(generated_ns, self.teleop_requested_ns[side])
            and is_post_engagement(generated_ns, self.teleop_established_ns[side])
            and is_post_engagement(generated_ns, self.ik_ns[side])
        )
        if not valid_current_lease:
            self.ik_sync_wait_ns[side] = None
            self.ik_sync_wait_generated_ns[side] = None
            return
        if self.ik_sync_wait_started_ns[side] is None:
            self.ik_sync_wait_started_ns[side] = now_ns
        self.ik_sync_wait_ns[side] = now_ns
        self.ik_sync_wait_generated_ns[side] = generated_ns

    def _gripper_intent_callback(self, side: str, msg: Float32) -> None:
        value = float(msg.data)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            self.gripper_intent[side] = None
            self.gripper_intent_ns[side] = None
            self._latch_fault(side, "normalized gripper intent must be finite and in [0, 1]")
            return
        self.gripper_intent[side] = value
        self.gripper_intent_ns[side] = time.monotonic_ns()

    def _mode_callback(self, side: str, msg: UInt64) -> None:
        raw_token = int(msg.data)
        try:
            _, requested = decode_intent_token(raw_token)
        except (TypeError, ValueError) as exc:
            now_ns = time.monotonic_ns()
            self._invalidate_bimanual_ik_state(now_ns)
            reason = f"invalid {side} intent token: {exc}"
            self._latch_fault("right", reason)
            self._latch_fault("left", reason)
            return

        now_ns = time.monotonic_ns()
        if raw_token != self.current_intent_token[side]:
            # Publish streams are asynchronous, so update the pair first and
            # then invalidate both sides as one logical session boundary. This
            # includes TELEOP -> TELEOP epoch changes: neither arm may commit a
            # solve produced against the previous bimanual intent pair.
            self.current_intent_token[side] = raw_token
            self.requested_mode[side] = requested
            self.mode_ns[side] = now_ns
            self._invalidate_bimanual_ik_state(now_ns)
            return

        # A repeated token is the mode heartbeat. It refreshes only freshness;
        # it must never replenish an arming window or synchronization episode.
        self.mode_ns[side] = now_ns

    def _current_intent_tokens(self) -> tuple[int, int] | None:
        right_token = self.current_intent_token["right"]
        left_token = self.current_intent_token["left"]
        if right_token is None or left_token is None:
            return None
        return right_token, left_token

    def _teleop_tokens_are_current(
        self, side: str, intent_tokens: tuple[int, int] | None
    ) -> bool:
        return (
            intent_tokens is not None
            and intent_tokens == self._current_intent_tokens()
            and self.requested_mode[side] == SideMode.TELEOP
        )

    def _invalidate_bimanual_ik_state(self, now_ns: int) -> None:
        """Atomically revoke every solve/cache tied to the previous token pair."""

        for selected in ("right", "left"):
            self.ik[selected] = None
            self.ik_solver_base[selected] = None
            self.ik_intent_tokens[selected] = None
            self.ik_ns[selected] = None
            self._reset_ik_acceptance_cache(selected)
            self.ik_sync_wait_ns[selected] = None
            self.ik_sync_wait_generated_ns[selected] = None
            self.ik_sync_wait_started_ns[selected] = None
            self.teleop_established[selected] = False
            self.teleop_established_ns[selected] = None
            if self.requested_mode[selected] == SideMode.TELEOP:
                self.teleop_requested_ns[selected] = now_ns
            else:
                self.teleop_requested_ns[selected] = None
            self.arm_backpressure_active[selected] = False

    def _reset_ik_acceptance_cache(self, side: str) -> None:
        self.last_processed_ik_payload[side] = None
        self.last_guard_acceptance[side] = None
        self.last_handled_ik_generation[side] = self.ik_generation[side]

    def _latch_fault(self, side: str, reason: str) -> None:
        if not self.fault_latched[side] or reason != self.fault_reason[side]:
            self.get_logger().error(f"[{side}] command FAULT latched: {reason}")
        self.fault_latched[side] = True
        self.teleop_established[side] = False
        self.teleop_established_ns[side] = None
        self.ik_sync_wait_ns[side] = None
        self.ik_sync_wait_generated_ns[side] = None
        self.ik_sync_wait_started_ns[side] = None
        self.fault_reason[side] = reason
        self.hold_rearm_started_ns[side] = None
        self.arm_backpressure_active[side] = False
        self.gripper_backpressure_active[side] = False
        self.gripper_reversal_interlock[side].reset_pending()
        self.gripper_opening_after_rearm[side] = True
        self.gripper_opening_started_ns[side] = None
        self.gripper_open_settle_started_ns[side] = None
        self._reset_ik_acceptance_cache(side)

    def _publish_effective_mode(self, side: str, mode: SideMode) -> None:
        self.guard_mode_pub[side].publish(UInt8(data=int(mode)))
        if mode != self.last_effective_mode[side]:
            self.get_logger().info(
                f"[{side}] guard {self.last_effective_mode[side].name} -> {mode.name}"
            )
            self.last_effective_mode[side] = mode

    def _publish_controller_anchor(self, side: str) -> None:
        """Publish one retained command bound to the same bimanual token pair."""

        anchor = self.last_command[side]
        intent_tokens = self._current_intent_tokens()
        if anchor is None or intent_tokens is None or self.fault_latched[side]:
            return
        self.guard_anchor_pub[side].publish(
            Float64MultiArray(data=list(build_guard_anchor(anchor, intent_tokens)))
        )

    def _feedback_within_limits(self, side: str) -> bool:
        values = self.feedback[side]
        if values is None:
            return False
        return all(
            lo <= value <= hi
            for value, lo, hi in zip(values, self.lower[side], self.upper[side])
        )

    def _try_rearm(self, side: str, requested: SideMode, now_ns: int) -> bool:
        normalized_gripper = self.gripper_intent[side]
        trigger_released = (
            normalized_gripper is not None
            and normalized_gripper >= self.gripper_rearm_open_intent_min
        )
        if requested != SideMode.HOLD or not trigger_released:
            self.hold_rearm_started_ns[side] = None
            return False
        if self.hold_rearm_started_ns[side] is None:
            self.hold_rearm_started_ns[side] = now_ns
            return False
        held_sec = (now_ns - self.hold_rearm_started_ns[side]) / 1e9
        if held_sec < self.fault_rearm_hold_sec:
            return False
        # Keep both last controller commands across a fault. Re-anchoring an
        # existing command to newer feedback would silently erase the command
        # continuity contract while each forward-position controller still
        # retains its previous command. Startup is the only case without one.
        if self.last_command[side] is None:
            self.last_command[side] = self.feedback[side]
        if self.last_gripper_command[side] is None:
            self.last_gripper_command[side] = self.gripper_feedback[side]
        # Do not announce a successful re-arm while either retained controller
        # command is still outside its unchanged tracking gate.
        if not self._tracking_ok(side):
            self.hold_rearm_started_ns[side] = None
            return False
        self.fault_latched[side] = False
        self.fault_reason[side] = ""
        self.hold_rearm_started_ns[side] = None
        self.gripper_opening_after_rearm[side] = True
        self.gripper_opening_started_ns[side] = now_ns
        self.gripper_open_settle_started_ns[side] = None
        self.get_logger().info(
            f"[{side}] command guard re-armed in HOLD; opening gripper before "
            "accepting a new Trigger press"
        )
        return True

    def _tracking_ok(self, side: str) -> bool:
        measured = self.feedback[side]
        previous = self.last_command[side]
        if measured is None or previous is None:
            return True
        arm_ok = all(
            abs(actual - expected) <= tracking_limit
            for actual, expected, tracking_limit in zip(
                measured, previous, self.max_tracking_error_rad
            )
        )
        gripper_measured = self.gripper_feedback[side]
        gripper_previous = self.last_gripper_command[side]
        gripper_ok = (
            gripper_measured is None
            or gripper_previous is None
            or abs(gripper_measured - gripper_previous)
            <= self.gripper_max_tracking_error
        )
        return arm_ok and gripper_ok

    def _shape_arm_target(
        self,
        side: str,
        target: tuple[float, ...],
        measured: tuple[float, ...],
        monotonic_base: tuple[float, ...] | None = None,
    ) -> tuple[float, ...]:
        reference = self.last_command[side]
        if reference is None:
            reference = measured
        normal_candidate = step_toward(reference, target, self.max_step[side])
        candidate = shape_monotonic_guard_command(
            current_command=reference,
            monotonic_base=monotonic_base,
            target=target,
            feedback=measured,
            max_step=self.max_step[side],
            max_tracking_error=self.max_tracking_error_rad,
        )
        backpressure = any(
            not math.isclose(shaped, normal, rel_tol=0.0, abs_tol=1e-12)
            for shaped, normal in zip(candidate, normal_candidate)
        )
        if backpressure and not self.arm_backpressure_active[side]:
            tracking_error = max(
                abs(command - actual)
                for command, actual in zip(candidate, measured)
            )
            self.get_logger().warn(
                f"[{side}] arm tracking backpressure active: "
                f"max command/feedback error={tracking_error:.6f}rad"
            )
        elif not backpressure and self.arm_backpressure_active[side]:
            self.get_logger().info(f"[{side}] arm tracking backpressure released")
        self.arm_backpressure_active[side] = backpressure
        return candidate

    def _shape_gripper_target(
        self,
        side: str,
        normalized_opening: float,
        measured: float,
    ) -> tuple[float, bool, float]:
        """Prepare one independent, rate-limited Piper-style Trigger command."""

        reference = self.last_gripper_command[side]
        if reference is None:
            reference = measured
        requested = gripper_target_from_normalized(
            normalized_opening,
            self.gripper_lower_limit,
            self.gripper_open_position,
        )
        unconstrained = step_gripper_toward(
            reference,
            requested,
            self.gripper_open_max_step,
            self.gripper_close_max_step,
        )
        interlock = self.gripper_reversal_interlock[side]
        previous_pending_direction = interlock.pending_direction
        reversal_hold = interlock.should_hold(
            current_command=reference,
            target=requested,
            feedback=measured,
            feedback_generation=self.gripper_feedback_generation[side],
            catchup_tolerance=self.gripper_reversal_catchup_tolerance,
        )
        if previous_pending_direction == 0 and interlock.pending_direction != 0:
            direction_name = (
                "opening" if interlock.pending_direction > 0 else "closing"
            )
            self.get_logger().warn(
                f"[{side}] gripper reversal interlock active for {direction_name}: "
                f"feedback={measured:.6f}, command={reference:.6f}, "
                f"target={requested:.6f}, catchup="
                f"{self.gripper_reversal_catchup_tolerance:.6f}m, "
                f"motion_tolerance="
                f"{self.gripper_reversal_feedback_motion_tolerance:.6f}m, "
                f"required_fresh_samples={self.gripper_reversal_settle_samples}"
            )
        elif previous_pending_direction != 0 and interlock.pending_direction == 0:
            self.get_logger().info(
                f"[{side}] gripper reversal interlock cancelled before release"
            )

        if reversal_hold:
            bounded = shape_gripper_reversal_brake_command(
                current_command=reference,
                feedback=measured,
                max_open_step=self.gripper_open_max_step,
                max_close_step=self.gripper_close_max_step,
                max_tracking_error=self.gripper_max_tracking_error,
            )
        else:
            bounded = shape_gripper_command(
                current_command=reference,
                target=requested,
                feedback=measured,
                max_open_step=self.gripper_open_max_step,
                max_close_step=self.gripper_close_max_step,
                max_tracking_error=self.gripper_max_tracking_error,
            )
        backpressure = not math.isclose(
            bounded,
            unconstrained,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
        if backpressure and not self.gripper_backpressure_active[side]:
            self.get_logger().warn(
                f"[{side}] gripper tracking backpressure active: "
                f"feedback={measured:.6f}, previous={reference:.6f}, "
                f"requested={requested:.6f}, bounded={bounded:.6f}"
            )
        elif not backpressure and self.gripper_backpressure_active[side]:
            self.get_logger().info(
                f"[{side}] gripper tracking backpressure released"
            )
        self.gripper_backpressure_active[side] = backpressure

        decision = validate_gripper_command(
            candidate=bounded,
            feedback=measured,
            previous_command=self.last_gripper_command[side],
            lower_limit=self.gripper_lower_limit,
            upper_limit=self.gripper_upper_limit,
            max_open_step=self.gripper_open_max_step,
            max_close_step=self.gripper_close_max_step,
            max_tracking_error=self.gripper_max_tracking_error,
        )
        if not decision.accepted or decision.command is None:
            raise ValueError(decision.reason)
        return decision.command[0], reversal_hold, requested

    def _process_side(self, side: str, now_ns: int) -> None:
        if not is_fresh(now_ns, self.mode_ns[side], self.mode_timeout_sec):
            self._latch_fault(side, "mode intent stale")
            self._publish_effective_mode(side, SideMode.FAULT)
            return
        requested = self.requested_mode[side]
        if not is_fresh(now_ns, self.feedback_ns[side], self.feedback_timeout_sec):
            self._latch_fault(side, "joint feedback stale")
            self._publish_effective_mode(side, SideMode.FAULT)
            return
        if not is_fresh(
            now_ns, self.gripper_feedback_ns[side], self.feedback_timeout_sec
        ):
            self._latch_fault(side, "gripper feedback stale")
            self._publish_effective_mode(side, SideMode.FAULT)
            return
        gripper_measured = self.gripper_feedback[side]
        if gripper_measured is None or not (
            self.gripper_lower_limit
            <= gripper_measured
            <= self.gripper_upper_limit
        ):
            self._latch_fault(side, "measured gripper position outside configured limits")
            self._publish_effective_mode(side, SideMode.FAULT)
            return
        if not self._feedback_within_limits(side):
            self._latch_fault(side, "measured joint position outside configured limits")
            self._publish_effective_mode(side, SideMode.FAULT)
            return
        if requested == SideMode.FAULT:
            self._latch_fault(side, "upstream requested FAULT")
            self._publish_effective_mode(side, SideMode.FAULT)
            return

        # Match the original Piper semantics: gripper intent is independent of
        # the arm Grip/deadman.  It must nevertheless be a fresh, valid sample
        # from the reviewed Quest input path before either ID8 controller moves.
        if not is_fresh(
            now_ns,
            self.gripper_intent_ns[side],
            self.gripper_intent_timeout_sec,
        ):
            self._latch_fault(side, "gripper intent stale")
            self._publish_effective_mode(side, SideMode.FAULT)
            return
        normalized_gripper = self.gripper_intent[side]
        if normalized_gripper is None:
            self._latch_fault(side, "gripper intent unavailable")
            self._publish_effective_mode(side, SideMode.FAULT)
            return

        if self.fault_latched[side]:
            rearmed = self._try_rearm(side, requested, now_ns)
            if rearmed:
                self._publish_controller_anchor(side)
            self._publish_effective_mode(
                side, SideMode.HOLD if not self.fault_latched[side] else SideMode.FAULT
            )
            return

        if not self._tracking_ok(side):
            self._latch_fault(side, "tracking error exceeds configured limit")
            self._publish_effective_mode(side, SideMode.FAULT)
            return

        measured = self.feedback[side]
        assert measured is not None
        arm_candidate: tuple[float, ...] | None = None
        guard_echo: (
            tuple[tuple[float, ...], tuple[float, ...], tuple[int, int]] | None
        ) = None
        retry_acceptance: tuple[float, ...] | None = None
        retry_intent_tokens: tuple[int, int] | None = None
        effective_mode = requested

        if self.gripper_opening_after_rearm[side]:
            opening_started_ns = self.gripper_opening_started_ns[side]
            if opening_started_ns is None:
                self.gripper_opening_started_ns[side] = now_ns
            previous_gripper = self.last_gripper_command[side]
            command_is_open = previous_gripper is not None and math.isclose(
                previous_gripper,
                self.gripper_open_position,
                rel_tol=0.0,
                abs_tol=1e-9,
            )
            feedback_is_open = (
                gripper_measured
                >= self.gripper_open_position
                - self.gripper_rearm_open_tolerance[side]
            )
            trigger_is_released = (
                normalized_gripper >= self.gripper_rearm_open_intent_min
            )
            endpoint_ready = (
                command_is_open and feedback_is_open and trigger_is_released
            )
            (
                endpoint_settled,
                self.gripper_open_settle_started_ns[side],
            ) = update_gripper_rearm_settle(
                now_ns=now_ns,
                endpoint_ready=endpoint_ready,
                settle_started_ns=self.gripper_open_settle_started_ns[side],
                settle_sec=self.gripper_rearm_open_settle_sec,
            )
            if endpoint_settled:
                self.gripper_opening_after_rearm[side] = False
                self.gripper_opening_started_ns[side] = None
                self.gripper_open_settle_started_ns[side] = None
                self.get_logger().info(
                    f"[{side}] startup/rearm gripper opening complete; "
                    "new Trigger presses are enabled"
                )
            else:
                if (
                    opening_started_ns is not None
                    and now_ns - opening_started_ns
                    > int(self.gripper_rearm_open_timeout_sec * 1_000_000_000)
                ):
                    self._latch_fault(
                        side,
                        "gripper did not reach the startup/rearm open endpoint "
                        f"within {self.gripper_rearm_open_timeout_sec:.2f}s "
                        f"(feedback={gripper_measured:.6f}, "
                        f"command={previous_gripper}, "
                        f"target={self.gripper_open_position:.6f}, "
                        f"required_feedback>="
                        f"{self.gripper_open_position - self.gripper_rearm_open_tolerance[side]:.6f}, "
                        f"reversal_pending="
                        f"{self.gripper_reversal_interlock[side].pending_direction}, "
                        f"settled_samples="
                        f"{self.gripper_reversal_interlock[side].settled_fresh_samples})",
                    )
                    self._publish_effective_mode(side, SideMode.FAULT)
                    return
                # A Trigger held during startup/recovery cannot turn the first
                # post-enable action into closure. Reach the open endpoint and
                # observe release before accepting a later press.
                normalized_gripper = 1.0

        if requested == SideMode.HOLD:
            # Retain the arm controller's last command, while the independent
            # gripper path below continues to follow its side's Index Trigger.
            pass
        if requested == SideMode.TELEOP:
            engage_ns = self.teleop_requested_ns[side]
            ik_after_engage = is_post_engagement(self.ik_ns[side], engage_ns)
            arming_within_deadline = teleop_arming_within_deadline(
                now_ns,
                engage_ns,
                self.teleop_arming_timeout_sec,
            )
            sync_wait_fresh = (
                is_fresh(
                    now_ns,
                    self.ik_sync_wait_ns[side],
                    self.ik_timeout_sec,
                )
                and is_post_engagement(
                    self.ik_sync_wait_generated_ns[side],
                    self.teleop_established_ns[side],
                )
                and is_post_engagement(
                    self.ik_sync_wait_generated_ns[side],
                    engage_ns,
                )
                and self.ik_sync_wait_started_ns[side] is not None
                and now_ns - self.ik_sync_wait_started_ns[side]
                <= int(self.sync_wait_max_sec * 1_000_000_000)
            )
            ik_mode = guard_teleop_ik_mode(
                self.teleop_established[side],
                ik_after_engage,
                is_fresh(now_ns, self.ik_ns[side], self.ik_timeout_sec),
                sync_wait_fresh,
                arming_within_deadline,
            )
            if ik_mode == SideMode.HOLD:
                # Either the bounded first-command handshake is still arming,
                # or an established side holds a causally validated sync-wait
                # lease. Retain the controller command and publish no arm ACK;
                # Grip release still cancels the engagement immediately.
                effective_mode = SideMode.HOLD
            elif ik_mode == SideMode.FAULT:
                if (
                    not self.teleop_established[side]
                    and not arming_within_deadline
                ):
                    reason = (
                        "first post-engagement IK command missed the "
                        f"{self.teleop_arming_timeout_sec:.2f}s arming deadline"
                    )
                else:
                    reason = "IK command stale"
                self._latch_fault(side, reason)
                self._publish_effective_mode(side, SideMode.FAULT)
                return
            else:
                candidate = self.ik[side]
                solver_base = self.ik_solver_base[side]
                intent_tokens = self.ik_intent_tokens[side]
                if (
                    candidate is None
                    or solver_base is None
                    or intent_tokens is None
                ):
                    self._latch_fault(side, "IK command unavailable")
                    self._publish_effective_mode(side, SideMode.FAULT)
                    return
                if not self._teleop_tokens_are_current(side, intent_tokens):
                    # A token edge normally clears this state in the mode
                    # callback. Recheck here as a fail-closed boundary in case
                    # executor behavior changes in the future.
                    return
                generation = self.ik_generation[side]
                if generation != self.last_handled_ik_generation[side]:
                    payload = (candidate, solver_base, intent_tokens)
                    cached_acceptance = self.last_guard_acceptance[side]
                    if (
                        payload == self.last_processed_ik_payload[side]
                        and cached_acceptance is not None
                    ):
                        # IK retries carry the exact same candidate/base pair.
                        # Re-echo the already committed acceptance without
                        # issuing another arm controller command. The gripper
                        # path below still runs independently on every tick.
                        retry_acceptance = cached_acceptance
                        retry_intent_tokens = intent_tokens
                    else:
                        solver_decision = validate_solver_candidate_step(
                            candidate=candidate,
                            solver_base=solver_base,
                            max_step=self.max_step[side],
                        )
                        if (
                            not solver_decision.accepted
                            or solver_decision.command is None
                        ):
                            self._latch_fault(side, solver_decision.reason)
                            self._publish_effective_mode(side, SideMode.FAULT)
                            return
                        guard_echo = payload
                        # The raw IK delta is safe relative to the exact
                        # guard-accepted state it solved from. Follow that
                        # target from the retained controller command through
                        # rate and tracking envelopes.
                        try:
                            arm_candidate = self._shape_arm_target(
                                side,
                                solver_decision.command,
                                measured,
                                monotonic_base=solver_base,
                            )
                        except ValueError as exc:
                            self._latch_fault(side, str(exc))
                            self._publish_effective_mode(side, SideMode.FAULT)
                            return
        elif requested == SideMode.HOME:
            if max(
                abs(actual - target) for actual, target in zip(measured, self.home[side])
            ) <= self.home_tolerance_rad:
                effective_mode = SideMode.HOLD
            else:
                try:
                    arm_candidate = self._shape_arm_target(
                        side, self.home[side], measured
                    )
                except ValueError as exc:
                    self._latch_fault(side, str(exc))
                    self._publish_effective_mode(side, SideMode.FAULT)
                    return
        elif requested != SideMode.HOLD:
            self._latch_fault(side, f"unsupported mode {requested}")
            self._publish_effective_mode(side, SideMode.FAULT)
            return

        arm_decision = None
        acceptance_decision = None
        if arm_candidate is not None:
            arm_decision = validate_guard_command(
                candidate=arm_candidate,
                feedback=measured,
                previous_command=self.last_command[side],
                lower_limits=self.lower[side],
                upper_limits=self.upper[side],
                max_step=self.max_step[side],
                max_tracking_error=self.max_tracking_error_rad,
            )
            if not arm_decision.accepted or arm_decision.command is None:
                self._latch_fault(side, arm_decision.reason)
                self._publish_effective_mode(side, SideMode.FAULT)
                return
            if guard_echo is not None:
                echoed_candidate, echoed_base, _ = guard_echo
                acceptance_decision = validate_guard_acceptance_progress(
                    accepted_command=arm_decision.command,
                    candidate=echoed_candidate,
                    solver_base=echoed_base,
                    max_step=self.max_step[side],
                )
                if (
                    not acceptance_decision.accepted
                    or acceptance_decision.command is None
                ):
                    self._latch_fault(
                        side,
                        "guard refused to publish an invalid IK acknowledgement: "
                        f"{acceptance_decision.reason}",
                    )
                    self._publish_effective_mode(side, SideMode.FAULT)
                    return

        try:
            (
                gripper_command,
                gripper_reversal_brake,
                gripper_requested_target,
            ) = self._shape_gripper_target(
                side, normalized_gripper, gripper_measured
            )
        except ValueError as exc:
            self._latch_fault(side, str(exc))
            self._publish_effective_mode(side, SideMode.FAULT)
            return

        # Commit only after both independently prepared commands have passed.
        if arm_decision is not None and arm_decision.command is not None:
            first_teleop_commit_ns = None
            if guard_echo is not None and not self.teleop_established[side]:
                # Check again immediately before publication: an IK callback
                # arriving inside the budget does not authorize a command if
                # validation/shaping delayed its actual commit past the same
                # Grip edge's deadline.
                first_teleop_commit_ns = time.monotonic_ns()
                if not teleop_arming_within_deadline(
                    first_teleop_commit_ns,
                    self.teleop_requested_ns[side],
                    self.teleop_arming_timeout_sec,
                ):
                    self._latch_fault(
                        side,
                        "first post-engagement IK command missed the "
                        f"{self.teleop_arming_timeout_sec:.2f}s arming deadline",
                    )
                    self._publish_effective_mode(side, SideMode.FAULT)
                    return
            if guard_echo is not None and not self._teleop_tokens_are_current(
                side, guard_echo[2]
            ):
                # This is the last authorization check before touching the
                # controller. A raw edge on either mode token revokes a solve
                # for both arms, even if this side remains TELEOP.
                return
            self.command_pub[side].publish(
                Float64MultiArray(data=list(arm_decision.command))
            )
            self.last_command[side] = arm_decision.command
            if guard_echo is not None:
                assert acceptance_decision is not None
                assert acceptance_decision.command is not None
                echoed_candidate, echoed_base, echoed_tokens = guard_echo
                acceptance = build_guard_acceptance(
                    acceptance_decision.command,
                    echoed_candidate,
                    echoed_base,
                    echoed_tokens,
                )
                self.guard_acceptance_pub[side].publish(
                    Float64MultiArray(data=list(acceptance))
                )
                self.last_processed_ik_payload[side] = guard_echo
                self.last_guard_acceptance[side] = acceptance
                self.last_handled_ik_generation[side] = self.ik_generation[side]
                if not self.teleop_established[side]:
                    assert first_teleop_commit_ns is not None
                    self.teleop_established[side] = True
                    self.teleop_established_ns[side] = first_teleop_commit_ns
                    self.get_logger().info(
                        f"[{side}] first post-engagement IK command established"
                    )
                self.ik_sync_wait_ns[side] = None
                self.ik_sync_wait_generated_ns[side] = None
                self.ik_sync_wait_started_ns[side] = None
        elif retry_acceptance is not None:
            if not self._teleop_tokens_are_current(side, retry_intent_tokens):
                return
            self.guard_acceptance_pub[side].publish(
                Float64MultiArray(data=list(retry_acceptance))
            )
            self.last_handled_ik_generation[side] = self.ik_generation[side]
            # A cached retry ACK is still a successfully handled new IK
            # generation. It ends the current synchronization episode just as
            # a newly committed command+ACK does. Do not clear this cumulative
            # start timestamp on heartbeat/False/new-target input paths: only
            # successful guard acceptance may replenish the next episode.
            self.ik_sync_wait_ns[side] = None
            self.ik_sync_wait_generated_ns[side] = None
            self.ik_sync_wait_started_ns[side] = None
        previous_gripper_command = self.last_gripper_command[side]
        pending_direction = self.gripper_reversal_interlock[side].pending_direction
        settled_samples = self.gripper_reversal_interlock[side].settled_fresh_samples
        self.gripper_command_pub[side].publish(
            Float64MultiArray(data=[gripper_command])
        )
        if previous_gripper_command is not None and not gripper_reversal_brake:
            self.gripper_reversal_interlock[side].commit_follow_command(
                previous_gripper_command,
                gripper_command,
                target=gripper_requested_target,
                feedback=gripper_measured,
                intent_deadband=self.gripper_reversal_catchup_tolerance,
            )
        if (
            pending_direction != 0
            and self.gripper_reversal_interlock[side].pending_direction == 0
        ):
            direction_name = "opening" if pending_direction > 0 else "closing"
            self.get_logger().info(
                f"[{side}] gripper reversal interlock released for {direction_name} "
                f"after {settled_samples} fresh settled samples"
            )
        self.last_gripper_command[side] = gripper_command
        self._publish_controller_anchor(side)
        self._publish_effective_mode(side, effective_mode)

    def _control_loop(self) -> None:
        now_ns = time.monotonic_ns()
        self._process_side("right", now_ns)
        self._process_side("left", now_ns)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OpenArmCommandGuard()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except RuntimeError as exc:
            shutdown_take_race = is_rclpy_humble_shutdown_take_race(
                exc, context_ok=rclpy.ok()
            )
            if not shutdown_take_race:
                raise
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
