#!/usr/bin/env python3
"""Bimanual ROS wrapper around the official ``openarm_control.Kinematics`` API."""

from __future__ import annotations

from collections import deque
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, UInt64

from openarm_teleop_core import (
    LEFT_JOINT_NAMES,
    OPENARM_LEFT_JOINT_HARD_LOWER_RAD,
    OPENARM_LEFT_JOINT_HARD_UPPER_RAD,
    OPENARM_LEFT_JOINT_SOFT_LOWER_RAD,
    OPENARM_LEFT_JOINT_SOFT_UPPER_RAD,
    OPENARM_RIGHT_JOINT_HARD_LOWER_RAD,
    OPENARM_RIGHT_JOINT_HARD_UPPER_RAD,
    OPENARM_RIGHT_JOINT_SOFT_LOWER_RAD,
    OPENARM_RIGHT_JOINT_SOFT_UPPER_RAD,
    OPENARM_TELEOP_CONTROL_RATE_HZ,
    OPENARM_TELEOP_MAX_JOINT_VELOCITY_RAD_S,
    OPENARM_TELEOP_TRACKING_ERROR_RAD,
    RIGHT_JOINT_NAMES,
    SideMode,
    TARGET_HISTORY_CAPACITY,
    TargetHistorySample,
    apply_joint_soft_limits,
    build_atomic_ik_command,
    build_driver_state,
    decode_intent_token,
    is_fresh,
    is_post_engagement,
    is_rclpy_humble_shutdown_take_race,
    map_named_positions,
    parse_atomic_target,
    parse_guard_acceptance,
    parse_guard_anchor,
    select_solver_base,
    select_solver_target,
    select_coherent_target_history_pair,
    split_driver_state,
    target_only_commit_expiry_allows_sync_wait,
    validate_guard_acceptance_progress,
    validate_guard_anchor,
    validate_solver_candidate_step,
    validate_joint_names,
)


DEFAULT_VELOCITY_CAPS = list(OPENARM_TELEOP_MAX_JOINT_VELOCITY_RAD_S)
DEFAULT_TRACKING_LIMITS = list(OPENARM_TELEOP_TRACKING_ERROR_RAD)
DEFAULT_LEFT_LOWER = list(OPENARM_LEFT_JOINT_SOFT_LOWER_RAD)
DEFAULT_LEFT_UPPER = list(OPENARM_LEFT_JOINT_SOFT_UPPER_RAD)
DEFAULT_RIGHT_LOWER = list(OPENARM_RIGHT_JOINT_SOFT_LOWER_RAD)
DEFAULT_RIGHT_UPPER = list(OPENARM_RIGHT_JOINT_SOFT_UPPER_RAD)


class OpenArmBimanualIK(Node):
    """Run one shared official solver while preserving independent side modes."""

    def __init__(self):
        super().__init__("openarm_bimanual_ik")
        self.declare_parameter("model_xml", "")
        self.declare_parameter("frame_right", "openarm_right_hand_tcp")
        self.declare_parameter("frame_left", "openarm_left_hand_tcp")
        self.declare_parameter("origin_frame", "arm_origin")
        self.declare_parameter("keyframe", "home")
        self.declare_parameter("right_joint_names", list(RIGHT_JOINT_NAMES))
        self.declare_parameter("left_joint_names", list(LEFT_JOINT_NAMES))
        self.declare_parameter("right_gripper_joint_name", "openarm_right_finger_joint1")
        self.declare_parameter("left_gripper_joint_name", "openarm_left_finger_joint1")
        self.declare_parameter("right_gripper_hold", 0.0)
        self.declare_parameter("left_gripper_hold", 0.0)
        self.declare_parameter("joint_state_topic", "/joint_states")
        # Retained only to document the matching RViz/debug streams. IK never
        # subscribes to these unbound PoseStamped topics.
        self.declare_parameter("right_target_pose_topic", "/openarm/target_pose/right")
        self.declare_parameter("left_target_pose_topic", "/openarm/target_pose/left")
        self.declare_parameter(
            "right_target_intent_topic", "/openarm/target_intent/right"
        )
        self.declare_parameter(
            "left_target_intent_topic", "/openarm/target_intent/left"
        )
        self.declare_parameter("right_mode_topic", "/openarm/mode/right")
        self.declare_parameter("left_mode_topic", "/openarm/mode/left")
        self.declare_parameter("right_ik_topic", "/openarm/ik/right")
        self.declare_parameter("left_ik_topic", "/openarm/ik/left")
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
        self.declare_parameter("right_tcp_pose_topic", "/right_arm/feedback/tcp_pose")
        self.declare_parameter("left_tcp_pose_topic", "/left_arm/feedback/tcp_pose")
        self.declare_parameter("solve_rate_hz", OPENARM_TELEOP_CONTROL_RATE_HZ)
        self.declare_parameter("feedback_timeout_sec", 0.25)
        self.declare_parameter("target_timeout_sec", 0.20)
        self.declare_parameter("mode_timeout_sec", 0.25)
        self.declare_parameter("guard_ack_timeout_sec", 0.25)
        self.declare_parameter("guard_anchor_timeout_sec", 0.30)
        self.declare_parameter("bimanual_target_max_skew_sec", 0.03)
        self.declare_parameter("position_cost", 1.0)
        self.declare_parameter("orientation_cost", 1.0)
        self.declare_parameter("lm_damping", 0.01)
        self.declare_parameter("damping", 0.10)
        self.declare_parameter("posture_cost", 0.01)
        self.declare_parameter("dt", 0.02)
        self.declare_parameter("max_iters", 10)
        self.declare_parameter("solver", "daqp")
        self.declare_parameter("right_max_velocity_rad_s", DEFAULT_VELOCITY_CAPS)
        self.declare_parameter("left_max_velocity_rad_s", DEFAULT_VELOCITY_CAPS)
        self.declare_parameter("max_tracking_error_rad", DEFAULT_TRACKING_LIMITS)
        self.declare_parameter("right_lower_limits", DEFAULT_RIGHT_LOWER)
        self.declare_parameter("right_upper_limits", DEFAULT_RIGHT_UPPER)
        self.declare_parameter("left_lower_limits", DEFAULT_LEFT_LOWER)
        self.declare_parameter("left_upper_limits", DEFAULT_LEFT_UPPER)

        self.model_xml = str(self.get_parameter("model_xml").value).strip()
        if not self.model_xml:
            from ament_index_python.packages import get_package_share_directory

            self.model_xml = str(
                Path(get_package_share_directory("oculus_reader"))
                / "assets" / "openarm_mujoco" / "v1" / "scene.xml"
            )
        if not self.model_xml or not Path(self.model_xml).is_file():
            raise ValueError(
                f"model_xml must name an existing OpenArm v1 MJCF: {self.model_xml!r}"
            )
        self.origin_frame = str(self.get_parameter("origin_frame").value)
        self.right_joint_names = validate_joint_names(
            list(self.get_parameter("right_joint_names").value), "right_joint_names"
        )
        self.left_joint_names = validate_joint_names(
            list(self.get_parameter("left_joint_names").value), "left_joint_names"
        )
        overlap = set(self.right_joint_names) & set(self.left_joint_names)
        if overlap:
            raise ValueError(f"left/right joint name sets overlap: {sorted(overlap)}")

        self._lower = {
            "right": self._seven_values("right_lower_limits"),
            "left": self._seven_values("left_lower_limits"),
        }
        self._upper = {
            "right": self._seven_values("right_upper_limits"),
            "left": self._seven_values("left_upper_limits"),
        }
        self._hard_lower = {
            "right": tuple(OPENARM_RIGHT_JOINT_HARD_LOWER_RAD),
            "left": tuple(OPENARM_LEFT_JOINT_HARD_LOWER_RAD),
        }
        self._hard_upper = {
            "right": tuple(OPENARM_RIGHT_JOINT_HARD_UPPER_RAD),
            "left": tuple(OPENARM_LEFT_JOINT_HARD_UPPER_RAD),
        }
        for side in ("right", "left"):
            if any(lo >= hi for lo, hi in zip(self._lower[side], self._upper[side])):
                raise ValueError(f"{side} lower joint limits must be below upper limits")
            if any(
                soft_lo < hard_lo - 1e-9 or soft_hi > hard_hi + 1e-9
                for soft_lo, soft_hi, hard_lo, hard_hi in zip(
                    self._lower[side],
                    self._upper[side],
                    self._hard_lower[side],
                    self._hard_upper[side],
                )
            ):
                raise ValueError(
                    f"{side} teleoperation soft limits must remain inside the "
                    "official hard limits"
                )

        self.right_gripper_name = str(self.get_parameter("right_gripper_joint_name").value)
        self.left_gripper_name = str(self.get_parameter("left_gripper_joint_name").value)
        self._gripper = {
            "right": float(self.get_parameter("right_gripper_hold").value),
            "left": float(self.get_parameter("left_gripper_hold").value),
        }
        if not all(math.isfinite(value) for value in self._gripper.values()):
            raise ValueError("configured gripper hold values must be finite")

        self.solve_rate_hz = max(1.0, float(self.get_parameter("solve_rate_hz").value))
        self.feedback_timeout_sec = float(self.get_parameter("feedback_timeout_sec").value)
        self.target_timeout_sec = float(self.get_parameter("target_timeout_sec").value)
        self.mode_timeout_sec = float(self.get_parameter("mode_timeout_sec").value)
        self.guard_ack_timeout_sec = float(
            self.get_parameter("guard_ack_timeout_sec").value
        )
        self.guard_anchor_timeout_sec = float(
            self.get_parameter("guard_anchor_timeout_sec").value
        )
        self.bimanual_target_max_skew_sec = float(
            self.get_parameter("bimanual_target_max_skew_sec").value
        )
        timeout_values = (
            self.feedback_timeout_sec,
            self.target_timeout_sec,
            self.mode_timeout_sec,
            self.guard_ack_timeout_sec,
            self.guard_anchor_timeout_sec,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in timeout_values):
            raise ValueError("IK timeouts must be positive")
        if (
            not math.isfinite(self.bimanual_target_max_skew_sec)
            or self.bimanual_target_max_skew_sec < 0.0
        ):
            raise ValueError("bimanual_target_max_skew_sec must be finite and non-negative")
        self._max_tracking_error = self._seven_values("max_tracking_error_rad")
        if any(value <= 0.0 for value in self._max_tracking_error):
            raise ValueError("max_tracking_error_rad values must be positive")

        self._joints: dict[str, tuple[float, ...] | None] = {"right": None, "left": None}
        self._joint_received_ns: dict[str, int | None] = {"right": None, "left": None}
        self._target: dict[str, tuple[float, ...] | None] = {"right": None, "left": None}
        self._target_received_ns: dict[str, int | None] = {"right": None, "left": None}
        self._target_epoch_ns: dict[str, int | None] = {"right": None, "left": None}
        self._target_history = {
            side: deque(maxlen=TARGET_HISTORY_CAPACITY)
            for side in ("right", "left")
        }
        self._last_selected_target_epoch_ns: dict[str, int | None] = {
            "right": None,
            "left": None,
        }
        self._mode: dict[str, SideMode] = {
            "right": SideMode.FAULT,
            "left": SideMode.FAULT,
        }
        self._mode_token: dict[str, int | None] = {"right": None, "left": None}
        self._mode_received_ns: dict[str, int | None] = {"right": None, "left": None}
        self._guard_anchor: dict[str, tuple[float, ...] | None] = {
            "right": None,
            "left": None,
        }
        self._guard_anchor_received_ns: dict[str, int | None] = {
            "right": None,
            "left": None,
        }
        self._guard_anchor_tokens: dict[str, tuple[int, int] | None] = {
            "right": None,
            "left": None,
        }
        self._teleop_engaged_ns: dict[str, int | None] = {
            "right": None,
            "left": None,
        }
        self._accepted_command: dict[str, tuple[float, ...] | None] = {
            "right": None,
            "left": None,
        }
        self._pending_ik: dict[
            str,
            tuple[tuple[float, ...], tuple[float, ...], tuple[int, int]] | None,
        ] = {"right": None, "left": None}
        self._pending_acknowledged = {"right": False, "left": False}
        self._pending_started_ns: dict[str, int | None] = {
            "right": None,
            "left": None,
        }
        self._guard_ack_fault = {"right": False, "left": False}
        self._sync_wait_active = {"right": False, "left": False}
        self._solver_max_step: dict[str, tuple[float, ...]] = {}
        self._joint_soft_limit_active = {"right": False, "left": False}
        self._last_warning_ns = 0

        self.right_ik_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("right_ik_topic").value), 1
        )
        self.left_ik_pub = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("left_ik_topic").value), 1
        )
        self.right_tcp_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("right_tcp_pose_topic").value), 1
        )
        self.left_tcp_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("left_tcp_pose_topic").value), 1
        )
        self.sync_wait_pub = {
            side: self.create_publisher(
                UInt64,
                str(self.get_parameter(f"{side}_sync_wait_topic").value),
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
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("right_target_intent_topic").value),
            lambda msg: self._target_callback("right", msg),
            1,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("left_target_intent_topic").value),
            lambda msg: self._target_callback("left", msg),
            1,
        )
        self.create_subscription(
            UInt64,
            str(self.get_parameter("right_mode_topic").value),
            lambda msg: self._mode_callback("right", msg),
            1,
        )
        self.create_subscription(
            UInt64,
            str(self.get_parameter("left_mode_topic").value),
            lambda msg: self._mode_callback("left", msg),
            1,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("right_guard_acceptance_topic").value),
            lambda msg: self._guard_acceptance_callback("right", msg),
            1,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("left_guard_acceptance_topic").value),
            lambda msg: self._guard_acceptance_callback("left", msg),
            1,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("right_guard_anchor_topic").value),
            lambda msg: self._guard_anchor_callback("right", msg),
            1,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("left_guard_anchor_topic").value),
            lambda msg: self._guard_anchor_callback("left", msg),
            1,
        )

        # Deliberately import the heavy official stack only when this ROS node is
        # instantiated. Pure-Python tests and other Quest nodes do not require it.
        self.kinematics = self._create_official_kinematics()
        self.timer = self.create_timer(1.0 / self.solve_rate_hz, self._solve_once)
        self.get_logger().info(
            f"Official OpenArm bimanual IK ready: model={self.model_xml}, "
            f"origin={self.origin_frame}, rate={self.solve_rate_hz:.1f}Hz"
        )

    def _velocity_limits(self, max_iters: int, dt: float) -> dict[str, float]:
        result: dict[str, float] = {}
        for side, names, parameter in (
            ("right", self.right_joint_names, "right_max_velocity_rad_s"),
            ("left", self.left_joint_names, "left_max_velocity_rad_s"),
        ):
            caps = [float(value) for value in self.get_parameter(parameter).value]
            if len(caps) != 7 or any(value <= 0.0 or not math.isfinite(value) for value in caps):
                raise ValueError(f"{parameter} must contain seven finite positive values")
            # The official QP integrates max_iters times per ROS tick. Scale its
            # internal velocity bound so the whole tick respects physical rad/s.
            scale = max_iters * dt * self.solve_rate_hz
            for name, cap in zip(names, caps):
                result[name] = cap / scale
            self._solver_max_step[side] = tuple(
                cap / self.solve_rate_hz for cap in caps
            )
            self.get_logger().info(f"{side} IK velocity caps(rad/s)={caps}")
        return result

    def _seven_values(self, parameter: str) -> tuple[float, ...]:
        values = tuple(float(value) for value in self.get_parameter(parameter).value)
        if len(values) != 7 or not all(math.isfinite(value) for value in values):
            raise ValueError(f"{parameter} must contain seven finite values")
        return values

    def _verify_official_hard_joint_limits(self, setup) -> None:
        """Verify the pinned model's hard ranges without narrowing Mink's model."""

        for side, names in (
            ("right", self.right_joint_names),
            ("left", self.left_joint_names),
        ):
            for name, expected_lower, expected_upper in zip(
                names, self._hard_lower[side], self._hard_upper[side]
            ):
                try:
                    joint = setup.model.joint(name)
                except KeyError as exc:
                    raise ValueError(
                        f"official OpenArm model is missing configured joint {name!r}"
                    ) from exc
                model_lower, model_upper = (float(value) for value in joint.range)
                if not bool(setup.model.jnt_limited[joint.id]):
                    raise ValueError(
                        f"official OpenArm model joint {name!r} is not limited"
                    )
                if not (
                    math.isclose(
                        model_lower, expected_lower, rel_tol=0.0, abs_tol=1e-12
                    )
                    and math.isclose(
                        model_upper, expected_upper, rel_tol=0.0, abs_tol=1e-12
                    )
                ):
                    raise ValueError(
                        f"official OpenArm model joint {name!r} range "
                        f"[{model_lower}, {model_upper}] does not match the pinned "
                        f"hard range [{expected_lower}, {expected_upper}]"
                    )
            self.get_logger().info(
                f"{side} official IK hard limits verified; 95% teleoperation "
                f"soft limits(rad): lower={list(self._lower[side])}, "
                f"upper={list(self._upper[side])}"
            )

    def _create_official_kinematics(self):
        try:
            from openarm_control import ArmSetup, IKParams, Kinematics
        except ImportError as exc:
            raise RuntimeError(
                "openarm-control is unavailable in this Python environment; "
                "build/use the OpenArm Docker image"
            ) from exc

        max_iters = int(self.get_parameter("max_iters").value)
        dt = float(self.get_parameter("dt").value)
        if max_iters <= 0 or dt <= 0.0:
            raise ValueError("max_iters and dt must be positive")
        keyframe_value = str(self.get_parameter("keyframe").value).strip()
        setup = ArmSetup.from_args(
            xml=self.model_xml,
            mode="bimanual",
            frame_right=str(self.get_parameter("frame_right").value),
            frame_type_right="body",
            frame_left=str(self.get_parameter("frame_left").value),
            frame_type_left="body",
            keyframe=keyframe_value or None,
            origin_frame=self.origin_frame,
            origin_frame_type="site",
        )
        # Mink must retain the official 100% hard model. Narrowing its
        # ConfigurationLimit to 95% can make takeover infeasible when measured
        # feedback starts legally inside the hard range but outside the soft
        # range. The output layer below applies a no-worsening soft saturation.
        self._verify_official_hard_joint_limits(setup)
        params = IKParams(
            position_cost=float(self.get_parameter("position_cost").value),
            orientation_cost=float(self.get_parameter("orientation_cost").value),
            lm_damping=float(self.get_parameter("lm_damping").value),
            damping=float(self.get_parameter("damping").value),
            solver=str(self.get_parameter("solver").value),
            posture_cost=float(self.get_parameter("posture_cost").value),
            dt=dt,
            max_iters=max_iters,
            velocity_limits=self._velocity_limits(max_iters, dt),
        )
        return Kinematics(setup, params)

    def _joint_callback(self, msg: JointState) -> None:
        now_ns = time.monotonic_ns()
        for side, required in (
            ("right", self.right_joint_names),
            ("left", self.left_joint_names),
        ):
            try:
                self._joints[side] = map_named_positions(msg.name, msg.position, required)
                self._joint_received_ns[side] = now_ns
            except ValueError:
                # Do not refresh a side when any of its seven arm axes is absent.
                pass

        position_by_name = {
            str(name): float(msg.position[index])
            for index, name in enumerate(msg.name)
            if index < len(msg.position) and math.isfinite(float(msg.position[index]))
        }
        if self.right_gripper_name in position_by_name:
            self._gripper["right"] = position_by_name[self.right_gripper_name]
        if self.left_gripper_name in position_by_name:
            self._gripper["left"] = position_by_name[self.left_gripper_name]

    def _target_callback(self, side: str, msg: Float64MultiArray) -> None:
        try:
            target, epoch_ns, raw_token = parse_atomic_target(msg.data)
        except ValueError as exc:
            self._target_history[side].clear()
            self._target[side] = None
            self._target_received_ns[side] = None
            self._target_epoch_ns[side] = None
            self.get_logger().error(f"Rejecting invalid {side} target: {exc}")
            return

        # Pose debug and mode intent travel on separate ROS topics. Only an
        # atomically token-bound target from this side's current TELEOP session
        # may refresh solver state; delayed samples from older sessions are
        # ignored without disturbing a newer valid target.
        current_tokens = self._current_intent_tokens()
        if (
            current_tokens is None
            or raw_token != self._mode_token[side]
            or self._mode[side] != SideMode.TELEOP
        ):
            return
        receipt_ns = time.monotonic_ns()
        self._target_history[side].append(
            TargetHistorySample(
                pose=target,
                receipt_ns=receipt_ns,
                source_epoch_ns=epoch_ns,
                intent_tokens=current_tokens,
            )
        )
        self._target[side] = target
        self._target_received_ns[side] = receipt_ns
        self._target_epoch_ns[side] = epoch_ns

    def _mode_callback(self, side: str, msg: UInt64) -> None:
        now_ns = time.monotonic_ns()
        raw_token = int(msg.data)
        try:
            _, new_mode = decode_intent_token(raw_token)
        except ValueError as exc:
            self.get_logger().error(
                f"Rejecting invalid {side} mode intent token {raw_token}: {exc}"
            )
            self._mode[side] = SideMode.FAULT
            self._mode_token[side] = None
            self._mode_received_ns[side] = now_ns
            self._reset_bimanual_intent_session(now_ns)
            return

        token_changed = self._mode_token[side] != raw_token
        self._mode[side] = new_mode
        self._mode_token[side] = raw_token
        self._mode_received_ns[side] = now_ns
        if token_changed:
            # Each raw token carries the delta node's TELEOP-boundary epoch.
            # Reset both sides even when the decoded mode remains TELEOP: a
            # release/re-engage message may have been overwritten in a depth-1
            # queue, but the final epoch still invalidates every older solve.
            self._reset_bimanual_intent_session(now_ns)

    def _guard_anchor_callback(
        self, side: str, msg: Float64MultiArray
    ) -> None:
        """Record a hard-bounded, dual-token anchor from the final guard."""

        now_ns = time.monotonic_ns()
        try:
            anchor, intent_tokens = parse_guard_anchor(msg.data)
            if any(
                value < lower or value > upper
                for value, lower, upper in zip(
                    anchor, self._hard_lower[side], self._hard_upper[side]
                )
            ):
                raise ValueError("guard controller anchor exceeds hard limits")
        except (TypeError, ValueError) as exc:
            self._warn_throttled(
                f"Ignoring invalid {side} guard controller anchor: {exc}", now_ns
            )
            return
        current_tokens = self._current_intent_tokens()
        if current_tokens is None or intent_tokens != current_tokens:
            self._warn_throttled(
                f"Ignoring stale {side} guard controller anchor intent tokens",
                now_ns,
            )
            return

        pending_token_sets = {
            pending[2]
            for pending in self._pending_ik.values()
            if pending is not None
        }
        if pending_token_sets and pending_token_sets != {intent_tokens}:
            # Defensive recovery for a valid current anchor racing an older
            # pending round. Cancel both sides atomically; never wait for an ACK
            # that the guard is required to reject under the new token pair.
            self._reset_bimanual_intent_session(now_ns)
        self._guard_anchor[side] = anchor
        self._guard_anchor_received_ns[side] = now_ns
        self._guard_anchor_tokens[side] = intent_tokens

    def _guard_acceptance_callback(
        self, side: str, msg: Float64MultiArray
    ) -> None:
        """Accept only an acknowledgement bound to this side's pending step."""

        now_ns = time.monotonic_ns()
        try:
            (
                accepted,
                echoed_candidate,
                echoed_base,
                acknowledged_tokens,
            ) = parse_guard_acceptance(msg.data)
        except (TypeError, ValueError) as exc:
            self._warn_throttled(
                f"Ignoring invalid {side} guard acknowledgement: {exc}", now_ns
            )
            return
        pending = self._pending_ik[side]
        if self._mode[side] != SideMode.TELEOP or pending is None:
            return
        # The first valid acknowledgement closes this side's round. Later
        # duplicates are retries of the same ROS delivery and must never move
        # the solver base a second time.
        if self._pending_acknowledged[side]:
            return
        pending_candidate, pending_base, pending_tokens = pending
        current_tokens = self._current_intent_tokens()
        if (
            current_tokens is None
            or acknowledged_tokens != current_tokens
            or acknowledged_tokens != pending_tokens
        ):
            self._warn_throttled(
                f"Ignoring stale {side} guard acknowledgement intent tokens",
                now_ns,
            )
            return
        exact_echo = all(
            math.isclose(received, expected, rel_tol=0.0, abs_tol=1e-12)
            for received, expected in zip(
                echoed_candidate + echoed_base,
                pending_candidate + pending_base,
            )
        )
        if not exact_echo:
            self._warn_throttled(
                f"Ignoring unmatched {side} guard acknowledgement", now_ns
            )
            return
        if any(
            value < lower or value > upper
            for value, lower, upper in zip(
                accepted, self._hard_lower[side], self._hard_upper[side]
            )
        ):
            self._warn_throttled(
                f"Ignoring out-of-range {side} guard acknowledgement", now_ns
            )
            return
        accepted_step = validate_guard_acceptance_progress(
            accepted_command=accepted,
            candidate=pending_candidate,
            solver_base=pending_base,
            max_step=self._solver_max_step[side],
        )
        if not accepted_step.accepted or accepted_step.command is None:
            self._warn_throttled(
                f"Ignoring invalid {side} guard acknowledgement: "
                f"{accepted_step.reason}",
                now_ns,
            )
            return
        self._accepted_command[side] = accepted_step.command
        self._pending_acknowledged[side] = True

    def _effective_mode(self, side: str, now_ns: int) -> SideMode:
        if not is_fresh(now_ns, self._mode_received_ns[side], self.mode_timeout_sec):
            return SideMode.FAULT
        # Target readiness is a separate all-active-side barrier below. Never
        # silently remove a late TELEOP peer from the bimanual cohort merely
        # because its atomic target has not crossed ROS yet.
        return self._mode[side]

    def _publish_tcp(self, side: str, pose_wxyz, stamp) -> None:
        pose = tuple(float(value) for value in pose_wxyz)
        if len(pose) != 7 or not all(math.isfinite(value) for value in pose):
            raise ValueError(f"official FK returned invalid {side} pose")
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.origin_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pose[:3]
        msg.pose.orientation.w = pose[3]
        msg.pose.orientation.x = pose[4]
        msg.pose.orientation.y = pose[5]
        msg.pose.orientation.z = pose[6]
        (self.right_tcp_pub if side == "right" else self.left_tcp_pub).publish(msg)

    def _warn_throttled(self, message: str, now_ns: int) -> None:
        if now_ns - self._last_warning_ns >= 1_000_000_000:
            self.get_logger().warn(message)
            self._last_warning_ns = now_ns

    def _reset_pending_handshake(self, side: str | None = None) -> None:
        selected = (side,) if side is not None else ("right", "left")
        for current in selected:
            self._pending_ik[current] = None
            self._pending_acknowledged[current] = False
            self._pending_started_ns[current] = None
            self._guard_ack_fault[current] = False

    def _reset_solver_handshake(self, side: str | None = None) -> None:
        selected = (side,) if side is not None else ("right", "left")
        for current in selected:
            self._accepted_command[current] = None
        self._reset_pending_handshake(side)

    def _apply_commit_gate_failure(
        self,
        *,
        target_only_expiry: bool,
        active_sides: tuple[str, ...],
    ) -> None:
        """Apply the fail-closed state transition chosen by the commit gate."""

        if target_only_expiry:
            # Retain only the exact command already acknowledged for this
            # current token pair. No candidate, ACK wait, or ACK-fault state may
            # cross into the bounded no-command synchronization HOLD.
            self._reset_pending_handshake()
            self._publish_sync_wait(active_sides)
            return
        self._reset_solver_handshake()
        self._publish_sync_wait(())

    def _current_intent_tokens(self) -> tuple[int, int] | None:
        right_token = self._mode_token["right"]
        left_token = self._mode_token["left"]
        if right_token is None or left_token is None:
            return None
        return right_token, left_token

    def _latest_current_target_sample(
        self,
        side: str,
        now_ns: int,
        intent_tokens: tuple[int, int],
        *,
        enforce_watermark: bool,
    ) -> TargetHistorySample | None:
        """Return this side's newest fresh sample for the current token pair."""

        watermark = self._last_selected_target_epoch_ns[side]
        minimum_epoch = (
            0 if not enforce_watermark or watermark is None else watermark
        )
        return max(
            (
                sample
                for sample in self._target_history[side]
                if sample.intent_tokens == intent_tokens
                and sample.source_epoch_ns >= minimum_epoch
                and is_fresh(
                    now_ns,
                    sample.receipt_ns,
                    self.target_timeout_sec,
                )
            ),
            key=lambda sample: (sample.source_epoch_ns, sample.receipt_ns),
            default=None,
        )

    def _reset_bimanual_intent_session(self, now_ns: int) -> None:
        """Invalidate both sides after any raw mode-intent token change."""

        self._reset_solver_handshake()
        for side in ("right", "left"):
            # Target poses are published on different topics from the atomic
            # mode intents. A target that was fresh just before a release or
            # peer-session edge must not seed the first solve of the new pair.
            self._target[side] = None
            self._target_received_ns[side] = None
            self._target_epoch_ns[side] = None
            self._target_history[side].clear()
            self._last_selected_target_epoch_ns[side] = None
            self._guard_anchor[side] = None
            self._guard_anchor_received_ns[side] = None
            self._guard_anchor_tokens[side] = None
            self._sync_wait_active[side] = False
            self._teleop_engaged_ns[side] = (
                now_ns if self._mode[side] == SideMode.TELEOP else None
            )

    def _publish_pending_ik(self, side: str) -> None:
        pending = self._pending_ik[side]
        if pending is None:
            return
        candidate, solver_base, intent_tokens = pending
        publisher = self.right_ik_pub if side == "right" else self.left_ik_pub
        publisher.publish(
            Float64MultiArray(
                data=list(
                    build_atomic_ik_command(
                        candidate,
                        solver_base,
                        intent_tokens,
                    )
                )
            )
        )

    def _publish_sync_wait(self, active_sides: tuple[str, ...]) -> None:
        """Heartbeat established sides that are safely waiting for a peer.

        This message never carries or authorizes a command.  It lets the final
        guard distinguish a live, intentional bimanual synchronization HOLD
        from an IK process that has stopped producing data.
        """

        waiting = {
            side
            for side in active_sides
            if self._accepted_command[side] is not None
        }
        generated_ns = time.monotonic_ns()
        for side in ("right", "left"):
            should_wait = side in waiting
            if should_wait:
                self.sync_wait_pub[side].publish(UInt64(data=generated_ns))
            elif self._sync_wait_active[side]:
                self.sync_wait_pub[side].publish(UInt64(data=0))
            self._sync_wait_active[side] = should_wait

    def _handoff_sync_wait_to_pending_ik(self) -> None:
        """Stop local heartbeats without racing a zero ahead of fresh IK.

        Call this only after every active candidate has passed validation and
        immediately before publishing the new pending IK round.  The final
        guard retains the last positive heartbeat until its new-IK callback
        clears that lease.  If publication stalls or this process stops, the
        unrefreshed heartbeat still expires at the guard's ordinary IK timeout.
        """

        for side in ("right", "left"):
            self._sync_wait_active[side] = False

    def _solve_once(self) -> None:
        now_ns = time.monotonic_ns()
        for side in ("right", "left"):
            if not is_fresh(
                now_ns, self._joint_received_ns[side], self.feedback_timeout_sec
            ):
                # No accepted-command handshake may bridge a feedback outage.
                # The next healthy engagement must restart from measured state.
                self._reset_solver_handshake()
                self._publish_sync_wait(())
                self._warn_throttled(
                    f"IK suppressed: complete seven-axis {side} feedback is stale", now_ns
                )
                return
        assert self._joints["right"] is not None and self._joints["left"] is not None

        feedback_driver_state = build_driver_state(
            self._joints["right"],
            self._gripper["right"],
            self._joints["left"],
            self._gripper["left"],
        )
        feedback_state = np.asarray(feedback_driver_state, dtype=np.float32)
        right_feedback, left_feedback = split_driver_state(feedback_state)
        effective_modes = {
            side: self._effective_mode(side, now_ns) for side in ("right", "left")
        }
        for side in ("right", "left"):
            if effective_modes[side] != SideMode.TELEOP:
                self._reset_solver_handshake(side)
        active_sides = tuple(
            side
            for side in ("right", "left")
            if effective_modes[side] == SideMode.TELEOP
        )
        intent_tokens = self._current_intent_tokens()
        if intent_tokens is None:
            self._reset_solver_handshake()
            self._publish_sync_wait(())
            self._warn_throttled(
                "IK suppressed: complete right/left mode intent tokens are unavailable",
                now_ns,
            )
            return

        try:
            # FK remains an observation of the real robot, never an accepted or
            # pending controller command. It supplies public TCP feedback and
            # the internal hold target for a non-TELEOP side.
            right_fk, left_fk = self.kinematics.fk_bimanual(
                np.asarray(right_feedback, dtype=np.float32),
                np.asarray(left_feedback, dtype=np.float32),
            )
            stamp = self.get_clock().now().to_msg()
            self._publish_tcp("right", right_fk, stamp)
            self._publish_tcp("left", left_fk, stamp)

            # HOLD/FAULT/HOME needs FK only. Skipping the unused ten-iteration
            # QP removes the old high idle CPU load without changing TELEOP IK.
            if not active_sides:
                self._publish_sync_wait(())
                return

            # FK and ROS publication above are outside our control and may
            # block. Start every target/anchor/pending freshness decision from
            # a new monotonic sample, never the timer callback's entry time.
            fresh_now_ns = time.monotonic_ns()
            fresh_joint_feedback = all(
                is_fresh(
                    fresh_now_ns,
                    self._joint_received_ns[side],
                    self.feedback_timeout_sec,
                )
                for side in ("right", "left")
            )
            fresh_mode_cohort = (
                self._current_intent_tokens() == intent_tokens
                and all(
                    self._mode[side] == SideMode.TELEOP
                    and is_fresh(
                        fresh_now_ns,
                        self._mode_received_ns[side],
                        self.mode_timeout_sec,
                    )
                    for side in active_sides
                )
            )
            fresh_current_targets = {
                side: self._latest_current_target_sample(
                    side,
                    fresh_now_ns,
                    intent_tokens,
                    enforce_watermark=False,
                )
                for side in active_sides
            }
            if (
                any(sample is None for sample in fresh_current_targets.values())
                or not fresh_mode_cohort
                or not fresh_joint_feedback
            ):
                # No pending payload may outlive the fresh target observation
                # and live mode/feedback cohort that authorized it. Revoke both
                # sides before the retry branch so stale causal state cannot
                # refresh guard IK freshness.
                self._reset_solver_handshake()
                self._publish_sync_wait(())
                self._warn_throttled(
                    "IK waiting for fresh current-token targets, active TELEOP "
                    "mode heartbeats, and bimanual feedback",
                    fresh_now_ns,
                )
                return

            pending_sides = tuple(
                side for side in active_sides if self._pending_ik[side] is not None
            )
            if pending_sides:
                self._publish_sync_wait(())
                if any(
                    self._pending_ik[side] is not None
                    and self._pending_ik[side][2] != intent_tokens
                    for side in pending_sides
                ):
                    self._reset_bimanual_intent_session(fresh_now_ns)
                    self._warn_throttled(
                        "IK cancelled a pending round with stale mode intent tokens",
                        fresh_now_ns,
                    )
                    return
                # Repeat the exact same atomic payload until the final guard
                # echoes what it actually published. Never integrate a second
                # official-IK step from an unacknowledged candidate.
                unacknowledged_sides = tuple(
                    side
                    for side in pending_sides
                    if not self._pending_acknowledged[side]
                )
                for side in unacknowledged_sides:
                    started_ns = self._pending_started_ns[side]
                    if (
                        started_ns is not None
                        and fresh_now_ns - started_ns
                        > int(self.guard_ack_timeout_sec * 1_000_000_000)
                    ):
                        self._guard_ack_fault[side] = True
                        self._warn_throttled(
                            f"{side} IK guard acknowledgement timed out; "
                            "release Grip to reset the fail-closed handshake",
                            fresh_now_ns,
                        )
                if any(self._guard_ack_fault[side] for side in active_sides):
                    return
                if unacknowledged_sides:
                    # Re-send every pending side, including one already ACKed,
                    # while waiting for its peer. This keeps the guard's strict
                    # IK freshness contract alive; the guard handles retries
                    # idempotently and never repeats the controller command.
                    for side in pending_sides:
                        self._publish_pending_ik(side)
                    return
                for side in pending_sides:
                    self._pending_ik[side] = None
                    self._pending_acknowledged[side] = False
                    self._pending_started_ns[side] = None

            # A fresh Grip engagement cannot start from measured feedback: the
            # position controller may still retain a safe command that feedback
            # has not reached yet.  Wait for the final guard to publish that
            # exact retained command after this engagement, then independently
            # verify its hard limits and tracking distance before solving. Any
            # older pending round above is serviced first so its peer never
            # loses IK freshness while this asynchronous anchor crosses ROS.
            feedback_by_side = {
                "right": right_feedback[:7],
                "left": left_feedback[:7],
            }
            for side in active_sides:
                if self._accepted_command[side] is not None:
                    continue
                anchor_received_ns = self._guard_anchor_received_ns[side]
                if not is_post_engagement(
                    anchor_received_ns, self._teleop_engaged_ns[side]
                ) or not is_fresh(
                    fresh_now_ns,
                    anchor_received_ns,
                    self.guard_anchor_timeout_sec,
                ):
                    self._warn_throttled(
                        f"IK waiting for a fresh post-engagement {side} "
                        "controller anchor from the final guard",
                        fresh_now_ns,
                    )
                    self._publish_sync_wait(active_sides)
                    return
                if self._guard_anchor_tokens[side] != intent_tokens:
                    self._warn_throttled(
                        f"IK waiting for a dual-token-current {side} controller anchor",
                        fresh_now_ns,
                    )
                    self._publish_sync_wait(active_sides)
                    return
                anchor = self._guard_anchor[side]
                if anchor is None:
                    self._warn_throttled(
                        f"IK waiting for an available {side} controller anchor",
                        fresh_now_ns,
                    )
                    self._publish_sync_wait(active_sides)
                    return
                anchor_decision = validate_guard_anchor(
                    anchor=anchor,
                    feedback=feedback_by_side[side],
                    lower_limits=self._hard_lower[side],
                    upper_limits=self._hard_upper[side],
                    max_tracking_error=self._max_tracking_error,
                )
                if (
                    not anchor_decision.accepted
                    or anchor_decision.command is None
                ):
                    self._publish_sync_wait(())
                    self._warn_throttled(
                        f"IK rejected {side} controller anchor: "
                        f"{anchor_decision.reason}",
                        fresh_now_ns,
                    )
                    return
                # Retain the canonical tuple returned by the shared validator.
                self._guard_anchor[side] = anchor_decision.command

            selected_targets: dict[str, TargetHistorySample] = {}
            if len(active_sides) == 2:
                selected_pair = select_coherent_target_history_pair(
                    self._target_history["right"],
                    self._target_history["left"],
                    now_ns=fresh_now_ns,
                    timeout_sec=self.target_timeout_sec,
                    max_skew_ns=int(
                        self.bimanual_target_max_skew_sec * 1_000_000_000
                    ),
                    current_intent_tokens=intent_tokens,
                    last_selected_epoch_ns=(
                        self._last_selected_target_epoch_ns["right"],
                        self._last_selected_target_epoch_ns["left"],
                    ),
                )
                if selected_pair is None:
                    # Both current-token streams are fresh, so this is a
                    # bounded epoch-skew/watermark synchronization wait. Only
                    # already-established sides receive the existing no-command
                    # lease; an unestablished side never extends its deadline.
                    self._publish_sync_wait(active_sides)
                    self._warn_throttled(
                        "IK waiting for a non-regressing left/right target pair "
                        "within the configured Quest epoch skew",
                        fresh_now_ns,
                    )
                    return
                selected_targets = {
                    "right": selected_pair[0],
                    "left": selected_pair[1],
                }
            else:
                side = active_sides[0]
                selected = self._latest_current_target_sample(
                    side,
                    fresh_now_ns,
                    intent_tokens,
                    enforce_watermark=True,
                )
                if selected is None:
                    self._publish_sync_wait(())
                    self._warn_throttled(
                        f"IK waiting for a non-regressing {side} target",
                        fresh_now_ns,
                    )
                    return
                selected_targets[side] = selected

            # Preserve the last synchronization heartbeat during this bounded
            # solver-to-IK handoff. Do not publish zero ahead of the fresh IK
            # on a different ROS topic, and do not refresh True here. Every
            # handled failure below explicitly withdraws the lease; a blocked
            # solver/node leaves it unrefreshed so the guard's normal IK timeout
            # still fails closed.

            # Snapshot which solver bases are controller anchors. A later
            # acknowledgement must not allow the commit gate to forget that
            # this round was actually solved from a fresh-engagement anchor.
            anchor_based_sides = tuple(
                side
                for side in active_sides
                if self._accepted_command[side] is None
            )
            right_solver_joints = select_solver_base(
                mode=effective_modes["right"],
                feedback=right_feedback[:7],
                accepted_command=self._accepted_command["right"],
                controller_anchor=self._guard_anchor["right"],
            )
            left_solver_joints = select_solver_base(
                mode=effective_modes["left"],
                feedback=left_feedback[:7],
                accepted_command=self._accepted_command["left"],
                controller_anchor=self._guard_anchor["left"],
            )
            # Preserve the exact float32 state consumed by the official solver.
            # TELEOP advances from the final guard's acknowledged command; a
            # fresh engagement starts from the final guard's validated retained
            # controller anchor. This retains static-friction progress without
            # allowing a free-running shadow or a feedback/base discontinuity.
            solver_state = np.asarray(
                build_driver_state(
                    right_solver_joints,
                    right_feedback[7],
                    left_solver_joints,
                    left_feedback[7],
                ),
                dtype=np.float32,
            )
            right_solver_base, left_solver_base = split_driver_state(solver_state)
            self.kinematics.sync(solver_state)
            for side, hold_pose in (("right", right_fk), ("left", left_fk)):
                selected_target = selected_targets.get(side)
                solver_target = select_solver_target(
                    effective_modes[side],
                    None if selected_target is None else selected_target.pose,
                    hold_pose,
                )
                self.kinematics.set_target(
                    side, np.asarray(solver_target, dtype=np.float32)
                )

            if not self.kinematics.ready():
                self._publish_sync_wait(())
                self._warn_throttled("IK suppressed: shared solver lacks a side target", now_ns)
                return
            result = self.kinematics.solve()
        except (RuntimeError, ValueError) as exc:
            self._publish_sync_wait(())
            self._warn_throttled(f"IK cycle rejected: {exc}", now_ns)
            return

        if result is None:
            self._publish_sync_wait(())
            self._warn_throttled("Official IK returned no solution", now_ns)
            return
        try:
            right_result, left_result = split_driver_state(result)
        except (TypeError, ValueError) as exc:
            self._publish_sync_wait(())
            self._warn_throttled(f"Invalid official IK result: {exc}", now_ns)
            return

        results = {"right": right_result, "left": left_result}
        solver_bases = {"right": right_solver_base, "left": left_solver_base}
        validated_candidates: dict[str, tuple[float, ...]] = {}
        for side in ("right", "left"):
            if effective_modes[side] != SideMode.TELEOP:
                continue
            decision = validate_solver_candidate_step(
                results[side][:7],
                solver_bases[side][:7],
                self._solver_max_step[side],
            )
            if not decision.accepted or decision.command is None:
                # Neither side publishes when either TELEOP side violates the
                # exact per-tick contract.
                self._publish_sync_wait(())
                self._warn_throttled(
                    f"Invalid official {side} IK step: {decision.reason}", now_ns
                )
                return
            try:
                candidate = apply_joint_soft_limits(
                    decision.command,
                    solver_bases[side][:7],
                    self._lower[side],
                    self._upper[side],
                )
            except (TypeError, ValueError) as exc:
                self._publish_sync_wait(())
                self._warn_throttled(
                    f"Invalid official {side} IK soft limit: {exc}", now_ns
                )
                return
            bounded_decision = validate_solver_candidate_step(
                candidate,
                solver_bases[side][:7],
                self._solver_max_step[side],
            )
            if not bounded_decision.accepted or bounded_decision.command is None:
                self._publish_sync_wait(())
                self._warn_throttled(
                    f"Invalid soft-limited {side} IK step: "
                    f"{bounded_decision.reason}",
                    now_ns,
                )
                return
            candidate = bounded_decision.command
            soft_limited = any(
                not math.isclose(raw, bounded, rel_tol=0.0, abs_tol=1e-12)
                for raw, bounded in zip(decision.command, candidate)
            )
            if soft_limited and not self._joint_soft_limit_active[side]:
                self.get_logger().warn(
                    f"{side} IK 95% joint soft-limit saturation active: "
                    f"raw={list(decision.command)}, saturated={list(candidate)}"
                )
            elif not soft_limited and self._joint_soft_limit_active[side]:
                self.get_logger().info(
                    f"{side} IK 95% joint soft-limit saturation released"
                )
            self._joint_soft_limit_active[side] = soft_limited
            validated_candidates[side] = candidate

        # The official solver and candidate validation may consume most of a
        # freshness budget. Re-sample time immediately before installing the
        # atomic pending round and revalidate every causal input that selected
        # its target and solver base. A target-history-only expiry may retain the
        # already-acknowledged base solely for bounded no-command HOLD; every
        # authority/live-input failure revokes both sides. Target watermarks are
        # advanced only after this gate.
        commit_ns = time.monotonic_ns()
        current_commit_tokens = self._current_intent_tokens()
        intent_tokens_still_current = current_commit_tokens == intent_tokens
        selected_targets_token_bound = all(
            selected_targets[side].intent_tokens == intent_tokens
            for side in active_sides
        )
        selected_targets_still_fresh = all(
            is_fresh(
                commit_ns,
                selected_targets[side].receipt_ns,
                self.target_timeout_sec,
            )
            for side in active_sides
        )
        current_target_streams_still_fresh = all(
            self._latest_current_target_sample(
                side,
                commit_ns,
                intent_tokens,
                enforce_watermark=False,
            )
            is not None
            for side in active_sides
        )
        modes_still_current = all(
            self._mode[side] == SideMode.TELEOP
            and is_fresh(
                commit_ns,
                self._mode_received_ns[side],
                self.mode_timeout_sec,
            )
            for side in active_sides
        )
        feedback_still_fresh = all(
            is_fresh(
                commit_ns,
                self._joint_received_ns[side],
                self.feedback_timeout_sec,
            )
            for side in ("right", "left")
        )
        anchors_still_current = all(
            self._guard_anchor[side] is not None
            and self._guard_anchor_tokens[side] == intent_tokens
            and is_post_engagement(
                self._guard_anchor_received_ns[side],
                self._teleop_engaged_ns[side],
            )
            and is_fresh(
                commit_ns,
                self._guard_anchor_received_ns[side],
                self.guard_anchor_timeout_sec,
            )
            for side in anchor_based_sides
        )
        target_only_expiry = target_only_commit_expiry_allows_sync_wait(
            intent_tokens_current=intent_tokens_still_current,
            selected_targets_token_bound=selected_targets_token_bound,
            selected_targets_fresh=selected_targets_still_fresh,
            current_target_streams_fresh=current_target_streams_still_fresh,
            modes_current=modes_still_current,
            feedback_fresh=feedback_still_fresh,
            anchors_current=anchors_still_current,
        )
        if target_only_expiry:
            # A fresh current-token stream superseded the now-expired coherent
            # historical pair while the solver ran. Discard this candidate but
            # retain the guard-acknowledged base, so established sides enter the
            # existing bounded no-command synchronization HOLD immediately.
            self._apply_commit_gate_failure(
                target_only_expiry=True,
                active_sides=active_sides,
            )
            self._warn_throttled(
                "IK discarded an expired coherent target pair and entered "
                "bounded synchronization HOLD",
                commit_ns,
            )
            return

        if not (
            intent_tokens_still_current
            and selected_targets_token_bound
            and selected_targets_still_fresh
            and current_target_streams_still_fresh
            and modes_still_current
            and feedback_still_fresh
            and anchors_still_current
        ):
            self._apply_commit_gate_failure(
                target_only_expiry=False,
                active_sides=active_sides,
            )
            self._warn_throttled(
                "IK discarded a solve whose causal inputs expired before "
                "pending install",
                commit_ns,
            )
            return

        # All active sides pass before any pending round is installed. Each
        # side now waits for a matching guard acceptance before the shared
        # official solver may advance again.
        for side, candidate in validated_candidates.items():
            self._pending_ik[side] = (
                candidate,
                tuple(float(value) for value in solver_bases[side][:7]),
                intent_tokens,
            )
            self._pending_acknowledged[side] = False
            self._pending_started_ns[side] = commit_ns
        for side, selected_target in selected_targets.items():
            self._last_selected_target_epoch_ns[side] = (
                selected_target.source_epoch_ns
            )
        self._handoff_sync_wait_to_pending_ik()
        for side in validated_candidates:
            self._publish_pending_ik(side)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OpenArmBimanualIK()
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
