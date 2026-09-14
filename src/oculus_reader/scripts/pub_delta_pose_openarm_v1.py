#!/usr/bin/env python3
"""Per-side Quest relative-pose intent node for OpenArm v1.

Unlike the Piper node, this process never publishes a hardware command. It
publishes pose/mode intent only; ``openarm_command_guard_node.py`` is the sole
publisher to the ros2_control forward-position controllers.
"""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Float32, Float64MultiArray, UInt64

from openarm_teleop_core import (
    LEFT_JOINT_NAMES,
    ReleaseToRearmLatch,
    RIGHT_JOINT_NAMES,
    SideMode,
    build_atomic_target,
    encode_intent_token,
    is_fresh,
    is_rclpy_humble_shutdown_take_race,
    map_named_positions,
    side_control_defaults,
    transition_intent_token,
    update_grip_deadman,
    validate_joint_names,
)


BUTTON_INDEX = {"LG": 0, "RG": 1, "A": 2, "B": 3, "X": 4, "Y": 5, "LJ": 6, "RJ": 7}
AXIS_INDEX = {
    "leftTrig": 0,
    "rightTrig": 1,
    "leftGrip": 2,
    "rightGrip": 3,
}
ALLOWED_BUTTONS = frozenset((*BUTTON_INDEX, "N"))


def pose_to_matrix(msg: PoseStamped) -> np.ndarray:
    values = (
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("pose contains a non-finite value")
    quat = np.asarray(values[3:], dtype=float)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        raise ValueError("pose quaternion has zero norm")
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = Rotation.from_quat(quat / norm).as_matrix()
    matrix[:3, 3] = np.asarray(values[:3], dtype=float)
    return matrix


def matrix_to_pose(matrix: np.ndarray, frame_id: str, stamp) -> PoseStamped:
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("target transform must be a finite 4x4 matrix")
    quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(matrix[0, 3])
    msg.pose.position.y = float(matrix[1, 3])
    msg.pose.position.z = float(matrix[2, 3])
    msg.pose.orientation.x = float(quat[0])
    msg.pose.orientation.y = float(quat[1])
    msg.pose.orientation.z = float(quat[2])
    msg.pose.orientation.w = float(quat[3])
    return msg


def compose_relative_target(
    zero_tcp: np.ndarray,
    start_handle: np.ndarray,
    current_handle: np.ndarray,
) -> np.ndarray:
    """Apply qrafty's world-frame relative controller motion.

    Each side independently latches the handle and TCP poses on its Grip edge.
    The upstream handle poses are already expressed in the robot FLU frame, so
    their world-frame translation and rotation deltas are applied directly to
    the latched TCP pose.
    """

    matrices = (zero_tcp, start_handle, current_handle)
    if any(matrix.shape != (4, 4) for matrix in matrices):
        raise ValueError("relative pose inputs must be 4x4 matrices")
    if not all(np.all(np.isfinite(matrix)) for matrix in matrices):
        raise ValueError("relative pose inputs must be finite")
    homogeneous_row = np.array((0.0, 0.0, 0.0, 1.0), dtype=float)
    if not all(
        np.allclose(matrix[3], homogeneous_row, atol=1e-9) for matrix in matrices
    ):
        raise ValueError("relative pose inputs must be homogeneous transforms")

    target = np.eye(4, dtype=float)
    delta_position = current_handle[:3, 3] - start_handle[:3, 3]
    target[:3, 3] = zero_tcp[:3, 3] + delta_position
    relative_rotation = current_handle[:3, :3] @ start_handle[:3, :3].T
    target[:3, :3] = relative_rotation @ zero_tcp[:3, :3]
    if not np.all(np.isfinite(target)):
        raise ValueError("relative target transform is non-finite")
    return target


class OpenArmDeltaPosePublisher(Node):
    """Preserve Quest relative control while exposing a safe OpenArm intent API."""

    def __init__(self):
        super().__init__("pub_delta_pose_openarm_v1_node")
        self.declare_parameter("side", "right")
        self.side = str(self.get_parameter("side").value).strip().lower()
        if self.side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")

        is_left = self.side == "left"
        default_joint_names = list(LEFT_JOINT_NAMES if is_left else RIGHT_JOINT_NAMES)
        default_hold, default_home, default_trigger = side_control_defaults(self.side)

        self.declare_parameter("handle_pose_topic", f"/{self.side}_handle_pose")
        self.declare_parameter("feedback_tcp_pose_topic", f"/{self.side}_arm/feedback/tcp_pose")
        self.declare_parameter("feedback_joint_topic", "/joint_states")
        self.declare_parameter("button_state_topic", "/oculus/buttons")
        self.declare_parameter("source_valid_topic", "/oculus/source_valid")
        self.declare_parameter("target_pose_topic", f"/openarm/target_pose/{self.side}")
        self.declare_parameter(
            "target_intent_topic", f"/openarm/target_intent/{self.side}"
        )
        self.declare_parameter("mode_topic", f"/openarm/mode/{self.side}")
        self.declare_parameter("gripper_intent_topic", f"/openarm/gripper_intent/{self.side}")
        self.declare_parameter("joint_names", default_joint_names)
        self.declare_parameter("hold_button", default_hold)
        self.declare_parameter("home_button", default_home)
        self.declare_parameter("trigger_axis", default_trigger)
        self.declare_parameter("grip_axis", f"{self.side}Grip")
        self.declare_parameter("grip_press_threshold", 0.55)
        self.declare_parameter("grip_release_threshold", 0.35)
        self.declare_parameter("trigger_release_threshold", 0.02)
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("input_timeout_sec", 0.25)
        self.declare_parameter("feedback_timeout_sec", 0.25)
        self.declare_parameter("target_frame_id", "arm_origin")
        self.declare_parameter("home_joint_positions", [0.0] * 7)
        self.declare_parameter("home_tolerance_rad", 0.02)
        self.declare_parameter("enable_home", False)
        self.declare_parameter("fault_rearm_release_sec", 0.50)

        self.joint_names = validate_joint_names(
            list(self.get_parameter("joint_names").value), "joint_names"
        )
        self.home_joint_positions = tuple(
            float(value) for value in self.get_parameter("home_joint_positions").value
        )
        if len(self.home_joint_positions) != len(self.joint_names):
            raise ValueError("home_joint_positions must match the seven joint_names")
        if not all(math.isfinite(value) for value in self.home_joint_positions):
            raise ValueError("home_joint_positions contains a non-finite value")

        self.hold_button = self._button_name(
            self.get_parameter("hold_button").value, "hold_button"
        )
        self.home_button = self._button_name(
            self.get_parameter("home_button").value, "home_button"
        )
        self.trigger_axis = str(self.get_parameter("trigger_axis").value)
        expected_trigger_axis = f"{self.side}Trig"
        if self.trigger_axis != expected_trigger_axis:
            raise ValueError(
                f"trigger_axis for {self.side} must be {expected_trigger_axis}"
            )
        self.grip_axis = str(self.get_parameter("grip_axis").value)
        expected_grip_axis = f"{self.side}Grip"
        if self.grip_axis != expected_grip_axis:
            raise ValueError(f"grip_axis for {self.side} must be {expected_grip_axis}")
        self.grip_press_threshold = float(
            self.get_parameter("grip_press_threshold").value
        )
        self.grip_release_threshold = float(
            self.get_parameter("grip_release_threshold").value
        )
        self.trigger_release_threshold = float(
            self.get_parameter("trigger_release_threshold").value
        )
        update_grip_deadman(
            raw_pressed=False,
            analog_value=0.0,
            previous=False,
            press_threshold=self.grip_press_threshold,
            release_threshold=self.grip_release_threshold,
        )
        if not (
            math.isfinite(self.trigger_release_threshold)
            and 0.0 <= self.trigger_release_threshold < 1.0
        ):
            raise ValueError("trigger_release_threshold must be in [0, 1)")

        self.control_rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.input_timeout_sec = float(self.get_parameter("input_timeout_sec").value)
        self.feedback_timeout_sec = float(self.get_parameter("feedback_timeout_sec").value)
        if self.input_timeout_sec <= 0.0 or self.feedback_timeout_sec <= 0.0:
            raise ValueError("input and feedback timeouts must be positive")
        self.target_frame_id = str(self.get_parameter("target_frame_id").value)
        self.home_tolerance_rad = float(self.get_parameter("home_tolerance_rad").value)
        if self.home_tolerance_rad <= 0.0:
            raise ValueError("home_tolerance_rad must be positive")
        self.enable_home = bool(self.get_parameter("enable_home").value)
        self.fault_rearm_release_sec = float(
            self.get_parameter("fault_rearm_release_sec").value
        )

        self.target_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("target_pose_topic").value), 1
        )
        self.target_intent_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("target_intent_topic").value),
            1,
        )
        self.mode_pub = self.create_publisher(
            UInt64, str(self.get_parameter("mode_topic").value), 1
        )
        self.gripper_pub = self.create_publisher(
            Float32, str(self.get_parameter("gripper_intent_topic").value), 1
        )

        self._handle_matrix: np.ndarray | None = None
        self._tcp_matrix: np.ndarray | None = None
        self._joint_feedback: tuple[float, ...] | None = None
        self._buttons: dict[str, Any] = {}
        self._source_valid = False
        self._handle_received_ns: int | None = None
        self._handle_source_stamp_ns: int | None = None
        self._tcp_received_ns: int | None = None
        self._joint_received_ns: int | None = None
        self._buttons_received_ns: int | None = None
        self._source_valid_received_ns: int | None = None
        self._teleop_active = False
        self._start_handle: np.ndarray | None = None
        self._zero_tcp: np.ndarray | None = None
        self._home_active = False
        self._home_button_previous = False
        self._last_mode = SideMode.HOLD
        self._intent_token = encode_intent_token(1, self._last_mode)
        self._last_fault_reason = ""
        self._fault_latch = ReleaseToRearmLatch(self.fault_rearm_release_sec)
        self._grip_active = False

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("handle_pose_topic").value),
            self._handle_callback,
            1,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("feedback_tcp_pose_topic").value),
            self._tcp_callback,
            1,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("feedback_joint_topic").value),
            self._joint_callback,
            1,
        )
        self.create_subscription(
            Joy,
            str(self.get_parameter("button_state_topic").value),
            self._button_callback,
            1,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("source_valid_topic").value),
            self._source_valid_callback,
            1,
        )
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.get_logger().info(
            f"OpenArm {self.side} intent ready: hold={self.hold_button}, "
            f"home={self.home_button}, joints={list(self.joint_names)}"
        )

    @staticmethod
    def _button_name(raw_value: Any, label: str) -> str:
        normalized = str(raw_value).strip().strip("'\"").upper()
        if normalized not in ALLOWED_BUTTONS:
            raise ValueError(f"{label} must be one of {sorted(ALLOWED_BUTTONS)}")
        return normalized

    def _handle_callback(self, msg: PoseStamped) -> None:
        try:
            self._handle_matrix = pose_to_matrix(msg)
            self._handle_received_ns = time.monotonic_ns()
            source_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
                msg.header.stamp.nanosec
            )
            self._handle_source_stamp_ns = source_stamp_ns if source_stamp_ns > 0 else None
        except ValueError as exc:
            self._handle_matrix = None
            self._handle_received_ns = None
            self._handle_source_stamp_ns = None
            self.get_logger().error(f"[{self.side}] invalid handle pose: {exc}")

    def _tcp_callback(self, msg: PoseStamped) -> None:
        try:
            self._tcp_matrix = pose_to_matrix(msg)
            self._tcp_received_ns = time.monotonic_ns()
        except ValueError as exc:
            self._tcp_matrix = None
            self._tcp_received_ns = None
            self.get_logger().error(f"[{self.side}] invalid TCP feedback: {exc}")

    def _joint_callback(self, msg: JointState) -> None:
        try:
            self._joint_feedback = map_named_positions(msg.name, msg.position, self.joint_names)
            self._joint_received_ns = time.monotonic_ns()
        except ValueError:
            # A shared /joint_states stream may contain partial messages. A partial
            # message must never refresh the seven-axis safety timestamp.
            return

    def _button_callback(self, msg: Joy) -> None:
        buttons: dict[str, Any] = {}
        for name, index in BUTTON_INDEX.items():
            buttons[name] = index < len(msg.buttons) and bool(msg.buttons[index])
        for name, index in AXIS_INDEX.items():
            buttons[name] = float(msg.axes[index]) if index < len(msg.axes) else 0.0
        self._buttons = buttons
        self._buttons_received_ns = time.monotonic_ns()

    def _source_valid_callback(self, msg: Bool) -> None:
        self._source_valid = bool(msg.data)
        self._source_valid_received_ns = time.monotonic_ns()

    def _fresh(self, stamp: int | None, timeout: float, now_ns: int) -> bool:
        return is_fresh(now_ns, stamp, timeout)

    def _publish_mode(self, mode: SideMode, reason: str = "") -> None:
        self._intent_token = transition_intent_token(self._intent_token, mode)
        self.mode_pub.publish(UInt64(data=self._intent_token))
        if mode != self._last_mode or (
            mode == SideMode.FAULT and reason != self._last_fault_reason
        ):
            if mode == SideMode.FAULT:
                self.get_logger().error(f"[{self.side}] entering FAULT: {reason}")
            else:
                self.get_logger().info(f"[{self.side}] mode {self._last_mode.name} -> {mode.name}")
        self._last_mode = mode
        self._last_fault_reason = reason if mode == SideMode.FAULT else ""

    def _fail(self, reason: str) -> None:
        self._fault_latch.trip()
        self._teleop_active = False
        self._home_active = False
        self._start_handle = None
        self._zero_tcp = None
        self._publish_mode(SideMode.FAULT, reason)

    def _trigger_value(self) -> float:
        value = self._buttons.get(self.trigger_axis, 0.0)
        try:
            return max(0.0, min(float(value), 1.0))
        except (TypeError, ValueError):
            return 0.0

    def _home_reached(self) -> bool:
        if self._joint_feedback is None:
            return False
        return max(
            abs(current - target)
            for current, target in zip(self._joint_feedback, self.home_joint_positions)
        ) <= self.home_tolerance_rad

    def _control_loop(self) -> None:
        now_ns = time.monotonic_ns()
        source_signal_fresh = self._fresh(
            self._source_valid_received_ns, self.input_timeout_sec, now_ns
        )
        buttons_fresh = self._fresh(self._buttons_received_ns, self.input_timeout_sec, now_ns)
        if not source_signal_fresh or not self._source_valid:
            self._fail("Quest source invalid or stale")
            return
        if not buttons_fresh:
            self._fail("button packet stale")
            return

        # Preserve the original Piper gripper semantics independently of the
        # arm Grip/deadman: released Trigger is fully open, pressed Trigger is
        # fully closed, with continuous proportional positions in between. The
        # final guard maps this into the reviewed 0..0.040 m runtime range while
        # retaining the official 0..0.044 m hard envelope, then applies velocity,
        # tracking, feedback and freshness checks before ID8 sees a command.
        self.gripper_pub.publish(Float32(data=1.0 - self._trigger_value()))

        previous_grip = self._grip_active
        self._grip_active = update_grip_deadman(
            raw_pressed=bool(self._buttons.get(self.hold_button, False)),
            analog_value=self._buttons.get(self.grip_axis, 0.0),
            previous=previous_grip,
            press_threshold=self.grip_press_threshold,
            release_threshold=self.grip_release_threshold,
        )
        hold_pressed = self._grip_active
        if hold_pressed != previous_grip:
            self.get_logger().info(
                f"[{self.side}] Grip {'engaged' if hold_pressed else 'released'}: "
                f"raw={bool(self._buttons.get(self.hold_button, False))}, "
                f"analog={float(self._buttons.get(self.grip_axis, 0.0)):.3f}"
            )
        home_pressed = bool(self._buttons.get(self.home_button, False))
        home_rising = home_pressed and not self._home_button_previous
        self._home_button_previous = home_pressed

        if self._fault_latch.latched:
            all_inputs_ready = all(
                (
                    self._fresh(self._handle_received_ns, self.input_timeout_sec, now_ns),
                    self._fresh(self._tcp_received_ns, self.feedback_timeout_sec, now_ns),
                    self._fresh(self._joint_received_ns, self.feedback_timeout_sec, now_ns),
                    self._handle_matrix is not None,
                    self._tcp_matrix is not None,
                    self._joint_feedback is not None,
                    self._handle_source_stamp_ns is not None,
                )
            )
            still_latched = self._fault_latch.update(
                now_ns,
                controls_released=(
                    not hold_pressed
                    and not home_pressed
                    and self._trigger_value() <= self.trigger_release_threshold
                ),
                inputs_ready=all_inputs_ready,
            )
            if still_latched:
                self._publish_mode(
                    SideMode.FAULT,
                    "awaiting healthy inputs and released controls to re-arm",
                )
            else:
                self._publish_mode(SideMode.HOLD)
            return

        if hold_pressed:
            self._home_active = False
            if not self._fresh(self._handle_received_ns, self.input_timeout_sec, now_ns):
                self._fail("handle pose stale")
                return
            if not self._fresh(self._tcp_received_ns, self.feedback_timeout_sec, now_ns):
                self._fail("TCP feedback stale")
                return
            if self._handle_matrix is None or self._tcp_matrix is None:
                self._fail("handle or TCP pose unavailable")
                return
            if self._handle_source_stamp_ns is None:
                self._fail("handle pose has no source epoch")
                return

            if not self._teleop_active:
                self._start_handle = self._handle_matrix.copy()
                self._zero_tcp = self._tcp_matrix.copy()
                self._teleop_active = True

            assert self._start_handle is not None and self._zero_tcp is not None
            try:
                target_matrix = compose_relative_target(
                    self._zero_tcp,
                    self._start_handle,
                    self._handle_matrix,
                )
                target = matrix_to_pose(
                    target_matrix,
                    self.target_frame_id,
                    # Preserve the Quest packet stamp. The bimanual IK node
                    # permits only a small configured left/right source skew.
                    RosTime(
                        sec=self._handle_source_stamp_ns // 1_000_000_000,
                        nanosec=self._handle_source_stamp_ns % 1_000_000_000,
                    ),
                )
            except (ValueError, np.linalg.LinAlgError) as exc:
                self._fail(f"relative pose calculation failed: {exc}")
                return

            self._publish_mode(SideMode.TELEOP)
            atomic_target = build_atomic_target(
                (
                    target.pose.position.x,
                    target.pose.position.y,
                    target.pose.position.z,
                    target.pose.orientation.x,
                    target.pose.orientation.y,
                    target.pose.orientation.z,
                    target.pose.orientation.w,
                ),
                target.header.stamp.sec,
                target.header.stamp.nanosec,
                self._intent_token,
            )
            # PoseStamped remains a human/RViz debug stream. Only the atomic
            # Float64 intent is consumed by the IK safety path.
            self.target_pub.publish(target)
            self.target_intent_pub.publish(
                Float64MultiArray(data=list(atomic_target))
            )
            return

        if self._teleop_active:
            self._teleop_active = False
            self._start_handle = None
            self._zero_tcp = None

        if home_rising:
            if not self.enable_home:
                self.get_logger().warn(f"[{self.side}] Home ignored because enable_home=false")
            else:
                self._home_active = True

        if self._home_active:
            if not self._fresh(self._joint_received_ns, self.feedback_timeout_sec, now_ns):
                self._fail("joint feedback stale during HOME")
                return
            if self._home_reached():
                self._home_active = False
                self._publish_mode(SideMode.HOLD)
            else:
                self._publish_mode(SideMode.HOME)
            return

        self._publish_mode(SideMode.HOLD)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OpenArmDeltaPosePublisher()
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
