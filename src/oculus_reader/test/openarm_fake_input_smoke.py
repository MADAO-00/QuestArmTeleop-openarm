#!/usr/bin/env python3
"""Inject synthetic Quest input into the fake-only OpenArm ROS graph.

This helper never starts hardware.  It is intended to run alongside
``teleop_double_openarm_v1.launch.py`` with fake hardware.  It first proves
that both grippers open while both arm Grip controls remain released, then
measures a full-Trigger close and release-open trajectory before exercising a
TELEOP -> HOLD -> TELEOP re-engagement and side-specific Trigger closure. It
also holds deliberately incoherent bimanual target epochs after
both arms have established TELEOP, proving that synchronization wait produces
no new arm controller command and recovers without a fault. Success requires
named GenericSystem feedback after every command. Before the original
bimanual stages it establishes the right arm alone, adds the left peer late,
and proves that a post-edge historical epoch pair can recover while the two
latest target samples remain incoherent.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Float64MultiArray, UInt8, UInt64


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from openarm_teleop_core import (
    GRIPPER_CLOSE_VELOCITY_M_S,
    GRIPPER_OPEN_POSITION_M,
    GRIPPER_OPEN_VELOCITY_M_S,
    decode_intent_token,
    parse_atomic_target,
    parse_guard_acceptance,
)


HOLD = 0
TELEOP = 1
FAULT = 3
SIDES = ("right", "left")
JOINT_NAMES = {
    side: tuple(f"openarm_{side}_joint{joint}" for joint in range(1, 8))
    for side in SIDES
}
GRIPPER_JOINT_NAMES = {
    side: f"openarm_{side}_finger_joint1" for side in SIDES
}
JOINT_FOLLOW_TOLERANCE_RAD = 1e-4
GRIPPER_FOLLOW_TOLERANCE_M = 1e-5
GRIPPER_COMMAND_STEP_TOLERANCE_M = 1e-7
GRIPPER_CONTROL_RATE_HZ = 100.0
GRIPPER_COMMAND_QOS_DEPTH = 256
GRIPPER_OPEN_STEP_M = GRIPPER_OPEN_VELOCITY_M_S / GRIPPER_CONTROL_RATE_HZ
GRIPPER_CLOSE_STEP_M = GRIPPER_CLOSE_VELOCITY_M_S / GRIPPER_CONTROL_RATE_HZ
# These are only broad test-hang bounds. Trigger controls a proportional
# position target; reaching either endpoint inside a particular time window is
# deliberately not part of the teleoperation contract.
FULL_CLOSE_TEST_TIMEOUT_NS = 5_000_000_000
FULL_REOPEN_TEST_TIMEOUT_NS = 5_000_000_000
GRIPPER_TARGET_M = {
    "right": GRIPPER_OPEN_POSITION_M * (1.0 - 0.25),
    "left": GRIPPER_OPEN_POSITION_M * (1.0 - 0.75),
}
COHERENT_LEFT_EPOCH_OFFSET_NS = 10_000_000
# Far beyond the configured 30 ms bimanual target skew, but held for only
# about 0.40 s -- comfortably inside the reviewed 0.75 s sync-wait lease.
INCOHERENT_LEFT_EPOCH_OFFSET_NS = 400_000_000
SYNC_WAIT_SETTLE_NS = 50_000_000
SYNC_WAIT_NO_COMMAND_NS = 350_000_000
SYNC_PREPARE_DISPLACEMENT_M = 0.001
# The preceding zero-target TELEOP has already been stable for 300 ms. A
# change above this threshold is far beyond its floating-point noise, while a
# unique 1 mm prepare target produces a comfortably larger joint-space step.
PREPARE_PAYLOAD_CHANGE_TOLERANCE = 1e-5
PREPARE_ACCEPTED_COMMAND_TOLERANCE_RAD = 1e-9

# Late-peer regression: establish one arm first, then make the peer's TELEOP
# token edge invalidate the already-established side. Stage A has no target
# epoch pair inside the configured 30 ms window. A right-only bridge update
# removes every cross-topic switch window before Stage B adds one valid pair
# to post-edge history while keeping latest/latest deliberately incoherent.
LATE_PEER_FIRST_SIDE_HOLD_NS = 150_000_000
LATE_PEER_STAGE_A_LEFT_OFFSET_NS = 400_000_000
LATE_PEER_STAGE_B_RIGHT_OFFSET_NS = 800_000_000
LATE_PEER_STAGE_B_LEFT_OFFSET_NS = 10_000_000
LATE_PEER_COMMAND_DRAIN_NS = 50_000_000
LATE_PEER_NO_COMMAND_NS = 80_000_000
LATE_PEER_BRIDGE_DRAIN_NS = 100_000_000
LATE_PEER_HISTORY_TRAP_HOLD_NS = 60_000_000
LATE_PEER_ESTABLISH_DEADLINE_NS = 750_000_000
LATE_PEER_MAX_COHERENT_SKEW_NS = 30_000_000
TARGET_HISTORY_LIMIT = 256


def offset_stamp(stamp: Time, offset_ns: int) -> Time:
    total_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec) + int(offset_ns)
    return Time(sec=total_ns // 1_000_000_000, nanosec=total_ns % 1_000_000_000)


def stamp_from_ns(total_ns: int) -> Time:
    if int(total_ns) <= 0:
        raise ValueError("synthetic target epoch must be positive")
    return Time(
        sec=int(total_ns) // 1_000_000_000,
        nanosec=int(total_ns) % 1_000_000_000,
    )


class FakeQuestSmoke(Node):
    def __init__(self) -> None:
        super().__init__("openarm_fake_input_smoke")
        self.source_pub = self.create_publisher(Bool, "/oculus/source_valid", 10)
        self.button_pub = self.create_publisher(Joy, "/oculus/buttons", 10)
        self.pose_pub = {
            "right": self.create_publisher(PoseStamped, "/right_handle_pose", 10),
            "left": self.create_publisher(PoseStamped, "/left_handle_pose", 10),
        }
        self.guard_mode = {"right": None, "left": None}
        self.mode_token = {"right": None, "left": None}
        self.mode_token_mode = {"right": None, "left": None}
        self.mode_token_history = {"right": [], "left": []}
        self.latest_target_intent = {"right": None, "left": None}
        self.target_intent_history = {"right": [], "left": []}
        self.guard_acceptance_count = {"right": 0, "left": 0}
        self.latest_guard_acceptance = {"right": None, "left": None}
        self.guard_acceptance_history = {"right": [], "left": []}
        self.command = {"right": None, "left": None}
        self.latest_command = {"right": None, "left": None}
        self.latest_command_ns = {"right": None, "left": None}
        self.command_history = {"right": [], "left": []}
        self.command_count = {"right": 0, "left": 0}
        self.feedback = {"right": None, "left": None}
        self.command_ns = {"right": None, "left": None}
        self.feedback_ns = {"right": None, "left": None}
        self.gripper_command = {"right": None, "left": None}
        self.gripper_feedback = {"right": None, "left": None}
        self.gripper_command_ns = {"right": None, "left": None}
        self.gripper_feedback_ns = {"right": None, "left": None}
        self.gripper_command_history = {"right": [], "left": []}
        for side in SIDES:
            self.create_subscription(
                UInt8,
                f"/openarm/guard_mode/{side}",
                lambda msg, selected=side: self._mode_callback(selected, msg),
                10,
            )
            self.create_subscription(
                UInt64,
                f"/openarm/mode/{side}",
                lambda msg, selected=side: self._mode_token_callback(
                    selected, msg
                ),
                10,
            )
            self.create_subscription(
                Float64MultiArray,
                f"/openarm/target_intent/{side}",
                lambda msg, selected=side: self._target_intent_callback(
                    selected, msg
                ),
                10,
            )
            self.create_subscription(
                Float64MultiArray,
                f"/openarm/guard_acceptance/{side}",
                lambda msg, selected=side: self._guard_acceptance_callback(
                    selected, msg
                ),
                10,
            )
            self.create_subscription(
                Float64MultiArray,
                f"/{side}_forward_position_controller/commands",
                lambda msg, selected=side: self._command_callback(selected, msg),
                10,
            )
            self.create_subscription(
                Float64MultiArray,
                f"/{side}_gripper_forward_position_controller/commands",
                lambda msg, selected=side: self._gripper_command_callback(
                    selected, msg
                ),
                GRIPPER_COMMAND_QOS_DEPTH,
            )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

        self.started_ns = time.monotonic_ns()
        self.phase = "initial_trigger_held"
        self.phase_condition_started_ns: int | None = None
        self.moving = False
        self.open_gripper_command = {"right": None, "left": None}
        self.full_close_requested_ns: int | None = None
        self.full_close_endpoint_ns = {"right": None, "left": None}
        self.full_close_max_step_m = {"right": 0.0, "left": 0.0}
        self.full_close_verified = False
        self.full_reopen_requested_ns: int | None = None
        self.full_reopen_endpoint_ns = {"right": None, "left": None}
        self.full_reopen_max_step_m = {"right": 0.0, "left": 0.0}
        self.full_reopen_verified = False
        self.late_peer_right_phase_started_ns: int | None = None
        self.late_peer_right_established_ns: int | None = None
        self.late_peer_prejoin_tokens: tuple[int, int] | None = None
        self.late_peer_epoch_base_ns: int | None = None
        self.late_peer_join_requested_ns: int | None = None
        self.late_peer_pair_seen_ns: int | None = None
        self.late_peer_intent_tokens: tuple[int, int] | None = None
        self.late_peer_stage_a_last_command_counts: dict[str, int] | None = None
        self.late_peer_stage_a_counts_stable_ns: int | None = None
        self.late_peer_stage_a_command_baseline: dict[str, int] | None = None
        self.late_peer_stage_b_bridge_started_ns: int | None = None
        self.late_peer_stage_b_bridge_stable_ns: int | None = None
        self.late_peer_stage_b_started_ns: int | None = None
        self.late_peer_stage_b_trap_started_ns: int | None = None
        self.late_peer_stage_b_ack_fence_ns: int | None = None
        self.late_peer_stage_b_acceptance_baseline: dict[str, int] | None = None
        self.late_peer_stage_b_post_fence_ack = {"right": None, "left": None}
        self.late_peer_history_pair_verified = False
        self.late_peer_no_command_verified = False
        self.late_peer_same_pair_established = False
        self.late_peer_establish_elapsed_ns: int | None = None
        self.sync_prepare_acceptance_baseline: dict[str, int] | None = None
        self.sync_prepare_command_snapshot = {"right": None, "left": None}
        self.sync_prepare_acceptance_snapshot = {"right": None, "left": None}
        self.sync_prepare_started_ns: int | None = None
        self.sync_prepare_matched_command = {"right": None, "left": None}
        self.sync_prepare_matched_acceptance = {"right": None, "left": None}
        self.sync_prepare_feedback_matched = {"right": False, "left": False}
        self.sync_wait_command_baseline: dict[str, int] | None = None
        self.sync_wait_hold_observed = False
        self.sync_wait_no_command_verified = False
        self.sync_wait_recovered = False
        self.success = False
        self.failure = ""
        # Match the production Quest publisher's configured 100 Hz source rate.
        self.timer = self.create_timer(0.01, self._publish_once)

    def _mode_callback(self, side: str, msg: UInt8) -> None:
        self.guard_mode[side] = int(msg.data)

    def _mode_token_callback(self, side: str, msg: UInt64) -> None:
        raw_token = int(msg.data)
        try:
            _, mode = decode_intent_token(raw_token)
        except (TypeError, ValueError, OverflowError) as exc:
            self.failure = (
                f"{side} delta emitted an invalid mode token {raw_token}: {exc}"
            )
            return
        received_ns = time.monotonic_ns()
        self.mode_token[side] = raw_token
        self.mode_token_mode[side] = int(mode)
        self.mode_token_history[side].append(
            (received_ns, raw_token, int(mode))
        )
        self.mode_token_history[side] = self.mode_token_history[side][
            -TARGET_HISTORY_LIMIT:
        ]
        if self.phase == "late_peer_stage_a_wait":
            self._capture_late_peer_pair(received_ns)

    def _target_intent_callback(
        self, side: str, msg: Float64MultiArray
    ) -> None:
        try:
            _, epoch_ns, raw_token = parse_atomic_target(msg.data)
        except (TypeError, ValueError, OverflowError) as exc:
            self.failure = (
                f"{side} delta emitted an invalid 10-value atomic target: "
                f"{tuple(msg.data)} ({exc})"
            )
            return
        received_ns = time.monotonic_ns()
        record = (received_ns, int(epoch_ns), int(raw_token))
        self.latest_target_intent[side] = record
        self.target_intent_history[side].append(record)
        self.target_intent_history[side] = self.target_intent_history[side][
            -TARGET_HISTORY_LIMIT:
        ]

    def _guard_acceptance_callback(
        self, side: str, msg: Float64MultiArray
    ) -> None:
        try:
            (
                accepted_command,
                echoed_candidate,
                echoed_solver_base,
                intent_tokens,
            ) = parse_guard_acceptance(msg.data)
            # Keep the complete canonical wire payload for the prepare snapshot
            # and payload-change fence, including both exact intent tokens.
            values = (
                accepted_command
                + echoed_candidate
                + echoed_solver_base
                + tuple(float(token) for token in intent_tokens)
            )
        except (TypeError, ValueError, OverflowError) as exc:
            self.failure = (
                f"{side} guard emitted an invalid 23-value dual-token "
                f"acceptance: {tuple(msg.data)} ({exc})"
            )
            return
        received_ns = time.monotonic_ns()
        self.guard_acceptance_count[side] += 1
        self.latest_guard_acceptance[side] = values
        self.guard_acceptance_history[side].append(
            (received_ns, values, accepted_command, intent_tokens)
        )
        self.guard_acceptance_history[side] = self.guard_acceptance_history[side][
            -100:
        ]
        if (
            self.phase == "late_peer_stage_b_history"
            and self.late_peer_stage_b_ack_fence_ns is not None
            and self.late_peer_history_pair_verified
            and self.late_peer_stage_b_trap_started_ns is not None
            and received_ns - self.late_peer_stage_b_trap_started_ns
            >= LATE_PEER_HISTORY_TRAP_HOLD_NS
            and received_ns > self.late_peer_stage_b_ack_fence_ns
            and intent_tokens == self.late_peer_intent_tokens
            and self._late_peer_final_latest_is_incoherent()
        ):
            # Record only ACKs received after both final B latest samples have
            # remained at 790 ms skew through the transition-drain fence. A
            # cached retry ACK is valid here; demanding another controller
            # publication would make the test depend on solver convergence.
            self.late_peer_stage_b_post_fence_ack[side] = (
                received_ns,
                accepted_command,
            )
        self._update_sync_prepare_fence(side)

    def _command_callback(self, side: str, msg: Float64MultiArray) -> None:
        values = tuple(float(value) for value in msg.data)
        if len(values) != 7 or not all(math.isfinite(value) for value in values):
            self.failure = f"{side} controller emitted an invalid command: {values}"
            return
        received_ns = time.monotonic_ns()
        self.latest_command[side] = values
        self.latest_command_ns[side] = received_ns
        self.command_history[side].append((received_ns, values))
        self.command_history[side] = self.command_history[side][-100:]
        self.command_count[side] += 1
        if (
            self.phase == "sync_wait_hold"
            and self.sync_wait_command_baseline is not None
            and self.command_count[side] != self.sync_wait_command_baseline[side]
        ):
            self.failure = (
                f"{side} controller emitted a new arm command during "
                "bimanual target synchronization HOLD: "
                f"baseline={self.sync_wait_command_baseline}, "
                f"observed={self.command_count}"
            )
            return
        if (
            self.phase == "late_peer_stage_a_verify"
            and self.late_peer_stage_a_command_baseline is not None
            and self.command_count[side]
            != self.late_peer_stage_a_command_baseline[side]
        ):
            self.failure = (
                f"{side} controller emitted a new arm command after the "
                "late-peer token edge while both guards were in HOLD: "
                f"baseline={self.late_peer_stage_a_command_baseline}, "
                f"observed={self.command_count}"
            )
            return
        self._update_sync_prepare_fence(side)
        if self.moving:
            self.command[side] = values
            self.command_ns[side] = received_ns
            self._update_success()

    @staticmethod
    def _payload_changed(
        current: tuple[float, ...],
        snapshot: tuple[float, ...],
        tolerance: float = PREPARE_PAYLOAD_CHANGE_TOLERANCE,
    ) -> bool:
        return max(
            abs(value - previous)
            for value, previous in zip(current, snapshot, strict=True)
        ) > tolerance

    def _current_intent_pair(self) -> tuple[int, int] | None:
        right_token = self.mode_token["right"]
        left_token = self.mode_token["left"]
        if right_token is None or left_token is None:
            return None
        return int(right_token), int(left_token)

    def _capture_late_peer_pair(self, observed_ns: int) -> None:
        if self.late_peer_intent_tokens is not None:
            return
        intent_pair = self._current_intent_pair()
        if not (
            intent_pair is not None
            and self.mode_token_mode["right"] == TELEOP
            and self.mode_token_mode["left"] == TELEOP
        ):
            return
        prejoin_pair = self.late_peer_prejoin_tokens
        if (
            prejoin_pair is None
            or intent_pair[0] != prejoin_pair[0]
            or intent_pair[1] == prejoin_pair[1]
        ):
            self.failure = (
                "late-peer mode edge did not preserve the right TELEOP token "
                "and advance only the joining left token: "
                f"before={prejoin_pair}, after={intent_pair}"
            )
            return
        self.late_peer_intent_tokens = intent_pair
        self.late_peer_pair_seen_ns = int(observed_ns)

    def _latest_target_is(
        self,
        side: str,
        *,
        epoch_ns: int,
        raw_token: int,
        after_ns: int,
    ) -> bool:
        record = self.latest_target_intent[side]
        return bool(
            record is not None
            and record[0] >= after_ns
            and record[1] == int(epoch_ns)
            and record[2] == int(raw_token)
        )

    def _target_was_seen(
        self,
        side: str,
        *,
        epoch_ns: int,
        raw_token: int,
        after_ns: int,
    ) -> bool:
        return any(
            received_ns >= after_ns
            and observed_epoch_ns == int(epoch_ns)
            and observed_token == int(raw_token)
            for received_ns, observed_epoch_ns, observed_token in self.target_intent_history[
                side
            ]
        )

    def _acceptance_for_pair_seen_since(
        self, intent_tokens: tuple[int, int], started_ns: int
    ) -> bool:
        return any(
            received_ns >= started_ns and observed_tokens == intent_tokens
            for side in SIDES
            for received_ns, _, _, observed_tokens in self.guard_acceptance_history[
                side
            ]
        )

    def _pair_acceptance_count(
        self, side: str, intent_tokens: tuple[int, int]
    ) -> int:
        return sum(
            observed_tokens == intent_tokens
            for _, _, _, observed_tokens in self.guard_acceptance_history[side]
        )

    def _late_peer_final_latest_is_incoherent(self) -> bool:
        pair = self.late_peer_intent_tokens
        epoch_base_ns = self.late_peer_epoch_base_ns
        stage_b_started_ns = self.late_peer_stage_b_started_ns
        if pair is None or epoch_base_ns is None or stage_b_started_ns is None:
            return False
        right_epoch_ns = epoch_base_ns + LATE_PEER_STAGE_B_RIGHT_OFFSET_NS
        left_epoch_ns = epoch_base_ns + LATE_PEER_STAGE_B_LEFT_OFFSET_NS
        return bool(
            abs(right_epoch_ns - left_epoch_ns)
            > LATE_PEER_MAX_COHERENT_SKEW_NS
            and self._latest_target_is(
                "right",
                epoch_ns=right_epoch_ns,
                raw_token=pair[0],
                after_ns=stage_b_started_ns,
            )
            and self._latest_target_is(
                "left",
                epoch_ns=left_epoch_ns,
                raw_token=pair[1],
                after_ns=stage_b_started_ns,
            )
        )

    def _late_peer_bridge_latest_is_incoherent(self) -> bool:
        pair = self.late_peer_intent_tokens
        epoch_base_ns = self.late_peer_epoch_base_ns
        bridge_started_ns = self.late_peer_stage_b_bridge_started_ns
        if pair is None or epoch_base_ns is None or bridge_started_ns is None:
            return False
        right_epoch_ns = epoch_base_ns + LATE_PEER_STAGE_B_RIGHT_OFFSET_NS
        left_epoch_ns = epoch_base_ns + LATE_PEER_STAGE_A_LEFT_OFFSET_NS
        return bool(
            abs(right_epoch_ns - left_epoch_ns)
            > LATE_PEER_MAX_COHERENT_SKEW_NS
            and self._latest_target_is(
                "right",
                epoch_ns=right_epoch_ns,
                raw_token=pair[0],
                after_ns=bridge_started_ns,
            )
            and self._latest_target_is(
                "left",
                epoch_ns=left_epoch_ns,
                raw_token=pair[1],
                after_ns=bridge_started_ns,
            )
        )

    def _post_edge_epoch_pair_exists(
        self,
        *,
        right_epoch_ns: int,
        left_epoch_ns: int,
        intent_tokens: tuple[int, int],
        edge_seen_ns: int,
    ) -> bool:
        if abs(int(right_epoch_ns) - int(left_epoch_ns)) > (
            LATE_PEER_MAX_COHERENT_SKEW_NS
        ):
            return False
        return self._target_was_seen(
            "right",
            epoch_ns=right_epoch_ns,
            raw_token=intent_tokens[0],
            after_ns=edge_seen_ns,
        ) and self._target_was_seen(
            "left",
            epoch_ns=left_epoch_ns,
            raw_token=intent_tokens[1],
            after_ns=edge_seen_ns,
        )

    def _any_post_edge_coherent_pair(
        self, intent_tokens: tuple[int, int], edge_seen_ns: int
    ) -> bool:
        right_epochs = (
            epoch_ns
            for received_ns, epoch_ns, raw_token in self.target_intent_history[
                "right"
            ]
            if received_ns >= edge_seen_ns and raw_token == intent_tokens[0]
        )
        left_epochs = tuple(
            epoch_ns
            for received_ns, epoch_ns, raw_token in self.target_intent_history[
                "left"
            ]
            if received_ns >= edge_seen_ns and raw_token == intent_tokens[1]
        )
        return any(
            abs(right_epoch_ns - left_epoch_ns)
            <= LATE_PEER_MAX_COHERENT_SKEW_NS
            for right_epoch_ns in right_epochs
            for left_epoch_ns in left_epochs
        )

    def _side_has_closed_round(
        self,
        side: str,
        *,
        started_ns: int,
        intent_tokens: tuple[int, int],
        acceptance_started_ns: int | None = None,
    ) -> bool:
        feedback = self.feedback[side]
        feedback_ns = self.feedback_ns[side]
        if feedback is None or feedback_ns is None:
            return False
        for command_ns, command in reversed(self.command_history[side]):
            if command_ns < started_ns:
                break
            if feedback_ns < command_ns:
                continue
            if (
                max(
                    abs(actual - requested)
                    for actual, requested in zip(feedback, command, strict=True)
                )
                > JOINT_FOLLOW_TOLERANCE_RAD
            ):
                continue
            for (
                acceptance_ns,
                _,
                accepted_command,
                observed_tokens,
            ) in reversed(self.guard_acceptance_history[side]):
                minimum_acceptance_ns = (
                    started_ns
                    if acceptance_started_ns is None
                    else acceptance_started_ns
                )
                if acceptance_ns < minimum_acceptance_ns:
                    break
                if observed_tokens != intent_tokens:
                    continue
                if all(
                    math.isclose(
                        accepted,
                        published,
                        rel_tol=0.0,
                        abs_tol=PREPARE_ACCEPTED_COMMAND_TOLERANCE_RAD,
                    )
                    for accepted, published in zip(
                        accepted_command, command, strict=True
                    )
                ):
                    return True
        return False

    def _update_sync_prepare_fence(self, side: str) -> None:
        if self.phase != "sync_wait_prepare":
            return
        started_ns = self.sync_prepare_started_ns
        command_snapshot = self.sync_prepare_command_snapshot[side]
        acceptance_snapshot = self.sync_prepare_acceptance_snapshot[side]
        if (
            started_ns is None
            or command_snapshot is None
            or acceptance_snapshot is None
        ):
            return
        eligible_ns = started_ns
        feedback = self.feedback[side]
        feedback_ns = self.feedback_ns[side]
        for command_ns, command in reversed(self.command_history[side]):
            if command_ns < eligible_ns:
                break
            if not self._payload_changed(command, command_snapshot):
                continue
            for acceptance_ns, acceptance, accepted_command, _ in reversed(
                self.guard_acceptance_history[side]
            ):
                if acceptance_ns < eligible_ns:
                    break
                if not self._payload_changed(acceptance, acceptance_snapshot):
                    continue
                if not all(
                    math.isclose(
                        accepted,
                        published,
                        rel_tol=0.0,
                        abs_tol=PREPARE_ACCEPTED_COMMAND_TOLERANCE_RAD,
                    )
                    for accepted, published in zip(
                        accepted_command, command, strict=True
                    )
                ):
                    continue
                if (
                    feedback is None
                    or feedback_ns is None
                    or feedback_ns < command_ns
                    or max(
                        abs(actual - requested)
                        for actual, requested in zip(
                            feedback, command, strict=True
                        )
                    )
                    > JOINT_FOLLOW_TOLERANCE_RAD
                ):
                    continue
                self.sync_prepare_matched_command[side] = command
                self.sync_prepare_matched_acceptance[side] = acceptance
                self.sync_prepare_feedback_matched[side] = True
                return

    def _gripper_command_callback(
        self, side: str, msg: Float64MultiArray
    ) -> None:
        values = tuple(float(value) for value in msg.data)
        if len(values) != 1 or not all(math.isfinite(value) for value in values):
            self.failure = f"{side} gripper emitted an invalid command: {values}"
            return
        received_ns = time.monotonic_ns()
        value = values[0]
        previous = self.gripper_command[side]
        self.gripper_command_history[side].append((received_ns, value))
        self.gripper_command_history[side] = self.gripper_command_history[side][
            -1000:
        ]
        if self.phase == "full_trigger_close":
            self._validate_full_close_command(
                side, previous, value, received_ns
            )
        elif self.phase == "full_trigger_reopen":
            self._validate_full_reopen_command(
                side, previous, value, received_ns
            )
        self.gripper_command[side] = value
        self.gripper_command_ns[side] = received_ns
        self._update_success()

    def _validate_full_close_command(
        self,
        side: str,
        previous: float | None,
        command: float,
        received_ns: int,
    ) -> None:
        requested_ns = self.full_close_requested_ns
        if requested_ns is None:
            self.failure = "full-close command arrived without its Trigger edge fence"
            return
        if previous is not None:
            if command > previous + GRIPPER_COMMAND_STEP_TOLERANCE_M:
                self.failure = (
                    f"{side} gripper reversed toward open during full closure: "
                    f"previous={previous:.9f}, command={command:.9f}"
                )
                return
            close_step = max(0.0, previous - command)
            if close_step > (
                GRIPPER_CLOSE_STEP_M + GRIPPER_COMMAND_STEP_TOLERANCE_M
            ):
                self.failure = (
                    f"{side} gripper close step exceeded "
                    f"{GRIPPER_CLOSE_VELOCITY_M_S:.4f} m/s at 100 Hz: "
                    f"step={close_step:.9f}"
                )
                return
            self.full_close_max_step_m[side] = max(
                self.full_close_max_step_m[side], close_step
            )
        if (
            command <= GRIPPER_FOLLOW_TOLERANCE_M
            and self.full_close_endpoint_ns[side] is None
        ):
            self.full_close_endpoint_ns[side] = received_ns

    def _validate_full_reopen_command(
        self,
        side: str,
        previous: float | None,
        command: float,
        received_ns: int,
    ) -> None:
        requested_ns = self.full_reopen_requested_ns
        if requested_ns is None:
            self.failure = "full-reopen command arrived without its Trigger edge fence"
            return
        if previous is not None:
            if command < previous - GRIPPER_COMMAND_STEP_TOLERANCE_M:
                self.failure = (
                    f"{side} gripper reversed toward closed during full reopen: "
                    f"previous={previous:.9f}, command={command:.9f}"
                )
                return
            open_step = max(0.0, command - previous)
            if open_step > (
                GRIPPER_OPEN_STEP_M + GRIPPER_COMMAND_STEP_TOLERANCE_M
            ):
                self.failure = (
                    f"{side} gripper open step exceeded "
                    f"{GRIPPER_OPEN_VELOCITY_M_S:.4f} m/s at 100 Hz: "
                    f"step={open_step:.9f}"
                )
                return
            self.full_reopen_max_step_m[side] = max(
                self.full_reopen_max_step_m[side], open_step
            )
        if (
            command >= GRIPPER_OPEN_POSITION_M - GRIPPER_FOLLOW_TOLERANCE_M
            and self.full_reopen_endpoint_ns[side] is None
        ):
            self.full_reopen_endpoint_ns[side] = received_ns

    def _joint_state_callback(self, msg: JointState) -> None:
        if len(msg.name) != len(set(msg.name)):
            self.failure = "joint_states contains duplicate joint names"
            return
        if len(msg.position) < len(msg.name):
            self.failure = "joint_states position array is shorter than its name array"
            return
        by_name = {
            name: float(msg.position[index]) for index, name in enumerate(msg.name)
        }
        received_ns = time.monotonic_ns()
        for side in SIDES:
            names = JOINT_NAMES[side]
            if all(name in by_name for name in names):
                values = tuple(by_name[name] for name in names)
                if not all(math.isfinite(value) for value in values):
                    self.failure = f"{side} joint feedback contains non-finite values"
                    return
                self.feedback[side] = values
                self.feedback_ns[side] = received_ns
                self._update_sync_prepare_fence(side)
            gripper_name = GRIPPER_JOINT_NAMES[side]
            if gripper_name in by_name:
                value = by_name[gripper_name]
                if not math.isfinite(value):
                    self.failure = f"{side} gripper feedback is non-finite"
                    return
                self.gripper_feedback[side] = value
                self.gripper_feedback_ns[side] = received_ns
        self._update_success()

    def _update_success(self) -> None:
        if not self.moving:
            return
        if not (
            self.full_close_verified
            and self.full_reopen_verified
            and self.late_peer_history_pair_verified
            and self.late_peer_no_command_verified
            and self.late_peer_same_pair_established
            and self.late_peer_establish_elapsed_ns is not None
            and self.late_peer_establish_elapsed_ns
            < LATE_PEER_ESTABLISH_DEADLINE_NS
            and self.sync_wait_hold_observed
            and self.sync_wait_no_command_verified
            and self.sync_wait_recovered
        ):
            return
        if not all(
            self.command[side] is not None and self.feedback[side] is not None
            for side in SIDES
        ):
            return
        if not all(
            self.gripper_command[side] is not None
            and self.gripper_feedback[side] is not None
            for side in SIDES
        ):
            return
        if not all(
            max(abs(value) for value in self.command[side]) > 1e-7
            for side in SIDES
        ):
            return
        if not all(self.open_gripper_command[side] is not None for side in SIDES):
            return
        # This endpoint check proves the full proportional mapping, rather than
        # accepting a transient while both rate-limited grippers are still on
        # their way to the requested positions.
        if not all(
            abs(self.gripper_command[side] - GRIPPER_TARGET_M[side])
            <= GRIPPER_FOLLOW_TOLERANCE_M
            for side in SIDES
        ):
            return
        # A feedback sample must be received after each command callback. This
        # prevents a coincidentally equal pre-command state from passing.
        if not all(
            self.command_ns[side] is not None
            and self.feedback_ns[side] is not None
            and self.feedback_ns[side] >= self.command_ns[side]
            and self.gripper_command_ns[side] is not None
            and self.gripper_feedback_ns[side] is not None
            and self.gripper_feedback_ns[side] >= self.gripper_command_ns[side]
            for side in SIDES
        ):
            return
        # Seeing the guard's command topic alone would only prove publication.
        # Matching named joint feedback proves the ForwardCommandController
        # consumed the ordered seven-axis command and GenericSystem applied it.
        arms_follow = all(
            max(
                abs(actual - requested)
                for actual, requested in zip(
                    self.feedback[side], self.command[side], strict=True
                )
            )
            <= JOINT_FOLLOW_TOLERANCE_RAD
            for side in SIDES
        )
        grippers_follow = all(
            abs(self.gripper_feedback[side] - self.gripper_command[side])
            <= GRIPPER_FOLLOW_TOLERANCE_M
            for side in SIDES
        )
        self.success = arms_follow and grippers_follow

    def _both_guards_are(self, mode: int) -> bool:
        return all(self.guard_mode[side] == mode for side in SIDES)

    def _condition_held(self, condition: bool, now_ns: int, duration_ns: int) -> bool:
        if not condition:
            self.phase_condition_started_ns = None
            return False
        if self.phase_condition_started_ns is None:
            self.phase_condition_started_ns = now_ns
            return False
        return now_ns - self.phase_condition_started_ns >= duration_ns

    def _enter_phase(self, phase: str) -> None:
        self.phase = phase
        self.phase_condition_started_ns = None

    @staticmethod
    def _pose(stamp: Time, x: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "arm_origin"
        msg.pose.position.x = float(x)
        msg.pose.orientation.w = 1.0
        return msg

    def _publish_once(self) -> None:
        now_ns = time.monotonic_ns()
        elapsed_sec = (now_ns - self.started_ns) / 1e9
        if elapsed_sec > 40.0:
            self.failure = (
                "timed out waiting for guarded commands; "
                f"phase={self.phase}, guard_modes={self.guard_mode}, "
                f"guard_acceptance_counts={self.guard_acceptance_count}, "
                f"sync_prepare_acceptance_baseline="
                f"{self.sync_prepare_acceptance_baseline}, "
                f"sync_prepare_feedback_matched="
                f"{self.sync_prepare_feedback_matched}, "
                f"arm_command_counts={self.command_count}, "
                f"arm_commands={self.command}, arm_feedback={self.feedback}, "
                f"gripper_commands={self.gripper_command}, "
                f"gripper_feedback={self.gripper_feedback}, "
                f"full_close_endpoint_ns={self.full_close_endpoint_ns}, "
                f"full_reopen_endpoint_ns={self.full_reopen_endpoint_ns}, "
                f"late_peer_pair={self.late_peer_intent_tokens}, "
                f"late_peer_no_command_verified="
                f"{self.late_peer_no_command_verified}, "
                f"late_peer_history_pair_verified="
                f"{self.late_peer_history_pair_verified}, "
                f"late_peer_stage_b_ack_fence_ns="
                f"{self.late_peer_stage_b_ack_fence_ns}, "
                f"late_peer_stage_b_post_fence_ack="
                f"{self.late_peer_stage_b_post_fence_ack}, "
                f"late_peer_establish_elapsed_ns="
                f"{self.late_peer_establish_elapsed_ns}"
            )
            return

        if (
            self.phase
            in (
                "late_peer_stage_a_wait",
                "late_peer_stage_a_verify",
                "late_peer_stage_b_bridge",
                "late_peer_stage_b_history",
            )
            and self.late_peer_join_requested_ns is not None
            and now_ns - self.late_peer_join_requested_ns
            >= LATE_PEER_ESTABLISH_DEADLINE_NS
        ):
            self.failure = (
                "late peer did not establish one dual-token-current command "
                "round on both sides inside the strict 0.75 s deadline: "
                f"phase={self.phase}, guard_modes={self.guard_mode}, "
                f"mode_tokens={self.mode_token}, "
                f"latest_targets={self.latest_target_intent}, "
                f"command_counts={self.command_count}"
            )
            return

        if self.phase == "initial_trigger_held":
            # Startup must not re-arm while either Trigger is held. This also
            # proves that the first post-enable ID8 action cannot be closure.
            if self._condition_held(
                self._both_guards_are(FAULT), now_ns, 500_000_000
            ):
                self._enter_phase("initial_release")
        elif self.phase == "initial_release":
            grippers_open_without_grip = all(
                self.gripper_command[side] is not None
                and self.gripper_feedback[side] is not None
                and abs(
                    self.gripper_command[side] - GRIPPER_OPEN_POSITION_M
                ) <= GRIPPER_FOLLOW_TOLERANCE_M
                and self.gripper_feedback[side]
                >= GRIPPER_OPEN_POSITION_M - GRIPPER_FOLLOW_TOLERANCE_M
                for side in SIDES
            )
            if self._condition_held(
                self._both_guards_are(HOLD) and grippers_open_without_grip,
                now_ns,
                300_000_000,
            ):
                self.open_gripper_command = {
                    side: float(self.gripper_command[side]) for side in SIDES
                }
                self.full_close_requested_ns = now_ns
                self._enter_phase("full_trigger_close")
        elif self.phase == "full_trigger_close":
            requested_ns = self.full_close_requested_ns
            if requested_ns is None:
                self.failure = "full-close phase is missing its Trigger edge fence"
                return
            if (
                now_ns - requested_ns > FULL_CLOSE_TEST_TIMEOUT_NS
                and not all(
                    self.full_close_endpoint_ns[side] is not None
                    for side in SIDES
                )
            ):
                self.failure = (
                    "timed out waiting for the proportional full-close "
                    "endpoint during the fake test: "
                    f"endpoint_ns={self.full_close_endpoint_ns}, "
                    f"commands={self.gripper_command}, "
                    f"max_steps={self.full_close_max_step_m}"
                )
                return
            fully_closed = all(
                self.full_close_endpoint_ns[side] is not None
                and self.gripper_command[side] is not None
                and self.gripper_feedback[side] is not None
                and self.gripper_command[side] <= GRIPPER_FOLLOW_TOLERANCE_M
                and self.gripper_feedback[side] <= GRIPPER_FOLLOW_TOLERANCE_M
                for side in SIDES
            )
            full_close_rate_step_observed = all(
                self.full_close_max_step_m[side]
                >= GRIPPER_CLOSE_STEP_M - GRIPPER_COMMAND_STEP_TOLERANCE_M
                for side in SIDES
            )
            if fully_closed and not full_close_rate_step_observed:
                self.failure = (
                    "full-close endpoint was reached without observing a near-limit "
                    f"{GRIPPER_CLOSE_STEP_M:.6f} m command step on both sides: "
                    f"max_steps={self.full_close_max_step_m}"
                )
                return
            if self._condition_held(
                self._both_guards_are(HOLD)
                and fully_closed
                and full_close_rate_step_observed,
                now_ns,
                200_000_000,
            ):
                self.full_close_verified = True
                self.full_reopen_requested_ns = now_ns
                self._enter_phase("full_trigger_reopen")
        elif self.phase == "full_trigger_reopen":
            requested_ns = self.full_reopen_requested_ns
            if requested_ns is None:
                self.failure = "full-reopen phase is missing its Trigger edge fence"
                return
            if (
                now_ns - requested_ns > FULL_REOPEN_TEST_TIMEOUT_NS
                and not all(
                    self.full_reopen_endpoint_ns[side] is not None
                    for side in SIDES
                )
            ):
                self.failure = (
                    "timed out waiting for the proportional full-reopen "
                    "endpoint during the fake test: "
                    f"endpoint_ns={self.full_reopen_endpoint_ns}, "
                    f"commands={self.gripper_command}, "
                    f"max_steps={self.full_reopen_max_step_m}"
                )
                return
            fully_reopened = all(
                self.full_reopen_endpoint_ns[side] is not None
                and self.gripper_command[side] is not None
                and self.gripper_feedback[side] is not None
                and abs(self.gripper_command[side] - GRIPPER_OPEN_POSITION_M)
                <= GRIPPER_FOLLOW_TOLERANCE_M
                and self.gripper_feedback[side]
                >= GRIPPER_OPEN_POSITION_M - GRIPPER_FOLLOW_TOLERANCE_M
                for side in SIDES
            )
            full_reopen_rate_step_observed = all(
                self.full_reopen_max_step_m[side]
                >= GRIPPER_OPEN_STEP_M - GRIPPER_COMMAND_STEP_TOLERANCE_M
                for side in SIDES
            )
            if fully_reopened and not full_reopen_rate_step_observed:
                self.failure = (
                    "full-reopen endpoint was reached without observing a near-limit "
                    f"{GRIPPER_OPEN_STEP_M:.6f} m command step on both sides: "
                    f"max_steps={self.full_reopen_max_step_m}"
                )
                return
            if self._condition_held(
                self._both_guards_are(HOLD)
                and fully_reopened
                and full_reopen_rate_step_observed,
                now_ns,
                300_000_000,
            ):
                self.full_reopen_verified = True
                self._enter_phase("hold_trigger_close")
        elif self.phase == "hold_trigger_close":
            proportional_targets_reached = all(
                self.gripper_command[side] is not None
                and self.gripper_feedback[side] is not None
                and abs(self.gripper_command[side] - GRIPPER_TARGET_M[side])
                <= GRIPPER_FOLLOW_TOLERANCE_M
                and abs(self.gripper_feedback[side] - GRIPPER_TARGET_M[side])
                <= GRIPPER_FOLLOW_TOLERANCE_M
                for side in SIDES
            )
            if self._condition_held(
                self._both_guards_are(HOLD) and proportional_targets_reached,
                now_ns,
                300_000_000,
            ):
                self.late_peer_right_phase_started_ns = now_ns
                self._enter_phase("late_peer_right_engage")
        elif self.phase == "late_peer_right_engage":
            started_ns = self.late_peer_right_phase_started_ns
            intent_pair = self._current_intent_pair()
            right_round_closed = bool(
                started_ns is not None
                and intent_pair is not None
                and self._side_has_closed_round(
                    "right",
                    started_ns=started_ns,
                    intent_tokens=intent_pair,
                )
            )
            right_is_established = bool(
                intent_pair is not None
                and self.mode_token_mode["right"] == TELEOP
                and self.mode_token_mode["left"] == HOLD
                and self.guard_mode["right"] == TELEOP
                and self.guard_mode["left"] == HOLD
                and right_round_closed
            )
            if self.late_peer_right_established_ns is None:
                if right_is_established:
                    self.late_peer_right_established_ns = now_ns
                    self.late_peer_prejoin_tokens = intent_pair
                    # Prime both delta-node handle caches with the upcoming
                    # stage-A epochs for the whole 150 ms single-arm hold. The
                    # left Grip remains released, so this cannot create a left
                    # target or move the joining arm before its token edge.
                    self.late_peer_epoch_base_ns = int(
                        self.get_clock().now().nanoseconds
                    )
            else:
                if (
                    not right_is_established
                    or intent_pair != self.late_peer_prejoin_tokens
                ):
                    self.failure = (
                        "right arm lost its independently established TELEOP "
                        "round before the 150 ms late-peer hold completed: "
                        f"guard_modes={self.guard_mode}, "
                        f"mode_tokens={self.mode_token}"
                    )
                    return
                if (
                    now_ns - self.late_peer_right_established_ns
                    >= LATE_PEER_FIRST_SIDE_HOLD_NS
                ):
                    epoch_base_ns = self.late_peer_epoch_base_ns
                    assert epoch_base_ns is not None
                    if self._latest_target_is(
                        "right",
                        epoch_ns=epoch_base_ns,
                        raw_token=intent_pair[0],
                        after_ns=self.late_peer_right_established_ns,
                    ):
                        self.late_peer_join_requested_ns = now_ns
                        self.late_peer_stage_a_last_command_counts = dict(
                            self.command_count
                        )
                        self._enter_phase("late_peer_stage_a_wait")
        elif self.phase == "late_peer_stage_a_wait":
            self._capture_late_peer_pair(now_ns)
            if self.failure:
                return

            pair = self.late_peer_intent_tokens
            edge_seen_ns = self.late_peer_pair_seen_ns
            epoch_base_ns = self.late_peer_epoch_base_ns
            if pair is not None and edge_seen_ns is not None:
                assert self.late_peer_join_requested_ns is not None
                if self._acceptance_for_pair_seen_since(
                    pair, self.late_peer_join_requested_ns
                ):
                    self.failure = (
                        "guard accepted a new-pair arm command during late-peer "
                        "stage A even though no post-edge target epoch pair was "
                        "inside the configured 30 ms window"
                    )
                    return
                if self._any_post_edge_coherent_pair(pair, edge_seen_ns):
                    self.failure = (
                        "late-peer stage A accidentally injected a coherent "
                        "post-edge target pair"
                    )
                    return

            stage_a_targets_observed = bool(
                pair is not None
                and edge_seen_ns is not None
                and epoch_base_ns is not None
                and self._latest_target_is(
                    "right",
                    epoch_ns=epoch_base_ns,
                    raw_token=pair[0],
                    after_ns=edge_seen_ns,
                )
                and self._latest_target_is(
                    "left",
                    epoch_ns=(
                        epoch_base_ns + LATE_PEER_STAGE_A_LEFT_OFFSET_NS
                    ),
                    raw_token=pair[1],
                    after_ns=edge_seen_ns,
                )
            )
            drain_ready = stage_a_targets_observed and self._both_guards_are(HOLD)
            current_counts = dict(self.command_count)
            if not drain_ready:
                self.late_peer_stage_a_last_command_counts = current_counts
                self.late_peer_stage_a_counts_stable_ns = None
            elif current_counts != self.late_peer_stage_a_last_command_counts:
                self.late_peer_stage_a_last_command_counts = current_counts
                self.late_peer_stage_a_counts_stable_ns = now_ns
            elif self.late_peer_stage_a_counts_stable_ns is None:
                self.late_peer_stage_a_counts_stable_ns = now_ns
            elif (
                now_ns - self.late_peer_stage_a_counts_stable_ns
                >= LATE_PEER_COMMAND_DRAIN_NS
            ):
                self.late_peer_stage_a_command_baseline = current_counts
                self._enter_phase("late_peer_stage_a_verify")
        elif self.phase == "late_peer_stage_a_verify":
            pair = self.late_peer_intent_tokens
            edge_seen_ns = self.late_peer_pair_seen_ns
            baseline = self.late_peer_stage_a_command_baseline
            if pair is None or edge_seen_ns is None or baseline is None:
                self.failure = "late-peer stage A verification is missing its fence"
                return
            assert self.late_peer_join_requested_ns is not None
            if self._acceptance_for_pair_seen_since(
                pair, self.late_peer_join_requested_ns
            ):
                self.failure = (
                    "guard accepted a new-pair arm command during incoherent "
                    "late-peer stage A"
                )
                return
            if not self._both_guards_are(HOLD):
                self.failure = (
                    "guard left late-peer stage A HOLD before history-pair "
                    f"injection: guard_modes={self.guard_mode}"
                )
                return
            if self.command_count != baseline:
                self.failure = (
                    "arm command count changed during late-peer stage A HOLD: "
                    f"baseline={baseline}, observed={self.command_count}"
                )
                return
            if self._any_post_edge_coherent_pair(pair, edge_seen_ns):
                self.failure = (
                    "late-peer stage A gained an unexpected coherent target pair"
                )
                return
            if self._condition_held(True, now_ns, LATE_PEER_NO_COMMAND_NS):
                self.late_peer_no_command_verified = True
                self.late_peer_stage_b_bridge_started_ns = now_ns
                self._enter_phase("late_peer_stage_b_bridge")
        elif self.phase == "late_peer_stage_b_bridge":
            pair = self.late_peer_intent_tokens
            edge_seen_ns = self.late_peer_pair_seen_ns
            baseline = self.late_peer_stage_a_command_baseline
            bridge_started_ns = self.late_peer_stage_b_bridge_started_ns
            if (
                pair is None
                or edge_seen_ns is None
                or baseline is None
                or bridge_started_ns is None
            ):
                self.failure = "late-peer bridge is missing its token/command fence"
                return
            assert self.late_peer_join_requested_ns is not None
            if self._acceptance_for_pair_seen_since(
                pair, self.late_peer_join_requested_ns
            ):
                self.failure = (
                    "guard accepted a new-pair arm command before the late-peer "
                    "history sample was introduced"
                )
                return
            if self._any_post_edge_coherent_pair(pair, edge_seen_ns):
                self.failure = (
                    "late-peer bridge accidentally produced a coherent "
                    "post-edge target pair"
                )
                return
            if not self._both_guards_are(HOLD):
                self.failure = (
                    "guard left HOLD during the no-pair late-peer bridge: "
                    f"guard_modes={self.guard_mode}"
                )
                return
            if self.command_count != baseline:
                self.failure = (
                    "arm command count changed during the no-pair late-peer "
                    f"bridge: baseline={baseline}, observed={self.command_count}"
                )
                return
            if not self._late_peer_bridge_latest_is_incoherent():
                self.late_peer_stage_b_bridge_stable_ns = None
            elif self.late_peer_stage_b_bridge_stable_ns is None:
                self.late_peer_stage_b_bridge_stable_ns = now_ns
            elif (
                now_ns - self.late_peer_stage_b_bridge_stable_ns
                >= LATE_PEER_BRIDGE_DRAIN_NS
            ):
                self.late_peer_stage_b_started_ns = now_ns
                self._enter_phase("late_peer_stage_b_history")
        elif self.phase == "late_peer_stage_b_history":
            pair = self.late_peer_intent_tokens
            edge_seen_ns = self.late_peer_pair_seen_ns
            epoch_base_ns = self.late_peer_epoch_base_ns
            stage_b_started_ns = self.late_peer_stage_b_started_ns
            if (
                pair is None
                or edge_seen_ns is None
                or epoch_base_ns is None
                or stage_b_started_ns is None
            ):
                self.failure = "late-peer stage B is missing its token/epoch fence"
                return

            right_latest_epoch = (
                epoch_base_ns + LATE_PEER_STAGE_B_RIGHT_OFFSET_NS
            )
            left_latest_epoch = epoch_base_ns + LATE_PEER_STAGE_B_LEFT_OFFSET_NS
            latest_history_trap_observed = bool(
                self._late_peer_final_latest_is_incoherent()
                and self._post_edge_epoch_pair_exists(
                    right_epoch_ns=epoch_base_ns,
                    left_epoch_ns=left_latest_epoch,
                    intent_tokens=pair,
                    edge_seen_ns=edge_seen_ns,
                )
            )
            if not latest_history_trap_observed:
                self.late_peer_stage_b_trap_started_ns = None
                self.late_peer_history_pair_verified = False
                self.late_peer_stage_b_post_fence_ack = {
                    "right": None,
                    "left": None,
                }
            elif self.late_peer_stage_b_trap_started_ns is None:
                self.late_peer_stage_b_trap_started_ns = now_ns
            elif (
                now_ns - self.late_peer_stage_b_trap_started_ns
                >= LATE_PEER_HISTORY_TRAP_HOLD_NS
            ):
                self.late_peer_history_pair_verified = True
                if self.late_peer_stage_b_ack_fence_ns is None:
                    # The R800/L400 bridge removes every transient coherent
                    # latest pair. Still require both final R800/L10 samples to
                    # remain observed before taking the current-pair ACK fence;
                    # only a later ACK can prove history selection.
                    self.late_peer_stage_b_acceptance_baseline = {
                        side: self._pair_acceptance_count(side, pair)
                        for side in SIDES
                    }
                    self.late_peer_stage_b_ack_fence_ns = now_ns

            ack_fence_ns = self.late_peer_stage_b_ack_fence_ns
            acceptance_baseline = self.late_peer_stage_b_acceptance_baseline
            closed_rounds = all(
                self._side_has_closed_round(
                    side,
                    started_ns=stage_b_started_ns,
                    intent_tokens=pair,
                    acceptance_started_ns=ack_fence_ns,
                )
                for side in SIDES
            ) if ack_fence_ns is not None else False
            post_fence_new_acks = bool(
                ack_fence_ns is not None
                and acceptance_baseline is not None
                and all(
                    self.late_peer_stage_b_post_fence_ack[side] is not None
                    and self._pair_acceptance_count(side, pair)
                    > acceptance_baseline[side]
                    for side in SIDES
                )
            )
            if (
                self.late_peer_history_pair_verified
                and latest_history_trap_observed
                and post_fence_new_acks
                and closed_rounds
                and self._both_guards_are(TELEOP)
            ):
                assert self.late_peer_join_requested_ns is not None
                elapsed_ns = now_ns - self.late_peer_join_requested_ns
                if elapsed_ns >= LATE_PEER_ESTABLISH_DEADLINE_NS:
                    self.failure = (
                        "late-peer same-pair command rounds crossed the strict "
                        f"0.75 s deadline: elapsed_ns={elapsed_ns}"
                    )
                    return
                self.late_peer_same_pair_established = True
                self.late_peer_establish_elapsed_ns = elapsed_ns
                self._enter_phase("first_engage")
        elif self.phase == "first_engage":
            if self._condition_held(
                self._both_guards_are(TELEOP), now_ns, 300_000_000
            ):
                if not all(self.command_count[side] > 0 for side in SIDES):
                    self.failure = (
                        "both guards reported TELEOP before both arm controllers "
                        f"received a command: counts={self.command_count}"
                    )
                    return
                # A guard ACK, rather than elapsed TELEOP time, is the
                # production protocol event that closes any earlier sync-wait
                # episode. Keep coherent epochs and require one new valid ACK
                # from each side before beginning the deliberate skew.
                self.sync_prepare_acceptance_baseline = dict(
                    self.guard_acceptance_count
                )
                if not all(
                    self.latest_command[side] is not None
                    and self.latest_guard_acceptance[side] is not None
                    for side in SIDES
                ):
                    self.failure = (
                        "missing command or guard ACK payload before coherent "
                        f"prepare fence: commands={self.latest_command}, "
                        f"acceptances={self.latest_guard_acceptance}"
                    )
                    return
                self.sync_prepare_command_snapshot = dict(self.latest_command)
                self.sync_prepare_acceptance_snapshot = dict(
                    self.latest_guard_acceptance
                )
                self.sync_prepare_started_ns = now_ns
                self._enter_phase("sync_wait_prepare")
        elif self.phase == "sync_wait_prepare":
            baseline = self.sync_prepare_acceptance_baseline
            if baseline is None:
                self.failure = "missing guard ACK baseline before synchronization wait"
                return
            post_snapshot_fence = all(
                self.guard_acceptance_count[side] > baseline[side]
                and self.sync_prepare_matched_command[side] is not None
                and self.sync_prepare_matched_acceptance[side] is not None
                and self.sync_prepare_feedback_matched[side]
                for side in SIDES
            )
            if self._condition_held(
                self._both_guards_are(TELEOP) and post_snapshot_fence,
                now_ns,
                50_000_000,
            ):
                self._enter_phase("sync_wait_enter")
        elif self.phase == "sync_wait_enter":
            # Give already-published pre-wait commands time to cross DDS before
            # taking the exact command-count baseline. The following phase then
            # fails on even one additional arm controller publication.
            if self._condition_held(
                self._both_guards_are(HOLD), now_ns, SYNC_WAIT_SETTLE_NS
            ):
                self.sync_wait_hold_observed = True
                self.sync_wait_command_baseline = dict(self.command_count)
                self._enter_phase("sync_wait_hold")
        elif self.phase == "sync_wait_hold":
            if not self._both_guards_are(HOLD):
                self.failure = (
                    "guard left synchronization HOLD before epoch recovery: "
                    f"guard_modes={self.guard_mode}, counts={self.command_count}"
                )
                return
            if self.sync_wait_command_baseline is None:
                self.failure = "missing arm command baseline during synchronization HOLD"
                return
            if self.command_count != self.sync_wait_command_baseline:
                self.failure = (
                    "arm command count changed during synchronization HOLD: "
                    f"baseline={self.sync_wait_command_baseline}, "
                    f"observed={self.command_count}"
                )
                return
            if self._condition_held(True, now_ns, SYNC_WAIT_NO_COMMAND_NS):
                self.sync_wait_no_command_verified = True
                self._enter_phase("sync_wait_recover")
        elif self.phase == "sync_wait_recover":
            if self._condition_held(
                self._both_guards_are(TELEOP), now_ns, 300_000_000
            ):
                self.sync_wait_recovered = True
                self._enter_phase("release_between_engagements")
        elif self.phase == "release_between_engagements":
            if self._condition_held(
                self._both_guards_are(HOLD), now_ns, 300_000_000
            ):
                self._enter_phase("second_engage")
        elif self.phase == "second_engage":
            if self._condition_held(
                self._both_guards_are(TELEOP), now_ns, 500_000_000
            ):
                self.moving = True

        both_grips_requested = self.phase in (
            "late_peer_stage_a_wait",
            "late_peer_stage_a_verify",
            "late_peer_stage_b_bridge",
            "late_peer_stage_b_history",
            "first_engage",
            "sync_wait_prepare",
            "sync_wait_enter",
            "sync_wait_hold",
            "sync_wait_recover",
            "second_engage",
        )
        right_grip_requested = bool(
            self.phase == "late_peer_right_engage" or both_grips_requested
        )
        left_grip_requested = bool(both_grips_requested)
        teleop_requested = right_grip_requested or left_grip_requested
        if teleop_requested and any(
            self.guard_mode[side] == FAULT for side in SIDES
        ):
            self.failure = (
                f"guard latched FAULT during {self.phase}: "
                f"guard_modes={self.guard_mode}, "
                f"guard_acceptance_counts={self.guard_acceptance_count}, "
                f"sync_prepare_acceptance_baseline="
                f"{self.sync_prepare_acceptance_baseline}"
            )
            return

        self.source_pub.publish(Bool(data=True))
        buttons = Joy()
        # Production order: leftTrig, rightTrig, leftGrip, rightGrip.  LG/RG
        # deliberately remain false for the whole run: only analog Grip may
        # engage TELEOP. Distinct trigger values also exercise both grippers.
        left_grip = 0.90 if left_grip_requested else 0.0
        right_grip = 0.90 if right_grip_requested else 0.0
        # Released Trigger must open both grippers before either Grip engages.
        # Then distinct Trigger values close both grippers to their proportional
        # endpoints while the arm remains in HOLD, proving this path is genuinely
        # independent from the arm deadman.
        if self.phase == "initial_trigger_held":
            left_trigger, right_trigger = (1.0, 1.0)
        elif self.phase in ("initial_release", "full_trigger_reopen"):
            left_trigger, right_trigger = (0.0, 0.0)
        elif self.phase == "full_trigger_close":
            left_trigger, right_trigger = (1.0, 1.0)
        else:
            left_trigger, right_trigger = (0.75, 0.25)
        buttons.axes = [left_trigger, right_trigger, left_grip, right_grip]
        buttons.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        self.button_pub.publish(buttons)

        stamp = self.get_clock().now().to_msg()
        if self.phase == "sync_wait_prepare":
            displacement = SYNC_PREPARE_DISPLACEMENT_M
        else:
            displacement = 0.003 if self.moving else 0.0
        # Normally use neighboring (10 ms apart) Quest epochs. During the
        # explicit synchronization-wait phases, force a 400 ms separation so
        # the IK node must HOLD without issuing a new arm controller command.
        late_peer_epoch_base_ns = self.late_peer_epoch_base_ns
        if (
            self.phase == "late_peer_right_engage"
            and self.late_peer_right_established_ns is not None
            and late_peer_epoch_base_ns is not None
        ) or self.phase in (
            "late_peer_stage_a_wait",
            "late_peer_stage_a_verify",
        ):
            assert late_peer_epoch_base_ns is not None
            right_stamp = stamp_from_ns(late_peer_epoch_base_ns)
            left_stamp = stamp_from_ns(
                late_peer_epoch_base_ns + LATE_PEER_STAGE_A_LEFT_OFFSET_NS
            )
        elif self.phase == "late_peer_stage_b_bridge":
            assert late_peer_epoch_base_ns is not None
            right_stamp = stamp_from_ns(
                late_peer_epoch_base_ns + LATE_PEER_STAGE_B_RIGHT_OFFSET_NS
            )
            left_stamp = stamp_from_ns(
                late_peer_epoch_base_ns + LATE_PEER_STAGE_A_LEFT_OFFSET_NS
            )
        elif self.phase == "late_peer_stage_b_history":
            assert late_peer_epoch_base_ns is not None
            right_stamp = stamp_from_ns(
                late_peer_epoch_base_ns + LATE_PEER_STAGE_B_RIGHT_OFFSET_NS
            )
            left_stamp = stamp_from_ns(
                late_peer_epoch_base_ns + LATE_PEER_STAGE_B_LEFT_OFFSET_NS
            )
        else:
            left_epoch_offset_ns = (
                INCOHERENT_LEFT_EPOCH_OFFSET_NS
                if self.phase in ("sync_wait_enter", "sync_wait_hold")
                else COHERENT_LEFT_EPOCH_OFFSET_NS
            )
            right_stamp = stamp
            left_stamp = offset_stamp(stamp, left_epoch_offset_ns)
        self.pose_pub["right"].publish(self._pose(right_stamp, displacement))
        self.pose_pub["left"].publish(self._pose(left_stamp, -displacement))


def main() -> int:
    rclpy.init()
    node = FakeQuestSmoke()
    try:
        while rclpy.ok() and not node.success and not node.failure:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.success:
        print(
            "OPENARM_FAKE_TELEOP_OK "
            f"right_command={node.command['right']} "
            f"right_feedback={node.feedback['right']} "
            f"left_command={node.command['left']} "
            f"left_feedback={node.feedback['left']} "
            f"right_gripper_command={node.gripper_command['right']} "
            f"right_gripper_feedback={node.gripper_feedback['right']} "
            f"left_gripper_command={node.gripper_command['left']} "
            f"left_gripper_feedback={node.gripper_feedback['left']} "
            f"full_close_elapsed_sec="
            f"{ {side: (node.full_close_endpoint_ns[side] - node.full_close_requested_ns) / 1e9 for side in SIDES} } "
            f"full_reopen_elapsed_sec="
            f"{ {side: (node.full_reopen_endpoint_ns[side] - node.full_reopen_requested_ns) / 1e9 for side in SIDES} } "
            f"full_close_max_step_m={node.full_close_max_step_m} "
            f"full_reopen_max_step_m={node.full_reopen_max_step_m} "
            f"sync_prepare_acceptance_baseline="
            f"{node.sync_prepare_acceptance_baseline} "
            f"guard_acceptance_counts={node.guard_acceptance_count} "
            f"sync_prepare_feedback_matched="
            f"{node.sync_prepare_feedback_matched} "
            f"sync_wait_command_baseline={node.sync_wait_command_baseline} "
            f"arm_command_counts={node.command_count} "
            f"late_peer_pair={node.late_peer_intent_tokens} "
            f"late_peer_stage_a_command_baseline="
            f"{node.late_peer_stage_a_command_baseline} "
            f"late_peer_history_pair_verified="
            f"{node.late_peer_history_pair_verified} "
            f"late_peer_stage_b_acceptance_baseline="
            f"{node.late_peer_stage_b_acceptance_baseline} "
            f"late_peer_stage_b_post_fence_ack="
            f"{node.late_peer_stage_b_post_fence_ack} "
            f"late_peer_establish_elapsed_ns="
            f"{node.late_peer_establish_elapsed_ns}"
        )
        return 0
    print(f"OPENARM_FAKE_TELEOP_FAILED: {node.failure}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
