#!/usr/bin/env python3
"""Isolated SocketCAN/OpenArmHW regression tests for OpenArm v1.

Run this only in a disposable Docker network namespace created with
``--network none``.  The script refuses to start unless the namespace initially
contains exactly the loopback interface, creates only ``vcan42``, and never
accepts a CAN-interface argument.  It therefore cannot select host can0/can1.

The raw protocol tests cover staggered eight-ID feedback, one missing ID, and
an unexpected raw motor status.  The lifecycle tests compile a tiny probe in
``/tmp`` and exercise activation, the one-shot runtime bootstrap transaction,
the gripper's CTRL_MODE acknowledgement, 100 Hz runtime feedback, fresh-sample
arm/gripper velocity boundaries, torque persistence, and directional gripper
rate credit against the same fake motors.
"""

from __future__ import annotations

import errno
import heapq
import os
from pathlib import Path
import select
import shlex
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from typing import Iterable, Mapping


TEST_INTERFACE = "vcan42"
FORBIDDEN_REAL_INTERFACES = frozenset({"can0", "can1"})
MOTOR_IDS = tuple(range(1, 9))
RESPONSE_IDS = frozenset(0x10 + motor_id for motor_id in MOTOR_IDS)
GRIPPER_MOTOR_ID = 8
CTRL_MODE_RID = 10
MIT_CONTROL_MODE = 1

# The pinned OpenArm v1 arm motor sequence is DM8009, DM8009, DM4340, DM4340,
# DM4310, DM4310, DM4310.  Their 12-bit state-feedback velocity codecs span
# respectively +/-45, +/-10, and +/-30 rad/s.  These samples are the exact last
# representable positive values inside the commissioned per-axis gates and the
# first representable values outside them.  First-over samples alternate sign
# so the dynamic suite also exercises both branches of abs(dq) without adding
# redundant cases.
ARM_VELOCITY_CODEC_MAX_RAD_S = {
    1: 45.0,
    2: 45.0,
    3: 10.0,
    4: 10.0,
    5: 30.0,
    6: 30.0,
    7: 30.0,
}
ARM_VELOCITY_LIMIT_RAD_S = {
    1: 15.079644737,
    2: 15.079644737,
    3: 4.900884539,
    4: 4.900884539,
    5: 18.849555921,
    6: 18.849555921,
    7: 18.849555921,
}
ARM_VELOCITY_LAST_SAFE_RAW = {
    1: 2733,
    2: 2733,
    3: 3050,
    4: 3050,
    5: 3333,
    6: 3333,
    7: 3333,
}
ARM_VELOCITY_FIRST_OVER_RAW = {
    1: 2734,  # +15.087912088 rad/s
    2: 1361,  # -15.087912088 rad/s
    3: 3051,  # +4.901098901 rad/s
    4: 1044,  # -4.901098901 rad/s
    5: 3334,  # +18.849816850 rad/s
    6: 761,   # -18.849816850 rad/s
    7: 3334,  # +18.849816850 rad/s
}

# DM4340 torque feedback spans [-28, 28] Nm over 12 bits. The manufacturer
# peak output torque is 27 Nm: raw 4021 is the last representable sample below
# it (26.988 Nm), and raw 4022 is the first sample above it (27.002 Nm).
J4_TORQUE_LAST_SAFE_RAW = 4021
J4_TORQUE_FIRST_OVER_RAW = 4022
# DM4310 (ID8) spans [-10, 10] Nm. Its 7 Nm peak boundary is raw 3480/3481
# (6.996/7.001 Nm respectively).
GRIPPER_TORQUE_LAST_SAFE_RAW = 3480
GRIPPER_TORQUE_FIRST_OVER_RAW = 3481
# DM4310 ID8 velocity feedback spans [-30, 30] rad/s over 12 bits.  The
# commissioned gross-feedback gate is 90% of the motor's 200 rpm rated speed:
# 18.849555921 rad/s.  No separate nominal/persistence band exists.  Raw 3333
# is the last positive sample inside the gate; 3334 and its negative mirror 761
# are the first samples outside it.
GRIPPER_VELOCITY_LIMIT_RAD_S = 18.849555921
GRIPPER_VELOCITY_LAST_SAFE_RAW = 3333
GRIPPER_VELOCITY_FIRST_OVER_POSITIVE_RAW = 3334
GRIPPER_VELOCITY_FIRST_OVER_NEGATIVE_RAW = 761

CAN_RAW_FD_FRAMES = 5
CANFD_BRS = 0x01
SOL_CAN_RAW = 101
CAN_EFF_MASK = 0x1FFFFFFF
CANFD_FRAME = struct.Struct("=IBBBB64s")

ROS_SETUP_FILES = (
    Path("/opt/ros/humble/setup.bash"),
    Path("/opt/openarm_ws/install/setup.bash"),
)


class RegressionFailure(RuntimeError):
    """A test precondition or expected behavior failed."""


class FeedbackTimeout(RegressionFailure):
    """Not every expected OpenArm motor replied before the deadline."""


class RawStatusRejected(RegressionFailure):
    """A syntactically valid frame carried an unsafe raw status nibble."""


def _decode_arm_velocity_raw(motor_id: int, raw: int) -> float:
    velocity_max = ARM_VELOCITY_CODEC_MAX_RAD_S[motor_id]
    return raw / 4095.0 * (2.0 * velocity_max) - velocity_max


def _assert_velocity_boundary_samples() -> None:
    for motor_id in range(1, GRIPPER_MOTOR_ID):
        limit = ARM_VELOCITY_LIMIT_RAD_S[motor_id]
        safe_raw = ARM_VELOCITY_LAST_SAFE_RAW[motor_id]
        first_over_raw = ARM_VELOCITY_FIRST_OVER_RAW[motor_id]
        safe_velocity = _decode_arm_velocity_raw(motor_id, safe_raw)
        first_over_velocity = _decode_arm_velocity_raw(motor_id, first_over_raw)
        toward_zero_raw = (
            first_over_raw - 1 if first_over_raw > 2047 else first_over_raw + 1
        )
        toward_zero_velocity = _decode_arm_velocity_raw(motor_id, toward_zero_raw)
        if not (
            abs(safe_velocity) <= limit
            and abs(toward_zero_velocity) <= limit
            and abs(first_over_velocity) > limit
        ):
            raise RegressionFailure(
                "incorrect arm velocity boundary samples for "
                f"ID{motor_id}: safe={safe_velocity:.12f}, "
                f"toward_zero={toward_zero_velocity:.12f}, "
                f"first_over={first_over_velocity:.12f}, limit={limit:.12f}"
            )

    def decode_gripper(raw: int) -> float:
        return raw / 4095.0 * 60.0 - 30.0

    safe_velocity = decode_gripper(GRIPPER_VELOCITY_LAST_SAFE_RAW)
    positive_over_velocity = decode_gripper(
        GRIPPER_VELOCITY_FIRST_OVER_POSITIVE_RAW
    )
    negative_over_velocity = decode_gripper(
        GRIPPER_VELOCITY_FIRST_OVER_NEGATIVE_RAW
    )
    if not (
        abs(safe_velocity) <= GRIPPER_VELOCITY_LIMIT_RAD_S
        and abs(positive_over_velocity) > GRIPPER_VELOCITY_LIMIT_RAD_S
        and abs(negative_over_velocity) > GRIPPER_VELOCITY_LIMIT_RAD_S
        and abs(decode_gripper(GRIPPER_VELOCITY_FIRST_OVER_POSITIVE_RAW - 1))
        <= GRIPPER_VELOCITY_LIMIT_RAD_S
        and abs(decode_gripper(GRIPPER_VELOCITY_FIRST_OVER_NEGATIVE_RAW + 1))
        <= GRIPPER_VELOCITY_LIMIT_RAD_S
    ):
        raise RegressionFailure(
            "incorrect gripper velocity boundary samples: "
            f"safe={safe_velocity:.12f}, "
            f"positive_over={positive_over_velocity:.12f}, "
            f"negative_over={negative_over_velocity:.12f}, "
            f"limit={GRIPPER_VELOCITY_LIMIT_RAD_S:.12f}"
        )


def _run_checked(command: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )


def _network_interfaces() -> set[str]:
    return {entry.name for entry in Path("/sys/class/net").iterdir()}


def _assert_isolated_namespace_before_vcan() -> None:
    interfaces = _network_interfaces()
    if interfaces & FORBIDDEN_REAL_INTERFACES:
        raise RegressionFailure(
            "refusing to run: real CAN names are visible in this network "
            f"namespace: {sorted(interfaces & FORBIDDEN_REAL_INTERFACES)}"
        )
    if interfaces != {"lo"}:
        raise RegressionFailure(
            "refusing to run: expected an empty '--network none' namespace "
            f"containing only lo, found {sorted(interfaces)}"
        )
    print(
        "PASS isolation_guard initial_interfaces=lo "
        f"netns={os.readlink('/proc/self/ns/net')}",
        flush=True,
    )


def _create_vcan() -> None:
    _run_checked(["ip", "link", "add", "dev", TEST_INTERFACE, "type", "vcan"])
    _run_checked(["ip", "link", "set", "dev", TEST_INTERFACE, "up"])
    interfaces = _network_interfaces()
    if interfaces != {"lo", TEST_INTERFACE}:
        raise RegressionFailure(
            f"unexpected interfaces after vcan creation: {sorted(interfaces)}"
        )
    print(f"PASS vcan_created interface={TEST_INTERFACE}", flush=True)


def _delete_vcan() -> None:
    if TEST_INTERFACE in _network_interfaces():
        subprocess.run(
            ["ip", "link", "del", "dev", TEST_INTERFACE],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def _open_canfd_socket() -> socket.socket:
    can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    can_socket.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
    can_socket.bind((TEST_INTERFACE,))
    return can_socket


def _pack_canfd(can_id: int, payload: bytes) -> bytes:
    if len(payload) > 64:
        raise ValueError("CAN-FD payload exceeds 64 bytes")
    return CANFD_FRAME.pack(
        can_id,
        len(payload),
        CANFD_BRS,
        0,
        0,
        payload.ljust(64, b"\x00"),
    )


def _unpack_canfd(frame: bytes) -> tuple[int, bytes]:
    if len(frame) != CANFD_FRAME.size:
        raise RegressionFailure(f"unexpected SocketCAN frame size: {len(frame)}")
    can_id, length, _flags, _reserved0, _reserved1, payload = CANFD_FRAME.unpack(frame)
    if length > 64:
        raise RegressionFailure(f"invalid CAN-FD length: {length}")
    return can_id & CAN_EFF_MASK, payload[:length]


def _state_payload(
    motor_id: int,
    raw_status: int,
    *,
    dq_uint: int = 2048,
    tau_uint: int = 2048,
) -> bytes:
    # Slightly-positive near-zero values survive the integer quantization and
    # satisfy joint 4's [0, ...] position limit in the patched safety gate.
    q_uint = 32768
    if not 0 <= dq_uint <= 0xFFF:
        raise RegressionFailure(f"invalid 12-bit velocity sample: {dq_uint}")
    if not 0 <= tau_uint <= 0xFFF:
        raise RegressionFailure(f"invalid 12-bit torque sample: {tau_uint}")
    return bytes(
        (
            ((raw_status & 0x0F) << 4) | motor_id,
            (q_uint >> 8) & 0xFF,
            q_uint & 0xFF,
            (dq_uint >> 4) & 0xFF,
            ((dq_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F),
            tau_uint & 0xFF,
            25,
            25,
        )
    )


class FakeOpenArmMotors:
    """Eight small state machines responding on a single isolated vcan bus."""

    def __init__(
        self,
        *,
        delays: Mapping[int, float],
        missing_id: int | None = None,
        forced_raw_status: int | None = None,
        bootstrap_missing_id: int | None = None,
        bootstrap_fault_id: int | None = None,
        bootstrap_fault_status: int | None = None,
        runtime_missing_id: int | None = None,
        runtime_fault_id: int | None = None,
        runtime_fault_status: int | None = None,
        activation_velocity_raw: Mapping[int, int] | None = None,
        activation_torque_raw: Mapping[int, int] | None = None,
        runtime_velocity_raw_by_batch: Mapping[int, Mapping[int, int]] | None = None,
        runtime_torque_raw_by_batch: Mapping[int, Mapping[int, int]] | None = None,
        runtime_missing_by_batch: Mapping[int, frozenset[int]] | None = None,
        coalesce_replies: bool = False,
    ) -> None:
        self._delays = dict(delays)
        self._missing_id = missing_id
        self._forced_raw_status = forced_raw_status
        self._bootstrap_missing_id = bootstrap_missing_id
        self._bootstrap_fault_id = bootstrap_fault_id
        self._bootstrap_fault_status = bootstrap_fault_status
        self._runtime_missing_id = runtime_missing_id
        self._runtime_fault_id = runtime_fault_id
        self._runtime_fault_status = runtime_fault_status
        self._activation_velocity_raw = dict(activation_velocity_raw or {})
        self._activation_torque_raw = dict(activation_torque_raw or {})
        self._runtime_velocity_raw_by_batch = {
            batch: dict(samples)
            for batch, samples in (runtime_velocity_raw_by_batch or {}).items()
        }
        self._runtime_torque_raw_by_batch = {
            batch: dict(samples)
            for batch, samples in (runtime_torque_raw_by_batch or {}).items()
        }
        self._runtime_missing_by_batch = {
            batch: frozenset(motor_ids)
            for batch, motor_ids in (runtime_missing_by_batch or {}).items()
        }
        self._coalesce_replies = coalesce_replies
        self._enabled = {motor_id: False for motor_id in MOTOR_IDS}
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._errors: list[BaseException] = []
        self._scheduled: list[tuple[float, int, int, bytes]] = []
        self._sequence = 0
        self._batch_due = 0.0
        self._post_enable_mit_commands = 0
        self._bootstrap_refresh_commands = 0
        self._runtime_mit_commands = 0
        self._gripper_mode_ack_commands = 0
        self.command_count = 0

    def __enter__(self) -> "FakeOpenArmMotors":
        self._thread.start()
        if not self._ready.wait(timeout=2.0):
            raise RegressionFailure("fake motor thread did not bind to vcan")
        self.check()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            raise RegressionFailure("fake motor thread did not stop")
        if exc_type is None:
            self.check()

    def check(self) -> None:
        if self._errors:
            raise RegressionFailure(f"fake motor failure: {self._errors[0]}")

    @property
    def all_disabled(self) -> bool:
        return not any(self._enabled.values())

    @property
    def bootstrap_refresh_command_count(self) -> int:
        return self._bootstrap_refresh_commands

    @property
    def runtime_mit_command_count(self) -> int:
        return self._runtime_mit_commands

    @property
    def gripper_mode_ack_command_count(self) -> int:
        return self._gripper_mode_ack_commands

    def _status(self, motor_id: int) -> int:
        if self._forced_raw_status is not None:
            return self._forced_raw_status
        return 1 if self._enabled[motor_id] else 0

    def _schedule_payload(
        self, can_id: int, payload: bytes, due: float
    ) -> None:
        self._sequence += 1
        heapq.heappush(self._scheduled, (due, self._sequence, can_id, payload))

    def _schedule_state(
        self,
        motor_id: int,
        now: float,
        raw_status_override: int | None = None,
        raw_velocity_override: int | None = None,
        raw_torque_override: int | None = None,
    ) -> None:
        if motor_id not in MOTOR_IDS or motor_id == self._missing_id:
            return
        delay = self._delays.get(motor_id, 0.0)
        if self._coalesce_replies:
            if now >= self._batch_due:
                self._batch_due = now + delay
            due = self._batch_due
        else:
            due = now + delay
        raw_status = (
            self._status(motor_id)
            if raw_status_override is None
            else raw_status_override
        )
        self._schedule_payload(
            0x10 + motor_id,
            _state_payload(
                motor_id,
                raw_status,
                dq_uint=(
                    2048
                    if raw_velocity_override is None
                    else raw_velocity_override
                ),
                tau_uint=(
                    2048 if raw_torque_override is None else raw_torque_override
                ),
            ),
            due,
        )

    def _schedule_gripper_mode_ack(self, payload: bytes, now: float) -> None:
        # The real motor echoes the 0x55 parameter write on its receive CAN ID.
        # OpenArmHW keeps the gripper callback in PARAM mode until it observes
        # this exact CTRL_MODE=MIT acknowledgement.
        delay = self._delays.get(GRIPPER_MOTOR_ID, 0.0)
        self._schedule_payload(0x10 + GRIPPER_MOTOR_ID, payload[:8], now + delay)

    def _handle_command(self, can_id: int, payload: bytes, now: float) -> None:
        motor_id: int | None = None
        is_mit_command = False
        is_refresh_command = False
        is_gripper_mode_write = False
        if can_id == 0x7FF and len(payload) >= 4:
            motor_id = payload[0] | (payload[1] << 8)
            if payload[2] == 0xCC:
                is_refresh_command = True
            elif (
                motor_id == GRIPPER_MOTOR_ID
                and payload[2] == 0x55
                and payload[3] == CTRL_MODE_RID
                and int.from_bytes(payload[4:8], byteorder="little")
                == MIT_CONTROL_MODE
            ):
                is_gripper_mode_write = True
        elif can_id in MOTOR_IDS and len(payload) >= 8:
            motor_id = can_id
            if payload[:7] == b"\xFF" * 7:
                if payload[7] == 0xFC:
                    self._enabled[motor_id] = True
                elif payload[7] == 0xFD:
                    self._enabled[motor_id] = False
            else:
                is_mit_command = True

        if motor_id in MOTOR_IDS:
            self.command_count += 1
            if is_gripper_mode_write:
                self._gripper_mode_ack_commands += 1
                self._schedule_gripper_mode_ack(payload, now)
                return
            bootstrap_phase = (
                is_refresh_command
                and all(self._enabled.values())
                and self._post_enable_mit_commands == len(MOTOR_IDS)
                and self._bootstrap_refresh_commands < len(MOTOR_IDS)
            )
            bootstrap_status = None
            if bootstrap_phase:
                self._bootstrap_refresh_commands += 1
                if motor_id == self._bootstrap_missing_id:
                    return
                if motor_id == self._bootstrap_fault_id:
                    bootstrap_status = self._bootstrap_fault_status

            activation_hold_phase = False
            runtime_phase = False
            runtime_batch = 0
            if is_mit_command and all(self._enabled.values()):
                self._post_enable_mit_commands += 1
                # The first complete MIT batch after enable is the activation
                # hold transaction.  Fault injection starts only on later MIT
                # batches, so on_activate() must first succeed normally.
                activation_hold_phase = (
                    self._post_enable_mit_commands <= len(MOTOR_IDS)
                )
                runtime_phase = not activation_hold_phase
                if runtime_phase:
                    self._runtime_mit_commands += 1
                    runtime_batch = (
                        (self._runtime_mit_commands - 1) // len(MOTOR_IDS)
                    ) + 1
            if runtime_phase and motor_id == self._runtime_missing_id:
                return
            if (
                runtime_phase
                and motor_id in self._runtime_missing_by_batch.get(
                    runtime_batch, frozenset()
                )
            ):
                return
            response_status = bootstrap_status
            if runtime_phase and motor_id == self._runtime_fault_id:
                response_status = self._runtime_fault_status
            response_velocity = None
            response_torque = None
            if activation_hold_phase:
                response_velocity = self._activation_velocity_raw.get(motor_id)
                response_torque = self._activation_torque_raw.get(motor_id)
            elif runtime_phase:
                response_velocity = self._runtime_velocity_raw_by_batch.get(
                    runtime_batch, {}
                ).get(motor_id)
                response_torque = self._runtime_torque_raw_by_batch.get(
                    runtime_batch, {}
                ).get(motor_id)
            self._schedule_state(
                motor_id,
                now,
                raw_status_override=response_status,
                raw_velocity_override=response_velocity,
                raw_torque_override=response_torque,
            )

    def _send_due(self, can_socket: socket.socket, now: float) -> None:
        while self._scheduled and self._scheduled[0][0] <= now:
            _due, _sequence, can_id, payload = heapq.heappop(self._scheduled)
            can_socket.send(_pack_canfd(can_id, payload))

    def _run(self) -> None:
        try:
            with _open_canfd_socket() as can_socket:
                can_socket.setblocking(False)
                self._ready.set()
                while not self._stop.is_set():
                    now = time.monotonic()
                    self._send_due(can_socket, now)
                    timeout = 0.02
                    if self._scheduled:
                        timeout = min(timeout, max(0.0, self._scheduled[0][0] - now))
                    readable, _, _ = select.select([can_socket], [], [], timeout)
                    if not readable:
                        continue
                    while True:
                        try:
                            frame = can_socket.recv(CANFD_FRAME.size)
                        except BlockingIOError as exception:
                            if exception.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                                break
                            raise
                        can_id, payload = _unpack_canfd(frame)
                        self._handle_command(can_id, payload, time.monotonic())
        except BaseException as exception:  # surfaced synchronously by check()
            self._errors.append(exception)
            self._ready.set()


def _request_all_feedback(can_socket: socket.socket) -> None:
    for motor_id in MOTOR_IDS:
        payload = bytes((motor_id, 0, 0xCC, 0, 0, 0, 0, 0))
        can_socket.send(_pack_canfd(0x7FF, payload))


def _collect_feedback(
    can_socket: socket.socket,
    *,
    expected_status: int,
    deadline_seconds: float,
) -> dict[int, bytes]:
    deadline = time.monotonic() + deadline_seconds
    received: dict[int, bytes] = {}
    while len(received) < len(RESPONSE_IDS):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            missing = sorted(RESPONSE_IDS - received.keys())
            raise FeedbackTimeout(
                "feedback deadline expired; missing response IDs "
                + ",".join(f"0x{can_id:02x}" for can_id in missing)
            )
        readable, _, _ = select.select([can_socket], [], [], remaining)
        if not readable:
            continue
        can_id, payload = _unpack_canfd(can_socket.recv(CANFD_FRAME.size))
        if can_id not in RESPONSE_IDS:
            continue
        if len(payload) < 8:
            raise RegressionFailure(f"short state frame from 0x{can_id:02x}")
        expected_motor_id = can_id - 0x10
        if (payload[0] & 0x0F) != expected_motor_id:
            raise RegressionFailure(f"motor ID mismatch in response 0x{can_id:02x}")
        raw_status = payload[0] >> 4
        if raw_status != expected_status:
            raise RawStatusRejected(
                f"response 0x{can_id:02x} raw status {raw_status} != "
                f"expected {expected_status}"
            )
        received[can_id] = payload
    return received


def _raw_feedback_regressions() -> None:
    staggered_delays = {motor_id: 0.003 * motor_id for motor_id in MOTOR_IDS}
    with FakeOpenArmMotors(delays=staggered_delays) as fake:
        with _open_canfd_socket() as client:
            _request_all_feedback(client)
            responses = _collect_feedback(
                client, expected_status=0, deadline_seconds=0.25
            )
        fake.check()
    if set(responses) != RESPONSE_IDS:
        raise RegressionFailure("eight-ID delayed success returned the wrong IDs")
    print(
        "PASS raw_eight_id_staggered_success "
        "ids=0x11..0x18 max_delay_ms=24",
        flush=True,
    )

    with FakeOpenArmMotors(
        delays={motor_id: 0.002 for motor_id in MOTOR_IDS}, missing_id=7
    ):
        with _open_canfd_socket() as client:
            _request_all_feedback(client)
            try:
                _collect_feedback(client, expected_status=0, deadline_seconds=0.08)
            except FeedbackTimeout as exception:
                if "0x17" not in str(exception):
                    raise
            else:
                raise RegressionFailure("missing ID 0x17 was not detected")
    print("PASS raw_missing_id_timeout missing=0x17", flush=True)

    with FakeOpenArmMotors(
        delays={motor_id: 0.002 for motor_id in MOTOR_IDS},
        forced_raw_status=2,
    ):
        with _open_canfd_socket() as client:
            _request_all_feedback(client)
            try:
                _collect_feedback(client, expected_status=0, deadline_seconds=0.08)
            except RawStatusRejected as exception:
                if "raw status 2" not in str(exception):
                    raise
            else:
                raise RegressionFailure("unsafe raw status 2 was accepted")
    print("PASS raw_status_2_rejected", flush=True)


def _ros_environment() -> dict[str, str]:
    missing = [str(path) for path in ROS_SETUP_FILES if not path.is_file()]
    if missing:
        raise RegressionFailure(f"ROS/OpenArm setup files are missing: {missing}")
    source_command = " && ".join(
        f"source {shlex.quote(str(path))}" for path in ROS_SETUP_FILES
    )
    completed = subprocess.run(
        ["/bin/bash", "-lc", f"{source_command} && env -0"],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    environment: dict[str, str] = {}
    for item in completed.stdout.split(b"\0"):
        if not item:
            continue
        key, value = item.split(b"=", 1)
        environment[key.decode()] = value.decode()
    return environment


def _build_activation_probe(build_root: Path) -> tuple[Path, dict[str, str]]:
    source_root = Path(__file__).resolve().parent / "openarm_vcan_probe"
    if not (source_root / "CMakeLists.txt").is_file():
        raise RegressionFailure(f"activation probe sources not found: {source_root}")
    environment = _ros_environment()
    environment["HOME"] = str(build_root / "home")
    environment["XDG_CACHE_HOME"] = str(build_root / "cache")
    Path(environment["HOME"]).mkdir()
    Path(environment["XDG_CACHE_HOME"]).mkdir()

    build_dir = build_root / "build"
    configure = subprocess.run(
        [
            "cmake",
            "-S",
            str(source_root),
            "-B",
            str(build_dir),
            "-DCMAKE_BUILD_TYPE=Release",
        ],
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if configure.returncode != 0:
        raise RegressionFailure("activation probe CMake failed:\n" + configure.stdout)
    build = subprocess.run(
        ["cmake", "--build", str(build_dir), "--parallel", "2"],
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if build.returncode != 0:
        raise RegressionFailure("activation probe build failed:\n" + build.stdout)
    binary = build_dir / "openarm_vcan_activation_probe"
    if not binary.is_file():
        raise RegressionFailure(f"activation probe binary missing: {binary}")
    print("PASS patched_openarmhw_probe_compiled", flush=True)
    return binary, environment


def _run_activation_case(
    binary: Path,
    environment: Mapping[str, str],
    *,
    name: str,
    expected_outcome: str,
    missing_id: int | None = None,
    forced_raw_status: int | None = None,
    bootstrap_missing_id: int | None = None,
    bootstrap_fault_id: int | None = None,
    bootstrap_fault_status: int | None = None,
    runtime_missing_id: int | None = None,
    runtime_fault_id: int | None = None,
    runtime_fault_status: int | None = None,
    activation_velocity_raw: Mapping[int, int] | None = None,
    activation_torque_raw: Mapping[int, int] | None = None,
    runtime_velocity_raw_by_batch: Mapping[int, Mapping[int, int]] | None = None,
    runtime_torque_raw_by_batch: Mapping[int, Mapping[int, int]] | None = None,
    runtime_missing_by_batch: Mapping[int, frozenset[int]] | None = None,
    post_activate_delay_ms: int | None = None,
    runtime_error_cycle_range: tuple[int, int] | None = None,
    expected_log: str | None = None,
    expected_logs: Iterable[str] = (),
    gripper_gate_mode: str | None = None,
) -> None:
    # Leave a real inter-frame gap so a single upstream recv_all() call sees
    # only the first reply.  The patched hardware collector must keep receiving
    # under one total deadline until all eight unique motor counters advance.
    batch_delays = {motor_id: 0.001 * motor_id for motor_id in MOTOR_IDS}
    with FakeOpenArmMotors(
        delays=batch_delays,
        missing_id=missing_id,
        forced_raw_status=forced_raw_status,
        bootstrap_missing_id=bootstrap_missing_id,
        bootstrap_fault_id=bootstrap_fault_id,
        bootstrap_fault_status=bootstrap_fault_status,
        runtime_missing_id=runtime_missing_id,
        runtime_fault_id=runtime_fault_id,
        runtime_fault_status=runtime_fault_status,
        activation_velocity_raw=activation_velocity_raw,
        activation_torque_raw=activation_torque_raw,
        runtime_velocity_raw_by_batch=runtime_velocity_raw_by_batch,
        runtime_torque_raw_by_batch=runtime_torque_raw_by_batch,
        runtime_missing_by_batch=runtime_missing_by_batch,
        coalesce_replies=False,
    ) as fake:
        case_environment = dict(environment)
        if expected_outcome == "runtime_rejected":
            case_environment["OPENARM_VCAN_EXPECT_RUNTIME_ERROR"] = "1"
        elif expected_outcome == "bootstrap_rejected":
            case_environment["OPENARM_VCAN_EXPECT_BOOTSTRAP_ERROR"] = "1"
        if post_activate_delay_ms is not None:
            case_environment["OPENARM_VCAN_POST_ACTIVATE_DELAY_MS"] = str(
                post_activate_delay_ms
            )
        if runtime_error_cycle_range is not None:
            minimum_cycle, maximum_cycle = runtime_error_cycle_range
            case_environment["OPENARM_VCAN_RUNTIME_ERROR_MIN_CYCLE"] = str(
                minimum_cycle
            )
            case_environment["OPENARM_VCAN_RUNTIME_ERROR_MAX_CYCLE"] = str(
                maximum_cycle
            )
        if gripper_gate_mode is not None:
            case_environment["OPENARM_VCAN_GRIPPER_GATE_MODE"] = gripper_gate_mode
        completed = subprocess.run(
            [str(binary)],
            env=case_environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=15.0,
        )
        fake.check()
        fake_all_disabled = fake.all_disabled
        bootstrap_refresh_commands = fake.bootstrap_refresh_command_count
        runtime_mit_commands = fake.runtime_mit_command_count
        gripper_mode_ack_commands = fake.gripper_mode_ack_command_count

    if gripper_mode_ack_commands != 1:
        raise RegressionFailure(
            f"actual OpenArmHW case '{name}' did not perform exactly one "
            "gripper CTRL_MODE=MIT acknowledgement transaction "
            f"(commands={gripper_mode_ack_commands}):\n{completed.stdout}"
        )

    delay_observed = post_activate_delay_ms is None or (
        f"OPENARM_VCAN_POST_ACTIVATE_DELAY_MS={post_activate_delay_ms}"
        in completed.stdout
    )
    required_logs = tuple(expected_logs)
    if expected_log is not None:
        required_logs += (expected_log,)
    expected_logs_observed = all(log in completed.stdout for log in required_logs)
    production_shutdown_observed = (
        "OPENARM_VCAN_RCLCPP_SHUTDOWN=YES" in completed.stdout
    )

    succeeded = (
        completed.returncode == 0
        and "OPENARM_VCAN_ACTIVATION=SUCCESS" in completed.stdout
        and "OPENARM_VCAN_RUNTIME=SUCCESS cycles=30" in completed.stdout
        and "OPENARM_VCAN_DEACTIVATION=SUCCESS" in completed.stdout
        and fake_all_disabled
        and delay_observed
        and expected_logs_observed
    )
    rejected = (
        completed.returncode != 0
        and "OPENARM_VCAN_ACTIVATION=ERROR" in completed.stdout
        and fake_all_disabled
        and expected_logs_observed
    )
    runtime_rejected = (
        completed.returncode == 0
        and "OPENARM_VCAN_ACTIVATION=SUCCESS" in completed.stdout
        and "OPENARM_VCAN_RUNTIME=EXPECTED_ERROR" in completed.stdout
        and "OPENARM_VCAN_DEACTIVATION=SUCCESS" in completed.stdout
        and "OPENARM_VCAN_RUNTIME=SUCCESS" not in completed.stdout
        and fake_all_disabled
        and runtime_mit_commands > 0
        and expected_logs_observed
        and production_shutdown_observed
    )
    bootstrap_rejected = (
        completed.returncode == 0
        and "OPENARM_VCAN_ACTIVATION=SUCCESS" in completed.stdout
        and "OPENARM_VCAN_BOOTSTRAP=EXPECTED_ERROR cycle=0 runtime_writes=0"
        in completed.stdout
        and "OPENARM_VCAN_DEACTIVATION=SUCCESS" in completed.stdout
        and "OPENARM_VCAN_RUNTIME=EXPECTED_ERROR" not in completed.stdout
        and "OPENARM_VCAN_RUNTIME=SUCCESS" not in completed.stdout
        and fake_all_disabled
        and bootstrap_refresh_commands == len(MOTOR_IDS)
        and runtime_mit_commands == 0
        and expected_logs_observed
        and production_shutdown_observed
    )
    if expected_outcome == "success" and not succeeded:
        raise RegressionFailure(
            f"actual OpenArmHW activation case '{name}' failed "
            f"(rc={completed.returncode}):\n{completed.stdout}"
        )
    if expected_outcome == "activation_rejected" and not rejected:
        raise RegressionFailure(
            f"actual OpenArmHW activation case '{name}' was not rejected "
            f"(rc={completed.returncode}):\n{completed.stdout}"
        )
    if expected_outcome == "runtime_rejected" and not runtime_rejected:
        raise RegressionFailure(
            f"actual OpenArmHW runtime case '{name}' did not fail closed "
            f"(rc={completed.returncode}):\n{completed.stdout}"
        )
    if expected_outcome == "bootstrap_rejected" and not bootstrap_rejected:
        raise RegressionFailure(
            f"actual OpenArmHW bootstrap case '{name}' did not fail closed "
            f"before runtime MIT (bootstrap_refresh_commands="
            f"{bootstrap_refresh_commands}, runtime_mit_commands="
            f"{runtime_mit_commands}, rc={completed.returncode}):\n"
            f"{completed.stdout}"
        )
    if expected_outcome not in {
        "success",
        "activation_rejected",
        "bootstrap_rejected",
        "runtime_rejected",
    }:
        raise RegressionFailure(f"invalid expected outcome: {expected_outcome}")
    print(
        f"PASS patched_openarmhw_{name} commands_seen={fake.command_count} "
        f"gripper_mode_ack_commands={gripper_mode_ack_commands} "
        f"bootstrap_refresh_commands={bootstrap_refresh_commands} "
        f"runtime_mit_commands={runtime_mit_commands}",
        flush=True,
    )


def _patched_openarmhw_regressions() -> None:
    _assert_velocity_boundary_samples()
    with tempfile.TemporaryDirectory(prefix="openarm-vcan-regression-") as temp_dir:
        binary, environment = _build_activation_probe(Path(temp_dir))
        _run_activation_case(
            binary,
            environment,
            name="eight_motor_100hz_runtime_success",
            expected_outcome="success",
        )
        _run_activation_case(
            binary,
            environment,
            name="post_activate_150ms_bootstrap_success",
            expected_outcome="success",
            post_activate_delay_ms=150,
        )
        _run_activation_case(
            binary,
            environment,
            name="runtime_arm_velocity_last_safe_all_axes_success",
            expected_outcome="success",
            runtime_velocity_raw_by_batch={
                1: ARM_VELOCITY_LAST_SAFE_RAW,
            },
        )
        for motor_id in range(1, GRIPPER_MOTOR_ID):
            velocity_limit = ARM_VELOCITY_LIMIT_RAD_S[motor_id]
            _run_activation_case(
                binary,
                environment,
                name=(
                    f"runtime_arm_id{motor_id}_first_over_velocity_"
                    "single_sample_rejected"
                ),
                expected_outcome="runtime_rejected",
                runtime_velocity_raw_by_batch={
                    1: {
                        motor_id: ARM_VELOCITY_FIRST_OVER_RAW[motor_id],
                    }
                },
                runtime_error_cycle_range=(1, 1),
                expected_logs=(
                    "runtime side=left can=vcan42 arm "
                    f"axis={motor_id} can_id=0x{0x10 + motor_id:02x}",
                    "velocity_limit_ok=no "
                    f"velocity_limit={velocity_limit:.6f} "
                    "velocity_gate_ok=no velocity_pending=no",
                    "OPENARM_VCAN_RCLCPP_SHUTDOWN=YES "
                    "phase=runtime_read cycle=1",
                ),
            )
        _run_activation_case(
            binary,
            environment,
            name="missing_id_rejected",
            expected_outcome="activation_rejected",
            missing_id=7,
        )
        _run_activation_case(
            binary,
            environment,
            name="raw_status_2_rejected",
            expected_outcome="activation_rejected",
            forced_raw_status=2,
        )
        _run_activation_case(
            binary,
            environment,
            name="activation_j4_peak_last_safe_success",
            expected_outcome="success",
            activation_torque_raw={4: J4_TORQUE_LAST_SAFE_RAW},
        )
        _run_activation_case(
            binary,
            environment,
            name="activation_j4_first_over_peak_strictly_rejected",
            expected_outcome="activation_rejected",
            activation_torque_raw={4: J4_TORQUE_FIRST_OVER_RAW},
            expected_log="initial current-position hold arm axis=4",
        )
        _run_activation_case(
            binary,
            environment,
            name="activation_id8_last_safe_velocity_success",
            expected_outcome="success",
            activation_velocity_raw={
                GRIPPER_MOTOR_ID: GRIPPER_VELOCITY_LAST_SAFE_RAW
            },
        )
        _run_activation_case(
            binary,
            environment,
            name="activation_id8_first_over_velocity_strictly_rejected",
            expected_outcome="activation_rejected",
            activation_velocity_raw={
                GRIPPER_MOTOR_ID: GRIPPER_VELOCITY_FIRST_OVER_POSITIVE_RAW
            },
            expected_logs=(
                "initial current-position hold gripper axis=1",
                "velocity_ok=no velocity_limit=18.849556",
            ),
        )
        _run_activation_case(
            binary,
            environment,
            name="bootstrap_missing_id_rejected",
            expected_outcome="bootstrap_rejected",
            bootstrap_missing_id=7,
        )
        _run_activation_case(
            binary,
            environment,
            name="bootstrap_raw_status_2_rejected",
            expected_outcome="bootstrap_rejected",
            bootstrap_fault_id=7,
            bootstrap_fault_status=2,
        )
        _run_activation_case(
            binary,
            environment,
            name="runtime_missing_id_watchdog_rejected",
            expected_outcome="runtime_rejected",
            runtime_missing_id=7,
        )
        _run_activation_case(
            binary,
            environment,
            name="runtime_gripper_id8_missing_20ms_watchdog_rejected",
            expected_outcome="runtime_rejected",
            runtime_missing_id=GRIPPER_MOTOR_ID,
        )
        _run_activation_case(
            binary,
            environment,
            name="runtime_raw_status_2_rejected",
            expected_outcome="runtime_rejected",
            runtime_fault_id=7,
            runtime_fault_status=2,
        )
        _run_activation_case(
            binary,
            environment,
            name="runtime_j4_peak_last_safe_success",
            expected_outcome="success",
            runtime_torque_raw_by_batch={
                batch: {4: J4_TORQUE_LAST_SAFE_RAW}
                for batch in range(1, 31)
            },
        )
        _run_activation_case(
            binary,
            environment,
            name="runtime_j4_first_over_peak_rejected_immediately",
            expected_outcome="runtime_rejected",
            runtime_torque_raw_by_batch={
                1: {4: J4_TORQUE_FIRST_OVER_RAW}
            },
            runtime_error_cycle_range=(1, 1),
            expected_log="Runtime torque trip reason=instant_hard side=left can=vcan42 axis=4",
        )
        _run_activation_case(
            binary,
            environment,
            name="runtime_id8_peak_last_safe_success",
            expected_outcome="success",
            runtime_torque_raw_by_batch={
                batch: {GRIPPER_MOTOR_ID: GRIPPER_TORQUE_LAST_SAFE_RAW}
                for batch in range(1, 31)
            },
        )
        _run_activation_case(
            binary,
            environment,
            name="runtime_id8_first_over_peak_remains_single_sample_strict",
            expected_outcome="runtime_rejected",
            runtime_torque_raw_by_batch={
                1: {GRIPPER_MOTOR_ID: GRIPPER_TORQUE_FIRST_OVER_RAW}
            },
            runtime_error_cycle_range=(1, 1),
            expected_log="runtime side=left can=vcan42 gripper axis=1",
        )
        _run_activation_case(
            binary,
            environment,
            name="runtime_id8_last_safe_velocity_success",
            expected_outcome="success",
            runtime_velocity_raw_by_batch={
                batch: {
                    GRIPPER_MOTOR_ID: GRIPPER_VELOCITY_LAST_SAFE_RAW
                }
                for batch in range(1, 31)
            },
        )
        for direction, first_over_raw in (
            ("positive", GRIPPER_VELOCITY_FIRST_OVER_POSITIVE_RAW),
            ("negative", GRIPPER_VELOCITY_FIRST_OVER_NEGATIVE_RAW),
        ):
            _run_activation_case(
                binary,
                environment,
                name=(
                    f"runtime_id8_{direction}_first_over_velocity_"
                    "single_sample_rejected"
                ),
                expected_outcome="runtime_rejected",
                runtime_velocity_raw_by_batch={
                    1: {GRIPPER_MOTOR_ID: first_over_raw}
                },
                runtime_error_cycle_range=(1, 1),
                expected_logs=(
                    "runtime side=left can=vcan42 gripper axis=1 can_id=0x18",
                    "velocity_limit_ok=no velocity_limit=18.849556 "
                    "velocity_gate_ok=no velocity_pending=no",
                    "OPENARM_VCAN_RCLCPP_SHUTDOWN=YES "
                    "phase=runtime_read cycle=1",
                ),
            )
        _run_activation_case(
            binary,
            environment,
            name="gripper_9ms_11ms_continuous_reversal_success",
            expected_outcome="success",
            gripper_gate_mode="jitter_reversal",
            expected_log=(
                "OPENARM_VCAN_GRIPPER_GATE=SUCCESS mode=jitter_reversal "
                "cycles=30 preload=1 reversals=12 hold=17 "
                "periods_ms=20,9,11,10 "
                "gate_m_s=+0.5000,-0.5000 preload_step_m=0.005000000 "
                "reversal_step_m=0.004500000"
            ),
        )
        _run_activation_case(
            binary,
            environment,
            name="gripper_time_credit_exhaustion_rejected_without_tracking_or_hard_violation",
            expected_outcome="runtime_rejected",
            gripper_gate_mode="time_credit_exhaustion_reject",
            runtime_error_cycle_range=(2, 2),
            expected_logs=(
                "Rejected gripper open command exceeding the 0.5000 m/s "
                "hardware rate gate",
                "requested_step=0.005000000 requested_time=0.010000000 "
                "time_credit=0.002000000",
                "OPENARM_VCAN_RUNTIME=EXPECTED_ERROR cycle=2 "
                "runtime_writes=2 source=gripper_time_credit_exhaustion",
            ),
        )
        _run_activation_case(
            binary,
            environment,
            name="gripper_tracking_5p000mm_safe_then_5p041mm_rejected",
            expected_outcome="runtime_rejected",
            gripper_gate_mode="tracking_first_over_reject",
            runtime_error_cycle_range=(1, 1),
            expected_logs=(
                "Rejected gripper command with excessive tracking error",
                "OPENARM_VCAN_RUNTIME=EXPECTED_ERROR cycle=1 "
                "runtime_writes=1 source=gripper_tracking_first_over",
            ),
        )


def main(arguments: Iterable[str]) -> int:
    if list(arguments):
        raise RegressionFailure(
            "this safety regression accepts no arguments and cannot select a CAN interface"
        )
    _assert_isolated_namespace_before_vcan()
    try:
        _create_vcan()
        _raw_feedback_regressions()
        _patched_openarmhw_regressions()
    finally:
        _delete_vcan()
    print("PASS openarm_vcan_feedback_regression complete", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except (RegressionFailure, subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
        print(f"FAIL openarm_vcan_feedback_regression: {error}", file=sys.stderr)
        raise SystemExit(1)
