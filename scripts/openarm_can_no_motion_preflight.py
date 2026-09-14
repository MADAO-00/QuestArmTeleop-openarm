#!/usr/bin/env python3
"""Whole-robot OpenArm CAN preflight: disable and refresh motor IDs 1-8."""

from __future__ import annotations

import argparse
import select
import socket
import struct
import time
from collections.abc import Callable


INTERFACES = ("can0", "can1")
MOTOR_SEND_IDS = tuple(range(0x01, 0x09))
MOTOR_RECV_IDS = frozenset(range(0x11, 0x19))
DISABLE_PAYLOAD = b"\xff" * 7 + b"\xfd"
CANFD_BRS = 0x01
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_ID_MASK = 0x1FFFFFFF
FRAME = struct.Struct("=IBBBB64s")


def encode_frame(can_id: int, payload: bytes) -> bytes:
    if not 0 <= can_id <= 0x7FF:
        raise ValueError(f"only standard CAN IDs are permitted, got 0x{can_id:x}")
    if len(payload) != 8:
        raise ValueError("OpenArm safety frames must contain exactly 8 bytes")
    return FRAME.pack(can_id, len(payload), CANFD_BRS, 0, 0, payload)


def disable_frame(motor_id: int) -> bytes:
    if motor_id not in MOTOR_SEND_IDS:
        raise ValueError(f"whole-robot disable forbids motor ID{motor_id}")
    return encode_frame(motor_id, DISABLE_PAYLOAD)


def refresh_frame(motor_id: int) -> bytes:
    if motor_id not in MOTOR_SEND_IDS:
        raise ValueError(f"whole-robot refresh forbids motor ID{motor_id}")
    payload = bytes((motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0))
    return encode_frame(0x7FF, payload)


def decode_disabled_feedback(raw: bytes) -> tuple[int, bytes] | None:
    if len(raw) < 16:
        return None
    raw_can_id, payload_length = struct.unpack_from("=IB", raw)
    if raw_can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG):
        return None
    can_id = raw_can_id & CAN_ID_MASK
    if can_id not in MOTOR_RECV_IDS or payload_length < 8 or len(raw) < 16:
        return None
    payload = raw[8:16]
    motor_id = can_id - 0x10
    if payload[0] & 0x0F != motor_id:
        return None
    status = payload[0] >> 4
    if status != 0:
        raise RuntimeError(
            f"motor reply 0x{can_id:02x} reported status={status}; expected disabled status=0"
        )
    return can_id, payload


class WholeRobotEndpoint:
    """A deliberately tiny endpoint that cannot encode enable/control frames."""

    def __init__(
        self,
        interface: str,
        *,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        if interface not in INTERFACES:
            raise ValueError(f"unexpected OpenArm interface: {interface}")
        self.interface = interface
        self.socket = socket_factory(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            self.socket.bind((interface,))
            self.socket.setblocking(False)
        except Exception:
            self.socket.close()
            raise

    def close(self) -> None:
        self.socket.close()

    def drain(self) -> None:
        while True:
            try:
                self.socket.recv(FRAME.size)
            except BlockingIOError:
                return

    def _send_frames(self, frames: tuple[bytes, ...]) -> None:
        write_errors: list[str] = []
        first_exception: Exception | None = None
        for motor_id, frame in zip(MOTOR_SEND_IDS, frames, strict=True):
            try:
                sent = self.socket.send(frame)
            except Exception as exc:
                if first_exception is None:
                    first_exception = exc
                write_errors.append(
                    f"motor ID{motor_id} raised {type(exc).__name__}: {exc}"
                )
                continue
            if sent != FRAME.size:
                write_errors.append(
                    f"motor ID{motor_id} short write {sent}/{FRAME.size} bytes"
                )
        if write_errors:
            error = RuntimeError(
                f"{self.interface} CAN-FD safety write failed after attempting motor IDs1-8: "
                + "; ".join(write_errors)
            )
            if first_exception is not None:
                raise error from first_exception
            raise error

    def require_disabled_feedback(self, timeout_s: float = 0.25) -> None:
        pending = set(MOTOR_RECV_IDS)
        deadline = time.monotonic() + timeout_s
        while pending:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            readable, _, _ = select.select([self.socket], [], [], remaining)
            if not readable:
                break
            try:
                raw = self.socket.recv(FRAME.size)
            except BlockingIOError:
                continue
            decoded = decode_disabled_feedback(raw)
            if decoded is not None:
                pending.discard(decoded[0])
        if pending:
            missing = ",".join(f"0x{can_id:02x}" for can_id in sorted(pending))
            raise RuntimeError(
                f"{self.interface} missing fresh disabled motor feedback: {missing}"
            )

    def disable_once(self) -> None:
        self.drain()
        self._send_frames(tuple(disable_frame(motor_id) for motor_id in MOTOR_SEND_IDS))
        self.require_disabled_feedback()

    def refresh_once(self) -> None:
        self.drain()
        self._send_frames(tuple(refresh_frame(motor_id) for motor_id in MOTOR_SEND_IDS))
        self.require_disabled_feedback()

    def disable_confirmed(self, attempts: int = 3) -> None:
        last_error: Exception | None = None
        for attempt in range(1, attempts + 1):
            try:
                self.disable_once()
                return
            except Exception as exc:  # cleanup must make all bounded attempts
                last_error = exc
                if attempt < attempts:
                    time.sleep(0.02)
        raise RuntimeError(
            f"{self.interface} whole-robot disable was not confirmed after {attempts} attempts: "
            f"{last_error}"
        )


def run(mode: str) -> None:
    endpoints: list[WholeRobotEndpoint] = []
    try:
        open_errors: list[str] = []
        for interface in INTERFACES:
            try:
                endpoints.append(WholeRobotEndpoint(interface))
            except Exception as exc:
                # A broken second interface must never prevent a successfully
                # opened first arm from receiving the bounded shutdown frames.
                open_errors.append(f"{interface}: open/bind failed: {exc}")

        if mode == "socket-smoke":
            if open_errors:
                raise RuntimeError(
                    "SocketCAN endpoint check failed: " + "; ".join(open_errors)
                )
            return

        disable_errors = list(open_errors)
        for endpoint in endpoints:
            try:
                endpoint.disable_confirmed()
            except Exception as exc:
                disable_errors.append(f"{endpoint.interface}: {exc}")
        if disable_errors:
            raise RuntimeError(
                "whole-robot disable failed: " + "; ".join(disable_errors)
            )

        if mode == "preflight":
            refresh_errors: list[str] = []
            for round_number in range(2):
                if round_number:
                    time.sleep(0.02)
                for endpoint in endpoints:
                    try:
                        endpoint.refresh_once()
                    except Exception as exc:
                        refresh_errors.append(
                            f"{endpoint.interface} round {round_number + 1}: {exc}"
                        )
            if refresh_errors:
                raise RuntimeError(
                    "whole-robot refresh failed: " + "; ".join(refresh_errors)
                )
    finally:
        for endpoint in endpoints:
            endpoint.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("preflight", "disable-only", "socket-smoke"),
        default="preflight",
    )
    args = parser.parse_args()
    try:
        run(args.mode)
    except (OSError, RuntimeError, ValueError) as exc:
        parser.error(str(exc))
    if args.mode == "preflight":
        print(
            "PASS OpenArm whole-robot no-motion preflight: can0/can1 ID1-8 "
            "disabled and two complete fresh feedback rounds received."
        )
    elif args.mode == "disable-only":
        print(
            "PASS OpenArm whole-robot shutdown: can0/can1 ID1-8 disabled with "
            "fresh status=0 confirmation."
        )
    else:
        print(
            "PASS OpenArm control-container SocketCAN smoke: PF_CAN/SOCK_RAW CAN-FD "
            "sockets bound to can0/can1; no frame transmitted."
        )


if __name__ == "__main__":
    main()
