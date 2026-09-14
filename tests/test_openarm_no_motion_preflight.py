from __future__ import annotations

import importlib.util
import struct
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "openarm_can_no_motion_preflight.py"
SPEC = importlib.util.spec_from_file_location("openarm_no_motion", SCRIPT)
assert SPEC and SPEC.loader
safety = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(safety)


def feedback_frame(motor_id: int, status: int = 0) -> bytes:
    payload = bytes(((status << 4) | motor_id, 0, 0, 0, 0, 0, 25, 25))
    return safety.FRAME.pack(0x10 + motor_id, 8, safety.CANFD_BRS, 0, 0, payload)


class FakeCanSocket:
    def __init__(self) -> None:
        self.queue: list[bytes] = []
        self.sent: list[bytes] = []
        self.bound = None
        self.closed = False

    def setsockopt(self, *_args) -> None:
        pass

    def bind(self, address) -> None:
        self.bound = address

    def setblocking(self, _value) -> None:
        pass

    def close(self) -> None:
        self.closed = True

    def recv(self, _size: int) -> bytes:
        if not self.queue:
            raise BlockingIOError
        return self.queue.pop(0)

    def send(self, frame: bytes) -> int:
        self.sent.append(frame)
        can_id, payload_length = struct.unpack_from("=IB", frame)
        payload = frame[8 : 8 + payload_length]
        motor_id = can_id if can_id in safety.MOTOR_SEND_IDS else payload[0]
        self.queue.append(feedback_frame(motor_id))
        return len(frame)


class FaultInjectingCanSocket(FakeCanSocket):
    def __init__(
        self,
        *,
        exception_ids: tuple[int, ...] = (),
        short_write_ids: tuple[int, ...] = (),
    ) -> None:
        super().__init__()
        self.exception_ids = frozenset(exception_ids)
        self.short_write_ids = frozenset(short_write_ids)
        self.attempted_ids: list[int] = []
        self.injected_exceptions = {
            motor_id: OSError(f"injected write failure ID{motor_id}")
            for motor_id in self.exception_ids
        }

    def send(self, frame: bytes) -> int:
        can_id, payload_length = struct.unpack_from("=IB", frame)
        payload = frame[8 : 8 + payload_length]
        motor_id = can_id if can_id in safety.MOTOR_SEND_IDS else payload[0]
        self.attempted_ids.append(motor_id)
        if motor_id in self.exception_ids:
            raise self.injected_exceptions[motor_id]
        if motor_id in self.short_write_ids:
            return len(frame) - 1
        return len(frame)


def test_only_id1_through_id8_can_be_encoded() -> None:
    for motor_id in safety.MOTOR_SEND_IDS:
        assert struct.unpack_from("=I", safety.disable_frame(motor_id))[0] == motor_id
        refresh = safety.refresh_frame(motor_id)
        assert struct.unpack_from("=I", refresh)[0] == 0x7FF
        assert refresh[8] == motor_id
    for forbidden in (0, 9, 10):
        with pytest.raises(ValueError, match="forbids"):
            safety.disable_frame(forbidden)
        with pytest.raises(ValueError, match="forbids"):
            safety.refresh_frame(forbidden)


def test_disable_and_two_refresh_frames_target_every_motor_id1_through_id8(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake_socket = FakeCanSocket()
    monkeypatch.setattr(
        safety.select,
        "select",
        lambda readable, _writable, _errors, _timeout: (
            readable if fake_socket.queue else [],
            [],
            [],
        ),
    )
    endpoint = safety.WholeRobotEndpoint(
        "can0",
        socket_factory=lambda *_args: fake_socket,
    )

    endpoint.disable_once()
    endpoint.refresh_once()
    endpoint.refresh_once()

    assert fake_socket.bound == ("can0",)
    expected_ids = list(range(1, 9))
    assert len(fake_socket.sent) == 24
    for index, frame in enumerate(fake_socket.sent):
        can_id, payload_length, flags, _, _, payload = safety.FRAME.unpack(frame)
        assert payload_length == 8
        assert flags == safety.CANFD_BRS
        if index < 8:
            assert can_id == expected_ids[index]
            assert payload[:8] == safety.DISABLE_PAYLOAD
        else:
            target_id = expected_ids[(index - 8) % 8]
            assert can_id == 0x7FF
            assert payload[:8] == bytes((target_id, 0, 0xCC, 0, 0, 0, 0, 0))


def test_send_errors_are_aggregated_after_all_motor_ids_are_attempted() -> None:
    fake_socket = FaultInjectingCanSocket(
        exception_ids=(1, 3),
        short_write_ids=(2,),
    )
    endpoint = safety.WholeRobotEndpoint(
        "can0",
        socket_factory=lambda *_args: fake_socket,
    )

    with pytest.raises(RuntimeError) as error:
        endpoint._send_frames(
            tuple(safety.disable_frame(motor_id) for motor_id in safety.MOTOR_SEND_IDS)
        )

    assert fake_socket.attempted_ids == list(safety.MOTOR_SEND_IDS)
    assert "after attempting motor IDs1-8" in str(error.value)
    assert "motor ID1 raised OSError: injected write failure ID1" in str(error.value)
    assert f"motor ID2 short write {safety.FRAME.size - 1}/{safety.FRAME.size} bytes" in str(
        error.value
    )
    assert "motor ID3 raised OSError: injected write failure ID3" in str(error.value)
    assert error.value.__cause__ is fake_socket.injected_exceptions[1]


def test_short_writes_are_aggregated_and_propagated_after_all_motor_ids() -> None:
    fake_socket = FaultInjectingCanSocket(short_write_ids=(1, 4))
    endpoint = safety.WholeRobotEndpoint(
        "can1",
        socket_factory=lambda *_args: fake_socket,
    )

    with pytest.raises(RuntimeError) as error:
        endpoint._send_frames(
            tuple(safety.refresh_frame(motor_id) for motor_id in safety.MOTOR_SEND_IDS)
        )

    assert fake_socket.attempted_ids == list(safety.MOTOR_SEND_IDS)
    assert "after attempting motor IDs1-8" in str(error.value)
    assert f"motor ID1 short write {safety.FRAME.size - 1}/{safety.FRAME.size} bytes" in str(
        error.value
    )
    assert f"motor ID4 short write {safety.FRAME.size - 1}/{safety.FRAME.size} bytes" in str(
        error.value
    )
    assert error.value.__cause__ is None


def test_enabled_feedback_is_rejected() -> None:
    with pytest.raises(RuntimeError, match="status=1"):
        safety.decode_disabled_feedback(feedback_frame(1, status=1))


def test_missing_id8_feedback_is_rejected(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class MissingId8FeedbackSocket(FakeCanSocket):
        def send(self, frame: bytes) -> int:
            can_id, payload_length = struct.unpack_from("=IB", frame)
            payload = frame[8 : 8 + payload_length]
            motor_id = can_id if can_id in safety.MOTOR_SEND_IDS else payload[0]
            self.sent.append(frame)
            if motor_id != 8:
                self.queue.append(feedback_frame(motor_id))
            return len(frame)

    fake_socket = MissingId8FeedbackSocket()
    monkeypatch.setattr(
        safety.select,
        "select",
        lambda readable, _writable, _errors, _timeout: (
            readable if fake_socket.queue else [],
            [],
            [],
        ),
    )
    endpoint = safety.WholeRobotEndpoint(
        "can0",
        socket_factory=lambda *_args: fake_socket,
    )

    with pytest.raises(RuntimeError, match=r"missing fresh disabled motor feedback: 0x18"):
        endpoint.disable_once()


def test_id8_error_status_is_rejected(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class Id8ErrorStatusSocket(FakeCanSocket):
        def send(self, frame: bytes) -> int:
            can_id, payload_length = struct.unpack_from("=IB", frame)
            payload = frame[8 : 8 + payload_length]
            motor_id = can_id if can_id in safety.MOTOR_SEND_IDS else payload[0]
            self.sent.append(frame)
            self.queue.append(feedback_frame(motor_id, status=2 if motor_id == 8 else 0))
            return len(frame)

    fake_socket = Id8ErrorStatusSocket()
    monkeypatch.setattr(
        safety.select,
        "select",
        lambda readable, _writable, _errors, _timeout: (
            readable if fake_socket.queue else [],
            [],
            [],
        ),
    )
    endpoint = safety.WholeRobotEndpoint(
        "can1",
        socket_factory=lambda *_args: fake_socket,
    )

    with pytest.raises(RuntimeError, match=r"reply 0x18 reported status=2"):
        endpoint.disable_once()


def test_socket_smoke_binds_and_closes_without_transmission(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[tuple[str, str]] = []

    class SmokeEndpoint:
        def __init__(self, interface: str) -> None:
            events.append(("open", interface))

        def disable_confirmed(self) -> None:
            raise AssertionError("socket smoke must not transmit disable frames")

        def refresh_once(self) -> None:
            raise AssertionError("socket smoke must not transmit refresh frames")

        def close(self) -> None:
            events.append(("close", ""))

    monkeypatch.setattr(safety, "WholeRobotEndpoint", SmokeEndpoint)
    safety.run("socket-smoke")

    assert events[:2] == [("open", "can0"), ("open", "can1")]
    assert events[2:] == [("close", ""), ("close", "")]


def test_disable_failure_on_one_side_does_not_skip_the_other_side(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    attempts: list[str] = []

    class Endpoint:
        def __init__(self, interface: str) -> None:
            self.interface = interface

        def disable_confirmed(self) -> None:
            attempts.append(self.interface)
            if self.interface == "can0":
                raise RuntimeError("left failure")

        def close(self) -> None:
            pass

    monkeypatch.setattr(safety, "WholeRobotEndpoint", Endpoint)
    with pytest.raises(RuntimeError, match="can0: left failure"):
        safety.run("disable-only")
    assert attempts == ["can0", "can1"]


@pytest.mark.parametrize("failed_interface", safety.INTERFACES)
def test_endpoint_open_failure_does_not_skip_disable_on_available_arm(
    monkeypatch: pytest.MonkeyPatch,
    failed_interface: str,
) -> None:
    events: list[tuple[str, str]] = []

    class Endpoint:
        def __init__(self, interface: str) -> None:
            self.interface = interface
            events.append(("open", interface))
            if interface == failed_interface:
                raise OSError("bind failed")

        def disable_confirmed(self) -> None:
            events.append(("disable", self.interface))

        def close(self) -> None:
            events.append(("close", self.interface))

    monkeypatch.setattr(safety, "WholeRobotEndpoint", Endpoint)
    with pytest.raises(RuntimeError, match=f"{failed_interface}: open/bind failed"):
        safety.run("disable-only")

    available_interface = next(
        interface for interface in safety.INTERFACES if interface != failed_interface
    )
    assert ("disable", available_interface) in events
    assert ("close", available_interface) in events
