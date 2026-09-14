#!/usr/bin/env python3
"""Fail-closed, read-only host preflight for the OpenArm v1 ROS real path."""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import socket
import subprocess


COMMISSIONED_CAN_IDENTITIES = {
    "can0": {"side": "left", "bus_info": "3-1:1.0", "dev_id": 0},
    "can1": {"side": "right", "bus_info": "3-1:1.0", "dev_id": 1},
}
EXPECTED_DRIVER = "peak_usb"
EXPECTED_ARBITRATION_BITRATE = 1_000_000
EXPECTED_DATA_BITRATE = 5_000_000
SYS_CLASS_NET = pathlib.Path("/sys/class/net")
PROC_CAN_ROOT = pathlib.Path("/proc/net/can")
CAN_RECEIVER_LISTS = {
    "rcvlist_all": "rx_all",
    "rcvlist_eff": "rx_eff",
    "rcvlist_err": "rx_err",
    "rcvlist_fil": "rx_fil",
    "rcvlist_inv": "rx_inv",
    "rcvlist_sff": "rx_sff",
}
_RECEIVER_COLUMNS = (
    "device",
    "can_id",
    "can_mask",
    "function",
    "userdata",
    "matches",
    "ident",
)
_RECEIVER_TITLE = re.compile(r"receive list '(?P<name>rx_[a-z]+)':")
_NO_RECEIVER = re.compile(r"\((?P<device>[^\s:()]+): no entry\)")
_DEVICE_NAME = re.compile(r"[^\s:()]+")
_CAN_ID = re.compile(r"(?:[0-9A-Fa-f]{3}|[0-9A-Fa-f]{8})")
_CAN_MASK = re.compile(r"[0-9A-Fa-f]{8}")
_KERNEL_POINTER = re.compile(
    r"(?:[0-9A-Fa-f]{8}|[0-9A-Fa-f]{16}|\(____ptrval____\)|\(null\))"
)
_MATCH_COUNT = re.compile(r"[0-9]+")


def read_link(interface: str) -> dict:
    result = subprocess.run(
        ["ip", "-details", "-json", "link", "show", "dev", interface],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise ValueError(f"{interface} does not exist or cannot be inspected")
    links = json.loads(result.stdout)
    if len(links) != 1:
        raise ValueError(f"expected exactly one link named {interface}")
    return links[0]


def parse_driver_info(output: str, interface: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for line_number, raw_line in enumerate(output.splitlines(), start=1):
        line = raw_line.strip()
        if not line:
            continue
        key, separator, value = line.partition(":")
        key = key.strip()
        if not separator or not key or not re.fullmatch(r"[a-z][a-z0-9-]*", key):
            raise ValueError(
                f"malformed ethtool driver information for {interface} "
                f"at line {line_number}"
            )
        if key in fields:
            raise ValueError(f"duplicate ethtool driver field for {interface}: {key}")
        fields[key] = value.strip()
    if not fields:
        raise ValueError(f"empty ethtool driver information for {interface}")
    return fields


def read_driver_info(interface: str) -> dict[str, str]:
    result = subprocess.run(
        ["ethtool", "--driver", interface],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise ValueError(f"cannot read ethtool driver information for {interface}")
    return parse_driver_info(result.stdout, interface)


def parse_can_dev_id(value: str, interface: str) -> int:
    rendered = value.strip()
    if not re.fullmatch(r"(?:0[xX][0-9A-Fa-f]+|[0-9]+)", rendered):
        raise ValueError(
            f"malformed sysfs dev_id for {interface}: {rendered or 'empty'}"
        )
    return int(rendered, 16 if rendered.lower().startswith("0x") else 10)


def read_can_dev_id(
    interface: str,
    sys_class_net: pathlib.Path = SYS_CLASS_NET,
) -> int:
    path = sys_class_net / interface / "dev_id"
    try:
        value = path.read_text(encoding="ascii")
    except OSError as exc:
        raise ValueError(f"cannot read PEAK channel identity {path}: {exc}") from exc
    return parse_can_dev_id(value, interface)


def validate_can_identity(
    interface: str,
    driver_info: dict[str, str],
    actual_dev_id: int,
) -> None:
    expected = COMMISSIONED_CAN_IDENTITIES[interface]
    actual_driver = driver_info.get("driver")
    actual_bus_info = driver_info.get("bus-info")
    if actual_driver != EXPECTED_DRIVER:
        raise ValueError(
            f"{interface} driver must be {EXPECTED_DRIVER}, got "
            f"{actual_driver or 'missing'}"
        )
    if (
        actual_bus_info != expected["bus_info"]
        or actual_dev_id != expected["dev_id"]
    ):
        raise ValueError(
            f"{interface} physical identity mismatch: expected side={expected['side']}, "
            f"bus-info={expected['bus_info']}, dev_id=0x{expected['dev_id']:x}; "
            f"got bus-info={actual_bus_info or 'missing'}, dev_id=0x{actual_dev_id:x}"
        )


def _fd_enabled(ctrlmode: object) -> bool:
    if isinstance(ctrlmode, dict):
        return any(
            str(mode).lower() == "fd" and bool(enabled)
            for mode, enabled in ctrlmode.items()
        )
    if isinstance(ctrlmode, list):
        return any(str(mode).lower() == "fd" for mode in ctrlmode)
    return False


def validate_can_link(link: dict, interface: str) -> None:
    if link.get("ifname") != interface:
        raise ValueError(f"interface mismatch for {interface}")
    info = link.get("linkinfo", {})
    if link.get("link_type") != "can" and info.get("info_kind") != "can":
        raise ValueError(f"{interface} is not a CAN interface")
    if "UP" not in link.get("flags", []):
        raise ValueError(f"{interface} is not UP")
    if link.get("mtu") != 72:
        raise ValueError(f"{interface} MTU is not 72-byte CAN FD MTU")

    data = info.get("info_data", {})
    if data.get("state") != "ERROR-ACTIVE":
        raise ValueError(f"{interface} CAN state is not ERROR-ACTIVE")
    if data.get("bittiming", {}).get("bitrate") != EXPECTED_ARBITRATION_BITRATE:
        raise ValueError(
            f"{interface} arbitration bitrate is not "
            f"{EXPECTED_ARBITRATION_BITRATE}"
        )
    if data.get("data_bittiming", {}).get("bitrate") != EXPECTED_DATA_BITRATE:
        raise ValueError(f"{interface} data bitrate is not {EXPECTED_DATA_BITRATE}")
    if not _fd_enabled(data.get("ctrlmode", [])):
        raise ValueError(f"{interface} CAN FD mode is not enabled")

    counters = data.get("berr_counter")
    if not isinstance(counters, dict) or not all(
        isinstance(counters.get(name), int) and counters[name] >= 0
        for name in ("tx", "rx")
    ):
        raise ValueError(f"{interface} CAN error counters are unavailable")


def parse_can_receiver_list(
    text: str,
    expected_name: str,
    source: str,
) -> tuple[set[str], set[str]]:
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if not lines:
        raise ValueError(f"empty CAN receiver ownership list: {source}")

    title = _RECEIVER_TITLE.fullmatch(lines[0])
    if title is None or title.group("name") != expected_name:
        raise ValueError(f"unexpected CAN receiver ownership list title in {source}")

    receivers: set[str] = set()
    reported_devices: set[str] = set()
    header_active = False
    entries_after_header = 0
    section_device: str | None = None

    for line_number, line in enumerate(lines[1:], start=2):
        columns = tuple(line.split())
        if columns == _RECEIVER_COLUMNS:
            if header_active and entries_after_header == 0:
                raise ValueError(
                    f"CAN receiver header without entries in {source} "
                    f"at line {line_number}"
                )
            header_active = True
            entries_after_header = 0
            section_device = None
            continue

        no_receiver = _NO_RECEIVER.fullmatch(line)
        if no_receiver is not None:
            if header_active and entries_after_header == 0:
                raise ValueError(
                    f"CAN receiver header without entries in {source} "
                    f"at line {line_number}"
                )
            header_active = False
            entries_after_header = 0
            section_device = None
            device = no_receiver.group("device")
            if device in reported_devices:
                raise ValueError(
                    f"duplicate CAN receiver device section for {device} in {source}"
                )
            reported_devices.add(device)
            continue

        if not header_active or len(columns) != len(_RECEIVER_COLUMNS):
            raise ValueError(
                f"unrecognized CAN receiver ownership data in {source} "
                f"at line {line_number}"
            )

        device, can_id, can_mask, function, userdata, matches, ident = columns
        if not (
            _DEVICE_NAME.fullmatch(device)
            and _CAN_ID.fullmatch(can_id)
            and _CAN_MASK.fullmatch(can_mask)
            and _KERNEL_POINTER.fullmatch(function)
            and _KERNEL_POINTER.fullmatch(userdata)
            and _MATCH_COUNT.fullmatch(matches)
            and ident
        ):
            raise ValueError(
                f"malformed CAN receiver entry in {source} at line {line_number}"
            )
        if section_device is None:
            if device in reported_devices:
                raise ValueError(
                    f"duplicate CAN receiver device section for {device} in {source}"
                )
            section_device = device
            reported_devices.add(device)
        elif device != section_device:
            raise ValueError(
                f"mixed CAN devices under one receiver header in {source} "
                f"at line {line_number}"
            )
        entries_after_header += 1
        receivers.add(device)

    if header_active and entries_after_header == 0:
        raise ValueError(f"CAN receiver header without entries at end of {source}")
    return receivers, reported_devices


def materialize_can_receiver_lists(proc_can_root: pathlib.Path) -> None:
    if proc_can_root.is_dir():
        return
    if proc_can_root != PROC_CAN_ROOT:
        raise ValueError(f"CAN receiver ownership directory is missing: {proc_can_root}")

    # Loading CAN_RAW by opening and immediately closing an unbound socket does
    # not bind an interface or transmit a frame.  It only makes the kernel's
    # receiver ownership lists available for the fail-closed checks below.
    try:
        probe = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        probe.close()
    except OSError as exc:
        raise ValueError(f"cannot initialize read-only CAN receiver audit: {exc}") from exc
    if not proc_can_root.is_dir():
        raise ValueError(f"CAN receiver ownership directory is missing: {proc_can_root}")


def validate_no_can_receivers(
    proc_can_root: pathlib.Path,
    interfaces: tuple[str, ...],
) -> None:
    required_devices = {"any", *interfaces}
    for filename, expected_name in CAN_RECEIVER_LISTS.items():
        path = proc_can_root / filename
        try:
            text = path.read_text(encoding="utf-8")
        except OSError as exc:
            raise ValueError(f"cannot read CAN receiver ownership list {path}: {exc}") from exc

        receivers, reported_devices = parse_can_receiver_list(
            text,
            expected_name,
            str(path),
        )
        missing_devices = required_devices - reported_devices
        if missing_devices:
            missing = ", ".join(sorted(missing_devices))
            raise ValueError(f"CAN receiver ownership list {path} does not cover: {missing}")
        conflicts = receivers & required_devices
        if conflicts:
            occupied = ", ".join(sorted(conflicts))
            raise ValueError(
                f"existing CAN receiver registered on {occupied} according to {path}"
            )


def run_preflight(
    identity_only: bool = False,
    *,
    audit_receivers: bool = True,
) -> None:
    interfaces = tuple(COMMISSIONED_CAN_IDENTITIES)
    for interface in interfaces:
        driver_info = read_driver_info(interface)
        validate_can_identity(
            interface,
            driver_info,
            read_can_dev_id(interface),
        )
        if not identity_only:
            validate_can_link(read_link(interface), interface)

    if audit_receivers:
        materialize_can_receiver_lists(PROC_CAN_ROOT)
        validate_no_can_receivers(PROC_CAN_ROOT, interfaces)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--identity-only",
        action="store_true",
        help=(
            "validate immutable PEAK identity and receiver ownership before "
            "an explicitly requested link reconfiguration"
        ),
    )
    parser.add_argument(
        "--skip-receiver-audit",
        action="store_true",
        help=(
            "host identity bootstrap only; permitted solely with --identity-only. "
            "The locked runner immediately follows it with a NET_RAW-only audit"
        ),
    )
    args = parser.parse_args()

    if args.skip_receiver_audit and not args.identity_only:
        parser.error("--skip-receiver-audit requires --identity-only")

    try:
        run_preflight(
            identity_only=args.identity_only,
            audit_receivers=not args.skip_receiver_audit,
        )
    except (ValueError, OSError, json.JSONDecodeError) as exc:
        parser.error(str(exc))

    if args.skip_receiver_audit:
        stage = "host identity bootstrap"
    else:
        stage = "identity/receiver preflight" if args.identity_only else "full preflight"
    print(
        f"PASS OpenArm v1 ROS real {stage}: "
        "left=can0@3-1:1.0/dev_id=0x0, "
        "right=can1@3-1:1.0/dev_id=0x1, driver=peak_usb, "
        + (
            "identity=matched"
            if args.skip_receiver_audit
            else "receivers=none"
            if args.identity_only
            else "CAN-FD=1M/5M, state=ERROR-ACTIVE, receivers=none"
        )
    )


if __name__ == "__main__":
    main()
