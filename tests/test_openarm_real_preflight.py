from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "preflight_openarm_v1_ros_real.py"
SPEC = importlib.util.spec_from_file_location("openarm_real_preflight", SCRIPT)
assert SPEC and SPEC.loader
preflight = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preflight)


def valid_link(interface: str) -> dict:
    return {
        "ifname": interface,
        "flags": ["UP", "NOARP", "ECHO"],
        "mtu": 72,
        "link_type": "can",
        "linkinfo": {
            "info_kind": "can",
            "info_data": {
                "state": "ERROR-ACTIVE",
                "bittiming": {"bitrate": 1_000_000},
                "data_bittiming": {"bitrate": 5_000_000},
                "ctrlmode": ["FD"],
                "berr_counter": {"tx": 0, "rx": 0},
            },
        },
    }


def test_fixed_can_identity_contract() -> None:
    assert preflight.COMMISSIONED_CAN_IDENTITIES == {
        "can0": {"side": "left", "bus_info": "3-1:1.0", "dev_id": 0},
        "can1": {"side": "right", "bus_info": "3-1:1.0", "dev_id": 1},
    }
    assert preflight.EXPECTED_DRIVER == "peak_usb"
    assert preflight.EXPECTED_ARBITRATION_BITRATE == 1_000_000
    assert preflight.EXPECTED_DATA_BITRATE == 5_000_000


def test_ip_and_ethtool_are_read_only_and_parsed(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[list[str]] = []

    def fake_run(command, **_kwargs):
        calls.append(command)
        if command[0] == "ip":
            return SimpleNamespace(returncode=0, stdout=json.dumps([valid_link("can0")]))
        if command[0] == "ethtool":
            return SimpleNamespace(
                returncode=0,
                stdout="driver: peak_usb\nbus-info: 3-1:1.0\n",
            )
        raise AssertionError(command)

    monkeypatch.setattr(preflight.subprocess, "run", fake_run)

    assert preflight.read_link("can0")["ifname"] == "can0"
    assert preflight.read_driver_info("can0") == {
        "driver": "peak_usb",
        "bus-info": "3-1:1.0",
    }
    assert calls == [
        ["ip", "-details", "-json", "link", "show", "dev", "can0"],
        ["ethtool", "--driver", "can0"],
    ]


@pytest.mark.parametrize(
    ("path", "value", "message"),
    [
        (("mtu",), 16, "MTU"),
        (("flags",), ["NOARP"], "not UP"),
        (("linkinfo", "info_data", "state"), "ERROR-PASSIVE", "ERROR-ACTIVE"),
        (("linkinfo", "info_data", "bittiming", "bitrate"), 500_000, "1000000"),
        (("linkinfo", "info_data", "data_bittiming", "bitrate"), 2_000_000, "5000000"),
        (("linkinfo", "info_data", "ctrlmode"), [], "FD mode"),
    ],
)
def test_link_preflight_fails_closed(path, value, message) -> None:
    link = valid_link("can0")
    target = link
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value

    with pytest.raises(ValueError, match=message):
        preflight.validate_can_link(link, "can0")


def test_identity_rejects_swapped_peak_channel() -> None:
    with pytest.raises(ValueError, match="physical identity mismatch"):
        preflight.validate_can_identity(
            "can0",
            {"driver": "peak_usb", "bus-info": "3-1:1.0"},
            1,
        )


def _empty_receiver_list(name: str) -> str:
    return (
        f"receive list '{name}':\n"
        "  (any: no entry)\n"
        "  (can0: no entry)\n"
        "  (can1: no entry)\n"
    )


def test_complete_receiver_audit_requires_all_six_empty_lists(tmp_path: Path) -> None:
    for filename, list_name in preflight.CAN_RECEIVER_LISTS.items():
        (tmp_path / filename).write_text(_empty_receiver_list(list_name), encoding="utf-8")

    preflight.validate_no_can_receivers(tmp_path, ("can0", "can1"))


def test_receiver_audit_rejects_existing_can0_owner(tmp_path: Path) -> None:
    for filename, list_name in preflight.CAN_RECEIVER_LISTS.items():
        text = _empty_receiver_list(list_name)
        if filename == "rcvlist_all":
            text = (
                "receive list 'rx_all':\n"
                "device can_id can_mask function userdata matches ident\n"
                "can0 000 00000000 0000000000000001 0000000000000002 0 python\n"
                "(any: no entry)\n"
                "(can1: no entry)\n"
            )
        (tmp_path / filename).write_text(text, encoding="utf-8")

    with pytest.raises(ValueError, match="existing CAN receiver registered on can0"):
        preflight.validate_no_can_receivers(tmp_path, ("can0", "can1"))


@pytest.mark.parametrize(("value", "expected"), [("0x0\n", 0), ("0x1", 1), ("1", 1)])
def test_dev_id_parser(value: str, expected: int) -> None:
    assert preflight.parse_can_dev_id(value, "can0") == expected

