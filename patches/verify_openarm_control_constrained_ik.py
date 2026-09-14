#!/usr/bin/env python3
"""Verify the pinned OpenArm IK cannot fall back to an unbounded QP."""

from __future__ import annotations

import hashlib
import importlib.metadata
from pathlib import Path

import openarm_control.kinematics as kinematics


EXPECTED_VERSION = "0.2.0"
EXPECTED_SOURCE_SHA256 = (
    "aa0ca15e2b45486c5e62093acee3b26985743834cf57b10bf6aeae6547a77d1d"
)


def main() -> int:
    assert importlib.metadata.version("openarm-control") == EXPECTED_VERSION
    source_path = Path(kinematics.__file__).resolve()
    source_bytes = source_path.read_bytes()
    assert hashlib.sha256(source_bytes).hexdigest() == EXPECTED_SOURCE_SHA256
    source = source_bytes.decode("utf-8")
    assert "limits=[]" not in source
    assert "Warning: constrained IK solver failed. Skipping step." in source

    # Fault-inject the exact constrained Mink call. A safe build calls Mink
    # once, returns no candidate, resets target readiness, and never integrates
    # or attempts the former limits=[] fallback.
    calls: list[object] = []
    configured_limits = [object(), object()]

    class RejectIntegration:
        def integrate_inplace(self, *_args, **_kwargs) -> None:
            raise AssertionError("failed constrained IK must not integrate")

    solver = object.__new__(kinematics._IKSolver)
    solver._tasks = {}
    solver._posture_cost = 0.0
    solver._posture_task = None
    solver._freeze_task = None
    solver._max_iters = 10
    solver._config = RejectIntegration()
    solver._dt = 0.02
    solver._solver_name = "daqp"
    solver._limits = configured_limits
    solver._solver_params = {}
    solver._sides = ("right", "left")
    solver._pending = set()

    original_solve_ik = kinematics.mink.solve_ik

    def reject_constrained(*_args, **kwargs):
        calls.append(kwargs.get("limits"))
        raise kinematics.mink.exceptions.NoSolutionFound("fault-injection")

    kinematics.mink.solve_ik = reject_constrained
    try:
        result = solver.solve()
    finally:
        kinematics.mink.solve_ik = original_solve_ik

    assert result is None
    assert len(calls) == 1
    assert calls[0] is configured_limits
    assert solver._pending == {"right", "left"}
    print("OpenArm constrained-only IK safety verification passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
