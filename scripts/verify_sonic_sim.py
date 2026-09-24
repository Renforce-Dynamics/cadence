#!/usr/bin/env python3
"""Closed-loop MuJoCo verification driver for the SONIC states.

Runs the real config chain (backend, registry, states, policy) in-process
through cadence's execute_cycle, on sim time (deterministic, not wall-paced):

  damping -> fixedpos (PD_STAND entry gate) -> sonic_clip(stand) 12 s
          -> fixedpos -> sonic_clip(walk) 15 s

Metrics per stage: pelvis site z min/max, joint tracking RMSE against the
reference during PLAYING, safety halts. Exit 0 only if every band holds.

Usage:
  .venv/bin/python scripts/verify_sonic_sim.py \
      [--config configs/entry/a3/mock/entry_basketball_sonic_sim.yaml] \
      [--stand-s 12] [--walk-s 15]
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
from cadence.deployment import load_run_config, prepare_deployment  # noqa: E402
from cadence.plugins import ControlFrame  # noqa: E402
from cadence.runtime import RuntimeConfig, RuntimeInput, RuntimeKernel  # noqa: E402
from cadence.runtime.loop import execute_cycle  # noqa: E402

from cadence.sonic.contract import isaaclab_to_cadence
from cadence.sonic.reference import SonicClip

STAND_CLIP = ROOT / "data/motions/BMD_0319_stand.npz"
WALK_CLIP = ROOT / "data/motions/001_walk_front_slow.npz"

# Acceptance bands. Nominal standing pelvis site z is ~0.857.
Z_MIN, Z_MAX = 0.60, 1.00
RMSE_MAX = 0.15  # rad


def run_stage(kernel, backend, make_input, ticks, label):
    """Drive the kernel for ``ticks`` control cycles, collecting metrics."""
    zmin, zmax = math.inf, -math.inf
    errors = []
    halted = None
    state = kernel.plugin(6)
    for _ in range(ticks):
        cycle = execute_cycle(kernel, backend, make_input())
        output = cycle.output
        if output.safety_halted:
            halted = output.safety_reason
            break
        robot = backend.read_state()
        z = float(robot.root_position_w[2])
        zmin, zmax = min(zmin, z), max(zmax, z)
        if output.skill_state == "PLAYING":
            t = min(state.play_tick, state.clip.frames - 1)
            errors.append(robot.joint_pos - isaaclab_to_cadence(state.clip.q_ref[t]))
    rmse = float(np.sqrt(np.mean(np.square(errors)))) if errors else float("nan")
    ok = (halted is None and zmin > Z_MIN and zmax < Z_MAX
          and (not errors or rmse < RMSE_MAX))
    print(f"{label}: z=[{zmin:.3f},{zmax:.3f}] rmse={rmse:.4f} rad "
          f"({len(errors)} PLAYING ticks) halted={halted} -> {'PASS' if ok else 'FAIL'}")
    return ok


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "configs/entry/a3/mock/entry_a3_sonic_sim.yaml"))
    parser.add_argument("--stand-s", type=float, default=12.0)
    parser.add_argument("--walk-s", type=float, default=15.0)
    args = parser.parse_args(argv)

    resolved = load_run_config(args.config)
    # Stage A plays the stand clip; swap the clip resource before construction.
    stand_cfg = resolved.data["catalog"]["states"][6]["config"]
    stand_cfg["resources"]["clip"] = str(STAND_CLIP)
    plan = prepare_deployment(resolved)
    cfg = plan.resolved.data
    backend = plan.backend
    services = type("Services", (), {"dimension": plan.dimension,
                                     "joint_names": tuple(cfg["robot"]["joints"])})
    backend.start()
    tick = 0
    failures = []
    try:
        robot = plan.backend.read_state()
        kernel = RuntimeKernel(
            RuntimeConfig(plan.catalog, services, ControlFrame,
                          cfg["robot"]["position_min"], cfg["robot"]["position_max"],
                          "damping", plan.deadline_s), robot, 0.0)

        def make_input(request=None):
            nonlocal tick, robot
            tick += 1
            robot = backend.read_state()
            return RuntimeInput(tick / plan.control_hz, robot, None, requested_state=request)

        def settle(request, max_ticks, until_gate=False):
            for _ in range(max_ticks):
                execute_cycle(kernel, backend, make_input(request))
                if until_gate and kernel.entry_gate_ready:
                    return True
            return not until_gate

        # fixedpos entry gate (PD_STAND in the sim registry).
        if not settle(2, 200, until_gate=True):
            print("fixedpos: entry gate never ready -> FAIL")
            return 1
        z = float(robot.root_position_w[2])
        print(f"fixedpos: gate ready, z={z:.3f} -> {'PASS' if Z_MIN < z < Z_MAX else 'FAIL'}")
        if not Z_MIN < z < Z_MAX:
            failures.append("fixedpos")

        # Stage A: stand clip.
        execute_cycle(kernel, backend, make_input(6))
        if not run_stage(kernel, backend, make_input, round(args.stand_s * plan.control_hz),
                         "sonic_clip(stand)"):
            failures.append("sonic_clip(stand)")

        # Back to fixedpos, re-gate, then the walk clip.
        if not settle(2, 200, until_gate=True):
            print("fixedpos re-gate after stand clip failed -> FAIL")
            failures.append("fixedpos re-gate")
        else:
            state = kernel.plugin(6)
            state.clip = SonicClip.load(WALK_CLIP)
            execute_cycle(kernel, backend, make_input(6))
            if not run_stage(kernel, backend, make_input, round(args.walk_s * plan.control_hz),
                             "sonic_clip(walk)"):
                failures.append("sonic_clip(walk)")
    finally:
        kernel.reject()
        kernel.current.on_exit(ControlFrame(tick / plan.control_hz, robot), [])
        plan.backend.close()
    print("verify_sonic_sim:", "PASS" if not failures else f"FAIL ({', '.join(failures)})")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
