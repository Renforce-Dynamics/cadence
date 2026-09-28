#!/usr/bin/env python3
"""Publish a PLNJ state request until the receiver enters the expected mode.

Minimal driver for verification scripts: keeps requesting (the receiver gates
active states on its own entry conditions), then sends neutral frames before
exiting. Optionally waits for a substate, pulses a D-pad direction (edge
trigger for states like sonic_clip), then waits for another substate.
Exits 3 immediately if the receiver safety-halts.

Usage:
  scripts/request_state.py 6 --expect-mode SONIC_CLIP
  scripts/request_state.py 6 --expect-mode SONIC_CLIP \
      --await-substate READY --pulse-dpad-x 1 --then-substate PLAYING
"""

from __future__ import annotations

import argparse
import secrets
import socket
import sys
import time

from cadence_protocol.client import OperatorClient
from cadence_protocol.operator import (
    JoystickCommandPacket,
    JoystickFlags,
    encode_joystick_command,
)


def status(host, port):
    with OperatorClient(host, port, timeout_s=0.5) as client:
        return client.status()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("state_id", type=int)
    parser.add_argument("--expect-mode", required=True)
    parser.add_argument("--await-substate", help="after the mode: go neutral and wait for this substate")
    parser.add_argument("--pulse-dpad-x", type=int, choices=(-1, 0, 1), default=0)
    parser.add_argument("--pulse-dpad-y", type=int, choices=(-1, 0, 1), default=0)
    parser.add_argument("--pulse-s", type=float, default=0.5)
    parser.add_argument("--then-substate", help="after the pulse: wait for this substate")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=50560)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=50.0)
    args = parser.parse_args(argv)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # A fresh session per invocation: the receiver drops same-session packets
    # whose sequence does not exceed the watermark of previous invocations.
    session_id = secrets.randbits(32)
    sequence = 0
    deadline = time.monotonic() + args.timeout_s

    def publish(request=None, dpad=(0, 0)):
        nonlocal sequence
        flags = JoystickFlags.CONNECTED
        if request is not None:
            flags |= JoystickFlags.REQUEST_VALID
        packet = JoystickCommandPacket(
            session_id=session_id, packet_seq=sequence, source_age_us=0, ttl_ms=100,
            flags=flags, request_id=0 if request is None else request,
            dpad_x=dpad[0], dpad_y=dpad[1])
        sock.sendto(encode_joystick_command(packet), (args.host, args.port))
        sequence = (sequence + 1) & 0xFFFFFFFF

    def run_phase(label, predicate, *, request=None, dpad=(0, 0), duration_s=None):
        nonlocal sequence
        last = "no reply"
        phase_end = None if duration_s is None else time.monotonic() + duration_s
        while time.monotonic() < deadline:
            publish(request, dpad)
            if sequence % 10 == 0:
                try:
                    current = status(args.host, args.port)
                except OSError as error:
                    last = repr(error)
                    time.sleep(1.0 / args.hz)
                    continue
                last = {k: current.get(k) for k in ("mode", "substate", "safety_halted")}
                if current.get("safety_halted"):
                    print("safety_halted=True:", current, flush=True)
                    raise SystemExit(3)
                if predicate is not None and predicate(current):
                    print(f"{label} reached:", last, flush=True)
                    return last
            if phase_end is not None and time.monotonic() >= phase_end:
                print(f"{label}: pulsed {duration_s}s", flush=True)
                return last
            time.sleep(1.0 / args.hz)
        print(f"timeout in {label}; last status: {last}", flush=True)
        raise SystemExit(1)

    try:
        run_phase("mode", lambda s: s.get("mode", "").upper() == args.expect_mode.upper(),
                  request=args.state_id)
        if args.await_substate:
            run_phase("substate", lambda s: s.get("substate") == args.await_substate)
        if args.pulse_dpad_x or args.pulse_dpad_y:
            run_phase("pulse", None, dpad=(args.pulse_dpad_x, args.pulse_dpad_y),
                      duration_s=args.pulse_s)
        if args.then_substate:
            run_phase("substate", lambda s: s.get("substate") == args.then_substate)
    finally:
        for _ in range(10):
            publish()
            time.sleep(0.02)
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
