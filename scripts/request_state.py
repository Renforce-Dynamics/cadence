#!/usr/bin/env python3
"""Publish a PLNJ state request until the receiver enters the expected mode.

Minimal driver for verification scripts: keeps requesting (the receiver gates
active states on its own entry conditions), then sends neutral frames before
exiting. Exits 3 immediately if the receiver safety-halts.

Usage:
  scripts/request_state.py 6 --expect-mode SONIC_CLIP
"""

from __future__ import annotations

import argparse
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
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=50560)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=50.0)
    args = parser.parse_args(argv)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    deadline = time.monotonic() + args.timeout_s
    last = "no reply"
    try:
        while time.monotonic() < deadline:
            packet = JoystickCommandPacket(
                session_id=42, packet_seq=sequence, source_age_us=0, ttl_ms=100,
                flags=JoystickFlags.CONNECTED | JoystickFlags.REQUEST_VALID,
                request_id=args.state_id)
            sock.sendto(encode_joystick_command(packet), (args.host, args.port))
            sequence = (sequence + 1) & 0xFFFFFFFF
            if sequence % 10 == 0:
                try:
                    current = status(args.host, args.port)
                except OSError as error:
                    last = repr(error)
                    continue
                last = {k: current.get(k) for k in ("mode", "substate", "safety_halted")}
                if current.get("safety_halted"):
                    print("safety_halted=True:", current, flush=True)
                    return 3
                if current.get("mode", "").upper() == args.expect_mode.upper():
                    break
            time.sleep(1.0 / args.hz)
        else:
            print("timeout; last status:", last, flush=True)
            return 1
    finally:
        for _ in range(10):
            packet = JoystickCommandPacket(
                session_id=42, packet_seq=sequence, source_age_us=0, ttl_ms=100,
                flags=JoystickFlags.CONNECTED, request_id=0)
            sock.sendto(encode_joystick_command(packet), (args.host, args.port))
            sequence = (sequence + 1) & 0xFFFFFFFF
            time.sleep(0.02)
        sock.close()
    print("reached:", last, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
