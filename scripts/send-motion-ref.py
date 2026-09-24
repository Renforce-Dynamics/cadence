#!/usr/bin/env python3
"""Replay a sonic NPZ clip as a cadence.motion-ref.v1 test stream.

Queries the receiver status for the current activation, then publishes frames
at the clip rate: each packet carries a small batch ending at the current
frame, so a lost packet only costs one batch of redundancy. Frames are
stamped by the receiver at arrival; the control state's delay line turns the
stream into the policy reference.

Example (against the mock, sim or onboard runtime in sonic_stream):
  scripts/send-motion-ref.py data/motions/BMD_0319_stand.npz --loop
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import time

import numpy as np

SCHEMA = "cadence.motion-ref.v1"


def load_clip(path):
    with np.load(path, allow_pickle=False) as data:
        q = np.asarray(data["q_ref"], dtype=np.float64)
        dq = np.asarray(data["dq_ref"], dtype=np.float64)
        quat = np.asarray(data["root_quat_wxyz"], dtype=np.float64)
        dt = float(np.asarray(data["dt"]))
    if q.ndim != 2 or q.shape[1] != 29 or dq.shape != q.shape or quat.shape != (len(q), 4):
        raise ValueError("NPZ must contain q_ref[T,29], dq_ref[T,29], root_quat_wxyz[T,4]")
    return q, dq, quat, dt


def exchange(sock, address, request, timeout_s=1.0):
    sock.sendto(json.dumps(request).encode("utf-8"), address)
    sock.settimeout(timeout_s)
    data, _ = sock.recvfrom(65535)
    reply = json.loads(data)
    if reply.get("schema") != SCHEMA:
        raise RuntimeError(f"unexpected reply schema: {reply.get('schema')}")
    return reply


def wait_for_activation(sock, address, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            reply = exchange(sock, address, {"schema": SCHEMA, "type": "status"}, 0.5)
        except (socket.timeout, OSError):
            continue
        if reply.get("type") == "status" and reply.get("activation") is not None:
            print(f"receiver live: state={reply.get('state_key')} activation={reply['activation']}")
            return int(reply["activation"])
        time.sleep(0.2)
    raise RuntimeError("timed out waiting for a sonic_stream activation; enter the state first")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("clip", type=Path, help="NPZ clip from scripts/convert_sonic_clip.py")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15120)
    parser.add_argument("--batch", type=int, default=3,
                        help="frames per packet; the newest frame is sent last")
    parser.add_argument("--loop", action="store_true", help="wrap at the clip end")
    parser.add_argument("--activation-timeout-s", type=float, default=30.0)
    args = parser.parse_args(argv)
    if not 1 <= args.batch <= 16:
        parser.error("--batch must be in [1, 16]")
    q, dq, quat, dt = load_clip(args.clip)
    address = (args.host, args.port)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        activation = wait_for_activation(sock, address, args.activation_timeout_s)
        sequence = 0
        accepted = rejected = 0
        index = 0
        print(f"streaming {len(q)} frames at {1.0 / dt:.0f} Hz to {args.host}:{args.port}")
        next_tick = time.monotonic()
        while True:
            first = max(0, index - args.batch + 1)
            frames = [
                {"root_quat_wxyz": quat[i].tolist(), "q": q[i].tolist(), "dq": dq[i].tolist()}
                for i in range(first, index + 1)
            ]
            request = {"schema": SCHEMA, "type": "frames", "activation": activation,
                       "sequence": sequence, "dt_ms": round(dt * 1000), "frames": frames}
            try:
                reply = exchange(sock, address, request, 0.05)
                if reply.get("accepted"):
                    accepted += 1
                else:
                    rejected += 1
                    if reply.get("reason") == "inactive_or_stale":
                        print("receiver left the stream state; waiting for a new activation")
                        activation = wait_for_activation(sock, address, args.activation_timeout_s)
                        sequence = 0
                        continue
            except socket.timeout:
                rejected += 1
            sequence += 1
            index += 1
            if index >= len(q):
                print(f"clip done: {accepted} accepted, {rejected} rejected/lost")
                if not args.loop:
                    return 0
                index = 0
                accepted = rejected = 0
            next_tick += dt
            time.sleep(max(0.0, next_tick - time.monotonic()))


if __name__ == "__main__":
    raise SystemExit(main())
