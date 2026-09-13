#!/usr/bin/env python3
"""Query or send joint targets to an explicitly enabled local motion receiver.

Example (choose the configured upper-joint count and order):
  python scripts/send-upper-target.py --port 50620 --status
  python scripts/send-upper-target.py --port 50620 --sequence 0 --q-des 0.1 -0.2

For a continuous producer, obtain the activation once when entering the state,
then send increasing sequence numbers with that token. A receipt reports only
mailbox reception; it does not confirm that the robot executed the command.
"""

import argparse
import ipaddress
import json
import math
import socket
import sys


SCHEMA = "cadence.joint-target.v1"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", required=True, type=int)
    parser.add_argument("--status", action="store_true", help="show activation and dimension")
    parser.add_argument("--activation", type=int, help="otherwise query the current activation")
    parser.add_argument("--sequence", type=int, help="monotonically increasing within activation")
    parser.add_argument("--q-des", nargs="+", type=float, help="joint positions in radians")
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()
    try:
        address = ipaddress.ip_address(args.host)
    except ValueError:
        parser.error("--host must be a loopback IP address")
    if not address.is_loopback:
        parser.error("--host must be a loopback IP address")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be positive and finite")
    if args.status and any(value is not None for value in (args.activation, args.sequence, args.q_des)):
        parser.error("--status cannot be combined with target fields")
    if not args.status:
        if args.sequence is None or args.sequence < 0 or args.q_des is None:
            parser.error("a target requires --sequence >= 0 and --q-des")
        if args.activation is not None and args.activation < 1:
            parser.error("--activation must be positive")
        if not all(math.isfinite(value) for value in args.q_des):
            parser.error("--q-des must contain finite positions")
    family = socket.AF_INET6 if address.version == 6 else socket.AF_INET
    with socket.socket(family, socket.SOCK_DGRAM) as sock:
        sock.settimeout(args.timeout)
        sock.connect((str(address), args.port))

        def exchange(payload):
            sock.send(json.dumps(payload, allow_nan=False).encode("utf-8"))
            return json.loads(sock.recv(65535))

        try:
            if args.status or args.activation is None:
                status = exchange({"schema": SCHEMA, "type": "status"})
                if args.status:
                    print(json.dumps(status, sort_keys=True))
                    return 0 if status.get("type") == "status" else 1
                args.activation = status.get("activation")
                if args.activation is None:
                    print("motion state is inactive; no target sent", file=sys.stderr)
                    return 1
                if status.get("dimension") != len(args.q_des):
                    print(f"expected {status.get('dimension')} joint positions", file=sys.stderr)
                    return 1
            receipt = exchange({
                "schema": SCHEMA,
                "type": "target",
                "activation": args.activation,
                "sequence": args.sequence,
                "q_des": args.q_des,
            })
        except (OSError, ValueError) as exc:
            print(f"target receiver: {exc}", file=sys.stderr)
            return 1
    print(json.dumps(receipt, sort_keys=True))
    return 0 if receipt.get("accepted") is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
