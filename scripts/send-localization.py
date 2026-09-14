#!/usr/bin/env python3
"""Publish explicit synthetic localization samples using cadence-protocol only.

Without timing flags, send exactly one sample. Continuous publication requires
both --duration-s and --hz. This is a transport/integration fixture, not a sensor
driver: an actual producer must supply measured pose, velocity and sample age.
"""

import argparse
import json
import math
import sys
import time

from cadence_protocol.localization import LocalizationClient


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15110)
    parser.add_argument("--source", default="localization")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--child-frame-id", default="policy_root")
    parser.add_argument("--session-id", type=int, help="ordered producer epoch; default Unix microseconds")
    parser.add_argument("--position-m", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--orientation-wxyz", nargs=4, type=float, default=[1, 0, 0, 0], metavar=("W", "X", "Y", "Z"))
    parser.add_argument("--velocity-mps", nargs=3, type=float, default=[0, 0, 0], metavar=("VX", "VY", "VZ"))
    parser.add_argument("--ttl-s", type=float, default=0.25)
    parser.add_argument("--invalid", action="store_true", help="withdraw localization; position is then optional")
    parser.add_argument("--duration-s", type=float, help="finite, positive synthetic stream duration")
    parser.add_argument("--hz", type=float, help="finite, positive synthetic publication rate")
    args = parser.parse_args(argv)
    if args.position_m is None and not args.invalid:
        parser.error("--position-m is required for a valid synthetic pose")
    if (args.duration_s is None) != (args.hz is None):
        parser.error("continuous publication requires both --duration-s and --hz")
    if args.duration_s is not None and any(
        not math.isfinite(value) or value <= 0 for value in (args.duration_s, args.hz)
    ):
        parser.error("--duration-s and --hz must be finite and positive")
    sent = 0
    interrupted = False
    try:
        with LocalizationClient(
            args.host, args.port, source=args.source, frame_id=args.frame_id,
            child_frame_id=args.child_frame_id, session_id=args.session_id, ttl_s=args.ttl_s,
        ) as publisher:
            started = time.monotonic()
            due = started
            while True:
                now = time.monotonic()
                if sent and (args.duration_s is None or now - started >= args.duration_s):
                    break
                if now < due:
                    time.sleep(min(due - now, 0.05))
                    continue
                # Each publication is a newly generated synthetic pose fixture.
                sample = publisher.send(
                    args.position_m or [0, 0, 0], args.orientation_wxyz, args.velocity_mps,
                    valid=not args.invalid, source_age_s=0.0,
                    source_timestamp_us=time.time_ns() // 1000,
                )
                sent += 1
                if args.hz is not None:
                    due = max(due + 1.0 / args.hz, time.monotonic())
    except KeyboardInterrupt:
        interrupted = True
    except (OSError, ValueError) as error:
        print(f"localization publisher: {error}", file=sys.stderr)
        return 1
    print(json.dumps({"sent": sent, "interrupted": interrupted,
                      "session_id": sample.session_id if sent else None,
                      "last_sequence": sample.sequence if sent else None,
                      "valid": not args.invalid,
                      "delivery": "local UDP send; receipt is not acknowledged"}, sort_keys=True))
    return 130 if interrupted else 0


if __name__ == "__main__":
    raise SystemExit(main())
