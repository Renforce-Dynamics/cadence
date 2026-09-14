"""Run Cadence from one editable entry configuration."""

import argparse
from datetime import datetime, timezone
from importlib.metadata import version, PackageNotFoundError
from pathlib import Path
import uuid
import yaml
from cadence_config import load_config


# Compatibility imports for callers of the original generic runner.
from .deployment import load_run_config, run_config


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    for name in ("run", "deploy"):
        run = sub.add_parser(name)
        run.add_argument("--config", required=True, help="Entry YAML file, e.g. configs/entry/examples/entry_sim.yaml")
        run.add_argument("--output", help="Snapshot directory (defaults to runs/deploy-*)")
        run.add_argument("--check", action="store_true", help="Validate the deployment and load models without opening inputs or RobotIO")
    config = sub.add_parser("config")
    config.add_argument("action", choices=["resolve", "validate", "diff"])
    config.add_argument("path")
    config.add_argument("other", nargs="?")
    config.add_argument("--set", action="append", default=[])
    config.add_argument("--output")
    sub.add_parser("doctor")
    args = p.parse_args(argv)
    try:
        if args.command == "doctor":
            for name in (
                "cadence",
                "cadence-api",
                "cadence-config",
                "cadence-protocol",
                "mujoco",
                "onnxruntime",
                "agi3sdk",
            ):
                try:
                    print(name, version(name))
                except PackageNotFoundError:
                    print(name, "not installed (optional)")
            return 0
        if args.command == "config":
            resolved = load_config(args.path, overrides=args.set)
            if args.output:
                resolved.freeze(args.output)
            if args.action == "diff":
                if not args.other:
                    p.error("diff requires two configs")
                import difflib

                other = load_config(args.other)
                print(
                    "".join(
                        difflib.unified_diff(
                            yaml.safe_dump(resolved.data).splitlines(True),
                            yaml.safe_dump(other.data).splitlines(True),
                        )
                    )
                )
            elif args.action == "resolve":
                print(yaml.safe_dump(resolved.data, sort_keys=False))
            else:
                print("Configuration valid:", resolved.digest)
            return 0
        if args.output is None:
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            args.output = str(Path("runs") / f"deploy-{stamp}-{uuid.uuid4().hex[:8]}")
        return run_config(args.config, output=args.output, check=args.check)
    except (ValueError, OSError, ImportError, RuntimeError) as error:
        p.error(str(error))
