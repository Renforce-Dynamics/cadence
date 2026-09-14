"""Cadence control runtime, configuration tools, and installed application launcher."""

import argparse
from datetime import datetime, timezone
from importlib.metadata import entry_points, version, PackageNotFoundError
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
        run.add_argument("--config", help="Deployment YAML or pkg:// resource")
        run.add_argument("--profile", help="Installed application/profile")
        run.add_argument("--backend")
        run.add_argument("--duration-s", type=float)
        run.add_argument("--headless", action="store_true")
        run.add_argument("--output", help="Snapshot directory (deploy defaults to runs/deploy-*)")
        run.add_argument("--set", action="append", default=[])
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
        if args.command == "deploy" and args.output is None:
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            args.output = str(Path("runs") / f"deploy-{stamp}-{uuid.uuid4().hex[:8]}")
        if args.profile:
            name, sep, profile = args.profile.partition("/")
            for entry in entry_points(group="cadence.applications"):
                if entry.name == name:
                    options = dict(backend=args.backend, duration=args.duration_s,
                                   headless=args.headless, output=args.output)
                    if args.config is not None:
                        options["config"] = args.config
                    if args.set:
                        options["overrides"] = args.set
                    if args.check:
                        options["check"] = True
                    if args.command == "deploy":
                        options["deploy"] = True
                    return entry.load().run(profile or None, **options)
            raise ValueError(f"application {name!r} is not installed")
        return run_config(
            args.config or "pkg://cadence/data/demo.yaml",
            backend_override=args.backend,
            duration=args.duration_s,
            output=args.output,
            overrides=args.set,
            check=args.check,
        )
    except (ValueError, OSError, ImportError, RuntimeError) as error:
        p.error(str(error))
