"""Cadence control runtime, configuration tools, and installed application launcher."""

import argparse
from importlib.metadata import entry_points, version, PackageNotFoundError
from types import SimpleNamespace
from pathlib import Path
import json
import time
import yaml
from cadence_config import load_config, validate_keys, ConfigError


def run_config(path, *, backend_override=None, duration=None, output=None):
    from .plugins import PluginCatalog, ControlFrame
    from .runtime import RuntimeKernel, RuntimeConfig, RuntimeInput

    resolved = load_config(
        path,
        allowed={"schema_version", "robot", "backend", "runtime", "catalog"},
        required={"schema_version", "robot", "backend", "runtime", "catalog"},
    )
    cfg = resolved.data
    if cfg["schema_version"] != 1:
        raise ConfigError("unsupported schema version")
    validate_keys(
        cfg["runtime"],
        {"control_hz", "deadline_ms", "duration_s", "start_state"},
        required={"control_hz", "deadline_ms", "duration_s", "start_state"},
    )
    validate_keys(
        cfg["robot"],
        {"joints", "position_min", "position_max"},
        required={"joints", "position_min", "position_max"},
    )
    validate_keys(
        cfg["backend"], {"kind", "model", "physics_timestep_s"}, required={"kind"}
    )
    hz = float(cfg["runtime"]["control_hz"])
    duration = float(cfg["runtime"]["duration_s"] if duration is None else duration)
    if hz <= 0 or duration < 0:
        raise ValueError("invalid rate or duration")
    dt = 1 / hz
    kind = backend_override or cfg["backend"]["kind"]
    n = len(cfg["robot"]["joints"])
    if kind == "mock":
        from .backends.mock import MockBackend

        backend = MockBackend(n, dt)
    elif kind == "mujoco":
        from .backends.mujoco import MujocoBackend

        backend = MujocoBackend(
            resolved.path("backend.model"),
            cfg["robot"]["joints"],
            dt,
            cfg["backend"].get("physics_timestep_s", 0.005),
        )
    else:
        raise ValueError(
            "generic examples support mock or mujoco; use an application profile for hardware"
        )
    if output:
        resolved.freeze(output)
    backend.start()
    state = backend.read_state()
    kernel = RuntimeKernel(
        RuntimeConfig(
            PluginCatalog(cfg["catalog"]),
            SimpleNamespace(dimension=n),
            ControlFrame,
            cfg["robot"]["position_min"],
            cfg["robot"]["position_max"],
            cfg["runtime"]["start_state"],
            cfg["runtime"]["deadline_ms"] / 1000,
        ),
        state,
        0.0,
    )
    ticks = 0
    started = time.monotonic()
    try:
        while duration == 0 or ticks * dt < duration - 1e-9:
            state = backend.read_state()
            before = time.monotonic()
            result = kernel.prepare(RuntimeInput(ticks * dt, state, None))
            kernel.observe_control_duration(time.monotonic() - before)
            result = kernel.guard_pending()
            try:
                backend.write_command(result.command, state.sequence)
            except Exception:
                kernel.reject()
                raise
            result = kernel.commit()
            ticks += 1
            if duration == 0:
                time.sleep(max(0, dt - (time.monotonic() - before)))
    except KeyboardInterrupt:
        pass
    finally:
        backend.close()
    print(
        json.dumps(
            {
                "backend": kind,
                "ticks": ticks,
                "dimension": n,
                "mode": kernel.current.key,
                "halted": kernel.safety.halted,
                "elapsed_s": time.monotonic() - started,
            }
        )
    )
    return int(kernel.safety.halted)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run")
    run.add_argument("--config", default="pkg://cadence/data/demo.yaml")
    run.add_argument("--profile")
    run.add_argument("--backend")
    run.add_argument("--duration-s", type=float)
    run.add_argument("--headless", action="store_true")
    run.add_argument("--output")
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
        if args.profile:
            name, sep, profile = args.profile.partition("/")
            for entry in entry_points(group="cadence.applications"):
                if entry.name == name:
                    return entry.load().run(
                        profile or "baseline",
                        backend=args.backend,
                        duration=args.duration_s,
                        headless=args.headless,
                        output=args.output,
                    )
            raise ValueError(f"application {name!r} is not installed")
        return run_config(
            args.config,
            backend_override=args.backend,
            duration=args.duration_s,
            output=args.output,
        )
    except (ValueError, OSError) as error:
        p.error(str(error))
