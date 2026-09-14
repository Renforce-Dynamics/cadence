"""Cadence control runtime, configuration tools, and installed application launcher."""

import argparse
from contextlib import ExitStack
from importlib.metadata import entry_points, version, PackageNotFoundError
from types import SimpleNamespace
from pathlib import Path
import json
import time
import yaml
from copy import deepcopy
from collections.abc import Mapping
from cadence_config import load_config, validate_keys, ConfigError, ResolvedConfig


def load_run_config(path):
    """Resolve state config files and retain their declaration-relative paths."""
    resolved = load_config(
        path,
        allowed={"schema_version", "robot", "backend", "runtime", "catalog"},
        required={"schema_version", "robot", "backend", "runtime", "catalog"},
    )
    data, origins = deepcopy(resolved.data), dict(resolved.origins)
    from .plugins import PluginCatalog

    catalog = data["catalog"]
    if not isinstance(catalog, Mapping) or not isinstance(catalog.get("states"), Mapping):
        raise ConfigError("catalog.states must be a mapping")
    PluginCatalog(catalog)
    for state_id, definition in data["catalog"]["states"].items():
        prefix = f"catalog.states.{state_id}.config"
        if isinstance(definition.get("config"), str):
            child = load_config(resolved.path(prefix))
            definition["config"] = child.data
            origins.pop(prefix, None)
            origins.update({f"{prefix}.{key}": value for key, value in child.origins.items()})
        if not isinstance(definition.get("config"), Mapping):
            raise ConfigError(f"{prefix} must be a mapping or configuration path")
    expanded = ResolvedConfig(data, origins, resolved.source, resolved.overrides)
    for state_id, definition in data["catalog"]["states"].items():
        lower = definition.get("config", {}).get("lower")
        if isinstance(lower, dict) and "model" in lower:
            lower["model"] = str(expanded.path(f"catalog.states.{state_id}.config.lower.model"))
    return expanded


def run_config(path, *, backend_override=None, duration=None, output=None):
    from .plugins import PluginCatalog, ControlFrame
    from .runtime import RuntimeKernel, RuntimeConfig, RuntimeInput

    resolved = load_run_config(path)
    cfg = resolved.data
    if cfg["schema_version"] != 1:
        raise ConfigError("unsupported schema version")
    validate_keys(
        cfg["runtime"],
        {"control_hz", "deadline_ms", "duration_s", "start_state", "velocity_command", "upper_target_udp", "operator"},
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
    deadline = float(cfg["runtime"]["deadline_ms"]) / 1000
    import numpy as np

    if not np.all(np.isfinite([hz, duration, deadline])) or hz <= 0 or duration < 0 or deadline <= 0:
        raise ValueError("rate and deadline must be finite and positive; duration must be finite and nonnegative")
    velocity = np.asarray(cfg["runtime"].get("velocity_command", (0, 0, 0)), dtype=float)
    if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
        raise ValueError("runtime.velocity_command requires three finite values")
    from .operator.config import OperatorIngressConfig

    operator_config = OperatorIngressConfig.from_mapping(cfg["runtime"].get("operator"))
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
    ticks = 0
    started = time.monotonic()
    receiver = kernel = operator = None
    try:
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
                deadline,
            ),
            state,
            0.0,
        )
        if operator_config is not None:
            from .operator import JoystickCommandReceiver, OperatorInputAdapter
            from .control.filters import VelocitySlewRateLimiter

            operator = JoystickCommandReceiver(operator_config.host, operator_config.port)
            operator.set_catalog(kernel.config.state_catalog)
            operator.update_status(SimpleNamespace(mode=kernel.current.key, safety_halted=False, events=()))
            input_adapter = OperatorInputAdapter(operator_config.mapping, operator_config.signal_mode)
            velocity_limiter = VelocitySlewRateLimiter(
                operator_config.linear_slew_rate_mps2, operator_config.yaw_slew_rate_radps2,
                operator_config.velocity_deadzone,
            )
        udp = cfg["runtime"].get("upper_target_udp")
        if udp is not None:
            from .motion.targets import JointTargetUdpReceiver

            validate_keys(udp, {"state", "host", "port"}, required={"state", "host", "port"})
            targets = getattr(kernel.plugin(udp["state"]), "upper_targets", None)
            if targets is None:
                raise ValueError("upper_target_udp.state must select a streamed upper motion state")
            receiver = JointTargetUdpReceiver(targets, bind=(udp["host"], udp["port"]))
            receiver.start()
        while duration == 0 or ticks * dt < duration - 1e-9:
            state = backend.read_state()
            before = time.monotonic()
            inputs = {"velocity_command": tuple(velocity)}
            if operator is not None:
                sample = input_adapter.map(operator.poll(), before)
                # Keep the immutable packet object; dataclasses.asdict would
                # turn it into a dict and break task-defined input consumers.
                inputs = {field: getattr(sample, field) for field in sample.__dataclass_fields__}
                if not kernel.current.uses_loco_velocity or kernel.safety.halted:
                    velocity_limiter.reset(before)
                inputs["velocity_command"] = velocity_limiter.update(sample.velocity_command, before)
            result = kernel.prepare(RuntimeInput(ticks * dt, state, None, **inputs))
            kernel.observe_control_duration(time.monotonic() - before)
            result = kernel.guard_pending()
            try:
                backend.write_command(result.command, state.sequence)
            except Exception:
                kernel.reject()
                raise
            result = kernel.commit()
            if operator is not None:
                operator.update_status(result)
            ticks += 1
            if duration == 0 or receiver is not None or operator is not None:
                time.sleep(max(0, dt - (time.monotonic() - before)))
    except KeyboardInterrupt:
        pass
    finally:
        # Run all cleanup callbacks even when an input or state fails to close.
        with ExitStack() as cleanup:
            cleanup.callback(backend.close)
            if kernel is not None:
                cleanup.callback(kernel.current.on_exit, ControlFrame(ticks * dt, state), [])
                cleanup.callback(kernel.reject)
            if receiver is not None:
                cleanup.callback(receiver.stop)
            if operator is not None:
                cleanup.callback(operator.close)
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
