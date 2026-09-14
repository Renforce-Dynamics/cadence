"""Configuration-driven deployments using Cadence state and backend contracts."""

from contextlib import ExitStack
from copy import deepcopy
from dataclasses import dataclass
from collections.abc import Mapping
from importlib.metadata import PackageNotFoundError, version
import hashlib
import json
from pathlib import Path
import signal
import threading
import time
from types import SimpleNamespace

import numpy as np
from cadence_config import load_config, validate_keys, ConfigError, ResolvedConfig

def load_run_config(path, *, overrides=(), backend_override=None, duration=None):
    """Resolve state config files and retain their declaration-relative paths."""
    resolved = load_config(
        path,
        overrides=overrides,
        allowed={"schema_version", "robot", "backend", "runtime", "catalog"},
        required={"schema_version", "robot", "backend", "runtime", "catalog"},
    )
    data, origins = deepcopy(resolved.data), dict(resolved.origins)
    applied = list(resolved.overrides)
    flags = {}
    if backend_override is not None:
        flags["backend.kind"] = "a3" if backend_override == "readonly" else backend_override
        if backend_override == "readonly":
            flags["backend.read_only"] = True
            flags["backend.command_publish_enabled"] = False
    if duration is not None:
        flags["runtime.duration_s"] = duration
    for key, value in flags.items():
        section, field = key.split(".")
        data[section][field] = value
        origins[key] = str(resolved.source)
        applied.append(f"{key}={json.dumps(value)}")
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
    expanded = ResolvedConfig(data, origins, resolved.source, tuple(applied))
    for state_id, definition in data["catalog"]["states"].items():
        lower = definition.get("config", {}).get("lower")
        if isinstance(lower, dict) and "model" in lower:
            lower["model"] = str(expanded.path(f"catalog.states.{state_id}.config.lower.model"))
    backend = data["backend"]
    for key in ("model", "library_path"):
        if backend.get(key) is not None:
            backend[key] = str(expanded.path(f"backend.{key}"))
    if isinstance(backend.get("aimrt"), dict) and "config_path" in backend["aimrt"]:
        backend["aimrt"]["config_path"] = str(expanded.path("backend.aimrt.config_path"))
    return expanded


@dataclass(frozen=True)
class DeploymentPlan:
    resolved: ResolvedConfig
    catalog: object
    backend: object
    control_hz: float
    duration_s: float
    deadline_s: float
    dimension: int
    operator: object
    localization: object
    upper: dict | None
    velocity: tuple


def prepare_deployment(resolved):
    """Validate the complete deployment before starting any input or RobotIO."""
    from .operator.config import OperatorIngressConfig
    from .localization import LocalizationIngressConfig
    from .plugins import PluginCatalog

    cfg = resolved.data
    if type(cfg["schema_version"]) is not int or cfg["schema_version"] != 1:
        raise ConfigError("unsupported schema version")
    validate_keys(cfg["runtime"], {
        "control_hz", "deadline_ms", "duration_s", "start_state", "velocity_command",
        "operator", "localization", "upper_target_udp",
    }, required={"control_hz", "deadline_ms", "duration_s", "start_state"}, label="runtime")
    validate_keys(cfg["robot"], {"joints", "position_min", "position_max"},
                  required={"joints", "position_min", "position_max"}, label="robot")
    joints = cfg["robot"]["joints"]
    if not isinstance(joints, list) or not joints or any(not isinstance(j, str) or not j for j in joints) or len(set(joints)) != len(joints):
        raise ConfigError("robot.joints must contain unique, nonempty joint names")
    dimension = len(joints)
    low, high = (np.asarray(cfg["robot"][k], dtype=float) for k in ("position_min", "position_max"))
    if low.shape != (dimension,) or high.shape != (dimension,) or not np.all(np.isfinite([low, high])) or np.any(low > high):
        raise ConfigError("robot bounds must be finite ordered vectors matching robot.joints")
    hz, duration, deadline_ms = (cfg["runtime"][k] for k in ("control_hz", "duration_s", "deadline_ms"))
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in (hz, duration, deadline_ms)):
        raise ConfigError("rate, duration and deadline must be finite numbers")
    hz, duration, deadline_ms = float(hz), float(duration), float(deadline_ms)
    if not np.all(np.isfinite([hz, duration, deadline_ms])) or hz <= 0 or duration < 0 or deadline_ms <= 0:
        raise ConfigError("rate and deadline must be finite and positive; duration must be finite and nonnegative")
    velocity = np.asarray(cfg["runtime"].get("velocity_command", (0, 0, 0)), dtype=float)
    if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
        raise ConfigError("runtime.velocity_command requires three finite values")
    operator = OperatorIngressConfig.from_mapping(cfg["runtime"].get("operator"))
    localization = LocalizationIngressConfig.from_mapping(cfg["runtime"].get("localization"))
    catalog = PluginCatalog(cfg["catalog"])
    catalog.canonical_key(cfg["runtime"]["start_state"])
    upper = cfg["runtime"].get("upper_target_udp")
    if upper is not None:
        import ipaddress
        validate_keys(upper, {"state", "host", "port"}, required={"state", "host", "port"}, label="runtime.upper_target_udp")
        catalog.canonical_key(upper["state"])
        if not ipaddress.ip_address(upper["host"]).is_loopback:
            raise ConfigError("upper target receiver requires a loopback IP address")
        if type(upper["port"]) is not int or not 0 <= upper["port"] <= 65535:
            raise ConfigError("upper target port must be an integer in [0, 65535]")
    raw = cfg["backend"]
    simulation_fields = {"model", "physics_timestep_s", "initial_joint_position", "gyro_sensor", "quaternion_sensor", "root_site"}
    a3_fields = {"transport", "read_only", "command_publish_enabled", "state_timeout_ms", "command_timeout_ms", "startup_timeout_s", "safe_damping_kd", "neck_kp", "neck_kd", "library_path", "aimrt"}
    validate_keys(raw, {"kind"} | simulation_fields | a3_fields, required={"kind"}, label="backend")
    kind = raw["kind"]
    if kind == "mock":
        from .backends.mock import MockBackend
        backend = MockBackend(dimension, 1 / hz)
    elif kind == "mujoco":
        from .backends.mujoco import MujocoBackend
        if not raw.get("model"):
            raise ConfigError("MuJoCo requires backend.model")
        if raw.get("root_site") is not None and raw.get("quaternion_sensor") is None:
            raise ConfigError("MuJoCo backend.root_site requires backend.quaternion_sensor")
        backend = MujocoBackend(
            raw["model"], joints, 1 / hz, raw.get("physics_timestep_s", .005),
            **{key: raw[key] for key in ("initial_joint_position", "gyro_sensor", "quaternion_sensor", "root_site") if key in raw},
        )
    elif kind == "a3":
        from .backends.a3 import A3Backend, CONTROL_JOINT_NAMES
        if tuple(joints) != CONTROL_JOINT_NAMES:
            raise ConfigError("A3 backend requires the canonical 29 joint names in SDK order")
        backend = A3Backend.from_mapping({key: raw[key] for key in {"kind"} | a3_fields if key in raw})
        for key in ("library_path",):
            if raw.get(key) is not None and not Path(raw[key]).is_file():
                raise ConfigError(f"backend.{key} does not exist: {raw[key]}")
        if raw["transport"] == "aimrt" and not Path(raw["aimrt"]["config_path"]).is_file():
            raise ConfigError(f"AimRT configuration does not exist: {raw['aimrt']['config_path']}")
    else:
        raise ConfigError("backend.kind must be mock, mujoco or a3; --backend readonly selects read-only A3")
    return DeploymentPlan(resolved, catalog, backend, hz, duration, deadline_ms / 1000,
                          dimension, operator, localization, upper, tuple(velocity))


def _freeze(plan, output):
    if output is None:
        return
    cfg = plan.resolved.data
    plan.resolved.freeze(output)
    resources = {}
    def collect(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key in {"model", "library_path", "config_path"} and isinstance(item, str) and Path(item).is_file():
                    resources[item] = hashlib.sha256(Path(item).read_bytes()).hexdigest()
                else:
                    collect(item)
        elif isinstance(value, list):
            for item in value:
                collect(item)
    collect(cfg)
    packages = {}
    for name in ("cadence", "cadence-api", "cadence-config", "cadence-protocol", "agi3sdk", "onnxruntime", "mujoco"):
        try:
            packages[name] = version(name)
        except PackageNotFoundError:
            pass
    (Path(output) / "deployment.json").write_text(json.dumps({
        "source": str(plan.resolved.source), "backend": cfg["backend"],
        "duration_s": plan.duration_s, "control_hz": plan.control_hz,
        "config_sha256": plan.resolved.digest, "resource_sha256": resources, "packages": packages,
    }, indent=2) + "\n")


def _backend_localization(backend, state):
    from .runtime import LocalizationState
    if hasattr(backend, "root_site") and backend.root_site is None:
        return None
    if state.root_position_w is None or state.root_linear_velocity_w is None:
        return None
    return LocalizationState(state.root_position_w, state.quaternion_wxyz, state.root_linear_velocity_w)


def _publish_status(receiver, output, *, shadow):
    receiver.update_status({
        "mode": output.mode, "safety_halted": output.safety_halted,
        "events": output.events, "execution": "shadow" if shadow else "backend",
    })


def run_config(path, *, backend_override=None, duration=None, output=None, overrides=(), check=False):
    """Run a standalone deployment; optional checks never start RobotIO or UDP."""
    from .plugins import ControlFrame
    from .runtime import RuntimeKernel, RuntimeConfig, RuntimeInput
    from .runtime.loop import execute_cycle, read_control_state

    resolved = load_run_config(path, overrides=overrides, backend_override=backend_override, duration=duration)
    plan = prepare_deployment(resolved)
    cfg, backend = resolved.data, plan.backend
    _freeze(plan, output)
    if check:
        try:
            states = plan.catalog.instantiate(SimpleNamespace(dimension=plan.dimension))
            if plan.upper is not None and not hasattr(states[plan.catalog.canonical_key(plan.upper["state"])], "upper_targets"):
                raise ConfigError("upper_target_udp.state must select a streamed upper motion state")
            print(json.dumps({"configuration_valid": True, "backend": cfg["backend"]["kind"],
                              "states": plan.catalog.ids, "config_sha256": resolved.digest, "started": False}))
            return 0
        finally:
            backend.close()

    dt = 1 / plan.control_hz
    hardware = cfg["backend"]["kind"] == "a3"
    shadow = bool(getattr(backend, "read_only", False))
    realtime = hardware or plan.duration_s == 0 or any((plan.operator, plan.localization, plan.upper))
    ticks = writes = shadow_steps = stale_skips = state_timeouts = 0
    failure = None
    kernel = receiver = operator = localization = None
    state = None
    state_now = 0.0
    stop = threading.Event()
    started = time.monotonic()
    with ExitStack() as cleanup:
        cleanup.callback(backend.close)
        if threading.current_thread() is threading.main_thread():
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous = signal.signal(sig, lambda *_: stop.set())
                cleanup.callback(signal.signal, sig, previous)
        try:
            services = SimpleNamespace(dimension=plan.dimension)
            states = plan.catalog.instantiate(services)
            if plan.localization is not None:
                from .localization import LocalizationReceiver
                localization = LocalizationReceiver(plan.localization)
                cleanup.callback(localization.close)
            backend.start()
            state = backend.read_state(timeout_s=getattr(backend, "startup_timeout_s", None))
            initial_sample = localization.poll() if localization is not None else None
            initial_root = (None if initial_sample is None else initial_sample.localization) if localization is not None else _backend_localization(backend, state)
            start_key = plan.catalog.canonical_key(cfg["runtime"]["start_state"])
            if states[start_key].requires_world_root and initial_root is None:
                raise ConfigError("start_state requires localization; start in a state that does not require world localization, then request motion after localization arrives")
            kernel = RuntimeKernel(
                RuntimeConfig(plan.catalog, services, ControlFrame,
                              cfg["robot"]["position_min"], cfg["robot"]["position_max"],
                              cfg["runtime"]["start_state"], plan.deadline_s),
                state, 0.0, initial_localization=initial_root, preloaded_plugins=states,
            )
            cleanup.callback(lambda: kernel.current.on_exit(ControlFrame(state_now, state), []))
            cleanup.callback(kernel.reject)
            if plan.operator is not None:
                from .operator import JoystickCommandReceiver, OperatorInputAdapter
                from .control.filters import VelocitySlewRateLimiter
                operator = JoystickCommandReceiver(plan.operator.host, plan.operator.port)
                cleanup.callback(operator.close)
                operator.set_catalog(plan.catalog)
                _publish_status(operator, SimpleNamespace(mode=kernel.current.key, safety_halted=False, events=()), shadow=shadow)
                input_adapter = OperatorInputAdapter(plan.operator.mapping, plan.operator.signal_mode)
                velocity_limiter = VelocitySlewRateLimiter(plan.operator.linear_slew_rate_mps2, plan.operator.yaw_slew_rate_radps2, plan.operator.velocity_deadzone)
            if plan.upper is not None:
                from .motion.targets import JointTargetUdpReceiver
                targets = getattr(kernel.plugin(plan.upper["state"]), "upper_targets", None)
                if targets is None:
                    raise ConfigError("upper_target_udp.state must select a streamed upper motion state")
                receiver = JointTargetUdpReceiver(targets, bind=(plan.upper["host"], plan.upper["port"]))
                cleanup.callback(receiver.stop)
                receiver.start()
            # Model loading and the first state wait are outside deployment duration.
            started = next_tick = last_fresh_state = time.monotonic()
            observed_watchdog = getattr(backend, "watchdog_trip_count", 0)
            while not stop.is_set():
                if plan.duration_s and (time.monotonic() - started if hardware else ticks * dt) >= plan.duration_s - 1e-9:
                    break
                try:
                    if hardware:
                        state = read_control_state(backend, next_tick, backend.state_timeout_s)
                    else:
                        state = backend.read_state()
                except TimeoutError:
                    if not hardware:
                        raise
                    state_timeouts += 1
                    if time.monotonic() - last_fresh_state >= backend.command_timeout_ms / 1000:
                        failure = "synchronized state missing beyond command watchdog"
                        kernel.safety.halt(failure)
                        break
                    continue
                if stop.is_set():
                    break
                now = time.monotonic()
                last_fresh_state = now
                if hardware and backend.watchdog_trip_count > observed_watchdog:
                    failure = "SDK watchdog entered safe damping"
                    kernel.safety.halt(failure)
                    break
                next_tick = max(next_tick + dt, now)
                state_now = now - started if hardware else ticks * dt
                inputs = {"velocity_command": plan.velocity}
                if operator is not None:
                    sample = input_adapter.map(operator.poll(), now)
                    inputs = {field: getattr(sample, field) for field in sample.__dataclass_fields__}
                    if not kernel.current.uses_loco_velocity or kernel.safety.halted:
                        velocity_limiter.reset(now)
                    inputs["velocity_command"] = velocity_limiter.update(sample.velocity_command, now)
                if localization is not None:
                    observation = localization.poll(now_monotonic_s=now)
                    root = None if observation is None else observation.localization
                else:
                    root = _backend_localization(backend, state)
                if stop.is_set():
                    break
                try:
                    cycle = execute_cycle(kernel, backend, RuntimeInput(state_now, state, root, **inputs), read_only=shadow)
                except Exception as error:
                    if hardware and backend.is_state_stale_error(error):
                        stale_skips += 1
                        ticks += 1
                        continue
                    raise
                writes += cycle.submitted
                shadow_steps += cycle.shadow
                ticks += 1
                if operator is not None:
                    _publish_status(operator, cycle.output, shadow=shadow)
                if realtime and not hardware:
                    time.sleep(max(0, dt - (time.monotonic() - now)))
        except KeyboardInterrupt:
            if kernel is None:
                raise
            stop.set()
    summary = {
        "backend": cfg["backend"]["kind"], "ticks": ticks, "dimension": plan.dimension,
        "mode": kernel.current.key, "halted": kernel.safety.halted,
        "elapsed_s": time.monotonic() - started, "read_only": shadow,
        "command_writes": writes, "shadow_steps": shadow_steps,
        "stale_command_skips": stale_skips, "state_wait_timeouts": state_timeouts,
        "command_publish_enabled": bool(getattr(backend, "command_publish_enabled", False)),
        "failure": failure,
    }
    if output is not None:
        (Path(output) / "result.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary))
    return 2 if failure else int(kernel.safety.halted)
