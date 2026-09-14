"""Configuration-loaded control plugins, with no fixed robot or task registry."""

from dataclasses import dataclass, field
from importlib import import_module
from types import SimpleNamespace
from typing import Any
import numpy as np
from cadence_api import JointCommand
from cadence_api.plugins import RootLossMode
from cadence_config import validate_keys


@dataclass(frozen=True)
class ControlFrame:
    now_s: float
    robot_state: Any
    localization: Any = None
    goal: Any = None
    velocity_command: tuple = (0.0, 0.0, 0.0)
    operator_input: Any = None


@dataclass(frozen=True)
class ControlResult:
    command: JointCommand
    diagnostics: Any = None
    substate: str = "READY"
    events: tuple = ()
    entry_gate_ready: bool | None = None
    goal_used: Any = None
    next_state: str | int | None = None

    def __post_init__(self):
        if self.diagnostics is None:
            n = len(self.command.q_des)
            object.__setattr__(
                self,
                "diagnostics",
                SimpleNamespace(
                    observation_kind="none",
                    policy_model="none",
                    observation=np.empty(0),
                    raw_action=np.zeros(n),
                    executed_action=np.zeros(n),
                    position_clip_mask=np.zeros(n, dtype=bool),
                    position_clip_excess=np.zeros(n),
                ),
            )


class ControlState:
    active_policy = False
    active_entry_gate = False
    requires_world_root = False
    requires_operator_link = False
    uses_loco_velocity = False
    root_loss_mode = RootLossMode.NONE
    root_loss_exit_steps = None
    root_loss_fallback_state = None
    entry_smoothing_s = 0.0

    def __init__(self, state_id, key, config, services):
        self.state_id = state_id
        self.key = key
        self.config = config
        self.services = services

    def on_enter(self, frame, events):
        pass

    def on_exit(self, frame, events):
        pass

    def step(self, frame):
        raise NotImplementedError

    def on_command_applied(self, command):
        pass

    def on_command_rejected(self):
        pass

    def diagnostics_after_command(self, diagnostics):
        return diagnostics


class BasicState(ControlState):
    def __init__(self, state_id, key, config, services):
        super().__init__(state_id, key, config, services)
        validate_keys(
            config,
            {
                "kind",
                "kp",
                "kd",
                "target",
                "duration_s",
                "entry_gate_progress",
                "entry_gate_tolerance",
            },
            required={"kind"},
        )
        self.kind = config["kind"]
        self.n = services.dimension
        if self.kind not in {"passive", "damping", "fixed_position"}:
            raise ValueError("unknown basic control kind")
        self.active_entry_gate = self.kind == "fixed_position"
        self.kp = np.broadcast_to(config.get("kp", 20.0), (self.n,)).copy()
        self.kd = np.broadcast_to(config.get("kd", 2.0), (self.n,)).copy()
        self.target = np.asarray(config.get("target", np.zeros(self.n)), dtype=float)
        self.duration = float(config.get("duration_s", 1.0))
        self.entry_gate_progress = float(config.get("entry_gate_progress", 1.0))
        self.entry_gate_tolerance = float(config.get("entry_gate_tolerance", 0.05))
        if (
            self.target.shape != (self.n,)
            or self.duration <= 0
            or not np.all(np.isfinite(self.target))
        ):
            raise ValueError("invalid fixed position configuration")
        if (
            not np.all(np.isfinite(self.kp))
            or not np.all(np.isfinite(self.kd))
            or np.any(self.kp < 0)
            or np.any(self.kd < 0)
        ):
            raise ValueError("invalid gain")
        if (
            not 0.0 <= self.entry_gate_progress <= 1.0
            or not np.isfinite(self.entry_gate_tolerance)
            or self.entry_gate_tolerance <= 0
        ):
            raise ValueError("invalid entry gate configuration")

    def on_enter(self, frame, events):
        self.start = frame.now_s
        self.initial = frame.robot_state.joint_pos.copy()

    def step(self, frame):
        z = np.zeros(self.n)
        q = frame.robot_state.joint_pos
        if self.kind == "passive":
            return ControlResult(JointCommand(q, z, z, z, z))
        if self.kind == "damping":
            return ControlResult(JointCommand(q, z, z, self.kd, z))
        t = min(1.0, max(0.0, (frame.now_s - self.start) / self.duration))
        a = t * t * (3 - 2 * t)
        target = (1 - a) * self.initial + a * self.target
        return ControlResult(
            JointCommand(target, z, self.kp, self.kd, z),
            entry_gate_ready=(
                t >= self.entry_gate_progress
                and np.max(np.abs(q - self.target)) < self.entry_gate_tolerance
            ),
        )


class PluginCatalog:
    def __init__(self, manifest):
        validate_keys(
            manifest,
            {"states", "aliases", "reset_state_id", "safety_fallback_state_id"},
            required={"states", "reset_state_id", "safety_fallback_state_id"},
        )
        self.definitions = {}
        self.ids = {}
        self.aliases = manifest.get("aliases", {})
        for state_id, definition in manifest["states"].items():
            validate_keys(
                definition,
                {"key", "factory", "config"},
                required={"key", "factory", "config"},
            )
            key = definition["key"]
            state_id = int(state_id)
            if (
                not isinstance(key, str)
                or not key
                or key in self.definitions
                or state_id in self.ids
            ):
                raise ValueError("duplicate/invalid state")
            self.ids[state_id] = key
            self.definitions[key] = (state_id, definition)
        self.reset_state_id = int(manifest["reset_state_id"])
        self.safety_fallback_state_id = int(manifest["safety_fallback_state_id"])
        for i in (self.reset_state_id, self.safety_fallback_state_id):
            self.key_for_id(i)
        for alias, target in self.aliases.items():
            if alias in self.definitions or target not in self.definitions:
                raise ValueError("invalid state alias")

    def canonical_key(self, value):
        if isinstance(value, int):
            return self.key_for_id(value)
        key = self.aliases.get(str(value), str(value))
        if key not in self.definitions:
            raise ValueError(f"unknown control state {value}")
        return key

    def key_for_id(self, value):
        if value not in self.ids:
            raise ValueError(f"unknown state ID {value}")
        return self.ids[value]

    def instantiate(self, services):
        result = {}
        for key, (state_id, d) in self.definitions.items():
            module, sep, symbol = d["factory"].partition(":")
            if not sep:
                raise ValueError("factory must be module:Class")
            cls = getattr(import_module(module), symbol)
            if not issubclass(cls, ControlState):
                raise TypeError("factory must implement ControlState")
            result[key] = cls(state_id, key, d["config"], services)
        return result
