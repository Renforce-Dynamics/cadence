"""Backend-independent state-plugin execution kernel.

The kernel deliberately has no concrete state, policy, observation or model
imports.  A checked startup manifest composes persistent state instances.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from ..control import JointCommand, SafetySupervisor
from cadence_api import RobotState
from cadence_api.plugins import RootLossMode


@dataclass(frozen=True, slots=True)
class LocalizationState:
    position_w: np.ndarray
    orientation_wxyz: np.ndarray
    linear_velocity_w: np.ndarray

    def __post_init__(self) -> None:
        for name, dimension in (
            ("position_w", 3),
            ("orientation_wxyz", 4),
            ("linear_velocity_w", 3),
        ):
            value = np.asarray(getattr(self, name), dtype=np.float64).reshape(-1)
            if value.shape != (dimension,) or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must contain {dimension} finite values")
            object.__setattr__(self, name, value.copy())
        norm = np.linalg.norm(self.orientation_wxyz)
        if norm < 1e-6:
            raise ValueError("orientation_wxyz must be non-zero")
        object.__setattr__(self, "orientation_wxyz", self.orientation_wxyz / norm)


@dataclass(frozen=True, slots=True)
class RuntimeConfig:
    state_catalog: Any
    services: Any
    frame_factory: Any
    position_min: Any
    position_max: Any
    start_state: str | int = "passive"
    deadline_s: float = 0.020
    handoff_resolver: Any = None


@dataclass(frozen=True, slots=True)
class RuntimeInput:
    now_s: float
    robot_state: RobotState
    localization: LocalizationState | None
    goal: Any | None = None
    requested_state: str | int | None = None
    emergency_halt: bool = False
    reset_safety: bool = False
    velocity_command: tuple[float, float, float] = (0.0, 0.0, 0.0)
    operator_input: Any | None = None
    operator_link_usable: bool = True


@dataclass(frozen=True, slots=True)
class RuntimeOutput:
    command: JointCommand
    mode: str
    skill_state: str
    safety_halted: bool
    safety_reason: str
    raw_action_abs_max: float
    position_clip_count: int
    position_clip_max_rad: float
    observation_kind: str
    policy_model: str
    observation: np.ndarray
    raw_action: np.ndarray
    executed_action: np.ndarray
    position_clip_mask: np.ndarray
    position_clip_excess: np.ndarray
    events: tuple[str, ...]
    goal_used: Any | None = None


class RuntimeKernel:
    """Generic dispatcher over preloaded, manifest-defined state plugins."""

    def __init__(
        self, config: RuntimeConfig, initial_state: RobotState, now_s: float
    ) -> None:
        if config.deadline_s <= 0:
            raise ValueError("deadline_s must be positive")
        self.config = config
        self.dimension = len(initial_state.joint_pos)
        self.position_min = np.asarray(config.position_min, dtype=float)
        self.position_max = np.asarray(config.position_max, dtype=float)
        if (
            self.position_min.shape != (self.dimension,)
            or self.position_max.shape != (self.dimension,)
            or not np.all(np.isfinite(self.position_min))
            or not np.all(np.isfinite(self.position_max))
            or np.any(self.position_min > self.position_max)
        ):
            raise ValueError("invalid joint position bounds")
        self._pending = None
        self._ticket = 0
        services = config.services
        # Every enabled state and ONNX session is parsed, loaded and warmed by
        # construction before the frontend opens its command loop.
        self.plugins = config.state_catalog.instantiate(services)
        self.current_key = config.state_catalog.canonical_key(config.start_state)
        self.current = self.plugins[self.current_key]
        self.safety = SafetySupervisor(config.deadline_s)
        self._entry_gate_ready = False
        self._last_command_q_des = np.asarray(initial_state.joint_pos).copy()
        self._transition_started_s: float | None = None
        self._transition_duration_s = 0.0
        self._transition_from_q_des = self._last_command_q_des.copy()
        self._last_valid_root: LocalizationState | None = None
        self._root_loss_steps = 0
        self._root_loss_blocked_mode: str | None = None
        initial_frame = self.config.frame_factory(
            float(now_s), initial_state, None, None, (0.0, 0.0, 0.0), None
        )
        self.current.on_enter(initial_frame, [])

    def plugin(self, value: str | int):
        return self.plugins[self.config.state_catalog.canonical_key(value)]

    def observe_control_duration(self, duration_s: float) -> None:
        self.safety.observe_control_duration(duration_s)

    def on_command_rejected(self):
        self.reject()

    def _frame(self, value: RuntimeInput, root) -> Any:
        return self.config.frame_factory(
            float(value.now_s),
            value.robot_state,
            root,
            value.goal,
            value.velocity_command,
            value.operator_input,
        )

    def prepare(self, value: RuntimeInput) -> RuntimeOutput:
        if self._pending is not None:
            raise RuntimeError("previous command needs commit or reject")
        self._ticket += 1
        events: list[str] = []
        now = float(value.now_s)
        robot = value.robot_state
        live_root = value.localization

        if self.current.requires_operator_link and not value.operator_link_usable:
            newly_halted = not self.safety.halted
            self.safety.halt("operator link disconnected or stale")
            if newly_halted:
                events.append("operator link lost; latched safety damping")
        if value.emergency_halt:
            self.safety.halt("operator emergency halt")
            events.append("operator emergency halt")
        if (
            value.reset_safety
            and not value.emergency_halt
            and (not self.current.requires_operator_link or value.operator_link_usable)
        ):
            self.safety.reset()
            reset_key = self.config.state_catalog.key_for_id(
                self.config.state_catalog.reset_state_id
            )
            self._transition(reset_key, self._frame(value, live_root), events)
            self._root_loss_steps = 0
            self._root_loss_blocked_mode = None
            events.append(f"safety reset -> {reset_key.upper()}")

        root = live_root
        if live_root is not None:
            if self._root_loss_steps:
                events.append(
                    f"ROOT localization recovered after {self._root_loss_steps} lost steps"
                )
            self._last_valid_root = live_root
            self._root_loss_steps = 0
        elif self.current.root_loss_mode in {
            RootLossMode.HOLD_LAST,
            RootLossMode.FALLBACK,
        }:
            self._root_loss_steps += 1
            if self._last_valid_root is not None:
                root = self._last_valid_root
                if self._root_loss_steps == 1:
                    message = "ROOT localization lost; holding last valid root"
                    if self.current.root_loss_exit_steps is not None:
                        message += f" for {self.current.root_loss_exit_steps} steps"
                    events.append(message)

        requested_key = None
        if value.requested_state is not None:
            try:
                requested_key = self.config.state_catalog.canonical_key(
                    value.requested_state
                )
            except ValueError:
                requested_key = str(value.requested_state).strip().lower()
        if self._root_loss_blocked_mode is not None:
            if requested_key == self._root_loss_blocked_mode:
                requested_key = None
            elif requested_key is not None:
                self._root_loss_blocked_mode = None

        exit_steps = self.current.root_loss_exit_steps
        if (
            live_root is None
            and self.current.root_loss_mode == RootLossMode.FALLBACK
            and exit_steps is not None
            and self._root_loss_steps >= exit_steps
        ):
            old = self.current.key.upper()
            self._root_loss_blocked_mode = self.current.key
            requested_key = self.config.state_catalog.canonical_key(
                self.current.root_loss_fallback_state
            )
            events.append(
                f"ROOT localization lost for {self._root_loss_steps} steps; "
                f"{old} -> {requested_key.upper()}"
            )

        if requested_key in self.plugins and requested_key != self.current_key:
            target = self.plugins[requested_key]
            if target.requires_world_root and live_root is None and root is None:
                events.append(
                    f"rejected {requested_key.upper()}: world localization unavailable"
                )
            elif (
                target.active_policy
                and not self.current.active_policy
                and (not self.current.active_entry_gate or not self._entry_gate_ready)
            ):
                events.append(
                    f"rejected {requested_key.upper()}: enter alignment mode and wait for convergence"
                )
            else:
                self._transition(requested_key, self._frame(value, root), events)

        self.safety.validate_state(
            robot.gyro_b,
            robot.quaternion_wxyz,
            robot.joint_pos,
            robot.joint_vel,
        )
        if root is not None:
            self.safety.validate_state(
                root.position_w,
                root.orientation_wxyz,
                root.linear_velocity_w,
            )

        result = None
        result_owner = None
        try:
            if self.safety.halted:
                command = self._safety_fallback_command(value, root)
            else:
                result_owner = self.current
                result = self.current.step(self._frame(value, root))
                events.extend(result.events)
                command = self._smooth_transition(result.command, now)
            if not self.safety.validate_command(command):
                raise ValueError("invalid command")
            arrays = (
                command.q_des,
                command.dq_des,
                command.kp,
                command.kd,
                command.tau_ff,
            )
            if any(
                np.asarray(v).shape != (self.dimension,) or not np.all(np.isfinite(v))
                for v in arrays
            ):
                raise ValueError("command layout does not match robot state")
            if np.any(command.kp < 0) or np.any(command.kd < 0):
                raise ValueError("negative control gain")
            # Position is only relevant when stiffness is enabled.
            controlled = command.kp > 0
            if np.any(
                command.q_des[controlled] < self.position_min[controlled]
            ) or np.any(command.q_des[controlled] > self.position_max[controlled]):
                raise ValueError("joint command exceeds configured limits")
        except (RuntimeError, ValueError) as error:
            self.safety.halt(f"control pipeline failure: {error}")
            events.append(f"SAFETY HALT: {self.safety.reason}")
            command = self._safety_fallback_command(value, root)
            self._transition_started_s = None
            self._transition_duration_s = 0.0
            if result_owner is not None:
                result_owner.on_command_rejected()
            result = None

        diagnostics = (
            None
            if result is None
            else result_owner.diagnostics_after_command(result.diagnostics)
        )
        if diagnostics is None:
            observation_kind = "none"
            policy_model = "none"
            observation = np.empty(0, dtype=np.float32)
            raw_action = np.zeros(self.dimension, dtype=np.float32)
            executed_action = np.zeros(self.dimension, dtype=np.float32)
            clip_mask = np.zeros(self.dimension, dtype=np.bool_)
            clip_excess = np.zeros(self.dimension, dtype=np.float32)
            substate = "INACTIVE"
        else:
            observation_kind = diagnostics.observation_kind
            policy_model = diagnostics.policy_model
            observation = diagnostics.observation
            raw_action = diagnostics.raw_action
            executed_action = diagnostics.executed_action
            clip_mask = diagnostics.position_clip_mask
            clip_excess = diagnostics.position_clip_excess
            substate = result.substate
        output = RuntimeOutput(
            command=command,
            mode=self.current.key.upper(),
            skill_state=substate,
            safety_halted=self.safety.halted,
            safety_reason=self.safety.reason,
            raw_action_abs_max=float(np.max(np.abs(raw_action))),
            position_clip_count=int(np.count_nonzero(clip_mask)),
            position_clip_max_rad=float(np.max(clip_excess)),
            observation_kind=observation_kind,
            policy_model=policy_model,
            observation=observation.copy(),
            raw_action=raw_action.copy(),
            executed_action=executed_action.copy(),
            position_clip_mask=clip_mask.copy(),
            position_clip_excess=clip_excess.copy(),
            events=tuple(events),
            goal_used=(None if result is None else result.goal_used),
        )

        self._pending = (
            self._ticket,
            result_owner,
            result,
            self._frame(value, root),
            output,
        )
        return output

    def guard_pending(self):
        if self._pending is None:
            raise RuntimeError("no pending command")
        ticket, owner, result, frame, output = self._pending
        if self.safety.halted and result is not None:
            owner.on_command_rejected()
            key = self.config.state_catalog.key_for_id(
                self.config.state_catalog.safety_fallback_state_id
            )
            command = self.plugins[key].step(frame).command
            output = replace(
                output,
                command=command,
                safety_halted=True,
                safety_reason=self.safety.reason,
                events=(*output.events, self.safety.reason),
            )
            self._pending = (ticket, None, None, frame, output)
        return output

    @property
    def pending_ticket(self):
        return None if self._pending is None else self._pending[0]

    def commit(self, ticket=None):
        if self._pending is None:
            raise RuntimeError("no pending command")
        current_ticket, owner, result, frame, output = self._pending
        if ticket is not None and ticket != current_ticket:
            raise RuntimeError("stale command acknowledgement")
        self._pending = None
        self._last_command_q_des = output.command.q_des.copy()
        events = list(output.events)
        if result is not None and owner is self.current and not self.safety.halted:
            owner.on_command_applied(output.command)
            if result.entry_gate_ready is not None:
                self._entry_gate_ready = result.entry_gate_ready
            diag = owner.diagnostics_after_command(result.diagnostics)
            output = replace(output, executed_action=diag.executed_action.copy())
            if result.next_state is not None:
                target = result.next_state
                if self.config.handoff_resolver is not None:
                    target = self.config.handoff_resolver(owner.key, target)
                if target is not None:
                    self._state_requested_transition(
                        target, frame, events, source=owner
                    )
        return replace(output, mode=self.current.key.upper(), events=tuple(events))

    def reject(self, ticket=None):
        if self._pending is None:
            return
        current_ticket, owner, result, frame, output = self._pending
        if ticket is not None and ticket != current_ticket:
            raise RuntimeError("stale command acknowledgement")
        self._pending = None
        if result is not None and owner is not None:
            owner.on_command_rejected()

    def tick(self, value):
        """In-memory accepted step for tests/shadow evaluation; live I/O uses prepare/commit."""
        self.prepare(value)
        return self.commit()

    def _state_requested_transition(
        self,
        target: str | int,
        frame: Any,
        events: list[str],
        *,
        source,
    ) -> None:
        """Honor a plugin handoff after its boundary command was accepted."""
        if source is not self.current:
            raise RuntimeError("state handoff source is no longer active")
        target_key = self.config.state_catalog.canonical_key(target)
        if target_key == self.current_key:
            return
        target_state = self.plugins[target_key]
        if target_state.requires_world_root and frame.localization is None:
            raise RuntimeError(
                f"state handoff to {target_key.upper()} requires world localization"
            )
        events.append(f"STATE handoff {source.key.upper()} -> {target_key.upper()}")
        self._transition(target_key, frame, events)

    def _transition(self, target_key: str, frame: Any, events: list[str]) -> None:
        if target_key == self.current_key:
            return
        previous = self.current.key.upper()
        self.current.on_exit(frame, events)
        self.current_key = target_key
        self.current = self.plugins[target_key]
        self.current.on_enter(frame, events)
        events.append(f"MODE {previous} -> {self.current.key.upper()}")
        self._entry_gate_ready = False
        duration = self.current.entry_smoothing_s
        if duration > 0:
            self._transition_started_s = frame.now_s
            self._transition_duration_s = duration
            self._transition_from_q_des = self._last_command_q_des.copy()
            events.append(f"STATE transition smoothing {duration:.3f}s")
        else:
            self._transition_started_s = None
            self._transition_duration_s = 0.0

    def _smooth_transition(self, command: JointCommand, now_s: float) -> JointCommand:
        if self._transition_started_s is None:
            return command
        progress = max(
            0.0,
            min(
                1.0,
                (now_s - self._transition_started_s) / self._transition_duration_s,
            ),
        )
        alpha = progress * progress * (3.0 - 2.0 * progress)
        q_des = (
            (1.0 - alpha) * self._transition_from_q_des + alpha * command.q_des
        ).clip(self.position_min, self.position_max)
        if progress >= 1.0:
            self._transition_started_s = None
            self._transition_duration_s = 0.0
        return JointCommand(
            q_des, command.dq_des, command.kp, command.kd, command.tau_ff
        )

    def _safety_fallback_command(self, value: RuntimeInput, root) -> JointCommand:
        key = self.config.state_catalog.key_for_id(
            self.config.state_catalog.safety_fallback_state_id
        )
        return self.plugins[key].step(self._frame(value, root)).command
