"""Lower-body policy states composed with fixed or streamed upper joint targets."""

from __future__ import annotations

from dataclasses import dataclass, replace
from importlib import import_module

import numpy as np

from cadence_api import JointCommand
from cadence.plugins import ControlResult, ControlState

from .config import MotionConfig
from .targets import LatestJointTarget


@dataclass(frozen=True, slots=True)
class MotionDiagnostics:
    observation_kind: str
    policy_model: str
    observation: np.ndarray
    raw_action: np.ndarray
    executed_action: np.ndarray
    position_clip_mask: np.ndarray
    position_clip_excess: np.ndarray


class LowerLocoState(ControlState):
    """Run a lower-joint policy while holding a configured upper-joint posture.

    The adapter owns model-specific observations. The state owns joint partitioning,
    PD targets, and the command acknowledgement boundary. Stateful adapters implement
    snapshot()/restore() so a rejected command also rolls back observation history.
    """

    active_policy = True
    uses_loco_velocity = True
    streamed_upper = False

    @classmethod
    def load_config(cls, raw, services):
        return MotionConfig.from_mapping(raw, getattr(services, "dimension", None))

    def __init__(self, state_id, key, config, services):
        if not isinstance(config, MotionConfig):
            config = self.load_config(config, services)
        dimension = getattr(services, "dimension", config.robot.dimension)
        if dimension != config.robot.dimension:
            raise ValueError("motion configuration dimension does not match services")
        super().__init__(state_id, key, config, services)
        self.n = config.robot.dimension
        self.entry_smoothing_s = config.entry_smoothing_s
        module, symbol = config.lower.factory.split(":")
        self.adapter = getattr(import_module(module), symbol)(config, services)
        if not callable(getattr(self.adapter, "reset", None)) or not callable(getattr(self.adapter, "infer", None)):
            raise TypeError("lower adapter must implement reset() and infer(frame, last_action)")
        snapshot = callable(getattr(self.adapter, "snapshot", None))
        restore = callable(getattr(self.adapter, "restore", None))
        if snapshot != restore:
            raise TypeError("lower adapter must implement both snapshot() and restore()")
        self._has_snapshot = snapshot
        for name in ("policy", "policy_spec", "policy_config", "observations"):
            if hasattr(self.adapter, name):
                setattr(self, name, getattr(self.adapter, name))
        if not hasattr(self, "policy_config"):
            self.policy_config = config.lower
        self.last_action = np.zeros(self.n, dtype=np.float64)
        self.upper_position = config.upper.default_position.copy()
        self.committed_upper_sequence = None
        self.phase = "INACTIVE"
        self._active = False
        self._entered_command_applied = False
        self._pending = None
        self._last_diagnostics = self._diagnostics(np.empty(0), self.last_action, np.zeros(self.n))
        if self.streamed_upper:
            upper = config.robot.upper_joints
            self.upper_targets = LatestJointTarget(
                len(upper), config.robot.position_min[upper], config.robot.position_max[upper]
            )

    @property
    def default_pose(self):
        return self.config.robot.default_position

    @property
    def action_scale(self):
        return self.config.robot.action_scale

    def on_enter(self, frame, events):
        self.on_command_rejected()
        self.adapter.reset()
        self.last_action.fill(0)
        self.upper_position = self.config.upper.default_position.copy()
        self.committed_upper_sequence = None
        self._entered_command_applied = False
        self._active = True
        self.phase = "DEFAULT"
        self.started_s = float(frame.now_s)
        self._last_diagnostics = self._diagnostics(np.empty(0), self.last_action, np.zeros(self.n))
        if self.streamed_upper:
            self.upper_targets.activate()

    def on_exit(self, frame, events):
        self.on_command_rejected()
        if self.streamed_upper:
            self.upper_targets.deactivate()
        self._active = False
        self.phase = "INACTIVE"

    def _upper_candidate(self):
        if not self._entered_command_applied:
            return self.config.upper.default_position.copy(), None, "DEFAULT"
        if self.streamed_upper:
            latest = self.upper_targets.latest()
            if latest is not None and latest.sequence != self.committed_upper_sequence:
                return np.asarray(latest.q_des, dtype=np.float64), latest.sequence, "TRACKING"
        return self.upper_position.copy(), self.committed_upper_sequence, "HOLDING"

    def step(self, frame):
        if not self._active:
            raise RuntimeError("motion state must be entered before stepping")
        if self._pending is not None:
            raise RuntimeError("previous motion command needs commit or reject")
        upper_position, upper_sequence, phase = self._upper_candidate()
        snapshot = self.adapter.snapshot() if self._has_snapshot else None
        self._pending = (snapshot, upper_position.copy(), upper_sequence, phase)
        try:
            action, observation = self.adapter.infer(frame, self.last_action.copy())
            action = np.asarray(action, dtype=np.float64)
            observation = np.asarray(observation, dtype=np.float64)
            robot = self.config.robot
            if action.shape != (len(robot.lower_joints),) or not np.all(np.isfinite(action)):
                raise ValueError("lower policy action must match lower_joints and contain finite values")
            if observation.ndim != 1 or not np.all(np.isfinite(observation)):
                raise ValueError("lower policy observation must be a finite vector")
            requested = robot.default_position.copy()
            requested[robot.lower_joints] += robot.action_scale[robot.lower_joints] * action
            requested[robot.upper_joints] = upper_position
            if not np.all(np.isfinite(requested)):
                raise ValueError("lower policy target overflow")
            target = np.clip(requested, robot.position_min, robot.position_max)
            raw_action = (requested - robot.default_position) / robot.action_scale
            if not np.all(np.isfinite(raw_action)):
                raise ValueError("motion normalized action overflow")
            self._last_diagnostics = self._diagnostics(
                observation, raw_action, np.abs(requested - target)
            )
            zeros = np.zeros(self.n)
            command = JointCommand(target, zeros, self.config.control.kp, self.config.control.kd, zeros)
            return ControlResult(command, self._last_diagnostics, substate=phase)
        except Exception:
            self.on_command_rejected()
            raise

    def on_command_applied(self, command):
        if self._pending is None:
            raise RuntimeError("no pending motion command")
        actual = np.asarray(command.q_des, dtype=np.float64)
        if actual.shape != (self.n,) or not np.all(np.isfinite(actual)):
            raise ValueError("applied command must match motion joint dimension")
        action = (actual - self.default_pose) / self.action_scale
        if not np.all(np.isfinite(action)):
            raise ValueError("applied normalized action must be finite")
        _, upper_position, upper_sequence, phase = self._pending
        self.last_action = action.copy()
        self.upper_position = upper_position.copy()
        self.committed_upper_sequence = upper_sequence
        self.phase = phase
        self._entered_command_applied = True
        self._pending = None
        self._last_diagnostics = replace(self._last_diagnostics, executed_action=self.last_action.copy())

    def on_command_rejected(self):
        if self._pending is not None:
            snapshot, _, _, _ = self._pending
            if self._has_snapshot:
                self.adapter.restore(snapshot)
            self._pending = None

    def diagnostics_after_command(self, diagnostics):
        return self._last_diagnostics

    def _diagnostics(self, observation, raw_action, excess):
        return MotionDiagnostics(
            str(getattr(self.adapter, "observation_kind", "lower_loco")),
            str(getattr(self.adapter, "policy_model", self.config.lower.model)),
            np.asarray(observation).copy(),
            np.asarray(raw_action).copy(),
            self.last_action.copy(),
            excess > 1e-9,
            excess.copy(),
        )


class LowerLocoStreamState(LowerLocoState):
    """Apply the newest upper-joint target after the entry posture is accepted.

    Producers publish activation-scoped JointTargetFrame values to upper_targets.
    With no newer frame, the last accepted target remains the commanded target;
    exiting invalidates producers and entering starts from the configured posture.
    """

    streamed_upper = True
