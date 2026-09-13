"""A3 lower-body velocity model and its deployment observation contract.

The actor observes 29 joints and predicts the first 15 (legs and waist).
Its 25-value command context is a fixed training ABI: velocity occupies
indices 5:8, while the remaining values are model-defined placeholders.
No planner, event, ball or task-state input is required at deployment.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np

from cadence_config import resolve_resource

from cadence.inference import OnnxPolicy, OnnxRuntimeOptions


ACTION_DIM = 29
LOWER_ACTION_DIM = 15
LOWER_VELOCITY_OBSERVATION_DIM = 118
LOWER_VELOCITY_H4_OBSERVATION_DIM = 397
LOWER_VELOCITY_H4_HISTORY_FRAMES = 4
OBSERVATION_CLIP = 100.0

# Training reference pose, in the model's 29-joint order. The separately
# configured upper-body command pose must not silently change this reference.
DEFAULT_JOINT_POSITION = (
    -0.1311, 0.0056, -0.0348, 0.2468, -0.1204, -0.0078,
    -0.1311, -0.0056, 0.0348, 0.2468, -0.1204, 0.0078,
    0.0, 0.0, 0.0, 0.3, 0.12, 0.0, 0.8, 0.0, 0.0, 0.0,
    0.3, -0.12, 0.0, 0.8, 0.0, 0.0, 0.0,
)


def _vector(values, size: int, name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def _projected_gravity(quaternion_wxyz) -> tuple[float, float, float]:
    w, x, y, z = _vector(quaternion_wxyz, 4, "quaternion_wxyz")
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-6:
        raise ValueError("quaternion norm is zero")
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    return (2.0 * (-z*x + w*y), -2.0 * (z*y + w*x), 1.0 - 2.0 * (w*w + z*z))


def lower_velocity_actor_command(
    vx_mps: float, vy_mps: float, yaw_radps: float,
) -> tuple[float, ...]:
    velocity = _vector((vx_mps, vy_mps, yaw_radps), 3, "velocity command")
    return (
        1.0, 0.0, 0.0,
        0.0,
        0.0,
        *velocity,
        0.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0, 0.0,
    )


class LowerVelocityObservationBuilder:
    """Single-frame [gyro, gravity, relative q, dq, executed action, context]."""

    def __init__(self, default_joint_position=DEFAULT_JOINT_POSITION) -> None:
        self.default_joint_position = _vector(
            default_joint_position, ACTION_DIM, "default_joint_position"
        )
        self.observation_dimension = LOWER_VELOCITY_OBSERVATION_DIM

    def reset(self) -> None:
        pass

    def snapshot(self):
        return ()

    def restore(self, snapshot) -> None:
        if snapshot != ():
            raise ValueError("single-frame observation snapshot must be empty")

    def _terms(self, gyro_b, quaternion_wxyz, joint_pos, joint_vel, last_action):
        gyro = _vector(gyro_b, 3, "gyro_b")
        gravity = _projected_gravity(quaternion_wxyz)
        q = _vector(joint_pos, ACTION_DIM, "joint_pos")
        dq = _vector(joint_vel, ACTION_DIM, "joint_vel")
        action = _vector(last_action, ACTION_DIM, "last_executed_action")
        q_rel = tuple(
            value - default for value, default in zip(q, self.default_joint_position)
        )
        return gyro, gravity, q_rel, dq, action

    def build(
        self, gyro_b, quaternion_wxyz, joint_pos, joint_vel,
        last_executed_action, command,
    ) -> tuple[float, ...]:
        terms = self._terms(
            gyro_b, quaternion_wxyz, joint_pos, joint_vel, last_executed_action
        )
        context = _vector(command, 25, "lower velocity command")
        return tuple(
            max(-OBSERVATION_CLIP, min(OBSERVATION_CLIP, value))
            for term in (*terms, context) for value in term
        )


class LowerVelocityH4ObservationBuilder(LowerVelocityObservationBuilder):
    """Four frames per term, oldest first; append the current 25-D context.

    The first measurement fills all four history slots after reset. History
    snapshots let the state roll back a control tick that was not executed.
    """

    def __init__(self, default_joint_position=DEFAULT_JOINT_POSITION) -> None:
        super().__init__(default_joint_position)
        self.history_length = LOWER_VELOCITY_H4_HISTORY_FRAMES
        self.observation_dimension = LOWER_VELOCITY_H4_OBSERVATION_DIM
        self.reset()

    def reset(self) -> None:
        self._histories = [deque(maxlen=self.history_length) for _ in range(5)]

    def snapshot(self):
        return tuple(tuple(history) for history in self._histories)

    def restore(self, snapshot) -> None:
        if len(snapshot) != 5:
            raise ValueError("observation history snapshot has wrong term count")
        histories = []
        for frames, size in zip(snapshot, (3, 3, 29, 29, 29)):
            if len(frames) not in (0, self.history_length):
                raise ValueError("observation history snapshot has wrong frame count")
            histories.append(deque(
                (_vector(frame, size, "history frame") for frame in frames),
                maxlen=self.history_length,
            ))
        if len({len(history) for history in histories}) != 1:
            raise ValueError("observation history snapshot has inconsistent terms")
        self._histories = histories

    def build(
        self, gyro_b, quaternion_wxyz, joint_pos, joint_vel,
        last_action, command,
    ) -> tuple[float, ...]:
        terms = self._terms(
            gyro_b, quaternion_wxyz, joint_pos, joint_vel, last_action
        )
        context = _vector(command, 25, "lower velocity command")
        for history, term in zip(self._histories, terms):
            if not history:
                history.extend([term] * self.history_length)
            else:
                history.append(term)
        observation = [
            max(-OBSERVATION_CLIP, min(OBSERVATION_CLIP, value))
            for history in self._histories for term in history for value in term
        ]
        observation.extend(
            max(-OBSERVATION_CLIP, min(OBSERVATION_CLIP, value)) for value in context
        )
        return tuple(observation)


@dataclass(frozen=True, slots=True)
class _PolicyConfig:
    """Structural interface accepted by an optional application session cache."""

    model: Path
    policy_key: str
    runtime: OnnxRuntimeOptions


@dataclass(frozen=True, slots=True)
class _PolicySpec:
    key: str = "velocity_lower_h4"
    observation_key: str = "velocity_lower_h4_397"
    observation_dimension: int = LOWER_VELOCITY_H4_OBSERVATION_DIM
    action_dimension: int = LOWER_ACTION_DIM


class A3LowerPolicy:
    """Adapt motion frames to the A3 H4 actor, independent of task packages.

    An optional ``services.policies.load(config)`` session cache may supply a
    runner and its shape specification. Otherwise the adapter loads its own
    Cadence ONNX runner. Observation history always belongs to this adapter.
    """

    observation_kind = "velocity_lower_h4"

    def __init__(self, config, services) -> None:
        robot = config.robot
        if len(robot.default_position) != ACTION_DIM:
            raise ValueError("A3 lower policy requires the 29-joint robot contract")
        if tuple(robot.lower_joints) != tuple(range(LOWER_ACTION_DIM)) or tuple(
            robot.upper_joints
        ) != tuple(range(LOWER_ACTION_DIM, ACTION_DIM)):
            raise ValueError("A3 lower policy requires legs/waist first, then 14 arm joints")
        if config.lower.history_frames != LOWER_VELOCITY_H4_HISTORY_FRAMES:
            raise ValueError("A3 lower policy requires history_frames=4")
        self.config = config
        options = config.lower.runtime
        if not isinstance(options, OnnxRuntimeOptions):
            options = OnnxRuntimeOptions(**dict(options))
        model = resolve_resource(
            config.lower.model, base=getattr(services, "repository", None)
        )
        self.policy_config = _PolicyConfig(model, "velocity_lower_h4", options)
        self.policy_model = str(model)
        self.policy_spec = _PolicySpec()
        loader = getattr(getattr(services, "policies", None), "load", None)
        if loader is None:
            self.policy = OnnxPolicy(
                model, LOWER_VELOCITY_H4_OBSERVATION_DIM, LOWER_ACTION_DIM, options
            )
            # Session initialization and first inference happen before a state
            # enters the control loop. Application caches may warm their own
            # runners when load() is called.
            self.policy.infer(np.zeros(LOWER_VELOCITY_H4_OBSERVATION_DIM, dtype=np.float32))
        else:
            self.policy_spec, self.policy = loader(self.policy_config)
            if (
                self.policy_spec.observation_dimension != LOWER_VELOCITY_H4_OBSERVATION_DIM
                or self.policy_spec.action_dimension != LOWER_ACTION_DIM
            ):
                raise ValueError("A3 lower policy requires 397 observations and 15 actions")
        self.observations = LowerVelocityH4ObservationBuilder(robot.default_position)

    def reset(self) -> None:
        self.observations.reset()

    def snapshot(self):
        return self.observations.snapshot()

    def restore(self, snapshot) -> None:
        self.observations.restore(snapshot)

    def infer(self, frame, last_action):
        velocity = getattr(frame, "velocity_command", None)
        if velocity is None:
            velocity = getattr(frame, "loco_velocity", (0.0, 0.0, 0.0))
        command = lower_velocity_actor_command(*velocity)
        robot = frame.robot_state
        q = np.asarray(_vector(robot.joint_pos, ACTION_DIM, "joint_pos"))
        dq = np.asarray(_vector(robot.joint_vel, ACTION_DIM, "joint_vel"))
        executed = np.asarray(_vector(last_action, ACTION_DIM, "last_action"))
        if self.config.lower.mask_upper_observation:
            upper = self.config.robot.upper_joints
            q[upper] = np.asarray(self.config.robot.default_position)[upper]
            dq[upper] = 0.0
            executed[upper] = 0.0
        observation = self.observations.build(
            robot.gyro_b, robot.quaternion_wxyz, q, dq, executed, command
        )
        action = np.asarray(self.policy.infer(observation), dtype=np.float32)
        if action.shape != (LOWER_ACTION_DIM,) or not np.all(np.isfinite(action)):
            raise RuntimeError("A3 lower policy must return 15 finite actions")
        return action, observation
