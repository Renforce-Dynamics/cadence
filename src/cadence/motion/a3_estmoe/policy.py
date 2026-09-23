"""A3 H32 lower-only EstMoE policy adapter for the generic motion states."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from ..a3 import ACTION_DIM, LOWER_ACTION_DIM, _PolicyConfig, _PolicySpec, _vector
from cadence.inference import OnnxPolicy, OnnxRuntimeOptions
from .observation import (
    LOWER_VELOCITY_H32_HISTORY_FRAMES,
    LOWER_VELOCITY_H32_OBSERVATION_DIM,
    LowerVelocityH32ObservationBuilder,
)


class A3LowerEstMoEPolicy:
    """Adapt motion frames to the A3 H32 lower-only EstMoE actor.

    The actor observes legs and waist only, so arm joints never enter the
    observation and no masking option exists. The 3-value velocity command is
    appended directly instead of the legacy 25-value context ABI.
    Observation history always belongs to this adapter.
    """

    observation_kind = "velocity_lower_h32"

    def __init__(self, config, services) -> None:
        robot = config.robot
        if len(robot.default_position) != ACTION_DIM:
            raise ValueError("A3 lower EstMoE policy requires the 29-joint robot contract")
        if tuple(robot.lower_joints) != tuple(range(LOWER_ACTION_DIM)) or tuple(
            robot.upper_joints
        ) != tuple(range(LOWER_ACTION_DIM, ACTION_DIM)):
            raise ValueError(
                "A3 lower EstMoE policy requires legs/waist first, then 14 arm joints"
            )
        if config.lower.history_frames != LOWER_VELOCITY_H32_HISTORY_FRAMES:
            raise ValueError("A3 lower EstMoE policy requires history_frames=32")
        self.config = config
        options = config.lower.runtime
        if not isinstance(options, OnnxRuntimeOptions):
            options = OnnxRuntimeOptions(**dict(options))
        if "://" in str(config.lower.model):
            raise ValueError("lower.model requires an explicit filesystem path")
        model = Path(config.lower.model).expanduser()
        if not model.is_absolute():
            raise ValueError(
                "lower.model must be resolved relative to its declaring configuration"
            )
        model = model.resolve()
        if not model.is_file():
            raise FileNotFoundError(model)
        self.policy_config = _PolicyConfig(model, "velocity_lower_h32", options)
        self.policy_model = str(model)
        self.policy_spec = _PolicySpec(
            key="velocity_lower_h32",
            observation_key="velocity_lower_h32_1635",
            observation_dimension=LOWER_VELOCITY_H32_OBSERVATION_DIM,
            action_dimension=LOWER_ACTION_DIM,
        )
        loader = getattr(getattr(services, "policies", None), "load", None)
        if loader is None:
            self.policy = OnnxPolicy(
                model, LOWER_VELOCITY_H32_OBSERVATION_DIM, LOWER_ACTION_DIM, options
            )
            # Session initialization and first inference happen before a state
            # enters the control loop. Application caches may warm their own
            # runners when load() is called.
            self.policy.infer(np.zeros(LOWER_VELOCITY_H32_OBSERVATION_DIM, dtype=np.float32))
        else:
            self.policy_spec, self.policy = loader(self.policy_config)
            if (
                self.policy_spec.observation_dimension != LOWER_VELOCITY_H32_OBSERVATION_DIM
                or self.policy_spec.action_dimension != LOWER_ACTION_DIM
            ):
                raise ValueError(
                    "A3 lower EstMoE policy requires 1635 observations and 15 actions"
                )
        self.observations = LowerVelocityH32ObservationBuilder(
            np.asarray(robot.default_position)[:LOWER_ACTION_DIM]
        )

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
        command = _vector(velocity, 3, "lower velocity command")
        robot = frame.robot_state
        q = _vector(robot.joint_pos, ACTION_DIM, "joint_pos")[:LOWER_ACTION_DIM]
        dq = _vector(robot.joint_vel, ACTION_DIM, "joint_vel")[:LOWER_ACTION_DIM]
        executed = _vector(last_action, ACTION_DIM, "last_action")[:LOWER_ACTION_DIM]
        observation = self.observations.build(
            robot.gyro_b, robot.quaternion_wxyz, q, dq, executed, command
        )
        action = np.asarray(self.policy.infer(observation), dtype=np.float32)
        if action.shape != (LOWER_ACTION_DIM,) or not np.all(np.isfinite(action)):
            raise RuntimeError("A3 lower EstMoE policy must return 15 finite actions")
        return action, observation
