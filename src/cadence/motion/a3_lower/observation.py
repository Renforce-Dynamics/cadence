"""Lower-joint velocity observation history for A3 lower-only actors.

Shared by the H32 EstMoE and H4 MLP deployment contracts: gyro, projected
gravity, default-relative joint position, joint velocity and executed action,
each term oldest first, followed by the current velocity command verbatim.
Input scaling or normalization lives inside each model; this builder only
validates and clips.
"""

from __future__ import annotations

from collections import deque
from numbers import Integral

from ..a3 import (
    DEFAULT_JOINT_POSITION,
    LOWER_ACTION_DIM,
    OBSERVATION_CLIP,
    _projected_gravity,
    _vector,
)

LOWER_VELOCITY_TERM_WIDTHS = (3, 3, LOWER_ACTION_DIM, LOWER_ACTION_DIM, LOWER_ACTION_DIM)
LOWER_VELOCITY_H32_HISTORY_FRAMES = 32
LOWER_VELOCITY_MLP_H4_HISTORY_FRAMES = 4


def lower_velocity_observation_dim(history_frames: int) -> int:
    return sum(LOWER_VELOCITY_TERM_WIDTHS) * history_frames + 3


LOWER_VELOCITY_H32_OBSERVATION_DIM = lower_velocity_observation_dim(
    LOWER_VELOCITY_H32_HISTORY_FRAMES
)
LOWER_VELOCITY_MLP_H4_OBSERVATION_DIM = lower_velocity_observation_dim(
    LOWER_VELOCITY_MLP_H4_HISTORY_FRAMES
)


class LowerVelocityHistoryObservationBuilder:
    """Term-major history over the 15 lower joints, then the 3-D command."""

    def __init__(self, default_lower_position=None, *, history_frames) -> None:
        if (
            isinstance(history_frames, bool)
            or not isinstance(history_frames, Integral)
            or history_frames < 1
        ):
            raise ValueError("history_frames must be a positive integer")
        if default_lower_position is None:
            default_lower_position = DEFAULT_JOINT_POSITION[:LOWER_ACTION_DIM]
        self.default_lower_position = _vector(
            default_lower_position, LOWER_ACTION_DIM, "default_lower_position"
        )
        self.history_length = int(history_frames)
        self.observation_dimension = lower_velocity_observation_dim(self.history_length)
        self.reset()

    def reset(self) -> None:
        self._histories = [deque(maxlen=self.history_length) for _ in range(5)]

    def snapshot(self):
        return tuple(tuple(history) for history in self._histories)

    def restore(self, snapshot) -> None:
        if len(snapshot) != len(LOWER_VELOCITY_TERM_WIDTHS):
            raise ValueError("observation history snapshot has wrong term count")
        histories = []
        for frames, size in zip(snapshot, LOWER_VELOCITY_TERM_WIDTHS):
            if len(frames) not in (0, self.history_length):
                raise ValueError("observation history snapshot has wrong frame count")
            histories.append(deque(
                (_vector(frame, size, "history frame") for frame in frames),
                maxlen=self.history_length,
            ))
        if len({len(history) for history in histories}) != 1:
            raise ValueError("observation history snapshot has inconsistent terms")
        self._histories = histories

    def _terms(self, gyro_b, quaternion_wxyz, joint_pos, joint_vel, last_action):
        gyro = _vector(gyro_b, 3, "gyro_b")
        gravity = _projected_gravity(quaternion_wxyz)
        q = _vector(joint_pos, LOWER_ACTION_DIM, "joint_pos")
        dq = _vector(joint_vel, LOWER_ACTION_DIM, "joint_vel")
        action = _vector(last_action, LOWER_ACTION_DIM, "last_executed_action")
        q_rel = tuple(
            value - default
            for value, default in zip(q, self.default_lower_position)
        )
        return gyro, gravity, q_rel, dq, action

    def build(
        self, gyro_b, quaternion_wxyz, joint_pos, joint_vel,
        last_executed_action, command,
    ) -> tuple[float, ...]:
        terms = self._terms(
            gyro_b, quaternion_wxyz, joint_pos, joint_vel, last_executed_action
        )
        context = _vector(command, 3, "lower velocity command")
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
