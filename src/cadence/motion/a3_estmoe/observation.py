"""H32 lower-only observation contract for the A3 EstMoE velocity actor."""

from __future__ import annotations

from collections import deque

from ..a3 import (
    DEFAULT_JOINT_POSITION,
    LOWER_ACTION_DIM,
    OBSERVATION_CLIP,
    _projected_gravity,
    _vector,
)

LOWER_VELOCITY_H32_HISTORY_FRAMES = 32
LOWER_VELOCITY_H32_TERM_WIDTHS = (3, 3, LOWER_ACTION_DIM, LOWER_ACTION_DIM, LOWER_ACTION_DIM)
LOWER_VELOCITY_H32_OBSERVATION_DIM = (
    sum(LOWER_VELOCITY_H32_TERM_WIDTHS) * LOWER_VELOCITY_H32_HISTORY_FRAMES + 3
)


class LowerVelocityH32ObservationBuilder:
    """H32 term-major history over the 15 lower joints, then the 3-D command.

    EstMoE deployment contract: gyro, projected gravity, default-relative
    joint position, joint velocity and executed action, each oldest first
    over 32 frames, followed by the current velocity command verbatim. Input
    scaling lives inside the model; this builder only validates and clips.
    """

    def __init__(self, default_lower_position=None) -> None:
        if default_lower_position is None:
            default_lower_position = DEFAULT_JOINT_POSITION[:LOWER_ACTION_DIM]
        self.default_lower_position = _vector(
            default_lower_position, LOWER_ACTION_DIM, "default_lower_position"
        )
        self.history_length = LOWER_VELOCITY_H32_HISTORY_FRAMES
        self.observation_dimension = LOWER_VELOCITY_H32_OBSERVATION_DIM
        self.reset()

    def reset(self) -> None:
        self._histories = [deque(maxlen=self.history_length) for _ in range(5)]

    def snapshot(self):
        return tuple(tuple(history) for history in self._histories)

    def restore(self, snapshot) -> None:
        if len(snapshot) != len(LOWER_VELOCITY_H32_TERM_WIDTHS):
            raise ValueError("observation history snapshot has wrong term count")
        histories = []
        for frames, size in zip(snapshot, LOWER_VELOCITY_H32_TERM_WIDTHS):
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
