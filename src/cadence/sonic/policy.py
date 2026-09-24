"""SONIC whole-body policy adapter: observation assembly and action decode.

Mirrors the SONIC C++ ``A3ObsBuilder`` and ``DecodeAction`` (Apache License
2.0, ``gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/a3_deploy/
a3_obs_builder.cpp`` and ``a3_action_decoder.cpp``):

- Proprio history is 10 frames, term-major, oldest to newest:
  base_ang_vel(3), joint_pos_rel(29, q - default), joint_vel_rel(29),
  last_raw_action(29, the clipped raw policy action), gravity_dir(3).
- joint_pos_rel is computed in float32 in IsaacLab order, matching training.
- The 640-float tokenizer prefix (from a clip or a stream) is prepended
  verbatim. No normalization is applied anywhere.
- Decode: raw action clipped to +/-20, q_des = raw * scale + default in
  IsaacLab order, permuted to cadence order and clipped to the configured
  joint position limits. Non-finite raw actions decay to zero before decode.

History mutates only inside ``build_observation``; the owning state snapshots
before stepping and restores on a rejected command, and commits the clipped
raw action only from ``on_command_applied``.
"""

from __future__ import annotations

from collections import deque
from numbers import Integral
from pathlib import Path

import numpy as np

from cadence.inference import OnnxPolicy, OnnxRuntimeOptions

from .contract import (
    ACTION_CLIP,
    ACTION_SCALE_024_ISAACLAB,
    CADENCE_JOINT_NAMES,
    DEFAULT_ANGLES_ISAACLAB,
    JOINT_DIM,
    OBSERVATION_DIM,
    PROPRIO_HISTORY,
    PROPRIO_TERMS,
    TOKENIZER_DIM,
    cadence_to_isaaclab,
    gravity_direction,
    isaaclab_to_cadence,
)


class SonicProprioHistory:
    """Term-major 10-frame proprioception history in IsaacLab order."""

    def __init__(self, history_frames=PROPRIO_HISTORY):
        if isinstance(history_frames, bool) or not isinstance(history_frames, Integral) or history_frames < 1:
            raise ValueError("history_frames must be a positive integer")
        self.history_length = int(history_frames)
        self.reset()

    def reset(self):
        self._histories = [deque(maxlen=self.history_length) for _ in PROPRIO_TERMS]

    def snapshot(self):
        return tuple(
            tuple(np.asarray(frame, dtype=np.float32).copy() for frame in history)
            for history in self._histories
        )

    def restore(self, snapshot):
        if len(snapshot) != len(PROPRIO_TERMS):
            raise ValueError("observation history snapshot has wrong term count")
        histories = []
        for frames, (_, width) in zip(snapshot, PROPRIO_TERMS):
            if len(frames) not in (0, self.history_length):
                raise ValueError("observation history snapshot has wrong frame count")
            restored = deque(maxlen=self.history_length)
            for frame in frames:
                array = np.asarray(frame, dtype=np.float32)
                if array.shape != (width,) or not np.all(np.isfinite(array)):
                    raise ValueError("observation history snapshot has invalid frames")
                restored.append(array.copy())
            histories.append(restored)
        if len({len(history) for history in histories}) != 1:
            raise ValueError("observation history snapshot has inconsistent terms")
        self._histories = histories

    def push(self, gyro_b, quaternion_wxyz, joint_pos_isaaclab, joint_vel_isaaclab,
             last_raw_action_isaaclab):
        """Append one measurement frame; the first push seeds every slot."""
        gyro = np.asarray(gyro_b, dtype=np.float64).reshape(3)
        quat = np.asarray(quaternion_wxyz, dtype=np.float64).reshape(4)
        q = np.asarray(joint_pos_isaaclab, dtype=np.float64).reshape(JOINT_DIM)
        dq = np.asarray(joint_vel_isaaclab, dtype=np.float64).reshape(JOINT_DIM)
        action = np.asarray(last_raw_action_isaaclab, dtype=np.float64).reshape(JOINT_DIM)
        if not np.all(np.isfinite(gyro)) or not np.all(np.isfinite(quat)):
            raise ValueError("gyro and quaternion must be finite")
        if np.linalg.norm(quat) < 1e-6:
            raise ValueError("quaternion_wxyz must be non-zero")
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)) or not np.all(np.isfinite(action)):
            raise ValueError("joint state and last action must be finite")
        terms = (
            gyro.astype(np.float32),
            (q.astype(np.float32) - np.asarray(DEFAULT_ANGLES_ISAACLAB, dtype=np.float32)),
            dq.astype(np.float32),
            np.clip(action, -ACTION_CLIP, ACTION_CLIP).astype(np.float32),
            gravity_direction(quat).astype(np.float32),
        )
        for history, term in zip(self._histories, terms):
            if not history:
                history.extend([term.copy() for _ in range(self.history_length)])
            else:
                history.append(term)

    def build(self):
        """Emit the 930-float term-major history, oldest frame first."""
        blocks = []
        for history, (_, width) in zip(self._histories, PROPRIO_TERMS):
            if not history:
                blocks.append(np.zeros(self.history_length * width, dtype=np.float32))
            else:
                blocks.append(np.concatenate([np.asarray(f, dtype=np.float32) for f in history]))
        return np.concatenate(blocks)


class SonicWholeBodyPolicy:
    """Adapt cadence frames to the released SONIC 035 a3_fast actor."""

    observation_kind = "sonic_whole_body_h10"

    def __init__(self, model_path, runtime_options=None, position_min_cadence=None,
                 position_max_cadence=None, services=None):
        if "://" in str(model_path):
            raise ValueError("model requires an explicit filesystem path")
        model = Path(model_path).expanduser()
        if not model.is_absolute():
            raise ValueError("model must be resolved relative to its declaring configuration")
        model = model.resolve()
        if not model.is_file():
            raise FileNotFoundError(model)
        if not isinstance(runtime_options, OnnxRuntimeOptions):
            runtime_options = OnnxRuntimeOptions(**dict(runtime_options or {}))
        self.policy_model = str(model)
        self.policy = OnnxPolicy(model, OBSERVATION_DIM, JOINT_DIM, runtime_options)
        # Session initialization and first inference happen before a state
        # enters the control loop.
        self.policy.infer(np.zeros(OBSERVATION_DIM, dtype=np.float32))
        if position_min_cadence is None or position_max_cadence is None:
            raise ValueError("sonic policy requires explicit cadence-order joint limits")
        low = np.asarray(position_min_cadence, dtype=np.float64).reshape(JOINT_DIM)
        high = np.asarray(position_max_cadence, dtype=np.float64).reshape(JOINT_DIM)
        if not np.all(np.isfinite(low)) or not np.all(np.isfinite(high)) or np.any(low > high):
            raise ValueError("invalid joint position limits")
        self.position_min = low
        self.position_max = high
        joint_names = getattr(services, "joint_names", None)
        if joint_names is not None and tuple(joint_names) != CADENCE_JOINT_NAMES:
            raise ValueError("runtime joint order must match the canonical A3 cadence order")
        self.default_isaaclab = np.asarray(DEFAULT_ANGLES_ISAACLAB, dtype=np.float64)
        self.action_scale_isaaclab = np.asarray(ACTION_SCALE_024_ISAACLAB, dtype=np.float64)
        self.default_cadence = isaaclab_to_cadence(self.default_isaaclab)
        self.observations = SonicProprioHistory()

    def reset(self):
        self.observations.reset()

    def snapshot(self):
        return self.observations.snapshot()

    def restore(self, snapshot):
        self.observations.restore(snapshot)

    def infer(self, robot_state, last_raw_action_isaaclab, tokenizer_prefix):
        """Push one proprio frame, run the actor, decode to cadence targets.

        Returns (q_des_cadence, q_requested_cadence, raw_clipped_isaaclab,
        observation): the limit-clipped command target, the pre-limit-clip
        target, and the clipped raw policy action the caller commits as the
        next tick's last action only after the command is applied.
        """
        prefix = np.asarray(tokenizer_prefix, dtype=np.float32).reshape(TOKENIZER_DIM)
        if not np.all(np.isfinite(prefix)):
            raise ValueError("tokenizer prefix must be finite")
        self.observations.push(
            robot_state.gyro_b,
            robot_state.quaternion_wxyz,
            cadence_to_isaaclab(robot_state.joint_pos),
            cadence_to_isaaclab(robot_state.joint_vel),
            last_raw_action_isaaclab,
        )
        observation = np.concatenate([prefix, self.observations.build()])
        if observation.shape != (OBSERVATION_DIM,) or not np.all(np.isfinite(observation)):
            raise RuntimeError("sonic observation must be a finite 1570-vector")
        raw = np.asarray(self.policy.infer(observation), dtype=np.float32)
        if raw.shape != (JOINT_DIM,):
            raise RuntimeError("sonic policy must return 29 actions")
        raw_clipped = np.clip(np.nan_to_num(raw, nan=0.0, posinf=ACTION_CLIP, neginf=-ACTION_CLIP),
                              -ACTION_CLIP, ACTION_CLIP).astype(np.float64)
        requested = isaaclab_to_cadence(
            raw_clipped * self.action_scale_isaaclab + self.default_isaaclab)
        q_des = np.clip(requested, self.position_min, self.position_max)
        if not np.all(np.isfinite(q_des)):
            raise RuntimeError("sonic decoded command must be finite")
        return q_des, requested, raw_clipped, observation
