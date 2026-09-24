"""SONIC clip reference: NPZ loading and 640-float tokenizer prefix generation.

Mirrors the SONIC C++ ``A3CsvMotionReference`` (Apache License 2.0,
``gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/a3_deploy/
a3_csv_motion_reference.cpp``): future frames are clamped to the final frame
once the clip ends (``hold_last``), and the anchor is the yaw-locked reference
root orientation relative to the current pelvis quaternion.

The NPZ contract (produced by ``tools/convert_sonic_clip.py``):
  q_ref           [T,29] float64, absolute joint positions, IsaacLab order
  dq_ref          [T,29] float64, joint velocities, IsaacLab order
  root_quat_wxyz  [T,4]  float64, reference pelvis orientation
  dt              scalar, frame spacing in seconds (0.02)
  joint_names     [29] strings, must equal ISAACLAB_JOINT_NAMES
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .contract import (
    COMMAND_DIM,
    FUTURE_FRAMES,
    ISAACLAB_JOINT_NAMES,
    JOINT_DIM,
    POLICY_DT,
    TOKENIZER_DIM,
    anchor_6d,
    compute_yaw_offset,
)


def _numbers(value, name, shape):
    array = np.asarray(value)
    if array.dtype.kind not in "fiu" or (shape is not None and array.shape != shape):
        raise ValueError(f"{name} has an invalid numeric shape or dtype")
    array = array.astype(np.float64)
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be finite")
    return array


class SonicClip:
    """One immutable reference clip on the 50 Hz policy timeline."""

    def __init__(self, source, q_ref, dq_ref, root_quat_wxyz, dt):
        self.source = source
        self.q_ref = q_ref
        self.dq_ref = dq_ref
        self.root_quat_wxyz = root_quat_wxyz
        self.dt = dt

    @property
    def frames(self):
        return self.q_ref.shape[0]

    @property
    def duration_s(self):
        return (self.frames - 1) * self.dt

    @classmethod
    def load(cls, path):
        if not isinstance(path, (str, Path)) or "://" in str(path) or not str(path).strip():
            raise ValueError("clip requires an explicit filesystem path")
        source = Path(path).expanduser().resolve()
        if not source.is_file():
            raise FileNotFoundError(source)
        required = {"q_ref", "dq_ref", "root_quat_wxyz", "dt", "joint_names"}
        with np.load(source, allow_pickle=False) as data:
            if not required.issubset(data.files):
                raise ValueError(f"clip is missing fields: {sorted(required - set(data.files))}")
            names = np.asarray(data["joint_names"])
            if names.shape != (JOINT_DIM,) or names.dtype.kind not in "US":
                raise ValueError("joint_names must be a 29-string array")
            if tuple(names.astype(str)) != ISAACLAB_JOINT_NAMES:
                raise ValueError("clip joint_names must match the IsaacLab policy order")
            dt = float(_numbers(data["dt"], "dt", ()))
            if dt <= 0 or abs(dt - POLICY_DT) > 1e-9:
                raise ValueError("clip dt must be 0.02 (50 Hz policy timeline)")
            q_ref = _numbers(data["q_ref"], "q_ref", None)
            if q_ref.ndim != 2 or q_ref.shape[1] != JOINT_DIM or q_ref.shape[0] < 1:
                raise ValueError("q_ref must have shape [T,29] with T >= 1")
            frames = q_ref.shape[0]
            dq_ref = _numbers(data["dq_ref"], "dq_ref", (frames, JOINT_DIM))
            quats = _numbers(data["root_quat_wxyz"], "root_quat_wxyz", (frames, 4))
            norms = np.linalg.norm(quats, axis=1)
            if np.any(norms < 1e-6):
                raise ValueError("root_quat_wxyz must be nonzero")
            quats = quats / norms[:, None]
        result = cls(
            source,
            q_ref, dq_ref, quats, dt,
        )
        for array in (result.q_ref, result.dq_ref, result.root_quat_wxyz):
            array.setflags(write=False)
        return result

    def lock_yaw(self, robot_root_quat_wxyz):
        """Yaw offset aligning the clip's first frame to the current IMU yaw."""
        return compute_yaw_offset(robot_root_quat_wxyz, self.root_quat_wxyz[0])

    def tokenizer_slice(self, tick_idx, robot_root_quat_wxyz, yaw_offset_rad,
                        future_frame_skip):
        """Build the 640-float tokenizer prefix at ``tick_idx``.

        [0:290]   q_ref for 10 future frames (frame-major, IsaacLab order)
        [290:580] dq_ref for the same frames
        [580:640] 10 x 6D anchor of the yaw-locked reference root orientation
                  relative to the current pelvis quaternion
        Frame indices beyond the clip clamp to the final frame (hold_last).
        """
        if tick_idx < 0 or future_frame_skip < 1:
            raise ValueError("invalid tick or future frame skip")
        out = np.zeros(TOKENIZER_DIM, dtype=np.float64)
        last = self.frames - 1
        for k in range(FUTURE_FRAMES):
            idx = min(tick_idx + k * future_frame_skip, last)
            out[k * JOINT_DIM:(k + 1) * JOINT_DIM] = self.q_ref[idx]
            start = COMMAND_DIM // 2 + k * JOINT_DIM
            out[start:start + JOINT_DIM] = self.dq_ref[idx]
            anchor = anchor_6d(robot_root_quat_wxyz, self.root_quat_wxyz[idx],
                               yaw_offset_rad)
            offset = COMMAND_DIM + k * 6
            out[offset:offset + 6] = anchor
        return out.astype(np.float32)
