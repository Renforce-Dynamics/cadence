#!/usr/bin/env python3
"""Convert an iKobe A3 motion NPZ into a sonic_clip NPZ resource.

The iKobe training-side motions (``src/ikobe/data/motions/**.npz``) carry
``joint_pos``/``joint_vel`` [T,29] in the cadence canonical joint order
(``CADENCE_JOINT_NAMES``), a wxyz ``root_quat`` and an ``fps`` scalar (60 Hz
for the balldata/palm_rebind family). They are resampled to the 50 Hz policy
timeline with linear interpolation and slerp; joint velocities come from
forward finite differences on the resampled timeline (last frame repeats the
previous difference), matching ``convert_sonic_clip.py`` / the SONIC C++
loader. Root translation, ball and intent channels are training-side data and
are not part of the clip contract.

Output NPZ fields (identical to convert_sonic_clip.py):
  q_ref [T,29] and dq_ref [T,29] in IsaacLab policy order, root_quat_wxyz
  [T,4], dt (0.02), joint_names (IsaacLab policy order).

Example:
  scripts/convert_ikobe_clip.py \
      path/to/chest_one_step.npz \
      data/motions/layup.npz
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from cadence.sonic.contract import (
    CADENCE_JOINT_NAMES,
    ISAACLAB_JOINT_NAMES,
    ISAACLAB_TO_CADENCE,
    POLICY_DT,
    quat_normalize,
    quat_slerp,
)

_REQUIRED = ("joint_pos", "root_quat", "joint_names", "fps")


def load_ikobe_npz(path):
    """Load and validate; returns (q_cadence[T,29], quat_wxyz[T,4], fps)."""
    if not isinstance(path, (str, Path)) or "://" in str(path):
        raise ValueError("iKobe motion requires an explicit filesystem path")
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    with np.load(source, allow_pickle=False) as data:
        missing = [name for name in _REQUIRED if name not in data.files]
        if missing:
            raise ValueError(f"iKobe motion is missing fields: {missing}")
        names = np.asarray(data["joint_names"])
        if names.shape != (len(CADENCE_JOINT_NAMES),) or names.dtype.kind not in "US":
            raise ValueError("joint_names must be a 29-string array")
        if tuple(names.astype(str)) != CADENCE_JOINT_NAMES:
            raise ValueError("joint_names must match the cadence canonical order")
        q = np.asarray(data["joint_pos"], dtype=np.float64)
        quats = np.asarray(data["root_quat"], dtype=np.float64)
        fps = float(np.asarray(data["fps"]))
    if q.ndim != 2 or q.shape[1] != len(CADENCE_JOINT_NAMES) or q.shape[0] < 2:
        raise ValueError("joint_pos must have shape [T,29] with T >= 2")
    if quats.shape != (q.shape[0], 4):
        raise ValueError("root_quat must have shape [T,4]")
    if not np.all(np.isfinite(q)) or not np.all(np.isfinite(quats)):
        raise ValueError("joint_pos and root_quat must be finite")
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be a positive number")
    norms = np.linalg.norm(quats, axis=1)
    if np.any(np.abs(norms - 1.0) > 1e-3):
        raise ValueError("root_quat must be unit wxyz quaternions")
    return q, quats, fps


def resample(q, quats, source_fps, target_fps):
    """Linear/slerp resample to the target timeline; returns (q, quats)."""
    duration = (len(q) - 1) / source_fps
    frames = max(2, int(round(duration * target_fps)) + 1)
    times = np.arange(frames) / target_fps
    out_q = np.zeros((frames, q.shape[1]))
    out_quat = np.zeros((frames, 4))
    for k, t in enumerate(times):
        src = min(t * source_fps, len(q) - 1 - 1e-12)
        i0 = int(np.floor(src))
        alpha = min(1.0, max(0.0, src - i0))
        out_q[k] = (1.0 - alpha) * q[i0] + alpha * q[i0 + 1]
        out_quat[k] = quat_slerp(quat_normalize(quats[i0]), quat_normalize(quats[i0 + 1]), alpha)
    return out_q, out_quat


def convert(ikobe_path, npz_path, target_fps=50.0):
    if target_fps != 50.0:
        raise ValueError("the policy timeline is 50 Hz; target_fps must be 50.0")
    q_cadence, quats, source_fps = load_ikobe_npz(ikobe_path)
    q_resampled, quat_resampled = resample(q_cadence, quats, source_fps, target_fps)
    q_ref = q_resampled[:, np.asarray(ISAACLAB_TO_CADENCE)]
    dq_ref = np.zeros_like(q_ref)
    dq_ref[:-1] = (q_ref[1:] - q_ref[:-1]) * target_fps
    dq_ref[-1] = dq_ref[-2]
    npz_path = Path(npz_path)
    npz_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        npz_path,
        q_ref=q_ref,
        dq_ref=dq_ref,
        root_quat_wxyz=quat_resampled,
        dt=np.float64(POLICY_DT),
        joint_names=np.asarray(ISAACLAB_JOINT_NAMES),
    )
    return len(q_ref), (len(q_ref) - 1) / target_fps


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ikobe", type=Path, help="iKobe A3 motion NPZ")
    parser.add_argument("output", type=Path, help="destination NPZ")
    args = parser.parse_args(argv)
    frames, duration = convert(args.ikobe, args.output)
    print(f"{args.output}: {frames} frames, {duration:.2f}s at 50 Hz")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
