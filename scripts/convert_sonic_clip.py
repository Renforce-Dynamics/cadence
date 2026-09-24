#!/usr/bin/env python3
"""Convert a SONIC flat A3 motion CSV into a sonic_clip NPZ resource.

Ports the SONIC C++ ``A3CsvMotionReference::Load`` (Apache License 2.0,
``gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/a3_deploy/
a3_csv_motion_reference.cpp``): the bundled CSVs are 120 Hz; keep every
``--csv-frame-stride``-th row (``--source-fps`` is the post-stride rate), then
resample to the 50 Hz policy timeline with linear interpolation and slerp.
Joint columns are matched by name; root rotation is extrinsic XYZ Euler in
degrees; root translation is centimeters. Joint velocities come from forward
finite differences on the resampled timeline (last frame repeats the previous
difference), exactly as the C++ loader.

Output NPZ fields:
  q_ref [T,29] and dq_ref [T,29] in IsaacLab policy order, root_quat_wxyz
  [T,4], dt (0.02), joint_names (IsaacLAB policy order).

Example:
  scripts/convert_sonic_clip.py \
      path/to/001_walk_front_slow.csv \
      data/motions/001_walk_front_slow.npz
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np

from cadence.sonic.contract import (
    A3_MUJOCO_TO_ISAACLAB_DOF,
    ISAACLAB_JOINT_NAMES,
    MUJOCO_JOINT_NAMES,
    POLICY_DT,
    quat_multiply,
    quat_normalize,
    quat_slerp,
)

CENTIMETERS_TO_METERS = 0.01
ROOT_COLUMNS = (
    "root_translateX", "root_translateY", "root_translateZ",
    "root_rotateX", "root_rotateY", "root_rotateZ",
)


def quat_from_euler_xyz_degrees(x_deg, y_deg, z_deg):
    """scipy Rotation.from_euler("xyz", degrees=True): extrinsic X, Y, Z."""
    hx, hy, hz = (0.5 * math.radians(v) for v in (x_deg, y_deg, z_deg))
    qx = np.array([math.cos(hx), math.sin(hx), 0.0, 0.0])
    qy = np.array([math.cos(hy), 0.0, math.sin(hy), 0.0])
    qz = np.array([math.cos(hz), 0.0, 0.0, math.sin(hz)])
    return quat_normalize(quat_multiply(quat_multiply(qz, qy), qx))


def load_flat_csv(path, source_fps, csv_frame_stride, target_fps):
    """Load and resample; returns (times, q_mujoco, quat_wxyz, root_pos_m)."""
    with open(path, newline="") as handle:
        rows = [row for row in csv.reader(handle) if any(cell.strip() for cell in row)]
    if len(rows) < 2:
        raise ValueError("CSV has no data rows")
    header = [cell.strip().strip('"') for cell in rows[0]]
    column = {name: index for index, name in enumerate(header)}
    for name in ROOT_COLUMNS:
        if name not in column:
            raise ValueError(f"missing column: {name}")
    joint_columns = []
    for name in MUJOCO_JOINT_NAMES:
        if name in column:
            joint_columns.append(column[name])
        elif name + "_dof" in column:
            joint_columns.append(column[name + "_dof"])
        else:
            raise ValueError(f"missing A3 joint column: {name}")

    raw_rows = rows[1:]
    kept = raw_rows[::csv_frame_stride]
    frames = []
    for row_number, fields in enumerate(kept, start=1):
        if len(fields) < len(header):
            raise ValueError(f"row {row_number} has too few columns")

        def value(index, label):
            try:
                result = float(fields[index])
            except ValueError as exc:
                raise ValueError(f"row {row_number}: invalid {label}") from exc
            if not math.isfinite(result):
                raise ValueError(f"row {row_number}: non-finite {label}")
            return result

        root_pos = [value(column[name], name) * CENTIMETERS_TO_METERS for name in ROOT_COLUMNS[:3]]
        root_rot = [value(column[name], name) for name in ROOT_COLUMNS[3:]]
        q_mujoco = [value(index, MUJOCO_JOINT_NAMES[j]) * (math.pi / 180.0)
                    for j, index in enumerate(joint_columns)]
        frames.append((root_pos, quat_from_euler_xyz_degrees(*root_rot), q_mujoco))

    source_dt = 1.0 / source_fps
    duration = 0.0 if len(frames) <= 1 else (len(frames) - 1) * source_dt
    times = [0.0] if len(frames) == 1 else list(
        np.arange(0.0, duration - 1e-12, 1.0 / target_fps)) or [0.0]
    out_q, out_quat, out_pos = [], [], []
    for t in times:
        src_pos = t * source_fps
        i0 = int(math.floor(src_pos))
        if i0 >= len(frames) - 1:
            out_pos.append(frames[-1][0])
            out_quat.append(frames[-1][1])
            out_q.append(frames[-1][2])
            continue
        i1 = i0 + 1
        alpha = min(1.0, max(0.0, src_pos - i0))
        out_pos.append([(1 - alpha) * a + alpha * b
                        for a, b in zip(frames[i0][0], frames[i1][0])])
        out_quat.append(quat_slerp(frames[i0][1], frames[i1][1], alpha))
        out_q.append([(1 - alpha) * a + alpha * b
                      for a, b in zip(frames[i0][2], frames[i1][2])])
    return (np.asarray(times), np.asarray(out_q), np.asarray(out_quat),
            np.asarray(out_pos))


def convert(csv_path, npz_path, source_fps, csv_frame_stride, target_fps):
    if target_fps != 50.0:
        raise ValueError("the policy timeline is 50 Hz; target_fps must be 50.0")
    times, q_mujoco, quats, _ = load_flat_csv(
        csv_path, source_fps, csv_frame_stride, target_fps)
    q_ref = q_mujoco[:, np.asarray(A3_MUJOCO_TO_ISAACLAB_DOF)]
    dq_ref = np.zeros_like(q_ref)
    if len(times) >= 2:
        dq_ref[:-1] = (q_ref[1:] - q_ref[:-1]) * target_fps
        dq_ref[-1] = dq_ref[-2]
    npz_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        npz_path,
        q_ref=q_ref,
        dq_ref=dq_ref,
        root_quat_wxyz=quats,
        dt=np.float64(POLICY_DT),
        joint_names=np.asarray(ISAACLAB_JOINT_NAMES),
    )
    return len(times), float(times[-1]) if len(times) else 0.0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="SONIC flat A3 motion CSV")
    parser.add_argument("output", type=Path, help="destination NPZ")
    parser.add_argument("--source-fps", type=float, default=30.0,
                        help="post-stride source rate (bundled CSVs: 30 with stride 4)")
    parser.add_argument("--csv-frame-stride", type=int, default=4,
                        help="keep every Nth source row (bundled 120 Hz CSVs: 4)")
    parser.add_argument("--target-fps", type=float, default=50.0)
    args = parser.parse_args(argv)
    if args.csv_frame_stride < 1 or args.source_fps <= 0:
        parser.error("invalid stride or source fps")
    frames, duration = convert(args.csv, args.output, args.source_fps,
                               args.csv_frame_stride, args.target_fps)
    print(f"{args.output}: {frames} frames, {duration:.2f}s at 50 Hz")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
