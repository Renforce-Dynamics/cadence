"""Converter sanity for scripts/convert_sonic_clip.py (SONIC flat CSV -> NPZ)."""

import csv
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from convert_sonic_clip import convert, load_flat_csv  # noqa: E402
from cadence.sonic.contract import (  # noqa: E402
    ISAACLAB_JOINT_NAMES,
    MUJOCO_JOINT_NAMES,
    quat_yaw,
)
from cadence.sonic.reference import SonicClip  # noqa: E402

# SONIC flat CSV column order (kA3CsvJointNames in a3_csv_motion_reference.cpp):
# policy-view joints with head_yaw/head_pitch inserted after the waist.
CSV_JOINTS = (
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "head_yaw_joint", "head_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
)
HEADER = ["Frame", "root_translateX", "root_translateY", "root_translateZ",
          "root_rotateX", "root_rotateY", "root_rotateZ", *CSV_JOINTS]


def _write_csv(path, rows=9):
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(HEADER)
        for i in range(rows):
            joints = []
            for name in CSV_JOINTS:
                if name == "waist_yaw_joint":
                    joints.append(10.0 * i)          # degrees, ramp
                elif name == "left_hip_pitch_joint":
                    joints.append(-2.0)              # constant
                else:
                    joints.append(0.0)
            writer.writerow([i, 100.0 * i, 50.0, 90.0, 0.0, 0.0, 90.0, *joints])
    return path


def test_convert_produces_the_npz_contract(tmp_path):
    csv_path = _write_csv(tmp_path / "tiny.csv")
    npz_path = tmp_path / "out" / "tiny.npz"
    frames, duration = convert(csv_path, npz_path, 30.0, 4, 50.0)
    clip = SonicClip.load(npz_path)
    # Stride 4 over 9 rows keeps rows 0/4/8 (3 source frames, 2/30 s at 30 Hz);
    # 50 Hz targets t = 0, 0.02, 0.04, 0.06.
    assert frames == 4
    assert duration == pytest.approx(0.06)
    assert clip.q_ref.shape == (4, 29)
    assert clip.dq_ref.shape == (4, 29)
    assert clip.root_quat_wxyz.shape == (4, 4)
    assert clip.dt == pytest.approx(0.02)


def test_joint_columns_map_to_isaaclab_order(tmp_path):
    csv_path = _write_csv(tmp_path / "tiny.csv")
    _, q_mujoco, _, _ = load_flat_csv(csv_path, 30.0, 4, 50.0)
    npz_path = tmp_path / "tiny.npz"
    convert(csv_path, npz_path, 30.0, 4, 50.0)
    clip = SonicClip.load(npz_path)
    waist_isaaclab = ISAACLAB_JOINT_NAMES.index("waist_yaw_joint")
    hip_isaaclab = ISAACLAB_JOINT_NAMES.index("left_hip_pitch_joint")
    # waist_yaw ramp: kept source rows are 0, 40, 80 degrees at 30 Hz;
    # t=0.02 lands at alpha=0.6 between the first two kept frames.
    assert clip.q_ref[0, waist_isaaclab] == pytest.approx(0.0)
    assert clip.q_ref[1, waist_isaaclab] == pytest.approx(math.radians(40.0) * 0.6)
    assert clip.q_ref[3, waist_isaaclab] == pytest.approx(
        math.radians(40.0) + 0.8 * math.radians(40.0))
    # Constant column survives the resample verbatim.
    np.testing.assert_allclose(
        clip.q_ref[:, hip_isaaclab], math.radians(-2.0), atol=1e-12)
    # dq is a forward difference at 50 Hz; the last frame repeats the previous.
    assert clip.dq_ref[0, waist_isaaclab] == pytest.approx(
        (clip.q_ref[1, waist_isaaclab] - clip.q_ref[0, waist_isaaclab]) * 50.0)
    np.testing.assert_array_equal(clip.dq_ref[-1], clip.dq_ref[-2])


def test_root_euler_becomes_quaternion(tmp_path):
    csv_path = _write_csv(tmp_path / "tiny.csv")
    npz_path = tmp_path / "tiny.npz"
    convert(csv_path, npz_path, 30.0, 4, 50.0)
    clip = SonicClip.load(npz_path)
    # Constant root_rotateZ = 90 degrees -> yaw pi/2 for every frame.
    for quat in clip.root_quat_wxyz:
        assert quat_yaw(quat) == pytest.approx(math.pi / 2)
        np.testing.assert_allclose(np.linalg.norm(quat), 1.0)


def test_converter_rejects_missing_joint_column(tmp_path):
    path = tmp_path / "broken.csv"
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([name for name in HEADER if name != "left_knee_joint"])
        writer.writerow([0] * (len(HEADER) - 1))
    with pytest.raises(ValueError, match="left_knee_joint"):
        convert(path, tmp_path / "out.npz", 30.0, 4, 50.0)


def test_converter_rejects_non_50hz_target(tmp_path):
    csv_path = _write_csv(tmp_path / "tiny.csv")
    with pytest.raises(ValueError, match="50 Hz"):
        convert(csv_path, tmp_path / "out.npz", 30.0, 4, 25.0)


def test_generated_repo_clips_load(tmp_path):
    for name in ("001_walk_front_slow.npz", "BMD_0319_stand.npz"):
        clip = SonicClip.load(ROOT / "data/motions" / name)
        assert clip.q_ref.shape[1] == 29 and clip.frames > 10
        assert clip.dt == pytest.approx(0.02)
