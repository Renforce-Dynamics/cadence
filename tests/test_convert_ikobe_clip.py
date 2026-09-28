"""Converter sanity for scripts/convert_ikobe_clip.py (iKobe NPZ -> NPZ)."""

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from convert_ikobe_clip import convert, load_ikobe_npz, resample  # noqa: E402
from cadence.sonic.contract import (  # noqa: E402
    CADENCE_JOINT_NAMES,
    ISAACLAB_JOINT_NAMES,
    cadence_to_isaaclab,
)
from cadence.sonic.reference import SonicClip  # noqa: E402


def _write_ikobe(path, frames=121, fps=60.0, yaw_step=0.0):
    t = np.arange(frames) / fps
    q = np.zeros((frames, 29))
    q[:, 0] = t * 0.1                      # left hip pitch ramps
    q[:, 12] = 0.3                         # waist yaw constant
    yaw = t * yaw_step
    quats = np.stack([np.cos(yaw / 2), np.zeros(frames),
                      np.zeros(frames), np.sin(yaw / 2)], axis=1)
    np.savez(path,
             joint_pos=q.astype(np.float32),
             joint_vel=np.zeros((frames, 29), dtype=np.float32),
             root_quat=quats.astype(np.float32),
             root_pos=np.zeros((frames, 3), dtype=np.float32),
             joint_names=np.asarray(CADENCE_JOINT_NAMES),
             fps=np.float32(fps))
    return q


def test_convert_matches_contract_and_resamples(tmp_path):
    source = tmp_path / "in.npz"
    q = _write_ikobe(source, frames=121, fps=60.0)      # 2.0 s
    out = tmp_path / "out.npz"
    frames, duration = convert(source, out)
    assert frames == 101 and duration == pytest.approx(2.0)
    clip = SonicClip.load(out)                          # full contract validation
    assert clip.frames == 101 and clip.dt == 0.02
    np.testing.assert_allclose(clip.q_ref[0], cadence_to_isaaclab(q[0]), atol=1e-9)
    np.testing.assert_allclose(clip.q_ref[-1], cadence_to_isaaclab(q[-1]), atol=1e-6)
    # 60 -> 50 Hz midpoint is interpolated.
    np.testing.assert_allclose(clip.q_ref[1], cadence_to_isaaclab(q[1] * 0.8 + q[2] * 0.2),
                               atol=1e-6)
    # dq_ref is the forward finite difference on the 50 Hz timeline.
    np.testing.assert_allclose(clip.dq_ref[0], (clip.q_ref[1] - clip.q_ref[0]) * 50.0,
                               rtol=1e-9, atol=1e-9)
    np.testing.assert_allclose(clip.dq_ref[-1], clip.dq_ref[-2])
    with np.load(out) as data:
        assert tuple(np.asarray(data["joint_names"]).astype(str)) == ISAACLAB_JOINT_NAMES


def test_convert_preserves_yaw_rotation(tmp_path):
    source = tmp_path / "in.npz"
    _write_ikobe(source, frames=121, fps=60.0, yaw_step=0.5)
    out = tmp_path / "out.npz"
    convert(source, out)
    clip = SonicClip.load(out)
    from cadence.sonic.contract import quat_yaw
    assert quat_yaw(clip.root_quat_wxyz[0]) == pytest.approx(0.0, abs=1e-6)
    assert quat_yaw(clip.root_quat_wxyz[-1]) == pytest.approx(1.0, abs=1e-3)


def test_load_rejects_wrong_joint_order(tmp_path):
    source = tmp_path / "in.npz"
    q = np.zeros((4, 29), dtype=np.float32)
    np.savez(source, joint_pos=q, root_quat=np.tile([1.0, 0, 0, 0], (4, 1)),
             joint_names=np.asarray(tuple(reversed(CADENCE_JOINT_NAMES))), fps=np.float32(60.0))
    with pytest.raises(ValueError, match="cadence canonical order"):
        load_ikobe_npz(source)


def test_load_rejects_missing_fields_and_bad_quats(tmp_path):
    source = tmp_path / "in.npz"
    np.savez(source, joint_pos=np.zeros((4, 29)))
    with pytest.raises(ValueError, match="missing fields"):
        load_ikobe_npz(source)
    source2 = tmp_path / "bad_quat.npz"
    np.savez(source2, joint_pos=np.zeros((4, 29)), root_quat=np.zeros((4, 4)),
             joint_names=np.asarray(CADENCE_JOINT_NAMES), fps=np.float32(60.0))
    with pytest.raises(ValueError, match="unit wxyz"):
        load_ikobe_npz(source2)
