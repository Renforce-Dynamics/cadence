"""Golden action regression against the real SONIC 035 a3_fast ONNX actor.

Neutral frame: the stand clip's frame-0 tokenizer prefix (identity robot quat,
no yaw offset, 20 ms future spacing) plus a history seeded at the default pose
with zero gyro, velocity and last action. Golden values recorded from
``models/model_step_200000_a3_fast.onnx`` (SHA-256
66f967af0ebf791c31c3095bf1f0d38fc1bb1ca96f34a212161646e328aa804c).
"""

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("onnxruntime")

from cadence.sonic.contract import DEFAULT_ANGLES_CADENCE, CADENCE_JOINT_NAMES
from cadence.sonic.policy import SonicWholeBodyPolicy
from cadence.sonic.reference import SonicClip

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "models/model_step_200000_a3_fast.onnx"
STAND_CLIP = ROOT / "data/motions/BMD_0319_stand.npz"

GOLDEN_ACTION = [
    0.271362245, 0.049299344, 0.145280853, 0.275860548, -0.205647722,
    -0.030557334, 0.148355901, -0.181443840, -0.150420427, -0.181443959,
    -0.158737451, 0.258584023, -0.319191158, 0.008406058, 0.038366146,
    0.527387500, -0.449829280, -0.110623851, 0.107148245, -0.701808870,
    0.971844733, 0.353358597, 0.399212122, 0.218627691, 0.042168219,
    -0.180828333, 0.061196178, -0.129143730, 0.002766691,
]


def _neutral_frame():
    from types import SimpleNamespace
    return SimpleNamespace(
        gyro_b=np.zeros(3),
        quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        joint_pos=np.asarray(DEFAULT_ANGLES_CADENCE, dtype=float),
        joint_vel=np.zeros(29),
    )


def test_neutral_frame_action_matches_golden():
    clip = SonicClip.load(STAND_CLIP)
    prefix = clip.tokenizer_slice(0, [1.0, 0.0, 0.0, 0.0], 0.0, 1)
    from types import SimpleNamespace
    adapter = SonicWholeBodyPolicy(
        MODEL, None, np.full(29, -3.5), np.full(29, 3.5),
        services=SimpleNamespace(joint_names=CADENCE_JOINT_NAMES))
    # Ten identical default-pose pushes fully seed the history.
    for _ in range(10):
        q_des, requested, raw, observation = adapter.infer(
            _neutral_frame(), np.zeros(29), prefix)
    assert observation.shape == (1570,) and observation.dtype == np.float32
    np.testing.assert_allclose(raw, GOLDEN_ACTION, rtol=2e-5, atol=2e-6)
    # Decoded target stays near the default pose on a neutral stand frame.
    assert np.abs(q_des - np.asarray(DEFAULT_ANGLES_CADENCE)).max() < 0.35
