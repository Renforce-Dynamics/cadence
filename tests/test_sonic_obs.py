"""Golden-bytes regression for the SONIC 1570-float observation assembly.

The fixture clip and measurement frames are synthetic and fully deterministic;
the SHA-256 pins the exact float32 byte stream the ONNX session consumes,
mirroring tests/test_a3_lower_motion.py in cadence.
"""

import hashlib

import numpy as np
import pytest

from cadence.sonic.contract import (
    COMMAND_DIM,
    DEFAULT_ANGLES_CADENCE,
    DEFAULT_ANGLES_ISAACLAB,
    ISAACLAB_JOINT_NAMES,
    JOINT_DIM,
    OBSERVATION_DIM,
    TOKENIZER_DIM,
    cadence_to_isaaclab,
    quat_from_yaw,
)
from cadence.sonic.policy import SonicProprioHistory
from cadence.sonic.reference import SonicClip

CLIP_FRAMES = 30


def _clip_arrays():
    joints = np.arange(JOINT_DIM)
    ticks = np.arange(CLIP_FRAMES)[:, None]
    q_ref = (np.asarray(DEFAULT_ANGLES_ISAACLAB)[None, :]
             + 0.002 * joints[None, :]
             + 0.01 * np.sin(0.2 * ticks + 0.01 * joints[None, :]))
    dq_ref = 0.1 * np.cos(0.2 * ticks + 0.005 * joints[None, :])
    quats = np.array([quat_from_yaw(0.01 * k) for k in range(CLIP_FRAMES)])
    return q_ref, dq_ref, quats


def _write_clip(path):
    q_ref, dq_ref, quats = _clip_arrays()
    np.savez(path, q_ref=q_ref, dq_ref=dq_ref, root_quat_wxyz=quats,
             dt=np.float64(0.02), joint_names=np.asarray(ISAACLAB_JOINT_NAMES))
    return path


@pytest.fixture()
def clip(tmp_path):
    return SonicClip.load(_write_clip(tmp_path / "fixture.npz"))


def _push(history, index, action):
    gyro = np.array([0.1 * index, -0.2 * index, 0.05 * index])
    quat = quat_from_yaw(0.02 * index)
    q_cadence = np.asarray(DEFAULT_ANGLES_CADENCE) + (np.arange(JOINT_DIM) - 14 + index) * 0.003
    dq_cadence = np.arange(JOINT_DIM) * 0.01 - 0.05 * index
    history.push(gyro, quat, cadence_to_isaaclab(q_cadence), cadence_to_isaaclab(dq_cadence),
                 action)


def _observation(clip):
    prefix = clip.tokenizer_slice(5, quat_from_yaw(0.05), 0.25, 1)
    history = SonicProprioHistory()
    for index in range(12):
        _push(history, index,
              np.zeros(JOINT_DIM) if index < 6 else (np.arange(JOINT_DIM) - 14) * 0.1)
    return np.concatenate([prefix, history.build()]).astype(np.float32)


def test_observation_matches_golden_bytes(clip):
    observation = _observation(clip)
    assert observation.shape == (OBSERVATION_DIM,)
    # Captured from this fixture: 12 pushes (6 zero-action, 6 ramped-action),
    # clip tick 5, robot yaw 0.05, reference yaw offset 0.25, skip 1.
    assert hashlib.sha256(observation.tobytes()).hexdigest() == (
        "c1471c3cd5a4810ae24157acfa26273db002e2cc3bdfdbb9949e3978e6a8d8f4")


def test_observation_spot_values(clip):
    observation = _observation(clip)
    # Command block: q frames then dq frames, each frame-major IsaacLab order.
    q_ref, dq_ref, _ = _clip_arrays()
    np.testing.assert_allclose(observation[0:29], q_ref[5], atol=1e-6)
    np.testing.assert_allclose(observation[29:58], q_ref[6], atol=1e-6)
    np.testing.assert_allclose(observation[290:319], dq_ref[5], atol=1e-6)
    # Anchor block: 10 x 6D. With yaw-only rotations the z row stays zero.
    anchor = np.asarray(observation[COMMAND_DIM:TOKENIZER_DIM]).reshape(10, 6)
    np.testing.assert_allclose(anchor[:, 4:], 0.0, atol=1e-7)
    np.testing.assert_allclose(anchor[0, 1], -anchor[0, 2], atol=1e-7)
    # Proprio: gyro term, newest frame is the twelfth push (index 11).
    proprio = observation[TOKENIZER_DIM:]
    gyro = proprio[0:30].reshape(10, 3)
    np.testing.assert_allclose(gyro[-1], [1.1, -2.2, 0.55], atol=1e-6)
    np.testing.assert_allclose(gyro[0], [0.2, -0.4, 0.1], atol=1e-6)
    # Gravity term for yaw-only quats is always straight down.
    gravity = proprio[900:930].reshape(10, 3)
    np.testing.assert_allclose(gravity, np.tile([0, 0, -1], (10, 1)), atol=1e-6)
    # Actions term: first six pushes carried zero action, then (j-14)*0.1.
    actions = proprio[610:900].reshape(10, 29)
    np.testing.assert_allclose(actions[:4], 0.0, atol=1e-7)
    np.testing.assert_allclose(actions[-1], (np.arange(JOINT_DIM) - 14) * 0.1, atol=1e-6)


def test_first_push_seeds_every_history_slot():
    history = SonicProprioHistory()
    _push(history, 3, np.ones(JOINT_DIM) * 0.5)
    block = history.build()
    assert block.shape == (930,)
    gyro = block[0:30].reshape(10, 3)
    np.testing.assert_allclose(gyro, np.tile([0.3, -0.6, 0.15], (10, 1)))
    actions = block[610:900].reshape(10, 29)
    np.testing.assert_allclose(actions, 0.5)


def test_empty_history_builds_zeros():
    block = SonicProprioHistory().build()
    assert block.shape == (930,)
    assert not np.any(block)


def test_history_evicts_oldest_frame():
    history = SonicProprioHistory()
    for index in range(12):
        _push(history, index, np.zeros(JOINT_DIM))
    gyro = history.build()[0:30].reshape(10, 3)
    # Frames 2..11 remain; frames 0 and 1 were evicted.
    np.testing.assert_allclose(gyro[0], [0.2, -0.4, 0.1], atol=1e-6)
    np.testing.assert_allclose(gyro[-1], [1.1, -2.2, 0.55], atol=1e-6)


def test_history_rollback_reproduces_the_accepted_sequence():
    history = SonicProprioHistory()
    fresh = SonicProprioHistory()
    _push(history, 0, np.zeros(JOINT_DIM))
    snapshot = history.snapshot()
    _push(history, 1, np.ones(JOINT_DIM))
    history.restore(snapshot)
    assert history.build().tobytes() == snapshot_build(fresh, 0)
    _push(history, 1, np.ones(JOINT_DIM))
    _push(fresh, 0, np.zeros(JOINT_DIM))
    _push(fresh, 1, np.ones(JOINT_DIM))
    assert history.build().tobytes() == fresh.build().tobytes()


def snapshot_build(history, index):
    _push(history, index, np.zeros(JOINT_DIM))
    return history.build().tobytes()


@pytest.mark.parametrize("field,value", [
    ("gyro", (float("nan"),) * 3),
    ("quat", (0.0, 0.0, 0.0, 0.0)),
    ("q", (0.0,) * 28),
    ("dq", (float("inf"),) * 29),
    ("action", (float("nan"),) * 29),
])
def test_invalid_push_does_not_pollute_history(field, value):
    history = SonicProprioHistory()
    _push(history, 0, np.zeros(JOINT_DIM))
    before = history.build().tobytes()
    parts = {"gyro": np.zeros(3), "quat": (1.0, 0.0, 0.0, 0.0),
             "q": np.zeros(JOINT_DIM), "dq": np.zeros(JOINT_DIM),
             "action": np.zeros(JOINT_DIM)}
    parts[field] = value
    with pytest.raises((ValueError, RuntimeError)):
        history.push(parts["gyro"], parts["quat"], parts["q"], parts["dq"], parts["action"])
    assert history.build().tobytes() == before


def test_clip_loader_validates_contract(clip, tmp_path):
    assert clip.frames == CLIP_FRAMES
    assert clip.dt == pytest.approx(0.02)
    assert clip.duration_s == pytest.approx((CLIP_FRAMES - 1) * 0.02)
    q_ref, dq_ref, quats = _clip_arrays()
    bad = tmp_path / "bad_order.npz"
    np.savez(bad, q_ref=q_ref, dq_ref=dq_ref, root_quat_wxyz=quats,
             dt=np.float64(0.02), joint_names=np.asarray(tuple(reversed(ISAACLAB_JOINT_NAMES))))
    with pytest.raises(ValueError, match="IsaacLab policy order"):
        SonicClip.load(bad)
    bad_dt = tmp_path / "bad_dt.npz"
    np.savez(bad_dt, q_ref=q_ref, dq_ref=dq_ref, root_quat_wxyz=quats,
             dt=np.float64(0.01), joint_names=np.asarray(ISAACLAB_JOINT_NAMES))
    with pytest.raises(ValueError, match="50 Hz"):
        SonicClip.load(bad_dt)


def test_clip_future_frames_clamp_at_the_end(clip):
    tail = clip.tokenizer_slice(CLIP_FRAMES - 2, [1.0, 0.0, 0.0, 0.0], 0.0, 1)
    q_ref, dq_ref, _ = _clip_arrays()
    np.testing.assert_allclose(tail[0:29], q_ref[-2], atol=1e-6)
    np.testing.assert_allclose(tail[29:58], q_ref[-1], atol=1e-6)
    # Beyond the clip every future slot repeats the final frame.
    np.testing.assert_allclose(tail[58:87], q_ref[-1], atol=1e-6)


def test_clip_yaw_lock_uses_first_frame(clip):
    _, _, quats = _clip_arrays()
    offset = clip.lock_yaw(quats[0])
    assert offset == pytest.approx(0.0, abs=1e-12)
    offset = clip.lock_yaw(quat_from_yaw(0.5))
    assert offset == pytest.approx(0.5, abs=1e-12)
