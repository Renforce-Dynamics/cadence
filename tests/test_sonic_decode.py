"""Decode-path regressions for the SONIC whole-body policy adapter.

A fake runner replaces ONNX inference (monkeypatched ``OnnxPolicy``); the
tests pin the decode math: clip raw at +/-20, q_des = raw * scale + default in
IsaacLab order, permute to cadence order, clip to the configured joint limits,
NaN -> 0. Ported from ``a3_action_decoder.cpp``.
"""

from types import SimpleNamespace

import numpy as np
import pytest

from cadence.sonic import policy as policy_module
from cadence.sonic.contract import (
    ACTION_CLIP,
    ACTION_SCALE_024_ISAACLAB,
    CADENCE_JOINT_NAMES,
    DEFAULT_ANGLES_CADENCE,
    DEFAULT_ANGLES_ISAACLAB,
    TOKENIZER_DIM,
    cadence_to_isaaclab,
    isaaclab_to_cadence,
)


class _FakeRunner:
    def __init__(self, action):
        self.action = np.asarray(action, dtype=np.float32)
        self.observations = []

    def infer(self, observation):
        self.observations.append(np.asarray(observation, dtype=np.float32).copy())
        return self.action


def _adapter(tmp_path, monkeypatch, action, *, position_min=None, position_max=None):
    model = tmp_path / "sonic.onnx"
    model.touch()
    runner = _FakeRunner(action)
    monkeypatch.setattr(policy_module, "OnnxPolicy",
                        lambda *args, **kwargs: runner)
    wide_min = np.full(29, -1e6)
    wide_max = np.full(29, 1e6)
    adapter = policy_module.SonicWholeBodyPolicy(
        model, None,
        wide_min if position_min is None else position_min,
        wide_max if position_max is None else position_max,
        services=SimpleNamespace(joint_names=CADENCE_JOINT_NAMES))
    return adapter, runner


def _robot(joint_pos=None, joint_vel=None):
    return SimpleNamespace(
        gyro_b=np.zeros(3),
        quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        joint_pos=np.asarray(DEFAULT_ANGLES_CADENCE if joint_pos is None else joint_pos,
                             dtype=float),
        joint_vel=np.zeros(29) if joint_vel is None else np.asarray(joint_vel, dtype=float),
    )


def test_raw_action_clip_bounds_decode(tmp_path, monkeypatch):
    adapter, _ = _adapter(tmp_path, monkeypatch, np.full(29, 100.0))
    q_des, requested, raw_clipped, _ = adapter.infer(
        _robot(), np.zeros(29), np.zeros(TOKENIZER_DIM))
    np.testing.assert_array_equal(raw_clipped, np.full(29, ACTION_CLIP))
    expected = isaaclab_to_cadence(
        ACTION_CLIP * np.asarray(ACTION_SCALE_024_ISAACLAB) + np.asarray(DEFAULT_ANGLES_ISAACLAB))
    np.testing.assert_allclose(q_des, expected, atol=1e-12)
    np.testing.assert_array_equal(q_des, requested)  # wide limits: no clipping


def test_negative_raw_action_clip(tmp_path, monkeypatch):
    adapter, _ = _adapter(tmp_path, monkeypatch, np.full(29, -25.0))
    q_des, _, raw_clipped, _ = adapter.infer(_robot(), np.zeros(29), np.zeros(TOKENIZER_DIM))
    np.testing.assert_array_equal(raw_clipped, np.full(29, -ACTION_CLIP))
    expected = isaaclab_to_cadence(
        -ACTION_CLIP * np.asarray(ACTION_SCALE_024_ISAACLAB) + np.asarray(DEFAULT_ANGLES_ISAACLAB))
    np.testing.assert_allclose(q_des, expected, atol=1e-12)


def test_non_finite_raw_action_decays_to_zero(tmp_path, monkeypatch):
    action = np.full(29, np.nan)
    action[3] = np.inf
    action[7] = -np.inf
    adapter, _ = _adapter(tmp_path, monkeypatch, action)
    q_des, _, raw_clipped, _ = adapter.infer(_robot(), np.zeros(29), np.zeros(TOKENIZER_DIM))
    default = np.asarray(DEFAULT_ANGLES_CADENCE)
    # NaN -> 0 lands exactly on the default pose. The +/-inf entries are
    # IsaacLab slots 3/7 (left_hip_roll, right_hip_yaw), cadence slots 1/8.
    mask = np.ones(29, dtype=bool)
    mask[[1, 8]] = False
    np.testing.assert_array_equal(q_des[mask], default[mask])
    np.testing.assert_array_equal(np.delete(raw_clipped, [3, 7]), 0.0)
    assert raw_clipped[3] == ACTION_CLIP and raw_clipped[7] == -ACTION_CLIP
    scale = np.asarray(ACTION_SCALE_024_ISAACLAB)
    default_il = np.asarray(DEFAULT_ANGLES_ISAACLAB)
    assert q_des[1] == pytest.approx(ACTION_CLIP * scale[3] + default_il[3])
    assert q_des[8] == pytest.approx(-ACTION_CLIP * scale[7] + default_il[7])


def test_decode_clips_to_configured_joint_limits(tmp_path, monkeypatch):
    position_min = np.full(29, -3.5)
    position_max = np.full(29, 3.5)
    # left_knee_joint (cadence 3) and right_shoulder_pitch_joint (cadence 22).
    knee = 3
    shoulder = 22
    knee_default = DEFAULT_ANGLES_CADENCE[knee]
    shoulder_default = DEFAULT_ANGLES_CADENCE[shoulder]
    position_min[knee] = knee_default + 0.05
    position_max[knee] = knee_default + 0.10
    position_min[shoulder] = shoulder_default - 0.02
    position_max[shoulder] = shoulder_default + 0.02
    adapter, _ = _adapter(tmp_path, monkeypatch, np.full(29, 100.0),
                          position_min=position_min, position_max=position_max)
    q_des, requested, _, _ = adapter.infer(_robot(), np.zeros(29), np.zeros(TOKENIZER_DIM))
    assert q_des[knee] == pytest.approx(knee_default + 0.10)
    assert requested[knee] > q_des[knee]
    assert q_des[shoulder] == pytest.approx(shoulder_default + 0.02)
    assert requested[shoulder] > q_des[shoulder]


def test_decode_permutation_places_values_in_cadence_order(tmp_path, monkeypatch):
    action = np.arange(29, dtype=float) * 0.1 - 1.4
    adapter, _ = _adapter(tmp_path, monkeypatch, action)
    q_des, _, raw_clipped, _ = adapter.infer(_robot(), np.zeros(29), np.zeros(TOKENIZER_DIM))
    np.testing.assert_allclose(raw_clipped, action, atol=1e-6)
    np.testing.assert_allclose(
        q_des, isaaclab_to_cadence(
            action * np.asarray(ACTION_SCALE_024_ISAACLAB)
            + np.asarray(DEFAULT_ANGLES_ISAACLAB)),
        atol=1e-12)


def test_committed_raw_action_enters_history_in_isaaclab_order(tmp_path, monkeypatch):
    action = np.arange(29, dtype=float) * 0.1 - 1.4
    adapter, _ = _adapter(tmp_path, monkeypatch, action)
    _, _, raw_clipped, _ = adapter.infer(_robot(), np.zeros(29), np.zeros(TOKENIZER_DIM))
    # The next tick observes the previously applied (clipped) raw action.
    _, _, _, observation = adapter.infer(
        _robot(), raw_clipped, np.zeros(TOKENIZER_DIM))
    actions = observation[TOKENIZER_DIM + 610:TOKENIZER_DIM + 900].reshape(10, 29)
    np.testing.assert_allclose(actions[-1], raw_clipped, atol=1e-6)
    np.testing.assert_allclose(actions[-1], action, atol=1e-6)
    # IsaacLab order: slot 0 is left_hip_pitch_joint, slot 2 is waist_yaw_joint.
    assert actions[-1, 0] == pytest.approx(-1.4)
    assert actions[-1, 2] == pytest.approx(-1.2)


def test_clipped_feedback_stores_the_clip_not_the_raw(tmp_path, monkeypatch):
    adapter, _ = _adapter(tmp_path, monkeypatch, np.zeros(29))
    oversized = np.full(29, 100.0)
    _, _, _, observation = adapter.infer(_robot(), oversized, np.zeros(TOKENIZER_DIM))
    actions = observation[TOKENIZER_DIM + 610:TOKENIZER_DIM + 900].reshape(10, 29)
    np.testing.assert_allclose(actions[-1], ACTION_CLIP)


def test_joint_rel_term_uses_isaaclab_order_and_float32(tmp_path, monkeypatch):
    adapter, _ = _adapter(tmp_path, monkeypatch, np.zeros(29))
    moved = np.asarray(DEFAULT_ANGLES_CADENCE).copy()
    moved[0] += 0.25   # left_hip_pitch_joint (cadence 0, IsaacLab 0)
    moved[12] += 0.5   # waist_yaw_joint (cadence 12, IsaacLab 2)
    _, _, _, observation = adapter.infer(_robot(joint_pos=moved), np.zeros(29),
                                         np.zeros(TOKENIZER_DIM))
    q_rel = observation[TOKENIZER_DIM + 30:TOKENIZER_DIM + 320].reshape(10, 29)
    assert q_rel[-1, 0] == pytest.approx(np.float32(moved[0]) - np.float32(DEFAULT_ANGLES_ISAACLAB[0]))
    assert q_rel[-1, 2] == pytest.approx(np.float32(0.5), abs=1e-6)
    np.testing.assert_allclose(q_rel[-1, 1], 0.0, atol=1e-7)


def test_adapter_rejects_bad_model_paths(tmp_path):
    with pytest.raises(ValueError, match="explicit filesystem path"):
        policy_module.SonicWholeBodyPolicy("pkg://x", None, np.full(29, -3.5), np.full(29, 3.5))
    with pytest.raises(ValueError, match="resolved relative"):
        policy_module.SonicWholeBodyPolicy("relative.onnx", None,
                                           np.full(29, -3.5), np.full(29, 3.5))
    with pytest.raises(FileNotFoundError):
        policy_module.SonicWholeBodyPolicy(tmp_path / "missing.onnx", None,
                                           np.full(29, -3.5), np.full(29, 3.5))


def test_adapter_validates_runtime_joint_names(tmp_path, monkeypatch):
    model = tmp_path / "sonic.onnx"
    model.touch()
    monkeypatch.setattr(policy_module, "OnnxPolicy", lambda *args, **kwargs: _FakeRunner(np.zeros(29)))
    with pytest.raises(ValueError, match="canonical A3 cadence order"):
        policy_module.SonicWholeBodyPolicy(
            model, None, np.full(29, -3.5), np.full(29, 3.5),
            services=SimpleNamespace(joint_names=tuple(reversed(CADENCE_JOINT_NAMES))))
