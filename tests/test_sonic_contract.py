"""Pin the ported SONIC A3 whole-body policy contract.

Values are transcribed from the SONIC-for-A3 release (Apache License 2.0):
``gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/a3_policy_parameters.hpp``
(024 gains/scales/defaults in the 29-DOF MuJoCo policy view) and
``gear_sonic/envs/manager_env/robots/a3.py`` (``A3_ACTIVE_JOINT_NAMES``,
``A3_ISAACLAB_TO_MUJOCO_DOF``).
"""

import numpy as np
import pytest

from cadence.sonic import contract


def test_joint_orders_are_29_unique_names():
    for names in (contract.ISAACLAB_JOINT_NAMES, contract.CADENCE_JOINT_NAMES,
                  contract.MUJOCO_JOINT_NAMES):
        assert len(names) == 29 and len(set(names)) == 29
    assert set(contract.ISAACLAB_JOINT_NAMES) == set(contract.CADENCE_JOINT_NAMES)


def test_mujoco_order_matches_policy_view_layout():
    # Policy view: waist, left arm, right arm, left leg, right leg.
    assert contract.MUJOCO_JOINT_NAMES[:3] == (
        "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint")
    assert contract.MUJOCO_JOINT_NAMES[3] == "left_shoulder_pitch_joint"
    assert contract.MUJOCO_JOINT_NAMES[10] == "right_shoulder_pitch_joint"
    assert contract.MUJOCO_JOINT_NAMES[17] == "left_hip_pitch_joint"
    assert contract.MUJOCO_JOINT_NAMES[23] == "right_hip_pitch_joint"


def test_isaaclab_to_mujoco_table_is_verbatim_and_its_inverse_is_corrected():
    # Verbatim A3_ISAACLAB_TO_MUJOCO_DOF from a3.py / the hpp.
    assert tuple(contract.A3_ISAACLAB_TO_MUJOCO_DOF) == (
        2, 5, 8, 11, 15, 19, 21, 23, 25, 27,
        12, 16, 20, 22, 24, 26, 28,
        0, 3, 6, 9, 13, 17,
        1, 4, 7, 10, 14, 18,
    )
    # The corrected argsort inverse from the hpp (a3.py's own
    # A3_MUJOCO_TO_ISAACLAB_DOF literal is buggy and must NOT be used).
    assert tuple(contract.A3_MUJOCO_TO_ISAACLAB_DOF) == (
        17, 23, 0, 18, 24, 1, 19, 25, 2, 20,
        26, 3, 10, 21, 27, 4, 11, 22, 28, 5,
        12, 6, 13, 7, 14, 8, 15, 9, 16,
    )
    forward = np.asarray(contract.A3_ISAACLAB_TO_MUJOCO_DOF)
    inverse = np.asarray(contract.A3_MUJOCO_TO_ISAACLAB_DOF)
    np.testing.assert_array_equal(inverse[forward], np.arange(29))


def test_cadence_isaaclab_permutations_round_trip():
    vector = np.arange(29, dtype=float) * 0.01 - 0.2
    round_trip = contract.isaaclab_to_cadence(contract.cadence_to_isaaclab(vector))
    np.testing.assert_array_equal(round_trip, vector)
    np.testing.assert_array_equal(
        contract.cadence_to_isaaclab(contract.isaaclab_to_cadence(vector)), vector)


@pytest.mark.parametrize("name,isaaclab_index,cadence_index", [
    ("left_hip_pitch_joint", 0, 0),
    ("right_hip_pitch_joint", 1, 6),
    ("waist_yaw_joint", 2, 12),
    ("waist_pitch_joint", 8, 14),
    ("left_knee_joint", 9, 3),
    ("left_shoulder_pitch_joint", 11, 15),
    ("left_ankle_roll_joint", 17, 5),
    ("left_wrist_yaw_joint", 27, 21),
    ("right_wrist_yaw_joint", 28, 28),
])
def test_known_joints_land_at_expected_indices(name, isaaclab_index, cadence_index):
    assert contract.ISAACLAB_JOINT_NAMES[isaaclab_index] == name
    assert contract.CADENCE_JOINT_NAMES[cadence_index] == name
    vector = np.zeros(29)
    vector[isaaclab_index] = 1.0
    assert contract.isaaclab_to_cadence(vector)[cadence_index] == 1.0


@pytest.mark.parametrize("name,default,scale,kp,kd", [
    # Transcribed from a3_policy_parameters.hpp (MuJoCo policy view).
    ("waist_yaw_joint", 0.0, 0.25 * 200.0 / 85.0, 85.0, 3.0),
    ("waist_roll_joint", 0.0, 0.25 * 42.0 / 50.0, 50.0, 2.0),
    ("waist_pitch_joint", 0.0, 0.25 * 105.0 / 50.0, 50.0, 2.0),
    ("left_hip_pitch_joint", -0.1311, 0.6875, 80.0, 3.0),
    ("left_hip_roll_joint", 0.0056, 0.4583333333333333, 120.0, 4.0),
    ("left_knee_joint", 0.2468, 0.25 * 300.0 / 250.0, 250.0, 8.0),
    # The 024 profile raises ankle stiffness to 60 and lowers damping to 1.2.
    ("left_ankle_pitch_joint", -0.1204, 0.25 * 118.2 / 60.0, 60.0, 1.2),
    ("right_ankle_roll_joint", 0.0078, 0.25 * 54.75 / 60.0, 60.0, 1.2),
    ("right_hip_yaw_joint", 0.0348, 0.6875, 80.0, 3.0),
    ("left_shoulder_pitch_joint", 0.3, 0.375, 40.0, 3.0),
    ("right_shoulder_roll_joint", -0.12, 0.375, 40.0, 3.0),
    ("left_elbow_joint", 0.8, 0.2, 30.0, 2.0),
    ("left_wrist_pitch_joint", 0.0, 0.075, 20.0, 2.0),
    ("right_wrist_yaw_joint", 0.0, 0.075, 20.0, 2.0),
])
def test_024_tables_match_the_hpp_in_both_orders(name, default, scale, kp, kd):
    isaaclab = contract.ISAACLAB_JOINT_NAMES.index(name)
    cadence = contract.CADENCE_JOINT_NAMES.index(name)
    assert contract.DEFAULT_ANGLES_ISAACLAB[isaaclab] == pytest.approx(default)
    assert contract.ACTION_SCALE_024_ISAACLAB[isaaclab] == pytest.approx(scale)
    assert contract.KP_024_ISAACLAB[isaaclab] == pytest.approx(kp)
    assert contract.KD_024_ISAACLAB[isaaclab] == pytest.approx(kd)
    assert contract.DEFAULT_ANGLES_CADENCE[cadence] == pytest.approx(default)
    assert contract.KP_024_CADENCE[cadence] == pytest.approx(kp)
    assert contract.KD_024_CADENCE[cadence] == pytest.approx(kd)


def test_default_pose_matches_cadence_lower_contract():
    from cadence.motion.a3 import DEFAULT_JOINT_POSITION
    np.testing.assert_allclose(contract.DEFAULT_ANGLES_CADENCE, DEFAULT_JOINT_POSITION)


def test_observation_layout_constants():
    assert contract.OBSERVATION_DIM == 1570
    assert contract.TOKENIZER_DIM == 640
    assert contract.PROPRIO_DIM == 930
    assert contract.COMMAND_DIM == 580 == 10 * 29 * 2
    assert contract.ANCHOR_DIM == 60 == 10 * 6
    assert contract.PROPRIO_PER_STEP == 93 == 3 + 29 + 29 + 29 + 3
    assert contract.TOKENIZER_DIM + contract.PROPRIO_DIM == contract.OBSERVATION_DIM
    assert contract.ACTION_CLIP == 20.0
    assert contract.POLICY_DT == pytest.approx(0.02)


def test_proprio_term_layout_boundaries():
    widths = [width for _, width in contract.PROPRIO_TERMS]
    assert [name for name, _ in contract.PROPRIO_TERMS] == [
        "base_ang_vel", "joint_pos_rel", "joint_vel_rel", "last_raw_action", "gravity_dir"]
    assert sum(widths) * 10 == 930
    # Newest-frame offsets inside the 930-float block.
    offset = 0
    for name, width in contract.PROPRIO_TERMS:
        assert width * 10 % 10 == 0
        offset += width * 10
    assert offset == 930


def test_quaternion_helpers():
    np.testing.assert_allclose(contract.gravity_direction([1, 0, 0, 0]), [0, 0, -1])
    yaw = 0.3
    q = contract.quat_from_yaw(yaw)
    assert contract.quat_yaw(q) == pytest.approx(yaw)
    assert contract.compute_yaw_offset(q, [1, 0, 0, 0]) == pytest.approx(yaw)
    anchor = contract.anchor_6d([1, 0, 0, 0], [1, 0, 0, 0])
    np.testing.assert_allclose(anchor, [1, 0, 0, 1, 0, 0], atol=1e-12)
    # 6D anchor of a pure-yaw rotation is a z-axis rotation matrix's first columns.
    anchor = contract.anchor_6d([1, 0, 0, 0], q)
    np.testing.assert_allclose(
        anchor, [np.cos(yaw), -np.sin(yaw), np.sin(yaw), np.cos(yaw), 0, 0], atol=1e-12)
