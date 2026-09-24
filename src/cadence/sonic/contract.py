"""SONIC A3 whole-body policy contract constants and math helpers.

Ported from the SONIC-for-A3 release (Apache License 2.0, AgiBot Inc. 2026),
``repos/sonic_for_a3``:

- ``gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/a3_policy_parameters.hpp``
  (per-joint 024 gains, default angles, action scales, IsaacLab<->MuJoCo DoF
  permutation)
- ``gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/a3_deploy/a3_obs_builder.hpp``
  and ``src/a3_deploy/a3_obs_builder.cpp`` (observation layout)
- ``gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/a3_deploy/a3_action_decoder.cpp``
  (action decode)
- ``gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/src/a3_deploy/a3_csv_motion_reference.cpp``
  and ``src/a3_deploy/a3_teleop_reference.cpp`` (tokenizer prefix, yaw lock)
- ``gear_sonic/envs/manager_env/robots/a3.py`` (``A3_ACTIVE_JOINT_NAMES``,
  ``A3_ISAACLAB_TO_MUJOCO_DOF``)

The policy consumes and produces 29-vectors in IsaacLab joint order
(``ISAACLAB_JOINT_NAMES``). Cadence robot state and commands use the canonical
backend order (``CADENCE_JOINT_NAMES`` == ``cadence.backends.a3.A3_POLICY_JOINTS``).
Per-joint tables below are stored verbatim in the SONIC 29-DOF MuJoCo policy
view and reordered into IsaacLab and cadence order at import; assertions keep
every convention consistent.

Note: a3.py's ``A3_MUJOCO_TO_ISAACLAB_DOF`` literal is buggy (contains 29/30);
the corrected inverse is derived here via argsort, matching the C++ header's
``a3_mujoco_to_isaaclab``.
"""

from __future__ import annotations

import math

import numpy as np

from ..backends.a3 import A3_POLICY_JOINTS

# ---------------------------------------------------------------------------
# Joint orders
# ---------------------------------------------------------------------------

# IsaacLab training order (verbatim A3_ACTIVE_JOINT_NAMES from a3.py).
ISAACLAB_JOINT_NAMES = (
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
    "left_knee_joint", "right_knee_joint",
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
    "left_elbow_joint", "right_elbow_joint",
    "left_wrist_roll_joint", "right_wrist_roll_joint",
    "left_wrist_pitch_joint", "right_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_wrist_yaw_joint",
)

# Cadence canonical backend order: legs L/R, waist, arms L/R.
CADENCE_JOINT_NAMES = A3_POLICY_JOINTS

JOINT_DIM = 29

# Verbatim A3_ISAACLAB_TO_MUJOCO_DOF (a3.py / a3_policy_parameters.hpp).
# Gather-index convention: mujoco_vector[m] == isaaclab_vector[table[m]].
A3_ISAACLAB_TO_MUJOCO_DOF = (
    2, 5, 8, 11, 15, 19, 21, 23, 25, 27,
    12, 16, 20, 22, 24, 26, 28,
    0, 3, 6, 9, 13, 17,
    1, 4, 7, 10, 14, 18,
)

# MuJoCo 29-DOF policy view order (waist, L arm, R arm, L leg, R leg),
# derived from the name tables and the forward permutation.
MUJOCO_JOINT_NAMES = tuple(ISAACLAB_JOINT_NAMES[i] for i in A3_ISAACLAB_TO_MUJOCO_DOF)

# Corrected inverse permutation (argsort): isaaclab_vector[i] == mujoco_vector[table[i]].
A3_MUJOCO_TO_ISAACLAB_DOF = tuple(int(i) for i in np.argsort(np.asarray(A3_ISAACLAB_TO_MUJOCO_DOF)))
_A3_MUJOCO_TO_ISAACLAB_REFERENCE = (
    17, 23, 0, 18, 24, 1, 19, 25, 2, 20,
    26, 3, 10, 21, 27, 4, 11, 22, 28, 5,
    12, 6, 13, 7, 14, 8, 15, 9, 16,
)
assert A3_MUJOCO_TO_ISAACLAB_DOF == _A3_MUJOCO_TO_ISAACLAB_REFERENCE, (
    "argsort must reproduce the corrected a3_mujoco_to_isaaclab table"
)

# Position map: IsaacLab index -> cadence index. Also the gather table for the
# cadence -> IsaacLab conversion: isaaclab_vector[i] == cadence_vector[table[i]].
ISAACLAB_TO_CADENCE = tuple(CADENCE_JOINT_NAMES.index(name) for name in ISAACLAB_JOINT_NAMES)
# Argsort-derived inverse; the gather table for IsaacLab -> cadence conversion:
# cadence_vector[j] == isaaclab_vector[table[j]].
CADENCE_TO_ISAACLAB = tuple(int(i) for i in np.argsort(np.asarray(ISAACLAB_TO_CADENCE)))
assert tuple(ISAACLAB_JOINT_NAMES[i] for i in CADENCE_TO_ISAACLAB) == CADENCE_JOINT_NAMES

# ---------------------------------------------------------------------------
# Per-joint 024 tables, verbatim from a3_policy_parameters.hpp in the MuJoCo
# 29-DOF policy view (MUJOCO_JOINT_NAMES order).
# ---------------------------------------------------------------------------

_DEFAULT_ANGLES_MUJOCO = (
    0.0, 0.0, 0.0,                                    # waist
    0.3, 0.12, 0.0, 0.8, 0.0, 0.0, 0.0,               # left arm
    0.3, -0.12, 0.0, 0.8, 0.0, 0.0, 0.0,              # right arm
    -0.1311, 0.0056, -0.0348, 0.2468, -0.1204, -0.0078,  # left leg
    -0.1311, -0.0056, 0.0348, 0.2468, -0.1204, 0.0078,   # right leg
)

# 024 profile: base a3_kps with the four ankle entries raised to 60.0.
_KP_024_MUJOCO = (
    85.0, 50.0, 50.0,
    40.0, 40.0, 30.0, 30.0, 30.0, 20.0, 20.0,
    40.0, 40.0, 30.0, 30.0, 30.0, 20.0, 20.0,
    80.0, 120.0, 80.0, 250.0, 60.0, 60.0,
    80.0, 120.0, 80.0, 250.0, 60.0, 60.0,
)

# Real-robot PD_STAND bring-up gains (a3_pd_stand_kps/kds in the hpp).
_KP_PD_STAND_MUJOCO = (
    400.0, 500.0, 500.0,
    200.0, 200.0, 100.0, 200.0, 100.0, 50.0, 50.0,
    200.0, 200.0, 100.0, 200.0, 100.0, 50.0, 50.0,
    1500.0, 400.0, 300.0, 2000.0, 500.0, 500.0,
    1500.0, 400.0, 300.0, 2000.0, 500.0, 500.0,
)

# 024 profile: base a3_kds with the four ankle entries lowered to 1.2.
_KD_024_MUJOCO = (
    3.0, 2.0, 2.0,
    3.0, 3.0, 2.0, 2.0, 2.0, 2.0, 2.0,
    3.0, 3.0, 2.0, 2.0, 2.0, 2.0, 2.0,
    3.0, 4.0, 3.0, 8.0, 1.2, 1.2,
    3.0, 4.0, 3.0, 8.0, 1.2, 1.2,
)

_KD_PD_STAND_MUJOCO = (
    4.0, 4.0, 4.0,
    2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0,
    8.0, 7.0, 7.0, 8.0, 5.0, 5.0,
    8.0, 7.0, 7.0, 8.0, 5.0, 5.0,
)

# 024 profile action scale: 0.25 * effort_limit_sim / stiffness per joint.
_ACTION_SCALE_024_MUJOCO = (
    0.25 * 200.0 / 85.0,   # waist_yaw
    0.25 * 42.0 / 50.0,    # waist_roll
    0.25 * 105.0 / 50.0,   # waist_pitch
    0.375, 0.375, 0.2, 0.2, 0.2, 0.075, 0.075,   # left arm
    0.375, 0.375, 0.2, 0.2, 0.2, 0.075, 0.075,   # right arm
    0.6875, 0.4583333333333333, 0.6875, 0.25 * 300.0 / 250.0,
    0.25 * 118.2 / 60.0, 0.25 * 54.75 / 60.0,    # left leg
    0.6875, 0.4583333333333333, 0.6875, 0.25 * 300.0 / 250.0,
    0.25 * 118.2 / 60.0, 0.25 * 54.75 / 60.0,    # right leg
)


def _to_isaaclab(values_mujoco):
    return tuple(float(values_mujoco[m]) for m in A3_MUJOCO_TO_ISAACLAB_DOF)


def _to_cadence(values_isaaclab):
    return tuple(float(values_isaaclab[i]) for i in CADENCE_TO_ISAACLAB)


# IsaacLab-order contract vectors (policy I/O frame).
DEFAULT_ANGLES_ISAACLAB = _to_isaaclab(_DEFAULT_ANGLES_MUJOCO)
KP_024_ISAACLAB = _to_isaaclab(_KP_024_MUJOCO)
KD_024_ISAACLAB = _to_isaaclab(_KD_024_MUJOCO)
ACTION_SCALE_024_ISAACLAB = _to_isaaclab(_ACTION_SCALE_024_MUJOCO)

# Cadence-order vectors (robot command frame).
DEFAULT_ANGLES_CADENCE = _to_cadence(DEFAULT_ANGLES_ISAACLAB)
KP_024_CADENCE = _to_cadence(KP_024_ISAACLAB)
KD_024_CADENCE = _to_cadence(KD_024_ISAACLAB)
KP_PD_STAND_CADENCE = _to_cadence(_to_isaaclab(_KP_PD_STAND_MUJOCO))
KD_PD_STAND_CADENCE = _to_cadence(_to_isaaclab(_KD_PD_STAND_MUJOCO))

# ---------------------------------------------------------------------------
# Observation layout ([1,1570] float32, 50 Hz)
# ---------------------------------------------------------------------------

POLICY_HZ = 50
POLICY_DT = 1.0 / POLICY_HZ
FUTURE_FRAMES = 10
COMMAND_DIM = FUTURE_FRAMES * JOINT_DIM * 2       # 580: q[10x29] then dq[10x29]
ANCHOR_DIM = FUTURE_FRAMES * 6                    # 60
TOKENIZER_DIM = COMMAND_DIM + ANCHOR_DIM          # 640
PROPRIO_HISTORY = 10
PROPRIO_PER_STEP = 3 + JOINT_DIM + JOINT_DIM + JOINT_DIM + 3  # 93
PROPRIO_DIM = PROPRIO_HISTORY * PROPRIO_PER_STEP  # 930
OBSERVATION_DIM = TOKENIZER_DIM + PROPRIO_DIM     # 1570
ACTION_CLIP = 20.0

# Term-major proprio layout, each term oldest -> newest over 10 frames:
# base_ang_vel(3), joint_pos_rel(29), joint_vel_rel(29), last_raw_action(29),
# gravity_dir(3).
PROPRIO_TERMS = (
    ("base_ang_vel", 3),
    ("joint_pos_rel", JOINT_DIM),
    ("joint_vel_rel", JOINT_DIM),
    ("last_raw_action", JOINT_DIM),
    ("gravity_dir", 3),
)

# ---------------------------------------------------------------------------
# Quaternion helpers (wxyz), ported from the SONIC C++ reference.
# ---------------------------------------------------------------------------


def quat_normalize(q):
    q = np.asarray(q, dtype=np.float64).reshape(4)
    norm = np.linalg.norm(q)
    if norm <= np.finfo(np.float64).eps:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def quat_conjugate(q):
    q = np.asarray(q, dtype=np.float64).reshape(4)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_multiply(a, b):
    w1, x1, y1, z1 = np.asarray(a, dtype=np.float64).reshape(4)
    w2, x2, y2, z2 = np.asarray(b, dtype=np.float64).reshape(4)
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_from_yaw(yaw_rad):
    half = 0.5 * float(yaw_rad)
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)])


def quat_yaw(q):
    w, x, y, z = quat_normalize(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(angle):
    angle = float(angle)
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quat_apply(q, v):
    """Rotate vector ``v`` by quaternion ``q`` (wxyz)."""
    qv = quat_multiply(quat_multiply(q, np.concatenate(([0.0], np.asarray(v, dtype=np.float64).reshape(3)))),
                       quat_conjugate(q))
    return qv[1:]


def gravity_direction(root_quat_wxyz):
    """Projected gravity: quat_apply(quat_inv(pelvis_quat), [0, 0, -1])."""
    return quat_apply(quat_conjugate(quat_normalize(root_quat_wxyz)),
                      np.array([0.0, 0.0, -1.0]))


def quat_slerp(qa, qb, alpha):
    a = quat_normalize(qa)
    b = quat_normalize(qb)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        dot, b = -dot, -b
    if dot > 0.9995:
        return quat_normalize((1.0 - alpha) * a + alpha * b)
    dot = min(1.0, max(-1.0, dot))
    theta0 = math.acos(dot)
    theta = theta0 * alpha
    sin_theta, sin_theta0 = math.sin(theta), math.sin(theta0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta0
    s1 = sin_theta / sin_theta0
    return quat_normalize(s0 * a + s1 * b)


def rotation_6d_first_two_columns(q):
    """First two rotation-matrix columns, flattened column-wise.

    Emits (m00, m01, m10, m11, m20, m21), matching WriteOrientation6d.
    """
    w, x, y, z = quat_normalize(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        1.0 - 2.0 * (yy + zz),  # m00
        2.0 * (xy - wz),        # m01
        2.0 * (xy + wz),        # m10
        1.0 - 2.0 * (xx + zz),  # m11
        2.0 * (xz - wy),        # m20
        2.0 * (yz + wx),        # m21
    )


def anchor_6d(robot_root_quat_wxyz, reference_quat_wxyz, yaw_offset_rad=0.0):
    """6D orientation of the yaw-aligned reference root relative to the robot.

    rel = inv(robot_root_quat) * (yaw_offset_quat * reference_quat)
    """
    inv_robot = quat_conjugate(quat_normalize(robot_root_quat_wxyz))
    aligned = quat_multiply(quat_from_yaw(yaw_offset_rad),
                            quat_normalize(reference_quat_wxyz))
    return rotation_6d_first_two_columns(quat_multiply(inv_robot, aligned))


def compute_yaw_offset(robot_root_quat_wxyz, reference_quat_wxyz):
    """wrap_pi(yaw(robot) - yaw(reference)); locks the reference heading."""
    return wrap_pi(quat_yaw(robot_root_quat_wxyz) - quat_yaw(reference_quat_wxyz))


def cadence_to_isaaclab(vector):
    vector = np.asarray(vector, dtype=np.float64).reshape(JOINT_DIM)
    return vector[np.asarray(ISAACLAB_TO_CADENCE)]


def isaaclab_to_cadence(vector):
    vector = np.asarray(vector, dtype=np.float64).reshape(JOINT_DIM)
    return vector[np.asarray(CADENCE_TO_ISAACLAB)]
