"""Head stops must constrain motion even though the policy has no neck actions."""
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
ROOT = Path(__file__).resolve().parents[1]


def test_passive_head_cannot_rotate_past_agi3dep_stops_under_load():
    model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/a3_mujoco/a3.xml"))
    data = mujoco.MjData(model)
    head = [model.joint(f"head_{axis}_joint") for axis in ("yaw", "pitch")]
    qpos = np.array([j.qposadr[0] for j in head])
    qvel = np.array([j.dofadr[0] for j in head])
    # The former range="0 0" compiled with limited=False, allowing a full flip.
    assert all(j.limited[0] for j in head)
    ranges = np.deg2rad([[-60, 60], [-25, 15]])
    np.testing.assert_allclose([j.range for j in head], ranges, atol=3e-6)

    # Fixture the rest of the real asset; gravity and neck dynamics remain live.
    other_q = np.setdiff1d(np.arange(model.nq), qpos)
    other_v = np.setdiff1d(np.arange(model.nv), qvel)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    initial = data.qpos.copy()
    for force, stop in ((2., 1), (-2., 0)):
        for _ in range(round(3 / model.opt.timestep)):
            data.qpos[other_q] = initial[other_q]
            data.qvel[other_v] = 0.
            data.qfrc_applied[qvel] = force
            mujoco.mj_step(model, data)
            # MuJoCo joint stops are compliant, not infinitely rigid.
            assert np.all(data.qpos[qpos] >= ranges[:, 0] - .02)
            assert np.all(data.qpos[qpos] <= ranges[:, 1] + .02)
        np.testing.assert_allclose(data.qpos[qpos], ranges[:, stop], atol=.005)
