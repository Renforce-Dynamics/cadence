"""The visible A3 hands must participate in self-collision, beyond the wrists."""
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
from cadence.backends.a3 import A3_POLICY_JOINTS

ROOT = Path(__file__).resolve().parents[1]


def test_recorded_crossed_hands_generate_physical_contact():
    model = mujoco.MjModel.from_xml_path(str(ROOT / "assets/a3_mujoco/a3.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    # Measured Mimic posture at t=0.22 s, formerly 8 cm of invisible-to-physics
    # hand overlap. Joint order stays the deployment's existing 29-D contract.
    joints = [
        -.0626526, .01923669, .04906183, .17113792, -.07746359, -.01759071,
        -.06040012, -.02050932, -.08790953, .18698306, -.09189732, .01045723,
        .00836606, -.00533316, -.03692699,
        -.40465579, .02171713, -.84148293, -.16088424, -.45597315, -.2434638, .60200302,
        -.41842057, -.0177999, .81898632, -.17880151, .48333623, -.09951916, -.61255246,
    ]
    data.qpos[[model.joint(n).qposadr[0] for n in A3_POLICY_JOINTS]] = joints
    mujoco.mj_forward(model, data)
    hands = {model.geom(f"{side}_hand_collision").id for side in ("left", "right")}
    contacts = [c for c in data.contact if set(c.geom) == hands]
    assert contacts and min(c.dist for c in contacts) < -.05
    for geom in hands:
        assert model.geom_contype[geom] and model.geom_conaffinity[geom]
    assert np.isfinite(data.qpos).all()
