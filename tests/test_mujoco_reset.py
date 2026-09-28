"""Physical simulation reset is separate from operator safety reset."""
from pathlib import Path
import numpy as np
import pytest
import yaml

mujoco = pytest.importorskip("mujoco")

from cadence.backends.mujoco import MujocoBackend
from cadence.deployment import load_run_config, prepare_deployment, run_config

ROOT = Path(__file__).resolve().parents[1]


def test_reset_restores_configured_free_base_pose_and_invalidates_old_writes():
    from cadence_api import JointCommand
    from cadence.backends.a3 import A3_POLICY_JOINTS

    backend = MujocoBackend(
        ROOT / "assets/a3_mujoco/a3.xml", A3_POLICY_JOINTS,
        physics_timestep_s=.001, initial_joint_position=np.zeros(29),
    )
    initial = backend.data.qpos.copy()
    sequence = backend.read_state().sequence
    backend.data.qpos[:3] = [3, 4, .1]
    backend.data.qvel[:] = 2.
    backend.data.ctrl[:] = 1.
    backend.data.xfrc_applied[:] = 4.
    backend.reset()
    np.testing.assert_array_equal(backend.data.qpos, initial)
    np.testing.assert_array_equal(backend.data.qvel, 0.)
    np.testing.assert_array_equal(backend.data.ctrl, 0.)
    np.testing.assert_array_equal(backend.data.xfrc_applied, 0.)
    assert backend.read_state().sequence > sequence
    zero = np.zeros(29)
    with pytest.raises(RuntimeError, match="sequence mismatch"):
        backend.write_command(JointCommand(zero, zero, zero, zero, zero), sequence)


@pytest.mark.parametrize("key", [259, ord("R"), None])
def test_window_reset_restores_pose_and_keeps_fixedpos_selected(tmp_path, monkeypatch, key):
    import mujoco.viewer
    import cadence.deployment as deployment

    entry = tmp_path / "sim.yaml"
    entry.write_text(yaml.safe_dump({
        "extends": str(ROOT / "configs/entry/examples/entry_sim.yaml"),
        "runtime": {"duration_s": .08, "headless": False},
    }))
    plan = prepare_deployment(load_run_config(entry))
    backend = plan.backend
    initial = backend.data.qpos.copy()
    snapshots, callback, resets = [], [], []
    reset = backend.reset

    def physical_reset():
        reset()
        resets.append(backend.data.qpos.copy())

    class Viewer:
        def __enter__(self):
            return self

        def __exit__(self, *_):
            pass

        def is_running(self):
            return True

        def sync(self):
            snapshots.append(backend.data.qpos.copy())
            if len(snapshots) == 1:
                backend.data.qpos[:] = .7
                backend.data.qvel[:] = 2.
                if key is None:
                    # UI Reset is applied inside viewer.sync, without a key.
                    backend.data.time = 0.
                else:
                    callback[0](key)

    def launch(*args, key_callback):
        callback.append(key_callback)
        return Viewer()

    monkeypatch.setattr(deployment, "prepare_deployment", lambda _: plan)
    monkeypatch.setattr(backend, "reset", physical_reset)
    monkeypatch.setattr(mujoco.viewer, "launch_passive", launch)
    monkeypatch.setattr(deployment.time, "sleep", lambda _: None)
    output = tmp_path / "output"
    assert run_config(entry, output=output) == 0
    assert len(resets) == 1
    np.testing.assert_array_equal(resets[0], initial)
    import json
    summary = json.loads((output / "result.json").read_text())
    assert summary["mode"] == "fixed_position"
