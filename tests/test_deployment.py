"""Standalone deployment contracts, including hardware failures without hardware."""

from dataclasses import replace
import json
from pathlib import Path
import socket
import signal
import subprocess
import sys

import numpy as np
import pytest
import yaml

from cadence.backends.a3 import A3Backend, CONTROL_JOINT_NAMES
from cadence.backends.mock import MockBackend
from cadence.cli import main, run_config
from cadence.plugins import ControlState, ControlResult
from cadence_api import JointCommand, RobotState


class ProgressState(ControlState):
    instances = []

    def __init__(self, *args):
        super().__init__(*args)
        self.applied = 0
        self.rejected = 0
        self.exited = False
        self.instances.append(self)

    def on_enter(self, frame, events):
        self.initial_localization = frame.localization

    def step(self, frame):
        n = self.services.dimension
        q = np.full(n, .01 * (self.applied + 1))
        return ControlResult(JointCommand(q, np.zeros(n), np.ones(n), np.ones(n), np.zeros(n)))

    def on_command_applied(self, command):
        self.applied += 1

    def on_command_rejected(self):
        self.rejected += 1

    def on_exit(self, frame, events):
        self.exited = True


class WorldState(ProgressState):
    requires_world_root = True

    def on_enter(self, frame, events):
        # A state is allowed to rely on its declared entry requirement.
        assert frame.localization is not None
        super().on_enter(frame, events)


class StaleCommand(RuntimeError):
    pass


class HardwareFixture:
    state_timeout_s = .001
    startup_timeout_s = .1
    command_timeout_ms = 1
    command_publish_enabled = False

    def __init__(self, *, read_only=False, failure=None):
        self.read_only = read_only
        self.failure = failure
        self.reads = self.attempts = 0
        self.writes = []
        self.closed = False
        self.watchdog_trip_count = 0

    def start(self):
        pass

    def read_state(self, timeout_s=None):
        self.reads += 1
        if self.failure == "timeout" and self.reads >= 3:
            raise TimeoutError("state unavailable")
        if self.failure == "watchdog" and self.reads >= 3:
            self.watchdog_trip_count = 1
        return RobotState(self.reads, self.reads * 20_000_000, np.zeros(3),
                          np.array([1, 0, 0, 0]), np.zeros(29), np.zeros(29), np.zeros(29))

    def write_command(self, command, sequence):
        self.attempts += 1
        if self.failure == "stale" and self.attempts == 1:
            raise StaleCommand("expired delivered state")
        if self.failure == "transport":
            raise RuntimeError("transport rejected command")
        assert not self.read_only
        self.writes.append(command.q_des.copy())

    def is_state_stale_error(self, error):
        return isinstance(error, StaleCommand)

    def close(self):
        self.closed = True


def hardware_config(tmp_path, *, read_only=False):
    path = tmp_path / "hardware.yaml"
    path.write_text(yaml.safe_dump({
        "schema_version": 1,
        "robot": {"joints": list(CONTROL_JOINT_NAMES), "position_min": [-1] * 29, "position_max": [1] * 29},
        "backend": {"kind": "a3", "transport": "sdk_mock", "read_only": read_only},
        "runtime": {"control_hz": 50, "deadline_ms": 20, "duration_s": .045, "start_state": "progress"},
        "catalog": {"reset_state_id": 0, "safety_fallback_state_id": 1, "states": {
            0: {"key": "progress", "factory": f"{__name__}:ProgressState", "config": {}},
            1: {"key": "damping", "factory": "cadence.plugins:BasicState", "config": {"kind": "damping"}},
        }},
    }))
    return path


@pytest.mark.parametrize("read_only", [False, True])
def test_hardware_and_readonly_deployments_report_actual_write_boundary(tmp_path, monkeypatch, read_only):
    backend = HardwareFixture(read_only=read_only)
    monkeypatch.setattr(A3Backend, "from_mapping", lambda *a, **kw: backend)
    output = tmp_path / "result"
    assert run_config(hardware_config(tmp_path, read_only=read_only), output=output) == 0
    result = json.loads((output / "result.json").read_text())
    state = ProgressState.instances[-1]
    assert state.exited and backend.closed
    assert result["read_only"] is read_only
    assert state.applied == result["ticks"] > 0
    if read_only:
        assert not backend.writes and backend.attempts == 0
        assert result["shadow_steps"] == result["ticks"] and result["command_writes"] == 0
    else:
        assert len(backend.writes) == result["command_writes"] == state.applied
        assert result["shadow_steps"] == 0


def test_stale_sdk_command_does_not_advance_progress_and_next_fresh_frame_can_commit(tmp_path, monkeypatch):
    backend = HardwareFixture(failure="stale")
    monkeypatch.setattr(A3Backend, "from_mapping", lambda *a, **kw: backend)
    assert run_config(hardware_config(tmp_path), output=tmp_path / "result") == 0
    state = ProgressState.instances[-1]
    result = json.loads((tmp_path / "result/result.json").read_text())
    assert state.rejected == result["stale_command_skips"] == 1
    assert state.applied == len(backend.writes) > 0
    np.testing.assert_allclose(backend.writes[0], .01)
    assert backend.closed


@pytest.mark.parametrize("failure", ["timeout", "watchdog"])
def test_hardware_loss_stops_submission_and_closes_backend(tmp_path, monkeypatch, failure):
    backend = HardwareFixture(failure=failure)
    monkeypatch.setattr(A3Backend, "from_mapping", lambda *a, **kw: backend)
    assert run_config(hardware_config(tmp_path), output=tmp_path / "result") == 2
    result = json.loads((tmp_path / "result/result.json").read_text())
    assert result["halted"] and result["failure"]
    assert len(backend.writes) == 1
    assert ProgressState.instances[-1].exited and backend.closed


def test_transport_failure_rejects_pending_progress_and_cleans_up(tmp_path, monkeypatch):
    backend = HardwareFixture(failure="transport")
    monkeypatch.setattr(A3Backend, "from_mapping", lambda *a, **kw: backend)
    with pytest.raises(RuntimeError, match="transport rejected"):
        run_config(hardware_config(tmp_path))
    state = ProgressState.instances[-1]
    assert state.rejected == 1 and state.applied == 0
    assert state.exited and backend.closed


def test_stop_during_state_wait_prevents_new_submission(tmp_path, monkeypatch):
    class StoppingBackend(HardwareFixture):
        def read_state(self, timeout_s=None):
            state = super().read_state(timeout_s)
            if self.reads == 2:
                signal.raise_signal(signal.SIGTERM)
            return state

    backend = StoppingBackend()
    monkeypatch.setattr(A3Backend, "from_mapping", lambda *a, **kw: backend)
    assert run_config(hardware_config(tmp_path)) == 0
    assert not backend.writes and backend.closed


def test_a3_layout_check_rejects_permuted_names_before_sdk_start(tmp_path, monkeypatch):
    path = hardware_config(tmp_path)
    cfg = yaml.safe_load(path.read_text())
    cfg["robot"]["joints"][:2] = reversed(cfg["robot"]["joints"][:2])
    path.write_text(yaml.safe_dump(cfg))
    monkeypatch.setattr(A3Backend, "start", lambda _: pytest.fail("must not start SDK"))
    with pytest.raises(ValueError, match="canonical 29 joint names"):
        run_config(path, check=True)


def test_check_loads_states_without_opening_sdk_or_localization_socket(tmp_path, monkeypatch, capsys):
    from cadence.localization import LocalizationReceiver
    path = hardware_config(tmp_path)
    cfg = yaml.safe_load(path.read_text())
    cfg["runtime"]["localization"] = {"port": 0}
    path.write_text(yaml.safe_dump(cfg))
    monkeypatch.setattr(A3Backend, "start", lambda _: pytest.fail("check opened SDK"))
    monkeypatch.setattr(LocalizationReceiver, "__init__", lambda *a: pytest.fail("check opened UDP"))
    assert run_config(path, check=True, output=tmp_path / "snapshot") == 0
    assert json.loads(capsys.readouterr().out)["started"] is False


def test_deploy_script_preserves_caller_paths_and_snapshots_effective_flags(tmp_path):
    script = Path(__file__).resolve().parents[1] / "scripts/deploy.sh"
    folder = tmp_path / "site folder"
    folder.mkdir()
    config = folder / "entry config.yaml"
    config.write_text("extends: pkg://cadence/data/demo.yaml\n")
    subprocess.run([str(script), "--venv", sys.prefix, "--config", "site folder/entry config.yaml",
                    "--duration-s", ".02", "--backend", "mock", "--set", "runtime.control_hz=100",
                    "--output", "run folder", "--check"], cwd=tmp_path, check=True, capture_output=True, text=True)
    target = tmp_path / "run folder"
    effective = yaml.safe_load((target / "resolved.yaml").read_text())
    assert effective["runtime"]["duration_s"] == .02 and effective["runtime"]["control_hz"] == 100
    assert effective["backend"]["kind"] == "mock"
    overrides = json.loads((target / "overrides.json").read_text())
    assert "runtime.duration_s=0.02" in overrides and "runtime.control_hz=100" in overrides


def test_deploy_creates_default_snapshot_under_caller_directory(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    assert main(["deploy", "--check"]) == 0
    targets = list((tmp_path / "runs").glob("deploy-*/deployment.json"))
    assert len(targets) == 1


def test_initial_world_state_requires_pose_before_on_enter(tmp_path, monkeypatch):
    path = hardware_config(tmp_path)
    cfg = yaml.safe_load(path.read_text())
    cfg["catalog"]["states"][0]["factory"] = f"{__name__}:WorldState"
    path.write_text(yaml.safe_dump(cfg))
    backend = HardwareFixture()
    monkeypatch.setattr(A3Backend, "from_mapping", lambda *a, **kw: backend)
    with pytest.raises(ValueError, match="start_state requires localization"):
        run_config(path)
    assert not backend.writes and backend.closed


def test_external_localization_reaches_initial_entry_and_expires_without_backend_fallback(tmp_path, monkeypatch):
    from cadence_protocol.localization import LocalizationClient
    import cadence.runtime as runtime_module

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as free:
        free.bind(("127.0.0.1", 0))
        port = free.getsockname()[1]
    observations = []
    real_kernel = runtime_module.RuntimeKernel

    class ObservedKernel(real_kernel):
        def prepare(self, value):
            observations.append(value.localization)
            return super().prepare(value)

    class LocatedBackend(HardwareFixture):
        def start(self):
            with LocalizationClient("127.0.0.1", port, ttl_s=.025) as sender:
                sender.send((1., 2., 3.), (1., 0., 0., 0.), (0., 0., 0.))

        def read_state(self, timeout_s=None):
            return replace(super().read_state(timeout_s), root_position_w=np.full(3, 99.), root_linear_velocity_w=np.zeros(3))

    backend = LocatedBackend()
    monkeypatch.setattr(A3Backend, "from_mapping", lambda *a, **kw: backend)
    monkeypatch.setattr(runtime_module, "RuntimeKernel", ObservedKernel)
    path = hardware_config(tmp_path)
    cfg = yaml.safe_load(path.read_text())
    cfg["runtime"]["localization"] = {"port": port}
    path.write_text(yaml.safe_dump(cfg))
    assert run_config(path, duration=.065) == 0
    np.testing.assert_allclose(ProgressState.instances[-1].initial_localization.position_w, (1, 2, 3))
    assert observations[0] is not None and observations[-1] is None
    assert all(x is None or np.array_equal(x.position_w, (1., 2., 3.)) for x in observations)
