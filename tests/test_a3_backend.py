"""Hardware adapter contract tests; no test configures a real transport."""

from dataclasses import fields
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest

from cadence_api import JointCommand, RobotState
from cadence.backends import a3


def state(sequence=7):
    return RobotState(
        sequence, 123456789, np.array([1.0, 2.0, 3.0]), np.array([1.0, 0, 0, 0]),
        np.arange(29) / 100, np.zeros(29), np.ones(29),
        sync_skew_ns=250, has_torso_imu=True,
        torso_gyro_b=np.array([4.0, 5.0, 6.0]),
        root_position_w=np.array([1.0, 2.0, 3.0]),
    )


def command():
    zeros = np.zeros(29)
    return JointCommand(zeros, zeros, zeros, zeros, zeros)


def aimrt_config(tmp_path, *, publish=False):
    path = tmp_path / "aimrt.yaml"
    path.write_text("aimrt: {}\n")
    return {
        "kind": "a3", "transport": "aimrt",
        "read_only": not publish, "command_publish_enabled": publish,
        "aimrt": {
            "config_path": "aimrt.yaml", "maximum_sync_skew_ms": 12.5,
            "state_topics": {name: f"/state/{name}" for name in a3.STATE_TOPICS},
            "command_topics": {name: f"/command/{name}" for name in a3.COMMAND_TOPICS},
        },
    }


@pytest.fixture
def fake_sdk(monkeypatch):
    instances = []

    class FakeIO:
        def __init__(self, **kwargs):
            self.options = kwargs
            self.calls = []
            self.state = state()
            self.watchdog_trip_count = 3
            self.aimrt_stats = {"command_batches": 4}
            self.write_error = self.read_error = None
            instances.append(self)

        def start(self):
            self.calls.append("start")

        def close(self):
            self.calls.append("close")

        def read_state(self, timeout_s):
            self.calls.append(("read", timeout_s))
            if self.read_error:
                raise self.read_error
            return self.state

        def write_command(self, cmd, sequence):
            self.calls.append(("write", cmd, sequence))
            if self.write_error:
                raise self.write_error

        def _inject_hardware_state_for_test(self, sequence, q):
            self.calls.append(("inject", sequence, tuple(q)))

        def _install_memory_command_writer_for_test(self):
            self.calls.append("memory_writer")

    monkeypatch.setattr(a3, "_load_sdk", lambda: SimpleNamespace(A3RobotIO=FakeIO))
    return instances, FakeIO


def test_hardware_publish_gate_precedes_sdk_construction(tmp_path, fake_sdk, monkeypatch):
    monkeypatch.delenv("A3_CONFIRM_ONBOARD", raising=False)
    backend = a3.A3Backend.from_mapping(aimrt_config(tmp_path, publish=True), base_dir=tmp_path)
    assert fake_sdk[0] == []  # --check can construct without hardware or confirmation.
    with pytest.raises(RuntimeError, match="A3_CONFIRM_ONBOARD=YES"):
        backend.start()
    assert fake_sdk[0] == []
    monkeypatch.setenv("A3_CONFIRM_ONBOARD", "YES")
    with backend:
        assert fake_sdk[0][0].options["command_publish_enabled"] is True
        assert fake_sdk[0][0].options["maximum_sync_skew_ns"] == 12500000
    assert fake_sdk[0][0].calls == ["start", "close"]


def test_readonly_preserves_state_and_never_passes_command_to_sdk(tmp_path, fake_sdk):
    backend = a3.A3Backend(aimrt_config(tmp_path), base_dir=tmp_path)
    backend.start()
    backend.start()
    io = fake_sdk[0][0]
    result = backend.read_state()
    assert len(fake_sdk[0]) == 1
    assert io.calls == ["start", ("read", .05)]
    assert io.options["read_only"] and not io.options["command_publish_enabled"]
    assert isinstance(result, RobotState) and result is not io.state
    for field in fields(RobotState):
        np.testing.assert_equal(getattr(result, field.name), getattr(io.state, field.name))
    io.state.joint_pos.fill(100)
    assert result.joint_pos[0] == 0
    with pytest.raises(RuntimeError, match="read-only"):
        backend.write_command(command(), result.sequence)
    assert not any(isinstance(item, tuple) and item[0] == "write" for item in io.calls)
    assert backend.watchdog_trip_count == 3
    assert backend.aimrt_stats == {"command_batches": 4}
    backend.close()
    backend.close()
    assert io.calls[-1] == "close" and io.calls.count("close") == 1
    with pytest.raises(RuntimeError, match="not started"):
        backend.read_state()


def test_sdk_timeout_and_rejection_are_not_retried_or_suppressed(fake_sdk):
    with a3.A3Backend({"transport": "sdk_mock", "read_only": False}) as backend:
        io = fake_sdk[0][0]
        assert io.calls == ["memory_writer", "start"]
        assert "aimrt_config_path" not in io.options
        io.read_error = TimeoutError("state missing")
        with pytest.raises(TimeoutError, match="state missing") as error:
            backend.read_state(.2)
        assert error.value is io.read_error
        io.read_error = None
        sample = backend.read_state()
        assert [entry[1] for entry in io.calls if isinstance(entry, tuple) and entry[0] == "inject"] == [1, 2]
        rejection = RuntimeError("SDK command rejected")
        io.write_error = rejection
        with pytest.raises(RuntimeError) as error:
            backend.write_command(command(), sample.sequence)
        assert error.value is rejection
        assert len([entry for entry in io.calls if isinstance(entry, tuple) and entry[0] == "write"]) == 1


def test_start_failure_closes_allocated_sdk(fake_sdk, monkeypatch):
    def fail_start(self):
        raise RuntimeError("start failed")

    monkeypatch.setattr(fake_sdk[1], "start", fail_start)
    backend = a3.A3Backend({"transport": "sdk_mock"})
    with pytest.raises(RuntimeError, match="start failed"):
        backend.start()
    assert fake_sdk[0][0].calls == ["close"]
    backend.close()
    assert fake_sdk[0][0].calls == ["close"]


@pytest.mark.parametrize("overlay", [
    {"transport": "automatic"}, {"transport": "aimrt"},
    {"read_only": "false"}, {"command_publish_enabled": 0},
    {"state_timeout_ms": True}, {"state_timeout_ms": 0}, {"state_timeout_ms": 1.5},
    {"command_timeout_ms": 2**32}, {"startup_timeout_s": float("nan")},
    {"neck_kp": -1}, {"safe_damping_kd": float("inf")},
    {"command_publish_enabled": True}, {"aimrt": {}}, {"unknown": 1},
])
def test_bad_configuration_fails_without_loading_sdk(overlay, fake_sdk):
    with pytest.raises(ValueError):
        a3.A3Backend({"transport": "sdk_mock", **overlay})
    assert fake_sdk[0] == []


def test_aimrt_topics_checked_without_starting_sdk(tmp_path, fake_sdk):
    config = aimrt_config(tmp_path)
    config["aimrt"]["state_topics"]["legs"] = "relative/topic"
    with pytest.raises(ValueError, match="absolute"):
        a3.A3Backend(config, base_dir=tmp_path)
    assert fake_sdk[0] == []


def test_missing_aimrt_file_does_not_allocate_sdk(tmp_path, fake_sdk):
    config = aimrt_config(tmp_path)
    (tmp_path / "aimrt.yaml").unlink()
    backend = a3.A3Backend(config, base_dir=tmp_path)
    with pytest.raises(FileNotFoundError, match="aimrt_config_path"):
        backend.start()
    assert fake_sdk[0] == []


@pytest.fixture
def native_library():
    sdk = pytest.importorskip("agi3sdk")
    library = Path(sdk.__file__).resolve().parents[2] / "build/native/libagi3sdk.so"
    if not library.is_file():
        pytest.skip("optional native SDK build is unavailable")
    return library


def test_native_mock_snapshot_sequence_and_sdk_command_acknowledgment(native_library):
    from agi3sdk import SdkError

    with a3.A3Backend({
        "transport": "sdk_mock", "read_only": False,
        "library_path": native_library, "command_timeout_ms": 1000,
    }) as backend:
        first = backend.read_state()
        second = backend.read_state()
        assert first.sequence == 1 and second.sequence == 2
        assert len(second.joint_pos) == 29
        assert second.timestamp_ns > first.timestamp_ns
        with pytest.raises(SdkError, match="sequence mismatch"):
            backend.write_command(command(), first.sequence)
        backend.write_command(command(), second.sequence)
        assert backend.watchdog_trip_count == 0
        assert backend.aimrt_stats == {}  # No AimRT transport was configured.


def test_native_mock_preserves_stale_snapshot_rejection(native_library):
    from agi3sdk import StateStaleError

    with a3.A3Backend({
        "transport": "sdk_mock", "read_only": False,
        "library_path": native_library, "state_timeout_ms": 5,
        "command_timeout_ms": 1000,
    }) as backend:
        sample = backend.read_state()
        time.sleep(.015)
        with pytest.raises(StateStaleError) as error:
            backend.write_command(command(), sample.sequence)
        assert backend.is_state_stale_error(error.value)
        assert not backend.is_state_stale_error(TimeoutError())


def test_native_readonly_snapshot_without_command_sink(native_library):
    with a3.A3Backend({"transport": "sdk_mock", "library_path": native_library}) as backend:
        assert backend.read_state().sequence == 1
        with pytest.raises(RuntimeError, match="read-only"):
            backend.write_command(command(), 1)
