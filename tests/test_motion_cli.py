"""Installed-resource and actual UDP-to-command coverage for the generic runner."""

import json
import socket

import numpy as np
import pytest
import yaml

from cadence.cli import load_run_config, run_config
from cadence_config import load_config, resolve_resource


def test_state_resource_inheritance_and_declaration_relative_model(tmp_path, monkeypatch):
    state = load_config("pkg://cadence/data/motion/a3_lower.yaml").data
    model = tmp_path / "actor.onnx"
    model.symlink_to(resolve_resource(state["lower"]["model"]))
    state["lower"]["model"] = "actor.onnx"
    (tmp_path / "state.yaml").write_text(yaml.safe_dump(state))
    config = tmp_path / "run.yaml"
    config.write_text(yaml.safe_dump({
        "extends": "pkg://cadence/data/a3_lower_demo.yaml",
        "catalog": {"states": {3: {"config": "state.yaml"}}},
    }))
    monkeypatch.chdir(tmp_path.parent)
    resolved = load_run_config(config)
    expanded = resolved.data["catalog"]["states"][3]["config"]
    assert expanded["lower"]["model"] == str(model.resolve())
    assert resolved.origins["catalog.states.3.config.lower.model"] == str(tmp_path / "state.yaml")
    assert len(expanded["upper"]["default_position"]) == 14


def test_real_actor_runs_without_task_packages(tmp_path, capsys):
    pytest.importorskip("onnxruntime")
    assert run_config("pkg://cadence/data/a3_lower_demo.yaml", duration=.1, output=tmp_path) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["ticks"] == 5 and result["mode"] == "loco" and not result["halted"]
    frozen = yaml.safe_load((tmp_path / "resolved.yaml").read_text())
    assert frozen["catalog"]["states"][3]["config"]["lower"]["model"].endswith("a3_loco_lower.onnx")


def test_udp_latest_command_survives_sender_disconnect(tmp_path, monkeypatch):
    pytest.importorskip("onnxruntime")
    from cadence.backends.mock import MockBackend
    from cadence.motion.targets import JointTargetUdpReceiver

    receivers, commands = [], []
    original_start = JointTargetUdpReceiver.start
    original_write = MockBackend.write_command
    posture = np.asarray(load_config("pkg://cadence/data/motion/a3_lower.yaml").data["upper"]["default_position"])
    latest = posture.copy()
    latest[0] += .15

    def start(receiver):
        result = original_start(receiver)
        receivers.append(receiver)
        return result

    def write(backend, command, sequence):
        commands.append(command.q_des.copy())
        if len(commands) == 1:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sender.settimeout(1)

                def exchange(payload):
                    sender.sendto(json.dumps({"schema": "cadence.joint-target.v1", **payload}).encode(), receivers[0].address)
                    return json.loads(sender.recv(8192))

                status = exchange({"type": "status"})
                target = {"type": "target", "activation": status["activation"], "sequence": 2, "q_des": latest.tolist()}
                assert exchange(target)["accepted"]
                target.update(sequence=1, q_des=posture.tolist())
                assert not exchange(target)["accepted"]
            # The producer is now disconnected for every subsequent PD command.
        original_write(backend, command, sequence)

    monkeypatch.setattr(JointTargetUdpReceiver, "start", start)
    monkeypatch.setattr(MockBackend, "write_command", write)
    config = tmp_path / "stream.yaml"
    config.write_text("extends: pkg://cadence/data/a3_lower_stream_demo.yaml\nruntime:\n  upper_target_udp: {port: 0}\n")
    assert run_config(config, duration=.12) == 0
    np.testing.assert_allclose(commands[0][15:], posture)
    assert len(commands) == 6
    for command in commands[1:]:
        np.testing.assert_allclose(command[15:], latest)
    assert receivers[0].targets.activation is None


def test_udp_binding_rejects_fixed_state_and_closes_backend(tmp_path, monkeypatch):
    pytest.importorskip("onnxruntime")
    from cadence.backends.mock import MockBackend

    closed = []
    original_close = MockBackend.close

    def close(backend):
        closed.append(True)
        original_close(backend)

    monkeypatch.setattr(MockBackend, "close", close)
    config = tmp_path / "invalid.yaml"
    config.write_text("extends: pkg://cadence/data/a3_lower_demo.yaml\nruntime:\n  upper_target_udp: {state: loco, host: 127.0.0.1, port: 0}\n")
    with pytest.raises(ValueError, match="streamed"):
        run_config(config, duration=.1)
    assert closed == [True]


def test_operator_cleanup_failure_still_exits_state_and_closes_backend(tmp_path, monkeypatch):
    from cadence.backends.mock import MockBackend
    from cadence.operator import JoystickCommandReceiver
    from cadence.plugins import BasicState

    cleanup = []
    original_receiver_close = JoystickCommandReceiver.close
    original_backend_close = MockBackend.close

    def close_receiver(receiver):
        original_receiver_close(receiver)
        cleanup.append("operator")
        raise RuntimeError("operator cleanup failed")

    def exit_state(state, frame, events):
        cleanup.append("state")

    def close_backend(backend):
        cleanup.append("backend")
        original_backend_close(backend)

    monkeypatch.setattr(JoystickCommandReceiver, "close", close_receiver)
    monkeypatch.setattr(BasicState, "on_exit", exit_state)
    monkeypatch.setattr(MockBackend, "close", close_backend)
    config = tmp_path / "operator.yaml"
    config.write_text(yaml.safe_dump({
        "extends": ["pkg://cadence/data/demo.yaml", "pkg://cadence/data/operator.yaml"],
        "runtime": {"operator": {"port": 0}},
    }))
    with pytest.raises(RuntimeError, match="operator cleanup failed"):
        run_config(config, duration=.02)
    assert cleanup == ["operator", "state", "backend"]


@pytest.mark.parametrize("field", ["control_hz", "duration_s", "deadline_ms"])
@pytest.mark.parametrize("value", [".nan", ".inf", "-.inf"])
def test_nonfinite_timing_rejected_before_backend_start(tmp_path, monkeypatch, field, value):
    from cadence.backends.mock import MockBackend

    def unexpected_start(backend):
        pytest.fail("invalid timing must fail before opening a backend")

    monkeypatch.setattr(MockBackend, "start", unexpected_start)
    config = tmp_path / "invalid.yaml"
    config.write_text(f"extends: pkg://cadence/data/demo.yaml\nruntime:\n  {field}: {value}\n")
    with pytest.raises(ValueError, match="finite"):
        run_config(config)
