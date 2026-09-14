"""Exercise the standalone CLI through real operator and upper-target UDP ports."""

import socket
from pathlib import Path
import threading
import time

import numpy as np
import pytest
import yaml

from cadence.cli import run_config
from cadence.motion.a3 import DEFAULT_JOINT_POSITION
from cadence_protocol.client import JointTargetClient, OperatorClient
from cadence_protocol.operator import (
    JoystickCommandPacket, JoystickFlags, encode_joystick_command,
)


def free_ports():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as operator:
        operator.bind(("127.0.0.1", 0))
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as upper:
            upper.bind(("127.0.0.1", 0))
            return operator.getsockname()[1], upper.getsockname()[1]


def wait_until(predicate, *, timeout=0.45):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = predicate()
        if last:
            return last
        time.sleep(0.005)
    raise AssertionError(f"condition did not become true (last={last!r})")


def test_cli_operator_requests_queries_and_stream_holding(tmp_path, monkeypatch):
    pytest.importorskip("onnxruntime")
    import cadence.backends.mock as mock_module
    import cadence.operator as operator_module
    import cadence.runtime as runtime_module

    original_backend = mock_module.MockBackend
    original_receiver = operator_module.JoystickCommandReceiver
    original_kernel = runtime_module.RuntimeKernel
    ready = threading.Event()
    commands, kernels, failures, observations = [], [], [], {}
    operator_port, upper_port = free_ports()

    class IdealTrackingMock(original_backend):
        def write_command(self, command, state_sequence):
            super().write_command(command, state_sequence)
            # Ideal tracking isolates ingress/state behavior from toy dynamics.
            # The real command validator, gains, filters and deadlines remain active.
            self.q = np.asarray(command.q_des).copy()
            self.dq.fill(0)
            commands.append(np.asarray(command.q_des).copy())

    class ReadyReceiver(original_receiver):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            ready.set()

    class ObservedKernel(original_kernel):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            kernels.append(self)

    monkeypatch.setattr(mock_module, "MockBackend", IdealTrackingMock)
    monkeypatch.setattr(operator_module, "JoystickCommandReceiver", ReadyReceiver)
    monkeypatch.setattr(runtime_module, "RuntimeKernel", ObservedKernel)
    configs = Path(__file__).resolve().parents[1] / "configs"
    (tmp_path / "fixed.yaml").write_text(yaml.safe_dump({
        "extends": str(configs / "states/a3_fixedpos.yaml"), "duration_s": .1,
    }))
    (tmp_path / "registry.yaml").write_text(yaml.safe_dump({
        "extends": str(configs / "state_registries/a3_operator_stream.yaml"),
        "states": {2: {"config": "fixed.yaml"}},
    }))
    config = tmp_path / "operator.yaml"
    config.write_text(yaml.safe_dump({
        "extends": str(configs / "entry/entry_a3_operator_stream.yaml"),
        "runtime": {
            "duration_s": 1.5,
            "state_registry_config": "registry.yaml",
            "operator": {"port": operator_port},
            "upper_target_udp": {"port": upper_port},
        },
    }))

    def exercise():
        try:
            assert ready.wait(1), "operator receiver did not start"
            with OperatorClient("127.0.0.1", operator_port, timeout_s=0.15) as operator, \
                    JointTargetClient("127.0.0.1", upper_port, timeout_s=0.15) as upper, \
                    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                sequence = 0

                def request(state=None, signals=0, axes=(0.0,) * 6):
                    nonlocal sequence
                    flags = JoystickFlags.CONNECTED
                    if state is not None:
                        flags |= JoystickFlags.REQUEST_VALID
                    packet = JoystickCommandPacket(
                        session_id=127, packet_seq=sequence, source_age_us=0,
                        ttl_ms=1500, flags=flags, request_id=state or 0,
                        signal_bits=signals, signal_seq=sequence, axes=axes,
                    )
                    sender.sendto(encode_joystick_command(packet), ("127.0.0.1", operator_port))
                    sequence += 1

                def status_when(predicate):
                    def check():
                        status = operator.status()
                        return status if predicate(status) else None
                    return wait_until(check)

                description = operator.validate_bindings([(0, "passive"), (1, "damping"), (2, "fixedpos"), (3, "loco")])
                assert {state["id"] for state in description["states"]} == {0, 1, 2, 3}
                with pytest.raises(ValueError, match="bound to"):
                    operator.validate_bindings([(3, "damping")])
                assert upper.status()["activation"] is None

                request(3)
                rejected = status_when(lambda status: any("alignment" in event for event in status["events"]))
                assert rejected["mode"].lower() == "damping"
                assert not rejected["safety_halted"]
                request(2)
                status_when(lambda status: status["mode"].lower() == "fixedpos")
                wait_until(lambda: kernels[0]._entry_gate_ready)
                request(3)
                status_when(lambda status: status["mode"].lower() == "loco")
                state = kernels[0].plugin("loco")
                wait_until(lambda: state._entered_command_applied)
                np.testing.assert_allclose(state.upper_position, DEFAULT_JOINT_POSITION[15:])
                activation = upper.status()["activation"]
                target = np.asarray(DEFAULT_JOINT_POSITION[15:]).copy()
                target[[0, 3, 7, 10]] += (0.04, 0.08, -0.04, 0.08)
                assert upper.send(activation, 10, target)["accepted"]
                wait_until(lambda: state.committed_upper_sequence == 10)
                wait_until(lambda: kernels[0]._transition_started_s is None)
                hold_start = len(commands)
                request(3, axes=(0.0, 0.8, 0.2, 0.0, 0.0, 0.0))
                wait_until(lambda: len(commands) >= hold_start + 6)
                held_commands = np.asarray(commands[hold_start:])
                np.testing.assert_allclose(held_commands[:, 15:], np.broadcast_to(target, held_commands[:, 15:].shape), atol=1e-12)
                assert state.committed_upper_sequence == 10
                assert state._last_diagnostics.observation.shape == (397,)
                assert np.max(np.abs(held_commands[:, :15] - np.asarray(DEFAULT_JOINT_POSITION[:15]))) > 0.01
                assert not upper.send(activation, 9, np.zeros(14))["accepted"]

                request(8)
                unknown = status_when(lambda status: any("rejected" in event.lower() and "8" in event for event in status["events"]))
                assert unknown["mode"].lower() == "loco"
                request(signals=1)
                halted = status_when(lambda status: status["safety_halted"])
                assert halted["mode"].lower() == "loco"
                assert upper.status()["activation"] == activation
                request(signals=2)
                reset = status_when(lambda status: not status["safety_halted"] and status["mode"].lower() == "passive")
                request()
                assert upper.status()["activation"] is None
                assert not upper.send(activation, 11, target)["accepted"]
                observations.update(description=description, unknown=unknown, halted=halted,
                                    reset=reset, held_ticks=len(held_commands))
        except BaseException as exc:
            failures.append(exc)

    worker = threading.Thread(target=exercise, daemon=True)
    worker.start()
    result = run_config(config)
    worker.join(1)
    assert not worker.is_alive()
    if failures:
        raise failures[0]
    assert result == 0
    assert observations["held_ticks"] >= 6
    assert not kernels[0].safety.halted
    assert len(commands) == 75
