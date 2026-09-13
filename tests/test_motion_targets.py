"""Target retention, activation isolation, and the optional local UDP adapter."""

from dataclasses import FrozenInstanceError
import json
from pathlib import Path
import socket
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pytest

import cadence.motion.targets as targets_module
from cadence.motion.targets import (
    JointTargetFrame,
    JointTargetUdpReceiver,
    LatestJointTarget,
    TARGET_SCHEMA,
)


def target(mailbox, sequence=0, q_des=(0.1, -0.2), activation=None):
    return JointTargetFrame(
        sequence, q_des, mailbox.activation if activation is None else activation
    )


def test_initial_entry_latest_retention_and_reentry_isolation():
    mailbox = LatestJointTarget(2)
    assert mailbox.activation is None and mailbox.latest() is None
    assert not mailbox.publish(JointTargetFrame(0, (0, 0), 1))
    first = mailbox.activate()
    assert mailbox.latest() is None
    latest = target(mailbox, sequence=12)
    assert mailbox.publish(latest)
    # Repeated reads and rejected writes do not consume the most recent frame.
    for _ in range(100):
        assert mailbox.latest() is latest
    assert not mailbox.publish(target(mailbox, sequence=12, q_des=(0.5, 0.5)))
    assert not mailbox.publish(target(mailbox, sequence=11))
    assert mailbox.latest() is latest
    mailbox.deactivate()
    assert mailbox.activation is None and mailbox.latest() is None
    assert not mailbox.publish(JointTargetFrame(13, (0.7, 0.7), first))
    second = mailbox.activate()
    assert second != first and mailbox.latest() is None
    assert not mailbox.publish(JointTargetFrame(99999, (0.7, 0.7), first))
    assert mailbox.publish(target(mailbox, sequence=0))
    assert mailbox.latest().activation == second


def test_activation_can_restart_without_deactivation():
    mailbox = LatestJointTarget(2)
    first = mailbox.activate()
    mailbox.publish(target(mailbox))
    assert mailbox.activate() > first
    assert mailbox.latest() is None
    assert not mailbox.publish(JointTargetFrame(1, (0, 0), first))


def test_activation_seed_isolates_mailboxes_sharing_an_endpoint(monkeypatch):
    seeds = iter([10_000, 20_000])
    monkeypatch.setattr(targets_module.secrets, "randbits", lambda bits: next(seeds))
    old_state = LatestJointTarget(2)
    new_state = LatestJointTarget(2)
    old_activation = old_state.activate()
    new_activation = new_state.activate()
    assert 0 < old_activation != new_activation <= (1 << 53) - 1
    delayed_frame = JointTargetFrame(999, (0.7, -0.7), old_activation)
    assert not new_state.publish(delayed_frame)
    assert new_state.latest() is None
    assert new_state.publish(JointTargetFrame(0, (0.1, -0.1), new_activation))


def test_activation_remains_exactly_representable_in_json_clients(monkeypatch):
    monkeypatch.setattr(targets_module.secrets, "randbits", lambda bits: (1 << bits) - 1)
    mailbox = LatestJointTarget(2)
    assert mailbox.activate() == (1 << 52)
    assert int(float(mailbox.activation)) == mailbox.activation
    # Even a synthetic lifetime exhausting the increment range reseeds safely.
    mailbox._next_activation = (1 << 53) - 1
    activation = mailbox.activate()
    assert 0 < activation <= (1 << 53) - 1
    assert int(float(activation)) == activation


def test_frames_and_limits_do_not_alias_producer_arrays():
    limits = np.array([0.5, 0.5])
    mailbox = LatestJointTarget(2, -limits, limits)
    mailbox.activate()
    positions = np.array([0.1, -0.2])
    frame = target(mailbox, q_des=positions)
    positions[:] = 5.0
    limits[:] = 10.0
    assert frame.q_des == (0.1, -0.2)
    assert mailbox.publish(frame)
    with pytest.raises(FrozenInstanceError):
        frame.sequence = 5
    with pytest.raises(ValueError, match="position_max"):
        mailbox.publish(target(mailbox, sequence=1, q_des=(1, 1)))
    assert mailbox.latest() is frame


@pytest.mark.parametrize("positions", [(0,), (0, 0, 0), (-1.1, 0), (0, 1.1)])
def test_invalid_position_dimensions_or_limits_preserve_latest(positions):
    mailbox = LatestJointTarget(2, (-1, -1), (1, 1))
    mailbox.activate()
    retained = target(mailbox)
    mailbox.publish(retained)
    with pytest.raises(ValueError):
        mailbox.publish(target(mailbox, sequence=1, q_des=positions))
    assert mailbox.latest() is retained


@pytest.mark.parametrize("positions", [(float("nan"), 0), (0, float("inf")), [[0, 0]], 1])
def test_frame_rejects_nonfinite_or_nonvector_positions(positions):
    with pytest.raises(ValueError, match="finite one-dimensional"):
        JointTargetFrame(0, positions, 1)


@pytest.mark.parametrize("sequence, activation", [(-1, 1), (1.2, 1), (True, 1), (0, 0), (0, 1.2)])
def test_frame_rejects_invalid_identity(sequence, activation):
    with pytest.raises(ValueError):
        JointTargetFrame(sequence, (0, 0), activation)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"dimension": 0},
        {"dimension": True},
        {"dimension": 2, "position_min": [0]},
        {"dimension": 2, "position_max": [0, float("inf")]},
        {"dimension": 2, "position_min": [1, 0], "position_max": [0, 1]},
    ],
)
def test_mailbox_rejects_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        LatestJointTarget(**kwargs)


def test_multiple_producers_keep_highest_sequence_and_consistent_vectors():
    mailbox = LatestJointTarget(2)
    activation = mailbox.activate()

    def publish(sequence):
        mailbox.publish(JointTargetFrame(sequence, (sequence, -sequence), activation))
        current = mailbox.latest()
        assert current.q_des == (float(current.sequence), -float(current.sequence))

    with ThreadPoolExecutor(max_workers=4) as executor:
        list(executor.map(publish, range(500)))
    assert mailbox.latest().sequence == 499


def exchange(sock, destination, payload):
    data = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
    sock.sendto(data, destination)
    response, _ = sock.recvfrom(65535)
    return json.loads(response)


def test_udp_protocol_receipt_is_not_execution_and_old_activation_is_rejected():
    mailbox = LatestJointTarget(2, (-1, -1), (1, 1))
    receiver = JointTargetUdpReceiver(mailbox)
    assert receiver.address is None  # Construction does not open a socket.
    with receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as producer:
        producer.settimeout(1)
        status = {"schema": TARGET_SCHEMA, "type": "status"}
        assert exchange(producer, receiver.address, status)["activation"] is None
        first = mailbox.activate()
        assert exchange(producer, receiver.address, status) == {
            "schema": TARGET_SCHEMA,
            "type": "status",
            "activation": first,
            "dimension": 2,
        }
        frame = {
            "schema": TARGET_SCHEMA,
            "type": "target",
            "activation": first,
            "sequence": 0,
            "q_des": [0.25, -0.25],
        }
        receipt = exchange(producer, receiver.address, frame)
        assert receipt["accepted"] and receipt["reason"] == "received"
        retained = mailbox.latest()
        assert retained.q_des == (0.25, -0.25)
        assert not exchange(producer, receiver.address, frame)["accepted"]
        assert mailbox.latest() is retained
        invalid = dict(frame, sequence=1, q_des=[5, 5])
        assert exchange(producer, receiver.address, invalid)["reason"] == "invalid"
        assert mailbox.latest() is retained
        assert exchange(producer, receiver.address, b"not json")["reason"] == "invalid"
        missing_activation = {key: value for key, value in frame.items() if key != "activation"}
        assert exchange(producer, receiver.address, missing_activation)["reason"] == "invalid"
        mailbox.activate()
        assert not exchange(producer, receiver.address, dict(frame, sequence=99))["accepted"]
        assert mailbox.latest() is None
        assert exchange(
            producer, receiver.address, dict(frame, activation=mailbox.activation)
        )["accepted"]
    assert receiver.address is None
    assert mailbox.latest() is not None  # Only the state controls activation.


@pytest.mark.parametrize("host", ["0.0.0.0", "192.168.1.2", "::", "example.com"])
def test_udp_receiver_rejects_non_loopback_bind(host):
    with pytest.raises(ValueError, match="loopback"):
        JointTargetUdpReceiver(LatestJointTarget(2), bind=(host, 50620))


def test_sender_script_queries_activation_and_sends_target():
    mailbox = LatestJointTarget(2)
    mailbox.activate()
    script = Path(__file__).resolve().parents[1] / "scripts" / "send-upper-target.py"
    with JointTargetUdpReceiver(mailbox) as receiver:
        result = subprocess.run(
            [sys.executable, str(script), "--port", str(receiver.address[1]),
             "--sequence", "0", "--q-des", "0.2", "-0.3"],
            text=True, capture_output=True, timeout=5,
        )
        assert result.returncode == 0, result.stderr
        assert json.loads(result.stdout)["accepted"]
        assert mailbox.latest().q_des == (0.2, -0.3)
