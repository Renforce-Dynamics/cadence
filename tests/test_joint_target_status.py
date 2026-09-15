"""Receipt order, committed commands and activation identity remain distinct."""

import threading
from pathlib import Path
import socket
import json
import runpy
import sys

import pytest
import yaml

from cadence.deployment import run_config
from cadence.motion.targets import JointTargetFrame, JointTargetUdpReceiver, LatestJointTarget
from planet_protocol.client import JointTargetClient


def test_udp_status_reports_owner_receipt_and_actual_command_separately():
    names = ["left_arm", "right_arm"]
    targets = LatestJointTarget(2, [-1, -1], [1, 1], joint_names=names, state_id=5, state_key="upper_stream")
    names[0] = "mutated"
    with JointTargetUdpReceiver(targets) as receiver, JointTargetClient(*receiver.address) as client:
        inactive = client.status()
        assert inactive["state_id"] == 5 and inactive["state_key"] == "upper_stream"
        assert inactive["joint_names"] == ["left_arm", "right_arm"]
        assert all(inactive[key] is None for key in ("activation", "sequence", "q_des"))
        first = targets.activate()
        assert client.send(first, 7, [.8, -.8])["accepted"]
        received = client.status()
        assert received["sequence"] == 7 and received["q_des"] is None
        assert targets.record_applied(first, [.2, -.2])
        committed = client.status()
        assert committed["q_des"] == [.2, -.2]
        assert client.send(first, committed["sequence"] + 1, [.6, -.6])["accepted"]
        newer = client.status()
        assert newer["sequence"] == 8 and newer["q_des"] == [.2, -.2]
        # Returned lists never alias mailbox state.
        direct = targets.status()
        direct["q_des"][0] = .99
        direct["joint_names"][0] = "changed"
        assert targets.status()["q_des"] == [.2, -.2]
        assert targets.status()["joint_names"] == ["left_arm", "right_arm"]
        targets.deactivate()
        inactive = client.status()
        assert all(inactive[key] is None for key in ("activation", "sequence", "q_des"))
        second = targets.activate()
        assert second != first
        assert not targets.record_applied(first, [.9, -.9])
        assert not client.send(first, 99, [.7, -.7])["accepted"]
        assert client.status()["q_des"] is None
        assert client.status()["sequence"] is None


def test_status_never_mixes_activation_receipt_and_command_snapshots():
    targets = LatestJointTarget(1)
    ready = threading.Barrier(2, timeout=2)
    sampled = threading.Barrier(2, timeout=2)
    failures = []

    def writer():
        try:
            for _ in range(300):
                token = targets.activate()
                targets.publish(JointTargetFrame(token, [token], token))
                targets.record_applied(token, [token])
                ready.wait()
                targets.deactivate()
                sampled.wait()
        except BaseException as error:
            failures.append(error)

    worker = threading.Thread(target=writer)
    worker.start()
    try:
        for _ in range(300):
            ready.wait()
            status = targets.status()
            if status["sequence"] is not None:
                assert status["sequence"] == status["activation"]
            if status["q_des"] is not None:
                assert status["q_des"] == [status["activation"]]
            if status["activation"] is None:
                assert status["sequence"] is None and status["q_des"] is None
            sampled.wait()
    finally:
        worker.join(2)
        if worker.is_alive():
            ready.abort()
            sampled.abort()
            worker.join(2)
    assert not failures and not worker.is_alive()


def test_ipv6_loopback_stream_status_and_receipts():
    if not socket.has_ipv6:
        pytest.skip("IPv6 is unavailable")
    targets = LatestJointTarget(1, joint_names=["arm"])
    token = targets.activate()
    receiver = JointTargetUdpReceiver(targets, bind=("::1", 0))
    try:
        receiver.start()
    except OSError as error:
        pytest.skip(f"IPv6 loopback is unavailable: {error}")
    try:
        with JointTargetClient(*receiver.address) as client:
            assert client.status()["joint_names"] == ["arm"]
            assert client.send(token, 9, [.3])["accepted"]
            assert targets.record_applied(token, [.2])
            status = client.status()
            assert status["sequence"] == 9 and status["q_des"] == [.2]
    finally:
        receiver.stop()


@pytest.mark.parametrize("options", [
    {"joint_names": "ab"}, {"joint_names": 1}, {"joint_names": ["arm"]},
    {"joint_names": ["arm", "arm"]}, {"joint_names": ["arm", ""]},
    {"joint_names": ["arm", False]}, {"state_id": True}, {"state_id": -1},
    {"state_id": 65536}, {"state_key": ""}, {"state_key": False},
])
def test_status_identity_metadata_is_validated(options):
    with pytest.raises(ValueError):
        LatestJointTarget(2, **options)


@pytest.mark.parametrize("host", ["192.0.2.5", "2001:db8::5", "0.0.0.0", "::"])
def test_deployment_check_accepts_explicit_remote_bind_without_starting_it(tmp_path, monkeypatch, host):
    pytest.importorskip("onnxruntime")
    root = Path(__file__).resolve().parents[1]
    path = tmp_path / "entry.yaml"
    path.write_text(yaml.safe_dump({
        "extends": str(root / "configs/entry/a3/mock/entry_a3_lower_stream.yaml"),
        "runtime": {"upper_target_udp": {"host": host, "port": 0}},
    }))

    def unexpected_start(*args):
        pytest.fail("--check must not open the configured UDP endpoint")

    monkeypatch.setattr(JointTargetUdpReceiver, "start", unexpected_start)
    assert run_config(path, check=True) == 0


@pytest.mark.parametrize("host,family,address", [
    ("192.0.2.5", socket.AF_INET, "192.0.2.5"),
    ("robot.example", socket.AF_INET6, "2001:db8::5"),
])
def test_manual_sender_accepts_remote_hosts_without_network(monkeypatch, capsys, host, family, address):
    calls = []
    status = {"schema": "cadence.joint-target.v1", "type": "status", "dimension": 1, "activation": None}

    class Socket:
        def __init__(self, selected_family, kind):
            assert selected_family == family and kind == socket.SOCK_DGRAM

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def settimeout(self, timeout):
            pass

        def connect(self, endpoint):
            calls.append(endpoint)

        def send(self, payload):
            assert json.loads(payload)["type"] == "status"

        def recv(self, maximum):
            return json.dumps(status).encode()

    monkeypatch.setattr(socket, "getaddrinfo", lambda *a, **kw: [(family, socket.SOCK_DGRAM, 0, "", (address, 15100))])
    monkeypatch.setattr(socket, "socket", Socket)
    monkeypatch.setattr(sys, "argv", ["send-upper-target.py", "--host", host, "--port", "15100", "--status"])
    script = Path(__file__).resolve().parents[1] / "scripts/send-upper-target.py"
    assert runpy.run_path(str(script))["main"]() == 0
    assert calls == [(address, 15100)]
    assert json.loads(capsys.readouterr().out) == status
