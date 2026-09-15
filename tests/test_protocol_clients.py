"""Wire compatibility and real UDP client behavior without task dependencies."""

from contextlib import contextmanager
import json
from pathlib import Path
import socket
import subprocess
import sys
import threading

import pytest

from cadence_protocol.client import (
    JOINT_TARGET_SCHEMA, OPERATOR_SCHEMA, JointTargetClient, OperatorClient,
)
from cadence_protocol.operator import (
    JoystickCommandPacket, JoystickFlags, decode_joystick_command,
    encode_joystick_command, sequence_newer,
)


DESCRIPTION = {
    "schema": OPERATOR_SCHEMA, "type": "description",
    "states": [{"id": 0, "key": "passive"}, {"id": 1, "key": "damping"},
               {"id": 3, "key": "loco"}],
    "aliases": {"loco_lower": "loco"},
    "reset_state_id": 0, "safety_fallback_state_id": 1,
}
STATUS = {
    "schema": OPERATOR_SCHEMA, "type": "status", "mode": "loco",
    "safety_halted": False, "events": ["entered LOCO"],
}


@contextmanager
def server(handler):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(0.05)
    port = sock.getsockname()[1]
    requests, failures = [], []
    stop = threading.Event()

    def receive():
        while not stop.is_set():
            try:
                payload, peer = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                request = json.loads(payload)
                requests.append(request)
                response = handler(request)
                data = response if isinstance(response, bytes) else json.dumps(response).encode()
                sock.sendto(data, peer)
            except Exception as exc:
                failures.append(exc)
                return

    worker = threading.Thread(target=receive, daemon=True)
    worker.start()
    try:
        yield port, requests
    finally:
        stop.set()
        sock.close()
        worker.join(1)
        assert not worker.is_alive()
        assert not failures


def receipt(request, **overrides):
    return {
        "schema": JOINT_TARGET_SCHEMA, "type": "receipt", "accepted": True,
        "activation": request["activation"], "sequence": request["sequence"],
        "reason": "received", **overrides,
    }


def test_operator_description_status_and_binding_validation_use_udp():
    with server(lambda request: DESCRIPTION if request["type"] == "describe" else STATUS) as (port, requests):
        with OperatorClient("127.0.0.1", port) as client:
            assert client.describe() == DESCRIPTION
            assert client.status() == STATUS
            assert client.validate_bindings([(3, "loco_lower"), (0, "passive"), (3, "loco")]) == DESCRIPTION
        assert [request["type"] for request in requests] == ["describe", "status", "describe"]
        assert all(request["schema"] == OPERATOR_SCHEMA for request in requests)


@pytest.mark.parametrize("bindings", [[(3, "damping")], [(2, "loco")], [(3, "unknown")]])
def test_wrong_request_bindings_fail_before_operator_publication(bindings):
    with server(lambda request: DESCRIPTION) as (port, requests):
        with OperatorClient("127.0.0.1", port) as client:
            with pytest.raises(ValueError, match="bound to"):
                client.validate_bindings(bindings)
        assert len(requests) == 1
        assert requests[0]["type"] == "describe"


@pytest.mark.parametrize("response", [
    {**DESCRIPTION, "schema": "other"},
    {**DESCRIPTION, "type": "status"},
    {**DESCRIPTION, "states": [{"id": True, "key": "bad"}]},
    {**DESCRIPTION, "states": DESCRIPTION["states"] * 2},
    {**DESCRIPTION, "aliases": {"invalid": "not_registered"}},
    {**DESCRIPTION, "reset_state_id": 99},
    {"schema": OPERATOR_SCHEMA, "type": "error", "error": "catalog unavailable"},
    b'{"schema":"cadence.operator.v1","schema":"cadence.operator.v1","type":"description"}',
    b'{"schema":"cadence.operator.v1","type":"description","states":NaN}',
])
def test_invalid_operator_descriptions_are_rejected(response):
    with server(lambda request: response) as (port, _):
        with OperatorClient("127.0.0.1", port) as client:
            with pytest.raises(ValueError):
                client.describe()


@pytest.mark.parametrize("changes", [
    {"mode": None}, {"safety_halted": 0}, {"events": "not an event list"},
    {"execution": "published"}, {"execution": None}, {"execution": []},
])
def test_operator_status_validates_runtime_field_types(changes):
    with server(lambda request: {**STATUS, **changes}) as (port, _):
        with OperatorClient("127.0.0.1", port) as client:
            with pytest.raises(ValueError):
                client.status()


def test_joint_target_client_queries_activation_and_sends_explicit_frames():
    def response(request):
        if request["type"] == "status":
            return {"schema": JOINT_TARGET_SCHEMA, "type": "status", "activation": 7, "dimension": 2}
        return receipt(request)

    with server(response) as (port, requests):
        with JointTargetClient("127.0.0.1", port) as client:
            assert client.status()["activation"] == 7
            positions = [0.25, -0.5]
            result = client.send(activation=7, sequence=4, q_des=positions)
            assert result["accepted"] is True
            assert result["sequence"] == 4
        assert requests[1] == {
            "schema": JOINT_TARGET_SCHEMA, "type": "target", "activation": 7,
            "sequence": 4, "q_des": [0.25, -0.5],
        }
        assert len(requests) == 2  # close does not send a reset or fallback pose


def test_joint_target_status_allows_inactive_activation():
    response = {"schema": JOINT_TARGET_SCHEMA, "type": "status", "activation": None, "dimension": 14}
    with server(lambda request: response) as (port, _):
        with JointTargetClient("127.0.0.1", port) as client:
            assert client.status()["activation"] is None


def test_matching_negative_receipt_remains_a_reception_result():
    with server(lambda request: receipt(request, accepted=False, reason="inactive_or_stale")) as (port, _):
        with JointTargetClient("127.0.0.1", port) as client:
            result = client.send(2, 1, [0.2, -0.1])
            assert result["accepted"] is False


def test_receiver_validation_error_does_not_acknowledge_a_target():
    response = {
        "schema": JOINT_TARGET_SCHEMA, "type": "receipt", "accepted": False,
        "reason": "invalid", "error": "q_des is above position_max",
    }
    with server(lambda request: response) as (port, _):
        with JointTargetClient("127.0.0.1", port) as client:
            with pytest.raises(ValueError, match="above position_max"):
                client.send(2, 1, [0.2, -0.1])


@pytest.mark.parametrize("changes", [
    {"schema": "other"}, {"type": "status"}, {"activation": 9},
    {"sequence": 3}, {"accepted": 1}, {"activation": True}, {"sequence": None},
])
def test_unrelated_or_invalid_receipts_do_not_acknowledge_a_target(changes):
    with server(lambda request: receipt(request, **changes)) as (port, _):
        with JointTargetClient("127.0.0.1", port) as client:
            with pytest.raises(ValueError):
                client.send(2, 1, [0.2, -0.1])


@pytest.mark.parametrize("changes", [
    {"activation": 0}, {"activation": True}, {"dimension": 0}, {"dimension": "14"},
])
def test_invalid_joint_status_is_rejected(changes):
    response = {"schema": JOINT_TARGET_SCHEMA, "type": "status", "activation": 1, "dimension": 14, **changes}
    with server(lambda request: response) as (port, _):
        with JointTargetClient("127.0.0.1", port) as client:
            with pytest.raises(ValueError):
                client.status()


@pytest.mark.parametrize("target", [
    (True, 0, [0]), (0, 0, [0]), (1, -1, [0]), (1, True, [0]),
    (1, 0, [float("nan")]), (1, 0, []), (1, 0, [True]), (1, 0, [[0]]),
])
def test_invalid_target_values_fail_without_sending(target):
    with server(receipt) as (port, requests):
        with JointTargetClient("127.0.0.1", port) as client:
            with pytest.raises(ValueError):
                client.send(*target)
        assert not requests


def test_upper_target_legacy_client_accepts_an_explicit_remote_address(monkeypatch):
    calls = []

    class Socket:
        def __init__(self, *args):
            pass

        def settimeout(self, value):
            pass

        def connect(self, endpoint):
            self.endpoint = endpoint
            calls.append(endpoint)

        def getsockname(self):
            return ("192.0.2.2", 15101)

        def getpeername(self):
            return self.endpoint

        def close(self):
            pass

    monkeypatch.setattr(socket, "getaddrinfo", lambda *a, **kw: [(socket.AF_INET, socket.SOCK_DGRAM, 0, "", ("192.0.2.1", 15100))])
    monkeypatch.setattr(socket, "socket", Socket)
    with JointTargetClient("192.0.2.1", 15100):
        assert calls == [("192.0.2.1", 15100)]


@pytest.mark.parametrize("client_type", [OperatorClient, JointTargetClient])
def test_timeout_and_closed_client_are_explicit(client_type):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as silent:
        silent.bind(("127.0.0.1", 0))
        with client_type("127.0.0.1", silent.getsockname()[1], timeout_s=0.02) as client:
            with pytest.raises(socket.timeout):
                client.status()
        with pytest.raises(ValueError, match="closed"):
            client.status()


def test_v3_binary_contract_matches_deployed_golden_bytes():
    packet = JoystickCommandPacket(
        session_id=101, packet_seq=0xFFFFFFFF, source_age_us=300, ttl_ms=200,
        flags=JoystickFlags.CONNECTED | JoystickFlags.REQUEST_VALID,
        request_id=3, signal_bits=5, signal_seq=7,
        axes=(0.25, -0.5, 0.75, -1, 0.125, 1), dpad_x=-1, dpad_y=1,
    )
    expected = bytes.fromhex(
        "504c4e4a0300480065000000ffffffff2c010000c800030003000500000007000000"
        "0000803e000000bf0000403f000080bf0000003e0000803fff0100000000000000004dd046e5"
    )
    assert encode_joystick_command(packet) == expected
    assert decode_joystick_command(expected) == packet
    corrupt = bytearray(expected)
    corrupt[32] ^= 1
    with pytest.raises(ValueError, match="crc"):
        decode_joystick_command(corrupt)
    assert sequence_newer(0, 0xFFFFFFFF)
    assert not sequence_newer(0xFFFFFFFF, 0)


@pytest.mark.parametrize("version,raw", [
    (1, "504c4e4a01002c00010000000700000008000000640007030000003f000080be0000003e0000000072b23193"),
    (2, "504c4e4a02003400010000000700000008000000640007030000003f000080be0000003e00010205090000000000000023da4b91"),
])
def test_deployed_legacy_packets_still_decode(version, raw):
    packet = decode_joystick_command(bytes.fromhex(raw))
    assert packet.wire_version == version
    assert packet.request_id == 3
    assert packet.legacy_velocity_command == (0.5, -0.25, 0.125)
    assert packet.flags == JoystickFlags.CONNECTED | JoystickFlags.REQUEST_VALID
    assert packet.signal_bits == 1
    if version == 2:
        assert packet.signal_seq == 9
        assert packet.legacy_variant_slot == 5


def test_protocol_package_imports_with_only_the_standard_library():
    import cadence_protocol
    import planet_protocol

    source = Path(cadence_protocol.__file__).parent.parent
    shared = Path(planet_protocol.__file__).parent.parent
    script = """
import sys
sys.path[:0] = sys.argv[1:]
import cadence_protocol.operator
import cadence_protocol.client
assert not any(name in sys.modules for name in ('numpy', 'cadence', 'cadence_api', 'agi3sdk', 'planetj'))
"""
    subprocess.run([sys.executable, "-S", "-c", script, str(source), str(shared)], check=True)
