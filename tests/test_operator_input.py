"""Generic operator mapping, legacy wire ingress, and read-only state queries."""

from dataclasses import replace
import json
import socket
import struct
import threading
from types import SimpleNamespace
import zlib

import pytest

from cadence.operator import (
    JoystickCommandReceiver, OperatorInputAdapter, OperatorInputMapping,
    ReceivedJoystickCommand,
)
from cadence.plugins import PluginCatalog
from cadence_protocol.operator import (
    JoystickCommandPacket, JoystickFlags, encode_joystick_command,
)


def mapping():
    return OperatorInputMapping.from_mapping({
        "velocity_axes": ["left_y", "left_x", "right_x"],
        "velocity_scales": [0.4, 0.2, 0.5],
        "emergency_signal_id": 0, "reset_signal_id": 1,
    })


def packet(**changes):
    return replace(JoystickCommandPacket(
        session_id=12, packet_seq=1, source_age_us=10_000, ttl_ms=100,
        flags=JoystickFlags.CONNECTED | JoystickFlags.REQUEST_VALID,
        request_id=42, signal_bits=3, axes=(0.5, -0.5, 0.25, 0, 0, 0),
    ), **changes)


def catalog():
    return PluginCatalog({
        "reset_state_id": 17, "safety_fallback_state_id": 42,
        "aliases": {"stop": "damping"},
        "states": {
            17: {"key": "passive", "factory": "cadence.plugins:BasicState", "config": {"kind": "passive"}},
            42: {"key": "damping", "factory": "cadence.plugins:BasicState", "config": {"kind": "damping"}},
        },
    })


def test_snapshot_maps_runtime_fields_and_stale_packet_clears_every_input():
    adapter = OperatorInputAdapter(mapping())
    received = ReceivedJoystickCommand(packet(), 10)
    value = adapter.map(received, 10.02)
    assert value.requested_state == 42
    assert value.velocity_command == pytest.approx((-0.2, 0.1, 0.125))
    assert value.emergency_halt and value.reset_safety and value.operator_link_usable
    assert value.operator_input is received.packet
    with pytest.raises(AttributeError):
        value.requested_state = 17
    for stale in (adapter.map(received, 10.091), adapter.map(None, 10.02)):
        assert not stale.operator_link_usable
        assert stale.velocity_command == (0, 0, 0)
        assert stale.requested_state is None and stale.operator_input is None
        assert not stale.emergency_halt and not stale.reset_safety


@pytest.mark.parametrize("mode,repeated", [("level", True), ("rising", False)])
def test_signal_modes_keep_simulation_edges_and_onboard_levels(mode, repeated):
    adapter = OperatorInputAdapter(mapping(), signal_mode=mode)
    received = ReceivedJoystickCommand(packet(), 10)
    assert adapter.map(received, 10).emergency_halt
    assert adapter.map(received, 10.02).emergency_halt is repeated
    assert adapter.map(received, 10.04).reset_safety is repeated
    adapter.map(None, 10.05)
    assert adapter.map(received, 10.06).emergency_halt
    assert not adapter.map(ReceivedJoystickCommand(packet(flags=JoystickFlags(0)), 10), 10).operator_link_usable


@pytest.mark.parametrize("version", [1, 2])
def test_legacy_datagrams_keep_physical_velocity_and_safety_flags(version):
    if version == 1:
        body = struct.pack("<4sHHIIIHBB3f4s", b"PLNJ", 1, 44, 1, 2, 0, 100,
                           0b1111, 42, .4, -.3, .2, b"\0" * 4)
    else:
        body = struct.pack("<4sHHIIIHBB3fBBBBI4s", b"PLNJ", 2, 52, 1, 2, 0, 100,
                           0b1111, 42, .4, -.3, .2, 0, 0, 0, 0, 0, b"\0" * 4)
    raw = body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)
    receiver = JoystickCommandReceiver("127.0.0.1", 0)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.sendto(raw, receiver.address)
            received = receiver.poll()
        assert received.packet.wire_version == version
        result = OperatorInputAdapter(mapping()).map(received, received.received_monotonic_s)
        assert result.velocity_command == pytest.approx((.4, -.3, .2))
        assert result.emergency_halt and result.reset_safety
        assert result.requested_state == 42
    finally:
        receiver.close()


def test_receiver_orders_wrapped_sequences_and_ignores_corrupt_datagrams():
    receiver = JoystickCommandReceiver("127.0.0.1", 0)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for value in (0xFFFFFFFE, 1, 1, 0):
                sender.sendto(encode_joystick_command(packet(packet_seq=value)), receiver.address)
            sender.sendto(b"bad binary packet", receiver.address)
            latest = receiver.poll()
            assert latest.packet.packet_seq == 1
            assert receiver.rejected_sequence == 2 and receiver.rejected_decode == 1
            sender.sendto(encode_joystick_command(packet(session_id=2, packet_seq=0)), receiver.address)
            assert receiver.poll().packet.session_id == 2
    finally:
        receiver.close()


def test_description_and_committed_status_are_copied_and_queries_do_not_change_input():
    receiver = JoystickCommandReceiver("127.0.0.1", 0)
    selected = catalog()
    receiver.set_catalog(selected)
    events = ["startup"]
    receiver.update_status(SimpleNamespace(operator_state="DAMPING", safety_halted=False, events=events))
    selected.aliases["stop"] = "passive"
    events.append("uncommitted")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
            client.settimeout(.5)

            def query(kind):
                client.sendto(json.dumps({"schema": "cadence.operator.v1", "type": kind}).encode(), receiver.address)
                assert receiver.poll() is None
                return json.loads(client.recv(65535))

            description = query("describe")
            assert description["states"] == [{"id": 17, "key": "passive"}, {"id": 42, "key": "damping"}]
            assert description["aliases"] == {"stop": "damping"}
            assert description["reset_state_id"] == 17 and description["safety_fallback_state_id"] == 42
            assert query("status") == {"schema": "cadence.operator.v1", "type": "status",
                                       "mode": "DAMPING", "safety_halted": False, "events": ["startup"]}
            assert query("transition")["type"] == "error"
    finally:
        receiver.close()


def test_generic_operator_imports_have_no_task_or_sender_dependency():
    import ast
    from pathlib import Path
    import cadence.operator

    for path in Path(cadence.operator.__file__).parent.glob("*.py"):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.level == 0:
                assert not node.module.startswith(("planetj", "planet_pingpong", "cadence_rally"))
            if isinstance(node, ast.Import):
                assert all(not alias.name.startswith(("planetj", "planet_pingpong", "cadence_rally")) for alias in node.names)


def test_client_validates_aliases_and_status_changes_only_when_published_after_commit():
    from cadence.backends.mock import MockBackend
    from cadence.plugins import ControlFrame
    from cadence.runtime import RuntimeConfig, RuntimeInput, RuntimeKernel
    from cadence_protocol.client import OperatorClient

    selected = catalog()
    backend = MockBackend(2)
    backend.start()
    robot = backend.read_state()
    kernel = RuntimeKernel(RuntimeConfig(
        selected, SimpleNamespace(dimension=2), ControlFrame,
        [-1, -1], [1, 1], start_state=17,
    ), robot, 0)
    receiver = JoystickCommandReceiver("127.0.0.1", 0)
    receiver.set_catalog(selected)
    receiver.update_status({"mode": "PASSIVE", "safety_halted": False, "events": []})
    stop = threading.Event()

    def serve_queries():
        while not stop.is_set():
            receiver.poll()
            stop.wait(.001)

    thread = threading.Thread(target=serve_queries)
    thread.start()
    try:
        with OperatorClient(*receiver.address, timeout_s=.5) as client:
            assert client.validate_bindings([(42, "stop")])["reset_state_id"] == 17
            with pytest.raises(ValueError, match="not configured state"):
                client.validate_bindings([(42, "passive")])
            kernel.prepare(RuntimeInput(.02, robot, None, requested_state=42))
            assert client.status()["mode"] == "PASSIVE"
            kernel.reject()
            assert client.status()["mode"] == "PASSIVE"
            kernel.prepare(RuntimeInput(.04, robot, None))
            backend.write_command(kernel.guard_pending().command, robot.sequence)
            receiver.update_status(kernel.commit())
            assert client.status()["mode"] == "DAMPING"
    finally:
        stop.set()
        thread.join(timeout=1)
        receiver.close()
        backend.close()


def test_failed_bind_closes_socket_even_when_exception_traceback_is_retained(monkeypatch):
    import cadence.operator.transport as transport

    real_socket = socket.socket
    created = []

    def tracked_socket(*args, **kwargs):
        sock = real_socket(*args, **kwargs)
        created.append(sock)
        return sock

    with real_socket(socket.AF_INET, socket.SOCK_DGRAM) as occupied:
        occupied.bind(("127.0.0.1", 0))
        monkeypatch.setattr(transport.socket, "socket", tracked_socket)
        with pytest.raises(OSError) as failure:
            JoystickCommandReceiver("127.0.0.1", occupied.getsockname()[1])
        assert failure.traceback is not None
        assert len(created) == 1
        assert created[0].fileno() == -1


def test_startup_interruption_closes_the_partially_configured_socket(monkeypatch):
    import cadence.operator.transport as transport

    created = []

    class InterruptedSocket(socket.socket):
        def setblocking(self, value):
            created.append(self)
            raise KeyboardInterrupt("startup interrupted after bind")

    monkeypatch.setattr(transport.socket, "socket", InterruptedSocket)
    with pytest.raises(KeyboardInterrupt):
        JoystickCommandReceiver("127.0.0.1", 0)
    assert len(created) == 1
    assert created[0].fileno() == -1


def test_packet_backlog_is_deferred_and_each_batch_keeps_its_latest_command():
    receiver = JoystickCommandReceiver("127.0.0.1", 0)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for sequence in range(70):
                sender.sendto(encode_joystick_command(packet(packet_seq=sequence)), receiver.address)
            assert receiver.poll().packet.packet_seq == 63
            assert receiver.poll().packet.packet_seq == 69
            assert receiver.poll().packet.packet_seq == 69
    finally:
        receiver.close()


def test_queries_share_the_poll_budget_without_changing_latest_input():
    receiver = JoystickCommandReceiver("127.0.0.1", 0)
    receiver.set_catalog(catalog())
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.setblocking(False)
            sender.sendto(encode_joystick_command(packet(packet_seq=1)), receiver.address)
            assert receiver.poll().packet.packet_seq == 1
            query = json.dumps({"schema": "cadence.operator.v1", "type": "describe"}).encode()
            for _ in range(64):
                sender.sendto(query, receiver.address)
            sender.sendto(encode_joystick_command(packet(packet_seq=2)), receiver.address)
            assert receiver.poll().packet.packet_seq == 1
            replies = []
            while True:
                try:
                    replies.append(json.loads(sender.recv(65535)))
                except BlockingIOError:
                    break
            assert len(replies) == 64
            assert all(reply["type"] == "description" for reply in replies)
            assert receiver.poll().packet.packet_seq == 2
    finally:
        receiver.close()
