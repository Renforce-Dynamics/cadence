"""Optional operator substates stay generic and compatible with both clients."""

from dataclasses import replace
import json
import socket
import threading
from types import SimpleNamespace

import pytest

from cadence.backends.mock import MockBackend
from cadence.deployment import _publish_status
from cadence.operator import JoystickCommandReceiver
from cadence.plugins import ControlFrame, PluginCatalog
from cadence.runtime import RuntimeConfig, RuntimeInput, RuntimeKernel, RuntimeOutput
from cadence_protocol.client import OperatorClient as LegacyOperatorClient
from planet_protocol.client import OperatorClient


@pytest.fixture
def receiver():
    value = JoystickCommandReceiver("127.0.0.1", 0)
    try:
        yield value
    finally:
        value.close()


def query(receiver, schema="cadence.operator.v1"):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.settimeout(.5)
        client.sendto(json.dumps({"schema": schema, "type": "status"}).encode(), receiver.address)
        assert receiver.poll() is None
        return json.loads(client.recv(65535))


def test_runtime_skill_state_is_published_only_when_status_is_updated(receiver):
    selected = PluginCatalog({
        "reset_state_id": 0, "safety_fallback_state_id": 0,
        "states": {0: {"key": "passive", "factory": "cadence.plugins:BasicState",
                       "config": {"kind": "passive"}}},
    })
    backend = MockBackend(2)
    backend.start()
    try:
        robot = backend.read_state()
        kernel = RuntimeKernel(RuntimeConfig(
            selected, SimpleNamespace(dimension=2), ControlFrame,
            [-1, -1], [1, 1], start_state=0,
        ), robot, 0)
        receiver.update_status({"mode": "PASSIVE", "safety_halted": False, "events": []})
        kernel.prepare(RuntimeInput(.02, robot, None))
        assert "substate" not in query(receiver)
        output = kernel.commit()
        assert isinstance(output, RuntimeOutput)
        assert output.skill_state == "READY"
        assert "substate" not in query(receiver)
        receiver.update_status(output)
        assert query(receiver) == {
            "schema": "cadence.operator.v1", "type": "status", "mode": "PASSIVE",
            "safety_halted": False, "events": [], "substate": "READY",
        }
        receiver.update_status(replace(output, skill_state="HOLD"))
        assert query(receiver)["substate"] == "HOLD"
    finally:
        backend.close()


@pytest.mark.parametrize("shadow", [False, True])
def test_deployment_forwards_substate_and_omits_it_at_startup(receiver, shadow):
    startup = SimpleNamespace(mode="REFERENCE", safety_halted=False, events=())
    _publish_status(receiver, startup, shadow=shadow, entry_gate_ready=False)
    assert "substate" not in query(receiver)
    assert query(receiver)["entry_gate_ready"] is False
    accepted = SimpleNamespace(mode="REFERENCE", safety_halted=False, events=(), skill_state="HOLD")
    _publish_status(receiver, accepted, shadow=shadow, entry_gate_ready=True)
    snapshot = query(receiver)
    assert snapshot["substate"] == "HOLD"
    assert snapshot["entry_gate_ready"] is True
    assert snapshot["execution"] == ("shadow" if shadow else "backend")


@pytest.mark.parametrize("schema", ["planet.operator.v1", "cadence.operator.v1"])
def test_mapping_substate_is_opaque_and_snapshot_preserves_existing_fields(receiver, schema):
    snapshot = {"mode": "CUSTOM", "safety_halted": True, "events": ["accepted"],
                "execution": "shadow", "substate": "WAITING_FOR_ALIGNMENT", "skill_state": "READY"}
    receiver.update_status(snapshot)
    snapshot["substate"] = "HOLD"
    snapshot["events"].append("unpublished")
    assert query(receiver, schema) == {
        "schema": schema, "type": "status", "mode": "CUSTOM", "safety_halted": True,
        "events": ["accepted"], "execution": "shadow", "substate": "WAITING_FOR_ALIGNMENT",
    }


@pytest.mark.parametrize("optional_fields", [{}, {"substate": None}, {"skill_state": None},
                                           {"substate": None, "skill_state": "READY"}])
def test_absent_or_none_substate_restores_the_legacy_status_shape(receiver, optional_fields):
    snapshot = {"mode": "DAMPING", "safety_halted": False, "events": (), "execution": "backend"}
    receiver.update_status({**snapshot, "skill_state": "HOLD"})
    assert query(receiver)["substate"] == "HOLD"
    receiver.update_status({**snapshot, **optional_fields})
    assert query(receiver) == {"schema": "cadence.operator.v1", "type": "status",
                               **snapshot, "events": []}


@pytest.mark.parametrize("field", ["substate", "skill_state"])
@pytest.mark.parametrize("invalid", ["", "  ", False, 1, [], {}])
def test_invalid_substate_does_not_replace_the_accepted_snapshot(receiver, field, invalid):
    snapshot = {"mode": "CUSTOM", "safety_halted": False, "events": (), "substate": "READY"}
    receiver.update_status(snapshot)
    accepted = query(receiver)
    with pytest.raises(ValueError, match="substate must be a nonempty string"):
        receiver.update_status({"mode": "NEXT", "safety_halted": True, "events": (), field: invalid})
    assert query(receiver) == accepted


@pytest.mark.parametrize("invalid", ["true", 0, 1, [], {}])
def test_invalid_entry_gate_does_not_replace_the_accepted_snapshot(receiver, invalid):
    snapshot = {"mode": "FIXEDPOS", "safety_halted": False, "events": (), "entry_gate_ready": False}
    receiver.update_status(snapshot)
    accepted = query(receiver)
    with pytest.raises(ValueError, match="entry_gate_ready must be a boolean"):
        receiver.update_status({**snapshot, "entry_gate_ready": invalid})
    assert query(receiver) == accepted


def test_unknown_entry_gate_is_omitted(receiver):
    receiver.update_status({"mode": "FIXEDPOS", "safety_halted": False, "entry_gate_ready": None})
    assert "entry_gate_ready" not in query(receiver)


@pytest.mark.parametrize("client_type", [OperatorClient, LegacyOperatorClient])
def test_existing_clients_accept_optional_substate_without_protocol_changes(receiver, client_type):
    stopped = threading.Event()
    failures = []

    def poll():
        try:
            while not stopped.wait(.001):
                receiver.poll()
        except BaseException as error:
            failures.append(error)

    worker = threading.Thread(target=poll)
    worker.start()
    try:
        with client_type(*receiver.address, timeout_s=.5) as client:
            snapshot = {"mode": "CUSTOM", "safety_halted": False,
                        "events": ["committed"], "execution": "backend"}
            for optional_fields in ({}, {"substate": "READY", "entry_gate_ready": False},
                                    {"substate": "HOLD", "entry_gate_ready": True}, {}):
                receiver.update_status({**snapshot, **optional_fields})
                assert client.status() == {"schema": client.schema, "type": "status",
                                           **snapshot, **optional_fields}
    finally:
        stopped.set()
        worker.join(1)
        assert not worker.is_alive()
        assert not failures
