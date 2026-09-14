"""Small synchronous UDP clients for Cadence operator and joint-target ports.

Clients return decoded response dictionaries. Network failures raise OSError
(including socket.timeout); invalid inputs or responses raise ValueError.
A joint-target receipt acknowledges reception only, never robot execution.
"""

from __future__ import annotations

import ipaddress
from collections.abc import Mapping
import json
import math
from numbers import Integral, Real
import socket

from .localization import LocalizationClient


OPERATOR_SCHEMA = "cadence.operator.v1"
JOINT_TARGET_SCHEMA = "cadence.joint-target.v1"


def _integer(value, name, minimum=0, maximum=None):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f"{name} must be an integer")
    result = int(value)
    if result < minimum or (maximum is not None and result > maximum):
        raise ValueError(f"{name} is out of range")
    return result


def _name(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")
    return value


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate response field: {key}")
        result[key] = value
    return result


def _nonfinite(value):
    raise ValueError(f"nonfinite JSON value: {value}")


def _checked_bindings(bindings, *, allow_unnamed=False):
    checked = []
    try:
        for request_id, key in bindings:
            checked.append((
                _integer(request_id, "request_id", 0, 65535),
                None if allow_unnamed and key is None else _name(key, "state key"),
            ))
    except TypeError as exc:
        raise ValueError("bindings must contain (request_id, state_key) pairs") from exc
    return checked


def validate_operator_bindings(bindings, states_by_id, aliases, *, allow_unnamed=False):
    """Match input bindings against a loaded catalog without network or runtime imports.

    ``states_by_id`` maps request IDs to canonical state names. Named bindings
    must identify that exact state or an advertised alias. ``allow_unnamed`` is
    an explicit compatibility mode for legacy profiles: a None state name only
    checks that its ID exists, without guessing meaning from a display label.
    Unknown IDs are always rejected. Remote checks use the strict default.
    """
    if not isinstance(states_by_id, Mapping) or not states_by_id:
        raise ValueError("states_by_id must be a nonempty catalog mapping")
    states = {
        _integer(state_id, "state.id", 0, 65535): _name(key, "state.key")
        for state_id, key in states_by_id.items()
    }
    if len(set(states.values())) != len(states):
        raise ValueError("catalog state keys must be unique")
    if not isinstance(aliases, Mapping):
        raise ValueError("aliases must be a catalog mapping")
    for alias, key in aliases.items():
        _name(alias, "state alias")
        _name(key, "state alias target")
        if alias in states.values() or key not in states.values():
            raise ValueError("catalog contains an invalid state alias")
    for request_id, key in _checked_bindings(bindings, allow_unnamed=allow_unnamed):
        if key is None:
            if request_id not in states:
                raise ValueError(f"legacy request {request_id} is not registered in the runtime catalog")
            continue
        expected = aliases.get(key, key)
        if states.get(request_id) != expected:
            raise ValueError(
                f"request {request_id} is bound to {states.get(request_id)!r}, "
                f"not configured state {key!r}"
            )


class _UdpClient:
    """One request at a time, with responses restricted to the connected peer."""

    def __init__(self, host, port, timeout_s=1.0, *, loopback=False):
        _name(host, "host")
        port = _integer(port, "port", 1, 65535)
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, Real):
            raise ValueError("timeout_s must be positive and finite")
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive and finite")
        if loopback:
            try:
                address = ipaddress.ip_address(host)
            except ValueError as exc:
                raise ValueError("joint-target host must be a loopback IP address") from exc
            if not address.is_loopback:
                raise ValueError("joint-target host must be a loopback IP address")
            family = socket.AF_INET6 if address.version == 6 else socket.AF_INET
            endpoint = (str(address), port)
        else:
            candidates = socket.getaddrinfo(host, port, type=socket.SOCK_DGRAM)
            family, _, _, _, endpoint = candidates[0]
        self._socket = socket.socket(family, socket.SOCK_DGRAM)
        try:
            self._socket.settimeout(timeout_s)
            self._socket.connect(endpoint)
        except BaseException:
            self._socket.close()
            raise
        self._closed = False

    def close(self):
        self._socket.close()
        self._closed = True

    def __enter__(self):
        if self._closed:
            raise ValueError("client is closed")
        return self

    def __exit__(self, *args):
        self.close()

    def _exchange(self, request, response_type):
        if self._closed:
            raise ValueError("client is closed")
        payload = json.dumps(request, allow_nan=False).encode("utf-8")
        if len(payload) > 65507:
            raise ValueError("request exceeds maximum UDP payload size")
        self._socket.send(payload)
        try:
            response = json.loads(
                self._socket.recv(65535), object_pairs_hook=_object,
                parse_constant=_nonfinite,
            )
        except (UnicodeError, json.JSONDecodeError, RecursionError) as exc:
            raise ValueError("invalid JSON response") from exc
        if not isinstance(response, dict) or response.get("schema") != request["schema"]:
            raise ValueError("response schema does not match request")
        if response.get("type") == "error":
            raise ValueError(f"Cadence receiver: {response.get('error', 'request rejected')}")
        if response.get("type") != response_type:
            raise ValueError(f"expected response type {response_type!r}")
        return response


class OperatorClient(_UdpClient):
    """Discover operator state bindings without importing the control runtime.

    Queries use the same UDP endpoint as binary PLNJ operator commands. They
    inspect configuration/status and do not request a state transition.
    """

    def describe(self):
        response = self._exchange(
            {"schema": OPERATOR_SCHEMA, "type": "describe"}, "description"
        )
        states = response.get("states")
        if not isinstance(states, list) or not states:
            raise ValueError("description.states must be a nonempty list")
        ids, keys = set(), set()
        for state in states:
            if not isinstance(state, dict):
                raise ValueError("description.states entries must be mappings")
            state_id = _integer(state.get("id"), "state.id", 0, 65535)
            key = _name(state.get("key"), "state.key")
            if state_id in ids or key in keys:
                raise ValueError("description states contain duplicate IDs or keys")
            ids.add(state_id)
            keys.add(key)
        aliases = response.get("aliases")
        if not isinstance(aliases, dict):
            raise ValueError("description.aliases must be a mapping")
        for alias, target in aliases.items():
            _name(alias, "state alias")
            _name(target, "state alias target")
            if alias in keys or target not in keys:
                raise ValueError("description contains an invalid state alias")
        for name in ("reset_state_id", "safety_fallback_state_id"):
            if _integer(response.get(name), name, 0, 65535) not in ids:
                raise ValueError(f"{name} does not identify a described state")
        return response

    def status(self):
        response = self._exchange(
            {"schema": OPERATOR_SCHEMA, "type": "status"}, "status"
        )
        _name(response.get("mode"), "status.mode")
        if not isinstance(response.get("safety_halted"), bool):
            raise ValueError("status.safety_halted must be a boolean")
        events = response.get("events")
        if not isinstance(events, list) or any(not isinstance(event, str) for event in events):
            raise ValueError("status.events must be a list of strings")
        if "execution" in response and response["execution"] not in ("backend", "shadow"):
            raise ValueError("status.execution must be backend or shadow")
        return response

    def validate_bindings(self, bindings):
        """Validate (request_id, state_key) pairs; return the description.

        Keys may be canonical state keys or advertised aliases. Repeated
        bindings are valid when multiple buttons request the same state.
        """
        checked = _checked_bindings(bindings)
        description = self.describe()
        states = {state["id"]: state["key"] for state in description["states"]}
        validate_operator_bindings(checked, states, description["aliases"])
        return description


class JointTargetClient(_UdpClient):
    """Send explicit joint angles to a local, activation-scoped target mailbox.

    The client never chooses a new activation, increments sequences, sends a
    fallback, or retries a frame automatically. Those decisions belong to the
    producer. The receiver holds the latest accepted target when input stops.
    """

    def __init__(self, host, port, timeout_s=1.0):
        super().__init__(host, port, timeout_s, loopback=True)

    def status(self):
        response = self._exchange(
            {"schema": JOINT_TARGET_SCHEMA, "type": "status"}, "status"
        )
        if "activation" not in response:
            raise ValueError("status.activation is missing")
        if response["activation"] is not None:
            _integer(response["activation"], "activation", 1)
        _integer(response.get("dimension"), "dimension", 1)
        return response

    def send(self, activation, sequence, q_des):
        activation = _integer(activation, "activation", 1)
        sequence = _integer(sequence, "sequence", 0)
        try:
            values = tuple(q_des)
        except TypeError as exc:
            raise ValueError("q_des must be a nonempty sequence of finite angles") from exc
        if not values or any(isinstance(value, bool) or not isinstance(value, Real) for value in values):
            raise ValueError("q_des must be a nonempty sequence of finite angles")
        positions = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in positions):
            raise ValueError("q_des must be a nonempty sequence of finite angles")
        response = self._exchange({
            "schema": JOINT_TARGET_SCHEMA, "type": "target",
            "activation": activation, "sequence": sequence, "q_des": positions,
        }, "receipt")
        if not isinstance(response.get("accepted"), bool):
            raise ValueError("receipt.accepted must be a boolean")
        if response["accepted"] is False and "error" in response:
            raise ValueError(f"joint target rejected: {response['error']}")
        if (
            _integer(response.get("activation"), "receipt.activation", 1) != activation
            or _integer(response.get("sequence"), "receipt.sequence", 0) != sequence
        ):
            raise ValueError("receipt activation or sequence does not match submitted target")
        return response
