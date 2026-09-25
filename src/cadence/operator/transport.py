"""Nonblocking operator packets and read-only state discovery on one UDP port."""

from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
import json
import socket
import time

from planet_protocol.operator import decode_joystick_command, sequence_newer
from planet_protocol.client import OPERATOR_SCHEMA as PLANET_OPERATOR_SCHEMA, _object, _nonfinite

from .input import ReceivedJoystickCommand


OPERATOR_SCHEMA = "cadence.operator.v1"
OPERATOR_SCHEMAS = (PLANET_OPERATOR_SCHEMA, OPERATOR_SCHEMA)
MAX_DATAGRAMS_PER_POLL = 64


class JoystickCommandReceiver:
    """Latest-only PLNJ ingress with description and committed-status queries.

    The frontend calls poll() every control period. Query handling only reads
    snapshots installed by set_catalog()/update_status(); no query invokes a
    state, model or runtime transition. Invalid and out-of-order binary packets
    never replace the last accepted packet. Each poll processes at most 64
    datagrams, including queries; backlog remains for a subsequent poll. This
    bounds work by packet count, not by a hard real-time execution guarantee.
    """

    def __init__(self, host: str, port: int) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.bind((str(host), int(port)))
            self._socket.setblocking(False)
        except BaseException:
            self._socket.close()
            raise
        self._latest: ReceivedJoystickCommand | None = None
        self._session_id: int | None = None
        self._packet_seq: int | None = None
        self._description = None
        self._status = None
        self.rejected_decode = 0
        self.rejected_sequence = 0

    @property
    def address(self) -> tuple[str, int]:
        host, port = self._socket.getsockname()[:2]
        return str(host), int(port)

    def close(self) -> None:
        self._socket.close()

    def set_catalog(self, catalog) -> None:
        """Publish the selected startup catalog without retaining mutable aliases."""
        self._description = {
            "schema": OPERATOR_SCHEMA,
            "type": "description",
            "states": [{"id": int(state_id), "key": str(key)} for state_id, key in sorted(catalog.ids.items())],
            "aliases": deepcopy(dict(catalog.aliases)),
            "reset_state_id": int(catalog.reset_state_id),
            "safety_fallback_state_id": int(catalog.safety_fallback_state_id),
        }

    def update_status(self, output) -> None:
        """Publish an accepted runtime result (or an explicit startup snapshot).

        An optional ``substate`` takes precedence over RuntimeOutput's
        ``skill_state``. Missing or None values preserve the legacy response
        shape; state names are opaque to this transport.
        """
        def field(name, default=None):
            return output.get(name, default) if isinstance(output, Mapping) else getattr(output, name, default)

        mode = field("mode", field("operator_state"))
        halted = field("safety_halted")
        events = field("events", ())
        if not isinstance(mode, str) or not isinstance(halted, bool):
            raise ValueError("operator status requires a string mode and boolean safety_halted")
        if not isinstance(events, (tuple, list)) or not all(isinstance(event, str) for event in events):
            raise ValueError("operator status events must be strings")
        execution = field("execution")
        if execution is not None and execution not in ("backend", "shadow"):
            raise ValueError("operator execution must be backend or shadow")
        substate = field("substate", field("skill_state"))
        if substate is not None and (not isinstance(substate, str) or not substate.strip()):
            raise ValueError("operator substate must be a nonempty string or None")
        entry_gate_ready = field("entry_gate_ready")
        if entry_gate_ready is not None and type(entry_gate_ready) is not bool:
            raise ValueError("operator entry_gate_ready must be a boolean")
        safety_reason = field("safety_reason")
        if safety_reason is not None and not isinstance(safety_reason, str):
            raise ValueError("operator safety_reason must be a string or None")
        self._status = {"schema": OPERATOR_SCHEMA, "type": "status", "mode": mode,
                        "safety_halted": halted, "events": list(events)}
        if safety_reason:
            self._status["safety_reason"] = safety_reason
        if execution is not None:
            self._status["execution"] = execution
        if substate is not None:
            self._status["substate"] = substate
        if entry_gate_ready is not None:
            self._status["entry_gate_ready"] = entry_gate_ready

    def _answer_query(self, raw, peer):
        schema = OPERATOR_SCHEMA
        try:
            request = json.loads(raw, object_pairs_hook=_object, parse_constant=_nonfinite)
            if isinstance(request, dict) and request.get("schema") in OPERATOR_SCHEMAS:
                schema = request["schema"]
            if not isinstance(request, dict) or set(request) != {"schema", "type"} or request["schema"] not in OPERATOR_SCHEMAS:
                raise ValueError(f"query requires schema in {OPERATOR_SCHEMAS} and type")
            if request["type"] == "describe":
                response = self._description
            elif request["type"] == "status":
                response = self._status
            else:
                raise ValueError("query type must be describe or status")
            if response is None:
                raise ValueError("operator runtime is not ready")
            response = {**response, "schema": schema}
        except (ValueError, TypeError, RecursionError) as error:
            response = {"schema": schema, "type": "error", "error": str(error)}
        try:
            self._socket.sendto(json.dumps(response, allow_nan=False).encode(), peer)
        except (BlockingIOError, OSError):
            # Query consumers may close their socket before receiving the reply.
            pass

    def poll(self) -> ReceivedJoystickCommand | None:
        for _ in range(MAX_DATAGRAMS_PER_POLL):
            try:
                raw, peer = self._socket.recvfrom(65535)
            except BlockingIOError:
                break
            if raw.lstrip().startswith(b"{"):
                self._answer_query(raw, peer)
                continue
            try:
                packet = decode_joystick_command(raw)
            except ValueError:
                self.rejected_decode += 1
                continue
            if self._session_id == packet.session_id:
                if self._packet_seq is not None and not sequence_newer(packet.packet_seq, self._packet_seq):
                    self.rejected_sequence += 1
                    continue
            else:
                self._session_id = packet.session_id
            self._packet_seq = packet.packet_seq
            self._latest = ReceivedJoystickCommand(packet, time.monotonic())
        return self._latest
