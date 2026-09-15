"""Activation-scoped joint targets and an optional explicitly bound UDP adapter.

Targets are positions in radians, in the joint order chosen by the consuming
state. Reception never implies that a command has been applied to the robot.
"""

from __future__ import annotations

from dataclasses import dataclass
import ipaddress
import json
from numbers import Integral
import secrets
import socket
import threading
from typing import Sequence

import numpy as np
from planet_protocol.client import JOINT_TARGET_SCHEMA, _object, _nonfinite


_MAX_SAFE_JSON_INTEGER = (1 << 53) - 1


def _integer(value: object, name: str, minimum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f"{name} must be an integer")
    value = int(value)
    if value < minimum:
        raise ValueError(f"{name} must be at least {minimum}")
    return value


def _positions(value: Sequence[float], name: str) -> tuple[float, ...]:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite one-dimensional vector") from exc
    if array.ndim != 1 or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite one-dimensional vector")
    return tuple(float(item) for item in array)


@dataclass(frozen=True, slots=True)
class JointTargetFrame:
    """An immutable position target for one activation of a motion state.

    Sequence numbers start at zero (or any greater integer) and increase within
    an activation. Producers must obtain a fresh activation after state entry.
    The input vector is copied so later producer mutations cannot alter a frame.
    """

    sequence: int
    q_des: tuple[float, ...]
    activation: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "sequence", _integer(self.sequence, "sequence", 0))
        object.__setattr__(self, "activation", _integer(self.activation, "activation", 1))
        object.__setattr__(self, "q_des", _positions(self.q_des, "q_des"))


@dataclass(frozen=True, slots=True)
class _Snapshot:
    activation: int | None = None
    frame: JointTargetFrame | None = None
    q_des: tuple[float, ...] | None = None


class LatestJointTarget:
    """Keep only the newest valid frame for the current state activation.

    Writes and lifecycle changes are serialized. Reads return an immutable
    snapshot without waiting for a producer lock. Reading does not consume a
    frame: no new frame means keep the most recent target indefinitely. The
    state, rather than this mailbox, owns applied-command acknowledgement.
    """

    def __init__(
        self,
        dimension: int,
        position_min: Sequence[float] | None = None,
        position_max: Sequence[float] | None = None,
        *,
        joint_names: Sequence[str] | None = None,
        state_id: int | None = None,
        state_key: str | None = None,
    ) -> None:
        self.dimension = _integer(dimension, "dimension", 1)
        self.position_min = self._bounds(position_min, "position_min")
        self.position_max = self._bounds(position_max, "position_max")
        if self.position_min is not None and self.position_max is not None:
            if any(lo > hi for lo, hi in zip(self.position_min, self.position_max)):
                raise ValueError("position_min must not exceed position_max")
        if joint_names is not None:
            if isinstance(joint_names, (str, bytes)):
                raise ValueError("joint_names must contain unique names matching dimension")
            try:
                joint_names = tuple(joint_names)
            except TypeError as exc:
                raise ValueError("joint_names must contain unique names matching dimension") from exc
            if (len(joint_names) != self.dimension
                    or any(not isinstance(name, str) or not name.strip() for name in joint_names)
                    or len(set(joint_names)) != self.dimension):
                raise ValueError("joint_names must contain unique names matching dimension")
        self.joint_names = joint_names
        self.state_id = None if state_id is None else _integer(state_id, "state_id", 0)
        if self.state_id is not None and self.state_id > 65535:
            raise ValueError("state_id must not exceed 65535")
        if state_key is not None and (not isinstance(state_key, str) or not state_key.strip()):
            raise ValueError("state_key must be a nonempty string")
        self.state_key = state_key
        self._lock = threading.Lock()
        # An opaque random starting point isolates separate state objects and
        # process restarts sharing an endpoint. Leave room for increments while
        # remaining exactly representable by JSON clients using binary64 numbers.
        self._next_activation = secrets.randbits(52)
        self._snapshot = _Snapshot()

    def _bounds(self, value: Sequence[float] | None, name: str) -> tuple[float, ...] | None:
        if value is None:
            return None
        result = _positions(value, name)
        if len(result) != self.dimension:
            raise ValueError(f"{name} must contain {self.dimension} positions")
        return result

    @property
    def activation(self) -> int | None:
        """Current activation token, or None while the state is inactive."""
        return self._snapshot.activation

    def activate(self) -> int:
        """Start a fresh activation and discard every previous target."""
        with self._lock:
            self._next_activation += 1
            if self._next_activation > _MAX_SAFE_JSON_INTEGER:
                self._next_activation = secrets.randbits(52) + 1
            self._snapshot = _Snapshot(self._next_activation)
            return self._next_activation

    def deactivate(self) -> None:
        """Stop accepting targets and discard the previous activation's frame."""
        with self._lock:
            self._snapshot = _Snapshot()

    def latest(self) -> JointTargetFrame | None:
        return self._snapshot.frame

    def status(self) -> dict:
        """Read one immutable activation/receipt/command snapshot.

        Sequence describes reception, while q_des describes the last command
        accepted by the runtime. Neither is a measurement of physical tracking.
        """
        snapshot = self._snapshot
        return {
            "activation": snapshot.activation,
            "dimension": self.dimension,
            "joint_names": None if self.joint_names is None else list(self.joint_names),
            "q_des": None if snapshot.q_des is None else list(snapshot.q_des),
            "sequence": None if snapshot.frame is None else snapshot.frame.sequence,
            "state_id": self.state_id,
            "state_key": self.state_key,
        }

    def record_applied(self, activation: int, q_des: Sequence[float]) -> bool:
        """Record a committed command only in the activation that prepared it."""
        activation = _integer(activation, "activation", 1)
        positions = _positions(q_des, "q_des")
        if len(positions) != self.dimension:
            raise ValueError(f"q_des must contain {self.dimension} positions")
        with self._lock:
            current = self._snapshot
            if current.activation != activation:
                return False
            self._snapshot = _Snapshot(current.activation, current.frame, positions)
            return True

    def publish(self, frame: JointTargetFrame) -> bool:
        """Accept a valid newer target; return False for inactive or stale frames.

        Invalid dimensions, nonfinite positions, and configured limit violations
        raise ValueError and leave the latest target unchanged.
        """
        if not isinstance(frame, JointTargetFrame):
            raise TypeError("frame must be a JointTargetFrame")
        if len(frame.q_des) != self.dimension:
            raise ValueError(f"q_des must contain {self.dimension} positions")
        if self.position_min is not None:
            if any(q < limit for q, limit in zip(frame.q_des, self.position_min)):
                raise ValueError("q_des is below position_min")
        if self.position_max is not None:
            if any(q > limit for q, limit in zip(frame.q_des, self.position_max)):
                raise ValueError("q_des is above position_max")
        with self._lock:
            current = self._snapshot
            if current.activation is None or frame.activation != current.activation:
                return False
            if current.frame is not None and frame.sequence <= current.frame.sequence:
                return False
            self._snapshot = _Snapshot(current.activation, frame, current.q_des)
            return True


TARGET_SCHEMA = "cadence.joint-target.v1"
TARGET_SCHEMAS = (JOINT_TARGET_SCHEMA, TARGET_SCHEMA)


def _bind_address(host):
    if not isinstance(host, str) or not host.strip():
        raise ValueError("upper target receiver requires an explicit unicast IP address or wildcard")
    try:
        address = ipaddress.ip_address(host)
    except ValueError as exc:
        raise ValueError("upper target receiver requires an explicit unicast IP address or wildcard") from exc
    routed = getattr(address, "ipv4_mapped", None) or address
    if routed.is_multicast or str(routed) == "255.255.255.255":
        raise ValueError("upper target receiver requires a unicast IP address or wildcard")
    return address


class JointTargetUdpReceiver:
    """An explicitly started JSON adapter on an IPv4/IPv6 address or wildcard.

    A status request is {"schema": "cadence.joint-target.v1", "type": "status"}.
    A target request adds type="target", activation, sequence, and q_des. The
    returned accepted flag acknowledges mailbox reception only, never execution.
    Starting or stopping the receiver does not activate or deactivate the state.
    """

    def __init__(
        self,
        targets: LatestJointTarget,
        bind: tuple[str, int] = ("127.0.0.1", 0),
    ) -> None:
        address = _bind_address(bind[0])
        port = _integer(bind[1], "port", 0)
        if port > 65535:
            raise ValueError("port must not exceed 65535")
        self.targets = targets
        self.bind = (str(address), port)
        self._family = socket.AF_INET6 if address.version == 6 else socket.AF_INET
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    @property
    def address(self) -> tuple[str, int] | None:
        sock = self._socket
        return None if sock is None else sock.getsockname()[:2]

    def start(self) -> "JointTargetUdpReceiver":
        if self._thread is not None:
            raise RuntimeError("upper target receiver is already started")
        sock = socket.socket(self._family, socket.SOCK_DGRAM)
        try:
            sock.bind(self.bind)
            sock.settimeout(0.05)
        except BaseException:
            sock.close()
            raise
        self._socket = sock
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="cadence-upper-target", daemon=True
        )
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            if self._thread.is_alive():
                raise RuntimeError("upper target receiver failed to stop")
        if self._socket is not None:
            self._socket.close()
        self._thread = None
        self._socket = None

    def __enter__(self) -> "JointTargetUdpReceiver":
        return self.start()

    def __exit__(self, *args: object) -> None:
        self.stop()

    def _response(self, data: bytes) -> dict:
        schema = TARGET_SCHEMA
        try:
            request = json.loads(data, object_pairs_hook=_object, parse_constant=_nonfinite)
            if isinstance(request, dict) and request.get("schema") in TARGET_SCHEMAS:
                schema = request["schema"]
            if not isinstance(request, dict) or request.get("schema") not in TARGET_SCHEMAS:
                raise ValueError(f"request schema must be in {TARGET_SCHEMAS}")
            kind = request.get("type")
            if kind == "status":
                if set(request) != {"schema", "type"}:
                    raise ValueError("status request contains unknown fields")
                return {
                    "schema": schema,
                    "type": "status",
                    **self.targets.status(),
                }
            if kind != "target":
                raise ValueError("request type must be status or target")
            if set(request) != {"schema", "type", "activation", "sequence", "q_des"}:
                raise ValueError("target request requires activation, sequence, and q_des only")
            frame = JointTargetFrame(
                sequence=request["sequence"],
                q_des=request["q_des"],
                activation=request["activation"],
            )
            accepted = self.targets.publish(frame)
            return {
                "schema": schema,
                "type": "receipt",
                "accepted": accepted,
                "activation": frame.activation,
                "sequence": frame.sequence,
                "reason": "received" if accepted else "inactive_or_stale",
            }
        except (ValueError, TypeError, OverflowError, RecursionError) as exc:
            return {
                "schema": schema,
                "type": "receipt",
                "accepted": False,
                "reason": "invalid",
                "error": str(exc),
            }

    def _run(self) -> None:
        sock = self._socket
        assert sock is not None
        while not self._stop.is_set():
            try:
                data, peer = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            response = self._response(data)
            try:
                sock.sendto(json.dumps(response, allow_nan=False).encode("utf-8"), peer)
            except OSError:
                # A producer may close its socket without waiting for a receipt.
                continue
