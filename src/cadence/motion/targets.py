"""Activation-scoped joint targets and an optional loopback UDP adapter.

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
    ) -> None:
        self.dimension = _integer(dimension, "dimension", 1)
        self.position_min = self._bounds(position_min, "position_min")
        self.position_max = self._bounds(position_max, "position_max")
        if self.position_min is not None and self.position_max is not None:
            if any(lo > hi for lo, hi in zip(self.position_min, self.position_max)):
                raise ValueError("position_min must not exceed position_max")
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
            self._snapshot = _Snapshot(current.activation, frame)
            return True


TARGET_SCHEMA = "cadence.joint-target.v1"


class JointTargetUdpReceiver:
    """An explicitly started, loopback-only JSON adapter for a target mailbox.

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
        try:
            address = ipaddress.ip_address(bind[0])
        except ValueError as exc:
            raise ValueError("upper target receiver requires a loopback IP address") from exc
        if not address.is_loopback:
            raise ValueError("upper target receiver requires a loopback IP address")
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
        try:
            request = json.loads(data)
            if not isinstance(request, dict) or request.get("schema") != TARGET_SCHEMA:
                raise ValueError(f"request schema must be {TARGET_SCHEMA}")
            kind = request.get("type")
            if kind == "status":
                if set(request) != {"schema", "type"}:
                    raise ValueError("status request contains unknown fields")
                return {
                    "schema": TARGET_SCHEMA,
                    "type": "status",
                    "activation": self.targets.activation,
                    "dimension": self.targets.dimension,
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
                "schema": TARGET_SCHEMA,
                "type": "receipt",
                "accepted": accepted,
                "activation": frame.activation,
                "sequence": frame.sequence,
                "reason": "received" if accepted else "inactive_or_stale",
            }
        except (ValueError, TypeError, OverflowError, RecursionError) as exc:
            return {
                "schema": TARGET_SCHEMA,
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
