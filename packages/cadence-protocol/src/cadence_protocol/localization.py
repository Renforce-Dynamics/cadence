"""Versioned, task-independent localization packets and a UDP producer.

Positions and linear velocities are expressed in ``frame_id`` (usually world),
and the unit wxyz quaternion rotates vectors from ``child_frame_id`` (usually
policy_root) into that frame. The protocol never performs a frame transform.

``source_age_s`` is sample age when sent, measured on the producer's monotonic
clock. Receivers add their own elapsed time and cap the advertised TTL. This
avoids comparing unrelated clocks; unmeasured transport/OS queue delay is not
included. Applications requiring that delay to be measured need synchronized
clock timestamps or a transport with a bounded delay before publishing here.
``source_timestamp_us`` is Unix microseconds at sampling, for trace alignment
only (zero means unavailable); receivers never use it as a monotonic deadline.

Session IDs are ordered producer epochs, not random tokens. A restarted source
must use an ID greater than its previous one. Unix microseconds are a convenient
default; producers with a clock that can move backwards must persist the epoch.
Sequences increase within a session without wrapping. Both are JSON-safe ints.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
import math
from numbers import Integral, Real
import socket
import threading
import time


LOCALIZATION_SCHEMA = "cadence.localization.v1"
MAX_DATAGRAM_BYTES = 8192
MAX_SAFE_INTEGER = (1 << 53) - 1


def _integer(value, name, minimum=0, maximum=MAX_SAFE_INTEGER):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f"{name} must be an integer")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be in [{minimum}, {maximum}]")
    return int(value)


def _real(value, name, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(result) or result < 0 or (positive and result == 0):
        qualifier = "positive" if positive else "nonnegative"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return result


def _name(value, name):
    if (not isinstance(value, str) or not value or len(value) > 128
            or any(character.isspace() or ord(character) < 32 for character in value)):
        raise ValueError(f"{name} must contain 1..128 characters without whitespace")
    return value


def _vector(value, name, dimension):
    if not isinstance(value, (tuple, list)) or len(value) != dimension:
        raise ValueError(f"{name} must contain {dimension} finite numbers")
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, Real):
            raise ValueError(f"{name} must contain {dimension} finite numbers")
        try:
            item = float(item)
        except (ValueError, OverflowError) as error:
            raise ValueError(f"{name} must contain {dimension} finite numbers") from error
        if not math.isfinite(item):
            raise ValueError(f"{name} must contain {dimension} finite numbers")
        result.append(item)
    return tuple(result)


@dataclass(frozen=True, slots=True)
class LocalizationSample:
    """An immutable sample; ``valid=False`` explicitly withdraws localization."""

    position_w_m: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]
    linear_velocity_w_mps: tuple[float, float, float]
    source: str
    session_id: int
    sequence: int
    source_timestamp_us: int = 0
    source_age_s: float = 0.0
    ttl_s: float = 0.25
    frame_id: str = "world"
    child_frame_id: str = "policy_root"
    valid: bool = True

    def __post_init__(self):
        for name in ("source", "frame_id", "child_frame_id"):
            object.__setattr__(self, name, _name(getattr(self, name), name))
        if self.frame_id == self.child_frame_id:
            raise ValueError("frame_id and child_frame_id must be distinct")
        for name, minimum in (("session_id", 1), ("sequence", 0), ("source_timestamp_us", 0)):
            object.__setattr__(self, name, _integer(getattr(self, name), name, minimum))
        for name, positive in (("source_age_s", False), ("ttl_s", True)):
            object.__setattr__(self, name, _real(getattr(self, name), name, positive=positive))
        for name, dimension in (("position_w_m", 3), ("linear_velocity_w_mps", 3),
                                ("orientation_wxyz", 4)):
            object.__setattr__(self, name, _vector(getattr(self, name), name, dimension))
        norm = math.hypot(*self.orientation_wxyz)
        if abs(norm - 1.0) > 1e-3:
            raise ValueError("orientation_wxyz must be a unit quaternion (tolerance 0.001)")
        object.__setattr__(self, "orientation_wxyz", tuple(value / norm for value in self.orientation_wxyz))
        if not isinstance(self.valid, bool):
            raise ValueError("valid must be a boolean")


def encode_localization(sample: LocalizationSample) -> bytes:
    if not isinstance(sample, LocalizationSample):
        raise TypeError("sample must be a LocalizationSample")
    data = {"schema": LOCALIZATION_SCHEMA, "type": "localization", **asdict(sample)}
    return json.dumps(data, allow_nan=False, separators=(",", ":")).encode("utf-8")


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate localization field: {key}")
        result[key] = value
    return result


def _nonfinite(value):
    raise ValueError(f"nonfinite JSON value: {value}")


def decode_localization(payload: bytes) -> LocalizationSample:
    if not isinstance(payload, bytes) or not payload or len(payload) > MAX_DATAGRAM_BYTES:
        raise ValueError(f"localization packet must contain 1..{MAX_DATAGRAM_BYTES} bytes")
    try:
        data = json.loads(payload, object_pairs_hook=_object, parse_constant=_nonfinite)
    except (UnicodeError, json.JSONDecodeError, RecursionError) as error:
        raise ValueError("invalid localization JSON") from error
    fields = set(LocalizationSample.__dataclass_fields__) | {"schema", "type"}
    if not isinstance(data, dict) or set(data) != fields:
        raise ValueError("localization packet fields do not match the versioned schema")
    if data.pop("schema") != LOCALIZATION_SCHEMA or data.pop("type") != "localization":
        raise ValueError(f"expected {LOCALIZATION_SCHEMA} localization packet")
    return LocalizationSample(**data)


_session_lock = threading.Lock()
_last_session_id = 0


def new_localization_session() -> int:
    """Create an ordered epoch; cross-process clock regressions require persistence."""
    global _last_session_id
    with _session_lock:
        epoch = max(time.time_ns() // 1000, _last_session_id + 1)
        _last_session_id = _integer(epoch, "session_id", 1)
        return _last_session_id


class LocalizationClient:
    """One-way localization publisher with automatic session/sequence management.

    send() returning a sample acknowledges a local UDP send only. It neither
    confirms receipt nor changes a runtime state. Sequences are consumed even if
    sending fails, so retrying cannot accidentally reuse a sequence number.
    """

    def __init__(self, host="127.0.0.1", port=15110, *, source="localization",
                 frame_id="world", child_frame_id="policy_root", session_id=None,
                 ttl_s=0.25):
        _name(host, "host")
        port = _integer(port, "port", 1, 65535)
        self._base = LocalizationSample(
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            source, new_localization_session() if session_id is None else session_id,
            0, ttl_s=ttl_s, frame_id=frame_id, child_frame_id=child_frame_id,
        )
        family, _, _, _, endpoint = socket.getaddrinfo(host, port, type=socket.SOCK_DGRAM)[0]
        self._socket = socket.socket(family, socket.SOCK_DGRAM)
        try:
            self._socket.setblocking(False)
            self._socket.connect(endpoint)
        except BaseException:
            self._socket.close()
            raise
        self._sequence = 0
        self._closed = False

    @property
    def session_id(self):
        return self._base.session_id

    def send(self, position_w_m, orientation_wxyz, linear_velocity_w_mps,
             *, source_age_s=0.0, source_timestamp_us=0, valid=True):
        if self._closed:
            raise ValueError("localization client is closed")
        sample = replace(
            self._base, sequence=self._sequence, position_w_m=position_w_m,
            orientation_wxyz=orientation_wxyz, linear_velocity_w_mps=linear_velocity_w_mps,
            source_age_s=source_age_s, source_timestamp_us=source_timestamp_us, valid=valid,
        )
        payload = encode_localization(sample)
        self._sequence += 1
        self._socket.send(payload)
        return sample

    def close(self):
        self._socket.close()
        self._closed = True

    def __enter__(self):
        if self._closed:
            raise ValueError("localization client is closed")
        return self

    def __exit__(self, *args):
        self.close()
