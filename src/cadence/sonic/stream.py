"""UDP motion-reference stream receiver for the sonic_stream state.

Wire schema ``cadence.motion-ref.v1`` (JSON over UDP, one request per
datagram, one reply per request):

  status query:  {"schema", "type": "status"}
  frames:        {"schema", "type": "frames", "activation": int,
                  "sequence": int, "dt_ms": 20,
                  "frames": [{"root_quat_wxyz": [4], "q": [29], "dq": [29]}, ...]}

Frames carry absolute joint positions and velocities in IsaacLab order plus
the reference pelvis quaternion. Frames in one packet are consecutive; the
last frame is stamped at receipt, earlier frames at ``dt_ms`` spacing before
it. The mailbox samples a delay line: reference time is
``newest_frame_time - delay_s``, and the 10 future frames are taken at
``frame_dt * future_frame_skip`` spacing with nearest-neighbor lookup
(lerp/slerp between the two bracketing frames, matching the SONIC teleop
reference buffer). Reception never implies that a command has been applied to
the robot.
"""

from __future__ import annotations

from collections.abc import Mapping
import ipaddress
import json
from numbers import Integral, Real
import math
import secrets
import socket
import threading
import time

import numpy as np

from .contract import (
    FUTURE_FRAMES,
    JOINT_DIM,
    anchor_6d,
    compute_yaw_offset,
    quat_slerp,
)

MOTION_REF_SCHEMA = "cadence.motion-ref.v1"

_MAX_SAFE_JSON_INTEGER = (1 << 53) - 1
_MAX_FRAMES_PER_PACKET = 64
_MAX_PACKET_BYTES = 65535


def _integer(value, name, minimum):
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f"{name} must be an integer")
    value = int(value)
    if value < minimum:
        raise ValueError(f"{name} must be at least {minimum}")
    return value


def _vector(value, name, size):
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite vector of {size}") from exc
    if array.shape != (size,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite vector of {size}")
    return array


def _frame(value):
    if not isinstance(value, Mapping) or set(value) != {"root_quat_wxyz", "q", "dq"}:
        raise ValueError("frame requires exactly root_quat_wxyz, q and dq")
    quat = _vector(value["root_quat_wxyz"], "root_quat_wxyz", 4)
    norm = np.linalg.norm(quat)
    if norm < 1e-6:
        raise ValueError("root_quat_wxyz must be nonzero")
    return (quat / norm,
            _vector(value["q"], "q", JOINT_DIM),
            _vector(value["dq"], "dq", JOINT_DIM))


class LatestMotionRef:
    """Delay-line buffer for the current state activation's reference stream.

    Writes and lifecycle changes are serialized; reads return immutable
    snapshots. Stamps are seconds on ``time.monotonic``; the control loop
    compares them against the same clock.
    """

    def __init__(self, *, state_id=None, state_key=None, max_frames=512):
        self.state_id = None if state_id is None else _integer(state_id, "state_id", 0)
        if state_key is not None and (not isinstance(state_key, str) or not state_key.strip()):
            raise ValueError("state_key must be a nonempty string")
        self.state_key = state_key
        self.max_frames = _integer(max_frames, "max_frames", FUTURE_FRAMES + 1)
        self._lock = threading.Lock()
        self._next_activation = secrets.randbits(52)
        self._activation = None
        self._frames = []          # list of (stamp_s, quat, q, dq), oldest first
        self._last_sequence = None

    @property
    def activation(self):
        """Current activation token, or None while the state is inactive."""
        return self._activation

    def activate(self):
        """Start a fresh activation and discard every buffered frame."""
        with self._lock:
            self._next_activation += 1
            if self._next_activation > _MAX_SAFE_JSON_INTEGER:
                self._next_activation = secrets.randbits(52) + 1
            self._activation = self._next_activation
            self._frames = []
            self._last_sequence = None
            return self._next_activation

    def deactivate(self):
        """Stop accepting frames and discard the buffered reference."""
        with self._lock:
            self._activation = None
            self._frames = []
            self._last_sequence = None

    def publish(self, activation, sequence, frame_dt_s, frames, receipt_s=None):
        """Append one packet's frames; return False for inactive/stale packets.

        The last frame is stamped at ``receipt_s`` (time.monotonic by default);
        earlier frames are spaced ``frame_dt_s`` before it. Out-of-order frames
        already covered by the buffer are dropped.
        """
        activation = _integer(activation, "activation", 1)
        sequence = _integer(sequence, "sequence", 0)
        if (isinstance(frame_dt_s, bool) or not isinstance(frame_dt_s, Real)
                or not math.isfinite(frame_dt_s) or not 0.001 <= frame_dt_s <= 1.0):
            raise ValueError("frame_dt_s must be in [0.001, 1.0] seconds")
        if not isinstance(frames, (list, tuple)) or not 1 <= len(frames) <= _MAX_FRAMES_PER_PACKET:
            raise ValueError(f"frames must contain 1..{_MAX_FRAMES_PER_PACKET} frames")
        parsed = tuple(_frame(item) for item in frames)
        if receipt_s is None:
            receipt_s = time.monotonic()
        receipt_s = float(receipt_s)
        with self._lock:
            if self._activation is None or activation != self._activation:
                return False
            if self._last_sequence is not None and sequence <= self._last_sequence:
                return False
            self._last_sequence = sequence
            last_stamp = receipt_s
            for offset, (quat, q, dq) in enumerate(reversed(parsed)):
                stamp = last_stamp - offset * frame_dt_s
                if self._frames and stamp <= self._frames[-1][0]:
                    continue
                self._frames.append((stamp, quat, q, dq))
            if len(self._frames) > self.max_frames:
                del self._frames[: len(self._frames) - self.max_frames]
            return True

    def status(self):
        """Read one immutable activation/reception snapshot."""
        with self._lock:
            return {
                "activation": self._activation,
                "frames": len(self._frames),
                "sequence": self._last_sequence,
                "state_id": self.state_id,
                "state_key": self.state_key,
                "latest_stamp_s": None if not self._frames else self._frames[-1][0],
                "oldest_stamp_s": None if not self._frames else self._frames[0][0],
            }

    def newest_stamp(self):
        with self._lock:
            return None if not self._frames else self._frames[-1][0]

    def reset_to_latest(self):
        """Drop all but the newest frame, matching SONIC's reconnect handling.

        Called when a stale segment is abandoned so the delay line refills
        from fresh data instead of interpolating across the gap.
        """
        with self._lock:
            if self._frames:
                self._frames = [self._frames[-1]]

    def is_stale(self, now_s, stale_s):
        """True once the newest frame is older than ``stale_s``."""
        with self._lock:
            if not self._frames:
                return False
            return now_s - self._frames[-1][0] > stale_s

    def _sample_locked(self, stamp_s):
        """Nearest-bracket lerp/slerp, clamped at both ends of the buffer."""
        frames = self._frames
        if stamp_s <= frames[0][0]:
            _, quat, q, dq = frames[0]
            return quat, q, dq
        if stamp_s >= frames[-1][0]:
            _, quat, q, dq = frames[-1]
            return quat, q, dq
        stamps = [f[0] for f in frames]
        hi = int(np.searchsorted(np.asarray(stamps), stamp_s, side="left"))
        lo = hi - 1
        t0, t1 = frames[lo][0], frames[hi][0]
        alpha = 0.0 if t1 <= t0 else (stamp_s - t0) / (t1 - t0)
        quat = quat_slerp(frames[lo][1], frames[hi][1], alpha)
        q = (1.0 - alpha) * frames[lo][2] + alpha * frames[hi][2]
        dq = (1.0 - alpha) * frames[lo][3] + alpha * frames[hi][3]
        return quat, q, dq

    def sample_reference(self, now_s, delay_s):
        """Sample the delayed reference frame; None while the line is unfilled."""
        with self._lock:
            if len(self._frames) < 2:
                return None
            base = now_s - delay_s
            if base < self._frames[0][0] or base > self._frames[-1][0]:
                return None
            return self._sample_locked(base)

    def latest_yaw_offset(self, robot_root_quat_wxyz):
        """Yaw offset aligning the newest reference frame to the robot yaw."""
        with self._lock:
            if not self._frames:
                return None
            return compute_yaw_offset(robot_root_quat_wxyz, self._frames[-1][1])

    def tokenizer_slice(self, now_s, delay_s, robot_root_quat_wxyz,
                        yaw_offset_rad, frame_dt_s, future_frame_skip,
                        require_full_window):
        """Build the 640-float prefix from the delay line, or None.

        Before the first successful slice (``require_full_window``) the delayed
        base time must lie inside the buffered window; afterwards samples clamp
        at the buffer ends, matching the SONIC teleop reference buffer.
        """
        out = np.zeros(FUTURE_FRAMES * (2 * JOINT_DIM + 6), dtype=np.float64)
        with self._lock:
            if len(self._frames) < 2:
                return None
            base = now_s - delay_s
            if require_full_window and not self._frames[0][0] <= base <= self._frames[-1][0]:
                return None
            step = frame_dt_s * future_frame_skip
            for k in range(FUTURE_FRAMES):
                quat, q, dq = self._sample_locked(base + k * step)
                out[k * JOINT_DIM:(k + 1) * JOINT_DIM] = q
                start = FUTURE_FRAMES * JOINT_DIM + k * JOINT_DIM
                out[start:start + JOINT_DIM] = dq
                offset = FUTURE_FRAMES * 2 * JOINT_DIM + k * 6
                out[offset:offset + 6] = anchor_6d(
                    robot_root_quat_wxyz, quat, yaw_offset_rad)
        return out.astype(np.float32)


def _bind_address(host):
    if not isinstance(host, str) or not host.strip():
        raise ValueError("motion-ref receiver requires an explicit unicast IP address or wildcard")
    try:
        address = ipaddress.ip_address(host)
    except ValueError as exc:
        raise ValueError("motion-ref receiver requires an explicit unicast IP address or wildcard") from exc
    routed = getattr(address, "ipv4_mapped", None) or address
    if routed.is_multicast or str(routed) == "255.255.255.255":
        raise ValueError("motion-ref receiver requires a unicast IP address or wildcard")
    return address


class MotionRefUdpReceiver:
    """An explicitly started JSON adapter owned by the sonic_stream state.

    A status request is {"schema": "cadence.motion-ref.v1", "type": "status"}.
    A frames request adds type="frames", activation, sequence, dt_ms and
    frames. The returned accepted flag acknowledges mailbox reception only,
    never execution. Starting or stopping the receiver does not activate or
    deactivate the mailbox.
    """

    def __init__(self, reference, bind=("127.0.0.1", 15120)):
        address = _bind_address(bind[0])
        port = _integer(bind[1], "port", 0)
        if port > 65535:
            raise ValueError("port must not exceed 65535")
        self.reference = reference
        self.bind = (str(address), port)
        self._family = socket.AF_INET6 if address.version == 6 else socket.AF_INET
        self._socket = None
        self._thread = None
        self._stop = threading.Event()

    @property
    def address(self):
        sock = self._socket
        return None if sock is None else sock.getsockname()[:2]

    def start(self):
        if self._thread is not None:
            raise RuntimeError("motion-ref receiver is already started")
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
            target=self._run, name="cadence-motion-ref", daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            if self._thread.is_alive():
                raise RuntimeError("motion-ref receiver failed to stop")
        if self._socket is not None:
            self._socket.close()
        self._thread = None
        self._socket = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *args):
        self.stop()

    def _response(self, data):
        try:
            request = json.loads(data)
            if not isinstance(request, dict) or request.get("schema") != MOTION_REF_SCHEMA:
                raise ValueError(f"request schema must be {MOTION_REF_SCHEMA}")
            kind = request.get("type")
            if kind == "status":
                if set(request) != {"schema", "type"}:
                    raise ValueError("status request contains unknown fields")
                return {"schema": MOTION_REF_SCHEMA, "type": "status",
                        **self.reference.status()}
            if kind != "frames":
                raise ValueError("request type must be status or frames")
            if set(request) != {"schema", "type", "activation", "sequence", "dt_ms", "frames"}:
                raise ValueError("frames request requires activation, sequence, dt_ms and frames only")
            dt_ms = request["dt_ms"]
            if isinstance(dt_ms, bool) or not isinstance(dt_ms, (int, float)) or not math.isfinite(dt_ms):
                raise ValueError("dt_ms must be a finite number")
            accepted = self.reference.publish(
                request["activation"], request["sequence"], dt_ms / 1000.0,
                request["frames"], receipt_s=time.monotonic())
            return {
                "schema": MOTION_REF_SCHEMA,
                "type": "receipt",
                "accepted": accepted,
                "activation": request["activation"],
                "sequence": request["sequence"],
                "reason": "received" if accepted else "inactive_or_stale",
            }
        except (ValueError, TypeError, OverflowError, RecursionError) as exc:
            return {
                "schema": MOTION_REF_SCHEMA,
                "type": "receipt",
                "accepted": False,
                "reason": "invalid",
                "error": str(exc),
            }

    def _run(self):
        sock = self._socket
        assert sock is not None
        while not self._stop.is_set():
            try:
                data, peer = sock.recvfrom(_MAX_PACKET_BYTES)
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
