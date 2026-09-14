"""Cadence adapter for the A3 SDK's synchronized state and command lifecycle.

``aimrt`` is the hardware transport. ``sdk_mock`` is an explicit, stationary
native SDK fixture with an in-memory command sink; it models no dynamics and
never configures an AimRT transport. The SDK owns snapshot sequencing, command
acceptance, watchdog damping, the neck joints, and shutdown damping.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import fields
import math
from numbers import Integral, Real
import os
from pathlib import Path

import numpy as np
from cadence_api import RobotState


A3_POLICY_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
CONTROL_JOINT_NAMES = A3_POLICY_JOINTS
STATE_TOPICS = ("waist", "neck", "arms", "legs", "pelvis_imu", "torso_imu")
COMMAND_TOPICS = ("waist", "neck", "arms", "legs")


def _keys(raw, allowed, required, name):
    if not isinstance(raw, Mapping):
        raise ValueError(f"{name} must be a mapping")
    if set(raw) - set(allowed):
        raise ValueError(f"unknown {name} fields: {sorted(set(raw) - set(allowed))}")
    if set(required) - set(raw):
        raise ValueError(f"missing {name} fields: {sorted(set(required) - set(raw))}")


def _number(value, name, *, positive=False, integer=False):
    if (
        isinstance(value, bool)
        or not isinstance(value, Integral if integer else Real)
        or not math.isfinite(value)
        or (value <= 0 if positive else value < 0)
    ):
        adjective = "positive" if positive else "nonnegative"
        raise ValueError(f"{name} must be a finite {adjective} {'integer' if integer else 'number'}")
    if integer and value > 2**31 - 1:
        raise ValueError(f"{name} exceeds the SDK integer range")
    return int(value) if integer else float(value)


def _path(value, base_dir, name):
    if not isinstance(value, (str, Path)) or not str(value).strip():
        raise ValueError(f"{name} must be a nonempty file path")
    if "://" in str(value):
        raise ValueError(f"{name} requires an explicit filesystem path")
    path = Path(value).expanduser()
    return (path if path.is_absolute() else base_dir / path).resolve()


def _topics(raw, names, label):
    _keys(raw, names, names, label)
    if any(not isinstance(value, str) or not value.startswith("/") or not value.strip("/") for value in raw.values()):
        raise ValueError(f"{label} requires absolute, nonempty topic names")
    return dict(raw)


def _load_sdk():
    try:
        import agi3sdk
    except ImportError as error:
        raise RuntimeError("A3 backends require the cadence[a3] extra and a built agi3sdk native library") from error
    return agi3sdk


class A3Backend:
    """A lazy SDK-backed RobotIO; construction performs no hardware access.

    Hardware publication retains the existing ``A3_CONFIRM_ONBOARD=YES`` gate.
    Read-only callers must skip ``write_command``; accidentally calling it raises
    before the SDK receives a command. A returned write means the SDK accepted
    the command, so callers may then commit their prepared runtime transaction.
    """

    dimension = len(A3_POLICY_JOINTS)

    def __init__(self, raw: Mapping, *, base_dir: str | Path | None = None):
        _keys(raw, {
            "kind", "transport", "read_only", "command_publish_enabled",
            "state_timeout_ms", "command_timeout_ms", "startup_timeout_s",
            "safe_damping_kd", "neck_kp", "neck_kd", "library_path", "aimrt",
        }, {"transport"}, "backend")
        if raw.get("kind", "a3") != "a3":
            raise ValueError("A3Backend requires backend.kind: a3")
        self.transport = raw["transport"]
        if not isinstance(self.transport, str) or self.transport not in {"aimrt", "sdk_mock"}:
            raise ValueError("backend.transport must be aimrt or sdk_mock")
        self.read_only = raw.get("read_only", True)
        self.command_publish_enabled = raw.get("command_publish_enabled", False)
        if type(self.read_only) is not bool or type(self.command_publish_enabled) is not bool:
            raise ValueError("read_only and command_publish_enabled must be booleans")
        if self.transport == "aimrt" and self.read_only == self.command_publish_enabled:
            raise ValueError("AimRT command_publish_enabled must be false exactly when read_only is true")
        if self.transport == "sdk_mock" and (self.command_publish_enabled or raw.get("aimrt") is not None):
            raise ValueError("sdk_mock cannot configure AimRT or enable hardware publication")
        self.state_timeout_s = _number(raw.get("state_timeout_ms", 50), "state_timeout_ms", positive=True, integer=True) / 1000
        self.command_timeout_ms = _number(raw.get("command_timeout_ms", 100), "command_timeout_ms", positive=True, integer=True)
        self.startup_timeout_s = _number(raw.get("startup_timeout_s", 5.0), "startup_timeout_s", positive=True)
        base_dir = Path.cwd() if base_dir is None else Path(base_dir)
        self._sdk_options = {
            "read_only": self.read_only,
            "state_timeout_ms": round(self.state_timeout_s * 1000),
            "command_timeout_ms": self.command_timeout_ms,
            "safe_damping_kd": _number(raw.get("safe_damping_kd", 3.0), "safe_damping_kd"),
            "neck_kp": _number(raw.get("neck_kp", 40.0), "neck_kp"),
            "neck_kd": _number(raw.get("neck_kd", 2.0), "neck_kd"),
            "command_publish_enabled": self.command_publish_enabled,
        }
        if raw.get("library_path") is not None:
            self._sdk_options["library_path"] = _path(raw["library_path"], base_dir, "library_path")
        if self.transport == "aimrt":
            aimrt = raw.get("aimrt")
            _keys(aimrt, {"config_path", "maximum_sync_skew_ms", "state_topics", "command_topics"}, {"config_path", "state_topics", "command_topics"}, "backend.aimrt")
            skew_ms = _number(aimrt.get("maximum_sync_skew_ms", 20), "maximum_sync_skew_ms")
            if skew_ms > (2**63 - 1) / 1_000_000:
                raise ValueError("maximum_sync_skew_ms exceeds the SDK integer range")
            self._sdk_options.update({
                "aimrt_config_path": _path(aimrt["config_path"], base_dir, "aimrt.config_path"),
                "maximum_sync_skew_ns": round(skew_ms * 1_000_000),
                "state_topics": _topics(aimrt["state_topics"], STATE_TOPICS, "aimrt.state_topics"),
                "command_topics": _topics(aimrt["command_topics"], COMMAND_TOPICS, "aimrt.command_topics"),
            })
        self._io = None
        self._sdk = None
        self._mock_sequence = 0

    @classmethod
    def from_mapping(cls, raw, *, base_dir=None):
        return cls(raw, base_dir=base_dir)

    def start(self):
        if self._io is not None:
            return
        if self.command_publish_enabled and os.environ.get("A3_CONFIRM_ONBOARD") != "YES":
            raise RuntimeError("command publication requires A3_CONFIRM_ONBOARD=YES")
        # Check before constructing the SDK, which allocates the native handle.
        for key in ("aimrt_config_path", "library_path"):
            if key in self._sdk_options and not self._sdk_options[key].is_file():
                raise FileNotFoundError(f"{key} not found: {self._sdk_options[key]}")
        sdk = _load_sdk()
        io = sdk.A3RobotIO(**self._sdk_options)
        try:
            if self.transport == "sdk_mock" and not self.read_only:
                io._install_memory_command_writer_for_test()
            io.start()
        except BaseException:
            try:
                io.close()
            except Exception:
                pass
            raise
        self._io = io
        self._sdk = sdk
        self._mock_sequence = 0

    def _running(self):
        if self._io is None:
            raise RuntimeError("A3 backend is not started")
        return self._io

    def read_state(self, timeout_s=None):
        io = self._running()
        timeout = self.state_timeout_s if timeout_s is None else _number(timeout_s, "timeout_s")
        if timeout > (2**31 - 1) / 1000:
            raise ValueError("timeout_s exceeds the SDK integer range")
        if self.transport == "sdk_mock":
            self._mock_sequence += 1
            io._inject_hardware_state_for_test(self._mock_sequence, np.zeros(31))
        state = io.read_state(timeout_s=timeout)
        # Translate into Cadence's owned, backend-neutral state without changing
        # SDK timestamps, sequence numbers, IMU fields, or optional world values.
        return RobotState(**{field.name: getattr(state, field.name) for field in fields(RobotState)})

    def write_command(self, command, state_sequence):
        io = self._running()
        if self.read_only:
            raise RuntimeError("read-only A3 backend does not accept joint commands")
        # The SDK checks dimensions, finite values, delivered-snapshot freshness,
        # sequence, and transport acceptance. Preserve its typed exceptions.
        io.write_command(command, state_sequence)

    def is_state_stale_error(self, error):
        """Let the shared runner classify SDK rejection without importing it."""
        return isinstance(error, getattr(self._sdk, "StateStaleError", ()))

    @property
    def watchdog_trip_count(self):
        return self._running().watchdog_trip_count

    @property
    def aimrt_stats(self):
        return self._running().aimrt_stats

    def close(self):
        io, self._io = self._io, None
        if io is not None:
            io.close()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *_):
        self.close()
