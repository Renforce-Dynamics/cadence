"""Transport-independent operator interpretation for Cadence runtimes."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any, Mapping
from planet_protocol.operator import AXIS_NAMES, JoystickCommandPacket, JoystickFlags


@dataclass(frozen=True, slots=True)
class OperatorInputMapping:
    """Robot/deployment interpretation of generic PLNJ controls."""

    velocity_axes: tuple[str, str, str]
    velocity_scales: tuple[float, float, float]
    emergency_signal_id: int
    reset_signal_id: int

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "OperatorInputMapping":
        if not isinstance(raw, Mapping):
            raise ValueError("joystick.operator must be a mapping")
        allowed = {
            "velocity_axes", "velocity_scales",
            "emergency_signal_id", "reset_signal_id",
        }
        unknown = set(raw) - allowed
        missing = allowed - set(raw)
        if unknown or missing:
            raise ValueError(
                f"invalid joystick.operator keys: missing={sorted(missing)} "
                f"unknown={sorted(unknown)}"
            )
        axes = tuple(str(value) for value in raw["velocity_axes"])
        scales = tuple(float(value) for value in raw["velocity_scales"])
        if len(axes) != 3 or any(name not in AXIS_NAMES for name in axes):
            raise ValueError("velocity_axes must contain three PLNJ axis names")
        if len(scales) != 3 or not all(math.isfinite(value) for value in scales):
            raise ValueError("velocity_scales must contain three finite values")
        if isinstance(raw["emergency_signal_id"], bool) or isinstance(
            raw["reset_signal_id"], bool
        ):
            raise ValueError("emergency/reset signal IDs must be integers")
        emergency = int(raw["emergency_signal_id"])
        reset = int(raw["reset_signal_id"])
        if not 0 <= emergency < 32 or not 0 <= reset < 32 or emergency == reset:
            raise ValueError("emergency/reset signal IDs must be distinct values in [0, 31]")
        return cls(axes, scales, emergency, reset)


@dataclass(frozen=True, slots=True)
class ReceivedJoystickCommand:
    packet: JoystickCommandPacket
    received_monotonic_s: float

    def age_s(self, now_monotonic_s: float | None = None) -> float:
        now = time.monotonic() if now_monotonic_s is None else float(now_monotonic_s)
        return max(0.0, now - self.received_monotonic_s)

    def usable(self, now_monotonic_s: float | None = None) -> bool:
        return bool(
            self.packet.flags & JoystickFlags.CONNECTED
            and self.age_s(now_monotonic_s) * 1000.0
            + self.packet.source_age_us * 0.001 <= self.packet.ttl_ms
        )

    def command(self, now_monotonic_s: float | None = None):
        return self.packet if self.usable(now_monotonic_s) else None

    def velocity(
        self, mapping: OperatorInputMapping,
        now_monotonic_s: float | None = None,
    ) -> tuple[float, float, float]:
        if not self.usable(now_monotonic_s):
            return (0.0, 0.0, 0.0)
        if self.packet.wire_version < 3:
            return self.packet.legacy_velocity_command
        return tuple(
            self.packet.axis(axis) * scale
            for axis, scale in zip(mapping.velocity_axes, mapping.velocity_scales)
        )

    def requested_state(self, now_monotonic_s: float | None = None):
        if not self.usable(now_monotonic_s) or not self.packet.flags & JoystickFlags.REQUEST_VALID:
            return None
        # Keep the wire ID intact; the selected catalog owns ID-to-state binding.
        return int(self.packet.request_id)

    def signal(self, signal_id: int, now_monotonic_s: float | None = None) -> bool:
        return bool(
            self.usable(now_monotonic_s)
            and self.packet.signal_active(signal_id)
        )


@dataclass(frozen=True, slots=True)
class OperatorInputSnapshot:
    """One immutable sample, using the names expected by RuntimeInput."""

    requested_state: int | None
    emergency_halt: bool
    reset_safety: bool
    velocity_command: tuple[float, float, float]
    operator_input: JoystickCommandPacket | None
    operator_link_usable: bool


class OperatorInputAdapter:
    """Map packet validity, axes and safety signals once per control period.

    ``level`` repeats a held safety signal; ``rising`` emits it only on a
    false-to-true edge. Stale/disconnected input clears the edge detector and
    produces zero velocity, no request and no task-specific operator payload.
    Slew limiting remains a separate controller concern.
    """

    def __init__(self, mapping: OperatorInputMapping, signal_mode: str = "level"):
        if signal_mode not in {"level", "rising"}:
            raise ValueError("signal_mode must be level or rising")
        if not isinstance(mapping, OperatorInputMapping):
            raise TypeError("mapping must be OperatorInputMapping")
        self.mapping = mapping
        self.signal_mode = signal_mode
        self._previous_emergency = False
        self._previous_reset = False

    def map(self, received: ReceivedJoystickCommand | None, now: float) -> OperatorInputSnapshot:
        if not math.isfinite(now):
            raise ValueError("operator input time must be finite")
        usable = bool(received is not None and received.usable(now))
        emergency = bool(usable and received.signal(self.mapping.emergency_signal_id, now))
        reset = bool(usable and received.signal(self.mapping.reset_signal_id, now))
        if self.signal_mode == "rising":
            emitted_emergency = emergency and not self._previous_emergency
            emitted_reset = reset and not self._previous_reset
        else:
            emitted_emergency, emitted_reset = emergency, reset
        self._previous_emergency, self._previous_reset = emergency, reset
        return OperatorInputSnapshot(
            received.requested_state(now) if usable else None,
            emitted_emergency,
            emitted_reset,
            received.velocity(self.mapping, now) if usable else (0.0, 0.0, 0.0),
            received.command(now) if usable else None,
            usable,
        )
