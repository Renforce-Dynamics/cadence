"""Cadence operator-input wire contract with deployed PLNJ v1/v2/v3 compatibility."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntFlag
import math
import struct
import zlib


MAGIC = b"PLNJ"
VERSION = 3
PACKET_SIZE = 72
AXIS_NAMES = (
    "left_x", "left_y", "right_x", "right_y", "left_trigger", "right_trigger",
)
_BODY = struct.Struct("<4sHHIIIHHHII6fbb8s")
_CRC = struct.Struct("<I")
assert _BODY.size + _CRC.size == PACKET_SIZE

# Read-only compatibility with deployed PLNJ v1/v2 senders.
LEGACY_V1_VERSION = 1
LEGACY_V1_PACKET_SIZE = 44
LEGACY_V2_VERSION = 2
LEGACY_V2_PACKET_SIZE = 52
_LEGACY_V1_BODY = struct.Struct("<4sHHIIIHBB3f4s")
_LEGACY_V2_BODY = struct.Struct("<4sHHIIIHBB3fBBBBI4s")


class JoystickProtocolError(ValueError):
    pass


class JoystickFlags(IntFlag):
    CONNECTED = 1 << 0
    REQUEST_VALID = 1 << 1
    # Source compatibility only; v3 wire flags use REQUEST_VALID.
    MODE_VALID = REQUEST_VALID


@dataclass(frozen=True, slots=True)
class JoystickCommandPacket:
    session_id: int
    packet_seq: int
    source_age_us: int
    ttl_ms: int
    flags: JoystickFlags
    request_id: int
    signal_bits: int = 0
    signal_seq: int = 0
    axes: tuple[float, float, float, float, float, float] = (0.0,) * 6
    dpad_x: int = 0
    dpad_y: int = 0
    wire_version: int = VERSION
    legacy_velocity_command: tuple[float, float, float] = (0.0, 0.0, 0.0)
    legacy_variant_slot: int = 0

    @property
    def requested_mode(self) -> int:
        """Compatibility alias; request IDs are owned by deploy config."""
        return int(self.request_id)

    def axis(self, name: str) -> float:
        try:
            return float(self.axes[AXIS_NAMES.index(str(name))])
        except ValueError as error:
            raise KeyError(f"unknown operator axis {name!r}") from error

    def signal_active(self, signal_id: int) -> bool:
        value = int(signal_id)
        return bool(0 <= value < 32 and self.signal_bits & (1 << value))


def sequence_newer(candidate: int, current: int) -> bool:
    delta = (int(candidate) - int(current)) & 0xFFFFFFFF
    return delta != 0 and delta < 0x80000000


def _crc_valid(raw: bytes, body_size: int) -> bool:
    return _CRC.unpack_from(raw, body_size)[0] == zlib.crc32(raw[:body_size]) & 0xFFFFFFFF


def encode_joystick_command(packet: JoystickCommandPacket) -> bytes:
    axes = tuple(float(value) for value in packet.axes)
    if len(axes) != 6 or not all(math.isfinite(value) and -1.0 <= value <= 1.0 for value in axes):
        raise JoystickProtocolError("axes must contain six finite values in [-1, 1]")
    flags = int(packet.flags)
    if flags & ~0x03:
        raise JoystickProtocolError("reserved v3 flag bits must be zero")
    request_id = int(packet.request_id)
    signal_bits = int(packet.signal_bits)
    if not 0 <= request_id <= 0xFFFF:
        raise JoystickProtocolError("request_id must be in [0, 65535]")
    if not 0 <= signal_bits <= 0xFFFFFFFF:
        raise JoystickProtocolError("signal_bits must be uint32")
    if int(packet.dpad_x) not in {-1, 0, 1} or int(packet.dpad_y) not in {-1, 0, 1}:
        raise JoystickProtocolError("dpad values must be -1, 0, or 1")
    body = _BODY.pack(
        MAGIC, VERSION, PACKET_SIZE,
        int(packet.session_id) & 0xFFFFFFFF,
        int(packet.packet_seq) & 0xFFFFFFFF,
        int(packet.source_age_us) & 0xFFFFFFFF,
        int(packet.ttl_ms) & 0xFFFF,
        flags, request_id, signal_bits,
        int(packet.signal_seq) & 0xFFFFFFFF,
        *axes, int(packet.dpad_x), int(packet.dpad_y), b"\0" * 8,
    )
    return body + _CRC.pack(zlib.crc32(body) & 0xFFFFFFFF)


def _legacy_signal_bits(raw_flags: int) -> int:
    bits = 0
    for legacy_flag, signal_id in ((1 << 2, 0), (1 << 3, 1), (1 << 4, 2), (1 << 5, 3)):
        if raw_flags & legacy_flag:
            bits |= 1 << signal_id
    return bits


def _decode_legacy(raw: bytes) -> JoystickCommandPacket:
    if len(raw) == LEGACY_V1_PACKET_SIZE:
        values = _LEGACY_V1_BODY.unpack_from(raw)
        if values[:3] != (MAGIC, LEGACY_V1_VERSION, LEGACY_V1_PACKET_SIZE):
            raise JoystickProtocolError("bad legacy v1 header")
        if values[7] & ~0x1F or values[12] != b"\0" * 4:
            raise JoystickProtocolError("reserved legacy v1 bits/bytes must be zero")
        if not _crc_valid(raw, _LEGACY_V1_BODY.size):
            raise JoystickProtocolError("crc mismatch")
        velocity = tuple(float(value) for value in values[9:12])
        if not all(math.isfinite(value) for value in velocity):
            raise JoystickProtocolError("legacy velocity contains non-finite values")
        raw_flags = int(values[7])
        flags = JoystickFlags(raw_flags & 0x03)
        return JoystickCommandPacket(
            session_id=values[3], packet_seq=values[4], source_age_us=values[5],
            ttl_ms=values[6], flags=flags, request_id=values[8],
            signal_bits=_legacy_signal_bits(raw_flags), wire_version=1,
            legacy_velocity_command=velocity,
        )
    values = _LEGACY_V2_BODY.unpack_from(raw)
    if values[:3] != (MAGIC, LEGACY_V2_VERSION, LEGACY_V2_PACKET_SIZE):
        raise JoystickProtocolError("bad legacy v2 header")
    if values[7] & ~0x3F or values[17] != b"\0" * 4:
        raise JoystickProtocolError("reserved legacy v2 bits/bytes must be zero")
    if values[12] not in {0, 1} or values[13] not in {0, 1} or values[14] not in {0, 1, 2}:
        raise JoystickProtocolError("invalid legacy v2 selection enum")
    if not _crc_valid(raw, _LEGACY_V2_BODY.size):
        raise JoystickProtocolError("crc mismatch")
    velocity = tuple(float(value) for value in values[9:12])
    if not all(math.isfinite(value) for value in velocity):
        raise JoystickProtocolError("legacy velocity contains non-finite values")
    raw_flags = int(values[7])
    return JoystickCommandPacket(
        session_id=values[3], packet_seq=values[4], source_age_us=values[5],
        ttl_ms=values[6], flags=JoystickFlags(raw_flags & 0x03),
        request_id=values[8], signal_bits=_legacy_signal_bits(raw_flags),
        signal_seq=values[16], wire_version=2,
        legacy_velocity_command=velocity, legacy_variant_slot=values[15],
    )


def decode_joystick_command(data) -> JoystickCommandPacket:
    raw = bytes(data)
    if len(raw) in {LEGACY_V1_PACKET_SIZE, LEGACY_V2_PACKET_SIZE}:
        return _decode_legacy(raw)
    if len(raw) != PACKET_SIZE:
        raise JoystickProtocolError(
            f"expected {LEGACY_V1_PACKET_SIZE}, {LEGACY_V2_PACKET_SIZE}, or {PACKET_SIZE} bytes, got {len(raw)}"
        )
    values = _BODY.unpack_from(raw)
    if values[:3] != (MAGIC, VERSION, PACKET_SIZE):
        raise JoystickProtocolError("bad magic, version, or packet size")
    if values[7] & ~0x03 or values[19] != b"\0" * 8:
        raise JoystickProtocolError("reserved v3 bits/bytes must be zero")
    if not _crc_valid(raw, _BODY.size):
        raise JoystickProtocolError("crc mismatch")
    axes = tuple(float(value) for value in values[11:17])
    if not all(math.isfinite(value) and -1.0 <= value <= 1.0 for value in axes):
        raise JoystickProtocolError("axes contain invalid values")
    if values[17] not in {-1, 0, 1} or values[18] not in {-1, 0, 1}:
        raise JoystickProtocolError("dpad values are invalid")
    return JoystickCommandPacket(
        session_id=values[3], packet_seq=values[4], source_age_us=values[5],
        ttl_ms=values[6], flags=JoystickFlags(values[7]), request_id=values[8],
        signal_bits=values[9], signal_seq=values[10], axes=axes,
        dpad_x=values[17], dpad_y=values[18], wire_version=3,
    )
