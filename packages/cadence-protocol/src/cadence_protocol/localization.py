"""Legacy localization schema around the standalone Planet implementation."""

from planet_protocol.localization import (
    MAX_DATAGRAM_BYTES, MAX_SAFE_INTEGER, LocalizationSample,
    LocalizationClient as _LocalizationClient, new_localization_session,
    encode_localization as _encode_localization, decode_localization as _decode_localization,
    _integer, _name, _real,
)

LOCALIZATION_SCHEMA = "cadence.localization.v1"


def encode_localization(sample):
    return _encode_localization(sample, schema=LOCALIZATION_SCHEMA)


def decode_localization(payload):
    return _decode_localization(payload, schema=LOCALIZATION_SCHEMA)


class LocalizationClient(_LocalizationClient):
    """Compatibility producer retaining the legacy localization schema."""

    schema = LOCALIZATION_SCHEMA
