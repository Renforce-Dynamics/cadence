"""Bounded, nonblocking ingress; root-loss behavior belongs to RuntimeKernel."""

from __future__ import annotations

from dataclasses import dataclass
import math
import socket
import time

from planet_protocol.localization import (
    LOCALIZATION_SCHEMA, MAX_DATAGRAM_BYTES, LocalizationSample, decode_localization,
)
from cadence.runtime import LocalizationState
from .config import LocalizationIngressConfig


LOCALIZATION_SCHEMAS = (LOCALIZATION_SCHEMA, "cadence.localization.v1")


@dataclass(frozen=True, slots=True)
class ReceivedLocalization:
    sample: LocalizationSample
    received_monotonic_s: float
    max_age_s: float

    def age_s(self, now_monotonic_s=None):
        now = time.monotonic() if now_monotonic_s is None else float(now_monotonic_s)
        if not math.isfinite(now):
            raise ValueError("now_monotonic_s must be finite")
        return self.sample.source_age_s + max(0.0, now - self.received_monotonic_s)

    def fresh(self, now_monotonic_s=None):
        return self.sample.valid and self.age_s(now_monotonic_s) < min(
            self.sample.ttl_s, self.max_age_s)

    @property
    def localization(self):
        """Return a detached runtime value in the configured, already matched frames."""
        return LocalizationState(self.sample.position_w_m, self.sample.orientation_wxyz,
                                 self.sample.linear_velocity_w_mps)


class LocalizationReceiver:
    """Latest accepted sample, with a fixed per-poll datagram budget and O(1) memory.

    An explicitly invalid or expired newer sample withdraws the previous value.
    Malformed, wrong-frame/source and out-of-order packets cannot overwrite it.
    poll() always checks age, including calls with no incoming packets. It never
    holds an expired root; RuntimeKernel owns any hold/fallback state policy.
    """

    def __init__(self, config: LocalizationIngressConfig | None = None):
        self.config = LocalizationIngressConfig() if config is None else config
        if not isinstance(self.config, LocalizationIngressConfig):
            raise TypeError("config must be LocalizationIngressConfig")
        family, _, _, _, endpoint = socket.getaddrinfo(
            self.config.host, self.config.port, type=socket.SOCK_DGRAM,
            flags=socket.AI_PASSIVE,
        )[0]
        self._socket = socket.socket(family, socket.SOCK_DGRAM)
        try:
            self._socket.bind(endpoint)
            self._socket.setblocking(False)
        except BaseException:
            self._socket.close()
            raise
        self._closed = False
        self._latest = None
        self._session_id = None
        self._sequence = None
        self.accepted = 0
        self.rejected_decode = 0
        self.rejected_identity = 0
        self.rejected_sequence = 0

    @property
    def address(self):
        host, port = self._socket.getsockname()[:2]
        return str(host), int(port)

    def poll(self, now_monotonic_s=None) -> ReceivedLocalization | None:
        if self._closed:
            raise ValueError("localization receiver is closed")
        now = time.monotonic() if now_monotonic_s is None else float(now_monotonic_s)
        if not math.isfinite(now):
            raise ValueError("now_monotonic_s must be finite")
        for _ in range(self.config.max_datagrams_per_poll):
            try:
                payload = self._socket.recv(MAX_DATAGRAM_BYTES + 1)
            except BlockingIOError:
                break
            try:
                sample = decode_localization(payload, schema=LOCALIZATION_SCHEMAS)
            except (ValueError, OverflowError):
                self.rejected_decode += 1
                continue
            if (sample.source != self.config.source or sample.frame_id != self.config.frame_id
                    or sample.child_frame_id != self.config.child_frame_id):
                self.rejected_identity += 1
                continue
            if self._session_id is not None and (
                sample.session_id < self._session_id
                or (sample.session_id == self._session_id and sample.sequence <= self._sequence)
            ):
                self.rejected_sequence += 1
                continue
            self._session_id, self._sequence = sample.session_id, sample.sequence
            self._latest = ReceivedLocalization(sample, now, self.config.max_age_s)
            self.accepted += 1
        if self._latest is None or not self._latest.fresh(now):
            return None
        return self._latest

    def close(self):
        self._socket.close()
        self._closed = True

    def __enter__(self):
        if self._closed:
            raise ValueError("localization receiver is closed")
        return self

    def __exit__(self, *args):
        self.close()
