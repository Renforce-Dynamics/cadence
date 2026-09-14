"""Explicit source/frame matching for optional external localization ingress."""

from dataclasses import dataclass

from cadence_config import validate_keys
from cadence_protocol.localization import _integer, _name, _real


@dataclass(frozen=True, slots=True)
class LocalizationIngressConfig:
    host: str = "127.0.0.1"
    port: int = 15110
    source: str = "localization"
    frame_id: str = "world"
    child_frame_id: str = "policy_root"
    max_age_s: float = 0.25
    max_datagrams_per_poll: int = 64

    def __post_init__(self):
        for name in ("host", "source", "frame_id", "child_frame_id"):
            object.__setattr__(self, name, _name(getattr(self, name), name))
        if self.frame_id == self.child_frame_id:
            raise ValueError("localization frame_id and child_frame_id must be distinct")
        object.__setattr__(self, "port", _integer(self.port, "port", 0, 65535))
        object.__setattr__(self, "max_age_s", _real(self.max_age_s, "max_age_s", positive=True))
        object.__setattr__(self, "max_datagrams_per_poll", _integer(
            self.max_datagrams_per_poll, "max_datagrams_per_poll", 1, 1024))

    @classmethod
    def from_mapping(cls, raw):
        if raw is None:
            return None
        validate_keys(raw, set(cls.__dataclass_fields__), label="runtime.localization")
        return cls(**raw)
