"""Configuration for the optional operator endpoint of a generic deployment."""

from dataclasses import dataclass
from numbers import Integral

from cadence_config import validate_keys
from cadence.control.filters import VelocitySlewRateLimiter
from .input import OperatorInputMapping, OperatorInputAdapter


@dataclass(frozen=True)
class OperatorIngressConfig:
    host: str
    port: int
    mapping: OperatorInputMapping
    signal_mode: str
    linear_slew_rate_mps2: float
    yaw_slew_rate_radps2: float
    velocity_deadzone: float

    @classmethod
    def from_mapping(cls, raw):
        if raw is None:
            return None
        fields = {"host", "port", "mapping", "signal_mode", "linear_slew_rate_mps2",
                  "yaw_slew_rate_radps2", "velocity_deadzone"}
        validate_keys(raw, fields, required=fields, label="runtime.operator")
        host, port = raw["host"], raw["port"]
        if not isinstance(host, str) or not host.strip():
            raise ValueError("runtime.operator.host must be a nonempty host")
        if isinstance(port, bool) or not isinstance(port, Integral) or not 0 <= port <= 65535:
            raise ValueError("runtime.operator.port must be an integer in [0, 65535]")
        mapping = OperatorInputMapping.from_mapping(raw["mapping"])
        OperatorInputAdapter(mapping, raw["signal_mode"])
        rates = (float(raw["linear_slew_rate_mps2"]), float(raw["yaw_slew_rate_radps2"]),
                 float(raw["velocity_deadzone"]))
        VelocitySlewRateLimiter(*rates)
        return cls(host, int(port), mapping, raw["signal_mode"], *rates)
