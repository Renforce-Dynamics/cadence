"""Stateful command filters shared by real and dry-run frontends."""

from __future__ import annotations

import math

import numpy as np


class VelocitySlewRateLimiter:
    """Limit per-axis joystick velocity changes using elapsed monotonic time."""

    def __init__(
        self,
        linear_rate_mps2: float,
        yaw_rate_radps2: float,
        deadzone: float = 0.0,
    ) -> None:
        rates = np.asarray(
            (linear_rate_mps2, linear_rate_mps2, yaw_rate_radps2),
            dtype=np.float64,
        )
        if not np.all(np.isfinite(rates)) or np.any(rates <= 0.0):
            raise ValueError("velocity slew rates must be finite and positive")
        if not math.isfinite(deadzone) or not 0.0 <= deadzone < 1.0:
            raise ValueError("velocity deadzone must be finite and in [0, 1)")
        self._rates = rates
        self._deadzone = float(deadzone)
        self._value = np.zeros(3, dtype=np.float64)
        self._last_time_s: float | None = None

    @property
    def value(self) -> tuple[float, float, float]:
        filtered = np.where(
            np.abs(self._value) < self._deadzone,
            0.0,
            self._value,
        )
        return tuple(float(value) for value in filtered)

    def reset(self, now_s: float | None = None) -> None:
        if now_s is not None and not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        self._value.fill(0.0)
        self._last_time_s = now_s

    def update(self, target, now_s: float) -> tuple[float, float, float]:
        now = float(now_s)
        desired = np.asarray(target, dtype=np.float64).reshape(-1)
        if not math.isfinite(now):
            raise ValueError("now_s must be finite")
        if desired.shape != (3,) or not np.all(np.isfinite(desired)):
            raise ValueError("velocity target must contain three finite values")
        desired[np.abs(desired) < self._deadzone] = 0.0
        if self._last_time_s is None:
            self._last_time_s = now
            return self.value
        elapsed_s = max(0.0, now - self._last_time_s)
        self._last_time_s = now
        maximum_delta = self._rates * elapsed_s
        self._value += np.clip(desired - self._value, -maximum_delta, maximum_delta)
        return self.value


__all__ = ["VelocitySlewRateLimiter"]
