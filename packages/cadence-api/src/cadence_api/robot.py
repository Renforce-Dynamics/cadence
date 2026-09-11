"""Backend-neutral proprioception and actuation boundary."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np

from .commands import JointCommand


@dataclass(frozen=True, slots=True)
class RobotState:
    """One owned policy-view snapshot.

    Joint arrays always use the explicit backend joint order. World/root fields
    are available in simulation and intentionally optional on hardware.
    """

    sequence: int
    timestamp_ns: int
    gyro_b: np.ndarray
    quaternion_wxyz: np.ndarray
    joint_pos: np.ndarray
    joint_vel: np.ndarray
    joint_tau_est: np.ndarray
    sync_complete: bool = True
    sync_skew_ns: int = 0
    root_position_w: np.ndarray | None = None
    root_linear_velocity_w: np.ndarray | None = None
    has_torso_imu: bool = False
    torso_gyro_b: np.ndarray | None = None
    torso_quaternion_wxyz: np.ndarray | None = None
    torso_acceleration_b: np.ndarray | None = None

    def __post_init__(self) -> None:
        if np.asarray(self.joint_pos).size == 0:
            raise ValueError("joint layout must be nonempty")
        if self.sequence <= 0:
            raise ValueError("sequence must be positive")
        dimensions = {
            "gyro_b": 3,
            "quaternion_wxyz": 4,
            "joint_pos": len(np.asarray(self.joint_pos).reshape(-1)),
            "joint_vel": len(np.asarray(self.joint_pos).reshape(-1)),
            "joint_tau_est": len(np.asarray(self.joint_pos).reshape(-1)),
        }
        for name, dimension in dimensions.items():
            value = np.asarray(getattr(self, name), dtype=np.float64).reshape(-1)
            if value.shape != (dimension,) or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must contain {dimension} finite values")
            object.__setattr__(self, name, value.copy())
        norm = np.linalg.norm(self.quaternion_wxyz)
        if norm < 1e-6:
            raise ValueError("quaternion_wxyz must be non-zero")
        object.__setattr__(self, "quaternion_wxyz", self.quaternion_wxyz / norm)
        torso_defaults = {
            "torso_gyro_b": self.gyro_b,
            "torso_quaternion_wxyz": self.quaternion_wxyz,
            "torso_acceleration_b": np.asarray((0.0, 0.0, 9.81), dtype=np.float64),
        }
        for name, default in torso_defaults.items():
            value = getattr(self, name)
            if value is None:
                value = default
            array = np.asarray(value, dtype=np.float64).reshape(-1)
            dimension = 4 if name == "torso_quaternion_wxyz" else 3
            if array.shape != (dimension,) or not np.all(np.isfinite(array)):
                raise ValueError(f"{name} must contain {dimension} finite values")
            if name == "torso_quaternion_wxyz":
                torso_norm = np.linalg.norm(array)
                if torso_norm < 1e-6:
                    raise ValueError("torso_quaternion_wxyz must be non-zero")
                array = array / torso_norm
            object.__setattr__(self, name, array.copy())
        object.__setattr__(self, "has_torso_imu", bool(self.has_torso_imu))
        for name in ("root_position_w", "root_linear_velocity_w"):
            value = getattr(self, name)
            if value is None:
                continue
            array = np.asarray(value, dtype=np.float64).reshape(-1)
            if array.shape != (3,) or not np.all(np.isfinite(array)):
                raise ValueError(f"{name} must contain 3 finite values")
            object.__setattr__(self, name, array.copy())


@runtime_checkable
class RobotIO(Protocol):
    """The independent policy runtime's only robot-facing dependency."""

    def start(self) -> None: ...

    def read_state(self, timeout_s: float | None = None) -> RobotState: ...

    def write_command(self, command: JointCommand, state_sequence: int) -> None: ...

    def close(self) -> None: ...
