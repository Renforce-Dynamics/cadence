"""Validated, robot-independent configuration for lower-body motion states."""

from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
from numbers import Integral

import numpy as np

from cadence_config import validate_keys


def _mapping(value, name, keys, required=None):
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    validate_keys(value, keys, required=keys if required is None else required)
    return value


def _vector(value, name, dimension=None, *, scalar=False):
    try:
        result = np.asarray(value, dtype=np.float64)
        if scalar and result.ndim == 0:
            result = np.full(dimension, float(result))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain finite numbers") from exc
    if result.ndim != 1 or not result.size or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a nonempty finite vector")
    if dimension is not None and result.shape != (dimension,):
        raise ValueError(f"{name} must contain {dimension} values")
    result = result.copy()
    result.setflags(write=False)
    return result


def _indices(value, name, dimension):
    if not isinstance(value, (list, tuple, np.ndarray)):
        raise ValueError(f"{name} must be a list of joint indices")
    values = list(value)
    if not values or any(
        isinstance(v, (bool, np.bool_)) or not isinstance(v, Integral)
        for v in values
    ):
        raise ValueError(f"{name} must contain integer joint indices")
    if len(set(values)) != len(values) or any(v < 0 or v >= dimension for v in values):
        raise ValueError(f"{name} contains duplicate or out-of-range joint indices")
    result = np.asarray(values, dtype=np.int64)
    result.setflags(write=False)
    return result


@dataclass(frozen=True, slots=True)
class MotionRobotConfig:
    default_position: np.ndarray
    action_scale: np.ndarray
    position_min: np.ndarray
    position_max: np.ndarray
    lower_joints: np.ndarray
    upper_joints: np.ndarray

    @property
    def dimension(self):
        return len(self.default_position)


@dataclass(frozen=True, slots=True)
class MotionControlConfig:
    kp: np.ndarray
    kd: np.ndarray


@dataclass(frozen=True, slots=True)
class LowerPolicyConfig:
    factory: str
    model: str
    history_frames: int
    runtime: Mapping
    mask_upper_observation: bool = True


@dataclass(frozen=True, slots=True)
class UpperPositionConfig:
    default_position: np.ndarray


@dataclass(frozen=True, slots=True)
class MotionConfig:
    robot: MotionRobotConfig
    control: MotionControlConfig
    lower: LowerPolicyConfig
    upper: UpperPositionConfig
    entry_smoothing_s: float = 0.0

    @property
    def history_frames(self):
        return self.lower.history_frames

    @property
    def policy(self):
        return self.lower

    @classmethod
    def from_mapping(cls, raw, dimension=None):
        raw = _mapping(
            raw,
            "motion",
            {"robot", "control", "lower", "upper", "entry_smoothing_s"},
            {"robot", "control", "lower", "upper"},
        )
        robot = _mapping(
            raw["robot"], "robot",
            {"default_position", "action_scale", "position_min", "position_max",
             "lower_joints", "upper_joints"},
        )
        default = _vector(robot["default_position"], "robot.default_position", dimension)
        n = len(default)
        scale = _vector(robot["action_scale"], "robot.action_scale", n)
        low = _vector(robot["position_min"], "robot.position_min", n)
        high = _vector(robot["position_max"], "robot.position_max", n)
        if np.any(scale <= 0):
            raise ValueError("robot.action_scale must be positive")
        if np.any(low > high):
            raise ValueError("robot.position_min must not exceed position_max")
        if np.any(default < low) or np.any(default > high):
            raise ValueError("robot.default_position must be within joint limits")
        lower_joints = _indices(robot["lower_joints"], "robot.lower_joints", n)
        upper_joints = _indices(robot["upper_joints"], "robot.upper_joints", n)
        if set(lower_joints) & set(upper_joints):
            raise ValueError("lower_joints and upper_joints must not overlap")
        if set(lower_joints) | set(upper_joints) != set(range(n)):
            raise ValueError("lower_joints and upper_joints must cover every joint")
        control = _mapping(raw["control"], "control", {"kp", "kd"})
        kp = _vector(control["kp"], "control.kp", n, scalar=True)
        kd = _vector(control["kd"], "control.kd", n, scalar=True)
        if np.any(kp < 0) or np.any(kd < 0):
            raise ValueError("control gains must be nonnegative")
        lower = _mapping(
            raw["lower"], "lower",
            {"factory", "model", "history_frames", "runtime", "mask_upper_observation"},
            {"factory", "model", "history_frames", "runtime"},
        )
        factory = lower["factory"]
        if not isinstance(factory, str) or factory.count(":") != 1 or not all(factory.split(":")):
            raise ValueError("lower.factory must be module:Class")
        model = lower["model"]
        if not isinstance(model, str) or not model.strip():
            raise ValueError("lower.model must be a nonempty resource path")
        history = lower["history_frames"]
        if isinstance(history, bool) or not isinstance(history, Integral) or history <= 0:
            raise ValueError("lower.history_frames must be a positive integer")
        runtime = lower["runtime"]
        if not isinstance(runtime, Mapping):
            raise ValueError("lower.runtime must be a mapping")
        mask = lower.get("mask_upper_observation", True)
        if not isinstance(mask, bool):
            raise ValueError("lower.mask_upper_observation must be a boolean")
        upper = _mapping(raw["upper"], "upper", {"default_position"})
        upper_default = _vector(
            upper["default_position"], "upper.default_position", len(upper_joints)
        )
        if np.any(upper_default < low[upper_joints]) or np.any(upper_default > high[upper_joints]):
            raise ValueError("upper.default_position must be within upper joint limits")
        try:
            smoothing = float(raw.get("entry_smoothing_s", 0.0))
        except (TypeError, ValueError) as exc:
            raise ValueError("entry_smoothing_s must be finite and nonnegative") from exc
        if not np.isfinite(smoothing) or smoothing < 0:
            raise ValueError("entry_smoothing_s must be finite and nonnegative")
        return cls(
            MotionRobotConfig(default, scale, low, high, lower_joints, upper_joints),
            MotionControlConfig(kp, kd),
            LowerPolicyConfig(factory, model, int(history), deepcopy(dict(runtime)), mask),
            UpperPositionConfig(upper_default),
            smoothing,
        )
