"""SONIC whole-body motion-tracking control state (clip and stream modes).

One class serves two registry entries: ``sonic_clip`` plays a converted
motion clip through the released SONIC 035 a3_fast whole-body policy, and
``sonic_stream`` tracks a live ``cadence.motion-ref.v1`` UDP stream. The state
owns its receiver lifecycle; deployment.py is unchanged.

Phases (``substate`` in status):
  clip:   RAMP -> PLAYING -> DONE (hold) or a next_state handoff
  stream: WAITING -> TRACKING -> LOST (blend to stand) -> WAITING

Transaction discipline mirrors ``cadence.motion.states.LowerLocoState``:
``step`` computes one candidate command and mutates the observation history
only after snapshotting it; ``on_command_applied`` commits the candidate and
the clipped raw action; ``on_command_rejected`` restores the snapshot and
retries the same time increment.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from numbers import Integral, Real
from pathlib import Path
import time

import numpy as np
from cadence_api import JointCommand
from cadence.plugins import ControlResult, ControlState

from .contract import (
    DEFAULT_ANGLES_CADENCE,
    JOINT_DIM,
    KD_PD_STAND_CADENCE,
    KP_PD_STAND_CADENCE,
    isaaclab_to_cadence,
)
from .policy import SonicWholeBodyPolicy
from .reference import SonicClip
from .stream import LatestMotionRef, MotionRefUdpReceiver

_MODES = ("clip", "stream")
_ON_FINISH = ("hold", "loco")
_ENTRY_GAINS = ("policy", "pd_stand")


def _vector(value, name, size):
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (size,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain {size} finite values")
    return array.copy()


def _number(value, name, *, minimum, maximum=None, integer=False):
    kind = Integral if integer else Real
    if isinstance(value, bool) or not isinstance(value, kind) or not np.isfinite(value):
        raise ValueError(f"{name} must be a finite {'integer' if integer else 'number'}")
    if value < minimum or (maximum is not None and value > maximum):
        raise ValueError(f"{name} must be in [{minimum}, {maximum}]")
    return int(value) if integer else float(value)


def _smoothstep(progress):
    progress = min(1.0, max(0.0, progress))
    return progress * progress * (3.0 - 2.0 * progress)


@dataclass(frozen=True, slots=True)
class SonicConfig:
    mode: str
    model: Path
    kp: np.ndarray
    kd: np.ndarray
    position_min: np.ndarray
    position_max: np.ndarray
    entry_smoothing_s: float
    runtime: Mapping
    clip_path: Path | None = None
    ramp_s: float = 1.0
    clip_future_frame_skip: int = 1
    on_finish: str = "hold"
    entry_gains: str = "policy"
    stream_host: str = "127.0.0.1"
    stream_port: int = 15120
    delay_ms: float = 220.0
    frame_ms: float = 20.0
    stale_ms: float = 250.0
    blend_s: float = 0.5
    stream_future_frame_skip: int = 1
    max_frames: int = 512

    @classmethod
    def from_mapping(cls, raw):
        if not isinstance(raw, Mapping):
            raise ValueError("sonic state configuration must be a mapping")
        allowed = {"mode", "resources", "control", "robot", "entry_smoothing_s",
                   "policy", "clip", "stream", "entry_gains"}
        unknown = set(raw) - allowed
        if unknown:
            raise ValueError(f"sonic configuration contains unknown fields: {sorted(unknown)}")
        mode = raw.get("mode")
        if mode not in _MODES:
            raise ValueError(f"mode must be one of {_MODES}")
        resources = raw.get("resources")
        expected = {"model", "clip"} if mode == "clip" else {"model"}
        if not isinstance(resources, Mapping) or set(resources) != expected:
            raise ValueError(f"resources must contain exactly {sorted(expected)}")
        resolved = {}
        for name, value in resources.items():
            if (not isinstance(value, (str, Path)) or "://" in str(value)
                    or not Path(value).expanduser().is_absolute()):
                raise ValueError(f"resources.{name} requires a resolved absolute filesystem path")
            resolved[name] = Path(value).expanduser().resolve()
        control = raw.get("control")
        if not isinstance(control, Mapping) or set(control) != {"kp", "kd"}:
            raise ValueError("control must contain exactly kp and kd")
        kp = _vector(control["kp"], "control.kp", JOINT_DIM)
        kd = _vector(control["kd"], "control.kd", JOINT_DIM)
        if np.any(kp < 0) or np.any(kd < 0):
            raise ValueError("control gains must be nonnegative")
        robot = raw.get("robot")
        if not isinstance(robot, Mapping) or set(robot) != {"position_min", "position_max"}:
            raise ValueError("robot must contain exactly position_min and position_max")
        low = _vector(robot["position_min"], "robot.position_min", JOINT_DIM)
        high = _vector(robot["position_max"], "robot.position_max", JOINT_DIM)
        if np.any(low > high):
            raise ValueError("robot.position_min must not exceed position_max")
        default = np.asarray(DEFAULT_ANGLES_CADENCE)
        if np.any(default < low) or np.any(default > high):
            raise ValueError("SONIC default pose must be within the configured limits")
        policy = raw.get("policy", {})
        if not isinstance(policy, Mapping) or set(policy) - {"runtime"}:
            raise ValueError("policy may contain only runtime")
        runtime = policy.get("runtime")
        if runtime is not None and not isinstance(runtime, Mapping):
            raise ValueError("policy.runtime must be a mapping")
        clip = raw.get("clip", {})
        if not isinstance(clip, Mapping) or set(clip) - {"ramp_s", "future_frame_skip", "on_finish"}:
            raise ValueError("clip may contain only ramp_s, future_frame_skip and on_finish")
        stream = raw.get("stream", {})
        if not isinstance(stream, Mapping) or set(stream) - {
            "host", "port", "delay_ms", "frame_ms", "stale_ms", "blend_s",
            "future_frame_skip", "max_frames",
        }:
            raise ValueError("stream contains unknown fields")
        on_finish = clip.get("on_finish", "hold")
        if on_finish not in _ON_FINISH:
            raise ValueError(f"clip.on_finish must be one of {_ON_FINISH}")
        entry_gains = raw.get("entry_gains", "policy")
        if entry_gains not in _ENTRY_GAINS:
            raise ValueError(f"entry_gains must be one of {_ENTRY_GAINS}")
        host = stream.get("host", "127.0.0.1")
        if not isinstance(host, str) or not host.strip():
            raise ValueError("stream.host must be a nonempty address")
        return cls(
            mode=mode,
            model=resolved["model"],
            kp=kp, kd=kd, position_min=low, position_max=high,
            entry_smoothing_s=_number(raw.get("entry_smoothing_s", 0.1),
                                      "entry_smoothing_s", minimum=0.0, maximum=10.0),
            runtime=dict(runtime or {}),
            clip_path=resolved.get("clip"),
            ramp_s=_number(clip.get("ramp_s", 1.0), "clip.ramp_s", minimum=0.0, maximum=60.0),
            clip_future_frame_skip=_number(clip.get("future_frame_skip", 1),
                                           "clip.future_frame_skip", minimum=1, maximum=50, integer=True),
            on_finish=on_finish,
            entry_gains=entry_gains,
            stream_host=host.strip(),
            stream_port=_number(stream.get("port", 15120), "stream.port", minimum=1, maximum=65535, integer=True),
            delay_ms=_number(stream.get("delay_ms", 220.0), "stream.delay_ms", minimum=0.0, maximum=5000.0),
            frame_ms=_number(stream.get("frame_ms", 20.0), "stream.frame_ms", minimum=1.0, maximum=1000.0),
            stale_ms=_number(stream.get("stale_ms", 250.0), "stream.stale_ms", minimum=1.0, maximum=60000.0),
            blend_s=_number(stream.get("blend_s", 0.5), "stream.blend_s", minimum=0.0, maximum=60.0),
            stream_future_frame_skip=_number(stream.get("future_frame_skip", 1),
                                             "stream.future_frame_skip", minimum=1, maximum=50, integer=True),
            max_frames=_number(stream.get("max_frames", 512), "stream.max_frames", minimum=16, maximum=65536, integer=True),
        )


@dataclass(frozen=True, slots=True)
class SonicDiagnostics:
    observation_kind: str
    policy_model: str
    observation: np.ndarray
    raw_action: np.ndarray
    executed_action: np.ndarray
    position_clip_mask: np.ndarray
    position_clip_excess: np.ndarray


@dataclass(frozen=True, slots=True)
class _Candidate:
    """One prepared command and the state progression it commits."""

    phase: str
    q_des: np.ndarray
    delta_s: float
    events: tuple = ()
    next_state: str | None = None
    raw_action_isaaclab: np.ndarray | None = None
    play_tick: int = 0
    elapsed_s: float = 0.0
    yaw_offset: float = 0.0
    tracking_started: bool = False
    blend_from: np.ndarray | None = None
    reset_observation: bool = False
    gains: tuple | None = None


class SonicTrackState(ControlState):
    """Track a SONIC reference (clip resource or UDP stream) with the 035 actor."""

    active_policy = True
    uses_loco_velocity = False

    @classmethod
    def load_config(cls, raw, services):
        return SonicConfig.from_mapping(raw)

    def __init__(self, state_id, key, config, services):
        config = config if isinstance(config, SonicConfig) else self.load_config(config, services)
        super().__init__(state_id, key, config, services)
        dimension = getattr(services, "dimension", JOINT_DIM)
        if dimension != JOINT_DIM:
            raise ValueError("sonic states require the 29-joint A3 robot")
        self.entry_smoothing_s = config.entry_smoothing_s
        self.policy = SonicWholeBodyPolicy(
            config.model, config.runtime, config.position_min, config.position_max,
            services=services)
        self.observations = self.policy.observations
        self.policy_config = {"model": str(config.model)}
        self.kp = config.kp
        self.kd = config.kd
        # Non-policy PD phases (clip RAMP; stream WAITING/LOST) can run the
        # released PD_STAND bring-up gains instead of the soft 024 policy
        # gains, matching SONIC production bring-up (a3_policy_parameters.hpp).
        if config.entry_gains == "pd_stand":
            self._entry_kp = np.asarray(KP_PD_STAND_CADENCE, dtype=np.float64)
            self._entry_kd = np.asarray(KD_PD_STAND_CADENCE, dtype=np.float64)
        else:
            self._entry_kp = self.kp
            self._entry_kd = self.kd
        self.zeros = np.zeros(JOINT_DIM)
        self.default_cadence = np.asarray(DEFAULT_ANGLES_CADENCE, dtype=np.float64)
        self._scale_cadence = isaaclab_to_cadence(self.policy.action_scale_isaaclab)
        self.clip = SonicClip.load(config.clip_path) if config.mode == "clip" else None
        self.motion_ref = None
        self.receiver = None
        if config.mode == "stream":
            self.motion_ref = LatestMotionRef(
                state_id=state_id, state_key=key, max_frames=config.max_frames)
            self.receiver = MotionRefUdpReceiver(
                self.motion_ref, bind=(config.stream_host, config.stream_port))
        self.phase = "INACTIVE"
        self._active = False
        self._pending = None
        self._adapter_snapshot = None
        self._last_step_s = None
        self._retry_delta_s = None
        self._last_raw_isaaclab = np.zeros(JOINT_DIM)
        self._last_q_des = self.default_cadence.copy()
        self.play_tick = 0
        self.elapsed_s = 0.0
        self.yaw_offset = 0.0
        self.tracking_started = False
        self.blend_from = None
        self._ramp_from = None
        self._last_diagnostics = self._diagnostics(
            np.empty(0), np.zeros(JOINT_DIM), np.zeros(JOINT_DIM))

    # -- lifecycle ---------------------------------------------------------

    def on_enter(self, frame, events):
        self.on_command_rejected()
        self.policy.reset()
        self._last_raw_isaaclab = np.zeros(JOINT_DIM)
        self._last_step_s = float(frame.now_s)
        if not np.isfinite(self._last_step_s):
            raise ValueError("sonic entry time must be finite")
        self._retry_delta_s = None
        self.play_tick = 0
        self.elapsed_s = 0.0
        self.tracking_started = False
        self.blend_from = None
        quat = np.asarray(frame.robot_state.quaternion_wxyz, dtype=np.float64)
        measured = np.asarray(frame.robot_state.joint_pos, dtype=np.float64)
        if measured.shape != (JOINT_DIM,) or not np.all(np.isfinite(measured)):
            raise ValueError("measured entry posture must contain 29 finite values")
        self._last_q_des = measured.copy()
        if self.config.mode == "clip":
            self.yaw_offset = self.clip.lock_yaw(quat)
            self._ramp_from = measured.copy()
            self.phase = "RAMP"
        else:
            self.yaw_offset = 0.0
            self._ramp_from = None
            self.motion_ref.activate()
            self.receiver.start()
            self.phase = "WAITING"
        self._active = True
        self._last_diagnostics = self._diagnostics(
            np.empty(0), np.zeros(JOINT_DIM), np.zeros(JOINT_DIM))

    def on_exit(self, frame, events):
        self.on_command_rejected()
        if self.receiver is not None and self.receiver.address is not None:
            self.receiver.stop()
        if self.motion_ref is not None:
            self.motion_ref.deactivate()
        self._active = False
        self.phase = "INACTIVE"

    # -- phase computers -----------------------------------------------------

    def _pd(self, q_des, phase, delta_s, **fields):
        target = np.clip(np.asarray(q_des, dtype=np.float64),
                         self.config.position_min, self.config.position_max)
        fields.setdefault("play_tick", self.play_tick)
        fields.setdefault("elapsed_s", self.elapsed_s)
        fields.setdefault("yaw_offset", self.yaw_offset)
        fields.setdefault("tracking_started", self.tracking_started)
        fields.setdefault("blend_from", self.blend_from)
        fields.setdefault("gains", (self._entry_kp, self._entry_kd))
        return _Candidate(phase, target, delta_s, **fields), None

    def _policy_step(self, frame, prefix, phase, delta_s, **fields):
        q_des, requested, raw_clipped, observation = self.policy.infer(
            frame.robot_state, self._last_raw_isaaclab, prefix)
        fields.setdefault("play_tick", self.play_tick)
        fields.setdefault("elapsed_s", self.elapsed_s)
        fields.setdefault("yaw_offset", self.yaw_offset)
        fields.setdefault("tracking_started", self.tracking_started)
        fields.setdefault("blend_from", self.blend_from)
        candidate = _Candidate(phase, q_des, delta_s, raw_action_isaaclab=raw_clipped, **fields)
        return candidate, (observation, requested)

    def _clip_step(self, frame, delta_s):
        pelvis = np.asarray(frame.robot_state.quaternion_wxyz, dtype=np.float64)
        if self.phase == "RAMP":
            elapsed = self.elapsed_s + delta_s
            alpha = _smoothstep(1.0 if self.config.ramp_s <= 0 else elapsed / self.config.ramp_s)
            first = isaaclab_to_cadence(self.clip.q_ref[0])
            q_des = (1.0 - alpha) * self._ramp_from + alpha * first
            if alpha >= 1.0:
                # The ramp's final command lands on the first reference frame;
                # the policy takes over next tick with a fresh history.
                return self._pd(q_des, "PLAYING", delta_s, elapsed_s=elapsed,
                                play_tick=0, events=("sonic_clip_started",),
                                reset_observation=True)
            return self._pd(q_des, "RAMP", delta_s, elapsed_s=elapsed)
        if self.phase == "PLAYING":
            finishing = self.play_tick >= self.clip.frames - 1
            prefix = self.clip.tokenizer_slice(
                self.play_tick, pelvis, self.yaw_offset, self.config.clip_future_frame_skip)
            if not finishing:
                return self._policy_step(frame, prefix, "PLAYING", delta_s,
                                         play_tick=self.play_tick + 1)
            # The final reference frame commits DONE even when a next_state
            # handoff is offered, so the finish event fires exactly once.
            next_state = "loco" if self.config.on_finish == "loco" else None
            return self._policy_step(frame, prefix, "DONE", delta_s,
                                     events=("sonic_clip_finished",),
                                     next_state=next_state)
        if self.phase == "DONE":
            prefix = self.clip.tokenizer_slice(
                self.clip.frames - 1, pelvis, self.yaw_offset,
                self.config.clip_future_frame_skip)
            return self._policy_step(frame, prefix, "DONE", delta_s)
        raise RuntimeError(f"invalid sonic clip phase {self.phase}")

    def _stream_step(self, frame, delta_s):
        config = self.config
        now_mono = time.monotonic()
        pelvis = np.asarray(frame.robot_state.quaternion_wxyz, dtype=np.float64)
        delay_s = config.delay_ms / 1000.0
        frame_dt_s = config.frame_ms / 1000.0
        stale_s = config.stale_ms / 1000.0
        if self.phase == "WAITING":
            if self.motion_ref.sample_reference(now_mono, delay_s) is not None:
                yaw = self.motion_ref.latest_yaw_offset(pelvis)
                return self._pd(self.default_cadence, "TRACKING", delta_s,
                                yaw_offset=yaw, reset_observation=True,
                                events=("sonic_stream_live",))
            return self._pd(self.default_cadence, "WAITING", delta_s)
        if self.phase == "TRACKING":
            if self.motion_ref.is_stale(now_mono, stale_s):
                self.motion_ref.reset_to_latest()
                return self._pd(self._last_q_des, "LOST", delta_s, elapsed_s=0.0,
                                blend_from=self._last_q_des.copy(),
                                tracking_started=False,
                                events=("sonic_stream_lost",))
            prefix = self.motion_ref.tokenizer_slice(
                now_mono, delay_s, pelvis, self.yaw_offset, frame_dt_s,
                config.stream_future_frame_skip,
                require_full_window=not self.tracking_started)
            if prefix is None:
                # Delay line momentarily unfilled; hold the default pose.
                return self._pd(self.default_cadence, "TRACKING", delta_s)
            return self._policy_step(frame, prefix, "TRACKING", delta_s,
                                     tracking_started=True)
        if self.phase == "LOST":
            elapsed = self.elapsed_s + delta_s
            alpha = _smoothstep(1.0 if config.blend_s <= 0 else elapsed / config.blend_s)
            q_des = (1.0 - alpha) * self.blend_from + alpha * self.default_cadence
            if alpha >= 1.0:
                return self._pd(self.default_cadence, "WAITING", delta_s,
                                elapsed_s=elapsed, blend_from=None)
            return self._pd(q_des, "LOST", delta_s, elapsed_s=elapsed)
        raise RuntimeError(f"invalid sonic stream phase {self.phase}")

    # -- transaction boundary ------------------------------------------------

    def step(self, frame):
        if not self._active:
            raise RuntimeError("sonic state must be entered before stepping")
        if self._pending is not None:
            raise RuntimeError("previous sonic command needs commit or reject")
        now = float(frame.now_s)
        if not np.isfinite(now) or self._last_step_s is None or now < self._last_step_s:
            raise ValueError("sonic step time must be finite and nondecreasing")
        delta = now - self._last_step_s if self._retry_delta_s is None else self._retry_delta_s
        self._last_step_s = now
        self._adapter_snapshot = self.policy.snapshot()
        try:
            if self.config.mode == "clip":
                candidate, inference = self._clip_step(frame, delta)
            else:
                candidate, inference = self._stream_step(frame, delta)
            if inference is None:
                diagnostics = self._diagnostics(
                    np.empty(0), np.zeros(JOINT_DIM), np.zeros(JOINT_DIM))
            else:
                observation, requested = inference
                raw_action = (requested - self.default_cadence) / self._scale_cadence
                if not np.all(np.isfinite(raw_action)):
                    raise ValueError("sonic normalized action overflow")
                diagnostics = self._diagnostics(
                    observation, raw_action, np.abs(requested - candidate.q_des))
        except Exception:
            self.policy.restore(self._adapter_snapshot)
            self._adapter_snapshot = None
            raise
        self._pending = candidate
        self._last_diagnostics = diagnostics
        kp, kd = candidate.gains if candidate.gains is not None else (self.kp, self.kd)
        return ControlResult(
            JointCommand(candidate.q_des, self.zeros, kp, kd, self.zeros),
            self._last_diagnostics,
            substate=candidate.phase,
            events=candidate.events,
            next_state=candidate.next_state,
        )

    def on_command_applied(self, command):
        candidate = self._pending
        if candidate is None:
            raise RuntimeError("no pending sonic command")
        actual = np.asarray(command.q_des, dtype=np.float64)
        if actual.shape != (JOINT_DIM,) or not np.all(np.isfinite(actual)):
            raise ValueError("applied command must match the 29-joint dimension")
        executed = (actual - self.default_cadence) / self._scale_cadence
        if not np.all(np.isfinite(executed)):
            raise ValueError("applied normalized action must be finite")
        if candidate.reset_observation:
            self.policy.reset()
            self._last_raw_isaaclab = np.zeros(JOINT_DIM)
        self.phase = candidate.phase
        self.play_tick = candidate.play_tick
        self.elapsed_s = candidate.elapsed_s
        self.yaw_offset = candidate.yaw_offset
        self.tracking_started = candidate.tracking_started
        self.blend_from = None if candidate.blend_from is None else candidate.blend_from.copy()
        if candidate.raw_action_isaaclab is not None:
            self._last_raw_isaaclab = candidate.raw_action_isaaclab.copy()
        self._last_q_des = actual.copy()
        self._pending = None
        self._adapter_snapshot = None
        self._retry_delta_s = None
        self._last_diagnostics = self._diagnostics(
            self._last_diagnostics.observation,
            self._last_diagnostics.raw_action,
            self._last_diagnostics.position_clip_excess,
            executed=executed,
        )

    def on_command_rejected(self):
        if self._pending is not None:
            if self._adapter_snapshot is not None:
                self.policy.restore(self._adapter_snapshot)
            self._retry_delta_s = self._pending.delta_s
            self._pending = None
            self._adapter_snapshot = None

    def diagnostics_after_command(self, diagnostics):
        return self._last_diagnostics

    def _diagnostics(self, observation, raw_action, excess, executed=None):
        return SonicDiagnostics(
            self.policy.observation_kind,
            self.policy.policy_model,
            np.asarray(observation, dtype=np.float32).copy(),
            np.asarray(raw_action, dtype=np.float64).copy(),
            np.zeros(JOINT_DIM) if executed is None else np.asarray(executed, dtype=np.float64).copy(),
            np.asarray(excess, dtype=np.float64) > 1e-9,
            np.asarray(excess, dtype=np.float64).copy(),
        )
