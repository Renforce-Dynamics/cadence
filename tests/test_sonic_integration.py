"""In-memory RuntimeKernel lifecycle tests for the SONIC states.

Mirrors tests/test_integration.py's ideal command-following feedback pattern
but drives ``requested_state`` directly, so no operator UDP timing is involved.
"""

import json
import socket
import time
from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from cadence.deployment import load_run_config, prepare_deployment
from cadence.plugins import ControlFrame
from cadence.runtime import RuntimeConfig, RuntimeInput, RuntimeKernel
from cadence_api import RobotState

from cadence.sonic.contract import (
    DEFAULT_ANGLES_CADENCE, ISAACLAB_JOINT_NAMES,
    KD_024_CADENCE, KD_PD_STAND_CADENCE, KP_024_CADENCE, KP_PD_STAND_CADENCE,
)

ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / "configs/entry/a3/mock/entry_a3_operator_sonic.yaml"


@pytest.fixture(scope="module")
def plan():
    prepared = prepare_deployment(load_run_config(ENTRY))
    yield prepared
    prepared.backend.close()


@dataclass
class Rig:
    kernel: RuntimeKernel
    robot: RobotState
    tick: int = 0

    @classmethod
    def start(cls, plan):
        cfg = plan.resolved.data
        services = SimpleNamespace(dimension=plan.dimension,
                                   joint_names=tuple(cfg["robot"]["joints"]))
        zero = np.zeros(plan.dimension)
        robot = RobotState(1, 0, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]),
                           zero, zero, zero)
        kernel = RuntimeKernel(
            RuntimeConfig(plan.catalog, services, ControlFrame,
                          cfg["robot"]["position_min"], cfg["robot"]["position_max"],
                          "damping", plan.deadline_s), robot, 0.0)
        return cls(kernel, robot)

    def cycle(self, request=None, dpad=(0, 0)):
        self.tick += 1
        now = self.tick / 50.0
        operator = SimpleNamespace(dpad_x=dpad[0], dpad_y=dpad[1])
        output = self.kernel.tick(RuntimeInput(now, self.robot, None, requested_state=request,
                                               operator_input=operator))
        assert not output.safety_halted, output.safety_reason
        self.robot = replace(self.robot, sequence=self.tick + 1,
                             timestamp_ns=round(now * 1e9),
                             joint_pos=output.command.q_des.copy())
        return output

    def close(self):
        self.kernel.reject()
        self.kernel.current.on_exit(ControlFrame(self.tick / 50.0, self.robot), [])


def _enter_via_fixedpos(rig, request_id):
    rig.cycle(2)
    for _ in range(100):
        rig.cycle(2)
        if rig.kernel.entry_gate_ready:
            break
    assert rig.kernel.entry_gate_ready
    output = rig.cycle(request_id)
    return output


def _await_substate(rig, substate, max_ticks=200, dpad=(0, 0)):
    events = []
    for _ in range(max_ticks):
        output = rig.cycle(dpad=dpad)
        events.extend(output.events)
        if output.skill_state == substate:
            return output, events
    raise AssertionError(f"substate {substate} not reached; last {output.skill_state}")


def _assert_gains(command, kp, kd):
    np.testing.assert_array_equal(command.kp, kp)
    np.testing.assert_array_equal(command.kd, kd)


def test_sonic_clip_lifecycle(plan):
    rig = Rig.start(plan)
    events = []
    try:
        output = _enter_via_fixedpos(rig, 6)
        assert output.mode == "SONIC_CLIP" and output.skill_state == "RAMP"
        _assert_gains(output.command, KP_PD_STAND_CADENCE, KD_PD_STAND_CADENCE)
        # Entry blends onto the stand pose and waits in READY.
        output, stage_events = _await_substate(rig, "READY")
        events.extend(stage_events)
        assert events.count("sonic_clip_ready") == 1
        state = rig.kernel.plugin(6)
        assert state.selection_name == "stand"
        assert state.selected_motion == "stand"
        default = np.asarray(DEFAULT_ANGLES_CADENCE)
        np.testing.assert_allclose(output.command.q_des, default, atol=1e-9)
        _assert_gains(output.command, KP_PD_STAND_CADENCE, KD_PD_STAND_CADENCE)

        # A held D-pad does not re-trigger; only a fresh edge selects/plays.
        output = rig.cycle(dpad=(-1, 0))  # left: return to READY after playback
        events.extend(output.events)
        assert output.skill_state == "CUE"
        assert "sonic_clip_cue:stand" in output.events
        _assert_gains(output.command, KP_PD_STAND_CADENCE, KD_PD_STAND_CADENCE)
        # CUE blends onto the clip's first frame, then the policy takes over.
        output, stage_events = _await_substate(rig, "PLAYING")
        events.extend(stage_events)
        assert events.count("sonic_clip_started") == 1
        assert state.play_tick == 0
        # The last CUE command reaches PLAYING but is still a PD command.
        _assert_gains(output.command, KP_PD_STAND_CADENCE, KD_PD_STAND_CADENCE)
        for _ in range(3):
            output = rig.cycle(dpad=(-1, 0))  # still held: no re-trigger
        assert state.play_tick == 3
        # PLAYING advances exactly one reference tick per applied command.
        for expected in range(4, 8):
            output = rig.cycle()
            assert state.play_tick == expected
            assert output.observation_kind == "sonic_whole_body_h10"
            assert output.observation.shape == (1570,)
            assert output.command.q_des.shape == (29,)
            robot_cfg = plan.resolved.data["robot"]
            assert np.all(output.command.q_des >= robot_cfg["position_min"])
            assert np.all(output.command.q_des <= robot_cfg["position_max"])
            np.testing.assert_allclose(output.command.dq_des, 0.0)
            np.testing.assert_allclose(output.command.tau_ff, 0.0)
            _assert_gains(output.command, KP_024_CADENCE, KD_024_CADENCE)
        # Jump near the clip end to reach RETURN without replaying 14.5 s.
        state.play_tick = state.clip.frames - 2
        output, stage_events = _await_substate(rig, "READY")
        events.extend(stage_events)
        assert events.count("sonic_clip_finished") == 1
        assert events.count("sonic_clip_ready") == 2
        assert state.play_tick == 0
        np.testing.assert_allclose(output.command.q_des, default, atol=1e-9)
        _assert_gains(output.command, KP_PD_STAND_CADENCE, KD_PD_STAND_CADENCE)

        # D-pad up/down cycles clips with wrap-around; selection is pure
        # bookkeeping (the standby pose is always the stand pose).
        output = rig.cycle(dpad=(0, -1))  # stand -> walk
        events.extend(output.events)
        assert "sonic_clip_selected:walk" in output.events
        assert output.skill_state == "READY"
        assert state.selection_name == "walk"
        assert state.selected_motion == "walk"
        output = rig.cycle(dpad=(0, -1))  # held: no edge, still walk
        assert not output.events
        output = rig.cycle()              # release
        output = rig.cycle(dpad=(0, -1))  # walk -> stand (wrap)
        assert "sonic_clip_selected:stand" in output.events
        assert state.selection_name == "stand"
    finally:
        rig.close()


def test_sonic_clip_right_play_hands_off_to_loco_only_after_accepted_finish(plan):
    rig = Rig.start(plan)
    try:
        _enter_via_fixedpos(rig, 6)
        _await_substate(rig, "READY")
        state = rig.kernel.plugin(6)
        output = rig.cycle(dpad=(1, 0))  # D-pad right: play, then loco
        assert output.skill_state == "CUE"
        _await_substate(rig, "PLAYING")
        state.play_tick = state.clip.frames - 1
        # Rejecting the last command must not advance the phase or hand off.
        now = (rig.tick + 1) / 50.0
        candidate = rig.kernel.prepare(RuntimeInput(now, rig.robot, None, requested_state=6))
        np.testing.assert_array_equal(candidate.command.kp, state.kp)
        rig.kernel.reject()
        assert rig.kernel.current_key == "sonic_clip"
        assert state.phase == "PLAYING"
        assert state.play_tick == state.clip.frames - 1

        # The accepted finishing command is still a policy command. No PD
        # RETURN/fixedpos command is inserted before locomotion takes over.
        output = rig.cycle(6, dpad=(1, 0))
        assert "sonic_clip_finished" in output.events
        np.testing.assert_array_equal(output.command.kp, state.kp)
        assert output.mode == "LOCO"
        for _ in range(25):
            output = rig.cycle(6, dpad=(1, 0))  # held selection cannot restart
            assert output.mode == "LOCO"
            assert "sonic_clip_finished" not in output.events
        rig.cycle()  # release the selection before a deliberate re-entry
        assert rig.cycle(6).mode == "SONIC_CLIP"
    finally:
        rig.close()


def test_sonic_clip_rejected_trigger_is_not_lost(plan):
    rig = Rig.start(plan)
    try:
        _enter_via_fixedpos(rig, 6)
        _await_substate(rig, "READY")
        # A rejected trigger tick keeps the D-pad edge pending: the committed
        # D-pad only advances on an applied command.
        now = (rig.tick + 1) / 50.0
        operator = SimpleNamespace(dpad_x=1, dpad_y=0)
        rejected = rig.kernel.prepare(RuntimeInput(now, rig.robot, None,
                                                   operator_input=operator))
        assert rejected.skill_state == "CUE"
        rig.kernel.reject()
        state = rig.kernel.plugin(6)
        assert state.phase == "READY"
        retried = rig.kernel.tick(RuntimeInput(now, rig.robot, None, operator_input=operator))
        rig.tick += 1
        assert retried.skill_state == "CUE"
        assert state.play_tick == 0
        rig.robot = replace(rig.robot, sequence=rig.tick + 1,
                            timestamp_ns=round(now * 1e9),
                            joint_pos=retried.command.q_des.copy())
        np.testing.assert_array_equal(retried.command.q_des, rejected.command.q_des)
    finally:
        rig.close()


def _publish_stand_frames(mailbox, activation, now_s, count=16, dt=0.02, sequence_start=0):
    from cadence.sonic.contract import DEFAULT_ANGLES_ISAACLAB
    frame = {"root_quat_wxyz": [1.0, 0.0, 0.0, 0.0],
             "q": list(DEFAULT_ANGLES_ISAACLAB), "dq": [0.0] * 29}
    for index in range(count):
        stamp = now_s - (count - 1 - index) * dt
        assert mailbox.publish(activation, sequence_start + index, dt, [frame], receipt_s=stamp)


def test_sonic_stream_lifecycle(plan):
    rig = Rig.start(plan)
    events = []
    try:
        output = _enter_via_fixedpos(rig, 7)
        assert output.mode == "SONIC_STREAM" and output.skill_state == "WAITING"
        default = np.asarray(DEFAULT_ANGLES_CADENCE)
        np.testing.assert_allclose(output.command.q_des, default)
        _assert_gains(output.command, KP_PD_STAND_CADENCE, KD_PD_STAND_CADENCE)
        state = rig.kernel.plugin(7)
        mailbox = state.motion_ref
        assert state.receiver.address is not None

        # The receiver answers status queries with the live activation.
        reply = _query_status(state.receiver.address)
        assert reply["type"] == "status" and reply["activation"] == mailbox.activation
        assert reply["state_key"] == "sonic_stream"

        _publish_stand_frames(mailbox, mailbox.activation, time.monotonic())
        for _ in range(10):
            output = rig.cycle()
            events.extend(output.events)
            if output.skill_state == "TRACKING" and output.observation.shape == (1570,):
                break
        assert output.skill_state == "TRACKING"
        assert events.count("sonic_stream_live") == 1
        _assert_gains(output.command, KP_024_CADENCE, KD_024_CADENCE)

        # Stop the producer: beyond stale_ms the state blends back to stand.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            output = rig.cycle()
            events.extend(output.events)
            if output.skill_state == "LOST":
                break
            time.sleep(0.01)
        assert output.skill_state == "LOST"
        assert events.count("sonic_stream_lost") == 1
        _assert_gains(output.command, KP_PD_STAND_CADENCE, KD_PD_STAND_CADENCE)
        for _ in range(40):
            output = rig.cycle()
            if output.skill_state == "WAITING":
                break
        assert output.skill_state == "WAITING"
        np.testing.assert_allclose(output.command.q_des, default)
        _assert_gains(output.command, KP_PD_STAND_CADENCE, KD_PD_STAND_CADENCE)
        # A re-fed stream re-locks yaw and resumes tracking in the same activation.
        _publish_stand_frames(mailbox, mailbox.activation, time.monotonic(),
                              sequence_start=100)
        for _ in range(10):
            output = rig.cycle()
            events.extend(output.events)
            if output.skill_state == "TRACKING" and output.observation.shape == (1570,):
                break
        assert output.skill_state == "TRACKING"
        assert events.count("sonic_stream_live") == 2
    finally:
        rig.close()


def _query_status(address, timeout_s=2.0):
    request = json.dumps({"schema": "cadence.motion-ref.v1", "type": "status"}).encode()
    deadline = time.monotonic() + timeout_s
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(0.1)
        while time.monotonic() < deadline:
            sock.sendto(request, address)
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue
            return json.loads(data)
    raise AssertionError("no status reply from the motion-ref receiver")


def test_sonic_clip_reject_restores_history_and_replays(plan):
    rig = Rig.start(plan)
    try:
        _enter_via_fixedpos(rig, 6)
        _await_substate(rig, "READY")
        rig.cycle(dpad=(1, 0))  # D-pad right: CUE -> PLAYING
        output, _ = _await_substate(rig, "PLAYING")
        state = rig.kernel.plugin(6)
        rig.cycle()
        rig.cycle()
        history_before = state.policy.observations.build().copy()
        tick_before = state.play_tick

        now = (rig.tick + 1) / 50.0
        rejected = rig.kernel.prepare(RuntimeInput(now, rig.robot, None))
        rig.kernel.reject()
        np.testing.assert_array_equal(state.policy.observations.build(), history_before)
        assert state.play_tick == tick_before

        retried = rig.kernel.tick(RuntimeInput(now, rig.robot, None))
        rig.tick += 1
        rig.robot = replace(rig.robot, sequence=rig.tick + 1, timestamp_ns=round(now * 1e9),
                            joint_pos=retried.command.q_des.copy())
        np.testing.assert_array_equal(retried.command.q_des, rejected.command.q_des)
        assert state.play_tick == tick_before + 1
    finally:
        rig.close()
