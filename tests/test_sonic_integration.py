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

from cadence.sonic.contract import DEFAULT_ANGLES_CADENCE, ISAACLAB_JOINT_NAMES

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

    def cycle(self, request=None):
        self.tick += 1
        now = self.tick / 50.0
        output = self.kernel.tick(RuntimeInput(now, self.robot, None, requested_state=request))
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


def test_sonic_clip_lifecycle(plan):
    rig = Rig.start(plan)
    events = []
    try:
        output = _enter_via_fixedpos(rig, 6)
        assert output.mode == "SONIC_CLIP" and output.skill_state == "RAMP"
        for _ in range(60):
            output = rig.cycle()
            events.extend(output.events)
            if output.skill_state == "PLAYING":
                break
        assert output.skill_state == "PLAYING"
        assert events.count("sonic_clip_started") == 1
        state = rig.kernel.plugin(6)
        default = np.asarray(DEFAULT_ANGLES_CADENCE)
        # PLAYING advances exactly one reference tick per applied command.
        for expected in range(1, 6):
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
        # Jump near the clip end to reach DONE without replaying 33 s.
        state.play_tick = state.clip.frames - 2
        for _ in range(4):
            output = rig.cycle()
            events.extend(output.events)
        assert output.skill_state == "DONE"
        assert events.count("sonic_clip_finished") == 1
        # DONE keeps the policy alive on the clamped final reference frame.
        for _ in range(3):
            output = rig.cycle()
        assert output.skill_state == "DONE"
        assert state.play_tick == state.clip.frames - 1
    finally:
        rig.close()


def test_sonic_clip_on_finish_loco_handoff(plan):
    cfg = plan.resolved.data
    services = SimpleNamespace(dimension=plan.dimension,
                               joint_names=tuple(cfg["robot"]["joints"]))
    base = plan.catalog.definitions["sonic_clip"][1]["config"]
    from dataclasses import replace as dc_replace
    from cadence.sonic.state import SonicConfig, SonicTrackState
    config = dc_replace(SonicConfig.from_mapping(base), on_finish="loco")
    state = SonicTrackState(6, "sonic_clip", config, services)
    zero = np.zeros(29)
    robot = RobotState(1, 0, np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]), zero, zero, zero)
    robot = replace(robot, joint_pos=np.asarray(DEFAULT_ANGLES_CADENCE).copy())
    tick = 0

    def cycle():
        nonlocal robot, tick
        tick += 1
        result = state.step(ControlFrame(tick / 50.0, robot))
        state.on_command_applied(result.command)
        robot = replace(robot, sequence=tick + 1, timestamp_ns=round(tick / 50.0 * 1e9),
                        joint_pos=result.command.q_des.copy())
        return result

    try:
        state.on_enter(ControlFrame(0.0, robot), [])
        for _ in range(60):
            result = cycle()
            if result.substate == "PLAYING":
                break
        assert result.substate == "PLAYING"
        state.play_tick = state.clip.frames - 2
        results = [cycle() for _ in range(3)]
        assert results[1].substate == "DONE" and results[1].next_state == "loco"
        assert "sonic_clip_finished" in results[1].events
        assert results[2].next_state is None  # handoff offered exactly once
    finally:
        state.on_exit(ControlFrame(tick / 50.0, robot), [])


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
        for _ in range(40):
            output = rig.cycle()
            if output.skill_state == "WAITING":
                break
        assert output.skill_state == "WAITING"
        np.testing.assert_allclose(output.command.q_des, default)
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
        for _ in range(60):
            output = rig.cycle()
            if output.skill_state == "PLAYING":
                break
        assert output.skill_state == "PLAYING"
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
