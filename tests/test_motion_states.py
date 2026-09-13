from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from cadence_api import JointCommand
from cadence.backends.mock import MockBackend
from cadence.motion import (
    JointTargetFrame, LowerLocoState, LowerLocoStreamState, MotionConfig,
)
from cadence.plugins import ControlFrame, PluginCatalog
from cadence.runtime import RuntimeConfig, RuntimeInput, RuntimeKernel


class FakeLowerAdapter:
    """History-dependent policy makes rejection observable without robot assets."""

    observation_kind = "fake_lower"
    policy_model = "fake"

    def __init__(self, config, services):
        self.config = config
        self.action = np.arange(1, len(config.robot.lower_joints) + 1) * 0.2
        self.history = []
        self.inputs = []

    def reset(self):
        self.history = []

    def snapshot(self):
        return deepcopy(self.history)

    def restore(self, snapshot):
        self.history = snapshot

    def infer(self, frame, last_action):
        self.history.append(frame.now_s)
        self.inputs.append(last_action.copy())
        return self.action.copy(), np.asarray(self.history)


def config(n=5):
    lower = list(range(0, n, 2))
    upper = list(range(1, n, 2))
    return {
        "entry_smoothing_s": 0.0,
        "robot": {
            "default_position": [0.1] * n,
            "action_scale": [0.5] * n,
            "position_min": [-1.0] * n,
            "position_max": [1.0] * n,
            "lower_joints": lower,
            "upper_joints": upper,
        },
        "control": {"kp": list(range(10, 10 + n)), "kd": 2.0},
        "lower": {
            "factory": f"{__name__}:FakeLowerAdapter",
            "model": "unused-fake.onnx",
            "history_frames": 4,
            "runtime": {},
        },
        "upper": {"default_position": [0.25] * len(upper)},
    }


def frame(n=5, now=0.0):
    backend = MockBackend(n)
    backend.start()
    return ControlFrame(now, backend.read_state())


def entered(cls=LowerLocoState, n=5):
    state = cls(3, "motion", config(n), SimpleNamespace(dimension=n))
    state.on_enter(frame(n), [])
    return state


def accept(state, now=0.02):
    result = state.step(frame(state.n, now))
    state.on_command_applied(result.command)
    return result


@pytest.mark.parametrize("n", [5, 35])
def test_arbitrary_dimension_partition_pd_and_fixed_posture(n):
    state = entered(n=n)
    expected = np.full(n, 0.1)
    expected[state.config.robot.lower_joints] += state.adapter.action * 0.5
    expected[state.config.robot.upper_joints] = 0.25
    result = accept(state)
    np.testing.assert_allclose(result.command.q_des, np.clip(expected, -1, 1))
    np.testing.assert_equal(result.command.kp, np.arange(10, 10 + n))
    np.testing.assert_equal(result.command.kd, np.full(n, 2))
    np.testing.assert_equal(result.command.dq_des, np.zeros(n))
    np.testing.assert_equal(result.command.tau_ff, np.zeros(n))
    np.testing.assert_allclose(state.last_action, (result.command.q_des - 0.1) / 0.5)
    following = accept(state, 0.04)
    np.testing.assert_allclose(following.command.q_des[state.config.robot.upper_joints], 0.25)
    np.testing.assert_allclose(state.adapter.inputs[-1], state.last_action)
    assert state.phase == "HOLDING"
    assert not hasattr(state, "upper_targets")


def test_stream_entry_default_newest_frame_hold_and_activation_isolation():
    state = LowerLocoStreamState(3, "motion", config(), SimpleNamespace(dimension=5))
    assert not state.upper_targets.publish(JointTargetFrame(9, [0.9, 0.9], 1))
    state.on_enter(frame(), [])
    activation = state.upper_targets.activation
    assert state.upper_targets.publish(JointTargetFrame(0, [0.4, -0.3], activation))
    # An input arriving during entry must not replace the configured entry command.
    result = accept(state)
    np.testing.assert_allclose(result.command.q_des[[1, 3]], [0.25, 0.25])
    assert state.committed_upper_sequence is None
    assert state.upper_targets.publish(JointTargetFrame(2, [0.6, -0.2], activation))
    assert not state.upper_targets.publish(JointTargetFrame(1, [0.8, 0.8], activation))
    np.testing.assert_allclose(accept(state, 0.04).command.q_des[[1, 3]], [0.6, -0.2])
    assert state.phase == "TRACKING" and state.committed_upper_sequence == 2
    assert not state.upper_targets.publish(JointTargetFrame(2, [0.7, 0.7], activation))
    np.testing.assert_allclose(accept(state, 200.0).command.q_des[[1, 3]], [0.6, -0.2])
    assert state.phase == "HOLDING"
    state.on_exit(frame(), [])
    assert not state.upper_targets.publish(JointTargetFrame(3, [0.7, 0.7], activation))
    state.on_enter(frame(), [])
    assert state.upper_targets.activation != activation
    assert not state.upper_targets.publish(JointTargetFrame(100, [0.7, 0.7], activation))
    np.testing.assert_allclose(accept(state).command.q_des[[1, 3]], [0.25, 0.25])
    assert state.committed_upper_sequence is None


def test_rejected_stream_frame_rolls_back_history_phase_and_can_be_retried():
    state = entered(LowerLocoStreamState)
    accept(state)
    previous_action = state.last_action.copy()
    token = state.upper_targets.activation
    state.upper_targets.publish(JointTargetFrame(4, [0.8, -0.5], token))
    candidate = state.step(frame(now=0.04))
    assert candidate.substate == "TRACKING"
    assert state.phase == "DEFAULT" and state.committed_upper_sequence is None
    state.on_command_rejected()
    assert state.adapter.history == [0.02]
    assert state.phase == "DEFAULT" and state.committed_upper_sequence is None
    np.testing.assert_array_equal(state.last_action, previous_action)
    np.testing.assert_array_equal(state.upper_position, [0.25, 0.25])
    retry = accept(state, 0.06)
    np.testing.assert_array_equal(retry.command.q_des, candidate.command.q_des)
    assert state.adapter.history == [0.02, 0.06]
    assert state.committed_upper_sequence == 4 and state.phase == "TRACKING"


def test_feedback_uses_actual_smoothed_command_and_keeps_full_target():
    state = entered(LowerLocoStreamState)
    accept(state)
    state.upper_targets.publish(JointTargetFrame(0, [0.8, -0.6], state.upper_targets.activation))
    result = state.step(frame(now=0.04))
    actual = JointCommand(result.command.q_des * 0.5, np.zeros(5), np.ones(5), np.ones(5), np.zeros(5))
    state.on_command_applied(actual)
    np.testing.assert_allclose(state.last_action, (actual.q_des - 0.1) / 0.5)
    np.testing.assert_allclose(state.diagnostics_after_command(result.diagnostics).executed_action, state.last_action)
    np.testing.assert_array_equal(accept(state, 0.06).command.q_des[[1, 3]], [0.8, -0.6])


@pytest.mark.parametrize("action", [[float("nan"), 0, 0], [0, 0], [[0, 0, 0]]])
def test_invalid_policy_output_does_not_advance_history(action):
    state = entered()
    state.adapter.action = np.asarray(action)
    with pytest.raises(ValueError, match="lower policy action"):
        state.step(frame())
    assert state.adapter.history == [] and state.phase == "DEFAULT"
    assert state._pending is None


@pytest.mark.parametrize(
    "section,key,value",
    [
        ("robot", "default_position", [0] * 4),
        ("robot", "action_scale", [0.5, 0.5, 0, 0.5, 0.5]),
        ("robot", "position_min", [float("nan")] * 5),
        ("robot", "position_max", [-2.0] * 5),
        ("robot", "lower_joints", [0, 0, 2, 4]),
        ("robot", "lower_joints", [0.0, 2, 4]),
        ("robot", "upper_joints", [1, 5]),
        ("robot", "upper_joints", [0, 1, 3]),
        ("robot", "upper_joints", [1]),
        ("control", "kp", [-1] * 5),
        ("control", "kd", [1, 2]),
        ("upper", "default_position", [0.1]),
        ("upper", "default_position", [0.1, float("inf")]),
        ("upper", "default_position", [0.1, 1.01]),
        ("lower", "history_frames", True),
        ("lower", "mask_upper_observation", "false"),
        ("lower", "factory", "missing_module_separator"),
    ],
)
def test_configuration_rejects_invalid_layout_or_values(section, key, value):
    raw = config()
    raw[section][key] = value
    with pytest.raises(ValueError):
        MotionConfig.from_mapping(raw, dimension=5)


def test_config_is_detached_and_service_dimension_is_checked():
    raw = config()
    parsed = MotionConfig.from_mapping(raw)
    raw["upper"]["default_position"][0] = 0.9
    assert parsed.upper.default_position[0] == 0.25
    with pytest.raises(ValueError):
        parsed.robot.default_position[0] = 0.8
    with pytest.raises(ValueError, match="dimension"):
        LowerLocoState(2, "motion", parsed, SimpleNamespace(dimension=6))
    # Task service bundles are free to omit dimension when the robot config defines it.
    assert LowerLocoState(2, "motion", parsed, SimpleNamespace()).n == 5


def test_catalog_and_runtime_commit_reject_use_the_generic_state():
    catalog = PluginCatalog({
        "reset_state_id": 0,
        "safety_fallback_state_id": 0,
        "states": {
            0: {"key": "damping", "factory": "cadence.plugins:BasicState", "config": {"kind": "damping"}},
            3: {"key": "motion", "factory": "cadence.motion:LowerLocoStreamState", "config": config()},
        },
    })
    robot = frame().robot_state
    kernel = RuntimeKernel(RuntimeConfig(
        catalog, SimpleNamespace(dimension=5), ControlFrame,
        [-1] * 5, [1] * 5, start_state="motion",
    ), robot, 0.0)
    state = kernel.current
    kernel.prepare(RuntimeInput(0.02, robot, None))
    kernel.reject()
    assert state.adapter.history == [] and state.phase == "DEFAULT"
    kernel.tick(RuntimeInput(0.04, robot, None))
    state.upper_targets.publish(JointTargetFrame(0, [0.8, -0.8], state.upper_targets.activation))
    prepared = kernel.prepare(RuntimeInput(0.06, robot, None))
    assert not prepared.safety_halted and state.committed_upper_sequence is None
    committed = kernel.commit()
    assert state.committed_upper_sequence == 0
    np.testing.assert_allclose(committed.executed_action, (committed.command.q_des - 0.1) / 0.5)
