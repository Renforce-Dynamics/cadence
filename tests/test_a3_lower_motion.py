"""A3 model ABI regressions; runnable without any task repository installed."""

import hashlib
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from cadence.motion import a3, a3_lower


def _frame(index=0, **kwargs):
    robot = SimpleNamespace(
        gyro_b=np.array([index * 0.5, -index * 0.25, 200 - index]),
        quaternion_wxyz=(1, 0, 0, 0) if index % 2 == 0 else (0.5,) * 4,
        joint_pos=np.array(a3.DEFAULT_JOINT_POSITION) + (np.arange(29) - 14 + index) * 0.125,
        joint_vel=np.arange(29) * 0.25 - index,
    )
    return SimpleNamespace(robot_state=robot, **kwargs)


@pytest.mark.parametrize("builder_type,expected", [
    (a3.LowerVelocityObservationBuilder,
     "258492df09149ca885bb1112339277b7a008f40ee69dc32558df544b436d9dbd"),
    (a3.LowerVelocityH4ObservationBuilder,
     "db4e754d8cc1ee5a69d9b06de5bcb49a983824f894b9b03b0c6e7813e9cbf1cb"),
])
def test_observation_matches_pre_migration_deployment_bytes(builder_type, expected):
    # These golden digests were captured from the deployed observation builders
    # before extraction. Binary-exact inputs exercise clipping, gravity rotation,
    # first-frame backfill and eviction after more than four control frames.
    builder = builder_type()
    observations = []
    for index in range(8):
        robot = _frame(index).robot_state
        observations.append(builder.build(
            robot.gyro_b, robot.quaternion_wxyz, robot.joint_pos, robot.joint_vel,
            (np.arange(29) - index) * 0.0625,
            a3.lower_velocity_actor_command(index * 0.125, -index * 0.0625, index * 0.25),
        ))
    assert hashlib.sha256(np.array(observations, dtype="<f8").tobytes()).hexdigest() == expected


def _build(builder, index):
    robot = _frame(index).robot_state
    return builder.build(
        robot.gyro_b, robot.quaternion_wxyz, robot.joint_pos, robot.joint_vel,
        np.zeros(29), a3.lower_velocity_actor_command(0.5, -0.25, 0.125),
    )


def test_history_rollback_and_reset_reproduce_the_accepted_sequence():
    builder = a3.LowerVelocityH4ObservationBuilder()
    fresh = a3.LowerVelocityH4ObservationBuilder()
    _build(builder, 0)
    snapshot = builder.snapshot()
    rejected = _build(builder, 1)
    builder.restore(snapshot)
    assert _build(builder, 1) == rejected
    builder.reset()
    assert _build(builder, 5) == _build(fresh, 5)


@pytest.mark.parametrize("field,value", [
    ("gyro_b", (float("nan"), 0, 0)),
    ("quaternion_wxyz", (0, 0, 0, 0)),
    ("joint_pos", (0,) * 28),
    ("joint_vel", (float("inf"),) * 29),
])
def test_invalid_measurement_does_not_pollute_history(field, value):
    builder = a3.LowerVelocityH4ObservationBuilder()
    _build(builder, 0)
    previous = builder.snapshot()
    robot = _frame(1).robot_state
    setattr(robot, field, value)
    with pytest.raises(ValueError):
        builder.build(
            robot.gyro_b, robot.quaternion_wxyz, robot.joint_pos, robot.joint_vel,
            np.zeros(29), a3.lower_velocity_actor_command(0, 0, 0),
        )
    assert builder.snapshot() == previous


def _config(model, *, mask=False, history_frames=4):
    return SimpleNamespace(
        robot=SimpleNamespace(
            default_position=np.array(a3.DEFAULT_JOINT_POSITION),
            lower_joints=np.arange(15), upper_joints=np.arange(15, 29),
        ),
        lower=SimpleNamespace(
            model=str(model), runtime={}, history_frames=history_frames,
            mask_upper_observation=mask,
        ),
    )


def _cached_adapter(tmp_path, *, mask=False, action=None, spec=None):
    model = tmp_path / "actor.onnx"
    model.touch()
    runner = SimpleNamespace(infer=lambda obs: np.zeros(15) if action is None else action)
    spec = spec or SimpleNamespace(observation_dimension=397, action_dimension=15)
    received = []

    def load(config):
        received.append(config)
        return spec, runner

    adapter = a3.A3LowerPolicy(
        _config(model, mask=mask), SimpleNamespace(policies=SimpleNamespace(load=load))
    )
    return adapter, runner, received


def test_adapter_accepts_application_cache_without_task_type_dependency(tmp_path):
    adapter, runner, received = _cached_adapter(tmp_path)
    assert adapter.policy is runner
    assert received[0].policy_key == "velocity_lower_h4"
    assert received[0].model == tmp_path / "actor.onnx"
    assert isinstance(received[0].runtime, a3.OnnxRuntimeOptions)


@pytest.mark.parametrize("attribute", ["velocity_command", "loco_velocity"])
def test_adapter_reads_generic_and_legacy_velocity_frames(tmp_path, attribute):
    adapter, _, _ = _cached_adapter(tmp_path)
    frame = _frame(**{attribute: (0.5, -0.25, 0.125)})
    action, observation = adapter.infer(frame, np.zeros(29))
    assert action.shape == (15,)
    assert len(observation) == 397
    assert observation[377:380] == (0.5, -0.25, 0.125)


def test_upper_mask_keeps_model_input_neutral_without_mutating_feedback(tmp_path):
    masked, _, _ = _cached_adapter(tmp_path, mask=True)
    unmasked, _, _ = _cached_adapter(tmp_path, mask=False)
    frame = _frame(2, velocity_command=(0, 0, 0))
    before_q = frame.robot_state.joint_pos.copy()
    before_dq = frame.robot_state.joint_vel.copy()
    executed = np.arange(29) * 0.125
    before_executed = executed.copy()
    _, masked_obs = masked.infer(frame, executed)
    _, unmasked_obs = unmasked.infer(frame, executed)
    # H4 term offsets: relative q=24, dq=140, executed action=256.
    for offset in (24, 140, 256):
        masked_term = np.array(masked_obs[offset:offset + 116]).reshape(4, 29)
        unmasked_term = np.array(unmasked_obs[offset:offset + 116]).reshape(4, 29)
        np.testing.assert_array_equal(masked_term[:, 15:], 0)
        np.testing.assert_array_equal(masked_term[:, :15], unmasked_term[:, :15])
    np.testing.assert_array_equal(frame.robot_state.joint_pos, before_q)
    np.testing.assert_array_equal(frame.robot_state.joint_vel, before_dq)
    np.testing.assert_array_equal(executed, before_executed)


def test_adapter_history_rollback_excludes_rejected_frame(tmp_path):
    adapter, _, _ = _cached_adapter(tmp_path)
    adapter.infer(_frame(0), np.zeros(29))
    snapshot = adapter.snapshot()
    _, rejected_obs = adapter.infer(_frame(1), np.zeros(29))
    adapter.restore(snapshot)
    _, retried_obs = adapter.infer(_frame(1), np.zeros(29))
    assert retried_obs == rejected_obs
    adapter.reset()
    assert adapter.snapshot() == ((),) * 5


@pytest.mark.parametrize("action", [np.zeros(14), np.zeros((1, 15)), np.full(15, np.nan)])
def test_adapter_rejects_invalid_actor_output(tmp_path, action):
    adapter, _, _ = _cached_adapter(tmp_path, action=action)
    with pytest.raises(RuntimeError, match="15 finite actions"):
        adapter.infer(_frame(), np.zeros(29))


def test_adapter_validates_loaded_model_contract(tmp_path):
    spec = SimpleNamespace(observation_dimension=397, action_dimension=29)
    with pytest.raises(ValueError, match="397 observations and 15 actions"):
        _cached_adapter(tmp_path, spec=spec)


def test_adapter_loads_cadence_runner_without_application_services(tmp_path, monkeypatch):
    model = tmp_path / "actor.onnx"
    model.touch()
    received = []

    def create(*args):
        received.append(args)
        return SimpleNamespace(infer=lambda obs: np.zeros(15))

    monkeypatch.setattr(a3, "OnnxPolicy", create)
    adapter = a3.A3LowerPolicy(_config(model), SimpleNamespace())
    assert received[0][:3] == (model, 397, 15)
    assert adapter.infer(_frame(), np.zeros(29))[0].shape == (15,)


def test_adapter_does_not_search_the_services_repository_for_a_model(tmp_path):
    (tmp_path / 'actor.onnx').touch()
    with pytest.raises(ValueError, match='resolved relative to its declaring configuration'):
        a3.A3LowerPolicy(_config('actor.onnx'), SimpleNamespace(repository=tmp_path))


@pytest.mark.parametrize('model', ['pkg://cadence/__init__.py', 'artifact://actor'])
def test_adapter_rejects_resource_uris(model):
    with pytest.raises(ValueError, match='explicit filesystem path'):
        a3.A3LowerPolicy(_config(model), SimpleNamespace())


def test_root_a3_actor_preserves_deployed_action_values():
    pytest.importorskip("onnxruntime")
    adapter = a3.A3LowerPolicy(
        _config(Path(__file__).resolve().parents[1] / "models/a3_loco_lower.onnx"), SimpleNamespace()
    )
    frame = SimpleNamespace(
        robot_state=SimpleNamespace(
            gyro_b=(0, 0, 0), quaternion_wxyz=(1, 0, 0, 0),
            joint_pos=a3.DEFAULT_JOINT_POSITION, joint_vel=np.zeros(29),
        ),
        velocity_command=(0.4, -0.2, 0.1),
    )
    action, _ = adapter.infer(frame, np.zeros(29))
    # Recorded from the previously deployed model on the same neutral frame.
    np.testing.assert_allclose(action, [
        -0.084434688, 0.474569261, -0.039062258, -0.393917114, -1.657441020,
        0.704627991, -0.450644344, 0.095969245, 0.147777081, 0.445311993,
        -1.357830882, -0.636149645, -0.118864842, -0.246953070, -0.080331981,
    ], rtol=2e-5, atol=2e-6)


def _build_h32(builder, index):
    robot = _frame(index).robot_state
    return builder.build(
        robot.gyro_b, robot.quaternion_wxyz,
        robot.joint_pos[:15], robot.joint_vel[:15],
        np.zeros(15), (0.5, -0.25, 0.125),
    )


def test_h32_first_frame_backfills_every_term_history():
    builder = a3_lower.LowerVelocityHistoryObservationBuilder(history_frames=32)
    observation = _build_h32(builder, 0)
    assert len(observation) == 1635
    robot = _frame(0).robot_state
    gyro = np.array(observation[0:96]).reshape(32, 3)
    np.testing.assert_allclose(gyro, np.tile(np.clip(robot.gyro_b, -100, 100), (32, 1)))
    np.testing.assert_allclose(
        np.array(observation[96:192]).reshape(32, 3), np.tile((0, 0, -1), (32, 1))
    )
    q_rel = np.array(observation[192:672]).reshape(32, 15)
    np.testing.assert_allclose(q_rel, np.tile((np.arange(15) - 14) * 0.125, (32, 1)))
    dq = np.array(observation[672:1152]).reshape(32, 15)
    np.testing.assert_allclose(dq, np.tile(np.arange(15) * 0.25, (32, 1)))
    np.testing.assert_allclose(observation[1152:1632], 0)
    assert observation[1632:1635] == (0.5, -0.25, 0.125)


def test_h32_history_evicts_oldest_frame_beyond_32_ticks():
    builder = a3_lower.LowerVelocityHistoryObservationBuilder(history_frames=32)
    for index in range(33):
        observation = _build_h32(builder, index)
    gyro = np.array(observation[0:96]).reshape(32, 3)
    np.testing.assert_allclose(
        gyro,
        [np.clip(_frame(index).robot_state.gyro_b, -100, 100) for index in range(1, 33)],
    )


def test_h32_history_rollback_and_reset_reproduce_the_accepted_sequence():
    builder = a3_lower.LowerVelocityHistoryObservationBuilder(history_frames=32)
    fresh = a3_lower.LowerVelocityHistoryObservationBuilder(history_frames=32)
    _build_h32(builder, 0)
    snapshot = builder.snapshot()
    rejected = _build_h32(builder, 1)
    builder.restore(snapshot)
    assert _build_h32(builder, 1) == rejected
    builder.reset()
    assert _build_h32(builder, 5) == _build_h32(fresh, 5)


@pytest.mark.parametrize("field,value", [
    ("gyro_b", (float("nan"), 0, 0)),
    ("quaternion_wxyz", (0, 0, 0, 0)),
    ("joint_pos", (0,) * 14),
    ("joint_vel", (float("inf"),) * 15),
])
def test_h32_invalid_measurement_does_not_pollute_history(field, value):
    builder = a3_lower.LowerVelocityHistoryObservationBuilder(history_frames=32)
    _build_h32(builder, 0)
    previous = builder.snapshot()
    robot = _frame(1).robot_state
    setattr(robot, field, value)
    with pytest.raises(ValueError):
        builder.build(
            robot.gyro_b, robot.quaternion_wxyz,
            robot.joint_pos[:15], robot.joint_vel[:15],
            np.zeros(15), (0, 0, 0),
        )
    assert builder.snapshot() == previous


def _config32(model):
    return SimpleNamespace(
        robot=SimpleNamespace(
            default_position=np.array(a3.DEFAULT_JOINT_POSITION),
            lower_joints=np.arange(15), upper_joints=np.arange(15, 29),
        ),
        lower=SimpleNamespace(
            model=str(model), runtime={}, history_frames=32,
            mask_upper_observation=False,
        ),
    )


def _cached_adapter32(tmp_path, *, action=None, spec=None):
    model = tmp_path / "actor.onnx"
    model.touch()
    runner = SimpleNamespace(infer=lambda obs: np.zeros(15) if action is None else action)
    spec = spec or SimpleNamespace(observation_dimension=1635, action_dimension=15)
    received = []

    def load(config):
        received.append(config)
        return spec, runner

    adapter = a3_lower.A3LowerEstMoEPolicy(
        _config32(model), SimpleNamespace(policies=SimpleNamespace(load=load))
    )
    return adapter, runner, received


def test_h32_adapter_accepts_application_cache_without_task_type_dependency(tmp_path):
    adapter, runner, received = _cached_adapter32(tmp_path)
    assert adapter.policy is runner
    assert received[0].policy_key == "velocity_lower_h32"
    assert received[0].model == tmp_path / "actor.onnx"
    assert isinstance(received[0].runtime, a3.OnnxRuntimeOptions)


@pytest.mark.parametrize("attribute", ["velocity_command", "loco_velocity"])
def test_h32_adapter_reads_generic_and_legacy_velocity_frames(tmp_path, attribute):
    adapter, _, _ = _cached_adapter32(tmp_path)
    frame = _frame(**{attribute: (0.5, -0.25, 0.125)})
    action, observation = adapter.infer(frame, np.zeros(29))
    assert action.shape == (15,)
    assert len(observation) == 1635
    assert observation[-3:] == (0.5, -0.25, 0.125)


def test_h32_adapter_defaults_to_zero_command_without_velocity_frame(tmp_path):
    adapter, _, _ = _cached_adapter32(tmp_path)
    _, observation = adapter.infer(_frame(), np.zeros(29))
    assert observation[-3:] == (0.0, 0.0, 0.0)


def test_h32_upper_joints_never_enter_the_observation(tmp_path):
    adapter, _, _ = _cached_adapter32(tmp_path)
    lower = _frame(2, velocity_command=(0, 0, 0))
    moved = _frame(2, velocity_command=(0, 0, 0))
    moved.robot_state.joint_pos[15:] += 0.5
    moved.robot_state.joint_vel[15:] += 1.0
    executed = np.arange(29) * 0.125
    _, base_obs = adapter.infer(lower, executed)
    adapter.reset()
    _, moved_obs = adapter.infer(moved, executed)
    assert base_obs == moved_obs


def test_h32_adapter_history_rollback_excludes_rejected_frame(tmp_path):
    adapter, _, _ = _cached_adapter32(tmp_path)
    adapter.infer(_frame(0), np.zeros(29))
    snapshot = adapter.snapshot()
    _, rejected_obs = adapter.infer(_frame(1), np.zeros(29))
    adapter.restore(snapshot)
    _, retried_obs = adapter.infer(_frame(1), np.zeros(29))
    assert retried_obs == rejected_obs
    adapter.reset()
    assert adapter.snapshot() == ((),) * 5


@pytest.mark.parametrize("action", [np.zeros(14), np.zeros((1, 15)), np.full(15, np.nan)])
def test_h32_adapter_rejects_invalid_actor_output(tmp_path, action):
    adapter, _, _ = _cached_adapter32(tmp_path, action=action)
    with pytest.raises(RuntimeError, match="15 finite actions"):
        adapter.infer(_frame(), np.zeros(29))


@pytest.mark.parametrize("spec,message", [
    (SimpleNamespace(observation_dimension=397, action_dimension=15),
     "1635 observations and 15 actions"),
    (SimpleNamespace(observation_dimension=1635, action_dimension=29),
     "1635 observations and 15 actions"),
])
def test_h32_adapter_validates_loaded_model_contract(tmp_path, spec, message):
    with pytest.raises(ValueError, match=message):
        _cached_adapter32(tmp_path, spec=spec)


def test_h32_adapter_loads_cadence_runner_without_application_services(tmp_path, monkeypatch):
    model = tmp_path / "actor.onnx"
    model.touch()
    received = []

    def create(*args):
        received.append(args)
        return SimpleNamespace(infer=lambda obs: np.zeros(15))

    monkeypatch.setattr(a3_lower.estmoe, "OnnxPolicy", create)
    adapter = a3_lower.A3LowerEstMoEPolicy(_config32(model), SimpleNamespace())
    assert received[0][:3] == (model, 1635, 15)
    assert adapter.infer(_frame(), np.zeros(29))[0].shape == (15,)


def test_h32_adapter_rejects_non_32_history(tmp_path):
    model = tmp_path / "actor.onnx"
    model.touch()
    config = _config32(model)
    config.lower.history_frames = 4
    with pytest.raises(ValueError, match="history_frames=32"):
        a3_lower.A3LowerEstMoEPolicy(config, SimpleNamespace(policies=SimpleNamespace(load=None)))


@pytest.mark.parametrize('model', ['pkg://cadence/__init__.py', 'artifact://actor'])
def test_h32_adapter_rejects_resource_uris(model):
    with pytest.raises(ValueError, match='explicit filesystem path'):
        a3_lower.A3LowerEstMoEPolicy(_config32(model), SimpleNamespace())


def test_root_a3_estmoe_actor_preserves_deployed_action_values():
    pytest.importorskip("onnxruntime")
    adapter = a3_lower.A3LowerEstMoEPolicy(
        _config32(
            Path(__file__).resolve().parents[1] / "models/a3_loco_lower_estmoe_h32.onnx"
        ),
        SimpleNamespace(),
    )
    frame = SimpleNamespace(
        robot_state=SimpleNamespace(
            gyro_b=(0, 0, 0), quaternion_wxyz=(1, 0, 0, 0),
            joint_pos=a3.DEFAULT_JOINT_POSITION, joint_vel=np.zeros(29),
        ),
        velocity_command=(0.4, -0.2, 0.1),
    )
    action, _ = adapter.infer(frame, np.zeros(29))
    # Recorded from the deployed H32 EstMoE model on the same neutral frame.
    np.testing.assert_allclose(action, [
        -0.005238550, 0.282683074, -0.020015270, -0.138580650, -0.539466500,
        0.337088168, -0.132470980, 0.022386385, -0.011797843, 0.420523077,
        -0.776185155, -0.306916654, -0.136782631, 0.085067093, -0.233053312,
    ], rtol=2e-5, atol=2e-6)

