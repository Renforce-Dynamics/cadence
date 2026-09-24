"""An editable entry tree, not a packaged default or CLI mode, owns execution."""
from pathlib import Path
import shutil

import numpy as np
import pytest
import yaml

from cadence.cli import main
from cadence.deployment import load_run_config, run_config
from cadence_config import ConfigError, load_config

ROOT = Path(__file__).resolve().parents[1]
SERVICE_ENTRIES = {
    'entry_joystick.yaml': {'version', 'device', 'target', 'publisher', 'inputs'},
}


@pytest.mark.parametrize('command', ['run', 'deploy'])
def test_entry_argument_is_required(command, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    with pytest.raises(SystemExit) as error:
        main([command, '--check'])
    assert error.value.code == 2
    assert not (tmp_path / 'runs').exists()


@pytest.mark.parametrize('flag,value', [
    ('--backend', 'mujoco'), ('--profile', 'cadence-rally/continuous'),
    ('--duration-s', '1'), ('--headless', None), ('--set', 'runtime.duration_s=1'),
])
def test_execution_choices_cannot_override_entry(flag, value):
    args = ['run', '--config', str(ROOT / 'configs/entry/examples/entry_mock.yaml'), flag]
    if value is not None:
        args.append(value)
    with pytest.raises(SystemExit) as error:
        main(args)
    assert error.value.code == 2


def test_repository_has_one_configuration_tree():
    assert not list((ROOT / 'src').rglob('*.yaml'))
    assert not list((ROOT / 'src').rglob('*.yml'))
    entries = sorted((ROOT / 'configs/entry').rglob('entry_*.yaml'))
    assert len(entries) == 14
    assert not list((ROOT / 'configs/entry').glob('*.yaml'))
    for entry in entries:
        if entry.name in SERVICE_ENTRIES:
            resolved = load_config(entry)
            assert set(resolved.data) == SERVICE_ENTRIES[entry.name]
        else:
            resolved = load_run_config(entry)
            assert Path(resolved.data['runtime']['state_registry_config']).is_relative_to(ROOT / 'configs')
        assert all(not source.startswith('pkg://') for source in resolved.origins.values())


def test_source_packages_contain_code_and_no_runtime_data():
    roots = [ROOT / 'src', *sorted((ROOT / 'packages').glob('*/src'))]
    for source in roots:
        for path in source.rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts and not any(part.endswith('.egg-info') for part in path.parts):
                assert path.suffix in {'.py', '.pyi'} or path.name == 'py.typed', path
    assert not (ROOT / 'src/cadence/data').exists()
    assert (ROOT / 'models/a3_loco_lower.onnx').is_file()
    assert (ROOT / 'assets/two_joint.xml').is_file()


@pytest.mark.parametrize('value', ['pkg://cadence/__init__.py', 'artifact://actor'])
def test_models_require_explicit_files_in_their_declaring_configuration(tmp_path, value):
    entry = tmp_path / 'entry.yaml'
    entry.write_text(yaml.safe_dump({
        'extends': str(ROOT / 'configs/entry/examples/entry_sim.yaml'),
        'backend': {'model': value},
    }))
    with pytest.raises(ConfigError, match='explicit filesystem path'):
        load_run_config(entry)


def test_missing_model_does_not_fall_back_to_repository_asset(tmp_path, monkeypatch):
    entry = tmp_path / 'entry.yaml'
    entry.write_text(yaml.safe_dump({
        'extends': str(ROOT / 'configs/entry/examples/entry_sim.yaml'),
        'backend': {'model': 'assets/two_joint.xml'},
    }))
    monkeypatch.chdir(ROOT)
    with pytest.raises(ConfigError, match='file does not exist'):
        load_run_config(entry)


def test_local_state_edit_changes_actual_pd_command(tmp_path, monkeypatch):
    from cadence.backends.mock import MockBackend

    shutil.copytree(ROOT / 'configs', tmp_path / 'configs')
    entry = tmp_path / 'configs/entry/examples/entry_mock.yaml'
    raw = yaml.safe_load(entry.read_text())
    raw['runtime']['duration_s'] = .06
    entry.write_text(yaml.safe_dump(raw))
    state_path = tmp_path / 'configs/states/two_joint_fixed.yaml'
    original = MockBackend.write_command
    commands = []

    def record(backend, command, sequence):
        commands.append(command.q_des.copy())
        original(backend, command, sequence)

    monkeypatch.setattr(MockBackend, 'write_command', record)
    monkeypatch.chdir('/')
    for target in ([.1, -.2], [.3, -.1]):
        state_path.write_text(yaml.safe_dump({'kind': 'fixed_position', 'target': target, 'duration_s': .001}))
        assert run_config(entry) == 0
        np.testing.assert_allclose(commands[-1], target)
        expanded = load_run_config(entry)
        assert expanded.origins['catalog.states.2.config.target'] == str(state_path)
    state_path.unlink()
    with pytest.raises(ConfigError, match='does not exist'):
        load_run_config(entry)


def test_registry_and_inline_catalog_are_not_competing_sources(tmp_path):
    entry = tmp_path / 'entry.yaml'
    entry.write_text(yaml.safe_dump({
        'extends': str(ROOT / 'configs/entry/examples/entry_mock.yaml'), 'catalog': {'states': {}},
    }))
    with pytest.raises(ConfigError, match='not both'):
        load_run_config(entry)


def test_missing_entry_does_not_fall_back_to_checkout(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    with pytest.raises(ConfigError, match='configuration does not exist'):
        load_run_config('configs/entry/examples/entry_mock.yaml')


def test_package_configuration_is_not_an_entry():
    with pytest.raises(ConfigError, match='not a package resource'):
        load_run_config('pkg://cadence/data/demo.yaml')


def test_visible_simulation_obeys_configured_wall_clock(tmp_path, monkeypatch):
    pytest.importorskip('mujoco')
    import mujoco.viewer
    import cadence.deployment as deployment

    sleeps, rendered, closed = [], [], []

    class Viewer:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            closed.append(True)

        def is_running(self):
            return True

        def sync(self):
            rendered.append(True)

    monkeypatch.setattr(mujoco.viewer, 'launch_passive', lambda *args: Viewer())
    monkeypatch.setattr(deployment.time, 'sleep', lambda value: sleeps.append(value))
    entry = tmp_path / 'entry_visible.yaml'
    entry.write_text(yaml.safe_dump({'extends': str(ROOT / 'configs/entry/examples/entry_sim.yaml'),
                                    'runtime': {'duration_s': .04, 'headless': False}}))
    assert run_config(entry) == 0
    assert len(rendered) == len(sleeps) == 2
    assert all(value > 0 for value in sleeps)
    assert closed == [True]
