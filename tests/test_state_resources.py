"""Task resources remain explicit files resolved at their declaration sites."""

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from cadence.backends.mock import MockBackend
from cadence.deployment import load_run_config, run_config
from cadence.plugins import ControlResult, ControlState
from cadence_api import JointCommand
from cadence_config import ConfigError


class ResourceState(ControlState):
    instances = []

    def __init__(self, *args):
        super().__init__(*args)
        self.entered = False
        self.instances.append(self)

    def on_enter(self, frame, events):
        self.entered = True

    def step(self, frame):
        zeros = np.zeros(self.services.dimension)
        return ControlResult(JointCommand(zeros, zeros, zeros, zeros, zeros), substate="HOLD")


def write_yaml(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(value))
    return path


def entry(path, state_config):
    return write_yaml(path, {
        "schema_version": 1,
        "robot": {"joints": ["arm", "hip"], "position_min": [-1, -1], "position_max": [1, 1]},
        "backend": {"kind": "mock"},
        "runtime": {"control_hz": 50, "deadline_ms": 20, "duration_s": .04, "start_state": "reference"},
        "catalog": {"reset_state_id": 0, "safety_fallback_state_id": 1, "states": {
            0: {"key": "reference", "factory": f"{__name__}:ResourceState", "config": state_config},
            1: {"key": "damping", "factory": "cadence.plugins:BasicState", "config": {"kind": "damping"}},
        }},
    })


def resources(resolved):
    return resolved.data["catalog"]["states"][0]["config"]["resources"]


def test_state_file_resource_inheritance_retains_each_leaf_origin(tmp_path, monkeypatch):
    parent = write_yaml(tmp_path / "shared/reference.yaml", {
        "resources": {"trajectory": "motion.npz", "joint.map": "joints.txt"},
    })
    (parent.parent / "motion.npz").write_bytes(b"parent motion")
    names = parent.parent / "joints.txt"
    names.write_text("arm\nhip\n")
    child = write_yaml(tmp_path / "states/reference.yaml", {
        "extends": "../shared/reference.yaml", "resources": {"trajectory": "child.npz"},
    })
    motion = child.parent / "child.npz"
    motion.write_bytes(b"child motion")
    selected = entry(tmp_path / "entry.yaml", "states/reference.yaml")
    raw = yaml.safe_load(selected.read_text())
    registry = write_yaml(tmp_path / "registries/default.yaml", raw.pop("catalog"))
    registry_raw = yaml.safe_load(registry.read_text())
    registry_raw["states"][0]["config"] = "../states/reference.yaml"
    write_yaml(registry, registry_raw)
    raw["runtime"]["state_registry_config"] = "registries/default.yaml"
    write_yaml(selected, raw)
    monkeypatch.chdir("/")

    resolved = load_run_config(selected)
    assert resources(resolved) == {"trajectory": str(motion), "joint.map": str(names)}
    assert resolved.origins["catalog.states.0.config.resources.trajectory"] == str(child)
    assert resolved.origins["catalog.states.0.config.resources.joint.map"] == str(parent)


def test_inline_state_resources_resolve_relative_to_inherited_entry(tmp_path, monkeypatch):
    inherited = entry(tmp_path / "base/entry.yaml", {"resources": {"reference": "sample.npz"}})
    sample = inherited.parent / "sample.npz"
    sample.write_bytes(b"sample")
    selected = write_yaml(tmp_path / "site/entry.yaml", {
        "extends": "../base/entry.yaml", "runtime": {"duration_s": .02},
    })
    monkeypatch.chdir("/")
    resolved = load_run_config(selected)
    assert resources(resolved)["reference"] == str(sample)
    assert resolved.origins["catalog.states.0.config.resources.reference"] == str(inherited)


@pytest.mark.parametrize("value", [None, True, 3, [], {}, "", " ",
                                        "pkg://task/reference.npz", "artifact://trajectory",
                                        "file:///tmp/reference.npz", "https://example.com/reference.npz",
                                        "urn:reference"])
def test_resource_values_require_explicit_file_strings(tmp_path, value):
    with pytest.raises(ConfigError, match="explicit filesystem path string"):
        load_run_config(entry(tmp_path / "entry.yaml", {"resources": {"reference": value}}))


@pytest.mark.parametrize("value", [None, True, 3, [], "motion.npz"])
def test_resources_requires_a_mapping(tmp_path, value):
    with pytest.raises(ConfigError, match="resources must be a mapping"):
        load_run_config(entry(tmp_path / "entry.yaml", {"resources": value}))


@pytest.mark.parametrize("name", [None, 3, "", " "])
def test_resource_names_are_nonempty_strings(tmp_path, name):
    with pytest.raises(ConfigError, match="resources names must be nonempty strings"):
        load_run_config(entry(tmp_path / "entry.yaml", {"resources": {name: "motion.npz"}}))


def test_missing_resource_does_not_use_a_same_named_file_in_cwd(tmp_path, monkeypatch):
    selected = entry(tmp_path / "site/entry.yaml", {"resources": {"reference": "motion.npz"}})
    (tmp_path / "motion.npz").write_bytes(b"wrong location")
    monkeypatch.chdir(tmp_path)
    with pytest.raises(ConfigError, match="file does not exist"):
        load_run_config(selected)


def test_resource_must_be_a_file(tmp_path):
    directory = tmp_path / "motion.npz"
    directory.mkdir()
    with pytest.raises(ConfigError, match="file does not exist"):
        load_run_config(entry(tmp_path / "entry.yaml", {"resources": {"reference": "motion.npz"}}))


@pytest.mark.parametrize("check", [False, True])
def test_factories_receive_resource_paths_and_canonical_joint_names(tmp_path, monkeypatch, check):
    motion = tmp_path / "motion.npz"
    motion.write_bytes(b"reference")
    selected = entry(tmp_path / "entry.yaml", {"resources": {"reference": str(motion)}})
    if check:
        def unexpected_start(*args):
            pytest.fail("--check must not start RobotIO or operator inputs")

        from cadence.operator import JoystickCommandReceiver
        monkeypatch.setattr(MockBackend, "start", unexpected_start)
        monkeypatch.setattr(JoystickCommandReceiver, "__init__", unexpected_start)
    assert run_config(selected, check=check) == 0
    state = ResourceState.instances[-1]
    assert state.config["resources"] == {"reference": str(motion)}
    assert state.services.joint_names == ("arm", "hip")
    assert state.services.dimension == len(state.services.joint_names) == 2
    assert state.entered is not check


def test_freeze_records_resource_content_hash_and_declaration(tmp_path):
    motion = tmp_path / "motion.npz"
    selected = entry(tmp_path / "entry.yaml", {"resources": {"reference": "motion.npz"}})
    digests = []
    for index, content in enumerate((b"first reference", b"updated reference")):
        motion.write_bytes(content)
        output = tmp_path / f"run-{index}"
        assert run_config(selected, output=output, check=True) == 0
        snapshot = json.loads((output / "deployment.json").read_text())
        assert snapshot["resource_sha256"][str(motion)] == hashlib.sha256(content).hexdigest()
        assert yaml.safe_load((output / "resolved.yaml").read_text())["catalog"]["states"][0]["config"]["resources"]["reference"] == str(motion)
        assert json.loads((output / "provenance.json").read_text())["catalog.states.0.config.resources.reference"] == str(selected)
        digests.append(snapshot)
    assert digests[0]["config_sha256"] == digests[1]["config_sha256"]
    assert digests[0]["resource_sha256"] != digests[1]["resource_sha256"]
