from pathlib import Path
import json
import hashlib
import pytest
from cadence_config import load_config, ConfigError, resolve_resource


def test_declaration_origin_survives_composition_and_cwd(tmp_path, monkeypatch):
    base = tmp_path / "base"
    base.mkdir()
    (base / "model.bin").write_bytes(b"weights")
    (base / "robot.yaml").write_text(
        "robot: {model: model.bin, joints: [a, b]}\nruntime: {hz: 50, duration: 1}\n"
    )
    (tmp_path / "entry.yaml").write_text(
        "compose: {robot: base/robot.yaml}\nruntime: {duration: 3}\n"
    )
    monkeypatch.chdir("/")
    cfg = load_config(tmp_path / "entry.yaml", overrides=["runtime.hz=100"])
    assert cfg.path("robot.model") == base / "model.bin"
    assert cfg.data["runtime"] == {"hz": 100, "duration": 3}
    assert cfg.origins["robot.model"] == str(base / "robot.yaml")
    cfg.freeze(tmp_path / "snapshot")
    assert (tmp_path / "snapshot/config.sha256").read_text().strip() == cfg.digest


def test_list_replacement_and_unknown_override(tmp_path):
    (tmp_path / "a.yaml").write_text("a: [1, 2]\n")
    (tmp_path / "b.yaml").write_text("extends: a.yaml\na: [3]\n")
    assert load_config(tmp_path / "b.yaml").data["a"] == [3]
    with pytest.raises(ConfigError):
        load_config(tmp_path / "b.yaml", overrides=["typo=2"])
    with pytest.raises(ConfigError):
        load_config(tmp_path / "b.yaml", allowed={"b"})


@pytest.mark.parametrize(
    "body", ["x: 1\nx: 2", "extends: bad.yaml", "compose: {unknown: a.yaml}"]
)
def test_invalid_composition(tmp_path, body):
    p = tmp_path / "bad.yaml"
    p.write_text(body)
    with pytest.raises(ConfigError):
        load_config(p)


def test_artifact_checksum_and_package_resource(tmp_path):
    p = tmp_path / "weights"
    p.write_bytes(b"one")
    lock = {"policy": {"path": str(p), "sha256": hashlib.sha256(b"one").hexdigest()}}
    assert resolve_resource("artifact://policy", artifacts=lock) == p
    p.write_bytes(b"two")
    with pytest.raises(ConfigError):
        resolve_resource("artifact://policy", artifacts=lock)
    assert resolve_resource("pkg://cadence/data/two_joint.xml").is_file()
    with pytest.raises(ConfigError):
        resolve_resource("pkg://cadence/../private")


def test_precedence_is_fixed_across_parents_layers_and_cli(tmp_path):
    layers = ["base", "second", "robot", "backend", "task", "site", "experiment"]
    for index, name in enumerate(layers):
        (tmp_path / f"{name}.yaml").write_text(
            f"runtime: {{hz: {index}, owner: {name}}}\n{name}: true\n"
        )
    entry = tmp_path / "entry.yaml"
    entry.write_text(
        "extends: [base.yaml, second.yaml]\n"
        "compose: {experiment: experiment.yaml, site: site.yaml, task: task.yaml, "
        "backend: backend.yaml, robot: robot.yaml}\n"
        "runtime: {hz: 50}\n"
    )
    cfg = load_config(entry, overrides=["runtime.hz=100"])
    assert cfg.data["runtime"] == {"hz": 100, "owner": "experiment"}
    assert all(cfg.data[name] for name in layers)
    assert cfg.origins["runtime.owner"] == str(tmp_path / "experiment.yaml")
    assert cfg.origins["runtime.hz"] == str(entry)
