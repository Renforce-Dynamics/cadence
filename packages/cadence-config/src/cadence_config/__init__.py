"""Strict configuration composition with declaration-relative resource paths."""

from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from importlib import resources
from collections.abc import Mapping
import copy
import hashlib
import json
import os
import yaml


class ConfigError(ValueError):
    pass


class _Loader(yaml.SafeLoader):
    pass


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ConfigError(f"duplicate config key: {key}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def resolve_resource(value, *, base=None, artifacts=None):
    value = str(value)
    if value.startswith("pkg://"):
        package, sep, name = value[6:].partition("/")
        if not sep or not name or ".." in Path(name).parts:
            raise ConfigError(f"invalid package resource: {value}")
        try:
            result = Path(str(resources.files(package).joinpath(name)))
        except (ImportError, TypeError) as exc:
            raise ConfigError(f"package resource unavailable: {value}") from exc
    elif value.startswith("artifact://"):
        key = value[11:]
        if not artifacts or key not in artifacts:
            raise ConfigError(f"artifact not locked: {key}")
        record = artifacts[key]
        if set(record) != {"path", "sha256"}:
            raise ConfigError(f"artifact {key} needs path and sha256")
        result = resolve_resource(record["path"], base=base)
        actual = hashlib.sha256(result.read_bytes()).hexdigest()
        if actual != record["sha256"]:
            raise ConfigError(f"artifact checksum mismatch: {key}")
    else:
        result = Path(value).expanduser()
        if not result.is_absolute():
            if base is None:
                base = Path.cwd()
            result = Path(base) / result
    result = result.resolve()
    if not result.exists():
        raise ConfigError(f"resource does not exist: {result}")
    return result


def _leaves(value, prefix=""):
    if isinstance(value, Mapping) and value:
        for k, v in value.items():
            yield from _leaves(v, f"{prefix}.{k}" if prefix else str(k))
    else:
        yield prefix


def deep_merge(base, override):
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, Mapping) and isinstance(result.get(key), Mapping):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _merge(a, ao, b, bo):
    data = copy.deepcopy(a)
    origins = dict(ao)
    for key, value in b.items():
        prefix = str(key)
        if isinstance(value, Mapping) and isinstance(data.get(key), Mapping):
            # Keep untouched child origins; replacing a leaf overwrites its origin.
            old_leaves = set(_leaves(data[key], prefix))
            merged = deep_merge(data[key], value)
            new_leaves = set(_leaves(merged, prefix))
            for leaf in old_leaves - new_leaves:
                origins.pop(leaf, None)
            data[key] = merged
        else:
            for leaf in list(origins):
                if leaf == prefix or leaf.startswith(prefix + "."):
                    origins.pop(leaf)
            data[key] = copy.deepcopy(value)
    origins.update(bo)
    valid = set(_leaves(data))
    return data, {k: v for k, v in origins.items() if k in valid}


def _load(path, stack=()):
    path = resolve_resource(path)
    if path in stack:
        raise ConfigError("extends cycle: " + " -> ".join(map(str, (*stack, path))))
    try:
        raw = yaml.load(path.read_text(), Loader=_Loader) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"{path}: {exc}") from exc
    if not isinstance(raw, dict):
        raise ConfigError(f"{path}: configuration must be a mapping")
    data, origins = {}, {}
    parents = raw.pop("extends", [])
    if isinstance(parents, str):
        parents = [parents]
    if not isinstance(parents, list) or not all(
        isinstance(v, str) and v for v in parents
    ):
        raise ConfigError("extends must be a path or a list of paths")
    compose = raw.pop("compose", {})
    order = ("robot", "backend", "task", "site", "experiment")
    if not isinstance(compose, dict) or set(compose) - set(order):
        raise ConfigError("compose supports robot/backend/task/site/experiment")
    parents += [compose[k] for k in order if k in compose]
    for parent in parents:
        if not isinstance(parent, str):
            raise ConfigError("compose references must be strings")
        inherited, inherited_origins = _load(
            resolve_resource(parent, base=path.parent), (*stack, path)
        )
        data, origins = _merge(data, origins, inherited, inherited_origins)
    return _merge(data, origins, raw, {key: str(path) for key in _leaves(raw)})


def validate_keys(data, allowed, *, required=(), label="config"):
    if not isinstance(data, Mapping):
        raise ConfigError(f"{label} must be a mapping")
    unknown = set(data) - set(allowed)
    missing = set(required) - set(data)
    if unknown or missing:
        raise ConfigError(
            f"{label}: unknown keys={sorted(unknown)}, missing keys={sorted(missing)}"
        )


@dataclass(frozen=True)
class ResolvedConfig:
    data: dict
    origins: dict
    source: Path
    overrides: tuple = ()

    @property
    def digest(self):
        # YAML supports integer-key registries; normalize without absolute source paths.
        def canonical(v):
            if isinstance(v, dict):
                return {str(k): canonical(x) for k, x in v.items()}
            if isinstance(v, list):
                return [canonical(x) for x in v]
            return v

        return hashlib.sha256(
            json.dumps(
                canonical(self.data),
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode()
        ).hexdigest()

    def path(self, key, *, artifacts=None):
        value = self.data
        for part in key.split("."):
            value = value[part if part in value else int(part)]
        origin = Path(self.origins.get(key, str(self.source)))
        return resolve_resource(value, base=origin.parent, artifacts=artifacts)

    def freeze(self, directory):
        target = Path(directory)
        target.mkdir(parents=True, exist_ok=True)
        (target / "resolved.yaml").write_text(
            yaml.safe_dump(self.data, sort_keys=False)
        )
        (target / "provenance.json").write_text(json.dumps(self.origins, indent=2))
        (target / "overrides.json").write_text(json.dumps(self.overrides, indent=2))
        (target / "config.sha256").write_text(self.digest + "\n")


def load_config(path, *, overrides=(), allowed=None, required=()):
    overrides = tuple(overrides)
    source = resolve_resource(path)
    data, origins = _load(source)
    for expression in overrides:
        key, sep, raw = expression.partition("=")
        if not sep or not key:
            raise ConfigError("override must be key=value")
        node = data
        parts = key.split(".")
        for part in parts[:-1]:
            if part not in node or not isinstance(node[part], dict):
                raise ConfigError(f"unknown override path: {key}")
            node = node[part]
        if parts[-1] not in node:
            raise ConfigError(f"unknown override field: {key}")
        node[parts[-1]] = yaml.load(raw, Loader=_Loader)
        for old in list(origins):
            if old == key or old.startswith(key + "."):
                origins.pop(old)
        for leaf in _leaves(node[parts[-1]], key):
            origins[leaf] = str(source)
    if allowed is not None:
        validate_keys(data, allowed, required=required)
    return ResolvedConfig(data, origins, source, overrides)


__all__ = [
    "ConfigError",
    "ResolvedConfig",
    "resolve_resource",
    "deep_merge",
    "load_config",
    "validate_keys",
]
