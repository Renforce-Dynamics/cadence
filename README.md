# cadence

**Composable robot execution and simulation.**

Cadence runs control cycles, composes skill outputs and commits skill progress after a backend accepts a command. The same application can use mock, MuJoCo or A3 backends.

- Hierarchical state machines with activation IDs and parent cancellation.
- Independent safety supervision and explicit joint ownership.
- Layered configuration, provenance and reproducible snapshots.

Robot dimensions and task behavior are supplied by applications.

## Quick start

Requires Linux, Python 3.10+ and `uv`.

```bash
git clone --recurse-submodules git@github.com:Renforce-Dynamics/cadence.git
cd cadence
./scripts/bootstrap.sh --extra sim
./scripts/run.sh -- --backend mock --duration-s 1
./scripts/run.sh -- --backend mujoco --duration-s 1
./scripts/test.sh
```

## Packages and dependencies

| Package | Responsibility |
| --- | --- |
| `cadence-api` | State, command and RobotIO contracts |
| `cadence-config` | YAML composition, resources, provenance and snapshots |
| `cadence` | Execution kernel, state machines, inference and backends |

`external/agi3sdk` pins the optional A3 backend dependency. The default bootstrap installs only Cadence packages. Use `./scripts/bootstrap.sh --extra a3` to install the SDK and build its mock transport; enable AimRT separately for hardware. Third-party Python dependencies use package version constraints.

## Configuration and applications

```bash
.venv/bin/cadence config resolve configs/demo.yaml --set runtime.duration_s=2 --output runs/config
```

Configuration precedence is ordered `extends`, then `compose` layers (`robot`, `backend`, `task`, `site`, `experiment`), then the current file and explicit overrides. Mappings merge recursively; lists replace. See the [configuration reference](docs/configuration.md).

Applications register through the `cadence.applications` entry point. [cadence-rally](https://github.com/Renforce-Dynamics/cadence-rally) provides A3 rally skills and deployment profiles. Cadence has no dependency on its task code.

Command submission follows `prepare` → `guard_pending` → backend write → `commit`; rejected writes call `reject`. See [execution tests](tests/test_execution.py).

## Development

```bash
./scripts/submodules.sh init    # initialize or restore pinned dependencies
./scripts/submodules.sh check
./scripts/test.sh
./scripts/build.sh
```

Submodules pin source commits; Python requirements describe package compatibility. Bootstrap installs only the explicit packages in `source-workspace.json`. `scripts/setup.sh --wheelhouse /path/to/wheels` is available for package-based installation. Upgrade dependencies by committing reviewed submodule revisions with the parent repository.

## Authorship and license

Developed and maintained by [Renforce Dynamics](https://github.com/Renforce-Dynamics). See [AUTHORS.md](AUTHORS.md). Project code is available under the [MIT License](LICENSE).
