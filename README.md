# cadence

**Composable robot execution and simulation.**

Cadence runs control cycles, composes skill outputs and commits skill progress after a backend accepts a command. The same application can use mock, MuJoCo or A3 backends.

- Hierarchical state machines with activation IDs and parent cancellation.
- Independent safety supervision and explicit joint ownership.
- Layered configuration, provenance and reproducible snapshots.
- Reusable lower-body locomotion with fixed or streamed upper-joint targets.
- A shared operator endpoint for state requests, velocity, safety signals and status queries.
- Optional external localization with explicit source, coordinate-frame and freshness checks.
- Independent deployment profiles for simulation, A3 readonly operation and A3 command execution.

Robot dimensions are configured by deployments. The package includes an A3 lower-body policy adapter and model; task behavior remains in applications.

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
| `cadence-config` | Compatibility imports forwarding to `planet-config` |
| `cadence-protocol` | Legacy protocol imports and schema defaults forwarding to `planet-protocol` |
| `cadence` | Execution kernel, reusable motion states, inference and backends |

Cadence consumes the shared configuration and protocol libraries from [planetConfig](https://github.com/Renforce-Dynamics/planetConfig): `planet-config` owns YAML composition and resources, and `planet-protocol` owns the operator, joint-target and localization wire contracts. Planet services use those libraries directly and have no source or package dependency on Cadence or the SDK. The two legacy Cadence packages are thin compatibility layers.

`external/planetConfig` pins the shared libraries. `external/agi3sdk` pins the optional A3 backend dependency. The default bootstrap installs Cadence and the shared libraries. Use `./scripts/bootstrap.sh --extra a3` to install the SDK and build its mock transport; enable AimRT separately for hardware. Third-party Python dependencies use package version constraints.

## Configuration and applications

```bash
.venv/bin/cadence config resolve configs/demo.yaml --set runtime.duration_s=2 --output runs/config
```

Configuration precedence is ordered `extends`, then `compose` layers (`robot`, `backend`, `task`, `site`, `experiment`), then the current file and explicit overrides. Mappings merge recursively; lists replace. See the [configuration reference](docs/configuration.md).

Applications register through the `cadence.applications` entry point. [cadence-rally](https://github.com/Renforce-Dynamics/cadence-rally) provides rally skills and deployment profiles, reusing Cadence's lower-body motion states. Cadence has no dependency on its task code.

[planetJoystick](https://github.com/Renforce-Dynamics/planetJoystick) can connect directly to Cadence through the shared Planet protocol. It supplies configurable requests and joint targets to compatible receivers; applications extend the selected state catalog and task bindings. Cadence owns its receiver and state semantics. See [repository responsibilities, dependency graph and matching rules](docs/architecture.md).

Command submission follows `prepare` → `guard_pending` → backend write → `commit`; rejected writes call `reject`. See [execution tests](tests/test_execution.py).

## Independent deployment

Cadence provides its own deploy command and script. It runs without a rally package or process; cadence-rally uses the same execution infrastructure with task states and protocol adapters.

```bash
./scripts/bootstrap.sh --extra a3 --extra inference
./scripts/deploy.sh --config pkg://cadence/data/deployment/a3_sdk_mock.yaml --check
./scripts/deploy.sh --config pkg://cadence/data/deployment/a3_sdk_mock.yaml --duration-s 1
./scripts/deploy.sh --config pkg://cadence/data/deployment/a3_readonly.yaml --check
```

`--check` validates configuration, state factories, model resources and backend parameters without opening I/O. Deploy writes a resolved configuration snapshot under `runs/deploy-...`, or the directory selected by `--output`. The script accepts `--venv`; relative configuration paths remain relative to the caller's working directory.

The native SDK mock uses stationary snapshots and an in-memory command sink. It exercises SDK sequencing and write acceptance without AimRT or dynamics simulation. Real A3 profiles require an AimRT-enabled SDK and the target middleware environment. Readonly operation evaluates shadow state without publishing commands; hardware command execution retains the `A3_CONFIRM_ONBOARD=YES` gate. See the [deployment guide](docs/deployment.md) for installation, site overrides and hardware profiles.

## Lower-body locomotion and arm motion

| State factory | Upper-body behavior |
| --- | --- |
| `cadence.motion:LowerLocoState` | Hold the configured joint posture |
| `cadence.motion:LowerLocoStreamState` | Start at the configured posture, then execute and hold the latest joint target |

Both states run the lower policy every control cycle and submit one complete PD command. The included A3 adapter controls 12 leg joints and 3 waist joints; the upper target contains 14 arm joint angles in radians. Joint partitions and `lower.factory` let other robots reuse the states with their own adapters.

```bash
./scripts/bootstrap.sh --extra inference
.venv/bin/cadence run --config pkg://cadence/data/a3_lower_demo.yaml --duration-s 1
.venv/bin/cadence run --config pkg://cadence/data/a3_lower_stream_demo.yaml --duration-s 0
```

These examples run the actual A3 actor against the mock backend. The streaming example explicitly enables a local UDP receiver on port `15100`; `scripts/send-upper-target.py` sends joint targets. During an activation, delayed or interrupted input keeps the latest target indefinitely. The default posture is reapplied on state entry, and safety supervision always retains priority. A reception receipt confirms mailbox acceptance; execution is committed only after the backend accepts the composed command.

See [configuration overlays, streaming examples and the motion contract](docs/motion-composition.md).

## Operator-controlled deployment

```bash
./scripts/bootstrap.sh --extra inference
.venv/bin/cadence run --config pkg://cadence/data/a3_operator_stream_demo.yaml
```

This mock deployment starts in damping, accepts PLNJ on `127.0.0.1:50560`, and exposes the upper-target endpoint on `127.0.0.1:15100`. Use PlanetJoystick's `pkg://planetj/data/operator.yaml` profile: request fixed position (RB+A), then locomotion (RB+X). `a3_operator_demo.yaml` selects fixed upper posture instead. Both profiles run without a rally package.

Before operating, `planetj --config pkg://planetj/data/operator.yaml --check-remote` checks the live state bindings without sending commands. The [PlanetJoystick example](https://github.com/Renforce-Dynamics/planetJoystick/tree/main/examples/upper_stream) provides the continuous producer. Joint target receipts confirm reception; the execution kernel commits only after backend acceptance.

## External localization

`runtime.localization` enables the optional `planet.localization.v1` UDP input. It carries robot position, orientation and linear velocity with source identity, coordinate frames, session/sequence ordering and sample freshness. Producers use `planet_protocol.localization.LocalizationClient` from planetConfig without installing Cadence or the SDK.

planetConfig owns the wire contract; Cadence owns its receiver and each state's localization-loss behavior. Cadence also accepts the legacy `cadence.*.v1` schemas for existing clients. Ball trajectories, strike timing and racket targets remain task protocols in rally. See the [localization configuration and producer example](docs/deployment.md#外部定位). Operator expiry supplies zero velocity input; whether a state requires a continuous operator link is an explicit state contract, not a universal emergency-stop rule.

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
