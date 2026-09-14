# External localization

planetConfig owns the common robot-localization wire contract in its standard-library-only `planet-protocol` package. Cadence implements the runtime receiver and state-specific root-loss behavior. A perception producer needs only the shared package; it does not need Cadence, the SDK, a joystick package or Rally. Task goals such as a ball trajectory, striking time and racket target remain task protocol data.

## Enable an endpoint

Compose the optional localization layer with a deployment:

```yaml
extends:
  - pkg://cadence/data/a3_operator_demo.yaml
  - pkg://cadence/data/localization.yaml
runtime:
  localization:
    source: mocap
    host: 127.0.0.1
    port: 15110
    frame_id: world
    child_frame_id: policy_root
    max_age_s: 0.25
    max_datagrams_per_poll: 64
```

The packaged layer contains all defaults shown above, except `source` defaults to `localization`. `runtime.localization: null`, or omitting the field, disables this endpoint. An enabled endpoint is the explicit localization source: missing, invalid or expired external samples produce `None`. The frontend does not silently replace them with backend localization or the Rally planner's embedded root. Without this endpoint, a frontend may use its existing backend or task localization source when valid.

The receiver defaults to loopback. Set a reachable bind address explicitly when the producer is on another host. The sender's destination is the receiver host, while the receiver's `source` is a logical producer name, not a host name. Both frame names and source must match exactly; unexpected values are rejected. Binding the same endpoint twice fails at startup.

Save the example as `external-localization.yaml` in your working directory. The selected A3 catalog includes the lower-body actor, so install the `inference` extra even when starting in damping or using `--check`. From a Cadence checkout, validate and launch its standalone mock deployment with:

```bash
./scripts/bootstrap.sh --extra inference
./scripts/deploy.sh --config ./external-localization.yaml --check
./scripts/deploy.sh --config ./external-localization.yaml --duration-s 5
```

`--check` validates and loads the selected configuration without opening the localization socket or starting a backend. During execution, the initial available root is passed to the initial state's `on_enter`. A generic deployment whose initial state requires world localization fails startup if no fresh root is available; start from `damping` and request the dependent state after localization arrives. Full backend selection and snapshot options are described in [standalone deployment](deployment.md).

Cadence-rally uses the same optional layer in its own entry schema. For example, a Rally simulation overlay can contain:

```yaml
extends:
  - pkg://cadence_rally/data/configs/entry/entry_sim_mixed.yaml
  - pkg://cadence/data/localization.yaml
runtime:
  localization:
    source: mocap
```

Save that overlay as `rally-localization.yaml` and use the **Cadence-rally checkout's** script:

```bash
./scripts/deploy.sh --config ./rally-localization.yaml --backend mujoco --check
./scripts/deploy.sh --config ./rally-localization.yaml --backend mujoco --headless --duration-s 5
```

This process contains the Cadence runtime; no second Cadence process is needed. Its PLNU receiver still carries Rally plans and task targets, while the explicitly selected localization endpoint supplies the world root. The same input setting is available for Rally dry-run and onboard entries. See [Cadence-rally configuration](https://github.com/Renforce-Dynamics/cadence-rally/blob/main/docs/configuration.md#external-localization) for task entry selection and the legacy sources used when localization ingress is disabled.

`max_datagrams_per_poll` limits each nonblocking control-loop poll to 1–1024 datagrams, default 64, including malformed datagrams. The receiver retains only one accepted sample and two ordering counters. Backlog is processed on later polls; this is a bounded packet-work budget, not a hard real-time timing guarantee. A received datagram cannot exceed 8192 bytes.

## Wire contract

Packets are UTF-8 JSON objects with exactly these fields:

```json
{
  "schema": "planet.localization.v1",
  "type": "localization",
  "source": "mocap",
  "session_id": 1790000000000000,
  "sequence": 0,
  "source_timestamp_us": 1790000000001000,
  "source_age_s": 0.005,
  "ttl_s": 0.25,
  "frame_id": "world",
  "child_frame_id": "policy_root",
  "position_w_m": [0.0, 0.0, 0.8],
  "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
  "linear_velocity_w_mps": [0.0, 0.0, 0.0],
  "valid": true
}
```

The canonical schema is `planet.localization.v1`. Cadence also accepts
`cadence.localization.v1` from existing producers, with the same validation and
ordering watermark; switching schema names cannot reset a producer's sequence.
The retained `cadence_protocol` compatibility client uses the legacy name by
default. New producers import `planet_protocol` and use the Planet schema.

| Field | Meaning |
| --- | --- |
| `position_w_m` | Child-frame origin in the configured reference frame, three meters-valued components. |
| `orientation_wxyz` | Unit quaternion rotating child-frame vectors into the reference frame, **w, x, y, z** order. Norm tolerance is 0.001; accepted roundoff is normalized. |
| `linear_velocity_w_mps` | Child-origin linear velocity in that reference frame, three meters-per-second components. |
| `frame_id`, `child_frame_id` | Defaults `world` and `policy_root`. They identify coordinate semantics, not a request to transform coordinates. |
| `source` | Configured logical producer identity. One endpoint accepts one source. |
| `session_id` | Positive, ordered producer epoch. A restarted producer must choose a strictly larger value. |
| `sequence` | Nonnegative integer strictly increasing within a session; no wraparound. |
| `source_timestamp_us` | Unix microseconds at sampling for trace alignment; zero means unavailable. |
| `source_age_s` | Nonnegative sample age at publication, measured on the producer's monotonic clock. |
| `ttl_s` | Positive maximum sample age; the receiver also applies its own `max_age_s`. |
| `valid` | Boolean availability. A newer `false` sample immediately withdraws localization. |

All numeric data must be finite. Integer fields must fit exactly in a JSON binary64 client, at most `2**53 - 1`. Booleans are not accepted as numeric fields. Duplicate keys, missing fields, unknown fields, incorrect vector dimensions and invalid quaternions are rejected. Even an invalidation packet must have structurally valid numeric fields.

Transform a sensor, pelvis or marker pose into the configured policy-root pose **before sending**. Cadence does not infer offsets, axis conventions or quaternion ordering from a producer's name. In particular, changing `child_frame_id` in configuration does not apply a pelvis-to-policy-root offset.

## Freshness and restart behavior

For each accepted sample:

```text
age = source_age_s + max(0, receiver_monotonic_now - received_monotonic_time)
usable = valid and age < min(ttl_s, configured_max_age_s)
```

Receiver monotonic time is never compared with a producer timestamp. The formula does **not** estimate unknown one-way network latency or time already spent waiting in an OS receive queue. Where those delays must be accounted for, establish bounded transport delays or synchronized timestamp handling upstream. Do not label this protocol as network-delay compensation.

Duplicate or lower sequences and all older sessions are rejected, including after the current sample expires. A new session may start at sequence zero. `LocalizationClient` defaults to a Unix-microsecond epoch and ensures increasing epochs within one process. Across producer processes, persist `max(previous_epoch + 1, current_epoch)` when the wall clock may move backwards. A clock rollback must not silently be treated as a new session; a deliberate receiver restart is an alternative when re-establishing a deployment session.

A newer invalid or already expired sample replaces the previous sample and returns `None` immediately. Malformed packets and identity mismatches leave the previous sample available only until its own expiry. Empty polls continue checking age, so a disconnected sender cannot keep localization fresh indefinitely.

The runtime state owns what happens next. `HOLD_LAST` can continue with a previously valid root, while `FALLBACK` can hold it for the state's configured number of control steps and then transition to its fallback. States without an available current or permitted cached root cannot enter a mode requiring world localization. State-specific recovery and transition rules remain authoritative; receipt of new localization does not automatically select a mode or reset safety. The ingress never reports a cached, expired root as fresh.

This policy is separate from upper-joint targets: the upper-target mailbox intentionally retains its latest command throughout an activation, even after disconnect.

## Publish measured localization

```python
import time
from planet_protocol.localization import LocalizationClient

# Capture this alongside the actual sensor sample, before processing it.
sampled_monotonic = time.monotonic()
sampled_unix_us = time.time_ns() // 1000
position = (0.0, 0.0, 0.8)
orientation = (1.0, 0.0, 0.0, 0.0)
velocity = (0.0, 0.0, 0.0)

with LocalizationClient("127.0.0.1", 15110, source="mocap") as producer:
    producer.send(
        position, orientation, velocity,
        source_age_s=time.monotonic() - sampled_monotonic,
        source_timestamp_us=sampled_unix_us,
    )
```

Keep one client for a continuous producer so its sequence increases within one session. Each send consumes a new sequence, including a send attempt that fails. `send()` returning successfully only means the local UDP operation succeeded; there is no receiver, state-transition or physical-execution acknowledgement. Network failures raise `OSError`; invalid inputs raise `ValueError`. Closing the client closes its socket without publishing a fallback sample.

## Run the synthetic sender

From a Cadence checkout with `planet-protocol` installed in `.venv`, send one explicit pose:

```bash
.venv/bin/python scripts/send-localization.py \
  --source mocap --position-m 0 0 0.8
```

Both timing flags are required for a finite continuous fixture:

```bash
.venv/bin/python scripts/send-localization.py \
  --source mocap --position-m 0 0 0.8 --duration-s 5 --hz 50
```

The script generates fresh synthetic samples from the specified constant pose and velocity; it is an integration fixture, not a sensor driver. On normal exit or Ctrl-C it stops publishing. The receiver then expires the last sample according to its TTL. To explicitly withdraw availability immediately:

```bash
.venv/bin/python scripts/send-localization.py --source mocap --invalid
```

The script accepts `--host`, `--port`, `--frame-id`, `--child-frame-id`, `--orientation-wxyz`, `--velocity-mps`, `--ttl-s` and an optional persisted `--session-id`. It imports only the lightweight protocol package and the Python standard library.
