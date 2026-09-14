# cadence-protocol

**Cadence wire contracts and lightweight clients.**

A component of [cadence](https://github.com/Renforce-Dynamics/cadence), developed and maintained by Renforce Dynamics. This package uses only the Python standard library; installing it does not install the control runtime, NumPy or a robot SDK.

- `cadence_protocol.operator`: deployed PLNJ v1/v2/v3 decoding and v3 encoding.
- `cadence_protocol.client.OperatorClient`: discover state bindings and query runtime status over the operator UDP port.
- `cadence_protocol.client.JointTargetClient`: query an activation and submit joint positions in radians.
- `cadence_protocol.localization`: versioned localization samples, strict JSON encoding/decoding and a one-way UDP producer (`LocalizationClient`, also exported from `cadence_protocol.client`).

```python
from cadence_protocol.client import OperatorClient, JointTargetClient

with OperatorClient("127.0.0.1", 50560) as operator:
    operator.validate_bindings([(0, "passive"), (1, "damping"), (3, "loco")])
    print(operator.status())

with JointTargetClient("127.0.0.1", 15100) as targets:
    status = targets.status()
    # A producer supplies q_des in the configured joint order and dimension.
    # Capture activation once per motion and increase sequence for each frame.
```

Operator `describe` and `status` queries share the binary PLNJ UDP endpoint using JSON schema `cadence.operator.v1`. Queries do not request state changes. `validate_bindings()` checks configured IDs against the receiver's canonical state keys or aliases and returns its description.

Deployment status adds `execution: backend` for accepted backend cycles or
`execution: shadow` for readonly/dry-run evaluation. Shadow progress does not
indicate robot execution. This optional field preserves compatibility with
older receivers; a startup status may precede the first command in either mode.

`JointTargetClient.send(activation, sequence, q_des)` returns a receipt dictionary. An `accepted: true` receipt means the receiver stored the target; it does not confirm backend submission or physical motion. The client checks that the receipt identifies the submitted activation and sequence. It does not choose activations, increment sequences, retry targets or send a default posture when closing. The current joint-target endpoint is loopback only.

Both clients support `with` and `close()`. Input and response validation failures raise `ValueError`; network failures raise `OSError`, including `socket.timeout`. Keep one outstanding request per client. Installing the legacy `planetj-protocol` package continues to expose the same PLNJ classes and functions through compatibility re-exports.

## External localization

```python
from cadence_protocol.localization import LocalizationClient

with LocalizationClient("127.0.0.1", 15110, source="mocap") as publisher:
    # Supply the measured policy-root pose, not an unrelated sensor/body frame.
    sample = publisher.send(
        position_w_m=(0.0, 0.0, 0.8),
        orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        linear_velocity_w_mps=(0.0, 0.0, 0.0),
        source_age_s=0.005,
        source_timestamp_us=0,  # Unknown sample timestamp; used for traces only.
    )
```

`cadence.localization.v1` carries positions in meters, linear velocities in meters per second, and a unit quaternion in **wxyz** order. The quaternion rotates `child_frame_id` vectors into `frame_id`; defaults are `policy_root` and `world`. The configured source and both frame names must match the receiver exactly. No implicit coordinate conversion is performed.

The producer assigns an ordered session epoch and increases sequence numbers. A restarted producer must use a strictly newer epoch: the default uses Unix microseconds and is monotonic within a process. Persist the epoch if the producer's wall clock can move backwards between processes. A `valid=False` sample immediately withdraws localization when accepted. The return from `send()` records a local UDP send, not receiver or robot acknowledgement.

Freshness is `source_age_s + receiver monotonic elapsed time`, limited by both the sample TTL and the receiver's configured maximum age. Unknown one-way network or OS queue delay is not measured. `source_timestamp_us` is a trace field, never compared with receiver monotonic time. Localization expires on disconnect; any subsequent hold or fallback is owned by the consuming state. This is separate from the persistent upper-joint target contract.

See the complete [localization contract and runnable example](../../docs/localization.md).

See [AUTHORS.md](AUTHORS.md). Licensed under the [MIT License](LICENSE).
