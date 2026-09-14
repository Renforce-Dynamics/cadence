# cadence-protocol

**Cadence wire contracts and lightweight clients.**

A component of [cadence](https://github.com/Renforce-Dynamics/cadence), developed and maintained by Renforce Dynamics. This package uses only the Python standard library; installing it does not install the control runtime, NumPy or a robot SDK.

- `cadence_protocol.operator`: deployed PLNJ v1/v2/v3 decoding and v3 encoding.
- `cadence_protocol.client.OperatorClient`: discover state bindings and query runtime status over the operator UDP port.
- `cadence_protocol.client.JointTargetClient`: query an activation and submit joint positions in radians.

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

`JointTargetClient.send(activation, sequence, q_des)` returns a receipt dictionary. An `accepted: true` receipt means the receiver stored the target; it does not confirm backend submission or physical motion. The client checks that the receipt identifies the submitted activation and sequence. It does not choose activations, increment sequences, retry targets or send a default posture when closing. The current joint-target endpoint is loopback only.

Both clients support `with` and `close()`. Input and response validation failures raise `ValueError`; network failures raise `OSError`, including `socket.timeout`. Keep one outstanding request per client. Installing the legacy `planetj-protocol` package continues to expose the same PLNJ classes and functions through compatibility re-exports.

See [AUTHORS.md](AUTHORS.md). Licensed under the [MIT License](LICENSE).
