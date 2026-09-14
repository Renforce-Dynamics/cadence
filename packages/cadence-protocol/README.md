# cadence-protocol

**Compatibility imports for existing Cadence applications.**

The canonical protocol implementation is the independent [`planet-protocol`](https://github.com/Renforce-Dynamics/planetConfig/tree/main/packages/planet-protocol) distribution in the planetConfig repository. This package contains lightweight wrappers only. Both distributions use the Python standard library and install no robot runtime or SDK.

New producers should import `planet_protocol`. Existing callers retain these imports and deployed schema defaults:

| Compatibility API | Behavior |
| --- | --- |
| `cadence_protocol.operator` | Direct PLNJ re-exports; packet types, binary bytes, CRC and v1/v2 decoding are identical to `planet_protocol.operator`. |
| `cadence_protocol.client.OperatorClient` | Uses the legacy `cadence.operator.v1` query schema. |
| `cadence_protocol.client.JointTargetClient` | Uses the legacy `cadence.joint-target.v1` schema. |
| `cadence_protocol.localization` | Uses `cadence.localization.v1` for legacy encoding, decoding and the producer client. |
| `cadence_protocol.client.validate_operator_bindings` | Direct re-export of the independent, network-free binding validator. |

`LocalizationClient` is also exported from `cadence_protocol.client`. Localization samples and PLNJ packet classes retain object identity across namespaces, so mixed imports do not break type checks.

```python
from cadence_protocol.client import OperatorClient

with OperatorClient("127.0.0.1", 50560) as operator:
    operator.validate_bindings([(0, "passive"), (1, "damping")])
    print(operator.status())
```

Cadence receivers accept both the new `planet.*.v1` schemas and the deployed `cadence.*.v1` aliases. Responses echo the request's schema. Both aliases share one state catalog/status snapshot, one upper-target activation/mailbox and one localization session/sequence watermark. Switching schema does not reset ordering or revive an expired sample.

Protocol semantics are unchanged: read-only operator queries do not request transitions; an upper-target receipt acknowledges mailbox reception, not robot execution; upper targets remain held throughout their activation; localization expires using source sample age plus receiver monotonic elapsed time. Localization invalidation and any state-specific recovery remain separate from upper-target retention.

The compatibility localization decoder accepts its legacy schema only. Use `planet_protocol.localization.decode_localization(..., schema=(...))` when an application explicitly needs multiple aliases. Compatibility clients preserve their former default schema rather than silently changing existing wire traffic.

See the [independent protocol reference](https://github.com/Renforce-Dynamics/planetConfig/tree/main/packages/planet-protocol) for units, fields, timestamps, ordered producer epochs and client examples. Runtime-specific receiver settings remain in [Cadence localization](../../docs/localization.md).

Developed and maintained by Renforce Dynamics. See [AUTHORS.md](AUTHORS.md) and [MIT License](LICENSE).
