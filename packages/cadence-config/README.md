# cadence-config

Compatibility imports for the shared [`planet-config`](https://github.com/Renforce-Dynamics/planetConfig) library, maintained by Renforce Dynamics.

The canonical YAML loader, resource resolution, provenance and snapshot implementation live in the independent planetConfig repository. New consumers should install `planet-config` and import `planet_config`:

```python
from planet_config import load_config

config = load_config("site.yaml")
config.freeze("runs/config")
```

Existing Cadence applications may keep `from cadence_config import ...`. This package forwards those public APIs to `planet_config`; it does not maintain another loader. Planet components depend directly on planetConfig and do not install this compatibility package, Cadence or the SDK.

See the [shared repository](https://github.com/Renforce-Dynamics/planetConfig) for library usage and [Cadence configuration](../../docs/configuration.md) for execution-specific fields. Licensed under MIT.
