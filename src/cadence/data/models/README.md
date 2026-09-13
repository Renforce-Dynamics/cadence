# A3 lower locomotion actor

Renforce Dynamics maintains this 397-observation, 15-action ONNX actor and its A3 deployment contract. It was relocated byte-for-byte from the extracted deployment bundle (`v1_loco_lower.onnx`). `artifacts.lock` records its SHA-256. The four-frame observation ABI and robot mapping are implemented in `cadence.motion.a3` and configured in `data/motion/a3_lower.yaml`.

The generic motion states do not require this model: another robot supplies its own lower policy adapter and joint configuration. ONNX Runtime is an optional `cadence[inference]` dependency.
