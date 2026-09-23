# A3 lower locomotion actor

Renforce Dynamics maintains this 397-observation, 15-action ONNX actor and its A3 deployment contract. It was relocated byte-for-byte from the extracted deployment bundle (`v1_loco_lower.onnx`). `artifacts.lock` records its SHA-256. The four-frame observation ABI and robot mapping are implemented in `cadence.motion.a3` and configured in `configs/states/a3_lower.yaml` at the repository root.

The generic motion states do not require this model: another robot supplies its own lower policy adapter and joint configuration. ONNX Runtime is an optional `cadence[inference]` dependency.

The model lives in this repository directory and is not included in the Python wheel. The state configuration resolves `../../models/a3_loco_lower.onnx` relative to its own file. Task repositories reuse that file through their pinned Cadence checkout.

## H32 EstMoE variant

`a3_loco_lower_estmoe_h32.onnx` is a 1635-observation, 15-action lower-only EstMoE actor (32-frame term-major history, 3-value velocity command, no upper-body inputs), trained with random arm PD disturbances and enhanced domain randomization. Its contract is implemented by `LowerVelocityHistoryObservationBuilder` and `A3LowerEstMoEPolicy` in `cadence.motion.a3_lower`; task configurations select it via `lower.factory` and `history_frames: 32`. Source checkpoint `model_015000.pt` (SHA-256 `26252b1d40f969bde9606ae8524dd96c61f655043b60b013c78604194ba71cc5`) from the iKobe `distill-estmoe-h32-upper-pd-v1` run. MLP-based lower policies are not deployable and were removed.
