from dataclasses import dataclass
from pathlib import Path
from collections.abc import Mapping
import numpy as np


@dataclass(frozen=True, slots=True)
class OnnxRuntimeOptions:
    """Per-state ONNX Runtime scheduler settings."""

    intra_op_threads: int = 1
    inter_op_threads: int = 1
    execution_mode: str = "sequential"
    allow_spinning: bool = False

    def __post_init__(self) -> None:
        if isinstance(self.intra_op_threads, bool) or self.intra_op_threads <= 0:
            raise ValueError("intra_op_threads must be positive")
        if isinstance(self.inter_op_threads, bool) or self.inter_op_threads <= 0:
            raise ValueError("inter_op_threads must be positive")
        if self.execution_mode not in {"sequential", "parallel"}:
            raise ValueError("execution_mode must be sequential or parallel")
        if not isinstance(self.allow_spinning, bool):
            raise ValueError("allow_spinning must be boolean")

    @classmethod
    def from_mapping(cls, raw: object) -> "OnnxRuntimeOptions":
        if not isinstance(raw, Mapping) or set(raw) != {
            "intra_op_threads",
            "inter_op_threads",
            "execution_mode",
            "allow_spinning",
        }:
            raise ValueError(
                "policy.runtime requires intra_op_threads, inter_op_threads, "
                "execution_mode and allow_spinning"
            )
        return cls(
            intra_op_threads=int(raw["intra_op_threads"]),
            inter_op_threads=int(raw["inter_op_threads"]),
            execution_mode=str(raw["execution_mode"]).strip().lower(),
            allow_spinning=raw["allow_spinning"],
        )


class OnnxPolicy:
    def __init__(
        self,
        model_path: str | Path,
        observation_dimension: int | None,
        action_dimension: int = 1,
        runtime_options: OnnxRuntimeOptions | None = None,
    ) -> None:
        try:
            import onnxruntime as ort
        except ImportError as error:
            raise RuntimeError(
                "onnxruntime is required for policy inference"
            ) from error
        path = Path(model_path)
        if not path.is_file():
            raise FileNotFoundError(path)
        options = runtime_options or OnnxRuntimeOptions()
        session_options = ort.SessionOptions()
        session_options.intra_op_num_threads = options.intra_op_threads
        session_options.inter_op_num_threads = options.inter_op_threads
        session_options.execution_mode = (
            ort.ExecutionMode.ORT_SEQUENTIAL
            if options.execution_mode == "sequential"
            else ort.ExecutionMode.ORT_PARALLEL
        )
        spinning = "1" if options.allow_spinning else "0"
        session_options.add_session_config_entry(
            "session.intra_op.allow_spinning", spinning
        )
        session_options.add_session_config_entry(
            "session.inter_op.allow_spinning", spinning
        )
        self.runtime_options = options
        self.session = ort.InferenceSession(
            str(path), sess_options=session_options, providers=["CPUExecutionProvider"]
        )
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.action_dimension = int(action_dimension)
        shape = self.session.get_inputs()[0].shape
        if (
            not isinstance(shape, list)
            or len(shape) != 2
            or shape[0] != 1
            or not isinstance(shape[1], int)
            or shape[1] <= 0
        ):
            raise RuntimeError(f"ONNX input must have static shape [1,D], got {shape}")
        self.observation_dimension = int(shape[1])
        if (
            observation_dimension is not None
            and self.observation_dimension != observation_dimension
        ):
            raise RuntimeError(
                f"ONNX input must be [1,{observation_dimension}], got {shape}"
            )

    def infer(self, observation) -> np.ndarray:
        obs = np.asarray(observation, dtype=np.float32).reshape(1, -1)
        output = self.session.run([self.output_name], {self.input_name: obs})[0]
        action = np.asarray(output, dtype=np.float32).reshape(-1)
        if action.shape != (self.action_dimension,) or not np.all(np.isfinite(action)):
            raise RuntimeError(f"invalid ONNX action shape/value: {action.shape}")
        return action
