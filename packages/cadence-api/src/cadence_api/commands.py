from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True, slots=True)
class JointCommand:
    q_des: np.ndarray
    dq_des: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    tau_ff: np.ndarray

    def __post_init__(self) -> None:
        for name in ("q_des", "dq_des", "kp", "kd", "tau_ff"):
            value = np.asarray(getattr(self, name), dtype=np.float64).reshape(-1)
            if value.shape != (len(np.asarray(self.q_des).reshape(-1)),) or not np.all(
                np.isfinite(value)
            ):
                raise ValueError(
                    f"{name} must contain {len(np.asarray(self.q_des).reshape(-1))} finite values"
                )
            if value.size == 0 or (name in {"kp", "kd"} and np.any(value < 0)):
                raise ValueError(f"invalid {name}")
            copied = value.copy()
            copied.setflags(write=False)
            object.__setattr__(self, name, copied)
