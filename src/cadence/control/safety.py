"""Latched runtime safety supervisor, independent of operator FSM modes."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from cadence_api import JointCommand


@dataclass(slots=True)
class SafetySupervisor:
    deadline_s: float
    consecutive_deadline_limit: int = 5
    halted: bool = False
    reason: str = ""
    consecutive_deadline_misses: int = 0
    total_deadline_misses: int = 0

    def halt(self, reason: str) -> None:
        if not self.halted:
            self.reason = str(reason)
        self.halted = True

    def reset(self) -> None:
        self.halted = False
        self.reason = ""
        self.consecutive_deadline_misses = 0

    def validate_state(self, *values) -> bool:
        if not all(np.all(np.isfinite(np.asarray(value))) for value in values):
            self.halt("non-finite robot state")
            return False
        return True

    def validate_command(self, command: JointCommand) -> bool:
        values = (command.q_des, command.dq_des, command.kp, command.kd, command.tau_ff)
        if not all(np.all(np.isfinite(value)) for value in values):
            self.halt("non-finite joint command")
            return False
        return True

    def observe_control_duration(self, duration_s: float) -> None:
        if duration_s > self.deadline_s:
            self.total_deadline_misses += 1
            self.consecutive_deadline_misses += 1
            if self.consecutive_deadline_misses >= self.consecutive_deadline_limit:
                self.halt(
                    f"control deadline missed {self.consecutive_deadline_misses} times "
                    f"({duration_s * 1000:.1f} ms > {self.deadline_s * 1000:.1f} ms)"
                )
        else:
            self.consecutive_deadline_misses = 0
