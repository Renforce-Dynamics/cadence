"""Deterministic in-memory backend for arbitrary joint layouts."""

import numpy as np
from cadence_api import RobotState


class MockBackend:
    def __init__(self, dimension=2, dt=0.02):
        if dimension < 1 or dt <= 0:
            raise ValueError("invalid mock dimensions/timestep")
        self.q = np.zeros(dimension)
        self.dq = self.q.copy()
        self.dt = dt
        self.sequence = 1
        self.time = 0.0
        self.started = False

    def start(self):
        self.started = True

    def close(self):
        self.started = False

    def read_state(self, timeout_s=None):
        if not self.started:
            raise RuntimeError("backend is not started")
        return RobotState(
            self.sequence,
            round(self.time * 1e9),
            np.zeros(3),
            np.array([1.0, 0.0, 0.0, 0.0]),
            self.q,
            self.dq,
            np.zeros_like(self.q),
        )

    def write_command(self, command, state_sequence):
        if not self.started or state_sequence != self.sequence:
            raise RuntimeError("stale command or stopped backend")
        if len(command.q_des) != len(self.q):
            raise ValueError("command dimension mismatch")
        force = (
            command.kp * (command.q_des - self.q)
            + command.kd * (command.dq_des - self.dq)
            + command.tau_ff
        )
        self.dq += force * self.dt
        self.q += self.dq * self.dt
        self.time += self.dt
        self.sequence += 1
