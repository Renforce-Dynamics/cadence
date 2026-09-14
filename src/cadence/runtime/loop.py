"""Shared control-cycle submission and monotonic backend read scheduling."""

from dataclasses import dataclass
from typing import Any
import time


@dataclass(frozen=True, slots=True)
class CycleResult:
    output: Any
    control_duration_s: float
    write_duration_s: float
    submitted: bool
    shadow: bool


def read_control_state(
    io, next_tick_s, timeout_s, *, monotonic=time.monotonic, sleep=time.sleep,
):
    """Wait for the policy deadline, then wait for a fresh backend snapshot.

    Scheduling delay is separate from the SDK's timeout for new state. Sensor
    timestamps never serve as the control loop's scheduling clock.
    """
    delay = float(next_tick_s) - monotonic()
    if delay > 0:
        sleep(delay)
    return io.read_state(timeout_s=timeout_s)


def execute_cycle(runtime, backend, value, *, read_only=False):
    """Prepare, guard and submit one command, then advance accepted progress.

    Read-only operation advances an explicitly marked shadow evaluation without
    calling the backend writer. Every failed cycle rejects its pending state;
    callers may handle a backend's stale-state exception without committing it.
    """
    started = time.perf_counter()
    try:
        output = runtime.prepare(value)
        control_duration = time.perf_counter() - started
        runtime.observe_control_duration(control_duration)
        output = runtime.guard_pending()
        write_duration = 0.0
        if not read_only:
            write_started = time.perf_counter()
            backend.write_command(output.command, value.robot_state.sequence)
            write_duration = time.perf_counter() - write_started
        output = runtime.commit()
    except BaseException:
        runtime.reject()
        raise
    return CycleResult(output, control_duration, write_duration, not read_only, bool(read_only))
