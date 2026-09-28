"""Operator-visible runtime messages, shared by hardware and simulation."""
from queue import Empty, Full, Queue
import sys
import threading


class ConsoleWriter:
    """Keep terminal/file writes off the control thread, with bounded memory."""

    def __init__(self, stream=None):
        self.stream = sys.stderr if stream is None else stream
        self.queue = Queue(maxsize=256)
        self.stopping = threading.Event()
        self.dropped = 0
        self.worker = threading.Thread(target=self._run, name="cadence-console", daemon=True)
        self.worker.start()

    def emit(self, line):
        try:
            if self.dropped:
                line = f"[cadence][LOG] dropped {self.dropped} messages while output was slow\n{line}"
            self.queue.put_nowait(line)
            self.dropped = 0
        except Full:
            self.dropped += 1

    def _run(self):
        while not self.stopping.is_set() or not self.queue.empty():
            try:
                line = self.queue.get(timeout=.05)
            except Empty:
                continue
            try:
                print(line, file=self.stream, flush=True)
            except (OSError, ValueError):
                # A closed log consumer must not interrupt robot control.
                return

    def close(self):
        self.stopping.set()
        # A blocked pipe cannot delay backend shutdown; this is called after IO
        # cleanup, and the writer is a daemon if its consumer never resumes.
        self.worker.join(timeout=.2)


class RuntimeConsole:
    """Print edges immediately and a one-second state/trajectory heartbeat."""

    def __init__(self, catalog, backend, shadow, emit):
        self.catalog, self.backend, self.emit = catalog, backend, emit
        self.execution = "shadow" if shadow else "backend"
        self._request = self._link = self._status = None
        self._dpad = (0, 0)
        self._signals = (False, False)
        self._events = set()
        self._next_status_s = 0.

    def message(self, now_s, kind, text):
        self.emit(f"[cadence:{self.backend}][{now_s:8.3f}s][{kind}] {text}")

    def start(self, state, *, operator_address=None):
        motion = getattr(state, "selected_motion", None) or "-"
        self.message(0., "START", f"execution={self.execution} state={state.state_id}:{state.key.upper()} "
                     f"trajectory={motion} operator={operator_address or 'disabled'}")

    def observe_input(self, sample, now_s):
        connected = sample.operator_link_usable
        if connected != self._link:
            self.message(now_s, "OPERATOR", "connected" if connected else "disconnected/stale")
            self._link = connected
        packet = sample.operator_input
        request = None if sample.requested_state is None else (
            getattr(packet, "session_id", None), sample.requested_state)
        if request is not None and request != self._request:
            try:
                key = self.catalog.canonical_key(sample.requested_state).upper()
            except ValueError:
                key = "UNKNOWN"
            self.message(now_s, "COMMAND", f"request={sample.requested_state}:{key}")
        self._request = request
        dpad = (0, 0) if packet is None else (packet.dpad_x, packet.dpad_y)
        if dpad != self._dpad and dpad != (0, 0):
            direction = " ".join(name for value, name in (
                (dpad[0] > 0, "RIGHT"), (dpad[0] < 0, "LEFT"),
                (dpad[1] > 0, "UP"), (dpad[1] < 0, "DOWN")) if value)
            self.message(now_s, "COMMAND", f"dpad={direction}")
        self._dpad = dpad
        signals = (sample.emergency_halt, sample.reset_safety)
        for index, name in enumerate(("emergency_halt", "reset_safety")):
            if signals[index] and not self._signals[index]:
                self.message(now_s, "COMMAND", name)
        self._signals = signals

    def observe_output(self, output, state, now_s, *, entry_gate_ready, velocity=(0., 0., 0.)):
        current_events = set(output.events)
        for event in output.events:
            if event not in self._events:
                self.message(now_s, "EVENT", event)
        self._events = current_events
        motion = getattr(state, "selected_motion", None)
        status = (output.mode, output.skill_state, motion, entry_gate_ready,
                  output.safety_halted, output.safety_reason)
        if status != self._status or now_s >= self._next_status_s:
            text = (f"state={state.state_id}:{output.mode} phase={output.skill_state} "
                    f"trajectory={motion or '-'} gate={entry_gate_ready} "
                    f"halted={output.safety_halted} execution={self.execution} "
                    f"velocity=({velocity[0]:+.2f},{velocity[1]:+.2f},{velocity[2]:+.2f})")
            if output.safety_reason:
                text += f" reason={output.safety_reason}"
            self.message(now_s, "STATUS", text)
            self._next_status_s = now_s + 1.
            self._status = status
