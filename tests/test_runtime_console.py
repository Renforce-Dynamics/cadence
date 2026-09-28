"""Logs expose committed state/selection without flooding or blocking control."""
from io import StringIO
import threading
from types import SimpleNamespace

from cadence.runtime.console import ConsoleWriter, RuntimeConsole


def sample(request=None, dpad=(0, 0), *, session=1, connected=True):
    return SimpleNamespace(
        requested_state=request, operator_link_usable=connected,
        operator_input=SimpleNamespace(session_id=session, dpad_x=dpad[0], dpad_y=dpad[1]) if connected else None,
        emergency_halt=False, reset_safety=False,
    )


def output(*, events=(), phase="READY"):
    return SimpleNamespace(mode="SONIC_CLIP", skill_state=phase, events=events,
                           safety_halted=False, safety_reason="")


def test_held_commands_are_logged_once_and_release_or_new_session_relogs():
    lines = []
    console = RuntimeConsole(SimpleNamespace(canonical_key=lambda _: "sonic_clip"), "mujoco", False, lines.append)
    for tick in range(50):
        console.observe_input(sample(6, (1, 0)), tick*.02)
    assert sum("request=6:SONIC_CLIP" in line for line in lines) == 1
    assert sum("dpad=RIGHT" in line for line in lines) == 1
    console.observe_input(sample(), 1.)
    console.observe_input(sample(6, (1, 0)), 1.02)
    console.observe_input(sample(6, (1, 0), session=2), 1.04)
    assert sum("request=6:SONIC_CLIP" in line for line in lines) == 3
    assert sum("dpad=RIGHT" in line for line in lines) == 2
    console.observe_input(sample(connected=False), 1.06)
    assert "disconnected/stale" in lines[-1]


def test_state_motion_changes_and_events_are_immediate_with_one_second_heartbeat():
    lines = []
    console = RuntimeConsole(None, "a3", True, lines.append)
    state = SimpleNamespace(state_id=6, key="sonic_clip", selected_motion="stand")
    console.start(state)
    for tick in range(50):
        console.observe_output(output(events=("rejected request",)), state, tick*.02, entry_gate_ready=False)
    assert sum("[STATUS]" in line for line in lines) == 1
    assert sum("[EVENT] rejected request" in line for line in lines) == 1
    console.observe_output(output(), state, 1., entry_gate_ready=False)
    assert sum("[STATUS]" in line for line in lines) == 2
    state.selected_motion = "dribble"
    console.observe_output(output(events=("sonic_clip_selected:dribble",)), state, 1.02, entry_gate_ready=False)
    assert "trajectory=dribble" in lines[-1]
    console.observe_output(output(phase="CUE"), state, 1.04, entry_gate_ready=False)
    assert "phase=CUE" in lines[-1] and "execution=shadow" in lines[-1]


def test_slow_console_has_bounded_queue_and_never_blocks_publisher():
    blocked, release, published = threading.Event(), threading.Event(), threading.Event()

    class SlowStream(StringIO):
        def write(self, text):
            blocked.set()
            release.wait(2.)
            return super().write(text)

    writer = ConsoleWriter(SlowStream())
    writer.emit("first")
    assert blocked.wait(1.)
    def publish():
        for _ in range(300):
            writer.emit("held input")
        published.set()
    producer = threading.Thread(target=publish, daemon=True)
    try:
        producer.start()
        assert published.wait(.5)
        assert writer.queue.qsize() <= 256
        assert writer.dropped > 0
    finally:
        release.set()
        producer.join(1.)
        writer.close()
    assert "first" in writer.stream.getvalue()
