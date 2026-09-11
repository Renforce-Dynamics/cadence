from types import SimpleNamespace
import numpy as np
import pytest
from cadence_api import JointCommand
from cadence.plugins import PluginCatalog, ControlFrame, ControlResult
from cadence.runtime import RuntimeKernel, RuntimeConfig, RuntimeInput
from cadence.backends.mock import MockBackend
from cadence.fsm.hierarchy import HierarchicalMachine, Transition
from cadence.control.composition import CommandPart, compose_commands


def kernel():
    cat = PluginCatalog(
        {
            "reset_state_id": 0,
            "safety_fallback_state_id": 1,
            "states": {
                0: {
                    "key": "passive",
                    "factory": "cadence.plugins:BasicState",
                    "config": {"kind": "passive"},
                },
                1: {
                    "key": "damping",
                    "factory": "cadence.plugins:BasicState",
                    "config": {"kind": "damping"},
                },
                2: {
                    "key": "align",
                    "factory": "cadence.plugins:BasicState",
                    "config": {"kind": "fixed_position", "target": [0, 0]},
                },
            },
        }
    )
    backend = MockBackend(2)
    backend.start()
    state = backend.read_state()
    k = RuntimeKernel(
        RuntimeConfig(
            cat, SimpleNamespace(dimension=2), ControlFrame, [-1, -1], [1, 1]
        ),
        state,
        0.0,
    )
    return k, backend, state


def test_acceptance_gate_and_stale_acknowledgement():
    k, b, s = kernel()
    p = k.current
    calls = []
    p.on_command_applied = lambda cmd: calls.append("accepted")
    p.on_command_rejected = lambda: calls.append("rejected")
    cmd = p.step(ControlFrame(0, s)).command
    p.step = lambda frame: ControlResult(
        cmd, entry_gate_ready=True, next_state="damping"
    )
    k.prepare(RuntimeInput(0.02, s, None))
    ticket = k.pending_ticket
    assert not k._entry_gate_ready and k.current_key == "passive"
    k.reject(ticket)
    assert calls == ["rejected"] and k.current_key == "passive"
    k.prepare(RuntimeInput(0.04, s, None))
    with pytest.raises(RuntimeError, match="stale"):
        k.commit(ticket)
    b.write_command(cmd, s.sequence)
    k.commit()
    assert calls == ["rejected", "accepted"] and k.current_key == "damping"


def test_emergency_wins_over_reset_and_deadline_rejects_skill():
    k, b, s = kernel()
    result = k.tick(RuntimeInput(0.02, s, None, emergency_halt=True, reset_safety=True))
    assert result.safety_halted and np.all(result.command.kp == 0)
    k.tick(RuntimeInput(0.04, s, None, reset_safety=True))
    assert not k.safety.halted
    calls = []
    k.current.on_command_rejected = lambda: calls.append("rollback")
    k.prepare(RuntimeInput(0.06, s, None))
    for _ in range(k.safety.consecutive_deadline_limit):
        k.observe_control_duration(1.0)
    result = k.guard_pending()
    k.commit()
    assert calls == ["rollback"] and result.safety_halted


def test_parent_cancel_prevents_stale_child_event():
    parent = HierarchicalMachine("mode", {"idle", "run"}, "run")
    child = HierarchicalMachine(
        "skill",
        {"ready", "done"},
        "ready",
        [Transition("ready", "finish", "done")],
        parent=parent,
    )
    token = child.activation
    parent.transition("idle")
    assert not child.dispatch("finish", activation=token)
    child.restart("ready")
    assert not child.dispatch("finish", activation=token)
    assert child.dispatch("finish", activation=child.activation)


def test_joint_ownership_cannot_overlap():
    z = np.zeros(3)
    fallback = JointCommand(z, z, z, np.ones(3), z)
    part = CommandPart("arm", (1,), JointCommand([0.2], [0], [10], [1], [0]))
    result = compose_commands([part], fallback)
    assert np.allclose(result.q_des, [0, 0.2, 0]) and result.kd[0] == 1
    with pytest.raises(ValueError, match="claimed"):
        compose_commands([part, part], fallback)
    with pytest.raises(ValueError, match="outside"):
        compose_commands([CommandPart("other", (3,), part.command)], fallback)
