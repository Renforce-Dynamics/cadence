from types import SimpleNamespace
from dataclasses import replace
import numpy as np
import pytest
from cadence_api import JointCommand
from cadence_api.plugins import RootLossMode
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


def test_handoff_requires_release_before_held_source_request_can_restart():
    k, backend, state = kernel()
    try:
        source = k.current
        command = source.step(ControlFrame(0, state)).command
        source.step = lambda frame: ControlResult(command, next_state="damping")
        assert k.tick(RuntimeInput(.02, state, None, requested_state="passive")).mode == "DAMPING"
        source.step = lambda frame: ControlResult(command)
        assert k.tick(RuntimeInput(.04, state, None, requested_state="passive")).mode == "DAMPING"
        k.tick(RuntimeInput(.06, state, None, operator_link_usable=False))
        assert k.tick(RuntimeInput(.08, state, None, requested_state="passive")).mode == "DAMPING"
        # A real neutral input releases it; a subsequent request is a new play.
        k.tick(RuntimeInput(.10, state, None))
        assert k.tick(RuntimeInput(.12, state, None, requested_state="passive")).mode == "PASSIVE"
        # The latch never prevents explicit manual fixedpos/damping takeover.
        source.step = lambda frame: ControlResult(command, next_state="damping")
        k.tick(RuntimeInput(.14, state, None))
        assert k.tick(RuntimeInput(.16, state, None, requested_state="align")).mode == "ALIGN"
    finally:
        backend.close()


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


@pytest.mark.parametrize("requested", [1, 2, "damping", "align"])
@pytest.mark.parametrize("halted", [False, True])
def test_manual_override_preempts_source_link_root_loss_and_safety_latch(requested, halted):
    k, backend, state = kernel()
    try:
        # The source would latch on link loss AND request a localization
        # fallback on this tick. Neither may consume the manual takeover.
        source = k.current
        source.requires_operator_link = True
        source.root_loss_mode = RootLossMode.FALLBACK
        source.root_loss_exit_steps = 1
        source.root_loss_fallback_state = "passive"
        if halted:
            k.safety.halt("previous policy failure")
        result = k.tick(RuntimeInput(.02, state, None, requested_state=requested,
                                     operator_link_usable=False))
        target = "DAMPING" if requested in (1, "damping") else "ALIGN"
        assert result.mode == target
        assert not result.safety_halted
        assert np.all(result.command.kp == (0 if target == "DAMPING" else 20))
        assert np.all(result.command.kd > 0)
        assert not k.entry_gate_ready
        assert k._root_loss_blocked_mode is None
    finally:
        backend.close()


def test_fixedpos_takeover_bounds_the_ramp_and_does_not_restart_on_held_request():
    k, backend, state = kernel()
    try:
        state = replace(state, joint_pos=np.array([1.1, -1.1]))
        entered = k.tick(RuntimeInput(.02, state, None, requested_state="align"))
        assert not entered.safety_halted
        np.testing.assert_allclose(entered.command.q_des, [1, -1])
        middle = k.tick(RuntimeInput(.52, state, None, requested_state="align"))
        np.testing.assert_allclose(middle.command.q_des, [.55, -.55])
        finished = k.tick(RuntimeInput(1.02, state, None, requested_state="align"))
        np.testing.assert_allclose(finished.command.q_des, [0, 0])
        assert not finished.safety_halted
        # The measured pose is deliberately not converged. Damping still wins.
        assert not k.entry_gate_ready
        damping = k.tick(RuntimeInput(1.04, state, None, requested_state="damping"))
        assert damping.mode == "DAMPING" and not damping.safety_halted
        assert np.all(damping.command.kp == 0)
    finally:
        backend.close()


def test_fixedpos_can_reenter_after_a_halt_from_the_current_measured_pose():
    k, backend, state = kernel()
    try:
        k.tick(RuntimeInput(.02, state, None, requested_state="align"))
        k.safety.halt("previous control deadline")
        state = replace(state, joint_pos=np.array([.4, -.4]))
        recovered = k.tick(RuntimeInput(2., state, None, requested_state="align"))
        assert not recovered.safety_halted
        np.testing.assert_allclose(recovered.command.q_des, state.joint_pos)
        assert k.current.start == 2.
        # A concurrent emergency signal still commands zero stiffness.
        emergency = k.tick(RuntimeInput(2.02, state, None, requested_state="align",
                                        emergency_halt=True))
        assert emergency.safety_halted
        assert np.all(emergency.command.kp == 0)
    finally:
        backend.close()


def test_physical_reset_discards_pending_work_and_restarts_the_same_mode():
    k, backend, state = kernel()
    try:
        k.tick(RuntimeInput(.02, state, None, requested_state="align"))
        calls = []
        k.current.on_command_rejected = lambda: calls.append("rejected")
        k.prepare(RuntimeInput(.5, state, None))
        ticket = k.pending_ticket
        k.safety.halt("old policy failure")
        k._root_loss_blocked_mode = "align"
        fresh = replace(state, joint_pos=np.array([.2, -.2]))
        k.reset_from_feedback(fresh, 1.)
        assert calls == ["rejected"]
        assert k.current_key == "align"
        assert k.pending_ticket is None and not k.safety.halted
        assert not k.entry_gate_ready and k._root_loss_blocked_mode is None
        prepared = k.prepare(RuntimeInput(1., fresh, None))
        np.testing.assert_allclose(prepared.command.q_des, fresh.joint_pos)
        with pytest.raises(RuntimeError, match="stale"):
            k.commit(ticket)
        assert k.commit().mode == "ALIGN"
    finally:
        backend.close()


def test_handoff_does_not_publish_source_substate_as_destination_readiness():
    k, backend, state = kernel()
    try:
        source = k.current
        command = source.step(ControlFrame(0, state)).command
        source.step = lambda frame: ControlResult(command, substate="HOLD", entry_gate_ready=True, next_state="damping")
        prepared = k.prepare(RuntimeInput(.02, state, None))
        assert (prepared.mode, prepared.skill_state) == ("PASSIVE", "HOLD")
        backend.write_command(prepared.command, state.sequence)
        entered = k.commit()
        assert (entered.mode, entered.skill_state) == ("DAMPING", "ENTERING")
        assert k.entry_gate_ready is False

        state = backend.read_state()
        candidate = k.prepare(RuntimeInput(.04, state, None))
        assert candidate.skill_state == "READY"
        k.reject()
        # A rejected candidate must not change the last committed status.
        assert entered.skill_state == "ENTERING"
        accepted = k.prepare(RuntimeInput(.06, state, None))
        backend.write_command(accepted.command, state.sequence)
        accepted = k.commit()
        assert (accepted.mode, accepted.skill_state) == ("DAMPING", "READY")
        assert k.entry_gate_ready is False
    finally:
        backend.close()


def test_entry_gate_property_only_reports_committed_progress_and_convergence():
    k, backend, state = kernel()
    try:
        pending = k.prepare(RuntimeInput(.02, state, None, requested_state="align"))
        assert "entry_gate_ready" not in pending.__dataclass_fields__
        assert k.entry_gate_ready is False
        with pytest.raises(AttributeError):
            k.entry_gate_ready = True
        waiting = k.commit()
        # BasicState's default substate is READY even before its entry gate opens.
        assert waiting.skill_state == "READY"
        assert k.entry_gate_ready is False
        k.prepare(RuntimeInput(1.1, state, None))
        k.reject()
        assert k.entry_gate_ready is False
        pending = k.prepare(RuntimeInput(1.12, state, None))
        assert k.entry_gate_ready is False
        k.commit()
        assert k.entry_gate_ready is True
        k.prepare(RuntimeInput(1.14, state, None, requested_state="damping"))
        k.commit()
        assert k.entry_gate_ready is False
    finally:
        backend.close()


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


def test_fixed_position_entry_gate_parameters():
    from cadence.plugins import BasicState

    services = SimpleNamespace(dimension=2)

    def frame(now, q):
        return ControlFrame(now, SimpleNamespace(joint_pos=np.asarray(q, dtype=float)))

    state = BasicState(
        2,
        "fixedpos",
        {
            "kind": "fixed_position",
            "target": [0.5, -0.5],
            "duration_s": 1.0,
            "entry_gate_progress": 0.5,
            "entry_gate_tolerance": 0.2,
        },
        services,
    )
    state.on_enter(frame(0.0, [0.0, 0.0]), [])
    assert state.step(frame(0.5, [0.4, -0.4])).entry_gate_ready
    assert not state.step(frame(0.5, [0.0, 0.0])).entry_gate_ready
    assert not state.step(frame(0.4, [0.4, -0.4])).entry_gate_ready

    strict = BasicState(
        2, "fixedpos", {"kind": "fixed_position", "target": [0.5, -0.5]}, services
    )
    strict.on_enter(frame(0.0, [0.0, 0.0]), [])
    assert not strict.step(frame(10.0, [0.4, -0.4])).entry_gate_ready
    assert strict.step(frame(10.0, [0.48, -0.48])).entry_gate_ready

    for bad in (
        {"entry_gate_progress": 1.5},
        {"entry_gate_progress": -0.1},
        {"entry_gate_tolerance": 0.0},
        {"entry_gate_tolerance": -0.2},
    ):
        with pytest.raises(ValueError, match="entry gate"):
            BasicState(2, "fixedpos", {"kind": "fixed_position", **bad}, services)
