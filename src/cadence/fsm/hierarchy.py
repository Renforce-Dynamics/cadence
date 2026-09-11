"""Hierarchical state machine: parent cancellation invalidates child activations."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Transition:
    source: str
    event: str
    target: str


class HierarchicalMachine:
    def __init__(
        self,
        machine_id,
        states,
        initial,
        transitions=(),
        *,
        parent=None,
        on_enter=None,
        on_exit=None,
    ):
        self.id = machine_id
        self.states = set(states)
        self.parent = parent
        self.children = []
        if not machine_id or initial not in self.states:
            raise ValueError("invalid machine/initial state")
        self.transitions = {}
        for item in transitions:
            t = item if isinstance(item, Transition) else Transition(**item)
            if t.source not in self.states or t.target not in self.states:
                raise ValueError("unknown transition state")
            if (t.source, t.event) in self.transitions:
                raise ValueError("ambiguous transition")
            self.transitions[t.source, t.event] = t.target
        self.current = initial
        self.activation = 1
        self.sequence = 0
        self.active = True
        self.on_enter = on_enter or (lambda state: None)
        self.on_exit = on_exit or (lambda state: None)
        if parent is not None:
            parent.children.append(self)
        self.on_enter(initial)

    def transition(self, target):
        if not self.active:
            raise RuntimeError("machine is cancelled")
        if target not in self.states:
            raise ValueError("unknown state")
        if target == self.current:
            return False
        for child in self.children:
            child.cancel()
        self.on_exit(self.current)
        self.current = target
        self.sequence += 1
        self.activation += 1
        self.on_enter(target)
        return True

    def dispatch(self, event, *, activation=None):
        if not self.active or activation is not None and activation != self.activation:
            return False
        target = self.transitions.get((self.current, event))
        return self.transition(target) if target is not None else False

    def cancel(self):
        if not self.active:
            return
        for child in self.children:
            child.cancel()
        self.on_exit(self.current)
        self.active = False
        self.activation += 1

    def restart(self, state=None):
        if self.parent is not None and not self.parent.active:
            raise RuntimeError("cannot activate a child of a cancelled parent")
        target = self.current if state is None else state
        if target not in self.states:
            raise ValueError("unknown state")
        self.cancel()
        self.current = target
        self.active = True
        self.activation += 1
        self.on_enter(target)

    def snapshot(self):
        return {
            "machine_id": self.id,
            "parent_id": None if self.parent is None else self.parent.id,
            "state": self.current,
            "active": self.active,
            "activation_id": self.activation,
            "transition_seq": self.sequence,
        }
