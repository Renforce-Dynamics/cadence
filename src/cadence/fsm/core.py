"""Small registered FSM core shared by sim and future robot adapters."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Generic, TypeVar


ContextT = TypeVar("ContextT")


class State(ABC, Generic[ContextT]):
    key: str

    def enter(self, context: ContextT, now_s: float) -> None:
        pass

    @abstractmethod
    def tick(self, context: ContextT, event: Any, now_s: float) -> str | None:
        """Return a registered target key to transition, or None to stay."""

    def exit(self, context: ContextT, now_s: float) -> None:
        pass


class StateRegistry(Generic[ContextT]):
    def __init__(self) -> None:
        self._types: dict[str, type[State[ContextT]]] = {}

    def register(self, key: str, state_type: type[State[ContextT]]) -> None:
        canonical = str(key).strip().lower()
        if not canonical or canonical in self._types:
            raise KeyError(f"duplicate/invalid state key: {key!r}")
        if not issubclass(state_type, State):
            raise TypeError("registered state must derive from State")
        self._types[canonical] = state_type

    def state(self, key: str):
        def decorator(state_type: type[State[ContextT]]):
            self.register(key, state_type)
            return state_type

        return decorator

    def create(self, key: str) -> State[ContextT]:
        canonical = str(key).strip().lower()
        try:
            return self._types[canonical]()
        except KeyError as error:
            raise KeyError(
                f"unknown state {canonical!r}; known={sorted(self._types)}"
            ) from error

    def keys(self) -> tuple[str, ...]:
        return tuple(self._types)


class StateMachine(Generic[ContextT]):
    def __init__(
        self,
        registry: StateRegistry[ContextT],
        context: ContextT,
        initial: str,
        now_s: float,
    ) -> None:
        self.registry = registry
        self.context = context
        self.current = registry.create(initial)
        self.current.enter(context, float(now_s))

    @property
    def state(self) -> str:
        return self.current.key.upper()

    def transition(self, target: str, now_s: float) -> None:
        canonical = str(target).lower()
        if canonical == self.current.key:
            return
        next_state = self.registry.create(canonical)
        self.current.exit(self.context, now_s)
        next_state.enter(self.context, now_s)
        self.current = next_state

    def tick(self, event: Any, now_s: float) -> str:
        target = self.current.tick(self.context, event, float(now_s))
        if target is not None:
            self.transition(target, float(now_s))
        return self.state
