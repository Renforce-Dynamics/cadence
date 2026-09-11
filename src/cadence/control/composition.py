"""Explicit joint ownership for concurrently evaluated control skills."""

from dataclasses import dataclass
import numpy as np
from cadence_api import JointCommand


@dataclass(frozen=True)
class CommandPart:
    owner: str
    joints: tuple[int, ...]
    command: JointCommand


def compose_commands(parts, fallback):
    """Merge disjoint joint claims; unclaimed joints retain the fallback command."""
    n = len(fallback.q_des)
    owners = {}
    arrays = [
        x.copy()
        for x in (
            fallback.q_des,
            fallback.dq_des,
            fallback.kp,
            fallback.kd,
            fallback.tau_ff,
        )
    ]
    for part in parts:
        indices = tuple(part.joints)
        if (
            not part.owner
            or len(set(indices)) != len(indices)
            or len(indices) != len(part.command.q_des)
        ):
            raise ValueError("invalid command owner or joint claim")
        if any(
            not isinstance(i, int) or isinstance(i, bool) or not 0 <= i < n
            for i in indices
        ):
            raise ValueError("joint claim outside layout")
        for i in indices:
            if i in owners:
                raise ValueError(
                    f"joint {i} claimed by both {owners[i]} and {part.owner}"
                )
            owners[i] = part.owner
        for target, source in zip(
            arrays,
            (
                part.command.q_des,
                part.command.dq_des,
                part.command.kp,
                part.command.kd,
                part.command.tau_ff,
            ),
        ):
            target[list(indices)] = source
    return JointCommand(*arrays)
