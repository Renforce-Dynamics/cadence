"""A3 lower-only velocity actor family: shared observation builder and adapter."""

from . import estmoe, observation
from .estmoe import A3LowerEstMoEPolicy
from .observation import (
    LOWER_VELOCITY_H32_HISTORY_FRAMES,
    LOWER_VELOCITY_H32_OBSERVATION_DIM,
    LOWER_VELOCITY_TERM_WIDTHS,
    LowerVelocityHistoryObservationBuilder,
    lower_velocity_observation_dim,
)

__all__ = [
    "estmoe",
    "observation",
    "A3LowerEstMoEPolicy",
    "LOWER_VELOCITY_H32_HISTORY_FRAMES",
    "LOWER_VELOCITY_H32_OBSERVATION_DIM",
    "LOWER_VELOCITY_TERM_WIDTHS",
    "LowerVelocityHistoryObservationBuilder",
    "lower_velocity_observation_dim",
]
