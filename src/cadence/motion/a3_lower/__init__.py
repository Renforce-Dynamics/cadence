"""A3 lower-only velocity actor family: shared observation builder and adapters."""

from . import estmoe, mlp, observation
from .estmoe import A3LowerEstMoEPolicy
from .mlp import A3LowerMlpPolicy
from .observation import (
    LOWER_VELOCITY_H32_HISTORY_FRAMES,
    LOWER_VELOCITY_H32_OBSERVATION_DIM,
    LOWER_VELOCITY_MLP_H4_HISTORY_FRAMES,
    LOWER_VELOCITY_MLP_H4_OBSERVATION_DIM,
    LOWER_VELOCITY_TERM_WIDTHS,
    LowerVelocityHistoryObservationBuilder,
    lower_velocity_observation_dim,
)

__all__ = [
    "estmoe",
    "mlp",
    "observation",
    "A3LowerEstMoEPolicy",
    "A3LowerMlpPolicy",
    "LOWER_VELOCITY_H32_HISTORY_FRAMES",
    "LOWER_VELOCITY_H32_OBSERVATION_DIM",
    "LOWER_VELOCITY_MLP_H4_HISTORY_FRAMES",
    "LOWER_VELOCITY_MLP_H4_OBSERVATION_DIM",
    "LOWER_VELOCITY_TERM_WIDTHS",
    "LowerVelocityHistoryObservationBuilder",
    "lower_velocity_observation_dim",
]
