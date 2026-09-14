"""Legacy client names preserving the deployed cadence.* schema aliases."""

from planet_protocol.client import (
    OperatorClient as _OperatorClient, JointTargetClient as _JointTargetClient,
    validate_operator_bindings,
)
from .localization import LocalizationClient

OPERATOR_SCHEMA = "cadence.operator.v1"
JOINT_TARGET_SCHEMA = "cadence.joint-target.v1"


class OperatorClient(_OperatorClient):
    """Compatibility client for the legacy operator query schema."""

    schema = OPERATOR_SCHEMA
    receiver_label = "Cadence receiver"


class JointTargetClient(_JointTargetClient):
    """Compatibility client for the legacy joint-target schema."""

    schema = JOINT_TARGET_SCHEMA
    receiver_label = "Cadence receiver"
