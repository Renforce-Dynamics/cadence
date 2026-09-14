"""Planet inputs and legacy Cadence aliases share one live control contract."""

from contextlib import contextmanager
from types import SimpleNamespace
import json
import socket
import threading

import pytest

from planet_protocol.client import OperatorClient, JointTargetClient
from planet_protocol.localization import LocalizationSample, encode_localization, LocalizationClient
from planet_protocol.operator import JoystickCommandPacket, JoystickFlags, encode_joystick_command
from cadence_protocol.client import OperatorClient as LegacyOperatorClient, JointTargetClient as LegacyJointTargetClient
from cadence_protocol.localization import (
    LocalizationSample as LegacySample, encode_localization as legacy_encode,
    decode_localization as legacy_decode, LocalizationClient as LegacyLocalizationClient,
)
from cadence_protocol.operator import JoystickCommandPacket as LegacyJoystickPacket
from cadence.operator import JoystickCommandReceiver
from cadence.localization import LocalizationIngressConfig, LocalizationReceiver
from cadence.motion.targets import LatestJointTarget, JointTargetUdpReceiver


@contextmanager
def operator_receiver():
    receiver = JoystickCommandReceiver("127.0.0.1", 0)
    receiver.set_catalog(SimpleNamespace(ids={0: "passive", 1: "damping", 3: "loco"},
        aliases={"loco_lower": "loco"}, reset_state_id=0, safety_fallback_state_id=1))
    receiver.update_status({"mode": "damping", "safety_halted": False,
                            "events": (), "execution": "backend"})
    stopped = threading.Event()
    errors = []

    def poll():
        try:
            while not stopped.wait(.001):
                receiver.poll()
        except BaseException as error:
            errors.append(error)

    worker = threading.Thread(target=poll)
    worker.start()
    try:
        yield receiver
    finally:
        stopped.set()
        worker.join(1)
        receiver.close()
        assert not worker.is_alive() and not errors


def test_shared_plnj_and_localization_types_keep_legacy_source_identity():
    assert LegacyJoystickPacket is JoystickCommandPacket
    assert LegacySample is LocalizationSample
    packet = JoystickCommandPacket(1, 2, 0, 100, JoystickFlags.CONNECTED, 0)
    from cadence_protocol.operator import encode_joystick_command as legacy_operator_encode
    assert legacy_operator_encode(packet) == encode_joystick_command(packet)


def test_operator_queries_echo_each_schema_without_separate_catalog_or_status():
    with operator_receiver() as receiver:
        with OperatorClient(*receiver.address) as modern, LegacyOperatorClient(*receiver.address) as legacy:
            assert modern.describe()["schema"] == "planet.operator.v1"
            assert legacy.describe()["schema"] == "cadence.operator.v1"
            assert modern.validate_bindings([(3, "loco_lower")])["states"] == legacy.describe()["states"]
            for client in (modern, legacy, modern):
                assert client.status() == {"schema": client.schema, "type": "status",
                    "mode": "damping", "safety_halted": False, "events": [], "execution": "backend"}
                with pytest.raises(ValueError, match="bound to"):
                    client.validate_bindings([(3, "damping")])
            assert receiver._description["schema"] == "cadence.operator.v1"
            assert receiver._status["schema"] == "cadence.operator.v1"


def test_upper_schema_switch_keeps_activation_order_and_latest_held_command():
    mailbox = LatestJointTarget(2)
    activation = mailbox.activate()
    with JointTargetUdpReceiver(mailbox) as receiver:
        with JointTargetClient(*receiver.address) as modern, LegacyJointTargetClient(*receiver.address) as legacy:
            assert modern.status()["activation"] == legacy.status()["activation"] == activation
            assert modern.send(activation, 0, [.1, -.1])["accepted"]
            assert legacy.send(activation, 1, [.2, -.2])["accepted"]
            assert not modern.send(activation, 0, [.5, -.5])["accepted"]
            assert not legacy.send(activation, 1, [.9, -.9])["accepted"]
            assert mailbox.latest().q_des == (.2, -.2)
            assert mailbox.latest().sequence == 1
            new_activation = mailbox.activate()
            assert not modern.send(activation, 2, [.8, -.8])["accepted"]
            assert legacy.send(new_activation, 0, [.3, -.3])["accepted"]
        # Disconnect does not supply a fallback or reset a held target.
        assert mailbox.latest().q_des == (.3, -.3)


@pytest.mark.parametrize("schema", ["planet.joint-target.v1", "cadence.joint-target.v1"])
def test_invalid_upper_frame_does_not_mutate_mailbox_and_echoes_schema(schema):
    mailbox = LatestJointTarget(1)
    mailbox.activate()
    receiver = JointTargetUdpReceiver(mailbox)
    reply = receiver._response(json.dumps({"schema": schema, "type": "target",
        "activation": mailbox.activation, "sequence": 0, "q_des": [1, 2]}).encode())
    assert not reply["accepted"] and reply["schema"] == schema
    assert mailbox.latest() is None


def test_localization_aliases_share_sequence_session_invalidation_and_ttl():
    base = dict(position_w_m=(0., 0., 1.), orientation_wxyz=(1., 0., 0., 0.),
                linear_velocity_w_mps=(0., 0., 0.), source="localization", session_id=100)
    with LocalizationReceiver(LocalizationIngressConfig(port=0)) as receiver:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            def publish(codec, *, now=10., **changes):
                sender.sendto(codec(LocalizationSample(**{**base, **changes})), receiver.address)
                return receiver.poll(now)

            assert publish(encode_localization, sequence=10).sample.sequence == 10
            assert publish(legacy_encode, sequence=9).sample.sequence == 10
            assert receiver.rejected_sequence == 1
            assert publish(legacy_encode, sequence=11, valid=False) is None
            assert publish(encode_localization, sequence=11) is None
            assert publish(encode_localization, sequence=12).sample.sequence == 12
            assert receiver.poll(10.3) is None
            assert publish(legacy_encode, now=10.31, sequence=0, session_id=101).sample.session_id == 101
            assert publish(encode_localization, now=10.32, sequence=999).sample.session_id == 101
            assert publish(encode_localization, now=10.33, sequence=1, session_id=101,
                           source_age_s=1.0) is None
            assert receiver.poll(10.34) is None


def test_legacy_localization_encoder_and_client_keep_legacy_defaults():
    sample = LocalizationSample((0, 0, 1), (1, 0, 0, 0), (0, 0, 0), "localization", 1, 0)
    old = legacy_encode(sample)
    assert json.loads(old)["schema"] == "cadence.localization.v1"
    assert legacy_decode(old) == sample
    with pytest.raises(ValueError, match="schema"):
        legacy_decode(encode_localization(sample))
    with LocalizationReceiver(LocalizationIngressConfig(port=0)) as receiver:
        with LegacyLocalizationClient(*receiver.address, session_id=100) as legacy:
            legacy.send([0, 0, 1], [1, 0, 0, 0], [0, 0, 0])
            assert receiver.poll().sample.session_id == 100
        with LocalizationClient(*receiver.address, session_id=101) as modern:
            modern.send([0, 0, 1], [1, 0, 0, 0], [0, 0, 0])
            assert receiver.poll().sample.session_id == 101
