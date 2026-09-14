"""Real UDP localization freshness, ordering, identity and resource ownership."""

from dataclasses import replace
import json
import socket
import subprocess
import sys

import numpy as np
import pytest

from cadence.localization import LocalizationIngressConfig, LocalizationReceiver
from cadence_protocol.localization import (
    MAX_DATAGRAM_BYTES, MAX_SAFE_INTEGER, LocalizationClient, LocalizationSample,
    decode_localization, encode_localization, new_localization_session,
)


def sample(**overrides):
    values = dict(position_w_m=(1., 2., 3.), orientation_wxyz=(1., 0., 0., 0.),
                  linear_velocity_w_mps=(0.1, 0.2, 0.3), source="mocap",
                  session_id=100, sequence=0, source_timestamp_us=1234567,
                  ttl_s=0.25, source_age_s=0.0)
    return LocalizationSample(**{**values, **overrides})


def receiver(**overrides):
    return LocalizationReceiver(LocalizationIngressConfig(
        **{"port": 0, "source": "mocap", **overrides}))


def transmit(endpoint, *values):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        for value in values:
            sender.sendto(value if isinstance(value, bytes) else encode_localization(value), endpoint)


def test_round_trip_and_detached_runtime_value():
    mutable = [1., 2., 3.]
    expected = sample(position_w_m=mutable)
    mutable[0] = 9.
    assert expected.position_w_m == (1., 2., 3.)
    assert decode_localization(encode_localization(expected)) == expected
    with receiver() as ingress:
        transmit(ingress.address, expected)
        received = ingress.poll(10.)
        assert received.sample == expected
        runtime = received.localization
        assert np.array_equal(runtime.position_w, expected.position_w_m)
        runtime.position_w[0] = 100
        assert received.localization.position_w[0] == 1.
        assert ingress.accepted == 1


@pytest.mark.parametrize("field,value", [
    ("position_w_m", [1, 2]), ("position_w_m", [1, 2, float("nan")]),
    ("position_w_m", [True, 0, 0]), ("position_w_m", ["1", 2, 3]),
    ("position_w_m", [10 ** 1000, 0, 0]),
    ("linear_velocity_w_mps", [[1], 2, 3]),
    ("orientation_wxyz", [0, 0, 0, 0]), ("orientation_wxyz", [2, 0, 0, 0]),
    ("orientation_wxyz", [float("inf"), 0, 0, 0]),
    ("sequence", -1), ("sequence", 0.1), ("sequence", True),
    ("session_id", 0), ("session_id", MAX_SAFE_INTEGER + 1),
    ("source_timestamp_us", -1), ("source_timestamp_us", MAX_SAFE_INTEGER + 1),
    ("source_age_s", -0.1), ("source_age_s", float("nan")),
    ("source_age_s", 10 ** 1000), ("ttl_s", 0), ("ttl_s", True),
    ("valid", 1), ("source", ""), ("source", "mocap stream"),
    ("frame_id", "policy_root"), ("child_frame_id", "x" * 129),
])
def test_sample_strict_validation(field, value):
    with pytest.raises(ValueError):
        sample(**{field: value})


def test_quaternion_convention_preserves_rotation_and_normalizes_roundoff():
    result = sample(orientation_wxyz=(0., 0., 0., 1.00001))
    assert result.orientation_wxyz == (0., 0., 0., 1.)
    assert decode_localization(encode_localization(result)).orientation_wxyz == (0., 0., 0., 1.)


@pytest.mark.parametrize("mutate", [
    lambda data: {**data, "unexpected": 1},
    lambda data: {key: value for key, value in data.items() if key != "valid"},
    lambda data: {**data, "schema": "cadence.localization.v2"},
    lambda data: {**data, "type": "target"},
    lambda data: {**data, "valid": "false"},
])
def test_wire_fields_are_versioned_and_strict(mutate):
    data = json.loads(encode_localization(sample()))
    with pytest.raises(ValueError):
        decode_localization(json.dumps(mutate(data)).encode())


@pytest.mark.parametrize("data", [
    b"", b"[]", b"true", b"{", b"\xff", b"{" * 2000,
    b'{"schema":"a","schema":"b"}',
    b'{"position_w_m":[NaN,0,0]}', b" " * (MAX_DATAGRAM_BYTES + 1),
])
def test_malformed_wire_is_rejected(data):
    with pytest.raises(ValueError):
        decode_localization(data)


def test_ttl_adds_producer_age_and_receiver_time_without_clock_alignment():
    with receiver(max_age_s=0.20) as ingress:
        # Timestamp is intentionally far from the receiver monotonic clock.
        transmit(ingress.address, sample(ttl_s=1000, source_age_s=0.05))
        received = ingress.poll(10.)
        assert received.age_s(10.10) == pytest.approx(0.15)
        assert ingress.poll(10.149) is received
        assert ingress.poll(10.151) is None
        assert ingress.poll(1000.) is None


def test_source_ttl_can_be_stricter_than_receiver_limit():
    with receiver(max_age_s=10.) as ingress:
        transmit(ingress.address, sample(ttl_s=0.1))
        assert ingress.poll(5.) is not None
        assert ingress.poll(5.101) is None


@pytest.mark.parametrize("update", [
    {"valid": False}, {"source_age_s": 0.3},
])
def test_newer_expired_or_invalid_sample_withdraws_previous_immediately(update):
    with receiver() as ingress:
        transmit(ingress.address, sample())
        assert ingress.poll(10.) is not None
        transmit(ingress.address, sample(sequence=1, **update))
        assert ingress.poll(10.01) is None
        transmit(ingress.address, sample(sequence=0))
        assert ingress.poll(10.02) is None
        transmit(ingress.address, sample(sequence=2))
        assert ingress.poll(10.03).sample.sequence == 2


def test_wrong_source_frames_and_malformed_packets_never_replace_good_sample():
    with receiver() as ingress:
        transmit(ingress.address, sample())
        first = ingress.poll(10.)
        transmit(ingress.address, sample(sequence=1, source="other"),
                 sample(sequence=2, frame_id="map"),
                 sample(sequence=3, child_frame_id="pelvis"), b"invalid")
        assert ingress.poll(10.1) is first
        assert ingress.rejected_identity == 3
        assert ingress.rejected_decode == 1
        assert ingress.poll(10.26) is None


def test_duplicates_reordering_and_retired_sessions_are_rejected():
    with receiver() as ingress:
        transmit(ingress.address, sample(sequence=10), sample(sequence=10),
                 sample(sequence=9))
        assert ingress.poll(10.).sample.sequence == 10
        assert ingress.rejected_sequence == 2
        transmit(ingress.address, sample(session_id=101, sequence=0))
        assert ingress.poll(10.01).sample.session_id == 101
        transmit(ingress.address, sample(session_id=100, sequence=MAX_SAFE_INTEGER),
                 sample(session_id=99, sequence=MAX_SAFE_INTEGER))
        assert ingress.poll(10.02).sample.session_id == 101
        assert ingress.rejected_sequence == 4
        # Even after TTL expiry, a delayed old session cannot revive localization.
        assert ingress.poll(11.) is None
        transmit(ingress.address, sample(session_id=100, sequence=MAX_SAFE_INTEGER))
        assert ingress.poll(11.1) is None
        transmit(ingress.address, sample(session_id=102, sequence=0))
        assert ingress.poll(11.2).sample.session_id == 102


def test_per_poll_budget_is_bounded_including_bad_packets():
    with receiver(max_datagrams_per_poll=2) as ingress:
        transmit(ingress.address, b"bad", b"bad", sample(sequence=10), sample(sequence=11))
        assert ingress.poll(10.) is None
        assert ingress.rejected_decode == 2
        assert ingress.poll(10.01).sample.sequence == 11
        assert ingress.accepted == 2


def test_client_real_udp_manages_sequence_and_closed_resources():
    with receiver() as ingress:
        host, port = ingress.address
        with LocalizationClient(host, port, source="mocap", session_id=100) as client:
            sent = client.send([1, 2, 3], [1, 0, 0, 0], [0, 0, 0],
                               source_timestamp_us=123, source_age_s=0.02)
            assert sent.sequence == 0
            assert sent.session_id == client.session_id == 100
            assert ingress.poll().sample == sent
            client.send([1, 2, 3], [1, 0, 0, 0], [0, 0, 0], valid=False)
            assert ingress.poll() is None
        with pytest.raises(ValueError, match="closed"):
            client.send([0, 0, 0], [1, 0, 0, 0], [0, 0, 0])
    ingress.close()  # Idempotent cleanup.
    with pytest.raises(ValueError, match="closed"):
        ingress.poll()


def test_client_new_sessions_are_ordered():
    assert new_localization_session() < new_localization_session()


def test_receive_address_cannot_be_owned_by_two_live_receivers():
    with receiver() as ingress:
        host, port = ingress.address
        with pytest.raises(OSError):
            LocalizationReceiver(LocalizationIngressConfig(host=host, port=port))
    # Failed construction and successful close leave the address reusable.
    with LocalizationReceiver(LocalizationIngressConfig(host=host, port=port)) as second:
        assert second.address == (host, port)


@pytest.mark.parametrize("config", [
    {"unknown": True}, {"port": True}, {"port": -1}, {"port": 65536},
    {"host": ""}, {"source": " "}, {"max_age_s": 0}, {"max_age_s": "1"},
    {"max_datagrams_per_poll": 0}, {"max_datagrams_per_poll": 1025},
    {"frame_id": "policy_root"},
])
def test_ingress_configuration_rejects_ambiguous_values(config):
    with pytest.raises(ValueError):
        LocalizationIngressConfig.from_mapping(config)


def test_configuration_is_optional_with_loopback_defaults():
    assert LocalizationIngressConfig.from_mapping(None) is None
    default = LocalizationIngressConfig.from_mapping({})
    assert default.host == "127.0.0.1"
    assert default.frame_id == "world"
    assert default.child_frame_id == "policy_root"


def test_codec_and_client_import_without_runtime_numpy_or_task_packages():
    result = subprocess.run([sys.executable, "-c", """
import sys
from cadence_protocol.localization import LocalizationSample, LocalizationClient, encode_localization, decode_localization
from cadence_protocol.client import LocalizationClient as CompatibilityImport
assert CompatibilityImport is LocalizationClient
sample = LocalizationSample((0,0,1), (1,0,0,0), (0,0,0), 'mocap', 1, 0)
assert decode_localization(encode_localization(sample)) == sample
assert not any(name.split('.')[0] in {'numpy', 'cadence', 'cadence_rally', 'planetj', 'agi3sdk'} for name in sys.modules)
"""], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
