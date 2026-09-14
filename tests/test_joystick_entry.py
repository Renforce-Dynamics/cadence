"""Cadence owns companion bindings without requiring the PlanetJoystick package."""
from pathlib import Path

import pytest

from cadence_config import load_config
from cadence.operator import OperatorInputMapping, ReceivedJoystickCommand
from cadence.plugins import PluginCatalog
from planet_protocol.client import validate_operator_bindings
from planet_protocol.operator import JoystickCommandPacket, decode_joystick_command, encode_joystick_command


ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / 'configs/entry/entry_joystick.yaml'


@pytest.mark.parametrize('runtime_entry', ['entry_a3_operator.yaml', 'entry_a3_operator_stream.yaml'])
def test_companion_bindings_match_selected_catalog_without_planetj(runtime_entry):
    producer = load_config(ENTRY)
    runtime = load_config(ROOT / 'configs/entry' / runtime_entry)
    catalog = PluginCatalog(load_config(runtime.path('runtime.state_registry_config')).data)
    requests = producer.data['inputs']['requests']
    bindings = [(mapping['request_id'], name) for name, mapping in requests.items()]
    validate_operator_bindings(bindings, catalog.ids, catalog.aliases)
    assert dict(bindings) == {0: 'passive', 1: 'damping', 2: 'fixedpos', 3: 'loco'}
    receiver = runtime.data['runtime']['operator']
    assert producer.data['target'] == {'host': receiver['host'], 'port': receiver['port']}
    signals = {item['debug_name']: item['signal_id'] for item in producer.data['inputs']['signals']}
    assert signals['EMERGENCY_HALT'] == receiver['mapping']['emergency_signal_id']
    assert signals['RESET_SAFETY'] == receiver['mapping']['reset_signal_id']
    assert set(receiver['mapping']['velocity_axes']) <= set(producer.data['inputs']['axes'])
    assert all(Path(source).is_relative_to(ROOT / 'configs') for source in producer.origins.values())


@pytest.mark.parametrize('name,buttons', [
    ('passive', [5, 9]), ('damping', [5, 1]), ('fixedpos', [5, 0]), ('loco', [5, 2]),
])
def test_planetj_loader_and_physical_chords_match_catalog(name, buttons):
    config_module = pytest.importorskip('planetj.config')
    from planetj.runtime import map_operator_input

    producer = config_module.load_config(ENTRY)
    raw_buttons = [index in buttons for index in range(11)]
    command = map_operator_input(True, raw_buttons, [0.0] * 8, producer.inputs)
    catalog = PluginCatalog(load_config(ROOT / 'configs/state_registries/a3_operator.yaml').data)
    assert catalog.key_for_id(command.request_id) == name
    validate_operator_bindings([(item.request_id, item.state_key) for item in producer.inputs.requests],
                               catalog.ids, catalog.aliases)


def test_planetj_axes_and_signals_reach_cadence_interpretation():
    config_module = pytest.importorskip('planetj.config')
    from planetj.runtime import map_operator_input

    producer = config_module.load_config(ENTRY)
    runtime = load_config(ROOT / 'configs/entry/entry_a3_operator.yaml')
    mapping = OperatorInputMapping.from_mapping(runtime.data['runtime']['operator']['mapping'])
    # Full left stick forward/right and right stick right; normalized once in PlanetJ.
    command = map_operator_input(True, [False] * 11, [1, -1, 0, 1, 0, 0, 0, 0], producer.inputs)
    packet = JoystickCommandPacket(session_id=1, packet_seq=1, source_age_us=0,
                                   request_id=command.request_id, flags=command.flags,
                                   axes=command.axes, ttl_ms=producer.publisher.ttl_ms)
    packet = decode_joystick_command(encode_joystick_command(packet))
    received = ReceivedJoystickCommand(packet, received_monotonic_s=0.0)
    assert received.velocity(mapping, 0.0) == pytest.approx((0.4, 0.2, 0.5))
    for buttons, signal_id in [([8], mapping.emergency_signal_id), ([5, 7], mapping.reset_signal_id)]:
        command = map_operator_input(True, [index in buttons for index in range(11)],
                                     [0.0] * 8, producer.inputs)
        assert command.signal_bits == 1 << signal_id
