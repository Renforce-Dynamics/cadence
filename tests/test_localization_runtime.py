"""UDP localization drives the real generic kernel's root-loss state policies."""

from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
import json
import socket
import subprocess
import sys

import numpy as np
import pytest

from cadence_api import JointCommand
from cadence_api.plugins import RootLossMode
from cadence.backends.mock import MockBackend
from cadence.localization import LocalizationIngressConfig, LocalizationReceiver
from cadence.plugins import ControlFrame, ControlResult, ControlState, PluginCatalog
from cadence.runtime import RuntimeConfig, RuntimeInput, RuntimeKernel
from cadence_protocol.localization import LocalizationSample, encode_localization


class WorldState(ControlState):
    """Small deterministic plugin that requires root input and records accepted work."""

    requires_world_root = True

    def __init__(self, state_id, key, config, services):
        super().__init__(state_id, key, config, services)
        self.root_loss_mode = RootLossMode(config["root_loss_mode"])
        self.root_loss_exit_steps = config.get("root_loss_exit_steps")
        self.root_loss_fallback_state = "damping"
        self.roots = []
        self.applied = 0

    def step(self, frame):
        assert frame.localization is not None, "state was entered without a root"
        self.roots.append(frame.localization.position_w.copy())
        zeros = np.zeros(self.services.dimension)
        return ControlResult(JointCommand(zeros, zeros, zeros, np.ones_like(zeros), zeros))

    def on_command_applied(self, command):
        self.applied += 1


@contextmanager
def deployment(mode):
    catalog = PluginCatalog({
        "reset_state_id": 0, "safety_fallback_state_id": 1,
        "states": {
            0: {"key": "passive", "factory": "cadence.plugins:BasicState", "config": {"kind": "passive"}},
            1: {"key": "damping", "factory": "cadence.plugins:BasicState", "config": {"kind": "damping"}},
            2: {"key": "world", "factory": f"{__name__}:WorldState",
                "config": {"root_loss_mode": mode.value,
                           "root_loss_exit_steps": 2 if mode == RootLossMode.FALLBACK else None}},
        },
    })
    backend = MockBackend(2)
    backend.start()
    kernel = RuntimeKernel(RuntimeConfig(catalog, SimpleNamespace(dimension=2),
                                        ControlFrame, [-1, -1], [1, 1],
                                        start_state="damping"), backend.read_state(), 0.)
    with LocalizationReceiver(LocalizationIngressConfig(port=0)) as receiver:
        def tick(now, request=None):
            received = receiver.poll(now)
            robot = backend.read_state()
            kernel.prepare(RuntimeInput(now, robot,
                received.localization if received else None, requested_state=request))
            prepared = kernel.guard_pending()
            backend.write_command(prepared.command, robot.sequence)
            return kernel.commit()

        try:
            yield kernel, receiver, tick
        finally:
            kernel.reject()
            backend.close()


def transmit(receiver, *, sequence=0, valid=True, position=(1., 2., 0.8)):
    sample = LocalizationSample(position, (1., 0., 0., 0.), (0., 0., 0.),
                                "localization", 1, sequence, valid=valid)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        sender.sendto(encode_localization(sample), receiver.address)


def test_world_state_cannot_be_entered_before_any_valid_localization():
    with deployment(RootLossMode.FALLBACK) as (kernel, receiver, tick):
        result = tick(10., "world")
        assert result.mode == "DAMPING"
        assert any("world localization unavailable" in event for event in result.events)
        assert not kernel.plugin("world").roots
        transmit(receiver, valid=False)
        result = tick(10.02, "world")
        assert result.mode == "DAMPING"
        assert not result.safety_halted
        assert kernel.plugin("world").applied == 0


@pytest.mark.parametrize("loss", ["invalid", "ttl"])
def test_fallback_holds_prior_root_then_recovers_through_explicit_state_requests(loss):
    with deployment(RootLossMode.FALLBACK) as (kernel, receiver, tick):
        transmit(receiver)
        entered = tick(10., "world")
        assert entered.mode == "WORLD" and not entered.safety_halted
        world = kernel.plugin("world")
        assert world.applied == 1
        if loss == "invalid":
            transmit(receiver, sequence=1, valid=False)
            lost_at = 10.02
        else:
            lost_at = 10.26
        assert receiver.poll(lost_at) is None
        held = tick(lost_at)
        assert held.mode == "WORLD"
        assert any("holding last valid root" in event for event in held.events)
        np.testing.assert_array_equal(world.roots[-1], [1., 2., 0.8])
        assert world.applied == 2
        fallback = tick(lost_at + 0.02)
        assert fallback.mode == "DAMPING" and not fallback.safety_halted
        assert any("WORLD -> DAMPING" in event for event in fallback.events)
        assert world.applied == 2
        transmit(receiver, sequence=2, position=(2., 1., 0.9))
        recovered = tick(lost_at + 0.04)
        assert recovered.mode == "DAMPING"  # A sensor recovery is not a state request.
        assert any("localization recovered" in event for event in recovered.events)
        # The existing kernel requires a different explicit request to clear the
        # root-loss blocked mode before the operator can select it again.
        assert tick(lost_at + 0.06, "passive").mode == "PASSIVE"
        assert tick(lost_at + 0.08, "world").mode == "WORLD"
        np.testing.assert_array_equal(world.roots[-1], [2., 1., 0.9])
        assert world.applied == 3


def test_hold_last_is_owned_by_the_state_while_ingress_stays_expired():
    with deployment(RootLossMode.HOLD_LAST) as (kernel, receiver, tick):
        transmit(receiver)
        assert tick(10., "world").mode == "WORLD"
        world = kernel.plugin("world")
        for now in (10.26, 20., 100.):
            assert receiver.poll(now) is None
            result = tick(now)
            assert result.mode == "WORLD" and not result.safety_halted
            np.testing.assert_array_equal(world.roots[-1], [1., 2., 0.8])
        transmit(receiver, sequence=1, position=(4., 3., 0.7))
        result = tick(100.02)
        assert any("localization recovered" in event for event in result.events)
        np.testing.assert_array_equal(world.roots[-1], [4., 3., 0.7])


SENDER = Path(__file__).resolve().parents[1] / "scripts" / "send-localization.py"


def run_sender(receiver, *arguments):
    return subprocess.run([sys.executable, str(SENDER), "--port", str(receiver.address[1]),
                           *arguments], capture_output=True, text=True, timeout=5)


def test_sender_script_publishes_one_explicit_pose_then_invalidates():
    with LocalizationReceiver(LocalizationIngressConfig(port=0)) as receiver:
        result = run_sender(receiver, "--position-m", "1", "2", "0.8")
        assert result.returncode == 0, result.stderr
        report = json.loads(result.stdout)
        assert report["sent"] == 1 and report["last_sequence"] == 0
        first = receiver.poll()
        assert first.sample.position_w_m == (1., 2., 0.8)
        assert first.sample.source_timestamp_us > 0
        assert receiver.accepted == 1
        result = run_sender(receiver, "--invalid")
        assert result.returncode == 0, result.stderr
        assert json.loads(result.stdout)["sent"] == 1
        assert receiver.poll() is None


def test_sender_continuous_fixture_is_finite_and_has_no_background_publisher():
    with LocalizationReceiver(LocalizationIngressConfig(port=0)) as receiver:
        result = run_sender(receiver, "--position-m", "0", "0", "0.8",
                            "--duration-s", "0.16", "--hz", "20")
        assert result.returncode == 0, result.stderr
        report = json.loads(result.stdout)
        assert 2 <= report["sent"] <= 5
        latest = receiver.poll(10.)
        assert latest.sample.sequence == report["sent"] - 1
        assert receiver.accepted == report["sent"]
        assert receiver.poll(10.26) is None


@pytest.mark.parametrize("arguments", [
    [], ["--invalid", "--hz", "50"], ["--invalid", "--duration-s", "1"],
    ["--invalid", "--duration-s", "0", "--hz", "50"],
    ["--invalid", "--duration-s", "1", "--hz", "nan"],
])
def test_sender_requires_explicit_pose_and_continuous_timing(arguments):
    with LocalizationReceiver(LocalizationIngressConfig(port=0)) as receiver:
        result = run_sender(receiver, *arguments)
        assert result.returncode != 0
        assert receiver.poll() is None


def test_packaged_config_layer_enables_explicit_matched_localization():
    from cadence_config import load_config

    resolved = load_config("pkg://cadence/data/localization.yaml")
    cfg = LocalizationIngressConfig.from_mapping(resolved.data["runtime"]["localization"])
    assert cfg.host == "127.0.0.1" and cfg.source == "localization"
    assert cfg.max_age_s == 0.25 and cfg.max_datagrams_per_poll == 64
