"""Mode rules and FIND & PARK enable conditions (modes.py)."""
import itertools

import pytest

from pinacle.config import load_config
from pinacle.modes import Mode, ModeManager, find_park_enabled, manual_command_to_lr


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def cfg():
    return load_config()


@pytest.fixture
def mm(cfg):
    clock = Clock()
    m = ModeManager(cfg, clock=clock)
    m.clock_obj = clock
    return m


def test_starts_manual(mm):
    assert mm.mode == Mode.MANUAL


def test_manual_input_aborts_auto(mm):
    mm.start_auto()
    assert mm.mode == Mode.AUTO
    assert mm.manual_input(150, 150) is True
    assert mm.mode == Mode.MANUAL
    assert mm.manual_input(150, 150) is False  # already manual: nothing to abort


def test_stop_input_also_counts_during_auto(mm):
    mm.start_auto()
    assert mm.manual_input(0, 0) is True
    assert mm.mode == Mode.MANUAL


@pytest.mark.parametrize("final", ["PARKED", "ABORT"])
def test_back_to_manual_after_fsm_finishes(mm, final):
    mm.start_auto()
    assert mm.on_fsm_state("MANEUVER") is False
    assert mm.mode == Mode.AUTO
    assert mm.on_fsm_state(final) is True
    assert mm.mode == Mode.MANUAL


def test_deadman(mm, cfg):
    c = mm.clock_obj
    mm.manual_input(150, 150)
    c.t += cfg["manual"]["deadman_ms"] / 1000 - 0.05
    assert mm.deadman_due() is False
    c.t += 0.1
    assert mm.deadman_due() is True
    assert mm.deadman_due() is False  # fires once


def test_deadman_refreshed_by_repeats(mm):
    c = mm.clock_obj
    for _ in range(10):
        mm.manual_input(150, 150)
        c.t += 0.1
        assert not mm.deadman_due()


def test_deadman_not_when_stopped_or_auto(mm):
    c = mm.clock_obj
    mm.manual_input(0, 0)
    c.t += 5
    assert not mm.deadman_due()
    mm.manual_input(100, 100)
    mm.start_auto()
    c.t += 5
    assert not mm.deadman_due()


def test_estop_always_allowed(mm):
    mm.estop()
    assert mm.mode == Mode.MANUAL
    mm.start_auto()
    mm.estop()
    assert mm.mode == Mode.MANUAL


@pytest.mark.parametrize("telem_ok,cam,hom", list(itertools.product([True, False], repeat=3)))
def test_find_park_all_combinations(cfg, telem_ok, cam, hom):
    age = 0.2 if telem_ok else 5.0
    enabled, reason = find_park_enabled(age, cam, hom, cfg)
    assert enabled == (telem_ok and cam and hom)
    if enabled:
        assert reason == "ready"
    else:
        assert ("telemetry" in reason) == (not telem_ok)
        assert ("camera" in reason) == (not cam)
        assert ("calibration" in reason) == (not hom)


def test_find_park_no_telemetry_ever(cfg):
    assert find_park_enabled(float("inf"), True, True, cfg)[0] is False
    assert find_park_enabled(1.01, True, True, cfg)[0] is False
    assert find_park_enabled(0.99, True, True, cfg)[0] is True


def test_manual_command_mapping(cfg):
    assert manual_command_to_lr("forward", 160, cfg) == (160, 160)
    assert manual_command_to_lr("back", 160, cfg) == (-160, -160)
    assert manual_command_to_lr("rotate_left", 160, cfg) == (-160, 160)
    assert manual_command_to_lr("rotate_right", 160, cfg) == (160, -160)
    l, r = manual_command_to_lr("left", 200, cfg)
    assert 0 < l < r == 200
    l, r = manual_command_to_lr("right", 200, cfg)
    assert 0 < r < l == 200
    assert manual_command_to_lr("stop", 200, cfg) == (0, 0)
    assert manual_command_to_lr("forward", 999, cfg) == (255, 255)
    with pytest.raises(ValueError):
        manual_command_to_lr("jump", 100, cfg)
