"""Fusion: side-sonar gap check, gap measurement, fused estimate."""
import copy

import pytest

from pinacle.config import load_config
from pinacle.fusion import (GapMeasurer, estimate_gap, front_distance, gap_clear,
                            rear_distance, required_gap_cm)


@pytest.fixture
def cfg():
    c = copy.deepcopy(load_config())
    c["bay"]["side"] = "right"
    return c


def d_with(r=-1.0, l=-1.0):
    return [50, 50, 50, 50, l, r]


def test_gap_clear_right(cfg):
    assert gap_clear(d_with(r=-1), cfg)
    assert gap_clear(d_with(r=cfg["fusion"]["side_clear_cm"] + 1), cfg)
    assert not gap_clear(d_with(r=10), cfg)
    assert not gap_clear(d_with(r=10, l=-1), cfg)


def test_gap_clear_left_uses_left_sensor(cfg):
    cfg["bay"]["side"] = "left"
    assert not gap_clear(d_with(r=-1, l=5), cfg)
    assert gap_clear(d_with(r=5, l=-1), cfg)


def test_front_rear_ignore_minus_one():
    assert front_distance([-1, 12, 3, -1, 0, 0]) == 12
    assert rear_distance([1, 1, -1, -1, 0, 0]) is None


def test_gap_measurer_complete_gap(cfg):
    g = GapMeasurer(cfg)
    t = 0.0
    # 1 s beside a box, 2 s beside the gap at 20 cm/s, then a box again
    for side, dur in ((15, 1.0), (-1, 2.0), (15, 1.0)):
        for _ in range(int(dur / 0.05)):
            g.update(t, side, 20.0)
            t += 0.05
    assert g.length_cm == pytest.approx(40, abs=2)


def test_gap_measurer_ignores_unbounded_run(cfg):
    g = GapMeasurer(cfg)
    t = 0.0
    for _ in range(20):  # starts already in a gap, never closes
        g.update(t, -1, 20.0)
        t += 0.05
    assert g.length_cm is None


def test_estimate_gap_sources(cfg):
    need = required_gap_cm(cfg)
    assert need == cfg["car"]["length_cm"] + cfg["fusion"]["min_gap_margin_cm"]
    e = estimate_gap(50, None, d_with(), cfg)
    assert (e.source, e.length_cm, e.ok, e.clear) == ("vision", 50, True, True)
    e = estimate_gap(None, 45, d_with(r=10), cfg)
    assert (e.source, e.ok, e.clear) == ("sonar", True, False)
    e = estimate_gap(50, need - 1, None, cfg)
    assert e.source == "fusion" and e.length_cm == need - 1 and not e.ok
    e = estimate_gap(None, None, None, cfg)
    assert e.source == "none" and e.length_cm is None and not e.ok
