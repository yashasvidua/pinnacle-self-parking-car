"""Planner step generation."""
import copy
from dataclasses import dataclass

import pytest

from pinacle.config import load_config
from pinacle.planner import plan_parallel_park


@dataclass
class Pose:
    x_cm: float
    y_cm: float
    angle_deg: float = 0.0
    confidence: float = 1.0


@pytest.fixture
def cfg():
    return load_config()


def test_sequence_and_parameters_right(cfg):
    cfg = copy.deepcopy(cfg)
    cfg["bay"]["side"] = "right"
    steps = plan_parallel_park(Pose(60, -35), cfg)
    assert [s.kind for s in steps] == ["drive", "rotate", "reverse", "rotate", "centre"]
    pl, car = cfg["planner"], cfg["car"]
    expect = 60 + pl["pull_past_cm"] + car["length_cm"] / 2
    assert steps[0].params["distance_cm"] == pytest.approx(expect)
    assert steps[0].params["pwm"] == pl["drive_pwm"]
    assert steps[0].params["duration_s"] == pytest.approx(
        expect / (pl["drive_pwm"] / pl["pwm_per_cm_s"]))
    assert steps[1].params["deg"] == pl["rotate_deg"]         # CCW: rear faces the right
    assert steps[3].params["deg"] == -pl["rotate_deg"]
    assert steps[2].params["stop_cm"] == pl["reverse_stop_cm"]
    assert steps[2].params["max_cm"] == pl["reverse_max_cm"]
    assert steps[2].params["pwm"] == pl["reverse_pwm"]
    assert steps[4].params["tol_cm"] == pl["centre_tol_cm"]
    assert steps[4].params["max_iters"] == cfg["fsm"]["adjust_max_iters"]
    assert all(s.description for s in steps)


def test_left_side_flips_rotation(cfg):
    cfg = copy.deepcopy(cfg)
    cfg["bay"]["side"] = "left"
    steps = plan_parallel_park(Pose(60, 35), cfg)
    assert steps[1].params["deg"] == -cfg["planner"]["rotate_deg"]
    assert steps[3].params["deg"] == cfg["planner"]["rotate_deg"]


def test_uses_config_values(cfg):
    cfg = copy.deepcopy(cfg)
    cfg["planner"].update(pull_past_cm=0, rotate_deg=80, reverse_stop_cm=15)
    cfg["car"]["length_cm"] = 40
    steps = plan_parallel_park(Pose(10, -30), cfg)
    assert steps[0].params["distance_cm"] == pytest.approx(30)
    assert steps[1].params["deg"] == 80
    assert steps[2].params["stop_cm"] == 15


def test_bay_far_behind_gives_zero_drive(cfg):
    steps = plan_parallel_park(Pose(-200, -35), cfg)
    assert steps[0].params["distance_cm"] == 0
