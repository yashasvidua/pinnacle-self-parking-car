"""Parking FSM with scripted inputs (no threads, no sockets, fake time)."""
import copy
from dataclasses import dataclass

import pytest

from pinacle.comms import Telemetry
from pinacle.config import load_config
from pinacle.fsm import FsmInputs, ParkingFSM, State
from pinacle.fusion import GapEstimate

DT = 0.05
GOOD_GAP = GapEstimate(clear=True, length_cm=50, source="vision", ok=True)
SHORT_GAP = GapEstimate(clear=True, length_cm=20, source="vision", ok=False)


@dataclass
class Pose:
    x_cm: float = 60
    y_cm: float = -35
    angle_deg: float = 0.0
    confidence: float = 0.9


@pytest.fixture
def cfg():
    return copy.deepcopy(load_config())


class World:
    """Tiny scripted DevKit: reacts to the FSM's commands with plausible telemetry."""

    def __init__(self):
        self.t = 0.0
        self.busy_until = None
        self.rear = 40.0
        self.front = 30.0
        self.l = self.r = 0
        self.state = "OK"
        self.yaw = 0.0
        self.seq = 0

    def apply(self, cmd):
        if cmd.kind == "rotate":
            self.busy_until = self.t + 0.5
            self.l = self.r = 0
        elif cmd.kind == "drive":
            self.l, self.r = cmd.left, cmd.right
        elif cmd.kind == "stop":
            self.l = self.r = 0

    def tick(self):
        self.t += DT
        if self.l == self.r and self.l != 0:  # straight: forward = front closer, rear further
            v = self.l / 4.0  # cm/s (pwm_per_cm_s = 4)
            self.front -= v * DT
            self.rear += v * DT
        self.seq += 1
        busy = self.busy_until is not None and self.t < self.busy_until
        return Telemetry(self.seq, self.state, [self.front, self.front, self.rear, self.rear, -1, -1],
                         self.yaw, busy, t_rx=self.t)


def run(fsm, world, pose=Pose(), gap=GOOD_GAP, max_t=60, until=(State.PARKED, State.ABORT)):
    out = None
    while world.t < max_t:
        tel = world.tick()
        out = fsm.update(FsmInputs(world.t, pose, tel, gap))
        world.apply(out.command)
        if fsm.state in until:
            break
    return out


def test_happy_path(cfg):
    fsm, w = ParkingFSM(cfg), World()
    fsm.start(w.t)
    assert fsm.state == State.SEARCH
    out = run(fsm, w)
    assert fsm.state == State.PARKED, fsm.history
    visited = [h.dst for h in fsm.history]
    assert visited == ["SEARCH", "ALIGN", "MEASURE", "MANEUVER", "ADJUST", "PARKED"]
    assert out.state == "PARKED"
    assert [s.kind for s in fsm.plan] == ["drive", "rotate", "reverse", "rotate", "centre"]


def test_search_creeps_then_times_out(cfg):
    fsm, w = ParkingFSM(cfg), World()
    fsm.start(0.0)
    out = fsm.update(FsmInputs(0.05, None, w.tick(), GOOD_GAP))
    assert out.command.kind == "drive"
    assert out.command.left == out.command.right == cfg["fsm"]["search_pwm"]
    run(fsm, w, pose=None)
    assert fsm.state == State.ABORT and "no bay" in fsm.reason
    assert w.t == pytest.approx(cfg["fsm"]["search_timeout_s"], abs=0.2)


def test_low_confidence_is_ignored(cfg):
    fsm, w = ParkingFSM(cfg), World()
    fsm.start(0.0)
    run(fsm, w, pose=Pose(confidence=0.1), max_t=1)
    assert fsm.state == State.SEARCH


def test_align_steers_toward_bay(cfg):
    fsm, w = ParkingFSM(cfg), World()
    fsm.start(0.0)
    out = fsm.update(FsmInputs(0.05, Pose(angle_deg=20), w.tick(), GOOD_GAP))
    assert fsm.state == State.ALIGN
    assert out.command.kind == "drive" and out.command.right > out.command.left  # turn left (CCW)
    run(fsm, w, pose=Pose(angle_deg=20))
    assert fsm.state == State.ABORT and "align" in fsm.reason


def test_short_gap_aborts(cfg):
    fsm, w = ParkingFSM(cfg), World()
    fsm.start(0.0)
    run(fsm, w, gap=SHORT_GAP)
    assert fsm.state == State.ABORT and "gap too short" in fsm.reason
    assert fsm.history[-2].dst == "MEASURE"


def test_rotate_step_timeout(cfg):
    fsm, w = ParkingFSM(cfg), World()
    w.apply = lambda cmd: None if cmd.kind == "rotate" else World.apply(w, cmd)  # never busy
    fsm.start(0.0)
    run(fsm, w)
    assert fsm.state == State.ABORT and "timed out" in fsm.reason and "Rotate" in fsm.reason


@pytest.mark.parametrize("bad_state", ["ESTOP", "WATCHDOG"])
def test_devkit_fault_aborts(cfg, bad_state):
    fsm, w = ParkingFSM(cfg), World()
    fsm.start(0.0)
    run(fsm, w, until=(State.MANEUVER,))
    w.state = bad_state
    out = run(fsm, w)
    assert fsm.state == State.ABORT and bad_state in fsm.reason
    assert out.command.kind == "stop"


def test_stale_telemetry_aborts(cfg):
    fsm = ParkingFSM(cfg)
    fsm.start(0.0)
    tel = Telemetry(1, "OK", [50] * 6, 0.0, False, t_rx=0.0)
    fsm.update(FsmInputs(0.5, None, tel, GOOD_GAP))
    assert fsm.state == State.SEARCH
    fsm.update(FsmInputs(1.6, None, tel, GOOD_GAP))
    assert fsm.state == State.ABORT and "telemetry" in fsm.reason
    fsm2 = ParkingFSM(cfg)
    fsm2.start(0.0)
    fsm2.update(FsmInputs(0.1, None, None, GOOD_GAP))
    assert fsm2.state == State.ABORT


ALL_STATES = [State.IDLE, State.SEARCH, State.ALIGN, State.MEASURE, State.MANEUVER,
              State.ADJUST, State.PARKED]


@pytest.mark.parametrize("target", ALL_STATES)
def test_abort_from_every_state(cfg, target):
    fsm, w = ParkingFSM(cfg), World()
    if target != State.IDLE:
        fsm.start(0.0)
        if target != State.SEARCH:
            pose = Pose(angle_deg=20) if target == State.ALIGN else Pose()  # stays in ALIGN
            run(fsm, w, pose=pose, until=(target,))
    assert fsm.state == target
    out = fsm.abort("test", w.t)
    assert fsm.state == State.ABORT and out.command.kind == "stop"
    assert fsm.history[-1].src == target.value and fsm.history[-1].dst == "ABORT"
    # once aborted, update does nothing
    assert fsm.update(FsmInputs(w.t + 0.1, Pose(), w.tick(), GOOD_GAP)).command.kind == "none"


def test_restart_after_abort_and_reset(cfg):
    fsm = ParkingFSM(cfg)
    fsm.abort("x", 0.0)
    fsm.start(1.0)
    assert fsm.state == State.SEARCH
    fsm.reset()
    assert fsm.state == State.IDLE


def test_adjust_centres_with_pulses(cfg):
    fsm, w = ParkingFSM(cfg), World()
    fsm.start(0.0)
    run(fsm, w, until=(State.ADJUST,))
    w.front, w.rear = 10.0, 40.0  # off-centre by 15 cm
    run(fsm, w)
    assert fsm.state == State.PARKED
    assert abs(w.front - w.rear) <= cfg["planner"]["centre_tol_cm"]
    assert "centred" in fsm.history[-1].reason


def test_adjust_gives_up_after_max_iters(cfg):
    fsm, w = ParkingFSM(cfg), World()
    fsm.start(0.0)
    run(fsm, w, until=(State.ADJUST,))
    w.front, w.rear = 10.0, 40.0
    w.apply = lambda cmd: None  # the car never moves
    run(fsm, w)
    assert fsm.state == State.PARKED and "not reached" in fsm.history[-1].reason


def test_measure_counts_frames_once(cfg):
    fsm, w = ParkingFSM(cfg), World()
    fsm.start(0.0)
    for i in range(40):
        tel = w.tick()
        fsm.update(FsmInputs(w.t, Pose(), tel, GOOD_GAP, frame_id=1))  # same frame
    assert fsm.state == State.MEASURE
