"""Parking state machine (pure: no threads, no sockets, no clock inside).

States::

    IDLE -> SEARCH -> ALIGN -> MEASURE -> MANEUVER -> ADJUST -> PARKED
              \\________\\________\\__________\\_________\\____-> ABORT  (from any state)

Call :meth:`ParkingFSM.update` regularly (~20 Hz) with :class:`FsmInputs`; it
returns an :class:`Output` whose ``command`` the caller sends through comms.py.
Time ``t`` is always passed in (use ``time.monotonic()``; telemetry ``t_rx``
must come from the same clock).

* SEARCH:   creep forward at ``fsm.search_pwm`` until a bay with confidence
            >= ``detection.min_confidence`` is seen (``fsm.search_timeout_s`` -> ABORT).
* ALIGN:    differential steering until |bay angle| <= ``fsm.align_tol_deg``.
* MEASURE:  stop, average the pose over ``fsm.measure_frames`` frames, check the
            fused gap estimate (not ok -> ABORT), then plan (planner.py).
* MANEUVER: run the plan steps; drive = timed, reverse = until the rear sonar
            reads ``stop_cm`` (or the max distance), rotate = send ``R`` and wait
            for ``busy`` to go true then false. ``fsm.step_timeout_s`` per step.
* ADJUST:   up to ``fsm.adjust_max_iters`` short centring pulses -> PARKED.

Telemetry state ESTOP/WATCHDOG, or stale/missing telemetry, during an active
state -> ABORT.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional

from .fusion import GapEstimate, front_distance, rear_distance
from .planner import Step, plan_parallel_park, speed_cm_s

log = logging.getLogger(__name__)


class State(str, Enum):
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    ALIGN = "ALIGN"
    MEASURE = "MEASURE"
    MANEUVER = "MANEUVER"
    ADJUST = "ADJUST"
    PARKED = "PARKED"
    ABORT = "ABORT"


ACTIVE_STATES = (State.SEARCH, State.ALIGN, State.MEASURE, State.MANEUVER, State.ADJUST)

STATE_TEXT = {
    State.IDLE: "Idle",
    State.SEARCH: "Searching for a bay",
    State.ALIGN: "Aligning parallel to the bay",
    State.MEASURE: "Measuring the bay",
    State.MANEUVER: "Parking maneuver",
    State.ADJUST: "Centring in the bay",
    State.PARKED: "Parked",
    State.ABORT: "Aborted",
}


@dataclass
class Command:
    """What the caller should send: drive(l, r) / stop / rotate(deg) / none."""

    kind: str = "none"
    left: int = 0
    right: int = 0
    deg: float = 0.0

    @staticmethod
    def drive(left: float, right: float) -> "Command":
        return Command("drive", int(round(left)), int(round(right)))

    @staticmethod
    def stop() -> "Command":
        return Command("stop")

    @staticmethod
    def rotate(deg: float) -> "Command":
        return Command("rotate", deg=float(deg))

    @staticmethod
    def none() -> "Command":
        return Command("none")


@dataclass
class FsmInputs:
    """One tick of inputs. ``frame_id`` (optional) avoids counting a frame twice."""

    t: float
    pose: Any = None                 # BayPose or None
    telemetry: Any = None            # comms.Telemetry or None
    gap: Optional[GapEstimate] = None
    frame_id: Optional[int] = None


@dataclass
class Output:
    command: Command
    state: str
    step_index: Optional[int]
    step_desc: str
    reason: str = ""


@dataclass
class Transition:
    t: float
    src: str
    dst: str
    reason: str


@dataclass
class _AvgPose:
    """Averaged bay pose (same fields as perception's BayPose)."""

    x_cm: float
    y_cm: float
    angle_deg: float
    confidence: float


class ParkingFSM:
    """The automatic FIND & PARK sequence. See the module docstring."""

    def __init__(self, cfg: dict) -> None:
        self.cfg = cfg
        f = cfg.get("fsm", {})
        self.search_pwm = int(f.get("search_pwm", 120))
        self.search_timeout_s = float(f.get("search_timeout_s", 20))
        self.align_tol_deg = float(f.get("align_tol_deg", 5))
        self.align_gain = float(f.get("align_gain", 3.0))      # PWM per degree
        self.measure_frames = int(f.get("measure_frames", 10))
        self.step_timeout_s = float(f.get("step_timeout_s", 8))
        self.adjust_max_iters = int(f.get("adjust_max_iters", 4))
        self.settle_s = float(f.get("settle_s", 0.4))           # pause after each move
        # Sonar readings lag (3-sample median + 10 Hz telemetry): stop this much earlier.
        self.sonar_latency_s = float(f.get("sonar_latency_s", 0.3))
        self.min_confidence = float(cfg.get("detection", {}).get("min_confidence", 0.5))
        self.telemetry_timeout_s = float(cfg.get("network", {}).get("telemetry_timeout_s", 1.0))
        car = cfg.get("car", {})
        self.min_pwm = int(car.get("min_pwm", 90))
        self.max_pwm = int(car.get("max_pwm", 255))
        self.history: list[Transition] = []
        self.reset()

    # ------------------------------------------------------------------ public API
    def reset(self) -> None:
        """Back to IDLE, forget the plan (history is kept)."""
        self.state = State.IDLE
        self.reason = ""
        self.plan: list[Step] = []
        self.step_index: Optional[int] = None
        self.measured_pose: Optional[_AvgPose] = None
        self.t_last = 0.0
        self.state_t = 0.0
        self._samples: list[Any] = []
        self._last_frame_id: Optional[int] = None
        self._last_cmd_kind = "none"
        self._phase = ""
        self._phase_t = 0.0
        self._pulse_dur = 0.0
        self._pulse_dir = 0
        self._iters = 0
        self._busy_seen = False
        self._sent = False
        self._yaw_start: Optional[float] = None

    def start(self, t: float) -> None:
        """Begin FIND & PARK (from IDLE, PARKED or ABORT)."""
        if self.state in ACTIVE_STATES:
            return
        self.reset()
        self.t_last = t
        self._go(State.SEARCH, t, "start")

    def abort(self, reason: str, t: Optional[float] = None) -> Output:
        """Go to ABORT from any state. Returns an Output with a stop command."""
        t = self.t_last if t is None else t
        if self.state != State.ABORT:
            self.reason = reason
            self._go(State.ABORT, t, reason)
            log.warning("FSM abort: %s", reason)
        self._last_cmd_kind = "stop"
        return self._out(Command.stop())

    @property
    def active(self) -> bool:
        return self.state in ACTIVE_STATES

    def update(self, inp: FsmInputs) -> Output:
        """Advance the machine by one tick and return the command to send."""
        t = inp.t
        self.t_last = t
        if not self.active:
            return self._out(Command.none())

        tel = inp.telemetry
        if tel is None or (t - tel.t_rx) > self.telemetry_timeout_s:
            return self.abort("telemetry lost", t)
        if tel.state in ("ESTOP", "WATCHDOG"):
            return self.abort(f"DevKit reported {tel.state}", t)

        handler = {
            State.SEARCH: self._search,
            State.ALIGN: self._align,
            State.MEASURE: self._measure,
            State.MANEUVER: self._maneuver,
            State.ADJUST: self._adjust,
        }[self.state]
        cmd = handler(inp)
        if self.state == State.ABORT:  # handler aborted
            return self._out(Command.stop())
        return self._out(cmd)

    # ------------------------------------------------------------------ helpers
    def _go(self, new: State, t: float, reason: str = "") -> None:
        self.history.append(Transition(t, self.state.value, new.value, reason))
        log.info("FSM %s -> %s %s", self.state.value, new.value, f"({reason})" if reason else "")
        self.state = new
        self.state_t = t
        self._phase = ""
        self._phase_t = t

    def _out(self, cmd: Command) -> Output:
        if self.step_index is not None and self.plan and self.state in (
                State.MANEUVER, State.ADJUST, State.PARKED):
            desc = self.plan[min(self.step_index, len(self.plan) - 1)].description
        else:
            desc = STATE_TEXT[self.state]
        return Output(cmd, self.state.value, self.step_index, desc, self.reason)

    def _drive(self, left: float, right: float) -> Command:
        self._last_cmd_kind = "drive"
        return Command.drive(left, right)

    def _stop_once(self) -> Command:
        """Emit stop only if the last thing we commanded was not already a stop."""
        if self._last_cmd_kind == "stop":
            return Command.none()
        self._last_cmd_kind = "stop"
        return Command.stop()

    def _good(self, pose: Any) -> bool:
        return pose is not None and float(pose.confidence) >= self.min_confidence

    # ------------------------------------------------------------------ states
    def _search(self, inp: FsmInputs) -> Command:
        if inp.t - self.state_t > self.search_timeout_s:
            self.abort(f"no bay found within {self.search_timeout_s:g} s", inp.t)
            return Command.stop()
        if self._good(inp.pose):
            self._go(State.ALIGN, inp.t, f"bay seen (conf {inp.pose.confidence:.2f})")
            return self._align(inp)
        return self._drive(self.search_pwm, self.search_pwm)

    def _align(self, inp: FsmInputs) -> Command:
        if inp.t - self.state_t > self.step_timeout_s:
            self.abort("could not align with the bay", inp.t)
            return Command.stop()
        pose = inp.pose
        if not self._good(pose):
            return self._stop_once()  # wait for the bay to reappear
        angle = float(pose.angle_deg)
        if abs(angle) <= self.align_tol_deg:
            self._go(State.MEASURE, inp.t, f"aligned ({angle:+.1f} deg)")
            self._samples = []
            self._last_frame_id = None
            return self._stop_once()
        # Bay rotated CCW (positive) -> turn left -> right side faster.
        corr = self.align_gain * angle
        lo, hi = self.min_pwm, self.max_pwm
        left = max(lo, min(hi, self.search_pwm - corr))
        right = max(lo, min(hi, self.search_pwm + corr))
        return self._drive(left, right)

    def _measure(self, inp: FsmInputs) -> Command:
        if inp.t - self.state_t > self.step_timeout_s:
            self.abort("bay lost while measuring", inp.t)
            return Command.stop()
        new_frame = inp.frame_id is None or inp.frame_id != self._last_frame_id
        if self._good(inp.pose) and new_frame:
            self._samples.append(inp.pose)
            self._last_frame_id = inp.frame_id
        if len(self._samples) < self.measure_frames:
            return self._stop_once()
        n = len(self._samples)
        avg = _AvgPose(
            x_cm=sum(p.x_cm for p in self._samples) / n,
            y_cm=sum(p.y_cm for p in self._samples) / n,
            angle_deg=sum(p.angle_deg for p in self._samples) / n,
            confidence=sum(p.confidence for p in self._samples) / n,
        )
        self.measured_pose = avg
        gap = inp.gap
        if gap is None:
            self.abort("no gap estimate", inp.t)
            return Command.stop()
        if not gap.ok:
            from .fusion import required_gap_cm
            need = required_gap_cm(self.cfg)
            if gap.length_cm is None:
                self.abort("gap length unknown", inp.t)
            else:
                self.abort(f"gap too short: {gap.length_cm:.0f} cm < {need:.0f} cm "
                           f"({gap.source})", inp.t)
            return Command.stop()
        self.plan = plan_parallel_park(avg, self.cfg)
        self._go(State.MANEUVER, inp.t,
                 f"bay at x={avg.x_cm:.0f} y={avg.y_cm:.0f} cm, gap {gap.length_cm:.0f} cm")
        self._begin_step(0, inp)
        return self._maneuver(inp)

    # ---- maneuver
    def _step_timeout(self, step: Step) -> float:
        dur = float(step.params.get("duration_s", step.params.get("max_duration_s", 0.0)))
        return max(self.step_timeout_s, dur + 2.0)

    def _begin_step(self, i: int, inp: FsmInputs) -> None:
        self.step_index = i
        self._phase = "run"
        self._phase_t = inp.t
        self._sent = False
        self._busy_seen = False
        self._yaw_start = inp.telemetry.yaw if inp.telemetry is not None else None

    def _finish_step(self, t: float) -> Command:
        self._phase = "settle"
        self._phase_t = t
        return self._stop_once()

    def _maneuver(self, inp: FsmInputs) -> Command:
        t, tel = inp.t, inp.telemetry
        step = self.plan[self.step_index]

        if step.kind == "centre":
            self._go(State.ADJUST, t, "start centring")
            self._phase, self._phase_t, self._iters = "settle", t, 0
            return self._stop_once()

        if self._phase == "settle":
            if t - self._phase_t < self.settle_s:
                return self._stop_once()
            if self.step_index + 1 >= len(self.plan):
                self._go(State.PARKED, t, "plan complete")
                return self._stop_once()
            self._begin_step(self.step_index + 1, inp)
            return self._maneuver(inp)

        elapsed = t - self._phase_t
        if elapsed > self._step_timeout(step):
            self.abort(f"step {self.step_index + 1} timed out: {step.description}", t)
            return Command.stop()
        fresh = tel.t_rx >= self._phase_t

        if step.kind == "drive":
            if elapsed >= float(step.params["duration_s"]):
                return self._finish_step(t)
            p = step.params["pwm"]
            return self._drive(p, p)

        if step.kind == "reverse":
            rear = rear_distance(tel.d) if fresh else None
            # predict where the rear really is now, given the sonar lag
            lag_cm = speed_cm_s(step.params["pwm"], self.cfg) * self.sonar_latency_s
            if rear is not None and rear - lag_cm <= float(step.params["stop_cm"]):
                return self._finish_step(t)
            if elapsed >= float(step.params["max_duration_s"]):
                return self._finish_step(t)  # capped at reverse_max_cm
            p = step.params["pwm"]
            return self._drive(-p, -p)

        if step.kind == "rotate":
            if not self._sent:
                self._sent = True
                self._last_cmd_kind = "rotate"
                return Command.rotate(step.params["deg"])
            if tel.t_rx > self._phase_t:
                if tel.busy:
                    self._busy_seen = True
                elif self._busy_seen:
                    return self._finish_step(t)
                elif (self._yaw_start is not None and tel.yaw is not None and
                      abs(tel.yaw - self._yaw_start - float(step.params["deg"])) <= 5.0):
                    # Very short rotation finished between two telemetry packets.
                    return self._finish_step(t)
            return Command.none()

        raise ValueError(f"unknown step kind {step.kind}")  # pragma: no cover

    # ---- adjust
    def _adjust(self, inp: FsmInputs) -> Command:
        t, tel = inp.t, inp.telemetry
        step = self.plan[self.step_index]
        tol = float(step.params.get("tol_cm", 2))
        pwm = int(step.params.get("pwm", 110))
        max_iters = int(step.params.get("max_iters", self.adjust_max_iters))

        if t - self._phase_t > self.step_timeout_s:
            self.abort("centring timed out", t)
            return Command.stop()

        if self._phase == "pulse":
            if t - self._phase_t >= self._pulse_dur:
                self._phase, self._phase_t = "settle", t
                return self._stop_once()
            return self._drive(self._pulse_dir * pwm, self._pulse_dir * pwm)

        # settle: wait, then evaluate with telemetry received after the settle began
        if t - self._phase_t < self.settle_s or tel.t_rx < self._phase_t + self.settle_s * 0.5:
            return self._stop_once()
        front, rear = front_distance(tel.d), rear_distance(tel.d)
        if front is None or rear is None:
            self._go(State.PARKED, t, "centring skipped (no front/rear echo)")
            return self._stop_once()
        diff = front - rear
        if abs(diff) <= tol:
            self._go(State.PARKED, t, f"centred (front {front:.0f} / rear {rear:.0f} cm)")
            return self._stop_once()
        if self._iters >= max_iters:
            self._go(State.PARKED, t, f"centre tolerance not reached after {self._iters} corrections")
            return self._stop_once()
        error = diff / 2.0  # moving forward by e shrinks front by e and grows rear by e
        v = speed_cm_s(pwm, self.cfg)
        self._pulse_dir = 1 if error > 0 else -1
        self._pulse_dur = max(0.1, abs(error) / v) if v > 0 else 0.1
        self._iters += 1
        self._phase, self._phase_t = "pulse", t
        return self._drive(self._pulse_dir * pwm, self._pulse_dir * pwm)
