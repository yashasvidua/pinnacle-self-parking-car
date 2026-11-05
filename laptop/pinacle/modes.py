"""MANUAL / AUTO mode rules (pure logic, no I/O).

The rules, in order of importance:

(a) Any manual drive input during AUTO immediately aborts the automation and
    switches to MANUAL. **The human always wins.**
(b) When the parking FSM reaches PARKED or ABORT, the mode returns to MANUAL.
(c) Manual dead-man: if no manual input arrives for ``manual.deadman_ms`` while
    the car is moving, the caller must send a stop.
(d) FIND & PARK is only enabled when telemetry is fresh, the camera is live and a
    homography is loaded (:func:`find_park_enabled`).
(e) The e-stop is always allowed; the caller puts the FSM in ABORT.

The clock is injected (``clock=...``) so the tests can control time.
"""
from __future__ import annotations

import logging
import math
import time
from enum import Enum
from typing import Callable, Optional

log = logging.getLogger(__name__)

TERMINAL_FSM_STATES = ("PARKED", "ABORT")

# Manual commands understood by manual_command_to_lr (buttons and keyboard use these).
MANUAL_COMMANDS = (
    "forward", "back", "left", "right", "back_left", "back_right",
    "rotate_left", "rotate_right", "stop",
)
TURN_RATIO = 0.45  # inner wheel speed / outer wheel speed for a gentle turn


class Mode(str, Enum):
    MANUAL = "MANUAL"
    AUTO = "AUTO"


def find_park_enabled(telem_age_s: float, cam_live: bool, homography_loaded: bool,
                      cfg: dict) -> tuple[bool, str]:
    """Return ``(enabled, reason)`` for the FIND & PARK button.

    Enabled only if telemetry is newer than ``network.telemetry_timeout_s``, the
    camera is live AND a homography is loaded. ``reason`` lists what is missing
    (or ``"ready"``).
    """
    timeout = float(cfg.get("network", {}).get("telemetry_timeout_s", 1.0))
    missing = []
    if telem_age_s is None or math.isnan(telem_age_s) or telem_age_s > timeout:
        missing.append("no DevKit telemetry")
    if not cam_live:
        missing.append("camera not live")
    if not homography_loaded:
        missing.append("no calibration")
    if missing:
        return False, "Waiting for: " + ", ".join(missing)
    return True, "ready"


def manual_command_to_lr(cmd: str, speed: float, cfg: dict) -> tuple[int, int]:
    """Translate a dashboard command into (left, right) PWM.

    * forward/back: both sides at ``speed``.
    * left/right (and back_left/back_right): a gentle turn, the inner side runs at
      ``TURN_RATIO * speed`` but not below ``car.min_pwm`` (the BO motors stall).
    * rotate_left/rotate_right: opposite speeds (rotate_left = CCW: left back, right forward).
    """
    car = cfg.get("car", {})
    max_pwm = int(car.get("max_pwm", 255))
    min_pwm = int(car.get("min_pwm", 90))
    s = int(max(0, min(max_pwm, round(float(speed)))))
    inner = int(round(s * TURN_RATIO))
    if s >= min_pwm:
        inner = max(inner, min_pwm)
    table = {
        "forward": (s, s),
        "back": (-s, -s),
        "left": (inner, s),
        "right": (s, inner),
        "back_left": (-inner, -s),
        "back_right": (-s, -inner),
        "rotate_left": (-s, s),
        "rotate_right": (s, -s),
        "stop": (0, 0),
    }
    if cmd not in table:
        raise ValueError(f"unknown manual command: {cmd!r}")
    return table[cmd]


class ModeManager:
    """Holds the current :class:`Mode` and applies the mode rules.

    The manager never sends anything itself: its methods return what the caller
    should do (e.g. ``manual_input`` returns True when AUTO must be aborted).
    """

    def __init__(self, cfg: dict, clock: Callable[[], float] = time.monotonic) -> None:
        self.clock = clock
        self.deadman_s = float(cfg.get("manual", {}).get("deadman_ms", 250)) / 1000.0
        self.mode = Mode.MANUAL
        self.moving = False           # last manual command was non-zero
        self.last_input_t: Optional[float] = None
        self.changes: list[tuple[float, str, str, str]] = []  # (t, from, to, reason)

    # ---- helpers
    def _set_mode(self, mode: Mode, reason: str) -> bool:
        if mode == self.mode:
            return False
        self.changes.append((self.clock(), self.mode.value, mode.value, reason))
        log.info("mode %s -> %s (%s)", self.mode.value, mode.value, reason)
        self.mode = mode
        return True

    @property
    def is_auto(self) -> bool:
        return self.mode == Mode.AUTO

    # ---- rule (a): manual input
    def manual_input(self, left: float, right: float) -> bool:
        """Register a manual drive input (stop counts too).

        Returns True if the automation was running and must be aborted now; the
        mode is then already MANUAL.
        """
        aborted = False
        if self.mode == Mode.AUTO:
            self._set_mode(Mode.MANUAL, "manual input")
            aborted = True
        self.last_input_t = self.clock()
        self.moving = (left != 0 or right != 0)
        return aborted

    # ---- starting AUTO
    def start_auto(self) -> None:
        """Enter AUTO (the caller must have checked :func:`find_park_enabled`)."""
        self.moving = False
        self._set_mode(Mode.AUTO, "find & park")

    # ---- rule (b): FSM finished
    def on_fsm_state(self, state: str) -> bool:
        """Call with the FSM state each tick. Returns True if we went back to MANUAL."""
        state = getattr(state, "value", state)
        if self.mode == Mode.AUTO and state in TERMINAL_FSM_STATES:
            return self._set_mode(Mode.MANUAL, f"fsm {state}")
        return False

    # ---- rule (c): dead-man
    def deadman_due(self) -> bool:
        """True (once) if MANUAL, moving, and no input for ``deadman_ms``: send a stop."""
        if self.mode != Mode.MANUAL or not self.moving or self.last_input_t is None:
            return False
        if self.clock() - self.last_input_t >= self.deadman_s:
            self.moving = False
            return True
        return False

    # ---- rule (e): e-stop
    def estop(self) -> None:
        """E-stop is always allowed: go to MANUAL, not moving."""
        self.moving = False
        self._set_mode(Mode.MANUAL, "e-stop")

    def abort_auto(self, reason: str) -> bool:
        """Leave AUTO for any other reason (UI silent, link lost ...)."""
        self.moving = False
        return self._set_mode(Mode.MANUAL, reason)
