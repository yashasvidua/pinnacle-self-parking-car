"""Scripted parallel-parking plan for the skid-steer car (pure, no I/O).

The car can rotate in place, so instead of the classic arc the maneuver is:

1. **drive**   forward until the car centre is ``planner.pull_past_cm`` past the bay centre
2. **rotate**  +rotate_deg (bay on the right) or -rotate_deg (bay on the left), so the
               rear faces the bay (positive = CCW, CONTRACT.md)
3. **reverse** into the bay until the rear sensors read ``reverse_stop_cm``
               (but never more than ``reverse_max_cm``)
4. **rotate**  back by the opposite angle
5. **centre**  between the front and rear obstacles until |front - rear| <= centre_tol_cm

Distances are in the car frame: origin at the front bumper, +x forward, so the car
centre is at ``x = -car.length_cm / 2``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

STEP_KINDS = ("drive", "rotate", "reverse", "centre")


@dataclass
class Step:
    """One step of the plan. ``params`` depend on ``kind`` (see plan_parallel_park)."""

    kind: str
    params: dict[str, Any] = field(default_factory=dict)
    description: str = ""

    def __post_init__(self) -> None:
        if self.kind not in STEP_KINDS:
            raise ValueError(f"bad step kind {self.kind!r}")


def speed_cm_s(pwm: float, cfg: dict) -> float:
    """Rough ground speed for a PWM, from ``planner.pwm_per_cm_s``."""
    ppc = float(cfg.get("planner", {}).get("pwm_per_cm_s", 4.0))
    return abs(float(pwm)) / ppc if ppc > 0 else 0.0


def plan_parallel_park(bay_pose: Any, cfg: dict) -> list[Step]:
    """Build the step list for a bay at ``bay_pose`` (a BayPose: x_cm, y_cm, angle_deg).

    Step params:

    * drive:   distance_cm, pwm, duration_s (timed, from pwm_per_cm_s)
    * rotate:  deg
    * reverse: pwm, stop_cm, max_cm, max_duration_s
    * centre:  pwm, tol_cm, max_iters
    """
    pl = cfg.get("planner", {})
    car = cfg.get("car", {})
    side = str(cfg.get("bay", {}).get("side", "right")).lower()
    fsm = cfg.get("fsm", {})

    car_len = float(car.get("length_cm", 30))
    pull_past = float(pl.get("pull_past_cm", 12))
    drive_pwm = int(pl.get("drive_pwm", 150))
    reverse_pwm = int(pl.get("reverse_pwm", 130))
    rotate_deg = float(pl.get("rotate_deg", 90))
    reverse_stop = float(pl.get("reverse_stop_cm", 10))
    reverse_max = float(pl.get("reverse_max_cm", 40))
    centre_tol = float(pl.get("centre_tol_cm", 2))
    centre_pwm = int(pl.get("centre_pwm", 110))

    # Car centre sits at x = -L/2 in the car frame; it must end up at bay_x + pull_past.
    car_centre_x = -car_len / 2.0
    distance = max(0.0, float(bay_pose.x_cm) + pull_past - car_centre_x)
    v_drive = speed_cm_s(drive_pwm, cfg)
    v_rev = speed_cm_s(reverse_pwm, cfg)

    turn = rotate_deg if side == "right" else -rotate_deg
    side_word = "right" if side == "right" else "left"

    return [
        Step("drive", {"distance_cm": round(distance, 1), "pwm": drive_pwm,
                       "duration_s": distance / v_drive if v_drive > 0 else 0.0},
             f"Drive forward {distance:.0f} cm (centre {pull_past:.0f} cm past the bay)"),
        Step("rotate", {"deg": turn},
             f"Rotate {turn:+.0f} deg so the rear faces the {side_word} bay"),
        Step("reverse", {"pwm": reverse_pwm, "stop_cm": reverse_stop, "max_cm": reverse_max,
                         "max_duration_s": reverse_max / v_rev if v_rev > 0 else 0.0},
             f"Reverse in until rear reads {reverse_stop:.0f} cm (max {reverse_max:.0f} cm)"),
        Step("rotate", {"deg": -turn}, f"Rotate back {-turn:+.0f} deg"),
        Step("centre", {"pwm": centre_pwm, "tol_cm": centre_tol,
                        "max_iters": int(fsm.get("adjust_max_iters", 4))},
             f"Centre until |front - rear| <= {centre_tol:g} cm"),
    ]
