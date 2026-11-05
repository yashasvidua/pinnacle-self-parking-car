"""Sensor fusion: vision bay pose + side/rear ultrasonics (pure, no I/O).

Sonar order (CONTRACT.md): ``d = [fl, fr, bl, br, l, r]``; ``-1`` = no echo.
The bay side comes from ``bay.side`` in the config.

* :func:`gap_clear` - is the bay-side sonar looking into an open gap?
* :class:`GapMeasurer` - integrates the distance travelled while the bay-side
  sonar reads "open", which gives a second, vision-independent gap length.
* :func:`estimate_gap` - combines that with the vision ``length_cm`` into a
  :class:`GapEstimate`.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

IDX = {"fl": 0, "fr": 1, "bl": 2, "br": 3, "l": 4, "r": 5}


def side_index(cfg: dict) -> int:
    """Index into ``d`` of the side sonar facing the bay."""
    side = str(cfg.get("bay", {}).get("side", "right")).lower()
    return IDX["l"] if side == "left" else IDX["r"]


def side_reading(d: Sequence[float], cfg: dict) -> float:
    """The bay-side sonar distance (cm, -1 = no echo)."""
    return float(d[side_index(cfg)])


def valid(v: Optional[float]) -> bool:
    """A real echo: not None and not -1 (any negative)."""
    return v is not None and v >= 0


def min_valid(*values: Optional[float]) -> Optional[float]:
    """Smallest real echo of ``values``, or None if none of them has one."""
    vs = [v for v in values if valid(v)]
    return min(vs) if vs else None


def front_distance(d: Sequence[float]) -> Optional[float]:
    return min_valid(d[IDX["fl"]], d[IDX["fr"]])


def rear_distance(d: Sequence[float]) -> Optional[float]:
    return min_valid(d[IDX["bl"]], d[IDX["br"]])


def is_open(side_cm: float, cfg: dict) -> bool:
    """Side reading counts as open if -1 (no echo) or > ``fusion.side_clear_cm``."""
    clear_cm = float(cfg.get("fusion", {}).get("side_clear_cm", 30))
    return side_cm < 0 or side_cm > clear_cm


def gap_clear(d: Sequence[float], cfg: dict) -> bool:
    """True if the bay-side sonar sees an open gap right now."""
    return is_open(side_reading(d, cfg), cfg)


def required_gap_cm(cfg: dict) -> float:
    """Minimum usable gap length: ``car.length_cm + fusion.min_gap_margin_cm``."""
    return float(cfg.get("car", {}).get("length_cm", 30)) + float(
        cfg.get("fusion", {}).get("min_gap_margin_cm", 8))


class GapMeasurer:
    """Measures the length of an open gap from the side sonar and odometry.

    Feed it ``update(t, side_cm, speed_cm_s)`` regularly (speed may be estimated
    from the commanded PWM). While the side sonar reads "open" the distance
    travelled is accumulated. Only *complete* gaps count, i.e. runs that started
    with a closed->open edge and ended with an open->closed edge; a run that was
    already open when measuring began (or is still open) is only a lower bound and
    is reported separately as ``current_run_cm``.
    """

    def __init__(self, cfg: dict) -> None:
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self.last_t: Optional[float] = None
        self.in_gap = False
        self.run_bounded = False      # current run started on a closed->open edge
        self.current_run_cm = 0.0
        self.runs: list[float] = []   # complete open runs (cm)

    def update(self, t: float, side_cm: float, speed_cm_s: float) -> None:
        """Integrate one sample. ``side_cm`` is the bay-side sonar reading."""
        open_now = is_open(side_cm, self.cfg)
        if self.last_t is not None and self.in_gap and open_now:
            self.current_run_cm += abs(speed_cm_s) * max(0.0, t - self.last_t)
        if self.last_t is None:
            self.run_bounded = False           # first sample: unknown history
        elif open_now and not self.in_gap:      # gap starts
            self.current_run_cm = 0.0
            self.run_bounded = True
        elif self.in_gap and not open_now:      # gap ends
            if self.run_bounded:
                self.runs.append(self.current_run_cm)
            self.current_run_cm = 0.0
            self.run_bounded = False
        self.in_gap = open_now
        self.last_t = t

    @property
    def length_cm(self) -> Optional[float]:
        """Longest complete gap measured so far, or None."""
        return max(self.runs) if self.runs else None


@dataclass
class GapEstimate:
    """Fused gap estimate.

    ``clear``: the bay-side sonar currently reads open (informational: during
    MEASURE the bay is usually still ahead of the side sensor).
    ``length_cm``: the gap length, the smaller of vision and sonar if both exist.
    ``source``: "vision" | "sonar" | "fusion" | "none".
    ``ok``: ``length_cm >= car.length_cm + fusion.min_gap_margin_cm``.
    """

    clear: bool
    length_cm: Optional[float]
    source: str
    ok: bool


def estimate_gap(vision_length_cm: Optional[float], sonar_length_cm: Optional[float],
                 d: Optional[Sequence[float]], cfg: dict) -> GapEstimate:
    """Combine the vision and sonar gap lengths (conservative: take the minimum)."""
    clear = gap_clear(d, cfg) if d is not None else False
    vis = vision_length_cm if vision_length_cm and vision_length_cm > 0 else None
    son = sonar_length_cm if sonar_length_cm and sonar_length_cm > 0 else None
    if vis is not None and son is not None:
        length, source = min(vis, son), "fusion"
    elif vis is not None:
        length, source = vis, "vision"
    elif son is not None:
        length, source = son, "sonar"
    else:
        length, source = None, "none"
    ok = length is not None and length >= required_gap_cm(cfg)
    return GapEstimate(clear=clear, length_cm=length, source=source, ok=ok)
