"""Fake ESP32 DevKit: a UDP simulator that implements docs/CONTRACT.md section 1.

Run it standalone::

    python laptop/sim/fake_car.py --port 4210 --telem-port 4211 [--no-gyro] [-v]

or import it (tests, ``main.py --sim``)::

    car = FakeCar(cmd_port=0, telem_port=50000).start()   # 0 = pick a free port
    ...  send to ("127.0.0.1", car.cmd_port)
    car.stop()

What it simulates:

* parses ``C``/``S``/``H``/``R``; malformed lines are ignored (and don't feed the watchdog)
* 300 ms watchdog -> motors off, rotation cancelled, ``state = "WATCHDOG"``
* < 8 cm hard stop (ESTOP) per direction, latched, released by a command away from
  the obstacle or by the reading rising to >= 10 cm
* ``R`` rotations (``busy``), with a gyro (closed loop, 3 deg, 4 s timeout) or
  without (open loop for ``abs(deg) * ROTATE_MS_PER_DEG``; ``yaw`` = null)
* skid-steer kinematics in a small 2D world (a kerb wall and two parked boxes with
  a gap between them), six raycast sonars with a 3-sample median, -1 beyond 400 cm
* JSON telemetry at 10 Hz with exactly the 5 contract keys, sent to the IP of the
  last valid packet on ``telem_port``
"""
from __future__ import annotations

import argparse
import json
import logging
import math
import re
import socket
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

# Strict grammar, same as the firmware parser: no spaces, decimal only.
_LINE_RE = re.compile(
    r"^(?:C,(\d+),([+-]?\d+),([+-]?\d+)|([SH]),(\d+)|R,(\d+),([+-]?(?:\d+\.?\d*|\.\d+)))$")

log = logging.getLogger("fake_car")

WATCHDOG_S = 0.300
ESTOP_CM = 8.0
ESTOP_RELEASE_CM = 10.0
MAX_RANGE_CM = 400.0
ROTATE_TOL_DEG = 3.0
ROTATE_TIMEOUT_S = 4.0
ROTATE_PWM = 130

# Car geometry, centre-relative (cm). x forward, y left.
CAR_LENGTH = 30.0
CAR_WIDTH = 18.0
TRACK_CM = 18.0
# name -> (x, y, beam direction in car frame, degrees)
SENSORS = {
    "fl": (CAR_LENGTH / 2, 6.0, 0.0),
    "fr": (CAR_LENGTH / 2, -6.0, 0.0),
    "bl": (-CAR_LENGTH / 2, 6.0, 180.0),
    "br": (-CAR_LENGTH / 2, -6.0, 180.0),
    "l": (0.0, CAR_WIDTH / 2, 90.0),
    "r": (0.0, -CAR_WIDTH / 2, -90.0),
}
SENSOR_ORDER = ("fl", "fr", "bl", "br", "l", "r")
BEAM_HALF_DEG = 7.5  # HC-SR04 cone, approximated with 3 rays


# --------------------------------------------------------------------------- world

Segment = tuple[float, float, float, float]


def rect_segments(x0: float, y0: float, x1: float, y1: float) -> list[Segment]:
    """Four wall segments of an axis-aligned box."""
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def default_world() -> list[Segment]:
    """A street with a kerb on the right and a parking gap between two parked boxes.

    The car starts with its centre at (0, 0) heading +x. The gap is x = 40..112 cm,
    the kerb is at y = -60 cm. This matches the synthetic sim video, where the
    taped bay (50 x 26 cm) is centred at world (75, -35).
    """
    segs: list[Segment] = []
    segs.append((-200.0, -60.0, 400.0, -60.0))      # kerb wall on the right
    segs += rect_segments(5.0, -46.0, 40.0, -26.0)   # parked box behind the gap
    segs += rect_segments(112.0, -46.0, 147.0, -26.0)  # parked box in front of the gap
    return segs


def ray_distance(ox: float, oy: float, ang: float, segs: list[Segment]) -> float:
    """Distance from (ox, oy) along angle ``ang`` (rad) to the nearest segment, or inf."""
    dx, dy = math.cos(ang), math.sin(ang)
    best = math.inf
    for x1, y1, x2, y2 in segs:
        sx, sy = x2 - x1, y2 - y1
        den = dx * sy - dy * sx
        if abs(den) < 1e-12:
            continue
        qx, qy = x1 - ox, y1 - oy
        t = (qx * sy - qy * sx) / den   # along the ray
        u = (qx * dy - qy * dx) / den   # along the segment
        if t >= 0 and 0 <= u <= 1 and t < best:
            best = t
    return best


# --------------------------------------------------------------------------- car


@dataclass
class CarState:
    x: float = 0.0           # centre, world cm
    y: float = 0.0
    heading: float = 0.0     # degrees, CCW, unwrapped
    cmd_left: int = 0        # commanded PWM (what C/R asked for)
    cmd_right: int = 0
    rot_target: Optional[float] = None   # heading target (gyro) for R
    rot_end_t: Optional[float] = None    # open-loop end time / closed-loop timeout
    rot_open_loop: bool = False
    estop: bool = False
    estop_dir: str = ""      # "fwd" | "rev" | "rot"
    estop_hold_rot: bool = False  # an R was cancelled by ESTOP: stay latched until C/S/R
    watchdog: bool = True    # like the firmware: WATCHDOG until the first valid packet
    history: dict = field(default_factory=lambda: {k: [-1.0] * 3 for k in SENSOR_ORDER})


class FakeCar:
    """UDP DevKit simulator. ``start()`` launches the threads, ``stop()`` ends them."""

    def __init__(
        self,
        cmd_port: int = 4210,
        telem_port: int = 4211,
        bind_ip: str = "0.0.0.0",
        gyro: bool = True,
        world: Optional[list[Segment]] = None,
        start_pose: tuple[float, float, float] = (0.0, 0.0, 0.0),
        pwm_per_cm_s: float = 4.0,
        min_pwm: int = 90,
        rotate_ms_per_deg: Optional[float] = None,
        telem_hz: float = 10.0,
        sim_hz: float = 100.0,
        verbose: int = 1,
        telem_dest_ip: Optional[str] = None,
    ) -> None:
        self.telem_port = int(telem_port)
        self.gyro = gyro
        self.world = default_world() if world is None else world
        self.pwm_per_cm_s = pwm_per_cm_s
        self.min_pwm = min_pwm
        self.telem_period = 1.0 / telem_hz
        self.sim_period = 1.0 / sim_hz
        self.verbose = verbose
        # Open-loop rotation calibration: matches this sim's kinematics at ROTATE_PWM,
        # with a 5 % error so it behaves a bit like a real, uncalibrated car.
        omega_deg_s = math.degrees(2 * (ROTATE_PWM / pwm_per_cm_s) / TRACK_CM)
        self.rotate_ms_per_deg = (rotate_ms_per_deg if rotate_ms_per_deg is not None
                                  else 1000.0 / omega_deg_s * 1.05)

        self.state = CarState(x=start_pose[0], y=start_pose[1], heading=start_pose[2])
        self.lock = threading.RLock()
        self.last_rx_t: Optional[float] = None
        self.boot_t = time.monotonic()
        self.dest_ip: Optional[str] = telem_dest_ip
        self.last_seq = 0
        self.telem_seq = 0
        self.rx_counts = {"C": 0, "S": 0, "H": 0, "R": 0, "bad": 0}
        self.distances = [-1.0] * 6
        self._last_print = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if sys.platform == "win32" and hasattr(socket, "SIO_UDP_CONNRESET"):
            try:
                self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            except OSError:
                pass
        self.sock.bind((bind_ip, int(cmd_port)))
        self.sock.settimeout(0.05)
        self.cmd_port = self.sock.getsockname()[1]
        self._stop_evt = threading.Event()
        self._threads: list[threading.Thread] = []

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> "FakeCar":
        for name, fn in (("fakecar-rx", self._rx_loop), ("fakecar-sim", self._sim_loop)):
            th = threading.Thread(target=fn, name=name, daemon=True)
            th.start()
            self._threads.append(th)
        self._say(1, f"FakeCar listening on UDP :{self.cmd_port}, telemetry -> :{self.telem_port}"
                     f" (gyro {'on' if self.gyro else 'off'})")
        return self

    def stop(self) -> None:
        self._stop_evt.set()
        for th in self._threads:
            th.join(timeout=1.0)
        try:
            self.sock.close()
        except OSError:
            pass

    def _say(self, level: int, msg: str) -> None:
        if self.verbose >= level:
            print(f"[fake_car] {msg}", flush=True)

    # ------------------------------------------------------------------ inspection
    def pose(self) -> tuple[float, float, float]:
        with self.lock:
            return self.state.x, self.state.y, self.state.heading

    def set_pose(self, x: float, y: float, heading: float = 0.0) -> None:
        with self.lock:
            self.state.x, self.state.y, self.state.heading = x, y, heading
            for h in self.state.history.values():
                h[:] = [-1.0] * 3

    @property
    def busy(self) -> bool:
        with self.lock:
            return self.state.rot_end_t is not None

    def report_state(self) -> str:
        with self.lock:
            if self.state.estop:
                return "ESTOP"
            if self.state.watchdog:
                return "WATCHDOG"
            return "OK"

    # ------------------------------------------------------------------ packets
    def handle_line(self, line: str, now: Optional[float] = None) -> bool:
        """Apply one command line. Returns False (and ignores it) if malformed."""
        now = time.monotonic() if now is None else now
        line = line.rstrip("\r\n")
        parts = line.split(",")
        m = _LINE_RE.match(line)
        if not m:
            self.rx_counts["bad"] += 1
            return False

        kind = parts[0]
        seq = int(parts[1])
        if kind == "C":
            left, right = int(parts[2]), int(parts[3])
        elif kind == "R":
            deg = float(parts[2])
        with self.lock:
            st = self.state
            self.last_rx_t = now
            self.last_seq = seq
            st.watchdog = False
            self.rx_counts[kind] += 1
            if kind in ("C", "S", "R"):
                st.estop_hold_rot = False
            if kind == "C":
                self._cancel_rotation()  # first, it zeroes the motors
                st.cmd_left = max(-255, min(255, left))
                st.cmd_right = max(-255, min(255, right))
            elif kind == "S":
                st.cmd_left = st.cmd_right = 0
                self._cancel_rotation()
            elif kind == "R":
                self._start_rotation(deg, now)
        self._print_cmd(kind, parts)
        return True

    def _print_cmd(self, kind: str, parts: list[str]) -> None:
        if kind == "H":
            if self.verbose >= 3:
                self._say(3, ",".join(parts))
            return
        key = (kind, tuple(parts[2:]))
        if self.verbose >= 2 or key != self._last_print:
            self._say(1, f"rx {','.join(parts)}")
        self._last_print = key

    def _cancel_rotation(self) -> None:
        st = self.state
        if st.rot_end_t is not None:
            st.rot_target = None
            st.rot_end_t = None
            st.cmd_left = st.cmd_right = 0

    def _start_rotation(self, deg: float, now: float) -> None:
        st = self.state
        sign = 1 if deg >= 0 else -1
        st.cmd_left, st.cmd_right = -sign * ROTATE_PWM, sign * ROTATE_PWM
        if self.gyro:
            st.rot_open_loop = False
            st.rot_target = st.heading + deg
            st.rot_end_t = now + ROTATE_TIMEOUT_S
        else:
            st.rot_open_loop = True
            st.rot_target = None
            st.rot_end_t = now + abs(deg) * self.rotate_ms_per_deg / 1000.0
        if abs(deg) < 1e-6:
            self._cancel_rotation()

    # ------------------------------------------------------------------ physics
    def _sense(self) -> list[float]:
        """Raycast all six sonars (raw, before the median filter)."""
        st = self.state
        h = math.radians(st.heading)
        ch, sh = math.cos(h), math.sin(h)
        out = []
        for name in SENSOR_ORDER:
            sx, sy, sa = SENSORS[name]
            ox = st.x + sx * ch - sy * sh
            oy = st.y + sx * sh + sy * ch
            best = math.inf
            for off in (-BEAM_HALF_DEG, 0.0, BEAM_HALF_DEG):
                best = min(best, ray_distance(ox, oy, h + math.radians(sa + off), self.world))
            out.append(-1.0 if best > MAX_RANGE_CM else round(best, 1))
        return out

    def _filtered(self, raw: list[float]) -> list[float]:
        """3-sample median per sensor, like the firmware. -1 stays -1."""
        res = []
        for name, v in zip(SENSOR_ORDER, raw):
            hist = self.state.history[name]
            hist.append(v)
            del hist[:-3]
            res.append(round(statistics.median(hist), 1))
        return res

    @staticmethod
    def _direction(left: int, right: int) -> str:
        v = left + right
        if v > 0:
            return "fwd"
        if v < 0:
            return "rev"
        if left != 0:
            return "rot"
        return ""

    def _blocked(self, direction: str, d: list[float], limit: float) -> bool:
        fl, fr, bl, br = d[0], d[1], d[2], d[3]

        def near(x: float) -> bool:
            return 0 <= x < limit
        if direction == "fwd":
            return near(fl) or near(fr)
        if direction == "rev":
            return near(bl) or near(br)
        if direction == "rot":
            return any(near(x) for x in (fl, fr, bl, br))
        return False

    def _update_estop(self) -> None:
        st = self.state
        direction = self._direction(st.cmd_left, st.cmd_right)
        if not direction and st.estop_hold_rot:
            direction = "rot"
        d = self.distances
        if st.estop:
            if direction != st.estop_dir:          # zero or away from the obstacle
                st.estop = False
            elif not self._blocked(direction, d, ESTOP_RELEASE_CM):  # hysteresis
                st.estop = False
        if not st.estop and direction and self._blocked(direction, d, ESTOP_CM):
            st.estop = True
            st.estop_dir = direction
            self._say(1, f"HARD STOP ({direction}) d={d[:4]}")
        # ESTOP cancels any rotation (also one started while already latched);
        # a blocked C is held and resumes once the path clears, like the firmware.
        if st.estop and st.rot_end_t is not None:
            self._cancel_rotation()
            st.estop_hold_rot = True
        if not st.estop:
            st.estop_hold_rot = False

    def _step(self, now: float, dt: float) -> None:
        st = self.state
        # watchdog
        ref = self.last_rx_t if self.last_rx_t is not None else self.boot_t
        if now - ref > WATCHDOG_S and not st.watchdog:
            st.watchdog = True
            st.cmd_left = st.cmd_right = 0
            self._cancel_rotation()
            if self.last_rx_t is not None:
                self._say(1, "WATCHDOG: no packet for 300 ms -> motors off")

        # rotation bookkeeping
        if st.rot_end_t is not None:
            if now >= st.rot_end_t:
                self._cancel_rotation()
            elif st.rot_target is not None:
                remaining = st.rot_target - st.heading
                if abs(remaining) <= ROTATE_TOL_DEG:
                    self._cancel_rotation()
                else:
                    sign = 1 if remaining > 0 else -1
                    st.cmd_left, st.cmd_right = -sign * ROTATE_PWM, sign * ROTATE_PWM

        self._update_estop()
        left = 0 if st.estop else st.cmd_left
        right = 0 if st.estop else st.cmd_right

        # skid-steer kinematics with a motor dead band
        def wheel(p: int) -> float:
            return 0.0 if abs(p) < self.min_pwm else p / self.pwm_per_cm_s
        vl, vr = wheel(left), wheel(right)
        v = (vl + vr) / 2.0
        omega = math.degrees((vr - vl) / TRACK_CM)
        dtheta = omega * dt
        if st.rot_target is not None and st.rot_end_t is not None:
            remaining = st.rot_target - st.heading
            if abs(dtheta) > abs(remaining):     # don't overshoot within one sim step
                dtheta = remaining
        h = math.radians(st.heading + dtheta / 2)
        st.x += v * math.cos(h) * dt
        st.y += v * math.sin(h) * dt
        st.heading += dtheta

    def _telemetry(self) -> bytes:
        st = self.state
        yaw = round(st.heading, 1) if self.gyro else None
        msg = {"seq": self.telem_seq, "state": self.report_state(),
               "d": [int(v) if float(v).is_integer() else v for v in self.distances],
               "yaw": yaw, "busy": st.rot_end_t is not None}
        self.telem_seq = (self.telem_seq + 1) & 0xFFFFFFFF
        return json.dumps(msg, separators=(",", ":")).encode()

    # ------------------------------------------------------------------ threads
    def _rx_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                data, addr = self.sock.recvfrom(1024)
            except (socket.timeout, ConnectionResetError):
                continue
            except OSError:
                if self._stop_evt.is_set():
                    break
                continue
            try:
                text = data.decode("ascii")
            except UnicodeDecodeError:
                continue
            if self.handle_line(text):
                self.dest_ip = addr[0]

    def _sim_loop(self) -> None:
        last = time.monotonic()
        next_telem = last
        while not self._stop_evt.is_set():
            now = time.monotonic()
            dt = min(0.05, now - last)
            last = now
            with self.lock:
                self._step(now, dt)
                if now >= next_telem:
                    next_telem += self.telem_period
                    if next_telem < now:
                        next_telem = now + self.telem_period
                    self.distances = self._filtered(self._sense())
                    self._update_estop()
                    payload = self._telemetry() if self.dest_ip else None
                    dest = self.dest_ip
                else:
                    payload = None
            if payload is not None:
                try:
                    self.sock.sendto(payload, (dest, self.telem_port))
                except OSError:
                    pass
            self._stop_evt.wait(self.sim_period)


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Pinacle fake DevKit (UDP simulator)")
    ap.add_argument("--port", type=int, default=4210, help="command port to listen on")
    ap.add_argument("--telem-port", type=int, default=4211, help="telemetry destination port")
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--no-gyro", action="store_true", help="send yaw null, open-loop rotations")
    ap.add_argument("-v", "--verbose", action="count", default=1,
                    help="-v prints every C/R/S, -vv also heartbeats")
    ap.add_argument("-q", "--quiet", action="store_true")
    args = ap.parse_args(argv)
    car = FakeCar(cmd_port=args.port, telem_port=args.telem_port, bind_ip=args.bind,
                  gyro=not args.no_gyro, verbose=0 if args.quiet else args.verbose).start()
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        car.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
