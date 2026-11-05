"""UDP link between the laptop and the ESP32 DevKit (see docs/CONTRACT.md section 1).

Two layers live here:

* Pure encode/decode functions (``encode_drive``, ``decode_telemetry``, ...). They do
  no I/O and are unit-tested in ``tests/test_protocol.py``.
* :class:`DevkitLink`, which owns the UDP socket, the sequence counter, a receive
  thread for telemetry and a heartbeat thread that sends ``H`` at 10 Hz in every
  mode (also while stopped).

**Every motor command in the app goes through this module.**
"""
from __future__ import annotations

import json
import logging
import math
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

log = logging.getLogger(__name__)

PWM_LIMIT = 255
SEQ_MASK = 0xFFFFFFFF
VALID_STATES = ("OK", "ESTOP", "WATCHDOG")
TELEMETRY_KEYS = ("seq", "state", "d", "yaw", "busy")
SENSOR_NAMES = ("fl", "fr", "bl", "br", "l", "r")


# --------------------------------------------------------------------------- encode


def clamp_pwm(value: float) -> int:
    """Round ``value`` to an int and clamp it to -255..255."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0
    if math.isnan(v):
        return 0
    return int(max(-PWM_LIMIT, min(PWM_LIMIT, round(v))))


def _seq(seq: int) -> int:
    return int(seq) & SEQ_MASK


def encode_drive(seq: int, left: float, right: float) -> bytes:
    """``C,<seq>,<left>,<right>`` with both speeds clamped to -255..255."""
    return f"C,{_seq(seq)},{clamp_pwm(left)},{clamp_pwm(right)}\n".encode("ascii")


def encode_stop(seq: int) -> bytes:
    """``S,<seq>``: stop the motors now (also cancels a rotation)."""
    return f"S,{_seq(seq)}\n".encode("ascii")


def encode_heartbeat(seq: int) -> bytes:
    """``H,<seq>``: feeds the DevKit watchdog, never changes motion."""
    return f"H,{_seq(seq)}\n".encode("ascii")


def encode_rotate(seq: int, deg: float) -> bytes:
    """``R,<seq>,<deg>``: rotate in place, positive = CCW (left). One decimal."""
    d = float(deg)
    if math.isnan(d) or math.isinf(d):
        d = 0.0
    return f"R,{_seq(seq)},{d:.1f}\n".encode("ascii")


# --------------------------------------------------------------------------- decode


@dataclass
class Telemetry:
    """One decoded telemetry packet from the DevKit.

    ``d`` keeps the raw values from the packet: ``-1`` means "no echo" (it is NOT
    converted to ``None``). Order: fl, fr, bl, br, l, r.
    """

    seq: int
    state: str
    d: list[float]
    yaw: Optional[float]
    busy: bool
    t_rx: float = field(default_factory=time.monotonic)

    def sensor(self, name: str) -> float:
        """Distance of one sensor by name (``"fl"``, ``"r"``, ...)."""
        return self.d[SENSOR_NAMES.index(name)]

    def to_dict(self) -> dict[str, Any]:
        return {"seq": self.seq, "state": self.state, "d": list(self.d),
                "yaw": self.yaw, "busy": self.busy}


def _is_number(x: Any) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def decode_telemetry(data: bytes | str, t_rx: Optional[float] = None) -> Optional[Telemetry]:
    """Parse a telemetry datagram. Returns ``None`` for anything malformed."""
    try:
        if isinstance(data, bytes):
            data = data.decode("utf-8")
        obj = json.loads(data)
    except (UnicodeDecodeError, ValueError):
        return None
    if not isinstance(obj, dict) or any(k not in obj for k in TELEMETRY_KEYS):
        return None
    seq, state, d, yaw, busy = (obj[k] for k in TELEMETRY_KEYS)
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
        return None
    if state not in VALID_STATES:
        return None
    if not isinstance(d, list) or len(d) != 6 or not all(_is_number(v) for v in d):
        return None
    if yaw is not None and not _is_number(yaw):
        return None
    if not isinstance(busy, bool):
        return None
    return Telemetry(
        seq=seq,
        state=state,
        d=[float(v) for v in d],
        yaw=None if yaw is None else float(yaw),
        busy=busy,
        t_rx=time.monotonic() if t_rx is None else t_rx,
    )


# --------------------------------------------------------------------------- link

LogCallback = Callable[[str, dict], None]


class DevkitLink:
    """UDP socket to the DevKit with a heartbeat thread and a telemetry RX thread.

    Usage::

        link = DevkitLink("192.168.43.101").start()
        link.drive(150, 150)
        link.stop()
        link.close()

    ``log_cb(kind, fields)`` (optional) is called for every sent command
    (kind ``"cmd"``) and every received telemetry packet (kind ``"telem"``).
    """

    def __init__(
        self,
        devkit_ip: str,
        cmd_port: int = 4210,
        telem_port: int = 4211,
        heartbeat_hz: float = 10.0,
        bind_ip: str = "0.0.0.0",
        log_cb: Optional[LogCallback] = None,
    ) -> None:
        self.devkit_addr = (devkit_ip, int(cmd_port))
        self.heartbeat_period = 1.0 / max(1e-3, float(heartbeat_hz))
        self.log_cb = log_cb
        self._seq = 0
        self._seq_lock = threading.Lock()
        self._telem_lock = threading.Lock()
        self._latest: Optional[Telemetry] = None
        self._rx_count = 0
        self._stop_evt = threading.Event()
        self._threads: list[threading.Thread] = []

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if sys.platform == "win32" and hasattr(socket, "SIO_UDP_CONNRESET"):
            # Without this, Windows raises ConnectionResetError on recv after we sent
            # to a port where nobody listens (ICMP port unreachable).
            try:
                self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            except OSError:
                pass
        self.sock.bind((bind_ip, int(telem_port)))
        self.sock.settimeout(0.2)  # short timeout so Ctrl+C works on Windows
        self.telem_port = self.sock.getsockname()[1]

    # ---- lifecycle
    def start(self) -> "DevkitLink":
        """Start the RX and heartbeat threads. Returns self."""
        for name, target in (("devkit-rx", self._rx_loop), ("devkit-hb", self._hb_loop)):
            th = threading.Thread(target=target, name=name, daemon=True)
            th.start()
            self._threads.append(th)
        log.info("DevkitLink -> %s:%d, telemetry on :%d",
                 self.devkit_addr[0], self.devkit_addr[1], self.telem_port)
        return self

    def close(self) -> None:
        """Stop the threads and close the socket (does not send S; call stop() first)."""
        self._stop_evt.set()
        for th in self._threads:
            th.join(timeout=1.0)
        try:
            self.sock.close()
        except OSError:
            pass

    # ---- sending
    def next_seq(self) -> int:
        """Thread-safe, monotonically increasing uint32 sequence number."""
        with self._seq_lock:
            self._seq = (self._seq + 1) & SEQ_MASK
            return self._seq

    def _send(self, payload: bytes, fields: dict) -> None:
        try:
            self.sock.sendto(payload, self.devkit_addr)
        except OSError as exc:
            log.debug("UDP send failed: %s", exc)
        if self.log_cb is not None:
            try:
                self.log_cb("cmd", fields)
            except Exception:  # never let logging break the link
                log.exception("log callback failed")

    def drive(self, left: float, right: float) -> None:
        """Send ``C``: set the left/right motor PWM (-255..255)."""
        seq = self.next_seq()
        l, r = clamp_pwm(left), clamp_pwm(right)
        self._send(encode_drive(seq, l, r), {"type": "C", "seq": seq, "left": l, "right": r})

    def stop(self) -> None:
        """Send ``S``: stop now, cancels a rotation."""
        seq = self.next_seq()
        self._send(encode_stop(seq), {"type": "S", "seq": seq})

    def rotate(self, deg: float) -> None:
        """Send ``R``: rotate in place by ``deg`` (positive = CCW)."""
        seq = self.next_seq()
        self._send(encode_rotate(seq, deg), {"type": "R", "seq": seq, "deg": round(float(deg), 1)})

    def heartbeat(self) -> None:
        """Send ``H`` (normally done by the heartbeat thread)."""
        seq = self.next_seq()
        self._send(encode_heartbeat(seq), {"type": "H", "seq": seq})

    # ---- receiving
    def latest(self) -> Optional[Telemetry]:
        """Most recent valid telemetry packet, or ``None``."""
        with self._telem_lock:
            return self._latest

    def telemetry_age(self) -> float:
        """Seconds since the last valid telemetry (``inf`` if none yet)."""
        t = self.latest()
        return math.inf if t is None else max(0.0, time.monotonic() - t.t_rx)

    def is_live(self, timeout_s: float = 1.0) -> bool:
        """True if telemetry arrived within ``timeout_s``."""
        return self.telemetry_age() <= timeout_s

    @property
    def rx_count(self) -> int:
        return self._rx_count

    # ---- threads
    def _hb_loop(self) -> None:
        next_t = time.monotonic()
        while not self._stop_evt.is_set():
            self.heartbeat()
            next_t += self.heartbeat_period
            delay = next_t - time.monotonic()
            if delay < -1.0:  # we fell far behind (e.g. laptop suspended); resync
                next_t = time.monotonic()
                delay = 0
            self._stop_evt.wait(max(0.0, delay))

    def _rx_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                data, _addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except ConnectionResetError:
                continue
            except OSError:
                if self._stop_evt.is_set():
                    break
                time.sleep(0.05)
                continue
            telem = decode_telemetry(data)
            if telem is None:
                log.debug("ignored malformed telemetry: %r", data[:80])
                continue
            with self._telem_lock:
                self._latest = telem
                self._rx_count += 1
            if self.log_cb is not None:
                try:
                    self.log_cb("telem", telem.to_dict())
                except Exception:
                    log.exception("log callback failed")
