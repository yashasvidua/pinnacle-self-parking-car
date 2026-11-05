"""AppContext: the one shared, lock-protected state object of the laptop app.

The Flask routes, the vision thread and the control thread all go through the
methods here. Rules for MANUAL/AUTO come from :mod:`pinacle.modes`, the parking
logic from :mod:`pinacle.fsm`, and every motor command goes through the
``link`` (a :class:`pinacle.comms.DevkitLink`, or a fake in tests).

Dependencies are injected so the tests can use fakes:

* ``link``: object with ``drive(l, r)``, ``stop()``, ``rotate(deg)``, ``latest()``,
  ``telemetry_age()``
* ``source``: perception ``FrameSource`` (``read()``, ``is_live()``, ``fps``) or None
* ``perception``: object with ``make_detector``, ``draw_overlay``,
  ``save_calibration`` (any may be None); defaults to the real perception modules.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Callable, Optional

import numpy as np

from ..fsm import Command, FsmInputs, ParkingFSM
from ..fusion import GapEstimate, GapMeasurer, estimate_gap, required_gap_cm, side_reading
from ..modes import Mode, ModeManager, find_park_enabled, manual_command_to_lr
from ..planner import speed_cm_s

log = logging.getLogger(__name__)


class Perception:
    """Lazy access to the perception API (CONTRACT.md section 3).

    If a module is missing (e.g. still being written), the matching attributes
    are None and the app keeps running without that feature.
    """

    def __init__(self) -> None:
        self.make_detector: Optional[Callable] = None
        self.draw_overlay: Optional[Callable] = None
        self.save_calibration: Optional[Callable] = None
        self.load_homography: Optional[Callable] = None
        try:
            from .. import detect
            self.make_detector = detect.make_detector
            self.draw_overlay = detect.draw_overlay
        except Exception as exc:  # ImportError or a bug in the module
            log.warning("perception detect.py unavailable: %s", exc)
        try:
            from .. import calib
            self.save_calibration = calib.save_calibration
            self.load_homography = calib.load_homography
        except Exception as exc:
            log.warning("perception calib.py unavailable: %s", exc)


def encode_jpeg(frame: np.ndarray, quality: int = 70) -> Optional[bytes]:
    import cv2
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return buf.tobytes() if ok else None


def _num(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


class AppContext:
    """Shared state + actions. All public methods are thread-safe."""

    def __init__(
        self,
        cfg: dict,
        link: Any,
        source: Any = None,
        homography: Any = None,
        *,
        perception: Any = None,
        config_path: Any = None,
        sim: bool = False,
        csv_logger: Any = None,
        recorder: Any = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.cfg = cfg
        self.link = link
        self.source = source
        self.homography = homography
        self.perception = perception if perception is not None else Perception()
        self.config_path = config_path
        self.sim = sim
        self.csv = csv_logger
        self.recorder = recorder
        self.clock = clock

        self.lock = threading.RLock()
        self.frame_cond = threading.Condition()
        self.fsm = ParkingFSM(cfg)
        self.modes = ModeManager(cfg, clock=clock)
        self.gap_measurer = GapMeasurer(cfg)
        self.detector = self._make_detector()

        self.latest_result: Any = None
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_frame_id: Optional[int] = None
        self.latest_jpeg: Optional[bytes] = None
        self.jpeg_id = 0
        self.calib_frame: Optional[np.ndarray] = None
        self.last_ui_hb: Optional[float] = None
        self.last_drive = (0, 0)          # last commanded PWM, for odometry estimates
        self.last_cmd_text = "—"
        self.last_gap: Optional[GapEstimate] = None
        self.last_output = None
        self._logged_mode = self.modes.mode.value
        self._logged_state = self.fsm.state.value
        self._logged_hist = 0
        self._vision_fps = 0.0
        self._vision_t = None

    # ------------------------------------------------------------------ helpers
    def _make_detector(self) -> Any:
        mk = getattr(self.perception, "make_detector", None)
        if mk is None:
            return None
        try:
            return mk(self.cfg, self.homography)
        except Exception:
            log.exception("make_detector failed")
            return None

    def _event(self, kind: str, fields: dict) -> None:
        if self.csv is not None:
            self.csv.log(kind, fields)

    def _track_changes(self, reason: str = "") -> None:
        """Log mode/state changes to the CSV."""
        m, s = self.modes.mode.value, self.fsm.state.value
        if m != self._logged_mode:
            self._event("mode", {"from": self._logged_mode, "to": m, "reason": reason})
            self._logged_mode = m
        # log every FSM transition (several can happen within one tick)
        for h in self.fsm.history[self._logged_hist:]:
            self._event("state", {"from": h.src, "to": h.dst, "reason": h.reason})
        self._logged_hist = len(self.fsm.history)
        self._logged_state = s

    def _issue(self, cmd: Command) -> None:
        """Send an FSM command through the link."""
        if cmd.kind == "drive":
            self.link.drive(cmd.left, cmd.right)
            self.last_drive = (cmd.left, cmd.right)
            self.last_cmd_text = f"C {cmd.left},{cmd.right}"
        elif cmd.kind == "stop":
            self.link.stop()
            self.last_drive = (0, 0)
            self.last_cmd_text = "S"
        elif cmd.kind == "rotate":
            self.link.rotate(cmd.deg)
            self.last_drive = (0, 0)
            self.last_cmd_text = f"R {cmd.deg:+.1f}"

    def _homography_loaded(self) -> bool:
        return self.homography is not None

    def camera_live(self) -> bool:
        if self.source is None:
            return False
        timeout = float(self.cfg.get("camera", {}).get("live_timeout_s", 1.0))
        try:
            return bool(self.source.is_live(timeout))
        except Exception:
            return False

    def telemetry_age(self) -> float:
        try:
            return float(self.link.telemetry_age())
        except Exception:
            return math.inf

    def find_park_state(self) -> tuple[bool, str]:
        return find_park_enabled(self.telemetry_age(), self.camera_live(),
                                 self._homography_loaded(), self.cfg)

    # ------------------------------------------------------------------ actions
    def manual(self, cmd: Optional[str] = None, speed: Optional[float] = None,
               left: Optional[float] = None, right: Optional[float] = None) -> dict:
        """Manual drive input from the dashboard. During AUTO this aborts first."""
        if speed is None:
            speed = self.cfg.get("manual", {}).get("default_speed", 160)
        if cmd is not None:
            l, r = manual_command_to_lr(cmd, speed, self.cfg)
        else:
            from ..comms import clamp_pwm
            l, r = clamp_pwm(left or 0), clamp_pwm(right or 0)
        with self.lock:
            aborted = self.modes.manual_input(l, r)
            if aborted:
                self.fsm.abort("manual override", self.clock())
                self._event("event", {"what": "manual override aborted AUTO"})
            if l == 0 and r == 0:
                self._issue(Command.stop())
            else:
                self._issue(Command.drive(l, r))
            self._track_changes("manual input")
            return {"ok": True, "mode": self.modes.mode.value, "aborted_auto": aborted,
                    "left": l, "right": r}

    def find_park(self) -> tuple[bool, str]:
        """Start FIND & PARK if allowed. Returns (started, reason)."""
        with self.lock:
            enabled, reason = self.find_park_state()
            if not enabled:
                return False, reason
            if self.modes.is_auto and self.fsm.active:
                return True, "already running"
            now = self.clock()
            self.gap_measurer.reset()
            self.fsm.start(now)
            self.modes.start_auto()
            self.last_ui_hb = now
            self._track_changes("find & park")
            return True, "started"

    def estop(self, reason: str = "E-STOP pressed") -> dict:
        """Always allowed: send S immediately, FSM -> ABORT, mode -> MANUAL."""
        self.link.stop()   # first, before taking any lock
        with self.lock:
            self.last_drive = (0, 0)
            self.last_cmd_text = "S"
            self.fsm.abort(reason, self.clock())
            self.modes.estop()
            self._event("event", {"what": "estop", "reason": reason})
            self._track_changes(reason)
            return {"ok": True, "mode": self.modes.mode.value, "state": self.fsm.state.value}

    def ui_heartbeat(self) -> None:
        with self.lock:
            self.last_ui_hb = self.clock()

    # ------------------------------------------------------------------ control loop
    def control_tick(self) -> None:
        """One control-loop iteration (~20 Hz): FSM in AUTO, dead-man in MANUAL."""
        with self.lock:
            now = self.clock()
            telem = self.link.latest()

            # odometry-based gap measurement (only meaningful while moving straight)
            if telem is not None:
                l, r = self.last_drive
                min_pwm = int(self.cfg.get("car", {}).get("min_pwm", 90))
                moving = (l > 0 and r > 0) and min(l, r) >= min_pwm
                v = speed_cm_s((l + r) / 2, self.cfg) if moving else 0.0
                self.gap_measurer.update(now, side_reading(telem.d, self.cfg), v)

            if self.modes.mode == Mode.AUTO:
                ui_timeout = float(self.cfg.get("safety", {}).get("ui_heartbeat_s", 1.0))
                if self.last_ui_hb is None or now - self.last_ui_hb > ui_timeout:
                    self._issue(self.fsm.abort("dashboard went silent", now).command)
                else:
                    res = self.latest_result
                    pose = getattr(res, "pose", None)
                    length = getattr(res, "length_cm", None)
                    self.last_gap = estimate_gap(length, self.gap_measurer.length_cm,
                                                 telem.d if telem else None, self.cfg)
                    out = self.fsm.update(FsmInputs(now, pose, telem, self.last_gap,
                                                    self.latest_frame_id))
                    self.last_output = out
                    self._issue(out.command)
                self.modes.on_fsm_state(self.fsm.state)
            else:
                if self.modes.deadman_due():
                    log.info("manual dead-man: no input for %d ms -> stop",
                             int(self.modes.deadman_s * 1000))
                    self._issue(Command.stop())
                    self._event("event", {"what": "deadman stop"})
            self._track_changes()

    # ------------------------------------------------------------------ vision
    def vision_tick(self) -> bool:
        """Grab the newest frame, detect, draw overlay, encode JPEG once. True if new."""
        if self.source is None:
            return False
        frame, _ts, fid = self.source.read()
        if frame is None or fid == self.latest_frame_id:
            return False
        with self.lock:
            detector = self.detector
            state = self.fsm.state.value
            mode = self.modes.mode.value
            step = self.last_output.step_desc if self.last_output and self.fsm.active else ""
        result = None
        if detector is not None:
            try:
                result = detector.detect(frame)
            except Exception:
                log.exception("detector failed")
        draw = getattr(self.perception, "draw_overlay", None)
        extra = f"{mode}  {step}".strip()
        if draw is not None:
            try:
                annotated = draw(frame, result, state, extra)
            except Exception:
                log.exception("draw_overlay failed")
                annotated = frame
        else:
            annotated = frame
        jpeg = encode_jpeg(annotated, int(self.cfg.get("web", {}).get("jpeg_quality", 70)))
        if self.recorder is not None:
            try:
                self.recorder.write(annotated)
            except Exception:
                log.exception("video recorder failed")
        now = time.monotonic()
        with self.lock:
            self.latest_result = result
            self.latest_frame = frame
            self.latest_frame_id = fid
            if self._vision_t is not None:
                dt = now - self._vision_t
                if dt > 0:
                    self._vision_fps = 0.9 * self._vision_fps + 0.1 * (1.0 / dt)
            self._vision_t = now
        if jpeg is not None:
            with self.frame_cond:
                self.latest_jpeg = jpeg
                self.jpeg_id += 1
                self.frame_cond.notify_all()
        return True

    def wait_jpeg(self, last_id: int, timeout: float = 1.0) -> tuple[int, Optional[bytes]]:
        """Block until a JPEG newer than ``last_id`` exists (or timeout)."""
        with self.frame_cond:
            if self.jpeg_id == last_id:
                self.frame_cond.wait(timeout)
            return self.jpeg_id, self.latest_jpeg

    # ------------------------------------------------------------------ status
    def status(self) -> dict:
        """Everything the dashboard shows, as a JSON-able dict."""
        with self.lock:
            telem = self.link.latest()
            age = self.telemetry_age()
            tmo = float(self.cfg.get("network", {}).get("telemetry_timeout_s", 1.0))
            enabled, reason = self.find_park_state()
            res = self.latest_result
            pose = getattr(res, "pose", None)
            out = self.last_output
            fsm = self.fsm
            step_count = len(fsm.plan)
            gap = self.last_gap
            return {
                "mode": self.modes.mode.value,
                "fsm": {
                    "state": fsm.state.value,
                    "active": fsm.active,
                    "step_index": fsm.step_index,
                    "step_count": step_count,
                    "step_desc": out.step_desc if (out and fsm.active) else
                    fsm._out(Command.none()).step_desc,
                    "reason": fsm.reason,
                    "history": [{"t": round(h.t, 2), "from": h.src, "to": h.dst,
                                 "reason": h.reason} for h in fsm.history[-8:]],
                },
                "telemetry": {
                    "live": age <= tmo,
                    "age_s": None if math.isinf(age) else round(age, 2),
                    "state": telem.state if telem else None,
                    "d": list(telem.d) if telem else None,
                    "yaw": telem.yaw if telem else None,
                    "busy": telem.busy if telem else False,
                    "seq": telem.seq if telem else None,
                },
                "camera": {
                    "live": self.camera_live(),
                    "fps": round(self._vision_fps, 1),
                },
                "detection": {
                    "method": getattr(res, "method", None),
                    "confidence": round(float(pose.confidence), 2) if pose else None,
                    "pose": ({"x_cm": round(float(pose.x_cm), 1), "y_cm": round(float(pose.y_cm), 1),
                              "angle_deg": round(float(pose.angle_deg), 1)} if pose else None),
                    "length_cm": _num(getattr(res, "length_cm", None)),
                    "width_cm": _num(getattr(res, "width_cm", None)),
                },
                "gap": {
                    "sonar_cm": self.gap_measurer.length_cm,
                    "length_cm": gap.length_cm if gap else None,
                    "source": gap.source if gap else None,
                    "ok": gap.ok if gap else None,
                    "required_cm": required_gap_cm(self.cfg),
                },
                "find_park": {"enabled": enabled, "reason": reason},
                "homography_loaded": self._homography_loaded(),
                "last_cmd": self.last_cmd_text,
                "sim": self.sim,
            }

    # ------------------------------------------------------------------ calibration
    def calib_freeze(self) -> Optional[bytes]:
        """Freeze the latest raw camera frame for the calibration page; returns a JPEG."""
        with self.lock:
            if self.latest_frame is None:
                return None
            self.calib_frame = self.latest_frame.copy()
            frame = self.calib_frame
        return encode_jpeg(frame, 90)

    def calib_info(self) -> dict:
        c = self.cfg.get("calibration", {}) or {}
        return {"loaded": self._homography_loaded(), "image_points": c.get("image_points"),
                "world_points_cm": c.get("world_points_cm"), "sim": self.sim}

    def calib_save(self, image_pts: list, world_pts: list) -> dict:
        """Save 4 point pairs via perception's save_calibration, reload, report errors."""
        save = getattr(self.perception, "save_calibration", None)
        if save is None:
            raise RuntimeError("calibration module (calib.py) is not available")
        img = np.asarray(image_pts, dtype=float).reshape(-1, 2)
        wld = np.asarray(world_pts, dtype=float).reshape(-1, 2)
        if len(img) < 4 or len(img) != len(wld) or not (np.isfinite(img).all()
                                                        and np.isfinite(wld).all()):
            raise ValueError("need 4 image points and 4 matching world points")
        H = save(img.tolist(), wld.tolist(), path=self.config_path)
        err_cm = np.linalg.norm(np.asarray(H.pixel_to_floor(img), float).reshape(-1, 2) - wld, axis=1)
        err_px = np.linalg.norm(np.asarray(H.floor_to_pixel(wld), float).reshape(-1, 2) - img, axis=1)
        with self.lock:
            self.homography = H
            cal = self.cfg.setdefault("calibration", {})
            cal["image_points"] = img.tolist()
            cal["world_points_cm"] = wld.tolist()
            cal["homography"] = np.asarray(H.H, float).tolist()
            self.detector = self._make_detector()
            self._event("event", {"what": "calibration saved",
                                  "mean_err_cm": round(float(err_cm.mean()), 3)})
        return {
            "ok": True,
            "mean_err_cm": round(float(err_cm.mean()), 3),
            "max_err_cm": round(float(err_cm.max()), 3),
            "mean_err_px": round(float(err_px.mean()), 3),
            "max_err_px": round(float(err_px.max()), 3),
            "per_point_cm": [round(float(e), 3) for e in err_cm],
        }

    # ------------------------------------------------------------------ shutdown
    def shutdown(self) -> None:
        """Send S and close the loggers (threads are stopped by main.py)."""
        try:
            self.link.stop()
        except Exception:
            log.exception("stop on shutdown failed")
        if self.recorder is not None:
            self.recorder.close()
        if self.csv is not None:
            self.csv.close()
