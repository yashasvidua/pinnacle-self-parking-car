"""Dashboard API via Flask's test client, with a fake link/camera/perception."""
import copy
import math
from dataclasses import dataclass

import numpy as np
import pytest

from pinacle.comms import Telemetry
from pinacle.config import load_config
from pinacle.web.context import AppContext
from pinacle.web.server import create_app


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class FakeLink:
    def __init__(self, clock):
        self.clock = clock
        self.sent = []
        self.telem = None

    def drive(self, l, r):
        self.sent.append(("C", l, r))

    def stop(self):
        self.sent.append(("S",))

    def rotate(self, deg):
        self.sent.append(("R", deg))

    def latest(self):
        return self.telem

    def telemetry_age(self):
        return math.inf if self.telem is None else self.clock() - self.telem.t_rx

    def give_telemetry(self):
        self.telem = Telemetry(1, "OK", [50, 50, 50, 50, -1, -1], 0.0, False, t_rx=self.clock())


class FakeSource:
    def __init__(self, live=True):
        self.live = live
        self.fid = 0

    def read(self):
        self.fid += 1
        return np.zeros((48, 64, 3), np.uint8), 0.0, self.fid

    def is_live(self, max_age_s=1.0):
        return self.live

    fps = 10.0


@dataclass
class Pose:
    x_cm: float = 60
    y_cm: float = -35
    angle_deg: float = 0
    confidence: float = 0.9


class FakeHomography:
    H = np.eye(3)

    def pixel_to_floor(self, pts):
        return np.asarray(pts, float).reshape(-1, 2)

    def floor_to_pixel(self, pts):
        return np.asarray(pts, float).reshape(-1, 2)


class FakePerception:
    def __init__(self):
        self.saved = None

    def make_detector(self, cfg, hom):
        class D:
            def detect(self, frame):
                class R:
                    pose = Pose()
                    method = "tape"
                    length_cm = 50.0
                    width_cm = 26.0
                return R()
        return D()

    def draw_overlay(self, frame, result, fsm_state="", extra_text=""):
        return frame.copy()

    def save_calibration(self, img, wld, path=None):
        self.saved = (img, wld, path)
        return FakeHomography()

    load_homography = None


@pytest.fixture
def env():
    clock = Clock()
    link = FakeLink(clock)
    cfg = copy.deepcopy(load_config())
    ctx = AppContext(cfg, link, FakeSource(), None, perception=FakePerception(),
                     config_path="unused.yaml", clock=clock)
    app = create_app(ctx)
    return ctx, app.test_client(), link, clock


def make_ready(ctx, link):
    link.give_telemetry()
    ctx.homography = FakeHomography()
    ctx.vision_tick()


def test_pages(env):
    ctx, c, *_ = env
    assert c.get("/").status_code == 200
    assert c.get("/calibrate").status_code == 200
    assert c.get("/static/app.js").status_code == 200
    assert c.get("/api/config").get_json()["repeat_ms"] == 100


def test_status_keys(env):
    ctx, c, *_ = env
    s = c.get("/api/status").get_json()
    for k in ("mode", "fsm", "telemetry", "camera", "detection", "find_park",
              "homography_loaded", "gap", "last_cmd"):
        assert k in s
    assert s["mode"] == "MANUAL" and s["fsm"]["state"] == "IDLE"
    assert s["find_park"]["enabled"] is False
    assert {"state", "step_index", "step_desc", "reason", "history"} <= set(s["fsm"])
    assert {"live", "age_s", "d", "yaw"} <= set(s["telemetry"])


def test_manual_drive_reaches_link(env):
    ctx, c, link, _ = env
    r = c.post("/api/manual", json={"cmd": "forward", "speed": 150}).get_json()
    assert r["left"] == r["right"] == 150
    assert link.sent[-1] == ("C", 150, 150)
    c.post("/api/manual", json={"cmd": "stop"})
    assert link.sent[-1] == ("S",)
    assert c.post("/api/manual", json={"cmd": "fly"}).status_code == 400


def test_find_park_rejected_when_not_ready(env):
    ctx, c, link, _ = env
    r = c.post("/api/find_park")
    assert r.status_code == 409
    assert "telemetry" in r.get_json()["reason"] and "calibration" in r.get_json()["reason"]
    link.give_telemetry()
    r = c.post("/api/find_park")
    assert r.status_code == 409 and "calibration" in r.get_json()["reason"]
    assert ctx.modes.mode.value == "MANUAL"


def test_find_park_then_manual_takes_over(env):
    ctx, c, link, clock = env
    make_ready(ctx, link)
    assert c.get("/api/status").get_json()["find_park"]["enabled"] is True
    assert c.post("/api/find_park").status_code == 200
    assert ctx.modes.mode.value == "AUTO" and ctx.fsm.state.value == "SEARCH"
    clock.t += 0.05
    link.give_telemetry()
    ctx.control_tick()
    assert ctx.fsm.active
    r = c.post("/api/manual", json={"cmd": "back"}).get_json()
    assert r["aborted_auto"] is True and r["mode"] == "MANUAL"
    assert ctx.fsm.state.value == "ABORT" and "manual" in ctx.fsm.reason
    assert link.sent[-1][0] == "C" and link.sent[-1][1] < 0


def test_estop(env):
    ctx, c, link, clock = env
    make_ready(ctx, link)
    c.post("/api/find_park")
    r = c.post("/api/estop").get_json()
    assert r["state"] == "ABORT" and r["mode"] == "MANUAL"
    assert link.sent[-1] == ("S",)
    # e-stop also works from IDLE/manual
    ctx.fsm.reset()
    c.post("/api/estop")
    assert ctx.fsm.state.value == "ABORT"


def test_ui_silence_aborts_auto(env):
    ctx, c, link, clock = env
    make_ready(ctx, link)
    c.post("/api/find_park")
    for _ in range(5):
        clock.t += 0.1
        link.give_telemetry()
        c.post("/api/heartbeat")
        ctx.control_tick()
    assert ctx.modes.mode.value == "AUTO"
    clock.t += 1.5
    link.give_telemetry()
    ctx.control_tick()
    assert ctx.fsm.state.value == "ABORT" and "silent" in ctx.fsm.reason
    assert ctx.modes.mode.value == "MANUAL"


def test_deadman_in_control_tick(env):
    ctx, c, link, clock = env
    c.post("/api/manual", json={"cmd": "forward"})
    clock.t += 0.1
    ctx.control_tick()
    assert link.sent[-1][0] == "C"
    clock.t += 0.3
    ctx.control_tick()
    assert link.sent[-1] == ("S",)


def test_calibration_endpoints(env):
    ctx, c, link, _ = env
    assert c.get("/api/calib/frame").status_code == 503  # no frame yet
    ctx.vision_tick()
    r = c.get("/api/calib/frame")
    assert r.status_code == 200 and r.mimetype == "image/jpeg"
    pts = [[1, 1], [10, 1], [10, 10], [1, 10]]
    r = c.post("/api/calib/save", json={"image_points": pts, "world_points_cm": pts})
    assert r.status_code == 200 and r.get_json()["mean_err_cm"] == 0
    assert ctx.homography is not None
    assert c.get("/api/calib/info").get_json()["loaded"] is True
    bad = c.post("/api/calib/save", json={"image_points": pts[:2], "world_points_cm": pts})
    assert bad.status_code == 400


def test_integration_sim_video_detects_bay():
    """Optional: real perception on the synthetic sim feed (skipped if perception is missing)."""
    calib = pytest.importorskip("pinacle.calib")
    detect = pytest.importorskip("pinacle.detect")
    from sim import synth
    H = calib.Homography.from_points(synth.SIM_CALIB_IMAGE_PTS, synth.SIM_CALIB_WORLD_PTS)
    det = detect.make_detector(load_config(), H)
    res = None
    for i in range(5):
        res = det.detect(synth.render_frame(i))
    assert res.pose is not None and res.pose.confidence >= 0.5
    assert res.pose.x_cm == pytest.approx(synth.BAY_CENTRE[0], abs=5)
    assert res.pose.y_cm == pytest.approx(synth.BAY_CENTRE[1], abs=5)
