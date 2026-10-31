"""Tests for pinacle.detect using synthetic camera images (no hardware needed).

A ground-truth homography defines a fake forward/down camera. The bay is drawn in
a high-resolution floor raster (car frame), then warped into the camera image.
"""
import math

import cv2
import numpy as np
import pytest

from pinacle.calib import Homography
from pinacle.detect import (BayPose, DetectionResult, draw_overlay, make_detector,
                            normalize_angle_deg)

RASTER_PPC = 4.0                       # floor raster resolution, px per cm
RASTER_X = (0.0, 200.0)                # cm covered by the floor raster
RASTER_Y = (-100.0, 100.0)


def make_camera(w: int = 320, h: int = 240) -> Homography:
    """Fake camera: near row (x=25 cm) fills the bottom, far row (x=130 cm) near the top."""
    s = w / 320.0
    img = np.array([[0, 239], [319, 239], [230, 40], [90, 40]], dtype=float) * s
    world = np.array([[25, 35], [25, -35], [130, -70], [130, 70]], dtype=float)
    return Homography.from_points(img, world)


def raster_to_floor_matrix() -> np.ndarray:
    """Floor-raster px (col, row) -> floor cm. col grows with -y, row grows with -x."""
    p = RASTER_PPC
    return np.array([[0, -1 / p, RASTER_X[1]],
                     [-1 / p, 0, RASTER_Y[1]],
                     [0, 0, 1]], dtype=float)


def floor_to_raster(pts_cm: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts_cm, float)
    return np.column_stack([(RASTER_Y[1] - pts[:, 1]) * RASTER_PPC,
                            (RASTER_X[1] - pts[:, 0]) * RASTER_PPC])


def blank_raster() -> np.ndarray:
    w = int((RASTER_Y[1] - RASTER_Y[0]) * RASTER_PPC)
    h = int((RASTER_X[1] - RASTER_X[0]) * RASTER_PPC)
    rng = np.random.default_rng(0)
    floor = rng.normal(115, 8, size=(h, w)).clip(0, 255).astype(np.uint8)
    return cv2.cvtColor(floor, cv2.COLOR_GRAY2BGR)


def render(raster: np.ndarray, cam: Homography, w: int = 320, h: int = 240) -> np.ndarray:
    """Warp the floor raster into the camera image (camera px <- floor cm <- raster px)."""
    M = np.linalg.inv(cam.H) @ raster_to_floor_matrix()
    img = cv2.warpPerspective(raster, M, (w, h), flags=cv2.INTER_AREA,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=(90, 90, 90))
    return cv2.GaussianBlur(img, (3, 3), 0)


def bay_corners(cx, cy, ang_deg, length=50.0, width=26.0) -> np.ndarray:
    t = math.radians(ang_deg)
    a = np.array([math.cos(t), math.sin(t)])
    b = np.array([-math.sin(t), math.cos(t)])
    c = np.array([cx, cy])
    hl, hw = length / 2, width / 2
    return np.array([c - hl * a - hw * b, c + hl * a - hw * b, c + hl * a + hw * b, c - hl * a + hw * b])


def make_cfg(method: str = "tape") -> dict:
    return {
        "detection": {
            "method": method, "min_confidence": 0.5, "smooth_alpha": 1.0,
            "tape": {"hsv_low": [0, 0, 180], "hsv_high": [180, 60, 255],
                     "canny_low": 50, "canny_high": 150, "hough_threshold": 40,
                     "hough_min_line_len": 30, "hough_max_line_gap": 10},
            "aruco": {"dictionary": "DICT_4X4_50", "corner_ids": [0, 1, 2, 3]},
        },
        "bay": {"side": "right", "length_cm": 50, "width_cm": 26, "length_tol_cm": 8},
    }


def angle_diff(a, b) -> float:
    return abs(normalize_angle_deg(a - b))


@pytest.mark.parametrize("cx,cy,ang", [(70, -15, 5), (65, -10, -8), (75, 0, 0)])
def test_tape_detector_recovers_pose(cx, cy, ang):
    cam = make_camera()
    corners = bay_corners(cx, cy, ang)
    px = cam.floor_to_pixel(corners)
    assert np.all((px >= 0) & (px < [320, 240])), "bay must be inside the test image"

    raster = blank_raster()
    tape_cm = 2.5
    cv2.polylines(raster, [np.round(floor_to_raster(corners)).astype(np.int32)], True,
                  (245, 245, 245), int(round(tape_cm * RASTER_PPC)), cv2.LINE_AA)
    frame = render(raster, cam)

    res = make_detector(make_cfg(), cam).detect(frame)
    assert res.method == "tape"
    assert res.pose is not None, f"no pose (lines={len(res.lines_px)})"
    assert abs(res.pose.x_cm - cx) < 3 and abs(res.pose.y_cm - cy) < 3, res.pose
    assert angle_diff(res.pose.angle_deg, ang) < 5, res.pose
    assert res.pose.confidence >= 0.5
    assert abs(res.length_cm - 50) < 8 and abs(res.width_cm - 26) < 8
    assert res.corners_px.shape == (4, 2) and res.corners_cm.shape == (4, 2)
    assert len(res.lines_px) > 0


def test_tape_without_homography_returns_no_pose():
    cam = make_camera()
    raster = blank_raster()
    cv2.polylines(raster, [np.round(floor_to_raster(bay_corners(70, -15, 0))).astype(np.int32)],
                  True, (245, 245, 245), 10)
    res = make_detector(make_cfg(), None).detect(render(raster, cam))
    assert res.pose is None
    assert len(res.lines_px) > 0          # lines still available for the overlay


def test_empty_image_gives_none():
    cam = make_camera()
    for method in ("tape", "aruco"):
        det = make_detector(make_cfg(method), cam)
        assert det.detect(np.full((240, 320, 3), 100, np.uint8)).pose is None
        assert det.detect(None).pose is None
        assert det.detect(np.zeros((0, 0, 3), np.uint8)).pose is None


def test_solid_white_blob_is_rejected():
    """A large filled white area is not a tape outline -> low confidence."""
    cam = make_camera()
    raster = blank_raster()
    cv2.fillPoly(raster, [np.round(floor_to_raster(bay_corners(70, -15, 0, 80, 60))).astype(np.int32)],
                 (250, 250, 250))
    res = make_detector(make_cfg(), cam).detect(render(raster, cam))
    assert res.pose is None


def _marker(dictionary, marker_id: int, size: int) -> np.ndarray:
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, size)
    return cv2.aruco.drawMarker(dictionary, marker_id, size)        # legacy API


def render_aruco(cam, corners_cm, ids, w, h, marker_cm=10.0):
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    else:
        d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    raster = blank_raster()
    size = int(marker_cm * RASTER_PPC)
    for mid, c in zip(ids, corners_cm):
        if mid is None:
            continue
        col, row = np.round(floor_to_raster(c[None])[0]).astype(int)
        q = size // 2 + 12                                 # white quiet zone
        raster[row - q:row + q, col - q:col + q] = 255
        m = cv2.cvtColor(_marker(d, mid, size), cv2.COLOR_GRAY2BGR)
        raster[row - size // 2:row - size // 2 + size, col - size // 2:col - size // 2 + size] = m
    return render(raster, cam, w, h)


@pytest.mark.parametrize("n_markers", [4, 3])
def test_aruco_detector(n_markers):
    w, h = 640, 480
    cam = make_camera(w, h)
    cx, cy, ang = 75.0, -12.0, 4.0
    corners = bay_corners(cx, cy, ang)
    # corner_ids order: entrance-left, entrance-right, back-right, back-left.
    # Seen from the car with the bay on the right: entrance = side nearer the car (+y side).
    ordered = np.array([corners[3], corners[2], corners[1], corners[0]])
    ids = [0, 1, 2, 3] if n_markers == 4 else [0, 1, None, 3]
    frame = render_aruco(cam, ordered, ids, w, h)

    res = make_detector(make_cfg("aruco"), cam).detect(frame)
    assert res.method == "aruco"
    assert res.pose is not None
    assert abs(res.pose.x_cm - cx) < 3 and abs(res.pose.y_cm - cy) < 3, res.pose
    assert angle_diff(res.pose.angle_deg, ang) < 5, res.pose
    assert res.pose.confidence == pytest.approx(1.0 if n_markers == 4 else 0.75)
    assert abs(res.length_cm - 50) < 4 and abs(res.width_cm - 26) < 4


def test_smoothing_blends_poses():
    cfg = make_cfg()
    cfg["detection"]["smooth_alpha"] = 0.5
    det = make_detector(cfg, make_camera())
    a = det._apply_smoothing(BayPose(60, -10, 80, 0.8))
    b = det._apply_smoothing(BayPose(62, -10, -88, 0.8))     # wraps across +-90
    assert a.x_cm == 60 and b.x_cm == pytest.approx(61)
    assert angle_diff(b.angle_deg, 86) < 1e-6


def test_draw_overlay_with_none_and_result():
    frame = np.zeros((240, 320, 3), np.uint8)
    out = draw_overlay(frame, None, "SEARCH", "fps 10")
    assert out.shape == frame.shape and out is not frame
    assert frame.sum() == 0                               # input not modified
    assert draw_overlay(None, None).shape == (240, 320, 3)

    res = DetectionResult(BayPose(70, -15, 3, 0.9), "tape", [(10, 10, 100, 100)],
                          np.array([[50, 50], [150, 50], [150, 120], [50, 120]], float),
                          None, 50.0, 26.0)
    out2 = draw_overlay(frame, res, "ALIGN")
    assert out2.sum() > 0
    gray = draw_overlay(np.zeros((240, 320), np.uint8), res)
    assert gray.shape == (240, 320, 3)
