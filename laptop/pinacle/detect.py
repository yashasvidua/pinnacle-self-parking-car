"""Parking-bay detection: white tape rectangle (primary) or ArUco corner markers.

Both detectors return a :class:`DetectionResult` whose ``pose`` is the bay centre
and long-axis angle in the **car frame** (cm / degrees, +x forward, +y left, angle
CCW from +x, normalised to (-90, 90]). See docs/CONTRACT.md sections 2 and 3.

Tape pipeline (``detection.method: tape``)::

    HSV threshold -> morphology -> Canny -> HoughLinesP           (image px)
    tape pixels + Hough segments -> homography -> floor           (car frame cm)
    dominant orientation from segments (4*theta average, so the long sides and
    the short ends vote for the same rectangle) or cv2.minAreaRect as fallback
    -> robust extents along/across that direction -> bay rectangle
    -> confidence = size_match * (0.6 * edge_fit + 0.4 * line_support)

Several candidates (each large tape blob, and all blobs together) are fitted and
the most confident one wins, so a stray white object does not spoil the result.

ArUco pipeline (``detection.method: aruco``): markers with ``corner_ids`` (in the
order entrance-left, entrance-right, back-right, back-left) are detected, their
centres mapped to the floor, a missing 4th corner is inferred from 3, and the
rectangle gives the pose. Confidence: 4 markers = 1.0, 3 markers = 0.75.

The pose is EMA-smoothed across calls (``detection.smooth_alpha`` = weight of the
newest measurement) and dropped (pose=None) below ``detection.min_confidence``.
``detect()`` never raises.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from pinacle.calib import Homography

log = logging.getLogger(__name__)

# Floor points outside this window (cm, car frame) are ignored: they are above the
# horizon or too far away to be useful.
FLOOR_X_RANGE_CM = (-50.0, 400.0)
FLOOR_Y_ABS_MAX_CM = 300.0
MAX_POINTS_PER_CANDIDATE = 4000
MIN_COMPONENT_PX = 15
MIN_FLOOR_POINTS = 20
MAX_CANDIDATE_BLOBS = 4
ORIENTATION_TOL_DEG = 10.0
EXTENT_PERCENTILE = 1.0
SMOOTH_RESET_MISSES = 5         # restart the EMA after this many frames without a pose
SMOOTH_RESET_JUMP_CM = 30.0     # ... or if the bay jumps this far

COLOR_LINES = (0, 255, 0)        # green: Hough segments / marker outlines
COLOR_BAY = (255, 0, 255)        # magenta: accepted bay
COLOR_BAY_WEAK = (0, 165, 255)   # orange: bay rejected (confidence too low)
COLOR_TEXT = (255, 255, 255)


# ---------------------------------------------------------------------- data types
@dataclass
class BayPose:
    x_cm: float
    y_cm: float
    angle_deg: float
    confidence: float


@dataclass
class DetectionResult:
    pose: BayPose | None
    method: str                       # "tape" | "aruco"
    lines_px: list                    # [(x1, y1, x2, y2), ...] for the overlay
    corners_px: np.ndarray | None     # (4,2) bay corners in image px
    corners_cm: np.ndarray | None     # (4,2) bay corners in the car frame
    length_cm: float | None
    width_cm: float | None


def normalize_angle_deg(a: float) -> float:
    """Map an axis angle (180 deg periodic) into (-90, 90]."""
    a = ((float(a) + 90.0) % 180.0) - 90.0
    return 90.0 if a == -90.0 else a


def _rect_corners(cx: float, cy: float, angle_deg: float, length: float, width: float) -> np.ndarray:
    """Corners of a rectangle, counter-clockwise, starting at (-L/2, -W/2) in its own axes."""
    t = math.radians(angle_deg)
    a = np.array([math.cos(t), math.sin(t)])       # along the long axis
    b = np.array([-math.sin(t), math.cos(t)])      # 90 deg CCW from a
    c = np.array([cx, cy])
    hl, hw = length / 2.0, width / 2.0
    return np.array([c - hl * a - hw * b, c + hl * a - hw * b,
                     c + hl * a + hw * b, c - hl * a + hw * b])


def _empty_result(method: str, lines_px: list | None = None) -> DetectionResult:
    return DetectionResult(None, method, lines_px or [], None, None, None, None)


def _to_bgr(frame: np.ndarray) -> np.ndarray:
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


# ---------------------------------------------------------------------- base class
class _BaseDetector:
    """Shared config handling, EMA smoothing, confidence gate and error catching."""

    method = "base"

    def __init__(self, cfg: dict, homography: Homography | None):
        self.homography = homography
        det = (cfg or {}).get("detection") or {}
        bay = (cfg or {}).get("bay") or {}
        self.min_confidence = float(det.get("min_confidence", 0.5))
        self.alpha = float(np.clip(det.get("smooth_alpha", 0.5), 0.0, 1.0))
        self.bay_length = float(bay.get("length_cm", 50.0))
        self.bay_width = float(bay.get("width_cm", 26.0))
        self.length_tol = float(bay.get("length_tol_cm", 8.0))
        self.width_tol = float(bay.get("width_tol_cm", self.length_tol))
        self._smooth: BayPose | None = None
        self._misses = 0

    def reset(self) -> None:
        """Forget the smoothed pose (e.g. when a new parking attempt starts)."""
        self._smooth = None
        self._misses = 0

    def detect(self, frame: np.ndarray | None) -> DetectionResult:
        try:
            if frame is None or getattr(frame, "size", 0) == 0:
                result = _empty_result(self.method)
            else:
                result = self._detect_raw(_to_bgr(frame))
        except Exception:  # noqa: BLE001 - detection must never crash the control loop
            log.exception("%s detector failed", self.method)
            result = _empty_result(self.method)

        raw = result.pose
        if raw is not None and raw.confidence >= self.min_confidence:
            result.pose = self._apply_smoothing(raw)
            self._misses = 0
        else:
            result.pose = None
            self._misses += 1
        return result

    def _apply_smoothing(self, p: BayPose) -> BayPose:
        s = self._smooth
        if (s is None or self._misses >= SMOOTH_RESET_MISSES
                or math.hypot(p.x_cm - s.x_cm, p.y_cm - s.y_cm) > SMOOTH_RESET_JUMP_CM):
            self._smooth = BayPose(p.x_cm, p.y_cm, normalize_angle_deg(p.angle_deg), p.confidence)
        else:
            a = self.alpha
            d_ang = normalize_angle_deg(p.angle_deg - s.angle_deg)
            self._smooth = BayPose(
                s.x_cm + a * (p.x_cm - s.x_cm),
                s.y_cm + a * (p.y_cm - s.y_cm),
                normalize_angle_deg(s.angle_deg + a * d_ang),
                s.confidence + a * (p.confidence - s.confidence),
            )
        s = self._smooth
        return BayPose(s.x_cm, s.y_cm, s.angle_deg, s.confidence)

    def _size_score(self, length: float, width: float) -> float:
        dl = (length - self.bay_length) / max(self.length_tol, 1e-6)
        dw = (width - self.bay_width) / max(self.width_tol, 1e-6)
        return float(math.exp(-0.5 * (dl * dl + dw * dw)))

    def _detect_raw(self, frame: np.ndarray) -> DetectionResult:  # pragma: no cover
        raise NotImplementedError


# ---------------------------------------------------------------------- tape
class TapeDetector(_BaseDetector):
    """Detects a rectangle of coloured (default white) floor tape."""

    method = "tape"

    def __init__(self, cfg: dict, homography: Homography | None):
        super().__init__(cfg, homography)
        t = ((cfg or {}).get("detection") or {}).get("tape") or {}
        self.hsv_low = np.array(t.get("hsv_low", [0, 0, 180]), dtype=np.uint8)
        self.hsv_high = np.array(t.get("hsv_high", [180, 60, 255]), dtype=np.uint8)
        self.canny_low = int(t.get("canny_low", 50))
        self.canny_high = int(t.get("canny_high", 150))
        self.hough_threshold = int(t.get("hough_threshold", 40))
        self.hough_min_len = int(t.get("hough_min_line_len", 30))
        self.hough_max_gap = int(t.get("hough_max_line_gap", 10))
        # The rectangle is fitted to the outer tape edge; subtracting the tape width
        # gives the centre-line size. 0 = report the outer size.
        self.tape_width_cm = float(t.get("tape_width_cm", 0.0))

    def tape_mask(self, frame: np.ndarray) -> np.ndarray:
        """Binary mask (uint8 0/255) of tape-coloured pixels after clean-up."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_low, self.hsv_high)
        # Small kernels: far-away tape is only 2-3 px wide at QVGA.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        return mask

    def _hough(self, mask: np.ndarray) -> np.ndarray:
        edges = cv2.Canny(mask, self.canny_low, self.canny_high)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, self.hough_threshold,
                                minLineLength=self.hough_min_len, maxLineGap=self.hough_max_gap)
        if lines is None:
            return np.zeros((0, 4), dtype=np.float64)
        return lines.reshape(-1, 4).astype(np.float64)

    def _detect_raw(self, frame: np.ndarray) -> DetectionResult:
        mask = self.tape_mask(frame)
        segs_px = self._hough(mask)
        lines_px = [tuple(int(v) for v in s) for s in segs_px]
        if self.homography is None or cv2.countNonZero(mask) < MIN_FLOOR_POINTS:
            return _empty_result(self.method, lines_px)
        H = self.homography

        # Segments in floor coordinates (cm), with their midpoints.
        if len(segs_px):
            ends = H.pixel_to_floor(segs_px.reshape(-1, 2)).reshape(-1, 4)
            ok = np.all(np.isfinite(ends), axis=1) & _in_floor_window(ends[:, :2]) \
                & _in_floor_window(ends[:, 2:])
            segs_cm = ends[ok]
        else:
            segs_cm = np.zeros((0, 4))

        best: dict | None = None
        for pts_px in self._candidate_point_sets(mask):
            pts_cm = H.pixel_to_floor(pts_px)
            keep = np.all(np.isfinite(pts_cm), axis=1) & _in_floor_window(pts_cm)
            pts_cm = pts_cm[keep]
            if len(pts_cm) < MIN_FLOOR_POINTS:
                continue
            fit = self._fit_candidate(pts_cm, segs_cm)
            if fit is not None and (best is None or fit["confidence"] > best["confidence"]):
                best = fit

        if best is None:
            return _empty_result(self.method, lines_px)
        corners_cm = _rect_corners(best["cx"], best["cy"], best["angle"], best["length"], best["width"])
        return DetectionResult(
            pose=BayPose(best["cx"], best["cy"], best["angle"], best["confidence"]),
            method=self.method,
            lines_px=lines_px,
            corners_px=H.floor_to_pixel(corners_cm),
            corners_cm=corners_cm,
            length_cm=best["length"],
            width_cm=best["width"],
        )

    @staticmethod
    def _candidate_point_sets(mask: np.ndarray) -> list[np.ndarray]:
        """Pixel point sets (N,2 as u,v) to try: the largest blobs alone, and all together."""
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        blobs = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= MIN_COMPONENT_PX]
        blobs.sort(key=lambda i: stats[i, cv2.CC_STAT_AREA], reverse=True)
        if not blobs:
            return []
        sets = []
        label_sets = [[i] for i in blobs[:MAX_CANDIDATE_BLOBS]]
        if len(blobs) > 1:
            label_sets.append(blobs)
        for ids in label_sets:
            sel = np.isin(labels, ids)
            vs, us = np.nonzero(sel)
            pts = np.column_stack([us, vs]).astype(np.float64)
            if len(pts) > MAX_POINTS_PER_CANDIDATE:
                step = int(math.ceil(len(pts) / MAX_POINTS_PER_CANDIDATE))
                pts = pts[::step]
            sets.append(pts)
        return sets

    def _fit_candidate(self, pts_cm: np.ndarray, segs_cm: np.ndarray) -> dict | None:
        """Fit the bay rectangle to one candidate's floor points."""
        lo, hi = pts_cm.min(axis=0) - 5.0, pts_cm.max(axis=0) + 5.0
        if len(segs_cm):
            mid = 0.5 * (segs_cm[:, :2] + segs_cm[:, 2:])
            near = np.all((mid >= lo) & (mid <= hi), axis=1)
            segs = segs_cm[near]
        else:
            segs = segs_cm

        thetas = []
        if len(segs):
            thetas.append(_dominant_orientation(segs))
        (_, _), (_, _), mar_angle = cv2.minAreaRect(pts_cm.astype(np.float32))
        thetas.append(math.radians(mar_angle))       # robust fallback

        best = None
        for theta in thetas:
            fit = self._fit_with_orientation(pts_cm, theta)
            fit["support"] = _line_support(segs, theta)
            size = self._size_score(fit["length"], fit["width"])
            fit["confidence"] = float(np.clip(size * (0.6 * fit["edge_fit"] + 0.4 * fit["support"]), 0, 1))
            if best is None or fit["confidence"] > best["confidence"]:
                best = fit
        return best

    def _fit_with_orientation(self, pts: np.ndarray, theta: float) -> dict:
        """Robust bounding rectangle of the points, aligned with direction theta (rad)."""
        c, s = math.cos(theta), math.sin(theta)
        mean = pts.mean(axis=0)
        d = pts - mean
        u = d[:, 0] * c + d[:, 1] * s
        v = -d[:, 0] * s + d[:, 1] * c
        u_lo, u_hi = np.percentile(u, [EXTENT_PERCENTILE, 100 - EXTENT_PERCENTILE])
        v_lo, v_hi = np.percentile(v, [EXTENT_PERCENTILE, 100 - EXTENT_PERCENTILE])
        uc, vc = 0.5 * (u_lo + u_hi), 0.5 * (v_lo + v_hi)
        cx = mean[0] + uc * c - vc * s
        cy = mean[1] + uc * s + vc * c
        tw = self.tape_width_cm
        eu, ev = max(u_hi - u_lo - tw, 1e-3), max(v_hi - v_lo - tw, 1e-3)

        # Edge fit: fraction of tape points close to the rectangle outline.
        # A hollow tape rectangle scores ~1, a solid white blob scores low.
        hu, hv = eu / 2.0, ev / 2.0
        pu, pv = np.abs(u - uc), np.abs(v - vc)
        dist = np.minimum(np.abs(hu - pu), np.abs(hv - pv))
        tol = 3.0 + tw / 2.0
        edge_fit = float(np.mean(dist < tol))

        if eu >= ev:
            length, width, angle = eu, ev, math.degrees(theta)
        else:
            length, width, angle = ev, eu, math.degrees(theta) + 90.0
        return {"cx": float(cx), "cy": float(cy), "angle": normalize_angle_deg(angle),
                "length": float(length), "width": float(width), "edge_fit": edge_fit}


def _in_floor_window(pts: np.ndarray) -> np.ndarray:
    return ((pts[:, 0] >= FLOOR_X_RANGE_CM[0]) & (pts[:, 0] <= FLOOR_X_RANGE_CM[1])
            & (np.abs(pts[:, 1]) <= FLOOR_Y_ABS_MAX_CM))


def _segment_angles(segs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    d = segs[:, 2:4] - segs[:, 0:2]
    return np.arctan2(d[:, 1], d[:, 0]), np.hypot(d[:, 0], d[:, 1])


def _dominant_orientation(segs: np.ndarray) -> float:
    """Rectangle orientation (rad, mod 90 deg) from floor segments.

    Angles are multiplied by 4 before averaging: then a direction, its opposite
    and the perpendicular direction all coincide, so long sides and short ends of
    the bay vote together (a length-weighted circular mean).
    """
    ang, lens = _segment_angles(segs)
    c = float(np.sum(lens * np.cos(4 * ang)))
    s = float(np.sum(lens * np.sin(4 * ang)))
    return 0.25 * math.atan2(s, c)


def _line_support(segs: np.ndarray, theta: float) -> float:
    """Length fraction of segments parallel or perpendicular to theta (+-10 deg)."""
    if len(segs) == 0:
        return 0.0
    ang, lens = _segment_angles(segs)
    total = float(lens.sum())
    if total <= 0:
        return 0.0
    q = math.pi / 2
    diff = np.mod(ang - theta + q / 2, q) - q / 2
    return float(lens[np.abs(diff) < math.radians(ORIENTATION_TOL_DEG)].sum() / total)


# ---------------------------------------------------------------------- aruco
def _aruco_dictionary(name: str):
    aruco = cv2.aruco
    dict_id = getattr(aruco, name)
    if hasattr(aruco, "getPredefinedDictionary"):
        return aruco.getPredefinedDictionary(dict_id)
    return aruco.Dictionary_get(dict_id)            # OpenCV < 4.7


class _ArucoShim:
    """Same detect() call for the new (>= 4.7) and the legacy cv2.aruco API."""

    def __init__(self, dictionary):
        aruco = cv2.aruco
        self._dictionary = dictionary
        if hasattr(aruco, "ArucoDetector"):
            self._detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
            self._params = None
        else:
            self._detector = None
            self._params = aruco.DetectorParameters_create()

    def detect(self, gray: np.ndarray):
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dictionary, parameters=self._params)
        return corners, ids


class ArucoBayDetector(_BaseDetector):
    """Detects ArUco markers placed at the four bay corners."""

    method = "aruco"

    def __init__(self, cfg: dict, homography: Homography | None):
        super().__init__(cfg, homography)
        a = ((cfg or {}).get("detection") or {}).get("aruco") or {}
        self.corner_ids = [int(i) for i in a.get("corner_ids", [0, 1, 2, 3])]
        if len(self.corner_ids) != 4:
            raise ValueError("detection.aruco.corner_ids must list 4 ids")
        self._shim = _ArucoShim(_aruco_dictionary(str(a.get("dictionary", "DICT_4X4_50"))))

    def _detect_raw(self, frame: np.ndarray) -> DetectionResult:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self._shim.detect(gray)
        lines_px: list = []
        centres_px: dict[int, np.ndarray] = {}
        if ids is not None:
            for quad, mid in zip(corners, ids.flatten()):
                q = quad.reshape(4, 2)
                for k in range(4):
                    p, r = q[k], q[(k + 1) % 4]
                    lines_px.append((int(p[0]), int(p[1]), int(r[0]), int(r[1])))
                if int(mid) in self.corner_ids:
                    centres_px[self.corner_ids.index(int(mid))] = q.mean(axis=0)
        if self.homography is None or len(centres_px) < 3:
            return _empty_result(self.method, lines_px)

        H = self.homography
        idx = sorted(centres_px)
        floor = {k: H.pixel_to_floor(centres_px[k]) for k in idx}
        if len(floor) == 3:
            # Parallelogram: missing corner = its two neighbours minus the opposite one.
            k = ({0, 1, 2, 3} - set(floor)).pop()
            floor[k] = floor[(k - 1) % 4] + floor[(k + 1) % 4] - floor[(k + 2) % 4]
        corners_cm = np.array([floor[k] for k in range(4)], dtype=np.float64)
        p0, p1, p2, p3 = corners_cm
        len_a = 0.5 * (np.linalg.norm(p1 - p0) + np.linalg.norm(p2 - p3))
        len_b = 0.5 * (np.linalg.norm(p2 - p1) + np.linalg.norm(p3 - p0))
        if len_a >= len_b:
            direction, length, width = (p1 - p0) + (p2 - p3), len_a, len_b
        else:
            direction, length, width = (p2 - p1) + (p3 - p0), len_b, len_a
        angle = normalize_angle_deg(math.degrees(math.atan2(direction[1], direction[0])))
        centre = corners_cm.mean(axis=0)
        confidence = 1.0 if len(centres_px) == 4 else 0.75
        return DetectionResult(
            pose=BayPose(float(centre[0]), float(centre[1]), angle, confidence),
            method=self.method,
            lines_px=lines_px,
            corners_px=H.floor_to_pixel(corners_cm),
            corners_cm=corners_cm,
            length_cm=float(length),
            width_cm=float(width),
        )


# ---------------------------------------------------------------------- factory + overlay
def make_detector(cfg: dict, homography: Homography | None):
    """Return a detector (``.detect(frame) -> DetectionResult``) for ``detection.method``."""
    method = str(((cfg or {}).get("detection") or {}).get("method", "tape")).lower()
    if method == "aruco":
        try:
            return ArucoBayDetector(cfg, homography)
        except Exception:  # noqa: BLE001 - e.g. bad dictionary name; keep the car usable
            log.exception("cannot create the ArUco detector, falling back to tape")
    elif method != "tape":
        log.warning("unknown detection.method %r, using tape", method)
    return TapeDetector(cfg, homography)


def _put_text(img: np.ndarray, text: str, org: tuple[int, int], scale: float = 0.4) -> None:
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, COLOR_TEXT, 1, cv2.LINE_AA)


def draw_overlay(frame: np.ndarray | None, result: DetectionResult | None,
                 fsm_state: str = "", extra_text: str = "") -> np.ndarray:
    """Draw lines, bay polygon, pose text and an FSM banner on a copy of the frame."""
    if frame is None:
        out = np.zeros((240, 320, 3), dtype=np.uint8)
    else:
        out = _to_bgr(np.asarray(frame)).copy()
    h, w = out.shape[:2]
    scale = max(0.35, min(1.0, w / 800.0))
    line_h = int(round(45 * scale))        # banner height, ~18 px at QVGA

    if result is not None:
        for seg in result.lines_px or []:
            x1, y1, x2, y2 = (int(v) for v in seg)
            cv2.line(out, (x1, y1), (x2, y2), COLOR_LINES, 1, cv2.LINE_AA)
        if result.corners_px is not None:
            cp = np.asarray(result.corners_px, dtype=np.float64)
            if cp.shape == (4, 2) and np.all(np.isfinite(cp)) and np.all(np.abs(cp) < 1e5):
                color = COLOR_BAY if result.pose is not None else COLOR_BAY_WEAK
                cv2.polylines(out, [np.round(cp).astype(np.int32).reshape(-1, 1, 2)], True,
                              color, 2, cv2.LINE_AA)
        if result.pose is not None:
            p = result.pose
            txt = f"x={p.x_cm:.1f} y={p.y_cm:.1f} a={p.angle_deg:.1f} c={p.confidence:.2f}"
        else:
            txt = f"{result.method}: no bay"
        _put_text(out, txt, (4, h - 6), scale)

    banner = " | ".join(t for t in (fsm_state, extra_text) if t)
    if banner:
        cv2.rectangle(out, (0, 0), (w, line_h), (40, 40, 40), -1)
        _put_text(out, banner, (4, line_h - 5), scale)
    return out
