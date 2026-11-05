"""Synthetic camera feed + matching calibration for ``main.py --sim``.

The fake camera looks forward/right from the car at the floor. We do not model a
real lens; instead a plausible floor->image homography is defined by four point
pairs (a trapezoid: near floor at the bottom of the image, far floor squeezed
towards the top). The video shows a grey, slightly noisy floor with a white tape
rectangle (the bay, 50 x 26 cm, centred at x=60, y=-35 cm in the car frame, i.e.
on the right) and two dark parked boxes. Small per-frame jitter imitates vibration.

:data:`SIM_CALIB_IMAGE_PTS` / :data:`SIM_CALIB_WORLD_PTS` are four floor marks
with their exact pixel positions, so the sim can be calibrated automatically.
"""
from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

log = logging.getLogger(__name__)

IMG_W, IMG_H = 640, 480
FPS = 15.0
SECONDS = 8

# Floor (car frame cm) -> image px. Defines the virtual camera.
_WORLD_REF = np.float32([[20, 20], [20, -70], [120, -70], [120, 20]])
_IMAGE_REF = np.float32([[40, 470], [620, 470], [470, 120], [170, 120]])
H_WORLD_TO_IMG = cv2.getPerspectiveTransform(_WORLD_REF, _IMAGE_REF)

BAY_CENTRE = (60.0, -35.0)   # car frame, cm (front bumper origin)
BAY_LENGTH, BAY_WIDTH = 50.0, 26.0
TAPE_CM = 4.0

# Calibration floor marks (car frame cm) and their exact pixels in the sim video.
SIM_CALIB_WORLD_PTS = [[30.0, 0.0], [30.0, -60.0], [100.0, -60.0], [100.0, 0.0]]


def world_to_image(pts_cm) -> np.ndarray:
    """(N,2) floor cm -> (N,2) px through the sim camera."""
    p = np.asarray(pts_cm, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(p, H_WORLD_TO_IMG).reshape(-1, 2)


SIM_CALIB_IMAGE_PTS = [[round(float(u), 2), round(float(v), 2)]
                       for u, v in world_to_image(SIM_CALIB_WORLD_PTS)]


def _quad(cx: float, cy: float, length: float, width: float, ang_deg: float) -> np.ndarray:
    """Corners of a rectangle centred at (cx, cy) with its long axis at ang_deg."""
    a = math.radians(ang_deg)
    ux, uy = math.cos(a), math.sin(a)
    vx, vy = -uy, ux
    hl, hw = length / 2, width / 2
    return np.float32([[cx + sx * hl * ux + sy * hw * vx, cy + sx * hl * uy + sy * hw * vy]
                       for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])


def _fill_world_poly(img: np.ndarray, poly_cm: np.ndarray, color) -> None:
    px = world_to_image(poly_cm)
    cv2.fillConvexPoly(img, np.round(px * 16).astype(np.int32), color, cv2.LINE_AA, shift=4)


def render_frame(i: int, rng: Optional[np.random.Generator] = None) -> np.ndarray:
    """Render frame ``i`` of the synthetic feed (BGR, 640x480)."""
    rng = rng or np.random.default_rng(i)
    img = np.full((IMG_H, IMG_W, 3), (52, 50, 48), np.uint8)  # dark "background wall"
    # floor polygon: a big area of the floor
    floor = np.float32([[0, 60], [0, -110], [400, -110], [400, 60]])
    _fill_world_poly(img, floor, (112, 116, 118))
    # gentle vignette/gradient so it is not perfectly flat
    grad = np.linspace(0.85, 1.05, IMG_H, dtype=np.float32)[:, None, None]
    img = np.clip(img.astype(np.float32) * grad, 0, 255).astype(np.uint8)

    jx = 0.8 * math.sin(i * 0.37)
    jy = 0.6 * math.cos(i * 0.23)
    ja = 0.6 * math.sin(i * 0.11)
    bx, by = BAY_CENTRE[0] + jx, BAY_CENTRE[1] + jy

    # parked boxes (dark) in front of / behind the bay (outside the tape)
    for dx in (-55.0, 55.0):
        _fill_world_poly(img, _quad(bx + dx, by, 34, 20, ja), (40, 40, 70))

    # white tape: four strips forming the bay outline
    outer = _quad(bx, by, BAY_LENGTH + TAPE_CM, BAY_WIDTH + TAPE_CM, ja)
    inner = _quad(bx, by, BAY_LENGTH - TAPE_CM, BAY_WIDTH - TAPE_CM, ja)
    for k in range(4):
        strip = np.float32([outer[k], outer[(k + 1) % 4], inner[(k + 1) % 4], inner[k]])
        _fill_world_poly(img, strip, (238, 240, 240))

    noise = rng.normal(0, 4, img.shape).astype(np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    cv2.putText(img, "SIM CAMERA", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (200, 200, 200), 1, cv2.LINE_AA)
    return img


def _readable(path: Path) -> bool:
    cap = cv2.VideoCapture(str(path))
    ok, _ = cap.read()
    cap.release()
    return bool(ok)


def generate_video(path: str | Path, seconds: float = SECONDS, fps: float = FPS) -> Path:
    """Write the synthetic feed to ``path``. Returns the path actually written.

    Tries mp4 (``mp4v``); if that codec is unavailable, writes MJPG ``.avi`` next to
    it; as a last resort a folder of JPGs (FrameSource accepts all three).
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    n = int(seconds * fps)
    candidates = [(path, "mp4v"), (path.with_suffix(".avi"), "MJPG")]
    for out, codec in candidates:
        writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*codec), fps, (IMG_W, IMG_H))
        if not writer.isOpened():
            continue
        for i in range(n):
            writer.write(render_frame(i))
        writer.release()
        if _readable(out):
            log.info("synthetic sim video written: %s (%d frames)", out, n)
            return out
    folder = path.with_suffix("")
    folder.mkdir(parents=True, exist_ok=True)
    for i in range(n):
        cv2.imwrite(str(folder / f"frame_{i:04d}.jpg"), render_frame(i))
    log.info("synthetic sim frames written to folder %s", folder)
    return folder


def ensure_sim_video(path: str | Path) -> Path:
    """Return an existing sim video for ``path`` (or its .avi / folder fallback), else generate."""
    path = Path(path)
    for cand in (path, path.with_suffix(".avi"), path.with_suffix("")):
        if cand.is_file() and _readable(cand):
            return cand
        if cand.is_dir() and any(cand.glob("*.jpg")):
            return cand
    return generate_video(path)


if __name__ == "__main__":  # python laptop/sim/synth.py out.mp4
    import sys
    logging.basicConfig(level=logging.INFO)
    print(generate_video(sys.argv[1] if len(sys.argv) > 1 else "laptop/sim/sample.mp4"))
