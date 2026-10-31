"""Camera-to-floor calibration (a planar homography).

The ESP32-CAM is fixed on the car, looking forward and down at a flat floor.
A homography ``H`` (3x3) maps an image pixel ``(u, v)`` to a floor point
``(x, y)`` in the **car frame** (see docs/CONTRACT.md section 2):

    origin = floor point under the centre of the front bumper,
    +x forward, +y left, all in cm.

Calibrate once by putting 4 (or more) marks on the floor, measuring their car-frame
coordinates with a tape measure and clicking them in the image
(``laptop/tools/calib_capture.py`` or the dashboard calibration page).
The result is stored in the ``calibration:`` section of config.yaml.
"""
from __future__ import annotations

import itertools
import logging
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np

from pinacle.config import update_config_section

log = logging.getLogger(__name__)

# A triangle of three calibration points whose area is below this fraction of the
# squared point spread counts as collinear (= degenerate homography).
_COLLINEAR_REL_AREA = 1e-3

# Default bird's-eye view window, used when config keys are missing.
_BIRDSEYE_DEFAULTS = {"x_min_cm": 0.0, "x_max_cm": 150.0,
                      "y_min_cm": -75.0, "y_max_cm": 75.0, "px_per_cm": 3.0}


def _as_points(pts: Any, name: str) -> np.ndarray:
    """Convert to a float64 (N,2) array, raising ValueError on a wrong shape."""
    arr = np.asarray(pts, dtype=np.float64)
    if arr.ndim == 1 and arr.shape[0] == 2:
        arr = arr.reshape(1, 2)
    if arr.ndim != 2 or arr.shape[1] != 2:
        raise ValueError(f"{name} must have shape (N, 2), got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} contains NaN or inf")
    return arr


def _check_not_degenerate(pts: np.ndarray, name: str) -> None:
    """Raise ValueError if the points cannot define a homography.

    With exactly 4 points, no three of them may be collinear. With more points,
    they must at least not all lie on one line.
    """
    centred = pts - pts.mean(axis=0)
    spread = float(np.max(np.sum(centred ** 2, axis=1)))
    if spread < 1e-9:
        raise ValueError(f"{name}: all points are identical")
    if len(pts) == 4:
        for a, b, c in itertools.combinations(pts, 3):
            area = 0.5 * abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
            if area < _COLLINEAR_REL_AREA * spread:
                raise ValueError(f"{name}: three of the four points are (nearly) collinear")
    else:
        sv = np.linalg.svd(centred, compute_uv=False)
        if sv[1] < _COLLINEAR_REL_AREA * sv[0]:
            raise ValueError(f"{name}: all points are (nearly) collinear")


def birdseye_matrix(cfg_birdseye: dict | None) -> tuple[np.ndarray, tuple[int, int]]:
    """Return (B, (width, height)) where B maps floor cm -> bird's-eye image px.

    The bird's-eye image is oriented like a map seen from above the car:
    forward (+x) points **up** and left (+y) points **left**. So
    ``col = (y_max - y) * ppc`` and ``row = (x_max - x) * ppc``.
    """
    c = dict(_BIRDSEYE_DEFAULTS)
    c.update({k: v for k, v in (cfg_birdseye or {}).items() if v is not None})
    ppc = float(c["px_per_cm"])
    x_min, x_max = float(c["x_min_cm"]), float(c["x_max_cm"])
    y_min, y_max = float(c["y_min_cm"]), float(c["y_max_cm"])
    if ppc <= 0 or x_max <= x_min or y_max <= y_min:
        raise ValueError(f"invalid birdseye config: {c}")
    B = np.array([[0.0, -ppc, y_max * ppc],
                  [-ppc, 0.0, x_max * ppc],
                  [0.0, 0.0, 1.0]])
    size = (int(round((y_max - y_min) * ppc)), int(round((x_max - x_min) * ppc)))
    return B, size


class Homography:
    """Image px <-> floor cm (car frame) mapping.

    Attributes:
        H: 3x3 float64 matrix, image px -> floor cm (normalised so H[2,2] == 1).
    """

    def __init__(self, H: Any):
        H = np.asarray(H, dtype=np.float64)
        if H.shape != (3, 3) or not np.all(np.isfinite(H)):
            raise ValueError(f"homography must be a finite 3x3 matrix, got shape {H.shape}")
        if abs(np.linalg.det(H)) < 1e-12:
            raise ValueError("homography is singular")
        if abs(H[2, 2]) > 1e-12:
            H = H / H[2, 2]
        self.H: np.ndarray = H
        self._H_inv: np.ndarray = np.linalg.inv(H)

    @classmethod
    def from_points(cls, image_pts: Any, world_pts_cm: Any) -> "Homography":
        """Fit H from 4+ point pairs (image px, floor cm).

        Exactly 4 pairs: direct solution. More than 4: RANSAC (robust to one bad click).
        Raises ValueError on wrong shapes or a degenerate (collinear) layout.
        """
        img = _as_points(image_pts, "image_pts")
        wld = _as_points(world_pts_cm, "world_pts_cm")
        if len(img) != len(wld):
            raise ValueError(f"got {len(img)} image points but {len(wld)} world points")
        if len(img) < 4:
            raise ValueError(f"need at least 4 point pairs, got {len(img)}")
        _check_not_degenerate(img, "image_pts")
        _check_not_degenerate(wld, "world_pts_cm")
        if len(img) == 4:
            H, _ = cv2.findHomography(img, wld, 0)
        else:
            H, _ = cv2.findHomography(img, wld, cv2.RANSAC, 3.0)
        if H is None:
            raise ValueError("cv2.findHomography failed (degenerate points?)")
        return cls(H)

    # ------------------------------------------------------------------ mapping
    @staticmethod
    def _transform(M: np.ndarray, pts: Any) -> np.ndarray:
        arr = np.asarray(pts, dtype=np.float64)
        single = arr.ndim == 1
        arr2 = _as_points(arr, "pts")
        if len(arr2) == 0:
            return np.zeros((0, 2))
        out = cv2.perspectiveTransform(arr2.reshape(-1, 1, 2), M).reshape(-1, 2)
        return out[0] if single else out

    def pixel_to_floor(self, pts: Any) -> np.ndarray:
        """Image px -> floor cm. (N,2) in -> (N,2) out; a single (u, v) gives shape (2,)."""
        return self._transform(self.H, pts)

    def floor_to_pixel(self, pts: Any) -> np.ndarray:
        """Floor cm -> image px. (N,2) in -> (N,2) out; a single (x, y) gives shape (2,)."""
        return self._transform(self._H_inv, pts)

    # ------------------------------------------------------------------ helpers
    def birdseye(self, frame: np.ndarray, cfg_birdseye: dict | None) -> np.ndarray:
        """Warp a camera frame into a top-down view (forward = up, left = left).

        ``cfg_birdseye`` is the ``calibration.birdseye`` config section
        (x_min_cm, x_max_cm, y_min_cm, y_max_cm, px_per_cm).
        """
        B, size = birdseye_matrix(cfg_birdseye)
        return cv2.warpPerspective(frame, B @ self.H, size, flags=cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    def reprojection_error_cm(self, image_pts: Any, world_pts_cm: Any) -> float:
        """Mean distance (cm) between mapped image points and their measured floor points.

        Note: with exactly 4 points the fit is exact, so this is ~0. Click a 5th/6th
        mark (or check against extra marks) to get a meaningful number.
        """
        img = _as_points(image_pts, "image_pts")
        wld = _as_points(world_pts_cm, "world_pts_cm")
        err = np.linalg.norm(self.pixel_to_floor(img) - wld, axis=1)
        return float(err.mean()) if len(err) else 0.0

    def to_list(self) -> list[list[float]]:
        """H as nested lists of plain floats (for YAML/JSON)."""
        return [[float(v) for v in row] for row in self.H]

    def __repr__(self) -> str:
        return f"Homography({np.array2string(self.H, precision=4)})"


def load_homography(cfg: dict) -> Homography | None:
    """Build a Homography from ``cfg['calibration']['homography']``; None if not calibrated."""
    raw = ((cfg or {}).get("calibration") or {}).get("homography")
    if raw is None:
        return None
    try:
        return Homography(raw)
    except (ValueError, TypeError) as exc:
        log.warning("calibration.homography in config is invalid (%s); ignoring it", exc)
        return None


def save_calibration(image_pts: Any, world_pts_cm: Any,
                     path: str | Path | None = None) -> Homography:
    """Fit H from the point pairs and write the ``calibration:`` section of config.yaml.

    Writes image_points, world_points_cm and homography as nested lists of floats.
    ``path`` defaults to the repo's config.yaml. Returns the fitted Homography.
    """
    img = _as_points(image_pts, "image_pts")
    wld = _as_points(world_pts_cm, "world_pts_cm")
    hom = Homography.from_points(img, wld)
    values = {
        "image_points": [[float(u), float(v)] for u, v in img],
        "world_points_cm": [[float(x), float(y)] for x, y in wld],
        "homography": hom.to_list(),
    }
    update_config_section("calibration", values, path=path)
    log.info("saved calibration (%d points, reprojection error %.2f cm)",
             len(img), hom.reprojection_error_cm(img, wld))
    return hom


__all__: Sequence[str] = ["Homography", "load_homography", "save_calibration", "birdseye_matrix"]
