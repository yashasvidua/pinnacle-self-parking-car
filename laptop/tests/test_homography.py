"""Tests for pinacle.calib (no hardware needed)."""
import shutil

import numpy as np
import pytest

from pinacle.calib import Homography, load_homography, save_calibration
from pinacle.config import CONFIG_PATH, load_config

# Four floor marks (cm, car frame) and where a forward/down camera sees them (px).
IMG_PTS = np.array([[60, 220], [260, 220], [200, 80], [120, 80]], dtype=float)
WORLD_PTS = np.array([[20, 20], [20, -20], [80, -25], [80, 25]], dtype=float)


def test_round_trip_px_cm_px():
    h = Homography.from_points(IMG_PTS, WORLD_PTS)
    px = np.array([[10, 200], [160, 120], [300, 230], [150, 60]], dtype=float)
    back = h.floor_to_pixel(h.pixel_to_floor(px))
    assert np.allclose(back, px, atol=1e-6)
    assert np.allclose(h.pixel_to_floor(IMG_PTS), WORLD_PTS, atol=1e-6)


def test_single_point_shape():
    h = Homography.from_points(IMG_PTS, WORLD_PTS)
    p = h.pixel_to_floor((60, 220))
    assert p.shape == (2,)
    assert np.allclose(p, [20, 20], atol=1e-6)
    assert h.pixel_to_floor([[60, 220]]).shape == (1, 2)


def test_known_homography_recovered():
    H_true = np.array([[0.02, -0.4, 120.0],
                       [-0.35, 0.01, 55.0],
                       [0.0002, 0.004, 1.0]])
    img = np.array([[30, 200], [290, 210], [250, 60], [70, 50]], dtype=float)
    true = Homography(H_true)
    world = true.pixel_to_floor(img)
    fit = Homography.from_points(img, world)
    assert np.allclose(fit.H, true.H, rtol=1e-6, atol=1e-8)
    assert fit.reprojection_error_cm(img, world) < 1e-4


def test_more_than_four_points_uses_ransac():
    true = Homography.from_points(IMG_PTS, WORLD_PTS)
    img = np.vstack([IMG_PTS, [[160, 150], [100, 190]]])
    world = true.pixel_to_floor(img)
    fit = Homography.from_points(img, world)
    assert fit.reprojection_error_cm(img, world) < 1e-3


@pytest.mark.parametrize("img,world", [
    ([[0, 0], [10, 10], [20, 20], [100, 0]], WORLD_PTS),          # 3 collinear px
    (IMG_PTS, [[0, 0], [10, 0], [20, 0], [30, 0]]),               # all collinear cm
    (IMG_PTS[:3], WORLD_PTS[:3]),                                  # too few
    (IMG_PTS, WORLD_PTS[:3]),                                      # count mismatch
    (np.zeros((4, 3)), WORLD_PTS),                                 # bad shape
])
def test_degenerate_input_raises(img, world):
    with pytest.raises(ValueError):
        Homography.from_points(img, world)


def test_save_and_load_calibration(tmp_path):
    cfg_path = tmp_path / "config.yaml"
    shutil.copy(CONFIG_PATH, cfg_path)
    h = save_calibration(IMG_PTS, WORLD_PTS, path=cfg_path)

    cfg = load_config(cfg_path)
    cal = cfg["calibration"]
    assert cal["image_points"] == IMG_PTS.tolist()
    assert cal["world_points_cm"] == WORLD_PTS.tolist()
    assert all(isinstance(v, float) for row in cal["homography"] for v in row)
    assert "birdseye" in cal                      # other keys preserved

    loaded = load_homography(cfg)
    assert loaded is not None
    assert np.allclose(loaded.H, h.H)


def test_load_homography_none_or_invalid():
    assert load_homography({"calibration": {"homography": None}}) is None
    assert load_homography({}) is None
    assert load_homography({"calibration": {"homography": [[1, 2], [3, 4]]}}) is None


def test_birdseye_orientation():
    h = Homography.from_points(IMG_PTS, WORLD_PTS)
    bcfg = {"x_min_cm": 0, "x_max_cm": 100, "y_min_cm": -50, "y_max_cm": 50, "px_per_cm": 2}
    # Paint the camera pixel that sees floor point (80, 25) (forward-left).
    frame = np.zeros((240, 320, 3), np.uint8)
    u, v = np.round(h.floor_to_pixel((80, 25))).astype(int)
    frame[v - 2:v + 3, u - 2:u + 3] = 255
    top = h.birdseye(frame, bcfg)
    assert top.shape == (200, 200, 3)
    rows, cols = np.nonzero(top[:, :, 0])
    # forward = up (row = (100-80)*2 = 40), left = left (col = (50-25)*2 = 50)
    assert abs(rows.mean() - 40) < 4 and abs(cols.mean() - 50) < 4
