"""Tests for pinacle.stream.FrameSource with local files (no camera, no network)."""
import time

import cv2
import numpy as np

from pinacle.stream import FrameSource


def _wait_for(pred, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.02)
    return False


def _write_folder(path, n=5):
    path.mkdir()
    for i in range(n):
        img = np.full((60, 80, 3), i * 40, np.uint8)
        cv2.imwrite(str(path / f"frame_{i:03d}.jpg"), img)
    return path


def test_folder_source_yields_frames_and_loops(tmp_path):
    folder = _write_folder(tmp_path / "frames", n=3)
    src = FrameSource(str(folder), default_fps=50).start()
    try:
        assert _wait_for(lambda: src.read()[2] >= 6), "folder did not loop"
        frame, ts, fid = src.read()
        assert frame is not None and frame.shape == (60, 80, 3)
        assert ts <= time.monotonic() and fid >= 6
        assert src.is_live(1.0)
        assert src.fps > 10
    finally:
        src.stop()
    time.sleep(0.15)
    assert not src.is_live(0.1)              # stopped: frames get old


def test_folder_no_loop_finishes(tmp_path):
    folder = _write_folder(tmp_path / "frames", n=3)
    src = FrameSource(str(folder), loop=False, default_fps=100).start()
    try:
        assert _wait_for(lambda: src.finished)
        assert src.read()[2] == 3
    finally:
        src.stop()


def test_video_source(tmp_path):
    path = tmp_path / "clip.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 25, (80, 60))
    assert writer.isOpened()
    for i in range(10):
        writer.write(np.full((60, 80, 3), i * 20, np.uint8))
    writer.release()

    src = FrameSource(str(path)).start()
    try:
        assert _wait_for(lambda: src.read()[2] >= 12), "video did not loop"
        frame, _, _ = src.read()
        assert frame.shape == (60, 80, 3)
        assert src.is_live(0.5)
    finally:
        src.stop()


def test_missing_source_is_not_live(tmp_path):
    src = FrameSource(str(tmp_path / "nope.mp4"), reconnect_s=0.05).start()
    try:
        time.sleep(0.2)
        frame, _, fid = src.read()
        assert frame is None and fid == 0
        assert not src.is_live()
    finally:
        src.stop()
