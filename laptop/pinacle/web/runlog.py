"""Run logging: a CSV event log and an optional annotated-video recorder.

CSV columns: ``t, kind, fields...`` where ``t`` is seconds since the log was
opened, ``kind`` is one of ``telem``, ``cmd``, ``mode``, ``state``, ``event``,
and the remaining columns are ``key=value`` pairs (easy to read in a spreadsheet
and easy to split in pandas).
"""
from __future__ import annotations

import csv
import logging
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

log = logging.getLogger(__name__)


class CsvLogger:
    """Thread-safe CSV logger. ``log(kind, fields)`` appends one row."""

    def __init__(self, directory: str | Path, prefix: str = "run",
                 skip_heartbeats: bool = True) -> None:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = directory / f"{prefix}_{stamp}.csv"
        self._f = open(self.path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(["t", "kind", "fields..."])
        self._lock = threading.Lock()
        self._t0 = time.monotonic()
        self._last_flush = self._t0
        self.skip_heartbeats = skip_heartbeats
        log.info("CSV log: %s", self.path)

    def log(self, kind: str, fields: dict[str, Any]) -> None:
        if self.skip_heartbeats and kind == "cmd" and fields.get("type") == "H":
            return
        now = time.monotonic()
        row = [f"{now - self._t0:.3f}", kind]
        for k, v in fields.items():
            if isinstance(v, (list, tuple)):
                v = " ".join(str(x) for x in v)
            row.append(f"{k}={v}")
        with self._lock:
            if self._f.closed:
                return
            self._w.writerow(row)
            if now - self._last_flush > 1.0:
                self._f.flush()
                self._last_flush = now

    def close(self) -> None:
        with self._lock:
            if not self._f.closed:
                self._f.close()


class VideoRecorder:
    """Writes annotated frames to an MJPG ``.avi`` (opened lazily on the first frame)."""

    def __init__(self, directory: str | Path, fps: float = 15.0) -> None:
        self.directory = Path(directory)
        self.fps = fps if fps and fps > 0 else 15.0
        self.path: Optional[Path] = None
        self._writer = None
        self._lock = threading.Lock()

    def write(self, frame) -> None:
        import cv2
        with self._lock:
            if self._writer is None:
                self.directory.mkdir(parents=True, exist_ok=True)
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.path = self.directory / f"run_{stamp}.avi"
                h, w = frame.shape[:2]
                self._writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"MJPG"),
                                               self.fps, (w, h))
                log.info("recording video to %s", self.path)
            self._writer.write(frame)

    def close(self) -> None:
        with self._lock:
            if self._writer is not None:
                self._writer.release()
                self._writer = None
