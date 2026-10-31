"""Threaded frame source: ESP32-CAM MJPEG stream, a video file or a folder of images.

Usage::

    src = FrameSource("http://192.168.43.100/stream").start()
    frame, t, frame_id = src.read()     # non-blocking, latest frame only
    ...
    src.stop()

* **HTTP** (``http://`` / ``https://``): the multipart MJPEG stream is read with
  ``requests`` and the JPEGs are cut out manually between the SOI (``FF D8``) and
  EOI (``FF D9``) markers, then decoded with ``cv2.imdecode``. This is more reliable
  on Windows than ``cv2.VideoCapture(url)``. On any error, or when no bytes arrive
  for ``stall_s`` seconds, it reconnects with exponential backoff.
* **Video file**: read with ``cv2.VideoCapture`` at its native fps (``default_fps``
  if the file does not say), looping if ``loop=True``.
* **Folder** of JPG/PNG files: played in name order at ``default_fps``, looping.
* A bare integer string (``"0"``) opens a local webcam (handy for bench tests).

The reader thread never raises; problems are logged with ``logging``.
"""
from __future__ import annotations

import logging
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import requests

log = logging.getLogger(__name__)

IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp")
SOI = b"\xff\xd8"
EOI = b"\xff\xd9"
MAX_BUFFER_BYTES = 2_000_000      # drop the buffer if no complete JPEG fits in this
MAX_BACKOFF_S = 10.0
FPS_EMA_ALPHA = 0.2


class FrameSource:
    """Background reader that always holds the latest frame."""

    def __init__(self, source: str, reconnect_s: float = 1.0, loop: bool = True, *,
                 default_fps: float = 10.0, stall_s: float = 2.0, connect_timeout_s: float = 3.0):
        self.source = str(source)
        self.reconnect_s = max(0.05, float(reconnect_s))
        self.loop = loop
        self.default_fps = float(default_fps) if default_fps and default_fps > 0 else 10.0
        self.stall_s = float(stall_s)
        self.connect_timeout_s = float(connect_timeout_s)

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._frame: np.ndarray | None = None
        self._ts = 0.0
        self._frame_id = 0
        self._fps = 0.0
        self._response: requests.Response | None = None
        self._finished = False          # file source ended (loop=False)

    # ------------------------------------------------------------------ public API
    def start(self) -> "FrameSource":
        if self._thread is not None and self._thread.is_alive():
            return self
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="FrameSource", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        resp = self._response
        if resp is not None:
            try:
                resp.close()        # unblocks a pending socket read
            except Exception:       # noqa: BLE001 - closing is best effort
                pass
        if self._thread is not None:
            self._thread.join(timeout=self.stall_s + 1.0)
            if self._thread.is_alive():
                log.warning("FrameSource thread did not stop in time (it is a daemon, ignoring)")
        self._thread = None

    def read(self) -> tuple[np.ndarray | None, float, int]:
        """Return (latest BGR frame or None, time.monotonic() timestamp, frame_id).

        The returned array is never modified by the reader thread (a new array is
        stored for each frame), but treat it as read-only or copy it.
        """
        with self._lock:
            return self._frame, self._ts, self._frame_id

    def is_live(self, max_age_s: float = 1.0) -> bool:
        with self._lock:
            return self._frame is not None and (time.monotonic() - self._ts) <= max_age_s

    @property
    def fps(self) -> float:
        """Exponential moving average of the received frame rate."""
        with self._lock:
            return self._fps

    @property
    def finished(self) -> bool:
        """True when a non-looping file source has played to the end."""
        return self._finished

    def __enter__(self) -> "FrameSource":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    # ------------------------------------------------------------------ internals
    def _publish(self, frame: np.ndarray) -> None:
        now = time.monotonic()
        with self._lock:
            if self._frame_id > 0:
                dt = now - self._ts
                if dt > 1e-4:
                    inst = 1.0 / dt
                    self._fps = inst if self._fps == 0.0 else (
                        (1 - FPS_EMA_ALPHA) * self._fps + FPS_EMA_ALPHA * inst)
            self._frame = frame
            self._ts = now
            self._frame_id += 1

    def _run(self) -> None:
        backoff = self.reconnect_s
        while not self._stop.is_set():
            got_before = self._frame_id
            try:
                if self.source.startswith(("http://", "https://")):
                    self._run_http()
                elif self.source.isdigit():
                    self._run_capture(cv2.VideoCapture(int(self.source)), live=True)
                else:
                    path = Path(self.source)
                    if path.is_dir():
                        self._run_folder(path)
                    elif path.is_file():
                        self._run_capture(cv2.VideoCapture(str(path)), live=False)
                    else:
                        log.error("frame source %r not found", self.source)
                if self._finished:
                    return
            except Exception as exc:  # noqa: BLE001 - the thread must never die
                log.warning("frame source %s error: %s", self.source, exc)
            if self._stop.is_set():
                break
            if self._frame_id > got_before:
                backoff = self.reconnect_s          # we had frames: reset the backoff
            log.info("reconnecting to %s in %.1f s", self.source, backoff)
            self._stop.wait(backoff)
            backoff = min(MAX_BACKOFF_S, backoff * 2)

    def _run_http(self) -> None:
        log.info("connecting to MJPEG stream %s", self.source)
        resp = requests.get(self.source, stream=True,
                            timeout=(self.connect_timeout_s, self.stall_s))
        self._response = resp
        try:
            resp.raise_for_status()
            buf = bytearray()
            # A read timeout (no bytes for stall_s) raises and triggers a reconnect.
            for chunk in resp.iter_content(chunk_size=4096):
                if self._stop.is_set():
                    return
                if not chunk:
                    continue
                buf += chunk
                while True:
                    start = buf.find(SOI)
                    if start < 0:
                        buf.clear()
                        break
                    end = buf.find(EOI, start + 2)
                    if end < 0:
                        if start > 0:
                            del buf[:start]
                        break
                    jpg = bytes(buf[start:end + 2])
                    del buf[:end + 2]
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        self._publish(frame)
                    else:
                        log.debug("dropped an undecodable JPEG (%d bytes)", len(jpg))
                if len(buf) > MAX_BUFFER_BYTES:
                    log.warning("MJPEG buffer overflow, resyncing")
                    buf.clear()
            log.warning("MJPEG stream ended")
        finally:
            self._response = None
            resp.close()

    def _pace(self, next_t: float, period: float) -> float:
        """Sleep until next_t (interruptible); return the next deadline."""
        now = time.monotonic()
        if next_t > now:
            self._stop.wait(next_t - now)
        else:
            next_t = now            # we are late: don't try to catch up
        return next_t + period

    def _run_capture(self, cap: cv2.VideoCapture, live: bool) -> None:
        try:
            if not cap.isOpened():
                raise IOError(f"cannot open {self.source}")
            fps = cap.get(cv2.CAP_PROP_FPS) if not live else 0.0
            if not fps or not np.isfinite(fps) or fps <= 0 or fps > 240:
                fps = self.default_fps
            period = 1.0 / fps
            next_t = time.monotonic()
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    if live:
                        raise IOError("camera read failed")
                    if not self.loop:
                        self._finished = True
                        return
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        raise IOError(f"cannot rewind {self.source}")
                self._publish(frame)
                if not live:
                    next_t = self._pace(next_t, period)
        finally:
            cap.release()

    def _run_folder(self, folder: Path) -> None:
        files = sorted(p for p in folder.iterdir() if p.suffix.lower() in IMAGE_EXTS)
        if not files:
            raise IOError(f"no images in {folder}")
        period = 1.0 / self.default_fps
        next_t = time.monotonic()
        while not self._stop.is_set():
            for p in files:
                if self._stop.is_set():
                    return
                frame = cv2.imread(str(p), cv2.IMREAD_COLOR)
                if frame is None:
                    log.warning("cannot read image %s", p)
                else:
                    self._publish(frame)
                next_t = self._pace(next_t, period)
            if not self.loop:
                self._finished = True
                return
