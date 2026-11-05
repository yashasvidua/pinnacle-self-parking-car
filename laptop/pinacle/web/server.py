"""Flask dashboard: live video, manual control, FIND & PARK, e-stop, calibration.

Use the factory so the tests can use ``app.test_client()``::

    app = create_app(ctx)          # ctx = pinacle.web.context.AppContext
    app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False)

All mode rules live in :mod:`pinacle.modes` (via the AppContext); the JavaScript
only displays what ``/api/status`` reports and forwards button/key presses.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from flask import Flask, Response, jsonify, request, send_from_directory

from ..modes import MANUAL_COMMANDS

log = logging.getLogger(__name__)
STATIC_DIR = Path(__file__).resolve().parent / "static"
BOUNDARY = "pinacleframe"


def _placeholder_jpeg(text: str = "NO VIDEO") -> bytes:
    """A small grey JPEG shown until the first camera frame arrives."""
    import cv2
    import numpy as np
    img = np.full((240, 320, 3), 40, np.uint8)
    cv2.putText(img, text, (80, 128), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (180, 180, 180), 2,
                cv2.LINE_AA)
    ok, buf = cv2.imencode(".jpg", img)
    return buf.tobytes() if ok else b""


def create_app(ctx: Any) -> Flask:
    """Build the Flask app around a shared AppContext."""
    app = Flask(__name__, static_folder=str(STATIC_DIR), static_url_path="/static")
    app.config["JSON_SORT_KEYS"] = False
    app.config["PINACLE_CTX"] = ctx
    placeholder = {"jpeg": None}

    @app.after_request
    def _no_cache(resp: Response) -> Response:
        if request.path.startswith("/api/"):
            resp.headers["Cache-Control"] = "no-store"
        return resp

    # ------------------------------------------------------------------ pages
    @app.get("/")
    def index() -> Response:
        return send_from_directory(STATIC_DIR, "index.html")

    @app.get("/calibrate")
    def calibrate() -> Response:
        return send_from_directory(STATIC_DIR, "calibrate.html")

    @app.get("/video")
    def video() -> Response:
        """MJPEG stream of the annotated frames (encoded once by the vision thread)."""
        if placeholder["jpeg"] is None:
            placeholder["jpeg"] = _placeholder_jpeg()

        def gen():
            last_id = -1
            while True:
                fid, jpeg = ctx.wait_jpeg(last_id, timeout=1.0)
                if jpeg is None:
                    jpeg = placeholder["jpeg"]
                elif fid == last_id:
                    continue  # no new frame yet; wait again
                last_id = fid
                yield (b"--" + BOUNDARY.encode() + b"\r\nContent-Type: image/jpeg\r\n"
                       b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")

        return Response(gen(), mimetype=f"multipart/x-mixed-replace; boundary={BOUNDARY}",
                        headers={"Cache-Control": "no-store"})

    # ------------------------------------------------------------------ API
    @app.get("/api/config")
    def api_config() -> Response:
        cfg = ctx.cfg
        return jsonify({
            "repeat_ms": int(cfg.get("manual", {}).get("repeat_ms", 100)),
            "default_speed": int(cfg.get("manual", {}).get("default_speed", 160)),
            "max_pwm": int(cfg.get("car", {}).get("max_pwm", 255)),
            "min_pwm": int(cfg.get("car", {}).get("min_pwm", 90)),
            "ui_heartbeat_s": float(cfg.get("safety", {}).get("ui_heartbeat_s", 1.0)),
            "bay_side": cfg.get("bay", {}).get("side", "right"),
            "sim": bool(getattr(ctx, "sim", False)),
        })

    @app.get("/api/status")
    def api_status() -> Response:
        return jsonify(ctx.status())

    @app.post("/api/manual")
    def api_manual() -> Any:
        data = request.get_json(silent=True) or {}
        cmd = data.get("cmd")
        speed = data.get("speed")
        try:
            speed = None if speed is None else float(speed)
        except (TypeError, ValueError):
            return jsonify({"ok": False, "error": "bad speed"}), 400
        if cmd is not None:
            if cmd not in MANUAL_COMMANDS:
                return jsonify({"ok": False, "error": f"unknown cmd {cmd!r}"}), 400
            return jsonify(ctx.manual(cmd=cmd, speed=speed))
        if "left" in data and "right" in data:
            try:
                left, right = float(data["left"]), float(data["right"])
            except (TypeError, ValueError):
                return jsonify({"ok": False, "error": "bad left/right"}), 400
            return jsonify(ctx.manual(left=left, right=right))
        return jsonify({"ok": False, "error": "need cmd or left/right"}), 400

    @app.post("/api/stop")
    def api_stop() -> Any:
        """Release of a drive button/key (counts as manual input)."""
        return jsonify(ctx.manual(cmd="stop"))

    @app.post("/api/find_park")
    def api_find_park() -> Any:
        ok, reason = ctx.find_park()
        if not ok:
            return jsonify({"ok": False, "reason": reason}), 409
        return jsonify({"ok": True, "reason": reason})

    @app.post("/api/estop")
    def api_estop() -> Any:
        return jsonify(ctx.estop())

    @app.post("/api/heartbeat")
    def api_heartbeat() -> Any:
        ctx.ui_heartbeat()
        return jsonify({"ok": True})

    # ------------------------------------------------------------------ calibration
    @app.get("/api/calib/frame")
    def api_calib_frame() -> Any:
        jpeg = ctx.calib_freeze()
        if jpeg is None:
            return jsonify({"ok": False, "error": "no camera frame yet"}), 503
        return Response(jpeg, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.get("/api/calib/info")
    def api_calib_info() -> Any:
        return jsonify(ctx.calib_info())

    @app.post("/api/calib/save")
    def api_calib_save() -> Any:
        data = request.get_json(silent=True) or {}
        try:
            res = ctx.calib_save(data.get("image_points") or [], data.get("world_points_cm") or [])
        except ValueError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400
        except Exception as exc:
            log.exception("calibration save failed")
            return jsonify({"ok": False, "error": str(exc)}), 500
        return jsonify(res)

    return app
