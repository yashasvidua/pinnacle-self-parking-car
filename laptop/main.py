"""Pinacle laptop app: vision + control + dashboard.

Run from the repo root::

    python laptop/main.py                 # real car (config.yaml / .env addresses)
    python laptop/main.py --sim           # fake DevKit on localhost + synthetic video
    python laptop/main.py --sim --record  # also save the annotated video + CSV
    python laptop/main.py --port 8080 --config my_config.yaml

Then open http://localhost:5000 (or http://<laptop-ip>:5000 from a phone).

Threads:
* FrameSource (perception)    - reads the camera
* vision thread               - detect -> overlay -> JPEG (encoded once, shared by all viewers)
* control thread (~20 Hz)     - FSM update in AUTO, dead-man check in MANUAL
* DevkitLink RX + heartbeat   - telemetry in, H at 10 Hz out
* Flask (main thread)         - dashboard

Ctrl+C sends S, then stops everything.
"""
from __future__ import annotations

import argparse
import logging
import shutil
import sys
import threading
import time
from pathlib import Path

# sys.path shim so `python laptop/main.py` finds the `pinacle` and `sim` packages.
LAPTOP_DIR = Path(__file__).resolve().parent
if str(LAPTOP_DIR) not in sys.path:
    sys.path.insert(0, str(LAPTOP_DIR))

from pinacle.comms import DevkitLink  # noqa: E402
from pinacle.config import CONFIG_PATH, REPO_ROOT, load_config  # noqa: E402
from pinacle.web.context import AppContext, Perception  # noqa: E402
from pinacle.web.runlog import CsvLogger, VideoRecorder  # noqa: E402
from pinacle.web.server import create_app  # noqa: E402

log = logging.getLogger("pinacle.main")
SIM_CONFIG_PATH = LAPTOP_DIR / "sim" / "sim_config.yaml"


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Pinacle self-parking car: laptop app")
    ap.add_argument("--sim", action="store_true",
                    help="run against sim/fake_car.py on localhost with a synthetic camera feed")
    ap.add_argument("--record", action="store_true",
                    help="record the annotated video (logging.record_dir, MJPG .avi)")
    ap.add_argument("--port", type=int, default=None, help="dashboard port (default web.port)")
    ap.add_argument("--config", default=None, help="config file (default config.yaml)")
    ap.add_argument("--no-gyro", action="store_true", help="sim only: fake car without a gyro")
    ap.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    return ap.parse_args(argv)


def setup_sim(base_config: Path):
    """Prepare the simulator: video, a sim copy of the config with a sim calibration.

    The real config.yaml is never modified in sim: a copy is written to
    laptop/sim/sim_config.yaml (regenerated on every --sim start), and the sim
    calibration (and anything saved from the calibration page) goes there.
    Returns (cfg, config_path, video_path).
    """
    from pinacle.calib import save_calibration
    from sim import synth

    cfg0 = load_config(base_config)
    video = REPO_ROOT / cfg0.get("camera", {}).get("sim_video", "laptop/sim/sample.mp4")
    video = synth.ensure_sim_video(video)

    shutil.copyfile(base_config, SIM_CONFIG_PATH)
    save_calibration(synth.SIM_CALIB_IMAGE_PTS, synth.SIM_CALIB_WORLD_PTS, path=SIM_CONFIG_PATH)
    cfg = load_config(SIM_CONFIG_PATH)
    cfg.setdefault("network", {})["devkit_ip"] = "127.0.0.1"
    cfg["network"]["cam_url"] = str(video)
    return cfg, SIM_CONFIG_PATH, video


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                        datefmt="%H:%M:%S")
    logging.getLogger("werkzeug").setLevel(logging.WARNING)  # no line per status poll

    base_config = Path(args.config).resolve() if args.config else CONFIG_PATH
    fake_car = None
    if args.sim:
        cfg, config_path, video = setup_sim(base_config)
        source_url = str(video)
    else:
        cfg, config_path = load_config(base_config), base_config
        source_url = cfg["network"]["cam_url"]
    net = cfg.get("network", {})
    cmd_port = int(net.get("cmd_port", 4210))
    telem_port = int(net.get("telem_port", 4211))

    if args.sim:
        from sim.fake_car import FakeCar
        car_cfg = cfg.get("car", {})
        fake_car = FakeCar(cmd_port=cmd_port, telem_port=telem_port, bind_ip="127.0.0.1",
                           gyro=not args.no_gyro,
                           pwm_per_cm_s=float(cfg.get("planner", {}).get("pwm_per_cm_s", 4.0)),
                           min_pwm=int(car_cfg.get("min_pwm", 90))).start()

    # logging to CSV (always) and optional video
    logs_dir = REPO_ROOT / cfg.get("logging", {}).get("dir", "logs")
    csv_logger = CsvLogger(logs_dir, prefix="sim" if args.sim else "run")
    from pinacle.stream import FrameSource
    source = FrameSource(source_url, reconnect_s=float(cfg.get("camera", {}).get("reconnect_s", 1.0)),
                         loop=True).start()
    recorder = None
    if args.record:
        recorder = VideoRecorder(REPO_ROOT / cfg.get("logging", {}).get("record_dir", "recordings"),
                                 fps=15.0)

    link = DevkitLink(net.get("devkit_ip", "127.0.0.1"), cmd_port, telem_port,
                      heartbeat_hz=float(net.get("heartbeat_hz", 10)),
                      log_cb=csv_logger.log).start()

    perception = Perception()
    homography = perception.load_homography(cfg) if perception.load_homography else None
    if homography is None:
        log.warning("no calibration loaded: FIND & PARK stays disabled until you calibrate")
    ctx = AppContext(cfg, link, source, homography, perception=perception,
                     config_path=config_path, sim=args.sim, csv_logger=csv_logger,
                     recorder=recorder)

    stop_evt = threading.Event()

    def vision_loop() -> None:
        while not stop_evt.is_set():
            try:
                if not ctx.vision_tick():
                    stop_evt.wait(0.005)
            except Exception:
                log.exception("vision loop error")
                stop_evt.wait(0.2)

    def control_loop() -> None:
        period = 0.05  # 20 Hz
        next_t = time.monotonic()
        while not stop_evt.is_set():
            try:
                ctx.control_tick()
            except Exception:
                log.exception("control loop error; stopping the car")
                try:
                    ctx.estop("control loop error")
                except Exception:
                    pass
            next_t += period
            delay = next_t - time.monotonic()
            if delay < -0.5:
                next_t = time.monotonic()
            stop_evt.wait(max(0.0, delay))

    threads = [threading.Thread(target=vision_loop, name="vision", daemon=True),
               threading.Thread(target=control_loop, name="control", daemon=True)]
    for th in threads:
        th.start()

    app = create_app(ctx)
    host = cfg.get("web", {}).get("host", "0.0.0.0")
    port = args.port or int(cfg.get("web", {}).get("port", 5000))
    log.info("dashboard: http://localhost:%d  (%s)", port, "SIM" if args.sim else "car")
    try:
        app.run(host=host, port=port, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        log.info("shutting down: sending S")
        ctx.shutdown()              # S first
        stop_evt.set()
        for th in threads:
            th.join(timeout=1.0)
        try:
            link.stop()
        except Exception:
            pass
        link.close()
        source.stop()
        if fake_car is not None:
            fake_car.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
