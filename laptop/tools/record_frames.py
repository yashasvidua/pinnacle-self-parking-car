"""Record frames from the ESP32-CAM (or any FrameSource) to timestamped JPGs.

Run from the repo root::

    python laptop/tools/record_frames.py                       # URL from config.yaml
    python laptop/tools/record_frames.py --url http://192.168.43.100/stream --every 2 --video

Stops with Ctrl+C. The folder can later be replayed with ``FrameSource(folder)``.
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # make `import pinacle` work

import cv2  # noqa: E402

from pinacle.config import REPO_ROOT, load_config  # noqa: E402
from pinacle.stream import FrameSource  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    cfg = load_config()
    default_out = REPO_ROOT / cfg.get("logging", {}).get("record_dir", "recordings") \
        / datetime.now().strftime("%Y%m%d_%H%M%S")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=cfg.get("network", {}).get("cam_url"),
                    help="stream URL, video file or image folder (default: network.cam_url)")
    ap.add_argument("--out", type=Path, default=default_out, help="output folder")
    ap.add_argument("--every", type=int, default=1, help="save every N-th new frame")
    ap.add_argument("--video", action="store_true", help="also write frames to out/video.avi")
    ap.add_argument("--fps", type=float, default=10.0, help="fps written into the .avi")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    args.out.mkdir(parents=True, exist_ok=True)
    src = FrameSource(str(args.url), reconnect_s=float(cfg.get("camera", {}).get("reconnect_s", 1.0))).start()
    writer = None
    last_id, seen, saved = 0, 0, 0
    print(f"Recording {args.url} -> {args.out}  (Ctrl+C to stop)")
    try:
        while True:
            frame, _, fid = src.read()
            if frame is None or fid == last_id:
                time.sleep(0.01)
                continue
            last_id = fid
            seen += 1
            if (seen - 1) % max(1, args.every):
                continue
            name = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3] + f"_{fid:06d}.jpg"
            cv2.imwrite(str(args.out / name), frame)
            if args.video:
                if writer is None:
                    h, w = frame.shape[:2]
                    writer = cv2.VideoWriter(str(args.out / "video.avi"),
                                             cv2.VideoWriter_fourcc(*"MJPG"), args.fps, (w, h))
                writer.write(frame)
            saved += 1
            if saved % 20 == 0:
                print(f"  {saved} frames saved, stream {src.fps:.1f} fps")
    except KeyboardInterrupt:
        pass
    finally:
        src.stop()
        if writer is not None:
            writer.release()
    print(f"Saved {saved} frames to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
