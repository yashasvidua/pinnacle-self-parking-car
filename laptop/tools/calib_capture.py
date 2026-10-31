"""Standalone floor calibration tool (camera px -> car-frame cm homography).

Run from the repo root::

    python laptop/tools/calib_capture.py                  # live stream from config.yaml
    python laptop/tools/calib_capture.py --image snap.jpg

1. Put 4 marks on the floor in front of the car (spread wide, not in a line) and
   measure each one in the car frame: x forward from the front bumper, y to the left
   of the car's centre line, in cm.
2. Press SPACE to freeze the image (not needed with --image).
3. Click the 4 marks. Keys: r = reset clicks, SPACE = unfreeze, q/ESC = quit.
4. Type each mark's "x y" in the terminal. The calibration is saved to config.yaml
   and a bird's-eye preview opens (any key to close).

The dashboard has a web calibration page too; this tool does the same offline.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # make `import pinacle` work

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from pinacle.calib import Homography, save_calibration  # noqa: E402
from pinacle.config import load_config  # noqa: E402
from pinacle.stream import FrameSource  # noqa: E402

WIN = "Pinacle calibration"
SCALE = 2          # QVGA is tiny: show the image enlarged


def ask_point(i: int, px) -> tuple[float, float]:
    while True:
        raw = input(f"  mark {i + 1} at pixel ({px[0]:.0f}, {px[1]:.0f}) -> x y in cm: ")
        try:
            x, y = (float(v) for v in raw.replace(",", " ").split())
            return x, y
        except ValueError:
            print("    please type two numbers, e.g. '40 -15'")


def draw(frame, clicks, frozen):
    view = cv2.resize(frame, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST)
    for i, (u, v) in enumerate(clicks):
        p = (int(u * SCALE), int(v * SCALE))
        cv2.circle(view, p, 5, (0, 0, 255), 2)
        cv2.putText(view, str(i + 1), (p[0] + 6, p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    msg = (f"FROZEN - click mark {len(clicks) + 1} of 4 (r=reset, SPACE=live)" if frozen
           else "LIVE - press SPACE to freeze")
    cv2.putText(view, msg, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    return view


def main(argv: list[str] | None = None) -> int:
    cfg = load_config()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=cfg.get("network", {}).get("cam_url"), help="stream URL / video / folder")
    ap.add_argument("--image", type=Path, help="calibrate on a still image instead of the stream")
    ap.add_argument("--points", type=int, default=4, help="number of marks to click (>= 4)")
    ap.add_argument("--config", type=Path, default=None, help="config.yaml to write (default: repo)")
    args = ap.parse_args(argv)
    n_points = max(4, args.points)

    src = None
    if args.image:
        still = cv2.imread(str(args.image))
        if still is None:
            print(f"cannot read {args.image}")
            return 1
    else:
        src = FrameSource(str(args.url)).start()
        still = None

    clicks: list[tuple[float, float]] = []
    frozen = still is not None
    frame = still

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and frozen and len(clicks) < n_points:
            clicks.append((x / SCALE, y / SCALE))

    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, on_mouse)
    try:
        while len(clicks) < n_points:
            if not frozen and src is not None:
                live, _, _ = src.read()
                if live is not None:
                    frame = live
            if frame is None:
                blank = np.zeros((240, 320, 3), np.uint8)
                cv2.putText(blank, "waiting for stream...", (40, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 1)
                cv2.imshow(WIN, blank)
            else:
                cv2.imshow(WIN, draw(frame, clicks, frozen))
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                return 1
            if key == ord(" ") and frame is not None and src is not None:
                frozen = not frozen
                clicks.clear()
            if key == ord("r"):
                clicks.clear()
        cv2.imshow(WIN, draw(frame, clicks, frozen))
        cv2.waitKey(1)
    finally:
        if src is not None:
            src.stop()

    print("Enter the car-frame position of each mark (x forward, y left, cm):")
    world = [ask_point(i, c) for i, c in enumerate(clicks)]
    try:
        hom: Homography = save_calibration(clicks, world, path=args.config)
    except ValueError as exc:
        print(f"Calibration failed: {exc}")
        return 1
    err = hom.reprojection_error_cm(clicks, world)
    print("Saved calibration. Homography:\n", np.array2string(hom.H, precision=5))
    print(f"Reprojection error: {err:.2f} cm"
          + ("  (exact with 4 points - click more marks for a real check)" if n_points == 4 else ""))

    top = hom.birdseye(frame, cfg.get("calibration", {}).get("birdseye"))
    cv2.imshow("Bird's-eye preview (forward = up)", top)
    print("Press any key in the preview window to close.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
