# Pinacle interface contract

This file is the single source of truth for the protocol between the laptop, the
DevKit and the simulator, and for the Python interfaces between laptop modules.
Every module must follow it exactly.

## 1. UDP protocol

### Laptop -> DevKit: UDP port 4210, one ASCII line per datagram (trailing `\n` optional)

| Packet | Meaning |
|---|---|
| `C,<seq>,<left>,<right>` | Set motor speeds. `left`, `right` are integers in -255..255 (PWM duty; sign = direction). Values outside the range are clamped. |
| `S,<seq>` | Stop motors now. Cancels any running rotation. Does **not** latch any state. |
| `H,<seq>` | Heartbeat. Feeds the watchdog only; never changes motion. |
| `R,<seq>,<deg>` | Rotate in place by `deg` degrees (float, relative). **Positive = counter-clockwise (left) seen from above.** With gyro: closed loop on yaw, tolerance 3 deg, timeout 4 s. Without gyro: open-loop spin for `abs(deg) * ROTATE_MS_PER_DEG` ms. |

- `seq` is an unsigned 32-bit counter, incremented by the sender for every packet. The DevKit records the last seq but **does not reject** out-of-order or lower seqs (so a restarted laptop works immediately).
- Malformed lines are ignored and do **not** feed the watchdog.
- A rotation is cancelled by: `S`, any `C`, a new `R` (which replaces it), ESTOP, or the watchdog. `H` does not cancel it.
- `busy` is true while a rotation is running.

### DevKit -> laptop: UDP port 4211, JSON, 10 Hz

```
{"seq":n,"state":"OK|ESTOP|WATCHDOG","d":[fl,fr,bl,br,l,r],"yaw":deg,"busy":bool}
```

- Exactly these five keys, no others.
- `seq`: the DevKit's own telemetry counter (increments per telemetry packet).
- `d`: six distances in cm, order **front-left, front-right, back-left, back-right, left, right**. Number with at most 1 decimal; `-1` = no echo / out of range. Each value is the median of the last 3 readings for that sensor.
- `yaw`: integrated gyro Z in degrees, CCW positive, unwrapped, zeroed at boot. **`null` if the MPU6050 is disabled or absent.**
- `busy`: `true` while an `R` rotation is running.
- The destination is the source IP of the last valid laptop packet, port 4211. Before any packet has been received, telemetry is not sent (or goes to `LAPTOP_IP` from secrets.h if one is defined).

### DevKit safety

- **Watchdog:** if no valid packet (C/S/H/R) arrives for 300 ms -> motors off, rotation cancelled, `state = "WATCHDOG"`. It clears to OK on the next valid packet (motors stay stopped until a new `C` or `R`).
- **Hard stop (ESTOP):** with `v = left + right`:
  - `v > 0` (forward): if `fl` or `fr` reads in `0 <= d < 8` cm -> stop.
  - `v < 0` (reverse): if `bl` or `br` reads in `0 <= d < 8` cm -> stop.
  - `v == 0` and `left != 0` (pure rotation, including `R`): if any of `fl,fr,bl,br` reads `< 8` cm -> stop.
  - `-1` (no echo) counts as clear.
  - While a hard stop is active, `state = "ESTOP"`. It stays latched until the commanded motion no longer points toward the obstacle (a zero command, or a command in the other direction is accepted immediately, so the car can always back away) or the reading rises to >= 10 cm (2 cm hysteresis).
  - If both apply, ESTOP takes priority over WATCHDOG in the reported state.
- The HC-SR04s are fired one at a time, round-robin, about 30 ms apart (echo timeout ~25 ms ≈ 400 cm).
- Median filter over the last 3 readings per sensor.

### Laptop-side timing

- The heartbeat thread in `comms.py` sends `H` at 10 Hz in **every** mode, including when the car is stopped.
- The dashboard repeats manual drive commands every 100 ms while a button or key is held and sends stop on release.
- Manual dead-man: if MANUAL mode receives no drive input for 250 ms while moving, the laptop sends `S`.
- The DevKit watchdog (300 ms) is the last line of defence if the laptop dies.

## 2. Coordinate frame (laptop)

- **Car frame**, all in **cm** and **degrees**: origin = the floor point directly under the centre of the front bumper; **+x forward, +y left**; angles CCW-positive from +x.
- Calibration floor points are entered in this frame (so the camera must stay fixed relative to the car).
- **Bay pose** = the bay **centre** in the car frame: `x_cm, y_cm`; `angle_deg` = the angle of the bay's **long axis** relative to the car's +x axis, normalised to (-90, 90]. A parallel bay alongside the car has angle ≈ 0. `confidence` is 0..1.

## 3. Python interfaces (laptop/pinacle)

Perception (owned by the perception agent; control code only imports these):

```python
# stream.py
class FrameSource:
    def __init__(self, source: str, reconnect_s: float = 1.0, loop: bool = True): ...
        # source = "http://.../stream" (MJPEG over HTTP) or a path to a video file / folder of JPGs.
        # Files play back at their native fps and loop if loop=True.
    def start(self) -> "FrameSource": ...
    def stop(self) -> None: ...
    def read(self) -> tuple["np.ndarray | None", float, int]: ...
        # (latest BGR frame or None, time.monotonic() timestamp, frame_id). Non-blocking, latest-only.
    def is_live(self, max_age_s: float = 1.0) -> bool: ...
    @property
    def fps(self) -> float: ...

# calib.py
class Homography:
    H: np.ndarray            # 3x3, image px -> floor cm (car frame)
    def __init__(self, H): ...
    @classmethod
    def from_points(cls, image_pts, world_pts_cm) -> "Homography": ...   # exactly 4+ pairs
    def pixel_to_floor(self, pts) -> np.ndarray: ...   # (N,2) px -> (N,2) cm; accepts a single (u,v) too
    def floor_to_pixel(self, pts) -> np.ndarray: ...
    def birdseye(self, frame, cfg_birdseye: dict) -> np.ndarray: ...
def load_homography(cfg: dict) -> "Homography | None": ...   # None if not calibrated
def save_calibration(image_pts, world_pts_cm, path=None) -> Homography: ...  # writes the calibration: section of config.yaml

# detect.py
@dataclass
class BayPose:
    x_cm: float; y_cm: float; angle_deg: float; confidence: float
@dataclass
class DetectionResult:
    pose: "BayPose | None"
    method: str                       # "tape" | "aruco"
    lines_px: list                    # [(x1,y1,x2,y2), ...] for the overlay
    corners_px: "np.ndarray | None"   # (4,2) bay corners in image px
    corners_cm: "np.ndarray | None"   # (4,2) bay corners in the car frame
    length_cm: "float | None"
    width_cm: "float | None"
def make_detector(cfg: dict, homography: "Homography | None"): ...  # object with .detect(frame) -> DetectionResult
                                                                  # never raises; pose=None if nothing found
def draw_overlay(frame, result: "DetectionResult | None", fsm_state: str = "", extra_text: str = "") -> np.ndarray: ...  # returns a copy
```

Control (owned by the control agent): `comms.py`, `fusion.py`, `planner.py`, `fsm.py`, `modes.py`, `web/`, `main.py`, `sim/fake_car.py`.

Shared (owned by the orchestrator): `config.py` (`load_config()`, `update_config_section()`, `REPO_ROOT`), `config.yaml`, `requirements.txt`, `.env.example`.
