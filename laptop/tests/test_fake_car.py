"""FakeCar over real localhost UDP on ephemeral ports."""
import json
import socket
import time

import pytest

from sim.fake_car import FakeCar, rect_segments


class Laptop:
    """Minimal laptop side: a socket bound to an ephemeral telemetry port."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.5)
        self.port = self.sock.getsockname()[1]
        self.seq = 0
        self.car = None

    def send(self, line):
        self.seq += 1
        self.sock.sendto(line.format(seq=self.seq).encode(), ("127.0.0.1", self.car.cmd_port))

    def telem(self):
        """Newest telemetry packet (drains the queue)."""
        last = None
        self.sock.settimeout(0.5)
        try:
            last = self.sock.recv(1024)
            self.sock.settimeout(0.0)
            while True:
                last = self.sock.recv(1024)
        except (BlockingIOError, socket.timeout, OSError):
            pass
        return json.loads(last) if last else None


def keep_alive(lap, line, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        lap.send(line)
        time.sleep(0.05)


@pytest.fixture
def make_car():
    cars, laps = [], []

    def _make(**kw):
        lap = Laptop()
        car = FakeCar(cmd_port=0, telem_port=lap.port, bind_ip="127.0.0.1", verbose=0, **kw).start()
        lap.car = car
        cars.append(car)
        laps.append(lap)
        return car, lap

    yield _make
    for c in cars:
        c.stop()
    for lap in laps:
        lap.sock.close()


def test_telemetry_has_exactly_five_keys(make_car):
    car, lap = make_car()
    lap.send("H,{seq}")
    t = lap.telem()
    assert t is not None
    assert set(t) == {"seq", "state", "d", "yaw", "busy"}
    assert len(t["d"]) == 6 and t["state"] in ("OK", "WATCHDOG", "ESTOP")
    assert all(v == -1 or (v >= 0 and round(v, 1) == v) for v in t["d"])
    assert isinstance(t["yaw"], float) and t["busy"] is False


def test_no_gyro_sends_null_yaw(make_car):
    car, lap = make_car(gyro=False)
    lap.send("H,{seq}")
    assert lap.telem()["yaw"] is None


def test_drive_moves_car(make_car):
    car, lap = make_car(world=[])
    x0 = car.pose()[0]
    keep_alive(lap, "C,{seq},160,160", 0.5)
    assert car.pose()[0] - x0 > 10
    assert lap.telem()["state"] == "OK"


def test_watchdog_after_300ms(make_car):
    car, lap = make_car(world=[])
    lap.send("C,{seq},160,160")
    time.sleep(0.15)
    assert car.report_state() == "OK"
    time.sleep(0.35)
    assert car.report_state() == "WATCHDOG"
    x = car.pose()[0]
    time.sleep(0.2)
    assert car.pose()[0] == pytest.approx(x)  # motors off
    lap.send("H,{seq}")
    time.sleep(0.05)
    assert car.report_state() == "OK"  # clears, but stays stopped
    time.sleep(0.1)
    assert car.pose()[0] == pytest.approx(x)


def test_malformed_does_not_feed_watchdog(make_car):
    car, lap = make_car(world=[])
    lap.send("H,{seq}")
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        lap.sock.sendto(b"C,abc", ("127.0.0.1", car.cmd_port))
        time.sleep(0.05)
    assert car.report_state() == "WATCHDOG"


def test_hard_stop_and_backing_away(make_car):
    wall = rect_segments(25.0, -50.0, 30.0, 50.0)  # wall 10 cm in front of the bumper (x=15)
    car, lap = make_car(world=wall)
    keep_alive(lap, "C,{seq},160,160", 1.0)
    assert car.report_state() == "ESTOP"
    assert lap.telem()["state"] == "ESTOP"
    x_stop = car.pose()[0]
    assert 15 + x_stop < 25  # front bumper did not pass the wall
    keep_alive(lap, "C,{seq},160,160", 0.3)
    assert car.pose()[0] == pytest.approx(x_stop, abs=0.5)  # still latched
    keep_alive(lap, "C,{seq},-160,-160", 0.4)  # away from the obstacle: accepted at once
    assert car.pose()[0] < x_stop - 3
    assert car.report_state() == "OK"


def test_rotation_busy_and_yaw(make_car):
    car, lap = make_car(world=[])
    lap.send("R,{seq},90")
    busy_seen = False
    end = time.monotonic() + 3
    while time.monotonic() < end:
        lap.send("H,{seq}")  # H does not cancel the rotation
        t = lap.telem()
        busy_seen |= bool(t and t["busy"])
        if busy_seen and t and not t["busy"]:
            break
    assert busy_seen
    assert t["yaw"] == pytest.approx(90, abs=3.5)


def test_stop_cancels_rotation(make_car):
    car, lap = make_car(world=[])
    lap.send("R,{seq},180")
    time.sleep(0.1)
    assert car.busy
    lap.send("S,{seq}")
    time.sleep(0.05)
    assert not car.busy


def _states_while(lap, line, seconds):
    """Send `line` every 50 ms and collect the telemetry states seen."""
    states = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        lap.send(line)
        t = lap.telem()
        if t:
            states.append(t["state"])
    return states


def test_estop_on_rotation_stays_latched_like_firmware(make_car):
    wall = rect_segments(20.0, -50.0, 25.0, 50.0)  # ~5 cm in front of the bumper
    car, lap = make_car(world=wall)
    keep_alive(lap, "H,{seq}", 0.3)
    lap.send("R,{seq},90.0")
    states = _states_while(lap, "H,{seq}", 0.6)   # H must not release the latch
    assert states and all(s == "ESTOP" for s in states[1:])
    assert car.busy is False                      # ESTOP cancelled the rotation
    lap.send("S,{seq}")                           # a new command releases it
    keep_alive(lap, "H,{seq}", 0.2)
    assert car.report_state() == "OK"


def test_blocked_spin_command_is_held(make_car):
    wall = rect_segments(20.0, -50.0, 25.0, 50.0)
    car, lap = make_car(world=wall)
    states = _states_while(lap, "C,{seq},-150,150", 0.8)
    assert states[-3:] == ["ESTOP"] * 3


def test_boots_in_watchdog_and_median_seeded():
    car = FakeCar(cmd_port=0, telem_port=0, bind_ip="127.0.0.1", verbose=0, world=[])
    assert car.report_state() == "WATCHDOG"
    assert car._filtered([10.0, 10.0, 10.0, 10.0, 10.0, 10.0]) == [-1.0] * 6


@pytest.mark.parametrize("line", ["C, 1, 2, 3", "C,+1,2,3", "C,1_0,2,3", "R,1,0x10", "C,1,2,3\nS,2", "R,1,nan"])
def test_strict_parser_rejects(line):
    car = FakeCar(cmd_port=0, telem_port=0, bind_ip="127.0.0.1", verbose=0, world=[])
    assert car.handle_line(line) is False


@pytest.mark.parametrize("line", ["C,1,-255,255\n", "S,7", "H,0", "R,3,-90.5", "R,3,45"])
def test_strict_parser_accepts(line):
    car = FakeCar(cmd_port=0, telem_port=0, bind_ip="127.0.0.1", verbose=0, world=[])
    assert car.handle_line(line) is True
