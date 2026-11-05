"""UDP protocol encode/decode (CONTRACT.md section 1)."""
import json
import socket
import time

import pytest

from pinacle.comms import (DevkitLink, Telemetry, decode_telemetry, encode_drive,
                           encode_heartbeat, encode_rotate, encode_stop)


def test_encode_drive_basic():
    assert encode_drive(1, 100, -100) == b"C,1,100,-100\n"


@pytest.mark.parametrize("l,r,exp", [(300, -300, "255,-255"), (255.4, -255.6, "255,-255"),
                                     (12.6, -0.4, "13,0"), (-1000, 0, "-255,0")])
def test_encode_drive_clamps(l, r, exp):
    assert encode_drive(7, l, r).decode().strip() == f"C,7,{exp}"


def test_encode_stop_heartbeat():
    assert encode_stop(3) == b"S,3\n"
    assert encode_heartbeat(4) == b"H,4\n"


def test_encode_rotate_one_decimal():
    assert encode_rotate(5, 90) == b"R,5,90.0\n"
    assert encode_rotate(5, -45.26) == b"R,5,-45.3\n"


def test_seq_wraps_uint32():
    assert encode_stop(2 ** 32 + 5) == b"S,5\n"


GOOD = {"seq": 12, "state": "OK", "d": [10, 20.5, -1, 33, 400, -1], "yaw": 12.5, "busy": False}


def test_decode_good():
    t = decode_telemetry(json.dumps(GOOD).encode(), t_rx=1.0)
    assert isinstance(t, Telemetry)
    assert t.seq == 12 and t.state == "OK" and t.busy is False
    assert t.d == [10, 20.5, -1, 33, 400, -1]   # -1 is kept, not None
    assert t.yaw == 12.5 and t.t_rx == 1.0


def test_decode_yaw_null():
    t = decode_telemetry(json.dumps({**GOOD, "yaw": None}))
    assert t is not None and t.yaw is None
    assert t.t_rx <= time.monotonic()


@pytest.mark.parametrize("bad", [
    b"", b"not json", b"[1,2,3]", b"\xff\xfe",
    json.dumps({k: v for k, v in GOOD.items() if k != "busy"}).encode(),
    json.dumps({**GOOD, "d": [1, 2, 3]}).encode(),
    json.dumps({**GOOD, "d": [1, 2, 3, 4, 5, "x"]}).encode(),
    json.dumps({**GOOD, "state": "FINE"}).encode(),
    json.dumps({**GOOD, "busy": "yes"}).encode(),
    json.dumps({**GOOD, "yaw": "north"}).encode(),
    json.dumps({**GOOD, "seq": -1}).encode(),
])
def test_decode_malformed(bad):
    assert decode_telemetry(bad) is None


def test_devkit_link_sends_and_receives():
    """DevkitLink over real localhost UDP: heartbeat, commands, telemetry."""
    dev = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dev.bind(("127.0.0.1", 0))
    dev.settimeout(1.0)
    logged = []
    link = DevkitLink("127.0.0.1", dev.getsockname()[1], telem_port=0, heartbeat_hz=20,
                      bind_ip="127.0.0.1", log_cb=lambda k, f: logged.append((k, f))).start()
    try:
        assert link.latest() is None and link.telemetry_age() == float("inf")
        assert not link.is_live(1.0)
        link.drive(300, -20)
        link.rotate(-90)
        link.stop()
        seen = []
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not {"C", "R", "S", "H"} <= {s[0] for s in seen}:
            seen.append(dev.recv(100).decode().strip())
        kinds = {s[0] for s in seen}
        assert {"C", "R", "S", "H"} <= kinds
        assert any(s.startswith("C,") and s.endswith(",255,-20") for s in seen)
        seqs = [int(s.split(",")[1]) for s in seen]
        assert len(set(seqs)) == len(seqs)  # unique sequence numbers

        dev.sendto(json.dumps(GOOD).encode(), ("127.0.0.1", link.telem_port))
        dev.sendto(b"garbage", ("127.0.0.1", link.telem_port))
        deadline = time.monotonic() + 2
        while link.latest() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert link.latest() is not None and link.latest().seq == 12
        assert link.is_live(1.0)
        assert any(k == "telem" for k, _ in logged) and any(k == "cmd" for k, _ in logged)
    finally:
        link.close()
        dev.close()
