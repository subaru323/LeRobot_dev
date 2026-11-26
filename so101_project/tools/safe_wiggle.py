"""Slow motion test of the follower, starting from the pose the user holds.

usage: python tools\\safe_wiggle.py [PORT] [AMPLITUDE_DEG]      (defaults: COM11, 10)

Moves one joint at a time by +-AMPLITUDE (gripper ~10 %) and back, with speed / acceleration /
torque limits in the motors. Stops and returns to the start pose on high load or Ctrl+C.
"""

import sys
import time

from so101_common import (
    FOLLOWER_PORT,
    check_motors_ok,
    connect_bus_no_torque,
    limit_follower,
    make_follower,
    max_load,
    release_all,
    torque_on_in_place,
)

PORT = sys.argv[1] if len(sys.argv) > 1 else FOLLOWER_PORT
AMP = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
LOAD_ABORT = 600
ORDER = ["wrist_roll", "gripper", "wrist_flex", "elbow_flex", "shoulder_pan", "shoulder_lift"]


class Abort(Exception):
    pass


def go(bus, m: str, target: int, timeout: float = 15.0) -> None:
    bus.write("Goal_Position", m, target, normalize=False)
    t0 = time.time()
    pos = None
    while time.time() - t0 < timeout:
        time.sleep(0.1)
        pos = bus.read("Present_Position", m, normalize=False, num_retry=2)
        worst, load = max_load(bus)
        if load > LOAD_ABORT:
            raise Abort(f"high load on {worst}: {load}")
        if abs(pos - target) < 15:
            return
    print(f"   {m}: did not reach {target} (at {pos}), continuing", flush=True)


robot = make_follower(PORT)
bus = robot.bus
connect_bus_no_torque(bus)
try:
    check_motors_ok(bus, "follower")
    limit_follower(bus, speed=150, accel=10, torque_limit=700)
    input("Hold the arm in the START pose and press Enter (torque turns on, the arm will hold)...")
    start = torque_on_in_place(bus)
    print("holding start pose:", start, flush=True)
    time.sleep(1.0)
    try:
        for m in ORDER:
            c = bus.calibration[m]
            lo, hi = (c.range_min, c.range_max) if m == "wrist_roll" else (c.range_min + 60, c.range_max - 60)
            d = int(AMP * 4096 / 360) if m != "gripper" else int((hi - lo) * 0.10)
            for tgt in (start[m] + d, start[m] - d, start[m]):
                tgt = min(max(tgt, lo), hi)
                print(f"{m}: -> {tgt}  (start {start[m]})", flush=True)
                go(bus, m, tgt)
                time.sleep(0.3)
        print("done. back at start pose.", flush=True)
    except (Abort, KeyboardInterrupt) as e:
        print(f"!! stopped: {e!r} -> returning to start pose", flush=True)
        for m in reversed(ORDER):
            try:
                bus.write("Goal_Position", m, start[m], normalize=False)
            except Exception:  # noqa: BLE001
                pass
        time.sleep(3)
    input("SUPPORT THE ARM with your hand, then press Enter to release torque...")
finally:
    release_all(bus)
    bus.disconnect(disable_torque=False)
    print("released.", flush=True)
