"""Teleoperation with a safe start: the follower never jumps.

usage: python tools\\safe_teleop.py [--fps 30] [--max-step 4] [--ramp-step 1]

1. Connects both arms with torque OFF and checks motors / calibration.
2. Turns follower torque on IN PLACE (Goal_Position = Present_Position first).
3. Moves the follower SLOWLY (ramp-step deg per cycle) to the leader pose; keep the leader still.
4. Then follows the leader, with each cycle's move capped to max-step deg (4 deg x 30 fps = 120 deg/s).
   Targets are clamped to the calibrated range (5 deg margin).
5. Stops on Ctrl+C or when a motor stays at high load for 1 s: the follower holds its pose and
   waits for Enter (support the arm!) before releasing torque.
"""

import argparse
import time

from so101_common import (
    MOTORS,
    check_motors_ok,
    connect_bus_no_torque,
    limit_follower,
    make_follower,
    make_leader,
    release_all,
    safe_limits_deg,
    torque_on_in_place,
)

p = argparse.ArgumentParser()
p.add_argument("--fps", type=float, default=30.0)
p.add_argument("--max-step", type=float, default=4.0, help="deg (gripper %%) per cycle while following")
p.add_argument("--ramp-step", type=float, default=1.0, help="deg (gripper %%) per cycle while catching up")
p.add_argument("--load-abort", type=int, default=750, help="Present_Load (of 1000) held for 1 s -> stop")
args = p.parse_args()

follower = make_follower()
leader = make_leader()
fbus, lbus = follower.bus, leader.bus


def clamp(v: float, lo: float, hi: float) -> float:
    return min(max(v, lo), hi)


connect_bus_no_torque(fbus)
connect_bus_no_torque(lbus)
try:
    check_motors_ok(fbus, "follower")
    check_motors_ok(lbus, "leader")
    limits = safe_limits_deg(fbus)
    limit_follower(fbus, speed=1500, accel=50, torque_limit=800)

    input("Support the FOLLOWER, keep the LEADER still, press Enter (follower torque turns on in place)...")
    torque_on_in_place(fbus)
    cmd = fbus.sync_read("Present_Position", num_retry=3)  # last commanded goal, normalized
    print("Follower holding. Catching up with the leader slowly - keep the leader still...", flush=True)

    period = 1.0 / args.fps
    following = False
    high_load_since: float | None = None
    i = 0
    try:
        while True:
            t0 = time.perf_counter()
            lead = lbus.sync_read("Present_Position", num_retry=3)
            step = args.max_step if following else args.ramp_step
            worst = 0.0
            for m in MOTORS:
                tgt = clamp(lead[m], *limits[m])
                d = clamp(tgt - cmd[m], -step, step)
                cmd[m] += d
                worst = max(worst, abs(tgt - cmd[m]))
            fbus.sync_write("Goal_Position", cmd)

            if not following and worst < 2.0:
                following = True
                print("Caught up -> following the leader. Ctrl+C to stop.", flush=True)

            i += 1
            if i % 5 == 0:
                load = {m: abs(v) for m, v in fbus.sync_read("Present_Load", normalize=False, num_retry=2).items()}
                hot = max(load, key=load.get)
                if load[hot] > args.load_abort:
                    high_load_since = high_load_since or time.time()
                    if time.time() - high_load_since > 1.0:
                        raise RuntimeError(f"high load on {hot}: {load[hot]}")
                else:
                    high_load_since = None

            time.sleep(max(0.0, period - (time.perf_counter() - t0)))
    except (KeyboardInterrupt, RuntimeError) as e:
        print(f"\nstopped: {e!r}. Follower holds its current pose.", flush=True)
        try:
            torque_on_in_place(fbus)  # goal = where it is now, so it stops where it is
        except Exception as e2:  # noqa: BLE001
            print("!! could not hold pose:", e2, flush=True)
    input("SUPPORT THE FOLLOWER with your hand, then press Enter to release torque...")
finally:
    release_all(fbus)
    fbus.disconnect(disable_torque=False)
    lbus.disconnect(disable_torque=False)
    print("released.", flush=True)
