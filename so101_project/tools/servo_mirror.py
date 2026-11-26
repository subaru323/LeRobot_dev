"""Two loose STS3215 on one board: turn motor A by hand, motor B copies it (leader/follower in miniature).

usage: python tools\\servo_mirror.py COM5 [--leader 1 --follower 2 --hz 50 --max-step 40 --load-stop 600]

This is what SO-101 teleoperation does with 6 joints: read the leader's Present_Position, write it to the
follower's Goal_Position, many times a second. Things to try (the video's 24:03):
  - turn A fast: B lags. Raise --max-step or the speed cap (--speed) and compare.
  - hold B with your fingers: its load rises; above --load-stop for 1 s the tool stops (protects the motor).
The two motors need different ids first: python tools\\servo_play.py COM5 setid --new 2 (one motor at a time).
Start: A torque OFF (you turn it), B torque ON where it is, then B walks to A at --max-step per cycle.
"""

import argparse
import time

from servo_common import check_answers, clamp, name_of, open_bus, refuse_if_arm, safe_close, torque_on_here

ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("--leader", type=int, default=1)
ap.add_argument("--follower", type=int, default=2)
ap.add_argument("--hz", type=float, default=50.0)
ap.add_argument("--max-step", type=int, default=40, help="max steps per cycle (40 x 50 Hz = 2000 steps/s)")
ap.add_argument("--speed", type=int, default=2500, help="follower speed cap, steps/s")
ap.add_argument("--accel", type=int, default=100)
ap.add_argument("--load-stop", type=int, default=600, help="|Present_Load| (of 1000) held 1 s -> stop")
args = ap.parse_args()
if args.leader == args.follower:
    raise SystemExit("leader and follower need different ids")

ids = [args.leader, args.follower]
bus = open_bus(args.port, ids)
refuse_if_arm(bus)
check_answers(bus, ids)
L, F = name_of(args.leader), name_of(args.follower)
period = 1.0 / args.hz
try:
    bus.disable_torque(L, num_retry=2)
    goal = torque_on_here(bus, args.follower, args.speed, args.accel)
    print(f"follower id {args.follower} holds at {goal}. Turn leader id {args.leader} by hand. Ctrl+C to stop.")
    heavy_since = None
    n, lag_sum, t_print = 0, 0, time.monotonic()
    prev_lead, warned = None, False
    while True:
        t0 = time.monotonic()
        lead = bus.read("Present_Position", L, normalize=False, num_retry=1)
        # The leader is turned by hand and can cross 4095 -> 0. Copying that would send the follower almost a
        # full turn the long way. Outside 50..4045, or after a jump of half a turn, hold the goal instead.
        wrapped = prev_lead is not None and abs(lead - prev_lead) > 2048
        prev_lead = lead
        if wrapped or not 50 <= lead <= 4045:
            if not warned:
                print(f"\n!! leader at {lead} (near or across the 0/4095 edge): follower holds. Turn the leader back.")
                warned = True
            time.sleep(period)
            continue
        warned = False
        goal += clamp(lead - goal, -args.max_step, args.max_step)  # never more than max-step per cycle
        goal = clamp(goal, 50, 4045)
        bus.write("Goal_Position", F, goal, normalize=False, num_retry=1)
        fpos = bus.read("Present_Position", F, normalize=False, num_retry=1)
        load = abs(bus.read("Present_Load", F, normalize=False, num_retry=1))
        heavy_since = (heavy_since or t0) if load > args.load_stop else None
        if heavy_since and t0 - heavy_since > 1.0:
            print(f"\n!! follower load {load} for over 1 s -> stopping")
            break
        n += 1
        lag_sum += abs(lead - fpos)
        if t0 - t_print > 0.5:
            print(f"leader {lead:>4}  follower {fpos:>4}  lag {lag_sum / n:6.1f} steps  load {load:>4}  "
                  f"loop {n / (t0 - t_print + 1e-9):5.1f} Hz", end="\r", flush=True)
            n, lag_sum, t_print = 0, 0, t0
        time.sleep(max(0.0, period - (time.monotonic() - t0)))
except KeyboardInterrupt:
    print("\nCtrl+C")
finally:
    safe_close(bus, ids)
    print("released both: torque off, position mode")
