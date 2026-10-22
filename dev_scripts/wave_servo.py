"""Move Feetech STS bus servos continuously in a travelling wave.

Each servo oscillates smoothly between its start pose and start +/- --amplitude ticks
(4096 ticks = 360 deg). Servo k lags servo k-1 by --phase-step of a period, in --ids order,
so the motion travels along the chain like a wave. The swing goes towards the inside of the
servo's angle limits. Runs until Ctrl+C, then returns to the start pose and disables torque.

Example:
    python dev_scripts/wave_servo.py --port COM19 --ids 1 2 3 4 5 6
    python dev_scripts/wave_servo.py --port COM19 --ids 1 2 3 4 5 6 --amplitude 300 --period 3
    # start one by one, then stop one by one, repeat
    python dev_scripts/wave_servo.py --port COM19 --ids 1 2 3 4 5 6 --cascade
"""

import argparse
import math
import sys
import time

import scservo_sdk as scs

ADDR_MIN_ANGLE_LIMIT = 9
ADDR_MAX_ANGLE_LIMIT = 11
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_PRESENT_POSITION = 56


def check(res, err, pk, what):
    if res != scs.COMM_SUCCESS:
        raise RuntimeError(f"{what}: {pk.getTxRxResult(res)}")
    if err:
        print(f"warning {what}: {pk.getRxPacketError(err)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", type=int, nargs="+", required=True, help="Servo IDs in wave order")
    parser.add_argument("--amplitude", type=int, default=200, help="Max swing from start pose in ticks")
    parser.add_argument("--period", type=float, default=2.0, help="Seconds per oscillation")
    parser.add_argument("--phase-step", type=float, default=1 / 6, help="Lag between neighbours (periods)")
    parser.add_argument("--rate", type=float, default=20.0, help="Goal updates per second")
    parser.add_argument("--speed", type=int, default=2000, help="Goal speed cap in ticks/s")
    parser.add_argument("--acceleration", type=int, default=50)
    parser.add_argument(
        "--cascade",
        action="store_true",
        help="Start servos one by one in --ids order, then stop them one by one in the same order; repeat",
    )
    parser.add_argument("--step-time", type=float, default=1.0, help="Cascade: seconds between starts/stops")
    parser.add_argument("--ramp-time", type=float, default=0.5, help="Cascade: seconds to fade a servo in/out")
    args = parser.parse_args()

    ph = scs.PortHandler(args.port)
    pk = scs.PacketHandler(0)
    if not ph.openPort() or not ph.setBaudRate(args.baudrate):
        sys.exit(f"Cannot open {args.port} at {args.baudrate} baud")

    missing = [i for i in args.ids if pk.ping(ph, i)[1] != scs.COMM_SUCCESS]
    if missing:
        print(f"warning: no response from ids {missing}, skipping them")
    ids = [i for i in args.ids if i not in missing]
    if not ids:
        ph.closePort()
        sys.exit("No servo responded. Check servo power supply, cabling and baudrate.")

    start, direction = {}, {}
    for i in ids:
        pos, res, err = pk.read2ByteTxRx(ph, i, ADDR_PRESENT_POSITION)
        check(res, err, pk, f"id {i} read position")
        lo, _, _ = pk.read2ByteTxRx(ph, i, ADDR_MIN_ANGLE_LIMIT)
        hi, _, _ = pk.read2ByteTxRx(ph, i, ADDR_MAX_ANGLE_LIMIT)
        if lo >= hi:
            lo, hi = 0, 4095
        pos = max(lo, min(hi, pos))
        start[i], direction[i] = pos, (1 if pos < (lo + hi) / 2 else -1)
        print(f"id {i}: start={pos} limits=[{lo}, {hi}] swing={'+' if direction[i] > 0 else '-'}")

    def write_goals(goals):
        for i, goal in goals.items():
            res, err = pk.write2ByteTxRx(ph, i, ADDR_GOAL_POSITION, goal)
            check(res, err, pk, f"id {i} write goal")

    try:
        for i in ids:
            check(*pk.write1ByteTxRx(ph, i, ADDR_ACCELERATION, args.acceleration), pk, f"id {i} accel")
            check(*pk.write2ByteTxRx(ph, i, ADDR_GOAL_SPEED, args.speed), pk, f"id {i} speed")
            check(*pk.write2ByteTxRx(ph, i, ADDR_GOAL_POSITION, start[i]), pk, f"id {i} hold")
            check(*pk.write1ByteTxRx(ph, i, ADDR_TORQUE_ENABLE, 1), pk, f"id {i} torque on")

        n = len(ids)
        # Cascade round: starts at k*T, all run until n*T, stops at (n+k)*T, idle until (2n+1)*T.
        round_time = (2 * n + 1) * args.step_time

        def envelope(k, t):
            if not args.cascade:
                return 1.0
            tr = t % round_time
            on, off = k * args.step_time, (n + k) * args.step_time
            ramp = max(args.ramp_time, 1e-6)
            return max(0.0, min(1.0, (tr - on) / ramp, 1 - (tr - off) / ramp))

        print("cascade" if args.cascade else "waving", "- Ctrl+C to stop")
        t0 = time.monotonic()
        dt = 1.0 / args.rate
        last_round = -1
        while True:
            t = time.monotonic() - t0
            if args.cascade and int(t // round_time) != last_round:
                last_round = int(t // round_time)
                print(f"round {last_round + 1}")
            goals = {}
            for k, i in enumerate(ids):
                phase = 2 * math.pi * (t / args.period - k * args.phase_step)
                offset = args.amplitude * (1 - math.cos(phase)) / 2  # 0..amplitude, smooth
                goals[i] = start[i] + direction[i] * round(offset * envelope(k, t))
            write_goals(goals)
            time.sleep(max(0.0, dt - (time.monotonic() - t0 - t)))
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        print("returning to start pose")
        ph.is_using = False  # Ctrl+C mid-transaction leaves the SDK's busy flag set
        try:
            write_goals(start)
            time.sleep(1.0)
        finally:
            for i in ids:
                pk.write1ByteTxRx(ph, i, ADDR_TORQUE_ENABLE, 0)
            ph.closePort()
            print("torque disabled")


if __name__ == "__main__":
    main()
