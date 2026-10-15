"""Move Feetech STS/SMS bus servos to random positions around their start pose.

Each servo moves within +/- --amplitude ticks of the position it had at startup
(4096 ticks = 360 deg), clamped to the servo's EEPROM angle limits. On exit
(Ctrl+C or end of cycles) servos return to the start pose and torque is disabled.

Example:
    python dev_scripts/random_servo.py --port COM19
    python dev_scripts/random_servo.py --port COM19 --ids 1 2 3 --amplitude 300 --cycles 20
"""

import argparse
import random
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


def scan(ph, pk, max_id):
    return [i for i in range(0, max_id + 1) if pk.ping(ph, i)[1] == scs.COMM_SUCCESS]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", type=int, nargs="*", help="Servo IDs (default: scan 0..--max-id)")
    parser.add_argument("--max-id", type=int, default=20)
    parser.add_argument("--amplitude", type=int, default=200, help="Max offset from start pose in ticks")
    parser.add_argument("--speed", type=int, default=600, help="Goal speed in ticks/s")
    parser.add_argument("--acceleration", type=int, default=20)
    parser.add_argument("--cycles", type=int, default=10)
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between targets")
    parser.add_argument("--seed", type=int)
    args = parser.parse_args()

    rng = random.Random(args.seed)
    ph = scs.PortHandler(args.port)
    pk = scs.PacketHandler(0)
    if not ph.openPort() or not ph.setBaudRate(args.baudrate):
        sys.exit(f"Cannot open {args.port} at {args.baudrate} baud")

    ids = args.ids or scan(ph, pk, args.max_id)
    if not ids:
        ph.closePort()
        sys.exit("No servo responded. Check servo power supply, cabling and baudrate.")

    start, limits = {}, {}
    for i in ids:
        pos, res, err = pk.read2ByteTxRx(ph, i, ADDR_PRESENT_POSITION)
        check(res, err, pk, f"id {i} read position")
        lo, _, _ = pk.read2ByteTxRx(ph, i, ADDR_MIN_ANGLE_LIMIT)
        hi, _, _ = pk.read2ByteTxRx(ph, i, ADDR_MAX_ANGLE_LIMIT)
        if lo >= hi:  # limits disabled (wheel/multi-turn) -> use full single-turn range
            lo, hi = 0, 4095
        start[i], limits[i] = pos, (lo, hi)
        print(f"id {i}: start={pos} limits=[{lo}, {hi}]")

    def move_all(targets):
        for i, goal in targets.items():
            res, err = pk.write2ByteTxRx(ph, i, ADDR_GOAL_POSITION, goal)
            check(res, err, pk, f"id {i} write goal")

    try:
        for i in ids:
            check(*pk.write1ByteTxRx(ph, i, ADDR_ACCELERATION, args.acceleration), pk, f"id {i} accel")
            check(*pk.write2ByteTxRx(ph, i, ADDR_GOAL_SPEED, args.speed), pk, f"id {i} speed")
            check(*pk.write2ByteTxRx(ph, i, ADDR_GOAL_POSITION, start[i]), pk, f"id {i} hold")
            check(*pk.write1ByteTxRx(ph, i, ADDR_TORQUE_ENABLE, 1), pk, f"id {i} torque on")

        for c in range(args.cycles):
            targets = {}
            for i in ids:
                lo, hi = limits[i]
                goal = start[i] + rng.randint(-args.amplitude, args.amplitude)
                targets[i] = max(lo, min(hi, goal))
            print(f"cycle {c + 1}/{args.cycles}: {targets}")
            move_all(targets)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        print("returning to start pose")
        try:
            move_all(start)
            time.sleep(max(1.0, args.amplitude / max(args.speed, 1) + 0.5))
        finally:
            for i in ids:
                pk.write1ByteTxRx(ph, i, ADDR_TORQUE_ENABLE, 0)
            ph.closePort()
            print("torque disabled")


if __name__ == "__main__":
    main()
