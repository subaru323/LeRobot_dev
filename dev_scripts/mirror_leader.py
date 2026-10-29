"""Simple leader -> follower mirroring for SO-101 arms on Feetech STS buses (no LeRobot calibration files).

Reads Present_Position of leader ids 1..6 and writes it as Goal_Position of follower ids 1..6
(L1 -> F1, ..., L6 -> F6), clamped to the follower's position limits. Both arms must already have
homing offsets so that the same pose gives the same raw position (middle pose ~2047).

The follower moves with a capped speed, so the first move towards the leader pose is gradual.
On exit the follower keeps torque (holds its pose) unless --release is given.

If the two arms' homing offsets do not match, pose the leader like the follower and run once with
--align-now: the per-joint offsets (follower - leader) are saved and reused when --offsets-file is given.

--shared-ids lists follower ids answered by more than one motor (e.g. two servos both set to id 5).
Their replies collide, so those ids are never read: they are excluded from the start check, driven
write-only (no status reply awaited) at --shared-speed, and every motor with that id gets the same goal.

Example:
    python dev_scripts/mirror_leader.py --leader-port COM19 --follower-port COM20
    python dev_scripts/mirror_leader.py --leader-port COM19 --follower-port COM20 --shared-ids 5
    python dev_scripts/mirror_leader.py --leader-port COM19 --follower-port COM20 \
        --align-now --offsets-file dev_calibration/mirror_offsets.json
"""

import argparse
import json
import sys
import time
from pathlib import Path

import scservo_sdk as scs

ADDR_MIN_POSITION_LIMIT = 9
ADDR_MAX_POSITION_LIMIT = 11
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_PRESENT_POSITION = 56
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def open_bus(port, baudrate):
    ph = scs.PortHandler(port)
    if not ph.openPort() or not ph.setBaudRate(baudrate):
        sys.exit(f"Cannot open {port} at {baudrate} baud")
    return ph


def decode(value):
    """Present_Position uses bit 15 as sign."""
    return -(value & 0x7FFF) if value & 0x8000 else value


def read_pos(ph, pk, id_):
    value, res, _ = pk.read2ByteTxRx(ph, id_, ADDR_PRESENT_POSITION)
    return decode(value) if res == scs.COMM_SUCCESS else None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    parser.add_argument("--speed", type=int, default=1500, help="Follower goal speed cap in ticks/s")
    parser.add_argument("--acceleration", type=int, default=50)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--release", action="store_true", help="Disable follower torque on exit (arm may drop)")
    parser.add_argument(
        "--max-start-diff",
        type=int,
        default=300,
        help="Refuse to start if any leader/follower joint differs by more than this many ticks",
    )
    parser.add_argument(
        "--wait-align",
        action="store_true",
        help="Instead of refusing, show live differences and start once every joint is within --max-start-diff",
    )
    parser.add_argument(
        "--align-now",
        action="store_true",
        help="Treat the current leader and follower poses as matching: store per-joint offsets in --offsets-file",
    )
    parser.add_argument(
        "--offsets-file",
        type=Path,
        help="Per-joint mapping follower = sign * leader + offset; written by --align-now, read otherwise. "
        "Omit for a 1:1 mapping (leader calibrated to the follower).",
    )
    parser.add_argument("--invert", type=int, nargs="*", default=[], help="With --align-now: ids moving mirrored")
    parser.add_argument("--shared-ids", type=int, nargs="*", default=[], help="Follower ids used by several motors")
    parser.add_argument("--shared-speed", type=int, default=400, help="Goal speed cap for --shared-ids (ticks/s)")
    args = parser.parse_args()
    if args.align_now and args.offsets_file is None:
        args.offsets_file = Path("dev_calibration/mirror_offsets.json")
    shared = set(args.shared_ids)

    pk = scs.PacketHandler(0)
    lph = open_bus(args.leader_port, args.baudrate)
    fph = open_bus(args.follower_port, args.baudrate)

    missing = [f"L{i}" for i in args.ids if pk.ping(lph, i)[1] != scs.COMM_SUCCESS]
    missing += [f"F{i}" for i in args.ids if i not in shared and pk.ping(fph, i)[1] != scs.COMM_SUCCESS]
    if missing:
        sys.exit(f"no response from {', '.join(missing)}")

    limits = {}
    for i in args.ids:
        pk.write1ByteTxRx(lph, i, ADDR_TORQUE_ENABLE, 0)  # leader is moved by hand
        if i in shared:
            limits[i] = (0, 4095)  # colliding replies make limit reads unreliable
            continue
        lo, _, _ = pk.read2ByteTxRx(fph, i, ADDR_MIN_POSITION_LIMIT)
        hi, _, _ = pk.read2ByteTxRx(fph, i, ADDR_MAX_POSITION_LIMIT)
        limits[i] = (lo, hi) if lo < hi else (0, 4095)

    # mapping[i] = (sign, offset): follower goal = sign * leader + offset
    mapping = {i: (1, 0) for i in args.ids}
    if args.align_now:
        for i in args.ids:
            if i in shared:
                continue
            lpos, fpos = read_pos(lph, pk, i), read_pos(fph, pk, i)
            if lpos is None or fpos is None:
                sys.exit(f"could not read L{i}/F{i} for alignment")
            sign = -1 if i in args.invert else 1
            mapping[i] = (sign, fpos - sign * lpos)
        args.offsets_file.parent.mkdir(parents=True, exist_ok=True)
        args.offsets_file.write_text(
            json.dumps({str(i): {"sign": s, "offset": o} for i, (s, o) in mapping.items()}, indent=2)
        )
        print(f"offsets saved to {args.offsets_file}")
    elif args.offsets_file is not None:
        saved = json.loads(args.offsets_file.read_text())
        mapping.update({int(i): (v["sign"], v["offset"]) for i, v in saved.items() if int(i) in mapping})
        print(f"offsets loaded from {args.offsets_file}")

    def target(i, lpos):
        sign, offset = mapping[i]
        return max(limits[i][0], min(limits[i][1], sign * lpos + offset))

    def start_diffs():
        diffs = {}
        for i in args.ids:
            if i in shared:
                continue
            lpos, fpos = read_pos(lph, pk, i), read_pos(fph, pk, i)
            if lpos is None or fpos is None:
                diffs[i] = None
                continue
            # signed: how far the follower would move (+ = towards larger ticks)
            diffs[i] = target(i, lpos) - fpos
        return diffs

    def far(diffs):
        return [i for i, d in diffs.items() if d is None or abs(d) > args.max_start_diff]

    diffs = start_diffs()
    for i in args.ids:
        name = NAMES[i - 1] if 1 <= i <= len(NAMES) else f"id{i}"
        if i in shared:
            print(f"L{i}->F{i} {name:<14} shared id: write-only at {args.shared_speed} ticks/s (not checked)")
        else:
            print(f"L{i}->F{i} {name:<14} follower would move {diffs[i]} ticks  F limits={limits[i]}")
    if far(diffs) and not args.wait_align:
        lph.closePort()
        fph.closePort()
        sys.exit(
            f"leader and follower poses differ too much on {['F%d' % i for i in far(diffs)]} "
            f"(> {args.max_start_diff} ticks). Pose the leader like the follower and retry, or use --wait-align."
        )
    try:
        while far(diffs):
            print("align leader: " + "  ".join(f"L{i}:{d:+d}" if d is not None else f"L{i}:?"
                                                for i, d in diffs.items()), flush=True)
            time.sleep(0.5)
            diffs = start_diffs()
    except KeyboardInterrupt:
        lph.closePort()
        fph.closePort()
        sys.exit("aborted before mirroring; follower was not moved")
    print("aligned")

    def write_goal(i, goal):
        if i in shared:
            pk.write2ByteTxOnly(fph, i, ADDR_GOAL_POSITION, goal)
        else:
            pk.write2ByteTxRx(fph, i, ADDR_GOAL_POSITION, goal)

    try:
        for i in args.ids:
            if i in shared:
                # cannot read where these motors are: send the leader target and let them travel slowly
                lpos = read_pos(lph, pk, i)
                pk.write1ByteTxOnly(fph, i, ADDR_ACCELERATION, args.acceleration)
                pk.write2ByteTxOnly(fph, i, ADDR_GOAL_SPEED, args.shared_speed)
                if lpos is not None:
                    pk.write2ByteTxOnly(fph, i, ADDR_GOAL_POSITION, target(i, lpos))
                pk.write1ByteTxOnly(fph, i, ADDR_TORQUE_ENABLE, 1)
                time.sleep(0.005)
                continue
            hold = read_pos(fph, pk, i)
            pk.write1ByteTxRx(fph, i, ADDR_ACCELERATION, args.acceleration)
            pk.write2ByteTxRx(fph, i, ADDR_GOAL_SPEED, args.speed)
            if hold is not None:
                pk.write2ByteTxRx(fph, i, ADDR_GOAL_POSITION, max(limits[i][0], min(limits[i][1], hold)))
            pk.write1ByteTxRx(fph, i, ADDR_TORQUE_ENABLE, 1)

        print("mirroring, Ctrl+C to stop")
        period = 1.0 / args.hz
        last_print = 0.0
        while True:
            t = time.perf_counter()
            goals = {}
            for i in args.ids:
                pos = read_pos(lph, pk, i)
                if pos is None:
                    continue  # skip a corrupted read, keep last goal
                goals[i] = target(i, pos)
                write_goal(i, goals[i])
            if t - last_print > 1.0:
                last_print = t
                print("  ".join(f"F{i}={g}" for i, g in goals.items()), flush=True)
            time.sleep(max(0.0, period - (time.perf_counter() - t)))
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        lph.is_using = fph.is_using = False  # Ctrl+C mid-transaction leaves the SDK's busy flag set
        if args.release:
            for i in args.ids:
                if i in shared:
                    pk.write1ByteTxOnly(fph, i, ADDR_TORQUE_ENABLE, 0)
                else:
                    pk.write1ByteTxRx(fph, i, ADDR_TORQUE_ENABLE, 0)
            print("follower torque disabled")
        else:
            print("follower keeps torque (holding pose); run with --release to let it go limp")
        lph.closePort()
        fph.closePort()


if __name__ == "__main__":
    main()
