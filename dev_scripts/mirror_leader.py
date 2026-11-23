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
import signal
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
ADDR_HOMING_OFFSET = 31
ADDR_OPERATING_MODE = 33
ADDR_LOCK = 55
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def open_bus(port, baudrate):
    ph = scs.PortHandler(port)
    if not ph.openPort() or not ph.setBaudRate(baudrate):
        sys.exit(f"Cannot open {port} at {baudrate} baud")
    return ph


def decode(value):
    """Present_Position uses bit 15 as sign."""
    return -(value & 0x7FFF) if value & 0x8000 else value


TURN = 4096


def wrap(value):
    return int(value) % TURN


def span(lo, hi):
    """Ticks from lo to hi going towards larger values (through 4095/0 if hi < lo)."""
    return (hi - lo) % TURN


def unwrap(pos, lo, rng_span):
    """Distance of pos from lo along the range; positions in the gap after the range split into >span / <0."""
    d = (pos - lo) % TURN
    gap = TURN - rng_span
    return d - TURN if d > rng_span + gap / 2 else d


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
    parser.add_argument(
        "--monitor-hz",
        type=float,
        default=5.0,
        help="Live leader/follower position display rate; reads follower positions for it (0 = goals once per s)",
    )
    parser.add_argument("--monitor-ids", type=int, nargs="+", help="Ids shown by the live display (default: all)")
    parser.add_argument(
        "--jump-threshold",
        type=int,
        default=300,
        help="Log a line when a monitored position changes by more than this between consecutive loop reads "
        "(follower is read every loop only when --monitor-ids is given; 0 = off)",
    )
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
        "--auto-align",
        action="store_true",
        help="Instead of refusing, slowly move the follower joints that differ too much to the leader pose "
        "(at --align-speed), then start mirroring",
    )
    parser.add_argument("--align-speed", type=int, default=600, help="Follower speed cap in ticks/s for --auto-align")
    parser.add_argument(
        "--align-only",
        action="store_true",
        help="Exit once aligned instead of mirroring (with --auto-align: the follower keeps torque at the "
        "leader pose), e.g. before handing the arms to lerobot-record",
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
    parser.add_argument(
        "--range-files",
        type=Path,
        nargs="*",
        default=[],
        help="Joint range files (from record_joint_range.py + center_joint_range.py). For those joints the "
        "follower goal is F_min + (L - L_min) * F_span / L_span, clamped to [F_min, F_max] shrunk by --range-margin. "
        "The follower is read every loop and its torque is cut if it leaves [F_min, F_max] by more than "
        "--guard-ticks (protects against the motor running round the 4095/0 seam).",
    )
    parser.add_argument(
        "--velocity-ids",
        type=int,
        nargs="*",
        default=[],
        help="Range-mapped follower ids driven in speed (wheel) mode with a software position loop, so the "
        "rotation direction follows the range (e.g. through 4095/0) instead of the dial. Restored to "
        "position mode with torque off on exit.",
    )
    parser.add_argument("--vel-kp", type=float, default=3.0, help="Speed per tick of error (speed units ~ ticks/s)")
    parser.add_argument("--vel-max", type=int, default=1500, help="Speed limit for --velocity-ids")
    parser.add_argument("--vel-deadband", type=int, default=8, help="Stop when within this many ticks of target")
    parser.add_argument("--range-margin", type=int, default=150, help="Keep range-mapped goals this far inside")
    parser.add_argument("--guard-ticks", type=int, default=60, help="Out-of-range tolerance before torque cut")
    parser.add_argument("--shared-ids", type=int, nargs="*", default=[], help="Follower ids used by several motors")
    parser.add_argument("--shared-speed", type=int, default=400, help="Goal speed cap for --shared-ids (ticks/s)")
    args = parser.parse_args()
    if args.align_now and args.offsets_file is None:
        args.offsets_file = Path("dev_calibration/mirror_offsets.json")
    shared = set(args.shared_ids)

    def stop_requested(signum, frame):
        raise KeyboardInterrupt

    # `kill`, systemd and a Pi shutdown send SIGTERM: stop like Ctrl+C so the cleanup below still runs.
    signal.signal(signal.SIGTERM, stop_requested)

    pk = scs.PacketHandler(0)
    lph = open_bus(args.leader_port, args.baudrate)
    fph = open_bus(args.follower_port, args.baudrate)

    missing = [f"L{i}" for i in args.ids if pk.ping(lph, i)[1] != scs.COMM_SUCCESS]
    missing += [f"F{i}" for i in args.ids if i not in shared and pk.ping(fph, i)[1] != scs.COMM_SUCCESS]
    if missing:
        sys.exit(f"no response from {', '.join(missing)}")

    def set_mode(i, mode):
        pk.write1ByteTxRx(fph, i, ADDR_TORQUE_ENABLE, 0)
        pk.write1ByteTxRx(fph, i, ADDR_LOCK, 0)
        pk.write1ByteTxRx(fph, i, ADDR_OPERATING_MODE, mode)
        pk.write1ByteTxRx(fph, i, ADDR_LOCK, 1)
        time.sleep(1.0)  # the motor restarts after an EEPROM mode change
        for _ in range(30):
            try:
                if pk.ping(fph, i)[1] == scs.COMM_SUCCESS:
                    break
            except IndexError:  # truncated reply while the motor is still booting
                fph.is_using = False
            time.sleep(0.1)
        fph.clearPort()

    # A run that was killed (or lost its host) mid-mirror can leave a joint in speed mode, turning at its
    # last speed: stop it and put it back in position mode before anything else reads or drives it.
    for i in args.ids:
        if i in shared:
            continue
        mode, res, _ = pk.read1ByteTxRx(fph, i, ADDR_OPERATING_MODE)
        if res == scs.COMM_SUCCESS and mode != 0:
            pk.write2ByteTxRx(fph, i, ADDR_GOAL_SPEED, 0)
            set_mode(i, 0)
            pk.write1ByteTxRx(fph, i, ADDR_TORQUE_ENABLE, 0)
            print(f"F{i} was left in operating mode {mode}: stopped and put back in position mode")

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

    # linear range mapping per joint: id -> (l_min, l_max, f_min, f_max); usable: id -> (goal_min, goal_max)
    ranges, usable = {}, {}
    for path in args.range_files:
        rng = json.loads(path.read_text(encoding="utf-8-sig"))
        i = int(rng["id"])
        # MIN -> MAX runs towards larger ticks; if MAX < MIN the range passes through the 4095/0 seam.
        ranges[i] = (rng["min"][f"L{i}"], rng["max"][f"L{i}"], rng["min"][f"F{i}"], rng["max"][f"F{i}"])
        # optional narrower window (dial values) the follower is driven in; default: the whole range
        u = rng.get("follower_usable", {})
        f_span = span(ranges[i][2], ranges[i][3])
        usable[i] = (
            unwrap(u["min"], ranges[i][2], f_span) if "min" in u else 0,
            unwrap(u["max"], ranges[i][2], f_span) if "max" in u else f_span,
        )
        print(
            f"L{i}->F{i} range mapping: L {ranges[i][0]}..{ranges[i][1]} ({span(ranges[i][0], ranges[i][1])} ticks) -> "
            f"F {ranges[i][2]}..{ranges[i][3]} ({f_span} ticks"
            f"{', through 4095/0' if ranges[i][3] < ranges[i][2] else ''}), "
            f"F goals limited to {wrap(ranges[i][2] + usable[i][0])}..{wrap(ranges[i][2] + usable[i][1])}"
        )

    def follower_offset(i, fpos):
        """Follower position as distance from its MIN along the range (may be <0 or >span outside it)."""
        f_min, f_max = ranges[i][2], ranges[i][3]
        return unwrap(fpos, f_min, span(f_min, f_max))

    def target(i, lpos):
        if i in ranges:
            l_min, l_max, f_min, f_max = ranges[i]
            l_span, f_span = span(l_min, l_max), span(f_min, f_max)
            offset = unwrap(lpos, l_min, l_span) * f_span / l_span
            offset = max(usable[i][0], min(usable[i][1], offset))
            return wrap(round(f_min + offset))
        sign, offset = mapping[i]
        return max(limits[i][0], min(limits[i][1], sign * lpos + offset))

    velocity = set(args.velocity_ids)
    if velocity - set(ranges):
        sys.exit(f"--velocity-ids {sorted(velocity - set(ranges))} need a --range-files entry")
    vel_active = set()  # ids currently in speed mode (their Present_Position is raw, without homing)
    homing = {}

    def read_f(i):
        """Follower position in the homed frame, also for joints running in speed mode."""
        pos = read_pos(fph, pk, i)
        if pos is not None and i in vel_active:
            pos = (pos - homing[i]) % TURN
        return pos

    def set_speed(i, v, cap=None):
        cap = args.vel_max if cap is None else min(cap, args.vel_max)
        v = int(max(-cap, min(cap, v)))
        pk.write2ByteTxRx(fph, i, ADDR_GOAL_SPEED, abs(v) | (0x8000 if v < 0 else 0))

    def start_velocity(i):
        raw, _, _ = pk.read2ByteTxRx(fph, i, ADDR_HOMING_OFFSET)
        homing[i] = -(raw & 0x7FF) if raw & 0x800 else raw & 0x7FF
        vel_active.add(i)  # before the mode change, so an interrupt during it still restores position mode
        set_mode(i, 1)
        set_speed(i, 0)
        pk.write1ByteTxRx(fph, i, ADDR_ACCELERATION, args.acceleration)
        pk.write1ByteTxRx(fph, i, ADDR_TORQUE_ENABLE, 1)
        print(f"F{i}: speed mode (homing {homing[i]}), direction follows the range")

    def stop_velocity(i):
        set_speed(i, 0)
        pos = read_f(i)
        set_mode(i, 0)
        vel_active.discard(i)
        # writing Goal_Position re-enables torque, and the motor may restart with torque on:
        # set the goal first, then disable torque last
        if pos is not None:
            pk.write2ByteTxRx(fph, i, ADDR_GOAL_POSITION, pos)
        pk.write1ByteTxRx(fph, i, ADDR_TORQUE_ENABLE, 0)
        torque, _, _ = pk.read1ByteTxRx(fph, i, ADDR_TORQUE_ENABLE)
        print(f"F{i}: back in position mode, torque={torque} (at {pos})")

    def drive_velocity(i, goal, fpos, cap=None):
        """Software position loop: speed towards goal along the range, never through the gap."""
        if fpos is None:
            return
        f_span = span(ranges[i][2], ranges[i][3])
        err = follower_offset(i, goal) - follower_offset(i, fpos)
        cur = follower_offset(i, fpos)
        v = 0 if abs(err) <= args.vel_deadband else args.vel_kp * err
        if (cur <= 0 and v < 0) or (cur >= f_span and v > 0):
            v = 0  # at an end of the range: only move back inside
        set_speed(i, v, cap)

    def restore_velocity_joints():
        """Stop every speed-mode joint and put it back in position mode; never cut short."""
        # A second Ctrl+C / stop signal must not cut the cleanup short: a joint left in speed mode keeps
        # turning at its last speed until it hits its mechanical stop.
        for name in ("SIGINT", "SIGTERM", "SIGBREAK"):
            if hasattr(signal, name):
                signal.signal(getattr(signal, name), signal.SIG_IGN)
        lph.is_using = fph.is_using = False  # Ctrl+C mid-transaction leaves the SDK's busy flag set
        for i in sorted(vel_active):
            try:
                set_speed(i, 0)  # stop every speed-mode joint first; the mode change below takes ~1 s each
            except Exception:
                fph.is_using = False
        for i in sorted(vel_active):
            try:
                stop_velocity(i)
            except Exception as e:  # keep restoring the other joints
                fph.is_using = False
                print(f"F{i}: could not restore position mode ({e!r}); its speed was set to 0")

    def start_diffs():
        diffs = {}
        for i in args.ids:
            if i in shared:
                continue
            lpos, fpos = read_pos(lph, pk, i), read_f(i)
            if lpos is None or fpos is None:
                diffs[i] = None
                continue
            if i in velocity:  # along the range, the way the speed loop will turn it
                diffs[i] = round(follower_offset(i, target(i, lpos)) - follower_offset(i, fpos))
                continue
            # signed: how far the follower would move (+ = towards larger ticks)
            diffs[i] = target(i, lpos) - fpos
            if i in ranges:  # shortest way round the dial
                diffs[i] = (diffs[i] + TURN // 2) % TURN - TURN // 2
        return diffs

    def far(diffs):
        # Range-mapped joints are excluded: they are enabled at their current position and re-armed safely.
        # With --auto-align, speed-mode joints are aligned too (slowly, along their range).
        def checked(i):
            return i not in ranges or (args.auto_align and i in velocity)

        return [i for i, d in diffs.items() if checked(i) and (d is None or abs(d) > args.max_start_diff)]

    diffs = start_diffs()
    for i in args.ids:
        name = NAMES[i - 1] if 1 <= i <= len(NAMES) else f"id{i}"
        if i in shared:
            print(f"L{i}->F{i} {name:<14} shared id: write-only at {args.shared_speed} ticks/s (not checked)")
        else:
            print(f"L{i}->F{i} {name:<14} follower would move {diffs[i]} ticks  F limits={limits[i]}")
    if far(diffs) and not (args.wait_align or args.auto_align):
        lph.closePort()
        fph.closePort()
        sys.exit(
            f"leader and follower poses differ too much on {['F%d' % i for i in far(diffs)]} "
            f"(> {args.max_start_diff} ticks). Pose the leader like the follower and retry, "
            f"or use --wait-align / --auto-align."
        )
    moved = set()  # follower joints driven by --auto-align (torque on)
    try:
        last_print = 0.0
        while far(diffs):
            if args.auto_align:
                # drive only the joints that are too far, slowly, towards where the leader is now
                too_far = far(diffs)
                for i in too_far:
                    lpos = read_pos(lph, pk, i)
                    if lpos is None or diffs[i] is None:
                        continue
                    if i in velocity:
                        if i not in vel_active:
                            start_velocity(i)
                        moved.add(i)
                        drive_velocity(i, target(i, lpos), read_f(i), cap=args.align_speed)
                        continue
                    if i not in moved:
                        pk.write1ByteTxRx(fph, i, ADDR_ACCELERATION, args.acceleration)
                        pk.write2ByteTxRx(fph, i, ADDR_GOAL_SPEED, args.align_speed)
                        moved.add(i)
                    pk.write2ByteTxRx(fph, i, ADDR_GOAL_POSITION, target(i, lpos))
                    pk.write1ByteTxRx(fph, i, ADDR_TORQUE_ENABLE, 1)
                for i in vel_active - set(too_far):
                    set_speed(i, 0)  # close enough: wait for the others
            if time.perf_counter() - last_print >= 0.5:
                last_print = time.perf_counter()
                label = "auto-align follower" if args.auto_align else "align leader"
                print(f"{label}: " + "  ".join(f"F{i}:{d:+d}" if d is not None else f"F{i}:?"
                                               for i, d in diffs.items()), flush=True)
            time.sleep(0.1 if args.auto_align else 0.5)
            diffs = start_diffs()
        for i in vel_active:
            set_speed(i, 0)
    except KeyboardInterrupt:
        restore_velocity_joints()
        lph.closePort()
        fph.closePort()
        sys.exit("aborted before mirroring; "
                 + (f"F{sorted(moved)} stopped where they were" if moved else "follower was not moved"))
    print("aligned", flush=True)
    if args.align_only:
        restore_velocity_joints()  # speed-mode joints go back to position mode (torque off)
        lph.closePort()
        fph.closePort()
        print("align-only: done" + (f", F{sorted(moved)} were moved to the leader pose" if moved else ""))
        return

    waiting = set()  # range-mapped joints held limp until they are inside their usable range

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
            if i in velocity:
                if i not in vel_active:  # --auto-align may have started it already
                    start_velocity(i)  # the speed loop itself only ever drives back inside the range
                continue
            hold = read_pos(fph, pk, i)
            lo, hi = limits[i]
            if i in ranges:
                lo, hi = 0, TURN - 1  # range-mapped joints hold exactly where they are
                if hold is None or not usable[i][0] <= follower_offset(i, hold) <= usable[i][1]:
                    pk.write1ByteTxRx(fph, i, ADDR_TORQUE_ENABLE, 0)
                    waiting.add(i)
                    print(f"F{i} reads {hold}, outside its usable range: torque off until it is moved inside")
                    continue
            pk.write1ByteTxRx(fph, i, ADDR_ACCELERATION, args.acceleration)
            pk.write2ByteTxRx(fph, i, ADDR_GOAL_SPEED, args.speed)
            if hold is not None:
                pk.write2ByteTxRx(fph, i, ADDR_GOAL_POSITION, max(lo, min(hi, hold)))
            pk.write1ByteTxRx(fph, i, ADDR_TORQUE_ENABLE, 1)

        print("mirroring, Ctrl+C to stop")
        shown = [i for i in args.ids if not args.monitor_ids or i in args.monitor_ids]
        if args.monitor_hz > 0:
            print("  ".join(f"{f'L{i} / F{i} (diff)':>20}" for i in shown))
        period = 1.0 / args.hz
        print_interval = 1.0 / args.monitor_hz if args.monitor_hz > 0 else 1.0
        last_print = 0.0
        t_start = time.perf_counter()
        track_follower = bool(args.monitor_ids) and args.jump_threshold > 0
        prev = {}  # (side, id) -> last position, for jump detection

        def check_jump(side, i, pos, t_now):
            key = (side, i)
            last = prev.get(key)
            if pos is not None and last is not None and abs(pos - last) > args.jump_threshold:
                print(
                    f"\n[JUMP {t_now - t_start:7.2f}s] {side}{i}: {last} -> {pos} ({pos - last:+d} ticks, "
                    f"{last * 360 / 4096:.1f} -> {pos * 360 / 4096:.1f} deg)",
                    flush=True,
                )
            if pos is not None:
                prev[key] = pos

        tripped = waiting  # range-mapped joints with torque off (outside usable range); re-armed when back inside

        def guard(i, fpos, t_now):
            """Cut torque if a range-mapped follower joint left its range (e.g. ran round the seam)."""
            if fpos is None:
                return
            off = follower_offset(i, fpos)
            if usable[i][0] - args.guard_ticks <= off <= usable[i][1] + args.guard_ticks:
                return
            pk.write1ByteTxRx(fph, i, ADDR_TORQUE_ENABLE, 0)
            tripped.add(i)
            print(
                f"\n[GUARD {t_now - t_start:7.2f}s] F{i} at {fpos} ({off:+.0f} from MIN), outside its range: "
                f"torque OFF until it is moved back inside",
                flush=True,
            )

        def rearm(i, fpos, t_now):
            """Re-enable a limp range-mapped joint once it is back inside its usable range."""
            if fpos is None or not usable[i][0] <= follower_offset(i, fpos) <= usable[i][1]:
                return
            pk.write1ByteTxRx(fph, i, ADDR_ACCELERATION, args.acceleration)
            pk.write2ByteTxRx(fph, i, ADDR_GOAL_SPEED, args.speed)
            pk.write2ByteTxRx(fph, i, ADDR_GOAL_POSITION, fpos)
            pk.write1ByteTxRx(fph, i, ADDR_TORQUE_ENABLE, 1)
            tripped.discard(i)
            print(f"\n[ARM {t_now - t_start:7.2f}s] F{i} at {fpos} is inside its range: mirroring", flush=True)

        while True:
            t = time.perf_counter()
            goals, leader, follower = {}, {}, {}
            for i in ranges:
                follower[i] = read_f(i)
                if i in velocity:
                    continue  # the speed loop keeps these inside the range itself
                if i in tripped:
                    rearm(i, follower[i], t)
                else:
                    guard(i, follower[i], t)
            for i in args.ids:
                pos = read_pos(lph, pk, i)
                leader[i] = pos
                if args.jump_threshold > 0 and i in shown:
                    check_jump("L", i, pos, t)
                if pos is None or i in tripped:
                    if i in vel_active:
                        set_speed(i, 0)
                    continue  # skip a corrupted read (keep last goal) or a joint cut by the guard
                goals[i] = target(i, pos)
                if i in vel_active:
                    drive_velocity(i, goals[i], follower.get(i))
                else:
                    write_goal(i, goals[i])
            if track_follower:
                for i in shown:
                    if i not in shared and i not in follower:
                        follower[i] = read_f(i)
                for i in shown:
                    if i in follower:
                        check_jump("F", i, follower[i], t)
            if t - last_print > print_interval:
                last_print = t
                if args.monitor_hz > 0:
                    cells = []
                    for i in shown:
                        fpos = follower[i] if i in follower else (None if i in shared else read_f(i))
                        lpos = leader[i]
                        diff = (
                            f"{(fpos - goals[i] + TURN // 2) % TURN - TURN // 2:+d}"
                            if fpos is not None and i in goals
                            else "?"
                        )
                        cells.append(f"{lpos if lpos is not None else '?':>5} /{fpos if fpos is not None else '?':>5} ({diff:>5})")
                    print("\r" + "  ".join(f"{c:>20}" for c in cells), end="", flush=True)
                else:
                    print("  ".join(f"F{i}={g}" for i, g in goals.items()), flush=True)
            time.sleep(max(0.0, period - (time.perf_counter() - t)))
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        restore_velocity_joints()
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
