"""Play with ONE loose STS3215 servo from the command line (the video's 20:39-24:11 part).

usage (run from the project folder with the LeRobot venv python):
  python tools\\servo_play.py COM5 scan                      who answers? (id, model, voltage, temp, mode)
  python tools\\servo_play.py COM5 setid --new 7             give the ONLY connected motor id 7 (EEPROM)
  python tools\\servo_play.py COM5 info --id 1 [--watch]     position / speed / load / voltage / temp
  python tools\\servo_play.py COM5 goto 3000 --id 1 [--speed 800 --accel 20]
  python tools\\servo_play.py COM5 sweep 1500 2500 --id 1 [--times 5 --speed 1500 --accel 50]
  python tools\\servo_play.py COM5 wheel 400 --id 1 [--seconds 3]    turn continuously (speed mode)

Units: 4096 steps = 1 turn; speed in steps/s; accel 1 = 100 steps/s^2 (see servo_common.py).
Every motion starts from where the motor is (no jump) and ends with torque OFF and position mode.
Do not put the horn against anything: a blocked servo heats up. Stop anytime with Ctrl+C.
"""

import argparse
import time

from servo_common import (
    POSITION_MODE,
    WHEEL_MODE,
    check_answers,
    clamp,
    found_ids,
    name_of,
    open_bus,
    ping_each,
    read_status,
    refuse_if_arm,
    safe_close,
    set_caps,
    torque_on_here,
    wait_until_near,
)

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("port")
sub = ap.add_subparsers(dest="cmd", required=True)
sub.add_parser("scan")
s = sub.add_parser("setid")
s.add_argument("--new", type=int, required=True)
s = sub.add_parser("info")
s.add_argument("--id", type=int, default=1)
s.add_argument("--watch", action="store_true", help="print 5 times a second until Ctrl+C")
s = sub.add_parser("goto")
s.add_argument("target", type=int)
s.add_argument("--id", type=int, default=1)
s.add_argument("--speed", type=int, default=800)
s.add_argument("--accel", type=int, default=20)
s.add_argument("--hold", action="store_true", help="keep torque on until Enter (otherwise release at the end)")
s = sub.add_parser("sweep")
s.add_argument("a", type=int)
s.add_argument("b", type=int)
s.add_argument("--id", type=int, default=1)
s.add_argument("--times", type=int, default=5)
s.add_argument("--speed", type=int, default=1500)
s.add_argument("--accel", type=int, default=50)
s = sub.add_parser("wheel")
s.add_argument("speed", type=int, help="steps/s, minus = other direction (try 200..1000)")
s.add_argument("--id", type=int, default=1)
s.add_argument("--seconds", type=float, default=3.0)
args = ap.parse_args()

MIN_POS, MAX_POS = 50, 4045  # keep away from the 0/4095 wrap

if args.cmd == "scan":
    bus = open_bus(args.port, [1])
    ids = found_ids(bus)
    if ids is None:
        each = ping_each(bus)
        bus.disconnect(disable_torque=False)
        raise SystemExit(f"the broadcast ping got no clean reply. Ids 1-10 answering one by one: {each}. "
                         "Two motors with the SAME id garble each other: connect one motor at a time.")
    bus.disconnect(disable_torque=False)
    if not ids:
        raise SystemExit("nobody answers at 1 Mbps. Power on? Cable in? A brand-new motor may still be at its "
                         "factory id 1 and 1 Mbps; if not, try: servo_play.py <PORT> setid --new 1")
    bus = open_bus(args.port, sorted(ids))
    not_position = []
    for i, model in sorted(ids.items()):
        st = read_status(bus, i)
        if st["mode"] != POSITION_MODE:
            not_position.append(i)
        print(f"id {i}: model {model} ({'STS3215' if model == 777 else '?'})  pos {st['position']}  "
              f"{st['volts']:.1f} V  {st['temp_c']} C  mode {st['mode']} ({'position' if st['mode'] == 0 else 'NOT position'})"
              f"  torque {st['torque']}  status {st['status']}")
    if not_position and len(ids) <= 2 and len(ping_each(bus)) <= 2:
        print(f"id {not_position}: not in position mode (left in wheel mode?) -> resetting to position mode")
        safe_close(bus, not_position)
    else:
        if not_position:
            print("!! an arm with a motor not in position mode: do not move it; ask before changing anything")
        bus.disconnect(disable_torque=False)  # scan never touches torque of motors that are fine
    raise SystemExit(0)

if args.cmd == "setid":
    if not 1 <= args.new <= 252:
        raise SystemExit("id must be 1..252")
    probe = open_bus(args.port, [1])
    ids = found_ids(probe)
    each = ping_each(probe)
    probe.disconnect(disable_torque=False)
    if ids is None or len(ids) > 1 or len(each) > 1:
        raise SystemExit(f"not exactly one clean answer (broadcast {ids}, one by one {each}): connect ONLY the "
                         "motor to change. Two new motors both at id 1 cannot be told apart on the bus.")
    print("setid writes the id into EEPROM. Two motors with the same id answer as one, so the tool cannot")
    if input("see them. Is ONLY ONE motor connected to the board? type yes: ").strip() != "yes":
        raise SystemExit("stopped; nothing written")
    bus = open_bus(args.port, [args.new])
    bus.setup_motor(name_of(args.new))  # LeRobot: finds the single motor at any baud, writes id + 1 Mbps
    print(f"done: id {args.new}, 1 Mbps. Check with: python tools\\servo_play.py {args.port} scan")
    bus.disconnect(disable_torque=False)
    raise SystemExit(0)

bus = open_bus(args.port, [args.id])
refuse_if_arm(bus)
check_answers(bus, [args.id])
n = name_of(args.id)
try:
    if args.cmd == "info":
        while True:
            st = read_status(bus, args.id)
            print("  ".join(f"{k} {v}" for k, v in st.items()), flush=True)
            if not args.watch:
                break
            time.sleep(0.2)

    elif args.cmd == "goto":
        target = clamp(args.target, MIN_POS, MAX_POS)
        here = torque_on_here(bus, args.id, args.speed, args.accel)
        print(f"from {here} to {target} at {args.speed} steps/s, accel {args.accel}")
        t0 = time.monotonic()
        bus.write("Goal_Position", n, target, normalize=False)
        pos = wait_until_near(bus, args.id, target)
        print(f"reached {pos} in {time.monotonic() - t0:.2f} s; load {read_status(bus, args.id)['load']}")
        if args.hold:
            input("holding. Press Enter to release torque ...")

    elif args.cmd == "sweep":
        a, b = clamp(args.a, MIN_POS, MAX_POS), clamp(args.b, MIN_POS, MAX_POS)
        here = torque_on_here(bus, args.id, args.speed, args.accel)
        print(f"sweep {a} <-> {b}, {args.times} times (start {here})")
        for k in range(args.times):
            for goal in (a, b):
                bus.write("Goal_Position", n, goal, normalize=False)
                t0 = time.monotonic()
                pos = wait_until_near(bus, args.id, goal)
                print(f"  {k + 1}: -> {goal}  at {pos}  {time.monotonic() - t0:.2f} s", flush=True)

    elif args.cmd == "wheel":
        if abs(args.speed) > 3000:
            raise SystemExit("keep |speed| <= 3000 steps/s for this test")
        bus.disable_torque(n, num_retry=2)
        bus.write("Operating_Mode", n, WHEEL_MODE, normalize=False)
        set_caps(bus, args.id, 1, 20)  # Goal_Velocity is the turning speed in wheel mode: start at ~0
        bus.enable_torque(n, num_retry=2)
        print(f"turning at {args.speed} steps/s for {args.seconds} s (mode {WHEEL_MODE} = wheel)")
        bus.write("Goal_Velocity", n, args.speed, normalize=False)
        t_end = time.monotonic() + args.seconds
        while time.monotonic() < t_end:
            st = read_status(bus, args.id)
            print(f"  velocity {st['velocity']}  load {st['load']}", flush=True)
            time.sleep(0.25)
        bus.write("Goal_Velocity", n, 0, normalize=False)  # in wheel mode 0 = stop
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\nCtrl+C")
finally:
    safe_close(bus, [args.id])
    print(f"released: torque off, mode {POSITION_MODE} (position)")
