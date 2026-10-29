"""Calibrate an SO-101 leader arm in separate, non-interactive steps.

Same procedure as `lerobot-calibrate --teleop.type=so101_leader`, split so each step can be started
without typing into the running process (e.g. when the operator is not at the keyboard):

    1) home    : arm held at the middle pose -> writes half-turn homing offsets to the motors
    2) record  : streams positions and tracks min/max while every joint (except wrist_roll) is moved
                 through its full range; stops when <calibration-dir>/<id>.stop exists, on Ctrl+C,
                 or after --timeout seconds; then writes and saves the calibration file

Example:
    python dev_scripts/calibrate_leader_steps.py home --port COM19
    python dev_scripts/calibrate_leader_steps.py record --port COM19
    # from another shell, to finish recording:
    New-Item dev_calibration/leader_L.stop
"""

import argparse
import json
import sys
import time
from pathlib import Path

from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import OperatingMode
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

FULL_TURN_MOTOR = "wrist_roll"


def connect(args) -> SO101Leader:
    leader = SO101Leader(SO101LeaderConfig(port=args.port, id=args.id, calibration_dir=args.calibration_dir))
    leader.bus.connect()
    leader.bus.disable_torque()
    for motor in leader.bus.motors:
        leader.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
    return leader


def step_home(args, homing_path: Path) -> None:
    leader = connect(args)
    try:
        offsets = leader.bus.set_half_turn_homings()
        positions = leader.bus.sync_read("Present_Position", normalize=False)
    finally:
        leader.bus.disconnect(disable_torque=False)
    homing_path.write_text(json.dumps({m: int(v) for m, v in offsets.items()}, indent=2))
    print(f"homing offsets written to motors and saved to {homing_path}")
    for m, v in offsets.items():
        print(f"  {m:<14} offset={int(v):>6}  position now={positions[m]:>5} (expected ~2047)")


def step_record(args, homing_path: Path, stop_path: Path) -> None:
    if not homing_path.is_file():
        sys.exit(f"{homing_path} not found. Run the 'home' step first.")
    offsets = json.loads(homing_path.read_text())
    stop_path.unlink(missing_ok=True)

    leader = connect(args)
    motors = list(leader.bus.motors)
    tracked = [m for m in motors if m != FULL_TURN_MOTOR]
    try:
        pos = leader.bus.sync_read("Present_Position", tracked, normalize=False, num_retry=5)
        mins, maxes = dict(pos), dict(pos)
        print(f"recording ranges; create {stop_path} or press Ctrl+C to finish (timeout {args.timeout}s)")
        t_end = time.monotonic() + args.timeout
        last_print = 0.0
        try:
            while not stop_path.exists() and time.monotonic() < t_end:
                pos = leader.bus.sync_read("Present_Position", tracked, normalize=False, num_retry=5)
                for m in tracked:
                    mins[m], maxes[m] = min(mins[m], pos[m]), max(maxes[m], pos[m])
                if time.monotonic() - last_print > 1.0:
                    last_print = time.monotonic()
                    print("  ".join(f"{m}:{mins[m]}-{maxes[m]}" for m in tracked), flush=True)
                time.sleep(0.02)
        except KeyboardInterrupt:
            leader.bus.port_handler.is_using = False

        stop_path.unlink(missing_ok=True)
        unmoved = [m for m in tracked if mins[m] == maxes[m]]
        if unmoved:
            sys.exit(f"these joints were not moved, calibration not saved: {unmoved}")
        mins[FULL_TURN_MOTOR], maxes[FULL_TURN_MOTOR] = 0, 4095

        leader.calibration = {
            m: MotorCalibration(
                id=leader.bus.motors[m].id,
                drive_mode=0,
                homing_offset=int(offsets[m]),
                range_min=int(mins[m]),
                range_max=int(maxes[m]),
            )
            for m in motors
        }
        leader.bus.write_calibration(leader.calibration)
        leader._save_calibration()
    finally:
        leader.bus.disconnect(disable_torque=False)

    print(f"calibration saved to {leader.calibration_fpath}")
    for m in motors:
        c = leader.calibration[m]
        print(f"  {m:<14} homing={c.homing_offset:>6}  range=[{c.range_min}, {c.range_max}]")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("step", choices=["home", "record"])
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", default="leader_L")
    parser.add_argument("--calibration-dir", type=Path, default=Path("dev_calibration"))
    parser.add_argument("--timeout", type=float, default=600.0, help="record: max seconds")
    args = parser.parse_args()

    args.calibration_dir.mkdir(parents=True, exist_ok=True)
    homing_path = args.calibration_dir / f"{args.id}.homing.json"
    stop_path = args.calibration_dir / f"{args.id}.stop"
    if args.step == "home":
        step_home(args, homing_path)
    else:
        step_record(args, homing_path, stop_path)


if __name__ == "__main__":
    main()
