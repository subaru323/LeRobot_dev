"""Re-home one joint on both arms so its recorded MIN..MAX range no longer crosses the 4095/0 seam.

Input is the range file written while recording (e.g. dev_calibration/joint5_maxmin.json), holding raw
MIN/MAX for leader (L<id>) and follower (F<id>), where moving MIN -> MAX may pass through the seam.
For each arm the homing offset is shifted so the middle of the range reads 2048
(Feetech: Present_Position = Actual_Position - Homing_Offset). The follower's position limits are set
to the new MIN..MAX so the motor cannot be driven through the gap outside its range. The physical
poses are unchanged; only the numbers move. The range file and the LeRobot calibration files are
updated with the new values.

Runs as a dry run unless --apply is given.

Example:
    python dev_scripts/center_joint_range.py --leader-port COM19 --follower-port COM20 \
        --range-file dev_calibration/joint5_maxmin.json --apply
"""

import argparse
import json
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

TURN = 4096
MAX_OFFSET = 2047  # Homing_Offset is 11-bit sign-magnitude on STS3215


def wrap_offset(value: int) -> int:
    value = (value + 2048) % TURN - 2048
    if abs(value) > MAX_OFFSET:
        raise SystemExit(f"homing offset {value} is outside +/-{MAX_OFFSET}")
    return value


def plan(raw_min: int, raw_max: int) -> dict:
    span = (raw_max - raw_min) % TURN
    mid = (raw_min + span // 2) % TURN
    shift = 2048 - mid  # added to every reading
    return {
        "span": span,
        "shift": shift,
        "new_min": (raw_min + shift) % TURN,
        "new_max": (raw_max + shift) % TURN,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--range-file", type=Path, required=True)
    parser.add_argument("--leader-id", default="leader_L")
    parser.add_argument("--follower-id", default="follower_F")
    parser.add_argument("--calibration-dir", type=Path, default=Path("dev_calibration"))
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()

    rng = json.loads(args.range_file.read_text(encoding="utf-8-sig"))
    if rng.get("centered"):
        raise SystemExit(f"{args.range_file} is already centered; record a new range first")
    i = int(rng["id"])
    leader = SO101Leader(
        SO101LeaderConfig(port=args.leader_port, id=args.leader_id, calibration_dir=args.calibration_dir)
    )
    follower = SO101Follower(
        SO101FollowerConfig(port=args.follower_port, id=args.follower_id, calibration_dir=args.calibration_dir)
    )
    motor = next(name for name, m in leader.bus.motors.items() if m.id == i)

    plans = {
        "leader": plan(rng["min"][f"L{i}"], rng["max"][f"L{i}"]),
        "follower": plan(rng["min"][f"F{i}"], rng["max"][f"F{i}"]),
    }

    leader.bus.connect()
    follower.bus.connect()
    try:
        arms = {"leader": leader, "follower": follower}
        before, offsets = {}, {}
        for side, arm in arms.items():
            p = plans[side]
            old = arm.bus.read("Homing_Offset", motor, normalize=False)
            offsets[side] = (old, wrap_offset(old - p["shift"]))
            before[side] = arm.bus.read("Present_Position", motor, normalize=False)
            print(
                f"{side:<8} {motor}: range {p['span']} ticks, shift {p['shift']:+d} -> "
                f"MIN..MAX {p['new_min']}..{p['new_max']}, homing {old} -> {offsets[side][1]}, "
                f"now {before[side]} -> {(before[side] + p['shift']) % TURN}"
            )
        if not args.apply:
            print("dry run: nothing written. Re-run with --apply.")
            return

        for side, arm in arms.items():
            p = plans[side]
            arm.bus.write("Torque_Enable", motor, 0)
            arm.bus.write("Homing_Offset", motor, offsets[side][1])
            if side == "follower":
                arm.bus.write("Min_Position_Limit", motor, p["new_min"])
                arm.bus.write("Max_Position_Limit", motor, p["new_max"])
            after = arm.bus.read("Present_Position", motor, normalize=False)
            expected = (before[side] + p["shift"]) % TURN
            print(f"{side:<8} {motor}: position {before[side]} -> {after} (expected ~{expected})")

        # keep LeRobot calibration files in sync with the motors
        for side, arm in arms.items():
            if not arm.calibration_fpath.is_file():
                continue
            cal = json.loads(arm.calibration_fpath.read_text())
            cal[motor]["homing_offset"] = offsets[side][1]
            if side == "follower":
                cal[motor]["range_min"] = plans[side]["new_min"]
                cal[motor]["range_max"] = plans[side]["new_max"]
            arm.calibration_fpath.write_text(json.dumps(cal, indent=4))

        rng["raw_before_centering"] = {"min": rng["min"], "max": rng["max"]}
        rng["min"] = {f"L{i}": plans["leader"]["new_min"], f"F{i}": plans["follower"]["new_min"]}
        rng["max"] = {f"L{i}": plans["leader"]["new_max"], f"F{i}": plans["follower"]["new_max"]}
        rng["centered"] = True
        rng["direction"] = "MIN < MAX, no seam inside the range"
        rng["homing_offset"] = {f"L{i}": offsets["leader"][1], f"F{i}": offsets["follower"][1]}
        args.range_file.write_text(json.dumps(rng, indent=2))
        print(f"updated {args.range_file}")
    finally:
        leader.bus.disconnect(disable_torque=False)
        follower.bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
