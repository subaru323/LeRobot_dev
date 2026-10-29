"""Make an SO-101 leader's calibration match an already calibrated follower.

With both arms held in the same pose, the leader's homing offsets are shifted so that its
Present_Position equals the follower's for every joint, and the follower's position ranges are copied
to the leader. Both arms then normalise to the same degrees, so `lerobot-teleoperate` maps L1->F1 ...
L6->F6 one to one. Nothing is written to the follower.

Feetech: Present_Position = Actual_Position - Homing_Offset, so
    new_leader_offset = old_leader_offset + (leader_present - follower_present)

Runs as a dry run unless --apply is given.

Example:
    python dev_scripts/match_leader_to_follower.py --leader-port COM19 --follower-port COM20
    python dev_scripts/match_leader_to_follower.py --leader-port COM19 --follower-port COM20 --apply
"""

import argparse
from pathlib import Path

from lerobot.motors import MotorCalibration
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

MAX_OFFSET = 2047  # Homing_Offset is 11-bit sign-magnitude on STS3215


def wrap_offset(value: int) -> int:
    """Bring an offset into [-2048, 2047] modulo one turn, then clamp to the encodable range."""
    value = (value + 2048) % 4096 - 2048
    return max(-MAX_OFFSET, min(MAX_OFFSET, value))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--leader-id", default="leader_L")
    parser.add_argument("--follower-id", default="follower_F")
    parser.add_argument("--calibration-dir", type=Path, default=Path("dev_calibration"))
    parser.add_argument("--apply", action="store_true", help="Write the new leader calibration")
    parser.add_argument(
        "--joints",
        nargs="+",
        help="Only re-align these joints (e.g. wrist_roll); the others keep their current leader calibration",
    )
    args = parser.parse_args()

    leader = SO101Leader(
        SO101LeaderConfig(port=args.leader_port, id=args.leader_id, calibration_dir=args.calibration_dir)
    )
    follower = SO101Follower(
        SO101FollowerConfig(port=args.follower_port, id=args.follower_id, calibration_dir=args.calibration_dir)
    )
    leader.bus.connect()
    follower.bus.connect()
    try:
        f_cal = follower.bus.read_calibration()
        l_cal = leader.bus.read_calibration()
        l_pos = leader.bus.sync_read("Present_Position", normalize=False, num_retry=5)
        f_pos = follower.bus.sync_read("Present_Position", normalize=False, num_retry=5)

        new_cal = {}
        print(f"{'joint':<14} {'L pos':>6} {'F pos':>6} {'diff':>6} {'L offset':>9} -> {'new':>6}  range(from F)")
        unknown = set(args.joints or []) - set(leader.bus.motors)
        if unknown:
            raise SystemExit(f"unknown joints {sorted(unknown)}; choose from {list(leader.bus.motors)}")
        for motor, m in leader.bus.motors.items():
            diff = l_pos[motor] - f_pos[motor] if not args.joints or motor in args.joints else 0
            new_offset = wrap_offset(l_cal[motor].homing_offset + diff)
            new_cal[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=new_offset,
                range_min=f_cal[motor].range_min,
                range_max=f_cal[motor].range_max,
            )
            print(
                f"{motor:<14} {l_pos[motor]:>6} {f_pos[motor]:>6} {diff:>+6} {l_cal[motor].homing_offset:>9} -> "
                f"{new_offset:>6}  [{f_cal[motor].range_min}, {f_cal[motor].range_max}]"
            )

        if not args.apply:
            print("dry run: nothing written. Re-run with --apply while both arms stay in the same pose.")
            return

        leader.bus.disable_torque()
        leader.bus.write_calibration(new_cal)
        leader.calibration = new_cal
        leader._save_calibration()
        follower.calibration = f_cal
        follower._save_calibration()

        l_after = leader.bus.sync_read("Present_Position", normalize=False, num_retry=5)
        f_after = follower.bus.sync_read("Present_Position", normalize=False, num_retry=5)
        print(f"saved {leader.calibration_fpath} and {follower.calibration_fpath}")
        print("residual (leader - follower) after write:")
        for motor in leader.bus.motors:
            print(f"  {motor:<14} {l_after[motor] - f_after[motor]:+d}")
    finally:
        leader.bus.disconnect(disable_torque=False)
        follower.bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
