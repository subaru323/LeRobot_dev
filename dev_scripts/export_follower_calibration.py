"""Save the calibration already stored in an SO-101 follower's motors to a LeRobot calibration file.

Use this when the arm was calibrated elsewhere (homing offsets and position limits are in the motors'
EEPROM) but the <id>.json file is missing on this machine. Nothing is written to the motors.

Example:
    python dev_scripts/export_follower_calibration.py --port COM20
"""

import argparse
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", default="follower_F")
    parser.add_argument("--calibration-dir", type=Path, default=Path("dev_calibration"))
    args = parser.parse_args()

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id, calibration_dir=args.calibration_dir))
    robot.bus.connect()
    try:
        robot.calibration = robot.bus.read_calibration()
        positions = robot.bus.sync_read("Present_Position", normalize=False)
    finally:
        robot.bus.disconnect(disable_torque=False)

    unset = [m for m, c in robot.calibration.items() if c.homing_offset == 0 and c.range_min == 0]
    if unset:
        print(f"warning: these motors look uncalibrated (offset 0, min 0): {unset}")
    robot._save_calibration()
    print(f"saved {robot.calibration_fpath}")
    for m, c in robot.calibration.items():
        print(f"  {m:<14} homing={c.homing_offset:>6}  range=[{c.range_min}, {c.range_max}]  pos={positions[m]}")


if __name__ == "__main__":
    main()
