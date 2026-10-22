"""Stream the joint positions of an SO-101 leader arm through LeRobot's SO101Leader.

Torque stays off, so the arm can be moved by hand while positions are printed.

Modes:
    --raw   raw encoder ticks (0..4095), no calibration needed
    default calibrated values (degrees, gripper 0..100) from the calibration file

Calibrate first (stores <calibration-dir>/<id>.json):
    lerobot-calibrate --teleop.type=so101_leader --teleop.port=COM19 \
        --teleop.id=leader_L --teleop.calibration_dir=dev_calibration

Example:
    python dev_scripts/read_leader.py --port COM19 --raw
    python dev_scripts/read_leader.py --port COM19
"""

import argparse
import sys
import time
from pathlib import Path

from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", default="leader_L", help="Calibration id (file name without .json)")
    parser.add_argument("--calibration-dir", type=Path, default=Path("dev_calibration"))
    parser.add_argument("--raw", action="store_true", help="Print raw ticks instead of calibrated values")
    parser.add_argument("--hz", type=float, default=10.0)
    args = parser.parse_args()

    config = SO101LeaderConfig(port=args.port, id=args.id, calibration_dir=args.calibration_dir)
    leader = SO101Leader(config)
    motors = list(leader.bus.motors)

    if args.raw:
        leader.bus.connect()
        leader.bus.disable_torque()
        read = lambda: leader.bus.sync_read("Present_Position", normalize=False)  # noqa: E731
        unit = "ticks"
    else:
        if not leader.calibration:
            sys.exit(f"No calibration file at {leader.calibration_fpath}. Run lerobot-calibrate first (see --help).")
        leader.connect(calibrate=False)
        if not leader.is_calibrated:
            leader.bus.write_calibration(leader.calibration)
        read = lambda: {k.removesuffix(".pos"): v for k, v in leader.get_action().items()}  # noqa: E731
        unit = "deg (gripper 0-100)"

    print(f"{leader} on {args.port} [{unit}] - Ctrl+C to stop")
    print("  ".join(f"{m:>13}" for m in motors))
    try:
        while True:
            t = time.perf_counter()
            values = read()
            line = "  ".join(f"{values[m]:>13.1f}" if not args.raw else f"{values[m]:>13d}" for m in motors)
            print(f"\r{line}", end="", flush=True)
            time.sleep(max(0.0, 1 / args.hz - (time.perf_counter() - t)))
    except KeyboardInterrupt:
        print()
    finally:
        leader.bus.port_handler.is_using = False  # Ctrl+C mid-transaction leaves the SDK's busy flag set
        leader.bus.disconnect(disable_torque=False) if args.raw else leader.disconnect()


if __name__ == "__main__":
    main()
