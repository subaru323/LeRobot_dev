"""Record the range of one joint on both arms while it is turned by hand, logging every reading.

Torque is disabled on that joint only (leader and follower); other follower joints keep their state.
Positions are read at --hz, appended to a CSV, and min/max are tracked per side. Large jumps between
consecutive reads are logged as JUMP lines (possible encoder wrap or glitch). Stops on Ctrl+C or when
the stop file exists, then writes min/max and the linear leader->follower mapping to a JSON file.

Example:
    python dev_scripts/record_joint_range.py --leader-port COM19 --follower-port COM20 --id 5
    New-Item dev_logs/joint5.stop    # to finish from another shell
"""

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import scservo_sdk as scs

ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56


def open_bus(port, baudrate):
    ph = scs.PortHandler(port)
    if not ph.openPort() or not ph.setBaudRate(baudrate):
        sys.exit(f"Cannot open {port} at {baudrate} baud")
    return ph


def read_pos(ph, pk, id_):
    value, res, _ = pk.read2ByteTxRx(ph, id_, ADDR_PRESENT_POSITION)
    if res != scs.COMM_SUCCESS:
        return None
    return -(value & 0x7FFF) if value & 0x8000 else value


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--jump-threshold", type=int, default=300)
    parser.add_argument("--log-dir", type=Path, default=Path("dev_logs"))
    parser.add_argument("--out", type=Path, help="Result JSON (default: dev_calibration/joint<id>_range.json)")
    args = parser.parse_args()

    i = args.id
    args.log_dir.mkdir(parents=True, exist_ok=True)
    stop_path = args.log_dir / f"joint{i}.stop"
    stop_path.unlink(missing_ok=True)
    csv_path = args.log_dir / f"joint{i}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    out_path = args.out or Path("dev_calibration") / f"joint{i}_range.json"

    pk = scs.PacketHandler(0)
    lph = open_bus(args.leader_port, args.baudrate)
    fph = open_bus(args.follower_port, args.baudrate)
    for name, ph in (("L", lph), ("F", fph)):
        if pk.ping(ph, i)[1] != scs.COMM_SUCCESS:
            sys.exit(f"no response from {name}{i}")
        pk.write1ByteTxRx(ph, i, ADDR_TORQUE_ENABLE, 0)  # feedback only
    print(f"torque off on L{i} and F{i}; logging to {csv_path}")
    print(f"turn both joints through their full range; Ctrl+C or create {stop_path} to finish")

    mins, maxs, last = {}, {}, {}
    jumps = []
    t0 = time.perf_counter()
    last_print = 0.0
    period = 1.0 / args.hz
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t", f"L{i}", f"F{i}"])
        try:
            while not stop_path.exists():
                t = time.perf_counter()
                now = {"L": read_pos(lph, pk, i), "F": read_pos(fph, pk, i)}
                writer.writerow([f"{t - t0:.3f}", now["L"], now["F"]])
                for side, pos in now.items():
                    if pos is None:
                        continue
                    if side in last and abs(pos - last[side]) > args.jump_threshold:
                        msg = f"[JUMP {t - t0:7.2f}s] {side}{i}: {last[side]} -> {pos} ({pos - last[side]:+d})"
                        jumps.append(msg)
                        print("\n" + msg, flush=True)
                    last[side] = pos
                    mins[side] = min(mins.get(side, pos), pos)
                    maxs[side] = max(maxs.get(side, pos), pos)
                if t - last_print > 1.0 / args.print_hz:
                    last_print = t
                    f.flush()
                    print(
                        f"\rL{i}={now['L']!s:>5} [{mins.get('L')!s:>5}..{maxs.get('L')!s:>5}]   "
                        f"F{i}={now['F']!s:>5} [{mins.get('F')!s:>5}..{maxs.get('F')!s:>5}]",
                        end="",
                        flush=True,
                    )
                time.sleep(max(0.0, period - (time.perf_counter() - t)))
        except KeyboardInterrupt:
            lph.is_using = fph.is_using = False
        finally:
            lph.closePort()
            fph.closePort()
    stop_path.unlink(missing_ok=True)

    print()
    if "L" not in mins or "F" not in mins:
        sys.exit("no readings recorded")
    l_span, f_span = maxs["L"] - mins["L"], maxs["F"] - mins["F"]
    scale = f_span / l_span if l_span else None
    result = {
        "id": i,
        "leader": {"min": mins["L"], "max": maxs["L"]},
        "follower": {"min": mins["F"], "max": maxs["F"]},
        "follower_per_leader_tick": scale,
        "mapping": "F = F_min + (L - L_min) * follower_per_leader_tick",
        "jumps": jumps,
        "log": str(csv_path),
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    print(f"saved {out_path}")


if __name__ == "__main__":
    main()
