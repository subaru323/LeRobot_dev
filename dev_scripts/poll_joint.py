"""Poll one servo's Present_Position as fast as the bus allows (torque off) and print every sample.

Each line: elapsed ms, interval since the previous read in ms, position, change since the previous read.
Failed reads and large changes between consecutive reads are flagged. Every sample is also written to
a CSV under dev_logs/. The real read rate is limited by the serial round trip (about 1 ms or more per
read on Windows), so --interval below that has no further effect.

Example:
    python dev_scripts/poll_joint.py --port COM20 --id 5
"""

import argparse
import csv
import sys
import time
from pathlib import Path

import scservo_sdk as scs

ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--interval", type=float, default=0.0001, help="Target seconds between reads")
    parser.add_argument("--jump", type=int, default=50, help="Flag changes larger than this between reads")
    parser.add_argument("--log-dir", type=Path, default=Path("dev_logs"))
    args = parser.parse_args()

    ph = scs.PortHandler(args.port)
    pk = scs.PacketHandler(0)
    if not ph.openPort() or not ph.setBaudRate(args.baudrate):
        sys.exit(f"Cannot open {args.port} at {args.baudrate} baud")
    args.log_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.log_dir / f"poll_id{args.id}_{time.strftime('%Y%m%d_%H%M%S')}.csv"

    res, _ = pk.write1ByteTxRx(ph, args.id, ADDR_TORQUE_ENABLE, 0)
    torque, _, _ = pk.read1ByteTxRx(ph, args.id, ADDR_TORQUE_ENABLE)
    print(f"id {args.id}: torque off {'ok' if res == scs.COMM_SUCCESS else pk.getTxRxResult(res)} (torque={torque})")
    print(f"logging every sample to {csv_path}; Ctrl+C to stop")
    print(f"{'t_ms':>10} {'dt_ms':>7} {'pos':>5} {'delta':>6}")

    n = fails = flagged = 0
    last_pos = None
    t0 = time.perf_counter()
    t_prev = t0
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_ms", "dt_ms", "pos", "ok"])
        try:
            while True:
                t_start = time.perf_counter()
                pos, r, _ = pk.read2ByteTxRx(ph, args.id, ADDR_PRESENT_POSITION)
                now = time.perf_counter()
                t_ms, dt_ms = (now - t0) * 1e3, (now - t_prev) * 1e3
                t_prev = now
                n += 1
                if r != scs.COMM_SUCCESS:
                    fails += 1
                    writer.writerow([f"{t_ms:.3f}", f"{dt_ms:.3f}", "", 0])
                    print(f"{t_ms:10.1f} {dt_ms:7.2f}  FAIL {pk.getTxRxResult(r)}")
                else:
                    delta = pos - last_pos if last_pos is not None else 0
                    mark = ""
                    if last_pos is not None and abs(delta) > args.jump:
                        flagged += 1
                        mark = "  <== JUMP"
                    writer.writerow([f"{t_ms:.3f}", f"{dt_ms:.3f}", pos, 1])
                    print(f"{t_ms:10.1f} {dt_ms:7.2f} {pos:5d} {delta:+6d}{mark}")
                    last_pos = pos
                remaining = args.interval - (time.perf_counter() - t_start)
                if remaining > 0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            pass
        finally:
            ph.is_using = False
            ph.closePort()
    elapsed = time.perf_counter() - t0
    print(f"\n{n} reads in {elapsed:.1f} s ({n / elapsed:.0f} reads/s, avg {elapsed / max(n, 1) * 1e3:.2f} ms), "
          f"{fails} failed, {flagged} jumps > {args.jump}")


if __name__ == "__main__":
    main()
