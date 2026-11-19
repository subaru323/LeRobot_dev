"""Poll servos' Present_Position as fast as the buses allow (torque off) and print every sample.

Each target is PORT:ID. Each line: elapsed ms, interval since the previous cycle in ms, then per target
the position and its change since the previous read. Failed reads and large changes between
consecutive reads are flagged. Every sample is also written to a CSV under dev_logs/. The real rate is
limited by the serial round trip (about 0.4-0.5 ms per read here), so --interval below that has no
further effect.

Example:
    python dev_scripts/poll_joint.py COM20:5
    python dev_scripts/poll_joint.py COM19:5 COM20:5 --interval 0.0003
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
    parser.add_argument("targets", nargs="+", help="PORT:ID, e.g. COM20:5")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--interval", type=float, default=0.0001, help="Target seconds between cycles")
    parser.add_argument("--jump", type=int, default=50, help="Flag changes larger than this between reads")
    parser.add_argument("--log-dir", type=Path, default=Path("dev_logs"))
    args = parser.parse_args()

    pk = scs.PacketHandler(0)
    ports, targets = {}, []
    for t in args.targets:
        port, id_ = t.split(":")
        if port not in ports:
            ph = scs.PortHandler(port)
            if not ph.openPort() or not ph.setBaudRate(args.baudrate):
                sys.exit(f"Cannot open {port} at {args.baudrate} baud")
            ports[port] = ph
        targets.append((port, int(id_)))
    names = [f"{p}:{i}" for p, i in targets]

    args.log_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.log_dir / f"poll_{'_'.join(n.replace(':', '-') for n in names)}_{time.strftime('%Y%m%d_%H%M%S')}.csv"

    for port, i in targets:
        res, _ = pk.write1ByteTxRx(ports[port], i, ADDR_TORQUE_ENABLE, 0)
        torque, _, _ = pk.read1ByteTxRx(ports[port], i, ADDR_TORQUE_ENABLE)
        print(f"{port}:{i} torque off {'ok' if res == scs.COMM_SUCCESS else pk.getTxRxResult(res)} (torque={torque})")
    print(f"logging every sample to {csv_path}; Ctrl+C to stop")
    print(f"{'t_ms':>10} {'dt_ms':>6}  " + "  ".join(f"{n:>17}" for n in names))

    n = fails = flagged = 0
    last = {}
    t0 = time.perf_counter()
    t_prev = t0
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_ms", "dt_ms"] + names)
        try:
            while True:
                t_start = time.perf_counter()
                values, cells, mark = [], [], ""
                for (port, i), name in zip(targets, names):
                    pos, r, _ = pk.read2ByteTxRx(ports[port], i, ADDR_PRESENT_POSITION)
                    if r != scs.COMM_SUCCESS:
                        fails += 1
                        values.append("")
                        cells.append(f"{'FAIL':>17}")
                        continue
                    delta = pos - last[name] if name in last else 0
                    if name in last and abs(delta) > args.jump:
                        flagged += 1
                        mark = "  <== JUMP"
                    last[name] = pos
                    values.append(pos)
                    cells.append(f"{pos:>10d} ({delta:+5d})")
                now = time.perf_counter()
                t_ms, dt_ms = (now - t0) * 1e3, (now - t_prev) * 1e3
                t_prev = now
                n += 1
                writer.writerow([f"{t_ms:.3f}", f"{dt_ms:.3f}"] + values)
                print(f"{t_ms:10.1f} {dt_ms:6.2f}  " + "  ".join(cells) + mark)
                remaining = args.interval - (time.perf_counter() - t_start)
                if remaining > 0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            pass
        finally:
            for ph in ports.values():
                ph.is_using = False
                ph.closePort()
    elapsed = time.perf_counter() - t0
    print(f"\n{n} cycles in {elapsed:.1f} s ({n / elapsed:.0f}/s, avg {elapsed / max(n, 1) * 1e3:.2f} ms), "
          f"{fails} failed reads, {flagged} jumps > {args.jump}")


if __name__ == "__main__":
    main()
