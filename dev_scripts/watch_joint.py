"""Disable torque on servos and continuously print their Present_Position on one line (feedback only).

Each target is PORT:ID; several targets on different buses are shown side by side.

Example:
    python dev_scripts/watch_joint.py COM20:5
    python dev_scripts/watch_joint.py COM19:5 COM20:5
"""

import argparse
import sys
import time

import scservo_sdk as scs

ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("targets", nargs="+", help="PORT:ID, e.g. COM19:5")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--hz", type=float, default=10.0)
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
    try:
        for port, i in targets:
            res, _ = pk.write1ByteTxRx(ports[port], i, ADDR_TORQUE_ENABLE, 0)
            torque, _, _ = pk.read1ByteTxRx(ports[port], i, ADDR_TORQUE_ENABLE)
            status = "ok" if res == scs.COMM_SUCCESS else pk.getTxRxResult(res)
            print(f"{port} id {i}: torque off {status} (torque={torque})")
        print("Ctrl+C to stop")
        while True:
            cells = []
            for port, i in targets:
                pos, res, _ = pk.read2ByteTxRx(ports[port], i, ADDR_PRESENT_POSITION)
                value = f"{pos:>5} ({pos * 360 / 4096:6.1f} deg)" if res == scs.COMM_SUCCESS else "read failed"
                cells.append(f"{port}:{i} {value}")
            print("\r" + "   ".join(cells) + "   ", end="", flush=True)
            time.sleep(1 / args.hz)
    except KeyboardInterrupt:
        print()
    finally:
        for ph in ports.values():
            ph.is_using = False
            ph.closePort()


if __name__ == "__main__":
    main()
