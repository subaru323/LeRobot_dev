"""Read or switch the follower arm's torque (Feetech STS bus).

Prints one JSON object to stdout:
    {"ok": bool, "torque": {"1": true, ...}, "errors": {"5": "..."}}

"on" first sets every Goal_Position to the Present_Position, so enabling torque
holds the current pose instead of driving to a stale goal.

Run from easy_lerobot/:
    python -m core.follower_torque status --port COM20
    python -m core.follower_torque on --port COM20
    python -m core.follower_torque off --port COM20
"""

import argparse
import json
import sys

import scservo_sdk as scs
import serial

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=["status", "on", "off"])
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    args = parser.parse_args()

    ph = scs.PortHandler(args.port)
    pk = scs.PacketHandler(0)
    try:
        opened = ph.openPort() and ph.setBaudRate(args.baudrate)
    except serial.SerialException as e:
        opened = False
        reason = str(e)
    else:
        reason = "open failed"
    if not opened:
        print(json.dumps({"ok": False, "torque": {}, "errors": {"port": f"cannot open {args.port}: {reason}"}}))
        return 1

    errors = {}
    try:
        if args.action == "on":
            for i in args.ids:
                pos, res, _ = pk.read2ByteTxRx(ph, i, ADDR_PRESENT_POSITION)
                if res != scs.COMM_SUCCESS:
                    errors[str(i)] = f"read position: {pk.getTxRxResult(res)}"
                    continue
                res, _ = pk.write2ByteTxRx(ph, i, ADDR_GOAL_POSITION, pos)
                if res != scs.COMM_SUCCESS:
                    errors[str(i)] = f"write goal: {pk.getTxRxResult(res)}"
            for i in args.ids:
                if str(i) in errors:
                    continue  # never enable a motor whose goal could not be aligned
                res, _ = pk.write1ByteTxRx(ph, i, ADDR_TORQUE_ENABLE, 1)
                if res != scs.COMM_SUCCESS:
                    errors[str(i)] = f"enable torque: {pk.getTxRxResult(res)}"
        elif args.action == "off":
            for i in args.ids:
                res, _ = pk.write1ByteTxRx(ph, i, ADDR_TORQUE_ENABLE, 0)
                if res != scs.COMM_SUCCESS:
                    errors[str(i)] = f"disable torque: {pk.getTxRxResult(res)}"

        torque = {}
        for i in args.ids:
            value, res, _ = pk.read1ByteTxRx(ph, i, ADDR_TORQUE_ENABLE)
            if res == scs.COMM_SUCCESS:
                torque[str(i)] = bool(value)
            else:
                errors.setdefault(str(i), f"read torque: {pk.getTxRxResult(res)}")
    finally:
        ph.closePort()

    print(json.dumps({"ok": not errors, "torque": torque, "errors": errors}))
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
