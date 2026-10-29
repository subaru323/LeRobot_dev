"""Set every follower motor's Goal_Position to its Present_Position (torque stays off).

Feetech motors keep the last Goal_Position. When torque is re-enabled (e.g. by `lerobot-teleoperate`
at connect time) they drive straight to that stale goal, which can swing a joint into an obstacle
and trigger an overload error. Run this right before teleoperation.

Example:
    python dev_scripts/hold_follower_pose.py --port COM20
"""

import argparse

import scservo_sdk as scs

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    args = parser.parse_args()

    ph = scs.PortHandler(args.port)
    pk = scs.PacketHandler(0)
    if not ph.openPort() or not ph.setBaudRate(args.baudrate):
        raise SystemExit(f"Cannot open {args.port} at {args.baudrate} baud")
    try:
        for i in args.ids:
            pk.write1ByteTxRx(ph, i, ADDR_TORQUE_ENABLE, 0)  # also clears a latched overload
            pos, res, _ = pk.read2ByteTxRx(ph, i, ADDR_PRESENT_POSITION)
            if res != scs.COMM_SUCCESS:
                print(f"id {i}: read failed ({pk.getTxRxResult(res)})")
                continue
            old, _, _ = pk.read2ByteTxRx(ph, i, ADDR_GOAL_POSITION)
            res, _ = pk.write2ByteTxRx(ph, i, ADDR_GOAL_POSITION, pos)
            status = "ok" if res == scs.COMM_SUCCESS else pk.getTxRxResult(res)
            print(f"id {i}: goal {old} -> {pos} ({status})")
    finally:
        ph.closePort()


if __name__ == "__main__":
    main()
