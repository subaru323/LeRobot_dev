"""Change the ID of a Feetech STS servo, for building a daisy chain one motor at a time.

New STS3215 servos all ship with ID 1. Add one new servo to the end of the chain, then run this
script to move it from --from-id (default 1) to --new-id. Because every previously added servo
already has a unique ID, only the new servo answers to ID 1.

Example (SO-101 leader, gripper first):
    python dev_scripts/assign_servo_id.py --port COM19 --new-id 6   # only L6 connected
    python dev_scripts/assign_servo_id.py --port COM19 --new-id 5   # L6 + L5 chained
    ...
    # L1 is added last and keeps ID 1
"""

import argparse
import sys
import time

import scservo_sdk as scs

ADDR_ID = 5
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56


def ping_ok(ph, pk, id_, tries=3):
    """True only if every attempt returns a clean reply (catches colliding duplicate IDs)."""
    for _ in range(tries):
        _, res, _ = pk.ping(ph, id_)
        if res != scs.COMM_SUCCESS:
            return False
        time.sleep(0.01)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--from-id", type=int, default=1)
    parser.add_argument("--new-id", type=int, required=True)
    parser.add_argument("--max-id", type=int, default=20, help="Range scanned to report the bus state")
    args = parser.parse_args()

    if not 0 <= args.new_id <= 253 or args.new_id == args.from_id:
        sys.exit("--new-id must be in 0..253 and differ from --from-id")

    ph = scs.PortHandler(args.port)
    pk = scs.PacketHandler(0)
    if not ph.openPort() or not ph.setBaudRate(args.baudrate):
        sys.exit(f"Cannot open {args.port} at {args.baudrate} baud")

    try:
        present = [i for i in range(args.max_id + 1) if pk.ping(ph, i)[1] == scs.COMM_SUCCESS]
        print(f"ids on bus before: {present}")
        if args.new_id in present:
            sys.exit(f"id {args.new_id} is already on the bus; nothing changed")
        if not ping_ok(ph, pk, args.from_id):
            sys.exit(
                f"id {args.from_id} did not answer cleanly. Either no servo has that id, or several do "
                "(colliding replies). Make sure exactly one servo with that id is connected."
            )

        res, err = pk.write1ByteTxRx(ph, args.from_id, ADDR_LOCK, 0)
        if res != scs.COMM_SUCCESS:
            sys.exit(f"unlock failed: {pk.getTxRxResult(res)}")
        pk.write1ByteTxRx(ph, args.from_id, ADDR_ID, args.new_id)  # reply comes from the new id
        time.sleep(0.05)
        res, err = pk.write1ByteTxRx(ph, args.new_id, ADDR_LOCK, 1)
        if res != scs.COMM_SUCCESS:
            sys.exit(f"lock on new id failed: {pk.getTxRxResult(res)}")

        if not ping_ok(ph, pk, args.new_id):
            sys.exit(f"id {args.new_id} does not answer after the change")
        pos, _, _ = pk.read2ByteTxRx(ph, args.new_id, ADDR_PRESENT_POSITION)
        present = [i for i in range(args.max_id + 1) if pk.ping(ph, i)[1] == scs.COMM_SUCCESS]
        print(f"id {args.from_id} -> {args.new_id} done (position={pos})")
        print(f"ids on bus after: {present}")
    finally:
        ph.closePort()


if __name__ == "__main__":
    main()
