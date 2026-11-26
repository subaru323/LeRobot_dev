"""Check that all 6 motors of one arm answer (no motion, torque is not touched).

usage: python tools\\scan_arm.py COM11
"""

import sys

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from so101_common import MOTORS

port = sys.argv[1]
bus = FeetechMotorsBus(port, {n: Motor(i + 1, "sts3215", MotorNormMode.RANGE_M100_100) for i, n in enumerate(MOTORS)})
bus.connect(handshake=False)
bus.set_baudrate(1_000_000)
print("broadcast ping (id: model, 777 = STS3215):", bus.broadcast_ping())
ok = True
for i, n in enumerate(MOTORS):
    found = bus.ping(i + 1) is not None
    ok &= found
    print(f"  id={i + 1} {n:<14} {'OK' if found else 'NO RESPONSE'}")
try:
    print("raw positions:", bus.sync_read("Present_Position", normalize=False))
except Exception as e:  # noqa: BLE001
    print("sync_read failed:", e)
bus.disconnect(disable_torque=False)
print("ALL 6 OK" if ok else "SOME MOTORS MISSING -> check 5V power and the 3-pin cables")
