"""Show leader vs follower joint angles live, to line the two arms up before teleoperation.

usage: python tools\\compare_pose.py [--once]

Neither arm gets torque (read only). Rows marked <<< differ by more than 10 deg (gripper: 10 %).
Ctrl+C to stop.
"""

import sys
import time

from so101_common import MOTORS, connect_bus_no_torque, make_follower, make_leader

THRESH = 10.0
once = "--once" in sys.argv

follower = make_follower()
leader = make_leader()
for name, arm in (("follower", follower), ("leader", leader)):
    if not arm.bus.calibration:
        raise SystemExit(f"{name}: no calibration file -> run lerobot-calibrate first")
connect_bus_no_torque(follower.bus)
connect_bus_no_torque(leader.bus)

try:
    while True:
        f = follower.bus.sync_read("Present_Position", num_retry=3)
        l = leader.bus.sync_read("Present_Position", num_retry=3)
        lines = [f"{'joint':<14} {'leader':>8} {'follower':>9} {'diff':>7}"]
        worst = 0.0
        for m in MOTORS:
            d = l[m] - f[m]
            worst = max(worst, abs(d))
            mark = "  <<<" if abs(d) > THRESH else ""
            lines.append(f"{m:<14} {l[m]:>8.1f} {f[m]:>9.1f} {d:>7.1f}{mark}")
        lines.append(f"max diff {worst:.1f}  ->  {'OK to start' if worst <= THRESH else 'move the LEADER closer'}")
        print("\n".join(lines), flush=True)
        if once:
            break
        time.sleep(0.3)
        print(f"\x1b[{len(lines)}A", end="")
except KeyboardInterrupt:
    pass
finally:
    follower.bus.disconnect(disable_torque=False)
    leader.bus.disconnect(disable_torque=False)
