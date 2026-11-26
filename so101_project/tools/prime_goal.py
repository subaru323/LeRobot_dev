"""Run right BEFORE lerobot-teleoperate / lerobot-record / lerobot-rollout (follower only, torque stays OFF).

usage: python tools\\prime_goal.py [PORT] [--speed 2000]

LeRobot turns follower torque on at connect without touching Goal_Position, so the arm jumps at full
speed to whatever goal is left in the motors. This writes Goal_Position = Present_Position, so at
connect the arm holds where it is. It also writes a speed cap (Goal_Velocity, steps/s; 0 = none).
Both live in motor RAM: they are lost when the 5V power is unplugged -> run this again after that.
Do not move the follower between this and starting the LeRobot command.
"""

import argparse

from so101_common import FOLLOWER_PORT, check_motors_ok, make_follower

p = argparse.ArgumentParser()
p.add_argument("port", nargs="?", default=FOLLOWER_PORT)
p.add_argument("--speed", type=int, default=2000, help="steps/s, 4096 = one turn; 0 = no limit")
args = p.parse_args()

bus = make_follower(args.port).bus
bus.connect()
try:
    bus.disable_torque(num_retry=3)
    check_motors_ok(bus, "follower")
    present = bus.sync_read("Present_Position", normalize=False, num_retry=3)
    for m, v in present.items():
        bus.write("Goal_Position", m, v, normalize=False)
        bus.write("Goal_Velocity", m, args.speed)
    # STS3215 can switch torque on by itself when Goal_Position is written -> switch it off again
    bus.disable_torque(num_retry=3)
    goal =bus.sync_read("Goal_Position", normalize=False, num_retry=3)
    speed = bus.sync_read("Goal_Velocity", normalize=False, num_retry=3)
    bad = [m for m in present if abs(goal[m] - present[m]) > 2]
    print("present :", present)
    print("goal    :", goal)
    print("speed   :", speed)
    torque = bus.sync_read("Torque_Enable", normalize=False)
    print("torque  :", torque)
    if any(torque.values()):
        bad.append("torque still on")
    print("OK: goal = present. Start the LeRobot command now, without moving the follower." if not bad else f"!! {bad}")
finally:
    bus.disconnect(disable_torque=False)
