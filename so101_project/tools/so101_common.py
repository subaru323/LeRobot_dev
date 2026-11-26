"""Shared helpers for the SO-101 tools in this folder.

Ports and ids for this PC (see docs/組み立て手順書.md, STEP 2).
"""

import time

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
from lerobot.teleoperators.so_leader import SOLeader, SOLeaderTeleopConfig

FOLLOWER_PORT = "COM11"
LEADER_PORT = "COM10"
FOLLOWER_ID = "follower1"
LEADER_ID = "leader1"

MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def make_follower(port: str = FOLLOWER_PORT) -> SOFollower:
    """Follower object. Connect with `connect_bus_no_torque(robot.bus)`, never `robot.connect()`:
    `connect()` turns torque on and the arm jumps to whatever Goal_Position is left in the motors."""
    return SOFollower(SOFollowerRobotConfig(port=port, id=FOLLOWER_ID))


def make_leader(port: str = LEADER_PORT) -> SOLeader:
    return SOLeader(SOLeaderTeleopConfig(port=port, id=LEADER_ID))


def connect_bus_no_torque(bus: FeetechMotorsBus) -> None:
    """Connect and apply LeRobot's motor setup (response delay, position mode), torque stays OFF."""
    bus.connect()
    bus.disable_torque(num_retry=3)
    bus.configure_motors()
    for m in bus.motors:
        bus.write("Operating_Mode", m, 0)  # 0 = position mode


def check_motors_ok(bus: FeetechMotorsBus, what: str) -> None:
    if not bus.calibration:
        raise SystemExit(f"{what}: no calibration file -> run lerobot-calibrate first")
    if not bus.is_calibrated:
        raise SystemExit(f"{what}: calibration in the motors differs from the file -> run lerobot-calibrate")
    status = {m: bus.read("Status", m, normalize=False, num_retry=2) for m in bus.motors}
    if any(status.values()):
        raise SystemExit(f"{what}: motor error status {status} -> unplug the 5V power, plug it back, retry")


def limit_follower(bus: FeetechMotorsBus, speed: int, accel: int, torque_limit: int) -> None:
    """Speed (steps/s, 0 = no limit), acceleration (x100 steps/s^2) and torque (of 1000) limits, in RAM."""
    for m in bus.motors:
        bus.write("Acceleration", m, accel)
        bus.write("Goal_Velocity", m, speed)
        # gripper: at most 50 %, like LeRobot, so squeezing an object does not burn the motor
        bus.write("Torque_Limit", m, min(torque_limit, 500) if m == "gripper" else torque_limit)


def torque_on_in_place(bus: FeetechMotorsBus) -> dict[str, int]:
    """Set Goal_Position = Present_Position, then enable torque, so nothing jumps."""
    for m in bus.motors:  # same position-mode gains as LeRobot's SOFollower.configure()
        bus.write("P_Coefficient", m, 16)
        bus.write("I_Coefficient", m, 0)
        bus.write("D_Coefficient", m, 32)
    present = bus.sync_read("Present_Position", normalize=False, num_retry=3)
    for m, v in present.items():
        bus.write("Goal_Position", m, v, normalize=False)
    bus.enable_torque(num_retry=3)
    return present


def max_load(bus: FeetechMotorsBus) -> tuple[str, int]:
    loads = {m: abs(bus.read("Present_Load", m, normalize=False, num_retry=2)) for m in bus.motors}
    worst = max(loads, key=loads.get)
    return worst, loads[worst]


def release_all(bus: FeetechMotorsBus) -> None:
    """Torque off motor by motor, so one failing motor does not keep the others stiff."""
    for m in bus.motors:
        err = None
        for _ in range(3):
            try:
                bus.write("Torque_Enable", m, 0, num_retry=2)
                err = None
                break
            except Exception as e:  # noqa: BLE001
                err = e
                time.sleep(0.05)
        if err is not None:
            print(f"!! could not release torque on {m}: {err}  -> unplug the 5V power", flush=True)


def safe_limits_deg(bus: FeetechMotorsBus, margin_deg: float = 5.0) -> dict[str, tuple[float, float]]:
    """Calibrated range of each joint in the normalized units (deg, gripper 0-100), with a margin."""
    out = {}
    for m, c in bus.calibration.items():
        if m == "gripper":
            out[m] = (2.0, 98.0)
            continue
        if m == "wrist_roll":
            out[m] = (-180.0, 180.0)
            continue
        half = (c.range_max - c.range_min) / 2 * 360 / 4095
        out[m] = (-half + margin_deg, half - margin_deg)
    return out
