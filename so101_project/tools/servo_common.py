"""Shared helpers for the single-servo tools (servo_play.py, servo_gui.py, servo_mirror.py).

For LOOSE STS3215 motors on a bus servo board (not built into an arm). Raw units only, no calibration:
  position : 0..4095 steps = one turn (2048 = middle, 4096 steps = 360 deg)
  speed    : steps/s (Goal_Velocity). 0 = NO limit (full speed) -> the tools never send 0 in position mode
  accel    : Acceleration register, 1 unit = 100 steps/s^2, 0 = no limit -> the tools never send 0
  load     : Present_Load, -1000..1000 (sign = direction)
  voltage  : Present_Voltage / 10 = volts;  temperature : deg C
Register names and addresses: MAIN_PG/lerobot/src/lerobot/motors/feetech/tables.py (STS_SMS_SERIES_CONTROL_TABLE).

Safety rules (same idea as P-001 in docs/rules/RULES.md):
  - torque is switched on only after Goal_Position = Present_Position (nothing jumps)
  - every motion gets a speed and acceleration cap first
  - Operating_Mode is in EEPROM and LeRobot unlocks EEPROM when it disables torque, so a wheel-mode
    setting would SURVIVE a power cycle. Every tool puts the motor back to position mode (0) on exit.
  - motion commands refuse to run when more than 2 motors answer (that is probably an arm).
"""

import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

BAUD = 1_000_000  # LeRobot's default; lerobot-setup-motors and servo_play.py setid write this
POSITION_MODE, WHEEL_MODE = 0, 1
MAX_MOTORS_FOR_PLAY = 2


def name_of(motor_id: int) -> str:
    return f"id{motor_id}"


def open_bus(port: str, ids: list[int]) -> FeetechMotorsBus:
    bus = FeetechMotorsBus(port, {name_of(i): Motor(i, "sts3215", MotorNormMode.RANGE_M100_100) for i in ids})
    bus.connect(handshake=False)  # handshake would insist that every listed id answers
    bus.set_baudrate(BAUD)
    return bus


def found_ids(bus: FeetechMotorsBus) -> dict[int, int] | None:
    """{id: model number} of every motor that answers (777 = STS3215).

    None = the broadcast ping got no clean reply. That is NOT "no motors": LeRobot returns None on a
    timeout or a garbled reply, e.g. two motors with the same id answering at once (feetech.py
    broadcast_ping). Callers must treat None as "unknown" and refuse to move anything.
    """
    return bus.broadcast_ping(num_retry=2)


def ping_each(bus: FeetechMotorsBus, ids: range = range(1, 11)) -> list[int]:
    """Ping ids one by one (slower, but one garbled reply does not hide the others)."""
    return [i for i in ids if bus.ping(i, num_retry=1) is not None]


def refuse_if_arm(bus: FeetechMotorsBus) -> dict[int, int]:
    ids = found_ids(bus)
    if ids is None:
        bus.disconnect(disable_torque=False)
        raise SystemExit("the broadcast ping got no clean reply (two motors with the same id? loose cable?). "
                         "Nothing was moved. Check with: python tools\\servo_play.py <PORT> scan")
    if len(ids) > MAX_MOTORS_FOR_PLAY or len(ping_each(bus)) > MAX_MOTORS_FOR_PLAY:
        bus.disconnect(disable_torque=False)
        raise SystemExit(
            f"more than {MAX_MOTORS_FOR_PLAY} motors answer: this looks like an arm. These tools are for 1-2 "
            "LOOSE motors. Use tools/safe_wiggle.py / safe_teleop.py for arms.")
    # An arm's calibration writes the position limits; a loose new motor has 0..4095 (assumed, unverified)
    for i in [i for i in ids if name_of(i) in bus.motors]:
        lo = bus.read("Min_Position_Limit", name_of(i), normalize=False, num_retry=2)
        hi = bus.read("Max_Position_Limit", name_of(i), normalize=False, num_retry=2)
        if (lo, hi) != (0, 4095):
            print(f"!! id {i}: position limits {lo}..{hi}, not 0..4095 -> it was probably calibrated inside an arm.")
            if input("   Is this motor LOOSE (not built into an arm)? type LOOSE to continue: ").strip() != "LOOSE":
                bus.disconnect(disable_torque=False)
                raise SystemExit("stopped; nothing was moved")
    return ids


def check_answers(bus: FeetechMotorsBus, ids: list[int]) -> None:
    missing = [i for i in ids if bus.ping(i, num_retry=2) is None]
    if missing:
        bus.disconnect(disable_torque=False)
        raise SystemExit(f"no answer from id {missing}. Check the 5V/12V power and the 3-pin cable, "
                         "then run: python tools\\servo_play.py <PORT> scan")


def read_status(bus: FeetechMotorsBus, motor_id: int) -> dict[str, float]:
    n = name_of(motor_id)

    def r(reg: str) -> int:
        return bus.read(reg, n, normalize=False, num_retry=2)

    return {
        "position": r("Present_Position"),
        "velocity": r("Present_Velocity"),
        "load": r("Present_Load"),
        "volts": r("Present_Voltage") / 10,
        "temp_c": r("Present_Temperature"),
        "mode": r("Operating_Mode"),
        "torque": r("Torque_Enable"),
        "status": r("Status"),
    }


def set_caps(bus: FeetechMotorsBus, motor_id: int, speed: int, accel: int, torque_limit: int = 1000) -> None:
    if speed <= 0 or accel <= 0:
        raise ValueError("speed and accel must be > 0 (0 means 'no limit' to the motor)")
    n = name_of(motor_id)
    bus.write("Goal_Velocity", n, speed, normalize=False)
    bus.write("Acceleration", n, accel, normalize=False)
    bus.write("Torque_Limit", n, torque_limit, normalize=False)


def torque_on_here(bus: FeetechMotorsBus, motor_id: int, speed: int, accel: int) -> int:
    """Position mode, caps, Goal = Present, then torque on. Returns the position it holds."""
    n = name_of(motor_id)
    bus.disable_torque(n, num_retry=2)
    ensure_position_mode(bus, motor_id)
    set_caps(bus, motor_id, speed, accel)
    here = bus.read("Present_Position", n, normalize=False, num_retry=2)
    bus.write("Goal_Position", n, here, normalize=False)
    bus.enable_torque(n, num_retry=2)
    return here


def wait_until_near(bus: FeetechMotorsBus, motor_id: int, goal: int, tol: int = 15, timeout_s: float = 10.0) -> int:
    n = name_of(motor_id)
    t0 = time.monotonic()
    pos = bus.read("Present_Position", n, normalize=False, num_retry=2)
    while abs(pos - goal) > tol and time.monotonic() - t0 < timeout_s:
        time.sleep(0.02)
        pos = bus.read("Present_Position", n, normalize=False, num_retry=2)
    return pos


def ensure_position_mode(bus: FeetechMotorsBus, motor_id: int) -> None:
    """Operating_Mode lives in EEPROM (limited write cycles): write it only when it is not already 0."""
    n = name_of(motor_id)
    if bus.read("Operating_Mode", n, normalize=False, num_retry=3) != POSITION_MODE:
        bus.write("Operating_Mode", n, POSITION_MODE, normalize=False, num_retry=3)


def safe_close(bus: FeetechMotorsBus, ids: list[int]) -> None:
    """Torque off and back to position mode on every motor; report anything that fails."""
    for i in ids:
        n = name_of(i)
        for step, fn in (("torque off", lambda: bus.disable_torque(n, num_retry=3)),
                         ("position mode", lambda: ensure_position_mode(bus, i))):
            try:
                fn()
            except Exception as e:  # noqa: BLE001
                print(f"!! id {i}: could not set {step}: {e}  -> unplug the power; next run of any servo tool "
                      "resets the mode", flush=True)
    bus.disconnect(disable_torque=False)


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))
