"""Which serial port is the leader and which is the follower.

Order of precedence:
1. EASY_LEROBOT_LEADER_PORT / EASY_LEROBOT_FOLLOWER_PORT, when set.
2. On Linux (the Raspberry Pi), detection: every servo board under /dev/serial/by-id
   is asked for the Homing_Offset of its motors, and the arm whose calibration file
   (dev_calibration/leader_L.json, follower_F.json) matches is assigned. Boards may be
   plugged into any socket, or swapped, and the arms are still told apart. Only
   registers are read; nothing moves.
3. Windows: the fixed COM19 / COM20. Linux: UNDETECTED, a name that matches no port,
   so nothing can be started until the arms are told apart. Falling back to
   /dev/ttyACM0 / ttyACM1 there could drive the leader as if it were the follower,
   because those numbers follow the plug order.

Detection opens the ports, so it never runs while a mirror or recording holds them,
and its result is reused until the set of plugged boards changes.
"""

import json
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

from serial.tools import list_ports

if sys.platform == "win32":
    _DEFAULT_LEADER, _DEFAULT_FOLLOWER = "COM19", "COM20"
else:
    _DEFAULT_LEADER, _DEFAULT_FOLLOWER = "/dev/ttyACM0", "/dev/ttyACM1"  # only with one env var set
UNDETECTED = "未判定"

REPO_DIR = Path(__file__).resolve().parents[2]
CALIBRATION = {"leader": "dev_calibration/leader_L.json", "follower": "dev_calibration/follower_F.json"}
BY_ID_DIR = Path("/dev/serial/by-id")
MOTOR_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
ADDR_HOMING_OFFSET = 31  # 2 bytes, sign in bit 11
BAUDRATE = 1_000_000
MATCH_TOLERANCE = 30  # ticks; a re-flashed offset never lands this close to the other arm's
RETRY_S = 5.0  # an incomplete detection (arm power off, one board) is retried this often


@dataclass
class Detection:
    """Outcome of the last detection, shown in the readiness log."""

    roles: dict[str, str] = field(default_factory=dict)  # "leader"/"follower" -> port
    notes: dict[str, str] = field(default_factory=dict)  # port -> what was found there
    skipped: str = ""  # why detection did not run (env vars, Windows, arm process running)


_cache: dict = {"boards": None, "detection": Detection(), "at": 0.0}


def present_ports() -> set[str]:
    """Serial ports the OS currently sees (cheap: does not open them).

    On Linux a port may be named by a symlink such as /dev/serial/by-id/...,
    so the by-id names of present devices are included as well.
    """
    ports = {p.device for p in list_ports.comports()}
    if sys.platform != "win32" and BY_ID_DIR.is_dir():
        ports |= {str(p) for p in BY_ID_DIR.iterdir() if os.path.realpath(p) in ports}
    for port in arm_ports(allow_detect=False):
        if os.path.realpath(port) in ports:
            ports.add(port)
    return ports


def _boards() -> tuple[str, ...]:
    """Servo boards by their stable by-id names (Linux)."""
    if not BY_ID_DIR.is_dir():
        return ()
    return tuple(sorted(str(p) for p in BY_ID_DIR.iterdir()))


def _read_offsets(port: str) -> dict[str, int]:
    import scservo_sdk as scs  # only needed where detection runs

    ph = scs.PortHandler(port)
    if not ph.openPort() or not ph.setBaudRate(BAUDRATE):
        return {}
    pk = scs.PacketHandler(0)
    out = {}
    try:
        for motor_id, name in enumerate(MOTOR_NAMES, start=1):
            value, res, _ = pk.read2ByteTxRx(ph, motor_id, ADDR_HOMING_OFFSET)
            if res == scs.COMM_SUCCESS:
                out[name] = -(value & 0x7FF) if value & 0x800 else value & 0x7FF
    finally:
        ph.closePort()
    return out


def _calibration(role: str) -> dict[str, int]:
    data = json.loads((REPO_DIR / CALIBRATION[role]).read_text(encoding="utf-8"))
    return {name: entry["homing_offset"] for name, entry in data.items()}


def detect(boards: tuple[str, ...]) -> Detection:
    """Assign leader / follower to the given boards by matching their motors' offsets."""
    result = Detection()
    try:
        expected = {role: _calibration(role) for role in CALIBRATION}
    except (FileNotFoundError, KeyError, json.JSONDecodeError) as e:
        result.skipped = f"校正ファイルを読めません（{e}）"
        return result
    for port in boards:
        offsets = _read_offsets(port)
        if not offsets:
            result.notes[port] = "モーターが応答しません（アームの電源？）"
            continue
        matches = {
            role: sum(abs(offsets[n] - cal[n]) <= MATCH_TOLERANCE for n in offsets if n in cal)
            for role, cal in expected.items()
        }
        role = max(matches, key=matches.get)
        other = min(matches, key=matches.get)
        if matches[role] == len(offsets) and matches[other] < matches[role] and role not in result.roles:
            result.roles[role] = port
            result.notes[port] = f"モーター{matches[role]}台が校正と一致"
        else:
            result.notes[port] = f"判定できません（リーダー{matches['leader']}台・フォロワー{matches['follower']}台が一致）"
    return result


def arm_ports(allow_detect: bool = True) -> tuple[str, str]:
    """(leader port, follower port) by the precedence in the module docstring."""
    env_leader = os.environ.get("EASY_LEROBOT_LEADER_PORT")
    env_follower = os.environ.get("EASY_LEROBOT_FOLLOWER_PORT")
    if env_leader or env_follower:
        _cache["detection"] = Detection(skipped="環境変数で指定")
        return env_leader or _DEFAULT_LEADER, env_follower or _DEFAULT_FOLLOWER
    if sys.platform == "win32":
        _cache["detection"] = Detection(skipped="Windows では固定")
        return _DEFAULT_LEADER, _DEFAULT_FOLLOWER
    boards = _boards()
    incomplete = len(_cache["detection"].roles) < 2 and time.monotonic() - _cache["at"] > RETRY_S
    if allow_detect and boards and (boards != _cache["boards"] or incomplete):
        from core import procs  # late import: procs does not need hardware

        if procs.running_name() is None:  # never open ports a mirror or recording is using
            _cache["detection"] = detect(boards)
            _cache["boards"] = boards
            _cache["at"] = time.monotonic()
    # A board that has been unplugged loses its role, so the readiness log cannot stay at
    # "ready" and nothing is started on a port that is gone; it is detected again when back.
    detection = _cache["detection"]
    gone = [role for role, port in detection.roles.items() if port not in boards]
    for role in gone:
        detection.notes.pop(detection.roles.pop(role), None)
    if gone:
        _cache["boards"] = None
    roles = detection.roles
    return roles.get("leader", UNDETECTED), roles.get("follower", UNDETECTED)


def last_detection() -> Detection:
    return _cache["detection"]
