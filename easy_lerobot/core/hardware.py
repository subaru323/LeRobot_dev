"""Hardware layout of this SO-101 setup (until project.json holds it).

Ports come from EASY_LEROBOT_LEADER_PORT / EASY_LEROBOT_FOLLOWER_PORT when set, so a
Raspberry Pi can name its boards by /dev/serial/by-id/... (stable across plug order)
without editing code. Otherwise each OS gets its default.
"""

import os
import sys

from serial.tools import list_ports

if sys.platform == "win32":
    _DEFAULT_LEADER, _DEFAULT_FOLLOWER = "COM19", "COM20"
else:
    _DEFAULT_LEADER, _DEFAULT_FOLLOWER = "/dev/ttyACM0", "/dev/ttyACM1"

LEADER_PORT = os.environ.get("EASY_LEROBOT_LEADER_PORT") or _DEFAULT_LEADER
FOLLOWER_PORT = os.environ.get("EASY_LEROBOT_FOLLOWER_PORT") or _DEFAULT_FOLLOWER


def present_ports() -> set[str]:
    """Serial ports the OS currently sees (cheap: does not open them).

    On Linux a configured port may be a symlink such as /dev/serial/by-id/...,
    so each configured port that resolves to a present device is included as well.
    """
    ports = {p.device for p in list_ports.comports()}
    if sys.platform != "win32":
        for port in (LEADER_PORT, FOLLOWER_PORT):
            if os.path.realpath(port) in ports:
                ports.add(port)
    return ports
