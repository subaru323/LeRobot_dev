"""Fixed hardware layout of this SO-101 setup (until project.json holds it)."""

from serial.tools import list_ports

LEADER_PORT = "COM19"
FOLLOWER_PORT = "COM20"


def present_ports() -> set[str]:
    """COM ports Windows currently sees (cheap: does not open them)."""
    return {p.device for p in list_ports.comports()}
