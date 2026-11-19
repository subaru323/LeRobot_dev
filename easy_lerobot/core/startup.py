"""Readiness checks shown on the home page after the Pi is switched on.

The phone can only open the page once the screen server runs, so this log starts
there: it lists what the arms need (both servo boards, the joint 5 range file) and
what is nice to have (cameras), each with the time it was first seen. Nothing here
opens a serial port, so the checks never get in the way of a running mirror.
"""

import sys
import time
from dataclasses import dataclass
from pathlib import Path

from core.hardware import FOLLOWER_PORT, LEADER_PORT, present_ports

REPO_DIR = Path(__file__).resolve().parents[2]
RANGE_FILE = REPO_DIR / "dev_calibration" / "joint5_maxmin.json"
CAMERA_DIR = Path("/dev/v4l/by-id")
EXPECTED_CAMERAS = 2


@dataclass
class Check:
    label: str
    ok: bool
    detail: str
    required: bool = True  # False: shown as a warning, does not hold back "ready"


def system_uptime_s() -> float | None:
    """Seconds since the machine booted (Linux only)."""
    try:
        return float(Path("/proc/uptime").read_text().split()[0])
    except (FileNotFoundError, ValueError, IndexError):
        return None


def camera_count() -> int | None:
    """Video capture devices named under /dev/v4l/by-id (each camera lists index0 for its picture)."""
    if sys.platform == "win32":
        return None
    try:
        return sum(1 for p in CAMERA_DIR.iterdir() if p.name.endswith("-index0"))
    except FileNotFoundError:
        return 0


def run_checks() -> list[Check]:
    ports = present_ports()
    checks = [
        Check("リーダーの基板", LEADER_PORT in ports, LEADER_PORT),
        Check("フォロワーの基板", FOLLOWER_PORT in ports, FOLLOWER_PORT),
        Check("5番関節の範囲ファイル", RANGE_FILE.is_file(), "dev_calibration/joint5_maxmin.json"),
    ]
    cameras = camera_count()
    if cameras is not None:
        checks.append(
            Check("カメラ", cameras >= EXPECTED_CAMERAS, f"{cameras} / {EXPECTED_CAMERAS} 台", required=False)
        )
    return checks


def ready(checks: list[Check]) -> bool:
    return all(c.ok for c in checks if c.required)


class StartupLog:
    """Remembers when each check first passed, so the page reads like a boot log."""

    def __init__(self):
        self.server_started = time.time()
        self.boot_uptime = system_uptime_s()  # how long the machine had been up when the server started
        self.first_ok: dict[str, float] = {}
        self.ready_at: float | None = None

    def update(self, checks: list[Check]) -> None:
        now = time.time()
        for c in checks:
            if c.ok:
                self.first_ok.setdefault(c.label, now)
            else:
                self.first_ok.pop(c.label, None)  # unplugged again: show it as waiting
        if ready(checks):
            if self.ready_at is None:
                self.ready_at = now
        else:
            self.ready_at = None

    def lines(self, checks: list[Check]) -> list[tuple[str, str, str]]:
        """(time, mark, text) rows: ✓ done, … waiting, ! optional and missing."""
        rows = []
        boot = f"（ラズパイの起動から {self.boot_uptime:.0f} 秒）" if self.boot_uptime is not None else ""
        rows.append((_clock(self.server_started), "✓", f"画面のサーバーが起動{boot}"))
        for c in checks:
            if c.ok:
                rows.append((_clock(self.first_ok[c.label]), "✓", f"{c.label}  {c.detail}"))
            elif c.required:
                rows.append(("", "…", f"{c.label}を待っています  {c.detail}"))
            else:
                rows.append(("", "!", f"{c.label}  {c.detail}（なくてもミラーはできます）"))
        if self.ready_at is not None:
            rows.append((_clock(self.ready_at), "✓", "準備完了"))
        return rows


def _clock(t: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(t))
