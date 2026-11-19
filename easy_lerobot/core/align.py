"""Automatic leader / follower pose alignment before the app drives the follower.

Whenever the app uses the follower's servos (mirror, recording, torque on), the follower is
first moved slowly to the leader's pose by dev_scripts/mirror_leader.py --auto-align, so it
never jumps to a distant pose at full speed. This is a fixed rule for every app feature.
"""

import subprocess
import sys
from pathlib import Path

REPO_DIR = Path(__file__).resolve().parents[2]
MIRROR_SCRIPT = REPO_DIR / "dev_scripts" / "mirror_leader.py"
# Move joints that differ by more than MAX_START_DIFF ticks (~9 deg) at the slow --align-speed
# before anything else drives the follower.
MAX_START_DIFF = 100
ALIGN_FLAGS = ["--auto-align", "--max-start-diff", str(MAX_START_DIFF)]
# Joint 5 crosses the 4095/0 seam: it is range-mapped and aligned in speed mode along its range, so it
# never takes the short way through the gap between its ends (docs/LOG.md P-004).
JOINT5_FLAGS = ["--range-files", "dev_calibration/joint5_maxmin.json", "--velocity-ids", "5"]
TIMEOUT_S = 60


def align_follower(leader_port: str, follower_port: str) -> tuple[bool, str]:
    """Move the follower to the leader pose and return (ok, last line printed)."""
    try:
        proc = subprocess.run(
            [sys.executable, str(MIRROR_SCRIPT), "--leader-port", leader_port, "--follower-port", follower_port,
             *ALIGN_FLAGS, *JOINT5_FLAGS, "--align-only"],
            cwd=REPO_DIR,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=TIMEOUT_S,
        )
    except subprocess.TimeoutExpired:
        return False, f"{TIMEOUT_S} 秒以内にフォロワーをリーダーの姿勢に合わせられませんでした（ぶつかっていないか確認してください）。"
    lines = [ln.strip() for ln in (proc.stdout + proc.stderr).splitlines() if ln.strip()]
    last = lines[-1] if lines else ""
    if proc.returncode == 0 and any(ln.startswith("align-only: done") for ln in lines):
        return True, last
    return False, f"姿勢を合わせられませんでした: {last}"
