"""Where the logs of mirror and recording runs are kept.

The newest run of each process writes <name>.log (the pages read it for progress and
error messages). When the next run starts, the previous file is renamed to
<name>-YYYYmmdd-HHMMSS.log (its last write time), so every run stays on disk across
restarts and power cuts. The oldest files beyond KEEP per name are removed so the SD
card does not fill up.
"""

import os
import time
from pathlib import Path

LOG_DIR = Path(os.environ.get("EASY_LEROBOT_LOG_DIR") or Path(__file__).resolve().parents[1] / "logs")
KEEP = 200  # archived runs per name


def path(name: str) -> Path:
    """Log file of the newest run of `name` (e.g. "mirror")."""
    return LOG_DIR / f"{name}.log"


def rotate(log_path: Path) -> None:
    """Keep the previous run's log under a timestamped name before a new run overwrites it."""
    try:
        if log_path.stat().st_size == 0:
            return
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(log_path.stat().st_mtime))
    except FileNotFoundError:
        return
    target = log_path.with_name(f"{log_path.stem}-{stamp}{log_path.suffix}")
    n = 1
    while target.exists():  # two runs ending in the same second
        target = log_path.with_name(f"{log_path.stem}-{stamp}-{n}{log_path.suffix}")
        n += 1
    log_path.rename(target)
    _prune(log_path)


def _prune(log_path: Path) -> None:
    archived = sorted(log_path.parent.glob(f"{log_path.stem}-*{log_path.suffix}"))
    for old in archived[:-KEEP]:
        old.unlink(missing_ok=True)
