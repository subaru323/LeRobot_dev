"""Long-running child processes (mirror, recording) shared by all pages.

What runs is kept in files, not in memory: Streamlit re-imports this module when
source files change and the server may be restarted, and a process holding the
arm's serial ports must never be forgotten. Per process name, the state directory
holds <name>.json (pid, creation time, whether the user asked it to stop) and
<name>.exit.json (exit code, written by break_as_interrupt when the program ends).

Only one hardware process may run at a time because they share the serial ports.

Windows and Linux (Raspberry Pi) differ only in how a process is started, checked and
stopped; the files and the rules above are the same on both.
"""

import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from core import logs

BREAK_WRAPPER = Path(__file__).with_name("break_as_interrupt.py")
STATE_DIR = Path(tempfile.gettempdir()) / "easy_lerobot" / "procs"
NAMES = ("mirror", "record")

WINDOWS = sys.platform == "win32"

if WINDOWS:
    import ctypes
    from ctypes import wintypes

    _kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _kernel32.OpenProcess.restype = wintypes.HANDLE
    _PROCESS_QUERY_LIMITED_INFORMATION = 0x1000
    _STILL_ACTIVE = 259

    def _creation_time(pid: int) -> int | None:
        """Creation time of a live process (FILETIME as an int), or None if it is not running."""
        handle = _kernel32.OpenProcess(_PROCESS_QUERY_LIMITED_INFORMATION, False, pid)
        if not handle:
            return None
        try:
            code = wintypes.DWORD()
            if not _kernel32.GetExitCodeProcess(handle, ctypes.byref(code)) or code.value != _STILL_ACTIVE:
                return None
            created, exited, kernel, user = (ctypes.c_ulonglong() for _ in range(4))
            if not _kernel32.GetProcessTimes(handle, ctypes.byref(created), ctypes.byref(exited),
                                             ctypes.byref(kernel), ctypes.byref(user)):
                return None
            return created.value
        finally:
            _kernel32.CloseHandle(handle)

else:

    def _creation_time(pid: int) -> int | None:
        """Start time of a live process (clock ticks since boot), or None if it is not running.

        A zombie (exited, not yet reaped) counts as not running.
        """
        try:
            stat = Path(f"/proc/{pid}/stat").read_text()
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            return None
        # The command name (2nd field) is in parentheses and may contain spaces.
        fields = stat[stat.rindex(")") + 2:].split()
        if fields[0] == "Z":
            return None
        return int(fields[19])  # field 22 of /proc/<pid>/stat: starttime


def _state_file(name: str) -> Path:
    return STATE_DIR / f"{name}.json"


def _exit_file(name: str) -> Path:
    return STATE_DIR / f"{name}.exit.json"


def _read(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def start(name: str, args: list[str], cwd: Path, log_path: Path, stdin_text: str | None = None):
    """Start `python core/break_as_interrupt.py <args>` with stdout/stderr going to log_path."""
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    logs.rotate(log_path)  # keep the previous run of this process on disk
    _exit_file(name).unlink(missing_ok=True)
    env = {
        **os.environ,
        "PYTHONUNBUFFERED": "1",
        "PYTHONIOENCODING": "utf-8",
        "EASY_LEROBOT_EXIT_FILE": str(_exit_file(name)),
    }
    with open(log_path, "w", encoding="utf-8") as log:
        proc = subprocess.Popen(
            [sys.executable, str(BREAK_WRAPPER), *args],
            cwd=cwd,
            stdin=subprocess.PIPE if stdin_text is not None else subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
            # Its own process group, so a stop signal reaches the program and its children only.
            **({"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP} if WINDOWS else {"start_new_session": True}),
        )
    if stdin_text is not None:
        proc.stdin.write(stdin_text.encode("utf-8"))
        proc.stdin.close()
    state = {"pid": proc.pid, "created": _creation_time(proc.pid), "stop_requested": False}
    _state_file(name).write_text(json.dumps(state), encoding="utf-8")


def running(name: str) -> bool:
    state = _read(_state_file(name))
    # Compare creation times so a recycled pid is not mistaken for our process.
    return state is not None and state["created"] is not None and _creation_time(state["pid"]) == state["created"]


def running_name() -> str | None:
    """Name of the process currently running, if any."""
    return next((name for name in NAMES if running(name)), None)


def failed(name: str) -> bool:
    """True when the last run ended with an error that the user did not cause.

    Stopping with Ctrl+Break (Windows) or SIGINT (Linux) re-raises KeyboardInterrupt in programs such as
    lerobot-record, so a non-zero exit after stop() is not a failure. A run that
    left no exit code at all (killed, crashed wrapper) counts as failed.
    """
    state = _read(_state_file(name))
    if state is None or running(name) or state["stop_requested"]:
        return False
    result = _read(_exit_file(name))
    return result is None or result["exit_code"] != 0


def finished_ok(name: str) -> bool:
    """True when the last run has ended normally or was stopped by the user."""
    return _read(_state_file(name)) is not None and not running(name) and not failed(name)


def stop(name: str, wait_s: float | None = 5.0):
    """Ask the process to stop as on Ctrl+C.

    With wait_s, kill its whole process tree if it is still alive after that many
    seconds. With None, return at once and let it finish its cleanup (e.g. video
    encoding) on its own.
    """
    if not running(name):
        return
    state = _read(_state_file(name))
    state["stop_requested"] = True
    _state_file(name).write_text(json.dumps(state), encoding="utf-8")
    if WINDOWS:
        os.kill(state["pid"], signal.CTRL_BREAK_EVENT)  # reaches the whole process group
    else:
        os.killpg(state["pid"], signal.SIGINT)  # start_new_session made the pid the group id
    if wait_s is None:
        return
    deadline = time.monotonic() + wait_s
    while running(name) and time.monotonic() < deadline:
        time.sleep(0.1)
    if running(name):
        if WINDOWS:
            # The venv's python.exe is a launcher with the real interpreter as its child: kill the tree.
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(state["pid"])], capture_output=True)
        else:
            try:
                os.killpg(state["pid"], signal.SIGKILL)
            except ProcessLookupError:
                pass
