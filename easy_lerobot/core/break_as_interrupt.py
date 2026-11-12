"""Run a Python script or module so that Ctrl+Break stops it like Ctrl+C.

A process started with CREATE_NEW_PROCESS_GROUP can only be signalled with
CTRL_BREAK_EVENT, which Python treats as SIGBREAK and terminates on at once.
Mapping it to KeyboardInterrupt lets the program run its own cleanup (closing
ports, finalizing a dataset) exactly as when Ctrl+C is pressed in a terminal.

When EASY_LEROBOT_EXIT_FILE is set, the exit code is written there as JSON, so
the UI can read the result even if it was restarted while the program ran.

Usage:
    python core/break_as_interrupt.py <script.py> [args...]
    python core/break_as_interrupt.py -m <module> [args...]
"""

import json
import os
import runpy
import signal
import sys
import traceback


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def _run():
    if sys.argv[1] == "-m":
        module = sys.argv[2]
        sys.argv = [module, *sys.argv[3:]]
        runpy.run_module(module, run_name="__main__", alter_sys=True)
    else:
        script = sys.argv[1]
        sys.argv = sys.argv[1:]
        runpy.run_path(script, run_name="__main__")


if __name__ == "__main__":
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, _raise_interrupt)
    exit_code = 0
    try:
        _run()
    except SystemExit as e:
        if isinstance(e.code, str):
            print(e.code, file=sys.stderr)  # what Python itself prints for sys.exit("message")
            exit_code = 1
        else:
            exit_code = e.code or 0
    except KeyboardInterrupt:
        exit_code = 130
    except BaseException:
        traceback.print_exc()
        exit_code = 1
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
    exit_file = os.environ.get("EASY_LEROBOT_EXIT_FILE")
    if exit_file:
        with open(exit_file, "w", encoding="utf-8") as f:
            json.dump({"exit_code": exit_code}, f)
    sys.exit(exit_code)
