"""Remote inference fault test: kill the policy server mid-rollout and record what the robot side does.

Starts lerobot-policy-server and lerobot-rollout (PushT sim robot) as child processes, kills only the
server's own process tree after KILL_AFTER_S seconds of rollout, then waits for the rollout to finish.
Logs: remote_e2e/fault_server.log, remote_e2e/fault_rollout.log
Usage: .venv\\Scripts\\python.exe remote_e2e\\disconnect_test.py
"""

import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / ".venv" / "Scripts"
OUT = ROOT / "remote_e2e"
KILL_AFTER_S = 8.0

env = {**os.environ, "PYTHONIOENCODING": "utf-8", "HF_HUB_DISABLE_SYMLINKS_WARNING": "1"}


def kill_tree(proc: subprocess.Popen) -> None:
    subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)], capture_output=True)


server_log = open(OUT / "fault_server.log", "w", encoding="utf-8")
server = subprocess.Popen(
    [str(SCRIPTS / "lerobot-policy-server.exe"), "--config_path", "remote_e2e/server_pusht.yaml"],
    cwd=ROOT, env=env, stdout=server_log, stderr=subprocess.STDOUT,
)
rollout = None
try:
    deadline = time.time() + 180
    while "Policy server ready" not in (OUT / "fault_server.log").read_text(encoding="utf-8", errors="replace"):
        if server.poll() is not None or time.time() > deadline:
            sys.exit("server failed to start; see remote_e2e/fault_server.log")
        time.sleep(0.5)
    print("server ready")

    rollout_log = open(OUT / "fault_rollout.log", "w", encoding="utf-8")
    rollout = subprocess.Popen(
        [
            str(SCRIPTS / "lerobot-rollout.exe"),
            "--strategy.type=base",
            "--inference.type=remote",
            "--inference.endpoint=tcp/127.0.0.1:7447",
            "--inference.deployment=pusht",
            "--inference.semantics=pusht-position-v1",
            "--robot.type=sim",
            "--robot.env.type=pusht",
            '--rename_map={"observation.images.image": "observation.image"}',
            "--task=Push the T-shaped block onto the T-shaped target.",
            "--duration=30",
            "--fps=10",
        ],
        cwd=ROOT, env=env, stdout=rollout_log, stderr=subprocess.STDOUT,
    )
    # Wait until the control loop actually runs, then let it go a few seconds.
    while "control loop started" not in (OUT / "fault_rollout.log").read_text(encoding="utf-8", errors="replace"):
        if rollout.poll() is not None:
            sys.exit("rollout exited before starting; see remote_e2e/fault_rollout.log")
        time.sleep(0.2)
    t_start = time.time()
    time.sleep(KILL_AFTER_S)
    print(f"killing server (pid {server.pid}) {KILL_AFTER_S:.0f}s into the control loop")
    t_kill = time.time()
    kill_tree(server)

    try:
        code = rollout.wait(timeout=90)
        print(f"rollout exited with code {code} {time.time() - t_kill:.1f}s after the kill "
              f"({time.time() - t_start:.1f}s after loop start)")
    except subprocess.TimeoutExpired:
        print("rollout did not exit within 90s of the kill; stopping it")
        kill_tree(rollout)
finally:
    if server.poll() is None:
        kill_tree(server)
    if rollout is not None and rollout.poll() is None:
        kill_tree(rollout)
