"""K1 (3)(4): run the plan made by sim/tools/k1-plan.js on the follower, slowly, and record where it went.

usage: python tools\\k1_run.py sim\\out\\k1-plan.json [PORT] [--loops N] [--dry] [--k1 FILE]
  --loops N  go around the points N times (repeatability, K1 (4)). default 1
  --dry      check the plan file only (no motor connection)
  --k1 FILE  the k1 file the plan must come from (default sim/calib/k1_<robot id>.json; compared by sha256)

Safety (docs/plan/計画書_完成形まで.md, "安全に流す道具の決まり"):
  - refuses if the calibration in the motors is not the one the plan was made with (calibKey)
  - refuses a plan lower than MIN_LIFT_MM, or made from another k1 file than the current one (sha256)
  - refuses unless the arm is near the plan's start pose (the reference pose of k1_calib.py), within START_TOL_DEG
  - torque on with torque_on_in_place() (P-001), speed / acceleration / torque limits in the motors
  - sends the joints in steps of at most STEP_DEG along the straight line the sim checked, waits for each step
  - stops on high load, Ctrl+C or any error: holds the pose, asks you to SUPPORT THE ARM (Ctrl+C does not skip it),
    then releases torque
  - the gripper is never commanded
At each point it waits: measure where the jaw tip is on the paper grid and type "x,z" in mm (or Enter to skip).
  The paper grid is the K5 sheet (python tools\\k5_camera.py sheet -> outputs/k5/sheet_A4.pdf, printed at actual size): read the
  sim table x,z printed on it (bottom edge toward the arm at x = 80, z = 0 on the arm's forward line, +z the arm's right).
  Its rings P1..P11 are the plan's points. Do not move the sheet between K1 and the top camera pose (K5).
  Drop the jaw tip straight down onto the paper with a square or a plumb line (reading it at a slant from 50-80 mm up
  turns into a false shoulder / elbow offset in k1-err.js), and measure the two 150 mm rulers on the sheet (x and z) with a steel rule.
  A plan with a second height (--lift2) runs only after the one-height plan went within AIR_GATE_MM (checked here).
Writes sim/calib/k1_run_<date>.json. Read it with: node sim/tools/k1-err.js --run sim/calib/k1_run_<date>.json
(a plan with two heights, k1-plan.js --lift2 50, is needed to get the shoulder / elbow offsets and the drift going down).
"""

import argparse
import datetime
import hashlib
import json
import os
import time

STEP_DEG = 2.0        # same as STEP_DEG in sim/tools/k1-plan.js
START_TOL_DEG = 8.0
REACH_TOL_DEG = 1.5
STEP_TIMEOUT_S = 6.0
LOAD_ABORT = 600
AIR_GATE_MM = 10.0    # a plan with a second, lower height (k1-plan.js --lift2) needs an earlier run of the one-height plan (80 mm)
                      # with the same k1 file, not stopped, >= 9 points measured, every measured point within this of its target
MIN_LIFT_MM = 50.0    # K1 runs in the air first (計画書_完成形まで, 安全に流す道具の決まり). The first run: k1-plan.js --lift 80 (the default)
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


class Abort(Exception):
    pass


def calib_key(calib) -> str:
    """Same string as calibKey() in sim/robot-map.js."""
    return ";".join(f"{m}:{c.id},{c.drive_mode},{c.homing_offset},{c.range_min},{c.range_max}"
                    for m, c in ((m, calib[m]) for m in JOINTS + ["gripper"]))


def load_plan(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        p = json.load(f)
    wp = p.get("waypoints") or []
    if len(wp) < 3 or wp[0]["name"] != "基準" or wp[-1]["name"] != "基準":
        raise SystemExit("plan: waypoints must start and end at the reference pose")
    if abs(float(p.get("step_deg", 0)) - STEP_DEG) > 1e-9:
        raise SystemExit(f"plan: step_deg {p.get('step_deg')} != {STEP_DEG} (the sim checked a different path)")
    for w in wp:
        if set(w["lerobot"]) != set(JOINTS):
            raise SystemExit(f"plan: {w['name']} does not have exactly the 5 joints")
    return p


def air_run_ok(plan: dict, calib_dir: str) -> str:
    """'' if an earlier one-height run (same k1 file) went where it was told; else why not (点検 C41 の 2: the lower height
    skips the reason the first run is at 80 mm, a 0 off by 10 deg leaves 1-6 mm at 50 mm)."""
    import glob
    seen = []
    for f in sorted(glob.glob(os.path.join(calib_dir, "k1_run_*.json"))):
        try:
            with open(f, encoding="utf-8") as fh:
                r = json.load(fh)
        except (OSError, ValueError):
            continue
        if r.get("k1_sha256") != plan.get("k1_sha256") or r.get("lift2_mm") is not None or r.get("stopped"):
            continue
        pts = [q for q in r.get("points", []) if isinstance(q.get("measured_xz_mm"), list) and len(q["measured_xz_mm"]) == 2]
        if len(pts) < 9:
            seen.append(f"{os.path.basename(f)}: only {len(pts)} points measured")
            continue
        worst = max(((q["measured_xz_mm"][0] - q["target_mm"][0]) ** 2 + (q["measured_xz_mm"][1] - q["target_mm"][2]) ** 2) ** 0.5 for q in pts)
        if worst <= AIR_GATE_MM:
            return ""
        seen.append(f"{os.path.basename(f)}: a point was {worst:.1f} mm from its target")
    return "no earlier one-height run of this k1 file within %.0f mm%s" % (AIR_GATE_MM, (" (" + "; ".join(seen) + ")") if seen else "")


def steps_between(a: dict, b: dict) -> list[dict]:
    n = max(1, int(-(-max(abs(b[m] - a[m]) for m in JOINTS) // STEP_DEG)))   # ceil, same as k1-plan.js
    return [{m: a[m] + (b[m] - a[m]) * k / n for m in JOINTS} for k in range(1, n + 1)]


def main() -> None:
    ap = argparse.ArgumentParser(usage=__doc__)
    ap.add_argument("plan")
    ap.add_argument("port", nargs="?", default=None)
    ap.add_argument("--loops", type=int, default=1)
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--k1", default=None)
    a = ap.parse_args()
    loops = a.loops
    plan = load_plan(a.plan)
    wp = plan["waypoints"]
    nsteps = sum(len(steps_between(wp[i]["lerobot"], wp[i + 1]["lerobot"])) for i in range(len(wp) - 1))
    print(f"plan: {len(wp) - 2} points, {nsteps} steps per loop, lift {plan['lift_mm']} mm"
          + (f" and {plan['lift2_mm']} mm" if plan.get("lift2_mm") is not None else "") + f", lowest {plan['lowest_mm']:.1f} mm "
          f"({plan['lowest_at']}), jaw tip lowest {plan.get('tip_lowest_mm', float('nan')):.1f} mm, self {plan['self_min_mm']:.1f} mm, k1 of {plan['k1_date']}", flush=True)
    if nsteps != plan["nFrames"] - 1:
        raise SystemExit(f"plan: {nsteps} steps here but the sim checked {plan['nFrames'] - 1} -> the path differs, not running")
    lows = [float(plan["lift_mm"])] + ([float(plan["lift2_mm"])] if plan.get("lift2_mm") is not None else [])
    lows += [float(w["target_mm"][1]) for w in wp if w.get("target_mm")]
    if min(lows) < MIN_LIFT_MM:
        raise SystemExit(f"plan: a height of {min(lows)} mm < {MIN_LIFT_MM} mm. Lower heights come after the in-air runs (K1).")
    k1file = a.k1 or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim", "calib", f"k1_{plan['robot_id']}.json")
    if not os.path.exists(k1file):
        raise SystemExit(f"no {os.path.normpath(k1file)} -> run k1_calib.py")
    with open(k1file, "rb") as f:
        if hashlib.sha256(f.read()).hexdigest() != plan.get("k1_sha256"):
            raise SystemExit("the plan was made from another k1 file (k1_calib.py was run again?) -> run k1-plan.js again")
    calib_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim", "calib")
    if plan.get("lift2_mm") is not None:
        why = air_run_ok(plan, calib_dir)
        if why:
            raise SystemExit(f"plan with a second height {plan['lift2_mm']} mm: {why}. Run the 80 mm plan (k1-plan.js without --lift2) first.")
    if a.dry:
        print("dry run: plan file OK", flush=True)
        return

    from so101_common import (FOLLOWER_PORT, check_motors_ok, connect_bus_no_torque, limit_follower, make_follower,
                              max_load, release_all, torque_on_in_place)
    port = a.port or FOLLOWER_PORT
    robot = make_follower(port)
    bus = robot.bus
    connect_bus_no_torque(bus)
    log = {"plan": os.path.abspath(a.plan), "date": datetime.datetime.now().isoformat(timespec="seconds"),
           "k1_sha256": plan.get("k1_sha256"), "lift_mm": plan.get("lift_mm"), "lift2_mm": plan.get("lift2_mm"), "points": []}
    torque = {"on": False}

    def support_then_release() -> None:
        """Do not let go before the user supports the arm. Ctrl+C here does not skip it."""
        while True:
            try:
                input("SUPPORT THE ARM with your hand, then press Enter to release torque...")
                return
            except (KeyboardInterrupt, EOFError):
                print("\n(the arm is still held. Support it, then press Enter. Or unplug the 5V power while supporting it.)", flush=True)
                time.sleep(2)

    try:
        check_motors_ok(bus, "follower")
        if calib_key(bus.calibration) != plan["calibKey"]:
            raise SystemExit("the calibration in the motors is not the one the plan was made with -> run k1_calib.py and k1-plan.js again")
        start = wp[0]["lerobot"]
        while True:
            now = bus.sync_read("Present_Position", num_retry=3)
            off = {m: now[m] - start[m] for m in JOINTS}
            worst = max(JOINTS, key=lambda m: abs(off[m]))
            if abs(off[worst]) <= START_TOL_DEG:
                break
            print(f"not at the reference pose: {worst} is {off[worst]:+.1f} deg away (limit {START_TOL_DEG}). "
                  "Put the arm in the reference pose (upper arm vertical, forearm horizontal, jaws down).", flush=True)
            if input("Enter = read again, q = quit: ").strip().lower() == "q":
                return
        limit_follower(bus, speed=150, accel=10, torque_limit=700)
        input("Torque will turn ON and the arm will hold this pose. Keep a hand near the arm. Press Enter...")
        torque["on"] = True
        torque_on_in_place(bus)
        time.sleep(0.5)

        def go(target: dict) -> dict:
            bus.sync_write("Goal_Position", target)
            t0 = time.time()
            while True:
                time.sleep(0.05)
                pos = bus.sync_read("Present_Position", num_retry=2)
                m, load = max_load(bus)
                if load > LOAD_ABORT:
                    raise Abort(f"high load on {m}: {load}")
                if max(abs(pos[j] - target[j]) for j in JOINTS) < REACH_TOL_DEG:
                    return pos
                if time.time() - t0 > STEP_TIMEOUT_S:
                    raise Abort(f"did not reach the step in {STEP_TIMEOUT_S}s: " +
                                ", ".join(f"{j} {pos[j] - target[j]:+.1f}" for j in JOINTS))

        try:
            now = bus.sync_read("Present_Position", num_retry=3)
            for s in steps_between({j: now[j] for j in JOINTS}, start):   # first, from where the arm is to the exact start pose
                go(s)
            for lp in range(loops):
                cur = dict(start)
                for i in range(len(wp) - 1):
                    for s in steps_between(cur, wp[i + 1]["lerobot"]):
                        go(s)
                    cur = wp[i + 1]["lerobot"]
                    w = wp[i + 1]
                    if not w.get("target_mm"):
                        continue
                    time.sleep(0.5)
                    pos = bus.sync_read("Present_Position", num_retry=3)
                    err = {j: round(pos[j] - w["lerobot"][j], 2) for j in JOINTS}
                    print(f"[loop {lp + 1} {w['name']}] target x {w['target_mm'][0]:.0f} z {w['target_mm'][2]:.0f} mm "
                          f"(height {w['target_mm'][1]:.0f}). joint error {err}", flush=True)
                    s = input("  measured x,z on the sheet in mm (the printed numbers; Enter = skip, q = stop): ").strip()
                    if s.lower() == "q":
                        raise Abort("stopped by user")
                    meas = None
                    if s:
                        try:
                            meas = [float(v) for v in s.split(",")]
                        except ValueError:
                            print("  not understood, saved as text", flush=True)
                            meas = s
                    log["points"].append({"loop": lp + 1, "name": w["name"], "target_mm": w["target_mm"],
                                          "lerobot_cmd": w["lerobot"], "lerobot_present": {j: pos[j] for j in JOINTS},
                                          "measured_xz_mm": meas})
            print("done, back at the reference pose.", flush=True)
        except BaseException as e:   # Abort, Ctrl+C, and also bus errors (ConnectionError): hold, never just let go
            print(f"!! stopped: {e!r}. Holding the current pose.", flush=True)
            try:
                pos = bus.sync_read("Present_Position", num_retry=3)
                bus.sync_write("Goal_Position", {j: pos[j] for j in JOINTS})
            except Exception:  # noqa: BLE001
                pass
            log["stopped"] = repr(e)
    finally:
        if torque["on"]:
            support_then_release()
        try:
            release_all(bus)
        finally:
            bus.disconnect(disable_torque=False)
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim", "calib",
                           "k1_run_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".json")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(log, f, indent=1, ensure_ascii=False)
        print(f"released. wrote {os.path.normpath(out)}", flush=True)


if __name__ == "__main__":
    main()
