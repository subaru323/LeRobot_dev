"""K1 (1)(2): find the conversion between the sim's URDF angles and LeRobot's angles, by hand, torque OFF.

usage: python tools\\k1_calib.py [PORT]      (default: COM11)

Nothing moves by itself: torque stays OFF the whole time, you move the arm by hand.
  1. Direction of each joint: move ONE joint by hand in the direction printed (about 20-30 deg), press Enter.
  2. Reference pose (REF_TEXT below). Press Enter. The offsets come from this pose.
  3. Check pose (CHECK_TEXT below): every joint is at least 45 deg away from the reference pose. The tool predicts
     each joint from the signs and offsets; if any joint is off by more than CHECK_TOL_DEG, it does NOT write the file
     (a wrong sign or a badly held reference pose would send the arm into the table later).
  4. Gripper: close it fully by hand, press Enter. Then hold it open on something of known width, type the width (mm).
"Upper arm" below means the line through the centers of the shoulder_lift and elbow_flex motor horns
(the outer shape of the upper-arm part is tilted about 14 deg from that line).
Writes sim/calib/k1_<robot id>.json, tied to the calibration in the motors (the sim tools refuse a plan if it changed).
Conversion (per joint j): q_urdf = sign_j * (q_lerobot - offset_j), all in degrees. Gripper: kept as raw pairs
(LeRobot 0-100, jaw gap in mm); sim/robot-map.js turns them into the URDF jaw angle.
"""

import datetime
import json
import os
import sys

from so101_common import FOLLOWER_ID, FOLLOWER_PORT, check_motors_ok, connect_bus_no_torque, make_follower

PORT = sys.argv[1] if len(sys.argv) > 1 else FOLLOWER_PORT
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
# Reference pose in URDF degrees (sim/tools/k1-plan.js REF_URDF must be the same numbers).
# Found with the sim's forward kinematics: upper arm vertical, forearm horizontal, jaws straight down.
REF_URDF = [0.0, -13.97, 16.18, 83.19, 0.0]
REF_TEXT = ("arm pointing straight forward; upper arm (line through the shoulder_lift and elbow motor horn centers) VERTICAL; "
            "forearm HORIZONTAL; jaws pointing STRAIGHT DOWN; moving jaw toward the front; wrist camera (if mounted) on the LEFT")
# Check pose (sim FK): pan +90, upper arm 45 deg forward, forearm horizontal, jaws horizontal, wrist_roll +90.
# sim/tools/k1-plan.js does not use it; k1-plan.js refuses a k1 file without "verified": true.
CHECK_URDF = [90.0, 31.03, -28.82, -2.31, 90.0]
CHECK_TEXT = ("seen from above, turn the whole arm 90 deg CLOCKWISE (it now points to its right); tilt the upper arm "
              "45 deg toward that direction (away from the base); forearm HORIZONTAL; jaws HORIZONTAL, pointing straight "
              "away from the base; moving jaw on the side facing where the arm pointed in the reference pose "
              "(the front of the robot); wrist camera, if mounted, UNDER the wrist")
# (sim FK at CHECK_URDF, 2026-11-18: moving jaw pivot 22.8 mm toward the robot's front (+x), camera 73 mm below.
#  So the moving jaw fixes wrist_roll without the camera mount; the other way round is wrist_roll 180 deg off.)
CHECK_TOL_DEG = 15.0
# What "+" means in the URDF, seen from the reference pose (from the sim's forward kinematics, +5 deg each).
PLUS = {
    "shoulder_pan": "seen from above, the arm turns to its RIGHT (clockwise)",
    "shoulder_lift": "the upper arm tilts FORWARD (the elbow goes forward, away from the base)",
    "elbow_flex": "the forearm tip goes DOWN",
    "wrist_flex": "the jaw tips swing BACKWARD (toward the base)",
    "wrist_roll": "seen from above, the jaws turn COUNTER-clockwise",
}
MIN_MOVE_DEG = 8.0
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim", "calib", f"k1_{FOLLOWER_ID}.json")


def read_pose(bus) -> dict:
    return {m: float(v) for m, v in bus.sync_read("Present_Position", num_retry=3).items()}


def main() -> None:
    robot = make_follower(PORT)
    bus = robot.bus
    connect_bus_no_torque(bus)
    try:
        check_motors_ok(bus, "follower")
        calib = {m: {"id": c.id, "drive_mode": c.drive_mode, "homing_offset": c.homing_offset,
                     "range_min": c.range_min, "range_max": c.range_max} for m, c in bus.calibration.items()}
        print("Torque is OFF. Nothing will move by itself.\n", flush=True)

        sign, steps = {}, []
        for m in JOINTS:
            while True:
                input(f"[{m}] Put the arm near the reference pose, then press Enter (before)...")
                a = read_pose(bus)
                input(f"[{m}] Now move ONLY {m} by hand about 20-30 deg so that {PLUS[m]}. Press Enter (after)...")
                b = read_pose(bus)
                d = {k: b[k] - a[k] for k in JOINTS}
                others = max(abs(d[k]) for k in JOINTS if k != m)
                print(f"   moved {m} by {d[m]:+.1f} (other joints at most {others:.1f})", flush=True)
                if abs(d[m]) < MIN_MOVE_DEG:
                    print(f"   less than {MIN_MOVE_DEG} deg -> again", flush=True)
                    continue
                if others > abs(d[m]) / 2:
                    print("   another joint moved more than half as much -> again", flush=True)
                    continue
                sign[m] = 1 if d[m] > 0 else -1
                steps.append({"joint": m, "before": a, "after": b})
                break

        input(f"\nReference pose: {REF_TEXT}.\nHold it there and press Enter...")
        ref = read_pose(bus)
        offset = {m: ref[m] - sign[m] * REF_URDF[i] for i, m in enumerate(JOINTS)}

        input(f"\nCheck pose: {CHECK_TEXT}.\nHold it there and press Enter...")
        chk = read_pose(bus)
        diff = {m: sign[m] * (chk[m] - offset[m]) - CHECK_URDF[i] for i, m in enumerate(JOINTS)}
        print("   check pose, sim angle from the reading minus expected: " + ", ".join(f"{m} {diff[m]:+.1f}" for m in JOINTS), flush=True)
        if any(abs(diff[m]) > CHECK_TOL_DEG for m in JOINTS):
            print(f"!! some joint is off by more than {CHECK_TOL_DEG} deg -> NOT writing the file. "
                  "Check the + directions and the reference pose, then run again.", flush=True)
            return

        input("\nGripper: close it fully by hand and press Enter...")
        closed = read_pose(bus)["gripper"]
        grip = [{"lerobot": closed, "gap_mm": 0.0}]
        while True:
            s = input("Hold the jaws open on something of known width, type the width in mm and press Enter (empty = done): ").strip()
            if not s:
                break
            try:
                w = float(s)
            except ValueError:
                print("   not a number", flush=True)
                continue
            grip.append({"lerobot": read_pose(bus)["gripper"], "gap_mm": w})
        if len(grip) < 2:
            print("!! need at least one open width for the gripper; the gripper part is not usable", flush=True)

        out = {
            "robot_id": FOLLOWER_ID, "port": PORT, "date": datetime.datetime.now().isoformat(timespec="seconds"),
            "calibration": calib, "joints": JOINTS, "ref_urdf": REF_URDF, "ref_lerobot": ref,
            "check_urdf": CHECK_URDF, "check_lerobot": chk, "check_diff_deg": diff, "verified": True,
            "sign": sign, "offset": offset, "gripper": grip, "steps": steps,
            "formula": "q_urdf = sign * (q_lerobot - offset), degrees",
        }
        os.makedirs(os.path.dirname(OUT), exist_ok=True)
        with open(OUT, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=1)
        print("\nsign:", sign, "\noffset:", {k: round(v, 2) for k, v in offset.items()}, f"\nwrote {os.path.normpath(OUT)}", flush=True)
    finally:
        bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
