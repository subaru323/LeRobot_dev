"""W3: the puzzle control program (docs/Simulation_Logic_Documents/08 section 5): see -> find -> plan in sim/ -> run, with the wrist check.

usage:
  python tools\\puzzle_run.py --sim [--set 2] [--seed 1] [--air MM] [--pieces N] [--no-wrist] [--shift X,Z] [--calib-mm MM] [--calib-deg DEG] [--arm-err DEG]
  python tools\\puzzle_run.py PORT --k1 FILE --top-cam N --top-intrinsics FILE --top-pose FILE --wrist-cam N --wrist-json FILE
                              [--set 2] [--pieces 1] [--air 40] [--speed 500,20]          (the real arm. NOT RUN YET on the real arm)
  python tools\\puzzle_run.py --make-sim-calib DIR      (writes K5-shaped files of the sim's own cameras, to try the file path with --sim)
  python tools\\puzzle_run.py --cam-selftest            (checks the undistortion to the sim's pinhole, pixel centres included)

  --sim        the world is the simulation (09 "SimRobot"): images are drawn by sim/vision.js from a hidden "true" layout,
               joints are just stored. The same loop and the same planner calls as on the real arm.
  --air MM     add MM to every height (the first real run is in the air: "安全に流す道具の決まり").
               Real arm: below MIN_AIR_REAL_MM only after an earlier real run that ended "ok" in the air (>= MIN_AIR_REAL_MM)
               with the same k1 file, camera files (and k1-err file), --set and code (air_run_ok(): the LATEST such air run in
               the run_*.json logs here must be ok, with the wrist check, no --shift, at least these --pieces, at least this --speed)
  --pieces N   only the first N moves of the plan (K7 / D4: 1). The other pieces are taken to lie at the plan's default
               places (the arm keeps clear of them); only these N are looked for in the top image (sim: only they are drawn)
  --top-cam N, --top-intrinsics FILE, --top-pose FILE   the overhead camera: OpenCV index (cv2.CAP_DSHOW), k5_camera.py
               intrinsics json, k5_camera.py pose json made WITH --k1err: its centre_arm_mm_without_rotation, turned by
               -k1err.shoulder_pan_deg about the pan axis (sim/plan-cli.js; k1-err's pan holds the sheet's turn and the pan zero,
               and the k1 file does not hold it). Real arm: --k1err FILE (that sim/out/k1-err json) is needed: its name and values
               must be the pose's, and its K1 run record must be of the --k1 file. Refused when the pose's reprojection is over
               POSE_REPROJ_MAX_PX or the camera lands over TOP_NOMINAL_TOL_MM from the rig sim/ draws
  --wrist-cam N, --wrist-json FILE   the wrist camera: index, k5_wrist.py solve's wrist_cam_<id>.json (its k1_sha256 must be
               the --k1 file's). Each shot: open, CAM_SETTLE_FRAMES frames to settle (exposure stays automatic; autofocus is
               switched off but the focus position is not set, as in k5_camera.py capture), one frame, undistort to the sim's pinhole
               (same size, f = fy, centre at the middle; vertical FOV from f), save .png (raw) and .rgb (undistorted)
               With --sim the files may be given too: the planner then uses them (the drawn pictures stay the sim's)
  --speed V,A  real arm: the servos' speed (steps/s) and acceleration (x100 steps/s^2) caps, default 500,20, at most 1000,40.
               NOTE (点検 C46 の 5): review C38's 500,20 / 1000,40 are for stretching the planned path in time and streaming it at
               30 Hz with the caps at 1250,254. That is NOT done here: poses go in STEP_DEG steps, each waited for, under these caps
               (slower than C38's 52.6 s; safe side). The log's "trace" keeps each step's goal, reading and time, for the lag
  --no-wrist   skip the wrist check and correction (negative check, 09 section 3)
  --shift X,Z  shift the layout found in the top image by X,Z mm (negative check: the wrist check must bring it back)
  --arm-err D  (sim) the true arm differs from the read angles by up to D deg per joint (seeded); score uses the true pose
  --arm-bias a,b,c,d,e  (sim) the same with these exact values (deg, shoulder_pan .. wrist_roll)
  --board-read (sim) read the empty board in the first top image (sim/plan-cli.js board): how the top camera's reading must be
               turned and moved to lie on the board. Every later top reading (the layout, --place-check) gets the same move (the
               camera does not move). Takes out a top camera placed wrong (--calib-mm, --calib-deg); sim only for now
  --sheet      (sim) the K1/K5 sheet v2 under the board in both images (sim/vision.js sheetTris)
  --sim-missing L,O  (sim) these pieces are planned for but are not on the table (the top camera cannot find them): the run must
               stop, never finish (negative check, tools/s0_run.py)
  --arm-margin MM  every plan also checks each pick (from above the piece until lifted) and each place (from above the board
               until back up) with the jaws, the wrist camera and the held piece moved MM across the table in 8 directions:
               no other piece may be hit (the arm lands off the commanded pose by its own error; sim/plan-cli.js arm_margin_mm).
               Default 0 = off. (sim) every plan also reports how near the TRUE arm (read angles + the arm error) comes to the
               other pieces in those moves (true_bias_deg; not used to decide)
  --place-check N  one piece at a time; after each placing, back at the photo pose, the top camera reads the piece on the
               board. Off its board target by more than PLACE_TOL_MM: pick it up from the board again (plan-cli layout
               on_board) and place it again with the place point moved back by what was seen (plan place_offset; the arm's
               error at the place pose repeats), at most N times per piece (else stop). Placed pieces stay put for the later
               plans (layout keep). Not with --air. (sim) the piece then lies where the true arm put it (score's placed_true),
               and the end score is the true placed position against the board target (< PLACE_MAX_MM) plus each pick's jaws.
               --sim only for now (点検 C49: it cannot be tried in the air first). Stops: read farther than PLACE_REDO_MAX_MM,
               the place point moved more than PLACE_OFFSET_MAX_MM, a piece placed before reads more than PLACE_MOVED_MM
               from where it was accepted. --board-read takes out a top camera placed wrong. The first correction also takes back the pick's own error (e_c, which does not
               repeat); a top camera placed wrong (--calib-mm) moves the pieces by its own error (sim/README)

  --mj         (S2, 2026-11-22; with --sim) the world is MuJoCo (tools/mj_world.py MjRobot): free rigid pieces, the same loop.
               Run with sim\\physics\\.venv\\Scripts\\python.exe (mujoco). The pieces not moved (--pieces) lie at the plan's default
               places. Turns on --end-check and --grip-check. The end score is the physics one (mj_world.score: each moved piece's
               cells within 3 mm of its board target, yaw 5 deg, flat on the table; each piece not moved within 3 mm of where it
               started). Not with --place-check, --sim-missing
  --mj-faults JSON, --mj-occluders JSON  (--mj) world faults and boxes drawn only in some pictures (tools/mj_world.py; not given
               to the control). JSON text or a file
  --gap MM     (S3, 2026-11-23; --sim) the board gap (plan-cli init gap; default 15). With --mj the MuJoCo scene
               sim/out/physics/set<N>_gap<MM>_stand1 must exist (node sim/tools/export-physics.js --set N --gap MM)
  --mj-errors JSON  (S3, 2026-11-23; --mj) world errors not given to the control (tools/mj_world.py: rep_deg, sag_deg_per_nm,
               table_mm, friction, start_dyaw, light). The log gets the contacts (mj.contacts, classified as in 総合計画書 §4)
  --end-check  back at the photo pose, the top camera reads the board: each moved piece within END_PLACE_MM of its board target,
               (S3: the readings are put onto the board read in the first top picture, the board still empty, as --board-read does,
               so that a top camera placed wrong does not move them; without --board-read the picking itself does not use it)
               each other piece that was found in the first top picture within END_MOVED_MM of where it was then (pushed?).
               Else stop. A piece not found then: stop (cannot confirm). Not with plain --sim (SimRobot does not draw placed pieces)
  --grip-check while a piece is held, the gripper must not read more than GRIP_EMPTY_UNITS closed past the planned width
               (empty jaws close by GRIP_SQUEEZE more; stop: missed or dropped), nor, from the third held pose on, more than
               GRIP_WIDE_UNITS wider (stop: the jaws did not close on it)
  --replay RUN.json  (S2) the robot only gives back the observations recorded in that --mj run, in order, and keeps what is
               sent. Same other arguments as the record. The log tells whether the commands are the same as recorded.
               (S4) Also a real-arm record (its "rec"): the record's arguments WITHOUT the port and without --sim; nothing is opened,
               the in-air gate is not asked, the log has "real": false
  --alone      (S4, 2026-11-23; with --pieces N) only those N pieces lie on the table (D4): the end check does not look for the others
  --soma PATH  (2026-11-24; with --sim) build the Soma cube (パズル/, 40 mm cells, stacked in 3D) along PATH, the collision-checked
               path that tools/soma_collide.js --path-json wrote (sim/ is still the only owner of the check: the path is refused
               unless every step passed there). No pictures yet: the pieces are taken to lie where the path file says (the
               person sets them out so). Each pose goes in steps of STEP_DEG as above; the gripper check (--grip-check) runs while a
               piece is held. With --mj the world is tools/soma_world.py (the SO-101 and free 3D pieces in MuJoCo; --mj-errors
               rep_deg, rep_seed, sag_deg_per_nm, friction, table_mm, absent; 2026-11-26: rep_moved_only, cam_sigma_mm, cam_seed) and the end score is each cell within 3 mm of its place in
               the cube (1 mm up/down, tilt 5 deg). Other options are refused with it. Writes sim/out/puzzle_run/soma_<date>.json
               (never taken for a run by air_run_ok). The real arm: not yet (refused)
  --soma-chamfer MM  (--soma --mj) the pieces' cells have their edges cut at 45 deg by MM (tools/soma_mj.py --chamfer-mm)
  --soma-correct (2026-11-26; --soma --mj, a path made with soma_collide.js --photo-q) after each piece the arm goes to the photo
               pose, the top camera reads the top cells of the cube (in the sim: the world's stand-in, soma_world.py measure_tops:
               the true cell centres + noise cam_sigma_mm, 0.5 by default; no picture is drawn or read yet), and each placed piece's
               offset is estimated (pieces covered from above keep their last estimate). The next piece's place is moved toward the
               estimated offset of the pieces it will touch (weighted by touching faces; none: the mean of all), less
               SOMA_FOLLOW_SLACK_MM per axis (the clearance takes that much), and its frames
               are asked from tools/soma_collide.js --serve (the same checks, the pieces in the cube taken at their estimates).
               Stops when a piece is estimated off its place by more than SOMA_OFF_STOP_MM, when the move would be more than
               SOMA_CORRECT_MAX_MM, or when the new frames do not pass

Real arm, added in S4 (2026-11-23; tried on tools/s4_run.py's fake bus and cameras over MuJoCo, not on the real arm):
  - on the table (--air 0) --grip-check and --end-check are on; in the air both are off (nothing held, nothing placed). The end
    check reads WITHOUT the board reading of the first picture (how the real board reads is not known yet: 総合計画書 R0)
  - a camera giving the same frame as its previous shot, byte for byte, stops the run (a frozen camera / driver)
  - at connect, if the motors still hold torque (an earlier run died, the cable was pulled), ask the user to support the arm first.
    Until every motor has read torque off, a stop also asks for support before letting go (a failed read = maybe held: C63)
  - while moving, every motor's Status is read each STATUS_EVERY waits (0.5 s, counted across steps): a protective stop (that
    joint goes limp) stops the run at once instead of 6 s later as "did not reach" (C63)
  - before moving, a piece to be moved that already lies within half a cell of its board target stops the run (a restart on a
    half-done board). In S4 the planner always refused such a layout first ("盤に重なる"); on the real arm that refusal now also
    says to put the pieces back (C63)
  - Operating_Mode and the gripper's stop write retry one lost reply (num_retry 2). The writes in so101_common (limits, gains,
    torque on) still have no retry: one lost reply there stops the run before or right after torque on (S4-04)
  - the log keeps every observation and command ("rec"), and "stop_at" (how many of each when it stopped): --replay above.
    A replay's log is replay_<time>.json (not run_*.json), so it is never taken for a run
  - FAKE_HW (set by tools/s4_run.py) marks the log; air_run_ok() never mixes fake and real logs, and never counts replays
Flow (08 section 5):
  (1)(2) the arm must already be near the plan's start pose (the photo pose); it goes there exactly in small steps,
         then the top image -> sim/plan-cli.js detect -> layout (mm, deg)
  (3)(4) plan-cli plan: joint path + collision check (sim/ is the only owner of the check), converted to LeRobot degrees
         by sim/robot-map.js (also: the conversion goes back exactly, every pose inside the calibrated range)
  (5)    send the poses in steps of at most STEP_DEG
  (6)    at each wrist check (above the piece, before going down): wrist image -> plan-cli wrist -> where the piece really is
         (less than WRIST_MIN_FRAC of its top seen: stop).
         If it is off by more than FIX_TOL_MM, plan again with the corrected layout. The new plan must be the same pose for
         pose up to where this piece's move starts (else stop: the earlier pieces would move differently). The arm goes
         BACK along the poses it came by (already checked) to that point, then follows the new plan (checked). Then the
         wrist check of the new plan looks again. At most MAX_FIX corrections per piece.
  end    (sim) score: did each pick land within the margin of the TRUE piece (same e_a / e_c as sim/tools/vision-loop.js)

Real arm (written 2026-11-20, NOT RUN on the real arm yet): the camera files (K5 top, W4 wrist) are read as above. The
joint side (RealRobot) follows tools/k1_run.py: calibration must match, start near the start pose, limit_follower,
torque_on_in_place (P-001), steps of STEP_DEG with a wait, stop on high load, hold and let the user support the arm.
No truth and no score on the real arm: "ok" = ran to the end without a stop. At the end one more top picture is saved
(the placed piece, to read by eye / later). Not done here: comparing the top picture with the sim's (D4 1.2), the cable.
Writes sim/out/puzzle_run/run_<date>_p<process id>.json, and the pictures in the folder of the same name (sim and real).
"""

import argparse
import datetime
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
SIM = os.path.join(ROOT, "sim")
OUT = os.path.join(SIM, "out", "puzzle_run")
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
STEP_DEG = 2.0          # largest joint change per command (same as tools/k1_run.py)
START_TOL_DEG = 8.0     # the arm must start this close to the plan's start pose
SOMA_OFF_STOP_MM = 3.0      # --soma-correct: stop when the camera puts a placed piece this far off its place (the end score's line)
SOMA_CORRECT_MAX_MM = 3.0   # --soma-correct: the largest move of a place (more: something is wrong, stop)
SOMA_FOLLOW_SLACK_MM = 0.5  # --soma-correct: per axis, this much of the neighbours' offset is left (the clearance takes it): the place
                            # moves by the rest only (following them fully carried their error on: 2026-11-26, rep 0.2 stopped at piece 2)
FIX_TOL_MM = 0.5        # wrist check: correct when the piece is off by more than this (worst cell centre)
MAX_FIX = 2             # corrections per piece
FIX_ACCEPT_MM = 1.5     # S3 (2026-11-23): after MAX_FIX corrections, go on if the wrist still reads at most this off【仮】(else stop).
                        #   Set on the S3 sweep (joint repeatability 0.2-1.0 deg: the reading never settles under FIX_TOL_MM because
                        #   each move lands a little elsewhere). The fingers' margin is 2 mm a side (S1 fit margin)
PLACE_TOL_MM = 2.0      # --place-check: re-place when the top camera sees the piece farther than this from its board target【仮】
                        #   (the top camera's own error on the board: about 0.75 mm mean, 1.8 mm worst at 640x480; sim/README)
PLACE_YAW_GAIN = 0.5    # --place-check: the turn is taken back by half what the top camera read. Its reading of the turn on the board
                        #   is off by 0.49 deg rms in --sim (68 placings after 点検 C49, 2026-11-20), as large as the arm's own turn error there
                        #   (0.53 deg mean): taking all of it back added as much as it removed (seen in the runs before the gain; no saved output). Position: all of it
PLACE_REDO_MAX_MM = 7.5  # --place-check: farther than this off (half the board gap 15 mm) is not re-picked: stop (点検 C49 の 3:
                        #   J 12 mm off passed the half-cell 14.5 mm guard and was re-picked, then the wrist fit failed above it)
PLACE_OFFSET_MAX_DEG = 3.0  # --place-check: nor turned more than this in all (点検 C50 の 7)
PLACE_OFFSET_MAX_MM = 8.0  # --place-check: the place point is never moved farther than this in all (点検 C49 の 3: +11.5 mm was passed on)
PLACE_MOVED_MM = 1.5    # --place-check: a piece already placed must read within this of where it was accepted, in every later
                        #   top picture (点検 C49 の 6: a neighbour pushed while placing). Not yet tried with a piece really pushed (the sim never pushes)
BOARD_MOVED_MM = 1.0    # --board-read: a later reading of the board farther than this (over the board's corners) from the first: stop
                        #   (the top camera moved; 点検 C51 の 6). Read only while the board shows (covered once all pieces are placed)
PLACE_MAX_MM = 3.0      # --sim --place-check: the true placed piece must end within this (plan-cli score PLACE_MAX_MM)
SAME_DEG = 1e-6         # "same pose" when comparing two plans
END_PLACE_MM = 2.5      # --end-check: a moved piece must read within this of its board target【仮】. S3 (2026-11-23): 3.0 (= the
                        #   physics line) let S3-A24 through (read 3.00, physics 3.23: a false completion); 2.0 stopped the default
                        #   board S3-R8 (read 2.02, physics 0.25); 2.5 stops the one and passes the other. Over 178 distinct S3
                        #   readings, reading - physics was -0.71 .. +1.83 mm (the big ones: the piece's turn read 1.1-1.7 deg off);
                        #   pieces within the physics line read up to 3.83 and pieces outside it down to 2.82, so no line keeps both
                        #   sides (point C62): S3 limit, see sim/README
END_MOVED_MM = 1.5      # --end-check: a piece not moved must read within this of where the first top picture had it (PLACE_MOVED_MM)【仮】
GRIP_EMPTY_UNITS = 3.0  # --grip-check: closed more than this (0-100) past the planned width while held = nothing in the jaws【仮】
                        #   (S2 2026-11-22, after the world waits 0.05 s per step: held 0.25-0.27 past; empty 5.0-6.0. Set on the graded cases)
WRIST_MIN_FRAC = 0.5    # wrist check: less than this part of the piece's top face seen (plan-cli wrist top_frac) = hidden: stop【仮】
                        #   (S2 2026-11-22: a card over the piece, the estimate 3 mm off in 5 ways: read up to 13.1 mm wrong at 0.08,
                        #   3.2 mm at 0.14, 1.2 mm at 0.25-0.29, at most 0.9 mm from 0.34 up; 0.31 mm with almost nothing hidden (0.94).
                        #   Not hidden: L 0.99-1.00, J 0.80 (scratchpad wrist_frac_sweep))
GRIP_WIDE_UNITS = 3.0   # --grip-check: from the third held pose on, wider than the planned width by more than this = the jaws did not
                        #   close on the piece (blocked, too weak, on a corner)【仮】 (tools/puzzle_mj.py GRIP_WIDE_DEG)
# real arm only
REACH_TOL_DEG = 1.5
STEP_TIMEOUT_S = 6.0
STATUS_EVERY = 10              # real arm: read every motor's Status each 10 waits (0.5 s) while moving (点検 C63 の 6)
LOAD_ABORT = 600
GRIP_LOAD_STOP = 350    # gripper: stop closing above this load (do not squeeze, 安全に流す道具の決まり)
GRIP_SQUEEZE = 5.0      # gripper: when closing, aim this much (0-100 units) past the planned width and stop on load
MIN_AIR_REAL_MM = 40.0  # real arm: lift at least this much until the in-air runs are done (安全に流す道具の決まり; +50 does not reach)
MIN_TIP_MM = 1.0        # any plan: the finger tips must stay this far above the table (sim/ minTip)
SPEED_DEFAULT = (500, 20)   # real arm: Goal_Velocity steps/s, Acceleration x100 steps/s^2 as CAPS (the numbers of review C38's
SPEED_MAX = (1000, 40)      #   first air run / later runs, but C38 meant a timed 30 Hz stream; see --speed). Refused above
CAM_SETTLE_FRAMES = 30      # frames read and thrown away after opening a camera (exposure / white balance settle)
FAKE_HW = False             # S4 (2026-11-23): tools/s4_run.py sets this when the "real arm" is its fake bus and fake cameras. The log
                            #   says so, and air_run_ok() counts only logs of the same kind (a fake run never opens the real table)
TOP_NOMINAL_MM = (157.3, 540.0, 19.5)   # so101.js OVERHEAD_C.lensTipMm (the rig the collision check draws)
TOP_NOMINAL_TOL_MM = 15.0   # 【仮】 the solved top camera farther than this from the drawing: the rig is not where the check thinks
POSE_REPROJ_MAX_PX = 2.0    # k5_camera.py warns above this
PAN_AXIS_X_MM = 38.8353     # so101.js: shoulder_pan's axis (vertical) at x 38.8, z 0 (only for the TOP_NOMINAL check; plan-cli turns the camera)
SIM_CODE = ["plan-cli.js", "scene-data.js", "robot-map.js", "so101.js", "motion.js", "puzzle.js", "kinematics.js", "collide.js",
            "self-collision.js", "chikuwa-collision.js", "vision.js", "vision-wrist.js",
            "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "tools/_load.js"]   # with this file: code_sha256
#   in the log. The models/ and _load.js were added 2026-11-22 (点検 C58 の 5: plan-cli loads them; they shape the images and the hits)


class Stop(Exception):
    pass


def file_sha(path: str) -> str:
    import hashlib
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def pinhole_K(K, size):
    """The sim's pinhole for a camera: same size, f = fy, principal point at the picture's middle. OpenCV's pixel i is
    centred at i, the sim's pixel i spans i..i+1 (so101.js overheadImage), so the middle W/2 is W/2 - 0.5 in OpenCV."""
    W, H = size
    f = float(K[1][1])
    return [[f, 0.0, W / 2 - 0.5], [0.0, f, H / 2 - 0.5], [0.0, 0.0, 1.0]], 2 * math.degrees(math.atan(H / 2 / f))


def undistort_maps(K, dist, size, newK):
    import cv2
    import numpy as np
    return cv2.initUndistortRectifyMap(np.array(K, np.float64), np.array(dist, np.float64), None, np.array(newK, np.float64),
                                       tuple(size), cv2.CV_32FC1)


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


def load_calib(a, k1_sha) -> tuple:
    """The camera files -> (calib for plan-cli init, {name: undistortion for the shots}, {file: sha256}).
    calib.top: tip [m] (table = 0, the arm's x/z), dir, up, fov. calib.wrist: t [m], R (9, row-major), fov, or None."""
    cams, shas = {}, {}
    for k in ("top_intrinsics", "top_pose"):
        if not getattr(a, k):
            raise Stop(f"--{k.replace('_', '-')} is needed")
    with open(a.top_intrinsics, encoding="utf-8") as fh:
        ti = json.load(fh)
    with open(a.top_pose, encoding="utf-8") as fh:
        tp = json.load(fh)
    shas.update({os.path.basename(a.top_intrinsics): file_sha(a.top_intrinsics), os.path.basename(a.top_pose): file_sha(a.top_pose)})
    if not tp.get("ok"):
        raise Stop(f"{a.top_pose}: the pose solve did not succeed")
    if tp.get("intrinsics") != os.path.basename(a.top_intrinsics):
        raise Stop(f"{a.top_pose} was solved with intrinsics {tp.get('intrinsics')}, not {os.path.basename(a.top_intrinsics)}")
    c = tp.get("centre_arm_mm_without_rotation")
    if not c:
        raise Stop(f"{a.top_pose} has no centre_arm_mm_without_rotation: run k5_camera.py pose again WITH --k1err "
                   "(the sheet frame is a few mm off the arm's)")
    k1e = tp.get("k1err") or {}
    if k1e.get("shoulder_pan_deg") is None:
        raise Stop(f"{a.top_pose} has no k1err.shoulder_pan_deg (no --k1err, or k1-err could not determine shoulder_pan): "
                   "the top camera cannot be put in the arm's frame")
    if k1_sha is not None:                       # the real arm: the k1-err file must come from a K1 run of THIS k1 file (点検 C46 の 4)
        if not a.k1err:
            raise Stop("real arm: --k1err FILE is needed (the sim/out/k1-err/<run>.json the pose was solved with)")
        with open(a.k1err, encoding="utf-8") as fh:
            ke = json.load(fh)
        shas[os.path.basename(a.k1err)] = file_sha(a.k1err)
        if os.path.basename(a.k1err) != k1e.get("file"):
            raise Stop(f"{a.top_pose} was solved with k1-err {k1e.get('file')}, not {os.path.basename(a.k1err)}")
        pp, pan = ke.get("paper") or {}, (ke.get("joints") or [{}])[0].get("deg")
        if [pp.get("dx_mm"), pp.get("dz_mm"), pp.get("scale_permil"), pan] != \
                [k1e.get("dx_mm"), k1e.get("dz_mm"), k1e.get("scale_permil"), k1e.get("shoulder_pan_deg")]:
            raise Stop(f"{a.k1err}: its paper / shoulder_pan values are not the ones in {a.top_pose}")
        try:
            with open(ke["run"], encoding="utf-8") as fh:
                run_sha = json.load(fh).get("k1_sha256")
        except (OSError, ValueError, KeyError) as e:
            raise Stop(f"{a.k1err}: cannot read its K1 run record ({e!r})")
        if run_sha != k1_sha:
            raise Stop(f"{a.k1err} comes from a K1 run of another k1 file ({str(run_sha)[:12]}, --k1 is {k1_sha[:12]}): "
                       "run K1 and k5_camera.py pose again with this k1 file")
    if float(tp.get("reproj_px_max", 0.0)) > POSE_REPROJ_MAX_PX:
        raise Stop(f"{a.top_pose}: reprojection {tp['reproj_px_max']:.2f} px > {POSE_REPROJ_MAX_PX} (sheet not flat, a marker misread, "
                   "or intrinsics of another mode)")
    # where the planner will put the camera (turned by -pan about the pan axis, as plan-cli does), against the rig drawn in sim/
    th = math.radians(float(k1e["shoulder_pan_deg"]))
    px, pz = c[0] - PAN_AXIS_X_MM, c[2]                 # the pan axis is vertical (URDF z = screen -y); -pan about -y = +pan about +y
    cc = [PAN_AXIS_X_MM + px * math.cos(th) + pz * math.sin(th), c[1], -px * math.sin(th) + pz * math.cos(th)]
    off = math.dist(cc, TOP_NOMINAL_MM)
    if off > TOP_NOMINAL_TOL_MM:
        raise Stop(f"the top camera is {off:.1f} mm from where sim/ draws it {TOP_NOMINAL_MM} (solved {[round(v, 1) for v in cc]}): "
                   f"the collision check would use the wrong rig. Over {TOP_NOMINAL_TOL_MM} mm")
    newK, fov = pinhole_K(ti["K"], ti["image_size"])
    cams["top"] = {"K": ti["K"], "dist": ti["dist"], "size": ti["image_size"], "newK": newK}
    # the sheet frame -> the planner's: (P - d) / (1 + scale) is done by k5_camera.py (centre_arm_mm_without_rotation); the turn
    # by -shoulder_pan (k1-err's, the sheet's turn and the pan zero together; the k1 file does not hold it) is done by plan-cli
    calib = {"top": {"tip": [v / 1000 for v in c], "dir": _unit(tp["dir"]), "up": _unit(tp["up"]), "fov": fov,
                     "panDeg": float(k1e["shoulder_pan_deg"])}, "wrist": None}
    if a.wrist_json:
        with open(a.wrist_json, encoding="utf-8") as fh:
            wj = json.load(fh)
        shas[os.path.basename(a.wrist_json)] = file_sha(a.wrist_json)
        if k1_sha is not None and wj.get("k1_sha256") != k1_sha:
            raise Stop(f"{a.wrist_json} was solved with another k1 file (k1_sha256 {str(wj.get('k1_sha256'))[:12]}, --k1 is {k1_sha[:12]}): "
                       "run k5_wrist.py solve again with this k1 file")
        newKw, fovw = pinhole_K(wj["K"], wj["image_size"])
        cams["wrist"] = {"K": wj["K"], "dist": wj["dist"], "size": wj["image_size"], "newK": newKw}
        g = wj["cam_in_gripper"]
        calib["wrist"] = {"t": [v / 1000 for v in g["t_mm"]], "R": list(g["R"]), "fov": fovw}
    return calib, cams, shas


class Camera:
    """One USB camera, opened for each shot (cv2.CAP_DSHOW, MJPG, the intrinsics' size, autofocus off; exposure left
    to the camera). The frame is undistorted to the sim's pinhole (pinhole_K) and written as raw RGB for plan-cli."""

    def __init__(self, index: int, cal: dict, folder: str, name: str):
        self.index, self.cal, self.folder, self.name, self.n, self.maps = index, cal, folder, name, 0, None
        self.last_sha = None        # S4: the previous frame's sha256 (a real camera never gives the same bytes twice)

    def shot(self) -> dict:
        import cv2
        W, H = self.cal["size"]
        cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            frame = None
            for _ in range(CAM_SETTLE_FRAMES + 1):
                ok, frame = cap.read()
                if not ok:
                    raise Stop(f"{self.name} camera {self.index}: cannot read a frame")
        finally:
            cap.release()
        if (frame.shape[1], frame.shape[0]) != (W, H):
            raise Stop(f"{self.name} camera {self.index} gave {frame.shape[1]}x{frame.shape[0]}, the intrinsics are for {W}x{H}")
        import hashlib
        sha = hashlib.sha256(frame.tobytes()).hexdigest()
        if sha == self.last_sha:    # S4: a frozen camera or driver hands back the old picture (sensor noise changes every real frame)
            raise Stop(f"{self.name} camera {self.index}: the same picture as the shot before (byte for byte): the camera may be frozen; "
                       "not trusting an old picture")
        self.last_sha = sha
        if self.maps is None:
            self.maps = undistort_maps(self.cal["K"], self.cal["dist"], (W, H), self.cal["newK"])
        self.n += 1
        os.makedirs(self.folder, exist_ok=True)
        stem = os.path.join(self.folder, f"{self.n:02d}_{self.name}")
        cv2.imwrite(stem + ".png", frame)
        und = cv2.cvtColor(cv2.remap(frame, self.maps[0], self.maps[1], cv2.INTER_LINEAR), cv2.COLOR_BGR2RGB)
        with open(stem + ".rgb", "wb") as fo:
            fo.write(und.tobytes())
        return {"out": stem + ".rgb", "W": W, "H": H, "png": stem + ".png", "sha256": sha, "t": round(time.time(), 3)}


def code_sha() -> str:
    """One sha256 over this file and the sim/ files the planner loads: an in-air run proves THIS code (点検 C46 の 3)."""
    import hashlib
    h = hashlib.sha256()
    for f in [os.path.abspath(__file__)] + [os.path.join(SIM, n) for n in SIM_CODE]:
        with open(f, "rb") as fh:
            h.update(fh.read())
    return h.hexdigest()


def air_run_ok(k1_sha: str, shas: dict, a, code: str, speed: tuple) -> str:
    """'' if the LATEST real in-air run (air >= MIN_AIR_REAL_MM) with the same k1 file, camera files, --set and code ended
    "ok", used the wrist check, had no --shift, covered at least the pieces asked now and ran at least this fast;
    else why not (点検 C46 の 3: one old good run must not let a table run through after a later air run stopped)."""
    import glob
    runs = []
    for f in sorted(glob.glob(os.path.join(OUT, "run_*.json"))):
        try:
            with open(f, encoding="utf-8") as fh:
                r = json.load(fh)
        except (OSError, ValueError):
            continue
        ra = r.get("args", {})
        if bool(r.get("fake_hardware")) != FAKE_HW or r.get("replay_of"):    # S4: fake-bus runs and replays never count for the real arm
            continue
        if (r.get("real") and r.get("k1_sha256") == k1_sha and r.get("calib_sha256") == shas and ra.get("set") == a.set
                and float(ra.get("air", 0)) >= MIN_AIR_REAL_MM):
            runs.append((f, r))
    # the latest = by the start time in the log, then the file time (not the name: "_p20212" sorts before "_p6880". 点検 C47 の 6)
    runs.sort(key=lambda fr: (str(fr[1].get("date", "")), os.path.getmtime(fr[0])))
    head = (f"in-air real runs (>= {MIN_AIR_REAL_MM:.0f} mm) with this k1 file, these camera files and --set {a.set}")
    if not runs:
        return "no " + head
    f, r = runs[-1]
    ra, why = r.get("args", {}), []
    if r.get("result") != "ok":
        why.append(f"result {r.get('result')}")
    if r.get("code_sha256") != code:
        why.append("the code (tools/puzzle_run.py or sim/) changed since")
    if ra.get("no_wrist"):
        why.append("it ran --no-wrist")
    if ra.get("shift"):
        why.append(f"it ran --shift {ra.get('shift')}")
    if ra.get("pieces") and (not a.pieces or a.pieces > ra.get("pieces")):
        why.append(f"it ran --pieces {ra.get('pieces')}, now {a.pieces or 'all'}")
    sp = r.get("speed") or [0, 0]
    if speed[0] > sp[0] or speed[1] > sp[1]:
        why.append(f"it ran --speed {sp[0]},{sp[1]}, now {speed[0]},{speed[1]}")
    if not why:
        return ""
    return f"the latest of the {head} is {os.path.basename(f)}: " + "; ".join(why)


class Planner:
    """sim/plan-cli.js as a child process: one JSON line in, one JSON line out."""

    def __init__(self):
        self.p = subprocess.Popen(["node", os.path.join(SIM, "plan-cli.js")], cwd=SIM, stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, text=True, encoding="utf-8", bufsize=1)

    def ask(self, cmd: str, **kw) -> dict:
        self.p.stdin.write(json.dumps(dict(cmd=cmd, **kw)) + "\n")
        self.p.stdin.flush()
        line = self.p.stdout.readline()
        if not line:
            raise Stop(f"plan-cli ended (cmd {cmd})")
        return json.loads(line)

    def need(self, cmd: str, **kw) -> dict:
        r = self.ask(cmd, **kw)
        if not r.get("ok"):
            raise Stop(f"{cmd}: {r.get('why')}")
        return r

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(timeout=5)
        except Exception:  # noqa: BLE001
            self.p.kill()


class SimRobot:
    """09 'SimRobot': the same three calls as the real arm (read joints, send joints, read a camera)."""
    torque = False

    def __init__(self, planner: Planner, truth: dict, start: dict, bias: list, absent=(), folder: str = OUT):
        self.pl, self.truth, self.folder = planner, truth, folder
        self.L = dict(start)           # LeRobot degrees, gripper 0-100 (what the encoders read)
        self.bias = bias               # URDF degrees per joint: the true arm = read angles + bias (--arm-err; K1 measures this kind of error)
        self.hide = set(absent)        # pieces picked up (not drawn any more), and with --pieces the ones not on the table
        self.n = 0

    def connect_no_torque(self):
        pass

    def read_joints(self) -> dict:
        return dict(self.L)

    def limit_and_torque_on_in_place(self):
        self.torque = True

    def send(self, target: dict):
        self.L.update(target)

    def read_camera(self, name: str) -> dict:
        self.n += 1
        out = os.path.join(self.folder, f"shot_{self.n:02d}_{name}.rgb")
        return self.pl.need("render", cam=name, truth=self.truth, hide=sorted(self.hide), lerobot=self.L, bias=self.bias, out=out)

    def hold(self):
        pass

    def release(self):
        self.torque = False


class ReplayRobot:
    """--replay (S2): gives back the observations of a recorded --mj run in order (joint readings and pictures), keeps what is
    sent. With the same observations the control must send the same commands; the truth is not here at all."""
    torque = False

    def __init__(self, record: dict):
        self.seq = list((record.get("mj") or record.get("rec"))["obs"])     # "rec": a real-arm record (S4)
        self.sent = []

    def _next(self, kind: str, name=None):
        if not self.seq:
            raise Stop(f"replay: no observation left (the control asked for {kind} {name or ''})")
        o = self.seq.pop(0)
        if o["k"] != kind or (name is not None and o.get("name") != name):
            raise Stop(f"replay: the control asked for {kind} {name or ''} where the record has {o['k']} {o.get('name') or ''}")
        return o["v"]

    def connect_no_torque(self):
        pass

    def limit_and_torque_on_in_place(self):
        self.torque = True

    def read_joints(self) -> dict:
        return dict(self._next("read"))

    def send(self, target: dict):
        self.sent.append(dict(target))

    def read_camera(self, name: str) -> dict:
        return dict(self._next("cam", name))

    def hold(self):
        pass

    def release(self):
        self.torque = False


class Recorder:
    """--mj: wraps the robot and keeps every observation given to the control and every command sent (for --replay)."""

    def __init__(self, robot):
        self.robot, self.obs, self.sent = robot, [], []

    # only these pass through (review C60 3): truth = the world at each pick, written to the log only (never read back:
    #   --place-check is refused with --mj); score = the physics score, after the control has ended. The four commands that
    #   return nothing (connect, torque on, hold, release) pass too (ReplayRobot has the same four). Nothing else of the world
    ALLOW = ("truth", "score", "connect_no_torque", "limit_and_torque_on_in_place", "hold", "release", "trace")   # trace: the real arm's steps (S4)

    def __getattr__(self, k):
        if k in Recorder.ALLOW:
            return getattr(self.robot, k)
        raise AttributeError(k)

    def read_joints(self) -> dict:
        v = self.robot.read_joints()
        self.obs.append({"k": "read", "v": dict(v)})
        return v

    def read_camera(self, name: str) -> dict:
        v = self.robot.read_camera(name)
        self.obs.append({"k": "cam", "name": name, "v": dict(v)})
        return v

    def send(self, target: dict):
        self.sent.append(dict(target))
        self.robot.send(target)


class RealRobot:
    """The real arm, written after tools/k1_run.py, and its two cameras (Camera). NOT RUN on the real arm yet."""

    def __init__(self, port, calib_key: str, cameras: dict, speed: tuple):
        from so101_common import FOLLOWER_PORT, make_follower
        self.robot = make_follower(port or FOLLOWER_PORT)
        self.bus = self.robot.bus
        self.calib_key = calib_key
        self.cameras = cameras
        self.speed = speed
        self.torque = False
        self.n_wait = 0                            # waits in send() since connect (the Status check every STATUS_EVERY)
        self.maybe_held = False                    # connect_no_torque: True while it is not known that the motors hold nothing
        self.grip_plan = self.grip_goal = None    # last planned / last commanded gripper value (grip_goal_for)
        self.grip_stopped = False                  # the load stopped the closing; hold there until the plan opens
        self.trace = []                            # each step: goal, reading when reached, how long it took (D4 2.6 reads it)
        self.t0 = time.time()

    def connect_no_torque(self):
        from k1_run import calib_key
        from so101_common import check_motors_ok
        # S4 (2026-11-23): so101_common.connect_bus_no_torque, with one step between: after a run that did not end cleanly (the PC
        #   or this program died, the cable was pulled) the motors may still hold the arm. Turning torque off then drops it: ask first
        self.maybe_held = True       # 点検 C63 の 1: until every motor reads torque off, release() asks for support too
        self.bus.connect()
        held = [m for m in self.bus.motors if self.bus.read("Torque_Enable", m, normalize=False, num_retry=2)]
        if not held:
            self.maybe_held = False
        if held:
            print(f"!! the motors still hold the arm ({', '.join(held)}): an earlier run did not end cleanly.", flush=True)
            while True:
                try:
                    input("SUPPORT THE ARM with your hand, then press Enter (torque goes off; this run then checks the start pose)...")
                    break
                except (KeyboardInterrupt, EOFError):
                    print("\n(the arm is still held. Support it, then press Enter.)", flush=True)
                    time.sleep(2)
        self.bus.disable_torque(num_retry=3)
        self.maybe_held = False
        self.bus.configure_motors()
        for m in self.bus.motors:
            self.bus.write("Operating_Mode", m, 0, num_retry=2)      # 0 = position mode (点検 C63 の 3: one lost reply is retried)
        check_motors_ok(self.bus, "follower")
        if calib_key(self.bus.calibration) != self.calib_key:
            raise Stop("the calibration in the motors is not the one in the k1 file -> run k1_calib.py again")

    def read_joints(self) -> dict:
        return dict(self.bus.sync_read("Present_Position", num_retry=3))

    def limit_and_torque_on_in_place(self):
        from so101_common import limit_follower, torque_on_in_place
        limit_follower(self.bus, speed=self.speed[0], accel=self.speed[1], torque_limit=700)
        input("Torque will turn ON and the arm will hold this pose. Keep a hand near the arm. Press Enter...")
        self.torque = True
        torque_on_in_place(self.bus)
        time.sleep(0.5)

    def send(self, target: dict):
        """Joints: wait until reached (REACH_TOL_DEG), stop on high load. Gripper (安全に流す道具の決まり): when closing, aim
        GRIP_SQUEEZE past the planned width (the planned width alone gives no grip) and stop closing as soon as its load
        passes GRIP_LOAD_STOP (Torque_Limit 500 is set by limit_follower). The gripper is not waited for."""
        from so101_common import max_load
        goal = dict(target)
        if "gripper" in goal:
            goal["gripper"] = self.grip_goal_for(goal["gripper"])
        self.bus.sync_write("Goal_Position", goal)
        t0 = time.time()
        # (the wait count runs on across steps: a short step must not skip the check)
        while True:
            time.sleep(0.05)
            self.n_wait += 1
            pos = self.bus.sync_read("Present_Position", num_retry=2)
            if self.n_wait % STATUS_EVERY == 0 or time.time() - t0 > STEP_TIMEOUT_S:   # 点検 C63 の 6: a motor that stopped itself goes limp
                bad = {m: s for m in self.bus.motors if (s := self.bus.read("Status", m, normalize=False, num_retry=2))}
                if bad:
                    raise Stop(f"motor error status {bad} while moving (a protective stop: that joint has no torque now)")
            if "gripper" in goal and goal["gripper"] < pos["gripper"] + 1e-6 and \
                    abs(self.bus.read("Present_Load", "gripper", normalize=False, num_retry=2)) > GRIP_LOAD_STOP:
                self.bus.write("Goal_Position", "gripper", pos["gripper"], num_retry=2)   # stop closing here, keep it until the plan opens
                goal["gripper"] = self.grip_goal = pos["gripper"]
                self.grip_stopped = True
            m, load = max_load(self.bus)
            if m != "gripper" and load > LOAD_ABORT:
                raise Stop(f"high load on {m}: {load}")
            if max(abs(pos[j] - goal[j]) for j in JOINTS) < REACH_TOL_DEG:
                self.trace.append({"t": round(time.time() - self.t0, 3), "dt": round(time.time() - t0, 3),
                                   "goal": {k: round(v, 2) for k, v in goal.items()}, "pos": {k: round(v, 2) for k, v in pos.items()}})
                return
            if time.time() - t0 > STEP_TIMEOUT_S:
                raise Stop(f"did not reach the step in {STEP_TIMEOUT_S}s")

    def grip_goal_for(self, planned: float) -> float:
        """The gripper goal for a planned value, from the PLANNED values (not the reading: point C46 2). Closing (planned
        below the last planned): aim GRIP_SQUEEZE deeper, or stay where the load stopped it. Same planned value: keep the
        last goal (do not loosen a grip, do not close an open jaw). Opening: the planned value, and forget the load stop."""
        prev = self.grip_plan
        if prev is None or planned > prev + 1e-6:
            self.grip_stopped = False
            g = planned
        elif planned < prev - 1e-6:
            g = self.grip_goal if self.grip_stopped else planned - GRIP_SQUEEZE
        else:
            g = self.grip_goal
        self.grip_plan, self.grip_goal = planned, g
        return g

    def read_camera(self, name: str) -> dict:
        if name not in self.cameras:
            raise Stop(f"no {name} camera given")
        return self.cameras[name].shot()

    def hold(self):
        try:
            pos = self.bus.sync_read("Present_Position", num_retry=3)
            self.bus.sync_write("Goal_Position", {j: pos[j] for j in JOINTS + ["gripper"]})   # the gripper too (stop squeezing)
        except Exception:  # noqa: BLE001
            pass

    def release(self):
        from so101_common import release_all
        if self.torque or self.maybe_held:
            while True:
                try:
                    input("SUPPORT THE ARM with your hand, then press Enter to release torque...")
                    break
                except (KeyboardInterrupt, EOFError):
                    print("\n(the arm is still held. Support it, then press Enter.)", flush=True)
                    time.sleep(2)
        try:
            release_all(self.bus)
        finally:
            self.bus.disconnect(disable_torque=False)


def steps_between(a: dict, b: dict, keys) -> list:
    """Straight line in joint space (what the sim checks between two poses), cut so no joint moves more than STEP_DEG."""
    n = max(1, math.ceil(max(abs(b[k] - a[k]) for k in keys) / STEP_DEG - 1e-9))
    return [{k: a[k] + (b[k] - a[k]) * i / n for k in keys} for i in range(1, n + 1)]


def first_diff(fa: list, fb: list) -> int:
    """Index of the first pose that differs between two plans (len if one is a prefix of the other)."""
    for i in range(min(len(fa), len(fb))):
        a, b = fa[i], fb[i]
        if abs(a["t"] - b["t"]) > 1e-9 or abs(a["jaw"] - b["jaw"]) > SAME_DEG or any(abs(x - y) > SAME_DEG for x, y in zip(a["q"], b["q"])):
            return i
    return min(len(fa), len(fb))


def make_sim_calib(folder: str) -> None:
    """K5-shaped files of the sim's own cameras (the ones sim/vision.js draws with): to run the file path with --sim."""
    pl = Planner()
    try:
        pl.need("init", set=2, k1="fake", calib="sim")
        c = pl.need("simcalib")
    finally:
        pl.close()
    os.makedirs(folder, exist_ok=True)
    t, w = c["top"], c["wrist"]

    def K_of(cam):
        f = (cam["H"] / 2) / math.tan(math.radians(cam["fov"]) / 2)
        return [[f, 0.0, cam["W"] / 2 - 0.5], [0.0, f, cam["H"] / 2 - 0.5], [0.0, 0.0, 1.0]]
    note = "made by puzzle_run.py --make-sim-calib from the sim's cameras (not a real calibration)"
    ti = os.path.join(folder, f"intrinsics_top_sim_{t['W']}x{t['H']}.json")
    files = {ti: {"image_size": [t["W"], t["H"]], "rms_px": 0.0, "K": K_of(t), "dist": [0.0] * 5, "note": note},
             os.path.join(folder, "pose_top_sim.json"): {
                 "ok": True, "centre_mm": [v * 1000 for v in t["tip"]], "centre_arm_mm_without_rotation": [v * 1000 for v in t["tip"]],
                 "dir": t["dir"], "up": t["up"], "intrinsics": os.path.basename(ti), "k1err": {"file": "(sim)", "shoulder_pan_deg": 0.0},
                 "note": note},
             os.path.join(folder, "wrist_cam_sim.json"): {
                 "image_size": [w["W"], w["H"]], "K": K_of(w), "dist": [0.0] * 5, "robot_id": "fake", "k1_sha256": None,
                 "cam_in_gripper": {"t_mm": [v * 1000 for v in w["t"]], "R": w["R"]}, "note": note}}
    for f, j in files.items():
        with open(f, "w", encoding="utf-8") as fo:
            json.dump(j, fo, indent=1)
        print("wrote", os.path.normpath(f))


def cam_selftest() -> bool:
    """The undistortion map must send each pixel centre of the sim's pinhole to where OpenCV projects the same ray
    through the real K and dist. Negative: with the principal point at W/2 (no half pixel) it must be off by 0.5 px."""
    import cv2
    import numpy as np
    K = [[905.0, 0.0, 951.3], [0.0, 902.0, 548.9], [0.0, 0.0, 1.0]]
    dist = [-0.12, 0.05, 0.0005, -0.0003, 0.0]
    W, H = 1920, 1080
    newK, fov = pinhole_K(K, (W, H))

    def worst(nk):
        mx, my = undistort_maps(K, dist, (W, H), nk)
        f = newK[1][1]
        ii, jj = np.meshgrid(np.arange(100, W - 100, 97), np.arange(100, H - 100, 89))
        ray = np.stack([(ii + 0.5 - W / 2) / f, (jj + 0.5 - H / 2) / f, np.ones(ii.shape)], -1).reshape(-1, 1, 3)
        pr, _ = cv2.projectPoints(ray.astype(np.float64), np.zeros(3), np.zeros(3), np.array(K), np.array(dist))
        got = np.stack([mx[jj, ii], my[jj, ii]], -1).reshape(-1, 2)
        return float(np.abs(pr.reshape(-1, 2) - got).max())
    e = worst(newK)
    bad = [[newK[0][0], 0.0, W / 2], [0.0, newK[1][1], H / 2], [0.0, 0.0, 1.0]]
    e2 = worst(bad)
    f_back = (H / 2) / math.tan(math.radians(fov) / 2)
    ok = e < 0.02 and 0.45 < e2 < 0.55 and abs(f_back - K[1][1]) < 1e-6
    print(f"undistortion to the sim's pinhole: worst {e:.4f} px (pass < 0.02); negative (centre at W/2): {e2:.3f} px (expect 0.5); "
          f"vertical FOV {fov:.3f} deg -> f {f_back:.3f} (fy {K[1][1]}) -> {'OK' if ok else 'NG'}")
    return ok


def board_moved(t1: dict, t2: dict) -> float:
    """How far apart two board readings (plan-cli board) put the board's corners (mm): the farthest of the 4 corners of a 200 mm
    square around the board centre."""
    def ap(t, x, z):
        c, s = math.cos(math.radians(t["dyaw_deg"])), math.sin(math.radians(t["dyaw_deg"]))
        X, Z = x - t["about_mm"][0], z - t["about_mm"][1]
        return c * X - s * Z + t["about_mm"][0] + t["dx_mm"], s * X + c * Z + t["about_mm"][1] + t["dz_mm"]
    cx, cz = t1["about_mm"]
    return max(math.dist(ap(t1, cx + dx, cz + dz), ap(t2, cx + dx, cz + dz)) for dx in (-100, 100) for dz in (-100, 100))


class SomaServer:
    """tools/soma_collide.js --serve as a child process (--soma-correct): one piece's checked frames per request."""

    def __init__(self, path: dict, errlog: str):
        if not path.get("photo_q") or not path.get("plan_file"):
            raise SystemExit("--soma-correct: the path needs photo poses (make it with tools/soma_collide.js --photo-q ... --path-json)")
        args = ["node", os.path.join(ROOT, "tools", "soma_collide.js"), path["plan_file"], path["reach_file"],
                "--raise-mm", str(path["raise_mm"]), "--cx-mm", str(path["cube_centre_from_pan_mm"]), "--clear-mm", str(path["clear_mm"]),
                "--photo-q", ",".join(str(v) for v in path["photo_q"]), "--serve"]
        self.err = open(errlog, "w", encoding="utf-8")
        self.p = subprocess.Popen(args, cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.err, text=True,
                                  encoding="utf-8", bufsize=1)

    def piece(self, **kw) -> dict:
        self.p.stdin.write(json.dumps(kw) + "\n")
        self.p.stdin.flush()
        line = self.p.stdout.readline()
        if not line:
            raise Stop("soma_collide.js --serve ended")
        r = json.loads(line)
        if not r.get("ok"):
            raise Stop(f"soma_collide.js --serve: {r.get('why')}")
        return r

    def close(self):
        try:
            self.p.stdin.close()
            self.p.wait(timeout=5)
        except Exception:  # noqa: BLE001
            self.p.kill()
        self.err.close()


def soma_touching(path: dict, n: int) -> dict:
    """pieces placed before n that piece n touches in the cube: {k: faces shared}"""
    C = path["cell_m"]
    mine = path["pieces"][n]["cube_cells"]
    out = {}
    for k in range(n):
        for c in path["pieces"][k]["cube_cells"]:
            for m in mine:
                if abs(sum(abs(a - b) for a, b in zip(c, m)) - C) < 1e-6:
                    out[k] = out.get(k, 0) + 1
    return out


class JointsOnly:
    """--soma without --mj: the joints are just stored (as SimRobot, without pictures)."""
    torque = False

    def __init__(self, start: dict):
        self.L = dict(start)

    def connect_no_torque(self):
        pass

    def read_joints(self) -> dict:
        return dict(self.L)

    def limit_and_torque_on_in_place(self):
        self.torque = True

    def send(self, target: dict):
        self.L.update(target)

    def read_camera(self, name: str) -> dict:
        raise Stop("soma: no pictures")

    def hold(self):
        pass

    def release(self):
        self.torque = False


def run_soma(a, ap) -> None:
    """--soma PATH (2026-11-24): the Soma build along a collision-checked path (see the usage). The same robot calls as the
    flat puzzle (read joints, send joints in STEP_DEG steps), the same gripper check; no pictures, no corrections yet."""
    allowed = {"soma", "soma_chamfer", "soma_correct", "sim", "mj", "mj_errors", "seed", "arm_err", "arm_bias", "grip_check", "end_check"}
    given = [k for k, v in vars(a).items() if k not in allowed and v != ap.get_default(k)]
    if given:
        raise SystemExit("--soma: not with " + ", ".join("--" + k.replace("_", "-") for k in given) + " (yet)")
    if not a.sim:
        raise SystemExit("--soma: --sim only for now (the real arm needs its in-air run of the 3D build first; not written)")
    if a.mj_errors and not a.mj:
        raise SystemExit("--mj-errors is --mj only")
    if a.soma_correct and not a.mj:
        raise SystemExit("--soma-correct: --mj only (the camera is the MuJoCo world's stand-in for now)")
    path = json.load(open(a.soma, encoding="utf-8"))
    bad = [i + 1 for i, v in enumerate(path.get("verdicts", [])) if v not in ("ok", "near")]
    if bad or path.get("self_verdict") != "ok" or not path.get("frames"):
        raise SystemExit(f"--soma: the path did not pass tools/soma_collide.js (steps {bad}, self {path.get('self_verdict')}): refused")
    a.grip_check = True
    os.makedirs(OUT, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + f"_p{os.getpid()}"
    log = {"date": datetime.datetime.now().isoformat(timespec="seconds"), "args": vars(a), "real": False, "code_sha256": code_sha(),
           "soma_path": {"file": os.path.abspath(a.soma), "sha256": file_sha(a.soma), "raise_mm": path["raise_mm"],
                         "cube_centre_from_pan_mm": path["cube_centre_from_pan_mm"], "frames": len(path["frames"])}, "events": []}

    def say(msg: str, **kw):
        print(msg, flush=True)
        log["events"].append(dict(msg=msg, **kw))

    pl = Planner()
    robot = None
    t_start = time.time()
    frames = path["frames"]
    names = [p["name"] for p in path["pieces"]]
    windows = {}                     # piece -> [world t when its pick starts, world t of its last "leave" pose]
    placed = 0
    server = None
    try:
        pl.need("init", k1="fake", calib="sim")
        Ls = [pl.need("map", urdf=f["q"], jaw=f["jaw"])["lerobot"] for f in frames]     # sim/robot-map.js (Python does not convert)
        rnd = __import__("random").Random(a.seed * 7 + 3)
        bias = [rnd.uniform(-a.arm_err, a.arm_err) for _ in JOINTS]
        if a.arm_bias:
            bias = [float(v) for v in a.arm_bias.split(",")]
            assert len(bias) == len(JOINTS), "--arm-bias needs one value per joint: " + ",".join(JOINTS)
        log["arm_bias_deg"] = bias
        keys = JOINTS + ["gripper"]
        if a.mj:
            from soma_world import SomaMjRobot
            src = a.mj_errors or "{}"
            errors = json.loads(open(src, encoding="utf-8").read() if os.path.exists(src) else src)
            folder = os.path.join(OUT, "soma_" + stamp)
            os.makedirs(folder, exist_ok=True)
            robot = SomaMjRobot(pl, path, Ls[0], bias, folder, Stop, say, errors=errors, chamfer_m=a.soma_chamfer / 1000)
            log["mj_errors"] = errors
        else:
            if any(bias) or a.soma_chamfer:
                raise SystemExit("--soma without --mj: --arm-err / --arm-bias / --soma-chamfer need the --mj world (nothing else uses them)")
            robot = JointsOnly(Ls[0])
        say(f"soma: {len(frames)} poses, {len(names)} pieces, arm base raised {path['raise_mm']} mm, cube {path['cube_centre_from_pan_mm']} mm"
            " in front of the pan axis" + (" (MuJoCo)" if a.mj else " (joints only)"))
        robot.connect_no_torque()
        cur = robot.read_joints()
        far = max(abs(cur[k] - Ls[0][k]) for k in JOINTS)
        if far > START_TOL_DEG:
            raise Stop(f"the arm is {far:.1f} deg from the path's first pose (> {START_TOL_DEG}): move it there first")
        robot.limit_and_torque_on_in_place()
        cur = dict(Ls[0])
        held_run, grip_seen = 0, []
        prev = None

        def do_frame(f, L):
            nonlocal cur, held_run, placed, prev
            i = 0 if prev is None else 1
            n = names[f["n"]]
            if a.mj:
                w = windows.setdefault(n, [None, None])
                if f["ph"] == "pick" and w[0] is None:
                    w[0] = round(robot.t, 4)
            if a.mj and i and (prev["ph"] != f["ph"] or prev["n"] != f["n"]):
                pn, pph = prev["n"], prev["ph"]   # the truth at each change of phase, for the log only
                log.setdefault("phase_trace", []).append({"i": i, "n": pn, "ph_done": pph, "t": round(robot.t, 3),
                                                          "offsets_mm": robot.offsets(pn + (1 if pph in ("place", "leave") else 0))})
            if i and prev["n"] != f["n"]:
                placed = f["n"]
                sp = robot.spread(placed) if a.mj else None     # the truth, for the log only (the control does not see it)
                if sp:
                    log.setdefault("spread_by_step", []).append(sp)
                say(f"[{f['n']}/{len(names)}] {names[f['n'] - 1]} placed (as commanded)" + (f" (truth: spread {sp['spread_mm']} mm)" if sp else ""))
            for st in steps_between(cur, L, keys):
                robot.send(st)
            cur = dict(L)
            if a.mj and f["ph"] == "leave":
                windows[n][1] = round(robot.t, 4)
                o = robot.offsets(f["n"] + 1)[-1]            # the truth per leave pose (open, off, up), for the log only
                log.setdefault("leave_trace", []).append({"n": f["n"], "jaw": round(f["jaw"], 2), "off_mm": o, "mm": round(math.hypot(*o), 2)})
            if f["held"]:
                held_run += 1
                if held_run >= 3:                  # from the third held pose on (the jaws have closed): as the flat puzzle's --grip-check
                    g = robot.read_joints()["gripper"]
                    grip_seen.append(round(g - L["gripper"], 3))
                    if g < L["gripper"] - GRIP_EMPTY_UNITS:
                        raise Stop(f"{n}: the gripper closed {L['gripper'] - g:.1f} past the planned width while holding (> {GRIP_EMPTY_UNITS}): missed or dropped")
                    if g > L["gripper"] + GRIP_WIDE_UNITS:
                        raise Stop(f"{n}: the gripper is {g - L['gripper']:.1f} wider than planned while holding (> {GRIP_WIDE_UNITS}): did not close on it")
            else:
                held_run = 0
            prev = f

        if not a.soma_correct:
            for f, L in zip(frames, Ls):
                do_frame(f, L)
        else:
            server = SomaServer(path, os.path.join(OUT, "soma_" + stamp + "_serve.txt"))
            est = [None] * len(names)              # each placed piece's offset across (x, z) from its place, as the camera saw it (mm)
            log["correct"] = []
            last, q_now, jaw_now = None, None, None
            for pn in range(len(names)):
                touch = soma_touching(path, pn)
                known = {k: v for k, v in touch.items() if est[k] is not None}
                if known:
                    w = sum(known.values())
                    d = [sum(est[k][j] * v for k, v in known.items()) / w for j in (0, 1)]
                else:
                    seen = [e for e in est[:pn] if e is not None]
                    d = [sum(e[j] for e in seen) / len(seen) for j in (0, 1)] if seen else [0.0, 0.0]
                d = [math.copysign(max(abs(v) - SOMA_FOLLOW_SLACK_MM, 0.0), v) for v in d]
                if math.hypot(*d) > SOMA_CORRECT_MAX_MM:
                    raise Stop(f"{names[pn]}: the place would move {math.hypot(*d):.1f} mm (> {SOMA_CORRECT_MAX_MM}): stop")
                r = server.piece(n=pn, place_mm=d if pn else None, cube_off_mm=[e or [0.0, 0.0] for e in est[:pn]],
                                 start_q=q_now, start_jaw=jaw_now, last=last)
                log["correct"].append({"n": pn, "place_mm": [round(v, 3) for v in d], "touching": touch, "verdict": r["verdict"],
                                       "self_min_mm": r["self_min_mm"], "frames": len(r["frames"])})
                if r["verdict"] not in ("ok", "near"):
                    raise Stop(f"{names[pn]}: the moved path did not pass tools/soma_collide.js ({r['verdict']}; {r.get('fail')})")
                say(f"[correct] {names[pn]}: place moved ({d[0]:+.2f}, {d[1]:+.2f}) mm (touching {sorted(touch)}); path {r['verdict']}")
                last = r["last"]
                for f in r["frames"]:
                    do_frame(f, pl.need("map", urdf=f["q"], jaw=f["jaw"])["lerobot"])
                    if f.get("shoot"):
                        seen = robot.measure_tops(pn + 1)
                        byp = {}
                        for c in seen:
                            e = path["pieces"][c["piece"]]["cube_cells"][c["cell"]]
                            byp.setdefault(c["piece"], []).append([c["xz_mm"][0] - e[0] * 1000, c["xz_mm"][1] - e[2] * 1000])
                        for k, v in byp.items():
                            est[k] = [sum(x[j] for x in v) / len(v) for j in (0, 1)]
                        log["correct"][-1]["est_mm"] = [e and [round(x, 3) for x in e] for e in est]
                        log["correct"][-1]["truth_mm"] = robot.offsets(pn + 1)     # the truth, for the log only
                        far = [(k, math.hypot(*e)) for k, e in enumerate(est) if e is not None and math.hypot(*e) > SOMA_OFF_STOP_MM]
                        if far:
                            raise Stop(f"the camera puts {names[far[0][0]]} {far[0][1]:.1f} mm off its place (> {SOMA_OFF_STOP_MM}): stop")
                q_now, jaw_now = r["frames"][-1]["q"], r["frames"][-1]["jaw"]
        placed = len(names)
        sp = robot.spread(placed) if a.mj else None
        if sp:
            log.setdefault("spread_by_step", []).append(sp)
        say(f"[{len(names)}/{len(names)}] {names[-1]} placed (as commanded)" + (f" (truth: spread {sp['spread_mm']} mm)" if sp else ""))
        log["grip_seen"] = {"n": len(grip_seen), "min": min(grip_seen) if grip_seen else None, "max": max(grip_seen) if grip_seen else None}
        log["control_done"] = True
        if a.mj:
            rows = robot.score()
            log["score"] = {"physics": rows}
            log["result"] = "ok" if all(r["ok"] for r in rows) else "ng"
            for r in rows:
                say(f"  {r['name']}: across {r['across_mm']:.2f} mm, up/down {r['up_mm']:.2f} mm, tilt {r['tilt_deg']:.2f} deg {'ok' if r['ok'] else 'NG'}")
        else:
            log["result"] = "ok"
        say(f"result: {log['result'].upper()} in {time.time() - t_start:.0f} s")
    except BaseException as e:   # as main(): hold, never just let go
        log["result"] = "stopped"
        log["stop_kind"] = type(e).__name__
        say(f"!! stopped: {e!r}")
        if robot is not None:
            robot.hold()
    finally:
        if robot is not None:
            robot.release()
            if server is not None:
                server.close()
            if a.mj and hasattr(robot, "contact_log"):
                cl = robot.contact_log()
                # contacts of the arm with a piece outside that piece's own pick..leave window (the truth; scoring only)
                def inside(c):
                    w = windows.get(c["piece"])
                    return bool(w) and w[0] is not None and w[0] - 0.05 <= c["t0"] and c["t1"] <= (w[1] if w[1] is not None else 1e9) + 0.05
                odd = [c for c in cl["contacts"] if c["kind"] == "arm-piece" and not inside(c)]
                pp = [c for c in cl["contacts"] if c["kind"] == "piece-piece"]
                log["mj"] = {"final_states": robot.piece_states(), "world_t": round(robot.t, 4), "windows": windows,
                             "contacts": cl["contacts"], "max_load_nm": cl["max_load_nm"], "near": cl["near"],
                             "arm_piece_outside_window": odd,
                             "score_at_end": robot.score(None if log.get("result") != "stopped" else placed)}
                say(f"[contacts] arm-piece outside the piece's own pick..leave: {len(odd)}; piece-piece intervals: {len(pp)}; max load "
                    + ", ".join(f"{k} {v:.2f}" for k, v in cl["max_load_nm"].items()))
        pl.close()
        out = os.path.join(OUT, "soma_" + stamp + ".json")
        with open(out, "w", encoding="utf-8") as fo:
            json.dump(log, fo, indent=1, ensure_ascii=False)
        print(f"wrote {os.path.normpath(out)}", flush=True)
    sys.exit(0 if log.get("result") == "ok" else 1)


def main() -> None:
    ap = argparse.ArgumentParser(usage=__doc__)
    ap.add_argument("port", nargs="?", default=None)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--k1", default=None)
    ap.add_argument("--set", type=int, default=2)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--air", type=float, default=0.0)
    ap.add_argument("--pieces", type=int, default=0)
    ap.add_argument("--top-cam", type=int, default=None)
    ap.add_argument("--top-intrinsics", default=None)
    ap.add_argument("--top-pose", default=None)
    ap.add_argument("--wrist-cam", type=int, default=None)
    ap.add_argument("--wrist-json", default=None)
    ap.add_argument("--k1err", default=None)
    ap.add_argument("--speed", default=None)
    ap.add_argument("--make-sim-calib", default=None)
    ap.add_argument("--cam-selftest", action="store_true")
    ap.add_argument("--no-wrist", action="store_true")
    ap.add_argument("--shift", default=None)
    ap.add_argument("--calib-mm", type=float, default=0.0)
    ap.add_argument("--calib-deg", type=float, default=0.0)
    ap.add_argument("--noise", type=float, default=3.0)
    ap.add_argument("--arm-err", type=float, default=0.0)
    ap.add_argument("--arm-bias", default=None)
    ap.add_argument("--sheet", action="store_true")
    ap.add_argument("--place-check", type=int, default=0)
    ap.add_argument("--board-read", action="store_true")
    ap.add_argument("--arm-margin", type=float, default=0.0)
    ap.add_argument("--sim-missing", default=None)
    ap.add_argument("--mj", action="store_true")
    ap.add_argument("--mj-faults", default=None)
    ap.add_argument("--mj-occluders", default=None)
    ap.add_argument("--mj-errors", default=None)
    ap.add_argument("--gap", type=float, default=None)
    ap.add_argument("--end-check", action="store_true")
    ap.add_argument("--grip-check", action="store_true")
    ap.add_argument("--replay", default=None)
    ap.add_argument("--alone", action="store_true")
    ap.add_argument("--soma", default=None)
    ap.add_argument("--soma-chamfer", type=float, default=0.0)
    ap.add_argument("--soma-correct", action="store_true")
    a = ap.parse_args()
    if a.soma:
        return run_soma(a, ap)
    if a.alone and not a.pieces:
        raise SystemExit("--alone goes with --pieces N (only those pieces lie on the table)")
    if a.mj and not a.sim:
        raise SystemExit("--mj goes with --sim")
    if a.gap is not None and not a.sim:
        raise SystemExit("--gap is --sim only (the real board is measured, not chosen)")
    if (a.mj_faults or a.mj_occluders or a.mj_errors) and not a.mj:
        raise SystemExit("--mj-faults / --mj-occluders / --mj-errors are --mj only")
    if (a.mj or a.replay) and (a.place_check or a.sim_missing):
        raise SystemExit("--mj / --replay: not with --place-check or --sim-missing yet")
    if a.replay and not a.sim and a.port:
        raise SystemExit("--replay of a real-arm record: give the record's other arguments but no port (nothing is opened)")
    if a.mj or a.replay:
        a.end_check = a.grip_check = True
    if not a.sim:                                  # S4 (2026-11-23): the real arm checks its grip and its end picture on the table.
        a.end_check = a.grip_check = a.air == 0    #   In the air nothing is held and nothing lands on the board: both off there
    if a.end_check and a.sim and not (a.mj or a.replay):
        raise SystemExit("--end-check: plain --sim does not draw the placed pieces (use --mj)")
    if a.make_sim_calib:
        make_sim_calib(a.make_sim_calib)
        return
    if a.cam_selftest:
        sys.exit(0 if cam_selftest() else 1)
    if a.pieces < 0:
        raise SystemExit("--pieces must be 1 or more (0 = all)")
    if a.place_check < 0 or (a.place_check and a.air > 0):
        raise SystemExit("--place-check N: N >= 0, and not with --air (a piece placed in the air does not lie on the board)")
    if a.sim_missing and not a.sim:
        raise SystemExit("--sim-missing is --sim only")
    if a.arm_margin < 0:
        raise SystemExit("--arm-margin MM: MM >= 0")
    if a.board_read and not a.sim:
        raise SystemExit("--board-read is --sim only for now (how the real board looks against the table is not known)")
    if a.place_check and not a.sim:
        # 点検 C49 の 1: re-picking from the board, the moved place point and the back-to-the-photo-pose loop cannot be tried in
        #   the air (--air cannot go with it), so air_run_ok() would let it run on the table untried. Until the user decides how
        #   it is first tried on the real arm, it is sim only
        raise SystemExit("--place-check is --sim only for now (it cannot be tried in the air first; the user decides how to try it)")
    speed = tuple(int(v) for v in a.speed.split(",")) if a.speed else SPEED_DEFAULT
    if len(speed) != 2 or not (0 < speed[0] <= SPEED_MAX[0] and 0 < speed[1] <= SPEED_MAX[1]):
        raise SystemExit(f"--speed V,A: 1..{SPEED_MAX[0]}, 1..{SPEED_MAX[1]} (review C38)")
    files = bool(a.top_intrinsics or a.top_pose or a.wrist_json)
    k1_sha = None
    if not a.sim:
        miss = [n for n, v in (("--k1", a.k1), ("--top-cam", a.top_cam), ("--top-intrinsics", a.top_intrinsics), ("--top-pose", a.top_pose))
                if v is None]
        if not a.no_wrist:
            miss += [n for n, v in (("--wrist-cam", a.wrist_cam), ("--wrist-json", a.wrist_json)) if v is None]
        if miss:
            raise SystemExit("real arm: needs " + ", ".join(miss))
        if a.wrist_cam is not None and a.wrist_cam == a.top_cam:
            raise SystemExit("--top-cam and --wrist-cam are the same index")
        k1_sha = file_sha(a.k1)
    try:
        calib, cams, shas = load_calib(a, k1_sha) if files or not a.sim else ("sim", {}, {})
    except Stop as e:
        raise SystemExit(str(e))
    except (OSError, ValueError, KeyError, TypeError, IndexError) as e:
        raise SystemExit(f"cannot read the camera files: {e!r}")
    if a.sim and files and (a.calib_mm or a.calib_deg):
        raise SystemExit("--calib-mm / --calib-deg move the sim's camera; with camera files, edit the files instead")
    code = code_sha()
    if not a.sim and a.air < MIN_AIR_REAL_MM and not a.replay:
        why = air_run_ok(k1_sha, shas, a, code, speed)
        if why:
            raise SystemExit(f"real arm: --air below {MIN_AIR_REAL_MM:.0f} mm needs an in-air run first: {why}")
    os.makedirs(OUT, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + f"_p{os.getpid()}"   # two runs in the same second: the process id keeps them apart
    log = {"date": datetime.datetime.now().isoformat(timespec="seconds"), "args": vars(a), "real": not a.sim and not a.replay, "k1_sha256": k1_sha,
           "calib_sha256": shas, "code_sha256": code, "calib": calib, "speed": list(speed) if not a.sim else None, "events": []}
    if FAKE_HW:
        log["fake_hardware"] = True

    def say(msg: str, **kw):
        print(msg, flush=True)
        log["events"].append(dict(msg=msg, **kw))

    def missing_ng():
        """--sim-missing: a piece that was never on the table cannot have been placed, whatever the score says (点検 C58 の 4).
        The sim moves the pieces it is told were grasped, so without this a missing piece read at a wrong place could score ok."""
        miss = [pieces[i]["name"] for i in active if pieces[i]["name"] in (log.get("sim_missing") or [])]   # only the pieces this run builds
        if miss and log.get("result") == "ok":
            log["result"] = "ng"
            say(f"result: NG (sim: {', '.join(miss)} was not on the table, so the run cannot be complete)")

    pl = Planner()
    robot = None
    t_start = time.time()
    try:
        init = pl.need("init", sheet=a.sheet, set=a.set, k1="fake" if a.sim else a.k1, calib=calib, calibMm=a.calib_mm, calibDeg=a.calib_deg,
                       noise=a.noise, seed=a.seed, **({"gap": a.gap} if a.gap is not None else {}))
        pieces = init["pieces"]
        start_L = init["start_lerobot"]
        active = init["order"][:a.pieces] if a.pieces else [p["idx"] for p in pieces]
        absent = [p["idx"] for p in pieces if p["idx"] not in active]
        if a.pieces:
            say(f"--pieces {a.pieces}: " + ", ".join(pieces[i]["name"] for i in active) + " (the others are taken to lie at the plan's places: "
                + ", ".join(pieces[i]["name"] for i in absent) + ")")
        if files:
            say("cameras from the files: top centre " + ", ".join(f"{v * 1000:.1f}" for v in calib["top"]["tip"])
                + f" mm (sheet frame; turned by {-calib['top']['panDeg']:+.2f} deg about the pan axis), FOV {calib['top']['fov']:.2f} deg"
                + (f"; wrist FOV {calib['wrist']['fov']:.2f} deg" if calib["wrist"] else "; wrist: the URDF's"))
        if a.sim:
            truth = pl.need("truth", seed=a.seed)["layout"]
            log["truth"] = json.loads(json.dumps(truth))       # a copy: --place-check moves the pieces of the world (点検 C49 の 4)
            # the user's pose: in --sim, a little away from the start pose (P-001: start from where the arm is)
            user_L = {k: v + (3.0 if k in JOINTS else 0.0) for k, v in start_L.items()}
            rnd = __import__("random").Random(a.seed * 7 + 3)
            bias = [rnd.uniform(-a.arm_err, a.arm_err) for _ in JOINTS]
            if a.arm_bias:
                bias = [float(v) for v in a.arm_bias.split(",")]
                assert len(bias) == len(JOINTS), "--arm-bias needs one value per joint: " + ",".join(JOINTS)
            log["arm_bias_deg"] = bias
            if any(bias):
                say("(sim) true arm = read angles + " + ", ".join(f"{j} {b:+.2f}" for j, b in zip(JOINTS, bias)) + " deg")
            if a.replay:                                       # S2: only the recorded observations; no world, no truth here
                rec = json.load(open(a.replay, encoding="utf-8"))
                robot, truth = ReplayRobot(rec), None
                log["truth"] = None
                log["replay_of"] = {"file": os.path.abspath(a.replay), "n_obs": len(rec["mj"]["obs"]), "n_sent": len(rec["mj"]["sent"])}
                say(f"(replay) {len(rec['mj']['obs'])} observations from {a.replay}")
            elif a.mj:                                         # S2: MuJoCo world (tools/mj_world.py)
                from mj_world import MjRobot                   # needs mujoco: sim\physics\.venv
                faults = json.loads(open(a.mj_faults, encoding="utf-8").read() if a.mj_faults and os.path.exists(a.mj_faults) else (a.mj_faults or "[]"))
                occl = json.loads(open(a.mj_occluders, encoding="utf-8").read() if a.mj_occluders and os.path.exists(a.mj_occluders) else (a.mj_occluders or "[]"))
                errs = json.loads(open(a.mj_errors, encoding="utf-8").read() if a.mj_errors and os.path.exists(a.mj_errors) else (a.mj_errors or "{}"))
                defaults = pl.need("truth", seed=a.seed, shiftMm=0, yawDeg=0)["layout"]
                folder = os.path.join(OUT, "run_" + stamp)
                os.makedirs(folder, exist_ok=True)
                world = MjRobot(pl, a.set, truth, defaults, set(active), user_L, bias, folder, Stop, say, faults=faults, occluders=occl,
                                errors=errs, gap=a.gap)
                log["mj_world"] = {"scene_json": world.proto_path, "start": world.start_layout, "faults": faults, "occluders": occl,
                                   "errors": errs}
                robot = Recorder(world)
            else:
                robot = SimRobot(pl, truth, user_L, bias, absent, os.path.join(OUT, "run_" + stamp))
            if a.sim_missing:                                  # negative check: planned for, but not in the world
                names = {p["name"]: p["idx"] for p in pieces}
                miss = [v.strip() for v in a.sim_missing.split(",") if v.strip()]
                bad = [v for v in miss if v not in names]
                if bad:
                    raise Stop(f"--sim-missing: no piece named {bad} (pieces: {sorted(names)})")
                for v in miss:
                    robot.hide.add(names[v])
                log["sim_missing"] = miss
                say("(sim) not on the table: " + ", ".join(miss))
            if files:
                for name, c in cams.items():
                    if tuple(c["size"]) != (640, 480):
                        raise Stop(f"--sim draws 640x480; the {name} file is for {c['size'][0]}x{c['size'][1]}")
        elif a.replay:                                         # S4: a real-arm record, given back observation by observation (no port)
            truth = None
            rec = json.load(open(a.replay, encoding="utf-8"))
            if "rec" not in rec:
                raise Stop(f"{a.replay} is not a real-arm record (no 'rec')")
            robot = ReplayRobot(rec)
            log["replay_of"] = {"file": os.path.abspath(a.replay), "n_obs": len(rec["rec"]["obs"]), "n_sent": len(rec["rec"]["sent"]),
                                "stop_at": rec.get("stop_at")}
            say(f"(replay of a real-arm record) {len(rec['rec']['obs'])} observations from {a.replay}")
        else:
            truth = None
            folder = os.path.join(OUT, "run_" + stamp)
            cameras = {"top": Camera(a.top_cam, cams["top"], folder, "top")}
            if "wrist" in cams and a.wrist_cam is not None:
                cameras["wrist"] = Camera(a.wrist_cam, cams["wrist"], folder, "wrist")
            robot = Recorder(RealRobot(a.port, init["calibKey"], cameras, speed))   # S4: every observation and command kept (--replay)
            say(f"real arm: speed {speed[0]} steps/s, acceleration {speed[1]}, air {a.air} mm"
                + ("; grip check and end check on" if a.air == 0 else "; in the air: no grip check, no end check"))

        robot.connect_no_torque()
        now = robot.read_joints()
        worst = max(JOINTS, key=lambda j: abs(now[j] - start_L[j]))
        if abs(now[worst] - start_L[worst]) > START_TOL_DEG:
            raise Stop(f"not at the start pose (the photo pose): {worst} is {now[worst] - start_L[worst]:+.1f} deg away")
        robot.limit_and_torque_on_in_place()
        now = robot.read_joints()                          # read again: the arm may have sagged while the user pressed Enter (点検 C46 の 1)
        worst = max(JOINTS, key=lambda j: abs(now[j] - start_L[j]))
        if abs(now[worst] - start_L[worst]) > START_TOL_DEG:
            raise Stop(f"after torque on, not at the start pose: {worst} is {now[worst] - start_L[worst]:+.1f} deg away")
        for s in steps_between(now, start_L, JOINTS + ["gripper"]):
            robot.send(s)

        # (1)(2) top image -> layout (with --pieces: only those pieces are looked for)
        im = robot.read_camera("top")
        board_fix = None
        if a.board_read:                                       # the board is empty now: read it once; the camera does not move
            b = pl.ask("board", img=im["out"], W=im["W"], H=im["H"])
            if not b.get("ok"):
                raise Stop(f"board: cannot read the board in the top image ({b.get('why')})")
            board_fix = {k: b[k] for k in ("dx_mm", "dz_mm", "dyaw_deg", "about_mm")}
            log["board"] = b
            say(f"board: the top camera's reading is moved by {b['dx_mm']:+.2f},{b['dz_mm']:+.2f} mm {b['dyaw_deg']:+.2f} deg onto the board "
                f"({b['n_edge']} edge px)" + (f" [sim: board cells {b['sim_before_mm']:.2f} -> {b['sim_resid_mm']:.2f} mm]" if "sim_resid_mm" in b else ""))
        det = pl.ask("detect", img=im["out"], W=im["W"], H=im["H"], only=active if a.pieces else None, board_fix=board_fix)
        if not det.get("ok"):
            log["detect_refused"] = {"why": det.get("why"), "per": det.get("per")}   # what each piece's colour gave (点検 C58 の 3)
            raise Stop(f"detect: {det.get('why')}")
        layout = det["layout"]
        if a.shift:
            dx, dz = (float(v) for v in a.shift.split(","))
            for p in layout["pieces"]:
                p["x"] += dx
                p["z"] += dz
            say(f"(negative check) shifted the found layout by {dx},{dz} mm")
        say("found: " + " / ".join(f"{p['name']} {p['x']:.1f},{p['z']:.1f} mm {p['yaw']:.1f} deg" for p in layout["pieces"]))
        log["found"] = json.loads(json.dumps(layout))
        before = {}                                            # --end-check: the pieces not moved, as the first top picture has them
        end_fix = board_fix                                    # --end-check reads against the board seen in the first top picture (S3)
        if a.end_check and end_fix is None and a.sim:          # S4: not on the real arm (how the real board reads is not known yet: R0)
            b = pl.ask("board", img=im["out"], W=im["W"], H=im["H"])
            if b.get("ok"):
                end_fix = {k: b[k] for k in ("dx_mm", "dz_mm", "dyaw_deg", "about_mm")}
                log["end_board"] = b
                say(f"end check: will read against the board: {b['dx_mm']:+.2f},{b['dz_mm']:+.2f} mm {b['dyaw_deg']:+.2f} deg ({b['n_edge']} edge px)")
            else:
                say(f"end check: cannot read the board in the first top picture ({b.get('why')}); the end check reads without it")
        if a.end_check and a.alone:                            # S4: only the --pieces lie on the table (D4): nothing else to be pushed
            say("end check: --alone: the pieces not moved are not on the table; only the moved ones are read at the end")
        elif a.end_check:
            for q in [p["idx"] for p in pieces if p["idx"] not in active]:
                dq = pl.ask("detect", img=im["out"], W=im["W"], H=im["H"], only=[q], board_fix=end_fix)
                if dq.get("ok"):
                    before[q] = dq["layout"]["pieces"][q]
            say("end check: pieces not moved, found in the first top picture: "
                + (", ".join(f"{v['name']} {v['x']:.1f},{v['z']:.1f} mm" for v in before.values()) or "none"))
            log["end_check_before"] = {str(k): v for k, v in before.items()}
            unseen = [p["name"] for p in pieces if p["idx"] not in active and p["idx"] not in before]
            if unseen:                                     # a push of these could not be seen at the end (review C60 2): hold now
                raise Stop(f"end check: {', '.join(unseen)} not found in the first top picture: a push could not be seen later; not starting")
        targets = {}                                           # --end-check: pid -> board target (from the plans run)
        grip_seen = {}                                         # --grip-check: pid -> (most open, most closed past the planned width (0-100), checks)
        log["grip_seen"] = grip_seen

        # (3)(4) plan
        settle = init["wrist_settle"]

        def marks(p: dict) -> tuple:
            """frame index -> step, for the wrist shot (first pose after settling above the piece) and for the pick (jaw closed)."""
            shot, pick, place = {}, {}, {}
            for st in p["steps"]:
                if settle is not None and st["tCheck1"] > 0:
                    shot[next(n for n, f in enumerate(p["frames"]) if f["t"] >= st["tCheck0"] + settle - 1e-9)] = st
                pick[next(n for n, f in enumerate(p["frames"]) if f["t"] >= st["tPick"] - 1e-9)] = st
                place[next(n for n, f in enumerate(p["frames"]) if f["t"] >= st["tPlace"] - 1e-9)] = st
            return shot, pick, place

        fixes = {p["idx"]: 0 for p in pieces}
        picks, pick_log = {}, []
        log["picks"] = pick_log                                # kept in the log even when it stops (点検 C49 の 4)
        st8 = {"n_back": 0}

        def margin_say(p: dict, who: str, pid=None) -> None:
            """--arm-margin and (sim) the true arm's nearness, for every plan that is run (the wrist's new plans too: 点検 C55 の 1)."""
            if p.get("arm_margin_mm"):
                near = (f"nearest other piece {p['arm_margin_clear_mm']:.1f} mm ({p['arm_margin_pair']} at {p['arm_margin_t']:.2f} s)"
                        if p.get("arm_margin_clear_mm") is not None else "no other piece near")   # none near: the answer has null
                say(f"{who}: picks and places checked with the jaws moved {p['arm_margin_mm']:g} mm in 8 directions: {near}", pid=pid,
                    arm_margin_mm=p["arm_margin_mm"], arm_margin_clear_mm=p.get("arm_margin_clear_mm"), arm_margin_pair=p.get("arm_margin_pair"))
            if p.get("true_clear_mm") is not None:
                say(f"[sim: {who}: the true arm comes {p['true_clear_mm']:.1f} mm near another piece at the picks and places "
                    f"({p['true_clear_pair']} at {p['true_clear_t']:.2f} s)]", pid=pid, true_clear_mm=p["true_clear_mm"],
                    true_clear_pair=p["true_clear_pair"])

        def execute(layout: dict, pieces_n: int, offsets, want: list) -> dict:
            """Plan with this layout and run it, with the wrist checks and corrections. Returns the last plan."""
            kw = dict(air=a.air, pieces=pieces_n or None, place_offset=offsets or None)
            if offsets:                                         # the offset undoes the arm's error at the place pose: the arm truly goes
                kw["offset_is_arm"] = True                      #   where it would without it, so the place is checked for hits there (plan-cli)
            if a.arm_margin > 0:                                # the arm lands off the commanded pose: no other piece within this at the picks and places
                kw["arm_margin_mm"] = a.arm_margin
            if isinstance(robot, SimRobot) and any(robot.bias):  # sim: how near the true arm comes (reported only)
                kw["true_bias_deg"] = list(robot.bias)
            for q in want:                                      # the wrist corrections are counted per pick (a re-pick counts afresh)
                fixes[q] = 0
            try:
                plan = pl.need("plan", layout=layout, **kw)
            except Stop as e:
                if not a.sim and "盤に重なる" in str(e):     # 点検 C63 の 4: on the real arm, say what the user should do
                    raise Stop(f"{e} -> real arm: a piece already lies on the board (an earlier run may have stopped half way). "
                               "Put the pieces back at their start places and run again") from None
                raise
            layout = plan.get("layout") or layout              # a symmetric piece may be planned in another of its forms (plan-cli sym)
            for sy in plan.get("sym") or []:
                say(f"plan: {sy['name']} planned as the same cells at {sy['to']['yaw']:.1f} deg (read as {sy['from']['yaw']:.1f} deg: refused)",
                    pid=sy["pid"])
            if plan.get("offset_is_arm"):                      # which frame the place was checked in (点検 C54 の 7)
                say(f"plan: hits at the place checked where the arm truly goes (the offset undone at the jaws and the piece, "
                    f"and +-{plan.get('offset_perp_mm')} mm across the jaws)", offset_is_arm=True, offset_perp_mm=plan.get("offset_perp_mm"))
            margin_say(plan, "plan")
            got = [st["pid"] for st in plan["steps"]]
            if pieces_n == 1 and len(want) > 1 and len(got) == 1 and got[0] in want:
                pass                                            # --place-check: the plan picks which of these goes next
            elif (got if pieces_n else sorted(got)) != (want if pieces_n else sorted(want)):
                raise Stop(f"plan: the moves are for pieces {got}, the pieces looked for are {want}")
            if plan["min_tip_mm"] < MIN_TIP_MM:
                raise Stop(f"plan: finger tips go down to {plan['min_tip_mm']:.1f} mm above the table (< {MIN_TIP_MM})")
            shot_at, pick_at, place_at = marks(plan)
            for g in plan["grasps"]:
                targets[g["pid"]] = {"name": g["name"], "x": g["target"][0] * 1000, "z": g["target"][1] * 1000, "yaw": g["target"][2]}
            if not a.sim and not st8.get("guarded"):         # S4: a run started again after a stop must not carry on the old one
                st8["guarded"] = True
                for g in plan["grasps"]:
                    p = layout["pieces"][g["pid"]]
                    if p.get("on_board"):
                        continue
                    mg = pl.need("match", pid=g["pid"], est=targets[g["pid"]], meas=p)
                    if mg["worst_cell_mm"] < init["pitch_mm"] / 2:
                        raise Stop(f"{g['name']} already lies at its board target ({mg['worst_cell_mm']:.1f} mm): an earlier run may have "
                                   "stopped half way. Put the pieces back at their start places and run again")
            say(f"plan: {len(plan['frames'])} poses, {plan['T']:.1f} s, tip lowest {plan['min_tip_mm']:.1f} mm, "
                f"arm-to-arm {plan['self_min_mm']} mm, air {plan['air_mm']} mm")
            cur = robot.read_joints()                          # where the arm IS (点検 C49 の 8: not the last pose sent)
            f0 = plan["frames"][0]["L"]
            far = max(JOINTS, key=lambda j: abs(cur[j] - f0[j]))
            if abs(cur[far] - f0[far]) > START_TOL_DEG:
                raise Stop(f"plan: it starts {cur[far] - f0[far]:+.1f} deg ({far}) away from where the arm is")
            for s in steps_between(cur, f0, JOINTS + ["gripper"]):
                robot.send(s)
            cur = f0
            i = 0
            while i < len(plan["frames"]) - 1:
                i += 1
                f = plan["frames"][i]
                for s in steps_between(cur, f["L"], JOINTS + ["gripper"]):
                    robot.send(s)
                cur = f["L"]
                if a.grip_check and f.get("held", -1) >= 0 and plan["frames"][i - 1].get("held", -1) == f["held"]:
                    # held (not the first held pose: the jaw may still be closing). Smaller = more closed (RealRobot.grip_goal_for)
                    gr = robot.read_joints()["gripper"]
                    past = f["L"]["gripper"] - gr
                    lo, hi, nh = grip_seen.get(f["held"], (1e9, -1e9, 0))
                    grip_seen[f["held"]] = (min(lo, past), max(hi, past), nh + 1)
                    if past < -GRIP_WIDE_UNITS and nh + 1 >= 2:          # the second check = the third held pose
                        nm = pieces[f["held"]]["name"]
                        raise Stop(f"{nm}: the gripper reads {gr:.2f}, {-past:.2f} wider than the planned width {f['L']['gripper']:.2f} "
                                   f"(> {GRIP_WIDE_UNITS}) while it should hold the piece: the jaws did not close on it")
                    if past > GRIP_EMPTY_UNITS:
                        nm = pieces[f["held"]]["name"]
                        raise Stop(f"{nm}: the gripper reads {gr:.2f}, {past:.2f} past the planned width {f['L']['gripper']:.2f} "
                                   f"(> {GRIP_EMPTY_UNITS}) while it should hold the piece: nothing in the jaws (missed or dropped)")
                if i in pick_at:                                       # the jaw has closed: record what was planned for this pick
                    st = pick_at[i]
                    g = next(g for g in plan["grasps"] if g["pid"] == st["pid"])
                    picks[st["pid"]] = {"pid": st["pid"], "est": layout["pieces"][st["pid"]], "cS": g["cS"], "yp": g["yp"],
                                        "lerobot": robot.read_joints()}
                    if truth is not None:                              # the world when it was picked (--place-check moves pieces)
                        picks[st["pid"]]["truth"] = json.loads(json.dumps(robot.truth))
                    pick_log.append(picks[st["pid"]])
                    if isinstance(robot, SimRobot):
                        robot.hide.add(st["pid"])
                if i in place_at and place_at[i]["pid"] in picks:  # the jaw is at the place pose (about to open): for the placed-position score
                    picks[place_at[i]["pid"]]["place_lerobot"] = robot.read_joints()
                if a.no_wrist or i not in shot_at:
                    continue
                st = shot_at[i]
                pid = st["pid"]
                im = robot.read_camera("wrist")
                w = pl.need("wrist", img=im["out"], W=im["W"], H=im["H"], lerobot=robot.read_joints(), pid=pid,
                            est=layout["pieces"][pid])
                m = pl.need("match", pid=pid, est=layout["pieces"][pid], meas=w["piece"])
                say(f"[wrist {st['name']}] off by {m['worst_cell_mm']:.2f} mm, {m['phi']:+.2f} deg ({w['nPix']} px, cost {w['cost']:.1e}"
                    + (f", top seen {w['top_frac']:.2f})" if w.get("top_frac") is not None else ")"),
                    pid=pid, off_mm=m["worst_cell_mm"], phi=m["phi"], cost=w["cost"], nTop=w["nTop"], top_frac=w.get("top_frac"))
                if w.get("top_frac") is not None and w["top_frac"] < WRIST_MIN_FRAC:
                    raise Stop(f"{st['name']}: the wrist sees only {w['top_frac']:.2f} of the piece's top (< {WRIST_MIN_FRAC}): "
                               "something hides it; not trusting the wrist reading")
                if m["worst_cell_mm"] <= FIX_TOL_MM:
                    continue
                if m["worst_cell_mm"] > init["pitch_mm"] / 2:   # the fit may have slipped by a whole cell (hidden cells; review C35 2)
                    raise Stop(f"{st['name']}: the wrist says {m['worst_cell_mm']:.1f} mm off, more than half a cell ({init['pitch_mm'] / 2:.1f} mm). "
                               "Not trusting it: look at the top image again")
                if fixes[pid] >= MAX_FIX:
                    if m["worst_cell_mm"] <= FIX_ACCEPT_MM:      # S3: the arm does not land twice on the same spot; accept a small rest
                        say(f"[wrist {st['name']}] still {m['worst_cell_mm']:.2f} mm off after {MAX_FIX} corrections (<= {FIX_ACCEPT_MM}): going on",
                            pid=pid, accepted_mm=m["worst_cell_mm"])
                        continue
                    raise Stop(f"{st['name']}: still off by {m['worst_cell_mm']:.2f} mm after {MAX_FIX} corrections (> {FIX_ACCEPT_MM})")
                fixes[pid] += 1
                new_layout = json.loads(json.dumps(layout))
                new_layout["pieces"][pid] = m["fixed"]
                plan2 = pl.ask("plan", layout=new_layout, sym_pids=[pid], **kw)   # only this piece's other forms (others would change earlier moves)
                if plan2.get("ok"):
                    margin_say(plan2, f"[wrist {st['name']}] new plan", pid=pid)
                if not plan2.get("ok"):
                    raise Stop(f"{st['name']}: no plan with the corrected place ({plan2.get('why')})")
                if [x["pid"] for x in plan2["steps"]] != [x["pid"] for x in plan["steps"]]:
                    raise Stop(f"{st['name']}: the corrected plan picks other pieces or in another order")
                if plan2["min_tip_mm"] < MIN_TIP_MM:
                    raise Stop(f"{st['name']}: the corrected plan takes the finger tips to {plan2['min_tip_mm']:.1f} mm")
                k = first_diff(plan["frames"], plan2["frames"])
                if k >= min(len(plan["frames"]), len(plan2["frames"])):
                    raise Stop(f"{st['name']}: the corrected plan is the same as the old one (nothing to correct?)")
                prev_done = max([s["tDone"] for s in plan["steps"] if s["tDone"] < st["tCheck0"]] + [0.0])
                if k == 0 or plan["frames"][k]["t"] <= prev_done + 1e-9:
                    raise Stop(f"{st['name']}: the corrected plan changes the moves before this piece "
                               f"(from {plan['frames'][min(k, len(plan['frames']) - 1)]['t']:.2f} s, the previous piece ended at {prev_done:.2f} s)")
                if k <= i:
                    for j in range(i - 1, k - 2, -1):            # go back the way we came (poses already checked)
                        for s in steps_between(cur, plan["frames"][j]["L"], JOINTS + ["gripper"]):
                            robot.send(s)
                        cur = plan["frames"][j]["L"]
                        st8["n_back"] += 1
                    i = k - 1
                say(f"[wrist {st['name']}] corrected to {m['fixed']['x']:.1f},{m['fixed']['z']:.1f} mm {m['fixed']['yaw']:.1f} deg; "
                    f"back to {plan['frames'][i]['t']:.2f} s, new plan {plan2['T']:.1f} s", pid=pid)
                for sy in plan2.get("sym") or []:
                    say(f"[wrist {st['name']}] the corrected plan has {sy['name']} as the same cells at {sy['to']['yaw']:.1f} deg "
                        f"(as {sy['from']['yaw']:.1f} deg: refused)", pid=pid)
                plan, layout = plan2, plan2.get("layout") or new_layout
                shot_at, pick_at, place_at = marks(plan)
            return plan

        final = {}                                             # --place-check: pid -> what the top camera saw at the end
        log["placed"] = final
        if not a.place_check:
            execute(layout, a.pieces, None, active)
        else:
            # --place-check N: one piece at a time. After each placing, back at the photo pose, the top camera reads where the piece
            #   lies on the board. Off by more than PLACE_TOL_MM (worst cell centre against the board target): pick it up again from
            #   the board (on_board) and place it again with the place point moved back by what was seen (place_offset: the arm's
            #   error at the place pose repeats). At most N times per piece. Placed pieces stay put (keep) for the later plans.
            placed = {}
            while len(placed) < len(active):
                lay = json.loads(json.dumps(layout))
                for q, pose in placed.items():
                    lay["pieces"][q] = dict(pose, keep=True)
                pid, off, tries = None, None, 0
                while True:
                    # the first time the plan picks the next piece (its order can differ from init's once pieces lie on the board)
                    plan = execute(lay, 1, [off] if tries else None, [pid] if tries else [q for q in active if q not in placed])
                    pid = plan["steps"][0]["pid"]
                    if off is None:
                        off = {"pid": pid, "dx_mm": 0.0, "dz_mm": 0.0, "dyaw_deg": 0.0}
                    g = next(g for g in plan["grasps"] if g["pid"] == pid)
                    tgt = {"name": g["name"], "x": g["target"][0] * 1000, "z": g["target"][1] * 1000, "yaw": g["target"][2]}
                    if truth is not None:                       # sim: the piece now lies where the true arm put it
                        sc1 = pl.need("score", truth=picks[pid]["truth"], picks=[picks[pid]], bias=robot.bias)
                        pt = sc1["rows"][0]["placed_true"]
                        robot.truth["pieces"][pid].update(x=pt["x"], z=pt["z"], yaw=pt["yaw"])
                        robot.hide.discard(pid)
                    im = robot.read_camera("top")
                    # look for the pieces already on the board too: a piece of a near colour next to it (L red, O orange) is then
                    #   given its own pixels (with only=[O], L's pixels joined O's: "outline 9.8 mm" in the first sim try)
                    det = pl.ask("detect", img=im["out"], W=im["W"], H=im["H"], only=sorted(active),   # all of them: 点検 C49 の 7
                                 board_fix=board_fix)
                    if board_fix is not None:                   # the camera must not have moved: read the board again while enough of it shows
                        b2 = pl.ask("board", img=im["out"], W=im["W"], H=im["H"])
                        if b2.get("ok"):
                            mv = board_moved(board_fix, b2)
                            if mv > BOARD_MOVED_MM:
                                raise Stop(f"board: read again, it is {mv:.2f} mm from the first reading (> {BOARD_MOVED_MM}): the top camera may have moved")
                            say(f"[board] read again: {mv:.2f} mm from the first reading")
                        else:
                            say(f"[board] not read again ({b2.get('why')})")
                    if not det.get("ok"):
                        raise Stop(f"{tgt['name']}: the top camera does not find it on the board ({det.get('why')})")
                    meas = det["layout"]["pieces"][pid]
                    for q, pose in placed.items():                # the pieces placed before must still be where they were accepted
                        mq = pl.need("match", pid=q, est=pose, meas=det["layout"]["pieces"][q])
                        if mq["worst_cell_mm"] > PLACE_MOVED_MM:
                            raise Stop(f"{pose['name']} (placed before) now reads {mq['worst_cell_mm']:.2f} mm from where it was accepted "
                                       f"(> {PLACE_MOVED_MM}): it may have been pushed")
                    mt = pl.need("match", pid=pid, est=tgt, meas=meas)
                    tru = ""
                    if truth is not None:                       # sim only: what is really there (not used to decide)
                        mtt = pl.need("match", pid=pid, est=tgt, meas=dict(robot.truth["pieces"][pid]))
                        tru = f" [sim: truly {mtt['worst_cell_mm']:.2f} mm, {mtt['phi']:+.2f} deg]"
                    say(f"[placed {tgt['name']}] off the board target by {mt['worst_cell_mm']:.2f} mm, {mt['phi']:+.2f} deg{tru}"
                        + (f" (re-placing {tries}; place point moved {off['dx_mm']:+.2f},{off['dz_mm']:+.2f} mm {off['dyaw_deg']:+.2f} deg)" if tries else ""),
                        pid=pid, place_off_mm=mt["worst_cell_mm"], phi=mt["phi"], tries=tries)
                    final[pid] = {"target": tgt, "meas": meas, "off_mm": mt["worst_cell_mm"], "tries": tries, "offset": dict(off)}
                    if mt["worst_cell_mm"] <= PLACE_TOL_MM:
                        placed[pid] = {"name": meas["name"], "x": meas["x"], "z": meas["z"], "yaw": meas["yaw"]}
                        break
                    if mt["worst_cell_mm"] > PLACE_REDO_MAX_MM:
                        raise Stop(f"{tgt['name']}: the top camera says it lies {mt['worst_cell_mm']:.1f} mm from the target "
                                   f"(> {PLACE_REDO_MAX_MM} mm): not picking it up from the board")
                    if tries >= a.place_check:
                        raise Stop(f"{tgt['name']}: still {mt['worst_cell_mm']:.2f} mm off the board target after {tries} re-placings")
                    # move the place point back by what the arm did: the grasp centre c went to T(c), turned by phi (T: target -> seen)
                    th, ph = math.radians(g["target"][2]), math.radians(mt["phi"])
                    gx, gz = g["gl"][0] * init["pitch_mm"], g["gl"][1] * init["pitch_mm"]
                    c = (tgt["x"] + gx * math.cos(th) - gz * math.sin(th), tgt["z"] + gx * math.sin(th) + gz * math.cos(th))
                    vx, vz = c[0] - tgt["x"], c[1] - tgt["z"]
                    tc = (mt["fixed"]["x"] + vx * math.cos(ph) - vz * math.sin(ph), mt["fixed"]["z"] + vx * math.sin(ph) + vz * math.cos(ph))
                    off = {"pid": pid, "dx_mm": off["dx_mm"] - (tc[0] - c[0]), "dz_mm": off["dz_mm"] - (tc[1] - c[1]),
                           "dyaw_deg": off["dyaw_deg"] - PLACE_YAW_GAIN * mt["phi"]}
                    if abs(off["dyaw_deg"]) > PLACE_OFFSET_MAX_DEG:
                        raise Stop(f"{tgt['name']}: the place point would turn {off['dyaw_deg']:+.2f} deg (> {PLACE_OFFSET_MAX_DEG}): not trusting it")
                    if math.hypot(off["dx_mm"], off["dz_mm"]) > PLACE_OFFSET_MAX_MM:
                        raise Stop(f"{tgt['name']}: the place point would move {math.hypot(off['dx_mm'], off['dz_mm']):.1f} mm "
                                   f"(> {PLACE_OFFSET_MAX_MM}): not trusting it")
                    tries += 1
                    lay = json.loads(json.dumps(lay))
                    lay["pieces"][pid] = {"name": meas["name"], "x": meas["x"], "z": meas["z"], "yaw": meas["yaw"], "on_board": True}
                    say(f"[placed {tgt['name']}] picking it up from the board again; the place point moves by "
                        f"{off['dx_mm']:+.2f},{off['dz_mm']:+.2f} mm {off['dyaw_deg']:+.2f} deg", pid=pid)
        if a.end_check:                                        # back at the photo pose: the top camera reads what was done
            im = robot.read_camera("top")
            look = sorted(set(active) | set(before))
            det = pl.ask("detect", img=im["out"], W=im["W"], H=im["H"], only=look, board_fix=end_fix)
            if not det.get("ok"):
                log["end_check"] = {"refused": det.get("why"), "per": det.get("per")}
                raise Stop(f"end check: the top camera cannot read the pieces ({det.get('why')}): the moves are not confirmed")
            rows = []
            for q in look:
                ref, what, tol = (targets[q], "its board target", END_PLACE_MM) if q in active else (before[q], "where it was at the start", END_MOVED_MM)
                m = pl.need("match", pid=q, est=ref, meas=det["layout"]["pieces"][q])
                rows.append({"pid": q, "name": pieces[q]["name"], "against": "target" if q in active else "start",
                             "off_mm": round(m["worst_cell_mm"], 3), "phi": round(m["phi"], 3), "tol_mm": tol, "ok": m["worst_cell_mm"] <= tol})
                say(f"[end check] {pieces[q]['name']}: {m['worst_cell_mm']:.2f} mm, {m['phi']:+.2f} deg from {what} (<= {tol}: "
                    f"{'ok' if m['worst_cell_mm'] <= tol else 'NOT ok'})", pid=q)
            log["end_check"] = {"rows": rows}
            bad = [r for r in rows if not r["ok"]]
            if bad:
                raise Stop("end check: " + ", ".join(f"{r['name']} {r['off_mm']:.2f} mm from the {r['against']}"
                                                     + (" (pushed?)" if r["against"] == "start" else "") for r in bad))
        say(f"done: back at the photo pose. went back {st8['n_back']} poses for corrections.")
        log["control_done"] = True                         # the control itself ran to the end without a stop (review C60 7)

        if truth is None:                                   # the real arm: no truth to score against
            try:                                            # the board after placing, for the record (D4 3.3 reads it). The moves are done:
                end_png = robot.read_camera("top").get("png")   # a camera failure here does not make the run fail
            except Stop as e:
                end_png = None
                say(f"(the end picture failed: {e})")
            log["end_top_png"] = end_png
            log["result"] = "ok" if len(picks) == len(active) else "ng"     # --place-check: each piece within PLACE_TOL_MM, or it stopped
            say(f"result: {log['result'].upper()} (ran to the end; picked {len(picks)}/{len(active)}) in {time.time() - t_start:.0f} s. "
                f"Placed where: read the end picture {end_png}")
        elif a.place_check:
            # each pick against the world as it was then (re-picks from the board included): the jaws must land as for any pick.
            #   The placed position is the true one at the end (where the true arm last put it) against the board target
            rows = []
            for k in pick_log:
                r = pl.need("score", truth=k["truth"], picks=[k], bias=robot.bias)["rows"][0]
                rows.append(r)
                say(f"  pick {r['name']}: e_a {r['e_a_mm']:+.2f} mm, e_c {r['e_c_mm']:+.2f} mm, yaw {r['phi']:+.2f} deg, need {r['need_mm']:.2f} mm: "
                    + ("OK" if r["need_mm"] < init["fit_margin_mm"] else "NG"))
            ends = []
            for pid in active:
                t = robot.truth["pieces"][pid]
                m = pl.need("match", pid=pid, est=final[pid]["target"], meas={"name": t["name"], "x": t["x"], "z": t["z"], "yaw": t["yaw"]})
                ends.append(m["worst_cell_mm"])
                say(f"  placed {final[pid]['target']['name']}: truly {m['worst_cell_mm']:.2f} mm off the board target "
                    f"(the top camera said {final[pid]['off_mm']:.2f}; re-placed {final[pid]['tries']} times): "
                    + ("OK" if m["worst_cell_mm"] < PLACE_MAX_MM else "NG"))
            # the jaws must land on each pick (need). |e_c| is not asked here: it was the stand-in for the placed error, which the
            #   top camera now measures and the end score takes from the truth
            grip_ok = all(r["need_mm"] < init["fit_margin_mm"] for r in rows)
            log["score"] = {"rows": rows, "placed_true_mm": ends}
            log["result"] = "ok" if grip_ok and all(e < PLACE_MAX_MM for e in ends) and len(ends) == len(active) else "ng"
            missing_ng()
            say(f"result: {log['result'].upper()} (picks {len(pick_log)}, placed within {PLACE_MAX_MM} mm "
                f"{sum(e < PLACE_MAX_MM for e in ends)}/{len(active)}) in {time.time() - t_start:.0f} s")
        elif a.mj:                                          # S2: the physics score (the pieces where MuJoCo has them now)
            rows = robot.score(set(active))
            for r in rows:
                say(f"  [physics] {r['name']}: " + (f"{r['place_err_mm']:.2f} mm from the board target, yaw {r['yaw_err_deg']:+.2f}, tilt {r['tilt_deg']:.1f}, "
                                                    f"lift {r['lift_mm']:.1f}" if r["moved"] else f"not moved: {r['pushed_mm']:.2f} mm from its start, tilt {r['tilt_deg']:.1f}")
                    + f": {'OK' if r['ok'] else 'NG'}")
            log["score"] = {"physics": rows}
            log["result"] = "ok" if all(r["ok"] for r in rows) and len(picks) == len(active) else "ng"
            say(f"result: {log['result'].upper()} (physics: {sum(r['ok'] for r in rows)}/{len(rows)} pieces right) in {time.time() - t_start:.0f} s")
        else:
            sc = pl.need("score", truth=truth, picks=list(picks.values()), bias=getattr(robot, "bias", None))
            log["score"] = sc
            for r in sc["rows"]:
                say(f"  {r['name']}: e_a {r['e_a_mm']:+.2f} mm, e_c {r['e_c_mm']:+.2f} mm, yaw {r['phi']:+.2f} deg, "
                    f"need {r['need_mm']:.2f} < {sc['fit_margin_mm']:.1f} mm, |e_c| < {sc['ec_max_mm']:.1f} mm, "
                    + (f"placed {r['place_mm']:.2f} < {sc['place_max_mm']:.1f} mm (no squeeze {r['place_rigid_mm']:.2f})"
                       if r.get("place_mm") is not None else "placed -")
                    + f": {'OK' if r['ok'] else 'NG'}")
            log["result"] = "ok" if sc["all_ok"] and len(picks) == len(active) else "ng"
            missing_ng()
            say(f"result: {log['result'].upper()} (picked {len(picks)}/{len(active)}, within the margin "
                f"{sum(r['ok'] for r in sc['rows'])}/{len(active)}) in {time.time() - t_start:.0f} s")
    except BaseException as e:   # Stop, Ctrl+C, and also bus errors or bugs: hold, never just let go (same as tools/k1_run.py)
        log["result"] = "stopped"
        log["stop_kind"] = type(e).__name__
        say(f"!! stopped: {e!r}")
        if robot is not None:
            robot.hold()
    finally:
        if robot is not None:
            robot.release()
            if getattr(robot, "trace", None):
                log["trace"] = robot.trace
            if isinstance(robot, Recorder) and a.mj:           # --mj: what the control saw and sent (for --replay), and the truth at the end
                w = robot.robot
                fin = w.piece_states()
                log["mj"] = {"obs": robot.obs, "sent": robot.sent, "final_states": fin, "piece_writes": w.piece_writes,
                             "world_t": round(w.t, 4), "shots": w.rec["shots"], "physics": w.score(set(active)),
                             "world_cannot_draw": log.get("stop_kind") == "WorldCannotDraw"}
                from mj_world import classify_contacts                # S3: contacts (the truth; scoring only, after the control)
                cl = w.contact_log()
                log["mj"].update(cl)
                log["mj"]["contact_verdict"] = classify_contacts(cl, {pieces[q]["name"] for q in active})
                cv = log["mj"]["contact_verdict"]["count"]
                say(f"[contacts] grip {cv['把持']}, forbidden {cv['禁止接触']}, unknown {cv['判定不能']}, piece-to-neighbour "
                    f"{cv['対象と隣']}, arm-table {cv['腕と机']}; max load " + ", ".join(f"{k} {v:.2f}" for k, v in cl["max_load_nm"].items()))
                if log.get("result") != "ok":
                    say("[physics at the stop] " + ", ".join(
                        f"{r['name']} " + (f"{r['place_err_mm']:.2f} mm from target" if r["moved"] else f"{r['pushed_mm']:.2f} mm from start")
                        + (" ok" if r["ok"] else " NG") for r in log["mj"]["physics"]))
            if isinstance(robot, ReplayRobot):
                log["replay"] = {"sent": robot.sent, "left": len(robot.seq)}
            if isinstance(robot, Recorder) and not a.sim:     # S4: the real arm's observations and commands (--replay), and where it stopped
                log["rec"] = {"obs": robot.obs, "sent": robot.sent}
            if isinstance(robot, (Recorder, ReplayRobot)) and log.get("result") == "stopped":
                log["stop_at"] = {"n_obs": len(robot.obs) if isinstance(robot, Recorder) else log["replay_of"]["n_obs"] - len(robot.seq),
                                  "n_sent": len(robot.sent),
                                  "msg": next((e["msg"] for e in reversed(log["events"]) if e["msg"].startswith("!! stopped")), "")[:400]}
        pl.close()
        out = os.path.join(OUT, ("replay_" if a.replay else "run_") + stamp + ".json")   # 点検 C63 の 8: a replay is not a run
        with open(out, "w", encoding="utf-8") as fo:
            json.dump(log, fo, indent=1, ensure_ascii=False)
        print(f"wrote {os.path.normpath(out)}", flush=True)
    sys.exit(0 if log.get("result") == "ok" else 1)


if __name__ == "__main__":
    main()
