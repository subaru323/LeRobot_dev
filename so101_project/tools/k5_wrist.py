r"""W4 (K5): where the wrist camera sits on the gripper (hand-eye), from photos of the K1/K5 sheet v2.

usage (the .venv python, it needs OpenCV):
  python tools\k5_wrist.py poses [--n 10] [--jaw 0] [--seed 3] [--out FILE]
        no port, no camera. Plans the recommended poses (URDF deg) with the sim's arm: 4+ markers of sheet v2 clear of the
        jaw's silhouette (STL of the fixed + moving jaw, grown PLAN_MASK_GROW_PX) for the URDF camera AND for a camera moved
        off it like the selftest's; the arm clear of itself (sim/self-collision.js), of the C rig (so101.js overheadRig() boxes)
        and of the table; wrist_flex and wrist_roll both turned. Checks itself: photos drawn with the jaw over them -> solve
        must come back. Writes outputs\k5\wrist_poses.json and a contact sheet .png. Poses are ordered for less hand travel.
  python tools\k5_wrist.py capture INDEX DIR --k1 FILE [PORT] [--plan outputs\k5\wrist_poses.json] [--width 640 ...]
        NOT RUN YET (opens the follower port). Torque stays OFF: you hold the arm still in a pose where the wrist camera sees
        4 or more markers of the sheet, press SPACE: the tool reads the joints twice (before and after the frame) and keeps
        the shot only if they agree within STILL_DEG. Q quits. Writes DIR\NNN.png and DIR\poses.json.
        8-12 poses; turn the wrist (wrist_flex AND wrist_roll) and move around the sheet so the views differ by 30 deg or more.
        --plan shows the next recommended pose and each joint's distance to it (green within PLAN_NEAR_DEG); a guide only.
  python tools\k5_wrist.py solve DIR --intrinsics FILE --k1 FILE [--ruler-mm 150.0[,150.0]]
        DIR\poses.json + the photos -> outputs\k5\wrist_cam_<robot id>.json (the form agreed with sim/ on 2026-11-19,
        docs/plan/進め方_利用者の稼働を絞る.md, the W4 row) + one overlay picture per photo (red rings on the K1 points).
  python tools\k5_wrist.py selftest
        the sim's own arm (node, sim/so101.js urdfFK): a wrist camera moved off the URDF by a known amount, synthetic photos
        of sheet v2 from several poses, solve, compare. Negative checks: swapped joint readings, poses too alike, one joint
        only (wrist_roll alone, wrist_flex alone), 3 photos. --seed N changes the random poses.
        This checks the MATH only: the photos and the solve use the same forward kinematics and the true K; no jaw in the
        picture, no lens distortion, no gear play. It is not the accuracy on the real arm.
Refuses (writes nothing) when: fewer than MIN_POSES photos, the gripper turned less than MIN_SPREAD_DEG or about one axis
only, rms over RMS_MAX_PX, one photo over PHOTO_MAX_PX, the fit's 1-sigma over SD_T_MAX_MM / SD_ROT_MAX_DEG, or the camera
farther than DELTA_MAX_MM / DELTA_TILT_MAX_DEG from the URDF's (a broken solve; roll-only photos land there).

Output json:
  image_size, K, dist           the wrist camera's intrinsics (from k5_camera.py intrinsics --name wrist --board small)
  cam_in_gripper {t_mm, R}      the camera in URDF gripper_link coordinates: t_mm = optical centre [mm];
                                R = 9 numbers, row-major; its COLUMNS are the camera's x (image right), y (image down),
                                z (optical axis) in gripper_link. Same axes as sim/'s wrist_camera_link
                                (sim/vision-wrist.js cameraAt: dir = link z, image up = -link y).
  urdf_delta                    the solved camera minus the URDF's wrist_camera_mount o wrist_camera:
                                dt_mm (gripper_link), rot_deg (angle), axis_tilt_deg (optical axis vs the URDF's),
                                rotvec_cam_deg (axis*angle in the URDF camera's axes), roll_about_axis_deg (its part about
                                the optical axis: the picture turned). Only a guide: it is relative to so101.js's URDF copy
  k1_sha256, robot_id           the k1 file whose sign/offset turned the readings into URDF angles (a new k1 -> solve again;
                                the photos stay valid while the motors' calibration (calib_key in poses.json) is the same)
  jaw_lerobot                   the gripper reading when the photos were taken (median; the jaw does not move gripper_link)
  rms_px                        reprojection rms over all marker corners of all photos
  also: per_photo_max_px, sd_t_mm / sd_rot_deg (1-sigma of the fit), sheet_in_base (where the solve put the sheet: a check),
        so101_js_sha256, pixels (K is OpenCV's; the sim's pixel 0 is the corner: +0.5 to cx, cy), use
What it can NOT separate: a zero offset of wrist_roll turns the solved camera about the wrist_roll axis (gripper_link z),
exactly, with no rise in rms: both its turn and its place change (selftest: +2 deg -> 2.0 deg and 2.7 mm), while the sim's
jaw stays where the URDF puts it. Other joint offsets bias it and raise the rms: 1 deg of wrist_flex or shoulder_lift is
refused, 0.3 deg passes with the camera about 0.6-0.8 mm off. So the camera is only as good as K1: run it with the k1 file
after K1 (D3).
"""

import argparse
import datetime
import glob
import hashlib
import json
import math
import os
import subprocess
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import k5_camera as K5   # noqa: E402  (sheet geometry, marker detection, synthetic render)

ROOT = K5.ROOT
SIM = os.path.join(ROOT, "sim")
OUT = K5.OUT
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
MIN_POSES = 6          # photos with 4+ markers
MIN_SPREAD_DEG = 30.0  # largest angle between two gripper orientations; below it the rotation is poorly fixed
RMS_MAX_PX = 1.5       # over this the joint readings and the photos do not agree (wrong k1, moved during the shot, wrong pairing).
                       # GUESS: in the selftest a 1 deg wrist_flex / shoulder_lift offset gives 4.0 / 2.3 px; real gear play and
                       # reading noise are not known yet. If good real photos stop here, look at per_photo_max_px before raising it
MIN_AXIS_DEG = 30.0    # the turns between photos must be about axes at least this far apart (not wrist_roll alone)
PHOTO_MAX_PX = 4.0     # GUESS (clean selftest photos reach 2.7 px): one photo off by more than this anywhere -> named and refused (one bad photo hides in the rms)
SD_T_MAX_MM, SD_ROT_MAX_DEG = 0.5, 0.2   # GUESS: 1-sigma of the solved camera from the fit; more means the poses do not fix it
DELTA_MAX_MM, DELTA_TILT_MAX_DEG = 15.0, 15.0   # GUESS: farther than this from the URDF's camera = a broken solve or another mount.
                       # A turn about the optical axis alone is only warned about (the module can sit turned in its holder)
# The selftest does NOT model: the jaw and fingers covering part of the picture, lens distortion, blur, a sheet that is not flat.
STILL_DEG = 0.3        # capture: the two joint reads around the frame must agree this well
PLAN_NEAR_DEG = 5.0    # capture --plan: a joint within this of the target shows green (a guide only; the shot is kept anyway)
BUFFER_DROP = 4        # capture: frames thrown away after the first joint read (old frames in the driver's buffer)
SELF_W, SELF_H, SELF_FOV = 640, 480, 60.0   # selftest camera (sim's CAM_FOV.wrist = 60, live.js 640x480)
SELF_T_TOL_MM, SELF_R_TOL_DEG = 0.5, 0.2    # selftest pass: solved camera vs the true one

# node: k1 conversion (same formula as sim/robot-map.js toUrdf: q_urdf = sign * (L - offset)) and sim/so101.js urdfFK.
# in: {k1: {sign, offset} | null, poses: [{L: {joint: deg}} | {q: [5 urdf deg]}]}
# out: {ty_mm, X0: {R, t_mm}, poses: [{q, G: {R, t_mm}}]}  (G = gripper_link in the sim screen frame, base_link origin = 0, mm)
FK_JS = r"""
const {load}=require('./tools/_load'); load('kinematics.js','so101.js');
const fs=require('fs'); const inp=JSON.parse(fs.readFileSync(0,'utf8')), D=Math.PI/180, J=SO101.ARM;
function frame(F){return {R:F.R, t_mm:F.p.map(v=>v*1000)};}
function rel(A,B){ // A^-1 B (both {R,p})
  const At=[A.R[0],A.R[3],A.R[6],A.R[1],A.R[4],A.R[7],A.R[2],A.R[5],A.R[8]], d=[B.p[0]-A.p[0],B.p[1]-A.p[1],B.p[2]-A.p[2]];
  return {R:SO101.mm(At,B.R), t_mm:SO101.mv(At,d).map(v=>v*1000)};
}
const Z=SO101.urdfFK([0,0,0,0,0],0);
const out={ty_mm:SO101.overheadTableY()*1000, X0:rel(Z.gripper_link,Z.wrist_camera_link), limits:SO101.limitsDeg, poses:[]};
for(const p of inp.poses){
  const q=p.q? p.q : J.map(m=>inp.k1.sign[m]*(p.L[m]-inp.k1.offset[m]));
  const F=SO101.urdfFK(q.map(v=>v*D),0);
  out.poses.push({q, G:frame(F.gripper_link)});
}
console.log(JSON.stringify(out));
"""


def node_fk(poses, k1=None):
    r = subprocess.run(["node", "-e", FK_JS], cwd=SIM, input=json.dumps({"k1": k1, "poses": poses}),
                       capture_output=True, text=True, encoding="utf-8")
    if r.returncode:
        sys.exit("node (sim/so101.js) failed: " + r.stderr[-400:])
    return json.loads(r.stdout)


# ---------------------------------------------------------------- 4x4 helpers (mm)
def T_of(R, t):
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, float).reshape(3, 3)
    T[:3, 3] = np.asarray(t, float).ravel()
    return T


def T_rv(rv, t):
    return T_of(cv2.Rodrigues(np.asarray(rv, float).reshape(3, 1))[0], t)


def inv(T):
    R, t = T[:3, :3], T[:3, 3]
    return T_of(R.T, -R.T @ t)


def rot_deg(R):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2))))


def sheet_obj(ids, scale):
    """marker corners in the sheet frame (sim table axes, y = 0 on the sheet), OpenCV corner order."""
    return {i: K5.marker_corners(i) * [scale, 1, scale] for i in ids}


# ---------------------------------------------------------------- the solve
def solve(views, X0, K, dist, scale=1.0):
    """views: [{G: 4x4 gripper_link in base (mm), found: {id: 4x2 corners}}]. Unknowns: X = camera in gripper_link,
    S = sheet in base. Start: OpenCV hand-eye (Park) from per-photo solvePnP, then least squares on all corners."""
    use = [v for v in views if len(v["found"]) >= 4]
    if len(use) < MIN_POSES:
        return {"ok": False, "why": f"{len(use)} photos with 4+ markers; need {MIN_POSES} or more"}
    Rs = [v["G"][:3, :3] for v in use]
    spread = max(rot_deg(a.T @ b) for a in Rs for b in Rs)
    if spread < MIN_SPREAD_DEG:
        return {"ok": False, "why": f"the gripper turned at most {spread:.1f} deg between photos (need {MIN_SPREAD_DEG:.0f}): "
                                    "turn wrist_flex and wrist_roll more between shots", "spread_deg": spread}
    # the turns between photos must be about 2 or more different axes: turning only wrist_roll (one axis) leaves the camera's
    # place along that axis and its turn about it free, and the rms stays small (点検 C42m の 1: roll only -> 100 mm off, rms 0.4 px)
    axes = []
    for i in range(len(Rs)):
        for j in range(i + 1, len(Rs)):
            rv_ = cv2.Rodrigues(Rs[i].T @ Rs[j])[0].ravel()
            if np.linalg.norm(rv_) > math.radians(10):
                axes.append(rv_ / np.linalg.norm(rv_))
    axis_spread = max((math.degrees(math.acos(min(1.0, abs(float(a @ b))))) for a in axes for b in axes), default=0.0)
    if axis_spread < MIN_AXIS_DEG:
        return {"ok": False, "spread_deg": spread, "axis_spread_deg": axis_spread,
                "why": f"the gripper turned about one axis only (axes within {axis_spread:.1f} deg; need {MIN_AXIS_DEG:.0f}): "
                       "change BOTH wrist_flex and wrist_roll (and the arm) between shots"}
    try:
        return _solve(use, X0, K, dist, scale, spread, axis_spread)
    except (np.linalg.LinAlgError, cv2.error) as ex:
        return {"ok": False, "why": f"the solve broke down ({type(ex).__name__}): the photos do not fix the camera; take more varied poses"}


def _solve(use, X0, K, dist, scale, spread, axis_spread):
    Rg, tg, Rt, tt, Tcs = [], [], [], [], []
    for v in use:
        obj = sheet_obj(sorted(v["found"]), scale)
        o = np.vstack([obj[i] for i in sorted(v["found"])])
        im = np.vstack([v["found"][i] for i in sorted(v["found"])]).astype(np.float64)
        ok, rv, tv = cv2.solvePnP(o, im, K, dist, flags=cv2.SOLVEPNP_SQPNP)
        rv, tv = cv2.solvePnPRefineLM(o, im, K, dist, rv, tv)
        Tcs.append(T_rv(rv, tv))                         # sheet -> camera
        Rg.append(v["G"][:3, :3]); tg.append(v["G"][:3, 3].reshape(3, 1))
        Rt.append(Tcs[-1][:3, :3]); tt.append(Tcs[-1][:3, 3].reshape(3, 1))
    Rx, tx = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=cv2.CALIB_HAND_EYE_PARK)
    X = T_of(Rx, tx)
    Ss = [v["G"] @ X @ Tc for v, Tc in zip(use, Tcs)]   # sheet in base, one per photo
    U, _, Vt = np.linalg.svd(sum(s[:3, :3] for s in Ss))
    S = T_of(U @ Vt, np.mean([s[:3, 3] for s in Ss], axis=0))

    data = []
    for v in use:
        obj = sheet_obj(sorted(v["found"]), scale)
        data.append((v["G"], np.vstack([obj[i] for i in sorted(v["found"])]),
                     np.vstack([v["found"][i] for i in sorted(v["found"])]).astype(np.float64)))

    def unpack(p):
        return T_rv(p[0:3], p[3:6]), T_rv(p[6:9], p[9:12])

    def resid(p):
        Xp, Sp = unpack(p)
        out = []
        for G, o, im in data:
            Tcs_ = inv(Xp) @ inv(G) @ Sp
            pr, _ = cv2.projectPoints(o, cv2.Rodrigues(Tcs_[:3, :3])[0], Tcs_[:3, 3], K, dist)
            out.append((pr.reshape(-1, 2) - im).ravel())
        return np.concatenate(out)

    p = np.concatenate([cv2.Rodrigues(X[:3, :3])[0].ravel(), X[:3, 3], cv2.Rodrigues(S[:3, :3])[0].ravel(), S[:3, 3]])
    lam, r = 1e-3, resid(p)
    for _ in range(100):                                # Levenberg-Marquardt, numerical Jacobian
        Jm = np.empty((r.size, 12))
        for k in range(12):
            h = 1e-6 if k % 6 < 3 else 1e-4
            d = np.zeros(12); d[k] = h
            Jm[:, k] = (resid(p + d) - r) / h
        A, g = Jm.T @ Jm, Jm.T @ r
        while True:
            step = -np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), g)
            r2 = resid(p + step)
            if r2 @ r2 < r @ r:
                p, r, lam = p + step, r2, max(lam / 10, 1e-9)
                break
            lam *= 10
            if lam > 1e8:
                break
        if lam > 1e8 or np.abs(step).max() < 1e-9:
            break
    X, S = unpack(p)
    e = np.linalg.norm(r.reshape(-1, 2), axis=1)
    rms = float(math.sqrt((e ** 2).mean()))
    per, k0 = [], 0
    for G, o, _ in data:
        per.append(float(e[k0:k0 + len(o)].max())); k0 += len(o)
    # how well the photos fix the camera: 1-sigma from (J^T J)^-1 scaled by the residual
    Jm = np.empty((r.size, 12))
    for k in range(12):
        h = 1e-6 if k % 6 < 3 else 1e-4
        d = np.zeros(12); d[k] = h
        Jm[:, k] = (resid(p + d) - r) / h
    cov = np.linalg.pinv(Jm.T @ Jm) * float(r @ r) / max(1, r.size - 12)
    sd_rot = math.degrees(math.sqrt(max(np.linalg.eigvalsh(cov[0:3, 0:3]).max(), 0)))
    sd_t = math.sqrt(max(np.linalg.eigvalsh(cov[3:6, 3:6]).max(), 0))
    D = inv(X0) @ X                                     # in the URDF camera's axes
    rv = cv2.Rodrigues(D[:3, :3])[0].ravel()
    tilt = math.degrees(math.acos(min(1.0, float(X0[:3, 2] @ X[:3, 2]))))   # optical axis vs the URDF's
    dt = X[:3, 3] - X0[:3, 3]
    res = {"ok": True, "X": X, "S": S, "rms_px": rms, "per_photo_max_px": per, "n_photos": len(use),
           "spread_deg": spread, "axis_spread_deg": axis_spread, "sd_t_mm": sd_t, "sd_rot_deg": sd_rot,
           "urdf_delta": {"dt_mm": dt.tolist(), "rot_deg": rot_deg(D[:3, :3]), "axis_tilt_deg": tilt,
                          "rotvec_cam_deg": np.degrees(rv).tolist(), "roll_about_axis_deg": float(math.degrees(rv[2]))}}
    bad = [str(v.get("image", k)) for k, (v, m) in enumerate(zip(use, per)) if m > PHOTO_MAX_PX]
    why = []
    if rms > RMS_MAX_PX:
        why.append(f"rms {rms:.2f} px over {RMS_MAX_PX} px: the joint readings and the photos do not agree "
                   "(another k1 file, the arm moved during a shot, photos and readings out of order, or wrong intrinsics)")
    if bad:
        why.append(f"photos {bad} are off by more than {PHOTO_MAX_PX} px somewhere (moved, blurred, misread marker): drop or retake them")
    if sd_t > SD_T_MAX_MM or sd_rot > SD_ROT_MAX_DEG:
        why.append(f"the photos fix the camera only to {sd_t:.2f} mm / {sd_rot:.2f} deg (1 sigma; need {SD_T_MAX_MM} / {SD_ROT_MAX_DEG}): more varied poses")
    if np.linalg.norm(dt) > DELTA_MAX_MM or tilt > DELTA_TILT_MAX_DEG:
        why.append(f"the camera came out {np.linalg.norm(dt):.1f} mm and {tilt:.1f} deg (optical axis) from the URDF's "
                   f"(limits {DELTA_MAX_MM} mm, {DELTA_TILT_MAX_DEG} deg): a wrong solve or a different mount -> do not use it")
    if why:
        res["ok"], res["why"] = False, " / ".join(why)
    return res


# ---------------------------------------------------------------- solve (real photos)
def load_k1(path):
    raw = open(path, "rb").read()
    k = json.loads(raw.decode("utf-8"))
    if not k.get("verified"):
        sys.exit(f"{path}: not verified by k1_calib.py")
    return k, hashlib.sha256(raw).hexdigest()


def k1_calib_key(k1):
    """the motors' calibration the k1 file was made with, as the same string as k1_run.calib_key / sim/robot-map.js calibKey."""
    if "calibration" not in k1:
        return None
    return ";".join(f"{m}:{c['id']},{c['drive_mode']},{c['homing_offset']},{c['range_min']},{c['range_max']}"
                    for m, c in ((m, k1["calibration"][m]) for m in JOINTS + ["gripper"]))


def file_sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def cmd_solve(a):
    K, dist, size = K5.load_intrinsics(a.intrinsics)
    k1, sha = load_k1(a.k1)
    shots = json.load(open(os.path.join(a.dir, "poses.json"), encoding="utf-8"))
    # the raw readings depend on the motors' calibration only; the k1 file (sign, offset) may be newer (K1 measured again)
    if shots.get("calib_key") and k1_calib_key(k1) and shots["calib_key"] != k1_calib_key(k1):
        sys.exit("the photos were taken with another motor calibration than this k1 file's -> take them again")
    if shots.get("k1_sha256") and shots["k1_sha256"] != sha:
        print("!! the k1 file differs from the one at capture (same motor calibration): solving with this one")
    rl = [float(v) for v in a.ruler_mm.split(",")]
    scale = sum(rl) / len(rl) / K5.RULER_MM
    fk = node_fk([{"L": s["lerobot"]} for s in shots["shots"]], k1)
    X0 = T_of(fk["X0"]["R"], fk["X0"]["t_mm"])
    views, grays = [], []
    for s, f in zip(shots["shots"], fk["poses"]):
        g = cv2.imread(os.path.join(a.dir, s["image"]), cv2.IMREAD_GRAYSCALE)
        if g is None:
            sys.exit(f"cannot read {s['image']}")
        if (g.shape[1], g.shape[0]) != size:
            sys.exit(f"{s['image']}: {g.shape[1]}x{g.shape[0]} but the intrinsics are for {size[0]}x{size[1]}")
        found = K5.detect_markers(g)
        print(f"  {s['image']}: markers {sorted(found)}")
        views.append({"G": T_of(f["G"]["R"], f["G"]["t_mm"]), "found": found, "image": s["image"]})
        grays.append(g)
    r = solve(views, X0, K, dist, scale)
    if "X" not in r:
        sys.exit("solve: " + r["why"])
    jaw = float(np.median([s["lerobot"]["gripper"] for s in shots["shots"]]))
    X, S = r["X"], r["S"]
    out = {"image_size": list(size), "K": K.tolist(), "dist": dist.ravel().tolist(),
           "cam_in_gripper": {"t_mm": X[:3, 3].tolist(), "R": X[:3, :3].ravel().tolist()},
           "urdf_delta": r["urdf_delta"], "k1_sha256": sha, "robot_id": k1.get("robot_id"), "jaw_lerobot": jaw,
           "rms_px": r["rms_px"], "date": datetime.datetime.now().isoformat(timespec="seconds"),
           "per_photo_max_px": r["per_photo_max_px"], "n_photos": r["n_photos"], "spread_deg": r["spread_deg"],
           "sheet_in_base": {"t_mm": S[:3, 3].tolist(), "R": S[:3, :3].ravel().tolist(), "ty_sim_mm": fk["ty_mm"]},
           "intrinsics": os.path.basename(a.intrinsics), "ruler_mm": rl, "photos": a.dir,
           "axis_spread_deg": r["axis_spread_deg"], "sd_t_mm": r["sd_t_mm"], "sd_rot_deg": r["sd_rot_deg"],
           "so101_js_sha256": file_sha(os.path.join(SIM, "so101.js")),
           "frame": "cam_in_gripper: URDF gripper_link; camera axes OpenCV (x right, y down, z forward) = sim wrist_camera_link",
           "pixels": "K is OpenCV's (pixel centre = integer). sim/so101.js overheadImage puts 0 at the pixel's top-left corner: "
                     "add 0.5 to cx, cy for the sim. Another image size: scale fx, fy, cx+0.5, cy+0.5 by the size ratio",
           "use": "readers use cam_in_gripper, K, dist. urdf_delta is only a guide (it is relative to so101.js's URDF copy, "
                  "so101_js_sha256)"}
    d = r["urdf_delta"]
    print(f"rms {r['rms_px']:.2f} px over {r['n_photos']} photos (worst photo {max(r['per_photo_max_px']):.2f} px), "
          f"gripper turned up to {r['spread_deg']:.0f} deg about axes {r['axis_spread_deg']:.0f} deg apart; "
          f"camera fixed to {r['sd_t_mm']:.2f} mm / {r['sd_rot_deg']:.3f} deg (1 sigma)")
    print(f"camera minus URDF: dt ({d['dt_mm'][0]:+.1f}, {d['dt_mm'][1]:+.1f}, {d['dt_mm'][2]:+.1f}) mm, "
          f"turned {d['rot_deg']:.2f} deg (about the optical axis {d['roll_about_axis_deg']:+.2f})")
    print(f"sheet in base: ({S[0, 3]:.1f}, {S[1, 3]:.1f}, {S[2, 3]:.1f}) mm (sim's table at y {fk['ty_mm']:.1f}); "
          f"sheet tilted {math.degrees(math.acos(min(1, S[1, 1]))):.2f} deg")
    if not r["ok"]:
        sys.exit("solve: " + r["why"] + "  (nothing written)")
    if abs(d["roll_about_axis_deg"]) > 20:
        print("!! the picture is turned more than 20 deg against the sim's wrist camera (image up = -link y): check the camera "
              "is mounted the right way round; the sim draws the wrist picture with the URDF's turn")
    od = a.out or OUT
    os.makedirs(od, exist_ok=True)
    p = os.path.join(od, f"wrist_cam_{k1.get('robot_id', 'robot')}.json")
    json.dump(out, open(p, "w"), indent=1)
    pts = np.array([[x, 0, z] for x, z in K5.k1_points()], np.float64) * [scale, 1, scale]
    for v, g in zip(views, grays):
        Tc = inv(X) @ inv(v["G"]) @ S
        pr, _ = cv2.projectPoints(pts, cv2.Rodrigues(Tc[:3, :3])[0], Tc[:3, 3], K, dist)
        ov = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        for q in pr.reshape(-1, 2):
            if np.isfinite(q).all() and -50 < q[0] < g.shape[1] + 50 and -50 < q[1] < g.shape[0] + 50:
                cv2.circle(ov, (int(q[0]), int(q[1])), 6, (0, 0, 255), 2)
        cv2.imwrite(os.path.join(od, f"wrist_overlay_{os.path.splitext(v['image'])[0]}.png"), ov)
    print(f"wrote {p} (+ wrist_overlay_*.png: red rings must sit on the printed K1 points)")


# ---------------------------------------------------------------- capture (opens the follower port; NOT RUN YET)
def cmd_capture(a):
    from so101_common import FOLLOWER_PORT, check_motors_ok, connect_bus_no_torque, make_follower
    k1, sha = load_k1(a.k1)
    os.makedirs(a.dir, exist_ok=True)
    pj = os.path.join(a.dir, "poses.json")
    shots = json.load(open(pj, encoding="utf-8")) if os.path.exists(pj) else \
        {"k1_sha256": sha, "calib_key": k1_calib_key(k1), "camera": {"index": a.index, "width": a.width, "height": a.height,
                                                                     "fps": a.fps, "fourcc": a.fourcc}, "shots": []}
    if shots.get("calib_key") != k1_calib_key(k1):
        sys.exit(f"{pj} was taken with another motor calibration -> use a new DIR")
    robot = make_follower(a.port or FOLLOWER_PORT)
    bus = robot.bus
    cap = None
    connect_bus_no_torque(bus)            # torque stays OFF: nothing moves by itself
    try:
        check_motors_ok(bus, "follower")
        from k1_run import calib_key
        if k1_calib_key(k1) is None or calib_key(bus.calibration) != k1_calib_key(k1):
            sys.exit("the calibration in the motors is not the one in the k1 file -> run k1_calib.py again")
        print("Keep the gripper at ONE opening for all shots (fully open or closed). Keep your hands off the sheet.")
        cap = cv2.VideoCapture(a.index, cv2.CAP_DSHOW)
        if a.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*a.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height); cap.set(cv2.CAP_PROP_FPS, a.fps)
        for prop, val, name in ((cv2.CAP_PROP_AUTOFOCUS, 0, "autofocus off"), (cv2.CAP_PROP_AUTO_EXPOSURE, 0.25, "auto exposure off")):
            print(f"  {name}: {'ok' if cap.set(prop, val) else 'not supported by this camera/driver'}")
        print("torque is OFF. Hold the arm still with the wrist camera on the sheet; SPACE = shot, Q = quit")
        n = len(shots["shots"])
        # --plan: the recommended poses (k5_wrist.py poses) as LeRobot readings through the k1 file (L = offset + q / sign)
        targets = []
        if a.plan:
            for pz in json.load(open(a.plan, encoding="utf-8"))["poses"]:
                targets.append({m: k1["offset"][m] + pz["q_urdf"][i] / k1["sign"][m] for i, m in enumerate(JOINTS)})
            print(f"{len(targets)} target poses from {a.plan}; the screen shows how far each joint is from the next one. "
                  f"Any pose with 4+ markers is fine: the targets are a guide (the real camera is a few mm off the sim's)")
        while True:
            ok, frame = cap.read()
            if not ok:
                sys.exit("cannot read from the camera")
            g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found = K5.detect_markers(g)
            show = frame.copy()
            for c in found.values():
                cv2.polylines(show, [c.astype(np.int32)], True, (0, 255, 0), 2)
            cv2.putText(show, f"markers {sorted(found)}  shots {n}  SPACE shot  Q quit", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if targets:
                ti = min(n, len(targets) - 1)
                now = dict(bus.sync_read("Present_Position", num_retry=1))
                diff = [now[m] - targets[ti][m] for m in JOINTS]
                near = max(abs(d) for d in diff) <= PLAN_NEAR_DEG
                cv2.putText(show, f"target {ti + 1}/{len(targets)}" + ("  (all done: free poses)" if n >= len(targets) else ""),
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0) if near else (0, 0, 255), 2)
                for k, (m, d) in enumerate(zip(JOINTS, diff)):
                    cv2.putText(show, f"{m:<13} now-target {d:+6.1f}", (10, 75 + 20 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 200, 0) if abs(d) <= PLAN_NEAR_DEG else (0, 0, 255), 1)
            cv2.imshow("k5 wrist", show)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k != ord(" "):
                continue
            j0 = dict(bus.sync_read("Present_Position", num_retry=3))
            for _ in range(BUFFER_DROP):          # DSHOW may hand back frames taken before the key press
                cap.read()
            ok, frame = cap.read()
            j1 = dict(bus.sync_read("Present_Position", num_retry=3))
            moved = max(abs(j0[m] - j1[m]) for m in JOINTS)
            nm = len(K5.detect_markers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))) if ok else 0
            if not ok or moved > STILL_DEG or nm < 4:
                print(f"  not kept: moved {moved:.2f} deg (max {STILL_DEG}), markers {nm} (need 4)")
                continue
            name = f"{n:03d}.png"
            cv2.imwrite(os.path.join(a.dir, name), frame)
            shots["shots"].append({"image": name, "lerobot": {m: (j0[m] + j1[m]) / 2 for m in j0}, "markers": nm})
            json.dump(shots, open(pj, "w", encoding="utf-8"), indent=1)
            n += 1
            print(f"  kept {name}: markers {nm}")
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        bus.disconnect(disable_torque=False)


# ---------------------------------------------------------------- recommended poses (plan; no port, no camera)
# node: the occluders of the wrist picture (the fixed jaw + gripper motor = group j5, the moving jaw) in the URDF camera's
# axes, and for each pose: self-clearance (sim/self-collision.js), clearance to the C rig boxes (so101.js overheadRig(),
# {id, name, lo, hi} since 2026-11-19) and the lowest arm point above the table. All mm, base_link origin = 0.
# in: {jaw (URDF deg), qs: [[5 URDF deg]...], every (vertex stride for the rig/table check), tableLift (mm, negative check)}
POSES_JS = r"""
const {load}=require('./tools/_load');
load('kinematics.js','so101.js','models/so101_visuals.js','models/so101_tris.js','chikuwa-collision.js','self-collision.js');
const fs=require('fs'); const inp=JSON.parse(fs.readFileSync(0,'utf8')), D=Math.PI/180, SC=SELF_COLLIDE;
const GT=SC.groupTris(), ty=SO101.overheadTableY()*1000+(inp.tableLift||0);
const rig=SO101.overheadRig().map(b=>({id:b.id,name:b.name,lo:b.lo.map(v=>v*1000),hi:b.hi.map(v=>v*1000)}));
function place(q,g,jaw){const st=KIN.fk(SO101.axes,q,SO101.tool), P=SC.placeOf(st,g,jaw);
  return x=>{const v=[x[0]-P.o[0],x[1]-P.o[1],x[2]-P.o[2]], r=P.R?KIN.matVec(P.R,v):v; return [(r[0]+P.p[0])*1000,(r[1]+P.p[1])*1000,(r[2]+P.p[2])*1000];};}
// occluders in the camera axes (q = 0; they ride on gripper_link, so the same for every pose)
const Z=SO101.urdfFK([0,0,0,0,0],(inp.jaw||0)*D).wrist_camera_link, Rt=[Z.R[0],Z.R[3],Z.R[6],Z.R[1],Z.R[4],Z.R[7],Z.R[2],Z.R[5],Z.R[8]];
const occ=[];
for(const g of ['j5','jaw']){const f=place([0,0,0,0,0],g,inp.jaw||0);
  for(const t of GT[g]){occ.push(t.map(x=>{const w=f(x);return SO101.mv(Rt,[w[0]-Z.p[0]*1000,w[1]-Z.p[1]*1000,w[2]-Z.p[2]*1000]);}));}}
function boxDist(p,lo,hi){const d=[0,1,2].map(k=>Math.max(lo[k]-p[k],0,p[k]-hi[k])), o=Math.hypot(d[0],d[1],d[2]);
  return o>0?o:-Math.min(...[0,1,2].map(k=>Math.min(p[k]-lo[k],hi[k]-p[k])));}
const every=inp.every||5, out={ty_mm:ty, rig:rig, occ:inp.occ?occ:undefined, poses:[]};
for(const q of inp.qs){
  const s=SC.pose(q,inp.jaw||0); let rigMin=Infinity, rigId=null, rigG=null, low=Infinity, lowG=null;
  for(const g of SC.GROUPS){ if(g==='base') continue; const f=place(q,g,inp.jaw||0); let n=0;
    for(const t of GT[g]) for(const x of t){ if((n++)%every) continue; const w=f(x);
      if(g!=='j1'&&w[1]-ty<low){low=w[1]-ty; lowG=g;}   // j1 only turns about the vertical axis: its height never changes
      for(const b of rig){const d=boxDist(w,b.lo,b.hi); if(d<rigMin){rigMin=d; rigId=b.id; rigG=g;}}}}
  out.poses.push({self_mm:s.clearance*1000, self_pair:s.pair, rig_mm:rigMin, rig_id:rigId, rig_group:rigG, table_mm:low, table_group:lowG});
}
console.log(JSON.stringify(out));
"""
PLAN_SELF_MIN_MM = 5.0     # GUESS: arm-to-arm clearance (sim's boxes are fatter than the parts, so this is on the safe side)
PLAN_RIG_MIN_MM = 30.0     # GUESS: away from the C rig boxes (hands holding the arm are not exact)
PLAN_TABLE_MIN_MM = 15.0   # GUESS: lowest arm point above the sheet / plywood
PLAN_MASK_GROW_PX = 6      # the jaw silhouette is grown by this before a marker counts as covered (real jaw a bit off the STL)
PLAN_N = 10
PLAN_MIN_FLEX_RANGE, PLAN_MIN_ROLL_RANGE = 30.0, 60.0   # the chosen set must turn wrist_flex and wrist_roll by at least this
PLAN_FLEX_BY, PLAN_FLEX_EACH = 15.0, 3   # and have 3+ poses 15 deg or more less bent than the most bent pose, and 3+ near it
PLAN_LIMIT_MARGIN_DEG = 5.0              # poses stay this far inside the URDF joint limits


def node_poses(qs, jaw, occ=False, every=5, table_lift=0.0):
    r = subprocess.run(["node", "-e", POSES_JS], cwd=SIM,
                       input=json.dumps({"qs": qs, "jaw": jaw, "occ": occ, "every": every, "tableLift": table_lift}),
                       capture_output=True, text=True, encoding="utf-8")
    if r.returncode:
        sys.exit("node (sim/self-collision.js, so101.js overheadRig) failed: " + r.stderr[-400:])
    return json.loads(r.stdout)


def jaw_mask(occ, K, size):
    """the jaw / gripper silhouette in the wrist picture (URDF camera, OpenCV axes = wrist_camera_link axes)."""
    W, H = size
    m = np.zeros((H, W), np.uint8)
    for tri in occ:
        P = np.asarray(tri, float)
        if (P[:, 2] < 1.0).any():          # behind or at the lens: dropped (like vision.js does)
            continue
        uv = (K @ P.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        if (np.abs(uv) > 1e5).any():
            continue
        cv2.fillConvexPoly(m, np.round(uv * 16).astype(np.int32), 255, lineType=cv2.LINE_8, shift=4)
    return m


def markers_clear(Tcs, K, mask):
    """marker_view_ok, then drop the markers that touch the (grown) jaw silhouette."""
    ok = []
    for i in marker_view_ok(Tcs, K):
        P = (Tcs[:3, :3] @ K5.marker_corners(i).T).T + Tcs[:3, 3]
        uv = (K @ P.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        q = np.zeros_like(mask)
        cv2.fillConvexPoly(q, np.round(uv).astype(np.int32), 255)
        if not (q & mask).any():
            ok.append(i)
    return ok


def plan_poses(n, jaw, seed, tries=20000, log=print):
    """n poses (URDF deg) for k5_wrist capture: 4+ markers clear of the jaw with the URDF camera AND with a camera moved
    off it like the selftest's, arm clear of itself / the C rig / the table, and wrist_flex and wrist_roll both turned."""
    f = SELF_H / 2 / math.tan(math.radians(SELF_FOV / 2))
    K = np.array([[f, 0, SELF_W / 2], [0, f, SELF_H / 2], [0, 0, 1]])
    base = node_fk([{"q": [0] * 5}])
    X0 = T_of(base["X0"]["R"], base["X0"]["t_mm"])
    Xoff = X0 @ T_rv(np.radians([1.5, -2.0, 4.0]), [2.0, -1.5, 3.0])     # same offset as the selftest's "real" camera
    S = T_of(np.eye(3), [0.0, base["ty_mm"], 0.0])                      # sheet v2 where K1 puts it (sim table axes)
    geo = node_poses([], jaw, occ=True)
    grow = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * PLAN_MASK_GROW_PX + 1,) * 2)
    mask = cv2.dilate(jaw_mask(geo["occ"], K, (SELF_W, SELF_H)), grow)
    # the jaw rides with the gripper: in the offset camera (Xoff = X0 D) it sits at inv(D) p, so it has its own silhouette
    Dinv = inv(inv(X0) @ Xoff)
    occ_off = [((Dinv[:3, :3] @ np.asarray(t, float).T).T + Dinv[:3, 3]).tolist() for t in geo["occ"]]
    mask_off = cv2.dilate(jaw_mask(occ_off, K, (SELF_W, SELF_H)), grow)
    log(f"  jaw {jaw:.0f} deg (URDF): the jaw/gripper covers {100 * (mask > 0).mean():.1f}% of the {SELF_W}x{SELF_H} wrist picture "
        f"(grown {PLAN_MASK_GROW_PX} px)")
    rng = np.random.default_rng(seed)
    lim = base["limits"]
    seen, cand, n_vis, n_jaw = [], [], 0, 0

    def look(qs):
        nonlocal n_vis, n_jaw
        fk = node_fk([{"q": q} for q in qs])
        for fpose in fk["poses"]:
            G = T_of(fpose["G"]["R"], fpose["G"]["t_mm"])
            if G[1, 3] <= S[1, 3] + 60:
                continue
            if len(marker_view_ok(inv(X0) @ inv(G) @ S, K)) < 4:
                continue
            n_vis += 1
            seen.append(fpose["q"])
            m0 = markers_clear(inv(X0) @ inv(G) @ S, K, mask)
            m1 = markers_clear(inv(Xoff) @ inv(G) @ S, K, mask_off)
            if len(m0) < 4 or len(m1) < 4:
                n_jaw += 1
                continue
            cand.append({"q": fpose["q"], "G": G, "ids": m0})

    m = PLAN_LIMIT_MARGIN_DEG       # keep off the URDF limits: the real joint may not reach them (calibrated range)
    look([[rng.uniform(lo + m, hi - m) for lo, hi in lim] for _ in range(tries)])
    log(f"  {tries} random poses: {n_vis} see 4+ markers without the jaw, {n_jaw} of them lose that to the jaw -> {len(cand)} left")
    # few random poses look at the sheet: search again around the ones that do (+-20 deg per joint, inside the limits)
    base_seen = seen[:]
    jit = [[min(max(q0[k] + rng.uniform(-20, 20), lim[k][0] + PLAN_LIMIT_MARGIN_DEG), lim[k][1] - PLAN_LIMIT_MARGIN_DEG)
            for k in range(5)]
           for q0 in base_seen for _ in range(max(1, 6000 // max(1, len(base_seen))))]
    look(jit)
    log(f"  + {len(jit)} poses near those: {n_vis} see 4+ markers, {n_jaw} lose that to the jaw -> {len(cand)} left")
    chk = node_poses([c["q"] for c in cand], jaw)
    safe = []
    for c, s in zip(cand, chk["poses"]):
        c.update(s)
        if s["self_mm"] >= PLAN_SELF_MIN_MM and s["rig_mm"] >= PLAN_RIG_MIN_MM and s["table_mm"] >= PLAN_TABLE_MIN_MM:
            safe.append(c)
    log(f"  {len(safe)} clear of the arm itself (>= {PLAN_SELF_MIN_MM} mm), the C rig (>= {PLAN_RIG_MIN_MM} mm) and the table "
        f"(>= {PLAN_TABLE_MIN_MM} mm)")
    if len(safe) < n:
        return None, {"why": f"only {len(safe)} safe poses", "K": K, "X0": X0, "Xoff": Xoff, "S": S, "mask": mask}
    fl = [c["q"][3] for c in safe]
    log(f"  the safe poses reach wrist_flex {min(fl):.0f}..{max(fl):.0f} deg, wrist_roll {min(c['q'][4] for c in safe):.0f}.."
        f"{max(c['q'][4] for c in safe):.0f} deg (URDF)")
    # greedy: most different gripper orientations, a little bonus for more markers; but first PLAN_FLEX_EACH poses with
    # wrist_flex at least PLAN_FLEX_BY below the most bent one (the jaw pushes most poses to the bent end)
    def diverse(pool, chosen):
        return max(pool, key=lambda c: min(rot_deg(c["G"][:3, :3].T @ d["G"][:3, :3]) for d in chosen) + 2.0 * len(c["ids"]))
    low = [c for c in safe if c["q"][3] <= max(fl) - PLAN_FLEX_BY]
    chosen = [min(safe, key=lambda c: c["q"][3])]
    while len(chosen) < min(PLAN_FLEX_EACH, len(low)):
        chosen.append(diverse([c for c in low if c not in chosen], chosen))
    while len(chosen) < n:
        chosen.append(diverse([c for c in safe if c not in chosen], chosen))
    # order: nearest neighbour in joint space from the pose nearest to the rest pose (less hand travel)
    rest = np.array([0, -100, 90, 70, 0], float)
    left = chosen[:]
    tour = [min(left, key=lambda c: np.abs(np.array(c["q"]) - rest).sum())]
    left.remove(tour[0])
    while left:
        nx = min(left, key=lambda c: np.abs(np.array(c["q"]) - np.array(tour[-1]["q"])).sum())
        tour.append(nx); left.remove(nx)
    return tour, {"K": K, "X0": X0, "Xoff": Xoff, "S": S, "mask": mask, "mask_off": mask_off, "n_vis": n_vis, "n_jaw": n_jaw,
                  "n_safe": len(safe), "flex_max_safe": max(fl)}


def render_with_jaw(sheet, K, Tcs, mask, seed):
    img = K5.render_sheet(sheet, K, Tcs[:3, :3], Tcs[:3, 3], (SELF_W, SELF_H), noise=2.0, seed=seed)
    img = img.copy()
    img[mask > 0] = 70           # the jaw as a dark grey silhouette (the real jaw is lit and textured; this is the worst case
    return img                   # for nothing: only the covered markers are lost)


def cmd_poses(a):
    print(f"sim/so101.js sha256 {file_sha(os.path.join(SIM, 'so101.js'))[:12]}, seed {a.seed}")
    tour, info = plan_poses(a.n, a.jaw, a.seed)
    fails = 0
    if tour is None:
        print("FAILED:", info["why"]); sys.exit(1)
    K, X0, Xoff, S, mask = info["K"], info["X0"], info["Xoff"], info["S"], info["mask"]
    qa = np.array([c["q"] for c in tour])
    fr, rr = np.ptp(qa[:, 3]), np.ptp(qa[:, 4])
    top = qa[:, 3].max()
    n_lo, n_hi = int((qa[:, 3] <= top - PLAN_FLEX_BY).sum()), int((qa[:, 3] > top - PLAN_FLEX_BY).sum())
    ok = n_lo >= PLAN_FLEX_EACH and n_hi >= PLAN_FLEX_EACH and fr >= PLAN_MIN_FLEX_RANGE and rr >= PLAN_MIN_ROLL_RANGE
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} wrist_flex spans {fr:.0f} deg (need {PLAN_MIN_FLEX_RANGE:.0f}); {n_lo} poses at least "
          f"{PLAN_FLEX_BY:.0f} deg less bent than the most bent and {n_hi} near it (need {PLAN_FLEX_EACH} each); "
          f"wrist_roll spans {rr:.0f} deg (need {PLAN_MIN_ROLL_RANGE:.0f})")
    # the check that matters: photos drawn with the jaw over them, a camera off the URDF, solve -> must pass and come back
    sheet = K5.make_sheet()
    views, tiles = [], []
    for j, c in enumerate(tour):
        Tcs = inv(Xoff) @ inv(c["G"]) @ S
        img = render_with_jaw(sheet, K, Tcs, info["mask_off"], a.seed + j)   # the jaw as the offset camera sees it
        found = K5.detect_markers(img)
        views.append({"G": c["G"], "found": found})
        t = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR) if img.ndim == 2 else img.copy()
        for cc in found.values():
            cv2.polylines(t, [cc.astype(np.int32)], True, (0, 200, 0), 2)
        cv2.putText(t, f"{j + 1}: q {[round(v) for v in c['q']]}  markers {len(found)}", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        tiles.append(t)
        c["found_in_render"] = sorted(int(k) for k in found)
    nmin = min(len(v["found"]) for v in views)
    ok = nmin >= 4
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} the drawn pictures (jaw over them, camera off the URDF like the selftest) show "
          f"{nmin}-{max(len(v['found']) for v in views)} markers (need 4)")
    r = solve(views, X0, K, np.zeros(5))
    if "X" in r:
        et = float(np.linalg.norm(r["X"][:3, 3] - Xoff[:3, 3])); er = rot_deg(r["X"][:3, :3].T @ Xoff[:3, :3])
    ok = r["ok"] and "X" in r and et < SELF_T_TOL_MM and er < SELF_R_TOL_DEG
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} solve on these {len(tour)} poses: "
          + (f"camera back within {et:.3f} mm, {er:.3f} deg; rms {r['rms_px']:.2f} px, turned up to {r['spread_deg']:.0f} deg about axes "
             f"{r['axis_spread_deg']:.0f} deg apart, 1 sigma {r['sd_t_mm']:.2f} mm / {r['sd_rot_deg']:.3f} deg" if "X" in r else "")
          + ("" if r["ok"] else f" REFUSED: {r['why']}"))
    # negative at the boundary: raise the table so that the lowest chosen pose is 1 mm under the line; exactly the poses
    # whose clearance is under (line + lift) must be flagged (the table check is really applied, point by point)
    tm = np.array([c["table_mm"] for c in tour])
    lift = float(tm.min() - PLAN_TABLE_MIN_MM + 1.0)
    neg = node_poses([c["q"] for c in tour], a.jaw, table_lift=lift)
    got = np.array([p["table_mm"] < PLAN_TABLE_MIN_MM for p in neg["poses"]])
    want = tm - lift < PLAN_TABLE_MIN_MM
    ok = bool(got.any()) and bool((got == want).all())
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} negative: table raised {lift:.1f} mm -> {int(got.sum())}/{len(tour)} poses flagged as too low "
          f"(expected {int(want.sum())})")
    # negative: the jaw mask really removes markers (some poses that see 4+ without it lose that with it)
    ok = info["n_jaw"] > 0
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} negative: the jaw silhouette removed {info['n_jaw']} of {info['n_vis']} candidate poses")
    print("  pose  shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll (URDF deg) | markers | self / C rig / table mm")
    for j, c in enumerate(tour):
        print(f"  {j + 1:>4}  " + " ".join(f"{v:>7.1f}" for v in c["q"]) +
              f" | {len(c['ids'])} | {c['self_mm']:.1f} ({c['self_pair']}) / {c['rig_mm']:.0f} ({c['rig_id']}) / {c['table_mm']:.0f} ({c['table_group']})")
    os.makedirs(OUT, exist_ok=True)
    out = {"what": "recommended wrist-camera poses for k5_wrist.py capture --plan (URDF deg; capture turns them into LeRobot "
                   "readings with the k1 file). From the sim's arm and the URDF camera: the real camera is a few mm off, so a pose "
                   "may show fewer markers; any pose with 4+ markers is fine, these are a guide",
           "date": datetime.datetime.now().isoformat(timespec="seconds"), "so101_js_sha256": file_sha(os.path.join(SIM, "so101.js")),
           "jaw_urdf_deg": a.jaw, "seed": a.seed, "sheet": "v2 where K1 puts it (sim table axes)",
           "limits": {"self_mm": PLAN_SELF_MIN_MM, "rig_mm": PLAN_RIG_MIN_MM, "table_mm": PLAN_TABLE_MIN_MM, "mask_grow_px": PLAN_MASK_GROW_PX},
           "poses": [{"q_urdf": [round(v, 1) for v in c["q"]], "markers_urdf_cam": [int(i) for i in c["ids"]],
                      "self_mm": round(c["self_mm"], 1), "rig_mm": round(c["rig_mm"], 1), "rig_id": c["rig_id"],
                      "table_mm": round(c["table_mm"], 1)} for c in tour]}
    pj = a.out or os.path.join(OUT, "wrist_poses.json")
    json.dump(out, open(pj, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    rows = [np.hstack(tiles[k:k + 5]) for k in range(0, len(tiles), 5)]
    rows[-1] = np.hstack([rows[-1], np.zeros((SELF_H, rows[0].shape[1] - rows[-1].shape[1], 3), np.uint8)]) \
        if rows[-1].shape[1] < rows[0].shape[1] else rows[-1]
    sheet_png = os.path.splitext(pj)[0] + ".png"
    cv2.imwrite(sheet_png, cv2.resize(np.vstack(rows), None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
    print(f"wrote {pj} and {sheet_png}")
    print("POSES", "OK" if not fails else f"FAILED ({fails})")
    sys.exit(1 if fails else 0)


# ---------------------------------------------------------------- selftest
FAKE_K1 ={"sign": {m: (-1 if i % 2 else 1) for i, m in enumerate(JOINTS)},
           "offset": {m: (i - 2) * 7.5 for i, m in enumerate(JOINTS)}}   # same idea as sim/plan-cli.js fakeK1


def to_lerobot(q):
    return {m: FAKE_K1["offset"][m] + q[i] / FAKE_K1["sign"][m] for i, m in enumerate(JOINTS)}


def marker_view_ok(Tcs, K):
    """ids whose 4 corners are in the selftest image with a margin, in front, at least 18 px wide and seen within 60 deg."""
    ok = []
    if Tcs[2, 1] > -math.cos(math.radians(60)):    # the sheet normal (+y, column 1) in camera axes: its z must be < -0.5
        return ok
    for i in K5.MARKERS:
        P = (Tcs[:3, :3] @ K5.marker_corners(i).T).T + Tcs[:3, 3]
        if (P[:, 2] < 50).any():
            continue
        uv = (K @ P.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        if (uv[:, 0] < 8).any() or (uv[:, 0] > SELF_W - 8).any() or (uv[:, 1] < 8).any() or (uv[:, 1] > SELF_H - 8).any():
            continue
        if np.linalg.norm(uv[0] - uv[2]) < 18 * math.sqrt(2):
            continue
        ok.append(i)
    return ok


def visible(qs, X, S, K):
    """the poses (URDF deg) among qs where the camera X sees 4+ markers, gripper_link 60 mm or more above the sheet."""
    fk = node_fk([{"q": q} for q in qs])
    cand = []
    for f in fk["poses"]:
        G = T_of(f["G"]["R"], f["G"]["t_mm"])
        ids = marker_view_ok(inv(X) @ inv(G) @ S, K)
        if len(ids) >= 4 and G[1, 3] > S[1, 3] + 60:
            cand.append((f["q"], G, ids))
    return cand


def pick_poses(X, S, K, n, seed, near=None, jit=0.0, tries=20000):
    """random arm poses (URDF deg) where the camera X sees 4+ markers; greedy for different gripper orientations."""
    rng = np.random.default_rng(seed)
    lim = node_fk([{"q": [0] * 5}])["limits"]
    if near is None:
        qs = [[rng.uniform(lo, hi) for lo, hi in lim] for _ in range(tries)]
    else:
        qs = [[near[k] + rng.uniform(-jit, jit) for k in range(5)] for _ in range(400)]
    cand = visible(qs, X, S, K)
    if not cand:
        return []
    chosen = [cand[0]]
    while len(chosen) < n and len(chosen) < len(cand):
        best = max(cand, key=lambda c: min(rot_deg(c[1][:3, :3].T @ d[1][:3, :3]) + 0.01 * len(c[2]) for d in chosen))
        chosen.append(best)
    return chosen


def one_joint_poses(X, S, K, seed, joint, span):
    """poses that differ in ONE joint only (start poses from a random search), 6 or more of them visible."""
    rng = np.random.default_rng(seed)
    lim = node_fk([{"q": [0] * 5}])["limits"]
    starts = visible([[rng.uniform(lo, hi) for lo, hi in lim] for _ in range(20000)], X, S, K)
    for q0, _, _ in starts:
        qs = [[q0[k] + (d if k == joint else 0.0) for k in range(5)] for d in np.linspace(-span, span, 13)]
        c = visible(qs, X, S, K)
        if len(c) >= MIN_POSES:
            return c
    return []


def synth_views(chosen, X_true, S, K, sheet, read_bias=None, seed=1):
    """photos drawn with the TRUE camera X_true; the joint readings may differ from the true pose by read_bias (URDF deg)."""
    qs_read = [[c[0][k] + (read_bias[k] if read_bias else 0.0) for k in range(5)] for c in chosen]
    fk = node_fk([{"L": to_lerobot(q)} for q in qs_read], FAKE_K1)     # through the k1 conversion like real readings
    views = []
    for j, (c, f) in enumerate(zip(chosen, fk["poses"])):
        Tcs = inv(X_true) @ inv(c[1]) @ S
        img = K5.render_sheet(sheet, K, Tcs[:3, :3], Tcs[:3, 3], (SELF_W, SELF_H), noise=2.0, seed=seed + j)
        views.append({"G": T_of(f["G"]["R"], f["G"]["t_mm"]), "found": K5.detect_markers(img), "q_true": c[0]})
    return views


def cmd_selftest(a):
    fails = 0
    f = SELF_H / 2 / math.tan(math.radians(SELF_FOV / 2))
    K = np.array([[f, 0, SELF_W / 2], [0, f, SELF_H / 2], [0, 0, 1]])
    dist = np.zeros(5)
    sheet = K5.make_sheet()
    base = node_fk([{"q": [0] * 5}])
    X0 = T_of(base["X0"]["R"], base["X0"]["t_mm"])
    # the "real" camera: off the URDF by a known amount (mount printed a bit off, camera turned in its holder)
    X_true = X0 @ T_rv(np.radians([1.5, -2.0, 4.0]), [2.0, -1.5, 3.0])
    # the sheet: on the sim's table, 3 mm forward, 2 mm left, turned 1 deg (as if taped a little off)
    S = T_rv(np.radians([0, 1.0, 0]), [3.0, base["ty_mm"], -2.0])
    chosen = pick_poses(X_true, S, K, 10, seed=a.seed)
    print(f"sim/so101.js sha256 {file_sha(os.path.join(SIM, 'so101.js'))[:12]}, table y {base['ty_mm']:.1f} mm, seed {a.seed}")
    print(f"== the sim's arm (sim/so101.js urdfFK, node) and a wrist camera moved off the URDF by (2.0, -1.5, 3.0) mm in its own axes and "
          f"{rot_deg(T_rv(np.radians([1.5, -2.0, 4.0]), [0, 0, 0])[:3, :3]):.2f} deg; {len(chosen)} poses, "
          f"{SELF_W}x{SELF_H} vertical FOV {SELF_FOV:.0f}, pixel noise 2 ==")
    print("   URDF camera in gripper_link: t", np.round(X0[:3, 3], 2).tolist(), "mm")
    ok = len(chosen) >= 8
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} the random search found {len(chosen)} poses that see 4+ markers (need 8 for this test)")
    views = synth_views(chosen, X_true, S, K, sheet, seed=a.seed)
    r = solve(views, X0, K, dist)
    if "X" not in r:
        print("  FAIL", r["why"]); fails += 1
    else:
        et = float(np.linalg.norm(r["X"][:3, 3] - X_true[:3, 3]))
        er = rot_deg(r["X"][:3, :3].T @ X_true[:3, :3])
        es = float(np.linalg.norm(r["S"][:3, 3] - S[:3, 3]))
        dtt = (X_true[:3, 3] - X0[:3, 3])
        ok = r["ok"] and et < SELF_T_TOL_MM and er < SELF_R_TOL_DEG
        fails += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} solved camera vs true: {et:.3f} mm, {er:.3f} deg (pass < {SELF_T_TOL_MM} mm, "
              f"< {SELF_R_TOL_DEG} deg); rms {r['rms_px']:.2f} px, {r['n_photos']} photos, turned up to {r['spread_deg']:.0f} deg "
              f"about axes {r['axis_spread_deg']:.0f} deg apart, 1 sigma {r['sd_t_mm']:.2f} mm / {r['sd_rot_deg']:.3f} deg; sheet {es:.2f} mm off"
              + ("" if r["ok"] else f"; REFUSED: {r['why']}"))
        print(f"       urdf_delta dt {np.round(r['urdf_delta']['dt_mm'], 2).tolist()} (true {np.round(dtt, 2).tolist()} in gripper_link), "
              f"rot {r['urdf_delta']['rot_deg']:.2f} deg (true {rot_deg(X0[:3, :3].T @ X_true[:3, :3]):.2f})")
        G = chosen[0][1]                 # where the picture centre lands on the sheet, true vs solved camera
        hit = []
        for Cm in (G @ X_true, G @ r["X"]):
            o, d = Cm[:3, 3], Cm[:3, 2]
            hit.append(o + (S[1, 3] - o[1]) / d[1] * d)
        print(f"       the picture centre lands on the sheet {np.linalg.norm(hit[0] - hit[1]):.2f} mm apart (true vs solved camera, pose 1)")
    # negative 1: photos and readings out of order -> refused
    sw = [dict(v, G=views[(k + 1) % len(views)]["G"]) for k, v in enumerate(views)]
    r2 = solve(sw, X0, K, dist)
    ok = not r2["ok"]
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} negative: readings shifted by one photo -> {'refused' if ok else 'NOT refused'} "
          f"({r2.get('why', '')[:70]}...)")
    # negative 2: poses all alike (the wrist hardly turned) -> refused
    near = pick_poses(X_true, S, K, 8, seed=a.seed + 4, near=chosen[0][0], jit=4.0)
    r3 = solve(synth_views(near, X_true, S, K, sheet, seed=50), X0, K, dist)
    ok = len(near) >= MIN_POSES and not r3["ok"]
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} negative: {len(near)} poses within 4 deg per joint -> {'refused' if not r3['ok'] else 'NOT refused'} "
          f"({r3.get('why', '')[:90]})")
    # negative 3/4: one joint only (wrist_roll alone: one turning axis; wrist_flex alone) -> refused, never a wrong json
    for jn, jk in (("wrist_roll", 4), ("wrist_flex", 3)):
        one = one_joint_poses(X_true, S, K, a.seed + 9, jk, 80.0 if jk == 4 else 40.0)
        ro = solve(synth_views(one, X_true, S, K, sheet, seed=70), X0, K, dist) if one else {"ok": False, "why": "no poses"}
        ok = len(one) >= MIN_POSES and not ro["ok"]
        fails += not ok
        off = (f"; the camera would be {np.linalg.norm(ro['X'][:3, 3] - X_true[:3, 3]):.0f} mm off" if "X" in ro else "")
        print(f"  {'ok  ' if ok else 'FAIL'} negative: {len(one)} poses turning {jn} only -> {'refused' if not ro['ok'] else 'NOT refused'} "
              f"({ro.get('why', '')[:80]}){off}")
    # negative 5: too few photos
    r4 = solve(views[:3], X0, K, dist)
    ok = (not r4["ok"]) and "need" in r4["why"]
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} negative: 3 photos -> refused ({r4['why']})")
    # what joint offsets do (information, not pass/fail): the readings differ from the true arm
    print("== joint zero offsets in the readings (information; K1 measures these; the camera error follows K1's accuracy) ==")
    for name, b in (("wrist_roll +2 deg", [0, 0, 0, 0, 2.0]), ("wrist_flex +1 deg", [0, 0, 0, 1.0, 0]),
                    ("wrist_flex +0.3 deg", [0, 0, 0, 0.3, 0]), ("shoulder_lift +1 deg", [0, 1.0, 0, 0, 0]),
                    ("shoulder_lift +0.3 deg", [0, 0.3, 0, 0, 0])):
        rb = solve(synth_views(chosen, X_true, S, K, sheet, read_bias=b, seed=a.seed), X0, K, dist)
        if "X" not in rb:
            print(f"   {name}: {rb['why']}")
            continue
        et = float(np.linalg.norm(rb["X"][:3, 3] - X_true[:3, 3]))
        er = rot_deg(rb["X"][:3, :3].T @ X_true[:3, :3])
        print(f"   {name}: rms {rb['rms_px']:.2f} px, worst photo {max(rb['per_photo_max_px']):.2f} px"
              f"{' (refused)' if not rb['ok'] else ' (passes)'}; camera off by {et:.2f} mm, {er:.2f} deg")
    print("== tools/k5_camera.py selftest still passes (imported here) ==")
    rr = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "k5_camera.py"), "selftest"], capture_output=True, text=True)
    ok = rr.returncode == 0 and "SELFTEST OK" in rr.stdout
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} k5_camera.py selftest exit {rr.returncode}")
    print("SELFTEST", "OK" if not fails else f"FAILED ({fails})")
    sys.exit(1 if fails else 0)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    s = p.add_subparsers(dest="cmd", required=True)
    q = s.add_parser("capture"); q.add_argument("index", type=int); q.add_argument("dir"); q.add_argument("port", nargs="?")
    q.add_argument("--k1", required=True)
    q.add_argument("--width", type=int, default=640); q.add_argument("--height", type=int, default=480)
    q.add_argument("--fps", type=int, default=15); q.add_argument("--fourcc", default="MJPG")
    q.add_argument("--plan", default=None, help="outputs\\k5\\wrist_poses.json from 'poses': show the next target pose")
    q = s.add_parser("solve"); q.add_argument("dir"); q.add_argument("--intrinsics", required=True); q.add_argument("--k1", required=True)
    q.add_argument("--ruler-mm", default=f"{K5.RULER_MM}")
    q.add_argument("--out", default=None, help="folder for the json and overlays (default outputs/k5)")
    q = s.add_parser("poses"); q.add_argument("--n", type=int, default=PLAN_N); q.add_argument("--jaw", type=float, default=0.0)
    q.add_argument("--seed", type=int, default=3); q.add_argument("--out", default=None)
    q = s.add_parser("selftest"); q.add_argument("--seed", type=int, default=7)
    a = p.parse_args()
    {"capture": cmd_capture, "solve": cmd_solve, "poses": cmd_poses, "selftest": cmd_selftest}[a.cmd](a)


if __name__ == "__main__":
    main()
