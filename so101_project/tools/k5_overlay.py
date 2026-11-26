r"""K5 (last part of W4): lay the sim's overhead picture over a real overhead photo and measure the board's four corners.
No robot, no serial port. It opens no camera either: it reads a photo file.

usage (the .venv python, it needs OpenCV; node must be on PATH for the sim):
  python tools\k5_overlay.py overlay PHOTO --intrinsics FILE --pose FILE [--mode nominal|calibrated] [--k1err FILE]
                             [--set 2] [--scene sheet|full] [--tol-mm X]
        PHOTO: one overhead photo (same camera mode as the intrinsics) with the K1/K5 sheet v2 lying where it lay for
        `k5_camera.py pose`. --pose is that pose json (camera in the SHEET frame).
        -> outputs\k5\overlay_<photo>_<mode>_<date_time>.png (photo and sim picture half and half, board corners: green + = where
           the sim camera puts them, red o = where the real camera puts them; marker centre arrows x10)
           and outputs\k5\overlay_<photo>_<mode>_<date_time>.json (numbers)
  python tools\k5_overlay.py selftest
        synthetic "real" photos with a known camera (made by k5_camera.render_sheet, NOT by the sim's renderer), the
        pose solved from them by k5_camera.solve_pose, and analytic answers for shifted / turned cameras. Negative checks:
        a pose from another photo, a mirrored photo, a wrong-size photo. Since 2026-11-20 (C42-W4) also: the half-pixel
        rule against the sim's OWN picture (vision.js render, 10), the sheet laid off its place with --k1err d (11), and
        the pose check on pictures drawn with 4 x 4 samples per pixel (5c).

What the two modes compare (the sim camera = so101.js overheadImage: pinhole, no distortion, principal point at the
image centre, f from the vertical field of view):
  nominal     the sim's DESIGNED camera (so101.js OVERHEAD_C, CAM_FOV.top) against the real camera. Answers "is the
              camera mounted where hardware/C_台の設計.md says". This is the K5 line of 計画書_完成形まで ("board corners
              within the K0 value"). The offsets are what the controller would get if it read real photos with the
              designed camera (plan-cli.js init calib "sim" does that today).
  calibrated  the sim camera put where the pose json says (centre, dir, up; f = the real fy). Then the sim picture and
              the photo must coincide; what is left is calibration error or a sheet that is not where the sim draws it.
In both modes the photo is first remapped to the sim's pinhole (cv2.undistort with K_sim = [[f,0,W/2-0.5],[0,f,H/2-0.5]]):
the lens distortion and the off-centre principal point are taken out, so the picture is one the sim could have drawn.
vision.js detect needs this remap too before it reads a real photo (it assumes the centre and no distortion).

Numbers:
  board corners (geometry): the four outer corners of the sim's board (scene-data.js boardCells +- cell/2, table height
    y = 0, and at the piece top 14 mm). px = sim camera pixel minus real camera pixel. mm and dxz = where the sim camera
    puts the point the real camera shows at the corner, minus the true corner (= where a piece at that corner would be
    FOUND with the sim camera, minus where it is; a camera moved by +d gives -d). These numbers come from the pose solved
    from THIS photo (the --pose of D3 is only compared with it: refused over POSE_AGREE_MM). In the selftest pictures
    drawn by point sampling (render_sheet ss=1) the ArUco corners move in steps of about 1 px (~0.6 mm on the table), so
    a move below ~1.5 mm may pass that check; drawn with 4 x 4 samples per pixel (ss=4, as a sensor averages) the check
    reads 0.3 mm as 0.25 mm and refuses 1.5 mm (selftest 5c, 2026-11-20). The corners are refined (CORNER_REFINE_SUBPIX).
  marker centres (image): the centre of the 4 ArUco corners found in the remapped photo against the centre of the
    black pixels of each sim square (vision.js sheetTris draws the 8 markers as plain black 25 mm squares with hard,
    aliased edges, so their corners are not used). px and mm on the table (both pixels sent
    back through the sim camera). This one is measured on pictures, so a frame mistake the geometry shares would show.
Frames: table frame of the sim (scene-data.js): y = 0 on the board top, x forward, z the arm's right, mm. The pose json
is in the SHEET frame. --k1err moves the sheet frame to the arm frame by paper dx, dz and scale only (the rotation is
inside shoulder_pan and is not taken out, same as k5_camera.py pose --k1err).
NOT checked here: a real photo (none yet), real lens distortion (the selftest photos have none), the arm drawn in the
sim picture against the real arm, where the REAL board (printed lines) lies (the board corners are the sim's board seen
by the two cameras), and the lens-tip vs optical
centre difference (the nominal camera puts the pinhole at the lens tip; a few mm in height = ~0.15 mm per mm at the corners).
"""

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import k5_camera as K5  # noqa: E402

ROOT = K5.ROOT
OUT = K5.OUT
SIM = os.path.join(ROOT, "sim")
SELFTEST_TOL_MM = 2.0     # GUESS for the selftest only. The real limit is K0 (not decided): 0.5 * pitch? the wrist camera
                          # corrects what is left (W3 --shift 5 came back), so the top camera only has to land the finger
                          # on the piece

SIM_JS = r"""
const fs = require("fs");
const { load } = require("./tools/_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js", "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js",
     "collide.js", "chikuwa-collision.js", "self-collision.js");
global.window = { SCENE_SKIP_DEFAULT: 1 }; load("scene-data.js"); delete global.window;
load("vision.js");
const a = JSON.parse(fs.readFileSync(process.argv[1], "utf8")), M = SCENE_MODEL, V = VISION, def = M.defaultUiValues();
const d0 = M.buildFromUi({ SET: a.set, GAP: def.GAP, STAND: 1, SOL: 0, NO_REFINE: true }), c = d0._check;
if (!c.solved || !d0.overheadCamera) { console.log(JSON.stringify({ ok: false, why: "sim scene not solved / no stand: " + c.why })); process.exit(0); }
const nom = d0.overheadCamera, out = { ok: true, nominal: { tip: nom.tip, dir: nom.dir, up: nom.up }, fov_top: SO101.CAM_FOV.top,
  cell: c.cell, boardCells: c.boardCells, robotY: c.robotY, set: a.set };
if (a.render) {
  const r = a.render, cam = { tip: r.tip, dir: r.dir, up: r.up };
  let tris = V.sheetTris();
  if (a.scene === "full") {
    const lay = { slots: c.slots.map(() => [5, 5]), rots: c.slotRot };
    tris = V.worldTris(d0, lay, c.photoQ || c.homeQ, c.homeJaw).concat(tris);
  }
  const im = V.render(tris, cam, { W: r.W, H: r.H, fov: r.fov, noise: 0, seed: 1 });
  fs.writeFileSync(r.out, Buffer.from(im.rgb.buffer, im.rgb.byteOffset, im.rgb.length));
  out.f = SO101.overheadImage(cam, r.fov, r.W, r.H).f;
  out.rendered = r.out;
}
console.log(JSON.stringify(out));
"""


# ---------------------------------------------------------------- sim side
def sim_call(req):
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8") as fh:
        json.dump(req, fh)
        p = fh.name
    try:
        r = subprocess.run(["node", "-e", SIM_JS, p], cwd=SIM, capture_output=True, text=True, encoding="utf-8")
    finally:
        os.unlink(p)
    if r.returncode:
        sys.exit("node failed: " + r.stderr[-600:])
    j = json.loads(r.stdout.strip().splitlines()[-1])
    if not j.get("ok"):
        sys.exit("sim: " + j.get("why", "?"))
    return j


def board_corners_mm(s):
    """outer corners of the board, table frame mm (y = 0), in the order near-left, near-right, far-right, far-left."""
    b = np.array(s["boardCells"]) * 1000.0
    h = s["cell"] * 1000.0 / 2
    x0, x1 = b[:, 0].min() - h, b[:, 0].max() + h
    z0, z1 = b[:, 1].min() - h, b[:, 1].max() + h
    return np.array([[x0, 0, z0], [x0, 0, z1], [x1, 0, z1], [x1, 0, z0]])


class SimCam:
    """so101.js overheadImage in numpy. tip in mm (table frame). pixels returned in OpenCV's convention (centre = integer)."""

    def __init__(self, tip_mm, dirv, up, f, W, H):
        z = np.asarray(dirv, float); z /= np.linalg.norm(z)
        u = np.asarray(up, float); u /= np.linalg.norm(u)
        x = np.cross(z, u); x /= np.linalg.norm(x)
        y = np.cross(x, z)
        self.C, self.x, self.y, self.z, self.f, self.W, self.H = np.asarray(tip_mm, float), x, y, z, float(f), W, H
        # same camera as OpenCV R, t, K: camera X = right = x, camera Y = down = -y, camera Z = forward = z
        self.R = np.vstack([x, -y, z])
        self.t = -self.R @ self.C
        self.K = np.array([[f, 0, W / 2 - 0.5], [0, f, H / 2 - 0.5], [0, 0, 1.0]])

    def project(self, P):
        d = np.asarray(P, float) - self.C
        zc = d @ self.z
        return np.stack([self.W / 2 + self.f * (d @ self.x) / zc, self.H / 2 - self.f * (d @ self.y) / zc], -1) - 0.5

    def to_table(self, uv, y=0.0):
        uv = np.atleast_2d(uv) + 0.5
        a = (uv[:, 0] - self.W / 2) / self.f
        b = (self.H / 2 - uv[:, 1]) / self.f
        r = self.z[None] + a[:, None] * self.x[None] + b[:, None] * self.y[None]
        s = (y - self.C[1]) / r[:, 1]
        return self.C[None] + r * s[:, None]


class RealCam:
    """pinhole K (after the remap) with the solved pose (sheet frame) and the sheet -> arm conversion."""

    def __init__(self, K, rvec, tvec, d=(0.0, 0.0), s=1.0, k=1.0):
        """the pose frame of k5_camera.py is the SHEET in true mm (the marker corners were scaled by the ruler, k =
        ruler / 150). A printed label L lies at k * L there. k1-err.js: label = s * arm + d (x, z). So
        sheet = k * (s * arm + d). Without --k1err (s = 1, d = 0) the arm frame is the sheet frame itself (k not applied:
        the sheet is assumed to lie at its nominal place, as k5_camera.py pose prints it)."""
        self.K = np.asarray(K, float)
        self.R = cv2.Rodrigues(np.asarray(rvec, float))[0]
        self.t = np.asarray(tvec, float).ravel()
        self.d, self.s, self.k = np.array([d[0], 0.0, d[1]]), s, k

    def arm_to_sheet(self, P):
        P = np.atleast_2d(P).astype(float).copy()
        P[:, [0, 2]] = P[:, [0, 2]] * self.s
        P = P + self.d
        P[:, [0, 2]] *= self.k
        return P

    def sheet_to_arm(self, P):
        P = np.atleast_2d(P).astype(float).copy()
        P[:, [0, 2]] /= self.k
        P = P - self.d
        P[:, [0, 2]] /= self.s
        return P

    def project(self, P_arm):
        p = (self.K @ (self.R @ self.arm_to_sheet(P_arm).T + self.t[:, None])).T
        return p[:, :2] / p[:, 2:]

    def to_table(self, uv, y=0.0):
        C = -self.R.T @ self.t
        uv = np.atleast_2d(uv)
        rays = (self.R.T @ np.linalg.inv(self.K) @ np.vstack([uv.T, np.ones(len(uv))])).T
        s = (y - C[1]) / rays[:, 1]
        return self.sheet_to_arm(C[None] + rays * s[:, None])

    def centre_dir_up_arm(self):
        C = (-self.R.T @ self.t)
        Ca = self.sheet_to_arm(C)[0]
        return Ca, self.R.T @ np.array([0, 0, 1.0]), self.R.T @ np.array([0, -1.0, 0])


# ---------------------------------------------------------------- picture side
def sim_squares(gray, cam, min_frac=0.5, max_frac=2.0):
    """black 25 mm squares of the sim picture (sheetTris' markers) -> {marker id: centre (OpenCV px)}.
    The centre is the mean of the square's pixels (the sim draws hard, aliased edges: corners found on them were off by up
    to 3.7 px in the first try, the centre of the pixels is not)."""
    exp = {i: cam.project(np.array([[*K5.MARKERS[i][:1], 0, K5.MARKERS[i][1]]]))[0] for i in K5.MARKERS}
    px_mm = cam.f / max(1.0, abs(cam.C[1]))
    area = (K5.MARKER_MM * px_mm) ** 2
    bw = cv2.inRange(gray, 5, 60)        # the sim's markers are about 19; its empty background is 0 (not the table)
    cnts, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = {}
    for c in cnts:
        a = cv2.contourArea(c)
        if not (min_frac * area < a < max_frac * area):
            continue
        if len(cv2.approxPolyDP(c, 0.04 * cv2.arcLength(c, True), True)) != 4:
            continue
        x, y, w, h = cv2.boundingRect(c)
        m = np.zeros((h, w), np.uint8)
        cv2.drawContours(m, [c - [x, y]], -1, 255, -1, offset=(0, 0))
        m &= bw[y:y + h, x:x + w]
        vs, us = np.nonzero(m)
        ctr = np.array([us.mean() + x, vs.mean() + y])
        i, dist_px = min(((i, np.linalg.norm(ctr - e)) for i, e in exp.items()), key=lambda v: v[1])
        if dist_px < 0.6 * K5.MARKER_MM * px_mm:
            out[i] = ctr
    return out


POSE_AGREE_MM = 1.0     # the --pose given and the pose solved from THIS photo must agree this well (table points over the
                        # board, mm) or the photo is refused: the camera or the sheet moved since the pose was taken (GUESS)
PIECE_TOP_MM = 14.0     # piece height (sim cell): the board corners are also given at this height


def compare(photo_bgr, intr, pose, mode, set_, scene, k1err=None, tol_mm=None, stem="photo", outdir=OUT, quiet=False):
    """the whole overlay. Returns the result dict (also written as json + png into outdir).
    pose: the json of k5_camera.py pose (the D3 photo). It is checked against the pose solved from THIS photo."""
    K, dist, size = np.array(intr["K"], float), np.array(intr["dist"], float), tuple(intr["image_size"])
    H, W = photo_bgr.shape[:2]
    if (W, H) != size:
        return {"ok": False, "why": f"photo {W}x{H} but the intrinsics are for {size[0]}x{size[1]}: same camera mode for both"}
    if not pose.get("ok", True) or "rvec" not in pose:
        return {"ok": False, "why": "the pose json has no solved pose (rvec)"}
    d, s = (0.0, 0.0), 1.0
    rl = pose.get("ruler_mm", [K5.RULER_MM])
    kr = sum(rl) / len(rl) / K5.RULER_MM
    if k1err:
        pp = k1err["paper"]
        d, s = (pp["dx_mm"], pp["dz_mm"]), 1 + pp["scale_permil"] / 1000
    # the photo itself must show the sheet where the pose says (a pose of another photo, or a camera that moved
    # since D3, would otherwise give board numbers that never looked at this photo)
    p2 = K5.solve_pose(cv2.cvtColor(photo_bgr, cv2.COLOR_BGR2GRAY), K, dist, kr)
    if not p2["ok"]:
        return {"ok": False, "why": "no pose from this photo: " + p2["why"]}
    a_ = RealCam(K, pose["rvec"], pose["tvec"]); b_ = RealCam(K, p2["rvec"], p2["tvec"])
    grid = np.array([[x, 0, z] for x in np.linspace(120, 280, 5) for z in np.linspace(-120, 120, 5)])
    agree = float(np.linalg.norm(b_.to_table(a_.project(grid)) - grid, axis=1).max())
    if agree > POSE_AGREE_MM:
        return {"ok": False, "why": f"the --pose does not fit this photo: up to {agree:.2f} mm on the table (limit {POSE_AGREE_MM}). "
                                     "The camera or the sheet moved since the pose was taken, or the pose is of another photo: "
                                     "run k5_camera.py pose on this photo"}
    s0 = sim_call({"set": set_})
    if mode == "nominal":
        fov = s0["fov_top"]
        f = (H / 2) / math.tan(math.radians(fov) / 2)
        n = s0["nominal"]
        tip, dirv, up = np.array(n["tip"]) * 1000, n["dir"], n["up"]
    elif mode == "calibrated":
        f = float(K[1, 1])
        fov = math.degrees(2 * math.atan(H / 2 / f))
        rc0 = RealCam(np.eye(3), p2["rvec"], p2["tvec"], d, s, kr if k1err else 1.0)
        tip, dirv, up = rc0.centre_dir_up_arm()
    else:
        return {"ok": False, "why": f"mode {mode}?"}
    simc = SimCam(tip, dirv, up, f, W, H)
    Kp = simc.K                                   # the sim's pinhole: remap the photo onto it
    real = RealCam(Kp, p2["rvec"], p2["tvec"], d, s, kr if k1err else 1.0)   # THIS photo's pose (review C42q part 2-1)
    und = cv2.undistort(photo_bgr, K, dist, None, Kp)
    raw = os.path.join(tempfile.gettempdir(), f"k5_overlay_{os.getpid()}.rgb")
    s1 = sim_call({"set": set_, "scene": scene, "render": {"tip": (tip / 1000).tolist(), "dir": list(map(float, dirv)),
                                                           "up": list(map(float, up)), "fov": fov, "W": W, "H": H, "out": raw}})
    simimg = np.frombuffer(open(raw, "rb").read(), np.uint8).reshape(H, W, 3)[:, :, ::-1].copy()
    os.unlink(raw)
    if abs(s1["f"] - f) > 1e-6 * f:
        return {"ok": False, "why": f"the sim's f {s1['f']} is not ours {f}"}

    # board corners (geometry)
    BC = board_corners_mm(s0)
    us, ur = simc.project(BC), real.project(BC)
    found = simc.to_table(ur)                       # where the sim camera puts what the real camera shows at the corner
    BT = BC + [0, PIECE_TOP_MM, 0]
    found14 = simc.to_table(real.project(BT), PIECE_TOP_MM)
    board = []
    for k, name in enumerate(["near-left", "near-right", "far-right", "far-left"]):
        dmm, d14 = found[k] - BC[k], found14[k] - BT[k]
        board.append({"corner": name, "table_mm": BC[k][[0, 2]].tolist(), "sim_px": us[k].tolist(), "real_px": ur[k].tolist(),
                      "px": float(np.linalg.norm(us[k] - ur[k])), "mm": float(np.hypot(dmm[0], dmm[2])), "dxz_mm": dmm[[0, 2]].tolist(),
                      "mm_at_piece_top": float(np.hypot(d14[0], d14[2])),
                      "in_picture": bool(0 <= us[k][0] < W and 0 <= us[k][1] < H and 0 <= ur[k][0] < W and 0 <= ur[k][1] < H)})
    # marker centres (pictures)
    gr = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
    real_m = K5.detect_markers(gr)
    sim_m = sim_squares(cv2.cvtColor(simimg, cv2.COLOR_BGR2GRAY), simc)
    marks = []
    for i in sorted(set(real_m) & set(sim_m)):
        c, sp = real_m[i].mean(0), sim_m[i]          # centre of the 4 ArUco corners / of the sim's black pixels
        a, b = simc.to_table(c), simc.to_table(sp)
        marks.append({"id": i, "real_px": c.tolist(), "sim_px": sp.tolist(), "px": float(np.linalg.norm(c - sp)),
                      "mm": float(np.hypot(*(a - b)[0][[0, 2]]))})
    res = {"ok": True, "mode": mode, "set": set_, "scene": scene, "image_size": [W, H], "sim_fov_deg": fov, "sim_f_px": f,
           "sim_cam_mm": {"tip": tip.tolist(), "dir": list(map(float, dirv)), "up": list(map(float, up))},
           "k1err": {"dx_mm": d[0], "dz_mm": d[1], "scale": s, "ruler_scale": kr} if k1err else None,
           "pose_vs_this_photo_mm": agree,
           "board": board, "board_max_mm": max(b["mm"] for b in board), "board_max_px": max(b["px"] for b in board),
           "markers_found": {"photo": sorted(real_m), "sim": sorted(sim_m)}, "marker_centres": marks,
           "marker_max_mm": max((m["mm"] for m in marks), default=None), "marker_max_px": max((m["px"] for m in marks), default=None),
           "tol_mm": tol_mm}
    if mode == "calibrated":
        res["note"] = ("board corners are 0 by construction here (sim camera = solved pose); the marker centres check the chain "
                       "(remap, frames, pixel rule), not the calibration: the pose came from the same markers"
                       + ("; with --k1err the sim draws the sheet at its printed labels, so the markers also show the sheet's "
                          "placement d" if k1err else ""))
    if tol_mm is None:
        res["verdict"] = "NO VERDICT (no --tol-mm: the limit K0 is not decided)"
    elif mode == "nominal":
        res["verdict"] = "OK" if res["board_max_mm"] <= tol_mm and all(b["in_picture"] for b in board) else "NG"
    elif tol_mm is not None:      # calibrated: the pictures must coincide (4 markers or more compared)
        res["verdict"] = "OK" if len(marks) >= 4 and res["marker_max_mm"] <= tol_mm else "NG"
    # picture
    ov = cv2.addWeighted(und, 0.5, simimg, 0.5, 0)
    for b in board:
        p, q = np.array(b["sim_px"]), np.array(b["real_px"])
        cv2.drawMarker(ov, tuple(int(round(v)) for v in p), (0, 200, 0), cv2.MARKER_CROSS, 40, 2)
        cv2.circle(ov, tuple(int(round(v)) for v in q), 14, (0, 0, 255), 2)
        cv2.putText(ov, f"{b['mm']:.1f}mm", tuple(int(v) + 18 for v in q), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    for m in marks:
        p, q = np.array(m["sim_px"]), np.array(m["real_px"])
        cv2.arrowedLine(ov, tuple(int(v) for v in p), tuple(int(v) for v in (p + 10 * (q - p))), (255, 0, 255), 2, tipLength=0.2)
    cv2.putText(ov, f"{mode}: board corners max {res['board_max_mm']:.2f} mm (green + sim, red o real). marker arrows x10",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    os.makedirs(outdir, exist_ok=True)
    base = os.path.join(outdir, f"overlay_{stem}_{mode}_{time.strftime('%Y%m%d_%H%M%S')}")
    cv2.imwrite(base + ".png", ov)
    json.dump(res, open(base + ".json", "w"), indent=1)
    res["files"] = [base + ".png", base + ".json"]
    if not quiet:
        report(res)
    return res


def report(r):
    print(f"the --pose fits this photo within {r['pose_vs_this_photo_mm']:.2f} mm")
    print(f"[{r['mode']}] sim camera tip {np.round(r['sim_cam_mm']['tip'], 2).tolist()} mm, f {r['sim_f_px']:.1f} px (vertical {r['sim_fov_deg']:.2f} deg)")
    for b in r["board"]:
        print(f"  board {b['corner']:10s} {np.round(b['table_mm'], 1).tolist()}: {b['px']:6.2f} px  {b['mm']:6.2f} mm "
              f"(found minus true: dx {b['dxz_mm'][0]:+.2f}, dz {b['dxz_mm'][1]:+.2f}; at the piece top {b['mm_at_piece_top']:.2f} mm)"
              + ("" if b["in_picture"] else "  NOT IN THE PICTURE"))
    mm = r["marker_max_mm"]
    print(f"  marker centres compared {len(r['marker_centres'])} (photo ids {r['markers_found']['photo']}, sim ids {r['markers_found']['sim']}): "
          + (f"max {r['marker_max_px']:.2f} px / {mm:.2f} mm" if mm is not None else "none"))
    if r["mode"] == "calibrated":
        print("  (calibrated: the board corners are 0 by construction -- the sim camera IS the solved pose. Read the markers)")
    if r.get("verdict"):
        print(f"  verdict" + (f" at {r['tol_mm']} mm" if r["tol_mm"] is not None else "") + f": {r['verdict']}")
    print("  wrote " + ", ".join(r.get("files", [])))


def cmd_overlay(a):
    img = cv2.imread(a.photo, cv2.IMREAD_COLOR)
    if img is None:
        sys.exit(f"cannot read {a.photo}")
    r = compare(img, json.load(open(a.intrinsics)), json.load(open(a.pose)), a.mode, a.set, a.scene,
                json.load(open(a.k1err)) if a.k1err else None, a.tol_mm, os.path.splitext(os.path.basename(a.photo))[0])
    if not r["ok"]:
        sys.exit("overlay: " + r["why"])
    if r.get("verdict") == "NG":
        sys.exit(1)


# ---------------------------------------------------------------- selftest
def cmd_selftest(_a):
    W, H, set_ = 1920, 1080, 2
    s0 = sim_call({"set": set_})
    nom = np.array(s0["nominal"]["tip"]) * 1000
    fy = H / 2 / math.tan(math.radians(64.0 / 2))                 # the "real" camera is NOT the sim's 60 deg
    Kt = np.array([[fy * 1.002, 0, W / 2 - 0.5 + 12], [0, fy, H / 2 - 0.5 - 8], [0, 0, 1]])
    intr = {"K": Kt.tolist(), "dist": [0, 0, 0, 0, 0], "image_size": [W, H]}
    sheet = K5.make_sheet()
    BC = board_corners_mm(s0)
    outdir = os.path.join(OUT, "overlay_selftest")
    fails = 0
    print(f"sim: nominal lens tip {np.round(nom, 2).tolist()} mm (table frame), board corners {BC[:, [0, 2]].round(1).tolist()} mm, "
          f"sim FOV {s0['fov_top']} deg. 'real' camera: vertical 64 deg, principal point (+12, -8) px off centre, no distortion")

    def shoot(C, yaw=0.0, tilt=(0.0, 0.0), seed=1, ss=None):    # ss None: k5_camera.RENDER_SS (env K5_RENDER_SS, default 1)
        Rc, t = K5.look_down(C, yaw, tilt)
        g = K5.render_sheet(sheet, Kt, Rc, t, (W, H), seed=seed, ss=ss)
        return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR), Rc, t

    def solve(img):
        p = K5.solve_pose(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), Kt, np.zeros(5))
        return p

    def check(name, ok, msg):
        nonlocal fails
        fails += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {name}: {msg}")

    cases = {}
    # 1. exactly where the sim says
    img, Rc, t = shoot(nom)
    p = solve(img)
    r = compare(img, intr, p, "nominal", set_, "sheet", None, SELFTEST_TOL_MM, "exact", outdir, quiet=True)
    rc = compare(img, intr, p, "calibrated", set_, "sheet", None, SELFTEST_TOL_MM, "exact", outdir, quiet=True)
    cases["exact"] = (r, rc)
    check("1 camera exactly at the sim's place", r["board_max_mm"] < 0.3 and rc["marker_max_mm"] < 0.5 and r["verdict"] == "OK"
          and len(r["marker_centres"]) == 8 and r["marker_max_mm"] < 0.5,
          f"board max {r['board_max_mm']:.3f} mm nominal; calibrated markers max {rc['marker_max_mm']:.3f} mm; nominal marker centres "
          f"{len(r['marker_centres'])}, max {r['marker_max_px']:.2f} px = {r['marker_max_mm']:.3f} mm")
    # 2. moved parallel to the table by +d: every corner must be FOUND at corner - d (the sim camera reads the photo)
    dxz = np.array([6.0, -4.0])
    img, Rc, t = shoot(nom + [dxz[0], 0, dxz[1]])
    p = solve(img)
    r = compare(img, intr, p, "nominal", set_, "sheet", None, SELFTEST_TOL_MM, "shift6m4", outdir, quiet=True)
    rc = compare(img, intr, p, "calibrated", set_, "sheet", None, SELFTEST_TOL_MM, "shift6m4", outdir, quiet=True)
    cases["shift"] = (r, rc)
    e = max(np.hypot(*(np.array(b["dxz_mm"]) + dxz)) for b in r["board"])
    mk = [m["mm"] for m in r["marker_centres"]]
    check("2 camera moved (+6, -4) mm along the table", e < 0.3 and r["verdict"] == "NG" and rc["marker_max_mm"] < 0.5
          and abs(np.median(mk) - np.hypot(*dxz)) < 0.4,
          f"nominal: each board corner found off by (dx, dz) within {e:.3f} mm of (-6, +4) (|.| = {np.hypot(*dxz):.2f}), verdict {r['verdict']}; "
          f"markers median {np.median(mk):.2f} mm; calibrated: markers max {rc['marker_max_mm']:.3f} mm")
    # 3. turned 2 deg about the vertical through the lens: corner off by |P - C| * 2 sin(1 deg)
    img, Rc, t = shoot(nom, yaw=2.0)
    p = solve(img)
    r = compare(img, intr, p, "nominal", set_, "sheet", None, SELFTEST_TOL_MM, "yaw2", outdir, quiet=True)
    rc = compare(img, intr, p, "calibrated", set_, "sheet", None, SELFTEST_TOL_MM, "yaw2", outdir, quiet=True)
    cases["yaw"] = (r, rc)
    exp = [np.hypot(b["table_mm"][0] - nom[0], b["table_mm"][1] - nom[2]) * 2 * math.sin(math.radians(1.0)) for b in r["board"]]
    e = max(abs(b["mm"] - x) for b, x in zip(r["board"], exp))
    check("3 camera turned 2 deg about the vertical", e < 0.3 and rc["marker_max_mm"] < 0.5,
          f"nominal board corners {[round(b['mm'], 2) for b in r['board']]} mm, analytic {[round(float(x), 2) for x in exp]} (worst diff {e:.3f}); "
          f"calibrated: markers max {rc['marker_max_mm']:.3f} mm")
    # 4. tilted 1.5 / -1 deg: the TRUE camera (not the solved pose, not RealCam) puts each corner on a pixel of the remapped
    #    photo; the nominal camera's straight-down ray through that pixel, written out here, gives where it is found
    img, Rc, t = shoot(nom + [2, 3, -1], tilt=(1.5, -1.0))
    p = solve(img)
    r = compare(img, intr, p, "nominal", set_, "sheet", None, SELFTEST_TOL_MM, "tilt", outdir, quiet=True)
    rc = compare(img, intr, p, "calibrated", set_, "sheet", None, SELFTEST_TOL_MM, "tilt", outdir, quiet=True)
    cases["tilt"] = (r, rc)
    worst = 0.0
    fs = r["sim_f_px"]
    for b in r["board"]:
        P = np.array([b["table_mm"][0], 0.0, b["table_mm"][1]])
        q = Rc @ P + t
        u, v = W / 2 - 0.5 + fs * q[0] / q[2], H / 2 - 0.5 + fs * q[1] / q[2]       # true camera with the sim's pinhole
        # nominal camera straight down at nom, image up = +x, right = +z: pixel -> table (y = 0)
        found = np.array([nom[0] + (H / 2 - 0.5 - v) / fs * nom[1], nom[2] + (u - (W / 2 - 0.5)) / fs * nom[1]])
        worst = max(worst, float(np.hypot(*(found - P[[0, 2]] - np.array(b["dxz_mm"])))))
    check("4 camera moved (2, 3, -1) mm and tilted (1.5, -1) deg", worst < 0.3 and r["verdict"] == "NG" and rc["marker_max_mm"] < 0.5,
          f"nominal board corners {[round(b['mm'], 2) for b in r['board']]} mm, true-camera answer (dx, dz) within {worst:.3f} mm, verdict {r['verdict']}; "
          f"calibrated: markers max {rc['marker_max_mm']:.3f} mm")
    # negative 5: a pose from ANOTHER photo (the camera 7.2 mm away: "it moved since D3") -> refused, in both modes
    img0, _, _ = shoot(nom)
    pw = solve(shoot(nom + [dxz[0], 0, dxz[1]])[0])
    r5 = [compare(img0, intr, pw, m_, set_, "sheet", None, SELFTEST_TOL_MM, "wrongpose", outdir, quiet=True) for m_ in ("nominal", "calibrated")]
    check("5 negative: pose of another photo (camera 7.2 mm away)", all((not x["ok"]) and "does not fit" in x["why"] for x in r5),
          r5[0].get("why", "accepted!")[:110] + "...")
    # 5b. a small move (0.3 mm) is still accepted (the check is not so tight that a re-take of the same scene fails)
    pw = solve(shoot(nom + [0.3, 0, 0])[0])
    r = compare(img0, intr, pw, "nominal", set_, "sheet", None, SELFTEST_TOL_MM, "smallmove", outdir, quiet=True)
    check("5b pose of a photo with the camera 0.3 mm away", r["ok"], f"accepted, pose vs this photo {r.get('pose_vs_this_photo_mm', float('nan')):.2f} mm (drawn with "
          f"{K5.RENDER_SS} x {K5.RENDER_SS} samples per pixel; with 1 (K5_RENDER_SS unset), in the "
          "point-sampled pictures the corners move in ~1 px steps up-down the picture, so 0.3 mm along x hardly shows and 1 mm reads about 0.6 mm: "
          "the drawing's fault, see 5c)")
    # 5c (2026-11-20 C42-W4). The coarse 5b is the drawing's fault, not the corner refinement's (k5_camera already uses
    #    CORNER_REFINE_SUBPIX): render_sheet point-samples the 300 DPI sheet, so edges land on whole pixels. Drawn with
    #    4 x 4 samples per pixel (ss=4, as a sensor averages), the same check sees 0.3 mm and refuses 1.5 mm
    img4 = shoot(nom, ss=4)[0]
    agrees = {}
    for dm in (0.3, 1.5):
        pw4 = solve(shoot(nom + [dm, 0, 0], ss=4, seed=2)[0])
        r = compare(img4, intr, pw4, "nominal", set_, "sheet", None, SELFTEST_TOL_MM, f"smallmove_ss4_{dm}", outdir, quiet=True)
        agrees[dm] = r
    a03 = agrees[0.3].get("pose_vs_this_photo_mm", float("nan"))
    check("5c drawn with 4 x 4 samples per pixel: camera 0.3 mm / 1.5 mm away",
          agrees[0.3]["ok"] and 0.15 <= a03 <= 0.45 and (not agrees[1.5]["ok"]) and "does not fit" in agrees[1.5].get("why", ""),
          f"0.3 mm: accepted, pose vs this photo {a03:.2f} mm (pass 0.15-0.45); 1.5 mm: "
          + ("refused" if not agrees[1.5]["ok"] else "ACCEPTED") + f" ({agrees[1.5].get('why', '')[:60]})")
    # negative 6: mirrored photo -> no pose (k5_camera refuses with the FLIPPED hint)
    pm = solve(cv2.flip(img0, 1))
    check("6 negative: mirrored photo", (not pm["ok"]) and "FLIPPED" in pm.get("why", ""), f"pose refused: {pm.get('why', '')[:70]}...")
    # negative 7: photo of another size
    r = compare(cv2.resize(img0, (1280, 720)), intr, solve(img0), "nominal", set_, "sheet", None, SELFTEST_TOL_MM, "size", outdir, quiet=True)
    check("7 negative: 1280x720 photo with 1920x1080 intrinsics", not r["ok"], r.get("why", "accepted!"))
    # 9. printed at 99.6 % (ruler 149.4 mm) and k1-err found label = 1.004016 * arm (the print's scale): the camera exactly
    #    nominal must give ~0. The first version multiplied the scale twice (review C42q-2: ~0.9 mm at the far corner)
    kp = 0.996
    Rc, t = K5.look_down(nom)
    img9 = cv2.cvtColor(K5.render_sheet(sheet, Kt, Rc, t, (W, H), scale=kp), cv2.COLOR_GRAY2BGR)
    p9 = K5.solve_pose(cv2.cvtColor(img9, cv2.COLOR_BGR2GRAY), Kt, np.zeros(5), kp)
    p9["ruler_mm"] = [K5.RULER_MM * kp]
    k1e = {"paper": {"dx_mm": 0.0, "dz_mm": 0.0, "scale_permil": (1 / kp - 1) * 1000}}
    r9 = compare(img9, intr, p9, "nominal", set_, "sheet", k1e, SELFTEST_TOL_MM, "ruler", outdir, quiet=True)
    k1w = {"paper": {"dx_mm": 0.0, "dz_mm": 0.0, "scale_permil": -(1 / kp - 1) * 1000}}
    r9w = compare(img9, intr, p9, "nominal", set_, "sheet", k1w, SELFTEST_TOL_MM, "ruler_wrong", outdir, quiet=True)
    check("9 print 99.6 % + ruler 149.4 + k1err scale +4.0 permil", r9["board_max_mm"] < 0.3 and r9w["board_max_mm"] > 1.0,
          f"board max {r9['board_max_mm']:.3f} mm; negative: k1err scale with the wrong sign -> {r9w['board_max_mm']:.2f} mm")
    # 11 (2026-11-20 C42-W4, review C42q 2-7). The sheet laid off its place: k1-err found label = arm + d, d = (3, -2) mm
    #    (s = 1). The camera is exactly nominal in the ARM frame, so in the sheet frame it sits at nom + d. With --k1err
    #    the board must come out ~0; without it, off by |d|; with d of the wrong sign, off by 2 |d| (negatives)
    d11 = np.array([3.0, -2.0])
    img11 = shoot(nom + [d11[0], 0, d11[1]], seed=3)[0]
    p11 = solve(img11)
    k1d = {"paper": {"dx_mm": float(d11[0]), "dz_mm": float(d11[1]), "scale_permil": 0.0}}
    k1dw = {"paper": {"dx_mm": float(-d11[0]), "dz_mm": float(-d11[1]), "scale_permil": 0.0}}
    r11 = compare(img11, intr, p11, "nominal", set_, "sheet", k1d, SELFTEST_TOL_MM, "paper_d", outdir, quiet=True)
    r11n = compare(img11, intr, p11, "nominal", set_, "sheet", None, SELFTEST_TOL_MM, "paper_d_none", outdir, quiet=True)
    r11w = compare(img11, intr, p11, "nominal", set_, "sheet", k1dw, SELFTEST_TOL_MM, "paper_d_wrong", outdir, quiet=True)
    nd = float(np.hypot(*d11))
    check("11 sheet laid (3, -2) mm off, k1err d given", r11["board_max_mm"] < 0.3 and abs(r11n["board_max_mm"] - nd) < 0.3
          and abs(r11w["board_max_mm"] - 2 * nd) < 0.3,
          f"board max {r11['board_max_mm']:.3f} mm; negatives: no --k1err {r11n['board_max_mm']:.2f} mm (|d| = {nd:.2f}), "
          f"d with the wrong sign {r11w['board_max_mm']:.2f} mm (2|d| = {2 * nd:.2f})")
    # 8. the sim picture is really the sim's (not ours): its marker squares sit where SimCam projects the marker centres
    rr = cases["exact"][0]
    e8 = max(np.linalg.norm(np.array(m["sim_px"]) - np.array(m["real_px"])) for m in rr["marker_centres"])
    check("8 sim renderer and k5_camera renderer agree on the same camera", e8 < 1.0,
          f"marker centres max {e8:.2f} px (two renderers: vision.js render and cv2.warpPerspective)")
    # 10 (2026-11-20 C42-W4, review C42q 1-5). The half-pixel rule against the sim's OWN picture: vision.js draws the sheet
    #    from the nominal camera moved by sub-pixel steps; the centre of each black square's pixels must sit where
    #    SimCam.project puts the marker centre. One square's centre is off by up to ~0.4 px (hard, aliased edges), so the
    #    MEAN over 8 markers x 6 shifts is checked (within 0.1 px; its own error ~0.03 px). The "without the -0.5" figure
    #    is a subtraction, not a new drawing: it only shows that a half-pixel slip would stand well clear of the noise
    n0 = s0["nominal"]
    fs = H / 2 / math.tan(math.radians(s0["fov_top"]) / 2)
    errs = []
    raw = os.path.join(tempfile.gettempdir(), f"k5_overlay_half_{os.getpid()}.rgb")
    for ddx, ddz in ((0, 0), (0.17, 0.29), (0.33, -0.11), (0.51, 0.41), (-0.23, 0.07), (-0.41, -0.37)):
        tip = np.array(n0["tip"]) * 1000 + [ddx, 0, ddz]
        sc = SimCam(tip, n0["dir"], n0["up"], fs, W, H)
        sim_call({"set": set_, "scene": "sheet", "render": {"tip": (tip / 1000).tolist(), "dir": list(map(float, n0["dir"])),
                                                            "up": list(map(float, n0["up"])), "fov": s0["fov_top"], "W": W, "H": H, "out": raw}})
        g10 = cv2.cvtColor(np.frombuffer(open(raw, "rb").read(), np.uint8).reshape(H, W, 3)[:, :, ::-1].copy(), cv2.COLOR_BGR2GRAY)
        os.unlink(raw)
        for i, c in sim_squares(g10, sc).items():
            errs.append(c - sc.project(np.array([[K5.MARKERS[i][0], 0, K5.MARKERS[i][1]]]))[0])
    errs = np.array(errs)
    m10, m10n = errs.mean(axis=0), (errs - 0.5).mean(axis=0)     # without the -0.5, project() is 0.5 px higher
    check("10 half-pixel rule against the sim's own picture", len(errs) == 48 and np.abs(m10).max() < 0.1 and np.abs(m10n).min() > 0.3,
          f"{len(errs)} squares (8 markers x 6 sub-pixel shifts): mean (u, v) {m10.round(3).tolist()} px, spread (sd) "
          f"{errs.std(axis=0).round(3).tolist()}, largest one {np.abs(errs).max():.2f} px; size check (a subtraction, not a "
          f"new drawing), without the -0.5: mean {m10n.round(3).tolist()} px (must be over 0.3)")
    print("SELFTEST " + ("OK" if not fails else f"FAILED ({fails})") + f"   pictures: {outdir}")
    sys.exit(1 if fails else 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    s = ap.add_subparsers(dest="cmd", required=True)
    q = s.add_parser("overlay")
    q.add_argument("photo"); q.add_argument("--intrinsics", required=True); q.add_argument("--pose", required=True)
    q.add_argument("--mode", choices=["nominal", "calibrated"], default="nominal")
    q.add_argument("--k1err"); q.add_argument("--set", type=int, default=2)
    q.add_argument("--scene", choices=["sheet", "full"], default="sheet",
                   help="full also draws the arm in the photo pose and the board (picture only; the arm's grey may be taken for a marker)")
    q.add_argument("--tol-mm", type=float, default=None)
    s.add_parser("selftest")
    a = ap.parse_args()
    {"overlay": cmd_overlay, "selftest": cmd_selftest}[a.cmd](a)


if __name__ == "__main__":
    main()
