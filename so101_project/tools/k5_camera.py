r"""K5: camera calibration for the overhead camera (placement C). No robot is touched.

usage (the .venv python, it needs OpenCV):
  python tools\k5_camera.py sheet
        print files -> outputs\k5\sheet_A4.pdf (+ .png), checker_A4.pdf (25 mm squares, overhead camera),
        checker_small_A4.pdf (15 mm squares, wrist camera). Print the PDFs at "actual size" (100 %), landscape.
  python tools\k5_camera.py capture INDEX DIR [--width 1920 --height 1080 --fps 5 --fourcc MJPG] [--board checker|small|sheet] [--no-window]
        one camera, the board detection on each picture. Without a window (LeRobot's .venv has opencv-python-headless:
        cv2.imshow fails there, 2026-11-23; or --no-window), in the terminal: Enter takes one picture and keeps it as
        DIR\NNN.png only if the whole checkerboard / 4 or more sheet markers are found, p + Enter only looks, q + Enter
        quits. Each time the detection is printed (where the board is in the picture / the marker ids) and the picture
        with it drawn goes to DIR\preview.png (intrinsics skips that file). The picture is taken after the Enter (a thread
        keeps reading the camera). With a window (an OpenCV that can show one): SPACE saves every picture, Q quits.
        Autofocus and auto exposure are switched off if the camera allows it (the focal length must not change).
        NOT run on a real camera yet (the no-window path: a fake camera in the selftest, 2026-11-23).
  python tools\k5_camera.py intrinsics DIR [--name top] [--board checker|small]
        15-20 checkerboard photos in DIR, one camera, one mode -> outputs\k5\intrinsics_<name>_<W>x<H>.json
  python tools\k5_camera.py pose IMAGE --intrinsics FILE [--ruler-mm 150.0[,150.0]] [--expect 157.3,540,19.5] [--k1err FILE]
        one overhead photo of the sheet lying flat -> camera position and direction in the SHEET frame
        -> outputs\k5\pose_<image name>.json and an overlay picture (red rings must sit on the printed K1 points)
  python tools\k5_camera.py selftest
        synthetic pictures with a known camera, the sim's own camera projection (node, sim/so101.js), a wrong-ruler
        negative check, the intrinsics solve, and capture's no-window path with a fake camera

The sheet (one A4, landscape) is the SAME sheet for K1 and K5:
  - a 10 mm grid labelled in the sim's table coordinates [mm]: +x = away from the arm (forward), +z = the arm's
    right (seen from behind the arm). The printed numbers are where a line would be if the sheet sat exactly at its
    nominal place (near edge x = 80, centre z = 0 on the arm's forward line). k1-err.js solves how far it really is
    (paper dx, dz and scale; a rotation of the sheet goes into shoulder_pan).
  - the 11 K1 points of sim/tools/k1-plan.js (P1..P11, same order) with a cross and their coordinates.
  - eight ArUco markers (DICT_4X4_50, ids 0-7, 25 mm): their corners are the known points for the camera pose.
  - two 150 mm rulers (x and z). Measure both with a steel rule and give them to --ruler-mm.
Keep the sheet (taped to the board) where it is from the K1 run until the puzzle runs: the camera pose here is in
the sheet frame, and k1-err.js's paper dx, dz, scale and shoulder_pan offset connect the sheet frame to the arm.
This tool does NOT make that connection for the controller (puzzle_run.py will, when K5 is wired in); --k1err only
shows the camera centre moved by (P / ruler_scale - d) / (1 + scale), without the rotation, to compare with --expect.
Heights: the sheet lies on the base board, y = 0 (the board top, the sim's table). Paper thickness is ignored.
Camera in the output: "centre" is the optical centre from solvePnP (the sim's lensTipMm is the lens tip; they differ
by a few mm along the axis for a small module), "dir" the optical axis, "up" the image's up direction, all in the
sheet frame (same convention as sim/so101.js overheadImage: image right = dir x up).
"""

import argparse
import glob
import json
import math
import os
import subprocess
import sys

import cv2
import numpy as np

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
OUT = os.path.join(ROOT, "outputs", "k5")
DPI = 300
PPM = DPI / 25.4                      # sheet pixels per mm
# sheet: A4 landscape. long side (297) along z, short side (210) along x. nominal table coordinates of the paper edges:
X_NEAR, X_FAR = 80.0, 290.0           # edge toward the arm / far edge [mm]
Z_LEFT, Z_RIGHT = -148.5, 148.5
EDGE_MM = 10.0                        # keep everything this far inside the paper edge (printer margins)
PAN_X = 38.8                          # shoulder_pan axis x [mm] (sim SO101.joints[0].origin = 0.0388353 m)
MARKER_MM = 25.0
QUIET_MM = 4.0                        # white border around each marker
MARKERS = {0: (105.0, -125.0), 1: (105.0, -60.0), 2: (105.0, 60.0), 3: (105.0, 125.0),
           4: (255.0, -125.0), 5: (255.0, -60.0), 6: (255.0, 60.0), 7: (255.0, 125.0)}   # id -> centre (x, z)
RULER_MM = 150.0
RULER_Z = ((125.0, -75.0), (125.0, 75.0))     # z ruler: x = 125, z -75..75
RULER_X = ((100.0, -20.0), (250.0, -20.0))    # x ruler: z = -20, x 100..250
BOARDS = {"checker": (9, 6, 25.0), "small": (9, 6, 15.0)}   # inner corners (cols, rows), square size [mm]
ARUCO = cv2.aruco.DICT_4X4_50
GROUND_TOL_MM = 1.0   # selftest: table-point error over the reach must be under this. GUESS: the wrist camera corrects what
                      # is left (W3 --shift 5 came back within 0.4 mm); 1 mm keeps the top camera well inside that
UP_WARN_DEG = 5.0     # the sim expects image up = +x (so101.js OVERHEAD_C.camUp); warn if the real camera is turned more


def k1_points():
    pts = [(r, a) for r in (130, 170, 210) for a in (-40, 0, 40)] + [(150, -20), (150, 20)]
    return [(PAN_X + r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for r, a in pts]


def sheet_px(x, z):
    """table (x, z) mm -> sheet pixel (u right, v down). up on the sheet = far from the arm (+x)."""
    return ((z - Z_LEFT) * PPM, (X_FAR - x) * PPM)


def marker_corners(mid):
    """the 4 corners of marker mid in the table frame, in OpenCV's order (top-left, top-right, bottom-right,
    bottom-left of the marker as printed, i.e. seen with the sheet's top = +x)."""
    cx, cz = MARKERS[mid]
    h = MARKER_MM / 2
    return np.array([[cx + h, 0, cz - h], [cx + h, 0, cz + h], [cx - h, 0, cz + h], [cx - h, 0, cz - h]], np.float64)


def text(img, s, x, z, scale=0.9, th=2):
    u, v = sheet_px(x, z)
    cv2.putText(img, s, (int(u), int(v)), cv2.FONT_HERSHEY_SIMPLEX, scale, 0, th)


def make_sheet():
    W, H = int(round((Z_RIGHT - Z_LEFT) * PPM)), int(round((X_FAR - X_NEAR) * PPM))
    img = np.full((H, W), 255, np.uint8)
    x0, x1, z0, z1 = X_NEAR + EDGE_MM, X_FAR - EDGE_MM, Z_LEFT + EDGE_MM, Z_RIGHT - EDGE_MM
    u0, u1 = int(sheet_px(0, z0)[0]), int(sheet_px(0, z1)[0])
    v0, v1 = int(sheet_px(x1, 0)[1]), int(sheet_px(x0, 0)[1])
    for x in range(int(math.ceil(x0 / 10) * 10), int(x1) + 1, 10):       # grid every 10 mm, darker every 50
        v = int(round(sheet_px(x, 0)[1]))
        cv2.line(img, (u0, v), (u1, v), 150 if x % 50 else 60, 1 if x % 50 else 2)
    for z in range(int(math.ceil(z0 / 10) * 10), int(z1) + 1, 10):
        u = int(round(sheet_px(0, z)[0]))
        cv2.line(img, (u, v0), (u, v1), 150 if z % 50 else 60, 1 if z % 50 else 2)
    for x in (100, 150, 200, 250):                                         # numbers away from markers and points
        text(img, f"x{x}", x + 1, -98)
    for z in (-100, -50, 0, 50, 100):
        text(img, f"z{z}", 231, z + 1)
    for i, (x, z) in enumerate(k1_points()):                               # K1 points: ring, 1 mm cross, name + coords
        u, v = sheet_px(x, z)
        cv2.circle(img, (int(u), int(v)), int(4 * PPM), 0, 2)
        c = int(3 * PPM)
        cv2.line(img, (int(u) - c, int(v)), (int(u) + c, int(v)), 0, 1)
        cv2.line(img, (int(u), int(v) - c), (int(u), int(v) + c), 0, 1)
        lz = z + 5 if z <= 90 else z - 40
        text(img, f"P{i + 1}", x + 2, lz, 1.0, 3)
        text(img, f"{x:.1f},{z:.1f}", x - 4.5, lz, 0.7, 2)
    d = cv2.aruco.getPredefinedDictionary(ARUCO)
    n = int(round(MARKER_MM * PPM))
    q = int(round(QUIET_MM * PPM))
    for mid, (x, z) in MARKERS.items():
        m = cv2.aruco.generateImageMarker(d, mid, n)
        u, v = sheet_px(x, z)
        a0, b0 = int(round(u - n / 2)), int(round(v - n / 2))
        img[b0 - q:b0 + n + q, a0 - q:a0 + n + q] = 255
        img[b0:b0 + n, a0:a0 + n] = m
        text(img, f"id{mid}", x - MARKER_MM / 2 - 3.5, z - MARKER_MM / 2, 0.6, 1)
    for (a, b), along in ((RULER_Z, "z"), (RULER_X, "x")):                # rulers with ticks every 10 mm
        p0, p1 = sheet_px(*a), sheet_px(*b)
        cv2.line(img, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), 0, 3)
        for k in range(0, int(RULER_MM) + 1, 10):
            x, z = (a[0], a[1] + k) if along == "z" else (a[0] + k, a[1])
            u, v = sheet_px(x, z)
            L = 15 if k % 50 else 35
            if along == "z":
                cv2.line(img, (int(u), int(v - L)), (int(u), int(v + L)), 0, 3)
            else:
                cv2.line(img, (int(u - L), int(v)), (int(u + L), int(v)), 0, 3)
    text(img, f"{RULER_MM:.0f} mm (z)", 127, 40, 0.7, 2)
    text(img, f"{RULER_MM:.0f} mm (x)", 238, -18, 0.7, 2)
    text(img, "K1/K5 sheet v2  PRINT AT ACTUAL SIZE (100%)", 95.5, -38, 0.75, 2)
    text(img, "this edge toward the arm", 91, -18, 0.75, 2)
    return img


def make_checker(cols, rows, mm):
    W, H = int(round(297 * PPM)), int(round(210 * PPM))
    img = np.full((H, W), 255, np.uint8)
    nx, ny = cols + 1, rows + 1
    s = mm * PPM
    u0, v0 = (W - nx * s) / 2, (H - ny * s) / 2
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:
                cv2.rectangle(img, (int(round(u0 + i * s)), int(round(v0 + j * s))),
                              (int(round(u0 + (i + 1) * s)) - 1, int(round(v0 + (j + 1) * s)) - 1), 0, -1)
    cv2.putText(img, f"checkerboard {cols}x{rows} inner corners, {mm:.0f} mm squares. Tape it flat on a stiff board",
                (int(15 * PPM), int(12 * PPM)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
    return img


def save_print(img, stem):
    """PNG with 300 dpi in the file, and a one-page PDF at 300 dpi (print it at actual size)."""
    from PIL import Image
    os.makedirs(OUT, exist_ok=True)
    im = Image.fromarray(img)
    png, pdf = os.path.join(OUT, stem + ".png"), os.path.join(OUT, stem + ".pdf")
    im.save(png, dpi=(DPI, DPI))
    im.convert("RGB").save(pdf, resolution=DPI)
    print(f"wrote {pdf} (+ .png)  {img.shape[1]}x{img.shape[0]} px = {img.shape[1] / PPM:.0f} x {img.shape[0] / PPM:.0f} mm")


def cmd_sheet(_a):
    save_print(make_sheet(), "sheet_A4")
    save_print(make_checker(*BOARDS["checker"]), "checker_A4")
    save_print(make_checker(*BOARDS["small"]), "checker_small_A4")
    print("print the PDFs at ACTUAL SIZE (100 %), landscape. On the sheet, measure both 150 mm rulers with a steel rule.")


# ---------------------------------------------------------------- intrinsics
def checker_object_points(board):
    cols, rows, mm = BOARDS[board]
    o = np.zeros((rows * cols, 3), np.float32)
    o[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * mm
    return o


def find_checker(gray, board="checker"):
    cols, rows, _ = BOARDS[board]
    ok, c = cv2.findChessboardCornersSB(gray, (cols, rows), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    return c if ok else None


def calibrate(obj, img_pts, size):
    rms, K, dist, _r, _t = cv2.calibrateCamera(obj, img_pts, size, None, None, flags=cv2.CALIB_FIX_K3)
    W, H = size
    fx, fy = K[0, 0], K[1, 1]
    return {"image_size": [W, H], "rms_px": float(rms), "K": K.tolist(), "dist": dist.ravel().tolist(),
            "fov_h_deg": math.degrees(2 * math.atan(W / 2 / fx)), "fov_v_deg": math.degrees(2 * math.atan(H / 2 / fy)),
            "fov_d_deg": math.degrees(2 * math.atan(math.hypot(W / 2 / fx, H / 2 / fy))), "n_images": len(obj)}


def coverage(pts, size):
    """share of a 4x4 grid over the image that has checker corners in it (the middle of the picture is what matters)."""
    W, H = size
    hit = np.zeros((4, 4), bool)
    for c in pts:
        for p in c.reshape(-1, 2):
            hit[min(3, int(p[1] / H * 4)), min(3, int(p[0] / W * 4))] = True
    return hit


def cmd_intrinsics(a):
    files = capture_files(a.dir)                 # not capture's preview.png
    obj, pts, size, used = [], [], None, []
    for f in files:
        g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        if g is None:
            continue
        if size is None:
            size = (g.shape[1], g.shape[0])
        if (g.shape[1], g.shape[0]) != size:
            sys.exit(f"{f}: size {g.shape[1]}x{g.shape[0]} differs from {size[0]}x{size[1]} -> use one image size only")
        c = find_checker(g, a.board)
        print(f"  {os.path.basename(f)}: {'found' if c is not None else 'NOT found'}")
        if c is not None:
            obj.append(checker_object_points(a.board)); pts.append(c); used.append(os.path.basename(f))
    if len(obj) < 10:
        sys.exit(f"only {len(obj)} images with the whole board found; need 10 or more (15-20 is better): tilt the board up to "
                 "about 30 deg, move it over the middle half of the picture, hold it still")
    r = calibrate(obj, pts, size)
    hit = coverage(pts, size)
    r.update({"name": a.name, "board": a.board, "images": used, "coverage_4x4": hit.astype(int).tolist()})
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, f"intrinsics_{a.name}_{size[0]}x{size[1]}.json")
    json.dump(r, open(p, "w"), indent=1)
    print(f"rms {r['rms_px']:.3f} px  fx {r['K'][0][0]:.1f}  fy {r['K'][1][1]:.1f}  "
          f"FOV h {r['fov_h_deg']:.1f} v {r['fov_v_deg']:.1f} d {r['fov_d_deg']:.1f} deg\nwrote {p}")
    print("coverage (rows top->bottom, 1 = some corners there):")
    for row in hit.astype(int):
        print("   ", " ".join(map(str, row)))
    if not hit[1:3, 1:3].all():
        print("!! the middle of the picture (where the board and pieces are) is not fully covered: add photos there")
    if r["rms_px"] > 1.0:
        print("!! rms over 1 px: blurred or moving pictures, or a board that is not flat. Retake before using this.")


# ---------------------------------------------------------------- capture (live; not run on a real camera yet)
PREVIEW = "preview.png"     # capture's look-at picture in DIR; intrinsics and the numbering skip it


def capture_files(d):
    """the photos capture saved in d (NNN.png and other pictures), without the preview picture."""
    fs = glob.glob(os.path.join(d, "*.png")) + glob.glob(os.path.join(d, "*.jpg"))
    return sorted(f for f in fs if os.path.basename(f).lower() != PREVIEW)


def next_index(d):
    """number for the next NNN.png in d: one more than the largest number there (0 for an empty folder)."""
    nums = [int(os.path.splitext(os.path.basename(f))[0]) for f in glob.glob(os.path.join(d, "*.png"))
            if os.path.splitext(os.path.basename(f))[0].isdigit()]
    return max(nums) + 1 if nums else 0


def window_ok(name="k5 capture"):
    """True if this OpenCV can open a window. opencv-python-headless (LeRobot's .venv, 2026-11-23) raises cv2.error
    "The function is not implemented" here."""
    try:
        cv2.namedWindow(name)
        return True
    except cv2.error:
        return False


def look(frame, board):
    """detect the board in one camera frame -> (picture with the detection drawn, one-line message, good to keep).
    good: the whole checkerboard found, or 4 or more sheet markers (pose needs 4)."""
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    show = frame.copy() if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    h, w = g.shape[:2]
    if board == "sheet":
        found = detect_markers(g)
        for c in found.values():
            cv2.polylines(show, [c.astype(np.int32)], True, (0, 255, 0), 2)
        miss = sorted(set(MARKERS) - set(found))
        msg = f"markers {sorted(found)} ({len(found)} of 8" + (f"; missing {miss})" if miss else ")")
        return show, msg, len(found) >= 4, None
    c = find_checker(g, board)
    if c is None:
        return show, "board NOT found", False, None
    cv2.drawChessboardCorners(show, BOARDS[board][:2], c, True)
    p = c.reshape(-1, 2)
    cx, cy = p.mean(axis=0) / [w, h]
    span = (p[:, 0].max() - p[:, 0].min()) / w
    return show, f"board found: centre at {cx:.2f}, {cy:.2f} of the picture (0.5 = middle), corners span {span:.0%} of the width", True, c


class LatestFrame:
    """reads the camera all the time in a thread and keeps only the newest frame, so a picture taken on Enter is
    from after the key, not one that waited in the driver's buffer."""

    def __init__(self, cap):
        import threading
        self.cap, self.frame, self.seq, self.failed = cap, None, 0, False
        self.cond = threading.Condition()
        self.running = True
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _run(self):
        while self.running:
            ok, f = self.cap.read()
            with self.cond:
                if not ok:
                    self.failed = True
                    self.cond.notify_all()
                    return
                self.frame, self.seq = f, self.seq + 1
                self.cond.notify_all()

    def fresh(self, after=2, timeout=5.0):
        """the newest frame once `after` new frames came in since this call (None if the camera stopped)."""
        with self.cond:
            s0 = self.seq
            self.cond.wait_for(lambda: self.failed or self.seq >= s0 + after, timeout)
            if self.seq < s0 + after:
                return None
            return self.frame.copy()

    def stop(self):
        self.running = False
        self.th.join(timeout=2.0)


def capture_no_window(cap, a, read_line=input):
    """capture without a window: Enter = take one picture (kept only if the board is found), p + Enter = only look,
    q + Enter = quit. Each time the detection is printed and the picture with it drawn goes to DIR\\preview.png."""
    grab = LatestFrame(cap)
    prev = os.path.join(a.dir, PREVIEW)
    need = "4 or more markers" if a.board == "sheet" else "the whole board"
    kept = []                                    # checker corners of the photos already in DIR (for the coverage line)
    for f in (capture_files(a.dir) if a.board != "sheet" else []):
        g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        c = find_checker(g, a.board) if g is not None else None
        if c is not None:
            kept.append(c)
    print(f"no window (this OpenCV cannot show one, or --no-window). Hold still, then:\n"
          f"  Enter = take one picture (kept only with {need})   p + Enter = only look   q + Enter = quit\n"
          f"  each time the picture with the detection drawn on it is written to {prev}")
    try:
        while True:
            try:
                cmd = read_line(f"[{next_index(a.dir):03d}] Enter / p / q > ").strip().lower()
            except EOFError:
                break
            if cmd in ("q", "quit", "exit"):
                break
            if cmd not in ("", "p"):
                print("  ? Enter = take one, p + Enter = only look, q + Enter = quit")
                continue
            frame = grab.fresh()
            if frame is None:
                print("!! no new picture from the camera (unplugged?) -- stopping")
                break
            show, msg, good, c = look(frame, a.board)
            cv2.putText(show, msg, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imwrite(prev, show)
            if cmd == "p":
                print(f"  look only: {msg}  (picture: {prev})")
                continue
            if not good:
                print(f"  NOT kept: {msg}. Needs {need}: move the board / your hand, then Enter again")
                continue
            p = os.path.join(a.dir, f"{next_index(a.dir):03d}.png")
            cv2.imwrite(p, frame)
            print(f"  kept {p}: {msg}")
            if c is not None:
                kept.append(c)
                hit = coverage(kept, (frame.shape[1], frame.shape[0]))
                print(f"    {len(kept)} kept with the board; corners seen in the picture's 4x4 cells (top row first): "
                      + " ".join("".join(map(str, r)) for r in hit.astype(int))
                      + ("" if hit[1:3, 1:3].all() else "  -- the middle 4 cells are not all covered yet"))
    finally:
        grab.stop()


def open_camera(a):
    cap = cv2.VideoCapture(a.index, cv2.CAP_DSHOW)
    if a.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*a.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    cap.set(cv2.CAP_PROP_FPS, a.fps)
    for prop, val, name in ((cv2.CAP_PROP_AUTOFOCUS, 0, "autofocus off"), (cv2.CAP_PROP_AUTO_EXPOSURE, 0.25, "auto exposure off")):
        print(f"  {name}: {'ok' if cap.set(prop, val) else 'not supported by this camera/driver'}")
    return cap


def cmd_capture(a, cap=None, read_line=input, window=None):
    """cap, read_line and window are for the selftest (a fake camera, scripted keys). window None = ask OpenCV."""
    os.makedirs(a.dir, exist_ok=True)
    if window is None:
        window = (not a.no_window) and window_ok()
    cap = open_camera(a) if cap is None else cap
    ok, frame = cap.read()
    if not ok:
        sys.exit("cannot read from the camera (index, size, fps or fourcc not supported?)")
    h, w = frame.shape[:2]
    print(f"camera {a.index}: {w}x{h} (asked {a.width}x{a.height})")
    if (w, h) != (a.width, a.height):
        print("!! the camera gave another size than asked; the intrinsics are only valid for the size it gives")
    if not window:
        try:
            capture_no_window(cap, a, read_line)
        finally:
            cap.release()
        return
    print("window: SPACE = save, Q = quit")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        show, msg, _good, _c = look(frame, a.board)
        cv2.putText(show, f"{msg}   next {next_index(a.dir):03d}   SPACE save  Q quit", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        s = 1280 / w
        cv2.imshow("k5 capture", cv2.resize(show, (int(w * s), int(h * s))) if s < 1 else show)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord(" "):
            p = os.path.join(a.dir, f"{next_index(a.dir):03d}.png")
            cv2.imwrite(p, frame)
            print(f"  saved {p}  ({msg})")
    cap.release()
    cv2.destroyAllWindows()


# ---------------------------------------------------------------- pose
def detect_markers(gray):
    prm = cv2.aruco.DetectorParameters()
    prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO), prm)
    corners, ids, _ = det.detectMarkers(gray)
    found = {}
    if ids is not None:
        for c, i in zip(corners, ids.ravel()):
            if int(i) in MARKERS:
                found[int(i)] = c.reshape(4, 2)
    return found


def solve_from_points(found, K, dist, scale=1.0):
    """found: {marker id: 4x2 image corners (OpenCV order)} -> camera pose in the sheet frame."""
    if len(found) < 4:
        return {"ok": False, "why": f"found markers {sorted(found)}; need 4 or more of ids 0-7", "ids": sorted(found)}
    obj = np.vstack([marker_corners(i) * [scale, 1, scale] for i in sorted(found)])
    img = np.vstack([found[i] for i in sorted(found)]).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        return {"ok": False, "why": "solvePnP failed"}
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
    R, _ = cv2.Rodrigues(rvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    err = np.linalg.norm(proj.reshape(-1, 2) - img, axis=1).reshape(-1, 4).max(axis=1)
    C = (-R.T @ tvec).ravel()
    d = R.T @ np.array([0, 0, 1.0])
    up = R.T @ np.array([0, -1.0, 0])
    tilt = math.degrees(math.acos(max(-1, min(1, -d[1]))))      # angle from straight down
    up_dev = math.degrees(math.atan2(up[2], up[0]))              # image up seen from above, 0 = +x
    return {"ok": True, "ids": sorted(found), "centre_mm": C.tolist(), "dir": d.tolist(), "up": up.tolist(),
            "tilt_from_down_deg": tilt, "up_from_x_deg": up_dev, "reproj_px_max": float(err.max()),
            "reproj_px_per_marker": {str(i): float(e) for i, e in zip(sorted(found), err)},
            "rvec": rvec.ravel().tolist(), "tvec": tvec.ravel().tolist()}


def solve_pose(gray, K, dist, scale=1.0):
    r = solve_from_points(detect_markers(gray), K, dist, scale)
    if not r["ok"] and len(detect_markers(cv2.flip(gray, 1))) >= 4:
        r["why"] += " -- but the LEFT-RIGHT FLIPPED picture has them: the camera or the viewer mirrors the image. Turn mirroring off"
    return r


def sheet_to_arm_centre(c, ruler_scale, paper):
    """camera centre c (true mm on the sheet: the ruler scale is already in the pose) -> arm frame, without the rotation.
    k1-err's model is label = (1 + scale_permil/1000) * arm + (dx, dz); a printed label L lies at ruler_scale * L on the
    sheet -> arm = (c / ruler_scale - d) / s. Until 2026-11-20 this was (c - d) / s: with both --ruler-mm and --k1err the
    print's scale was taken out twice (review C42q-2 and the peer's C46-7; ~0.6-0.8 mm in x for a 0.4 % print)."""
    s = 1 + paper["scale_permil"] / 1000
    return [(c[0] / ruler_scale - paper["dx_mm"]) / s, c[1], (c[2] / ruler_scale - paper["dz_mm"]) / s]


def load_intrinsics(path):
    j = json.load(open(path))
    return np.array(j["K"], np.float64), np.array(j["dist"], np.float64), tuple(j["image_size"])


def cmd_pose(a):
    K, dist, size = load_intrinsics(a.intrinsics)
    g = cv2.imread(a.image, cv2.IMREAD_GRAYSCALE)
    if g is None:
        sys.exit(f"cannot read {a.image}")
    if (g.shape[1], g.shape[0]) != size:
        sys.exit(f"image {g.shape[1]}x{g.shape[0]} but the intrinsics are for {size[0]}x{size[1]} -> same camera mode for both")
    rl = [float(v) for v in a.ruler_mm.split(",")]
    if len(rl) == 2 and abs(rl[0] - rl[1]) / RULER_MM > 0.005:
        print(f"!! the two rulers differ by {abs(rl[0] - rl[1]):.2f} mm: the printer stretched one direction. Using the mean; "
              "k1-err.js also uses one scale. Reprint if it is over 1 mm")
    scale = sum(rl) / len(rl) / RULER_MM
    r = solve_pose(g, K, dist, scale)
    r.update({"image": os.path.basename(a.image), "intrinsics": os.path.basename(a.intrinsics), "ruler_mm": rl,
              "frame": "sheet (sim table axes: +x away from the arm, +z the arm's right, y up from the board top)"})
    if not r["ok"]:
        sys.exit("pose: " + r["why"])
    c = r["centre_mm"]
    print(f"markers {r['ids']}; reprojection max {r['reproj_px_max']:.2f} px")
    print(f"camera centre in the SHEET frame (x, y, z) = ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}) mm; "
          f"tilt from straight down {r['tilt_from_down_deg']:.2f} deg; image up {r['up_from_x_deg']:+.1f} deg from +x")
    if abs(r["up_from_x_deg"]) > UP_WARN_DEG:
        print(f"!! the image's up is turned {r['up_from_x_deg']:+.1f} deg from +x (the sim assumes 0): turn the camera, or tell the sim")
    cmp_ = c
    if a.k1err:
        k = json.load(open(a.k1err))
        pp = k["paper"]
        s = 1 + pp["scale_permil"] / 1000
        cmp_ = sheet_to_arm_centre(c, scale, pp)
        pan = k["joints"][0]["deg"]
        r["centre_arm_mm_without_rotation"] = cmp_
        r["k1err"] = {"file": os.path.basename(a.k1err), "dx_mm": pp["dx_mm"], "dz_mm": pp["dz_mm"], "scale_permil": pp["scale_permil"],
                      "shoulder_pan_deg": pan}
        print(f"with k1-err (paper dx {pp['dx_mm']}, dz {pp['dz_mm']}, scale {pp['scale_permil']} permil): centre ({cmp_[0]:.1f}, {cmp_[1]:.1f}, "
              f"{cmp_[2]:.1f}) mm. NOT rotated: shoulder_pan {pan} deg holds the sheet's rotation and the pan zero together")
    if a.expect:
        e = [float(v) for v in a.expect.split(",")]
        r["expect_mm"] = e
        r["diff_mm"] = [cmp_[i] - e[i] for i in range(3)]
        print(f"minus expected {e}: ({r['diff_mm'][0]:+.1f}, {r['diff_mm'][1]:+.1f}, {r['diff_mm'][2]:+.1f}) mm"
              + ("" if a.k1err else "  (sheet frame: includes where the sheet lies, a few mm. Give --k1err to take it out)"))
    if r["reproj_px_max"] > 2.0:
        print("!! reprojection over 2 px: the sheet is not flat, a marker is misread, or the intrinsics are for another mode")
    os.makedirs(OUT, exist_ok=True)
    stem = os.path.splitext(os.path.basename(a.image))[0]
    p = os.path.join(OUT, f"pose_{stem}.json")
    json.dump(r, open(p, "w"), indent=1)
    ov = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    pts = np.array([[x, 0, z] for x, z in k1_points()], np.float64) * [scale, 1, scale]
    proj, _ = cv2.projectPoints(pts, np.array(r["rvec"]), np.array(r["tvec"]), K, dist)
    for q in proj.reshape(-1, 2):
        cv2.circle(ov, (int(q[0]), int(q[1])), 8, (0, 0, 255), 2)
    cv2.imwrite(os.path.join(OUT, f"pose_{stem}_overlay.png"), ov)
    print(f"wrote {p} (+ overlay: red rings must sit on the printed K1 points)")


# ---------------------------------------------------------------- selftest (synthetic pictures)
def look_down(C, yaw_deg=0.0, tilt_deg=(0.0, 0.0)):
    """R (world -> camera, OpenCV axes) for a camera at C looking straight down, image up = +x, then tilted."""
    d = np.array([0, -1.0, 0]); up = np.array([1.0, 0, 0])
    def rot(axis, deg):
        return cv2.Rodrigues(np.array(axis, float) * math.radians(deg))[0]
    Rw = rot([0, 1, 0], yaw_deg) @ rot([1, 0, 0], tilt_deg[0]) @ rot([0, 0, 1], tilt_deg[1])
    d, up = Rw @ d, Rw @ up
    right = np.cross(d, up)
    Rc = np.vstack([right, -up, d])                      # rows: camera X (right), Y (down), Z (forward) in world
    t = -Rc @ np.asarray(C, float)
    return Rc, t


RENDER_SS = int(os.environ.get("K5_RENDER_SS", "1"))   # render_sheet's samples per pixel when the caller gives none
                    # (2026-11-20 C42-W5: K5_RENDER_SS=4 runs the other selftests drawn as a sensor averages; unset = 1).
                    # Read once at import; it also changes sway_check's synth_trial, k5_wrist's and k5_overlay's pictures
if RENDER_SS < 1:
    sys.exit(f"K5_RENDER_SS must be a whole number 1 or more (got {RENDER_SS})")
if RENDER_SS != 1:
    print(f"[k5_camera] render_sheet draws {RENDER_SS} x {RENDER_SS} samples per pixel by default (K5_RENDER_SS)")


def render_sheet(sheet, K, Rc, t, size, scale=1.0, noise=2.0, seed=1, ss=None):
    """ss > 1 (2026-11-20 C42-W4): draw ss x ss samples per pixel and average them (INTER_AREA), as a lens and a sensor
    pixel do. With ss = 1 (the default, all the selftests so far) the 300 DPI sheet is shrunk about 7 x by point sampling:
    a sharp edge lands on whole pixels, so the ArUco corners move in steps, mostly up-down the picture (camera moved
    0.3 mm along x, FOV 64: the corners' mean moved 0.003-0.004 px instead of ~0.48 px; ss = 4: 0.386-0.387 px; along z
    the steps are smaller: 0.336 px. outputs/k5/C42-W4_checks/pose_step_ss1.txt, _ss4.txt). ss = 4 still reads small
    moves ~20 % short. Phase correlation over the whole sheet is hardly hit (many edges)."""
    ss = RENDER_SS if ss is None else ss
    # sheet pixel (u, v, 1) -> world (x, 0, z): x = X_FAR - v/PPM, z = Z_LEFT + u/PPM (times the print scale)
    A = np.array([[0, -1 / PPM, X_FAR], [1 / PPM, 0, Z_LEFT], [0, 0, 1]]) * np.array([[scale], [scale], [1]])
    P = K @ np.column_stack([Rc[:, 0], Rc[:, 2], t])     # world (x, z, 1) -> image
    if ss > 1:
        # the centre of pixel i of the picture is at ss * i + (ss - 1) / 2 in the big drawing (pixel centres on integers)
        S = np.array([[ss, 0, (ss - 1) / 2], [0, ss, (ss - 1) / 2], [0, 0, 1.0]])
        big = cv2.warpPerspective(sheet, S @ P @ A, (size[0] * ss, size[1] * ss), flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=150)
        img = cv2.resize(big, size, interpolation=cv2.INTER_AREA)
    else:
        img = cv2.warpPerspective(sheet, P @ A, size, flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=150)
    rng = np.random.default_rng(seed)
    img = np.clip(img.astype(np.float32) + rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return cv2.GaussianBlur(img, (3, 3), 0.6)


def ground_error(r, Rc, t, K, y=0.0, xs=(60, 300), zs=(-240, 240)):
    """max distance [mm] on the plane at height y between where the true camera and the solved camera send the same
    pixel, over a grid (default: the reach area x 60..300, z -240..240). This is the error that moves a found piece."""
    R2, _ = cv2.Rodrigues(np.array(r["rvec"])); t2 = np.array(r["tvec"]).ravel()
    C2 = -R2.T @ t2
    worst = 0.0
    for x in np.linspace(xs[0], xs[1], 9):
        for z in np.linspace(zs[0], zs[1], 11):
            P = np.array([x, y, z])
            p = K @ (Rc @ P + t); u = p[:2] / p[2]                     # true pixel
            ray = R2.T @ np.linalg.inv(K) @ np.array([u[0], u[1], 1.0])  # back through the solved camera
            Q = C2 + (y - C2[1]) / ray[1] * ray
            worst = max(worst, float(np.hypot(Q[0] - x, Q[2] - z)))
    return worst


SIM_JS = r"""
const {load}=require('./tools/_load'); load('kinematics.js','so101.js');
const W=1920,H=1080,FOV=60, cam=SO101.overheadCamera(), ty=SO101.overheadTableY(), im=SO101.overheadImage(cam,FOV,W,H);
const mk=JSON.parse(process.argv[1]), out={tip:cam.tip.map((v,i)=>i===1?(v-ty)*1000:v*1000), dir:cam.dir, up:cam.up, f:im.f, pts:{}};
for (const id in mk) out.pts[id]=mk[id].map(p=>im.project([p[0]/1000, ty, p[2]/1000]));
console.log(JSON.stringify(out));
"""


def sim_check():
    """the sim's own camera (so101.js overheadCamera + overheadImage) projects the marker corners; solve them here.
    Independent of render_sheet/look_down, so a mirrored or rotated sheet frame would show up as a large error."""
    mk = {str(i): marker_corners(i).tolist() for i in MARKERS}
    r = subprocess.run(["node", "-e", SIM_JS, json.dumps(mk)], cwd=os.path.join(ROOT, "sim"), capture_output=True, text=True)
    if r.returncode:
        return None, "node failed: " + r.stderr[-300:]
    s = json.loads(r.stdout)
    W, H = 1920, 1080
    K = np.array([[s["f"], 0, W / 2 - 0.5], [0, s["f"], H / 2 - 0.5], [0, 0, 1]])   # sim pixel i spans i..i+1; OpenCV centre is i
    found = {int(i): np.array(p) - 0.5 for i, p in s["pts"].items() if all(q is not None for q in p)}
    sol = solve_from_points(found, K, np.zeros(5))
    return (sol, s), None


class FakeCap:
    """a camera for the selftest: read() gives the picture of the scene the test set (about 200 frames a second)."""

    def __init__(self, scenes, scene):
        self.scenes, self.scene, self.released = scenes, scene, False

    def read(self):
        import time
        time.sleep(0.005)
        return True, self.scenes[self.scene].copy()

    def set(self, *_):
        return False

    def release(self):
        self.released = True


def capture_selftest(sheet, K, size):
    """capture's no-window path with FakeCap: kept only with the board / 4 markers, the picture is from after the key
    (the scene changes when the key comes; the frame before it is another scene), p only looks, the numbering goes on
    from what is in DIR, preview.png is written and intrinsics skips it."""
    import contextlib, io, tempfile, types
    W, H = size
    bgr = lambda g: cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    Rc, t = look_down((157.3, 540.0, 19.5))
    sh = render_sheet(sheet, K, Rc, t, size)
    three = sh.copy()                                   # markers 3-7 covered: 3 left, one short of what pose needs
    for i in (3, 4, 5, 6, 7):
        p = K @ (Rc @ marker_corners(i).T + t[:, None]); p = (p[:2] / p[2]).T
        (u0, v0), (u1, v1) = p.min(axis=0).astype(int) - 15, p.max(axis=0).astype(int) + 15
        three[max(0, v0):v1, max(0, u0):u1] = 150
    Hm = np.array([[0.45, 0.05, 300], [-0.03, 0.45, 150], [0.00001, 0.00002, 1]])
    scenes = {"blank": np.full((H, W, 3), 150, np.uint8), "sheet": bgr(sh), "three": bgr(three),
              "checker": bgr(cv2.warpPerspective(make_checker(*BOARDS["checker"]), Hm, size, flags=cv2.INTER_LINEAR, borderValue=150))}
    can_window = window_ok("k5 selftest")
    if can_window:
        cv2.destroyAllWindows()
    print(f"  info: this OpenCV can show a window: {'yes (the test forces --no-window)' if can_window else 'no -> capture runs without a window'}")

    def run(d, board, steps, start, no_window):
        cam = FakeCap(scenes, start)
        it = iter(steps)

        def read_line(_prompt):
            try:
                scene, cmd = next(it)
            except StopIteration:
                raise EOFError
            if scene:
                cam.scene = scene
            return cmd
        a = types.SimpleNamespace(index=0, dir=d, width=W, height=H, fps=5, fourcc="MJPG", board=board, no_window=no_window)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            cmd_capture(a, cap=cam, read_line=read_line, window=None)
        return out.getvalue(), cam

    fails = 0

    def check(ok, what):
        nonlocal fails
        fails += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {what}")

    with tempfile.TemporaryDirectory() as d:
        cv2.imwrite(os.path.join(d, "000.png"), scenes["blank"])          # a photo from before: numbering goes on from it
        cv2.imwrite(os.path.join(d, PREVIEW), scenes["blank"])            # an old preview
        out, cam = run(d, "sheet", [("sheet", ""), ("blank", ""), ("three", ""), ("sheet", "p"), ("sheet", ""), (None, "x"), (None, "q")],
                       "blank", True)
        names = sorted(os.listdir(d))
        check(names == ["000.png", "001.png", "002.png", PREVIEW], f"--board sheet: kept 001, 002 after the old 000; files {names}")
        k1 = cv2.imread(os.path.join(d, "001.png"))
        check(k1 is not None and np.array_equal(k1, scenes["sheet"]),
              "the first kept picture is the scene after the Enter (the camera showed a blank table before it)")
        check(out.count("NOT kept") == 2 and "markers [0, 1, 2] (3 of 8" in out and "markers [] (0 of 8" in out,
              "a blank table and 3 markers are not kept, and the marker ids are printed")
        check("look only: markers [0, 1, 2, 3, 4, 5, 6, 7] (8 of 8)" in out, "p + Enter only looks (8 markers printed, nothing kept)")
        pv = cv2.imread(os.path.join(d, PREVIEW))
        check(pv is not None and not np.array_equal(pv, scenes["blank"]) and pv.shape == scenes["sheet"].shape
              and len(detect_markers(cv2.cvtColor(pv, cv2.COLOR_BGR2GRAY))) >= 4, "preview.png is rewritten with the last picture")
        check("? Enter" in out and cam.released, "an unknown key is answered, q quits and the camera is let go")
    with tempfile.TemporaryDirectory() as d:
        # no --no-window: the window probe decides (this OpenCV cannot -> no window); an OpenCV that can is forced
        out, cam = run(d, "checker", [("checker", ""), ("blank", "")], "blank", can_window)
        names = sorted(os.listdir(d))
        check(names == ["000.png", PREVIEW] and "board found: centre at" in out and "4x4 cells" in out and "board NOT found" in out,
              f"--board checker: the board kept with where it is and the coverage, a blank one not; files {names}")
        check([os.path.basename(f) for f in capture_files(d)] == ["000.png"] and cam.released,
              "intrinsics' file list skips preview.png; the end of input (EOF) quits")
    return fails


def cmd_selftest(_a):
    W, H, fov_v = 1920, 1080, 60.0
    f = H / 2 / math.tan(math.radians(fov_v / 2))
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1]])
    dist = np.zeros(5)
    sheet = make_sheet()
    fails = 0
    cases = [((157.3, 540.0, 19.5), 0, (0, 0)), ((150.0, 530.0, 30.0), 3, (1.0, -0.7)), ((165.0, 548.0, 10.0), -2, (-1.5, 0.5))]
    print(f"== pose: synthetic overhead pictures of the sheet (1920x1080, vertical FOV 60 deg, noise 2). pass: error over the reach < {GROUND_TOL_MM} mm ==")
    for C, yaw, tilt in cases:
        Rc, t = look_down(C, yaw, tilt)
        img = render_sheet(sheet, K, Rc, t, (W, H))
        r = solve_pose(img, K, dist)
        if not r["ok"]:
            print("  FAIL", r["why"]); fails += 1; continue
        e = np.linalg.norm(np.array(r["centre_mm"]) - C)
        g = ground_error(r, Rc, t, K)
        gs = ground_error(r, Rc, t, K, xs=(X_NEAR, X_FAR), zs=(Z_LEFT, Z_RIGHT))
        ok = g < GROUND_TOL_MM
        fails += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} true {C} yaw {yaw} tilt {tilt}: table-point error max {gs:.2f} mm on the sheet, {g:.2f} mm over the reach "
              f"(camera centre {e:.2f} mm; centre and tilt trade off), markers {len(r['ids'])}, reproj max {r['reproj_px_max']:.2f} px")
    # corner noise: 0.5 px random on the true corners (points only), 200 tries -> 95th percentile over the reach
    Rc, t = look_down(cases[0][0])
    rng = np.random.default_rng(5)
    errs = []
    for _ in range(200):
        found = {}
        for i in MARKERS:
            p = K @ (Rc @ marker_corners(i).T + t[:, None]); p = (p[:2] / p[2]).T
            found[i] = p + rng.normal(0, 0.5, p.shape)
        errs.append(ground_error(solve_from_points(found, K, dist), Rc, t, K))
    p95 = float(np.percentile(errs, 95))
    ok = p95 < GROUND_TOL_MM
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} corner noise 0.5 px (200 tries, 8 markers): error over the reach, 95th percentile {p95:.2f} mm")
    # negative: printed 3 % small but told the nominal ruler -> the camera must come out ~3 % farther (wrong ruler is visible)
    img = render_sheet(sheet, K, Rc, t, (W, H), scale=0.97)
    r = solve_pose(img, K, dist, 1.0)
    dy = r["centre_mm"][1] - cases[0][0][1] if r["ok"] else float("nan")
    ok = r["ok"] and dy > 10
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} negative: sheet printed at 97 % but told {RULER_MM:.0f} mm -> height off by {dy:+.1f} mm (about +16: a smaller print looks farther)")
    r2 = solve_pose(img, K, dist, 0.97)
    e2 = ground_error(r2, Rc, t, K) if r2["ok"] else float("nan")
    ok = r2["ok"] and e2 < GROUND_TOL_MM
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} same picture with --ruler-mm {RULER_MM * 0.97:.1f} -> error over the reach {e2:.2f} mm")
    # mirrored picture must be refused with the mirror hint
    rm = solve_pose(cv2.flip(render_sheet(sheet, K, Rc, t, (W, H)), 1), K, dist)
    ok = (not rm["ok"]) and "FLIPPED" in rm.get("why", "")
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} a left-right mirrored picture is refused and the mirror is named")
    # --ruler-mm together with --k1err: printed at 99.6 %, label 0 laid on the arm's 0. The pose (solved with the ruler) is
    # then in true mm = the arm frame (the "same picture with --ruler-mm" line above checks the solve). k1-err reads the
    # labels at the arm's points: label = arm / 0.996 -> scale_permil +4.016, d = 0. The conversion must give the centre
    # back unchanged; the old formula (c - d) / s took the print out twice. (The solved centre itself wanders ~2 mm with
    # the tilt, so the true centre is fed here, not a solve: this checks the formula, the solve is checked above.)
    kp = 0.996
    C0 = np.array(cases[0][0])
    pp = {"dx_mm": 0.0, "dz_mm": 0.0, "scale_permil": (1 / kp - 1) * 1000}
    good = np.array(sheet_to_arm_centre(C0, kp, pp))
    sv = 1 + pp["scale_permil"] / 1000
    old = np.array([C0[0] / sv, C0[1], C0[2] / sv])
    e_good, e_old = float(np.hypot(*(good - C0)[[0, 2]])), float(np.hypot(*(old - C0)[[0, 2]]))
    ok = e_good < 0.01 and e_old > 0.5
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} --ruler-mm {RULER_MM * kp:.1f} with --k1err scale +{pp['scale_permil']:.2f} permil: centre in the arm frame "
          f"{e_good:.3f} mm from the true one; negative: the old formula (print taken out twice) {e_old:.2f} mm (must be > 0.5)")
    # the same with the sheet laid off its place, d = (3, -2) mm (2026-11-20 C42-W4, review C42q 2-7): label = s * arm + d,
    # a label L lies at kp * L on the sheet -> the camera at C0 (arm) is at kp * (s * C0 + d) (sheet). Back to C0;
    # negative: d with the wrong sign
    pd = dict(pp, dx_mm=3.0, dz_mm=-2.0)
    dv = np.array([pd["dx_mm"], 0.0, pd["dz_mm"]])
    c_sheet = kp * (sv * np.array([C0[0], 0.0, C0[2]]) + dv) + np.array([0.0, C0[1], 0.0])
    back = np.array(sheet_to_arm_centre(c_sheet, kp, pd))
    wrong = np.array(sheet_to_arm_centre(c_sheet, kp, dict(pd, dx_mm=-3.0, dz_mm=2.0)))
    e_d, e_w = float(np.hypot(*(back - C0)[[0, 2]])), float(np.hypot(*(wrong - C0)[[0, 2]]))
    ok = e_d < 0.01 and e_w > 5.0
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} the same with the sheet laid (3, -2) mm off (k1-err d): centre {e_d:.3f} mm from the true one; "
          f"negative: d with the wrong sign {e_w:.2f} mm (must be > 5)")

    print("== the sim's own camera (sim/so101.js overheadCamera + overheadImage, node) ==")
    res, err = sim_check()
    if err:
        print("  FAIL", err); fails += 1
    else:
        sol, s = res
        tip = np.array(s["tip"])
        ok = sol["ok"] and np.linalg.norm(np.array(sol["centre_mm"]) - tip) < 0.5 and np.dot(sol["dir"], s["dir"]) > math.cos(math.radians(0.1)) \
            and np.dot(sol["up"], s["up"]) > math.cos(math.radians(0.1))
        fails += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} sim lens tip {np.round(tip, 2).tolist()} -> solved {np.round(sol['centre_mm'], 2).tolist() if sol['ok'] else sol['why']}; "
              f"dir {np.round(sol.get('dir', [0, 0, 0]), 4).tolist()} (sim {s['dir']}), up {np.round(sol.get('up', [0, 0, 0]), 4).tolist()} (sim {s['up']})")

    print("== intrinsics: synthetic checkerboard corner sets with distortion (no picture; checks the solve) ==")
    rng = np.random.default_rng(3)
    obj1 = checker_object_points("checker")
    dist_true = np.array([-0.12, 0.05, 0.0005, -0.0003, 0.0])
    objs, pts = [], []
    for k in range(20):
        rv = rng.normal(0, 0.35, 3)
        tv = np.array([-110 + rng.normal(0, 40), -60 + rng.normal(0, 30), 450 + rng.normal(0, 80)])
        p, _ = cv2.projectPoints(obj1, rv, tv, K, dist_true)
        p = p.reshape(-1, 2) + rng.normal(0, 0.2, (len(obj1), 2))
        if (p[:, 0] < 0).any() or (p[:, 0] >= W).any() or (p[:, 1] < 0).any() or (p[:, 1] >= H).any():
            continue
        objs.append(obj1); pts.append(p.astype(np.float32).reshape(-1, 1, 2))
    r = calibrate(objs, pts, (W, H))
    ef = abs(r["K"][0][0] - f) / f * 100
    ok = len(objs) >= 10 and ef < 1.0 and abs(r["fov_v_deg"] - fov_v) < 0.6
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} {len(objs)} views: fx {r['K'][0][0]:.1f} (true {f:.1f}, {ef:.2f} %), vertical FOV {r['fov_v_deg']:.2f} (true {fov_v}), "
          f"k1 {r['dist'][0]:+.3f} (true {dist_true[0]:+.3f}), rms {r['rms_px']:.3f} px")
    print("== checker detection on synthetic pictures of the printed boards ==")
    for b in BOARDS:
        ch = make_checker(*BOARDS[b])
        Hm = np.array([[0.45, 0.05, 300], [-0.03, 0.45, 150], [0.00001, 0.00002, 1]])
        img = cv2.warpPerspective(ch, Hm, (W, H), flags=cv2.INTER_LINEAR, borderValue=150)
        ok = find_checker(img, b) is not None
        fails += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {b}: findChessboardCornersSB {'found' if ok else 'did not find'} the corners")
    print("== capture without a window: a fake camera and keys typed by the test (no real camera) ==")
    fails += capture_selftest(sheet, K, (W, H))
    print("SELFTEST", "OK" if not fails else f"FAILED ({fails})")
    sys.exit(1 if fails else 0)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    s = p.add_subparsers(dest="cmd", required=True)
    s.add_parser("sheet")
    q = s.add_parser("capture"); q.add_argument("index", type=int); q.add_argument("dir")
    q.add_argument("--width", type=int, default=1920); q.add_argument("--height", type=int, default=1080)
    q.add_argument("--fps", type=int, default=5); q.add_argument("--fourcc", default="MJPG")
    q.add_argument("--board", choices=["checker", "small", "sheet"], default="checker")
    q.add_argument("--no-window", action="store_true", help="Enter / p / q in the terminal even if a window could be shown")
    q = s.add_parser("intrinsics"); q.add_argument("dir"); q.add_argument("--name", default="top")
    q.add_argument("--board", choices=list(BOARDS), default="checker")
    q = s.add_parser("pose"); q.add_argument("image"); q.add_argument("--intrinsics", required=True)
    q.add_argument("--ruler-mm", default=f"{RULER_MM}", help="measured length of the printed 150 mm rulers: one value or 'x,z'")
    q.add_argument("--expect", default=None, help="x,y,z mm to compare with (e.g. the sim's lens tip 157.3,540,19.5)")
    q.add_argument("--k1err", default=None, help="sim/out/k1-err/<run>.json: take the sheet's offset and scale out before --expect")
    s.add_parser("selftest")
    a = p.parse_args()
    {"sheet": cmd_sheet, "capture": cmd_capture, "intrinsics": cmd_intrinsics, "pose": cmd_pose, "selftest": cmd_selftest}[a.cmd](a)


if __name__ == "__main__":
    main()
