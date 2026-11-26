r"""Sway (b) of the C stand (hardware/C_台の設計.md, "揺れの見積もり"): how long to wait after the arm stops before the top
camera's picture is still. No robot is touched; the capture opens the top camera only.

usage (the .venv python, it needs OpenCV):
  python tools\sway_check.py burst INDEX DIR [--seconds 2.0] [--settle 3.0] [--width 1920 --height 1080 --fps 30 --fourcc MJPG]
        NOT RUN YET (opens a camera). The K1/K5 sheet v2 lies under the top camera. Make the stand sway: in D2 (no powered
        arm moves after the pole is on) PRESS SPACE FIRST, THEN bump the base board; that gives the damping, not the wait
        after a real stop. The wait after a real arm stop is measured later (D3/D4), with the move run in another tab.
        The last RING_S seconds before SPACE are kept too (negative times), so the stop can be found in the pictures. The tool keeps every frame for --seconds with its time after SPACE, then waits
        until --settle and takes 5 frames as the still picture. One trial per SPACE, 5-10 trials. Q quits.
        Short exposure: set the camera's exposure low (a long exposure smears the sway into blur instead of a shift).
        Writes DIR\trial_NN\t_<ms>.png, ref_K.png and DIR\trial_NN\times.json.
  python tools\sway_check.py analyze DIR [--tol-mm 0.3] [--bin 0.1]
        every trial in DIR: each frame against the still picture by phase correlation (sub-pixel shift of the whole
        picture) -> shift on the sheet in mm (mm per pixel from the markers' printed size; 4+ markers must be seen) -> table: time bin -> largest shift over the trials,
        and the shortest wait after which no frame of any trial is off by more than --tol-mm. Writes DIR\sway.json.
        Also by TILES (2026-11-20): the sheet's area (the box round the markers) cut into 3 bands of rows x 2 columns,
        each tile phase-correlated on its own -> a straight fit over the tiles -> the largest move at the 4 CORNERS of
        the box per frame ("wait by tiles"), the turn of the picture (deg), the row shear and the row stretch (rolling
        shutter, px per 100 rows). Use the LONGER of the two waits.
  python tools\sway_check.py selftest            (all; about 4-5 min)
  python tools\sway_check.py selftest-tiles      (the tile cases only)
        synthetic top pictures (k5_camera's sheet and camera) with a damped sway of the camera (the stand's numbers: 27.4 Hz,
        damping 0.02-0.05, hardware/C_台の設計.md; the size 0.8 mm + 0.15 deg is made up), sampled at 30 frames/s ->
        analyze -> the wait matches the true swing AT THE SAME FRAMES (each frame an instant, translation only: it checks
        the reading, not the camera's shutter). The seconds it prints are NOT a forecast for the stand.
        Negative: a sway that does not die out, a sheet that moved for good during the trial.
        Tile cases (rolling shutter 25 ms GUESS with a swing up-down / across the picture, a twist of the pole; drawn with
        4 x 4 samples per pixel): the truth is the largest move over a 13 x 3 grid in the box (corners included) at the
        moment each point's band was drawn. Pass: the tiles' wait never short, at most 2 frames long, largest move within
        10 %, the case's slope within 20 %. Negative: the whole-picture shift must FAIL the same lines in every case.
        Only 3 seeds per case; the pass has NO margin on the wait (see the night log C42-W4 for other seeds).

Why a burst at 30 frames/s works although 27 Hz is faster than the frame rate: each frame is a short-exposure snapshot;
its offset from the still picture is the sway at that moment. The frequency aliases, but the largest offset in each time
bin over several trials (random phase) follows the envelope, which is what decides the wait.
The whole-picture shift misses two things, so the tiles are read too (2026-11-20, the tile cases of the selftest):
  - a TURN of the picture (a twist of the pole): the whole-picture shift reads ~0, the tiles at the sheet's ends move.
  - the ROLLING SHUTTER (one 27 Hz swing is 36 ms; a 1080p webcam may take 20-30 ms to read top to bottom): each band of
    rows sees the swing at another moment, the whole-picture shift is a blend and reads SMALLER, the wait may come out
    SHORT. A swing up-down the picture (along x: image up is +x) stretches or squeezes the rows ("stretch"), a swing
    across it (along z) slants them ("shear").
    The tiles are 3 bands over the markers' box (each about a third of the box's rows), 2 columns; each tile's reading is
    placed where its edges are (tile_points: the markers sit near the box's ends, so the tile middles were too close
    together; selftest drawn 4 x 4, middles -> edges: stretch +15 -> +7 %, shear +12 -> +2 %, largest move at the corners
    +13 -> +10 %), and the move is taken at the box's 4 corners from a straight fit over
    the 6 tiles (one tile alone reads up to ~0.2 px wrong; the corners move more than the tile middles when it turns).
  The tiles only cover the sheet (the rest of the picture has no texture); points off the sheet (the far reach) are
  not measured: a turn moves them more than the box's corners.
Not done here: a long exposure (blur instead of a shift; --exposure is set and read back), the readout time of the real
camera (unknown: the selftest uses 25 ms GUESS), a sway slower than the frames (it still shows, as long as --seconds covers
it), and t0 found from the pictures (SPACE is still the clock).
"""

import argparse
import collections
import glob
import json
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import k5_camera as K5   # noqa: E402  (the sheet, its markers, the synthetic overhead render)

TOL_LIST = (0.2, 0.3, 0.5, 1.0)
TOL_MM = 0.3        # GUESS: sway left in the top picture. The top camera's own error budget over the reach is 1 mm
                    # (k5_camera GROUND_TOL_MM) and the wrist check corrects what is left; 0.3 mm keeps the sway a small part
BIN_S = 0.1
MIN_MARKERS = 4
REF_FRAMES = 5
RING_S = 1.0       # burst: frames kept from before SPACE (so the stop can be found in the pictures later; SPACE comes late)
MIN_RESP = 0.3     # phase correlation peak below this: the frame is not the same scene (hand in the picture, light changed)
TILE_ROWS, TILE_COLS = 3, 2   # tiles over the sheet's box (rolling shutter: bands of rows; a turn: the two columns differ).
                              # 6 x 2 was tried first: thin bands on the 10 mm grid read 0.4-1.2 px wrong (2026-11-20); 3 x 2: 0.17-0.23 px
MIN_RESP_TILE = 0.15          # a tile's own peak below this: the tile is left out (GUESS; small tiles give lower peaks)
TILE_AT_EDGES = os.environ.get("SWAY_TILE_MIDDLE") is None   # place each tile's reading at its edges' centroid (tile_points);
                                                             # SWAY_TILE_MIDDLE=1 gives the old tile middles (for comparing)


# ---------------------------------------------------------------- measuring one frame against the still picture
def corners(gray):
    f = K5.detect_markers(gray)
    return {i: c.astype(np.float64) for i, c in f.items()}


def mm_per_px(ref):
    """the picture's scale on the sheet: printed marker side (25 mm) over its mean side in pixels."""
    sides = [np.linalg.norm(c[k] - c[(k + 1) % 4]) for c in ref.values() for k in range(4)]
    return K5.MARKER_MM / float(np.mean(sides))


def shift(ref_g, cur_g, s, win):
    """whole-picture shift of cur against the still picture (phase correlation, sub-pixel), in mm on the sheet.
    The ArUco corners alone wobble by 0.2-0.5 px on a still picture (selftest, 2026-11-19), too coarse for 0.3 mm."""
    (dx, dy), resp = cv2.phaseCorrelate(np.float32(ref_g), np.float32(cur_g), win)
    return {"mm": float(math.hypot(dx, dy)) * s, "px": [float(dx), float(dy)], "resp": float(resp)}


def sheet_box(ref_corners, shape):
    """the box round the markers (the sheet's textured area) in the still picture, (u0, v0, u1, v1), 10 px outside."""
    P = np.vstack(list(ref_corners.values()))
    h, w = shape
    return (max(0, int(P[:, 0].min()) - 10), max(0, int(P[:, 1].min()) - 10),
            min(w, int(P[:, 0].max()) + 10), min(h, int(P[:, 1].max()) + 10))


def box_corners(box):
    """the 4 corner pixels of a (u0, v0, u1, v1) box, as an (4, 2) array (u, v)."""
    u0, v0, u1, v1 = box
    return np.array([[u0, v0], [u1 - 1, v0], [u0, v1 - 1], [u1 - 1, v1 - 1]], np.float64)


def tile_boxes(ref_corners, shape):
    """TILE_ROWS x TILE_COLS tiles over the box round the markers (the sheet's textured area), as (u0, v0, u1, v1, uc, vc)."""
    u0, v0, u1, v1 = sheet_box(ref_corners, shape)
    us = np.linspace(u0, u1, TILE_COLS + 1).astype(int)
    vs = np.linspace(v0, v1, TILE_ROWS + 1).astype(int)
    return [(us[i], vs[j], us[i + 1], vs[j + 1], (us[i] + us[i + 1] - 1) / 2, (vs[j] + vs[j + 1] - 1) / 2)
            for j in range(TILE_ROWS) for i in range(TILE_COLS)]


def tile_points(ref_g, boxes):
    """where in each tile the phase correlation actually looks: the centroid of the edges (gradient size x the Hanning
    window) of the still picture, not the tile's middle. On sheet v2 the markers sit near the box's top and bottom rows
    and its left and right ends, so the middles are too close together and the fit's slopes (turn, shear, stretch) and
    the move at the corners came out high (2026-11-20 C42-W4, review C42q 2-3; numbers in the module docstring).
    Returns (n, 4): (u, v) where dx is read, (u, v) where dy is read."""
    g = np.float32(ref_g)
    gx, gy = np.abs(cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)), np.abs(cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3))
    out = []
    for (u0, v0, u1, v1, uc, vc) in boxes:
        han = cv2.createHanningWindow((u1 - u0, v1 - v0), cv2.CV_32F)
        vv, uu = np.mgrid[v0:v1, u0:u1]
        row = []
        # the sideways shift dx is read from the edges across u (|d/du|), the up-down dy from those across v: two places
        for m in (gx, gy):
            w = m[v0:v1, u0:u1] * han
            tot = float(w.sum())
            row += [uc, vc] if tot <= 0 else [float((w * uu).sum() / tot), float((w * vv).sum() / tot)]
        out.append(row)
    return np.array(out)     # (n, 4): (u, v) where dx is read, (u, v) where dy is read


def fit_moves(uv, dxy, at):
    """straight fit dx = a + b (v - vm) + c (u - um), dy = d + e (v - vm) + f (u - um) over the points uv with moves dxy
    (n, 2) [px]; returns the moves at the points `at` (m, 2) and the slopes. uv is (n, 2), or (n, 4) when dx and dy were
    read at different places (tile_points). Used both for the tiles and, in the selftest, for the truth (so the two are
    read the same way)."""
    uv, dxy, at = np.asarray(uv, float), np.asarray(dxy, float), np.asarray(at, float)
    uvx, uvy = (uv[:, :2], uv[:, 2:4]) if uv.shape[1] == 4 else (uv, uv)
    um, vm = uv[:, 0::2].mean(), uv[:, 1::2].mean()      # one centre for both (the slopes do not depend on it)

    def design(p):
        return np.column_stack([np.ones(len(p)), p[:, 1] - vm, p[:, 0] - um])
    cx = np.linalg.lstsq(design(uvx), dxy[:, 0], rcond=None)[0]
    cy = np.linalg.lstsq(design(uvy), dxy[:, 1], rcond=None)[0]
    B = design(at)
    turn = float(cy[2])
    # A turn by th gives b = -th, f = +th. A rolling shutter with a swing ACROSS the picture (along u) gives b alone; with a
    # swing UP-DOWN the picture (along v) gives e alone (the rows stretch or squeeze). A change of scale gives c = e.
    # So: turn = f, shear = b + f, stretch = e - c.
    return B @ cx, B @ cy, {"turn": turn, "shear": float(cx[1]) + turn, "stretch": float(cy[1] - cx[2])}


def tile_shifts(ref_g, cur_g, boxes, s, box, pts=None):
    """each tile against the same tile of the still picture -> a straight fit over the tiles (fit_moves; each tile's
    reading placed at pts = tile_points, or the tile's middle) -> the largest move at the 4 CORNERS of the sheet's box [mm]
    (2026-11-20 C42-W4: the corners move more than the tile middles when the picture turns), the picture's turn [deg],
    the row shear and the row stretch [px per 100 rows]."""
    rows = []
    if pts is None:
        pts = [(bx[4], bx[5], bx[4], bx[5]) for bx in boxes]
    for (u0, v0, u1, v1, _, _), p in zip(boxes, pts):
        a, b = np.float32(ref_g[v0:v1, u0:u1]), np.float32(cur_g[v0:v1, u0:u1])
        win = cv2.createHanningWindow((u1 - u0, v1 - v0), cv2.CV_32F)
        (dx, dy), resp = cv2.phaseCorrelate(a, b, win)
        if resp >= MIN_RESP_TILE:
            rows.append((*p, dx, dy))
    if len(rows) < max(4, len(boxes) // 2):
        return None
    R = np.array(rows)
    # the largest move from the FIT (a single tile reads up to ~0.2 px wrong; the raw largest tile would read the noise as
    # sway). The raw one and the one at the tile middles are kept for the record
    fx, fy, sl = fit_moves(R[:, :4], R[:, 4:], box_corners(box))
    mids = np.array([(bx[4], bx[5]) for bx in boxes])
    cxs, cys, _ = fit_moves(R[:, :4], R[:, 4:], mids)
    return {"max_mm": float(np.hypot(fx, fy).max()) * s, "centre_max_mm": float(np.hypot(cxs, cys).max()) * s,
            "raw_max_mm": float(np.hypot(R[:, 4], R[:, 5]).max()) * s, "turn_deg": math.degrees(sl["turn"]),
            "shear_px_per_100rows": sl["shear"] * 100, "stretch_px_per_100rows": sl["stretch"] * 100, "n": len(rows)}


def analyze_trials(trials, tol, bin_s):
    """trials: [{"ref": gray, "frames": [(t_s, gray)]}] -> table and wait."""
    rows, per_trial, problems = {}, [], []
    for k, tr in enumerate(trials):
        ref = corners(tr["ref"])
        if len(ref) < MIN_MARKERS:
            problems.append(f"trial {k}: {len(ref)} markers in the still picture (need {MIN_MARKERS})")
            continue
        s = mm_per_px(ref)
        h, w = tr["ref"].shape
        win = cv2.createHanningWindow((w, h), cv2.CV_32F)
        boxes = tile_boxes(ref, tr["ref"].shape)
        box = sheet_box(ref, tr["ref"].shape)
        tp = tile_points(tr["ref"], boxes) if TILE_AT_EDGES else None
        pts, tpts, turns, shears, stretches, skipped = [], [], [], [], [], []
        for t, g in tr["frames"]:
            r = shift(tr["ref"], g, s, win)
            if r["resp"] < MIN_RESP:
                problems.append(f"trial {k}: frame at {t:.3f} s does not match the still picture (correlation {r['resp']:.2f})")
                continue
            pts.append((t, r["mm"]))
            if t < 0:            # frames from before SPACE (ring buffer): kept for finding the stop later, not binned, no tiles
                continue
            ti = tile_shifts(tr["ref"], g, boxes, s, box, tp)
            if ti is None:       # a hand or the arm over the sheet: the frame is left out of the tile reading (not fatal)
                skipped.append(t)
            else:
                tpts.append((t, ti["max_mm"]))
                turns.append(abs(ti["turn_deg"])); shears.append(abs(ti["shear_px_per_100rows"]))
                stretches.append(abs(ti["stretch_px_per_100rows"]))
            b = int(t / bin_s)
            rows[b] = max(rows.get(b, 0.0), r["mm"])
        tr["_pts"], tr["_tpts"] = pts, tpts
        if not pts:
            continue
        last = np.array([m for t, m in pts if t >= pts[-1][0] - 0.3])
        # a step: some 0.3 s stretch sits off by about the same amount (a swing sampled at 30/s keeps passing near 0).
        # On the real stand the still picture is taken after the step, so the step shows as "off, then suddenly fine"
        steady = [t for t, _ in pts if t >= 0 and len(w := [m for u, m in pts if t <= u < t + 0.3]) >= 5
                  and min(w) > tol and max(w) - min(w) < 0.5 * float(np.median(w))]
        if steady and np.median(last) <= tol:
            problems.append(f"trial {k}: MOVED FOR GOOD at about {steady[-1] + 0.3:.2f} s: before it the picture sat off by the "
                            "same amount, after it matches the still picture (the sheet slid, the camera or the stand moved) "
                            "-> fix it and take the trial again")
        elif np.median(last) > tol and last.max() - last.min() < tol:   # off by about the same at the end: a step, not a swing
            problems.append(f"trial {k}: MOVED FOR GOOD: the end of the burst sits {np.median(last):.2f} mm from the still picture "
                            "and hardly changes (the sheet slid, the camera or the stand moved) -> fix it and take the trial again")
        elif last.max() > tol:
            problems.append(f"trial {k}: STILL SWINGING: up to {last.max():.2f} mm in the last 0.3 s of the burst "
                            "-> a longer --seconds, or the stand needs stiffening")
        bad = [t for t, m in pts if m > tol and t >= 0]
        tbad = [t for t, m in tpts if m > tol and t >= 0]
        per_trial.append({"trial": k, "mm_per_px": s, "last_over_tol_s": max(bad) if bad else 0.0, "max_mm": max(m for _, m in pts),
                          "tiles_last_over_tol_s": max(tbad) if tbad else 0.0, "tiles_max_mm": max((m for _, m in tpts), default=None),
                          "turn_max_deg": max(turns, default=None), "shear_max_px_per_100rows": max(shears, default=None),
                          "stretch_max_px_per_100rows": max(stretches, default=None), "box_px": list(box),
                          "tiles_skipped_frames_s": skipped})
    table = [{"from_s": b * bin_s, "to_s": (b + 1) * bin_s, "max_mm": rows[b]} for b in sorted(rows)]
    wait = wait_t = None
    if per_trial and not problems:
        wait = max(p["last_over_tol_s"] for p in per_trial)
        wait_t = max(p["tiles_last_over_tol_s"] for p in per_trial)
    # the same for other limits (the wait grows with log(start / limit), so the limit matters; TOL_MM is a guess)
    by_tol = {}
    if per_trial and not problems:
        for tl in TOL_LIST:
            by_tol[str(tl)] = max((t for tr in trials for t, m in tr.get("_pts", []) if m > tl and t >= 0), default=0.0)
    for tr in trials:
        tr.pop("_pts", None); tr.pop("_tpts", None)
    return {"table": table, "trials": per_trial, "problems": problems, "tol_mm": tol, "wait_s": wait, "wait_by_tol_s": by_tol,
            "wait_tiles_s": wait_t, "wait_use_s": None if wait is None else max(wait, wait_t)}


def print_result(r):
    print(f"largest shift on the sheet per {BIN_S:.1f} s after the stop (all trials):")
    for row in r["table"]:
        bar = "#" * min(60, int(row["max_mm"] / r["tol_mm"] * 10))
        print(f"  {row['from_s']:4.1f}-{row['to_s']:4.1f} s  {row['max_mm']:6.3f} mm  {bar}")
    for p in r["problems"]:
        print("!!", p)
    if r["wait_s"] is not None:
        print(f"wait at least {r['wait_s']:.2f} s after SPACE (the last frame over {r['tol_mm']} mm, over {len(r['trials'])} trials); "
              "other limits: " + ", ".join(f"{k} mm {v:.2f} s" for k, v in r["wait_by_tol_s"].items()))
        print(f"by tiles (largest move at the 4 corners of the sheet's box; catches a turn and the rolling shutter): wait {r['wait_tiles_s']:.2f} s -> USE {r['wait_use_s']:.2f} s. "
              "per trial: " + ", ".join(f"turn {p['turn_max_deg']:.3f} deg / shear {p['shear_max_px_per_100rows']:.2f} / "
                                        f"stretch {p['stretch_max_px_per_100rows']:.2f} px per 100 rows"
                                        for p in r["trials"] if p["turn_max_deg"] is not None))
        print("   compare: the sim's settle time T_SETTLE (sim/tools/cycle-tact.js; RELOOK_SHOT in scene-data.js is settle + shot) "
              "is counted from the END OF THE COMMANDED MOVE; SPACE was pressed when the stop was SEEN (later), and a frame comes "
              "out a little after it was exposed. Not the same clock: add a margin")
    else:
        print("no wait given (see the !! lines)")


# ---------------------------------------------------------------- burst (camera only; NOT RUN YET)
def cmd_burst(a):
    cap = cv2.VideoCapture(a.index, cv2.CAP_DSHOW)
    if a.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*a.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height); cap.set(cv2.CAP_PROP_FPS, a.fps)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)            # manual exposure (DSHOW may ignore it and still say yes: read it back)
    cap.set(cv2.CAP_PROP_EXPOSURE, a.exposure)
    cam = {"exposure_asked": a.exposure, "exposure_read": cap.get(cv2.CAP_PROP_EXPOSURE),
           "auto_exposure_read": cap.get(cv2.CAP_PROP_AUTO_EXPOSURE), "fps_read": cap.get(cv2.CAP_PROP_FPS)}
    print("camera:", cam, "(a long exposure or a slow rolling shutter makes the sway look SMALLER: see the docstring)")
    ring = collections.deque()
    try:
        ok, frame = cap.read()
        if not ok:
            sys.exit("cannot read from the camera")
        n = len(glob.glob(os.path.join(a.dir, "trial_*")))
        print("SPACE = start a trial at the moment of the stop / bump, Q = quit")
        while True:
            ok, frame = cap.read()
            if not ok:
                sys.exit("camera stopped")
            now = time.perf_counter()
            ring.append((now, frame))
            while ring and now - ring[0][0] > RING_S:
                ring.popleft()
            g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            m = sorted(K5.detect_markers(g))
            show = frame.copy()
            cv2.putText(show, f"markers {m}  trials {n}  SPACE trial  Q quit", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            sc = 1280 / frame.shape[1]
            cv2.imshow("sway", cv2.resize(show, (int(frame.shape[1] * sc), int(frame.shape[0] * sc))) if sc < 1 else show)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k != ord(" "):
                continue
            t0 = time.perf_counter()
            frames = [(t - t0, fr) for t, fr in ring]            # the last RING_S before SPACE (negative times)
            while time.perf_counter() - t0 < a.seconds:
                ok, fr = cap.read()
                if ok:
                    frames.append((time.perf_counter() - t0, fr))   # time the frame came out (a frame or so after it was exposed)
            while time.perf_counter() - t0 < a.settle:
                cap.read()
            refs = [cap.read()[1] for _ in range(REF_FRAMES)]
            d = os.path.join(a.dir, f"trial_{n:02d}")
            os.makedirs(d, exist_ok=True)
            times = []
            for t, fr in frames:
                name = f"t_{int(round(t * 1000)):+06d}.png"
                cv2.imwrite(os.path.join(d, name), fr)
                times.append({"file": name, "t_s": t})
            for j, fr in enumerate(refs):
                cv2.imwrite(os.path.join(d, f"ref_{j}.png"), fr)
            json.dump({"frames": times, "seconds": a.seconds, "settle": a.settle, "fps_asked": a.fps, "camera": cam}, open(os.path.join(d, "times.json"), "w"), indent=1)
            print(f"  trial {n}: {len(frames)} frames in {a.seconds} s ({len(frames) / a.seconds:.1f}/s)")
            n += 1
    finally:
        cap.release()
        cv2.destroyAllWindows()


def load_trials(dirpath):
    trials = []
    for d in sorted(glob.glob(os.path.join(dirpath, "trial_*"))):
        tj = json.load(open(os.path.join(d, "times.json")))
        refs = [cv2.imread(p, cv2.IMREAD_GRAYSCALE) for p in sorted(glob.glob(os.path.join(d, "ref_*.png")))]
        ref = np.median(np.stack(refs), axis=0).astype(np.uint8)    # the still picture: median of the reference frames
        frames = [(f["t_s"], cv2.imread(os.path.join(d, f["file"]), cv2.IMREAD_GRAYSCALE)) for f in tj["frames"]]
        trials.append({"ref": ref, "frames": frames, "dir": os.path.basename(d)})
    return trials


def cmd_analyze(a):
    trials = load_trials(a.dir)
    if not trials:
        sys.exit(f"no trial_* folders in {a.dir}")
    r = analyze_trials(trials, a.tol_mm, a.bin)
    print_result(r)
    json.dump(r, open(os.path.join(a.dir, "sway.json"), "w"), indent=1)
    print("wrote", os.path.join(a.dir, "sway.json"))


# ---------------------------------------------------------------- selftest
def sway_pose(t, amp_mm, amp_deg, f_hz, zeta, phase):
    """camera offset at time t: a damped swing (translation along x and tilt about z, as a bending pole would)."""
    w = 2 * math.pi * f_hz
    e = math.exp(-zeta * w * t) * math.cos(w * math.sqrt(1 - zeta * zeta) * t + phase)
    return amp_mm * e, amp_deg * e


RS_BANDS = 36    # selftest: the rolling-shutter picture is drawn in this many bands of rows (each band one instant)
TILE_SEED0 = int(os.environ.get("SWAY_SEED0", "80"))       # selftest_tiles: seeds TILE_SEED0 .. +2 (other seeds: robustness)
TILE_SELFTEST_SS = int(os.environ.get("SWAY_SS", "4"))   # selftest_tiles: samples per pixel each way. With 1 (point
                 # sampling, as before 2026-11-20) a tile's reading jumps with the sub-pixel phase: the twist read 0.283 for
                 # a true 0.348 mm at one frame and the wait came out 1 frame short; with 4 the twist frames over 0.15 mm
                 # read 1-10 % under the truth, and the wait is right (scratch, C42-W4)


def band_time(v, H, readout_s, bands=RS_BANDS):
    """the moment (after the frame's start) the band holding row v was exposed in render_rs (the truth uses the SAME
    moment as the drawing, 2026-11-20 C42-W4; before, the truth used (v + 0.5) / H, up to half a band off)."""
    edges = np.linspace(0, H, bands + 1).astype(int)
    j = int(np.searchsorted(edges, v, side="right")) - 1
    j = min(max(j, 0), bands - 1)
    return (j + 0.5) / bands * readout_s


def render_rs(sheet, K, cam_at, size, readout_s, t_frame, noise, seed, bands=RS_BANDS, ss=1):
    """a rolling-shutter picture: band j of rows is exposed at t_frame + (j + 0.5) / bands * readout_s; cam_at(t) -> (R, t).
    ss: samples per pixel each way, averaged (as k5_camera.render_sheet; ss = 1 point-samples the 300 DPI sheet)."""
    W, H = size
    A = np.array([[0, -1 / K5.PPM, K5.X_FAR], [1 / K5.PPM, 0, K5.Z_LEFT], [0, 0, 1]])
    img = np.zeros((H, W), np.uint8)
    edges = np.linspace(0, H, bands + 1).astype(int)
    for j in range(bands):
        v0, v1 = edges[j], edges[j + 1]
        Rc, tv = cam_at(t_frame + band_time(v0, H, readout_s, bands))
        P = K @ np.column_stack([Rc[:, 0], Rc[:, 2], tv])
        T = np.array([[1, 0, 0], [0, 1, -v0], [0, 0, 1.0]])
        if ss > 1:
            S_ = np.array([[ss, 0, (ss - 1) / 2], [0, ss, (ss - 1) / 2], [0, 0, 1.0]])
            big = cv2.warpPerspective(sheet, S_ @ T @ P @ A, (W * ss, (v1 - v0) * ss), flags=cv2.INTER_LINEAR,
                                      borderMode=cv2.BORDER_CONSTANT, borderValue=150)
            img[v0:v1] = cv2.resize(big, (W, v1 - v0), interpolation=cv2.INTER_AREA)
            continue
        img[v0:v1] = cv2.warpPerspective(sheet, T @ P @ A, (W, v1 - v0), flags=cv2.INTER_LINEAR,
                                         borderMode=cv2.BORDER_CONSTANT, borderValue=150)
    rng = np.random.default_rng(seed)
    img = np.clip(img.astype(np.float32) + rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return cv2.GaussianBlur(img, (3, 3), 0.6)


def synth_trial_rs(sheet, K, C, f_hz, zeta, amp_mm, amp_deg, seconds, fps, seed, readout_s, twist_deg=0.0, along="x", ss=1):
    """like synth_trial, with a rolling shutter (readout_s) and/or a twist of the pole (turn about the vertical, deg).
    along: the swing's direction on the table, "x" (= up-down in the picture: image up is +x) or "z" (= across the picture).
    Truth (2026-11-20 C42-W4): over a GRID in the sheet's box of the still picture (13 rows x 3 columns, the 4 corners
    among them), the true move of that table point at the moment its band of rows was drawn (band_time); per frame the
    largest over the grid (what the tiles should read; with a rolling shutter a middle row can move more than the corners:
    the box is ~7.5 ms of a 25 ms readout, ~74 deg of a 27.4 Hz swing, review C42-W4 3), and the slopes of a straight fit
    over the grid (turn, shear, stretch: what the tiles should read too)."""
    rng = np.random.default_rng(seed)
    phase = rng.uniform(0, 2 * math.pi)
    t0 = rng.uniform(0, 1 / fps)
    size = (1920, 1080)
    W, H = size
    Rc, t = K5.look_down(C)

    def cam_at(tt):
        e = math.exp(-zeta * 2 * math.pi * f_hz * tt) * math.cos(2 * math.pi * f_hz * math.sqrt(1 - zeta * zeta) * tt + phase)
        Cs = (C[0] + amp_mm * e, C[1], C[2]) if along == "x" else (C[0], C[1], C[2] + amp_mm * e)
        tilt = (0.0, amp_deg * e) if along == "x" else (amp_deg * e, 0.0)
        return K5.look_down(Cs, twist_deg * e, tilt)
    ref = K5.render_sheet(sheet, K, Rc, t, size, noise=2.0, seed=seed * 101, ss=ss)
    bu0, bv0, bu1, bv1 = sheet_box(corners(ref), ref.shape)
    uv = np.array([(u, v) for v in np.linspace(bv0, bv1 - 1, 13) for u in (bu0, (bu0 + bu1 - 1) / 2, bu1 - 1)])
    Kinv = np.linalg.inv(K)
    C0 = -Rc.T @ t
    pts_tab = []
    for uc, vc in uv:                       # grid point -> table point (still camera)
        ray = Rc.T @ Kinv @ np.array([uc, vc, 1.0])
        pts_tab.append(C0 + (0 - C0[1]) / ray[1] * ray)
    s_true = mm_per_px(corners(ref))
    frames, truth_max, truth_slopes = [], [], []
    for j, tt in enumerate(np.arange(t0, seconds, 1 / fps)):
        frames.append((float(tt), render_rs(sheet, K, cam_at, size, readout_s, float(tt), 2.0, seed * 101 + j + 1, ss=ss)))
        d = []
        for (uc, vc), P in zip(uv, pts_tab):
            Rj, tj = cam_at(float(tt) + band_time(vc, H, readout_s))
            q = K @ (Rj @ P + tj)
            d.append((q[0] / q[2] - uc, q[1] / q[2] - vc))
        d = np.array(d)
        truth_max.append((float(tt), float(np.hypot(d[:, 0], d[:, 1]).max()) * s_true))
        truth_slopes.append(fit_moves(uv, d, uv)[2])
    return {"ref": ref, "frames": frames}, truth_max, truth_slopes


def judge(wait, largest, true_wait_s, true_largest, fps):
    """the pass lines for a reading against the truth (2026-11-20 C42-W4, tighter than C42q): the wait is never SHORT,
    at most 2 frames LONG, and the largest move is within 10 % of the true one. Returns (ok, reasons it fails)."""
    why = []
    if wait is None:
        return False, ["no wait"]
    if wait < true_wait_s - 1e-9:
        why.append(f"wait short by {true_wait_s - wait:.3f} s")
    if wait > true_wait_s + 2.0 / fps + 1e-9:
        why.append(f"wait long by {wait - true_wait_s:.3f} s (over 2 frames)")
    if abs(largest - true_largest) > 0.1 * true_largest:
        why.append(f"largest {largest:.2f} mm vs true {true_largest:.2f} mm (over 10 %)")
    return not why, why


def synth_trial(sheet, K, C, f_hz, zeta, amp_mm, amp_deg, seconds, fps, seed, moved_at=None):
    rng = np.random.default_rng(seed)
    phase = rng.uniform(0, 2 * math.pi)
    t0 = rng.uniform(0, 1 / fps)                        # frames do not start at the stop
    size = (1920, 1080)
    Rc, t = K5.look_down(C)
    if moved_at is not None:      # the still picture is taken after --settle, so it shows the moved place (as on the real stand)
        Rm, tm = K5.look_down((C[0], C[1], C[2] + 1.0))
        ref = K5.render_sheet(sheet, K, Rm, tm, size, noise=2.0, seed=seed * 101)
    else:
        ref = K5.render_sheet(sheet, K, Rc, t, size, noise=2.0, seed=seed * 101)
    P = np.vstack([K5.marker_corners(i) for i in K5.MARKERS])

    def proj(R, tv):
        q = (K @ (R @ P.T + np.asarray(tv).reshape(3, 1))).T
        return q[:, :2] / q[:, 2:3]
    p0 = proj(Rc, t)
    s_true = K5.MARKER_MM / float(np.mean([np.linalg.norm(p0[4 * i + k] - p0[4 * i + (k + 1) % 4]) for i in range(8) for k in range(4)]))
    frames, truth = [], []
    for j, tt in enumerate(np.arange(t0, seconds, 1 / fps)):
        dx, dg = sway_pose(tt, amp_mm, amp_deg, f_hz, zeta, phase)
        Cj = (C[0] + dx, C[1], C[2])
        if moved_at is not None and tt >= moved_at:
            Cj = (Cj[0], Cj[1], Cj[2] + 1.0)            # the camera (or sheet) moved 1 mm for good
        Rj, tj = K5.look_down(Cj, 0.0, (0.0, dg))
        frames.append((float(tt), K5.render_sheet(sheet, K, Rj, tj, size, noise=2.0, seed=seed * 101 + j + 1)))
        # true shift: the mean move of the 32 corners projected without noise by both cameras, in mm (same scale rule)
        truth.append((float(tt), float(np.linalg.norm((proj(Rj, tj) - p0).mean(axis=0))) * s_true))
    return {"ref": ref, "frames": frames}, truth


def true_wait(truth_all, tol):
    """the last moment (any trial) the true swing of the picture centre is over tol."""
    return max((t for tr in truth_all for t, m in tr if m > tol), default=0.0)


def cmd_selftest(_a):
    fails = 0
    W, H, fov_v = 1920, 1080, 60.0
    f = H / 2 / math.tan(math.radians(fov_v / 2))
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1]])
    sheet = K5.make_sheet()
    C = (157.3, 540.0, 19.5)          # the sim's C lens tip over the sheet (sim/so101.js OVERHEAD_C)
    fps, seconds = 30.0, 1.6
    print(f"== synthetic top pictures (1920x1080, vertical FOV 60, pixel noise 2), camera swinging at 27.4 Hz, sampled at {fps:.0f}/s ==")
    for zeta in (0.05, 0.02):
        trials, truth = [], []
        for s in range(6):
            tr, tt = synth_trial(sheet, K, C, 27.4, zeta, 0.8, 0.15, seconds, fps, seed=s + int(zeta * 1000))
            trials.append(tr); truth.append(tt)
        r = analyze_trials(trials, TOL_MM, BIN_S)
        tw = true_wait(truth, TOL_MM)
        ok = r["wait_s"] is not None and not r["problems"] and abs(r["wait_s"] - tw) <= 1.0 / fps + 1e-9
        fails += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} damping {zeta}: wait from the pictures {r['wait_s'] if r['wait_s'] is None else round(r['wait_s'], 3)} s, "
              f"from the true swing at the same frames {tw:.3f} s (pass: within one frame)")
        print("        " + "  ".join(f"{row['from_s']:.1f}:{row['max_mm']:.2f}" for row in r["table"][:8]) + "  (s: mm)")
        # the measured shift against the true one, frame by frame (does the phase correlation read the sway right?)
        errs = []
        for tr, tt in zip(trials, truth):
            s = mm_per_px(corners(tr["ref"]))
            win = cv2.createHanningWindow((tr["ref"].shape[1], tr["ref"].shape[0]), cv2.CV_32F)
            for (t, g), (_, m) in zip(tr["frames"], tt):
                errs.append(abs(shift(tr["ref"], g, s, win)["mm"] - m))
        e95 = float(np.percentile(errs, 95))
        ok = e95 < 0.1
        fails += not ok
        print(f"  {'ok  ' if ok else 'FAIL'}   measured vs true shift per frame: 95th percentile {e95:.3f} mm (pass < 0.1)")
    # negative 1: no damping -> the end of the burst is still off -> no wait given
    trials = [synth_trial(sheet, K, C, 27.4, 0.0005, 0.8, 0.15, seconds, fps, seed=40 + s)[0] for s in range(3)]
    r = analyze_trials(trials, TOL_MM, BIN_S)
    ok = r["wait_s"] is None and len(r["problems"]) == 3 and all("STILL SWINGING" in p for p in r["problems"])
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} negative: a sway that does not die out -> no wait, 'still swinging' in "
          f"{sum('STILL SWINGING' in p for p in r['problems'])} of 3 trials")
    # negative 2: moved for good half way -> named
    trials = [synth_trial(sheet, K, C, 27.4, 0.05, 0.8, 0.15, seconds, fps, seed=60 + s, moved_at=0.8)[0] for s in range(3)]
    r = analyze_trials(trials, TOL_MM, BIN_S)
    ok = r["wait_s"] is None and len(r["problems"]) == 3 and all("MOVED FOR GOOD" in p for p in r["problems"])
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} negative: the camera moved 1 mm for good at 0.8 s (still picture after the move) -> "
          f"no wait, 'moved for good' in {sum('MOVED FOR GOOD' in p for p in r['problems'])} of 3 trials")
    fails += selftest_tiles(sheet, K, C, fps, seconds)
    print("SELFTEST", "OK" if not fails else f"FAILED ({fails})")
    sys.exit(1 if fails else 0)


def selftest_tiles(sheet, K, C, fps, seconds):
    """rolling shutter (25 ms readout, GUESS) with a swing up-down / across the picture, and a twist of the pole. The truth
    is the move at the 4 corners of the sheet's box at the moment their band was drawn (synth_trial_rs). Pass (judge): the
    tiles' wait never short, at most 2 frames long, their largest move within 10 %. The slopes (turn, shear, stretch) read
    against the true ones (largest over the frames after t = 0). Negative: the whole-picture shift must FAIL the same
    lines in every case, else the case does not show what the tiles are for. The slopes are compared as largest |value|
    (a wrong sign would not show). SWAY_SEED0 / SWAY_SS change the seeds / the samples per pixel (robustness runs)."""
    fails = 0
    cases = (("rolling shutter 25 ms, swing up-down the picture (along x) 0.8 mm + 0.15 deg", 0.025, 0.8, 0.15, 0.0, "x", "stretch", None),
             ("rolling shutter 25 ms, swing across the picture (along z) 0.8 mm + 0.15 deg", 0.025, 0.8, 0.15, 0.0, "z", "shear", None),
             ("twist of the pole 0.3 deg (no swing), no rolling shutter", 0.0, 0.0, 0.0, 0.3, "x", "turn", "must fail"))
    for name, ro, amp_mm, amp_deg, tw, along, slope, whole_rule in cases:
        trials, tmax, tsl = [], [], []
        for s_ in range(3):
            tr, a_, b_ = synth_trial_rs(sheet, K, C, 27.4, 0.05, amp_mm, amp_deg, seconds, fps, seed=TILE_SEED0 + s_,
                                        readout_s=ro, twist_deg=tw, along=along, ss=TILE_SELFTEST_SS)
            trials.append(tr); tmax.append(a_); tsl.append(b_)
        r = analyze_trials(trials, TOL_MM, BIN_S)
        tw_max = true_wait(tmax, TOL_MM)
        tmx = max(m for tr in tmax for _, m in tr)
        if r["wait_s"] is None or r["problems"]:
            fails += 1
            print(f"  FAIL {name}: no wait ({r['problems']})")
            continue
        rmax = max(p["tiles_max_mm"] for p in r["trials"])
        wmax = max(p["max_mm"] for p in r["trials"])
        ok_t, why_t = judge(r["wait_tiles_s"], rmax, tw_max, tmx, fps)
        ok_w, why_w = judge(r["wait_s"], wmax, tw_max, tmx, fps)
        # the slopes: largest |value| read over the trials against the true largest (radians -> deg for the turn)
        key = {"turn": "turn_max_deg", "shear": "shear_max_px_per_100rows", "stretch": "stretch_max_px_per_100rows"}
        read = {k: max(p[v] for p in r["trials"]) for k, v in key.items()}
        true = {k: max(abs(sl[k]) for tr in tsl for sl in tr) * (180 / math.pi if k == "turn" else 100) for k in key}
        ok_s = abs(read[slope] - true[slope]) <= 0.2 * true[slope]
        # the whole picture must FAIL the same lines in every case (review C42q 2-4). On the twist that is bound to happen
        # (a pure turn has no mean shift); with the rolling shutter it fails on the largest move (it reads a mean)
        ok = ok_t and ok_s and not ok_w
        fails += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {name}:")
        print(f"        tiles: wait {r['wait_tiles_s']:.3f} s, largest {rmax:.3f} mm ({(rmax / tmx - 1) * 100:+.1f} %) -> {'pass' if ok_t else 'FAIL ' + '; '.join(why_t)}"
              f"   (truth over the box (13 x 3 points): wait {tw_max:.3f} s, largest {tmx:.3f} mm; pass: not short, at most 2 frames long, largest within 10 %)")
        print(f"        whole picture: wait {r['wait_s']:.3f} s, largest {wmax:.3f} mm ({(wmax / tmx - 1) * 100:+.1f} %) -> {'pass' if ok_w else 'fails: ' + '; '.join(why_w)}"
              + "   (NEGATIVE: must fail" + (", bound to on a pure turn)" if whole_rule else ")"))
        print(f"        {slope} read {read[slope]:.3f}, true {true[slope]:.3f} {'deg' if slope == 'turn' else 'px/100 rows'} "
              f"(pass: within 20 %). All read/true: " + ", ".join(f"{k} {read[k]:.3f}/{true[k]:.3f}" for k in key))
    return fails


def cmd_selftest_tiles(_a):
    W, H, fov_v = 1920, 1080, 60.0
    f = H / 2 / math.tan(math.radians(fov_v / 2))
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1]])
    fails = selftest_tiles(K5.make_sheet(), K, (157.3, 540.0, 19.5), 30.0, 1.6)
    print("SELFTEST (tiles only)", "OK" if not fails else f"FAILED ({fails})")
    sys.exit(1 if fails else 0)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    s = p.add_subparsers(dest="cmd", required=True)
    q = s.add_parser("burst"); q.add_argument("index", type=int); q.add_argument("dir")
    q.add_argument("--seconds", type=float, default=2.0); q.add_argument("--settle", type=float, default=3.0)
    q.add_argument("--width", type=int, default=1920); q.add_argument("--height", type=int, default=1080)
    q.add_argument("--fps", type=int, default=30); q.add_argument("--fourcc", default="MJPG")
    q.add_argument("--exposure", type=float, default=-11, help="DSHOW exposure (log2 seconds: -11 = about 0.5 ms, -8 = 4 ms)")
    q = s.add_parser("analyze"); q.add_argument("dir"); q.add_argument("--tol-mm", type=float, default=TOL_MM)
    q.add_argument("--bin", type=float, default=BIN_S)
    s.add_parser("selftest")
    s.add_parser("selftest-tiles")
    a = p.parse_args()
    {"burst": cmd_burst, "analyze": cmd_analyze, "selftest": cmd_selftest, "selftest-tiles": cmd_selftest_tiles}[a.cmd](a)


if __name__ == "__main__":
    main()
