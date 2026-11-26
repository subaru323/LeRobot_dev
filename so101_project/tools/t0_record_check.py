"""T0 (docs/plan/総合計画書_レビュー用.md §6): check a LeRobot recording with the main 0.6.2 read path.

What a lerobot-record dataset of this project holds (SO-101 follower, cameras "front" and "wrist"):
  observation.state (6 joints, *.pos), action (6, the leader's command), observation.images.front / .wrist (video),
  timestamp, frame_index, episode_index, index, task_index; meta/info.json and meta/episodes/*.parquet.

Note (read in 0.6.2, datasets/dataset_writer.py add_frame, line 224): "timestamp" is NOT a measured time. It is written
as frame_index / fps. So a late camera, a slow loop or a stuck reading never shows in the timestamp column. This tool
looks for them in the signals instead (image change against joint motion, command against state).

Usage (the python of MAIN_PG/lerobot/.venv):
  python tools\\t0_record_check.py check ROOT [--json OUT]        a dataset folder (the one with meta/ data/ videos/)
  python tools\\t0_record_check.py mock ROOT --fault NAME [--seed N]   write a mock recording, with one fault in it
  python tools\\t0_record_check.py suite [--out outputs/t0] [--seed N] [--reuse]  every fault: mock, check, compare

Checks (level fail = the file is wrong; warn = a sign to look at, judged from the signals):
  features          the expected keys, shapes and joint names                                   fail
  rows_total        rows in data/ == info.json total_frames                                      fail
  index             the global index is 0..N-1                                                   fail
  episode_boundary  each episode's rows match meta/episodes (count, dataset_from/to_index)       fail
  frame_index       frame_index is 0..length-1 in each episode                                   fail
  timestamp         timestamp == frame_index / fps (0.6.2 writes it so)                          fail
  video_span        each camera's from/to_timestamp spans the episode length                     fail
  joint_values      no NaN/inf; gripper in 0..100; body joints within +-200                     fail
  load              LeRobotDataset(root=...) opens (the 0.6.2 read path, pyav decoder)           fail
  decode            every row gives both camera frames                                           fail
  loader_mismatch   state/action from the loader == the parquet                                  fail
  stale_camera      a picture changes < 15% of what this joint motion makes it change in the other episodes, for
                    >= 3 frames (model: change ~ still + sum w_j |dq_j|, two rows together; judged only where the
                    expected change is >= 2 x still, so a camera that does not see the moving joint is not judged)  warn
  state_hold        all 6 joint readings stay exactly the same for >= 5 frames while the arm command moves  warn
  time_jump         state, command and a camera jump together, against the rows around (frames lost)  warn
  action_after_state  the state changes before the command (the columns are shifted)              warn
  lag_inconsistent  an episode's lag (command->state, state->camera) is >= 3 frames off the others' median  warn
  camera_lag_large  a camera is > 5 frames (167 ms at 30 fps) behind the joints                  warn
The warn thresholds are set on the mock (see `suite`); on real data they are a first guess (【仮】).
A lag is judged only when its correlation is >= 0.5: it needs stops and starts in the motion (a smooth endless motion
looks alike at every shift). The mock moves like teleop: holds, then minimum-jerk moves.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
from pathlib import Path

os.environ.setdefault("HF_HUB_OFFLINE", "1")   # never reach the Hub; the dataset is local

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
FPS = 30
MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
CAMS = ["front", "wrist"]
H, W = 480, 640                     # the cameras of docs/データ収集と学習の手順書.md (640x480, 30 fps)
N_EP = 3
N_FRAMES = 120                      # 4 s per mock episode
LAG = 2                             # mock: the follower's state follows the command 2 frames late
TICK_DEG = 360 / 4096               # mock: STS3215 reads 4096 steps a turn
GRIP_TICK = 0.1                     # mock: the gripper's 0..100 in steps of 0.1 【仮】
JITTER_TICKS = 0.4                  # mock: reading jitter (steps, 1 sigma)
DEADBAND_TICKS = 2                  # mock: the follower does not move for an error of 2 steps or less
CUBE_GRIP = 18.0                    # mock: the follower's gripper stops here (on the cube) while the command closes more
FAULT_EP = 1                        # the mock puts its fault in this episode
SEG = (40, 58)                      # ... over this many frames (18), placed where the arm moves (_busiest)

# thresholds of the signal checks (【仮】 set on the mock)
STALE_RUN = 3
HOLD_RUN = 5
MOVE_DEG = 1.0                      # arm motion per frame that counts as "moving" (degrees, summed over 5 joints)
STALE_RATIO = 0.15                  # picture change below this share of the expected change = not changing
STALE_MIN_X = 2.0                   # ... judged only where the expected change is this many times the still change
QUIET_DEG = 1.0                     # 5-joint motion below this (deg per row, two rows together) = the arm is still
JUMP_K = 4.0                        # a jump is K x the change of the rows around it ...
JUMP_FLOOR_DEG = 3.0                # ... and at least this much joint motion (degrees, summed)
LAG_MAX = 12
LAG_OFF = 3
CAM_LAG_MAX = 5
LAG_MIN_CORR = 0.5                  # a lag with a weaker correlation is not judged (smooth motion with no stop/start)

FAULTS = ["clean", "cam_lag_front", "stale_front", "state_hold", "frame_skip", "action_shift",
          "drop_rows", "bad_timestamp", "episode_boundary"]
# what `suite` expects: (check, episode) that must be flagged, and the episodes the fault touches
EXPECT = {
    "clean":            (set(), set()),
    "cam_lag_front":    ({("camera_lag_large", 1)}, {1}),
    "stale_front":      ({("stale_camera", 1)}, {1}),
    "state_hold":       ({("state_hold", 1)}, {1}),
    "frame_skip":       ({("time_jump", 1)}, {1}),
    "action_shift":     ({("action_after_state", 1)}, {1}),
    "drop_rows":        ({("frame_index", 1), ("episode_boundary", 1)}, {1, None}),
    "bad_timestamp":    ({("timestamp", 1)}, {1}),
    "episode_boundary": ({("episode_boundary", 0)}, {0, 1, None}),
}


# ---------------------------------------------------------------------------------------------- mock recording
def features_0_6_2() -> dict:
    """The features lerobot-record makes for so101_follower with cameras front and wrist (0.6.2)."""
    from lerobot.utils.feature_utils import hw_to_dataset_features
    hw = {f"{m}.pos": float for m in MOTORS}
    cams = {c: (H, W, 3) for c in CAMS}
    return {**hw_to_dataset_features(hw, "action", True), **hw_to_dataset_features({**hw, **cams}, "observation", True)}


def _texture(rng, h, w, blur=15):
    import cv2
    t = rng.normal(128, 40, (h // 4, w // 4)).astype(np.float32)
    t = cv2.GaussianBlur(t, (0, 0), blur / 4)
    t = cv2.resize(t, (w, h), interpolation=cv2.INTER_CUBIC)
    t = (t - t.mean()) / (t.std() + 1e-6) * 30 + 120
    return np.clip(np.stack([t, t * 0.95 + 8, t * 0.9 + 15], -1), 0, 255).astype(np.float32)


class Scene:
    """Draws the two camera pictures from a joint state (degrees; gripper 0..100). Only for the mock."""

    def __init__(self, rng):
        self.rng = rng
        self.bg_front = _texture(rng, H, W)
        self.bg_wrist = _texture(rng, H * 2, W * 2, blur=25)

    def front(self, s):
        import cv2
        img = self.bg_front.copy()
        cv2.rectangle(img, (430, 380), (480, 430), (40, 40, 200), -1)              # the red cube, still
        x, y = 300 + 2.0 * s[0], 430.0
        ang = np.deg2rad(s[1])
        pts = [(x, y)]
        for length, a in ((150, s[1]), (130, s[1] + s[2]), (60, s[1] + s[2] + s[3])):
            ang = np.deg2rad(a)
            x, y = x + length * np.sin(ang), y - length * np.cos(ang)
            pts.append((x, y))
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            cv2.line(img, (int(x0), int(y0)), (int(x1), int(y1)), (230, 230, 230), 14)
        gap = 6 + 0.3 * s[5]
        xe, ye = pts[-1]
        r = np.deg2rad(s[4])
        dx, dy = gap * np.cos(r), gap * np.sin(r)
        for sg in (-1, 1):
            cv2.circle(img, (int(xe + sg * dx), int(ye + sg * dy)), 7, (30, 160, 230), -1)
        img += self.rng.normal(0, 3, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)

    def wrist(self, s):
        import cv2
        cx = W + 6.0 * s[0] + 3.0 * s[3]
        cy = H + 4.0 * s[1] + 3.0 * s[2]
        m = cv2.getRotationMatrix2D((float(cx), float(cy)), float(s[4]), 1.0)
        m[0, 2] += W / 2 - cx
        m[1, 2] += H / 2 - cy
        img = cv2.warpAffine(self.bg_wrist, m, (W, H), borderMode=cv2.BORDER_REFLECT)
        gap = int(40 + 2.0 * s[5])
        for sg in (-1, 1):
            x0 = W // 2 + sg * gap
            cv2.rectangle(img, (min(x0, x0 + sg * 60), 400), (max(x0, x0 + sg * 60), H), (60, 60, 60), -1)
        img += self.rng.normal(0, 3, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)


def _trajectory(rng, n):
    """Teleop-like command (degrees; gripper 0..100): holds of 0.2-0.5 s, then minimum-jerk moves of four kinds:
    all joints (0.5-1.0 s), the gripper only, the wrist roll only, and one other joint small and slow (10 deg, 0.8-1.5 s)."""
    rest = np.array([0, -20, 30, 40, 0, 30], float)
    amp = np.array([35, 25, 30, 25, 40, 35], float)
    out, cur = [], rest.copy()
    while len(out) < n:
        out += [cur.copy()] * int(rng.uniform(0.2, 0.5) * FPS)
        kind = rng.choice(["all", "gripper", "roll", "single"], p=[0.4, 0.2, 0.15, 0.25])
        nxt = cur.copy()
        if kind == "all":
            nxt, dur = rest + amp * rng.uniform(-1, 1, 6), rng.uniform(0.5, 1.0)
        elif kind == "gripper":
            nxt[5], dur = rng.uniform(0, 100), rng.uniform(0.3, 0.6)
        elif kind == "roll":
            nxt[4], dur = rest[4] + amp[4] * rng.uniform(-1, 1), rng.uniform(0.4, 0.8)
        else:
            j = int(rng.integers(0, 4))
            nxt[j], dur = cur[j] + rng.choice([-1, 1]) * 10, rng.uniform(0.8, 1.5)
        nxt[5] = np.clip(nxt[5], 0, 100)
        tau = np.arange(1, int(dur * FPS) + 1)
        tau = tau / tau[-1]
        out += list(cur + (nxt - cur) * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)[:, None])
        cur = nxt
    return np.array(out[:n])


def _quantize(x):
    q = np.array(x, float)
    q[..., :5] = np.round(q[..., :5] / TICK_DEG) * TICK_DEG
    q[..., 5] = np.round(q[..., 5] / GRIP_TICK) * GRIP_TICK
    return q


def _read(x, rng):
    """A motor reading: about one step of jitter, then the steps."""
    j = rng.normal(0, JITTER_TICKS, x.shape)
    return _quantize(x + j * np.r_[[TICK_DEG] * 5, GRIP_TICK])


def _follower(src):
    """Where the follower really is at each row: it goes to the command of LAG rows before (src[t] = command of
    row t - LAG), does not move while the error is inside the deadband, and its gripper stops on the cube."""
    pos = np.empty_like(src)
    pos[0] = src[0]
    pos[0, 5] = max(pos[0, 5], CUBE_GRIP)
    db = DEADBAND_TICKS * np.r_[[TICK_DEG] * 5, GRIP_TICK]
    for t in range(1, len(src)):
        tgt = src[t].copy()
        tgt[5] = max(tgt[5], CUBE_GRIP)
        pos[t] = np.where(np.abs(tgt - pos[t - 1]) > db, tgt, pos[t - 1])
    return pos


def _busiest(a, length, lo=20, hi=90):
    """Start row in lo..hi where the command moves most over `length` rows (so a fault there is in a move)."""
    v = np.r_[0, np.abs(np.diff(a[:, :5], axis=0)).sum(1)]
    return max(range(lo, hi), key=lambda k: v[k:k + length].sum())


def make_mock(root: Path, fault: str, seed: int = 1) -> None:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    if fault not in FAULTS:
        raise SystemExit(f"unknown fault {fault!r}; one of {FAULTS}")
    rng = np.random.default_rng(seed)
    scene = Scene(rng)
    if root.exists():
        shutil.rmtree(root)
    ds = LeRobotDataset.create(repo_id=f"local/t0_{fault}", fps=FPS, features=features_0_6_2(), root=root,
                               robot_type="so101_follower", use_videos=True, video_backend="pyav")
    extra = 10                                                                 # source frames for frame_skip
    for e in range(N_EP):
        src = _trajectory(rng, N_FRAMES + extra + LAG)
        act = _read(src[LAG:], rng)                                             # the leader's reading at row t
        true_pos = _follower(src[:-LAG])                                        # the follower, LAG rows behind
        state = _read(true_pos, rng)
        rows = np.arange(N_FRAMES)
        front_from = true_pos.copy()                                            # the pictures show the real arm
        act_rec = act.copy()
        f = fault if e == FAULT_EP else "clean"
        k0 = _busiest(act[:N_FRAMES], 13 if f == "state_hold" else (SEG[1] - SEG[0]))
        if f == "frame_skip":                                                   # the loop lost 10 frames in a move
            k0 = _busiest(act[:N_FRAMES], extra)
            rows = np.r_[np.arange(k0), np.arange(k0 + extra, N_FRAMES + extra)]
        if f == "cam_lag_front":                                                # front shows the arm 6 frames late
            front_from = np.vstack([np.repeat(true_pos[:1], 6, 0), true_pos[:-6]])
        if f == "action_shift":                                                 # command column 6 frames late
            act_rec = np.vstack([np.repeat(act[:1], 6, 0), act[:-6]])
        if f == "state_hold":
            state[k0:k0 + 13] = state[k0 - 1]                                   # the reading stuck; the picture is real
        stale = None
        for k, r in enumerate(rows):
            fr = scene.front(front_from[r])
            if f == "stale_front" and k0 <= k < k0 + SEG[1] - SEG[0]:
                fr = stale if stale is not None else fr
            if f == "stale_front" and k == k0 - 1:
                stale = fr
            ds.add_frame({
                "action": act_rec[r].astype(np.float32),
                "observation.state": state[r].astype(np.float32),
                "observation.images.front": fr,
                "observation.images.wrist": scene.wrist(true_pos[r]),
                "task": "t0 mock: move the arm",
            })
        ds.save_episode()
    ds.finalize()
    _file_fault(root, fault)


def _data_files(root: Path):
    return sorted((root / "data").glob("chunk-*/file-*.parquet"))


def _file_fault(root: Path, fault: str) -> None:
    """Faults made by editing the written files (a broken copy, a bad merge)."""
    if fault not in ("drop_rows", "bad_timestamp", "episode_boundary"):
        return
    import pyarrow as pa
    import pyarrow.parquet as pq
    for p in _data_files(root):
        tab = pq.read_table(p)
        df = tab.to_pandas()
        ep, fi = df["episode_index"].to_numpy(), df["frame_index"].to_numpy()
        if fault == "drop_rows":                                               # 5 rows of episode 1 are gone
            df = df[~((ep == FAULT_EP) & (fi >= 50) & (fi < 55))]
        elif fault == "bad_timestamp":                                         # time edited from frame 60 on
            m = (ep == FAULT_EP) & (fi >= 60)
            df.loc[m, "timestamp"] = df.loc[m, "timestamp"] + 0.1
        elif fault == "episode_boundary":                                      # last 3 rows of episode 0 taken as 1
            last = fi[ep == 0].max()
            df.loc[(ep == 0) & (fi > last - 3), "episode_index"] = 1
        out = pa.Table.from_pandas(df.reset_index(drop=True), schema=tab.schema, preserve_index=False)
        pq.write_table(out, p)


# ---------------------------------------------------------------------------------------------- the check
class Report:
    def __init__(self):
        self.items: list[dict] = []
        self.info: dict = {}

    def add(self, name, level, episode, detail, frames=None):
        self.items.append({"check": name, "level": level, "episode": episode, "frames": frames, "detail": detail})

    def flagged(self):
        return {(i["check"], i["episode"]) for i in self.items}


def _runs(mask, min_len):
    out, start = [], None
    for i, v in enumerate(list(mask) + [False]):
        if v and start is None:
            start = i
        if not v and start is not None:
            if i - start >= min_len:
                out.append([start, i - 1])
            start = None
    return out


def _nnls(a, y):
    """Least squares with every weight >= 0, approximately (drop the most negative column until none is negative;
    not the exact Lawson-Hanson solution)."""
    keep = list(range(a.shape[1]))
    while keep:
        w = np.linalg.lstsq(a[:, keep], y, rcond=None)[0]
        if (w >= 0).all():
            out = np.zeros(a.shape[1])
            out[keep] = w
            return out
        keep.pop(int(np.argmin(w)))
    return np.zeros(a.shape[1])


def _pair(x):
    """Mean of each row and the row before. AV1 (g=2) makes every other frame a copy, so the picture change
    alternates (0 / 0.47 at rest in the mock); two rows together are steady."""
    x = np.asarray(x, float)
    return np.r_[x[:1], (x[1:] + x[:-1]) / 2] if len(x) else x


def _motion(s):
    """|change| of each joint per row, two rows together (n-1 rows x 6)."""
    return _pair(np.abs(np.diff(s, axis=0)))


def _picture_change(f):
    """Mean |change| of a gray picture per row, two rows together (n-1 rows)."""
    return _pair(np.abs(np.diff(f, axis=0)).mean((1, 2)))


def _picture_model(states, frames):
    """How much a camera's picture changes for each joint's motion: change - still ~ sum_j w_j |dq_j| (w >= 0).
    Fitted twice, the second time without rows far below the first fit (a stuck camera must not teach the model).
    check() fits it on the other episodes, never on the one it judges. Returns (w, still)."""
    dq = np.vstack([_motion(s) for s in states])
    mc = np.concatenate([_picture_change(f) for f in frames])
    quiet = dq[:, :5].sum(1) < QUIET_DEG
    still = float(np.median(mc[quiet])) if quiet.sum() >= 5 else float(np.percentile(mc, 10))
    y = mc - still
    w = _nnls(dq, y)
    ok = mc >= 0.3 * (still + dq @ w)
    if ok.sum() > 10:
        w = _nnls(dq[ok], y[ok])
    return w, still


def stale_runs(s, f, model):
    """Runs (>= STALE_RUN rows) where the picture changes < STALE_RATIO of what this joint motion should make it change,
    judged only where that expected change is >= STALE_MIN_X x the still change. Returns [(r0, r1, change, expected)]."""
    w, still = model
    mc, m_exp = _picture_change(f), _motion(s) @ w
    mask = (mc < STALE_RATIO * (still + m_exp)) & (m_exp > STALE_MIN_X * max(still, 0.02))
    return [(r0, r1, float(mc[r0:r1 + 1].mean()), float((still + m_exp[r0:r1 + 1]).mean()))
            for r0, r1 in _runs(mask, STALE_RUN)]


def _around(v, r, k=3):
    return float(np.median(np.r_[v[max(0, r - k):r], v[r + 1:r + 1 + k]]))


def _spikes(v, floor):
    """Rows where v is JUMP_K x the median of the 3 rows on each side, and above floor."""
    return np.array([v[r] > floor and v[r] > JUMP_K * max(_around(v, r), 1e-3) for r in range(len(v))], bool)


def _lag(a, b, max_lag=LAG_MAX):
    """Lag (frames) of b behind a, by the best correlation of the two series (a, b: (n,) or (n,k))."""
    a = np.atleast_2d(np.asarray(a, float).T).T
    b = np.atleast_2d(np.asarray(b, float).T).T
    a = a - a.mean(0)
    b = b - b.mean(0)
    best, best_c = 0, -np.inf
    n = len(a)
    for L in range(-max_lag, max_lag + 1):
        if L >= 0:
            x, y = a[: n - L], b[L:]
        else:
            x, y = a[-L:], b[: n + L]
        den = np.sqrt((x * x).sum() * (y * y).sum()) + 1e-12
        c = (x * y).sum() / den
        if c > best_c:
            best, best_c = L, c
    return best, float(best_c)


def read_episodes(root: Path):
    """Every row through LeRobotDataset (0.6.2, pyav): state, action, and both pictures as 80x60 gray."""
    import cv2
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset(repo_id="local/t0_check", root=root, video_backend="pyav")
    per_ep: dict[int, dict] = {}
    decode_bad: dict[int, list] = {}
    t0 = time.time()
    epi = ds.hf_dataset["episode_index"]
    for i in range(len(ds)):
        try:
            it = ds[i]
        except Exception as ex:                                                # noqa: BLE001
            e = int(epi[i]) if i < len(epi) else None                         # None: a row the files do not have
            decode_bad.setdefault(e, []).append((i, f"{type(ex).__name__}: {str(ex)[:120]}"))
            continue
        e = int(it["episode_index"])
        p = per_ep.setdefault(e, {"state": [], "action": [], "fi": [], **{c: [] for c in CAMS}})
        p["state"].append(it["observation.state"].numpy())
        p["action"].append(it["action"].numpy())
        p["fi"].append(int(it["frame_index"]))
        for c in CAMS:
            im = it[f"observation.images.{c}"].numpy()                         # (3, H, W) float 0..1
            g = (im.mean(0) * 255).astype(np.float32)
            p[c].append(cv2.resize(g, (80, 60), interpolation=cv2.INTER_AREA))
    return per_ep, decode_bad, round(time.time() - t0, 1)


def check(root: Path) -> Report:
    import pandas as pd
    import pyarrow.parquet as pq
    rep = Report()
    info = json.loads((root / "meta" / "info.json").read_text(encoding="utf-8"))
    fps = info["fps"]
    rep.info.update(fps=fps, codebase_version=info.get("codebase_version"), total_frames=info.get("total_frames"),
                    total_episodes=info.get("total_episodes"), robot_type=info.get("robot_type"))

    # --- features
    feats = info["features"]
    names = [f"{m}.pos" for m in MOTORS]
    for key in ("observation.state", "action"):
        ft = feats.get(key)
        if ft is None or list(ft.get("shape", [])) != [6] or list(ft.get("names") or []) != names:
            rep.add("features", "fail", None, f"{key}: {ft and (ft.get('shape'), ft.get('names'))}, want [6] {names}")
    for c in CAMS:
        ft = feats.get(f"observation.images.{c}")
        if ft is None or ft.get("dtype") != "video":
            rep.add("features", "fail", None, f"observation.images.{c}: {ft and ft.get('dtype')}, want video")

    # --- files
    data = pd.concat([pq.read_table(p).to_pandas() for p in _data_files(root)], ignore_index=True)
    epm = pd.concat([pq.read_table(p).to_pandas() for p in sorted((root / "meta" / "episodes").glob("chunk-*/file-*.parquet"))],
                    ignore_index=True)
    if len(data) != info["total_frames"]:
        rep.add("rows_total", "fail", None, f"{len(data)} rows in data/, info.json says {info['total_frames']}")
    idx = data["index"].to_numpy()
    if not np.array_equal(np.sort(idx), np.arange(len(idx))) or not np.array_equal(idx, np.sort(idx)):
        miss = sorted(set(range(int(idx.max()) + 1)) - set(idx.tolist()))[:10]
        rep.add("index", "fail", None, f"index is not 0..{len(idx) - 1} in order (missing e.g. {miss})")
    known = set(epm["episode_index"].tolist())
    for e in sorted(set(data["episode_index"].tolist()) - known):
        rep.add("episode_boundary", "fail", int(e), "rows of an episode that meta/episodes does not have")
    for _, row in epm.iterrows():
        e = int(row["episode_index"])
        d = data[data["episode_index"] == e]
        n, a, b = int(row["length"]), int(row["dataset_from_index"]), int(row["dataset_to_index"])
        got = d["index"].to_numpy()
        if len(d) != n or (len(d) and (got.min() != a or got.max() != b - 1)):
            rep.add("episode_boundary", "fail", e,
                    f"{len(d)} rows, index {got.min() if len(d) else '-'}..{got.max() if len(d) else '-'}; "
                    f"meta says length {n}, index {a}..{b - 1}")
        fi = d["frame_index"].to_numpy()
        if not np.array_equal(fi, np.arange(len(fi))):
            bad = np.nonzero(fi != np.arange(len(fi)))[0]
            rep.add("frame_index", "fail", e, f"frame_index not 0..{len(fi) - 1} (first off at row {bad[0]}: {fi[bad[0]]})",
                    [int(bad[0]), int(bad[-1])])
        ts = d["timestamp"].to_numpy()
        off = np.abs(ts - fi / fps)
        if len(off) and off.max() > 1e-4:
            bad = np.nonzero(off > 1e-4)[0]
            rep.add("timestamp", "fail", e, f"timestamp != frame_index/fps at {len(bad)} rows (max {off.max():.4f} s)",
                    [int(fi[bad[0]]), int(fi[bad[-1]])])
        for c in CAMS:
            k0, k1 = f"videos/observation.images.{c}/from_timestamp", f"videos/observation.images.{c}/to_timestamp"
            if k0 in row and k1 in row:
                span = (float(row[k1]) - float(row[k0])) * fps
                if abs(span - n) > 1.01:
                    rep.add("video_span", "fail", e, f"{c}: video spans {span:.1f} frames, episode length {n}")
    st = np.stack(data["observation.state"].to_numpy())
    ac = np.stack(data["action"].to_numpy())
    for nm, v in (("observation.state", st), ("action", ac)):
        if not np.isfinite(v).all():
            rep.add("joint_values", "fail", None, f"{nm}: NaN or inf")
        elif (v[:, 5] < -0.5).any() or (v[:, 5] > 100.5).any() or (np.abs(v[:, :5]) > 200).any():
            rep.add("joint_values", "fail", None, f"{nm}: out of range (gripper {v[:, 5].min():.1f}..{v[:, 5].max():.1f}, "
                                                 f"body max |{np.abs(v[:, :5]).max():.1f}|)")

    # --- the 0.6.2 read path
    try:
        per_ep, decode_bad, secs = read_episodes(root)
    except Exception as ex:                                                    # noqa: BLE001 - report, do not crash
        rep.add("load", "fail", None, f"LeRobotDataset could not open it: {type(ex).__name__}: {str(ex)[:300]}")
        return rep
    rep.info["decode_seconds"] = secs
    for e, bad in decode_bad.items():
        rep.add("decode", "fail", e, f"{len(bad)} rows could not be read; first: row {bad[0][0]} {bad[0][1]}")

    # loader == parquet
    for e, p in per_ep.items():
        d = data[data["episode_index"] == e]
        if len(d) == len(p["state"]):
            if not (np.allclose(np.stack(d["observation.state"].to_numpy()), np.stack(p["state"]), atol=1e-5)
                    and np.allclose(np.stack(d["action"].to_numpy()), np.stack(p["action"]), atol=1e-5)):
                rep.add("loader_mismatch", "fail", e, "state/action from the loader differ from the parquet")

    # --- signals
    lags = {}
    eps = [e for e, p in per_ep.items() if len(p["state"]) >= 20]

    def model_for(c, skip):
        use = [e for e in eps if e != skip] or eps                             # one episode alone: fit on itself
        return _picture_model([np.stack(per_ep[e]["state"]) for e in use], [np.stack(per_ep[e][c]) for e in use])

    rep.info["picture_model"] = {c: {str(e): {"w": [round(float(x), 4) for x in m[0]], "still": round(m[1], 3)}
                                     for e in eps for m in [model_for(c, e)]} for c in CAMS}
    for e, p in sorted(per_ep.items()):
        s, a = np.stack(p["state"]), np.stack(p["action"])
        if len(s) < 20:
            continue
        vs = np.abs(np.diff(s[:, :5], axis=0)).sum(1)                          # arm motion per frame (deg, 5 joints;
        va = np.abs(np.diff(a[:, :5], axis=0)).sum(1)                          # the gripper is 0..100, not degrees)
        mc = {c: np.abs(np.diff(np.stack(p[c]), axis=0)).mean((1, 2)) for c in CAMS}
        # stale camera: the picture changes much less than this joint motion makes it change in the other episodes
        for c in CAMS:
            for r0, r1, got, exp in stale_runs(s, np.stack(p[c]), model_for(c, e)):
                rep.add("stale_camera", "warn", e, f"{c}: picture unchanged for {r1 - r0 + 1} frames while the joints move "
                        f"(change {got:.2f}, expected {exp:.2f})", [r0 + 1, r1 + 1])
        # stuck reading: all 6 exactly equal while the command moves
        same = np.all(np.diff(s, axis=0) == 0, axis=1) & (va > MOVE_DEG)
        for r0, r1 in _runs(same, HOLD_RUN):
            rep.add("state_hold", "warn", e, f"all joint readings unchanged for {r1 - r0 + 1} frames while the command moves",
                    [r0 + 1, r1 + 1])
        # time jump: state, command and a camera all jump at one row, against the 3 rows on each side
        spikes = {"state": _spikes(vs, JUMP_FLOOR_DEG), "action": _spikes(va, JUMP_FLOOR_DEG)}
        for c in CAMS:
            spikes[c] = _spikes(mc[c], float(np.median(mc[c])))
        both = spikes["state"] & spikes["action"] & (spikes["front"] | spikes["wrist"])
        for r in np.nonzero(both)[0]:
            rep.add("time_jump", "warn", e, f"state {vs[r]:.1f}, command {va[r]:.1f} deg and the picture jump at once "
                                            f"(the rows around: {_around(vs, r):.1f}, {_around(va, r):.1f})",
                    [int(r), int(r + 1)])
        # lags
        da, ds_ = np.diff(a, axis=0), np.diff(s, axis=0)
        l_as, c_as = _lag(da, ds_)
        lg = {"action->state": (l_as, round(c_as, 3))}
        for c in CAMS:
            l, cc = _lag(vs, mc[c])
            lg[f"state->{c}"] = (l, round(cc, 3))
        lags[e] = lg
        if l_as < 0 and c_as >= LAG_MIN_CORR:
            rep.add("action_after_state", "warn", e, f"the state moves {-l_as} frames before the command (corr {c_as:.2f})")
        for c in CAMS:
            if lg[f"state->{c}"][1] >= LAG_MIN_CORR and lg[f"state->{c}"][0] > CAM_LAG_MAX:
                rep.add("camera_lag_large", "warn", e, f"{c} is {lg[f'state->{c}'][0]} frames behind the joints")
    rep.info["lags"] = {str(e): v for e, v in lags.items()}
    if len(lags) >= 3:
        for pair in next(iter(lags.values())):
            good = [e for e in lags if lags[e][pair][1] >= LAG_MIN_CORR]
            if len(good) < 3:
                continue
            med = float(np.median([lags[e][pair][0] for e in good]))
            for e in good:
                if abs(lags[e][pair][0] - med) >= LAG_OFF:
                    rep.add("lag_inconsistent", "warn", e, f"{pair}: {lags[e][pair][0]} frames, other episodes' median {med:g}")
    return rep


def print_report(rep: Report, out=sys.stdout):
    i = rep.info
    print(f"fps {i.get('fps')}, {i.get('total_episodes')} episodes, {i.get('total_frames')} frames, "
          f"version {i.get('codebase_version')}, robot {i.get('robot_type')}", file=out)
    for e, lg in (i.get("lags") or {}).items():
        print(f"  episode {e} lags (frames, corr): " + ", ".join(f"{k} {v[0]} ({v[1]})" for k, v in lg.items()), file=out)
    if not rep.items:
        print("  no findings", file=out)
    for it in rep.items:
        fr = f" frames {it['frames'][0]}..{it['frames'][1]}" if it["frames"] else ""
        ep = f"episode {it['episode']}" if it["episode"] is not None else "whole"
        print(f"  [{it['level']}] {it['check']} ({ep}{fr}): {it['detail']}", file=out)


def suite(out: Path, seed: int, reuse: bool = False) -> int:
    out.mkdir(parents=True, exist_ok=True)
    rows, ok_all = [], True
    for f in FAULTS:
        root = out / "datasets" / f
        t = time.time()
        stamp_file = root / "t0_mock_seed.txt"
        if not (reuse and stamp_file.exists() and stamp_file.read_text().strip() == str(seed)):
            make_mock(root, f, seed)
            stamp_file.write_text(str(seed))
        t_mock = time.time() - t
        rep = check(root)
        req, touched = EXPECT[f]
        got = rep.flagged()
        missing = sorted(req - got, key=str)
        stray = sorted({x for x in got if x[1] not in touched}, key=str)
        ok = not missing and not stray
        ok_all &= ok
        rows.append({"fault": f, "ok": ok, "missing": missing, "stray": stray, "flagged": sorted(got, key=str),
                     "info": rep.info, "items": rep.items, "mock_s": round(t_mock, 1)})
        print(f"== {f}: {'OK' if ok else 'NG'}  missing {missing}  stray {stray}")
        print_report(rep)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = out / f"suite_seed{seed}_{stamp}.json"
    path.write_text(json.dumps({"seed": seed, "ok": ok_all, "cases": rows}, ensure_ascii=False, indent=1, default=str),
                    encoding="utf-8")
    print(f"{sum(r['ok'] for r in rows)}/{len(rows)} as expected -> {path}")
    return 0 if ok_all else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("check")
    c.add_argument("root", type=Path)
    c.add_argument("--json", type=Path)
    m = sub.add_parser("mock")
    m.add_argument("root", type=Path)
    m.add_argument("--fault", default="clean")
    m.add_argument("--seed", type=int, default=1)
    s = sub.add_parser("suite")
    s.add_argument("--out", type=Path, default=PROJECT / "outputs" / "t0")
    s.add_argument("--seed", type=int, default=1)
    s.add_argument("--reuse", action="store_true", help="check the mocks already written with this seed again")
    a = ap.parse_args()
    if a.cmd == "mock":
        make_mock(a.root, a.fault, a.seed)
        print(f"wrote {a.root}")
    elif a.cmd == "check":
        rep = check(a.root)
        print_report(rep)
        if a.json:
            a.json.write_text(json.dumps({"info": rep.info, "items": rep.items}, ensure_ascii=False, indent=1, default=str),
                              encoding="utf-8")
        sys.exit(1 if any(i["level"] == "fail" for i in rep.items) else 0)
    else:
        sys.exit(suite(a.out, a.seed, a.reuse))


if __name__ == "__main__":
    main()
