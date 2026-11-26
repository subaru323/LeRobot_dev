"""Look inside any LeRobot dataset on the Hub WITHOUT downloading it: metadata + a few streamed frames.

Run with the v0.7.0 practice venv (CPU is fine):
  C:\\Users\\genes\\Documents\\PROJECTS\\PRACTICE\\lerobot_v0.7.0\\.venv\\Scripts\\python.exe tools\\v070\\peek_dataset.py lerobot/svla_so100_pickplace [--episode 0 --frames 90 --every 30]

Prints: robot type, fps, number of episodes / frames, the tasks (the language instructions), every feature
with its shape. Then streams one episode and writes, under outputs/v070/<repo>/:
  frames_ep<k>.png : camera images every --every frames, one row per camera (what the policy "sees")
  ep<k>.csv        : frame, timestamp, observation.state..., action...   (open in Excel to plot joints)
Datasets worth peeking at (public, SO-100/101): lerobot/svla_so100_pickplace (the SmolVLA paper's 50
episodes, 5 cube positions x 10), lerobot/pusht (2-D simulation).
"""

import argparse
import csv
import re
from pathlib import Path

import cv2
import numpy as np
import torch

from lerobot.datasets import LeRobotDatasetMetadata
from lerobot.datasets.streaming_dataset import StreamingLeRobotDataset

ap = argparse.ArgumentParser()
ap.add_argument("repo_id")
ap.add_argument("--episode", type=int, default=0)
ap.add_argument("--frames", type=int, default=90, help="how many frames of the episode to stream")
ap.add_argument("--every", type=int, default=30, help="keep one image every N frames")
args = ap.parse_args()

meta = LeRobotDatasetMetadata(args.repo_id)  # downloads only the small meta/ folder
print(f"{args.repo_id}: robot {meta.robot_type}, {meta.fps} fps, {meta.total_episodes} episodes, "
      f"{meta.total_frames} frames (~{meta.total_frames / meta.fps / 60:.1f} min)")
if meta.tasks is not None:
    print("tasks (language instructions):")
    for task in list(meta.tasks.index)[:10]:
        print(f"  - {task}")
print("features:")
for k, ft in meta.features.items():
    names = ft.get("names")
    extra = f"  names={names}" if isinstance(names, list) and len(names) <= 8 else ""
    print(f"  {k:<32} {ft.get('dtype'):<8} {tuple(ft.get('shape', ()))}{extra}")

out = Path(__file__).resolve().parents[2] / "outputs" / "v070" / re.sub(r"[^\w.-]", "_", args.repo_id)
out.mkdir(parents=True, exist_ok=True)
cams = meta.camera_keys
ds = StreamingLeRobotDataset(args.repo_id, episodes=[args.episode], shuffle=False, byte_budget_gb=0.5,
                             prefetch_episodes=1)
rows, shots = [], {c: [] for c in cams}
state_names = action_names = None
for i, item in enumerate(ds):
    if i >= args.frames:
        break
    st, ac = item.get("observation.state"), item.get("action")
    if state_names is None:
        state_names = [f"state{j}" for j in range(len(st))] if st is not None else []
        action_names = [f"action{j}" for j in range(len(ac))] if ac is not None else []
    rows.append([int(item["frame_index"]), float(item["timestamp"])]
                + (st.tolist() if st is not None else []) + (ac.tolist() if ac is not None else []))
    if i % args.every == 0:
        for c in cams:
            img = item[c]
            if isinstance(img, torch.Tensor):  # (3, H, W) float 0..1 -> (H, W, 3) uint8 BGR
                img = (img.permute(1, 2, 0).clamp(0, 1).numpy() * 255).astype(np.uint8)[:, :, ::-1]
            shots[c].append(np.ascontiguousarray(img))

with (out / f"ep{args.episode}.csv").open("w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["frame", "timestamp", *state_names, *action_names])
    w.writerows(rows)
print(f"streamed {len(rows)} frames -> {out / f'ep{args.episode}.csv'}")
if cams and any(shots.values()):
    h = min(im.shape[0] for c in cams for im in shots[c])
    lines = [np.hstack([cv2.resize(im, (int(im.shape[1] * h / im.shape[0]), h)) for im in shots[c]]) for c in cams]
    wmax = max(line.shape[1] for line in lines)
    grid = np.vstack([np.pad(line, ((0, 0), (0, wmax - line.shape[1]), (0, 0))) for line in lines])
    cv2.imwrite(str(out / f"frames_ep{args.episode}.png"), grid)
    print(f"images ({', '.join(cams)}) -> {out / f'frames_ep{args.episode}.png'}")
