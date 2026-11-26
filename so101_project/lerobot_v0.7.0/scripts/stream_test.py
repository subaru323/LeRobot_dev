"""Smoke test for LeRobot v0.7.0 dataset streaming (CPU only).

Streams a few frames from a public Hub dataset without downloading the whole dataset.
Usage:
    .venv\\Scripts\\python.exe scripts\\stream_test.py [repo_id] [num_samples]
"""

import sys
import time

import torch

from lerobot.datasets.streaming_dataset import StreamingLeRobotDataset

repo_id = sys.argv[1] if len(sys.argv) > 1 else "lerobot/pusht"
num_samples = int(sys.argv[2]) if len(sys.argv) > 2 else 20

t0 = time.perf_counter()
ds = StreamingLeRobotDataset(
    repo_id=repo_id,
    episodes=[0],  # one episode only, to keep traffic small
    shuffle=False,
    byte_budget_gb=0.5,
    prefetch_episodes=1,
)
print(f"init: {time.perf_counter() - t0:.1f}s")

t1 = time.perf_counter()
for i, item in enumerate(ds):
    if i == 0:
        print(f"first sample: {time.perf_counter() - t1:.1f}s")
        for k, v in item.items():
            desc = f"tensor{tuple(v.shape)} {v.dtype}" if isinstance(v, torch.Tensor) else repr(v)[:60]
            print(f"  {k}: {desc}")
    if i + 1 >= num_samples:
        break
elapsed = time.perf_counter() - t1
print(f"streamed {i + 1} samples in {elapsed:.1f}s ({(i + 1) / elapsed:.1f} samples/s)")
