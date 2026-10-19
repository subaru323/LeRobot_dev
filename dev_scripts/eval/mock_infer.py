"""Mock inference loop for SO-101 (D班 / AI推論).

Based on the official example examples/tutorial/act/act_using_example.py.
The only change: robot.get_observation() is replaced by fake_observation(),
so no robot is needed. The actions are not sent anywhere, only timed.
"""

import time

import numpy as np
import torch

from lerobot.datasets import LeRobotDatasetMetadata
from lerobot.policies import make_pre_post_processors
from lerobot.policies.act import ACTPolicy
from lerobot.policies.utils import build_inference_frame, make_robot_action

MODEL_PATH = "outputs/train/act_so101_mock/checkpoints/000100/pretrained_model"
DATASET_ID = "lerobot/svla_so101_pickplace"
NUM_STEPS = 200  # ACT predicts 100 actions per model call, so 200 steps = 2 model calls
FPS = 30  # the robot runs at 30 fps, so each step has 1000 / 30 = 33.3 ms


def fake_observation(features):
    """Make one fake observation in the same format as robot.get_observation()."""
    obs = {}
    for key, ft in features.items():
        if not key.startswith("observation."):
            continue
        if ft["dtype"] in ["image", "video"]:
            camera_name = key.removeprefix("observation.images.")
            obs[camera_name] = np.random.randint(0, 256, ft["shape"], dtype=np.uint8)
        else:
            for name in ft["names"]:
                obs[name] = float(np.random.uniform(-10, 10))
    return obs


def main():
    """Load the policy, run NUM_STEPS fake steps, and print the timing summary."""
    model = ACTPolicy.from_pretrained(MODEL_PATH)
    device = torch.device(model.config.device)  # falls back to cpu if cuda is not available

    dataset_metadata = LeRobotDatasetMetadata(DATASET_ID)  # metadata only, no videos
    preprocess, postprocess = make_pre_post_processors(model.config, dataset_stats=dataset_metadata.stats)

    print("device:", device)
    for key, ft in dataset_metadata.features.items():
        if key.startswith("observation.") or key == "action":
            print(f"  {key}: shape={ft['shape']} names={ft['names']}")

    model.reset()
    step_ms = []
    for step in range(NUM_STEPS):
        start = time.perf_counter()

        obs = fake_observation(dataset_metadata.features)
        obs_frame = build_inference_frame(
            observation=obs, ds_features=dataset_metadata.features, device=device
        )
        obs = preprocess(obs_frame)
        action = model.select_action(obs)
        action = postprocess(action)
        action = make_robot_action(action, dataset_metadata.features)

        step_ms.append((time.perf_counter() - start) * 1000)
        if step == 0:
            print("first action:", action)

    budget_ms = 1000 / FPS
    slow = [i for i, ms in enumerate(step_ms) if ms > budget_ms]
    print(f"steps: {NUM_STEPS}")
    mean_ms, max_ms = np.mean(step_ms), np.max(step_ms)
    print(f"mean: {mean_ms:.1f} ms   max: {max_ms:.1f} ms   budget at {FPS} fps: {budget_ms:.1f} ms")
    print(f"steps over budget: {slow}")


if __name__ == "__main__":
    main()
