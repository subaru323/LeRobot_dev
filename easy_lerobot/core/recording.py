"""Settings and helpers for recording demonstrations with lerobot-record (SO-101, Windows)."""

import json
import os
import re
import tempfile
from pathlib import Path

from core.hardware import FOLLOWER_PORT, LEADER_PORT

FOLLOWER_ID = "follower_F"
LEADER_ID = "leader_L"
CALIBRATION_DIR = "dev_calibration"  # relative to the repository root
FRONT_CAMERA_INDEX = 1  # Logitech C270; index 0 is the laptop's built-in camera

DEFAULT_DATASET = "so101_task"
DEFAULT_EPISODES = 10
DEFAULT_EPISODE_S = 20
DEFAULT_RESET_S = 10

RECORD_LOG = Path(tempfile.gettempdir()) / "easy_lerobot" / "record.log"
# Test mode keeps its fake datasets away from the real ones.
TEST_DATASETS = Path(tempfile.gettempdir()) / "easy_lerobot" / "test_datasets"
FAKE_RECORD = Path(__file__).with_name("fake_record.py")


def repo_id(dataset_name: str) -> str:
    return f"local/{dataset_name}"


def dataset_dir(dataset_name: str, test: bool = False) -> Path:
    """Dataset folder, passed as --dataset.root (lerobot's default location for the repo id)."""
    if test:
        return TEST_DATASETS / dataset_name
    hf_home = Path(os.getenv("HF_HOME", "~/.cache/huggingface")).expanduser()
    lerobot_home = Path(os.getenv("HF_LEROBOT_HOME", hf_home / "lerobot")).expanduser()
    return lerobot_home / repo_id(dataset_name)


def recorded_episodes(dataset_name: str, test: bool = False) -> int:
    """Episodes saved in the dataset so far (0 when it does not exist yet)."""
    try:
        info = json.loads((dataset_dir(dataset_name, test) / "meta" / "info.json").read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return 0
    return int(info.get("total_episodes", 0))


def dataset_exists(dataset_name: str) -> bool:
    return (dataset_dir(dataset_name) / "meta" / "info.json").is_file()


def record_args(
    dataset_name: str, task: str, episodes: int, episode_s: int, reset_s: int, test: bool = False
) -> list[str]:
    """Arguments after `python -m` for one lerobot-record session.

    An existing dataset is resumed, so `episodes` more episodes are appended to it.
    lerobot-record creates the dataset folder before connecting to the arms, so a
    failed connection leaves an empty dataset behind; it is resumed as well.
    In test mode a fake recorder runs instead, without arms or cameras.
    """
    if test:
        return [
            str(FAKE_RECORD),
            f"--root={dataset_dir(dataset_name, test=True)}",
            f"--episodes={episodes}",
            f"--episode-s={episode_s}",
            f"--reset-s={reset_s}",
        ]
    camera = (
        f"{{front: {{type: opencv, index_or_path: {FRONT_CAMERA_INDEX}, "
        f"width: 640, height: 480, fps: 30, fourcc: MJPG}}}}"
    )
    return [
        "-m", "lerobot.scripts.lerobot_record",
        "--robot.type=so101_follower",
        f"--robot.port={FOLLOWER_PORT}",
        f"--robot.id={FOLLOWER_ID}",
        f"--robot.calibration_dir={CALIBRATION_DIR}",
        f"--robot.cameras={camera}",
        "--teleop.type=so101_leader",
        f"--teleop.port={LEADER_PORT}",
        f"--teleop.id={LEADER_ID}",
        f"--teleop.calibration_dir={CALIBRATION_DIR}",
        f"--dataset.repo_id={repo_id(dataset_name)}",
        # resume() refuses to write without an explicit root, so always name the folder.
        f"--dataset.root={dataset_dir(dataset_name)}",
        "--dataset.no_stamp=true",
        f"--dataset.single_task={task}",
        f"--dataset.num_episodes={episodes}",
        f"--dataset.episode_time_s={episode_s}",
        f"--dataset.reset_time_s={reset_s}",
        "--dataset.push_to_hub=false",
        f"--resume={'true' if dataset_exists(dataset_name) else 'false'}",
    ]


_EPISODE_RE = re.compile(r"Recording episode (\d+)")


def session_phase() -> tuple[str, int | None]:
    """Phase and episode index read from the log.

    Phase is 'starting', 'recording', 'reset', 'redo' (resetting before the current
    episode is recorded again) or 'saving'.
    """
    try:
        text = RECORD_LOG.read_text(encoding="utf-8", errors="replace")
    except FileNotFoundError:
        return "starting", None
    episodes = _EPISODE_RE.findall(text)
    episode = int(episodes[-1]) if episodes else None
    markers = {
        "recording": text.rfind("Recording episode"),
        "reset": text.rfind("Reset the environment"),
        "saving": text.rfind("Stop recording"),
    }
    phase = max(markers, key=markers.get)
    if markers[phase] < 0:
        return "starting", episode
    # lerobot prints this when Left is pressed; the episode will be recorded again.
    if phase == "reset" and text.rfind("rerecord the last episode") > markers["recording"]:
        return "redo", episode
    return phase, episode


def error_message() -> str:
    """Last meaningful line of a failed session, in plain words where possible."""
    try:
        lines = [ln.strip() for ln in RECORD_LOG.read_text(encoding="utf-8", errors="replace").splitlines() if ln.strip()]
    except FileNotFoundError:
        return ""
    text = "\n".join(lines[-30:])
    if f"'{LEADER_PORT}'" in text:
        return f"リーダー（{LEADER_PORT}）に接続できませんでした。"
    if f"'{FOLLOWER_PORT}'" in text:
        return f"フォロワー（{FOLLOWER_PORT}）に接続できませんでした。"
    if "camera" in text.lower() and ("error" in text.lower() or "failed" in text.lower()):
        return "カメラを開けませんでした。USB を確認してください。"
    return f"録画が止まりました: {lines[-1] if lines else ''}"
