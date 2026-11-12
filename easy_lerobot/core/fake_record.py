"""Stand-in for lerobot-record in test mode: no arms or cameras are opened.

Prints the same progress lines as lerobot-record, reacts to the same keys
(Right = end the episode early, Left = re-record) through a pynput listener,
treats Ctrl+C / Ctrl+Break as "stop recording", and keeps meta/info.json's
total_episodes up to date so the UI can count saved episodes.
"""

import argparse
import json
import logging
import time
from pathlib import Path

from pynput import keyboard

events = {"exit_early": False, "rerecord_episode": False}


def on_press(key):
    if key == keyboard.Key.right:
        events["exit_early"] = True
    elif key == keyboard.Key.left:
        print("Left arrow key pressed. Exiting loop and rerecord the last episode...", flush=True)
        events["rerecord_episode"] = True
        events["exit_early"] = True


def wait(seconds: float):
    """Sleep until the time is up or Right / Left is pressed."""
    end = time.monotonic() + seconds
    while time.monotonic() < end and not events["exit_early"]:
        time.sleep(0.05)
    events["exit_early"] = False


def save_total(root: Path, total: int):
    meta = root / "meta"
    meta.mkdir(parents=True, exist_ok=True)
    (meta / "info.json").write_text(json.dumps({"total_episodes": total}), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--episodes", type=int, required=True)
    parser.add_argument("--episode-s", type=float, required=True)
    parser.add_argument("--reset-s", type=float, required=True)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="INFO %(asctime)s %(message)s")

    info = args.root / "meta" / "info.json"
    total = json.loads(info.read_text(encoding="utf-8"))["total_episodes"] if info.is_file() else 0
    save_total(args.root, total)

    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    time.sleep(1.0)  # pretend to connect to the arms and cameras
    recorded = 0
    try:
        while recorded < args.episodes:
            logging.info(f"Recording episode {total}")
            wait(args.episode_s)
            if recorded < args.episodes - 1 or events["rerecord_episode"]:
                logging.info("Reset the environment")
                wait(args.reset_s)
            if events["rerecord_episode"]:
                logging.info("Re-record episode")
                events["rerecord_episode"] = False
                continue
            total += 1
            recorded += 1
            save_total(args.root, total)
    finally:
        logging.info("Stop recording")
        listener.stop()
        time.sleep(2.0)  # pretend to encode the videos
        logging.info("Exiting")


if __name__ == "__main__":
    main()
