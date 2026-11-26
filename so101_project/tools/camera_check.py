"""Open several cameras AT THE SAME TIME, the way lerobot-record does, and measure the real frame rate.

usage: python tools\\camera_check.py 0 1 [--width 640 --height 480 --fps 30 --seconds 10 --fourcc MJPG]

Why: one camera alone often reaches 30 fps, but two on the same USB controller can drop to 15 fps or
stop (not enough USB bandwidth). The video this project follows hit this inside a virtual machine.
Record only after both cameras pass here with the same settings you will give lerobot-record.

Output:
  - per camera: frames received / seconds = measured fps, and OK / LOW
  - one snapshot per camera with a 3x3 grid and a centre cross, to aim the camera on the rig:
    outputs/camera_check/cam<index>.png   (project folder; not uploaded anywhere)
No robot is touched. Uses LeRobot's own OpenCVCamera class, so the result matches lerobot-record.
"""

import argparse
import threading
import time
from pathlib import Path

import cv2

from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig

p = argparse.ArgumentParser()
p.add_argument("indexes", nargs="+", help="camera numbers from lerobot-find-cameras opencv (e.g. 0 1)")
p.add_argument("--width", type=int, default=640)
p.add_argument("--height", type=int, default=480)
p.add_argument("--fps", type=int, default=30)
p.add_argument("--seconds", type=float, default=10.0)
p.add_argument("--fourcc", default=None, help='e.g. MJPG: compressed, needs less USB bandwidth than YUYV')
args = p.parse_args()

out_dir = Path(__file__).resolve().parent.parent / "outputs" / "camera_check"
out_dir.mkdir(parents=True, exist_ok=True)

cams = {}
for idx in args.indexes:
    key = int(idx) if idx.isdigit() else idx
    cfg = OpenCVCameraConfig(index_or_path=key, fps=args.fps, width=args.width, height=args.height, fourcc=args.fourcc)
    cam = OpenCVCamera(cfg)
    print(f"connecting camera {idx} ...", flush=True)
    cam.connect()  # raises if the camera refuses this size / fps -> try another --fourcc or size
    cams[idx] = cam

counts = {idx: 0 for idx in cams}
last = {}
errors = {idx: 0 for idx in cams}
stop = threading.Event()


def pull(idx: str, cam: OpenCVCamera) -> None:
    # async_read() returns only NEW frames (it waits for the camera thread), so counting calls = counting frames
    while not stop.is_set():
        try:
            last[idx] = cam.async_read(timeout_ms=1000)
            counts[idx] += 1
        except TimeoutError:
            errors[idx] += 1


threads = [threading.Thread(target=pull, args=(i, c), daemon=True) for i, c in cams.items()]
print(f"measuring {args.seconds:.0f} s with all {len(cams)} camera(s) running together ...", flush=True)
t0 = time.perf_counter()
for t in threads:
    t.start()
time.sleep(args.seconds)
stop.set()
for t in threads:
    t.join(timeout=2)
elapsed = time.perf_counter() - t0

all_ok = True
for idx, cam in cams.items():
    fps = counts[idx] / elapsed
    ok = fps >= 0.9 * args.fps
    all_ok &= ok
    print(f"camera {idx}: {counts[idx]} frames / {elapsed:.1f} s = {fps:.1f} fps  "
          f"(asked {args.fps}, timeouts {errors[idx]})  -> {'OK' if ok else 'LOW'}")
    if idx in last:
        img = cv2.cvtColor(last[idx], cv2.COLOR_RGB2BGR)  # LeRobot gives RGB, OpenCV writes BGR
        h, w = img.shape[:2]
        for k in (1, 2):
            cv2.line(img, (w * k // 3, 0), (w * k // 3, h), (0, 255, 0), 1)
            cv2.line(img, (0, h * k // 3), (w, h * k // 3), (0, 255, 0), 1)
        cv2.drawMarker(img, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
        path = out_dir / f"cam{idx}.png"
        cv2.imwrite(str(path), img)
        print(f"  snapshot: {path}")
    cam.disconnect()

if all_ok:
    print("OK: all cameras keep the frame rate together. Use the same settings in lerobot-record.")
else:
    print("!! LOW: try --fourcc MJPG, a lower size (e.g. 320x240), or plug the cameras into different "
          "USB ports on the PC itself (not the same hub). See guide 03 (rig and cameras) in docs.")
