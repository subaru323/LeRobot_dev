r"""Comparison video for the fictional reject cell (industrial plan: deliverable 1 of 4).

Runs the chosen cases with the chosen methods and draws them side by side on ONE world-time axis. Each panel shows
only what that controller was given (the latest top picture and its capture time), the pick point it chose (cross),
and its decisions with reasons as they happen. The scorer's verdict (truth: what really went into the bin) appears
only after the run ends and is labelled as the scorer's; it is never given to the controller.

usage (the .venv python: numpy + OpenCV + Pillow):
  python industrial\video.py                          normal and swap_visible x baseline and candidate
  python industrial\video.py --cases normal,miss_once --methods baseline,candidate --speed 0.5
writes outputs\industrial\<stamp>_video\compare_<cases>.mp4, a key-frame sheet (.png) and the timeline (.json).
"""

import argparse
import datetime
import json
import os
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from controllers import CONTROLLERS    # noqa: E402
from run import ROOT, run_case         # noqa: E402

FONT_PATH = r"C:\Windows\Fonts\meiryo.ttc"
FPS = 10
PIC_W, PIC_H, TXT_H = 440, 400, 190
JA = {"正常完了": "正常完了", "保留": "保留", "誤完了": "誤完了", "記録なし": "記録なし"}
MNAME = {"baseline": "基準（完了の信号で済み。絵は受け取るが使わない）", "candidate": "候補（追跡＋撮り直し）",
         "no_track": "候補－追跡", "no_reshoot": "候補－撮り直し"}
CNAME = {"normal": "正常", "swap_visible": "取り違え（照合の前に 2 個が入れ替わる）", "miss_once": "空振り 1 回",
         "stale_image": "古い絵が届く", "slip_large": "大きな滑り 45mm"}


def obs_time(ob):
    return ob.get("t", ob.get("t_capture", ob.get("t_request", 0.0)))


def timeline(r):
    """per step: time, the observations fed, the action, and the NEW controller history lines of that step."""
    ctrl = CONTROLLERS[r["ctrl"]]()
    out, n_hist = [], 0
    for s in r["steps"]:
        for ob in s["fed"]:
            ctrl.feed(ob)
        act = ctrl.next_action()
        assert act == s["action"], "replay differs from the recorded run"
        t = s["t"]     # world time when the controller decided (a stale picture keeps its old t_capture: not used here)
        out.append({"t": t, "fed": s["fed"], "action": act, "decisions": ctrl.history[n_hist:]})
        n_hist = len(ctrl.history)
    return out


def describe_action(a):
    if a is None:
        return "待つ"
    if "pick" in a:
        p = a["pick"]
        return f"取る {p['cmd_id']}  x={p['x']:.1f} z={p['z']:.1f} mm（{p['track_id']}）"
    if "reshoot" in a:
        return "撮り直す"
    return "終わり: " + "、".join(f"{t} {d['state']}" for t, d in sorted(a["end"].items()))


def describe_decision(d):
    k = d.get("decide")
    word = {"pick": "取る", "complete": "完了", "hold": "保留", "reject image": "絵を使わない", "still there": "まだある",
            "link": "再送に結ぶ", "conflict": "食い違い"}.get(k, k)
    return f"{word} {d.get('track', '')}: {d.get('why', '')}"


class Panel:
    def __init__(self, case, method, seed=0):
        self.r = run_case(case, method, seed)
        self.tl = timeline(self.r)
        self.case, self.method = case, method
        self.t_end = self.r["score"]["cycle_s"]

    def state_at(self, t):
        pic, pic_t, pick, acts, decs = None, None, None, [], []
        for st in self.tl:
            if st["t"] > t:
                break
            for ob in st["fed"]:
                if ob["type"] == "image":
                    pic, pic_t = ob.get("image"), obs_time(ob)
                    if pic is None:
                        pic_t = ("no frame", obs_time(ob))
            acts.append((st["t"], st["action"]))
            decs += [(st["t"], d) for d in st["decisions"]]
            if st["action"] and "pick" in st["action"]:
                pick = st["action"]["pick"]
        return pic, pic_t, pick, acts, decs


def draw_panel(p, t, font, font_s):
    pic, pic_t, pick, acts, decs = p.state_at(t)
    if pic is None:
        img = np.full((PIC_H, PIC_W, 3), 235, np.uint8)
        msg = "コンベアが動いている（絵はまだ）" if pic_t is None else f"絵が届かない（t={pic_t[1]}）"
    else:
        img = cv2.cvtColor(pic, cv2.COLOR_GRAY2BGR)
        msg = None
        if pick:     # image down = +x, right = +z; view x0=380, z0=-60, 2 px/mm (from the observation)
            cx, cy = int((pick["z"] + 60.0) * 2), int((pick["x"] - 380.0) * 2)
            cv2.drawMarker(img, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 40, 3)
    im = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    canvas = Image.new("RGB", (PIC_W, PIC_H + TXT_H + 44), (255, 255, 255))
    canvas.paste(im, (0, 44))
    d = ImageDraw.Draw(canvas)
    d.text((6, 2), f"{CNAME.get(p.case, p.case)}", font=font, fill=(0, 0, 0))
    d.text((6, 22), f"{MNAME.get(p.method, p.method)}", font=font_s, fill=(0, 0, 120))
    if msg:
        d.text((20, 44 + PIC_H // 2), msg, font=font, fill=(120, 0, 0))
    elif pic_t is not None:
        d.text((6, 48), f"制御が受け取った絵  撮影 t={pic_t:.2f}s", font=font_s, fill=(255, 255, 0))
    y = 44 + PIC_H + 4
    last_a = acts[-1] if acts else None
    d.text((6, y), "動作: " + (f"[{last_a[0]:.2f}s] " + describe_action(last_a[1]) if last_a else "-"), font=font_s, fill=(0, 0, 0))
    y += 22
    for tt, dd in decs[-4:]:
        col = (170, 0, 0) if dd.get("decide") in ("hold", "conflict", "reject image", "still there") else (0, 100, 0)
        line = f"[{tt:.2f}s] " + describe_decision(dd)
        d.text((6, y), line[:58], font=font_s, fill=col)
        y += 20
    if t > p.t_end:
        s = p.r["score"]
        y = 44 + PIC_H + TXT_H - 44
        d.rectangle([0, y - 2, PIC_W, y + 42], fill=(250, 240, 200))
        d.text((6, y), f"採点（制御には渡さない）: {JA[s['outcome']]}", font=font, fill=(0, 0, 0))
        d.text((6, y + 20), f"箱に入った物: {', '.join(s['bin']) or 'なし'}  誤排除 {s['wrong_reject']}", font=font_s, fill=(0, 0, 0))
    return np.array(canvas)[:, :, ::-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", default="normal,swap_visible")
    ap.add_argument("--methods", default="baseline,candidate")
    ap.add_argument("--speed", type=float, default=0.5, help="world seconds per video second")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    cases, methods = a.cases.split(","), a.methods.split(",")
    panels = [[Panel(c, m, a.seed) for m in methods] for c in cases]
    t_end = max(p.t_end for row in panels for p in row) + 0.4
    font, font_s = ImageFont.truetype(FONT_PATH, 16), ImageFont.truetype(FONT_PATH, 13)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(ROOT, "outputs", "industrial", stamp + "_video")
    os.makedirs(out_dir, exist_ok=True)
    name = "compare_" + "_".join(cases)
    frames, keys = [], []
    n = int(t_end / a.speed * FPS) + 1
    hold = 3 * FPS
    for i in range(n + hold):
        t = min(i, n - 1) * a.speed / FPS
        rows = [np.hstack([draw_panel(p, t, font, font_s) for p in row]) for row in panels]
        f = np.vstack(rows)
        bar = np.full((36, f.shape[1], 3), 30, np.uint8)
        bim = Image.fromarray(bar)
        ImageDraw.Draw(bim).text((10, 6), f"世界の時刻 t = {t:5.2f} s（全部の枠で同じ時刻。{a.speed} 倍の速さ）  乱数種 {a.seed}",
                                 font=font, fill=(255, 255, 255))
        f = np.vstack([np.array(bim), f])
        frames.append(f)
    h, w = frames[0].shape[:2]
    mp4 = os.path.join(out_dir, name + ".mp4")
    vw = cv2.VideoWriter(mp4, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (w, h))
    for f in frames:
        vw.write(f)
    vw.release()
    # key frames: when any panel's action changes, plus the last frame
    times = sorted({st["t"] for row in panels for p in row for st in p.tl if st["t"] > 0})
    for t in times:
        idx = min(int(round(t / a.speed * FPS)) + 1, len(frames) - 1)
        keys.append(frames[idx])
    keys.append(frames[-1])
    sheet = [cv2.resize(k, (k.shape[1] // 2, k.shape[0] // 2)) for k in keys]
    cv2.imencode(".png", np.vstack(sheet))[1].tofile(os.path.join(out_dir, name + "_keyframes.png"))
    tl = {f"{p.case}/{p.method}": [{"t": st["t"], "action": st["action"], "decisions": st["decisions"]} for st in p.tl]
          for row in panels for p in row}
    tl["_score"] = {f"{p.case}/{p.method}": p.r["score"] for row in panels for p in row}
    with open(os.path.join(out_dir, name + "_timeline.json"), "w", encoding="utf-8") as fh:
        json.dump(tl, fh, ensure_ascii=False, indent=1, default=str)
    ok = os.path.getsize(mp4) > 10000
    print(f"{'wrote' if ok else 'FAILED (empty mp4)'} {mp4}  {w}x{h}  {len(frames)} frames  {len(frames) / FPS:.1f} s")
    print(f"key frames {len(keys)} -> {name}_keyframes.png")
    for row in panels:
        for p in row:
            print(f"  {p.case:13s} {p.method:10s} {p.r['score']['outcome']}  bin {p.r['score']['bin']}  t_end {p.t_end}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
