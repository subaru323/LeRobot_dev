"""列 P: 置けたかの画像の確認の「元からの片寄り」を測る(点検 C42c の c-6。2026-11-19 夜の作業)。

物理を通さず、ピースを盤の目標の置き場そのままに描いて(写す姿勢・noise と種を変えて)上の画像 → detect → 目標と match。
出るずれは、画像の確認そのものの片寄り(MuJoCo の結果と無関係)。
  python(どれでもよい。node が要る): MAIN_PG\\lerobot\\.venv\\Scripts\\python.exe tools\\puzzle_mj_imgbias.py [--set 2] [--n 5]
出力: outputs/night/<時刻>_imgbias/result.json
"""
import argparse
import datetime
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
from puzzle_run import Planner  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", type=int, default=2)
    ap.add_argument("--n", type=int, default=5)
    a = ap.parse_args()
    out = os.path.join(ROOT, "outputs", "night", datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + "_imgbias")
    os.makedirs(out, exist_ok=True)
    scene = json.load(open(os.path.join(ROOT, "sim", "out", "physics", f"set{a.set}_gap15_stand1", "scene.json"), encoding="utf-8"))
    tg = {"pieces": [{"name": p["name"], "x": p["end"]["pos"][0] * 1000, "z": p["end"]["pos"][2] * 1000, "yaw": p["end"]["rot"]} for p in scene["pieces"]]}
    rows = []
    for noise in (0.0, 3.0):
        for seed in range(1, a.n + 1):
            pl = Planner()
            try:
                init = pl.need("init", set=a.set, k1="fake", calib="sim", calibMm=0, calibDeg=0, noise=noise, seed=seed)
                im = pl.need("render", cam="top", truth=tg, hide=[], lerobot=init["start_lerobot"], bias=None,
                             out=os.path.join(out, f"top_n{noise:g}_s{seed}.rgb"))
                det = pl.ask("detect", img=im["out"], W=im["W"], H=im["H"])
                if not det.get("ok"):
                    rows.append({"noise": noise, "seed": seed, "why": det.get("why")})
                    continue
                for pid, (p, t) in enumerate(zip(det["layout"]["pieces"], tg["pieces"])):
                    m = pl.need("match", pid=pid, est=t, meas=p)
                    rows.append({"noise": noise, "seed": seed, "name": t["name"], "off_mm": round(m["worst_cell_mm"], 3), "phi": round(m["phi"], 3)})
            finally:
                pl.close()
    for r in rows:
        print(r)
    json.dump({"targets": tg, "rows": rows}, open(os.path.join(out, "result.json"), "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    print("wrote", out)


if __name__ == "__main__":
    main()
