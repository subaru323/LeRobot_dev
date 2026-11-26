r"""物理で流す検算(計画書_完成形まで の S-15)。全部の種を node tools/export-physics.js で書き出し、run_physics.py で流して表にする。
最後に負の検算: 持っている間も爪を閉じない(握り込みを −20°)と、つかめずに NG になるか(合否の決め方が生きているか)。
  (摩擦 0.02 では、強く挟むので L・J は運べてしまい、O だけが落ちた。だから爪を閉じないほうにした)
  実行: physics\.venv\Scripts\python.exe physics\check_physics.py [--sets 0,2]   (1 種 2 分ほど)
"""
import argparse, json, os, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__)); SIM = os.path.dirname(HERE)
PY = sys.executable
sys.stdout.reconfigure(encoding="utf-8", line_buffering=True)
ap = argparse.ArgumentParser(); ap.add_argument("--sets", default="0,1,2,3,4,5,6"); a = ap.parse_args()
env = dict(os.environ, PYTHONIOENCODING="utf-8")
rows, bad = [], 0
for s in [int(x) for x in a.sets.split(",")]:
    name = "set%d_gap15_stand1" % s
    r = subprocess.run(["node", os.path.join("tools", "export-physics.js"), "--set", str(s), "--name", name], cwd=SIM, capture_output=True, text=True, encoding="utf-8")
    print(r.stdout.strip())
    if r.returncode:
        rows.append((s, "書き出せない", r.stdout.strip())); continue
    d = os.path.join(SIM, "out", "physics", name)
    r = subprocess.run([PY, os.path.join(HERE, "run_physics.py"), d], cwd=SIM, capture_output=True, text=True, encoding="utf-8", env=env)
    print(r.stdout.strip())
    res = json.load(open(os.path.join(d, "result.json"), encoding="utf-8"))
    rows.append((s, "OK" if res["ok"] else "NG", res))
# 負の検算
neg = os.path.join(SIM, "out", "physics", "set2_gap15_stand1")
if os.path.exists(os.path.join(neg, "scene.json")):
    r = subprocess.run([PY, os.path.join(HERE, "run_physics.py"), neg, "--squeeze", "-20"], cwd=SIM, capture_output=True, text=True, encoding="utf-8", env=env)
    res = json.load(open(os.path.join(neg, "result_squeeze-20.0.json"), encoding="utf-8"))
    caught = all(not p["ok"] for p in res["pieces"])     # 全部のピースが NG になること
    print("負の検算(爪を閉じない): " + ("全部のピースが NG(よい)" if caught else "★NG にならないピースがある") + "・置いた所のずれ 最大 %.1fmm・持ち上げ 最大 %.1fmm" % (res["worst_place_mm"], max(p["lift_mm"] for p in res["pieces"])))
    if not caught: bad += 1
print("\n種 | 物理 | 置いた所のずれ 最大 | 爪の中で滑った 最大 | 追い遅れ(爪の先)")
for s, st, res in rows:
    if isinstance(res, dict):
        print("%d | %s | %.2fmm | %.2fmm | %.1fmm" % (s, st, res["worst_place_mm"], max(p["slip_mm"] for p in res["pieces"]), res["tracking"]["tcp_max_mm"]))
    else:
        print("%d | %s | %s" % (s, st, res))
sys.exit(1 if bad else 0)
