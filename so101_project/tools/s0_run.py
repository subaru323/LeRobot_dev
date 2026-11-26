"""S0: the fixed test cases of the puzzle (総合計画書 改訂版の S0). Expectations are written BEFORE running and frozen.

  python tools\\s0_run.py freeze            write outputs/s0/frozen_<stamp>.json: the cases (tools/s0_cases.json) with their sha256,
                                           the code version (puzzle_run's code_sha256), and the board size from plan-cli init.
                                           Do this before the first run of new or changed cases.
  python tools\\s0_run.py run [--only S0-03,S0-05] [--frozen FILE]
                                           run each case with `puzzle_run.py --sim`, compare with the frozen expectation, write
                                           outputs/s0/result_<stamp>.json and .md. Exit 1 if any case does not match.
                                           The newest frozen file is used unless --frozen is given; a case whose text differs from
                                           the frozen one, or a code version that differs, is reported (not hidden).

What is compared (class in s0_cases.json):
  完成必須    result == "ok" (puzzle_run's own score: the true placed position within PLACE_MAX_MM and the jaws' need)
  拒否必須・停止必須   result == "stopped" by a deliberate Stop() (not a crash nor a wrapped "plan-cli の誤り") and every word of
              reason_all is in the stop message (words, not reason codes: a different stop with the same words would pass)
  完成禁止    result == expect.result (normally "ng")
  可否未確定  recorded as it is; expect.result and true_clear_max_mm describe today's behaviour, not a pass line

A run whose code (puzzle_run's code_sha256), judge (this file) or case text differs from the frozen one counts as not matching.
Each case also records: the arm bias, every placed step's true-arm nearness to another piece (sim: the last
"true arm comes X mm" line of the plans run before that place), the placed true errors, the run log path.
Runs only --sim (the cases are refused otherwise). Uses the LeRobot .venv python like the other tools (it imports nothing
from LeRobot itself, but puzzle_run.py is run with the same interpreter).
"""
import argparse
import datetime
import glob
import hashlib
import json
import os
import re
import subprocess
import sys

NEAR_FLAG_MM = 1.0   # 【仮】a 完成必須 run whose true arm came closer than this to another piece gets a mark (点検 C58 の 8)
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
CASES = os.path.join(HERE, "s0_cases.json")
OUTD = os.path.join(ROOT, "outputs", "s0")
sys.path.insert(0, HERE)
import puzzle_run as PR  # noqa: E402  (stdlib-only at import; used for code_sha and the planner)


def sha_text(obj) -> str:
    return hashlib.sha256(json.dumps(obj, ensure_ascii=False, sort_keys=True).encode("utf-8")).hexdigest()


def board_of(set_no: int) -> dict:
    pl = PR.Planner()
    try:
        r = pl.need("init", set=set_no, k1="fake", calib="sim", seed=1)
        return {"set": set_no, "cell_mm": r["cell_mm"], "pitch_mm": round(r["pitch_mm"], 6),
                "gap_mm": round(r["pitch_mm"] - r["cell_mm"], 6), "pieces": [p["name"] for p in r["pieces"]],
                "fit_margin_mm": r["fit_margin_mm"]}
    finally:
        pl.close()


SCORING = ("puzzle_run.py --sim: result ok = every active piece placed and its true placed position (the sim moves the piece "
           "with the true arm: read angles + joint error) within PLACE_MAX_MM of the board target, and the pick within the jaws' need "
           "(e_a with the yaw error < fit_margin, |e_c| < 2 mm); ng = finished but a score row failed; stopped = a Stop() was raised "
           "(plan refused, wrist / detect / board checks). Contact of the jaws with a neighbour is NOT measured (true_clear is a "
           "distance report only).")


def opt(args: list, name: str, default):
    return type(default)(args[args.index(name) + 1]) if name in args else default


def layouts(cases: list) -> dict:
    """The start layout (sim truth) of each (set, seed): the pieces' true positions before anything moves."""
    out = {}
    for s, seed in sorted({(opt(c["args"], "--set", 2), opt(c["args"], "--seed", 1)) for c in cases}):
        pl = PR.Planner()
        try:
            pl.need("init", set=s, k1="fake", calib="sim", seed=seed)
            out[f"set{s}_seed{seed}"] = pl.need("truth", seed=seed)["layout"]
        finally:
            pl.close()
    return out


def judge_sha() -> str:
    """This file: the judge itself is part of what is frozen (点検 C58 の 5)."""
    return hashlib.sha256(open(os.path.abspath(__file__), "rb").read()).hexdigest()


def node_version() -> str:
    try:
        return subprocess.run(["node", "--version"], capture_output=True, text=True).stdout.strip()
    except OSError:
        return "?"


def arm_bias(args: list) -> list:
    """The joint error puzzle_run.py --sim draws from the seed (same lines as there: Random(seed*7+3), one uniform per joint)."""
    if "--arm-bias" in args:
        return [float(v) for v in args[args.index("--arm-bias") + 1].split(",")]
    e = opt(args, "--arm-err", 0.0)
    rnd = __import__("random").Random(opt(args, "--seed", 1) * 7 + 3)
    return [rnd.uniform(-e, e) for _ in PR.JOINTS]


def freeze() -> str:
    cases = json.load(open(CASES, encoding="utf-8"))
    sets = sorted({opt(c["args"], "--set", 2) for c in cases["cases"]})
    out = {"frozen_at": datetime.datetime.now().isoformat(timespec="seconds"),
           "cases_file_sha256": hashlib.sha256(open(CASES, "rb").read()).hexdigest(),
           "cases_sha256": {c["id"]: sha_text(c) for c in cases["cases"]},
           "code_sha256": PR.code_sha(), "judge_sha256": judge_sha(), "node": node_version(),
           "boards": [board_of(s) for s in sets], "layouts": layouts(cases["cases"]),
           "arm_bias_deg": {c["id"]: arm_bias(c["args"]) for c in cases["cases"]},
           "calib": "sim: k1 fake and the sim's own cameras, both in the code (code_sha256); --calib-mm / --calib-deg 0 unless a case says. "
                    "The board targets are set 2 of the code (in code_sha256), not written out here",
           "scoring": SCORING, "cases": cases}
    os.makedirs(OUTD, exist_ok=True)
    p = os.path.join(OUTD, "frozen_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".json")
    with open(p, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    print("froze", len(cases["cases"]), "cases ->", os.path.relpath(p, ROOT))
    return p


def placed_nearness(events: list) -> list:
    """[(piece, re-placing?, true-arm nearness mm)] for each placed step: the last true_clear before it."""
    out, last = [], None
    for e in events:
        if e.get("true_clear_mm") is not None:
            last = e["true_clear_mm"]
        m = re.match(r"\[placed (\w+)\] off the board target", e.get("msg", ""))
        if m and last is not None:
            out.append([m.group(1), "re-placing" in e["msg"], last])
            last = None
    return out


def judge(case: dict, log: dict) -> tuple:
    ex, cls, res = case["expect"], case["class"], log.get("result")
    stop = next((e["msg"] for e in reversed(log.get("events", [])) if e.get("msg", "").startswith("!! stopped")), "")
    why = []
    if cls == "完成必須":
        ok = res == "ok"
        if not ok:
            why.append(f"result {res} (want ok)")
    elif cls in ("拒否必須", "停止必須"):
        miss = [w for w in ex.get("reason_all", []) if w not in stop]
        # a crash (any exception) is also logged as "stopped": only a deliberate Stop() that is not a wrapped plan-cli bug counts (点検 C58 の 1)
        crash = not stop.startswith("!! stopped: Stop(") or "plan-cli の誤り" in stop
        ok = res == "stopped" and not miss and not crash
        if res != "stopped":
            why.append(f"result {res} (want stopped)")
        if miss:
            why.append(f"stop message lacks {miss}")
        if res == "stopped" and crash:
            why.append("stopped by a crash, not by a deliberate Stop()")
    elif cls == "完成禁止":
        ok = res == ex.get("result", "ng")          # the expected way of not finishing, not just "anything but ok" (点検 C58 の 7)
        if not ok:
            why.append(f"result {res} (want {ex.get('result', 'ng')})")
    elif cls == "可否未確定":
        ok = res == ex.get("result")
        near = [n[2] for n in placed_nearness(log.get("events", []))]
        if "true_clear_max_mm" in ex and (not near or min(near) > ex["true_clear_max_mm"]):
            ok = False
            why.append(f"true-arm nearness {min(near) if near else None} (today's behaviour was <= {ex['true_clear_max_mm']})")
        if res != ex.get("result"):
            why.append(f"result {res} (today's behaviour was {ex.get('result')})")
    else:
        ok, why = False, [f"unknown class {cls}"]
    return ok, why, stop


def run(only: list, frozen_path: str) -> int:
    frozen_path = frozen_path or (sorted(glob.glob(os.path.join(OUTD, "frozen_*.json"))) or [None])[-1]
    if not frozen_path:
        raise SystemExit("no frozen file: run `s0_run.py freeze` first (the expectations are fixed before running)")
    fz = json.load(open(frozen_path, encoding="utf-8"))
    now_cases = {c["id"]: c for c in json.load(open(CASES, encoding="utf-8"))["cases"]}
    code_now, judge_now = PR.code_sha(), judge_sha()
    same = code_now == fz["code_sha256"] and judge_now == fz.get("judge_sha256")
    if not same:
        print("!! the code or this judge differs from the frozen one: every row below counts as not matching (点検 C58 の 5)", flush=True)
    rows = []
    for case in fz["cases"]["cases"]:
        if only and case["id"] not in only:
            continue
        changed = case["id"] not in now_cases or sha_text(now_cases[case["id"]]) != fz["cases_sha256"][case["id"]]
        args = [sys.executable, os.path.join(HERE, "puzzle_run.py"), "--sim"] + case["args"]
        print(f"== {case['id']} ({case['class']}): {' '.join(case['args'])}", flush=True)
        p = subprocess.run(args, cwd=ROOT, capture_output=True, text=True, encoding="utf-8", errors="replace",
                           env=dict(os.environ, PYTHONIOENCODING="utf-8"))
        m = re.search(r"^wrote (.+\.json)\s*$", p.stdout, re.M)
        if not m:
            rows.append({"id": case["id"], "class": case["class"], "match": False, "why": ["no run log (crashed?)"],
                         "stderr_tail": p.stderr[-800:], "stdout_tail": p.stdout[-800:]})
            print("   no run log", flush=True)
            continue
        logp = m.group(1).strip()
        log = json.load(open(logp, encoding="utf-8"))
        ok, why, stop = judge(case, log)
        if changed:
            ok = False
            why.append("the case differs from the frozen one")
        if not same:
            ok = False
            why.append("code or judge differs from the frozen version")
        if log.get("arm_bias_deg") is not None and fz.get("arm_bias_deg", {}).get(case["id"]) is not None and                 max(abs(x - y) for x, y in zip(log["arm_bias_deg"], fz["arm_bias_deg"][case["id"]])) > 1e-9:
            ok = False
            why.append("the arm bias differs from the frozen one")
        near_flag = case["class"] == "完成必須" and any(n[2] < NEAR_FLAG_MM for n in placed_nearness(log.get("events", [])))
        row = {"id": case["id"], "class": case["class"], "args": case["args"], "expect": case["expect"], "match": ok, "why": why,
               "result": log.get("result"), "stop": stop, "arm_bias_deg": log.get("arm_bias_deg"),
               "placed_true_mm": (log.get("score") or {}).get("placed_true_mm"), "placed_nearness": placed_nearness(log.get("events", [])),
               "code_sha256": log.get("code_sha256"), "log": os.path.relpath(logp, ROOT), "case_changed_since_freeze": changed,
               "near_flag": near_flag, "detect_refused": log.get("detect_refused")}
        rows.append(row)
        print(f"   {'MATCH' if ok else 'DIFFERS'}: result {row['result']}" + (f" | {stop[:150]}" if stop else "")
              + (f" | {'; '.join(why)}" if why else ""), flush=True)
    out = {"run_at": datetime.datetime.now().isoformat(timespec="seconds"), "frozen": os.path.relpath(frozen_path, ROOT),
           "code_sha256_now": code_now, "judge_sha256_now": judge_now, "code_same_as_frozen": same, "boards": fz["boards"], "rows": rows}
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    pj = os.path.join(OUTD, f"result_{stamp}.json")
    with open(pj, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=1)
    md = [f"# S0 の結果 {out['run_at']}", "",
          f"- 固定した期待: `{out['frozen']}`。コードと判定の版は固定のときと{'同じ' if out['code_same_as_frozen'] else '違う（全部を食い違いとした）'}。",
          f"- 完成必須の回で、置いた手の本当の腕の近さが {NEAR_FLAG_MM:g}mm 未満なら「近い」と印を付ける（合否には入れない。sim の OK は接触を採点しない）。",
          f"- 盤: " + "; ".join(f"set {b['set']}: 辺 {b['cell_mm']}mm・すき間 {b['gap_mm']:g}mm・{''.join(b['pieces'])}" for b in out["boards"]), "",
          "| 番号 | 種類 | 期待どおりか | 結果 | 止まった理由・食い違い | 置いた手の本当の腕の近さ（mm） |", "|---|---|---|---|---|---|"]
    for r in rows:
        near = ", ".join(f"{n[0]}{'(置き直し)' if n[1] else ''} {n[2]:.1f}" for n in r.get("placed_nearness") or [])
        md.append(f"| {r['id']} | {r['class']} | {'はい' if r['match'] else '**いいえ**'} | {r.get('result')}{'（近い）' if r.get('near_flag') else ''} | "
                  f"{(r.get('stop') or '')[:120].replace('|', '/')}{(' / ' + '; '.join(r['why'])) if r['why'] else ''} | {near} |")
    with open(pj[:-5] + ".md", "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    graded = [r for r in rows if r["class"] != "可否未確定"]      # 可否未確定 is a record of today's behaviour, not a pass (点検 C58 の 2)
    n_ok = sum(r["match"] for r in graded)
    n_rec = sum(r["match"] for r in rows if r["class"] == "可否未確定")
    print(f"S0: graded {n_ok}/{len(graded)} as expected; records (可否未確定) {n_rec}/{len(rows) - len(graded)} same as before "
          f"-> {os.path.relpath(pj, ROOT)}")
    return 0 if n_ok == len(graded) and n_rec == len(rows) - len(graded) else 1


def main():
    ap = argparse.ArgumentParser(description="S0 fixed test cases (sim)")
    ap.add_argument("cmd", choices=["freeze", "run"])
    ap.add_argument("--only", default=None)
    ap.add_argument("--frozen", default=None)
    a = ap.parse_args()
    if a.cmd == "freeze":
        freeze()
        return
    sys.exit(run([v.strip() for v in a.only.split(",")] if a.only else [], a.frozen))


if __name__ == "__main__":
    main()
