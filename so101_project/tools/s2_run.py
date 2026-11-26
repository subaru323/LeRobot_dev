"""S2: one move with the normal control in MuJoCo (総合計画書 改訂版の S2). Expectations are written BEFORE running and frozen.

  sim\\physics\\.venv\\Scripts\\python.exe tools\\s2_run.py freeze
        write outputs/s2/frozen_<stamp>.json: the cases (tools/s2_cases.json) with their sha256, the code version
        (puzzle_run's code_sha256 + tools/mj_world.py + sim/physics/run_physics.py), this file's sha256, node and mujoco versions
  sim\\physics\\.venv\\Scripts\\python.exe tools\\s2_run.py run [--only S2-01,S2-04] [--frozen FILE]
        run each case with `puzzle_run.py --sim --mj` (this python: it has mujoco), judge it, write outputs/s2/result_<stamp>.json
        and .md. Exit 1 if any graded case does not match. A code / judge / case text that differs from the frozen one: not matching.

Judged (class in s2_cases.json):
  完成必須    result "ok" (the control ran to the end, its end check passed) AND the physics score (truth: MuJoCo's pieces) has
              every row ok AND no piece was written by the world (mj.piece_writes empty)
  検出か保留  expect.outcome "stop": the control stopped itself (stop_kind "Stop", not WorldCannotDraw / WorldStop / a crash) with
              one of reason_any in the stop message. "ok": result ok and every physics row ok (a stop here = holding too much).
              "either" (cases not used to set any line: safety only): a stop as above, or ok with the physics right.
              Always wrong: the control ran to the end (control_done) with a physics row not ok (FALSE COMPLETION).
              The world may write the pieces only for the declared faults: every nudge / drop must have fired (piece_writes),
              nothing else written; every occluder must have been drawn in at least one picture (mj.shots occluded)
  漏れの検算  --replay of the ref case's record: the commands are the same, one for one, and every observation is used; the
              physics score of the ref record changes in every row when only the truth is moved 5 mm, and at least one row
              turns from ok to not ok; the first top picture replaced by the other case's makes the commands differ
  Every --mj case of the other classes is also replayed (--replay): its commands must be the same (review C60 3)
  可否未確定  recorded only
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

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
CASES = os.path.join(HERE, "s2_cases.json")
OUTD = os.path.join(ROOT, "outputs", "s2")
SIM = os.path.join(ROOT, "sim")
sys.path.insert(0, HERE)
import puzzle_run as PR  # noqa: E402  (stdlib-only at import)


def sha_text(obj) -> str:
    return hashlib.sha256(json.dumps(obj, ensure_ascii=False, sort_keys=True).encode("utf-8")).hexdigest()


def file_sha(p) -> str:
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def code_sha() -> str:
    h = hashlib.sha256(PR.code_sha().encode())
    for p in (os.path.join(HERE, "mj_world.py"), os.path.join(SIM, "physics", "run_physics.py")):
        h.update(open(p, "rb").read())
    return h.hexdigest()


def versions() -> dict:
    import mujoco
    node = subprocess.run(["node", "--version"], capture_output=True, text=True).stdout.strip()
    return {"node": node, "mujoco": mujoco.__version__, "python": sys.version.split()[0]}


def freeze():
    cases = json.load(open(CASES, encoding="utf-8"))
    os.makedirs(OUTD, exist_ok=True)
    fr = {"date": datetime.datetime.now().isoformat(timespec="seconds"), "cases_sha256": {c["id"]: sha_text(c) for c in cases["cases"]},
          "cases": cases["cases"], "code_sha256": code_sha(), "judge_sha256": file_sha(os.path.abspath(__file__)), "versions": versions()}
    out = os.path.join(OUTD, "frozen_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".json")
    json.dump(fr, open(out, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    print(f"froze {len(cases['cases'])} cases -> {os.path.relpath(out, ROOT)}")


def run_case(c, extra=None):
    cmd = [sys.executable, os.path.join(HERE, "puzzle_run.py"), "--sim"] + (["--mj"] if not extra else []) + c["args"] + (extra or [])
    if c.get("mj_faults"):
        cmd += ["--mj-faults", json.dumps(c["mj_faults"])]
    if c.get("mj_occluders"):
        cmd += ["--mj-occluders", json.dumps(c["mj_occluders"])]
    p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, encoding="utf-8", env=dict(os.environ, PYTHONIOENCODING="utf-8"))
    m = re.findall(r"wrote (.+\.json)", p.stdout)
    if not m:
        return None, p.stdout[-2000:] + p.stderr[-2000:]
    return json.load(open(m[-1].strip(), encoding="utf-8")) | {"_path": m[-1].strip()}, None


def stop_msg(log) -> str:
    return next((e["msg"] for e in log.get("events", []) if e["msg"].startswith("!! stopped")), "")


def summary(log) -> dict:
    mj = log.get("mj") or {}
    return {"result": log.get("result"), "control_done": bool(log.get("control_done")), "stop_kind": log.get("stop_kind"), "stop": stop_msg(log)[:400],
            "physics": mj.get("physics"), "piece_writes": mj.get("piece_writes"), "grip_seen": log.get("grip_seen"),
            "end_check": log.get("end_check"), "world_t": mj.get("world_t"), "log": os.path.relpath(log["_path"], ROOT)}


def fired(c, log) -> str:
    """the declared faults and occluders really happened (review C60 4). '' = all fired"""
    mj = log.get("mj") or {}
    writes = mj.get("piece_writes") or []
    for f in c.get("mj_faults", []):
        if f["kind"] in ("nudge", "drop") and not any(w["piece"] == f["piece"] and w["why"].startswith("fault " + f["kind"]) for w in writes):
            return f"fault {f['kind']} {f['piece']} did not fire"
        if f["kind"] == "place":
            st = next(p for p in log["mj_world"]["start"]["pieces"] if p["name"] == f["piece"])
            if abs(st["x"] - f["x_mm"]) > 1e-6 or abs(st["z"] - f["z_mm"]) > 1e-6:
                return f"fault place {f['piece']} not in the start layout"
    for o in c.get("mj_occluders", []):
        if not any(sh["cam"] == o["cam"] and sh["occluded"] for sh in mj.get("shots", [])):
            return f"occluder on {o['cam']} {o['shots']} was never drawn"
    return ""


def judge(c, log) -> tuple:
    cls, s = c["class"], summary(log)
    phys_ok = bool(s["physics"]) and all(r["ok"] for r in s["physics"])
    writes = s["piece_writes"] or []
    bad_writes = [w for w in writes if not w["why"].startswith("fault")]
    if bad_writes:
        return False, f"the world wrote pieces outside the declared faults: {bad_writes}", s
    graded = cls != "可否未確定"
    nf = fired(c, log)
    if nf:
        return (False if graded else None), "NOT FIRED: " + nf, s
    if log.get("control_done") and not phys_ok:
        return (False if graded else None), "FALSE COMPLETION: the control ran to the end, the physics score is not ok", s
    if cls == "完成必須":
        if writes:
            return False, f"pieces were written: {writes}", s
        ok = s["result"] == "ok" and phys_ok
        return ok, ("ok" if ok else f"result {s['result']}, physics ok {phys_ok}: {s['stop']}"), s
    if cls == "検出か保留":
        e = c["expect"]
        if s["result"] == "ok" and phys_ok:
            if e["outcome"] in ("ok", "either"):
                return True, "placed, the physics is right", s
            return False, "ran to the end: expected a stop", s
        if s["result"] != "stopped":
            return False, f"result {s['result']}", s
        if s["stop_kind"] != "Stop":
            return False, f"stopped, but not by the control ({s['stop_kind']}): {s['stop']}", s
        if e["outcome"] == "ok":
            return False, f"HOLDING TOO MUCH: expected to place, stopped: {s['stop']}", s
        hit = [w for w in e["reason_any"] if w in s["stop"]]
        return (bool(hit), f"stopped ({', '.join(hit)})" if hit else f"stopped for another reason: {s['stop']}", s)
    return None, "recorded", s


def replay_same(c, log) -> tuple:
    rp, err = run_case({"args": c["args"]}, ["--replay", log["_path"]])
    if rp is None:
        return False, "replay wrote no log: " + err
    r = rp.get("replay", {})
    return (r.get("sent") == log["mj"]["sent"] and r.get("left") == 0), os.path.relpath(rp["_path"], ROOT)


def leak(c, logs, frozen_dir):
    import mj_world
    ref = logs.get(c["ref"])
    oth = logs.get(c["other"])
    if not ref or not oth:
        return False, f"needs {c['ref']} and {c['other']} in the same run", {}
    # 1) the same observations -> the same commands
    rp, err = run_case(c, ["--replay", ref["_path"]])
    if rp is None:
        return False, "replay did not write a log: " + err, {}
    same = rp.get("replay", {}).get("sent") == ref["mj"]["sent"]
    left = rp.get("replay", {}).get("left")
    # 2) the truth only moved: the physics score changes in every row
    sc = json.load(open(ref["mj_world"]["scene_json"], encoding="utf-8"))
    active = {i for i, r in enumerate(ref["mj"]["physics"]) if r["moved"]}
    s0 = mj_world.score_states(sc, ref["mj_world"]["start"], ref["mj"]["final_states"], active)
    s1 = mj_world.score_states(sc, ref["mj_world"]["start"], ref["mj"]["final_states"], active, truth_shift=(5.0, 0.0))
    key = lambda r: r.get("place_err_mm", r.get("pushed_mm"))
    changed = [key(a) != key(b) for a, b in zip(s0, s1)]
    flipped = [a["ok"] and not b["ok"] for a, b in zip(s0, s1)]
    # 3) another first top picture -> other commands
    mod = json.loads(json.dumps({"mj": ref["mj"]}))
    i0 = next(i for i, o in enumerate(mod["mj"]["obs"]) if o["k"] == "cam" and o["name"] == "top")
    j0 = next(o for o in oth["mj"]["obs"] if o["k"] == "cam" and o["name"] == "top")
    mod["mj"]["obs"][i0]["v"] = dict(j0["v"])
    mp = os.path.join(frozen_dir, f"leak_other_{c['id']}.json")
    json.dump(mod, open(mp, "w", encoding="utf-8"))
    rp2, err2 = run_case(c, ["--replay", mp])
    sent2 = (rp2 or {}).get("replay", {}).get("sent") or []
    diff_at = next((n for n, (x, y) in enumerate(zip(sent2, ref["mj"]["sent"])) if x != y), None)
    if diff_at is None and len(sent2) != len(ref["mj"]["sent"]) and rp2 is not None:
        diff_at = min(len(sent2), len(ref["mj"]["sent"]))
    ok = same and left == 0 and all(changed) and any(flipped) and diff_at is not None
    det = {"commands_same": same, "n_commands": len(ref["mj"]["sent"]), "obs_left": left, "replay_log": os.path.relpath(rp["_path"], ROOT),
           "score_truth": s0, "score_truth_plus5mm": s1, "score_changed": changed, "score_flipped": flipped,
           "other_first_top": {"first_diff_command": diff_at, "n_sent": len(sent2), "result": (rp2 or {}).get("result"),
                               "stop": stop_msg(rp2)[:200] if rp2 else err2, "log": os.path.relpath(rp2["_path"], ROOT) if rp2 else None}}
    return ok, ("ok" if ok else "leak check failed"), det


def run(only, frozen_path):
    fr_files = sorted(glob.glob(os.path.join(OUTD, "frozen_*.json")))
    fp = frozen_path or (fr_files[-1] if fr_files else None)
    if not fp:
        raise SystemExit("no frozen file: run freeze first")
    fr = json.load(open(fp, encoding="utf-8"))
    cases = json.load(open(CASES, encoding="utf-8"))["cases"]
    code, jsha = code_sha(), file_sha(os.path.abspath(__file__))
    notes = []
    if code != fr["code_sha256"]:
        notes.append("code differs from the frozen version")
    if jsha != fr["judge_sha256"]:
        notes.append("judge (s2_run.py) differs from the frozen version")
    ver = versions()
    if ver != fr["versions"]:
        notes.append(f"versions differ: {fr['versions']} -> {ver}")
    rows, logs = [], {}
    for c in cases:
        if only and c["id"] not in only:
            continue
        t0 = datetime.datetime.now()
        if c["class"] == "漏れの検算":
            ok, why, det = leak(c, logs, OUTD)
            s = {"leak": det}
        else:
            log, err = run_case(c)
            if log is None:
                ok, why, s = False, "no log: " + err, {}
            else:
                logs[c["id"]] = log
                ok, why, s = judge(c, log)
                rs, rlog = replay_same(c, log)
                s = dict(s, replay_same=rs, replay_log=rlog)
                if not rs and ok is not None:
                    ok, why = False, why + "; REPLAY DIFFERS"
        text_ok = fr["cases_sha256"].get(c["id"]) == sha_text(c)
        graded = c["class"] != "可否未確定"
        match = (ok is True) and text_ok and not notes if graded else None
        rows.append({"id": c["id"], "class": c["class"], "match": match, "judge": why, "case_text_same": text_ok,
                     "secs": round((datetime.datetime.now() - t0).total_seconds()), **s})
        print(f"== {c['id']} ({c['class']}): {'MATCH' if match else ('record' if match is None else 'NOT MATCH')} {why}", flush=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    res = {"date": stamp, "frozen": os.path.relpath(fp, ROOT), "code_sha256": code, "judge_sha256": jsha, "versions": ver, "notes": notes, "rows": rows}
    out = os.path.join(OUTD, f"result_{stamp}.json")
    json.dump(res, open(out, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    md = [f"# S2 result {stamp}", "", f"frozen: `{res['frozen']}`" + ("; NOTES: " + "; ".join(notes) if notes else ""), "",
          "| id | class | match | judge | result | stop | physics |", "|---|---|---|---|---|---|---|"]
    for r in rows:
        ph = "; ".join(f"{p['name']} {p.get('place_err_mm', p.get('pushed_mm'))}{'' if p['ok'] else ' NG'}" for p in (r.get("physics") or []))
        md.append(f"| {r['id']} | {r['class']} | {r['match']} | {r['judge'][:80].replace('|', '/')} | {r.get('result', '')} | "
                  f"{(r.get('stop') or '')[:120].replace('|', '/')} | {ph} |")
    open(out[:-5] + ".md", "w", encoding="utf-8").write("\n".join(md) + "\n")
    graded = [r for r in rows if r["match"] is not None]
    print(f"S2: graded {sum(r['match'] for r in graded)}/{len(graded)} as expected; records {len(rows) - len(graded)} -> {os.path.relpath(out, ROOT)}")
    return all(r["match"] for r in graded)


def main():
    ap = argparse.ArgumentParser(usage=__doc__)
    ap.add_argument("what", choices=["freeze", "run"])
    ap.add_argument("--only", default=None)
    ap.add_argument("--frozen", default=None)
    a = ap.parse_args()
    if a.what == "freeze":
        freeze()
        return
    ok = run(set(a.only.split(",")) if a.only else None, a.frozen)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
