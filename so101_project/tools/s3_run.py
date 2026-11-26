"""S3 誤差と全体の組立（総合計画書 改訂版の S3、2026-11-23）: 固定した回（tools/s3_cases.json）を MuJoCo の世界で流し、§4 の採点表を出す。

  sim\\physics\\.venv\\Scripts\\python.exe tools\\s3_run.py freeze                       回を固定する（回・コード・判定の sha256 と版）
  sim\\physics\\.venv\\Scripts\\python.exe tools\\s3_run.py run [--only ID,..] [--skip-group 初見] [--jobs 2] [--frozen FILE] [--tag 名前]
  sim\\physics\\.venv\\Scripts\\python.exe tools\\s3_run.py table RESULT.json [RESULT2.json]   採点表（2 つ渡すと同じ回を並べて比べる）
  sim\\physics\\.venv\\Scripts\\python.exe tools\\s3_run.py rejudge RESULT.json [--tag 名前]   流し直さずに、結果の記録（log）から行を作り直す
                                                                                  （判定と表だけを直したとき。点検 C62）

Each case: tools/puzzle_run.py --sim --mj + args (+ --mj-errors / --mj-faults JSON). Two at a time by default (16GB: at most two heavy jobs).

Outcome of one run (from the log; the truth is used only here, after the control has ended):
  完成          the control ran to the end (control_done), every physics row ok, no forbidden contact, no 判定不能 contact, no piece write
  接触あり完成  as 完成 but a forbidden contact (the arm touched a piece other than the one it was moving; 総合計画書 §4: 不合格)
  誤完了        the control ran to the end, a physics row not ok
  拒否          the control stopped before sending any command (plan / init refused)
  停止          the control stopped itself (stop_kind "Stop") after moving
  判定不能      WorldCannotDraw, another stop kind, a crash, no log
Classes: 完成必須 -> 完成. 停止必須 -> 停止 (or 拒否) with one of expect.reason_any in the stop message. 拒否必須 -> 拒否 with one of the
words. 可否未確定 -> recorded. In every class 誤完了 and 接触あり完成 are counted apart as safety errors (and fail a graded case).
"""

import argparse
import concurrent.futures as CF
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
CASES = os.path.join(HERE, "s3_cases.json")
OUTD = os.path.join(ROOT, "outputs", "s3")
SIM = os.path.join(ROOT, "sim")
sys.path.insert(0, HERE)
import puzzle_run as PR  # noqa: E402  (stdlib-only at import)

OUTCOMES = ("完成", "接触あり完成", "誤完了", "拒否", "停止", "判定不能")


def sha_text(obj) -> str:
    return hashlib.sha256(json.dumps(obj, ensure_ascii=False, sort_keys=True).encode("utf-8")).hexdigest()


def file_sha(p) -> str:
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def code_sha() -> str:
    h = hashlib.sha256(PR.code_sha().encode())
    for p in (os.path.join(HERE, "mj_world.py"), os.path.join(SIM, "physics", "run_physics.py")):
        h.update(open(p, "rb").read())
    for p in sorted(glob.glob(os.path.join(SIM, "out", "physics", "set*_gap*_stand1", "scene.json"))):
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


def run_case(c):
    cmd = [sys.executable, os.path.join(HERE, "puzzle_run.py"), "--sim", "--mj"] + c["args"]
    for k, flag in (("mj_errors", "--mj-errors"), ("mj_faults", "--mj-faults"), ("mj_occluders", "--mj-occluders")):
        if c.get(k):
            cmd += [flag, json.dumps(c[k])]
    t0 = datetime.datetime.now()
    p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, encoding="utf-8", env=dict(os.environ, PYTHONIOENCODING="utf-8"))
    secs = round((datetime.datetime.now() - t0).total_seconds())
    m = re.findall(r"wrote (.+\.json)", p.stdout)
    if not m:
        return None, (p.stdout[-1500:] + p.stderr[-1500:]), secs
    return json.load(open(m[-1].strip(), encoding="utf-8")) | {"_path": m[-1].strip()}, None, secs


def stop_msg(log) -> str:
    return next((e["msg"] for e in log.get("events", []) if e["msg"].startswith("!! stopped")), "")


def events_num(log, pat):
    for e in log.get("events", []):
        m = re.search(pat, e["msg"])
        if m:
            return float(m.group(1))
    return None


def outcome(log) -> str:
    if log is None:
        return "判定不能"
    mj = log.get("mj") or {}
    phys = mj.get("physics") or []
    cv = (mj.get("contact_verdict") or {}).get("count", {})
    if log.get("control_done"):
        if not phys or not all(r["ok"] for r in phys):
            return "誤完了"
        if cv.get("禁止接触", 0) or cv.get("判定不能", 0) or mj.get("piece_writes"):
            return "接触あり完成"
        return "完成"
    if log.get("stop_kind") == "Stop":
        return "拒否" if not mj.get("sent") else "停止"
    return "判定不能"


def per_moves(log) -> list:
    """§4 one row per move: piece, time, pictures, settled error, nearest approach to another piece, contacts, pushes, retries"""
    if log is None or not log.get("mj"):
        return []
    mj = log["mj"]
    cvd = mj.get("contact_verdict") or {}
    phys = {r["name"]: r for r in mj.get("physics") or []}
    pushed = max((r["pushed_mm"] for r in phys.values() if not r["moved"]), default=None)
    out = []
    prev_t1 = -1.0
    for mv in cvd.get("moves", []):
        rows = [r for r in cvd.get("rows", []) if r.get("target") == mv["piece"]]
        shots = [s for s in mj.get("shots", []) if prev_t1 < s["t_world"] <= mv["t1"]]      # C62: this move's pictures only (was from the start)
        prev_t1 = mv["t1"]
        fixes = sum(1 for e in log.get("events", []) if e["msg"].startswith(f"[wrist {mv['piece']}] corrected"))
        out.append({"piece": mv["piece"], "t0": mv["t0"], "t1": mv["t1"], "settled_mm": (phys.get(mv["piece"]) or {}).get("place_err_mm"),
                    "near_other_mm": mv.get("near_other_mm"), "forbidden": sum(1 for r in rows if r["verdict"] == "禁止接触"),
                    "piece_to_neighbour": sum(1 for r in rows if r["verdict"] == "対象と隣"),
                    "arm_table": sum(1 for r in rows if r["verdict"] == "腕と机"), "pictures": len(shots),
                    "pictures_top": sum(1 for s in shots if s["cam"] == "top"), "wrist_fixes": fixes})
    if out:
        out[-1]["neighbour_pushed_mm"] = pushed
    return out


def judge(c, log, err) -> tuple:
    oc = outcome(log)
    cls = c["class"]
    if cls == "可否未確定":
        return None, oc
    if oc in ("誤完了", "接触あり完成"):
        return False, oc
    if cls == "完成必須":
        return oc == "完成", oc
    msg = stop_msg(log) if log else (err or "")
    words = c["expect"]["reason_any"]
    hit = [w for w in words if w in msg]
    if cls == "停止必須":
        return (oc in ("停止", "拒否") and bool(hit)), oc + (f" ({', '.join(hit)})" if hit else "")
    if cls == "拒否必須":
        return (oc == "拒否" and bool(hit)), oc + (f" ({', '.join(hit)})" if hit else "")
    return False, "unknown class"


def row_of(c, log, err, secs, fr, notes):
    ok, why = judge(c, log, err)
    mj = (log or {}).get("mj") or {}
    cvd = mj.get("contact_verdict") or {}
    text_ok = fr["cases_sha256"].get(c["id"]) == sha_text(c)
    graded = c["class"] != "可否未確定"
    match = (ok is True and text_ok and not notes) if graded else None
    return {"id": c["id"], "class": c["class"], "group": c.get("group"), "why_case": c["why"], "match": match, "judge": why,
            "outcome": outcome(log), "case_text_same": text_ok, "secs": secs,
            "stop": (stop_msg(log) if log else (err or ""))[:300], "stop_kind": (log or {}).get("stop_kind"),
            "physics": mj.get("physics"), "contacts": cvd.get("count"), "forbidden_max_pen_mm": cvd.get("forbidden_max_pen_mm"),
            "max_load_nm": mj.get("max_load_nm"), "moves": per_moves(log), "n_back": events_num(log or {}, r"went back (\d+) poses"),
            "plan_arm_to_arm_mm": events_num(log or {}, r"arm-to-arm ([0-9.]+) mm"), "world_t": mj.get("world_t"),
            "log": os.path.relpath(log["_path"], ROOT) if log else None}


def run(only, skip_groups, jobs, frozen_path, tag):
    fr_files = sorted(glob.glob(os.path.join(OUTD, "frozen_*.json")))
    fp = frozen_path or (fr_files[-1] if fr_files else None)
    if not fp:
        raise SystemExit("no frozen file: run freeze first")
    fr = json.load(open(fp, encoding="utf-8"))
    cases = [c for c in json.load(open(CASES, encoding="utf-8"))["cases"]
             if (not only or c["id"] in only) and c.get("group") not in skip_groups]
    code, jsha = code_sha(), file_sha(os.path.abspath(__file__))
    notes = []
    if code != fr["code_sha256"]:
        notes.append("code differs from the frozen version")
    if jsha != fr["judge_sha256"]:
        notes.append("judge (s3_run.py) differs from the frozen version")
    ver = versions()
    if ver != fr["versions"]:
        notes.append(f"versions differ: {fr['versions']} -> {ver}")
    rows = {}
    with CF.ThreadPoolExecutor(max_workers=jobs) as ex:
        fut = {ex.submit(run_case, c): c for c in cases}
        for f in CF.as_completed(fut):
            c = fut[f]
            log, err, secs = f.result()
            r = row_of(c, log, err, secs, fr, notes)
            rows[c["id"]] = r
            print(f"== {c['id']} ({c['class']}, {c.get('group')}): {'MATCH' if r['match'] else ('record' if r['match'] is None else 'NOT MATCH')} "
                  f"{r['judge']} [{secs}s]", flush=True)
    rows = [rows[c["id"]] for c in cases]
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    res = {"date": stamp, "tag": tag, "frozen": os.path.relpath(fp, ROOT), "code_sha256": code, "judge_sha256": jsha, "versions": ver,
           "notes": notes, "rows": rows}
    out = os.path.join(OUTD, f"result_{stamp}{'_' + tag if tag else ''}.json")
    json.dump(res, open(out, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    open(out[:-5] + ".md", "w", encoding="utf-8").write(table([res]))
    graded = [r for r in rows if r["match"] is not None]
    print(f"S3: graded {sum(bool(r['match']) for r in graded)}/{len(graded)} as expected; safety errors "
          f"{sum(r['outcome'] in ('誤完了', '接触あり完成') for r in rows)}; -> {os.path.relpath(out, ROOT)}")
    return all(r["match"] for r in graded)


def rejudge(result_path, tag):
    """Rebuild the rows of a result from its logs with the current judge (no sim run). Keeps the run's frozen file, code sha and secs;
    the cases are judged as they were frozen for that run (the frozen file keeps them), not as tools/s3_cases.json has them now."""
    old = json.load(open(result_path, encoding="utf-8"))
    fr = json.load(open(os.path.join(ROOT, old["frozen"]), encoding="utf-8"))
    cases = {c["id"]: c for c in fr["cases"]}
    notes = list(old["notes"]) + [f"rejudged from the logs of {os.path.basename(result_path)} with judge {file_sha(os.path.abspath(__file__))[:12]}"]
    rows = []
    for r in old["rows"]:
        c = cases[r["id"]]
        log = None
        if r.get("log"):
            lp = os.path.join(ROOT, r["log"])
            log = json.load(open(lp, encoding="utf-8")) | {"_path": lp}
        rows.append(row_of(c, log, None if log else r["stop"], r["secs"], fr, list(old["notes"])))
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    res = dict(old, date=stamp, tag=tag or (old.get("tag", "") + "_rejudged"), notes=notes, rows=rows,
               judge_sha256=file_sha(os.path.abspath(__file__)), rejudged_from=os.path.relpath(result_path, ROOT))
    out = os.path.join(OUTD, f"result_{stamp}_{res['tag']}.json")
    json.dump(res, open(out, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    open(out[:-5] + ".md", "w", encoding="utf-8").write(table([res]))
    diff = [r["id"] for r, o in zip(rows, old["rows"]) if (r["outcome"], r["match"]) != (o["outcome"], o["match"])]
    print(f"rejudged {len(rows)} rows; outcome or match changed: {diff or 'none'} -> {os.path.relpath(out, ROOT)}")


def fmt(v, nd=2):
    return "" if v is None else (f"{v:.{nd}f}" if isinstance(v, float) else str(v))


def table(results) -> str:
    """§4: per class the denominator and every outcome; then each case; then each move (first result only)"""
    L = []
    for res in results:
        rows = res["rows"]
        L += [f"# S3 result {res['date']}{' ' + res['tag'] if res.get('tag') else ''}", "",
              f"frozen `{res['frozen']}`, code {res['code_sha256'][:12]}" + ("; NOTES: " + "; ".join(res["notes"]) if res["notes"] else ""), "",
              "## 分類ごとの分母と結果", "", "| 分類 | 分母 | " + " | ".join(OUTCOMES) + " | 期待どおり |", "|---|---|" + "---|" * (len(OUTCOMES) + 1)]
        for cls in ("完成必須", "停止必須", "拒否必須", "可否未確定"):
            rs = [r for r in rows if r["class"] == cls]
            if not rs:
                continue
            exp = "" if cls == "可否未確定" else f"{sum(bool(r['match']) for r in rs)}/{len(rs)}"
            L.append(f"| {cls} | {len(rs)} | " + " | ".join(str(sum(r["outcome"] == o for r in rs)) for o in OUTCOMES) + f" | {exp} |")
        L += ["", f"誤完了と接触あり完成（安全の誤り）: {sum(r['outcome'] in ('誤完了', '接触あり完成') for r in rows)} 件。"
              f"拒否必須なのに動いた: {sum(r['class'] == '拒否必須' and r['outcome'] != '拒否' for r in rows)} 件", "",
              "## 誤差の軸ごと", "", "| 軸 | 回 | " + " | ".join(OUTCOMES) + " |", "|---|---|" + "---|" * len(OUTCOMES)]
        groups = []
        for r in rows:
            if r["group"] not in groups:
                groups.append(r["group"])
        for g in groups:
            rs = [r for r in rows if r["group"] == g]
            L.append(f"| {g} | {len(rs)} | " + " | ".join(str(sum(r["outcome"] == o for r in rs)) for o in OUTCOMES) + " |")
        L += ["", "## 回ごと", "", "結果の判定不能は記録が無い・描けないなどの回。当たりの判定不能（何も持ち上げずに動かすピースに触れた）は別の列。",
              "", "| id | 軸 | 中身 | 結果 | 期待 | 物理（mm） | 禁止接触 | 当たりの判定不能 | 腕と机 | 隣への最短（mm） | 戻った姿勢 | 止まり |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in rows:
            ph = " ".join(f"{p['name']}{fmt(p.get('place_err_mm', p.get('pushed_mm')))}{'' if p['ok'] else '!'}" for p in (r.get("physics") or []))
            near = min((m["near_other_mm"] for m in r["moves"] if m.get("near_other_mm") is not None), default=None)
            L.append(f"| {r['id']} | {r['group']} | {r['why_case'][:40].replace('|', '/')} | {r['outcome']} | "
                     f"{'' if r['match'] is None else ('○' if r['match'] else '×')} | {ph} | {(r.get('contacts') or {}).get('禁止接触', '')} | "
                     f"{(r.get('contacts') or {}).get('判定不能', '')} | {(r.get('contacts') or {}).get('腕と机', '')} | "
                     f"{fmt(near)} | {fmt(r['n_back'], 0)} | {r['stop'][:90].replace('|', '/')} |")
        L.append("")
    if len(results) == 2:
        a, b = ({r["id"]: r for r in res["rows"]} for res in results)
        L += ["## 同じ回の比べ（1 つ目 → 2 つ目）", "", "| id | 1 つ目 | 2 つ目 | 物理 1 | 物理 2 |", "|---|---|---|---|---|"]
        for i in a:
            if i in b:
                p1 = max((p.get("place_err_mm", p.get("pushed_mm")) or 0 for p in (a[i].get("physics") or [])), default=None)
                p2 = max((p.get("place_err_mm", p.get("pushed_mm")) or 0 for p in (b[i].get("physics") or [])), default=None)
                L.append(f"| {i} | {a[i]['outcome']} | {b[i]['outcome']} | {fmt(p1)} | {fmt(p2)} |")
        L.append("")
    res = results[0]
    L += ["## 一手ごと（1 つ目の結果）", "", "持ち上げた一手だけ（持ち上げる前に止まった回は行が無い）。絵はその一手の間に撮った数（括弧は上のカメラ）。"
          "手首の直しは、その一手で手首の絵から目標を直した回数（つかみ直し）。動かさないピースの動きは回の終わりの値で、最後の一手の行にだけ書く。", "",
          "| id | ピース | 時刻（s） | 絵（上） | 手首の直し | 置いた後の外れ（mm） | 隣への最短（mm） | 禁止接触 | 対象と隣 | 腕と机 | 動かさないピースの動き（mm） |",
          "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in res["rows"]:
        for m in r["moves"]:
            L.append(f"| {r['id']} | {m['piece']} | {fmt(m['t0'], 1)}–{fmt(m['t1'], 1)} | {m['pictures']}（{m.get('pictures_top', '')}） | "
                     f"{m.get('wrist_fixes', '')} | {fmt(m['settled_mm'])} | "
                     f"{fmt(m['near_other_mm'])} | {m['forbidden']} | {m['piece_to_neighbour']} | {m.get('arm_table', '')} | {fmt(m.get('neighbour_pushed_mm'))} |")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser(usage=__doc__)
    ap.add_argument("what", choices=["freeze", "run", "table", "rejudge"])
    ap.add_argument("files", nargs="*")
    ap.add_argument("--only", default=None)
    ap.add_argument("--skip-group", default="")
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--frozen", default=None)
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    if a.what == "freeze":
        freeze()
        return
    if a.what == "table":
        print(table([json.load(open(f, encoding="utf-8")) for f in a.files]))
        return
    if a.what == "rejudge":
        rejudge(a.files[0], a.tag)
        return
    ok = run(set(a.only.split(",")) if a.only else None, set(filter(None, a.skip_group.split(","))), a.jobs, a.frozen, a.tag)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
