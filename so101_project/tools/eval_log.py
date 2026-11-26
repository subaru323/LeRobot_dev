"""Plan, record and summarise policy evaluation trials (generalisation tests, ACT vs SmolVLA ...).

No robot, no LeRobot import: runs with any python 3.10+.

1) plan    : make the trial list in a RANDOM order, before running anything (so "the policy got tired" or
             "I got better at resetting" does not hide inside one condition).
   python tools\\eval_log.py plan <name> --conditions seen unseen blue two_cubes --repeats 5 [--seed 1]
   -> docs/records/eval/<name>.csv   (one row per trial, result columns empty)

2) record  : walk through the list; for each trial, run the policy, then type the result.
   python tools\\eval_log.py record <name>
   stage: 0 = did not grasp, 1 = grasped, 2 = lifted, 3 = carried over the target, 4 = released in the target,
          w = put the WRONG object in the target (counted as a failure), s = skip this trial for now
   Success = stage 4. You can stop with Ctrl+C at any time; finished rows are already saved.

3) summary : success count per condition, with the Wilson 95 % interval (for reading only).
   python tools\\eval_log.py summary <name> [<name2>]
   With two names, each common condition is also compared by the two-sided Fisher exact test
   (the decision rule: p < 0.05 = "difference seen"; same test as tools/success_interval.py --compare).

The CSV stays in the project (docs/records/eval/). Edit it by hand only to fix typos, and say so in "note".
"""

import argparse
import csv
import math
import random
import sys
from datetime import datetime
from pathlib import Path

EVAL_DIR = Path(__file__).resolve().parent.parent / "docs" / "records" / "eval"
FIELDS = ["trial", "condition", "repeat", "policy", "stage", "success", "failure", "operator", "time", "note"]
STAGES = {"0": "no grasp", "1": "grasped", "2": "lifted", "3": "over target", "4": "in target",
          "w": "wrong object in target (failure)"}


def path_of(name: str) -> Path:
    return EVAL_DIR / f"{name}.csv"


def read_rows(name: str) -> list[dict]:
    p = path_of(name)
    if not p.exists():
        sys.exit(f"no such log: {p}  (make it with: eval_log.py plan {name} ...)")
    with p.open(encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def write_rows(name: str, rows: list[dict]) -> None:
    p = path_of(name)
    tmp = p.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    while True:  # write-then-rename: a crash never leaves a half-written log
        try:
            tmp.replace(p)
            return
        except PermissionError:  # Windows: the CSV is open in Excel
            input(f"!! cannot write {p} (open in Excel?). Close it, then press Enter to retry ...")


def fisher_two_sided(a: int, n1: int, c: int, n2: int) -> float:
    """Same test as tools/success_interval.py --compare (two-sided Fisher exact)."""
    total, m = n1 + n2, a + c

    def prob(x: int) -> float:
        return math.comb(n1, x) * math.comb(n2, m - x) / math.comb(total, m)

    observed = prob(a)
    lo, hi = max(0, m - n2), min(n1, m)
    return sum(prob(x) for x in range(lo, hi + 1) if prob(x) <= observed * (1 + 1e-9))


def wilson(k: int, n: int, z: float = 1.959964) -> tuple[float, float]:
    if n == 0:
        return 0.0, 1.0
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return max(0.0, c - h), min(1.0, c + h)


def cmd_plan(a: argparse.Namespace) -> None:
    p = path_of(a.name)
    if p.exists() and not a.force:
        sys.exit(f"{p} exists. Pick another name, or add --force to overwrite (the old results are lost).")
    trials = [(c, r + 1) for c in a.conditions for r in range(a.repeats)]
    random.Random(a.seed).shuffle(trials)
    rows = [dict.fromkeys(FIELDS, "") | {"trial": i + 1, "condition": c, "repeat": r, "policy": a.policy}
            for i, (c, r) in enumerate(trials)]
    EVAL_DIR.mkdir(parents=True, exist_ok=True)
    write_rows(a.name, rows)
    print(f"wrote {len(rows)} trials to {p}  (seed {a.seed})")
    for row in rows:
        print(f"  {row['trial']:>3}  {row['condition']}")


def ask(prompt: str, allowed: set[str] | None = None) -> str:
    while True:
        s = input(prompt).strip()
        if allowed is None or s in allowed:
            return s
        print(f"   type one of: {' '.join(sorted(allowed))}")


def cmd_record(a: argparse.Namespace) -> None:
    rows = read_rows(a.name)
    todo = [r for r in rows if r["stage"] == ""]
    print(f"{len(rows) - len(todo)} done, {len(todo)} to go. Ctrl+C stops; finished rows are kept.")
    operator = a.operator or ask("operator name: ")
    try:
        for r in todo:
            print(f"\n--- trial {r['trial']}/{len(rows)}: condition = {r['condition']} (repeat {r['repeat']})")
            print("    set up the scene for this condition, run the policy, wait until the arm stops.")
            stage = ask("    stage 0-4 (0 none, 1 grasp, 2 lift, 3 over target, 4 in target), "
                        "w = WRONG object put in, s = skip: ", set(STAGES) | {"s"})
            if stage == "s":
                continue
            if stage == "w":
                failure = "wrong object"
            else:
                failure = "" if stage == "4" else ask("    failure type (e.g. missed grasp / dropped): ")
            note = ask("    note (Enter = none): ")
            r.update(stage=stage, success="1" if stage == "4" else "0", failure=failure,
                     operator=operator, time=datetime.now().isoformat(timespec="seconds"), note=note)
            write_rows(a.name, rows)  # save after every trial
    except (KeyboardInterrupt, EOFError):
        print("\nstopped; saved.")
    cmd_summary(argparse.Namespace(names=[a.name]))


def cmd_summary(a: argparse.Namespace) -> None:
    for name in a.names:
        rows = [r for r in read_rows(name) if r["stage"] != ""]
        print(f"\n{name}: {len(rows)} trials recorded")
        conds = sorted({r["condition"] for r in rows})
        print(f"  {'condition':<16}{'success':>9}  {'95% interval':>14}   stages 0,1,2,3,4,w   top failure")
        for c in conds:
            rs = [r for r in rows if r["condition"] == c]
            k, n = sum(r["success"] == "1" for r in rs), len(rs)
            lo, hi = wilson(k, n)
            hist = [sum(r["stage"] == s for r in rs) for s in STAGES]
            fails = [r["failure"] for r in rs if r["failure"]]
            top = max(set(fails), key=fails.count) if fails else "-"
            print(f"  {c:<16}{k:>4}/{n:<4}  {lo:>5.2f} - {hi:<5.2f}    {hist}   {top}")
        print("  The interval is only for reading: the range of success rates these trials cannot rule out.")
    if len(a.names) == 2:  # decision rule: Fisher exact, two-sided, p < 0.05 (same as success_interval.py)
        r1 = [r for r in read_rows(a.names[0]) if r["stage"] != ""]
        r2 = [r for r in read_rows(a.names[1]) if r["stage"] != ""]
        print(f"\ncompare {a.names[0]} vs {a.names[1]} (Fisher exact, two-sided; p < 0.05 = difference seen)")
        for c in sorted({r["condition"] for r in r1} & {r["condition"] for r in r2}):
            s1 = [r["success"] == "1" for r in r1 if r["condition"] == c]
            s2 = [r["success"] == "1" for r in r2 if r["condition"] == c]
            p = fisher_two_sided(sum(s1), len(s1), sum(s2), len(s2))
            verdict = "difference seen" if p < 0.05 else "not separated by these trials (do not write 'same')"
            print(f"  {c:<16}{sum(s1)}/{len(s1)} vs {sum(s2)}/{len(s2)}   p = {p:.3f}   {verdict}")


ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
sub = ap.add_subparsers(dest="cmd", required=True)
pp = sub.add_parser("plan")
pp.add_argument("name")
pp.add_argument("--conditions", nargs="+", required=True)
pp.add_argument("--repeats", type=int, default=5)
pp.add_argument("--policy", default="", help="e.g. act_cube_pick or smolvla_brick")
pp.add_argument("--seed", type=int, default=1)
pp.add_argument("--force", action="store_true")
pr = sub.add_parser("record")
pr.add_argument("name")
pr.add_argument("--operator", default="")
ps = sub.add_parser("summary")
ps.add_argument("names", nargs="+")
args = ap.parse_args()
{"plan": cmd_plan, "record": cmd_record, "summary": cmd_summary}[args.cmd](args)
