r"""Fictional reject cell: the independent SCORER. It reads only the world's truth log (which tracking number is
which body, where every body ended, what happened) and the controller's final declarations. It never looks at the
case name or at the controller's reasons.

Outcome of a run:
  記録なし (no_decision)   a tracking number has no declaration
  保留     (hold)          some tracking number is declared hold (a safe stop: the line waits for a person)
  正常完了 (ok_complete)  no hold; every defective body is in the bin, no good body is, and good tracks are 'pass'
  誤完了   (false_complete) anything else declared without a hold
Beside it: 誤排除 wrong_reject (good tracked bodies in the bin, and untracked bodies a pick put there; counted even
with a hold), 流出 escape (a defective body left on
the belt while its track is not held), false_done (run I-5: tracks declared complete whose own body is not in the
bin; counted apart, the outcome rule is unchanged), extra_commands (commands beyond 1 per defective body + 1 per arm miss: the
handbook wants one reject command per judgement), bin (what really went to the bin), cycle_s (world time at the end).
flip_truth=True swaps which body is defective (the truth-leak check: the commands must not change, the score must).
"""


def score(truth_log, cmd_log, flip_truth=False):
    tracks = [d for t, w, d in truth_log if w == "tracks"][0]           # tracking number -> body id
    end = [d for t, w, d in truth_log if w == "end"][-1]
    bodies = {b["id"]: dict(b) for b in end["snapshot"]["bodies"]}
    if flip_truth:
        for b in bodies.values():
            b["defect"] = not b["defect"]
    decl = end["declarations"]
    state = {tid: (decl.get(tid) or {}).get("state") for tid in tracks}
    bad = [tid for tid, bid in tracks.items() if bodies[bid]["defect"]]
    good = [tid for tid in tracks if tid not in bad]
    wrong = sum(1 for tid in good if bodies[tracks[tid]]["where"] == "bin")
    # run I-2 (review C42h-1): a body WITHOUT a tracking number that a pick put into the bin is also a wrong reject
    # (nothing ever judged it NG). A body that was already in the bin before any pick is not counted.
    taken = {d["took"] for t, w, d in truth_log if w == "pick" and d.get("took")}
    wrong += sum(1 for bid in taken if bid not in tracks.values())   # defect or not: it never had an NG verdict
    removed_all_bad = all(bodies[tracks[tid]]["where"] == "bin" for tid in bad)
    escape = any(bodies[tracks[tid]]["where"] != "bin" and state[tid] != "hold" for tid in bad)
    # run I-5 (review C42l-2): per tracking number, "declared complete but its own body is not in the bin". A hold on
    # another track makes the run's outcome "hold", which hid this; it is counted apart and does NOT change the outcome
    false_done = sum(1 for tid in tracks if state[tid] == "complete" and bodies[tracks[tid]]["where"] != "bin")
    if any(s is None for s in state.values()):
        outcome = "記録なし"
    elif any(s == "hold" for s in state.values()):
        outcome = "保留"
    elif removed_all_bad and wrong == 0 and all(state[t] == "complete" for t in bad) and all(state[t] == "pass" for t in good):
        outcome = "正常完了"
    else:
        outcome = "誤完了"
    misses = sum(1 for t, w, d in truth_log if w == "miss")
    return {"outcome": outcome, "wrong_reject": wrong, "escape": escape, "false_done": false_done, "commands": len(cmd_log),
            "extra_commands": max(0, len(cmd_log) - len(bad) - misses), "misses": misses,
            "bin": sorted(f"{bid}({'NG' if bodies[bid]['defect'] else 'OK'})" for bid in bodies if bodies[bid]["where"] == "bin"),
            "cycle_s": end.get("cycle_s")}
