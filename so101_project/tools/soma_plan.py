"""A build plan for the puzzle in パズル/ (the Soma-type 3x3x3 cube, 40 mm cells) with the stock SO-101 jaw from above
(2026-11-23, the way chosen after 点検 C64: keep the 40 mm pieces, print them with clearance, hold ONE cell of the piece's top
layer by its two side faces, and let go a little above the place when the neighbours there are already filled).

For every solution whose pieces can all lie still on the table in their cube orientation (tools/soma_check.py), it searches
the order and, per piece, the grip (which top-layer cell, along x or y), and keeps the best plan by:
  1. drops: pieces that must be let go above the place (a finger-side neighbour in the top layer is already filled; the
     piece then falls the finger depth). Fewer is better
  2. swing: the centre of the piece's cells off the vertical plane through the grip axis (cells). Held by one cell, the piece
     turns about the grip axis until its centre hangs under it; the angle is atan(offset / depth of the centre under the
     grip). The largest angle over the pieces, then the sum
Model only: no physics, no clearance, no friction (that is tools/soma_mj.py's job), no arm reach (sim/tools: soma_reach.js).

usage: python -W ignore tools/soma_plan.py outputs/soma/voxels.json > outputs/soma/plan.json
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import soma_check as S  # noqa: E402  (finds every solution on import: about 35 s)

CELL_MM = 40.0


def com(cells):
    n = len(cells)
    return [sum(c[k] for c in cells) / n for k in range(3)]


def grip_options(cells, occ):
    """1-cell grips in the top layer: fingers beyond the cell along the axis, the columns above the top layer free and never the
    piece itself (on the table and at the place). drop = a finger-side top-layer neighbour is inside the cube and filled."""
    mine = set(cells)
    ztop = max(z for _, _, z in cells)
    c0 = com(cells)
    out = []
    for x, y, z in cells:
        if z != ztop:
            continue
        for ax in (0, 1):
            c = [x, y, z]
            ends = []
            for dlt in (-1, 1):
                q = list(c)
                q[ax] += dlt
                ends.append(q)
            if any(tuple(q) in mine for q in ends):
                continue                       # the run along the axis is longer than 1 cell
            ok, drop = True, False
            for q in ends:
                inside = 0 <= q[ax] <= 2
                for zz in range(z + 1, 3):
                    r = (q[0], q[1], zz)
                    if r in mine or (inside and r in occ):
                        ok = False
                if inside and (q[0], q[1], z) in occ:
                    drop = True
            if not ok:
                continue
            b = 1 - ax
            off = abs(c0[b] - c[b])
            depth = max(c[2] - c0[2], 0.0) + 0.5 * (20.0 / CELL_MM)   # grip band centre is ~10 mm under the top; centre of cells under it
            ang = math.degrees(math.atan2(off, depth)) if off > 1e-9 else 0.0
            out.append({"cell": (x, y, z), "axis": "xy"[ax], "drop": drop, "offset_cells": round(off, 3), "swing_deg": round(ang, 1)})
    return out


def best_plan(sol):
    """DP over the set of placed pieces: minimise (drops, max swing, sum swing)."""
    pieces = list(sol)
    n = len(pieces)
    memo = {}

    def go(mask):
        if mask == (1 << n) - 1:
            return (0, 0.0, 0.0), ()
        if mask in memo:
            return memo[mask]
        occ = set()
        for j in range(n):
            if mask >> j & 1:
                occ |= set(sol[pieces[j]])
        best = None
        for j in range(n):
            if mask >> j & 1:
                continue
            cs = sol[pieces[j]]
            if any((x, y, zz) in occ for x, y, z in cs for zz in range(z + 1, 3)):
                continue                       # something already above it
            if not any(z == 0 or (x, y, z - 1) in occ for x, y, z in cs):
                continue                       # would not land there
            for g in grip_options(cs, occ):
                sub = go(mask | (1 << j))
                if sub is None or sub[0] is None:
                    continue
                (d0, m0, s0), steps = sub
                sc = (d0 + g["drop"], max(m0, g["swing_deg"]), s0 + g["swing_deg"])
                if best is None or sc < best[0]:
                    best = (sc, ((pieces[j], g),) + steps)
        memo[mask] = best if best is not None else (None, None)
        return memo[mask]

    r = go(0)
    return None if r is None or r[0] is None else r


def main():
    cands = []
    for s in S.sols:
        if not all(S.stable(s[i]) for i in s):
            continue
        r = best_plan(s)
        if r:
            cands.append((r[0], s, r[1]))
    cands.sort(key=lambda t: t[0])
    sc, sol, steps = cands[0]
    hist = {}
    for c in cands:
        hist[c[0][0]] = hist.get(c[0][0], 0) + 1
    plan = {
        "about": "tools/soma_plan.py (2026-11-23). Cube cells: x, y on the table, z up, 0..2; 1 cell = 40 mm. Each step: the piece, "
                 "its cells in the cube, the one top-layer cell held and the axis the jaw closes along, whether it is let go above "
                 "the place (drop), and how far it would swing about the grip axis. table_cells = the same orientation shifted to "
                 "start at 0 (it must lie on the table so; the arm turns it about the vertical only).",
        "cell_mm": CELL_MM, "grip_band_mm": 20,
        "n_table_stable": sum(1 for s in S.sols if all(S.stable(s[i]) for i in s)), "n_with_plan": len(cands),
        "plans_by_drops": dict(sorted(hist.items())),
        "score": {"drops": sc[0], "max_swing_deg": round(sc[1], 1), "sum_swing_deg": round(sc[2], 1)},
        "steps": [],
    }
    for i, g in steps:
        cs = sol[i]
        m = [min(c[k] for c in cs) for k in range(3)]
        plan["steps"].append({"piece": S.names[i], "cells": [list(c) for c in cs],
                              "table_cells": [[c[0] - m[0], c[1] - m[1], c[2] - m[2]] for c in cs],
                              "grip": {"cell": list(g["cell"]), "axis": g["axis"]}, "drop": g["drop"],
                              "offset_cells": g["offset_cells"], "swing_deg": g["swing_deg"]})
    print(json.dumps(plan, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
