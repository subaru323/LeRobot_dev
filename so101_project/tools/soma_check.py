"""Soma-type 3x3x3 puzzle from the voxelized STLs: all solutions, and which can be built by a top-down arm.
Checks per solution (z = up, gravity):
  order: an order exists where each piece goes straight down from above (no cell above any of its cells is filled yet)
         and lands there (some cell has the floor or a filled cell right under it)
  table: every piece's orientation in the cube can lie still on a flat table (centre of its cells strictly inside the
         convex hull of its lowest layer of squares). The arm grips from above and turns about the vertical only,
         so it cannot turn a piece over: the piece must start on the table in that orientation (yaw free)
  fingers: (at the end) a parallel gripper from above holds a straight run of 1..RUN cells by its two end faces; its two
         fingers come down the columns just beyond the run ends, which must be outside the cube or still empty
Model only (2026-11-23, the puzzle in パズル/): no physics, no tolerance between pieces, no tipping while placing.
usage: python tools/soma_voxel.py パズル > outputs/soma/voxels.json
       python -W ignore tools/soma_check.py outputs/soma/voxels.json      (any python with numpy; about 1.5 min)"""
import itertools
import json
import sys

import numpy as np

d = json.load(open(sys.argv[1], encoding="utf-8"))
names = list(d["pieces"])
P = [np.array(d["pieces"][k]) for k in names]

ROT = []
for p in itertools.permutations(range(3)):
    for s in itertools.product([1, -1], repeat=3):
        M = np.zeros((3, 3), int)
        for i in range(3):
            M[i, p[i]] = s[i]
        if round(np.linalg.det(M)) == 1:
            ROT.append(M)


def orients(c):
    seen, out = set(), []
    for M in ROT:
        x = c @ M.T
        x = x - x.min(0)
        t = tuple(sorted(map(tuple, x)))
        if t not in seen:
            seen.add(t)
            out.append(np.array(t))
    return out


def idx(c):
    return int(c[0] * 9 + c[1] * 3 + c[2])


place = []      # per piece: list of (mask, cells)
for c in P:
    L = []
    for o in orients(c):
        for off in itertools.product(range(3), repeat=3):
            x = o + np.array(off)
            if (x <= 2).all():
                m = 0
                for q in x:
                    m |= 1 << idx(q)
                L.append((int(m), tuple(tuple(int(v) for v in q) for q in x)))
    place.append(L)

FULL = (1 << 27) - 1
order_p = sorted(range(len(P)), key=lambda i: len(place[i]))
sols = []


def rec(k, used, chosen):
    if k == len(order_p):
        sols.append(dict(chosen))
        return
    # fill the lowest empty cell first (exact cover): choose among placements of remaining pieces covering it
    free = (~used) & FULL
    cell = (free & -free).bit_length() - 1
    rem = [i for i in range(len(P)) if i not in chosen]
    for i in rem:
        for m, cells in place[i]:
            if m & (1 << cell) and not (m & used):
                chosen[i] = cells
                rec(k + 1, used | m, chosen)
                del chosen[i]


rec(0, 0, {})


def stable(cells):
    a = np.array(cells, float)
    zmin = a[:, 2].min()
    bot = a[a[:, 2] == zmin]
    com = a[:, :2].mean(0)
    # convex hull of the unit squares of the bottom layer (cell i covers [i-0.5, i+0.5])
    pts = np.array([[x + dx, y + dy] for x, y in bot[:, :2] for dx in (-.5, .5) for dy in (-.5, .5)])
    from math import atan2
    hull = []
    for p in sorted(map(tuple, pts)):
        while len(hull) >= 2 and np.cross(np.subtract(hull[-1], hull[-2]), np.subtract(p, hull[-2])) <= 0:
            hull.pop()
        hull.append(p)
    lower = hull
    hull = []
    for p in sorted(map(tuple, pts), reverse=True):
        while len(hull) >= 2 and np.cross(np.subtract(hull[-1], hull[-2]), np.subtract(p, hull[-2])) <= 0:
            hull.pop()
        hull.append(p)
    H = lower[:-1] + hull[:-1]
    eps = 0.05
    for i in range(len(H)):
        a1, b1 = np.array(H[i]), np.array(H[(i + 1) % len(H)])
        if np.cross(b1 - a1, com - a1) < eps * np.linalg.norm(b1 - a1):
            return False
    return True


def order_ok(sol):
    """search an order (DFS with memo on the set placed)"""
    occ0 = set()
    from functools import lru_cache
    pieces = list(sol)

    @lru_cache(None)
    def go(mask):
        if mask == (1 << len(pieces)) - 1:
            return ()
        occ = set()
        for j, i in enumerate(pieces):
            if mask >> j & 1:
                occ |= set(sol[i])
        for j, i in enumerate(pieces):
            if mask >> j & 1:
                continue
            cs = sol[i]
            mine = set(cs)
            if any((x, y, zz) in occ for x, y, z in cs for zz in range(z + 1, 3) if (x, y, zz) not in mine):
                continue
            if not any(z == 0 or (x, y, z - 1) in occ for x, y, z in cs):
                continue
            r = go(mask | (1 << j))
            if r is not None:
                return (i,) + r
        return None
    return go(0)


def report_solutions():
    given = {i: tuple(map(tuple, P[i])) for i in range(len(P))}
    res = {"n_pieces": len(P), "cells": [len(c) for c in P], "n_solutions": len(sols)}
    n_order = n_table = n_both = 0
    both = []
    for s in sols:
        o = order_ok(s)
        t = all(stable(s[i]) for i in s)
        n_order += o is not None
        n_table += t
        if o is not None and t:
            n_both += 1
            both.append((s, o))
    res.update(n_order_ok=n_order, n_table_ok=n_table, n_both=n_both)
    go = order_ok(given)
    res["given"] = {"order": [names[i] for i in go] if go else None,
                    "unstable_on_table": [names[i] for i in given if not stable(given[i])]}
    # per piece: how many of its orientations can lie still (ignoring position)
    res["stable_orientations"] = {names[i]: f"{sum(stable(tuple(map(tuple, o))) for o in orients(P[i]))}/{len(orients(P[i]))}" for i in range(len(P))}
    if both:
        s, o = both[0]
        res["example"] = {"order": [names[i] for i in o], "cells": {names[i]: s[i] for i in s}}
    print(json.dumps(res, ensure_ascii=False, indent=1, default=int))


# ---- fingers: a parallel gripper from above holds a straight run of 1..RUN cells along x or y in one layer, by its two end
# faces. Its two fingers come down the columns just beyond the run ends: those cells, from the grip layer up, must be
# outside the cube or empty (not yet placed, and not the piece itself). The same grip lifts it from the table (alone).
def grips(cells, occ, run_max):
    mine = set(cells)
    out = []
    for ax in (0, 1):
        for x, y, z in cells:
            c = [x, y, z]
            lo = hi = c[ax]
            while tuple(c[:ax] + [lo - 1] + c[ax + 1:]) in mine:
                lo -= 1
            while tuple(c[:ax] + [hi + 1] + c[ax + 1:]) in mine:
                hi += 1
            if hi - lo + 1 > run_max:
                continue
            ok = True
            for end in (lo - 1, hi + 1):
                for zz in range(z, 3):
                    q = c[:ax] + [end] + c[ax + 1:]
                    q[2] = zz
                    q = tuple(q)
                    if q in mine or (0 <= end <= 2 and q in occ):
                        ok = False
            if ok:
                out.append((ax, lo, hi, tuple(c)))
    return out


def order_grip(sol, run_max):
    from functools import lru_cache
    pieces = list(sol)

    @lru_cache(None)
    def go(mask):
        if mask == (1 << len(pieces)) - 1:
            return ()
        occ = set()
        for j, i in enumerate(pieces):
            if mask >> j & 1:
                occ |= set(sol[i])
        for j, i in enumerate(pieces):
            if mask >> j & 1:
                continue
            cs = sol[i]
            mine = set(cs)
            if any((x, y, zz) in occ for x, y, z in cs for zz in range(z + 1, 3)):
                continue
            if not any(z == 0 or (x, y, z - 1) in occ for x, y, z in cs):
                continue
            if not GRIPS(cs, occ, run_max):
                continue
            r = go(mask | (1 << j))
            if r is not None:
                return (i,) + r
        return None
    return go(0)


def grips_drop(cells, occ, run_max):
    """(点検 C64 の 1) the other rule: hold a run in the piece's TOP layer and let go while the fingertips are still above
    that layer's neighbours (the piece drops the finger depth d, which needs clearance/chamfers between pieces). The
    finger columns must be free only ABOVE the top layer (and never the piece itself)."""
    mine = set(cells)
    ztop = max(z for _, _, z in cells)
    out = []
    for ax in (0, 1):
        for x, y, z in cells:
            if z != ztop:
                continue
            c = [x, y, z]
            lo = hi = c[ax]
            while tuple(c[:ax] + [lo - 1] + c[ax + 1:]) in mine:
                lo -= 1
            while tuple(c[:ax] + [hi + 1] + c[ax + 1:]) in mine:
                hi += 1
            if hi - lo + 1 > run_max:
                continue
            ok = True
            for end in (lo - 1, hi + 1):
                for zz in range(z + 1, 3):
                    q = c[:ax] + [end] + c[ax + 1:]
                    q[2] = zz
                    q = tuple(q)
                    if q in mine or (0 <= end <= 2 and q in occ):
                        ok = False
            if ok:
                out.append((ax, lo, hi, tuple(c)))
    return out


def report_grips():
    global GRIPS
    GRIPS = grips
    n_stable = sum(all(stable(s[i]) for i in s) for s in sols)
    for rule, fn in (("fingers down to the grip layer at the place (no drop)", grips), ("top layer, let go above the neighbours (drop d)", grips_drop)):
        GRIPS = fn
        for run_max in (1, 2, 3):
            n = 0
            ex = None
            for s in sols:
                if not all(stable(s[i]) for i in s):
                    continue
                o = order_grip(s, run_max)
                if o is not None:
                    n += 1
                    ex = ex or (s, o)
            print(f"[{rule}] grip run up to {run_max} cell(s): buildable {n} of the {n_stable} table-stable solutions ({len(sols)} in all)")
            if ex and run_max == 3 and fn is grips:
                s, o = ex
                print("  example order:", [names[i] for i in o])
                print("  cells:", json.dumps({names[i]: s[i] for i in o}, ensure_ascii=False))

    # (点検 C64 の 5) the given STL arrangement turned as a whole (24 ways): which turns put every piece table-stable, and buildable
    GRIPS = grips
    ok_turns = []
    for M in ROT:
        s = {}
        for i in given:
            x = np.array(given[i]) @ M.T
            s[i] = x
        allx = np.vstack(list(s.values()))
        m0 = allx.min(0)
        s = {i: tuple(tuple(int(v) for v in q) for q in (s[i] - m0)) for i in s}
        if all(stable(s[i]) for i in s):
            ok_turns.append((order_grip(s, 3) is not None, GRIPS is grips))
    print(f"given arrangement: {len(ok_turns)} of 24 turns put every piece table-stable; of those, buildable with a 3-cell grip "
          f"(no drop): {sum(a for a, _ in ok_turns)}")



GRIPS = grips
if __name__ == "__main__":
    report_solutions()
    report_grips()
