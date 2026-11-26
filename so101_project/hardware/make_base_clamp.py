r"""SPARE ONLY. Hold-down cleats for the SO-101 base on the plywood.

Since 2026-11-26 the base is screwed down through its own 4 countersunk holes (hardware/C_台の設計.md,
decision 3, check C71). Do not print these unless those holes cannot be used, and fix the known
weaknesses first (hardware/README.md, "押さえ（予備）"): 0.4 mm play lets the base turn ~1 deg;
the lip is an unsupported overhang (needs supports, or print on its side); the lip height is set by
the printed wall height, not by the screw.

usage: python hardware\make_base_clamp.py [--selftest]
       (the system python: needs cadquery, shapely, numpy, matplotlib)

The base outline is NOT typed by hand: it is sliced from the official base STL
(sim/models/src/assets/base_so101_v2.stl, units m) inside the two "wings"
(the flat plates the kit clamps grip). Each cleat is
  wall   : stands on the plywood beside the wing edge (gap CLEAR),
  lip    : reaches LIP_IN over the wing top (WING_TOP above the plywood),
  flange : a flat foot outside the wall, with one countersunk hole for a wood screw.

Frames
  STL frame of the base : y up (bottom = plywood = 0), x = left/right, z = back(-16)..front(+71) [mm].
  Arm (URDF) frame      : so101_new_calib.urdf, base visual origin xyz (-6.36, 0, -2.4) mm, rpy (90, 0, 90) deg
                          -> URDF (x, y, z) = (STL z - 6.36, STL x, STL y - 2.4). x = forward, y = left, z = up,
                          origin = the arm origin (2.4 mm above the plywood).
Outputs
  hardware/stl/base_clamp_<name>.stl  one cleat each, flange on the print bed (z = 0); the lip overhangs 6.4 mm (supports)
  hardware/out/base_clamp_preview.png top view in the arm frame: base outline, cleats, screw holes
  hardware/out/base_clamp.json        cleat boxes and screw points in the arm frame (for sim/ and the D2 template)

Numbers marked [tentative] are Claude's choices, not measured.
"""

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BASE_STL = os.path.join(ROOT, "sim", "models", "src", "assets", "base_so101_v2.stl")
URDF_OFF_X = -6.36471  # mm, so101_new_calib.urdf base_so101_v2 visual origin x
URDF_OFF_Z = -2.4  # mm

SLICE_Y = 10.0  # mm, inside the wing plate (2.4 .. 17.5)
WING_TOP = 17.5  # mm above the plywood; checked from the STL in selftest
BODY_Y = 18.5  # slice just above the wing: what is there is the fixed body the lip must avoid

CLEAR = 0.4  # mm gap wall <-> wing edge [tentative]
WALL = 5.0  # mm [tentative]
LIP_IN = 6.0  # mm the lip reaches over the wing [tentative]
LIP_T = 4.0  # mm [tentative]
LIP_GAP = 0.0  # mm lip bottom above WING_TOP (0 = touching as printed; the screw only pulls the flange) [tentative]
FLANGE = 14.0  # mm flange width outside the wall [tentative]
FLANGE_T = 5.0  # mm
BODY_KEEP = 2.0  # mm the lip keeps away from the fixed body [tentative]
SCREW_D = 3.8  # mm through hole for a 3.5 mm wood screw
SCREW_HEAD = 7.5  # mm countersink diameter
SCREW_LEN = 12.0  # mm
PLYWOOD_T = 12.0  # mm

# Windows in the STL plan (u = STL x, v = STL z) where each cleat may be [tentative]:
# back: the straight back edge of each wing, away from the centre body; side: the short flat side edge.
WINDOWS = {
    # u0, u1, v0, v1. Arm y = STL x, and +y is the arm's left, so u < 0 is the arm's RIGHT.
    "back_right": (-48.0, -24.0, -40.0, -4.0),
    "back_left": (24.0, 48.0, -40.0, -4.0),
    "side_right": (-90.0, -46.0, 12.0, 40.0),
    "side_left": (46.0, 90.0, 12.0, 40.0),
}


def load_stl(path):
    import struct

    b = open(path, "rb").read()
    if b[:5] == b"solid" and b"facet" in b[:300]:
        v = [list(map(float, l.split()[1:])) for l in b.decode().splitlines() if l.strip().startswith("vertex")]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84 : 84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(float)


def outline(tris, h):
    """Filled outline (holes removed) of the base cut at height h, as shapely geometry in the STL plan."""
    from shapely.geometry import MultiLineString, Polygon
    from shapely.ops import polygonize, unary_union

    segs = []
    for tri in tris:
        d = tri[:, 1] - h
        pts = []
        for i in range(3):
            a, b = tri[i], tri[(i + 1) % 3]
            da, db = d[i], d[(i + 1) % 3]
            if (da < 0) != (db < 0):
                p = a + da / (da - db) * (b - a)
                pts.append((round(p[0], 4), round(p[2], 4)))
        if len(pts) == 2 and pts[0] != pts[1]:
            segs.append(pts)
    polys = list(polygonize(unary_union(MultiLineString(segs))))
    return unary_union([Polygon(p.exterior) for p in polys if p.area > 1.0])


def parts_of(g):
    return [x for x in getattr(g, "geoms", [g]) if not x.is_empty and x.area > 0.5]


def to_arm(u, v):
    """STL plan (u = STL x, v = STL z) -> arm frame (x forward, y left)."""
    return v + URDF_OFF_X, u


def solid_from(poly, z0, z1):
    import cadquery as cq

    out = None
    for g in parts_of(poly):
        g = g.simplify(0.02)
        s = cq.Workplane("XY").workplane(offset=z0).polyline(list(g.exterior.coords)[:-1]).close().extrude(z1 - z0)
        for hole in g.interiors:
            s = s.cut(cq.Workplane("XY").workplane(offset=z0).polyline(list(hole.coords)[:-1]).close().extrude(z1 - z0))
        out = s if out is None else out.union(s)
    return out


def cleat_regions(wing, body, win, clear):
    from shapely.ops import polylabel, unary_union

    wall = (wing.buffer(clear + WALL) - wing.buffer(clear)) & win
    flange = (wing.buffer(clear + WALL + FLANGE) - wing.buffer(clear + WALL)) & win
    lip = ((wing.buffer(clear) - wing.buffer(-LIP_IN)) & win) - body.buffer(BODY_KEEP)
    lip = unary_union([g for g in parts_of(lip) if g.distance(wall) < 0.05])  # only pieces held by the wall
    big = max(parts_of(flange), key=lambda g: g.area)
    hole = polylabel(big, 0.05)
    return wall, flange, lip, hole, big.exterior.distance(hole)


def build(write=True, clear=CLEAR):
    import cadquery as cq
    from shapely.geometry import box

    tris = load_stl(BASE_STL) * 1000.0
    wing = outline(tris, SLICE_Y)
    body = outline(tris, BODY_Y)
    lip_top = WING_TOP + LIP_GAP + LIP_T
    report = {"source": "hardware/make_base_clamp.py", "base_stl": "sim/models/src/assets/base_so101_v2.stl",
              "frame": "arm (URDF): x forward, y left, z up, origin 2.4 mm above the plywood",
              "params": {k: globals()[k] for k in ("CLEAR", "WALL", "LIP_IN", "LIP_T", "LIP_GAP", "FLANGE", "FLANGE_T",
                                                    "BODY_KEEP", "SCREW_D", "SCREW_HEAD", "SCREW_LEN", "WING_TOP")},
              "cleats": {}}
    shapes, regions = {}, {}
    for name, (u0, u1, v0, v1) in WINDOWS.items():
        wall, flange, lip, hole, margin = cleat_regions(wing, body, box(u0, v0, u1, v1), clear)
        part = solid_from(wall, 0.0, lip_top).union(solid_from(flange, 0.0, FLANGE_T))
        if parts_of(lip):
            part = part.union(solid_from(lip, WING_TOP + LIP_GAP, lip_top))
        part = part.cut(cq.Workplane("XY").center(hole.x, hole.y).circle(SCREW_D / 2).extrude(FLANGE_T))
        dz = (SCREW_HEAD - SCREW_D) / 2  # 90 deg countersink
        csk = cq.Solid.makeCone(SCREW_D / 2, SCREW_HEAD / 2, dz, pnt=cq.Vector(hole.x, hole.y, FLANGE_T - dz))
        part = part.cut(cq.Workplane("XY").add(csk))
        shapes[name], regions[name] = part, (wall, flange, lip)
        allp = wall | flange | lip
        b = allp.bounds
        xs = sorted([to_arm(b[0], b[1])[0], to_arm(b[2], b[3])[0]])
        ys = sorted([to_arm(b[0], b[1])[1], to_arm(b[2], b[3])[1]])
        hx, hy = to_arm(hole.x, hole.y)
        report["cleats"][name] = {
            "box_arm_mm": {"x": [round(xs[0], 1), round(xs[1], 1)], "y": [round(ys[0], 1), round(ys[1], 1)],
                           "z": [URDF_OFF_Z, round(lip_top + URDF_OFF_Z, 1)]},
            "height_above_plywood_mm": round(lip_top, 1),
            "screw_arm_mm": [round(hx, 1), round(hy, 1)],
            "screw_edge_margin_mm": round(margin, 2),
            "lip_area_mm2": round(lip.area, 1),
            "volume_mm3": round(part.val().Volume()),
        }
        if write:
            bb = part.val().BoundingBox()
            os.makedirs(os.path.join(HERE, "stl"), exist_ok=True)
            cq.exporters.export(part.translate((-bb.xmin, -bb.ymin, 0)),
                                os.path.join(HERE, "stl", f"base_clamp_{name}.stl"))
    if write:
        os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
        with open(os.path.join(HERE, "out", "base_clamp.json"), "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=1)
        preview(wing, body, regions, report)
    return tris, wing, body, shapes, report


def preview(wing, body, regions, report):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))

    def draw(poly, **kw):
        first = True
        for g in parts_of(poly):
            x, y = g.exterior.xy
            ax_x, ax_y = zip(*[to_arm(a, b) for a, b in zip(x, y)])
            ax.fill(ax_y, ax_x, **kw, label=kw.pop("label", None) if first else None) if "fc" in kw else \
                ax.plot(ax_y, ax_x, **kw)
            first = False

    draw(wing, color="k", lw=1.2)
    draw(body, color="gray", lw=1, ls="--")
    for name, (wall, flange, lip) in regions.items():
        draw(flange, fc="#9ecae1", ec="tab:blue", lw=0.5)
        draw(wall, fc="#3182bd", ec="tab:blue", lw=0.5)
        draw(lip, fc="#fd8d3c", ec="tab:orange", lw=0.5, alpha=0.8)
        c = report["cleats"][name]
        ax.plot(c["screw_arm_mm"][1], c["screw_arm_mm"][0], "o", color="tab:red")
        ax.text(c["box_arm_mm"]["y"][0], c["box_arm_mm"]["x"][1] + 2, name, fontsize=8)
    ax.plot(0, 0, "+", color="tab:green", ms=12)
    ax.text(1, 1, "arm origin", color="tab:green", fontsize=8)
    ax.set_xlabel("arm y [mm] (+ = left of the arm)")
    ax.set_ylabel("arm x [mm] (+ = forward)")
    ax.invert_xaxis()  # looking down from above with the arm facing up the page: left of the arm is on the left
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3)
    ax.set_title("black: base wing / dashed: fixed body above it\nblue: wall+flange, orange: lip over the wing, red: screw")
    plt.tight_layout()
    plt.savefig(os.path.join(HERE, "out", "base_clamp_preview.png"), dpi=90)


def overlap_with_base(tris, part, heights):
    """Largest area [mm2] where a horizontal cut of the cleat overlaps the base cut at the same height."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    worst = 0.0
    for h in heights:
        faces = [Polygon([v.toTuple()[:2] for v in f.outerWire().Vertices()]).buffer(0)
                 for f in part.section(h).faces().vals()]
        cut = unary_union(faces) if faces else Polygon()
        worst = max(worst, cut.intersection(outline(tris, h)).area)
    return worst


def selftest():
    """Checks that do not trust the numbers typed above."""
    ok = True
    tris, wing, body, shapes, report = build(write=False)
    # 1. bottom at 0 and the wing top at WING_TOP (a big horizontal face there), nothing of the wing above it
    flat = np.all(np.abs(tris[:, :, 1] - tris[:, :1, 1]) < 0.01, axis=1)
    lv = tris[flat, 0, 1]
    a = 0.5 * np.linalg.norm(np.cross(tris[flat, 1] - tris[flat, 0], tris[flat, 2] - tris[flat, 0]), axis=1)
    area_top = a[np.abs(lv - WING_TOP) < 0.06].sum()
    print(f"bottom y = {tris[:, :, 1].min():.3f} (want 0); horizontal area at y={WING_TOP}: {area_top:.0f} mm2 (want > 2000)")
    ok &= abs(tris[:, :, 1].min()) < 0.01 and area_top > 2000
    # 2. arm frame: back edge of the base at x = -22.4 (C_台の設計.md)
    back = tris[:, :, 2].min() + URDF_OFF_X
    print(f"base back edge in the arm frame x = {back:.2f} (want -22.4)")
    ok &= abs(back + 22.4) < 0.1
    # 3. no cleat overlaps the base, from the plywood up to the lip top
    lip_top = WING_TOP + LIP_GAP + LIP_T
    hs = [0.5, 1.5, 3.0, 6.0, 10.0, 14.0, 17.0, WING_TOP + LIP_GAP + 0.3, lip_top - 0.3]
    for name, part in shapes.items():
        ov = overlap_with_base(tris, part, hs)
        c = report["cleats"][name]
        lip_ok = c["lip_area_mm2"] > 20
        scr_ok = c["screw_edge_margin_mm"] >= SCREW_HEAD / 2 + 1.0
        print(f"  {name}: overlap with the base {ov:.2f} mm2 (want < 0.5), lip {c['lip_area_mm2']} mm2 (want > 20), "
              f"screw edge margin {c['screw_edge_margin_mm']} mm (want >= {SCREW_HEAD / 2 + 1.0}), "
              f"height {c['height_above_plywood_mm']} mm")
        ok &= ov < 0.5 and lip_ok and scr_ok
    # 4. negative check: walls pushed 1 mm INTO the wing must be caught by check 3
    _, _, _, bad, _ = build(write=False, clear=-1.0)
    worst = max(overlap_with_base(tris, p, [6.0, 10.0]) for p in bad.values())
    print(f"negative check (wall 1 mm into the wing): overlap {worst:.1f} mm2 (want > 5: the check sees it)")
    ok &= worst > 5
    # 5. screw tip stays inside the plywood
    tip = FLANGE_T + PLYWOOD_T - SCREW_LEN
    print(f"screw {SCREW_LEN} mm through flange {FLANGE_T} into plywood {PLYWOOD_T}: tip {tip:.1f} mm above the plywood bottom (want >= 1)")
    ok &= tip >= 1
    print("SELFTEST", "OK" if ok else "FAILED")
    return ok


if __name__ == "__main__":
    # os._exit: on this PC the cadquery (OCP) teardown at interpreter exit crashes (0xC0000374)
    # after all the work is done, which would hide the real exit code.
    if "--selftest" in sys.argv:
        code = 0 if selftest() else 1
    else:
        *_, rep = build(write=True)
        print(json.dumps(rep, ensure_ascii=False, indent=1))
        code = 0
    sys.stdout.flush()
    os._exit(code)
