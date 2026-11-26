r"""Dimensions of the overhead camera rig C (pole + boom + camera hanger + base board), and two estimates:
how long the camera sways after the arm stops, and the keep-out zone around the pole.

usage: python hardware\c_mount.py            (any python; no numpy needed)
       python hardware\c_mount.py --check-sim  (also needs node: reads sim/so101.js OVERHEAD_C and overheadTableY and
                                               checks that the sim's six boxes are the ones this file derives)

This file owns the numbers in hardware/C_台の設計.md (the document copies them).
Coordinates: "table" frame of sim/so101.js OVERHEAD_C [mm]: the top of the base board is height 0 (y up),
x = z = 0 is straight below the arm's base_link origin, +x = arm forward.
Everything marked GUESS is not measured (provisional / 【仮】).
"""

import math

# ---- inputs -------------------------------------------------------------------------------------------
LENS_TIP = (157.3, 540.0, 19.5)   # sim/so101.js OVERHEAD_C.lensTipMm (decided placement C, values are GUESS)
PAN_X = 38.8                      # shoulder_pan axis, x [mm] (SO101.joints[0].origin[0] = 0.0388353 m)
POLE_FROM_PAN = 315.1             # sim: pole (349.7, 51.1) is 315.1mm from the pan axis; keep that distance
LENS_FROM_FACE = 26.3             # PCB front face -> lens tip (sim, official 32x32 camera STL)
HANGER_PLATE = 3.0                # printed plate under the PCB, lens through a hole
PCB_STACK = 6.0                   # PCB + parts on its back side, above the front face [mm] GUESS
HANGER_GAP = 3.0                  # air between the PCB back and the boom [mm] GUESS
EXT = 20.0                        # 2020 aluminium extrusion (MISUMI HFS5-2020)
EXT_I = 6955.0                    # second moment of area of HFS5-2020 [mm^4] (catalog 0.6955 cm^4) GUESS until checked
EXT_KG_PER_M = 0.49               # mass of 2020 [kg/m] GUESS (typical catalog value)
E_AL = 69000.0                    # aluminium Young's modulus [N/mm^2]
BRACKET = 20.0                    # 2020 corner bracket leg length [mm] GUESS (common size 20x20)
BRACKET_KG = 0.015
CAM_KG = 0.010                    # 32x32 UVC module + lens GUESS
HANGER_KG = 0.010
ARM_TOP_WITH_PLATE = 524.5        # highest arm point in the photo pose and its moves, sim k02-compare, WITH the stock arm plate
PLATE = 7.2                       # stock arm plate (overhead_arm_base) thickness; this design drops it (arm origin 9.6 -> 2.4)
ARM_TOP_Y = 517.4                 # highest arm point over the board, measured in the sim WITH this rig (sim/tools/c-gap.js,
                                  # 2026-11-19 night, so101.js sha256 9ccac89492ba, default 8 sets, all frames). Until then this
                                  # file used ARM_TOP_WITH_PLATE - PLATE = 517.3 (another planner run, old plate)
ARM_ORIGIN_Y = 2.4                # base_link origin over the board top (sim OVERHEAD.baseBelowOriginMm; plate 0) GUESS
KEEP_MARGIN = 20.0                # extra margin around the pole for the controller's keep-out zone [mm] GUESS
BOARD = (500.0, 600.0, 12.0)      # base board x * z * thickness [mm] GUESS (plywood, cut at the shop)
BOARD_X0 = -80.0                  # rear edge of the board [mm]: arm base footprint starts at x = -22.4 (URDF); room for printed hold-downs GUESS
BOOM_PAST_POLE = 35.0             # boom sticks out past the pole's outer face, so a corner bracket fits on that side too
BOOM_PAST_CAM = 40.0              # boom end past the camera centre, room for the hanger's screws
HANGER_EAR = 30.0                 # hanger plate half-size with screw ears (x and z) GUESS
CABLE_R = 5.0                     # camera cable bundle along the pole/boom [mm] GUESS
ZETAS = (0.02, 0.05)              # damping ratio of a bolted aluminium frame, low / high GUESS


def boxes():
    """the rig as axis-aligned boxes {id: (lo, hi)} in the table frame [mm], same ids and same 'generous side' as
    sim/so101.js OVERHEAD_C.boxesMm (the sim copied them from here on 2026-11-19)."""
    lx, ly, lz = LENS_TIP
    pole_x = PAN_X + math.sqrt(POLE_FROM_PAN ** 2 - lz ** 2)
    pcb_face_y = ly + LENS_FROM_FACE
    boom_bottom = pcb_face_y + PCB_STACK + HANGER_GAP
    reach = pole_x - lx
    boom_len = reach + EXT / 2 + BOOM_PAST_POLE + BOOM_PAST_CAM
    h, fb = EXT / 2, EXT / 2 + BRACKET
    bx1 = pole_x + h + BOOM_PAST_POLE
    return {
        "c_foot": ((pole_x - fb, 0.0, lz - fb), (pole_x + fb, BRACKET, lz + fb)),
        "c_pole": ((pole_x - h, 0.0, lz - h), (pole_x + h, boom_bottom, lz + h)),
        "c_arm": ((bx1 - boom_len, boom_bottom, lz - h), (bx1, boom_bottom + EXT, lz + h)),
        "c_bracket": ((pole_x - fb, boom_bottom - BRACKET, lz - h), (pole_x + fb, boom_bottom, lz + h)),
        "c_hanger": ((lx - HANGER_EAR, pcb_face_y - HANGER_PLATE, lz - HANGER_EAR), (lx + HANGER_EAR, boom_bottom, lz + HANGER_EAR)),
        "c_cam": ((lx - 16.0, ly, lz - 16.0), (lx + 16.0, pcb_face_y + PCB_STACK, lz + 16.0)),
    }


SIM_JS = r"""
const {load}=require('./tools/_load'); load('kinematics.js','so101.js');
console.log(JSON.stringify({boxes: SO101.OVERHEAD_C.boxesMm, lens: SO101.OVERHEAD_C.lensTipMm, plate: SO101.OVERHEAD_C.plateMm,
  tableY: SO101.overheadTableY()*1000, below: SO101.OVERHEAD.baseBelowOriginMm}));
"""


def check_sim(tol=0.05) -> int:
    """compare boxes() with sim/so101.js. Returns the number of differences (0 = same)."""
    import json
    import os
    import subprocess
    sim = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim")
    r = subprocess.run(["node", "-e", SIM_JS], cwd=sim, capture_output=True, text=True, encoding="utf-8")
    if r.returncode:
        print("node failed: " + r.stderr[-300:])
        return 1
    j = json.loads(r.stdout)
    sb = {b["id"]: (tuple(b["lo"]), tuple(b["hi"])) for b in j["boxes"]}
    mine = boxes()
    bad = 0
    print("\n== the sim's boxes (sim/so101.js OVERHEAD_C.boxesMm) against this file ==")
    for k in sorted(set(sb) | set(mine)):
        if k not in sb or k not in mine:
            print(f"  DIFF {k}: only in {'the sim' if k in sb else 'this file'}")
            bad += 1
            continue
        d = max(abs(a - b) for lo_hi in range(2) for a, b in zip(mine[k][lo_hi], sb[k][lo_hi]))
        ok = d <= tol
        bad += not ok
        print(f"  {'same' if ok else 'DIFF'} {k:9s} lo {[round(v, 1) for v in mine[k][0]]} hi {[round(v, 1) for v in mine[k][1]]}"
              + ("" if ok else f"   sim lo {sb[k][0]} hi {sb[k][1]} (max diff {d:.2f})"))
    for name, a, b in (("lens tip", LENS_TIP, j["lens"]),):
        d = max(abs(x - y) for x, y in zip(a, b))
        bad += d > tol
        print(f"  {'same' if d <= tol else 'DIFF'} {name} {list(a)} (sim {b})")
    ty = -j["tableY"]
    okp = abs(j["plate"]) < 1e-9 and abs(ty - ARM_ORIGIN_Y) <= tol
    bad += not okp
    print(f"  {'same' if okp else 'DIFF'} arm origin above the board {ARM_ORIGIN_Y} (sim plateMm {j['plate']} + baseBelowOriginMm {j['below']} "
          f"-> overheadTableY {j['tableY']:.1f} mm)")
    print("CHECK-SIM " + ("OK" if not bad else f"DIFFERENT ({bad})"))
    return bad


def main() -> None:
    lx, ly, lz = LENS_TIP
    # pole: same distance from the pan axis as the sim, but on the line z = lens z, so the boom runs straight along -x
    pole_x = PAN_X + math.sqrt(POLE_FROM_PAN ** 2 - lz ** 2)
    pole_z = lz
    pcb_face_y = ly + LENS_FROM_FACE
    boom_bottom = pcb_face_y + PCB_STACK + HANGER_GAP
    boom_top = boom_bottom + EXT
    pole_len = boom_bottom                      # boom sits ON the pole end (2 corner brackets under it)
    reach = pole_x - lx                          # pole centre -> camera centre
    boom_len = reach + EXT / 2 + BOOM_PAST_POLE + BOOM_PAST_CAM
    print("== dimensions [mm] ==")
    print(f"pole (vertical 2020): centre x {pole_x:.1f}, z {pole_z:.1f}; length {pole_len:.0f} (stands on the board)")
    print(f"   (until 2026-11-19 the sim had a round pole r 12 at (349.7, 51.1), top 570; this is {math.hypot(pole_x - 349.7, pole_z - 51.1):.1f}mm "
          f"from it, same {POLE_FROM_PAN}mm from the pan axis, boom along -x. The sim now uses the boxes below: --check-sim)")
    print(f"boom (horizontal 2020, on top of the pole): y {boom_bottom:.1f}..{boom_top:.1f}, cut length {boom_len:.0f} "
          f"(x {pole_x + EXT / 2 + BOOM_PAST_POLE:.1f} -> x {pole_x + EXT / 2 + BOOM_PAST_POLE - boom_len:.1f}; corner brackets on both sides of the pole)")
    print(f"   hanger: plate under the PCB, legs up {boom_bottom - pcb_face_y:.1f} to the boom's bottom slot (printed in one piece)")
    print(f"camera: lens tip y {ly:.1f} at (x {lx:.1f}, z {lz:.1f}); PCB front face y {pcb_face_y:.1f}; "
          f"hanger plate y {pcb_face_y - HANGER_PLATE:.1f}..{pcb_face_y:.1f}")
    print(f"   lowest rig point = lens tip {ly:.1f}; highest arm point {ARM_TOP_Y:.1f} (sim c-gap.js with this rig) -> gap {ly - ARM_TOP_Y:.1f}"
          f"  (old estimate {ARM_TOP_WITH_PLATE} - plate {PLATE} = {ARM_TOP_WITH_PLATE - PLATE:.1f})")
    print(f"   lens above the arm origin: {ly - ARM_ORIGIN_Y:.1f} (sim with the stock plate: {ly - 9.6:.1f})")
    bx, bz, bt = BOARD
    print(f"board: {bx:.0f} x {bz:.0f} x {bt:.0f}, x {BOARD_X0:.0f}..{BOARD_X0 + bx:.0f}, z {-bz / 2:.0f}..{bz / 2:.0f}; "
          f"pole foot brackets reach x {pole_x + EXT / 2 + BRACKET:.1f} (edge margin {BOARD_X0 + bx - (pole_x + EXT / 2 + BRACKET):.1f})")
    print(f"cut list 2020: {pole_len:.0f} x1, {boom_len:.0f} x1; corner brackets: 4 (pole foot) + 2 (boom on pole) = 6")

    # ---- sway: pole as a cantilever fixed at the board, lumped mass at the top --------------------------
    L = pole_len
    k = 3 * E_AL * EXT_I / L ** 3                      # N/mm
    m_top = boom_len / 1000 * EXT_KG_PER_M + 2 * BRACKET_KG + CAM_KG + HANGER_KG
    m_eff = m_top + 0.236 * pole_len / 1000 * EXT_KG_PER_M
    w = math.sqrt(k * 1000 / m_eff)
    print("\n== sway (pole bending only; the board, the brackets and the desk add softness that is NOT in this) ==")
    print(f"stiffness at the top {k:.2f} N/mm; moving mass {m_eff * 1000:.0f} g; first mode {w / 2 / math.pi:.1f} Hz")
    for z in ZETAS:
        print(f"   damping {z}: amplitude down to 1/10 after {math.log(10) / (z * w):.2f} s, to 1/100 after {math.log(100) / (z * w):.2f} s")
    print(f"   camera tilt 1 mrad moves the image of the table by {ly * 0.001:.2f} mm (straight-down camera, height {ly:.0f})")

    # ---- keep-out zone for the controller (08 §3-2 item 6) ----------------------------------------------
    half_diag = EXT / math.sqrt(2)
    foot = math.hypot(EXT / 2 + BRACKET, EXT / 2 + BRACKET)
    print("\n== keep-out zone around the pole (vertical cylinders, centre = pole centre) ==")
    print(f"pole section half-diagonal {half_diag:.1f} (the sim uses the 20x20 box itself since 2026-11-19; before, a round r 12)")
    print("(heights over the board; the controller works from the base_link origin: subtract 2.4)")
    print(f"y 0..{BRACKET:.0f} (foot brackets): r {foot:.1f} + margin {KEEP_MARGIN:.0f} = {foot + KEEP_MARGIN:.1f}")
    print(f"y {BRACKET:.0f}..{boom_bottom:.1f}: r {half_diag:.1f} + cable {CABLE_R:.0f} + margin {KEEP_MARGIN:.0f} = {half_diag + CABLE_R + KEEP_MARGIN:.1f}")
    bx0 = pole_x + EXT / 2 + BOOM_PAST_POLE - boom_len
    m = KEEP_MARGIN
    print(f"boom: box x {bx0 - m:.1f}..{pole_x + EXT / 2 + BOOM_PAST_POLE + m:.1f}, z {pole_z - EXT / 2 - CABLE_R - m:.1f}..{pole_z + EXT / 2 + CABLE_R + m:.1f}, "
          f"y {boom_bottom - m:.1f}..{boom_top + m:.1f} (margin {m:.0f}, cable {CABLE_R:.0f})")
    print(f"camera + hanger: box x {lx - HANGER_EAR - m:.1f}..{lx + HANGER_EAR + m:.1f}, z {lz - HANGER_EAR - m:.1f}..{lz + HANGER_EAR + m:.1f}, "
          f"y {ly - m:.1f}..{boom_bottom:.1f}")


if __name__ == "__main__":
    import sys
    main()
    if "--check-sim" in sys.argv[1:]:
        sys.exit(1 if check_sim() else 0)
