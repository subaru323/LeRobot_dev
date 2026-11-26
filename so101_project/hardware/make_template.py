r"""D2: full-size template for the base board of the C stand (marks to copy onto the plywood), printed on A4 sheets.

usage (the .venv python, it needs Pillow and numpy):
  python hardware\make_template.py            -> hardware\template_D2_A4.pdf (+ template_D2_full.png, one picture of the whole board)
  python hardware\make_template.py selftest   -> the pages put back together equal the whole picture, the 100 mm bars are 100 mm
                                                 at 300 dpi, the marks sit where docs/利用者の日/D2_Cの台を組む.md section 2 says

Numbers: hardware/c_mount.py (the owner). This file only draws them. Frame of hardware/C_台の設計.md: +x = arm forward,
+z = the arm's right seen from behind; the board is x -80..420, z -300..300. On the paper: up = +x (forward), right = +z,
so lay the sheets with the "rear edge" at the board's rear edge and the "left edge" at z = -300.
Print at ACTUAL SIZE (100 %), portrait. Check the 100 mm bars on every sheet with a steel rule (+-1 mm). Pages overlap by
OVERLAP_MM: lay the next sheet so its grid crosses sit on the previous one's, tape, then copy the marks with a pencil
(or prick through).
Not on it: the printed hold-downs for the arm's base (their shape waits for the real base) and the camera hanger.
"""

import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import c_mount as CM   # noqa: E402

DPI = 300
PPM = DPI / 25.4
A4 = (210.0, 297.0)          # portrait, mm
MARGIN_MM = 15.0             # printers do not print to the edge (3-5 mm, more on some); the page texts sit inside 7 mm
OVERLAP_MM = 10.0            # at least this; the steps are made equal, so the real overlaps are larger (printed on each sheet)
BX, BZ, _BT = CM.BOARD
X0 = CM.BOARD_X0             # rear edge
Z0 = -BZ / 2                 # left edge
BASE_REAR_X = -22.4          # arm base footprint rear edge (URDF; c_mount.py BOARD_X0 comment) GUESS on the real base


def layout():
    """the marks [mm, board frame], computed like c_mount.main()."""
    lx, ly, lz = CM.LENS_TIP
    pole_x = CM.PAN_X + math.sqrt(CM.POLE_FROM_PAN ** 2 - lz ** 2)
    return {"pan": (CM.PAN_X, 0.0), "pole": (pole_x, lz), "lens": (lx, lz), "base_rear_x": BASE_REAR_X,
            "pole_half": CM.EXT / 2, "foot_half": CM.EXT / 2 + CM.BRACKET}


def font(px):
    for f in ("YuGothM.ttc", "meiryo.ttc", "msgothic.ttc"):
        try:
            return ImageFont.truetype(os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts", f), px)
        except OSError:
            continue
    return ImageFont.load_default()


def px(x, z):
    """board (x, z) mm -> full picture pixel (u right, v down): up = +x."""
    return ((z - Z0) * PPM, (X0 + BX - x) * PPM)


def draw_full():
    W, H = int(round(BZ * PPM)), int(round(BX * PPM))
    img = Image.new("L", (W, H), 255)
    d = ImageDraw.Draw(img)
    f_s, f_m = font(int(3.2 * PPM)), font(int(4.5 * PPM))
    for x in range(int(X0), int(X0 + BX) + 1, 10):                 # grid every 10 mm, darker every 50
        u0, v = px(x, Z0); u1, _ = px(x, Z0 + BZ)
        d.line([(u0, v), (u1, v)], fill=170 if x % 50 else 90, width=2 if x % 50 else 4)
    for z in range(int(Z0), int(Z0 + BZ) + 1, 10):
        u, v0 = px(X0 + BX, z); _, v1 = px(X0, z)
        d.line([(u, v0), (u, v1)], fill=170 if z % 50 else 90, width=2 if z % 50 else 4)
    for x in range(int(math.ceil(X0 / 50) * 50), int(X0 + BX), 50):  # grid numbers on the dark lines, repeated so every sheet has some
        for z in range(int(Z0), int(Z0 + BZ), 50):           # every 50 mm both ways: every overlap band has numbers
            u, v = px(x, z)
            d.text((u + 6, v - 4.5 * PPM), f"x{x} z{z}", fill=120, font=f_s)
    L = layout()
    d.rectangle([px(X0 + BX, Z0), px(X0, Z0 + BZ)], outline=0, width=6)       # board edge
    for z, name in ((0.0, "腕の前の線 z 0（端から端まで引く）"), (L["pole"][1], f"柱とレンズを通る線 z {L['pole'][1]:.1f}（端から端まで引く）")):
        d.line([px(X0, z), px(X0 + BX, z)], fill=0, width=5)
        u, v = px(X0 + 30 if z == 0 else X0 + 150, z)
        d.text((u + (8 if z else -8 - 75 * PPM), v), name, fill=0, font=f_m)
    xb = L["base_rear_x"]                                          # arm base rear edge (dashed)
    for z in np.arange(-60, 60, 8):
        d.line([px(xb, z), px(xb, z + 4)], fill=0, width=4)
    u, v = px(xb, -58); d.text((u, v + 1.5 * PPM), f"腕の土台の後ろの縁 x {xb}【仮】（肩の軸まで {CM.PAN_X - xb:.1f}）", fill=0, font=f_m)

    def cross(p, r_mm, label, circle_mm=0.0):
        u, v = px(*p)
        r = r_mm * PPM
        d.line([(u - r, v), (u + r, v)], fill=0, width=4); d.line([(u, v - r), (u, v + r)], fill=0, width=4)
        if circle_mm:
            c = circle_mm * PPM
            d.ellipse([u - c, v - c, u + c, v + c], outline=0, width=4)
        d.text((u + 6 * PPM, v + 3 * PPM), label, fill=0, font=f_m)
    cross(L["pan"], 15, f"肩の回る軸 x {L['pan'][0]:.1f}", 4)
    cross(L["lens"], 15, f"レンズの先の真下 x {L['lens'][0]:.1f}・z {L['lens'][1]:.1f}", 6)
    cross(L["pole"], 40, f"柱の中心 x {L['pole'][0]:.1f}・z {L['pole'][1]:.1f}")
    px_, pz_ = L["pole"]
    h = L["pole_half"]
    d.rectangle([px(px_ + h, pz_ - h), px(px_ - h, pz_ + h)], outline=0, width=5)          # 20 x 20 pole
    g = L["foot_half"]
    for (a, b) in (((px_ + g, pz_ - g), (px_ + g, pz_ + g)), ((px_ + g, pz_ + g), (px_ - g, pz_ + g)),
                   ((px_ - g, pz_ + g), (px_ - g, pz_ - g)), ((px_ - g, pz_ - g), (px_ + g, pz_ - g))):
        n = 12
        for k in range(0, n, 2):                                    # dashed: where the 4 foot brackets go
            p0 = (a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
            p1 = (a[0] + (b[0] - a[0]) * (k + 1) / n, a[1] + (b[1] - a[1]) * (k + 1) / n)
            d.line([px(*p0), px(*p1)], fill=0, width=3)
    u, v = px(px_ - g - 2, pz_ - g)
    d.text((u, v + 2 * PPM), "点線：柱の足の金具 4 個が乗る範囲（目安）", fill=0, font=f_s)
    u, v = px(X0 + 6, Z0 + 8)
    d.text((u, v - 7 * PPM), "↓ 後ろの端（x −80）。左の端は z −300", fill=0, font=f_m)
    u, v = px(X0 + BX - 4, Z0 + 8)
    d.text((u, v), "↑ 前（+x、柱の側）", fill=0, font=f_m)
    return img


def tiles():
    """page crops of the full picture: [(row, col, x_lo, z_lo)] in mm; rows from the rear edge, cols from the left edge."""
    uw, uh = A4[0] - 2 * MARGIN_MM, A4[1] - 2 * MARGIN_MM
    nc = max(1, math.ceil((BZ - uw) / (uw - OVERLAP_MM)) + 1)
    nr = max(1, math.ceil((BX - uh) / (uh - OVERLAP_MM)) + 1)
    sw = (BZ - uw) / (nc - 1) if nc > 1 else 0.0          # equal steps -> equal overlaps
    sh = (BX - uh) / (nr - 1) if nr > 1 else 0.0
    out = []
    for r in range(nr):
        for c in range(nc):
            out.append((r, c, X0 + r * sh, Z0 + c * sw))
    return out, (uw, uh), (nr, nc)


def overlaps():
    """real overlap between neighbouring sheets [mm]: (left-right, rear-front)."""
    ts, (uw, uh), (nr, nc) = tiles()
    sw = ts[1][3] - ts[0][3] if nc > 1 else uw
    sh = ts[nc][2] - ts[0][2] if nr > 1 else uh
    return uw - sw, uh - sh


def make_pages(full):
    ts, (uw, uh), (nr, nc) = tiles()
    PW, PH = int(round(A4[0] * PPM)), int(round(A4[1] * PPM))
    f = font(int(3.5 * PPM))
    ov = overlaps()
    pages = []
    for r, c, xl, zl in ts:
        u0, v0 = px(xl + uh, zl)                                    # top-left of this crop in the full picture
        crop = full.crop((int(round(u0)), int(round(v0)), int(round(u0)) + int(round(uw * PPM)), int(round(v0)) + int(round(uh * PPM))))
        page = Image.new("L", (PW, PH), 255)
        m = int(round(MARGIN_MM * PPM))
        page.paste(crop, (m, m))
        d = ImageDraw.Draw(page)
        d.rectangle([m, m, m + crop.size[0] - 1, m + crop.size[1] - 1], outline=0, width=2)
        d.text((m, 7 * PPM), f"D2 合板の型紙  {r * nc + c + 1}/{len(ts)}  （後ろから {r + 1} 段目・左から {c + 1} 列目）  100% で刷る",
               fill=0, font=f)
        y = PH - m + 3 * PPM                                       # 100 mm bars (check with a steel rule)
        d.line([(m, y), (m + 100 * PPM, y)], fill=0, width=4)
        for k in (0, 50, 100):
            d.line([(m + k * PPM, y - 1.5 * PPM), (m + k * PPM, y + 1.5 * PPM)], fill=0, width=3)
        d.text((m + 103 * PPM, y - 2 * PPM), "← 100 mm を鋼の定規で確かめる", fill=0, font=f)
        xb = PW - m + 3 * PPM
        for k in (0, 50, 100):
            d.line([(xb - 1.5 * PPM, m + k * PPM), (xb + 1.5 * PPM, m + k * PPM)], fill=0, width=3)
        d.line([(xb, m), (xb, m + 100 * PPM)], fill=0, width=4)
        d.text((m, 10.8 * PPM), f"この紙：x {xl:.0f}〜{xl + uh:.0f}・z {zl:.0f}〜{zl + uw:.0f}（mm）。左右の紙と {ov[0]:.0f}mm、前後の紙と {ov[1]:.0f}mm 重なる。"
               "右の縦の線も 100 mm",
               fill=0, font=f)
        pages.append(page)
    return pages, ts


def main():
    full = draw_full()
    pages, ts = make_pages(full)
    pdf = os.path.join(HERE, "template_D2_A4.pdf")
    pages[0].save(pdf, save_all=True, append_images=pages[1:], resolution=DPI)
    full.save(os.path.join(HERE, "template_D2_full.png"), dpi=(DPI, DPI))
    L = layout()
    print(f"wrote {pdf} ({len(pages)} pages, A4 portrait, 300 dpi) and template_D2_full.png")
    print(f"marks: pan axis x {L['pan'][0]:.1f}; pole ({L['pole'][0]:.1f}, {L['pole'][1]:.1f}); lens below ({L['lens'][0]:.1f}, {L['lens'][1]:.1f}); "
          f"base rear x {L['base_rear_x']}")


def selftest():
    fails = 0
    L = layout()
    # 1. the numbers in D2 section 2 (from the rear edge x -80 / from the left edge z -300)
    want = {"pan": (118.8, 300.0), "pole": (433.3, 319.5), "lens": (237.3, 319.5)}
    for k, (fr, fl) in want.items():
        got = (L[k][0] - X0, L[k][1] - Z0)
        ok = abs(got[0] - fr) < 0.05 and abs(got[1] - fl) < 0.05
        fails += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} {k}: from the rear {got[0]:.1f} (D2 {fr}), from the left {got[1]:.1f} (D2 {fl})")
    # 2. the pages put back at their places give the whole picture again (no gap, no shift between sheets)
    full = draw_full()
    pages, ts = make_pages(full)
    rebuilt = np.full((full.size[1], full.size[0]), -1, np.int16)
    m = int(round(MARGIN_MM * PPM))
    _, (uw, uh), _ = tiles()
    cw, chh = int(round(uw * PPM)), int(round(uh * PPM))
    bad = 0
    for pg, (r, c, xl, zl) in zip(pages, ts):
        u0, v0 = px(xl + uh, zl)
        a = np.asarray(pg)[m + 2:m + chh - 2, m + 2:m + cw - 2].astype(np.int16)   # inside the drawn page frame
        U, V = int(round(u0)) + 2, int(round(v0)) + 2
        prev = rebuilt[V:V + a.shape[0], U:U + a.shape[1]]
        bad += int(((prev >= 0) & (prev != a)).sum())
        rebuilt[V:V + a.shape[0], U:U + a.shape[1]] = a
    F = np.asarray(full).astype(np.int16)
    covered = rebuilt >= 0
    diff = int((rebuilt[covered] != F[covered]).sum())
    cov = covered[3:-3, 3:-3].mean()
    ok = bad == 0 and diff == 0 and cov > 0.999
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} {len(pages)} pages back together: overlaps disagree in {bad} px, differ from the whole in {diff} px, "
          f"cover {cov * 100:.2f} % of the board")
    # 3. page size and the 100 mm bar
    PW, PH = pages[0].size
    ok = abs(PW / PPM - A4[0]) < 0.2 and abs(PH / PPM - A4[1]) < 0.2
    fails += not ok
    a = np.asarray(pages[0])
    y = int(round(PH - m + 3 * PPM))
    row = np.where(a[y, : m + int(102 * PPM)] < 128)[0]                     # left of the bar label
    bar = (row.max() - row.min()) / PPM if row.size else 0
    ok2 = abs(bar - 100) < 0.5
    fails += not ok2
    print(f"  {'ok  ' if ok else 'FAIL'} page {PW / PPM:.1f} x {PH / PPM:.1f} mm at {DPI} dpi;  {'ok  ' if ok2 else 'FAIL'} the bottom bar is {bar:.1f} mm "
          "(ticks included; pass 100 +- 0.5)")
    # 4. a mark is where it should be in the whole picture: the pole centre cross is dark, a point 15 mm off on the diagonal is not
    u, v = px(*L["pole"])
    ok = F[int(round(v)), int(round(u + 15 * PPM))] < 100 and F[int(round(v + 15 * PPM)), int(round(u + 15 * PPM))] > 100   # on the cross arm off the z line
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} the pole centre cross is drawn at ({L['pole'][0]:.1f}, {L['pole'][1]:.1f})")
    # 5. the saved PDF: page count and page size in points (A4 = 595.3 x 841.9)
    import re
    pdf = os.path.join(HERE, "template_D2_A4.pdf")
    if os.path.exists(pdf):
        raw = open(pdf, "rb").read()
        boxes = [tuple(float(v) for v in b) for b in re.findall(rb"/MediaBox\s*\[\s*0\s+0\s+([\d.]+)\s+([\d.]+)\s*\]", raw)]
        ok = len(boxes) == len(ts) and all(abs(w - 595.3) < 0.5 and abs(h - 841.9) < 0.5 for w, h in boxes)
        fails += not ok
        print(f"  {'ok  ' if ok else 'FAIL'} the saved PDF has {len(boxes)} pages, MediaBox {boxes[0] if boxes else None} pt (A4 595.3 x 841.9)")
    else:
        print("  FAIL no PDF: run without selftest first"); fails += 1
    ov = overlaps()
    ok = min(ov) >= OVERLAP_MM
    fails += not ok
    print(f"  {'ok  ' if ok else 'FAIL'} overlaps: left-right {ov[0]:.1f} mm, rear-front {ov[1]:.1f} mm (all seams the same; at least {OVERLAP_MM})")
    print("SELFTEST", "OK" if not fails else f"FAILED ({fails})")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    selftest() if sys.argv[1:] == ["selftest"] else main()
