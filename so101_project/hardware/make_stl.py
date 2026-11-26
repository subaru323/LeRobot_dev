r"""Write simple STL parts for this project (no CAD library: boxes only, with numpy).

usage: python hardware\make_stl.py        (any python with numpy)
Writes hardware/stl/*.stl (units mm).

Parts:
  gauge_10mm / gauge_20mm / gauge_30mm : blocks of a known width, to hold in the gripper in tools/k1_calib.py
      (docs/利用者の日/D1). The width is the X size. Measure it after printing (printers are off by 0.1-0.3mm)
      and type the measured value.
"""

import os
import struct

import numpy as np

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")


def box(x0, y0, z0, x1, y1, z1):
    """12 triangles of an axis-aligned box, outward normals."""
    v = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                  [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]], float)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4),
         (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return [v[list(t)] for t in f]


def write_stl(name, tris):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name + ".stl")
    with open(path, "wb") as fh:
        fh.write(name.encode()[:80].ljust(80, b" "))
        fh.write(struct.pack("<I", len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            n = n / (np.linalg.norm(n) or 1)
            fh.write(struct.pack("<12fH", *n, *t[0], *t[1], *t[2], 0))
    lo = np.min([t.min(0) for t in tris], 0)
    hi = np.max([t.max(0) for t in tris], 0)
    print(f"{path}: {len(tris)} triangles, size {np.round(hi - lo, 2)} mm")


def main():
    for w in (10, 20, 30):
        # width w (X), 15mm deep (Y), 30mm tall (Z): lies flat on the bed, the jaws close on the X faces
        write_stl(f"gauge_{w}mm", box(0, 0, 0, w, 15, 30))


if __name__ == "__main__":
    main()
