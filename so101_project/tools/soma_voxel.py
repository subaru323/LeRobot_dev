"""Voxelize the STL pieces in パズル/ on a 40 mm grid (ray parity, numpy). The STLs are hollow shells (1 mm wall): only the
outer surface (on the 40 mm grid) is used. usage: python tools/soma_voxel.py パズル > outputs/soma/voxels.json"""
import glob
import json
import os
import struct
import sys

import numpy as np

D = sys.argv[1]
CELL = 40.0


def load(f):
    b = open(f, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("x", "<u2")]))
    T = a["v"].reshape(n, 3, 3).astype(float)
    outer = (np.abs(T / CELL - np.round(T / CELL)) < 1e-3).all(axis=(1, 2))   # the 1 mm inner shell is not on the 40 mm grid
    return T[outer]


def inside(T, p):
    d = np.array([1.0, 0.000317, 0.000731])
    a, b, c = T[:, 0], T[:, 1], T[:, 2]
    e1, e2 = b - a, c - a
    h = np.cross(d, e2)
    det = (e1 * h).sum(1)
    ok = np.abs(det) > 1e-12
    s = p - a
    u = (s * h).sum(1) / np.where(ok, det, 1)
    q = np.cross(s, e1)
    v = (q @ d) / np.where(ok, det, 1)
    t = (e2 * q).sum(1) / np.where(ok, det, 1)
    hit = ok & (u >= 0) & (u <= 1) & (v >= 0) & (u + v <= 1) & (t > 0)
    return int(hit.sum()) % 2 == 1


files = sorted(glob.glob(os.path.join(D, "*.stl")), key=lambda f: (len(f), f))
allT = [load(f) for f in files]
lo = np.min([T.reshape(-1, 3).min(0) for T in allT], axis=0)
hi = np.max([T.reshape(-1, 3).max(0) for T in allT], axis=0)
n = np.round((hi - lo) / CELL).astype(int)
out = {}
for f, T in zip(files, allT):
    cells = []
    for i in range(n[0]):
        for j in range(n[1]):
            for k in range(n[2]):
                p = lo + CELL * (np.array([i, j, k]) + 0.5)
                if inside(T, p):
                    cells.append([i, j, k])
    out[os.path.basename(f)] = cells
print(json.dumps({"lo": lo.tolist(), "hi": hi.tolist(), "n": n.tolist(), "pieces": out}, ensure_ascii=False))
