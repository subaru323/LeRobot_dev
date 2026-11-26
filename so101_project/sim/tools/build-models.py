"""
build-models.py — 公式の SO-101 の形(STL)と URDF から、画面と当たり判定が読む JS を作る

  入力: models/src/so101_new_calib_camera.urdf と models/src/assets/*.stl
        (TheRobotStudio/SO-ARM100 の Simulation/SO101。2026-11-12 に取得、2026-11-16 に丸ごと取り直した)
  出力: models/so101_meshes.js  … 画面の形(MESHES。ちくわの画面と同じ形式: 16bit 量子化)
        models/so101_points.js  … 当たり判定の点(各 STL の表面を約 3mm 間隔でならした点)
        models/so101_visuals.js … どのリンクにどの STL をどこへ置くか(URDF の <visual> の写し)
        models/so101_tris.js    … 上のカメラの台の三角形(当たり判定で、腕の点から台の面までの距離を測る)と、
                                  爪のまわりの形(GRIP_TRIS)の三角形(ピースが近い所で、距離を三角形で測り直す。S-8)

  ★形の持ち主は STL。絵(MESHES)と当たり判定(点)は同じ STL から同じ回に作る(ちくわの 0-E)。
  実行: python tools/build-models.py   (sim/ で)
"""
import base64
import json
import os
import re
import struct
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "models", "src")
OUT = os.path.join(ROOT, "models")
POINT_STEP = 0.003        # 当たり判定の点の間隔[m]
CREASE_DEG = 35           # これより折れている角は、法線を分ける(角を丸めない)

# URDF に無い形(上のカメラの台。本家の Optional/Overhead_Cam_Mount_32x32_UVC_Module、単位 mm)。
# 置き場所は so101.js の OVERHEAD が持つ(ここは形だけ)
EXTRA = {"overhead_arm_base": "optional/overhead_arm_base.stl",
         "overhead_cam_mount_bottom": "optional/overhead_cam_mount_bottom.stl",
         "overhead_cam_mount_middle": "optional/overhead_cam_mount_middle.stl",
         "overhead_cam_mount_top": "optional/overhead_cam_mount_top.stl"}
EXTRA_SCALE = 0.001
# 三角形も書き出す腕の形(計画書_完成形まで の S-8: 爪の近くで、ピースまでの距離を三角形で測り直す)。
# 固定の爪・動く爪・手首の回転のモーター(爪の付け根に付く)・手首のカメラの台とカメラ
# 2026-11-17: 腕どうしの当たり(S-10)のため、URDF に置く腕の形は全部書き出すことにした(GRIP_TRIS は爪のまわりの印として残す)
GRIP_TRIS = {"wrist_roll_follower_so101_v1", "moving_jaw_so101_v1", "sts3215_03a_v1",
             "wrist_camera_mount_so101_v1", "wrist_camera_so101_v1"}

COLORS = {                # 絵の色(印刷の部品は白、モーターは黒、基板は緑、上のカメラの台は黄と黒)
    "overhead_arm_base": "#c8d84a", "overhead_cam_mount_bottom": "#c8d84a",
    "overhead_cam_mount_middle": "#2b2d33", "overhead_cam_mount_top": "#2b2d33",
    "sts3215_03a_v1": "#1d2127", "sts3215_03a_no_horn_v1": "#1d2127",
    "waveshare_mounting_plate_so101_v2": "#2f6b3a",
}
DEFAULT_COLOR = "#eeeeea"


def read_stl(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    if len(b) != 84 + 50 * n:
        raise ValueError("バイナリの STL ではない: " + path)
    dt = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    tri = np.frombuffer(b, dtype=dt, count=n, offset=84)
    return tri["v"].astype(np.float64)          # (n, 3, 3)


def weld(V):
    """角を丸めすぎない溶接。戻り値 pos(m,3), nor(m,3), idx(n*3)"""
    n = len(V)
    e1, e2 = V[:, 1] - V[:, 0], V[:, 2] - V[:, 0]
    fn = np.cross(e1, e2)
    area2 = np.linalg.norm(fn, axis=1)
    fnu = fn / np.maximum(area2[:, None], 1e-18)
    corners = V.reshape(-1, 3)
    cfn = np.repeat(fnu, 3, axis=0)
    cw = np.repeat(area2, 3)
    q = np.round(corners / 1e-6).astype(np.int64)
    _, pid = np.unique(q, axis=0, return_inverse=True)
    pid = pid.ravel()
    sm = np.zeros((pid.max() + 1, 3))
    np.add.at(sm, pid, cfn * cw[:, None])
    smu = sm / np.maximum(np.linalg.norm(sm, axis=1)[:, None], 1e-18)
    ok = (cfn * smu[pid]).sum(1) >= np.cos(np.radians(CREASE_DEG))
    # 折れている角は (位置, 面の法線の向きの粗い区分) で別の頂点にする
    fq = np.round(cfn * 3).astype(np.int64)
    key = np.stack([pid, np.where(ok, 9, fq[:, 0]), np.where(ok, 9, fq[:, 1]), np.where(ok, 9, fq[:, 2])], 1)
    _, vid = np.unique(key, axis=0, return_inverse=True)
    vid = vid.ravel()
    m = vid.max() + 1
    pos = np.zeros((m, 3)); pos[vid] = corners
    nor = np.zeros((m, 3)); np.add.at(nor, vid, cfn * cw[:, None])
    nor /= np.maximum(np.linalg.norm(nor, axis=1)[:, None], 1e-18)
    return pos, nor, vid


def surface_points(V, step):
    """三角形の表面に step 間隔ほどで点を打ち、step の格子で 1 つにならす"""
    pts = [V.reshape(-1, 3)]
    e1, e2 = V[:, 1] - V[:, 0], V[:, 2] - V[:, 0]
    L = np.maximum(np.linalg.norm(e1, axis=1), np.linalg.norm(e2, axis=1))
    k = np.ceil(L / step).astype(int)
    for kk in np.unique(k):
        if kk < 2:
            continue
        sel = V[k == kk]
        a, b = np.meshgrid(np.arange(kk + 1), np.arange(kk + 1))
        m = (a + b) <= kk
        u, w = a[m] / kk, b[m] / kk
        p = sel[:, 0][:, None, :] + u[None, :, None] * (sel[:, 1] - sel[:, 0])[:, None, :] \
            + w[None, :, None] * (sel[:, 2] - sel[:, 0])[:, None, :]
        pts.append(p.reshape(-1, 3))
    P = np.concatenate(pts)
    g = np.floor(P / step).astype(np.int64)
    _, first = np.unique(g, axis=0, return_index=True)
    return P[np.sort(first)]


def b64(a):
    return base64.b64encode(a.tobytes()).decode("ascii")


def parse_urdf(path):
    s = open(path, encoding="utf-8").read()
    vis = {}
    for m in re.finditer(r'<link\s+name="([^"]+)"\s*>(.*?)</link>', s, re.S):
        items = []
        for v in re.finditer(r"<visual>(.*?)</visual>", m.group(2), re.S):
            o = re.search(r'<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"', v.group(1))
            f = re.search(r'filename="(?:package://)?(?:assets/)?([^"/]+)\.stl"', v.group(1))
            sc = re.search(r'<mesh[^>]*scale="([^"]+)"', v.group(1))
            it = {"mesh": f.group(1), "xyz": [float(x) for x in o.group(1).split()],
                  "rpy": [float(x) for x in o.group(2).split()]}
            if sc:  # 単位が mm の STL(手首のカメラの台)。絵と点を作るときに m に直す
                it["scale"] = [float(x) for x in sc.group(1).split()]
            items.append(it)
        vis[m.group(1)] = items
    return vis


def main():
    # 手首のカメラ付きの URDF(本体の URDF に 2 リンクを足しただけ)。パズルの画面は LINK_GROUP に
    # 無いリンク(カメラ)を使わないので、本体だけのときと同じ結果になる
    vis = parse_urdf(os.path.join(SRC, "so101_new_calib_camera.urdf"))
    scale = {}
    for v in vis.values():
        for it in v:
            if "scale" in it:
                scale[it["mesh"]] = np.array(it.pop("scale"))
    names = sorted({it["mesh"] for v in vis.values() for it in v})
    mesh_js = ["// generated by tools/build-models.py from SO-ARM100 Simulation/SO101 (STL) -- 手で直さない",
               "var MESHES = (typeof MESHES !== 'undefined') ? MESHES : {};"]
    points = {}
    tris = {}
    report = []
    for nm in names + sorted(EXTRA):
        if nm in EXTRA:
            V = read_stl(os.path.join(SRC, "assets", EXTRA[nm])) * EXTRA_SCALE
        else:
            V = read_stl(os.path.join(SRC, "assets", nm + ".stl"))
        if nm in scale:
            V = V * scale[nm]
        pos, nor, idx = weld(V)
        lo, hi = pos.min(0), pos.max(0)
        sc = np.maximum((hi - lo) / 65535.0, 1e-12)
        qp = np.round((pos - lo) / sc).astype("<u2")
        qn = np.clip(np.round(nor * 127), -127, 127).astype("<i1")
        big = pos.shape[0] > 65535
        ii = idx.astype("<u4" if big else "<u2")
        g = {"c": COLORS.get(nm, DEFAULT_COLOR), "q": [float(x) for x in lo] + [float(x) for x in sc],
             "i32": bool(big), "p": b64(qp), "n": b64(qn), "i": b64(ii)}
        mesh_js.append("MESHES[%s] = {bbox:%s,groups:[%s]};" % (
            json.dumps(nm), json.dumps([round(float(x), 5) for x in list(lo) + list(hi)]), json.dumps(g)))
        P = surface_points(V, POINT_STEP)
        if nm in EXTRA or nm in GRIP_TRIS or nm in names:   # 腕の形は全部(腕どうしの当たり = 自分との干渉を三角形で測る。S-10)
            tris[nm] = np.round(V / 1e-4).astype(int).ravel().tolist()     # 0.1mm 単位・9 個で 1 つの三角形(STL の巻き順のまま)
        points[nm] = np.round(P / 1e-4).astype(int).ravel().tolist()     # 0.1mm 単位
        report.append("  %-36s 三角形 %6d → 頂点 %6d / 判定の点 %5d / 大きさ %s mm" % (
            nm, len(V), pos.shape[0], len(P), "×".join("%.0f" % ((hi - lo)[k] * 1000) for k in range(3))))

    def write(path, text):
        data = text.encode("utf-8")
        tmp = path + ".tmp"
        with open(tmp, "wb") as f:
            f.write(data)
        os.replace(tmp, path)
        return os.path.getsize(path)

    s1 = write(os.path.join(OUT, "so101_meshes.js"), "\n".join(mesh_js) + "\n")
    s2 = write(os.path.join(OUT, "so101_points.js"),
               "// generated by tools/build-models.py -- 手で直さない。当たり判定の点(0.1mm 単位・STL の座標)\n"
               "var SO101_POINTS = " + json.dumps(points, separators=(",", ":")) + ";\n"
               "if (typeof module !== 'undefined') module.exports = SO101_POINTS;\n")
    s3 = write(os.path.join(OUT, "so101_visuals.js"),
               "// generated by tools/build-models.py -- 手で直さない。URDF の <visual> の写し\n"
               "var SO101_VISUALS = " + json.dumps(vis, ensure_ascii=False, indent=1) + ";\n"
               "if (typeof module !== 'undefined') module.exports = SO101_VISUALS;\n")
    s4 = write(os.path.join(OUT, "so101_tris.js"),
               "// generated by tools/build-models.py -- 手で直さない。上のカメラの台と、爪のまわりの形の三角形(0.1mm 単位・STL の座標・9 個で 1 つ)\n"
               "var SO101_TRIS = " + json.dumps(tris, separators=(",", ":")) + ";\n"
               "if (typeof module !== 'undefined') module.exports = SO101_TRIS;\n")
    print("[build-models] STL %d 個(URDF のリンク %d 個に置く)+ 上のカメラの台 %d 個" % (len(names), len(vis), len(EXTRA)))
    print("\n".join(report))
    print("  出力: so101_meshes.js %.1f MB / so101_points.js %.1f MB / so101_visuals.js %d B / so101_tris.js %.1f MB" % (s1 / 1e6, s2 / 1e6, s3, s4 / 1e6))


if __name__ == "__main__":
    sys.exit(main())
