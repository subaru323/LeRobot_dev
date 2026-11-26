"""SO-101 パズルの動きを、物理エンジン MuJoCo で流す(計画書_完成形まで の S-15)。

sim/ の画面は「ピースは爪に貼り付いて動く」と決めて動きを組む(運動学だけ)。ここでは同じ関節の列を、
公式の MuJoCo の模型(models/src/so101_new_calib_camera.xml。関節の位置制御・関節の摩擦と遊び入り)に指令として渡し、
ピースを自由に動く剛体にして、本当につかめるか・運べるか・置いた所がずれないかを、重力と接触と摩擦で確かめる。

  読むもの:  out/physics/<名前>/scene.json(node tools/export-physics.js が書く)
  書くもの:  out/physics/<名前>/result.json(結果)、scene_mjcf.xml と assets/(場面を 1 つの MJCF にしたもの。
             MuJoCo では単独で開けるのを確かめた。Isaac Sim / Isaac Lab の MJCF の取り込みに渡す形だが、そちらで開けるかは確かめていない。
             noslip・solref などの MuJoCo の設定は PhysX には移らないので、同じ動きにはならない)、--snap で PNG。
             既定と違う値で流したとき(--hull・--friction など)は result_<条件>.json に書き、scene_mjcf.xml は書き換えない
  実行:      physics\\.venv\\Scripts\\python.exe physics\\run_physics.py out\\physics\\set2_gap15_stand1 [--view] [--snap 5,10]

★【仮】の値(実物で測って入れ替える): 摩擦 0.4(PLA どうし)、ピースの密度 1240 kg/m³(PLA)、つなぎの棒(6mm 角)、
  爪の挟む力 GRIP_NM(爪のサーボの力の上限。下の注)、接触の硬さ SOLREF。
★挟む力: 爪は位置制御で、指令をピースの幅より閉じる(SQUEEZE_DEG)と、ずれが 0.19° を超えた所で力の上限に張り付く。
  だから挟む力は角度でなく力の上限で決まる。公式の模型の上限 3.35 Nm(爪の先で 42N ほど)は、LeRobot が実機の爪を
  最大の 50%(負荷が続くと 25%)に絞るのより強い(MAIN_PG/lerobot/src/lerobot/robots/so_follower/so_follower.py)。
★模型の当たりの形は、MuJoCo が STL の凸包(へこみを埋めた形)で測る。爪の 2 つは小さく分けた(split_mesh)が、
  それでも面から外へ出る所が残る。ほかの腕と、台の L 字の内側などは、実物より太る。
  上のカメラの台は、見た目だけ(当たらない)にした(凸包だと柱の内側が埋まるため。台との近さは sim/ の判定で見ている)。
"""
import argparse, json, math, os, shutil, struct, sys, zlib
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
ROBOT_XML = os.path.join(SIM, "models", "src", "so101_new_calib_camera.xml")
STAND_DIR = os.path.join(SIM, "models", "src", "assets", "optional")

FRICTION = 0.4          # 【仮】PLA どうしの滑りの摩擦
DENSITY = 1240.0        # 【仮】PLA [kg/m³]
ROD = 0.006             # 【仮】すき間をつなぐ棒の太さ [m]
SQUEEZE_DEG = 8.0       # 持っている間、爪の指令を計画の角度よりこれだけ閉じる(0.19° を超えれば、力は GRIP_NM に張り付く)
GRIP_NM = 0.7           # 【仮】爪の力の上限 [Nm]。STS3215 の止まったときの力 1.9〜2.9 Nm(品の版による。確かめていない)の 25〜35% ほど。
                        #   LeRobot は爪を 50%、負荷が続くと 25% に絞る。公式の模型のまま(3.35 Nm)は --grip 3.35
NOSLIP = 10             # MuJoCo の noslip の反復の数。MuJoCo の接触は柔らかく、つかんだ物がゆっくり滑る(既定の 0 では、O を持つと 1 秒ほどで回って落ちた)。
                        #   つかむ場面ではこれで滑りを消すのが定石。0 にすると比べられる(遅さは 3 倍ほど)
SOLREF = 0.004          # 【仮】接触の硬さ(MuJoCo の solref の時定数 [秒]。刻み 0.001 秒の 2 倍より上)。MuJoCo の既定 0.02 は柔らかく、
                        #   強く挟むと爪がピースに 1.2〜1.35mm めり込み、置いた所が 3〜9mm ずれた。PLA は硬いので 0.004 にした(2026-11-17)
TORSION = 0.005         # 【仮】ねじりの摩擦 [m](接触の面の広さに比例する。爪の面 1 マスなら数 mm)
FINE = True             # 爪を細かく切った形で当てる(False = 公式の模型の凸包のまま。比べる用)
SETTLE = 0.5            # 動かす前に、ピースを机に落ち着かせる時間 [秒]
TAIL = 1.0              # 動きの終わりのあと流す時間 [秒]
TOL_MM, TOL_DEG = 3.0, 5.0   # 【仮】置いた所の合格: マスの真ん中のずれ・向きのずれ


def to_mj(p):
    """画面の座標 (Y が上) → MuJoCo (Z が上)"""
    return np.array([p[0], -p[2], p[1]])


def quat_z(deg):
    a = math.radians(deg) / 2
    return np.array([math.cos(a), 0, 0, math.sin(a)])


def read_stl(path):
    """2 進の STL → 三角形 (n, 3, 3)"""
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        raw = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
    return raw[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(float)


def _fib_dirs(n=256):
    i = np.arange(n) + 0.5
    z = 1 - 2 * i / n; r = np.sqrt(1 - z * z); ph = i * math.pi * (3 - math.sqrt(5))
    return np.stack([r * np.cos(ph), r * np.sin(ph), z], 1)


DIRS = _fib_dirs()


def subdivide(tri, max_edge):
    """辺が max_edge より長い三角形を、辺の中点で 4 つに割る(長い三角形が 1 つの升に入ったまま大きな凸包にならないように)"""
    out = []
    while len(tri):
        e = np.max(np.linalg.norm(tri - np.roll(tri, 1, axis=1), axis=2), axis=1)
        small = e <= max_edge
        out.append(tri[small])
        big = tri[~small]
        if not len(big):
            break
        a, b, c = big[:, 0], big[:, 1], big[:, 2]
        ab, bc, ca = (a + b) / 2, (b + c) / 2, (c + a) / 2
        tri = np.concatenate([np.stack([a, ab, ca], 1), np.stack([ab, b, bc], 1), np.stack([ca, bc, c], 1), np.stack([ab, bc, ca], 1)])
    return np.concatenate(out)


def split_mesh(path, vox, thick):
    """STL の三角形を vox の半分の辺まで割り、重心で vox の升に振り分け、升ごとの頂点 + 内向きに thick だけずらした頂点の組を返す
    (凸包を小さく分ける)。MuJoCo は 1 つのメッシュを凸包で当てるので、爪のくぼみが埋まる。小さく分けると、くぼみの多くが残る。
    ただし升の中で面が曲がる所・穴の縁では、凸包が面の外へ出る(点検 C23 の 3)。"""
    tri = subdivide(read_stl(path), vox / 2)
    e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    nrm = np.cross(e1, e2)
    ln = np.linalg.norm(nrm, axis=1)
    keep = ln > 1e-12
    tri, nrm = tri[keep], nrm[keep] / ln[keep, None]
    cen = tri.mean(axis=1)
    key = np.floor(cen / vox).astype(int)
    parts = {}
    for i, k in enumerate(map(tuple, key)):
        parts.setdefault(k, []).append(i)
    out = []
    for idx in parts.values():
        v = tri[idx].reshape(-1, 3)
        nn = np.repeat(nrm[idx], 3, axis=0)
        pts = np.vstack([v, v - nn * thick])          # 外向きの法線の逆(中)へ少し厚みを付ける(平らな面の凸包が潰れないように)
        pts = pts[np.unique(np.argmax(pts @ DIRS.T, axis=0))]   # 凸包に効く点だけ残す(256 の向きでいちばん外の点。凸包の差は 0.03mm ほど以下)
        if len(pts) >= 4:
            out.append(pts)
    return out


# 当たりの組の分け方(contype / conaffinity のビット): 1 = 腕の凸包どうし・机、2 = ピース・机・爪の細かい形
FINE_BODIES = ("gripper", "moving_jaw_so101_v1")
FINE_MESHES = ("wrist_roll_follower_so101_v1", "moving_jaw_so101_v1")
VOX = 0.004             # 【仮】爪を切る升 [m](腕の STL は m)
THICK = 0.0015          # 【仮】切った面に付ける厚み [m](薄いと、めり込みが厚みの半ばを越えたとき押し戻しの向きが裏返りうる)


def build(scene, out_dir, write_mjcf=True, extra=None):
    """extra (2026-11-24, tools/soma_world.py; optional, None = as before): called with the spec just before compiling (more bodies)"""
    spec = mujoco.MjSpec.from_file(ROBOT_XML)
    spec.option.timestep = 0.001
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10          # 滑りにくさを強める(つかむ場面の定石)
    spec.option.noslip_iterations = NOSLIP   # 接触の滑りを後から消す反復(0 = 使わない)
    base = spec.body("base")
    base.pos = [0, 0, scene["robotY"]]
    fine_src = []
    for g in list(spec.geoms):         # 腕の当たりの形の摩擦を【仮】の値に。爪の 2 つの凸包はピースに当てない(下の細かい形で当てる)
        if g.group == 3 and g.type == mujoco.mjtGeom.mjGEOM_MESH:
            g.friction = [FRICTION, TORSION, 0.0001]
            if FINE and g.meshname in FINE_MESHES:
                g.contype, g.conaffinity = 1, 1
                fine_src.append(g)
            else:
                g.contype, g.conaffinity = 3, 3
    nfine = 0
    for g in fine_src:
        body = g.parent
        for k, pts in enumerate(split_mesh(os.path.join(os.path.dirname(ROBOT_XML), "assets", g.meshname + ".stl"), VOX, THICK)):
            mn = "%s_part%d" % (g.meshname, k)
            spec.add_mesh(name=mn, uservert=pts.flatten().tolist())
            body.add_geom(name="fine_" + mn, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=mn, pos=g.pos, quat=g.quat,
                          contype=2, conaffinity=2, group=4, friction=[FRICTION, TORSION, 0.0001], rgba=[0.2, 0.8, 0.2, 0.5])
            nfine += 1
    scene["_nfine"] = nfine
    wb = spec.worldbody
    wb.add_light(pos=[0.2, 0, 1.0], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, diffuse=[0.7, 0.7, 0.7])
    wb.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0.6, 0.6, 0.01], contype=3, conaffinity=3,
                rgba=[0.82, 0.80, 0.76, 1], friction=[FRICTION, TORSION, 0.0001])
    # 上のカメラの台(見た目だけ)。STL は mm・Y が上 → X 軸まわりに +90° 回して Z を上に
    for s in scene.get("stand", []):
        if "lo" in s:  # 置き方 C の柱・吊る腕・吊る板・カメラ・足の金具(軸にそろった箱。見た目だけ。2026-11-19 から)
            c = [(s["lo"][i] + s["hi"][i]) / 2 for i in range(3)]
            h = [abs(s["hi"][i] - s["lo"][i]) / 2 for i in range(3)]
            wb.add_geom(name=s["id"], type=mujoco.mjtGeom.mjGEOM_BOX, pos=to_mj(c), size=[h[0], h[2], h[1]],
                        contype=0, conaffinity=0, group=1, rgba=[0.55, 0.55, 0.6, 1])
            continue
        if "a" in s:   # 置き方 C の柱・吊る腕・カメラ(線分と半径。見た目だけ)
            wb.add_geom(name=s["id"], type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=list(to_mj(s["a"])) + list(to_mj(s["b"])), size=[s["r"], 0, 0],
                        contype=0, conaffinity=0, group=1, rgba=[0.55, 0.55, 0.6, 1])
            continue
        spec.add_mesh(name="m_" + s["id"], file=os.path.join(STAND_DIR, s["mesh"] + ".stl"), scale=[0.001] * 3)
        wb.add_geom(name=s["id"], type=mujoco.mjtGeom.mjGEOM_MESH, meshname="m_" + s["id"],
                    pos=to_mj(s["pos"]), quat=[math.sqrt(0.5), math.sqrt(0.5), 0, 0],
                    contype=0, conaffinity=0, group=1, rgba=[0.55, 0.55, 0.6, 1])
    cell, P = scene["cell"], scene["pitch"]
    for pc in scene["pieces"]:
        st = pc["start"]
        col = [int(pc["color"][i:i + 2], 16) / 255 for i in (1, 3, 5)] + [1]
        b = wb.add_body(name="pc%d" % pc["idx"], pos=to_mj(st["pos"]) + [0, 0, cell / 2], quat=quat_z(-st["rot"]))
        b.add_freejoint(name="pc%d_free" % pc["idx"])
        cells = pc["cells"]
        for k, (a, bb) in enumerate(cells):   # 局所のマス (a, b) → MuJoCo の局所 (a·P, −b·P)
            b.add_geom(name="pc%d_c%d" % (pc["idx"], k), type=mujoco.mjtGeom.mjGEOM_BOX, size=[cell / 2] * 3,
                       pos=[a * P, -bb * P, 0], rgba=col, density=DENSITY, friction=[FRICTION, TORSION, 0.0001], condim=4, contype=2, conaffinity=2)
        gap = P - cell
        if gap > 1e-6:
            S = set(map(tuple, cells))
            for (a, bb) in cells:
                for (da, db) in ((1, 0), (0, 1)):
                    if (a + da, bb + db) in S:
                        mid = [(a + da / 2) * P, -(bb + db / 2) * P, 0]
                        half = [gap / 2 if da else ROD / 2, gap / 2 if db else ROD / 2, ROD / 2]
                        b.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=half, pos=mid, rgba=col, density=DENSITY,
                                   friction=[FRICTION, TORSION, 0.0001], condim=4, contype=2, conaffinity=2)
    if extra is not None:
        extra(spec)
    for g in spec.geoms:               # 接触の硬さ(ピースと、ピースに触れる形)
        if g.contype & 2:
            g.solref = [SOLREF, 1.0]
    for act in spec.actuators:         # 爪の力の上限(挟む力は、これで決まる)
        if act.name == "gripper":
            act.forcerange = [-GRIP_NM, GRIP_NM]
    model = spec.compile()
    if not write_mjcf:
        return model
    # 1 つの MJCF に書き出す(STL も横に写す。Isaac Sim の MJCF の取り込み・MuJoCo の画面で開ける)
    adir = os.path.join(out_dir, "assets")
    os.makedirs(adir, exist_ok=True)
    for m in spec.meshes:
        src = m.file if os.path.isabs(m.file) else os.path.join(os.path.dirname(ROBOT_XML), spec.meshdir or "", m.file)
        dst = os.path.join(adir, os.path.basename(src))
        if not os.path.exists(dst):
            shutil.copyfile(src, dst)
        m.file = os.path.basename(src)
    spec.meshdir = "assets"
    spec.modelfiledir = out_dir        # 書き出す MJCF から見た相対の場所で読み直す
    with open(os.path.join(out_dir, "scene_mjcf.xml"), "w", encoding="utf-8") as f:
        f.write(spec.to_xml())
    lic = os.path.join(STAND_DIR, "LICENSE_SO-ARM100.txt")
    if os.path.exists(lic):
        shutil.copyfile(lic, os.path.join(adir, "LICENSE_SO-ARM100.txt"))
    return model


def command(scene, t):
    """時刻 t の指令(関節 5 つ [rad]・爪 [rad]・持っているピース)。コマの間は線形に(sim/ の判定と同じ線)"""
    mo = scene["motion"]
    if t <= mo[0]["t"]:
        f = mo[0]; u = 0; g = f
    elif t >= mo[-1]["t"]:
        f = mo[-1]; u = 0; g = f
    else:
        lo, hi = 0, len(mo) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if mo[mid]["t"] <= t: lo = mid
            else: hi = mid
        f, g = mo[lo], mo[hi]
        u = (t - f["t"]) / max(1e-9, g["t"] - f["t"])
    q = [a + (b - a) * u for a, b in zip(f["q"], g["q"])]
    jaw = f["jaw"] + (g["jaw"] - f["jaw"]) * u
    held = g["held"] if u > 0 else f["held"]
    if held >= 0:
        jaw -= SQUEEZE_DEG * (1 if scene["jaw"]["grip"]["sign"] > 0 else -1)
    return np.radians(q), math.radians(jaw), held


def cell_centers_expected(scene, pc, pose):
    P, cell = scene["pitch"], scene["cell"]
    r = math.radians(pose["rot"]); c, s = math.cos(r), math.sin(r)
    out = []
    for a, b in pc["cells"]:
        x, z = a * P, b * P
        out.append(to_mj([pose["pos"][0] + x * c - z * s, pose["pos"][1] + cell / 2, pose["pos"][2] + x * s + z * c]))
    return np.array(out)


def cell_centers_actual(scene, pc, data):
    P = scene["pitch"]
    body = data.body("pc%d" % pc["idx"])
    R = body.xmat.reshape(3, 3)
    return np.array([body.xpos + R @ np.array([a * P, -b * P, 0]) for a, b in pc["cells"]]), R


def yaw_of(R):
    """MuJoCo の回転 → 画面の向き ψ[度](上から見て X → Z。MuJoCo では Z まわりに −ψ)"""
    return -math.degrees(math.atan2(R[1, 0], R[0, 0]))


def write_png(path, rgb):
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))
    def chunk(t, d): return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) +
                chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--view", action="store_true", help="MuJoCo の画面で、実時間で見る")
    ap.add_argument("--snap", default="", help="この時刻[秒]の絵を PNG に(カンマ区切り)")
    ap.add_argument("--squeeze", type=float, default=None)
    ap.add_argument("--friction", type=float, default=None)
    ap.add_argument("--noslip", type=int, default=None)
    ap.add_argument("--torsion", type=float, default=None)
    ap.add_argument("--solref", type=float, default=None)
    ap.add_argument("--grip", type=float, default=None, help="爪の力の上限 [Nm]")
    ap.add_argument("--hull", action="store_true", help="爪を公式の模型の凸包のまま当てる(比べる用)")
    a = ap.parse_args()
    global SQUEEZE_DEG, FRICTION, FINE, NOSLIP, TORSION, SOLREF, GRIP_NM
    if a.grip is not None: GRIP_NM = a.grip
    # 既定と違う値で流すときは、結果を別の名前に書き、Isaac Sim に渡す MJCF は書き換えない(点検 C23 の 7)
    tag = "_".join(k + str(v) for k, v in (("hull", 1 if a.hull else None), ("friction", a.friction), ("squeeze", a.squeeze), ("noslip", a.noslip),
                                           ("torsion", a.torsion), ("solref", a.solref), ("grip", a.grip)) if v is not None)
    FINE = not a.hull
    if a.noslip is not None: NOSLIP = a.noslip
    if a.torsion is not None: TORSION = a.torsion
    if a.solref is not None: SOLREF = a.solref
    if a.squeeze is not None: SQUEEZE_DEG = a.squeeze
    if a.friction is not None: FRICTION = a.friction
    scene = json.load(open(os.path.join(a.dir, "scene.json"), encoding="utf-8"))
    model = build(scene, a.dir, write_mjcf=not tag)
    data = mujoco.MjData(model)
    ref = mujoco.MjData(model)           # 指令どおりの姿勢(運動学)。追い遅れを測る
    jn = scene["joints"]
    jid = [model.joint(n).qposadr[0] for n in jn] + [model.joint("gripper").qposadr[0]]
    q0, jaw0, _ = command(scene, 0)
    for k, v in zip(jid, list(q0) + [jaw0]):
        data.qpos[k] = v
    data.ctrl[:] = list(q0) + [jaw0]
    mujoco.mj_forward(model, data)
    site = model.site("gripperframe").id
    pieces = scene["pieces"]
    picks = {s["pid"]: s for s in scene["steps"]}
    T = scene["T"]
    snaps = sorted(float(x) for x in a.snap.split(",") if x.strip())
    renderer = None
    if snaps:
        renderer = mujoco.Renderer(model, 480, 640)
        cam = mujoco.MjvCamera(); cam.lookat[:] = [0.17, 0.0, 0.04]; cam.distance = 0.42; cam.azimuth = 215; cam.elevation = -32
    viewer = None
    if a.view:
        import mujoco.viewer as mjv
        viewer = mjv.launch_passive(model, data)
    # 追い遅れ・ピースの記録
    track = {"joint_max_deg": [0.0] * 5, "tcp_max_mm": 0.0, "tcp_at": 0.0}
    start_pos = {pc["idx"]: cell_centers_actual(scene, pc, data)[0] for pc in pieces}
    pre_move = {pc["idx"]: 0.0 for pc in pieces}       # つかむ前に、ほかの物に押されて動いた量
    lift = {pc["idx"]: 0.0 for pc in pieces}           # 持っている間に、机から上がった高さ
    slip = {pc["idx"]: 0.0 for pc in pieces}           # 持っている間に、爪に対して動いた量
    rel0 = {}
    contacts_bad = {}
    t_end = SETTLE + T + TAIL
    n = int(round(t_end / model.opt.timestep))
    import time
    wall0 = time.time()
    for i in range(n):
        t = i * model.opt.timestep
        tm = t - SETTLE
        q, jaw, held = command(scene, max(0.0, tm))
        data.ctrl[:] = list(q) + [jaw]
        mujoco.mj_step(model, data)
        if viewer is not None:
            viewer.sync()
            lag = (time.time() - wall0) - t
            if lag < 0: time.sleep(-lag)
            if not viewer.is_running(): break
        if snaps and tm >= snaps[0] - 1e-9:
            renderer.update_scene(data, cam)
            write_png(os.path.join(a.dir, "snap_%s%05.2f.png" % (tag + "_" if tag else "", snaps[0])), renderer.render())
            snaps.pop(0)
        if i % 10 or tm < 0:
            continue
        # 追い遅れ
        for k in range(5):
            track["joint_max_deg"][k] = max(track["joint_max_deg"][k], abs(math.degrees(data.qpos[jid[k]] - q[k])))
        for k, v in zip(jid, list(q) + [jaw]):
            ref.qpos[k] = v
        mujoco.mj_kinematics(model, ref)
        e = np.linalg.norm(data.site_xpos[site] - ref.site_xpos[site]) * 1000
        if e > track["tcp_max_mm"]:
            track["tcp_max_mm"], track["tcp_at"] = e, tm
        # ピース
        g_pos = data.site_xpos[site]; g_R = data.site_xmat[site].reshape(3, 3)
        for pc in pieces:
            idx = pc["idx"]
            cs, R = cell_centers_actual(scene, pc, data)
            st = picks.get(idx)
            if st and tm < st["tPick"] - 0.45:      # 爪を閉じ始める前まで(横に寄せて固定の爪を当てる計画の動きは入る)
                pre_move[idx] = max(pre_move[idx], float(np.max(np.linalg.norm(cs - start_pos[idx], axis=1))) * 1000)
            if held == idx:
                lift[idx] = max(lift[idx], float(np.min(cs[:, 2]) - scene["cell"] / 2) * 1000)
                if st and tm < st["tPick"] + 0.15:      # 握り込みが効くまでは基準にしない(締めた動きを滑りに入れない)
                    continue
                rel = g_R.T @ (cs - g_pos).T            # すべてのマスの真ん中(回る分も入る)
                if idx not in rel0:
                    rel0[idx] = rel
                slip[idx] = max(slip[idx], float(np.max(np.linalg.norm(rel - rel0[idx], axis=0))) * 1000)
        # 腕(爪の細かい形のほか)がピースに触れたか。形の名前で外す(爪の体にある手首のサーボの凸包は数える)
        for c in data.contact[:data.ncon]:
            for ga, gb in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                pn = model.body(model.geom_bodyid[ga]).name
                if not pn.startswith("pc"):
                    continue
                ob, gname = model.body(model.geom_bodyid[gb]).name, model.geom(gb).name or ""
                if ob == "world" or ob.startswith("pc") or gname.startswith("fine_"):
                    continue
                key = pn + "|" + ob + ":" + (gname or "geom%d" % gb)
                if key not in contacts_bad:
                    contacts_bad[key] = round(tm, 2)
    if viewer is not None:
        viewer.close()
    # 置いた所
    res_p = []
    worst = 0.0
    ok_all = True
    for pc in pieces:
        idx = pc["idx"]
        exp = cell_centers_expected(scene, pc, pc["end"] or pc["start"])
        act, R = cell_centers_actual(scene, pc, data)
        e = float(np.max(np.linalg.norm((act - exp)[:, :2], axis=1))) * 1000
        dz = float(np.max(np.abs(act[:, 2] - exp[:, 2]))) * 1000
        yaw_err = ((yaw_of(R) - (pc["end"] or pc["start"])["rot"] + 180) % 360) - 180
        tilt = math.degrees(math.acos(max(-1, min(1, R[2, 2]))))
        touched = any(k.startswith("pc%d|" % idx) for k in contacts_bad)
        ok = e < TOL_MM and abs(yaw_err) < TOL_DEG and tilt < 5 and not touched
        ok_all &= ok
        worst = max(worst, e)
        res_p.append({"name": pc["name"], "ok": ok, "place_err_mm": round(e, 2), "height_err_mm": round(dz, 2),
                      "yaw_err_deg": round(yaw_err, 2), "tilt_deg": round(tilt, 2), "lift_mm": round(lift[idx], 1),
                      "slip_mm": round(slip[idx], 2), "pushed_before_pick_mm": round(pre_move[idx], 2), "arm_touched": touched})
    out = {"label": scene["label"], "ok": ok_all, "tag": tag,
           "note": "置いた所のずれ = 物理の結果 − 運動学の sim の最後の置き方(盤の目標との差ではない。sim の置いた位置の差は check-scene.js で 0.2mm 未満)", "worst_place_mm": round(worst, 2),
           "params": {"friction": FRICTION, "torsion": TORSION, "solref": SOLREF, "noslip": NOSLIP, "fine_jaw": FINE, "density": DENSITY, "rod_m": ROD, "squeeze_deg": SQUEEZE_DEG, "grip_nm": GRIP_NM, "thick_m": THICK, "vox_m": VOX, "timestep": model.opt.timestep,
                      "tol_mm": TOL_MM, "tol_deg": TOL_DEG},
           "tracking": {"joint_max_deg": [round(v, 2) for v in track["joint_max_deg"]], "tcp_max_mm": round(track["tcp_max_mm"], 2), "tcp_at_s": round(track["tcp_at"], 2)},
           "pieces": res_p, "arm_touched_piece": contacts_bad, "kinematic": scene.get("kinematic")}
    json.dump(out, open(os.path.join(a.dir, "result%s.json" % ("_" + tag if tag else "")), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("[%s]%s %s  置いた所のずれ 最大 %.2fmm  追い遅れ: 関節 最大 %s°・爪の先 最大 %.1fmm(%.1f 秒)" % (
        scene["label"], "(" + tag + ")" if tag else "", "OK" if ok_all else "★NG", worst, "/".join("%.1f" % v for v in track["joint_max_deg"]), track["tcp_max_mm"], track["tcp_at"]))
    for r in res_p:
        print("   %s %s ずれ %.2fmm・高さ %.2fmm・向き %+.2f°・傾き %.1f°・持ち上げ %.1fmm・爪の中で滑った %.2fmm・つかむ前に押された %.2fmm" % (
            r["name"], "OK" if r["ok"] else "★NG", r["place_err_mm"], r["height_err_mm"], r["yaw_err_deg"], r["tilt_deg"], r["lift_mm"], r["slip_mm"], r["pushed_before_pick_mm"]))
    if contacts_bad:
        print("   爪のほかの腕がピースに触れた: " + "、".join("%s(%.2f 秒)" % (k, v) for k, v in contacts_bad.items()))
    print("   (%.1f 秒で流した)" % (time.time() - wall0))
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
