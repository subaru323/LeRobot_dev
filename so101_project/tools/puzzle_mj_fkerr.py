"""列 P 3 回目(2026-11-20、点検 C42g): 関節の誤差(--arm-err)の回で、置いた所のずれを FK で分ける(読むだけの道具)。

  sim\\physics\\.venv\\Scripts\\python.exe tools\\puzzle_mj_fkerr.py RUN.json [RUN.json ...] [--out DIR]

  RUN.json は tools/puzzle_mj.py の記録(arm_bias_deg・commands・observations・truth・scene・final_states が要る)。
  制御には何も渡さない(終わった回の記録を読むだけ)。真値を使ってよいのは、この道具が採点の側だから。

何を出すか(1 ピースごと)
  - 3 つの姿勢(確かめ = つかむ前の最後の手首の絵・つかむ = held がそのピースになった最初のコマ・置く = held の最後のコマ)で、
    爪の基準点(MuJoCo の site "gripperframe")の FK(読み + bias) − FK(読み)。横(画面の x, z)のずれ [mm]・高さ [mm]・
    上から見た向きのずれ [度]。
  - 置いた所の見込み(持っている間はピースが爪に固く付いている、とした剛体の見込み。T = 爪の 3 次元の姿勢):
      T_a(置く) T_b(つかむ)^-1 を、最初の真値のピースのマスの真ん中に当て、盤の目標のマスと比べる。
        est   : a = 読み, b = 読み   … 見積もり(手首で直した後)の誤りだけ
        grasp : a = 読み, b = 本当   … つかむ姿勢の腕のずれだけを足す
        place : a = 本当, b = 読み   … 置く姿勢の腕のずれだけを足す
        full  : a = 本当, b = 本当   … 全部(剛体の見込み)
      これを MuJoCo の本当の置いた所(final_states)と比べる。差が大きければ、剛体でない分(爪が閉じるときにピースを寄せる・
      置くときに滑る)が効いている。
    ずれは、いちばん悪いマスの真ん中(採点と同じ、マスの組どうしで近い物)と、マスの真ん中の平均のずれ (dx, dz)。
  - 手首の確かめの FK の見込み: 確かめの姿勢の T_r T_t^-1 を真値に当てた物(カメラが腕と一緒にずれる、とした見込み。
    机の面との交わりは無視)を、見積もりの誤り(est の項)と比べる。
"""

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "sim", "physics"))
import numpy as np                     # noqa: E402
import mujoco                          # noqa: E402
import run_physics as RP               # noqa: E402

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def cmd_times(run, ts):
    """各コマを送り終えた世界の時刻(puzzle_mj.World._run と同じ刻み)"""
    t = max(1, int(round(0.5 / ts))) * ts
    out = []
    for c in run["commands"]:
        t += max(1, int(round(max(c["dt"], ts) / ts))) * ts
        out.append(t)
    return out


def pose(model, data, site, jid, q, jaw):
    for k, v in zip(jid, list(np.radians(q)) + [math.radians(jaw)]):
        data.qpos[k] = v
    mujoco.mj_kinematics(model, data)
    return data.site_xpos[site].copy(), data.site_xmat[site].reshape(3, 3).copy()


def screen(p):
    """MuJoCo → 画面の (x, z) [mm]"""
    return np.array([p[0] * 1000, -p[1] * 1000])


def yaw_delta(Rt, Rr):
    """上から見た向きのずれ [度](画面の向き ψ。MuJoCo の z まわりの回転の逆の符号)"""
    dR = Rt @ Rr.T
    return -math.degrees(math.atan2(dR[1, 0] - dR[0, 1], dR[0, 0] + dR[1, 1]))


def cells_true(scene, pc, t):
    P = scene["pitch"]
    r = math.radians(t["yaw"]); c, s = math.cos(r), math.sin(r)
    return np.array([RP.to_mj([t["x"] / 1000 + a * P * c - b * P * s, scene["cell"] / 2, t["z"] / 1000 + a * P * s + b * P * c])
                     for a, b in pc["cells"]])


def compare(cells_mj, target_mj):
    a = np.array([screen(p) for p in cells_mj]); b = np.array([screen(p) for p in target_mj])
    d = np.linalg.norm(b[:, None, :] - a[None, :, :], axis=2)
    nn = np.argmin(d, axis=1)
    worst = float(np.max(d[np.arange(len(b)), nn]))
    mean = (a[nn] - b).mean(axis=0)
    return worst, mean


GRASP_BACK = 0


def analyze(path, model, data, site, jid):
    run = json.load(open(path, encoding="utf-8"))
    sc = run["scene"]
    bias = run.get("arm_bias_deg") or [0.0] * 5
    ts = model.opt.timestep
    tc = cmd_times(run, ts)
    wr = [o["t_world"] for o in run["observations"] if o["cam"] == "wrist"]
    wr_idx = [int(np.argmin([abs(t - w) for t in tc])) for w in wr]
    for w, k in zip(wr, wr_idx):
        assert abs(tc[k] - w) < 2e-3, (w, tc[k])
    cmds = run["commands"]
    out = {"run": path, "seed": run["args"]["seed"], "arm_bias_deg": [round(b, 3) for b in bias], "pieces": []}
    for pc, tr, fs in zip(sc["pieces"], run["truth"]["pieces"], run["final_states"]):
        pid = pc["idx"]
        hk = [i for i, c in enumerate(cmds) if c["held"] == pid]
        if not hk:
            continue
        p_idx = hk[-1]
        g_idx = p_idx
        while g_idx - 1 >= 0 and cmds[g_idx - 1]["held"] == pid:
            g_idx -= 1
        g_idx = max(0, g_idx - GRASP_BACK)   # --grasp-back N: つかむ姿勢を N コマ前に(1 = 爪が閉じ終えて持ち上げる前。点検 C42g の 7。2026-11-20 C42i)
        chk = [k for k in wr_idx if k < g_idx]
        c_idx = chk[-1] if chk else None
        T = {}
        for name, k in (("check", c_idx), ("grasp", g_idx), ("place", p_idx)):
            if k is None:
                continue
            q, jaw = cmds[k]["q"], cmds[k]["jaw"]
            pr, Rr = pose(model, data, site, jid, q, jaw)
            pt, Rt = pose(model, data, site, jid, [a + b for a, b in zip(q, bias)], jaw)
            T[name] = {"r": (pr, Rr), "t": (pt, Rt), "k": k}
        fk = {}
        for name, v in T.items():
            (pr, Rr), (pt, Rt) = v["r"], v["t"]
            d = screen(pt) - screen(pr)
            fk[name] = {"cmd_index": v["k"], "dx_mm": round(float(d[0]), 2), "dz_mm": round(float(d[1]), 2),
                        "horiz_mm": round(float(np.linalg.norm(d)), 2), "dh_mm": round(float((pt[2] - pr[2]) * 1000), 2),
                        "dyaw_deg": round(yaw_delta(Rt, Rr), 2)}
        ct = cells_true(sc, pc, tr)
        tgt = RP.cell_centers_expected(sc, pc, pc["end"])

        def move(a, b, X):
            (pa, Ra), (pb, Rb) = a, b
            return (Ra @ Rb.T @ (X - pb).T).T + pa
        pred = {}
        for name, a, b in (("est", "r", "r"), ("grasp", "r", "t"), ("place", "t", "r"), ("full", "t", "t")):
            X = move(T["place"][a], T["grasp"][b], ct)
            w, m = compare(X, tgt)
            pred[name] = {"worst_mm": round(w, 2), "mean_dx_mm": round(float(m[0]), 2), "mean_dz_mm": round(float(m[1]), 2)}
        # 手首の確かめの FK の見込み: 見積もり ≈ T_r(確かめ) T_t(確かめ)^-1 (真値)。それを置く計画に通した所
        if "check" in T:
            # 計画は見積もり P_est を目標へ運ぶ(X)。P_est = C(P_true) なら、目標 = X(C(P_true))。真値を運ぶと X(P_true)。
            #   見込みのずれ = X(P_true) と X(C(P_true)) の差。これを est の項(X(P_true) と盤の目標の差)と比べる
            Xc_est = move(T["place"]["r"], T["grasp"]["r"], ct)
            XC = move(T["place"]["r"], T["grasp"]["r"], move(T["check"]["r"], T["check"]["t"], ct))
            w, m = compare(Xc_est, XC)
            pred["est_by_fk_check"] = {"worst_mm": round(w, 2), "mean_dx_mm": round(float(m[0]), 2), "mean_dz_mm": round(float(m[1]), 2),
                                       "note": "X(P_true) vs X(C(P_true)): 手首のカメラが確かめの姿勢の腕と一緒にずれるとした、見積もりの誤りの見込み"}
        # 本当の置いた所(MuJoCo)
        fa = cells_true(sc, pc, fs)
        w, m = compare(fa, tgt)
        actual = {"worst_mm": round(w, 2), "mean_dx_mm": round(float(m[0]), 2), "mean_dz_mm": round(float(m[1]), 2)}
        fw, fm = compare(move(T["place"]["t"], T["grasp"]["t"], ct), fa)
        out["pieces"].append({"name": pc["name"], "fk": fk, "pred": pred, "actual": actual,
                              "full_vs_actual": {"worst_mm": round(fw, 2), "mean_dx_mm": round(float(fm[0]), 2), "mean_dz_mm": round(float(fm[1]), 2)}})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", default=None)
    ap.add_argument("--grasp-back", type=int, default=0, help="つかむ姿勢を held の最初のコマから N コマ前にする(既定 0 = 前と同じ)")
    a = ap.parse_args()
    global GRASP_BACK
    GRASP_BACK = a.grasp_back
    run0 = json.load(open(a.runs[0], encoding="utf-8"))
    model = RP.build(run0["scene"], os.path.dirname(a.runs[0]), write_mjcf=False)
    data = mujoco.MjData(model)
    site = model.site("gripperframe").id
    jid = [model.joint(n).qposadr[0] for n in JOINTS] + [model.joint("gripper").qposadr[0]]
    res = []
    for p in a.runs:
        r = analyze(p, model, data, site, jid)
        res.append(r)
        print(f"== {p}  seed {r['seed']}  bias {r['arm_bias_deg']}")
        for pc in r["pieces"]:
            print(f"  {pc['name']}: FK diff " + "  ".join(f"{k} {v['horiz_mm']:.2f}mm ({v['dx_mm']:+.2f},{v['dz_mm']:+.2f}) h{v['dh_mm']:+.2f} yaw{v['dyaw_deg']:+.2f}"
                                                        for k, v in pc["fk"].items()))
            print("     pred worst/mean: " + "  ".join(f"{k} {v['worst_mm']:.2f} ({v['mean_dx_mm']:+.2f},{v['mean_dz_mm']:+.2f})" for k, v in pc["pred"].items()))
            print(f"     actual {pc['actual']['worst_mm']:.2f} ({pc['actual']['mean_dx_mm']:+.2f},{pc['actual']['mean_dz_mm']:+.2f})"
                  f"   full-vs-actual {pc['full_vs_actual']['worst_mm']:.2f} ({pc['full_vs_actual']['mean_dx_mm']:+.2f},{pc['full_vs_actual']['mean_dz_mm']:+.2f})")
    if a.out:
        os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
        with open(a.out, "w", encoding="utf-8") as fo:
            json.dump(res, fo, indent=1, ensure_ascii=False)
        print("wrote", a.out)


if __name__ == "__main__":
    main()
