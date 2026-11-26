"""列 P 段階 2: 画像 → 計画 → MuJoCo で動かす → 同じ世界から撮り直した画像と関節を制御へ戻す(夜の作業 2026-11-19 23 時の回が作った)。

  python は sim\\physics\\.venv の物(mujoco が要る):
    sim\\physics\\.venv\\Scripts\\python.exe tools\\puzzle_mj.py [--set 2] [--seed 1] [--pieces N] [--shift X,Z] [--no-wrist] [--arm-err DEG | --arm-bias a,b,c,d,e]
    sim\\physics\\.venv\\Scripts\\python.exe tools\\puzzle_mj.py --leak-test RUN.json      (情報漏れの検算。下)

1 手の入口と出口(産業の列が使い回す用。単位と時刻をここで決める)
  制御へ入れる物(観測):
    - 上の画像・手首の画像: 生の RGB(.rgb、W×H×3 バイト)。sim/plan-cli.js の render が、MuJoCo の世界のいまの状態
      (ピースの位置と向き・腕の関節)から描く。撮った時刻 = 世界の時計 t_world [秒](動き始めの前の落ち着かせ 0.5 秒を含む)。
    - 関節: LeRobot の度(gripper は 0〜100)。MuJoCo の関節の角度(rad)を URDF の度にし、計画の frames の (q, L) の組から
      当てはめた 1 次の換算で LeRobot の度に直す(換算の当てはめの誤差を始めに確かめ、1e-6 度を超えたら止める)。
    - 事前の情報: 盤の目標(既定の解き方の終わりの置き方。export-physics.js の scene.json の end)・ピースの形・上のカメラの較正(sim)。
  制御が返す物(指令): 計画の 1 コマ = {t [秒], q: URDF の度 5 個, jaw: URDF の度, held: 持つつもりのピースの番号 | -1}。
    世界はコマの間を時刻で線形に補い、held >= 0 の間は爪をさらに 8 度閉じる(run_physics.py と同じ握り込み)。
  制御へ渡さない物(計画書の欄 2): ピースの真値(MuJoCo の体の位置)・描画のピース ID・採点の出力。
    真値は World だけが持ち、Controller は World.observe() の戻り(画像のファイルと関節)しか読まない。
  採点(制御と別): 終わりに MuJoCo のピースの姿勢を盤の目標と比べる(マスの真ん中のずれ 3mm・向き 5 度・傾き 5 度。run_physics.py の【仮】と同じ)。

流れ(puzzle_run.py の --sim と同じ。違いは、世界が MuJoCo で、ピースが自由な剛体なこと):
  (1) 世界を真値の置き場で組み、0.5 秒落ち着かせる (2) 写す姿勢で上の画像 → detect → 置き場
  (3) plan → 関節の列 (4) コマごとに MuJoCo を進める。手首で見る所で手首の画像 → wrist → match →
      ずれ > 0.5mm なら直した置き場で計画し直し、来た道を戻ってから新しい計画に乗る(1 ピース 2 回まで。超えたら理由付きで止める)
  (5) 終わりに写す姿勢で上の画像 → detect → 盤の目標と比べる(置けたかの画像の確認) (6) 採点(真値。制御へは返さない)

情報漏れの検算(--leak-test): 1 回目の記録(観測の列)を、そのまま制御へ流し直す。採点の真値だけを 3mm ずらして、
  指令の列が 1 回目と 1 ビットも変わらず、採点だけが変わることを確かめる。

置く姿勢の腕のずれ(2026-11-20 C42g。tools/puzzle_mj_fkerr.py で分けた): 手首の確かめは、つかむ姿勢の腕のずれを打ち消すが、
  置く姿勢のずれは見えない。試した 2 つ:
  --place-comp: 各ピースの最初の手首の確かめ(上の絵の見積もりとの差)から、腕の模型(Kin。事前の情報の scene.json から組む)で
    関節のずれを見積もり、置く姿勢のずれを前もって打ち消す。いまは計画の口に「置き場をずらす」口が無いので、計画へ渡す見積もりを
    ずらして代える(つかむ所が同じだけずれる = つかむ余裕を食う)。
  --pieces 1 --replace N: 置いた後に上の絵で確かめて置き直す。いまの計画の口は、盤に重なったピースを拾う計画を断る(「解けない」)ので、
    置き直しは止まる(理由付き)。
  (2026-11-20 C42i) --place-comp の既定は、plan の place_offset で置く所だけを -e ずらす形(--comp-mode offset)。前の形は --comp-mode est。
落とした・つかめなかったピースに気づく(2026-11-20 C42i):
  - 持っている間の爪の読み: ピースを挟んでいれば、爪は計画の閉じた角度で止まる(握り込みの 8 度は効かない)。計画より GRIP_EMPTY_DEG を超えて
    閉じたら、何も持っていないとして理由付きで止める(観測は cam "joints" = 関節の読みだけ。絵は描かない)。
  - 終わりの上の絵: 置いたピースは盤の目標と、--pieces の回で置かないピースは最初の上の絵の置き場と比べ、外れていれば理由付きで止める。
    --pieces N は計画も N 手で作り、写す姿勢へ戻ってから撮る。
  - 拾い直しはまだ無い(止まるだけ)。
  負の検算(世界の側で起こす。制御へは渡さない): --world-fault drop:PID:N | nudge:PID:T:DX:DZ。
まだ無い物(次の回): 遮蔽の場面・L J O 以外の盤・
  持っているピース(机から浮いた物)は絵に描けないので外している(爪の中に隠れている扱い)。
"""

import argparse
import datetime
import hashlib
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
SIM = os.path.join(ROOT, "sim")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(SIM, "physics"))
import numpy as np                     # noqa: E402
import mujoco                          # noqa: E402
import run_physics as RP               # noqa: E402  (読むだけ。build・command の握り込み・姿勢の読み)
from puzzle_run import Planner, Stop, first_diff, JOINTS, FIX_TOL_MM, MAX_FIX, MIN_TIP_MM   # noqa: E402

OUTROOT = os.path.join(ROOT, "outputs", "night")
SETTLE_S = 0.5
LIFT_HIDE_MM = 3.0      # 机からこれより浮いたピースは絵に描かない(爪の中)
TILT_HIDE_DEG = 5.0
PLACE_IMG_MM = 3.0      # 置けたかの画像の確認: 盤の目標とのずれ(いちばん悪いマスの真ん中)
REPLACE_TOL_MM = 2.0    # 【仮】--replace: 上の絵でこれ以上ずれていたら置き直す(上の絵の片寄り 最大 1.8mm を少し超える所。2026-11-20 C42g)
FIT_TOL_DEG = 1e-6
GRIP_WIDE_DEG = 2.0    # 【仮】持っている間、爪が計画の閉じた角度よりこれだけ開いたまま止まったら「持ち方がおかしい」(既定の回は 0 度ほど。C42i の「裏返った O の 3.9 度開いたまま」は、O だけ向きが逆だった時の読みで、本当は 3.9 度閉じすぎ。C42k)
GRIP_EMPTY_DEG = 3.0   # 【仮】持っている間、爪が計画の閉じた角度(ピースの幅)よりこれだけ閉じたら「何も持っていない」(握り込みは 8 度。持っていれば 0.1 度ほど。2026-11-20 C42i)


class Conv:
    """URDF の度 ⇄ LeRobot の度。計画の frames の (q, L) の組から関節ごとに 1 次で当てはめ、全部のコマで誤差を確かめる。
    爪(jaw ⇄ gripper)は 1 次でないことがあるので、組の表を線形に補う。"""

    def __init__(self, frames):
        self.ab = []
        worst = 0.0
        for k, j in enumerate(JOINTS):
            xs = np.array([f["q"][k] for f in frames]); ys = np.array([f["L"][j] for f in frames])
            a, b = np.polyfit(xs, ys, 1)
            a = round(a)                                # 符号だけ(fake の k1 は ±1)。丸めて当てはめの誤差を見る
            b = float(np.median(ys - a * xs))
            worst = max(worst, float(np.max(np.abs(a * xs + b - ys))))
            self.ab.append((a, b))
        if worst > FIT_TOL_DEG:
            raise Stop(f"URDF → LeRobot の 1 次の当てはめが {worst:.2e} 度ずれる(換算が 1 次でない?)")
        pairs = sorted({(round(f["jaw"], 9), f["L"]["gripper"]) for f in frames})
        self.jx = np.array([p[0] for p in pairs]); self.jy = np.array([p[1] for p in pairs])
        self.fit_err = worst

    def to_L(self, q, jaw):
        L = {j: a * v + b for j, (a, b), v in zip(JOINTS, self.ab, q)}
        L["gripper"] = self._jaw(jaw)
        return L

    def _jaw(self, jaw):
        """表の外は端の 2 点の傾きで延ばす(2026-11-20 C42i: np.interp は端で止まるので、ピースが無くて計画より閉じた爪が、
        計画の閉じた値と同じに読めていた = 落としたことが関節の読みで見えない)。表の中は前と同じ"""
        x, y = self.jx, self.jy
        if len(x) >= 2 and jaw < x[0] and x[1] != x[0]:
            return float(y[0] + (jaw - x[0]) * (y[1] - y[0]) / (x[1] - x[0]))
        if len(x) >= 2 and jaw > x[-1] and x[-1] != x[-2]:
            return float(y[-1] + (jaw - x[-1]) * (y[-1] - y[-2]) / (x[-1] - x[-2]))
        return float(np.interp(jaw, x, y))


class World:
    """MuJoCo の世界。真値(ピースの体の姿勢)はここだけが持つ。制御へは observe() の画像と関節だけを返す。"""

    def __init__(self, pl, proto, truth, start_q, start_jaw, outdir, bias=None, faults=None, say=None):
        self.pl, self.outdir = pl, outdir
        # 世界の側で起こす失敗(--world-fault。負の検算 2026-11-20 C42i。制御へは渡さない):
        #   ("drop", pid, n): そのピースを持つ指令の n 回目に、ピースを爪から外して最初の置き場へ戻す(落とした扱い)
        #   ("nudge", pid, t, dx, dz): 世界の時計 t 秒を過ぎたら、ピースを机の上で (dx, dz) mm 動かす(つかむ直前に動けば、つかみ損ねる)
        self.faults = [list(f) + [False] for f in (faults or [])]
        self.say = say or (lambda *x, **k: None)
        self.nheld = {}
        self.start_pos = {t["name"]: (t["x"], t["z"]) for t in truth["pieces"]}
        # 関節の誤差(--arm-err。puzzle_run.py の SimRobot と同じ意味): 本当の腕 = 読んだ角度 + bias(URDF の度、5 関節)。
        #   指令(読んだ角度の目標)に bias を足して MuJoCo を動かし、関節の読みは MuJoCo − bias を返す。絵は本当の腕で描く
        self.bias = list(bias) if bias else [0.0] * 5
        sc = json.loads(json.dumps(proto))
        for pc, t in zip(sc["pieces"], truth["pieces"]):
            assert pc["name"] == t["name"]
            pc["start"] = {"pos": [t["x"] / 1000, 0.0, t["z"] / 1000], "rot": t["yaw"]}
        self.scene = sc
        self.model = RP.build(sc, outdir, write_mjcf=False)
        self.data = mujoco.MjData(self.model)
        self.jid = [self.model.joint(n).qposadr[0] for n in JOINTS] + [self.model.joint("gripper").qposadr[0]]
        self.cmd = {"q": list(start_q), "jaw": start_jaw, "held": -1}
        for k, v in zip(self.jid, list(np.radians([a + b for a, b in zip(start_q, self.bias)])) + [math.radians(start_jaw)]):
            self.data.qpos[k] = v
        mujoco.mj_forward(self.model, self.data)
        self.t = 0.0
        self.nshot = 0
        self.sign = 1 if sc["jaw"]["grip"]["sign"] > 0 else -1
        self._run(self.cmd, self.cmd, SETTLE_S)

    def _ctrl(self, q, jaw, held):
        if held >= 0:
            jaw -= RP.SQUEEZE_DEG * self.sign
        self.data.ctrl[:] = list(np.radians([a + b for a, b in zip(q, self.bias)])) + [math.radians(jaw)]

    def _run(self, fa, fb, dt):
        ts = self.model.opt.timestep
        n = max(1, int(round(dt / ts)))
        for i in range(n):
            u = (i + 1) / n
            q = [a + (b - a) * u for a, b in zip(fa["q"], fb["q"])]
            jaw = fa["jaw"] + (fb["jaw"] - fa["jaw"]) * u
            self._ctrl(q, jaw, fb["held"])
            mujoco.mj_step(self.model, self.data)
        self.t += n * ts

    def command(self, f, dt):
        """制御の 1 コマを、dt 秒かけて送る"""
        nf = {"q": list(f["q"]), "jaw": f["jaw"], "held": f["held"]}
        self._faults(nf)
        self._run(self.cmd, nf, max(dt, self.model.opt.timestep))
        self.cmd = nf

    def _move_piece(self, pid, x_mm, z_mm, lift_m=0.0):
        """ピースの自由な関節を、机の (x, z) mm へ置き直す(向きはそのまま・速さ 0)。世界の側の失敗を起こすときだけ使う"""
        jn = self.model.joint("pc%d_free" % self.scene["pieces"][pid]["idx"])
        a, d = jn.qposadr[0], jn.dofadr[0]
        p = RP.to_mj([x_mm / 1000, 0.0, z_mm / 1000])
        self.data.qpos[a:a + 3] = [p[0], p[1], self.scene["cell"] / 2 + lift_m]
        self.data.qvel[d:d + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _faults(self, nf):
        if nf["held"] >= 0:
            self.nheld[nf["held"]] = self.nheld.get(nf["held"], 0) + 1
        for fl in self.faults:
            if fl[-1]:
                continue
            if fl[0] == "drop" and nf["held"] == fl[1] and self.nheld.get(fl[1], 0) == fl[2]:
                name = self.scene["pieces"][fl[1]]["name"]
                x, z = self.start_pos[name]
                self._move_piece(fl[1], x, z, lift_m=0.002)
                fl[-1] = True
                self.say(f"(world fault) {name} slipped out of the gripper at t {self.t:.2f} s and fell back to {x:.1f},{z:.1f} mm", world_fault="drop")
            elif fl[0] == "nudge" and self.t >= fl[2]:
                st = self.piece_states()[fl[1]]
                self._move_piece(fl[1], st["x"] + fl[3], st["z"] + fl[4])
                fl[-1] = True
                self.say(f"(world fault) {st['name']} was moved by {fl[3]:+.1f},{fl[4]:+.1f} mm at t {self.t:.2f} s", world_fault="nudge")

    # ---- 真値(World と採点だけが使う)
    def piece_states(self):
        out = []
        for pc in self.scene["pieces"]:
            cs, R = RP.cell_centers_actual(self.scene, pc, self.data)
            b = self.data.body("pc%d" % pc["idx"]).xpos
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
            out.append({"name": pc["name"], "x": float(b[0]) * 1000, "z": float(-b[1]) * 1000, "yaw": RP.yaw_of(R),
                        "lift_mm": float(np.min(cs[:, 2]) - self.scene["cell"] / 2) * 1000, "tilt": tilt})
        return out

    def joints_urdf(self):
        q = [math.degrees(self.data.qpos[k]) - b for k, b in zip(self.jid[:5], self.bias)]   # 読み = 本当 − bias
        return q, math.degrees(self.data.qpos[self.jid[5]])

    # ---- 観測(制御へ渡す物)
    def observe(self, cam, conv):
        if cam == "joints":            # 関節の読みだけ(絵は描かない。爪の読みでつかめたかを見る。2026-11-20 C42i)
            q, jaw = self.joints_urdf()
            return {"cam": "joints", "lerobot": conv.to_L(q, jaw), "t_world": round(self.t, 4)}
        st = self.piece_states()
        lay = {"pieces": [{"name": s["name"], "x": s["x"], "z": s["z"], "yaw": s["yaw"]} for s in st]}
        # 消すのは爪に持たれて浮いたピースだけ。机の上で傾いた・倒れたピースは描けないので、消さずに止める(点検 C42c の 5)
        for s in st:
            if s["lift_mm"] <= LIFT_HIDE_MM and s["tilt"] > TILT_HIDE_DEG:
                raise Stop(f"world: {s['name']} is tilted {s['tilt']:.1f} deg on the table; the renderer cannot draw it (not hidden, stop)")
        hide = [i for i, s in enumerate(st) if s["lift_mm"] > LIFT_HIDE_MM]
        q, jaw = self.joints_urdf()
        L = conv.to_L(q, jaw)
        self.nshot += 1
        out = os.path.join(self.outdir, f"shot_{self.nshot:02d}_{cam}.rgb")
        im = self.pl.need("render", cam=cam, truth=lay, hide=hide, lerobot=L, bias=self.bias if any(self.bias) else None, out=out)
        return {"cam": cam, "img": im["out"], "W": im["W"], "H": im["H"], "lerobot": L, "t_world": round(self.t, 4)}


def score(scene, states, truth_shift=(0.0, 0.0)):
    """採点(制御と別)。盤の目標(scene の end)と、MuJoCo のピースの姿勢を比べる。truth_shift は漏れの検算で真値だけをずらす"""
    rows = []
    for pc, s in zip(scene["pieces"], states):
        end = pc["end"]
        exp = RP.cell_centers_expected(scene, pc, end)
        P = scene["pitch"]; r = math.radians(s["yaw"]); c, si = math.cos(r), math.sin(r)
        act = np.array([[s["x"] / 1000 + truth_shift[0] / 1000 + (a * P) * c - (b * P) * si,
                         -(s["z"] / 1000 + truth_shift[1] / 1000 + (a * P) * si + (b * P) * c)] for a, b in pc["cells"]])
        # マスの組どうしで比べる(番号で比べると、O を 180 度回して置いたとき、同じ形なのに 41mm と出た。23:19 の誤り)
        d = np.linalg.norm(exp[:, None, :2] - act[None, :, :], axis=2)
        e = float(np.max(np.min(d, axis=1))) * 1000
        one2one = sorted(np.argmin(d, axis=1).tolist()) == list(range(len(act)))   # 1 対 1 に対応しない = 形が合っていない
        yaw_err = ((s["yaw"] - end["rot"] + 45) % 90) - 45  # 立方体の向きは 90 度ごとに同じ(マスの組が合っていれば、残りはこれだけ)
        ok = one2one and e < RP.TOL_MM and abs(yaw_err) < RP.TOL_DEG and s["tilt"] < 5 and s["lift_mm"] < 1.0
        rows.append({"name": pc["name"], "place_err_mm": round(e, 2), "cells_match": one2one, "yaw_err_deg": round(yaw_err, 2), "tilt_deg": round(s["tilt"], 2),
                     "lift_mm": round(s["lift_mm"], 2), "ok": ok})
    return rows


class Kin:
    """--place-comp(2026-11-20 C42g): 腕の模型(事前の情報の scene.json から組んだ MuJoCo の運動学。真値の置き場は入れない)で、
    手首の確かめから関節のずれ b(URDF の度、5 個)を見積もり、置く姿勢のずれを前もって打ち消す。
      測り: そのピースの最初の確かめで、手首の測り(fixed)と上の絵の見積もり(est)の差 = マスの真ん中の平均 (dx, dz) [mm] と向き phi [度]。
      見込み: 手首のカメラは爪と一緒に動くので、測り ≈ G_r G_t^-1 (見積もり)。G は爪の基準点 gripperframe の姿勢、r = 読み、t = 読み + b。
      解き方: ガウス・ニュートン(数値の微分)。b の事前の幅 2 度・測りの幅 1mm(1 度)の重み【仮】。
      打ち消し: 見積もった b で、置いた所のずれ e を出し(T_t(置く) T_t(つかむ)^-1 を、手首で直した見積もりを b で戻した物に当てる)、
        計画へ渡す見積もりに s = R(-psi) e を足す(psi = つかむ → 置くの爪の向きの変わり。つかむ所が s ずれ、置く所で -R(psi)s 戻る)。"""

    def __init__(self, prior):
        proto = json.load(open(prior["scene_json"], encoding="utf-8"))
        self.model = RP.build(proto, os.path.dirname(prior["scene_json"]), write_mjcf=False)
        self.data = mujoco.MjData(self.model)
        self.site = self.model.site("gripperframe").id
        self.jid = [self.model.joint(n).qposadr[0] for n in JOINTS]
        self.P, self.cell = proto["pitch"], proto["cell"]
        self.cells = [p["cells"] for p in proto["pieces"]]
        self.checks = []

    def G(self, q):
        for k, v in zip(self.jid, np.radians(q)):
            self.data.qpos[k] = v
        mujoco.mj_kinematics(self.model, self.data)
        return self.data.site_xpos[self.site].copy(), self.data.site_xmat[self.site].reshape(3, 3).copy()

    @staticmethod
    def amap(A, B, X):                                     # A o B^-1
        (pa, Ra), (pb, Rb) = A, B
        return (Ra @ Rb.T @ (X - pb).T).T + pa

    @staticmethod
    def scr(X):
        return np.stack([X[:, 0] * 1000, -X[:, 1] * 1000], axis=1)

    @staticmethod
    def yaw(Ra, Rb):                                       # 画面の向き psi で、Ra Rb^T の上から見た回り
        dR = Ra @ Rb.T
        return -math.degrees(math.atan2(dR[1, 0] - dR[0, 1], dR[0, 0] + dR[1, 1]))

    def cells3(self, pid, pose):
        r = math.radians(pose["yaw"]); c, s = math.cos(r), math.sin(r)
        return np.array([RP.to_mj([pose["x"] / 1000 + a * self.P * c - b * self.P * s, self.cell / 2,
                                   pose["z"] / 1000 + a * self.P * s + b * self.P * c]) for a, b in self.cells[pid]])

    def add_check(self, q, pid, est, fixed, phi):
        ce, cf = self.cells3(pid, est), self.cells3(pid, fixed)
        d = (self.scr(cf) - self.scr(ce)).mean(axis=0)
        self.checks.append((list(q), ce, np.array([d[0], d[1], phi])))

    def _pred(self, b, q, ce):
        Gr, Gt = self.G(q), self.G([x + y for x, y in zip(q, b)])
        X = self.amap(Gr, Gt, ce)
        d = (self.scr(X) - self.scr(ce)).mean(axis=0)
        return np.array([d[0], d[1], self.yaw(Gr[1], Gt[1])])

    def solve(self, sig_b=2.0, iters=6):
        b = np.zeros(5)

        def res(b):
            r = [self._pred(b, q, ce) - meas for q, ce, meas in self.checks]
            return np.concatenate(r + [b / sig_b])
        for _ in range(iters):
            r0 = res(b)
            J = np.stack([(res(b + np.eye(5)[k] * 0.05) - r0) / 0.05 for k in range(5)], axis=1)
            b = b - np.linalg.lstsq(J, r0, rcond=None)[0]
        return [float(v) for v in b]

    def offset_for(self, pid, plan, fixed, q_chk, b):
        fr = plan["frames"]
        hk = [i for i, f in enumerate(fr) if f["held"] == pid]
        gi, pi = hk[0], hk[-1]
        qt = lambda q: [x + y for x, y in zip(q, b)]
        cf = self.cells3(pid, fixed)
        P_true = self.amap(self.G(qt(q_chk)), self.G(q_chk), cf)          # 手首の測りを、見積もった腕のずれで戻す
        Ggr_r, Gpl_r = self.G(fr[gi]["q"]), self.G(fr[pi]["q"])
        Ggr_t, Gpl_t = self.G(qt(fr[gi]["q"])), self.G(qt(fr[pi]["q"]))
        final = self.amap(Gpl_t, Ggr_t, P_true)
        target = self.amap(Gpl_r, Ggr_r, cf)
        e = (self.scr(final) - self.scr(target)).mean(axis=0)
        psi = math.radians(self.yaw(Gpl_r[1], Ggr_r[1]))
        c, s = math.cos(-psi), math.sin(-psi)
        return float(c * e[0] - s * e[1]), float(s * e[0] + c * e[1]), [float(e[0]), float(e[1])]


class Controller:
    """画像と関節だけから指令を作る。obs_fn(cam) → 観測、send(frame, dt) → 指令を送る。真値は受け取らない。"""

    def __init__(self, pl, init, prior, a, obs_fn, send, say):
        self.pl, self.init, self.prior, self.a = pl, init, prior, a
        self.obs, self.send, self.say = obs_fn, send, say
        self.cmds = []                                       # 送った指令の列(漏れの検算で比べる)
        self.offs = {}
        self.poffs = {}
        self.grip_past = {}
        self.grip_wide = {}
        self.plan_kw = {}
        self.comp_seen, self.comp_done = set(), set()
        self.kin = Kin(prior) if getattr(a, "place_comp", False) else None

    def _send(self, f, dt):
        rec = {"q": [round(v, 9) for v in f["q"]], "jaw": round(f["jaw"], 9), "held": f["held"], "dt": round(dt, 9)}
        self.cmds.append(rec)
        self.send(f, dt)

    def _plan(self, layout, ask=False):
        """計画を頼む。置き直しの補い(self.offs)があれば、そのピースの見積もりにだけ足して渡す(補いが無ければ前と同じ頼み)。
        置く所だけのずらし(self.poffs。--place-comp の既定 2026-11-20 C42i)は plan の place_offset で渡す(つかむ所は変わらない)"""
        lay = layout
        if self.offs:
            lay = json.loads(json.dumps(layout))
            for pid, (dx, dz) in self.offs.items():
                lay["pieces"][pid]["x"] += dx; lay["pieces"][pid]["z"] += dz
        kw = dict(self.plan_kw)
        if self.poffs:
            kw["place_offset"] = [{"pid": pid, "dx_mm": dx, "dz_mm": dz} for pid, (dx, dz) in sorted(self.poffs.items())]
        return (self.pl.ask if ask else self.pl.need)("plan", layout=lay, air=0, **kw)

    def _grip_check(self, plan, f):
        """持っている間の爪の読み(2026-11-20 C42i): ピースを挟んでいれば、爪は計画の閉じた角度(ピースの幅)で止まる。
        無ければ(つかみ損ね・落とした)、さらに閉じる。計画の開いた爪と閉じた爪の組から LeRobot の値 → 度の傾きを出し、
        計画の閉じた値より GRIP_EMPTY_DEG 度を超えて閉じていたら、理由付きで止める"""
        pid = f["held"]
        st = next(s for s in plan["steps"] if s["pid"] == pid)
        seg = [g for g in plan["frames"] if st["tHover"] - 1e-9 <= g["t"] <= st["tPick"] + 1e-9]
        # 開いた側 = 持つ値へ閉じる直前のコマの側(2026-11-20 C42k)。前は |差| のいちばん大きいコマを「開いた爪」にしていて、
        #   O のように下りる前の爪(1.75・15.6 度)が持つ値(21.2 度)より閉じている計画では、閉じた側のコマを選び、閉じる・開くの向きが逆だった
        ramp = [g for g in seg if abs(g["jaw"] - f["jaw"]) >= 1e-6]
        if not ramp:
            raise Stop(f"{st['name']}: grip check: the plan never opens the jaw before the pick")
        side = 1 if ramp[-1]["jaw"] > f["jaw"] else -1
        op = max((g for g in seg if (g["jaw"] - f["jaw"]) * side > 0), key=lambda g: abs(g["jaw"] - f["jaw"]))
        slope = (op["L"]["gripper"] - f["L"]["gripper"]) / (op["jaw"] - f["jaw"])        # LeRobot の値 / URDF の度
        o = self.obs("joints")
        past = (f["L"]["gripper"] - o["lerobot"]["gripper"]) * (1 if op["L"]["gripper"] > f["L"]["gripper"] else -1) / abs(slope)
        self.grip_past[pid] = max(self.grip_past.get(pid, -1e9), past)
        self.grip_wide[pid] = min(self.grip_wide.get(pid, 1e9), past)
        if past < -GRIP_WIDE_DEG:   # 計画の幅より開いたまま止まった = 傾いた・角を挟んだ(C42i: --comp-mode est の種 1 の O。運んだ後に裏返った)
            raise Stop(f"{st['name']}: the gripper stopped {-past:.1f} deg wider than the piece width at t_world {o['t_world']:.2f} s "
                       f"(reads {o['lerobot']['gripper']:.2f}, plan {f['L']['gripper']:.2f}): the piece is held in a wrong way (tilted or on a corner)")
        if past > GRIP_EMPTY_DEG:
            raise Stop(f"{st['name']}: the gripper closed {past:.1f} deg past the piece width at t_world {o['t_world']:.2f} s "
                       f"(reads {o['lerobot']['gripper']:.2f}, plan {f['L']['gripper']:.2f}): nothing in the gripper (missed or dropped)")

    def _execute(self, plan, layout, marks):
        """計画を 1 コマずつ送り、手首で見る所で確かめて直す(前の run の中の輪をそのまま移した。2026-11-20 C42g)"""
        a, pl, init = self.a, self.pl, self.init
        fixes = {p["idx"]: 0 for p in init["pieces"]}
        shot_at = marks(plan)
        i, n_back = 0, 0
        while i < len(plan["frames"]) - 1:
            i += 1
            f, fp = plan["frames"][i], plan["frames"][i - 1]
            self._send(f, f["t"] - fp["t"])
            if not getattr(a, "no_grip_check", False):
                if f["held"] >= 0 and fp["held"] == f["held"]:      # 持ち始めの 1 コマは握り込みの途中なので飛ばす
                    self._grip_check(plan, f)
                elif fp["held"] >= 0 and f["held"] != fp["held"] and fp["held"] in self.grip_past:
                    nm = next(s["name"] for s in plan["steps"] if s["pid"] == fp["held"])
                    self.say(f"[grip {nm}] held to the release; closed {self.grip_wide[fp['held']]:+.2f} .. {self.grip_past[fp['held']]:+.2f} deg past the piece width",
                             pid=fp["held"], grip_past_deg=round(self.grip_past[fp["held"]], 3), grip_min_deg=round(self.grip_wide[fp["held"]], 3))
            if a.no_wrist or i not in shot_at:
                continue
            st = shot_at[i]
            pid = st["pid"]
            o = self.obs("wrist")
            w = pl.ask("wrist", img=o["img"], W=o["W"], H=o["H"], lerobot=o["lerobot"], pid=pid, est=layout["pieces"][pid])
            if not w.get("ok"):
                raise Stop(f"{st['name']}: wrist image: {w.get('why')}")
            m = pl.need("match", pid=pid, est=layout["pieces"][pid], meas=w["piece"])
            # 見るだけの差(判断には使わない。2026-11-20 C42e): 見積もりと手首の測りの「置き場の基準点と向き」の差をそのまま引く。
            #   match はマスを近い順に組むので、ずれがピッチの半分(14.5mm)を越えると組を取り違え、測りが正しくても 80mm・104 度と言った(shift 16mm)
            e0, w0 = layout["pieces"][pid], w["piece"]
            pose_d = (round(math.hypot(w0["x"] - e0["x"], w0["z"] - e0["z"]), 2), round(((w0["yaw"] - e0["yaw"] + 180) % 360) - 180, 2))
            self.say(f"[wrist {st['name']}] off by {m['worst_cell_mm']:.2f} mm, {m['phi']:+.2f} deg ({w['nPix']} px) t_world {o['t_world']:.2f}"
                     f" (pose diff {pose_d[0]:.2f} mm, {pose_d[1]:+.2f} deg)",
                     pid=pid, off_mm=m["worst_cell_mm"], pose_diff_mm=pose_d[0], pose_diff_deg=pose_d[1])
            comp_now = self.kin is not None and pid not in self.comp_done
            if comp_now and pid not in self.comp_seen:   # そのピースの最初の確かめ(見積もりは上の絵 = 腕と別に据えたカメラ)だけを、腕のずれの推定に使う
                self.comp_seen.add(pid)
                self.kin.add_check(f["q"], pid, layout["pieces"][pid], m["fixed"], m["phi"])
            if m["worst_cell_mm"] <= FIX_TOL_MM and not comp_now:
                continue
            if m["worst_cell_mm"] > init["pitch_mm"] / 2:
                raise Stop(f"{st['name']}: wrist says {m['worst_cell_mm']:.1f} mm off (> half a cell)")
            new_layout = json.loads(json.dumps(layout))
            if m["worst_cell_mm"] > FIX_TOL_MM:
                if fixes[pid] >= MAX_FIX:
                    raise Stop(f"{st['name']}: still off by {m['worst_cell_mm']:.2f} mm after {MAX_FIX} corrections")
                fixes[pid] += 1
                new_layout["pieces"][pid] = m["fixed"]
            plan2 = self._plan(new_layout, ask=True)
            if comp_now and plan2.get("ok"):
                plan2_noc = plan2
                b_hat = self.kin.solve()
                sx, sz, e_pred = self.kin.offset_for(pid, plan2, new_layout["pieces"][pid], f["q"], b_hat)
                if getattr(a, "comp_mode", "offset") == "est":    # 前の形(C42g): 見積もりをずらす = つかむ所もずれる
                    self.offs[pid] = (sx, sz)
                else:                                             # 置く所だけを -e ずらす(plan の place_offset。2026-11-20 C42i)
                    sx, sz = -e_pred[0], -e_pred[1]
                    self.poffs[pid] = (sx, sz)
                self.comp_done.add(pid)
                self.say(f"[place comp {st['name']}] arm error estimate " + ", ".join(f"{v:+.2f}" for v in b_hat) +
                         f" deg; predicted place error ({e_pred[0]:+.2f},{e_pred[1]:+.2f}) mm -> {getattr(a, 'comp_mode', 'offset')} offset ({sx:+.2f},{sz:+.2f}) mm",
                         pid=pid, b_hat=list(b_hat), e_pred=list(e_pred), offset=[sx, sz], comp_mode=getattr(a, "comp_mode", "offset"))
                plan2 = self._plan(new_layout, ask=True)
                if not plan2.get("ok") and pid in self.poffs:   # ずらした置く所が当たる・届かない(2026-11-20 C42i 種 1 の L): 打ち消さずに続ける(計画は 1 回目の物)
                    self.say(f"[place comp {st['name']}] the plan refused the place offset ({plan2.get('why')}); going on without it",
                             pid=pid, comp_refused=plan2.get("why"))
                    del self.poffs[pid]
                    plan2 = plan2_noc
                    if m["worst_cell_mm"] <= FIX_TOL_MM:      # 直しも無い = 今の計画のまま続ける(点検 C42i の 5)
                        continue
            if not plan2.get("ok"):
                raise Stop(f"{st['name']}: no plan with the corrected place ({plan2.get('why')})")
            k = first_diff(plan["frames"], plan2["frames"])
            prev_done = max([s["tDone"] for s in plan["steps"] if s["tDone"] < st["tCheck0"]] + [0.0])
            if k == 0 or k >= min(len(plan["frames"]), len(plan2["frames"])) or plan["frames"][k]["t"] <= prev_done + 1e-9:
                raise Stop(f"{st['name']}: the corrected plan changes earlier moves or nothing (k={k})")
            if k <= i:
                for j in range(i - 1, k - 2, -1):          # 来た道を戻る(判定済みのコマ)
                    self._send(plan["frames"][j], abs(plan["frames"][j + 1]["t"] - plan["frames"][j]["t"]))
                    n_back += 1
                i = k - 1
            self.say(f"[wrist {st['name']}] corrected to {m['fixed']['x']:.1f},{m['fixed']['z']:.1f} mm {m['fixed']['yaw']:.1f} deg; back to {plan['frames'][i]['t']:.2f} s")
            plan, layout = plan2, new_layout
            shot_at = marks(plan)
        return {"plan": plan, "layout": layout, "n_back": n_back}

    def _cells(self, pid, pose):
        """事前の情報(盤の解き方の scene.json)のピースの形から、マスの真ん中 [mm](画面の x, z)"""
        P = self.prior["pitch_mm"]; r = math.radians(pose["yaw"]); c, s = math.cos(r), math.sin(r)
        return np.array([[pose["x"] + a * P * c - b * P * s, pose["z"] + a * P * s + b * P * c] for a, b in self.prior["cells"][pid]])

    def _replace_loop(self, plan, layout, marks, n_back):
        """置いた直後に上の絵で確かめ、ずれていれば置き直す(--pieces 1 --replace N。2026-11-20 C42g)。
        置く姿勢の腕のずれは、手首の確かめ(つかむ前)では見えない(tools/puzzle_mj_fkerr.py)。上のカメラは腕と別に据えてあるので、
        置いた所のずれ e を測れる。置き直しでは、計画へ渡す見積もりに e を足す(つかむ所が e だけずれ、置く所で −e 戻る。
        置き直しはほぼ回さずに運ぶので、回しの分は無視できる見込み。回しが 20 度を超えたら止める)。上の絵の片寄り(約 1mm)が残る。"""
        a, pl = self.a, self.pl
        pid = plan["steps"][0]["pid"]
        hist = []
        for attempt in range(a.replace + 1):
            o = self.obs("top")
            det = pl.ask("detect", img=o["img"], W=o["W"], H=o["H"])
            if not det.get("ok"):
                return {"stopped": "top image after placing: " + str(det.get("why")), "n_back": n_back, "full": False, "img_check": None, "replace": hist}
            meas, tgt = det["layout"]["pieces"][pid], self.prior["targets"][pid]
            m = pl.need("match", pid=pid, est=tgt, meas=meas)
            e = (self._cells(pid, m["fixed"]) - self._cells(pid, tgt)).mean(axis=0)
            hist.append({"attempt": attempt, "img_off_mm": round(m["worst_cell_mm"], 2), "phi": round(m["phi"], 2),
                         "mean_dx_mm": round(float(e[0]), 2), "mean_dz_mm": round(float(e[1]), 2), "offset_used": list(self.offs.get(pid, (0.0, 0.0)))})
            self.say(f"[top after place {attempt}] {tgt['name']}: {m['worst_cell_mm']:.2f} mm from the board target, mean ({e[0]:+.2f},{e[1]:+.2f}) mm, {m['phi']:+.2f} deg")
            chk = [{"name": tgt["name"], "img_off_mm": round(m["worst_cell_mm"], 2), "phi": round(m["phi"], 2), "ok": m["worst_cell_mm"] < PLACE_IMG_MM}]
            if m["worst_cell_mm"] < REPLACE_TOL_MM:
                return {"stopped": None, "n_back": n_back, "full": False, "replace": hist, "img_check": chk}
            if attempt == a.replace:
                return {"stopped": f"{tgt['name']}: still {m['worst_cell_mm']:.2f} mm off after {a.replace} re-place(s)", "n_back": n_back, "full": False,
                        "replace": hist, "img_check": chk}
            if abs(m["phi"]) > 20:
                raise Stop(f"{tgt['name']}: re-place would turn the piece {m['phi']:.1f} deg (the offset rule assumes almost no turn)")
            ox, oz = self.offs.get(pid, (0.0, 0.0))
            self.offs[pid] = (ox + float(e[0]), oz + float(e[1]))
            layout = det["layout"]
            plan = self._plan(layout, ask=True)
            if not plan.get("ok"):
                return {"stopped": f"{tgt['name']}: no plan to re-place ({plan.get('why')})", "n_back": n_back, "full": False, "replace": hist, "img_check": None}
            if plan["steps"][0]["pid"] != pid:
                raise Stop(f"re-place plan starts with another piece (pid {plan['steps'][0]['pid']})")
            L0 = o["lerobot"]
            worst = max(abs(L0[j] - plan["frames"][0]["L"][j]) for j in JOINTS)
            if worst > 2.0:
                raise Stop(f"re-place: not at the start pose: {worst:.2f} deg away")
            self.say(f"[re-place {attempt + 1}] {tgt['name']}: plan with offset ({self.offs[pid][0]:+.2f},{self.offs[pid][1]:+.2f}) mm")
            r = self._execute(plan, layout, marks)
            plan, layout, n_back = r["plan"], r["layout"], n_back + r["n_back"]

    def run(self):
        a, pl, init = self.a, self.pl, self.init
        o = self.obs("top")
        det = pl.need("detect", img=o["img"], W=o["W"], H=o["H"])
        layout = det["layout"]
        if a.shift:
            dx, dz = (float(v) for v in a.shift.split(","))
            for p in layout["pieces"]:
                p["x"] += dx; p["z"] += dz
            self.say(f"(negative check) shifted the found layout by {dx},{dz} mm")
        self.say("found: " + " / ".join(f"{p['name']} {p['x']:.1f},{p['z']:.1f} mm {p['yaw']:.1f} deg" for p in layout["pieces"]),
                 found=json.loads(json.dumps(layout)))
        settle = init["wrist_settle"]

        def marks(p):
            shot = {}
            for st in p["steps"]:
                if settle is not None and st["tCheck1"] > 0:
                    shot[next(n for n, f in enumerate(p["frames"]) if f["t"] >= st["tCheck0"] + settle - 1e-9)] = st
            return shot

        # self.offs: 置き直しの補い(--replace)・置く姿勢の打ち消し(--place-comp): pid → (dx, dz) mm。計画へ渡す見積もりにだけ足す
        # --pieces N: 計画も先頭の N 手だけにして、写す姿勢へ戻らせる(2026-11-20 C42i。前は N 手目の終わりで送るのを止め、終わりの上の絵が無かった)
        self.plan_kw = {"pieces": a.pieces} if a.pieces else {}
        found0 = json.loads(json.dumps(layout))
        plan = self._plan(layout)
        if plan["min_tip_mm"] < MIN_TIP_MM:
            raise Stop(f"plan: finger tips {plan['min_tip_mm']:.1f} mm")
        self.conv = Conv(plan["frames"])
        self.say(f"plan: {len(plan['frames'])} poses, {plan['T']:.1f} s; URDF->LeRobot fit error {self.conv.fit_err:.1e} deg")
        # いまの関節が計画の始まりに近いか(P-001 と同じ考え: いまの位置から始める)
        L0 = o["lerobot"]
        worst = max(abs(L0[j] - plan["frames"][0]["L"][j]) for j in JOINTS)
        if worst > 2.0:
            raise Stop(f"not at the start pose: {worst:.2f} deg away")
        r = self._execute(plan, layout, marks)
        plan, layout, n_back = r["plan"], r["layout"], r["n_back"]
        full = not a.pieces or a.pieces >= len(init["pieces"])
        if getattr(a, "replace", 0):
            return self._replace_loop(plan, layout, marks, n_back)
        # (5) 置けたかの画像の確認(写す姿勢に戻ってから。盤の目標は事前の情報)。
        #   --pieces の回(2026-11-20 C42i): 置いたピースは盤の目標と、置かないピースは最初の上の絵の置き場(found0)と比べる(動いていないか)
        placed = {s["pid"] for s in plan["steps"]}
        o = self.obs("top")
        det = pl.ask("detect", img=o["img"], W=o["W"], H=o["H"])
        check = []
        if not det.get("ok"):
            self.say(f"[top after] detect failed: {det.get('why')}")
            return {"stopped": "final top image: " + str(det.get("why")), "n_back": n_back, "full": full, "img_check": None}
        for pid, (p, tgt) in enumerate(zip(det["layout"]["pieces"], self.prior["targets"])):
            ref, what = (tgt, "the board target") if pid in placed else (found0["pieces"][pid], "where it was at the start (not to be moved)")
            m = pl.need("match", pid=pid, est=ref, meas=p)
            ok = m["worst_cell_mm"] < PLACE_IMG_MM
            check.append({"name": p["name"], "img_off_mm": round(m["worst_cell_mm"], 2), "phi": round(m["phi"], 2), "ok": ok,
                          "against": "target" if pid in placed else "start"})
            self.say(f"[top after] {p['name']}: {m['worst_cell_mm']:.2f} mm from {what} ({'ok' if ok else 'NOT ok'})")
        bad = [c["name"] for c in check if not c["ok"]]
        if bad:   # 気づいたら理由付きで止める(拾い直しはまだ無い)
            return {"stopped": "final top image: " + ", ".join(f"{c['name']} {c['img_off_mm']:.1f} mm from the {c['against']}" for c in check if not c["ok"]),
                    "n_back": n_back, "full": full, "img_check": check}
        return {"stopped": None, "n_back": n_back, "full": full, "img_check": check}


def main():
    ap = argparse.ArgumentParser(usage=__doc__)
    ap.add_argument("--set", type=int, default=2)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--pieces", type=int, default=0, help="この数のピースを置いたら止める(0 = 全部)")
    ap.add_argument("--shift", default=None)
    ap.add_argument("--no-wrist", action="store_true")
    ap.add_argument("--noise", type=float, default=3.0)
    ap.add_argument("--arm-err", type=float, default=0.0, help="本当の腕が読んだ角度から関節ごとに最大この度ずれる(種で決める。puzzle_run.py と同じ引き方)")
    ap.add_argument("--arm-bias", default=None, help="同じことを決めた値で(URDF の度、shoulder_pan..wrist_roll の 5 個)")
    ap.add_argument("--leak-test", default=None)
    ap.add_argument("--replace", type=int, default=0, help="--pieces 1 と一緒に: 置いた後に上の絵で確かめ、ずれていればこの回数まで置き直す(計画書の欄: 各 2 回まで)")
    ap.add_argument("--place-comp", action="store_true", help="手首の確かめから関節のずれを見積もり、置く姿勢のずれを前もって打ち消す(class Kin)")
    ap.add_argument("--leak-other", default=None, help="逆向きの検算: 別の種の run.json(最初の上の画像を差し替える)")
    ap.add_argument("--comp-mode", choices=["offset", "est"], default="offset",
                    help="--place-comp の打ち消し方: offset = plan の place_offset で置く所だけずらす(既定)、est = 前の形(見積もりをずらす。つかむ所もずれる)")
    ap.add_argument("--world-fault", action="append", default=[],
                    help="世界の側の失敗(負の検算。制御へは渡さない): drop:PID:N(持つ指令の N 回目に落とす) | nudge:PID:T:DX:DZ(世界の時計 T 秒でピースを動かす)")
    ap.add_argument("--no-grip-check", action="store_true", help="持っている間の爪の読みの確かめを外す(比べる用)")
    a = ap.parse_args()
    if a.replace and a.pieces != 1:
        ap.error("--replace は --pieces 1 と一緒に使う(計画の口が、置き済みの手を飛ばせないため。2026-11-20 C42g)")
    # 制御へは世界の側の引数(関節の誤差 = 真値)を渡さない(点検 C42e の 6)
    a_ctl = argparse.Namespace(**{k: v for k, v in vars(a).items() if k not in ("arm_err", "arm_bias", "world_fault")})
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = "".join([f"_p{a.pieces}" if a.pieces else "", f"_shift{a.shift}" if a.shift else "", "_nowrist" if a.no_wrist else "",
                   "_leak" if a.leak_test else "", f"_armerr{a.arm_err:g}" if a.arm_err else "", "_armbias" if a.arm_bias else "", f"_replace{a.replace}" if a.replace else "", ("_comp" + ("est" if a.comp_mode == "est" else "")) if a.place_comp else "",
                   "_fault" if a.world_fault else "", "_nogrip" if a.no_grip_check else ""]).replace(",", "x")
    outdir = os.path.join(OUTROOT, f"{stamp}_mj_set{a.set}_seed{a.seed}{tag}_{os.getpid()}")   # 同じ秒に 2 本流すと重なった(23:21)
    os.makedirs(outdir, exist_ok=True)
    log = {"date": stamp, "args": vars(a), "events": []}

    def say(msg, **kw):
        print(msg, flush=True)
        log["events"].append(dict(msg=msg, **kw))

    proto_path = os.path.join(SIM, "out", "physics", f"set{a.set}_gap15_stand1", "scene.json")
    proto = json.load(open(proto_path, encoding="utf-8"))
    prior = {"targets": [{"name": p["name"], "x": p["end"]["pos"][0] * 1000, "z": p["end"]["pos"][2] * 1000, "yaw": p["end"]["rot"]}
                         for p in proto["pieces"]],
             "pitch_mm": proto["pitch"] * 1000, "cells": [p["cells"] for p in proto["pieces"]],
             "scene_json": proto_path, "scene_sha256": hashlib.sha256(open(proto_path, "rb").read()).hexdigest()}
    log["prior"] = prior
    pl = Planner()
    t0 = time.time()
    try:
        init = pl.need("init", set=a.set, k1="fake", calib="sim", calibMm=0, calibDeg=0, noise=a.noise, seed=a.seed)
        if a.leak_test:
            rec = json.load(open(a.leak_test, encoding="utf-8"))
            obs_seq = list(rec["observations"])
            sent = []

            def obs_fn(cam):
                o = obs_seq.pop(0)
                assert o["cam"] == cam, (o["cam"], cam)
                return o

            ctl = Controller(pl, init, prior, a_ctl, obs_fn, lambda f, dt: sent.append(1), say)
            res = ctl.run()
            assert not obs_seq, f"{len(obs_seq)} observations left unused"   # 点検 C42c の 1
            same = ctl.cmds == rec["commands"]
            # 逆向きの検算(点検 C42c の 1): 最初の上の画像だけ別の種の物に差し替えると、指令が変わるか(制御が画像を読んでいる証拠)
            other_changed = None
            if a.leak_other:
                oth = json.load(open(a.leak_other, encoding="utf-8"))
                seq2 = list(rec["observations"]); seq2[0] = dict(seq2[0], img=oth["observations"][0]["img"])

                def obs_fn2(cam):
                    if not seq2:
                        raise Stop("observations ran out (the controller took another path)")
                    if seq2[0]["cam"] != cam:   # 爪の読み(cam "joints")が入ってから、別の道で絵を頼むと img が無い(C42i で落ちた)
                        raise Stop(f"the controller asked for {cam} where the record has {seq2[0]['cam']} (another path)")
                    return seq2.pop(0)
                ctl2 = Controller(pl, init, prior, a_ctl, obs_fn2, lambda f, dt: None, lambda *x, **k: None)
                try:
                    ctl2.run()
                except Stop:
                    pass
                # 点検 C42i の 4: 止まる前に指令が 1 つでも記録と違ったときだけ「変わった」とする(すぐ止まって 0 個でも True にしない)
                diff_at = next((n for n, (x, y) in enumerate(zip(ctl2.cmds, rec["commands"])) if x != y), None)
                other_changed = diff_at is not None
                log["leak_reverse"] = {"first_diff_index": diff_at, "n_commands": len(ctl2.cmds)}
                say(f"leak test (reverse): first top image from {a.leak_other} -> commands changed: {other_changed} "
                    f"(first different command #{diff_at}, {len(ctl2.cmds)} commands before it stopped or ended)")
            h = hashlib.sha256(json.dumps(ctl.cmds).encode()).hexdigest()
            s0 = score(rec["scene"], rec["final_states"])
            s1 = score(rec["scene"], rec["final_states"], truth_shift=(3.0, 0.0))
            changed = [r0["place_err_mm"] != r1["place_err_mm"] for r0, r1 in zip(s0, s1)]
            log.update(leak={"commands_same": same, "n_commands": len(ctl.cmds), "sha256": h, "sha256_first": rec["commands_sha256"],
                             "score_truth": s0, "score_truth_shift3mm": s1, "score_changed": changed})
            say(f"leak test: commands same as the first run: {same} ({len(ctl.cmds)} commands); score changed with truth +3mm: {changed}")
            log["leak"]["other_image_changed_commands"] = other_changed
            log["result"] = "ok" if same and all(changed) and other_changed is not False else "ng"
            return
        truth = pl.need("truth", seed=a.seed)["layout"]
        rnd = __import__("random").Random(a.seed * 7 + 3)          # puzzle_run.py と同じ引き方
        bias = [rnd.uniform(-a.arm_err, a.arm_err) for _ in JOINTS]
        if a.arm_bias:
            bias = [float(v) for v in a.arm_bias.split(",")]
            assert len(bias) == len(JOINTS), "--arm-bias needs one value per joint"
        log["arm_bias_deg"] = bias
        if any(bias):
            say("(sim) true arm = read angles + " + ", ".join(f"{j} {b:+.2f}" for j, b in zip(JOINTS, bias)) + " deg")
        faults = []
        for fs in a.world_fault:
            k, *v = fs.split(":")
            faults.append(("drop", int(v[0]), int(v[1])) if k == "drop" else ("nudge", int(v[0]), float(v[1]), float(v[2]), float(v[3])))
        log["world_faults"] = faults
        world = World(pl, proto, truth, init["start_urdf"], init["start_jaw"], outdir, bias, faults=faults, say=say)
        obs_log = []
        conv_box = {}

        def obs_fn(cam):
            if "c" not in conv_box:          # 最初の上の絵は、計画の前なので、init の始まりの姿勢の組で換算する(写す姿勢は計画の frames[0] と同じ)
                q, jaw = world.joints_urdf()
                L = {j: init["start_lerobot"][j] + 0 for j in JOINTS}
                L["gripper"] = init["start_lerobot"]["gripper"]
                o = world_obs_with(cam, L)
            else:
                o = world.observe(cam, conv_box["c"])
            obs_log.append(o)
            return o

        def world_obs_with(cam, L):
            st = world.piece_states()
            lay = {"pieces": [{"name": s["name"], "x": s["x"], "z": s["z"], "yaw": s["yaw"]} for s in st]}
            world.nshot += 1
            out = os.path.join(outdir, f"shot_{world.nshot:02d}_{cam}.rgb")
            im = pl.need("render", cam=cam, truth=lay, hide=[], lerobot=L, bias=world.bias if any(world.bias) else None, out=out)
            q, jaw = world.joints_urdf()
            dmax = max(abs(x - y) for x, y in zip(q, init["start_urdf"]))
            if dmax > 2.0:   # 点検 C42c の 4: 換算が計画の前に無いので、測った角度(URDF の度)で始まりの姿勢との差を見る
                raise Stop(f"not at the start pose before the first image: measured {dmax:.2f} deg (URDF) away")
            return {"cam": cam, "img": im["out"], "W": im["W"], "H": im["H"], "lerobot": L, "t_world": round(world.t, 4),
                    "note": "before the plan: joints = init start pose (the arm was set there; measured max diff %.3f deg)" %
                            max(abs(x - y) for x, y in zip(q, init["start_urdf"]))}

        def send(f, dt):
            # 世界が関節を LeRobot の度で返すための換算は、Controller が計画から作った物を使う(計画の数字。真値ではない)
            if "c" not in conv_box and hasattr(ctl, "conv"):
                conv_box["c"] = ctl.conv
            world.command(f, dt)

        ctl = Controller(pl, init, prior, a_ctl, obs_fn, send, say)
        res = ctl.run()
        if res.get("stopped"):
            say(f"!! controller stopped: {res['stopped']}")
        states = world.piece_states()
        rows = score(world.scene, states)
        for r in rows:
            say(f"  [score] {r['name']}: {r['place_err_mm']:.2f} mm, yaw {r['yaw_err_deg']:+.2f}, tilt {r['tilt_deg']:.1f}, lift {r['lift_mm']:.1f}: {'OK' if r['ok'] else 'NG'}")
        n_ok = sum(r["ok"] for r in rows)
        need = len(rows) if res["full"] else a.pieces
        log.update(truth=truth, result_ctrl=res, score=rows, final_states=states, scene=world.scene,
                   observations=obs_log, commands=ctl.cmds, commands_sha256=hashlib.sha256(json.dumps(ctl.cmds).encode()).hexdigest())
        img_ok = res.get("img_check") is None or all(c["ok"] for c in res["img_check"])
        log["result"] = "ok" if (res["stopped"] is None and n_ok >= need and img_ok) else "ng"
        say(f"result: {log['result'].upper()} (score {n_ok}/{len(rows)} placed, needed {need}; image check {'-' if res.get('img_check') is None else sum(c['ok'] for c in res['img_check'])}) "
            f"world {world.t:.1f} s, wall {time.time() - t0:.0f} s")
    except Stop as e:
        log["result"] = "stopped"
        say(f"!! stopped: {e}")
        # 止まった回も、後で読めるように中身を残す(点検 C42g の 4。採点の真値は記録だけ。制御へは渡らない)
        lv = locals()
        if "ctl" in lv and "commands" not in log:
            log["commands"] = ctl.cmds
            if "obs_log" in lv:
                log["observations"] = lv["obs_log"]
            if "world" in lv:
                log.update(truth=lv.get("truth"), scene=world.scene, final_states=world.piece_states())
    finally:
        pl.close()
        out = os.path.join(outdir, "run.json")
        with open(out, "w", encoding="utf-8") as fo:
            json.dump(log, fo, indent=1, ensure_ascii=False, default=float)
        print(f"wrote {out}", flush=True)
    sys.exit(0 if log.get("result") == "ok" else 1)


if __name__ == "__main__":
    main()
