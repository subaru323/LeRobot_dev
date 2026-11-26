"""S2 (総合計画書の S2「一手の画像制御と物理」、2026-11-22): tools/puzzle_run.py --mj の世界。MuJoCo のピースは自由な剛体。

puzzle_run.py の通常の制御(実機と同じ輪)から見ると、SimRobot・RealRobot と同じ口(関節を読む・関節を送る・カメラを読む)を持つ
「ロボット」。制御へは関節の読み(LeRobot の度)と絵のファイルしか返さない。真値(MuJoCo のピースの体の姿勢)はこの中だけにある。

  関節: 指令(LeRobot の度)を plan-cli map で URDF の度にし、関節の誤差 bias(--arm-err)を足して MuJoCo の位置の指令にする。
        1 歩(puzzle_run.py は 1 回に STEP_DEG まで)を、いちばん大きく動く関節が SPEED_DEG_S で動く時間(POLL_S より短ければ POLL_S)で補い、その後は
        実機の send と同じく、どの関節も REACH_TOL_DEG 以内に着くまで(最長 STEP_TIMEOUT_S)世界を進める。
        読みは MuJoCo の角度 − bias を plan-cli map で LeRobot の度にした物。
  爪:   実機(RealRobot.grip_goal_for)と同じ決め方: 閉じるときは計画の値より GRIP_SQUEEZE 深く狙う。実機は力が GRIP_LOAD_STOP を
        超えたら閉じるのを止めるが、ここでは MuJoCo の爪の力の上限(run_physics.GRIP_NM)がその代わり(止めはしない)。
        爪は待たない(実機と同じ)。
  絵:   plan-cli render に、MuJoCo のいまのピースの姿勢と関節の読みを渡して描く。机から LIFT_HIDE_MM より浮いたピースは描かない
        (爪の中)。机の上で TILT_HIDE_DEG より傾いたピースは描けないので WorldCannotDraw で止める(制御の判断ではない)。
  置き場: --pieces で動かさないピースは、計画が仮定する既定の置き場(plan-cli truth の shiftMm 0・yawDeg 0)に置く。動かすピースは
        真値の置き場(plan-cli truth の種)。
  書き戻し: ピースの自由な関節を書くのは、初めに組むときと、宣言した世界の失敗(faults の nudge・drop)だけ。書いたら
        piece_writes に残す(S2 の完了の条件「物体を指令位置に書き戻さない」の確かめ)。持つ間の溶接・拘束も無い。

世界の失敗と遮るもの(制御へは渡さない。tools/s2_run.py の回が使う):
  faults: [{"kind": "nudge", "piece": 名前, "dx_mm", "dz_mm", "dyaw"(度・既定 0), "t": 世界の秒 | "after_shot": ["wrist", n]}]
              その時刻を過ぎたら・そのカメラの n 枚目を撮った直後に、机の上で動かす(誰かが動かした扱い)
          [{"kind": "drop", "piece": 名前, "lift_mm": 浮き}]   そのピースが机からこれより浮いたら、爪から外して真下の机へ落とす
          [{"kind": "weak_grip", "nm": 力}]                                       爪の力の上限を初めから下げる(握りが弱い。書き戻しは無い)
          [{"kind": "place", "piece": 名前, "x_mm", "z_mm", "yaw"}]               初めの置き場を変える(組むときだけ。書き戻しではない)
  occluders: [{"cam": "top"|"wrist", "shots": [n, ...] | "all", "box": {x_mm, z_mm, y_mm, w_mm, d_mm, t_mm, rgb}}]
          そのカメラの n 枚目(1 から)の絵だけに箱を描く(手・紙が入った扱い。当たりは無い)

S3(総合計画書の S3「誤差と全体の組立」、2026-11-23)で足したもの:
  errors: 世界の誤差(制御へは渡さない。tools/s3_run.py の回が使う。どれも省けば前と同じ)
    rep_deg: 関節のくり返しばらつき。送るたびに腕の 5 関節へ平均 0・標準偏差この度の隠れたずれ(遊び。読みには出ない)。rep_seed で種
    sag_deg_per_nm: たわみ。送るたびに、送る前の重さの力(qfrc_bias。N·m)× この値の度だけ、肩・肘・手首の曲げが重さの向きへ下がる
                    (読みには出ない。歯車の遊びと腕のしなりの代わり)
    table_mm: 机の高さ。机が腕の土台から見てこの mm 高い(負は低い)。絵は描き換えない(描く道具は既定の高さのまま)
    friction: ピース・机・爪の摩擦(run_physics.FRICTION の代わり)
    start_dyaw: 動かすピースの初めの向きを、この度だけ回す(盤に置く計画は上の絵の読みから作るので、制御はこれを絵で知る)
    light: {"top": {...}, "wrist": {...}}: plan-cli render の光の乱れ(sim/vision.js の opts.light: gain・elev・azim・shadow・blob など)
  隠れたずれ(bias + くり返し + たわみ)は、送る指令に足し、読みから引き、絵の腕の姿勢に足す(関節の誤差 bias と同じ扱い)。
  実機の送り(RealRobot.send)と同じ止まり: 腕の 5 関節のどれかの力が LOAD_ABORT_NM を超えた(0.05 秒ごとに見る)、STEP_TIMEOUT_S で
    着かない。どちらも puzzle_run の Stop(制御の側の止まり。実機では送りの中の読みで止まる)。S2 までは着かないのを WorldStop にしていた。
  接触の記録 rec["contacts"]: 腕とピース・ピースとピース・腕と机の接触を、時刻の区間と最大のめり込みで残す。rec["lifts"]: ピースごとの
    机から 1mm より浮いた区間。どの接触が許されるか(つかむピースか)は classify_contacts が決める(採点の側)
"""

import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
SIM = os.path.join(ROOT, "sim")
sys.path.insert(0, os.path.join(SIM, "physics"))
import numpy as np                     # noqa: E402
import mujoco                          # noqa: E402
import run_physics as RP               # noqa: E402  (読むだけ: build・姿勢の読み)

SETTLE_S = 0.5          # 初めに組んだ後、落ち着かせる世界の秒
SPEED_DEG_S = 44.0      # 1 歩を補う速さ。実機の既定 --speed 500 steps/s ≈ 500 × 360 / 4096 = 44 度/秒(上限の値。加速は見ない)
POLL_S = 0.05           # puzzle_run.py の実機の send は、書いてから 0.05 秒待って読む。爪だけが動く歩もこれだけ進める
                        #   (2026-11-22 S2-13: 前は爪だけの歩を 1ms で進め、爪が閉じ終わる前に持ち上げ始めて J をつかみ損ねた)
REACH_TOL_DEG = 1.5     # puzzle_run.py の実機と同じ: どの関節もこれ以内に着いたら次の歩
STEP_TIMEOUT_S = 6.0    # puzzle_run.py の実機と同じ
GRIP_SQUEEZE = 5.0      # puzzle_run.py の実機と同じ(0〜100 の爪の値)
LIFT_HIDE_MM = 3.0      # tools/puzzle_mj.py と同じ
TILT_HIDE_DEG = 5.0
LOAD_ABORT_NM = 0.6 * 3.35   # puzzle_run.py の LOAD_ABORT 600(Present_Load の 0〜1000)を、模型の力の上限 3.35 N·m の 0.6 とみなす【仮】
LOAD_POLL_STEPS = 50         # 0.05 秒ごとに力を見る(実機の送りの読み直しと同じ間)
CONTACT_GAP_S = 0.05         # 同じ組の接触がこれより空いたら別の区間
LIFT_REC_MM = 1.0            # 浮いた区間の線(採点の浮き 1mm と同じ)
DIST_EVERY = 50              # 腕とピースのいちばん近い所を、0.05 秒ごとに測る
DIST_MAX_M = 0.02            # これより遠い所は測らない(20mm 以上として残さない)
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def _hit(stop_cls, name):
    """puzzle_run の Stop を継いだ「世界が描けない」(制御の判断と分けて数える)"""
    return type(name, (stop_cls,), {})


class MjRobot:
    torque = False

    def __init__(self, planner, set_no, truth, defaults, active, start_L, bias, folder, stop_cls, say,
                 faults=None, occluders=None, errors=None, gap=None):
        self.pl, self.folder, self.say, self.Stop = planner, folder, say, stop_cls
        self.errors = dict(errors or {})
        known = {"rep_deg", "rep_seed", "sag_deg_per_nm", "table_mm", "friction", "start_dyaw", "light"}
        if set(self.errors) - known:
            raise ValueError(f"mj errors: unknown keys {sorted(set(self.errors) - known)}")
        self.rng = np.random.default_rng(int(self.errors.get("rep_seed", 0)))
        self.jit = [0.0] * 5
        self.sagoff = [0.0] * 5
        self.WorldCannotDraw = _hit(stop_cls, "WorldCannotDraw")
        self.WorldStop = _hit(stop_cls, "WorldStop")     # the world could not do what was sent (not the control's own stop. review C60 7)
        self.bias = list(bias)
        self.faults = [dict(f, done=False) for f in (faults or [])]
        self.occluders = list(occluders or [])
        self.nshot = {"top": 0, "wrist": 0}
        self.piece_writes = []                 # ピースの姿勢を書いた所(初めに組むのは入れない)
        self.rec = {"reads": 0, "sends": 0, "shots": []}
        proto_path = os.path.join(SIM, "out", "physics", f"set{set_no}_gap{(15 if gap is None else gap):g}_stand1", "scene.json")
        if not os.path.exists(proto_path):
            raise FileNotFoundError(f"{proto_path}: make it with node sim/tools/export-physics.js --set {set_no} --gap {(15 if gap is None else gap):g}")
        self.proto_path = proto_path
        proto = json.load(open(proto_path, encoding="utf-8"))
        sc = json.loads(json.dumps(proto))
        start = {}
        for pc in sc["pieces"]:
            src = truth if pc["idx"] in active else defaults
            t = next(p for p in src["pieces"] if p["name"] == pc["name"])
            start[pc["name"]] = {"x": t["x"], "z": t["z"], "yaw": t["yaw"] + (self.errors.get("start_dyaw", 0.0) if pc["idx"] in active else 0.0)}
        for f in self.faults:
            if f["kind"] == "place":
                start[f["piece"]] = {"x": f["x_mm"], "z": f["z_mm"], "yaw": f["yaw"]}
                f["done"] = True
        for pc in sc["pieces"]:
            s = start[pc["name"]]
            pc["start"] = {"pos": [s["x"] / 1000, 0.0, s["z"] / 1000], "rot": s["yaw"]}
        self.start_layout = {"pieces": [dict(name=n, **v) for n, v in start.items()]}
        if self.errors.get("table_mm"):
            sc["robotY"] = sc["robotY"] - self.errors["table_mm"] / 1000.0
        self.scene = sc
        fr0 = RP.FRICTION
        if self.errors.get("friction") is not None:
            RP.FRICTION = float(self.errors["friction"])
        try:
            self.model = RP.build(sc, folder, write_mjcf=False)
        finally:
            RP.FRICTION = fr0
        for f in self.faults:
            if f["kind"] == "weak_grip":
                a = self.model.actuator("gripper").id if self._has_act("gripper") else self.model.nu - 1
                self.model.actuator_forcerange[a] = [-f["nm"], f["nm"]]
                f["done"] = True
                say(f"(world) gripper force limit {f['nm']} Nm (default {RP.GRIP_NM})", world_fault="weak_grip")
        self.data = mujoco.MjData(self.model)
        self.jid = [self.model.joint(n).qposadr[0] for n in ARM_JOINTS] + [self.model.joint("gripper").qposadr[0]]
        self.dof = [self.model.joint(n).dofadr[0] for n in ARM_JOINTS]
        self.aid = [self.model.actuator(n).id for n in ARM_JOINTS]
        self._contact_setup()
        u = self._to_urdf(start_L)
        self.ctrl_deg = [a + b for a, b in zip(u["urdf"], self.hidden())] + [u["jaw"]]
        for k, v in zip(self.jid, self.ctrl_deg):
            self.data.qpos[k] = math.radians(v)
        self.data.ctrl[:] = np.radians(self.ctrl_deg)
        mujoco.mj_forward(self.model, self.data)
        self.t = 0.0
        self.grip_plan = self.grip_goal = None
        self._run_for(SETTLE_S)

    def hidden(self):
        """the hidden joint offsets (deg, URDF): the fixed bias + this step's repeatability + sag. Never shown to the control"""
        return [b + j + g for b, j, g in zip(self.bias, self.jit, self.sagoff)]

    def _contact_setup(self):
        m = self.model
        base_root = m.body_rootid[m.body("base").id]
        self.pc_body = {}
        for pc in self.scene["pieces"]:
            self.pc_body[m.body("pc%d" % pc["idx"]).id] = pc["name"]
        self.g_owner = []
        for g in range(m.ngeom):
            b = m.geom_bodyid[g]
            if b in self.pc_body:
                self.g_owner.append(("piece", self.pc_body[b]))
            elif m.body_rootid[b] == base_root and b != 0:
                self.g_owner.append(("arm", m.body(b).name))
            else:
                self.g_owner.append(("world", "table"))
        self.pc_ids = np.array(sorted(self.pc_body))
        self.pc_names = [self.pc_body[b] for b in self.pc_ids]
        self.arm_geoms = [g for g in range(m.ngeom) if self.g_owner[g][0] == "arm" and (m.geom_contype[g] & 2 or m.geom_conaffinity[g] & 2)]
        self.pc_geoms = {n: [g for g in range(m.ngeom) if self.g_owner[g] == ("piece", n)] for n in self.pc_body.values()}
        self.near = []                         # [t, piece, min distance mm] when below DIST_MAX_M
        self.nstep = 0
        self.contacts = {}                     # (kind, piece, other) -> [[t0, t1, max_pen_mm], ...]
        self.lifts = {n: [] for n in self.pc_names}
        self.max_load = [0.0] * 5

    def _record_contacts(self):
        d, t = self.data, self.t
        for i in range(d.ncon):
            c = d.contact[i]
            if c.dist > 1e-5:
                continue
            a, b = self.g_owner[c.geom1], self.g_owner[c.geom2]
            if a[0] == "piece" and b[0] == "piece":
                p, q = sorted((a[1], b[1]))
                key = ("piece-piece", p, q)
            elif "piece" in (a[0], b[0]) and "arm" in (a[0], b[0]):
                pc, arm = (a, b) if a[0] == "piece" else (b, a)
                key = ("arm-piece", pc[1], arm[1])
            elif "arm" in (a[0], b[0]) and "world" in (a[0], b[0]):
                arm = a if a[0] == "arm" else b
                key = ("arm-table", "", arm[1])
            else:
                continue
            pen = max(0.0, -c.dist) * 1000
            iv = self.contacts.setdefault(key, [])
            if iv and t - iv[-1][1] <= CONTACT_GAP_S:
                iv[-1][1] = t
                iv[-1][2] = max(iv[-1][2], pen)
            else:
                iv.append([t, t, pen])
        z = d.xpos[self.pc_ids, 2] * 1000 - self.scene["cell"] * 1000 / 2
        for n, h in zip(self.pc_names, z):
            if h > LIFT_REC_MM:
                iv = self.lifts[n]
                if iv and t - iv[-1][1] <= 0.2:
                    iv[-1][1] = t
                else:
                    iv.append([t, t])

    def _record_near(self):
        fromto = np.zeros(6)
        for n, gs in self.pc_geoms.items():
            best = DIST_MAX_M
            for ga in self.arm_geoms:
                for gb in gs:
                    d = mujoco.mj_geomDistance(self.model, self.data, ga, gb, best, fromto)
                    if d < best:
                        best = d
            if best < DIST_MAX_M:
                self.near.append([round(self.t, 3), n, round(best * 1000, 3)])

    def contact_log(self):
        out = [{"kind": k[0], "piece": k[1], "other": k[2], "t0": round(iv[0], 3), "t1": round(iv[1], 3), "pen_mm": round(iv[2], 3)}
               for k, ivs in sorted(self.contacts.items()) for iv in ivs]
        return {"contacts": out, "lifts": {n: [[round(a, 3), round(b, 3)] for a, b in v] for n, v in self.lifts.items()},
                "max_load_nm": {j: round(v, 3) for j, v in zip(ARM_JOINTS, self.max_load)}, "load_abort_nm": LOAD_ABORT_NM,
                "near": self.near}

    def _has_act(self, name):
        try:
            self.model.actuator(name)
            return True
        except KeyError:
            return False

    # ---- 換算(plan-cli map。Python は換算しない)
    def _to_urdf(self, L):
        return self.pl.need("map", lerobot={k: float(v) for k, v in L.items()})

    def _to_L(self, q, jaw):
        return self.pl.need("map", urdf=[float(v) for v in q], jaw=float(jaw))["lerobot"]

    # ---- 世界を進める
    def _step(self):
        mujoco.mj_step(self.model, self.data)
        self.t += self.model.opt.timestep
        self._faults()
        self._record_contacts()
        self.nstep += 1
        if self.nstep % DIST_EVERY == 0:
            self._record_near()

    def _run_for(self, dt):
        for _ in range(max(1, int(round(dt / self.model.opt.timestep)))):
            self._step()

    def _faults(self):
        for f in self.faults:
            if f["done"] or f["kind"] not in ("nudge", "drop") or (f["kind"] == "nudge" and ("t" not in f or self.t < f["t"])):
                continue
            pid = next(pc["idx"] for pc in self.scene["pieces"] if pc["name"] == f["piece"])
            st = self.piece_states()[pid]
            if f["kind"] == "nudge":
                self._write_piece(pid, st["x"] + f["dx_mm"], st["z"] + f["dz_mm"], 0.0, why="fault nudge", dyaw=f.get("dyaw", 0.0))
                self.say(f"(world fault) {f['piece']} was moved by {f['dx_mm']:+.1f},{f['dz_mm']:+.1f} mm at t {self.t:.2f} s", world_fault="nudge")
                f["done"] = True
            elif f["kind"] == "drop" and st["lift_mm"] > f.get("lift_mm", LIFT_HIDE_MM):
                self._write_piece(pid, st["x"], st["z"], 0.002, why="fault drop")
                self.say(f"(world fault) {f['piece']} slipped out of the gripper at t {self.t:.2f} s (fell under the jaw)", world_fault="drop")
                f["done"] = True

    def _write_piece(self, pid, x_mm, z_mm, lift_m, why, dyaw=0.0):
        """ピースの自由な関節を書く。世界の失敗だけが使う(書いたら piece_writes に残す)。dyaw: 体の基準(形の [0,0] のマス)の回りに回す度"""
        jn = self.model.joint("pc%d_free" % self.scene["pieces"][pid]["idx"])
        a, d = jn.qposadr[0], jn.dofadr[0]
        p = RP.to_mj([x_mm / 1000, 0.0, z_mm / 1000])
        self.data.qpos[a:a + 3] = [p[0], p[1], self.scene["cell"] / 2 + lift_m]
        if dyaw:
            q = np.zeros(4)
            mujoco.mju_mulQuat(q, np.array(RP.quat_z(-dyaw), dtype=float), self.data.qpos[a + 3:a + 7].copy())
            self.data.qpos[a + 3:a + 7] = q
        self.data.qvel[d:d + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.piece_writes.append({"t": round(self.t, 4), "piece": self.scene["pieces"][pid]["name"], "why": why})

    # ---- 真値(この中と採点だけ)
    def piece_states(self):
        out = []
        for pc in self.scene["pieces"]:
            cs, R = RP.cell_centers_actual(self.scene, pc, self.data)
            b = self.data.body("pc%d" % pc["idx"]).xpos
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
            out.append({"name": pc["name"], "x": float(b[0]) * 1000, "z": float(-b[1]) * 1000, "yaw": RP.yaw_of(R),
                        "lift_mm": float(np.min(cs[:, 2]) - self.scene["cell"] / 2) * 1000, "tilt": tilt})
        return out

    @property
    def truth(self):
        """puzzle_run.py が拾った瞬間の世界を記録に残す(制御の判断には使わない)"""
        return {"pieces": [{"name": s["name"], "x": s["x"], "z": s["z"], "yaw": s["yaw"]} for s in self.piece_states()]}

    # ---- ロボットの口(puzzle_run.py の SimRobot・RealRobot と同じ)
    def connect_no_torque(self):
        pass

    def limit_and_torque_on_in_place(self):
        self.torque = True

    def read_joints(self) -> dict:
        self.rec["reads"] += 1
        q = [math.degrees(self.data.qpos[k]) - b for k, b in zip(self.jid[:5], self.hidden())]
        return self._to_L(q, math.degrees(self.data.qpos[self.jid[5]]))

    def grip_goal_for(self, planned: float) -> float:
        """RealRobot.grip_goal_for と同じ(力で止めた所を覚える部分は、MuJoCo の力の上限が代わる)"""
        prev = self.grip_plan
        if prev is None or planned > prev + 1e-6:
            g = planned
        elif planned < prev - 1e-6:
            g = planned - GRIP_SQUEEZE
        else:
            g = self.grip_goal
        self.grip_plan, self.grip_goal = planned, g
        return g

    def send(self, target: dict):
        self.rec["sends"] += 1
        goal = dict(target)
        if "gripper" in goal:
            goal["gripper"] = self.grip_goal_for(goal["gripper"])
        u = self._to_urdf(goal)
        if self.errors.get("rep_deg"):
            self.jit = [float(v) for v in self.rng.normal(0.0, float(self.errors["rep_deg"]), 5)]
        if self.errors.get("sag_deg_per_nm"):
            k = float(self.errors["sag_deg_per_nm"])
            self.sagoff = [0.0] + [-k * float(self.data.qfrc_bias[self.dof[j]]) for j in (1, 2, 3)] + [0.0]
        new = [a + b for a, b in zip(u["urdf"], self.hidden())] + [u["jaw"]]
        old = list(self.ctrl_deg)
        dt = max(max(abs(x - y) for x, y in zip(new[:5], old[:5])) / SPEED_DEG_S, POLL_S)
        n = max(1, int(round(dt / self.model.opt.timestep)))
        for i in range(n):
            w = (i + 1) / n
            self.data.ctrl[:] = np.radians([a + (b - a) * w for a, b in zip(old, new)])
            self._step()
            if (i + 1) % LOAD_POLL_STEPS == 0:
                self._load_check()
        self.ctrl_deg = new
        t0, k = self.t, 0
        while True:
            q = [math.degrees(self.data.qpos[j]) for j in self.jid[:5]]
            if max(abs(x - y) for x, y in zip(q, new[:5])) < REACH_TOL_DEG:
                return
            if self.t - t0 > STEP_TIMEOUT_S:      # RealRobot.send stops the same way (the control's own stop. S3)
                raise self.Stop(f"did not reach the step in {STEP_TIMEOUT_S}s")
            self._step()
            k += 1
            if k % LOAD_POLL_STEPS == 0:
                self._load_check()

    def _load_check(self):
        """RealRobot.send: a joint (not the gripper) above LOAD_ABORT -> Stop. Here the actuator force of the 5 arm joints"""
        f = [abs(float(self.data.actuator_force[a])) for a in self.aid]
        self.max_load = [max(x, y) for x, y in zip(self.max_load, f)]
        j = int(np.argmax(f))
        if f[j] > LOAD_ABORT_NM:
            raise self.Stop(f"high load on {ARM_JOINTS[j]}: {f[j]:.2f} N m (> {LOAD_ABORT_NM:.2f})")

    def read_camera(self, name: str) -> dict:
        self.nshot[name] += 1
        n = self.nshot[name]
        st = self.piece_states()
        for s in st:
            if s["lift_mm"] <= LIFT_HIDE_MM and s["tilt"] > TILT_HIDE_DEG:
                raise self.WorldCannotDraw(f"world: {s['name']} is tilted {s['tilt']:.1f} deg on the table; the renderer cannot draw it")
        lay = {"pieces": [{"name": s["name"], "x": s["x"], "z": s["z"], "yaw": s["yaw"]} for s in st]}
        hide = [i for i, s in enumerate(st) if s["lift_mm"] > LIFT_HIDE_MM]
        occ = [o["box"] for o in self.occluders if o["cam"] == name and (o["shots"] == "all" or n in o["shots"])]
        L = self.read_joints()
        out = os.path.join(self.folder, f"shot_{name}_{n:02d}.rgb")
        kw = {"occluders": occ} if occ else {}
        lt = (self.errors.get("light") or {}).get(name)
        if lt:
            kw["light"] = lt
        hb = self.hidden()
        im = self.pl.need("render", cam=name, truth=lay, hide=hide, lerobot=L, bias=hb if any(hb) else None, out=out, **kw)
        self.rec["shots"].append({"cam": name, "n": n, "t_world": round(self.t, 4), "out": im["out"], "occluded": bool(occ), "hidden": hide})
        for f in self.faults:                  # 撮った直後に起こす世界の失敗(制御は、この絵を見た後に変わった世界を知らない)
            if not f["done"] and f["kind"] == "nudge" and f.get("after_shot") == [name, n]:
                pid = next(pc["idx"] for pc in self.scene["pieces"] if pc["name"] == f["piece"])
                s = self.piece_states()[pid]
                self._write_piece(pid, s["x"] + f["dx_mm"], s["z"] + f["dz_mm"], 0.0, why="fault nudge after %s %d" % (name, n), dyaw=f.get("dyaw", 0.0))
                self.say(f"(world fault) {f['piece']} was moved by {f['dx_mm']:+.1f},{f['dz_mm']:+.1f} mm right after {name} picture {n}", world_fault="nudge")
                f["done"] = True
        return im

    def hold(self):
        pass

    def release(self):
        self.torque = False

    # ---- 採点(真値。制御とは別。tools/puzzle_mj.py の score と同じ線)
    def score(self, active, truth_shift=(0.0, 0.0)):
        """動かすピース: マスの真ん中が盤の目標(scene の end)から 3mm 未満・向き 5 度・傾き 5 度・浮き 1mm 未満。
        動かさないピース: 初めの置き場からマスの真ん中が 3mm 未満・傾き 5 度未満(押されていない)。truth_shift は漏れの検算で真値だけをずらす"""
        return score_states(self.scene, self.start_layout, self.piece_states(), active, truth_shift)


def _cells(scene, pc, x_mm, z_mm, yaw):
    P = scene["pitch"]
    r = math.radians(yaw)
    c, s = math.cos(r), math.sin(r)
    return np.array([[x_mm / 1000 + (a * P) * c - (b * P) * s, z_mm / 1000 + (a * P) * s + (b * P) * c] for a, b in pc["cells"]])


def _worst(A, B):
    d = np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2)
    return float(np.max(np.min(d, axis=1))) * 1000, sorted(np.argmin(d, axis=1).tolist()) == list(range(len(B)))


def score_states(scene, start_layout, states, active, truth_shift=(0.0, 0.0)):
    rows = []
    for pc, s in zip(scene["pieces"], states):
        act = _cells(scene, pc, s["x"] + truth_shift[0], s["z"] + truth_shift[1], s["yaw"])
        if pc["idx"] in active:
            end = pc["end"]
            exp = _cells(scene, pc, end["pos"][0] * 1000, end["pos"][2] * 1000, end["rot"])
            e, one2one = _worst(exp, act)
            yaw_err = ((s["yaw"] - end["rot"] + 45) % 90) - 45
            ok = one2one and e < RP.TOL_MM and abs(yaw_err) < RP.TOL_DEG and s["tilt"] < 5 and s["lift_mm"] < 1.0
            rows.append({"name": pc["name"], "moved": True, "place_err_mm": round(e, 3), "yaw_err_deg": round(yaw_err, 3),
                         "tilt_deg": round(s["tilt"], 2), "lift_mm": round(s["lift_mm"], 2), "ok": ok})
        else:
            st = next(p for p in start_layout["pieces"] if p["name"] == pc["name"])
            e, _ = _worst(_cells(scene, pc, st["x"], st["z"], st["yaw"]), act)
            ok = e < RP.TOL_MM and s["tilt"] < 5
            rows.append({"name": pc["name"], "moved": False, "pushed_mm": round(e, 3), "tilt_deg": round(s["tilt"], 2), "ok": ok})
    return rows


def classify_contacts(clog, active_names):
    """S3 (総合計画書 §4): which contacts are allowed. Moves come from the lifts: the pieces lifted, in time order (one piece's
    lifts in a row are one move). The time between two moves is split at the middle. In a move's span the arm may touch that
    piece (grip) and nothing else; a piece not lifted at all is never allowed to be touched (a non-moved piece) -- except when no
    piece was lifted at all and the piece is one to move: then 判定不能 (a miss: whether the touch was the intended grip is not known).
    piece-piece: the moving piece against another = "対象と隣" (recorded; the score's pushed column decides), others = "隣どうし".
    arm-table: recorded apart (the fingertips meant to stay 3 mm above the table)."""
    ivs = sorted((a, b, n) for n, v in clog["lifts"].items() for a, b in v)
    moves = []
    for a, b, n in ivs:
        if moves and moves[-1][2] == n:
            moves[-1][1] = max(moves[-1][1], b)
        else:
            moves.append([a, b, n])
    edges = [(moves[i][1] + moves[i + 1][0]) / 2 for i in range(len(moves) - 1)]

    def target_at(t):
        if not moves:
            return None
        k = sum(1 for e in edges if t >= e)
        return moves[k][2]

    rows = []
    for c in clog["contacts"]:
        tg = target_at(c["t0"])
        if c["kind"] == "arm-piece":
            if tg is None:
                v = "判定不能" if c["piece"] in active_names else "禁止接触"
            else:
                v = "把持" if c["piece"] == tg and target_at(c["t1"]) == tg else "禁止接触"
        elif c["kind"] == "piece-piece":
            v = "対象と隣" if tg in (c["piece"], c["other"]) else "隣どうし"
        else:
            v = "腕と机"
        rows.append(dict(c, verdict=v, target=tg))
    summ = {k: sum(1 for r in rows if r["verdict"] == k) for k in ("把持", "禁止接触", "判定不能", "対象と隣", "隣どうし", "腕と机")}
    near_other = {}                            # per move: the arm's nearest approach to a piece other than that move's piece (mm, sampled)
    for t, n, d in clog.get("near", []):
        tg = target_at(t)
        if tg is not None and n != tg:
            near_other[tg] = min(near_other.get(tg, DIST_MAX_M * 1000), d)
        elif tg is None and n not in active_names:
            near_other["-"] = min(near_other.get("-", DIST_MAX_M * 1000), d)
    worst = max((r["pen_mm"] for r in rows if r["verdict"] == "禁止接触"), default=None)
    return {"moves": [{"piece": n, "t0": round(a, 3), "t1": round(b, 3), "near_other_mm": near_other.get(n)} for a, b, n in moves],
            "rows": rows, "count": summ,
            "forbidden_max_pen_mm": worst}
