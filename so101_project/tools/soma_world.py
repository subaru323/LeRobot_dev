"""The MuJoCo world of tools/puzzle_run.py --soma --mj (2026-11-24): the SO-101 (the same model and joint handling as
tools/mj_world.py MjRobot) builds the Soma cube with free rigid 3D pieces.

To the control it is the same "robot" as MjRobot (read joints, send joints): joint readings in LeRobot degrees go out, the truth
(the pieces' bodies) stays in here. Differences from MjRobot:
  - the pieces: one free body per piece, one box per cell: (across x across) on the table, the cell's height tall (the path file's
    across_m = 40 - c mm and cell_m = 40 mm; no chamfer), MASS_G_PER_CELL each. They start where the path file (tools/
    soma_collide.js --path-json) puts them on the table (the person sets them out so). chamfer_m > 0: every cell's 12 edges cut at
    45 deg (as tools/soma_mj.py --chamfer-mm; the real print would cut only the piece's outer edges). The body's origin is its first cell on the
    table's bottom layer (MjRobot's "lifts" then read that cell's lift).
  - the stand under the base: a box as in tools/soma_collide.js (the base's footprint + 5 mm, the table up to the base). It touches
    the arm's links and the pieces (the base itself is fixed: MuJoCo does not touch two fixed things).
  - no camera: read_camera stops (the 3D build has no picture reading yet).
  - no faults, no occluders. World errors: rep_deg, rep_seed, sag_deg_per_nm, friction, table_mm (as MjRobot), and absent: [piece
    numbers from 0] (those pieces lie 450 mm behind their spots: the gripper closes on nothing; a negative check of the grip check).
  - score(): each piece's cells against their places in the cube (the path file's cube_cells).
  - 2026-11-26 (tools/puzzle_run.py --soma-correct): measure_tops() is the top camera's stand-in (the true top cells + noise
    cam_sigma_mm, cam_seed; only at the path's photo pose); rep_moved_only: true draws the wobble again only for the joints told
    to move (MjRobot draws all 5 at every send); offsets() is the truth for the log.
"""

import json
import math
import os

import numpy as np
import mujoco

import mj_world as MW
from mj_world import MjRobot, RP

MASS_G_PER_CELL = 12.0      # 【仮】as tools/soma_mj.py (a 1 mm shell)
PLACE_MM = 3.0              # score: each cell's centre within this of its place across (RP.TOL_MM)
SEAT_MM = 1.0               # score: and within this up / down (seated on what is under it; tools/soma_mj.py)
TILT_DEG = 5.0              # score: and the piece tilted less than this
PROTO = os.path.join(MW.SIM, "out", "physics", "set2_gap15_stand1", "scene.json")   # the robot, joints and the camera rig only


def screen_of(p):
    """MuJoCo (Z up) -> screen (Y up)"""
    return [float(p[0]), float(p[2]), float(-p[1])]


class SomaMjRobot(MjRobot):
    def __init__(self, planner, path, start_L, bias, folder, stop_cls, say, errors=None, mass_g_per_cell=MASS_G_PER_CELL, chamfer_m=0.0):
        self.pl, self.folder, self.say, self.Stop = planner, folder, say, stop_cls
        self.errors = dict(errors or {})
        known = {"rep_deg", "rep_seed", "sag_deg_per_nm", "table_mm", "friction", "absent", "cam_sigma_mm", "cam_seed", "rep_moved_only"}
        if set(self.errors) - known:
            raise ValueError(f"soma world errors: unknown keys {sorted(set(self.errors) - known)} (known: {sorted(known)})")
        self.rng = np.random.default_rng(int(self.errors.get("rep_seed", 0)))
        self.cam_rng = np.random.default_rng(1000 + int(self.errors.get("cam_seed", 0)))   # the camera stand-in's noise (its own stream)
        self.jit = [0.0] * 5
        self.sagoff = [0.0] * 5
        self.WorldCannotDraw = MW._hit(stop_cls, "WorldCannotDraw")
        self.WorldStop = MW._hit(stop_cls, "WorldStop")
        self.bias = list(bias)
        self.faults, self.occluders = [], []
        self.nshot = {"top": 0, "wrist": 0}
        self.piece_writes = []
        self.rec = {"reads": 0, "sends": 0, "shots": []}
        self.path = path
        if not os.path.exists(PROTO):
            raise FileNotFoundError(f"{PROTO}: make it with node sim/tools/export-physics.js --set 2 --gap 15")
        self.proto_path = PROTO
        sc = json.loads(json.dumps(json.load(open(PROTO, encoding="utf-8"))))
        sc["robotY"] = path["robot_y_m"] - self.errors.get("table_mm", 0.0) / 1000.0
        C, A = path["cell_m"], path["across_m"]
        sc["cell"], sc["pitch"] = C, C
        sc["pieces"] = [{"idx": n, "name": p["name"], "cells": [], "color": "#888888"} for n, p in enumerate(path["pieces"])]
        self.scene = sc
        self.start_layout = {"pieces": [{"name": p["name"], "cells_mm": [[v * 1000 for v in c] for c in p["table_cells"]]} for p in path["pieces"]]}
        self.cell_geoms = {}
        palette = [(0.89, 0.34, 0.18), (0.30, 0.55, 0.85), (0.35, 0.70, 0.35), (0.95, 0.75, 0.20), (0.60, 0.40, 0.75), (0.20, 0.70, 0.70), (0.85, 0.45, 0.60)]
        mass = mass_g_per_cell / 1000.0

        def add(spec):
            wb = spec.worldbody
            if chamfer_m > 0:                         # every cell's 12 edges cut at 45 deg (a convex mesh), as tools/soma_mj.py
                h, hz = A / 2, C / 2
                ex, ez = h - chamfer_m, hz - chamfer_m
                v = []
                for sx in (-1, 1):
                    for sy in (-1, 1):
                        for sz in (-1, 1):
                            v += [sx * h, sy * ex, sz * ez, sx * ex, sy * h, sz * ez, sx * ex, sy * ex, sz * hz]
                spec.add_mesh(name="soma_cell", uservert=v)
            st = path.get("stand")
            if st:
                wb.add_geom(name="soma_stand", type=mujoco.mjtGeom.mjGEOM_BOX, pos=RP.to_mj(st["c"]).tolist(),
                            size=[st["half"][0], st["half"][2], st["half"][1]], contype=3, conaffinity=3, rgba=[0.5, 0.5, 0.5, 1],
                            friction=[RP.FRICTION, RP.TORSION, 0.0001])
            for n, p in enumerate(path["pieces"]):
                cells = [np.array(c) for c in p["table_cells"]]
                if n in self.errors.get("absent", []):       # a negative check: this piece is not where the person should have set it
                    cells = [c + np.array([-0.45, 0.0, 0.0]) for c in cells]
                ymin = min(c[1] for c in cells)
                o = next(c for c in cells if abs(c[1] - ymin) < 1e-9)
                om = RP.to_mj(o)
                b = wb.add_body(name="pc%d" % n, pos=om.tolist())
                b.add_freejoint(name="pc%d_free" % n)
                col = list(palette[n % len(palette)]) + [1]
                for k, c in enumerate(cells):
                    shape = dict(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="soma_cell") if chamfer_m > 0 else dict(type=mujoco.mjtGeom.mjGEOM_BOX, size=[A / 2, A / 2, C / 2])
                    b.add_geom(name="pc%d_c%d" % (n, k), **shape,
                               pos=(RP.to_mj(c) - om).tolist(), rgba=col, mass=mass, friction=[RP.FRICTION, RP.TORSION, 0.0001],
                               condim=4, contype=2, conaffinity=2)

        fr0 = RP.FRICTION
        if self.errors.get("friction") is not None:
            RP.FRICTION = float(self.errors["friction"])
        try:
            self.model = RP.build(dict(sc, pieces=[]), folder, write_mjcf=False, extra=add)   # the pieces come from add (3D cells)
        finally:
            RP.FRICTION = fr0
        self.cell_geoms = {n: [self.model.geom("pc%d_c%d" % (n, k)).id for k in range(len(p["table_cells"]))] for n, p in enumerate(path["pieces"])}
        self.data = mujoco.MjData(self.model)
        self.jid = [self.model.joint(j).qposadr[0] for j in MW.ARM_JOINTS] + [self.model.joint("gripper").qposadr[0]]
        self.dof = [self.model.joint(j).dofadr[0] for j in MW.ARM_JOINTS]
        self.aid = [self.model.actuator(j).id for j in MW.ARM_JOINTS]
        self._contact_setup()
        u = self._to_urdf(start_L)
        self.ctrl_deg = [x + y for x, y in zip(u["urdf"], self.hidden())] + [u["jaw"]]
        for k, v in zip(self.jid, self.ctrl_deg):
            self.data.qpos[k] = math.radians(v)
        self.data.ctrl[:] = np.radians(self.ctrl_deg)
        mujoco.mj_forward(self.model, self.data)
        self.t = 0.0
        self.grip_plan = self.grip_goal = None
        self._run_for(MW.SETTLE_S)

    def _faults(self):
        pass

    def piece_states(self):
        out = []
        for n, p in enumerate(self.path["pieces"]):
            R = self.data.body("pc%d" % n).xmat.reshape(3, 3)
            cs = [screen_of(self.data.geom_xpos[g]) for g in self.cell_geoms[n]]
            out.append({"name": p["name"], "cells_mm": [[round(v * 1000, 3) for v in c] for c in cs],
                        "tilt": math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2])))),
                        "lift_mm": (min(c[1] for c in cs) - self.path["cell_m"] / 2) * 1000})
        return out

    @property
    def truth(self):
        return {"pieces": self.piece_states()}

    def read_camera(self, name: str) -> dict:
        raise self.Stop("soma: the 3D build has no camera reading yet (the control must not ask for a picture)")

    def spread(self, done):
        """the first `done` pieces: each cell's offset across from its place in the cube, minus their common offset (mm): the
        largest is how far a piece was pushed apart from the others (the whole cube sliding on the table is the common part)"""
        d = []
        for p, s in list(zip(self.path["pieces"], self.piece_states()))[:done]:
            d += [[a[0] - e[0] * 1000, a[2] - e[2] * 1000] for a, e in zip(s["cells_mm"], p["cube_cells"])]
        d = np.array(d)
        com = d.mean(axis=0)
        return {"common_mm": [round(float(v), 2) for v in com], "spread_mm": round(float(np.max(np.hypot(*(d - com).T))), 2)}

    def send(self, target: dict):
        """rep_moved_only (2026-11-26): the wobble (rep_deg) is drawn again only for the joints whose goal changed in this send;
        a joint that is not told to move keeps its offset (MjRobot draws all 5 again at every send, also when only the gripper
        moves: then the still arm shakes while the jaws hold or let go of a piece). A model choice, not measured"""
        rd = self.errors.get("rep_deg")
        if not (rd and self.errors.get("rep_moved_only")):
            return super().send(target)
        last = getattr(self, "_last_goal", None)
        moved = [last is None or abs(target[j] - last[j]) > 1e-9 for j in MW.ARM_JOINTS]
        self.jit = [float(self.rng.normal(0.0, float(rd))) if m else v for m, v in zip(moved, self.jit)]
        self.errors["rep_deg"] = 0
        try:
            super().send(target)
        finally:
            self.errors["rep_deg"] = rd
        self._last_goal = dict(target)

    def measure_tops(self, k):
        """the top camera's stand-in for --soma-correct (2026-11-26; no picture is drawn or read): with the arm at the path's
        photo pose (tools/soma_camera.js: nothing of the cube is hidden there), the top cell of every column of the first k pieces'
        planned cube cells: its true centre across (x, z, screen mm) + normal noise cam_sigma_mm (0.5 by default: about half a
        pixel on the cube's top, GUESS). [{piece, cell, xz_mm}]. Not modelled: a top cell pushed under another, tilt, the picture."""
        photo = self.path.get("photo_q")
        if not photo:
            raise self.Stop("soma: the path has no photo pose (soma_collide.js --photo-q)")
        now = [math.degrees(self.data.qpos[j]) for j in self.jid[:5]]
        off = max(abs(a - b) for a, b in zip(now, photo))
        if off > 3.0:
            raise self.Stop(f"soma: the arm is {off:.1f} deg from the photo pose: the camera would see the arm")
        sig = float(self.errors.get("cam_sigma_mm", 0.5))
        top = {}
        for m, p in enumerate(self.path["pieces"][:k]):
            for i, c in enumerate(p["cube_cells"]):
                key = (round(c[0] * 1000), round(c[2] * 1000))
                if key not in top or c[1] > top[key][2]:
                    top[key] = (m, i, c[1])
        st = self.piece_states()
        out = []
        for m, i, _ in top.values():
            a = st[m]["cells_mm"][i]
            out.append({"piece": m, "cell": i, "xz_mm": [float(a[0] + self.cam_rng.normal(0, sig)), float(a[2] + self.cam_rng.normal(0, sig))]})
        return out

    def offsets(self, k):
        """the first k pieces: each one's mean offset across (x, z) from its place in the cube (mm, the truth; for the log)"""
        out = []
        for p, s in list(zip(self.path["pieces"], self.piece_states()))[:k]:
            d = np.array([[a[0] - e[0] * 1000, a[2] - e[2] * 1000] for a, e in zip(s["cells_mm"], p["cube_cells"])])
            out.append([round(float(v), 2) for v in d.mean(axis=0)])
        return out

    def score(self, done=None):
        """each piece (all, or the first `done`): every cell's centre within PLACE_MM across and SEAT_MM up/down of its place in the
        cube, the piece tilted less than TILT_DEG. Pieces not built yet: within PLACE_MM of where they lay on the table (not pushed)"""
        rows = []
        for n, (p, s) in enumerate(zip(self.path["pieces"], self.piece_states())):
            built = done is None or n < done
            ref = p["cube_cells"] if built else p["table_cells"]
            act = np.array(s["cells_mm"]) / 1000.0
            exp = np.array(ref)
            across = float(np.max(np.hypot(act[:, 0] - exp[:, 0], act[:, 2] - exp[:, 2]))) * 1000
            up = float(np.max(np.abs(act[:, 1] - exp[:, 1]))) * 1000
            ok = across < PLACE_MM and up < SEAT_MM and s["tilt"] < TILT_DEG
            rows.append({"name": p["name"], "built": built, "across_mm": round(across, 2), "up_mm": round(up, 2), "tilt_deg": round(s["tilt"], 2), "ok": bool(ok)})
        return rows
