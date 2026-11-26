"""MuJoCo check of the Soma build plan (tools/soma_plan.py, 2026-11-23; redone after 点検 C65): can pieces held by ONE top-layer
cell be carried and put in, the last ones let go about 20 mm above the place (a drop), when the printed pieces have clearance?

The world: a flat table; the 7 pieces as free rigid bodies, one box per cell. Clearance c: each box is 40 - c mm across but a full
40 mm tall (the vertical clearance does not matter for putting in from above, and keeps the ideal grid exact in height, so a
perched piece shows). Chamfers (--chamfer-mm): every cell's 12 edges cut at 45 deg (a convex mesh); the real print would chamfer
only the piece's outer edges: here the faces between two cells of one piece get a groove too, and the pad touches the face only
from 17 mm under the top up to the top chamfer.
The hand is NOT the SO-101: it rides a stiff gantry (x, y, z and a turn about the vertical) that goes where told. The jaw
(--jaw real, the default): one FIXED pad 1 mm off the gripped cell's face and one moving pad opened 5.1 mm off the other face
(46.1 mm between the pads: the sim jaw model's opening at a 20 mm band, outputs/soma/grip_depth_20261123.txt), pushed by a constant
force --force-n (a motor; opened by the same force against its stop). --jaw two: both pads move, 3.05 mm each (to compare). The
pads hold the cell's side faces along the plan's axis, from the top to 17 mm under it.
Errors: --offset-mm moves the hand sideways at the place, --pick-offset-mm at the pick (each a random direction per piece from
--seed). Every piece starts on the table in its cube orientation, set apart.

Per step: down to the piece, close, lift to the carry height (the piece's bottom 10 mm over the cube top), across, down to the
place (for a drop: until the piece's top is 20 mm above where it belongs; otherwise --press-mm, default 2, below the seated
height: the pads slide on the piece), open, up. Recorded: the held piece's largest tilt while
lifted and carried (carry_tilt) and while lowered into the cube (lower_tilt). After each step, all placed pieces together
(Run.fit): ok = no cell centre more than UP_MM above where it belongs (seated, not perched), every piece upright within TOL_DEG,
no cell knocked more than KNOCK_MM sideways from the others' common shift, and that common shift (the cube as a whole: nothing on
the table centres it) not more than the place offset + c + SHIFT_SLACK_MM.

Model only. 【仮】 numbers: friction, torsion, contact stiffness and noslip as sim/physics/run_physics.py (PLA); the pad force
(default 7 N: run_physics.GRIP_NM 0.7 N m over a jaw of about 0.1 m); the pieces' mass (--mass-g-per-cell; a 1 mm shell is about
12 g a cell, a usual print with walls and some infill about 25-40 g, solid PLA about 79 g).

usage: sim/physics/.venv/Scripts/python.exe tools/soma_mj.py outputs/soma/plan.json [--clear-mm 1.0] [--chamfer-mm 0]
         [--offset-mm 0] [--pick-offset-mm 0] [--mass-g-per-cell 12] [--force-n 7] [--jaw real|two] [--seed 1] [--json out.json]
"""
import argparse
import json
import math
import os
import sys

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim", "physics"))
import run_physics as RP  # noqa: E402  (reads its numbers only)

C = 0.040
PAD_H = 0.017          # the pad reaches from the cell top to 17 mm under it
PAD_T = 0.004          # pad half thickness along the grip axis
PAD_W = 0.008          # pad half width across
DROP = 0.020           # a drop: let go with the piece's top this far above where it belongs
SPEED = 0.15           # hand speed [m/s]
TOL_DEG = 5.0          # a piece tilted more than this: fallen over
UP_MM = 1.0            # a cell centre higher than this above where it belongs: perched, not seated
KNOCK_MM = 10.0        # a cell centre this far sideways (after the common shift): knocked out of place
SHIFT_SLACK_MM = 4.0   # the common shift may be the place offset + c + this (1 mm of it: the fixed pad's gap)
FIX_GAP = 0.001        # real jaw: the fixed pad's gap to the face when the hand comes down
OPEN_MOV = 0.0051      # real jaw: the moving pad's gap to the other face when open (46.1 - 40 - 1 mm)


def build(plan, clear, mass_cell, force, chamfer=0.0, jaw="real"):
    spec = mujoco.MjSpec()
    spec.option.timestep = 0.001
    spec.option.noslip_iterations = RP.NOSLIP
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    w = spec.worldbody
    fr = [RP.FRICTION, RP.TORSION, 0.0001]
    g = w.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.01], friction=fr, condim=4)
    g.solref = [RP.SOLREF, 1.0]
    half = (C - clear) / 2          # across
    hz = C / 2                      # up: full height
    if chamfer > 0:
        ex, ez = half - chamfer, hz - chamfer
        v = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    v += [sx * half, sy * ex, sz * ez, sx * ex, sy * half, sz * ez, sx * ex, sy * ex, sz * hz]
        spec.add_mesh(name="cell", uservert=v)
    pieces = []
    for n, st in enumerate(plan["steps"]):
        cells = np.array(st["cells"], float)
        cen = np.stack([(cells[:, 0] - 1) * C, (cells[:, 1] - 1) * C, (cells[:, 2] + 0.5) * C], 1)   # where the cells belong
        mid = cen.mean(0)
        minz = cells[:, 2].min()
        park = np.array([-0.48 + 0.16 * n, 0.30, mid[2] - minz * C])     # on the table, the same orientation, set apart
        b = w.add_body(name=f"p{n}", pos=park.tolist())
        b.add_freejoint()
        for c in cen - mid:
            shape = dict(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="cell") if chamfer > 0 else dict(type=mujoco.mjtGeom.mjGEOM_BOX, size=[half, half, hz])
            gg = b.add_geom(**shape, pos=c.tolist(), mass=mass_cell, friction=fr, condim=4,
                            rgba=[0.3 + 0.1 * (n % 5), 0.5, 0.9 - 0.1 * (n % 5), 1])
            gg.solref = [RP.SOLREF, 1.0]
        pieces.append({"name": st["piece"], "local": cen - mid, "goal": mid, "park": park})
    # the hand rides on a stiff gantry (slides x, y, z and a turn about the vertical, each held by a stiff position servo, weight
    # compensated). A mocap hand would not do: a contact with a mocap body has no friction drag (the solver does not see its motion)
    hand = w.add_body(name="hand", pos=[0, 0, 0], gravcomp=1)
    for nm, ty, ax in (("hx", mujoco.mjtJoint.mjJNT_SLIDE, [1, 0, 0]), ("hy", mujoco.mjtJoint.mjJNT_SLIDE, [0, 1, 0]),
                       ("hz", mujoco.mjtJoint.mjJNT_SLIDE, [0, 0, 1]), ("hyaw", mujoco.mjtJoint.mjJNT_HINGE, [0, 0, 1])):
        hand.add_joint(name=nm, type=ty, axis=ax, range=[-200, 200] if nm == "hyaw" else [-1, 1])
        act = spec.add_actuator(name=nm, target=nm, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=20000 if nm != "hyaw" else 200, kv=400 if nm != "hyaw" else 4)
    hand.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.05], mass=1.0, contype=0, conaffinity=0)
    # pads: s = -1 and +1 sides along the hand's x. Slide position 0 = open; + = inward. Closed by a motor of constant force
    fixed_side = jaw == "real"
    for s in (-1, 1):
        if fixed_side and s == -1:
            gg = hand.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[PAD_T, PAD_W, PAD_H / 2], pos=[-(C / 2 + FIX_GAP + PAD_T), 0, 0],
                               mass=0.02, friction=fr, condim=4, rgba=[0.6, 0.2, 0.2, 1])
            gg.solref = [RP.SOLREF, 1.0]
            continue
        gap = OPEN_MOV if fixed_side else (OPEN_MOV + FIX_GAP) / 2
        f = hand.add_body(name=f"f{s}", pos=[0, 0, 0], gravcomp=1)
        f.add_joint(name=f"j{s}", type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[-s, 0, 0], range=[0, gap + 0.006], damping=5)
        gg = f.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[PAD_T, PAD_W, PAD_H / 2], pos=[s * (C / 2 + gap + PAD_T), 0, 0],
                        mass=0.02, friction=fr, condim=4, rgba=[0.9, 0.3, 0.2, 1])
        gg.solref = [RP.SOLREF, 1.0]
        a = spec.add_actuator(name=f"a{s}", target=f"j{s}", trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.ctrllimited = True
        a.ctrlrange = [-force, force]
    return spec.compile(), pieces


class Run:
    def __init__(self, model, pieces):
        self.m, self.p = model, pieces
        self.d = mujoco.MjData(model)
        mujoco.mj_forward(self.m, self.d)
        self.force = float(model.actuator_ctrlrange[4][1])
        self.set_jaw(False)
        self.held = None
        self.max_tilt = 0.0

    def set_jaw(self, closed):
        self.d.ctrl[4:] = self.force if closed else -self.force

    def body_pose(self, n):
        bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, f"p{n}")
        return self.d.xpos[bid].copy(), self.d.xmat[bid].reshape(3, 3).copy()

    def tilt(self, n):
        _, R = self.body_pose(n)
        return math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))

    def fit(self, upto):
        """pieces 0..upto together. Returns (xy, up, tilt, shift):
          xy   = the largest sideways distance of a cell centre from where it belongs, after taking away the whole cube's common
                 sideways shift (no fence on the table: nothing centres the cube). With clearance c the pieces may legitimately
                 sit up to about 2c apart from the ideal grid (loose cube), so this is reported, and only a piece knocked out
                 (over KNOCK_MM) fails
          up   = the largest height of a cell centre ABOVE where it belongs (a piece perched on another, not seated; the cells
                 are a full 40 mm tall, so a seated piece is at the ideal height)
          tilt = the largest tilt of the pieces (deg)
          shift = the common sideways shift (mm)"""
        act, goal = [], []
        for n in range(upto + 1):
            p, R = self.body_pose(n)
            act.append(p + self.p[n]["local"] @ R.T)
            goal.append(self.p[n]["goal"] + self.p[n]["local"])
        act, goal = np.vstack(act), np.vstack(goal)
        d = act - goal
        sh = d[:, :2].mean(0)
        xy = np.linalg.norm(d[:, :2] - sh, axis=1)
        return (float(xy.max() * 1000), float(max(0.0, d[:, 2].max()) * 1000), max(self.tilt(n) for n in range(upto + 1)),
                float(np.linalg.norm(sh) * 1000))

    def err_mm(self, n):
        p, R = self.body_pose(n)
        cells = p + self.p[n]["local"] @ R.T
        return float(np.max(np.linalg.norm(cells - (self.p[n]["goal"] + self.p[n]["local"]), axis=1)) * 1000)

    def step_for(self, t):
        for _ in range(int(round(t / self.m.opt.timestep))):
            mujoco.mj_step(self.m, self.d)
            if self.held is not None:
                self.max_tilt = max(self.max_tilt, self.tilt(self.held))

    def move(self, to, yaw):
        fr = self.d.ctrl[0:3].copy()
        to = np.array(to, float)
        self.d.ctrl[3] = math.radians(yaw)
        dist = float(np.linalg.norm(to - fr))
        n = max(1, int(dist / SPEED / self.m.opt.timestep))
        for i in range(1, n + 1):
            self.d.ctrl[0:3] = fr + (to - fr) * i / n
            mujoco.mj_step(self.m, self.d)
            if self.held is not None and i % 10 == 0:
                self.max_tilt = max(self.max_tilt, self.tilt(self.held))
        self.step_for(0.15)              # the gantry lags about 3 mm at speed; let it arrive

    def jaw(self, closed, t=0.4):
        self.set_jaw(closed)
        self.step_for(t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("plan")
    ap.add_argument("--clear-mm", type=float, default=1.0)
    ap.add_argument("--chamfer-mm", type=float, default=0.0)
    ap.add_argument("--offset-mm", type=float, default=0.0)
    ap.add_argument("--mass-g-per-cell", type=float, default=12.0)
    ap.add_argument("--force-n", type=float, default=7.0)
    ap.add_argument("--pick-offset-mm", type=float, default=0.0)
    ap.add_argument("--jaw", choices=["real", "two"], default="real")
    ap.add_argument("--press-mm", type=float, default=2.0)   # not a drop: the hand goes this far below the seated height
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--json")
    a = ap.parse_args()
    plan = json.load(open(a.plan, encoding="utf-8"))
    model, pieces = build(plan, a.clear_mm / 1000, a.mass_g_per_cell / 1000, a.force_n, a.chamfer_mm / 1000, a.jaw)
    R = Run(model, pieces)
    rng = np.random.default_rng(a.seed)
    R.move([0, 0, 0.5], 0)
    R.step_for(0.5)
    out = {"about": "tools/soma_mj.py", "args": vars(a), "steps": []}
    for n, st in enumerate(plan["steps"]):
        i, j, k = st["grip"]["cell"]
        yaw = 0 if st["grip"]["axis"] == "x" else 90
        cells = np.array(st["cells"])
        minz = cells[:, 2].min()
        lift_k = k - minz                                  # the gripped cell's layer counted from the piece's bottom
        loc = pieces[n]["local"]
        gi = [m for m, c in enumerate(st["cells"]) if c == [i, j, k]][0]
        p0, Rm = R.body_pose(n)
        grip_now = p0 + Rm @ loc[gi]                       # the gripped cell's centre now (on the table)
        top_now = grip_now[2] + C / 2
        hz = lambda top: top - PAD_H / 2                   # hand height for a cell top
        carry_top = 3 * C + 0.010 + (lift_k + 1) * C
        R.jaw(False, 0.2)
        pa = rng.uniform(0, 2 * math.pi)
        po = np.array([math.cos(pa), math.sin(pa), 0]) * a.pick_offset_mm / 1000
        R.move([grip_now[0] + po[0], grip_now[1] + po[1], carry_top], yaw)
        R.move([grip_now[0] + po[0], grip_now[1] + po[1], hz(top_now)], yaw)
        R.jaw(True, 0.6)
        R.held, R.max_tilt = n, 0.0
        R.move([grip_now[0] + po[0], grip_now[1] + po[1], hz(carry_top)], yaw)
        goal = pieces[n]["goal"] + loc[gi]                 # the gripped cell's centre where it belongs
        ang = rng.uniform(0, 2 * math.pi)
        off = np.array([math.cos(ang), math.sin(ang), 0]) * a.offset_mm / 1000
        R.move([goal[0] + off[0], goal[1] + off[1], hz(carry_top)], yaw)
        carry_tilt, R.max_tilt = R.max_tilt, 0.0
        place_top = goal[2] + C / 2 + (DROP if st["drop"] else -a.press_mm / 1000)   # not a drop: --press-mm lower than seated (it slides in the pads)
        R.move([goal[0] + off[0], goal[1] + off[1], hz(place_top)], yaw)
        R.step_for(0.2)
        lower_tilt = R.max_tilt
        R.jaw(False, 0.4)
        R.held = None
        R.step_for(0.5)
        R.move([goal[0] + off[0], goal[1] + off[1], hz(carry_top)], yaw)
        R.step_for(0.3)
        row = {"piece": st["piece"], "drop": st["drop"], "carry_tilt_deg": round(carry_tilt, 1), "lower_tilt_deg": round(lower_tilt, 1),
               "err_mm": round(R.err_mm(n), 2), "tilt_deg": round(R.tilt(n), 1),
               "others_worst_err_mm": round(max([R.err_mm(m) for m in range(n)] or [0.0]), 2)}
        xy, up, tl, sh = R.fit(n)
        row.update(xy_mm=round(xy, 2), up_mm=round(up, 2), all_tilt_deg=round(tl, 1), shift_mm=round(sh, 2))
        row["ok"] = (up <= UP_MM and tl <= TOL_DEG and xy <= KNOCK_MM
                     and sh <= a.offset_mm + a.clear_mm + SHIFT_SLACK_MM)
        out["steps"].append(row)
        print(f"{n + 1} {st['piece']}: {'drop ' if st['drop'] else ''}carry tilt {row['carry_tilt_deg']} lower {row['lower_tilt_deg']} deg, "
              f"placed err {row['err_mm']} mm tilt {row['tilt_deg']} deg, others worst {row['others_worst_err_mm']} mm; "
              f"all placed: sideways {row['xy_mm']} mm, above {row['up_mm']} mm, tilt {row['all_tilt_deg']} deg, shift {row['shift_mm']} mm "
              f"-> {'ok' if row['ok'] else 'NG'}", file=sys.stderr)
    out["all_ok"] = all(s["ok"] for s in out["steps"])
    print(f"jaw {a.jaw}, clear {a.clear_mm} mm, chamfer {a.chamfer_mm} mm, offset {a.offset_mm} mm, pick offset {a.pick_offset_mm} mm, "
          f"{a.mass_g_per_cell} g/cell, {a.force_n} N, seed {a.seed}: "
          f"{'ALL PLACED' if out['all_ok'] else 'NOT all'}", file=sys.stderr)
    if a.json:
        json.dump(out, open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
