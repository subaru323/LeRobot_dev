"""S4: what the real-arm day is handed (総合計画書 改訂版の S4). The REAL-ARM path of tools/puzzle_run.py (RealRobot, Camera,
load_calib, air_run_ok, the real so101_common functions) runs on a fake motor bus and fake USB cameras whose world is MuJoCo
(tools/mj_world.py). Faults are put in at the bus, the cameras and the files: calibration mix-ups, old pictures, a lost link, a
missed grasp, a protective stop, a restart. Expectations are written BEFORE running and frozen (as S0/S2/S3).

  sim\\physics\\.venv\\Scripts\\python.exe tools\\s4_run.py freeze
  sim\\physics\\.venv\\Scripts\\python.exe tools\\s4_run.py run [--only S4-01,S4-10] [--jobs 2] [--frozen FILE] [--tag T]
  sim\\physics\\.venv\\Scripts\\python.exe tools\\s4_run.py one CASE_ID OUTDIR      (one case in this process; run calls it)
  sim\\physics\\.venv\\Scripts\\python.exe tools\\s4_run.py probe OUTDIR [--pieces 1]   (the clean run, with the bus writes and when
                                                                                       a piece is held: to place the faults)

Nothing here opens a port or a camera: lerobot and cv2 are stand-ins (this python has neither), puzzle_run.FAKE_HW is set (its logs
say "fake_hardware" and never count for the real arm's in-air gate) and puzzle_run.OUT is the case's own folder.

A case is a list of steps run in ONE world (the arm and the pieces stay where a step left them). A step is a puzzle_run command
(the real-arm arguments are added here), or a "user" act between commands (move the arm back by hand, open the jaws).
Judged for every command step (class in s4_cases.json):
  expect.result  "ok" | "stopped" | "refused" (SystemExit before the run starts: no log) with one of reason_any in the message;
                 expect.torque false = torque never turned on in this step; expect.no_move_after_look = no command after the
                 first top picture (a restart must not carry on)
  always         (A) torque goes on only after the "Torque will turn ON" Enter; (B) after a stop nothing but the hold is sent;
                 (C) torque goes off, while the arm is held, only after a "SUPPORT THE ARM" Enter; (D) a step that ends "ok" on the
                 table has the physics right (else FALSE COMPLETION), in the air no piece moved; (E) every step that started
                 wrote its log, and a stopped one says where (stop_at); (F) --replay of its record (no port) sends the same commands
                 and stops at the same observation (a stop from an observation: the same message; a stop inside the driver: the
                 replay runs out of observations right there)
  可否未確定    (D)-(F) only, the result recorded
"""
import argparse
import builtins
import datetime
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import time
import types
import zlib
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
CASES = os.path.join(HERE, "s4_cases.json")
OUTD = os.path.join(ROOT, "outputs", "s4")
SIM = os.path.join(ROOT, "sim")
MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
LOAD_FULL_NM = 3.35          # mj_world: the model's force limit = Present_Load 1000 (LOAD_ABORT 600 = mj_world LOAD_ABORT_NM)
CAM_INDEX = {0: "top", 1: "wrist"}


def sha_text(obj) -> str:
    return hashlib.sha256(json.dumps(obj, ensure_ascii=False, sort_keys=True).encode("utf-8")).hexdigest()


def file_sha(p) -> str:
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


# ---------------------------------------------------------------- stand-ins (before puzzle_run / so101_common are imported)
class H:                      # the harness state one case shares with the stand-ins
    world = None
    bus = None
    cams = {}
    events = []               # (kind, detail) in order: bus, camera, Enter prompts
    cam_faults = {}
    crashed = False


def ev(kind, **kw):
    H.events.append(dict(k=kind, **kw))


def stub_lerobot():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    class SOFollower:
        def __init__(self, cfg):
            self.bus = H.bus

    class Cfg:
        def __init__(self, **kw):
            self.__dict__.update(kw)
    mod("lerobot")
    mod("lerobot.motors")
    mod("lerobot.motors.feetech", FeetechMotorsBus=object)
    mod("lerobot.robots")
    mod("lerobot.robots.so_follower", SOFollower=SOFollower, SOFollowerRobotConfig=Cfg)
    mod("lerobot.teleoperators")
    mod("lerobot.teleoperators.so_leader", SOLeader=SOFollower, SOLeaderTeleopConfig=Cfg)


class FakeVideoCapture:
    """cv2.VideoCapture over the MuJoCo world: one picture per open (puzzle_run.Camera opens the camera for each shot)."""
    opens = {}
    last = {}

    def __init__(self, index, api=None):
        self.name = CAM_INDEX.get(index)
        self.frame = None

    def set(self, *a):
        return True

    def read(self):
        if H.crashed:
            return False, None
        if self.frame is None:
            nm = self.name
            if nm is None or H.cam_faults.get("dead") == nm:
                ev("cam_fail", cam=nm)
                return False, None
            n = FakeVideoCapture.opens[nm] = FakeVideoCapture.opens.get(nm, 0) + 1
            st = H.cam_faults.get("stale")
            if st and st[0] == nm and st[1] == n and nm in FakeVideoCapture.last:
                self.frame = FakeVideoCapture.last[nm]         # the old picture, byte for byte
                ev("cam_stale", cam=nm, n=n)
            else:
                im = H.world.read_camera(nm)
                rgb = np.fromfile(im["out"], dtype=np.uint8).reshape(im["H"], im["W"], 3)
                self.frame = np.ascontiguousarray(rgb[..., ::-1])
                ev("cam", cam=nm, n=n)
            FakeVideoCapture.last[nm] = self.frame
        return True, self.frame.copy()

    def release(self):
        pass


def png_bytes(rgb):
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(t, d):
        return len(d).to_bytes(4, "big") + t + d + zlib.crc32(t + d).to_bytes(4, "big")
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", w.to_bytes(4, "big") + h.to_bytes(4, "big") + b"\x08\x02\x00\x00\x00") \
        + chunk(b"IDAT", zlib.compress(raw, 3)) + chunk(b"IEND", b"")


def stub_cv2():
    m = types.ModuleType("cv2")
    m.CAP_DSHOW, m.CAP_PROP_FOURCC, m.CAP_PROP_FRAME_WIDTH, m.CAP_PROP_FRAME_HEIGHT, m.CAP_PROP_AUTOFOCUS = 700, 6, 3, 4, 39
    m.CV_32FC1, m.INTER_LINEAR, m.COLOR_BGR2RGB = 5, 1, 4
    m.VideoWriter_fourcc = lambda *c: 0
    m.VideoCapture = FakeVideoCapture

    def imwrite(path, img):
        open(path, "wb").write(png_bytes(np.ascontiguousarray(img[..., ::-1])))
        return True

    def init_maps(K, dist, R, newK, size, t):
        W, Hh = size
        u, v = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(Hh, dtype=np.float64))
        K, newK, d = np.asarray(K, float), np.asarray(newK, float), np.asarray(dist, float).ravel()
        if not d.any() and np.allclose(K, newK):
            return u.astype(np.float32), v.astype(np.float32)
        x, y = (u - newK[0, 2]) / newK[0, 0], (v - newK[1, 2]) / newK[1, 1]
        k1, k2, p1, p2, k3 = (list(d) + [0.0] * 5)[:5]
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        xd, yd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x), y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        return (xd * K[0, 0] + K[0, 2]).astype(np.float32), (yd * K[1, 1] + K[1, 2]).astype(np.float32)

    def remap(img, mx, my, interp):
        Hh, W = img.shape[:2]
        x0, y0 = np.floor(mx).astype(int), np.floor(my).astype(int)
        fx, fy = (mx - x0)[..., None], (my - y0)[..., None]
        xa, xb, ya, yb = np.clip(x0, 0, W - 1), np.clip(x0 + 1, 0, W - 1), np.clip(y0, 0, Hh - 1), np.clip(y0 + 1, 0, Hh - 1)
        f = img.astype(np.float64)
        out = (f[ya, xa] * (1 - fx) * (1 - fy) + f[ya, xb] * fx * (1 - fy) + f[yb, xa] * (1 - fx) * fy + f[yb, xb] * fx * fy)
        return np.clip(np.rint(out), 0, 255).astype(np.uint8)
    m.imwrite, m.initUndistortRectifyMap, m.remap = imwrite, init_maps, remap
    m.cvtColor = lambda img, code: np.ascontiguousarray(img[..., ::-1])
    sys.modules["cv2"] = m


class ProcessDied(BaseException):
    pass


class FakeBus:
    """The lerobot FeetechMotorsBus calls puzzle_run.RealRobot and tools/so101_common.py make, over the MuJoCo world.
    Positions are LeRobot's units both 'normalized' and raw (the fake does not need steps). sync_write of Goal_Position moves the
    world to the goal at once (mj_world.MjRobot.send: the servo speed, collisions, gravity); the world's own driver stops are turned
    into readings (a high load -> Present_Load; not reached -> the position stays short) and puzzle_run decides."""

    def __init__(self, world, calib, motor_calib_differs=False):
        self.world, self.calibration, self.motors = world, calib, {m: None for m in MOTORS}
        self.calib_differs = motor_calib_differs
        self.connected = False
        self.torque = {m: False for m in MOTORS}
        self.status = {m: 0 for m in MOTORS}
        self.goal = None
        self.faults = {}
        self.n_write = 0           # Goal_Position sync_writes in this step
        self.n_read = 0
        self.dead = False
        self.limp = set()
        self.peak = {m: 0.0 for m in MOTORS}
        self.forced_load = None

    def new_step(self, faults):
        self.faults, self.n_write, self.n_read, self.forced_load = dict(faults or {}), 0, 0, None
        if self.faults.get("link_back"):          # the cable put back / the program started again: the link works again
            self.dead = False

    # ---- the link
    def _link(self, num_retry=0):
        if H.crashed:
            raise ProcessDied("process gone")
        self.n_read += 1
        if self.dead:
            raise ConnectionError("[TxRxResult] There is no status packet!")
        if self.n_read in set(self.faults.get("read_blip", [])) and num_retry == 0:
            raise ConnectionError("[TxRxResult] There is no status packet! (one lost)")

    def connect(self, handshake=True):
        if self.connected:
            raise RuntimeError("DeviceAlreadyConnectedError")
        if self.dead:
            raise ConnectionError("Could not connect on port")
        self.connected = True
        ev("connect")

    def disconnect(self, disable_torque=True):
        if H.crashed:
            return
        if disable_torque:
            self.disable_torque()
        self.connected = False
        ev("disconnect")

    def configure_motors(self, *a, **k):
        self._link(3)

    @property
    def is_calibrated(self):
        return not self.calib_differs

    def _set_torque(self, on, motors=None):
        held_before = any(self.torque.values())
        for m in (motors or MOTORS):
            if m in self.limp and on:
                continue
            self.torque[m] = on
        ev("torque_on" if on else "torque_off", held_before=held_before)

    def disable_torque(self, motors=None, num_retry=0):
        self._link(num_retry)
        self._set_torque(False, [motors] if isinstance(motors, str) else motors)

    def enable_torque(self, motors=None, num_retry=0):
        self._link(num_retry)
        self._set_torque(True, [motors] if isinstance(motors, str) else motors)

    def _load_raw(self, m):
        if self.forced_load and self.forced_load[0] == m:
            return self.forced_load[1]
        if m in self.peak and self.peak[m]:
            v, self.peak[m] = self.peak[m], 0.0                 # the first reading after a move sees its peak
            return int(v)
        return 0

    def read(self, reg, motor, *, normalize=True, num_retry=0):
        self._link(num_retry)
        if reg == "Torque_Enable":
            if self.faults.get("torque_read_fail"):           # 点検 C63 の 1: the link answers writes but not this read
                raise ConnectionError("[TxRxResult] There is no status packet! (Torque_Enable)")
            return int(self.torque[motor])
        if reg == "Status":
            return self.status[motor]
        if reg == "Present_Load":
            return self._load_raw(motor)
        return 0

    def write(self, reg, motor, value, *, normalize=True, num_retry=0):
        if H.crashed:
            return
        self._link(num_retry)
        if reg == "Torque_Enable":
            self._set_torque(bool(value), [motor])
        elif reg == "Goal_Position":
            ev("goal1", motor=motor)
            self.goal = dict(self.goal or self.world.read_joints())
            self.goal[motor] = float(value)
            if all(self.torque.values()):
                self._move()

    def sync_read(self, reg, motors=None, *, normalize=True, num_retry=0):
        self._link(num_retry)
        return dict(self.world.read_joints())

    def sync_write(self, reg, values, *, normalize=True, num_retry=0):
        if H.crashed or reg != "Goal_Position":
            return
        self.n_write += 1
        n, f = self.n_write, self.faults
        ev("goal", n=n, held=self.world_held())
        if f.get("interrupt_at") == n:
            ev("fault", what="Ctrl+C")
            raise KeyboardInterrupt()
        if f.get("crash_at") == n:
            ev("fault", what="process died")
            H.crashed = True
            raise ProcessDied("the program died here")
        if f.get("comm_loss_at") == n:
            ev("fault", what="link lost")
            self.dead = True
        if f.get("protect_at") == n:
            m = f.get("protect_motor", "elbow_flex")
            ev("fault", what=f"protective stop {m}")
            self.status[m], self.torque[m] = 32, False
            self.limp.add(m)
            self.world_limp(m)
        if f.get("load_at") == n:
            self.forced_load = (f.get("load_motor", "shoulder_lift"), int(f.get("load_raw", 700)))
            ev("fault", what=f"load {self.forced_load}")
        if self.dead or f.get("write_drop_at") == n:
            if f.get("write_drop_at") == n:
                ev("fault", what="one command lost")
            return
        self.goal = dict(self.goal or self.world.read_joints())
        self.goal.update({k: float(v) for k, v in values.items()})
        if any(self.torque.values()):
            self._move()

    def _move(self):
        w = self.world
        saved, w.max_load = list(w.max_load), [0.0] * 5
        try:
            w.send(dict(self.goal))
        except H.WorldStop as e:
            msg = str(e)
            if "high load" not in msg and "did not reach" not in msg:
                raise
        finally:
            for j, m in enumerate(MOTORS[:5]):
                self.peak[m] = max(self.peak[m], w.max_load[j] / LOAD_FULL_NM * 1000)
            w.max_load = [max(x, y) for x, y in zip(saved, w.max_load)]

    def world_held(self):
        try:
            return [s["name"] for s in self.world.piece_states() if s["lift_mm"] > 3.0]
        except Exception:  # noqa: BLE001
            return []

    def world_limp(self, m):
        w = self.world
        a = w.model.actuator(m).id
        w.model.actuator_gainprm[a, 0] = 0.0
        w.model.actuator_biasprm[a, 1] = 0.0


# ---------------------------------------------------------------- files the real arm reads (made here, consistent with the world)
def fake_k1(shift=None):
    """the same k1 as sim/plan-cli.js fakeK1 (the world converts with that one). shift {joint: deg} = an older / other K1"""
    cal = {m: {"id": i + 1, "drive_mode": 0, "homing_offset": 0, "range_min": 2048 - 1700, "range_max": 2048 + 1700} for i, m in enumerate(MOTORS)}
    sign = {m: (-1 if i % 2 else 1) for i, m in enumerate(MOTORS[:5])}
    offset = {m: (i - 2) * 7.5 + (shift or {}).get(m, 0.0) for i, m in enumerate(MOTORS[:5])}
    return {"robot_id": "s4fake", "date": "-", "verified": True, "calibration": cal, "sign": sign, "offset": offset,
            "gripper": [{"lerobot": 2, "gap_mm": 0}, {"lerobot": 60, "gap_mm": 30}]}


def make_files(d, PR, spec):
    """k1, K1 run record, k1-err, top intrinsics / pose, wrist json: the shapes puzzle_run.load_calib checks. spec: mix-ups"""
    os.makedirs(d, exist_ok=True)
    PR.make_sim_calib(d)
    k1p = os.path.join(d, "k1_s4.json")
    json.dump(fake_k1(spec.get("k1_shift")), open(k1p, "w"), indent=1)
    k1s = file_sha(k1p)
    run = os.path.join(d, "k1_run_s4.json")
    json.dump({"k1_sha256": "0" * 64 if spec.get("k1err_other_run") else k1s}, open(run, "w"))
    ke = os.path.join(d, "k1err_s4.json")
    json.dump({"paper": {"dx_mm": 0.0, "dz_mm": 0.0, "scale_permil": 0.0}, "joints": [{"deg": 0.0}], "run": run}, open(ke, "w"))
    ti = os.path.join(d, "intrinsics_top_sim_640x480.json")
    if spec.get("top_size"):
        j = json.load(open(ti))
        sw, sh = spec["top_size"]
        j["image_size"], j["K"] = [sw, sh], [[j["K"][0][0] * sh / 480, 0, sw / 2 - 0.5], [0, j["K"][1][1] * sh / 480, sh / 2 - 0.5], [0, 0, 1]]
        json.dump(j, open(ti, "w"), indent=1)
    tp = os.path.join(d, "pose_top_sim.json")
    p = json.load(open(tp))
    p["reproj_px_max"] = 0.3
    p["k1err"] = {"file": os.path.basename(ke), "dx_mm": 0.0, "dz_mm": 0.0, "scale_permil": 0.0, "shoulder_pan_deg": 0.0}
    if spec.get("pose_no_k1err"):
        del p["k1err"]
    if spec.get("pose_other_intrinsics"):
        p["intrinsics"] = "intrinsics_top_1280x720.json"
    if spec.get("pose_shift_mm"):
        p["centre_arm_mm_without_rotation"] = [v + s for v, s in zip(p["centre_arm_mm_without_rotation"], spec["pose_shift_mm"])]
    json.dump(p, open(tp, "w"), indent=1)
    wj = os.path.join(d, "wrist_cam_sim.json")
    w = json.load(open(wj))
    w["k1_sha256"] = "f" * 64 if spec.get("wrist_other_k1") else k1s
    json.dump(w, open(wj, "w"), indent=1)
    return {"k1": k1p, "k1err": ke, "top_intrinsics": ti, "top_pose": tp, "wrist_json": wj}


def real_args(files, swap_cams=False):
    return ["--k1", files["k1"], "--top-cam", "1" if swap_cams else "0", "--top-intrinsics", files["top_intrinsics"],
            "--top-pose", files["top_pose"], "--wrist-cam", "0" if swap_cams else "1", "--wrist-json", files["wrist_json"],
            "--k1err", files["k1err"]]


# ---------------------------------------------------------------- one case
def run_one(case, outdir, probe=False):
    stub_lerobot()
    stub_cv2()
    sys.path.insert(0, HERE)
    import puzzle_run as PR
    import mj_world
    PR.FAKE_HW = True
    PR.OUT = os.path.join(outdir, "puzzle_run")
    os.makedirs(PR.OUT, exist_ok=True)
    PR.time = SimpleNamespace(time=time.time, sleep=lambda s: time.sleep(min(s, 0.002)))   # the waits of RealRobot, shortened

    class WorldStop(Exception):
        pass
    H.WorldStop = WorldStop
    prompts = []

    def fake_input(msg=""):
        prompts.append(msg)
        ev("enter", msg=msg[:60])
        return ""
    builtins.input = fake_input
    orig_hold = PR.RealRobot.hold

    def hold(self):
        ev("hold")
        return orig_hold(self)
    PR.RealRobot.hold = hold

    files = make_files(os.path.join(outdir, "files"), PR, case.get("files", {}))
    wargs = case.get("world", {})
    seed, set_no, gap = int(wargs.get("seed", 1)), int(wargs.get("set", 2)), wargs.get("gap")
    pl = PR.Planner()
    init = pl.need("init", sheet=False, set=set_no, k1="fake", calib="sim", calibMm=0, calibDeg=0, noise=3.0, seed=seed,
                   **({"gap": gap} if gap is not None else {}))
    truth = pl.need("truth", seed=seed)["layout"]
    defaults = pl.need("truth", seed=seed, shiftMm=0, yawDeg=0)["layout"]
    npieces = int(wargs.get("pieces", 0))
    active = init["order"][:npieces] if npieces else [p["idx"] for p in init["pieces"]]
    user_L = {k: v + (3.0 if k in PR.JOINTS else 0.0) for k, v in init["start_lerobot"].items()}
    wf = os.path.join(outdir, "world")
    os.makedirs(wf, exist_ok=True)
    world_events = []
    world = mj_world.MjRobot(pl, set_no, truth, defaults, set(active), user_L, [0.0] * 5, wf, WorldStop,
                             lambda msg, **kw: world_events.append(dict(msg=msg, **kw)),
                             faults=wargs.get("mj_faults") or [], occluders=wargs.get("mj_occluders") or [],
                             errors=wargs.get("mj_errors") or {}, gap=gap)
    world.grip_goal_for = lambda planned: planned      # the gripper squeeze is RealRobot's (grip_goal_for); the world just moves
    H.world = world
    cal = {m: SimpleNamespace(**v) for m, v in fake_k1()["calibration"].items()}
    if case.get("bus", {}).get("motors_homing_shift"):
        m, dv = case["bus"]["motors_homing_shift"]
        cal[m].homing_offset += dv
    H.bus = FakeBus(world, cal, motor_calib_differs=bool(case.get("bus", {}).get("calib_file_differs")))
    steps_out = []
    for si, st in enumerate(case["steps"]):
        H.events.clear()
        prompts.clear()
        if "user" in st:
            steps_out.append(user_act(st["user"], world, init, PR, pl))
            continue
        H.bus.new_step(st.get("bus"))
        H.cam_faults = dict(st.get("cams") or {})
        H.crashed = False
        FakeVideoCapture.opens = {}
        H.bus.connected = False
        before = set(os.listdir(PR.OUT))
        argv = ["puzzle_run.py", "COM_S4"] + real_args(files, st.get("swap_cams")) + st["args"]
        sys.argv = argv
        t0, exit_msg = time.time(), None
        try:
            PR.main()
        except SystemExit as e:
            exit_msg = None if e.code in (0, 1, None) else str(e.code)
        except ProcessDied:
            exit_msg = None
        new = sorted(f for f in set(os.listdir(PR.OUT)) - before if f.endswith(".json"))
        log = json.load(open(os.path.join(PR.OUT, new[-1]), encoding="utf-8")) if new else None
        logp = os.path.join(PR.OUT, new[-1]) if new else None
        crashed = H.crashed
        if crashed and logp:                    # a dead program writes no log: keep it aside (air_run_ok must not read it)
            os.replace(logp, logp + ".crashed")
            logp += ".crashed"
        rec = {"step": si, "args": st["args"], "secs": round(time.time() - t0, 1), "exit": exit_msg, "crashed": crashed,
               "log": os.path.relpath(logp, ROOT) if logp else None, "result": (log or {}).get("result"),
               "stop_kind": (log or {}).get("stop_kind"), "stop": stop_msg(log) if log else "", "stop_at": (log or {}).get("stop_at"),
               "events": list(H.events), "prompts": list(prompts), "world_events": list(world_events),
               "physics": world.score(set(active)), "n_goal_writes": H.bus.n_write}
        world_events.clear()
        if log and not crashed and log.get("rec") and not probe:
            rec["replay"] = replay(PR, files, st, logp, log)
        steps_out.append(rec)
    pl.close()
    out = {"id": case["id"], "steps": steps_out, "contacts": world.contact_log()}
    try:
        out["contact_verdict"] = mj_world.classify_contacts(out["contacts"], {init["pieces"][q]["name"] for q in active})
    except Exception as e:  # noqa: BLE001
        out["contact_verdict"] = {"error": repr(e)}
    return out


def user_act(act, world, init, PR, pl):
    """between two commands: what the user does by hand (the procedure's 'after a stop')."""
    msgs = []
    if act in ("open_jaws", "to_start"):
        try:
            L = dict(world.read_joints())
            L["gripper"] = init["start_lerobot"]["gripper"]
            world.send(L)
            world._run_for(0.5)
            msgs.append("opened the jaws where the arm was")
        except Exception as e:  # noqa: BLE001
            msgs.append(f"opening the jaws: {e!r}")
    if act == "to_start":
        try:
            cur = dict(world.read_joints())
            up = dict(cur, shoulder_lift=cur["shoulder_lift"] - 15.0)       # lift a little first (away from the pieces)
            keys = PR.JOINTS + ["gripper"]                                   # MjRobot.send needs the gripper in every target (first run: 2026-11-23 it was left out)
            for s in PR.steps_between(cur, up, keys) + PR.steps_between(up, dict(init["start_lerobot"]), keys):
                world.send(s)
            world._run_for(0.5)
            msgs.append("moved the arm back to the photo pose by hand")
        except Exception as e:  # noqa: BLE001
            msgs.append(f"moving the arm back: {e!r}")
    return {"user": act, "msgs": msgs, "physics": world.score(set(range(len(init["pieces"])))) if False else None}


def replay(PR, files, st, logp, log):
    """(F) the record given back with --replay (no port): same commands, same stop place"""
    before = set(os.listdir(PR.OUT))
    sys.argv = ["puzzle_run.py"] + real_args(files, st.get("swap_cams")) + st["args"] + ["--replay", logp]
    try:
        PR.main()
    except SystemExit:
        pass
    except BaseException as e:  # noqa: BLE001
        return {"ok": False, "why": f"replay crashed: {e!r}"}
    new = sorted(f for f in set(os.listdir(PR.OUT)) - before if f.endswith(".json"))
    if not new:
        return {"ok": False, "why": "replay wrote no log"}
    rl = json.load(open(os.path.join(PR.OUT, new[-1]), encoding="utf-8"))
    os.replace(os.path.join(PR.OUT, new[-1]), os.path.join(PR.OUT, new[-1][:-5] + ".replay.json"))
    sent0, sent1 = log["rec"]["sent"], (rl.get("replay") or {}).get("sent", [])
    same = sent1[:len(sent0)] == sent0
    msg0, msg1 = stop_msg(log), stop_msg(rl)
    left = (rl.get("replay") or {}).get("left")
    driver = False
    if log.get("result") != "stopped":
        ok = same and len(sent1) == len(sent0) and rl.get("result") == log.get("result") and left == 0
        why = "" if ok else f"result {rl.get('result')} vs {log.get('result')}, sent {len(sent1)} vs {len(sent0)}, left {left}"
    elif msg1 == msg0:                     # the stop came from an observation: the replay stops the same way
        ok = same and len(sent1) == len(sent0)
        why = "" if ok else f"same message, sent {len(sent1)} vs {len(sent0)}"
    else:                                  # the stop came from inside the driver / camera (not an observation): the replay runs out there
        driver = True
        ok = same and "no observation left" in msg1 and left == 0
        why = "" if ok else f"replay said {msg1[:120]!r}, same commands {same}, left {left}"
    return {"ok": ok, "why": why, "n_sent": [len(sent0), len(sent1)], "driver_stop": driver}


def stop_msg(log) -> str:
    return next((e["msg"] for e in (log or {}).get("events", []) if e["msg"].startswith("!! stopped")), "")


# ---------------------------------------------------------------- judging
def checks(step, exp, is_air):
    """(A)-(F) for one command step. Returns list of failures."""
    bad, evs = [], step["events"]
    idx = {k: [i for i, e in enumerate(evs) if e["k"] == k] for k in ("torque_on", "torque_off", "hold", "goal", "goal1", "cam", "enter")}
    enters = [(i, e["msg"]) for i, e in enumerate(evs) if e["k"] == "enter"]
    on_prompt = [i for i, m in enters if m.startswith("Torque will turn ON")]
    sup_prompt = [i for i, m in enters if m.startswith("SUPPORT THE ARM")]
    if idx["torque_on"] and (not on_prompt or on_prompt[0] > idx["torque_on"][0]):
        bad.append("(A) torque on before the Enter")
    if exp.get("torque") is False and idx["torque_on"]:
        bad.append("torque went on (expected never)")
    if idx["hold"] and not step["crashed"]:
        h = idx["hold"][0]
        later = [i for i in idx["goal"] if i > h]
        if len(later) > 1:                 # one: the hold itself (the present position written as the goal)
            bad.append(f"(B) {len(later) - 1} commands sent after the stop besides the hold")
    if not step["crashed"]:
        for i in idx["torque_off"]:
            if evs[i].get("held_before") and not any(p < i for p in sup_prompt):
                bad.append("(C) torque off while held, without the SUPPORT Enter")
                break
    if exp.get("no_move_after_look"):
        tops = [i for i, e in enumerate(evs) if e["k"] in ("cam", "cam_stale") and e.get("cam") == "top"]
        hold_i = idx["hold"][0] if idx["hold"] else len(evs)
        if tops and any(tops[0] < i < hold_i for i in idx["goal"]):
            bad.append("restart: commands sent after the first top picture")
    ph = step["physics"] or []
    if step["result"] == "ok":
        if is_air:
            if any(r.get("moved") is False and not r["ok"] for r in ph):
                bad.append("(D) in the air, a piece not moved was pushed")
        elif not all(r["ok"] for r in ph if r.get("moved")) or not all(r["ok"] for r in ph if not r.get("moved")):
            bad.append("(D) FALSE COMPLETION: ok but the physics is not right")
    if step["exit"] is None and not step["crashed"] and not step["log"]:
        bad.append("(E) no log written")
    if step["result"] == "stopped" and not step["stop_at"]:
        bad.append("(E) stopped without stop_at")
    rp = step.get("replay")
    if rp is not None and not rp["ok"]:
        bad.append("(F) replay: " + rp["why"])
    return bad


def judge(case, out) -> tuple:
    """(match, notes). 可否未確定: only the always-checks"""
    notes, match = [], True
    cmd_steps = [s for s in case["steps"] if "user" not in s]
    res = [r for r in out["steps"] if "user" not in r]
    for st, r in zip(cmd_steps, res):
        exp = st.get("expect", {})
        is_air = "--air" in st["args"] and float(st["args"][st["args"].index("--air") + 1]) > 0
        bad = checks(r, exp, is_air)
        if case["class"] != "可否未確定" and exp:
            got = "refused" if r["exit"] else ("crashed" if r["crashed"] else r["result"])
            if exp.get("result") and got != exp["result"]:
                bad.append(f"result {got}, expected {exp['result']}")
            text = (r["exit"] or "") + " " + r["stop"]
            if exp.get("reason_any") and got in ("stopped", "refused") and not any(w in text for w in exp["reason_any"]):
                bad.append(f"reason: {text.strip()[:160]!r} has none of {exp['reason_any']}")
        if bad:
            match = False
            notes.append(f"step {r['step']}: " + "; ".join(bad))
    return match, notes


# ---------------------------------------------------------------- freeze / run
def code_sha() -> str:
    sys.path.insert(0, HERE)
    import puzzle_run as PR
    h = hashlib.sha256(PR.code_sha().encode())
    for p in (os.path.join(HERE, "mj_world.py"), os.path.join(SIM, "physics", "run_physics.py"), os.path.join(HERE, "so101_common.py"),
              os.path.join(HERE, "k1_run.py")):
        h.update(open(p, "rb").read())
    return h.hexdigest()


def freeze():
    cases = json.load(open(CASES, encoding="utf-8"))
    os.makedirs(OUTD, exist_ok=True)
    fr = {"date": datetime.datetime.now().isoformat(timespec="seconds"), "cases_sha256": {c["id"]: sha_text(c) for c in cases["cases"]},
          "cases": cases["cases"], "code_sha256": code_sha(), "judge_sha256": file_sha(os.path.abspath(__file__))}
    out = os.path.join(OUTD, "frozen_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".json")
    json.dump(fr, open(out, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    print(f"froze {len(cases['cases'])} cases -> {os.path.relpath(out, ROOT)}")


def run(only, jobs, frozen_path, tag):
    import glob
    frozen_path = frozen_path or sorted(glob.glob(os.path.join(OUTD, "frozen_*.json")))[-1]
    fr = json.load(open(frozen_path, encoding="utf-8"))
    cur = json.load(open(CASES, encoding="utf-8"))["cases"]
    stale = []
    if code_sha() != fr["code_sha256"]:
        stale.append("code differs from the frozen")
    if file_sha(os.path.abspath(__file__)) != fr["judge_sha256"]:
        stale.append("judge differs from the frozen")
    cases = [c for c in fr["cases"] if not only or c["id"] in only]
    for c in cases:
        cc = next((x for x in cur if x["id"] == c["id"]), None)
        if cc is None or sha_text(cc) != fr["cases_sha256"][c["id"]]:
            stale.append(f"{c['id']} text differs from the frozen")
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + (f"_{tag}" if tag else "")
    base = os.path.join(OUTD, "run_" + stamp)
    os.makedirs(base, exist_ok=True)
    procs, results, queue = {}, [], list(cases)
    while queue or procs:
        while queue and len(procs) < jobs:
            c = queue.pop(0)
            d = os.path.join(base, c["id"])
            os.makedirs(d, exist_ok=True)
            fo = open(os.path.join(d, "stdout.txt"), "w", encoding="utf-8")
            p = subprocess.Popen([sys.executable, os.path.abspath(__file__), "one", c["id"], d, "--frozen", frozen_path], cwd=ROOT,
                                 stdout=fo, stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONIOENCODING="utf-8"))
            procs[p] = (c, d, fo, time.time())
        for p in list(procs):
            if p.poll() is None:
                continue
            c, d, fo, t0 = procs.pop(p)
            fo.close()
            try:
                out = json.load(open(os.path.join(d, "case.json"), encoding="utf-8"))
                m, notes = judge(c, out)
            except (OSError, ValueError) as e:
                out, m, notes = None, False, [f"no case result ({e!r}); see {os.path.relpath(d, ROOT)}/stdout.txt"]
            results.append({"id": c["id"], "class": c["class"], "group": c.get("group"), "match": m, "notes": notes,
                            "secs": round(time.time() - t0), "steps": summarize(out)})
            print(f"{c['id']} {c['class']}: {'match' if m else 'NOT MATCH'} " + " | ".join(notes)[:300], flush=True)
        time.sleep(0.5)
    results.sort(key=lambda r: r["id"])
    doc = {"date": stamp, "frozen": os.path.relpath(frozen_path, ROOT), "stale": stale, "results": results}
    json.dump(doc, open(os.path.join(OUTD, f"result_{stamp}.json"), "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    open(os.path.join(OUTD, f"result_{stamp}.md"), "w", encoding="utf-8").write(table(doc, cases))
    graded = [r for r in results if r["class"] != "可否未確定"]
    print(f"graded {sum(r['match'] for r in graded)}/{len(graded)} match; all-check failures in "
          f"{sum(not r['match'] for r in results if r['class'] == '可否未確定')} unsure cases; stale: {stale or 'none'}")
    print(f"wrote outputs/s4/result_{stamp}.md")
    return 0 if all(r["match"] for r in results) and not stale else 1


def summarize(out):
    if not out:
        return []
    rows = []
    for r in out["steps"]:
        if "user" in r:
            rows.append({"user": r["user"], "msgs": r["msgs"]})
            continue
        ph = r["physics"] or []
        rows.append({"result": "refused" if r["exit"] else ("crashed" if r["crashed"] else r["result"]),
                     "why": ((r["exit"] or "") + r["stop"])[:220], "goal_writes": r["n_goal_writes"],
                     "torque_on": any(e["k"] == "torque_on" for e in r["events"]), "prompts": [p[:40] for p in r["prompts"]],
                     "physics": " ".join(f"{x['name']}:{'ok' if x['ok'] else 'NG'}" for x in ph),
                     "replay": (r.get("replay") or {}).get("ok"), "log": r["log"], "world": [w["msg"][:80] for w in r["world_events"]]})
    return rows


def table(doc, cases) -> str:
    L = [f"# S4 実機の経路の異常の試験（{doc['date']}）", "", f"固定: `{doc['frozen']}`。古い所: {', '.join(doc['stale']) or 'なし'}", "",
         "| 回 | 分類 | 群 | 一致 | 段ごとの結果（理由の頭） | 再生 | 注 |", "|---|---|---|---|---|---|---|"]
    for r in doc["results"]:
        parts, rp = [], []
        for s in r["steps"]:
            if "user" in s:
                parts.append(f"（人：{s['user']}）")
                continue
            parts.append(f"{s['result']}" + (f"「{s['why'][:70]}」" if s["why"] else "") + ("" if s["torque_on"] else "・力なし"))
            rp.append("—" if s["replay"] is None else ("同じ" if s["replay"] else "違う"))
        L.append(f"| {r['id']} | {r['class']} | {r.get('group') or ''} | {'○' if r['match'] else '×'} | {' → '.join(parts)} | "
                 f"{' '.join(rp)} | {' / '.join(r['notes'])[:200]} |")
    return "\n".join(L).replace("\\", "/") + "\n"


def main():
    ap = argparse.ArgumentParser(usage=__doc__)
    ap.add_argument("cmd", choices=["freeze", "run", "one", "probe"])
    ap.add_argument("rest", nargs="*")
    ap.add_argument("--only", default=None)
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--frozen", default=None)
    ap.add_argument("--tag", default=None)
    ap.add_argument("--pieces", type=int, default=1)
    a = ap.parse_args()
    if a.cmd == "freeze":
        freeze()
    elif a.cmd == "run":
        sys.exit(run(set(a.only.split(",")) if a.only else None, a.jobs, a.frozen, a.tag))
    elif a.cmd == "one":
        cid, d = a.rest
        src = json.load(open(a.frozen, encoding="utf-8"))["cases"] if a.frozen else json.load(open(CASES, encoding="utf-8"))["cases"]
        c = next(x for x in src if x["id"] == cid)
        out = run_one(c, d)
        json.dump(out, open(os.path.join(d, "case.json"), "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    elif a.cmd == "probe":
        d = a.rest[0]
        pc = ["--pieces", str(a.pieces)] if a.pieces else []
        c = {"id": "probe", "world": {"pieces": a.pieces}, "steps": [{"args": ["--set", "2"] + pc + ["--air", "40"]},
                                                                      {"args": ["--set", "2"] + pc}]}
        out = run_one(c, d, probe=True)
        json.dump(out, open(os.path.join(d, "case.json"), "w", encoding="utf-8"), indent=1, ensure_ascii=False)
        for r in out["steps"]:
            print(f"step {r['step']}: {r['result']} exit {r['exit']} writes {r['n_goal_writes']} {r['secs']} s; stop {r['stop'][:200]}")
            held = [(e["n"], tuple(e["held"])) for e in r["events"] if e["k"] == "goal"]
            chg, prev = [], None
            for n, h in held:
                if h != prev:
                    chg.append((n, h))
                    prev = h
            print("  held changes (write n, pieces lifted):", chg)
            print("  cams:", [(e["cam"], e["n"]) for e in r["events"] if e["k"] == "cam"], "prompts:", [p[:30] for p in r["prompts"]])
            print("  physics:", [(x["name"], x["ok"]) for x in r["physics"]])


if __name__ == "__main__":
    main()
