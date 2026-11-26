"""列 P 5 回目(2026-11-20 C42k): 爪の線(GRIP_EMPTY_DEG 3 度・GRIP_WIDE_DEG 2 度。どちらも【仮】)を、決めるのに使っていない例で確かめる。

tools/puzzle_mj.py は書き換えない(既定の回の指令の sha256 を保つため)。ここで World を子のクラスに差し替えて、puzzle_mj.main() を流す。
足すのは世界の側だけ(制御へは渡さない。Controller は前と同じ観測しか受け取らない):
  --grip-bias DEG   : 爪の読み(cam "joints" の gripper)に足すずれ。+ は「閉じた側に読む」(過去の past が + になる向き)
  --grip-noise DEG  : 爪の読みの雑音(正規分布の標準偏差、URDF の度)。読むたびに引く(種は --grip-seed、無ければ 1000 + --seed)
      爪の読みのずれは cam "joints" の観測だけに入れる。上・手首の絵の観測の gripper の値(描画に使う)には入れない
      (制御は絵の観測の gripper を使っていない。puzzle_mj.Controller を読んで確かめた)。
  --fault2 SPEC     : 世界の側の失敗を足す(何回でも)
      pnudge:PID:DX:DZ     そのピースを持つ指令の 1 回目(爪が閉じ始める時)に、ピースを机の上で (DX, DZ) mm 動かす
      tilt:PID:N:AXIS:DEG  そのピースを持つ指令の N 回目に、ピースを体の原点まわりに世界の軸 AXIS(x | y。MuJoCo の水平の軸)で DEG 度回す
                           (爪の間で傾いて残る滑りの代わり。その後は物理しだい)
  ほかの引数は puzzle_mj.py と同じ(--world-fault drop:PID:N も使える)。
流し方: sim\\physics\\.venv\\Scripts\\python.exe tools\\puzzle_mj_gripcheck.py --seed 6 --grip-bias 1 --grip-noise 0.3
"""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np                 # noqa: E402
import mujoco                      # noqa: E402
import puzzle_mj as PM             # noqa: E402


def _pop(argv, name, cast, default, many=False):
    out, i, rest = [], 0, []
    while i < len(argv):
        if argv[i] == name and i + 1 < len(argv):
            out.append(cast(argv[i + 1])); i += 2
        else:
            rest.append(argv[i]); i += 1
    argv[:] = rest
    return out if many else (out[-1] if out else default)


argv = sys.argv[1:]
GB = _pop(argv, "--grip-bias", float, 0.0)
GN = _pop(argv, "--grip-noise", float, 0.0)
GS = _pop(argv, "--grip-seed", int, None)
F2 = _pop(argv, "--fault2", str, None, many=True)
seed = 1
for k, v in zip(argv, argv[1:]):
    if k == "--seed":
        seed = int(v)
GS = 1000 + seed if GS is None else GS
sys.argv = [sys.argv[0]] + argv


class GWorld(PM.World):
    def __init__(self, *a, **kw):
        self.f2 = []
        for s in F2:
            k, *v = s.split(":")
            if k == "pnudge":
                self.f2.append(["pnudge", int(v[0]), float(v[1]), float(v[2]), False])
            elif k == "tilt":
                self.f2.append(["tilt", int(v[0]), int(v[1]), v[2], float(v[3]), False])
            else:
                raise SystemExit(f"unknown --fault2 {s}")
        self.grng = random.Random(GS)
        self.grip_reads = []
        super().__init__(*a, **kw)
        self.say(f"(world, C42k) gripper reading bias {GB:+.2f} deg (closed side +), noise sd {GN:.2f} deg, rng seed {GS}; extra faults {F2}",
                 grip_err={"bias_deg": GB, "noise_sd_deg": GN, "seed": GS, "fault2": F2})

    def observe(self, cam, conv):
        if cam != "joints":
            return super().observe(cam, conv)
        q, jaw = self.joints_urdf()
        e = GB + (self.grng.gauss(0.0, GN) if GN > 0 else 0.0)
        jaw_r = jaw - self.sign * e                     # 閉じる向き = sign が + なら角度が減る向き(World._ctrl の握り込みと同じ)
        self.grip_reads.append(round(e, 4))
        return {"cam": "joints", "lerobot": conv.to_L(q, jaw_r), "t_world": round(self.t, 4)}

    def _faults(self, nf):
        super()._faults(nf)
        for fl in self.f2:
            if fl[-1]:
                continue
            pid = fl[1]
            if nf["held"] != pid:
                continue
            n = self.nheld.get(pid, 0)
            name = self.scene["pieces"][pid]["name"]
            if fl[0] == "pnudge" and n == 1:
                st = self.piece_states()[pid]
                self._move_piece(pid, st["x"] + fl[2], st["z"] + fl[3])
                fl[-1] = True
                self.say(f"(world fault C42k) {name} was moved by {fl[2]:+.1f},{fl[3]:+.1f} mm as the gripper began to close at t {self.t:.2f} s", world_fault="pnudge")
            elif fl[0] == "tilt" and n == fl[2]:
                jn = self.model.joint("pc%d_free" % self.scene["pieces"][pid]["idx"])
                a = jn.qposadr[0]
                ax = {"x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}[fl[3]]
                dq = np.zeros(4); mujoco.mju_axisAngle2Quat(dq, np.array(ax), math.radians(fl[4]))
                q0 = np.array(self.data.qpos[a + 3:a + 7]); q1 = np.zeros(4)
                mujoco.mju_mulQuat(q1, dq, q0)            # 世界の軸で回す(左から掛ける)
                self.data.qpos[a + 3:a + 7] = q1
                mujoco.mj_forward(self.model, self.data)
                fl[-1] = True
                self.say(f"(world fault C42k) {name} was tilted {fl[4]:+.1f} deg about the world {fl[3]} axis in the gripper at t {self.t:.2f} s (held command {n})",
                         world_fault="tilt")


PM.World = GWorld

if __name__ == "__main__":
    PM.main()
