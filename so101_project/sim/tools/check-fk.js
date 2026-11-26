// 検算: KIN(関節軸の形)の順運動学 ⇄ URDF の行列の積 が一致するか。
//   2 つは別の作り。一致すれば、関節軸の作り方(so101.js)に写し間違いが無い。
//   見ていないもの: URDF の数字そのものが本物か(前提 U-1)・実機の角度の符号(U-2)。
const { load } = require("./_load");
load("kinematics.js", "so101.js");
const D = Math.PI / 180;
let seed = 7; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
let worst = 0, worstDir = 0;
const N = 2000;
for (let k = 0; k < N; k++) {
  const q = SO101.limitsDeg.map(([a, b]) => a + (b - a) * rnd());
  const A = KIN.fk(SO101.axes, q, SO101.tool);
  const B = SO101.urdfFK(q.map(v => v * D), 0).gripper_frame;
  const e = Math.hypot(A.pos[0] - B.p[0], A.pos[1] - B.p[1], A.pos[2] - B.p[2]);
  const bd = [B.R[2], B.R[5], B.R[8]];
  const c = A.dir[0] * bd[0] + A.dir[1] * bd[1] + A.dir[2] * bd[2];
  worst = Math.max(worst, e); worstDir = Math.max(worstDir, Math.acos(Math.min(1, c)) / D);
}
const z = SO101.urdfFK([0, 0, 0, 0, 0], 0);
const f = v => v.map(x => (x * 1000).toFixed(1)).join(", ");
console.log("[SO-101 / so101_new_calib.urdf] 全関節 0 の姿勢(画面の座標・mm):");
for (const k of Object.keys(z)) console.log("  " + k.padEnd(16) + " 位置 (" + f(z[k].p) + ")");
SO101.joints.forEach(j => console.log("  軸 " + j.name.padEnd(14) + " 向き (" + j.axis.map(x => x.toFixed(3)).join(", ") + ")  可動域 " + j.limitsDeg.map(x => x.toFixed(1)).join(" 〜 ") + "°"));
console.log("  道具の向き (" + SO101.tool.dir.map(x => x.toFixed(3)).join(", ") + ")");
console.log(`検算 ${N} 姿勢(可動域の中の乱数): 道具の点の差 最大 ${(worst * 1000).toExponential(2)} mm / 向きの差 最大 ${worstDir.toExponential(2)}°`);
const ok = worst < 1e-6 && worstDir < 1e-4;
console.log(ok ? "OK(KIN と URDF の行列の積が一致)" : "NG(一致しない)");
process.exit(ok ? 0 : 1);
