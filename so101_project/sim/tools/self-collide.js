// 腕どうしの当たり(自分との干渉。計画書_完成形まで の S-10)の下見: 動きの全コマで、腕の部品(グループ)どうしがめり込んでいないかを測る。
//   見方: グループ A の表面の点(SO101_POINTS。約 3mm 間隔)を、グループ B の形(STL の三角形。全関節 0 の姿勢で作った meshField)の座標へ移し、
//     面の向きで中か外かを見る(queryFast)。中と出た点は巻き数(inside)で確かめ直す。めり込みの深さ = 中の点の、面までのいちばん遠い距離。
//   組: base・j1〜j5・jaw(動く爪)・cam(手首のカメラと台)のすべての 2 つ。隣どうし(関節でつながる組)は、関節のまわりで触れているのが当たり前なので
//     分けて出す(しきい値 DEEP_ADJ より深いものだけ数える)。
//   出すもの: 組ごとに、めり込んだコマの数・いちばん深いめり込み・その時刻と関節角。
//   実行: node tools/self-collide.js [--set 2] [--every 2] / --pose "-90,0,90,90,0"(1 つの姿勢だけ)
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js");
const M = SCENE_MODEL, A = M.ARM;
const arg = k => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : undefined; };
const GROUPS = ["base", "j1", "j2", "j3", "j4", "j5", "jaw", "cam"];
const ADJ = new Set(["base|j1", "j1|j2", "j2|j3", "j3|j4", "j4|j5", "j5|jaw", "j5|cam"]);
const DEEP = 0.0005, DEEP_ADJ = 0.002;
// グループごとの形(全関節 0 の姿勢の座標)
const FIELD = {};
GROUPS.forEach(g => {
  const tri = [];
  Object.keys(SO101_VISUALS).forEach(link => {
    const gg = SO101.LINK_GROUP[link] || (SO101.CAMERA_GROUP[link] ? "cam" : null);
    if (gg !== g) return;
    SO101_VISUALS[link].forEach(v => {
      const T = SO101_TRIS[v.mesh];
      if (!T) throw new Error("三角形が無い: " + v.mesh + "(python tools/build-models.py を流す)");
      const P = SO101.visualPose(link, v.xyz, v.rpy);
      for (let i = 0; i < T.length; i += 3) { const w = SO101.mv(P.R, [T[i] * 1e-4, T[i + 1] * 1e-4, T[i + 2] * 1e-4]); tri.push(w[0] + P.p[0], w[1] + P.p[1], w[2] + P.p[2]); }
    });
  });
  FIELD[g] = COLLIDE.meshField(tri, 1, [0, 0, 0], 0.01);
});
function frameOf(st, g) { return g === "base" ? null : st.frames[(g === "jaw" || g === "cam") ? 4 : (+g.slice(1) - 1)]; }
function toWorld(st, g, p0, jaw) {
  const fr = frameOf(st, g); if (!fr) return p0;
  const p = g === "jaw" ? M.jawRotate(p0, jaw) : p0, d = [p[0] - fr.o[0], p[1] - fr.o[1], p[2] - fr.o[2]], r = KIN.matVec(fr.R, d);
  return [fr.p[0] + r[0], fr.p[1] + r[1], fr.p[2] + r[2]];
}
function toZero(st, g, w, jaw) {
  const fr = frameOf(st, g); if (!fr) return w;
  const R = fr.R, d = [w[0] - fr.p[0], w[1] - fr.p[1], w[2] - fr.p[2]];
  const p = [fr.o[0] + R[0] * d[0] + R[3] * d[1] + R[6] * d[2], fr.o[1] + R[1] * d[0] + R[4] * d[1] + R[7] * d[2], fr.o[2] + R[2] * d[0] + R[5] * d[1] + R[8] * d[2]];
  return g === "jaw" ? M.jawRotate(p, -jaw) : p;
}
// 姿勢 q(度)・爪 jaw(度)で、組ごとのいちばん深いめり込み(正 = めり込み mm 単位でなく m)
function pose(q, jaw) {
  const st = KIN.fk(SO101.axes, q, SO101.tool), out = {};
  const W = {}; GROUPS.forEach(g => { W[g] = A.points[g].map(p => toWorld(st, g, p, jaw)); });
  for (let i = 0; i < GROUPS.length; i++) for (let j = i + 1; j < GROUPS.length; j++) {
    const a = GROUPS[i], b = GROUPS[j], key = a + "|" + b;
    let deep = 0;
    for (const [x, y] of [[a, b], [b, a]]) {
      const F = FIELD[y];
      W[x].forEach(w => {
        const p = toZero(st, y, w, jaw), s = F.queryFast(p);
        if (s < -deep && s < -0.0002 && F.inside(p)) deep = -s;
      });
    }
    out[key] = deep;
  }
  return out;
}
const one = arg("pose");
if (one) {
  const q = one.split(",").map(Number), r = pose(q, +(arg("jaw") ?? 0));
  console.log(`姿勢 ${q.join(",")}°:`);
  Object.entries(r).filter(([, d]) => d > 0).sort((x, y) => y[1] - x[1]).forEach(([k, d]) => console.log(`  ${k}${ADJ.has(k) ? "(隣)" : ""} めり込み ${(d * 1000).toFixed(1)}mm`));
  process.exit(0);
}
const set = +(arg("set") ?? 2), every = +(arg("every") ?? 2);
const c = M.buildFromUi({ SET: set, SOL: 0, NO_REFINE: true })._check, mo = c.motion;
const stat = {};
let n = 0;
for (let k = 0; k < mo.length; k += every) {
  const f = mo[k], r = pose(f.q, f.jaw); n++;
  Object.entries(r).forEach(([key, d]) => {
    const lim = ADJ.has(key) ? DEEP_ADJ : DEEP;
    const s = stat[key] || (stat[key] = { frames: 0, worst: 0, t: 0, q: null });
    if (d > lim) s.frames++;
    if (d > s.worst) { s.worst = d; s.t = f.t; s.q = f.q.map(x => x.toFixed(0)).join(","); }
  });
}
console.log(`[種 ${set}] ${n} コマ(${every} コマおき・全 ${mo.length})。めり込みのしきい値: 隣でない組 ${DEEP * 1000}mm・隣の組 ${DEEP_ADJ * 1000}mm`);
Object.entries(stat).filter(([, s]) => s.worst > 0).sort((x, y) => y[1].worst - x[1].worst).forEach(([k, s]) =>
  console.log(`  ${k.padEnd(8)}${ADJ.has(k) ? "(隣)" : "    "} めり込んだコマ ${String(s.frames).padStart(3)}・いちばん深い ${(s.worst * 1000).toFixed(1)}mm(${s.t.toFixed(2)} 秒・関節 ${s.q})`));
