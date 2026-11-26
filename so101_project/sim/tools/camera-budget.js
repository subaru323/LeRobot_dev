// 上と手首のカメラの設計の下見(計画書_完成形まで の S-12・docs/plan/想定動作_SO101パズル.md)。
//   ちくわ排除機構の「光学設計の手順」(docs/固定カメラ_光学設計方針/3rd §3)の第 1〜4 段(要る細かさ → 画角 → 画素数)と
//   第 8 段(誤差の合計で成立を見る)を、SO-101 のパズルに置き直した。
//   見ている世界: scene-data.js の 4 種(既定のすき間・台あり・爪のまわりの測り直しあり)。
//   段 1(要る細かさ): 許せる位置のずれを、段ごとに分ける(点検 C14 の 1)。
//     つかむ段 T_pick = min(指の両側の余裕 FIT_MARGIN, つかむ段(真上に着く〜閉じる)のいちばん近い所) … カメラで見て直せる
//     離す段   T_place = 離す段(置く〜上へ戻る)のいちばん近い所 … 腕と盤の位置で決まり、つかむ前のカメラでは直せない(参考に出す)
//     要る細かさ = T_pick ÷ 2.5 mm/画素(ちくわの「欠陥の大きさ ÷ 2.5」を、位置のずれに読み替えた【仮】。位置は画素より細かく出せることが多く、厳しめ)。
//   段 2〜4(上のカメラ): 写す姿勢(最初のコマ)で、置き場のピースと盤のマスの上の面(高さ = 辺)の 4 つの角。縦の画角 × 絵の大きさ を振り、
//     角が全部絵に入るか(ちくわ shot-planner の「重心でなく四隅」)・縁からの余裕(絵の短い辺に対して。0.075 未満は「余裕なし」。
//     ちくわの視野の余裕 0.15 を両側に分けた読み【仮】)・角でのいちばん粗い mm/画素。
//   段 8(誤差の合計): √(腕² + カメラ²) × 2 ≤ T_pick を満たす腕の誤差の上限。カメラ = 1 画素 と 0.5 画素 × 角のいちばん粗い mm/画素【仮】。
//   手首のカメラ: つかむ場所の上に着いて爪を開き終えたコマ(tHover + 爪を開く 0.4 秒)で、URDF の wrist_camera_link の原点から
//     (光軸 = その z、絵の下 = その y【仮】。光の中心の位置は確かめていない)、つかむピースの上の面の角とマスの真ん中を見る(640×480・縦の画角【仮】)。
//     光軸は真下から 25° ほど傾く(真上から見るのではない)。爪(j5・動く爪の三角形)に遮られる視線を、部品ごとに数える。
//   検算: (a) 上のカメラの絵の真ん中の横の mm/画素 = 光軸に沿った距離 ÷ f (b) project → unproject で戻る
//         (c) 外れ: 上のカメラを 40° 傾けると角が絵から出る(大きな誤りしか捕まえない)
//         (d) 手首の光軸と指先の向きの差が 20° 以内 (e) 指先の向こう(カメラ → 指先 を 1.5 倍に伸ばした所)への視線は遮られる
//         (f) 動く爪の三角形の真ん中への視線は、動く爪に遮られる(動く爪の置き方を見る。(e) は j5 しか試していなかった。点検 C14 の 5)
//   見ていないもの: レンズのゆがみ・ピント・光と影・ピースを見つける道具の実力(K6)・絵の上の向きのほかの 3 通り・カメラの台とケーブルの遮り・
//     爪の三角形とカメラの位置のずれ(別の順運動学どうしの突き合わせは (e)(f) だけで、1cm ほどのずれは捕まえない)。
//   実行: node tools/camera-budget.js
const { load } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js");
const M = SCENE_MODEL;
const FOVS = [45, 50, 55, 60, 65, 70, 78, 90], SIZES = [[640, 480], [1280, 720], [1920, 1080]], WFOVS = [60, 78, 90];
const MARGIN_OK = 0.075, D = Math.PI / 180, JAW_OPEN_S = 0.4;
let bad = 0;
const mm = x => (x * 1000).toFixed(2);

function corners(c) {                       // 置き場のピースと盤のマスの、上の面の 4 つの角(机 = 0)
  const h = c.cell / 2, y = c.cell, out = [];
  c.pieceCells.forEach((cells, i) => cells.forEach(xz => {
    const r = c.slotRot[i] * D, cs = Math.cos(r), sn = Math.sin(r);
    for (const [a, b] of [[-h, -h], [h, -h], [h, h], [-h, h]]) out.push({ what: "置き場", p: [xz[0] + a * cs - b * sn, y, xz[1] + a * sn + b * cs] });
  }));
  c.boardCells.forEach(xz => { for (const [a, b] of [[-h, -h], [h, -h], [h, h], [-h, h]]) out.push({ what: "盤", p: [xz[0] + a, y, xz[1] + b] }); });
  return out;
}
function look(cam, fov, W, H, pts) {        // 角が全部入るか・縁の余裕・角でいちばん粗い mm/画素
  const img = SO101.overheadImage(cam, fov, W, H);
  let minMargin = Infinity, worst = 0, out = 0;
  pts.forEach(e => {
    const q = img.project(e.p);
    if (!q || q[0] < 0 || q[0] >= W || q[1] < 0 || q[1] >= H) { out++; return; }
    minMargin = Math.min(minMargin, q[0] / H, (W - q[0]) / H, q[1] / H, (H - q[1]) / H);
    const a = img.unproject(q[0], q[1], e.p[1]), b = img.unproject(q[0] + 1, q[1], e.p[1]), d = img.unproject(q[0], q[1] + 1, e.p[1]);
    worst = Math.max(worst, Math.hypot(b[0] - a[0], b[2] - a[2]), Math.hypot(d[0] - a[0], d[2] - a[2]));
  });
  return { out, minMargin, worst, img };
}
function armMax(T, camErr) { return (T / 2) ** 2 > camErr ** 2 ? Math.sqrt((T / 2) ** 2 - camErr ** 2) : -1; }
const armTxt = a => a < 0 ? "✗" : "≤" + mm(a);
function segHitsTri(o, p, tri) {            // 線分 o→p が三角形の集まり tri に当たるか(両端の 1mm は見ない。Möller–Trumbore)
  const d = [p[0] - o[0], p[1] - o[1], p[2] - o[2]], L = Math.hypot(...d), u = d.map(x => x / L);
  for (let i = 0; i < tri.length; i += 9) {
    const e1 = [tri[i + 3] - tri[i], tri[i + 4] - tri[i + 1], tri[i + 5] - tri[i + 2]], e2 = [tri[i + 6] - tri[i], tri[i + 7] - tri[i + 1], tri[i + 8] - tri[i + 2]];
    const pv = [u[1] * e2[2] - u[2] * e2[1], u[2] * e2[0] - u[0] * e2[2], u[0] * e2[1] - u[1] * e2[0]];
    const det = e1[0] * pv[0] + e1[1] * pv[1] + e1[2] * pv[2];
    if (Math.abs(det) < 1e-15) continue;
    const tv = [o[0] - tri[i], o[1] - tri[i + 1], o[2] - tri[i + 2]], a = (tv[0] * pv[0] + tv[1] * pv[1] + tv[2] * pv[2]) / det;
    if (a < 0 || a > 1) continue;
    const qv = [tv[1] * e1[2] - tv[2] * e1[1], tv[2] * e1[0] - tv[0] * e1[2], tv[0] * e1[1] - tv[1] * e1[0]], b = (u[0] * qv[0] + u[1] * qv[1] + u[2] * qv[2]) / det;
    if (b < 0 || a + b > 1) continue;
    const t = (e2[0] * qv[0] + e2[1] * qv[1] + e2[2] * qv[2]) / det;
    if (t > 0.001 && t < L - 0.001) return true;
  }
  return false;
}
const near = (c, t) => c.motion.reduce((a, g) => Math.abs(g.t - t) < Math.abs(a.t - t) ? g : a, c.motion[0]);
const phaseMin = (c, a, b) => c.clearAt.filter(e => e.t >= a - 1e-9 && e.t <= b + 1e-9).reduce((m, e) => e.d < m.d ? e : m, { d: Infinity, pair: "" });

const builds = [0, 1, 2, 3].map(set => ({ set, d: M.buildFromUi({ SET: set, SOL: 0 }) }));
console.log("[上のカメラ] 写す姿勢で、置き場と盤の角を見る(縦の画角 × 画素数。数字は角でいちばん粗い mm/画素)");
const T_PICK = {};
for (const { set, d } of builds) {
  const c = d._check;
  if (!c.solved || !d.overheadCamera) { console.log(`[種 ${set}] 解けない・台なし: ${c.why}`); bad++; continue; }
  let pick = { d: Infinity }, place = { d: Infinity };
  c.stepTimes.forEach(s => {
    const a = phaseMin(c, s.tHover, s.tPick), b = phaseMin(c, s.tPlace, s.tDone);
    if (a.d < pick.d) pick = Object.assign({ who: s.name }, a);
    if (b.d < place.d) place = Object.assign({ who: s.name }, b);
  });
  const T = Math.min(c.grip.fitMargin, pick.d), need = T / 2.5, pts = corners(c), cam = d.overheadCamera;
  T_PICK[set] = T;
  console.log(`[種 ${set}] 角 ${pts.length}。つかむ段のいちばん近い所 ${mm(pick.d)}mm(${pick.who}・${pick.pair})→ T_pick = min(指の余裕 ${mm(c.grip.fitMargin)}, それ) = ${mm(T)}mm → 要る細かさ ${mm(need)}mm/画素。` +
    `離す段は ${mm(place.d)}mm(${place.who}・${place.pair}。カメラでは直せない)・解の全体は ${mm(c.minClear)}mm`);
  for (const [W, H] of SIZES) {
    const row = FOVS.map(fov => {
      const r = look(cam, fov, W, H, pts);
      if (r.out) return `${fov}°: 出る ${r.out}`;
      return `${fov}°: ${mm(r.worst)}${r.worst <= need + 1e-9 ? "" : "✗"}・縁 ${r.minMargin.toFixed(3)}${r.minMargin >= MARGIN_OK ? "" : "✗"}・腕 ${armTxt(armMax(T, r.worst))}/${armTxt(armMax(T, r.worst / 2))}`;
    });
    console.log(`  ${W}×${H}: ` + row.join(" | "));
  }
  const img = SO101.overheadImage(cam, 60, 640, 480), ctr = img.unproject(320, 240, c.cell), ctr1 = img.unproject(321, 240, c.cell);
  const dist = Math.hypot(ctr[0] - cam.tip[0], ctr[1] - cam.tip[1], ctr[2] - cam.tip[2]), a1 = Math.hypot(ctr1[0] - ctr[0], ctr1[2] - ctr[2]), a2 = dist / img.f;
  const okA = Math.abs(a1 - a2) < 1e-4 * a2;
  let maxBack = 0;
  pts.forEach(e => { const q = img.project(e.p); if (!q) return; const b = img.unproject(q[0], q[1], e.p[1]); maxBack = Math.max(maxBack, Math.hypot(b[0] - e.p[0], b[2] - e.p[2])); });
  const okB = maxBack < 1e-9;
  const t = 40 * D, rot = v => [v[0] * Math.cos(t) - v[1] * Math.sin(t), v[0] * Math.sin(t) + v[1] * Math.cos(t), v[2]];
  const tilted = look({ tip: cam.tip, dir: rot(cam.dir), up: rot(cam.up) }, 60, 640, 480, pts), okC = tilted.out > 0;
  console.log(`  検算: (a) 真ん中の横 ${mm(a1)} = 距離 ${(dist * 1000).toFixed(1)}mm ÷ f ${img.f.toFixed(1)} = ${mm(a2)}mm/画素 ${okA ? "OK" : "★NG"}・` +
    `(b) 戻り ${(maxBack * 1e6).toExponential(1)}µm ${okB ? "OK" : "★NG"}・(c) 40° 傾けると絵から出る角 ${tilted.out} ${okC ? "OK" : "★NG(鈍い)"}`);
  if (!okA || !okB || !okC) bad++;
}

console.log("[手首のカメラ] つかむ場所の上で爪を開き終えたコマで、つかむピースを見る(640×480・縦の画角【仮】・光軸 = wrist_camera_link の z【仮】)");
for (const { set, d } of builds) {
  const c = d._check;
  if (!c.solved) continue;
  const T = T_PICK[set], need = T / 2.5, ry = c.robotY, h = c.cell / 2;
  const res = WFOVS.map(() => ({ worst: 0, out: 0, slack: Infinity }));
  let rays = 0, axisErr = 0, tilt = 0, tipBlocked = 0, jawBlocked = 0;
  const blockedBy = {}, perStep = [];
  c.stepTimes.forEach(stp => {
    const f = near(c, stp.tHover + JAW_OPEN_S);
    const F = SO101.urdfFK(f.q.map(x => x * D), f.jaw * D), cf = F.wrist_camera_link, gf = F.gripper_frame;
    const tip = [cf.p[0], cf.p[1] + ry, cf.p[2]], dir = [cf.R[2], cf.R[5], cf.R[8]], up = [-cf.R[1], -cf.R[4], -cf.R[7]];
    const toTip = [gf.p[0] - cf.p[0], gf.p[1] - cf.p[1], gf.p[2] - cf.p[2]], nt = Math.hypot(...toTip);
    axisErr = Math.max(axisErr, Math.acos((dir[0] * toTip[0] + dir[1] * toTip[1] + dir[2] * toTip[2]) / nt) / D);
    tilt = Math.max(tilt, Math.acos(-dir[1]) / D);
    const cells = c.pieceCells[stp.pid], r = c.slotRot[stp.pid] * D, cs = Math.cos(r), sn = Math.sin(r), pts = [];
    cells.forEach(xz => { pts.push({ p: [xz[0], c.cell, xz[1]] }); for (const [a, b] of [[-h, -h], [h, -h], [h, h], [-h, h]]) pts.push({ p: [xz[0] + a * cs - b * sn, c.cell, xz[1] + a * sn + b * cs] }); });
    WFOVS.forEach((fov, k) => {
      const lk = look({ tip, dir, up }, fov, 640, 480, pts);
      res[k].out += lk.out; res[k].worst = Math.max(res[k].worst, lk.worst);
      if (!lk.out) pts.forEach(e => { const q = lk.img.project(e.p); res[k].slack = Math.min(res[k].slack, Math.min(q[0], 640 - q[0], q[1], 480 - q[1]) * lk.worst); });
    });
    const tris = M.gripTrisWorld(f.q, f.jaw, ry, false);
    let nb = 0;
    pts.forEach(e => { rays++; const hit = tris.filter(t => segHitsTri(tip, e.p, t.tri)); if (hit.length) { nb++; hit.forEach(t => blockedBy[t.group] = (blockedBy[t.group] || 0) + 1); } });
    perStep.push(`${stp.name} ${nb}/${pts.length}(爪 ${f.jaw.toFixed(0)}°)`);
    const far = [tip[0] + toTip[0] * 1.5, tip[1] + toTip[1] * 1.5, tip[2] + toTip[2] * 1.5];
    if (tris.some(t => segHitsTri(tip, far, t.tri))) tipBlocked++;
    // (f) 動く爪の三角形の頂点の真ん中へ
    const jt = tris.filter(t => t.group === "jaw"), cen = [0, 0, 0];
    let nv = 0;
    jt.forEach(t => { for (let i = 0; i < t.tri.length; i += 3) { cen[0] += t.tri[i]; cen[1] += t.tri[i + 1]; cen[2] += t.tri[i + 2]; nv++; } });
    if (nv && jt.some(t => segHitsTri(tip, cen.map(x => x / nv), t.tri))) jawBlocked++;
  });
  console.log(`  [種 ${set}] 要る細かさ ${mm(need)}mm/画素(T_pick ${mm(T)}mm)・光軸は真下から ${tilt.toFixed(1)}° 傾く・` +
    WFOVS.map((fov, k) => `${fov}°: ${res[k].out ? "出る " + res[k].out : mm(res[k].worst) + (res[k].worst <= need + 1e-9 ? "" : "✗") + "・腕 " + armTxt(armMax(T, res[k].worst)) + "/" + armTxt(armMax(T, res[k].worst / 2)) + "・縁まで " + (res[k].slack * 1000).toFixed(0) + "mm"}`).join(" | "));
  console.log(`    爪に遮られる視線 ${Object.values(blockedBy).length ? Object.entries(blockedBy).map(([g, n]) => g + " " + n).join("・") : "0"}(手ごと: ${perStep.join("・")}。視線 ${rays} 本)`);
  const okD = axisErr < 20, okE = tipBlocked === c.stepTimes.length, okF = jawBlocked === c.stepTimes.length;
  console.log(`    検算: (d) 光軸と指先の向きの差 最大 ${axisErr.toFixed(1)}° ${okD ? "OK" : "★NG"}・(e) 指先の向こうが遮られた ${tipBlocked}/${c.stepTimes.length} ${okE ? "OK" : "★NG"}・` +
    `(f) 動く爪の真ん中が動く爪に遮られた ${jawBlocked}/${c.stepTimes.length} ${okF ? "OK" : "★NG"}`);
  if (!okD || !okE || !okF) bad++;
}
console.log("読み方: 数字 = 角でいちばん粗い mm/画素(✗ = 要る細かさより粗い)・縁 = 絵の短い辺に対する縁までの余裕(✗ = 0.075 未満)・" +
  "腕 ≤a/≤b = 誤差の合計 √(腕² + カメラ²)×2 ≤ T_pick を満たす腕の誤差の上限 mm(a はカメラ 1 画素、b は 0.5 画素。✗ = カメラだけで超える)・" +
  "縁まで = 角が絵の縁まで平らに動ける量(上のカメラが外してよい量の目安。向きのずれは見ていない)・出る n = 絵から出る角の数");
console.log(bad ? `★NG ${bad}` : "検算は全部 OK");
process.exit(bad ? 1 : 0);
