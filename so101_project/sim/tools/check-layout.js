// 置き場をファイルから読む(計画書_完成形まで の S-5)の検算。
//   見ている世界: scene-data.js の buildFromUi に LAYOUT = { pieces: [{ x, z, yaw }] }(机 = 0 の座標・mm・度)を渡す。
//   仮説: (1) こちらで決めた置き場をそのままファイルにして読ませると、動きが元と同じになる(読み方が置き場を変えない)
//         (2) ピースが回って置かれていても(上のカメラで見つけたときの向き)、届く所なら、盤の向きに直して置ける
//             (置いた位置の差 1mm 未満・向き 1° 未満・当たりなし)
//         (3) 届く輪の中の、盤・台・ほかのピースに重ならない所へでたらめに置くと、何割かは解けない。解けない理由を数える
//   負の検算: 重なる・台に乗る・届かない・位置が無い ファイルで、理由が出るか
//   実行: node tools/check-layout.js [--file layouts/xxx.json --set n] [--n 試す数(既定 24)] [--seed 1]
const fs = require("fs"), path = require("path");
const { load, ROOT } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js");
const M = SCENE_MODEL, def = M.defaultUiValues();
const arg = k => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : undefined; };
const sets = M.controls.find(c => c.key === "SET").options;
const mm = v => (v * 1000).toFixed(1);
function line(label, c) {
  return `[${label}] ` + (c.solved
    ? `解けた 順番 ${c.order.join("→")} / 掴んだ瞬間のずれ ${(c.worstPick * 1000).toFixed(2)}mm・${c.worstPickYaw.toFixed(2)}° / 置いた位置の差 ${(c.worstPlace * 1000).toFixed(3)}mm・向き ${c.worstYaw.toFixed(3)}° / いちばん近い ${mm(c.minClear)}mm(${c.minClearPair})/ 当たり ${c.hitFrames} 回 / ${c.T.toFixed(1)} 秒`
    : `解けない: ${c.why}`) + (c.layoutNotes.length ? `\n    置き場の注意: ${c.layoutNotes.join(" / ")}` : "");
}
// 置いた位置・向きは爪の側から作るので、掴む位置や向きの符号を間違えても 0 と出る(点検 C11 の 2)。掴んだ瞬間のずれも見る
const good = c => c.solved && c.worstPlace < 0.001 && c.worstYaw < 1 && c.worstPick < 0.001 && c.worstPickYaw < 1 && c.hitFrames === 0 && c.minTip > 0;
let ng = 0;

// ファイル 1 つだけ見る
if (arg("file")) {
  const L = JSON.parse(fs.readFileSync(path.resolve(arg("file")), "utf8"));
  const c = M.buildFromUi({ SET: +(arg("set") ?? L.set ?? 0), GAP: L.gap ?? def.GAP, SOL: 0, LAYOUT: L })._check;
  console.log(line(arg("file"), c));
  process.exit(good(c) ? 0 : 1);
}

// (1) こちらで決めた置き場をファイルにして読ませる → 元と同じ
console.log("(1) こちらで決めた置き場を、そのままファイルにして読ませる");
for (const o of sets) {
  const ui = { SET: o.value, GAP: def.GAP, SOL: 0 };
  const a = M.buildFromUi(ui)._check;
  const L = { pieces: a.slots.map(s => ({ x: s[0] * 1000, z: s[1] * 1000, yaw: 0 })) };
  const b = M.buildFromUi(Object.assign({ LAYOUT: L }, ui))._check;
  const same = a.order.join() === b.order.join() && a.frames === b.frames && Math.abs(a.T - b.T) < 1e-9 && Math.abs(a.worstPlace - b.worstPlace) < 1e-9 &&
    a.motion.every((f, i) => f.q.every((v, k) => Math.abs(v - b.motion[i].q[k]) < 1e-9));
  if (!same) ng++;
  console.log(`  ${o.label}: ${same ? "同じ(順番・コマ数・関節角のすべて)" : "★NG: 違う"}`);
}

// (2) 同じ位置で、ピースを回して置く
console.log("(2) 同じ位置で、ピースを回して置く(3×4。全部のピースを同じ角度だけ回す)");
{
  const ui = { SET: 2, GAP: def.GAP, SOL: 0 }, a = M.buildFromUi(ui)._check;
  for (const yaw of [15, 30, 45, 90, -60, -135, 180]) {
    // 回すとほかのピースや盤に近づくので、ピースの真ん中を中心に回す(局所の (0,0) のマスの位置をずらす)
    const L = { pieces: a.slots.map((s, i) => {
      const cells = a.pieceCells[i], cx = cells.reduce((v, c) => v + c[0], 0) / cells.length, cz = cells.reduce((v, c) => v + c[1], 0) / cells.length;
      const dx = s[0] - cx, dz = s[1] - cz, c = Math.cos(yaw * Math.PI / 180), sn = Math.sin(yaw * Math.PI / 180);
      return { x: (cx + dx * c - dz * sn) * 1000, z: (cz + dx * sn + dz * c) * 1000, yaw };
    }) };
    const c = M.buildFromUi(Object.assign({ LAYOUT: L }, ui))._check;
    console.log("  " + line(`${yaw}° 回す`, c));
    if (c.solved && !good(c)) { ng++; console.log("  ★NG: 解けたのに、置いた位置・向き・当たりのどれかが外れている"); }
  }
}

// (3) でたらめに置く
const N = +(arg("n") ?? 24);
let seed = +(arg("seed") ?? 1);
const rnd = () => { seed |= 0; seed = seed + 0x6D2B79F5 | 0; let t = Math.imul(seed ^ seed >>> 15, 1 | seed); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; };
console.log(`(3) 届く輪の中へでたらめに置く(3×4・${N} 回・種 ${arg("seed") ?? 1}。重なり・届く範囲の外は置き直す)`);
{
  const ui = { SET: 2, GAP: def.GAP, SOL: 0 }, base = M.buildFromUi(ui)._check;
  const PAN_X = SO101.joints[0].origin[0];
  const why = {}; let ok = 0, tries = 0, bad = 0, hit = 0, hitPairs = {}, done = 0;
  for (let n = 0; n < N; n++) {
    let c = null;
    for (let k = 0; k < 5000; k++) {
      tries++;
      const L = { pieces: base.slots.map(() => {
        const r = 0.11 + rnd() * 0.10, a = (rnd() * 2 - 1) * 100 * Math.PI / 180;   // 肩から 110〜210mm・左右 ±100°
        return { x: (PAN_X + r * Math.cos(a)) * 1000, z: r * Math.sin(a) * 1000, yaw: Math.round(rnd() * 360 - 180) };
      }) };
      const q = M.buildFromUi(Object.assign({ LAYOUT: L, LAYOUT_ONLY: true }, ui))._check;   // 置き場だけ先に確かめる
      if (q.layoutWhy || q.layoutNotes.some(m => /届く範囲|重なる/.test(m))) continue;
      c = M.buildFromUi(Object.assign({ LAYOUT: L }, ui))._check; c.L = L; break;
    }
    if (!c) { console.log("  ★置ける所が 5000 回で見つからない"); bad++; break; }
    done++;
    if (good(c)) ok++;
    else if (c.solved && c.hitFrames && c.worstPlace < 0.001 && c.worstYaw < 1 && c.worstPick < 0.001) { hit++; const k = c.minClearPair.replace(/pc\d/g, "ピース"); hitPairs[k] = (hitPairs[k] || 0) + 1; }
    else if (c.solved) { bad++; console.log("  ★NG: " + line("解けたが置いた位置か向きが外れ", c)); }
    else { const k = c.why.replace(/[-0-9.]+/g, "#").replace(/のピース: .*?: /, "のピース: ").slice(0, 60); why[k] = (why[k] || 0) + 1; }
  }
  ng += bad;
  console.log(`  解けた(当たりなし・差 1mm 未満): ${ok}/${done}(置き直しを含めて ${tries} 回作った)`);
  if (hit) console.log(`  動きは作れたが当たる ${hit} 回(いちばん深い組): ${Object.keys(hitPairs).map(k => k + " " + hitPairs[k] + " 回").join("・")}`);
  Object.keys(why).sort((a, b) => why[b] - why[a]).forEach(k => console.log(`  解けない ${why[k]} 回: ${k}`));
}

// 負の検算
console.log("負の検算(どれも理由か書き置きが出るはず)");
{
  const ui = { SET: 2, GAP: def.GAP, SOL: 0 }, a = M.buildFromUi(ui)._check;
  const base = a.slots.map(s => ({ x: s[0] * 1000, z: s[1] * 1000, yaw: 0 }));
  const PAN_X = SO101.joints[0].origin[0] * 1000;
  const cases = [
    ["2 つのピースを同じ所に置く", { pieces: [base[0], base[0], base[2]] }, c => /重なる/.test(c.why)],
    ["ピースを盤の上に置く", { pieces: [{ x: a.boardCells[0][0] * 1000, z: a.boardCells[0][1] * 1000, yaw: 0 }, base[1], base[2]] }, c => /盤に重なる/.test(c.why)],
    ["ピースを台の板の上に置く", { pieces: [{ x: (a.foot[0][0] + a.foot[0][2]) * 500, z: (a.foot[0][1] + a.foot[0][3]) * 500, yaw: 0 }, base[1], base[2]] }, c => /台に重なる/.test(c.why)],
    ["ピースを肩から 300mm に置く", { pieces: [{ x: PAN_X + 300, z: 0, yaw: 0 }, base[1], base[2]] }, c => !c.solved && c.layoutNotes.some(m => /届く範囲/.test(m))],
    ["ピースの位置がファイルに無い", { pieces: [base[0], base[1]] }, c => /ファイルに無い/.test(c.why)],
    ["ピースを台の足跡から 10mm に置く", { pieces: [{ x: (a.foot[0][0] + a.foot[0][2]) * 500, z: (a.foot[0][3] + 0.010 + 0.007) * 1000, yaw: 0 }, base[1], base[2]] }, c => c.layoutNotes.some(m => /台の足跡まで \d+mm/.test(m)) && !/足跡/.test(c.why || "")],   // 弾かずに書き置く(当たりは判定で見る。2026-11-18)
  ];
  for (const [label, L, want] of cases) {
    const c = M.buildFromUi(Object.assign({ LAYOUT: L }, ui))._check;
    const okc = want(c);
    if (!okc) ng++;
    console.log(`  ${label}: ${okc ? "OK" : "★NG"} — ${c.solved ? "解けた" : c.why}${c.layoutNotes.length ? "(" + c.layoutNotes.join(" / ") + ")" : ""}`);
  }
}
// 負の検算(符号): 30° 回して置いたファイルで、掴む位置か掴む向きの回し方を逆にすると、掴んだ瞬間のずれが出るはず(点検 C11 の 2)
{
  const L = JSON.parse(fs.readFileSync(path.join(ROOT, "layouts", "example_3x4_rot30.json"), "utf8"));
  for (const [flip, label, want] of [["glw", "掴む位置を逆に回す", c => c.worstPick > 0.001], ["yaw", "掴む向きを逆に回す", c => c.worstPickYaw > 1]]) {
    const c = M.buildFromUi({ SET: 2, GAP: def.GAP, SOL: 0, LAYOUT: L, _FLIP: flip })._check;
    const okc = want(c);
    if (!okc) ng++;
    console.log(`  ${label}: ${okc ? "OK" : "★NG"} — 掴んだ瞬間のずれ ${(c.worstPick * 1000).toFixed(2)}mm・${c.worstPickYaw.toFixed(2)}°(当たり ${c.hitFrames} 回)`);
  }
}
console.log(ng ? `NG ${ng} 件` : "OK");
process.exit(ng ? 1 : 0);
