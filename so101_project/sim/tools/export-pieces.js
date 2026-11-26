// ピースの STL を書き出す(計画書_完成形まで の S-7)。puzzle.js の形 × ブロックの辺 × すき間 × つなぐ棒。
//   形: 立方体(辺 CELL)を、すき間 GAP をあけて並べ、となりどうしを四角い棒(幅・高さ ROD)でつなぐ。
//     scene-data.js の絵と同じ置き方(棒は高さの真ん中・幅 10mm が既定)。単位 mm、Z が上、底が Z = 0。
//     立方体と棒の和を、座標を詰めた格子で 1 つの閉じた面にする(重なった殻を残さない)。
//   検算: (1) 閉じているか(どの辺も、向きが逆の 2 つの三角形に使われる)
//         (2) 体積 = 立方体 × 数 + 棒 × すき間の長さ(重なりは数えない)
//         (3) sim/ のマスの真ん中が中、ほかの所(すき間で棒の無い所・棒の上下)が外か(巻き数)
//         (4) build-models.py の read_stl で読めて、三角形の数が合うか
//         (5) 負の検算: 三角形を 1 つ抜くと (1) が NG、マスを 1 つずらすと (3) が NG
//   実行: node tools/export-pieces.js [--cell 14] [--gap 15] [--rod 10] [--rodz mid|bottom] [--shapes "L J O"] [--out out/pieces]
//   ★刷れるかは見ていない(棒は高さの真ん中にあるので、すき間 15mm を橋渡しで刷ることになる。--rodz bottom で底にそろえられる)
const fs = require("fs"), path = require("path"), { execFileSync } = require("child_process");
const { load, ROOT } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js");
const arg = k => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : undefined; };
const def = SCENE_MODEL.defaultUiValues();
const CELL = +(arg("cell") ?? SCENE_MODEL.CELL_DEFAULT), GAP = +(arg("gap") ?? def.GAP), ROD = +(arg("rod") ?? 10), RODZ = arg("rodz") || "mid";
const P = CELL + GAP;
const setShapes = [...new Set(SCENE_MODEL.controls.find(c => c.key === "SET").options.map(o => o.label.split("・")[1].split(" ")).flat())];
const shapes = arg("shapes") ? arg("shapes").split(" ") : setShapes;
const outDir = path.resolve(arg("out") || path.join(ROOT, "out", "pieces"));
fs.mkdirSync(outDir, { recursive: true });

function boxesOf(cells) {            // [x0, x1, y0, y1, z0, z1](mm)
  const has = new Set(cells.map(c => c.join(","))), out = [], h = CELL / 2, r = ROD / 2;
  const zc = RODZ === "bottom" ? r : h;
  cells.forEach(c => out.push([c[0] * P - h, c[0] * P + h, c[1] * P - h, c[1] * P + h, 0, CELL]));
  if (GAP > 0) cells.forEach(c => [[1, 0], [0, 1]].forEach(d => {
    if (!has.has((c[0] + d[0]) + "," + (c[1] + d[1]))) return;
    const mx = (c[0] + d[0] / 2) * P, my = (c[1] + d[1] / 2) * P;
    const lx = d[0] ? P / 2 : r, ly = d[1] ? P / 2 : r;          // 棒は立方体の真ん中から真ん中まで(和をとるので重なってよい)
    out.push([mx - lx, mx + lx, my - ly, my + ly, zc - r, zc + r]);
  }));
  return out;
}
// 箱の和の表面(座標を詰めた格子で、埋まったセルと空のセルの境の面を出す)
function unionMesh(boxes) {
  const ax = [0, 1, 2].map(a => [...new Set(boxes.flatMap(b => [b[2 * a], b[2 * a + 1]]))].sort((p, q) => p - q));
  const n = ax.map(v => v.length - 1);
  const filled = (i, j, k) => {
    if (i < 0 || j < 0 || k < 0 || i >= n[0] || j >= n[1] || k >= n[2]) return false;
    const c = [(ax[0][i] + ax[0][i + 1]) / 2, (ax[1][j] + ax[1][j + 1]) / 2, (ax[2][k] + ax[2][k + 1]) / 2];
    return boxes.some(b => c[0] > b[0] && c[0] < b[1] && c[1] > b[2] && c[1] < b[3] && c[2] > b[4] && c[2] < b[5]);
  };
  const tris = [];
  const quad = (a, b, c, d) => { tris.push([a, b, c], [a, c, d]); };   // 外から見て反時計回り
  for (let i = 0; i < n[0]; i++) for (let j = 0; j < n[1]; j++) for (let k = 0; k < n[2]; k++) {
    if (!filled(i, j, k)) continue;
    const x0 = ax[0][i], x1 = ax[0][i + 1], y0 = ax[1][j], y1 = ax[1][j + 1], z0 = ax[2][k], z1 = ax[2][k + 1];
    if (!filled(i - 1, j, k)) quad([x0, y0, z0], [x0, y0, z1], [x0, y1, z1], [x0, y1, z0]);
    if (!filled(i + 1, j, k)) quad([x1, y0, z0], [x1, y1, z0], [x1, y1, z1], [x1, y0, z1]);
    if (!filled(i, j - 1, k)) quad([x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1]);
    if (!filled(i, j + 1, k)) quad([x0, y1, z0], [x0, y1, z1], [x1, y1, z1], [x1, y1, z0]);
    if (!filled(i, j, k - 1)) quad([x0, y0, z0], [x0, y1, z0], [x1, y1, z0], [x1, y0, z0]);
    if (!filled(i, j, k + 1)) quad([x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]);
  }
  return tris;
}
function writeStl(file, tris, name) {
  const b = Buffer.alloc(84 + 50 * tris.length);
  b.write(("sim/tools/export-pieces.js " + name).slice(0, 79), 0, "ascii");
  b.writeUInt32LE(tris.length, 80);
  tris.forEach((t, i) => {
    const o = 84 + 50 * i, u = t[1].map((v, k) => v - t[0][k]), w = t[2].map((v, k) => v - t[0][k]);
    const nn = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]], L = Math.hypot(...nn) || 1;
    nn.forEach((v, k) => b.writeFloatLE(v / L, o + 4 * k));
    t.forEach((p, m) => p.forEach((v, k) => b.writeFloatLE(v, o + 12 + 12 * m + 4 * k)));
  });
  fs.writeFileSync(file, b);
}
function readStl(file) {
  const b = fs.readFileSync(file), n = b.readUInt32LE(80), tris = [];
  for (let i = 0; i < n; i++) { const o = 84 + 50 * i + 12; tris.push([0, 1, 2].map(m => [0, 1, 2].map(k => b.readFloatLE(o + 12 * m + 4 * k)))); }
  return tris;
}
// (1) 閉じているか: 向き付きの辺 a→b ごとに、b→a がちょうど 1 つ
function closed(tris) {
  const key = p => p.map(v => v.toFixed(4)).join(","), cnt = new Map();
  tris.forEach(t => [0, 1, 2].forEach(m => { const e = key(t[m]) + ">" + key(t[(m + 1) % 3]); cnt.set(e, (cnt.get(e) || 0) + 1); }));
  let bad = 0;
  cnt.forEach((v, e) => { const [a, b] = e.split(">"); if (v !== 1 || cnt.get(b + ">" + a) !== 1) bad++; });
  return bad;
}
const volume = tris => tris.reduce((s, t) => s + (t[0][0] * (t[1][1] * t[2][2] - t[1][2] * t[2][1]) - t[0][1] * (t[1][0] * t[2][2] - t[1][2] * t[2][0]) + t[0][2] * (t[1][0] * t[2][1] - t[1][1] * t[2][0])) / 6, 0);
// 巻き数(点が閉じた面の中なら 1)
function winding(tris, p) {
  let w = 0;
  tris.forEach(t => {
    const a = t[0].map((v, k) => v - p[k]), b = t[1].map((v, k) => v - p[k]), c = t[2].map((v, k) => v - p[k]);
    const la = Math.hypot(...a), lb = Math.hypot(...b), lc = Math.hypot(...c);
    const det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
    const dot = (u, v) => u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    w += 2 * Math.atan2(det, la * lb * lc + dot(a, b) * lc + dot(b, c) * la + dot(c, a) * lb);
  });
  return w / (4 * Math.PI);
}
function probes(cells) {             // [点, 中のはずか, 名前]
  const has = new Set(cells.map(c => c.join(","))), out = [];
  cells.forEach(c => {
    out.push([[c[0] * P, c[1] * P, CELL / 2], true, `マス (${c})の真ん中`]);
    out.push([[c[0] * P + CELL / 2 - 0.5, c[1] * P + CELL / 2 - 0.5, CELL - 0.5], true, `マス (${c})の角の内側`]);
    if (GAP > 0) [[1, 0], [0, 1], [-1, 0], [0, -1]].forEach(d => {
      const g = [c[0] * P + d[0] * (CELL / 2 + GAP / 2), c[1] * P + d[1] * (CELL / 2 + GAP / 2)];
      const linked = has.has((c[0] + d[0]) + "," + (c[1] + d[1]));
      const zc = RODZ === "bottom" ? ROD / 2 : CELL / 2;
      out.push([[g[0], g[1], zc], linked, `マス (${c})の ${d} のすき間の${linked ? "棒" : "(棒なし)"}`]);
      if (linked && zc + ROD / 2 + 0.5 < CELL) out.push([[g[0], g[1], zc + ROD / 2 + 0.5], false, `マス (${c})の ${d} の棒の上`]);
      if (linked) out.push([[g[0] + (d[1] ? ROD / 2 + 0.5 : 0), g[1] + (d[0] ? ROD / 2 + 0.5 : 0), zc], false, `マス (${c})の ${d} の棒の横`]);
    });
  });
  return out;
}
function checkProbes(tris, cells) {
  const bad = [];
  probes(cells).forEach(([p, inside, name]) => { const w = winding(tris, p); if ((w > 0.5) !== inside) bad.push(`${name} 巻き数 ${w.toFixed(2)}`); });
  return bad;
}

console.log(`[ピースの STL] ブロックの辺 ${CELL}mm・すき間 ${GAP}mm・棒 ${ROD}×${ROD}mm(${RODZ === "bottom" ? "底にそろえる" : "高さの真ん中"})→ ${path.relative(ROOT, outDir)}`);
let ng = 0;
const written = [];
shapes.forEach(sh => {
  const cells = PUZZLE.SHAPES[sh];
  if (!cells) { console.log(`  ${sh}: ★puzzle.js に無い形`); ng++; return; }
  const tris = unionMesh(boxesOf(cells));
  const file = path.join(outDir, `piece_${sh}_c${CELL}_g${GAP}_r${ROD}${RODZ === "bottom" ? "b" : ""}.stl`);
  writeStl(file, tris, sh);
  const back = readStl(file);
  const nRod = GAP > 0 ? boxesOf(cells).length - cells.length : 0;
  const want = cells.length * CELL ** 3 + nRod * GAP * ROD * ROD, vol = volume(back), open = closed(back), pr = checkProbes(back, cells);
  const ok = open === 0 && Math.abs(vol - want) < 1e-3 * want && pr.length === 0;
  if (!ok) ng++;
  written.push([file, back.length]);
  console.log(`  ${sh}: 三角形 ${back.length}・閉じていない辺 ${open}・体積 ${(vol / 1000).toFixed(3)} cm³(式で ${(want / 1000).toFixed(3)})・中と外の点 ${probes(cells).length - pr.length}/${probes(cells).length} が合う ${ok ? "OK" : "★NG " + pr.slice(0, 3).join(" / ")}`);
});
// (4) build-models.py の read_stl で読む
try {
  const py = `import importlib.util,sys\ns=importlib.util.spec_from_file_location('bm',r'${path.join(ROOT, "tools", "build-models.py")}')\nm=importlib.util.module_from_spec(s);s.loader.exec_module(m)\nfor f in sys.argv[1:]: print(len(m.read_stl(f)))`;
  const got = execFileSync("python", ["-c", py, ...written.map(w => w[0])], { encoding: "utf8" }).trim().split(/\s+/).map(Number);
  const okPy = got.length === written.length && got.every((v, i) => v === written[i][1]);
  if (!okPy) ng++;
  console.log(`  (4) build-models.py の read_stl で読んだ三角形の数: ${got.join("・")} ${okPy ? "OK(書いた数と同じ)" : "★NG"}`);
} catch (e) { ng++; console.log("  (4) ★NG: build-models.py で読めない: " + e.message.split("\n")[0]); }
// (5) 負の検算
{
  const cells = PUZZLE.SHAPES[shapes[0]], tris = unionMesh(boxesOf(cells));
  const open = closed(tris.slice(1));
  const moved = cells.map((c, i) => i === 0 ? [c[0] - 0.6, c[1]] : c);   // 1 つ目のマスを外へ 0.6 ピッチずらした形と、元のマスの点を比べる(すき間 0 でも真ん中が外に出る)
  const pr = checkProbes(unionMesh(boxesOf(moved)), cells);
  const ok5 = open > 0 && pr.length > 0;
  if (!ok5) ng++;
  console.log(`  (5) 負の検算(${shapes[0]}): 三角形を 1 つ抜くと閉じていない辺 ${open}・マスを 1 つずらすと合わない点 ${pr.length} ${ok5 ? "OK(どちらも見つけた)" : "★NG"}`);
}
console.log(ng ? `NG ${ng} 件` : "OK");
process.exit(ng ? 1 : 0);
