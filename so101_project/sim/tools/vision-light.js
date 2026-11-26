// 光の乱れの中で、上のカメラの絵からピースを見つけられるか(計画書_完成形まで の K6・2026-11-18)。
//   1) 正解の置き場: vision-loop.js と同じ引き方(既定の置き場を ±8mm・±20° ずらし、置き場として使え、正解のままで当たりなく組めるもの)を、種ごとに --trials 個。
//   2) 乱れごとに絵を描き(vision.js の render の opts.light)、見つけ(detect)、正解と比べる:
//      ・置き場のずれ: マスの真ん中のいちばん大きいずれ(mm)と、向きのずれ(度)。vision.js の match(回して同じ形は入れ替えて比べる)
//      ・つかめるか【近似】: 正解の置き場で組んだつかむ場所(つかむ点・爪の向き)を、見つけた置き場の側へ移し、そのずれを vision-loop.js と同じ式
//        |閉じる向きのずれ| + (辺/2)·|sin(向きのずれ)| < FIT_MARGIN で見る。見つけた置き場で組み直してはいない(組み直すと 1 回 数十秒かかる)。
//        見つけた置き場で組み直す本当の通しは vision-loop.js --light <名前>。
//      ・失敗: 色が見つからない(detect の why)・外れ(マスのずれが辺の半分 7mm 以上)・つかめない(上の近似)
//      ・余裕越え: 置き場を返したのに、マスのずれが余裕 FIT_MARGIN(2mm)以上のピース。置き場ごとには、why が立たずに余裕越えのピースを返した置き場を「危ない」と数える
//        (why が立てば、呼ぶ側はその絵では動かない。点検 C36 の 2)。置き場の成功 = why なし・外れなし・余裕越えなし・つかめない【近似】なし
//      負の検算の行は「失敗した かつ 危ない置き場 0」で合格(見つけられないと言って止まること)
//   3) 表を画面と sim/out/vision-light/table.md・table.json に書く(--set・--only・--trials・--seed を変えた流しは table_part.md・table_part.json。全部の表を上書きしない。点検 C36 の 1)。--png で乱れごとに 1 枚目の絵を sim/out/vision-light/ に書く。
//   乱れの一覧は下の CASES(vision-loop.js --light も同じ一覧を読む)。neg が付いた行は、失敗しなければおかしい(負の検算)。
//   実行: node tools/vision-light.js(8 種 × 2 回 × 乱れ全部。12 分ほど)/ --set 2 --trials 1 --only 斜め30,影30 --png
const fs = require("fs"), path = require("path"), zlib = require("zlib");
const { load, ROOT } = require("./_load");
const CASES = [
  // 名前・光(vision.js render の opts.light。省くと前からの絵)・雑音
  { name: "前からの絵", light: null, noise: 3 },
  { name: "片面の陰", light: {}, noise: 3, note: "光の向きは前と同じ。光の当たらない面を暗くする陰に替えただけ" },
  { name: "暗い0.5", light: { gain: 0.5 }, noise: 3 },
  { name: "暗い0.3", light: { gain: 0.3 }, noise: 3 },
  { name: "明るい1.4", light: { gain: 1.4 }, noise: 3 },
  { name: "明るい1.8", light: { gain: 1.8 }, noise: 3, note: "明るい面が 255 で飽和する" },
  { name: "斜め60", light: { elev: 60, azim: 30 }, noise: 3 },
  { name: "斜め45", light: { elev: 45, azim: 120 }, noise: 3 },
  { name: "斜め30", light: { elev: 30, azim: 200 }, noise: 3, note: "横の面のほうが上の面より明るい" },
  { name: "斜め15", light: { elev: 15, azim: 300 }, noise: 3 },
  { name: "影45", light: { elev: 45, azim: 120, shadow: true }, noise: 3 },
  { name: "影30", light: { elev: 30, azim: 200, shadow: true }, noise: 3, note: "ピースの影 24mm がすき間 15mm を越え、隣のピースにかかる" },
  { name: "むら0.4", light: { grad: 0.4, gradAzim: 0 }, noise: 3, note: "0.3m で明るさ ±40%" },
  { name: "暗がり", light: { blob: { x: 0.2, z: 0.05, r: 0.12, k: 0.9 } }, noise: 3, note: "直の光を 9 割減らす半径 120mm の丸" },
  { name: "色ばらつき8", light: { hue: 8, sat: 0.2 }, noise: 3 },
  { name: "色ばらつき15", light: { hue: 15, sat: 0.35 }, noise: 3, note: "赤(13°)と黄(39°)の間は 26°" },
  { name: "電球色3000K", light: { kelvin: 3000 }, noise: 3 },
  { name: "昼白色4500K", light: { kelvin: 4500 }, noise: 3 },
  { name: "青っぽい9000K", light: { kelvin: 9000 }, noise: 3 },
  { name: "てかり0.3", light: { spec: 0.3, shin: 30 }, noise: 3 },
  { name: "てかり0.6", light: { spec: 0.6, shin: 30 }, noise: 3, note: "真上の光で上の面が全部白っぽくなり、赤の上の面が黄の色合いになる" },
  { name: "雑音8", light: {}, noise: 8 },
  { name: "雑音15", light: {}, noise: 15 },
  { name: "部屋の例", light: { elev: 50, azim: 30, shadow: true, gain: 0.8, kelvin: 4000, hue: 6, sat: 0.2, grad: 0.25, gradAzim: 60, spec: 0.3 }, noise: 5,
    note: "斜めの天井灯・影・やや暗い・昼白色・色のばらつき・むら・てかり" },
  { name: "部屋の例・低い光", light: { elev: 25, azim: 160, shadow: true, gain: 0.7, kelvin: 3500, hue: 8, sat: 0.25, grad: 0.3, gradAzim: 200, spec: 0.3 }, noise: 6 },
  // 負の検算: 見つけられないはずのもの
  { name: "負:真っ暗0.04", light: { gain: 0.04 }, noise: 3, neg: true, note: "色の濃さも明るさも雑音に埋もれる" },
  { name: "負:色温度2000K+色ばらつき30", light: { kelvin: 2000, hue: 30, sat: 0.5 }, noise: 3, neg: true, note: "赤と黄の色合いが入れ替わる" },
];
module.exports = { CASES };
if (require.main !== module) return;

load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js",
     "chikuwa-collision.js", "self-collision.js", "vision.js");
const M = SCENE_MODEL, V = VISION;
const arg = k => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : undefined; };
const has = k => process.argv.includes("--" + k);
const SHIFT_MM = 8, YAW_DEG = 20, TRIALS = +(arg("trials") ?? 2), SEED = +(arg("seed") ?? 1), PNG = has("png"), PNG_AT = +(arg("png-at") ?? 0);   // --png-at: 何番目の置き場の絵を書くか(既定 0)
const DIAG = has("diag");   // ピースごとに、段(A/B)・上の面の広さの比・輪郭のずれを出す
const ONLY = arg("only") ? arg("only").split(",") : null;
const sets = arg("set") !== undefined ? [+arg("set")] : M.controls.find(c => c.key === "SET").options.map(o => o.value);
const OUT = path.join(ROOT, "out", "vision-light");
fs.mkdirSync(OUT, { recursive: true });
function png(file, W, H, rgb) {
  const raw = Buffer.alloc((W * 3 + 1) * H);
  for (let y = 0; y < H; y++) { raw[y * (W * 3 + 1)] = 0; for (let x = 0; x < W * 3; x++) raw[y * (W * 3 + 1) + 1 + x] = rgb[y * W * 3 + x]; }
  const crcT = []; for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; crcT[n] = c >>> 0; }
  const crc = b => { let c = 0xffffffff; for (const x of b) c = crcT[(c ^ x) & 255] ^ (c >>> 8); return (c ^ 0xffffffff) >>> 0; };
  const chunk = (t, d) => { const l = Buffer.alloc(4); l.writeUInt32BE(d.length); const td = Buffer.concat([Buffer.from(t), d]); const c = Buffer.alloc(4); c.writeUInt32BE(crc(td)); return Buffer.concat([l, td, c]); };
  const ih = Buffer.alloc(13); ih.writeUInt32BE(W, 0); ih.writeUInt32BE(H, 4); ih[8] = 8; ih[9] = 2;
  fs.writeFileSync(file, Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk("IHDR", ih), chunk("IDAT", zlib.deflateSync(raw)), chunk("IEND", Buffer.alloc(0))]));
}
const cases = CASES.filter(c => !ONLY || ONLY.includes(c.name));
if (!cases.length) { console.log("--only に合う乱れが無い。名前: " + CASES.map(c => c.name).join(",")); process.exit(1); }

// 1) 正解の置き場(vision-loop.js と同じ受け入れ)
const scenes = [];
let fitMargin = 0, cellMm = 0;
for (const set of sets) {
  const base = M.buildFromUi({ SET: set, SOL: 0, NO_REFINE: true })._check;
  fitMargin = base.grip.fitMargin; cellMm = base.cell * 1000;
  const r = V.rng(SEED * 1000 + set);
  for (let tr = 0; tr < TRIALS; tr++) {
    let truth = null, cA = null;
    for (let g = 0; g < 50 && !truth; g++) {
      const L = { pieces: base.pieces.map(p => ({ name: p.name, x: (base.slots[p.idx][0] * 1000 + (r() * 2 - 1) * SHIFT_MM), z: (base.slots[p.idx][1] * 1000 + (r() * 2 - 1) * SHIFT_MM), yaw: (base.slotRot[p.idx] || 0) + (r() * 2 - 1) * YAW_DEG })) };
      const chk = M.buildFromUi({ SET: set, SOL: 0, LAYOUT: L, LAYOUT_ONLY: true, NO_SYM: true })._check;
      if (chk.layoutWhy) continue;
      cA = M.buildFromUi({ SET: set, SOL: 0, LAYOUT: L, NO_REFINE: true, NO_SYM: true, SELF: 0 })._check;
      if (cA.solved && !cA.hitFrames) truth = L;
    }
    if (!truth) { console.log(`[種 ${set}・${tr}] 使える正解の置き場が引けない`); continue; }
    const dT = M.buildFromUi({ SET: set, SOL: 0, LAYOUT: truth, NO_REFINE: true }), cT = dT._check;
    const lay = { slots: truth.pieces.map(p => [p.x / 1000, p.z / 1000]), rots: truth.pieces.map(p => p.yaw) };
    scenes.push({ set, tr, truth, cT, cam: dT.overheadCamera, tris: V.worldTris(dT, lay, cT.photoQ, cT.homeJaw), grasps: cT.solved ? cT.grasps : [] });
  }
}
console.log(`[光の乱れ] 種 ${sets.join(",")} × ${TRIALS} 回 = 置き場 ${scenes.length} 個・ピース ${scenes.reduce((s, x) => s + x.cT.pieces.length, 0)} 個。乱れ ${cases.length} 通り`);

// 2) 乱れごとに見つけて比べる
const rows = [];
for (const cs of cases) {
  let nPc = 0, nMiss = 0, nOff = 0, nOver = 0, nPick = 0, nNoPick = 0, posW = 0, yawW = 0, posS = 0, why = [], nLayOk = 0, nDanger = 0;
  const t0 = Date.now();
  scenes.forEach((sc, si) => {
    const im = V.render(sc.tris, sc.cam, { noise: cs.noise, seed: SEED * 7919 + sc.set * 31 + sc.tr, light: cs.light });
    const det = V.detect(im, { cam: sc.cam, fov: SO101.CAM_FOV.top }, { pieces: sc.cT.pieces, pitch: sc.cT.pitch, cell: sc.cT.cell });
    const per = [];
    let layBad = !!det.why, layOver = false, np0 = nNoPick;
    sc.cT.pieces.forEach(p => {
      nPc++;
      const d = det.layout.pieces[p.idx], t = sc.truth.pieces[p.idx];
      if (!d) { nMiss++; per[p.idx] = null; return; }
      const m = V.match(p.shape, sc.cT.pitch, [t.x / 1000, t.z / 1000, t.yaw], [d.x / 1000, d.z / 1000, d.yaw]);   // 正解 → 見つけた
      per[p.idx] = m;
      const pos = m.worstCell * 1000;
      if (DIAG) { const q = det.per.find(e => e.name === p.name) || {}; console.log(`   ${cs.name} 種${sc.set}・${sc.tr} ${p.name}: 段 ${q.mode} 画素 ${q.nPix} 上 ${q.nTop} 広さの比 ${q.areaRatio}${q.areaRatioA != null ? "(A " + q.areaRatioA + ")" : ""} 輪郭 ${q.sil}${q.silA != null ? "(A " + q.silA + ")" : ""} → ずれ ${pos.toFixed(2)}mm・${m.phi.toFixed(2)}°`); }
      if (pos >= fitMargin * 1000) { layOver = true; layBad = true; }
      if (pos >= cellMm / 2) { nOff++; return; }
      if (pos >= fitMargin * 1000) nOver++;
      posW = Math.max(posW, pos); posS += pos; yawW = Math.max(yawW, Math.abs(m.phi));
    });
    if (det.why) why.push(`種${sc.set}・${sc.tr}: ${det.why}`);
    sc.grasps.forEach(g => {
      const m = per[g.pid]; nPick++;
      if (!m || m.worstCell * 1000 >= cellMm / 2) { nNoPick++; return; }
      const gd = m.T.apply([g.cS[0], g.cS[2]]), e = [g.cS[0] - gd[0], g.cS[2] - gd[1]];    // 見つけた置き場では、つかむ点は gd に来る
      const u = [Math.cos(g.yp * Math.PI / 180), Math.sin(g.yp * Math.PI / 180)], ea = e[0] * u[0] + e[1] * u[1];
      if (!(Math.abs(ea) + sc.cT.cell / 2 * Math.abs(Math.sin(m.phi * Math.PI / 180)) < fitMargin)) nNoPick++;
    });
    if (nNoPick > np0) layBad = true;
    if (!layBad) nLayOk++;
    if (!det.why && layOver) nDanger++;
    if (PNG && si === PNG_AT) {
      const file = path.join(OUT, `${cs.name.replace(/[:+]/g, "_")}.png`);
      png(file, im.W, im.H, im.rgb);
    }
  });
  const ok = nPc - nMiss - nOff;
  const row = { name: cs.name, noise: cs.noise, light: cs.light, neg: !!cs.neg, note: cs.note || "", pieces: nPc, miss: nMiss, off: nOff, over: nOver, layouts: scenes.length, layOk: nLayOk, danger: nDanger,
                posMaxMm: +posW.toFixed(2), posMeanMm: ok ? +(posS / ok).toFixed(2) : null, yawMaxDeg: +yawW.toFixed(2), picks: nPick, noPick: nNoPick, why: why.slice(0, 3), sec: +((Date.now() - t0) / 1000).toFixed(1) };
  row.fail = nMiss + nOff + nOver + nNoPick > 0 || nLayOk < scenes.length;
  rows.push(row);
  console.log(`${cs.neg ? "(負)" : "    "}${cs.name.padEnd(18)} 置き場の成功 ${nLayOk}/${scenes.length}・危ない ${nDanger}・見つからない ${nMiss}・外れ ${nOff}・余裕越え ${nOver}/${nPc} ピース・マスのずれ 最大 ${row.posMaxMm}mm(平均 ${row.posMeanMm}mm)・向き 最大 ${row.yawMaxDeg}°・つかめない【近似】 ${nNoPick}/${nPick}${why.length ? "・" + why[0] : ""}`);
}

// 3) 書く
const md = ["# 光の乱れの中で見つけた置き場のずれ（`node tools/vision-light.js` が書いた）", "",
  `種 ${sets.join(",")} × ${TRIALS} 回（置き場 ${scenes.length} 個・ピース ${scenes.reduce((s, x) => s + x.cT.pieces.length, 0)} 個）。絵 640×480・縦の画角 60°。マスのずれは外れを除いた最大と平均。外れ = マスのずれが ${cellMm / 2}mm 以上。余裕越え = 置き場を返したのに、マスのずれが ${(fitMargin * 1000).toFixed(1)}mm 以上。危ない = why が立たずに余裕越えのピースを返した置き場。置き場の成功 = why なし・外れなし・余裕越えなし・つかめない【近似】なし。` +
  `つかめない【近似】= 正解の置き場で組んだつかむ所を見つけた置き場へ移したずれが、余裕 ${(fitMargin * 1000).toFixed(1)}mm を越える（組み直してはいない）。`, "",
  `命令: node tools/vision-light.js ${process.argv.slice(2).join(" ")}（${new Date().toISOString()}）。雑音の種と色のばらつきの種は決まった値（1 通り）。`, "",
  "| 乱れ | 雑音 | 置き場の成功 | 危ない置き場 | 見つからない | 外れ | 余裕越え | マスのずれ 最大 mm | 平均 mm | 向き 最大 ° | つかめない【近似】 | 注 |", "|---|---|---|---|---|---|---|---|---|---|---|---|"];
rows.forEach(r => md.push(`| ${r.neg ? "（負）" : ""}${r.name} | ${r.noise} | ${r.layOk}/${r.layouts} | ${r.danger} | ${r.miss}/${r.pieces} | ${r.off}/${r.pieces} | ${r.over}/${r.pieces} | ${r.posMaxMm} | ${r.posMeanMm ?? "-"} | ${r.yawMaxDeg} | ${r.noPick}/${r.picks} | ${r.note} |`));
const PART = arg("set") !== undefined || ONLY || TRIALS !== 2 || SEED !== 1, TB = PART ? "table_part" : "table";
fs.writeFileSync(path.join(OUT, TB + ".md"), md.join("\n") + "\n");
fs.writeFileSync(path.join(OUT, TB + ".json"), JSON.stringify({ sets, trials: TRIALS, seed: SEED, fitMarginMm: fitMargin * 1000, rows }, null, 1));
console.log(`書いた: ${path.relative(ROOT, path.join(OUT, TB + ".md"))}`);
const negPass = rows.filter(r => r.neg).every(r => r.fail && !r.danger), posFail = rows.filter(r => !r.neg && r.fail).map(r => r.name);
console.log(`負の検算: ${rows.some(r => r.neg) ? (negPass ? "失敗するはずの乱れは失敗した" : "★失敗するはずの乱れで見つけられた、または危ない置き場を返した") : "なし"}。失敗した乱れ: ${posFail.length ? posFail.join("・") : "なし"}`);
if (!negPass || posFail.length) process.exitCode = 1;
