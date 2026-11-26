// 見て動かす の通し(計画書_完成形まで の S-14): 上のカメラの絵だけから置き場を見つけ、その置き場で腕の動きを組み、正解の置き場と比べる。
//   1) 正解の置き場: scene-data.js がこちらで決める置き場を、種で決まる乱数でずらす(位置 ±SHIFT_MM・向き ±YAW_DEG)。置き場として使えない(重なる・届かない)もの、
//      正解の置き場のままでも動きが組めないもの(見つけ方と関係なく解けない)は引き直す。
//   2) 絵: 正解の置き場・写す姿勢の腕で、上のカメラの絵を作る(vision.js の render)。sim/out/vision-loop/ に PNG で書く(見つけたマスの枠を重ねた絵も)。
//   3) 見つける: 絵とカメラの置き方・ピースの形と色だけから置き場を出す(vision.js の detect)。正解は渡さない。
//   4) 動きを組む: 見つけた置き場を LAYOUT として scene-data.js に渡し、つかむ場所と爪の向きを出させる。
//   5) 比べる: つかむ場所(見つけた置き場で決めたもの)を、正解のピースの上に移して、ずれを出す。ずれは爪の閉じる向き(e_a)と、それに直角(e_c)に分ける。
//      つかめる: |e_a| + (辺/2)·|sin(向きのずれ)| < FIT_MARGIN(指を下ろすとき、固定の爪がピースの上に乗らない)。
//      置いたときのずれ【仮定】: 閉じる向きのずれと向きのずれは、平らな爪で挟むと消える(爪に倣う)。直角のずれ e_c がそのまま残る。
//   条件を変える: --light <名前>(光の乱れ。tools/vision-light.js の CASES の名前。雑音もその行の値。2026-11-18)・--noise(画素の雑音・標準偏差 0〜255)・--calib-mm / --calib-deg(見つける側のカメラの置き方を、この量だけずらす = 較正のずれ)・--trials・--set・--seed
//   実行: node tools/vision-loop.js(8 種 × 5 回)/ --set 2 --trials 1 --png(絵を書く)
const fs = require("fs"), path = require("path"), zlib = require("zlib");
const { load, ROOT } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js",
     "chikuwa-collision.js", "self-collision.js", "vision.js");
const M = SCENE_MODEL, V = VISION;
const arg = k => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : undefined; };
const has = k => process.argv.includes("--" + k);
const LIGHT = arg("light") !== undefined ? require("./vision-light").CASES.find(c => c.name === arg("light")) : null;
if (arg("light") !== undefined && !LIGHT) { console.log("--light の名前が無い。名前: " + require("./vision-light").CASES.map(c => c.name).join(",")); process.exit(1); }
const SHIFT_MM = 8, YAW_DEG = 20, NOISE = +(arg("noise") ?? (LIGHT ? LIGHT.noise : 3)), CAL_MM = +(arg("calib-mm") ?? 0), CAL_DEG = +(arg("calib-deg") ?? 0);
const TRIALS = +(arg("trials") ?? 5), SEED = +(arg("seed") ?? 1), PNG = has("png");
const NO_SYM = has("no-sym");   // 負の検算用: 180° 回して同じ形のピースを「180° 回った置き場」と見直すのを止める(scene-data.js の views。2026-11-18)
// --sheet: 盤の下に K1/K5 の紙 v2 を敷いた絵にする(vision.js の sheetTris。2026-11-19)
const SHEET = has("sheet");
const sets = arg("set") !== undefined ? [+arg("set")] : M.controls.find(c => c.key === "SET").options.map(o => o.value);
const OUT = path.join(ROOT, "out", "vision-loop");
function png(file, W, H, rgb) {
  const raw = Buffer.alloc((W * 3 + 1) * H);
  for (let y = 0; y < H; y++) { raw[y * (W * 3 + 1)] = 0; for (let x = 0; x < W * 3; x++) raw[y * (W * 3 + 1) + 1 + x] = rgb[y * W * 3 + x]; }
  const crcT = []; for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; crcT[n] = c >>> 0; }
  const crc = b => { let c = 0xffffffff; for (const x of b) c = crcT[(c ^ x) & 255] ^ (c >>> 8); return (c ^ 0xffffffff) >>> 0; };
  const chunk = (t, d) => { const l = Buffer.alloc(4); l.writeUInt32BE(d.length); const td = Buffer.concat([Buffer.from(t), d]); const c = Buffer.alloc(4); c.writeUInt32BE(crc(td)); return Buffer.concat([l, td, c]); };
  const ih = Buffer.alloc(13); ih.writeUInt32BE(W, 0); ih.writeUInt32BE(H, 4); ih[8] = 8; ih[9] = 2;
  fs.writeFileSync(file, Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk("IHDR", ih), chunk("IDAT", zlib.deflateSync(raw)), chunk("IEND", Buffer.alloc(0))]));
}
function drawLine(im, a, b, col) {
  const n = Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1])) + 1;
  for (let k = 0; k <= n; k++) { const x = Math.floor(a[0] + (b[0] - a[0]) * k / n), y = Math.floor(a[1] + (b[1] - a[1]) * k / n); if (x < 0 || y < 0 || x >= im.W || y >= im.H) continue; const q = (y * im.W + x) * 3; im.rgb[q] = col[0]; im.rgb[q + 1] = col[1]; im.rgb[q + 2] = col[2]; }
}
// カメラの置き方をずらす(較正のずれ): 位置を mm、向きを度だけ、種で決まる向きに
function perturbCam(cam, mm, deg, r) {
  const u = () => { const v = [V.gauss(r), V.gauss(r), V.gauss(r)], n = Math.hypot(...v); return v.map(x => x / n); };
  const dp = u().map(x => x * mm / 1000), ax = u(), R = KIN.rotAA(ax, deg * Math.PI / 180);
  return { tip: cam.tip.map((v, i) => v + dp[i]), dir: KIN.matVec(R, cam.dir), up: KIN.matVec(R, cam.up) };
}
if (PNG) fs.mkdirSync(OUT, { recursive: true });
console.log(`[見て動かす] 絵 640×480・縦の画角 60°・画素の雑音 ${NOISE}・較正のずれ ${CAL_MM}mm / ${CAL_DEG}°・置き場のずらし ±${SHIFT_MM}mm / ±${YAW_DEG}°・${TRIALS} 回ずつ${LIGHT ? "・光の乱れ " + LIGHT.name : ""}`);
let nPick = 0, nOk = 0, nFail = 0, worstA = 0, worstC = 0, worstPos = 0, worstYaw = 0, bad = 0;
for (const set of sets) {
  const base = M.buildFromUi({ SET: set, SOL: 0, NO_REFINE: true, NO_SYM })._check;
  const r = V.rng(SEED * 1000 + set);
  for (let tr = 0; tr < TRIALS; tr++) {
    // 1) 正解の置き場
    let truth = null;
    for (let g = 0; g < 50 && !truth; g++) {
      const L = { pieces: base.pieces.map(p => ({ name: p.name, x: (base.slots[p.idx][0] * 1000 + (r() * 2 - 1) * SHIFT_MM), z: (base.slots[p.idx][1] * 1000 + (r() * 2 - 1) * SHIFT_MM), yaw: (base.slotRot[p.idx] || 0) + (r() * 2 - 1) * YAW_DEG })) };
      // 正解の置き場の受け入れは、--no-sym に依らず見直しなし(NO_SYM)で決める(あり・なしで同じ置き場を引いて比べるため。点検 C28 の 8)
      const chk = M.buildFromUi({ SET: set, SOL: 0, LAYOUT: L, LAYOUT_ONLY: true, NO_SYM: true })._check;
      if (chk.layoutWhy) continue;
      const cA = M.buildFromUi({ SET: set, SOL: 0, LAYOUT: L, NO_REFINE: true, NO_SYM: true, SELF: 0 })._check;
      if (cA.solved && !cA.hitFrames) truth = L;   // 当たりなしも見る(台の足跡まで 20mm 未満の置き場は、弾かずに当たり判定で見るようにしたため。2026-11-18)
    }
    if (!truth) { console.log(`[種 ${set}・${tr}] 使える正解の置き場が引けない`); bad++; continue; }
    const dT = M.buildFromUi({ SET: set, SOL: 0, LAYOUT: truth, NO_REFINE: true, NO_SYM }), cT = dT._check;
    if (!dT.overheadCamera) { console.log("上のカメラの台が無い"); process.exit(1); }
    // 2) 絵
    const lay = { slots: truth.pieces.map(p => [p.x / 1000, p.z / 1000]), rots: truth.pieces.map(p => p.yaw) };
    const im = V.render(V.worldTris(dT, lay, cT.photoQ, cT.homeJaw).concat(SHEET ? V.sheetTris() : []), dT.overheadCamera, { noise: NOISE, seed: SEED * 7919 + set * 31 + tr, light: LIGHT ? LIGHT.light : undefined });
    // 3) 見つける(カメラの置き方は、較正のずれを入れたもの)
    const calib = { cam: CAL_MM || CAL_DEG ? perturbCam(dT.overheadCamera, CAL_MM, CAL_DEG, r) : dT.overheadCamera, fov: SO101.CAM_FOV.top };
    const det = V.detect(im, calib, { pieces: cT.pieces, pitch: cT.pitch, cell: cT.cell });
    if (det.why) { console.log(`[種 ${set}・${tr}] 見つけられない: ${det.why}`); nFail++; continue; }
    // 見つけた置き場と正解の差(マスの真ん中で)
    const per = cT.pieces.map(p => {
      const d = det.layout.pieces[p.idx], t = truth.pieces[p.idx];
      return V.match(p.shape, cT.pitch, [d.x / 1000, d.z / 1000, d.yaw], [t.x / 1000, t.z / 1000, t.yaw]);
    });
    per.forEach(m => { worstPos = Math.max(worstPos, m.worstCell); worstYaw = Math.max(worstYaw, Math.abs(m.phi)); });
    // 4) 見つけた置き場で動きを組む
    const dD = M.buildFromUi({ SET: set, SOL: 0, LAYOUT: det.layout, NO_REFINE: true, NO_SYM }), cD = dD._check;
    if (!cD.solved) { console.log(`[種 ${set}・${tr}] 見つけた置き場で動きが組めない: ${cD.layoutWhy || cD.why}`); nFail++; continue; }
    if (!cD.ok) { console.log(`[種 ${set}・${tr}] 見つけた置き場で組んだ動きが当たる: 当たり ${cD.hitFrames} コマ(いちばん深い ${cD.minClearPair} ${(cD.minClear * 1000).toFixed(1)}mm・${cD.minClearAt != null ? cD.minClearAt.toFixed(1) + " 秒" : ""}・最初 ${cD.firstHit ? cD.firstHit.t.toFixed(1) + " 秒 " + cD.firstHit.bad.join(",") : ""})・腕どうし ${cD.self ? (cD.self.min * 1000).toFixed(1) + "mm" : "見ていない"}`); nFail++; continue; }   // _check.ok(当たり・腕どうしも見る)
    // 5) 比べる
    const rows = [];
    cD.grasps.forEach(g => {
      const m = per[g.pid], pT = m.T.apply([g.cS[0], g.cS[2]]), e = [pT[0] - g.cS[0], pT[1] - g.cS[2]];
      const u = [Math.cos(g.yp * Math.PI / 180), Math.sin(g.yp * Math.PI / 180)], ea = e[0] * u[0] + e[1] * u[1], ec = -e[0] * u[1] + e[1] * u[0];
      const need = Math.abs(ea) + cT.cell / 2 * Math.abs(Math.sin(m.phi * Math.PI / 180)), ok = need < cT.grip.fitMargin;
      nPick++; if (ok) nOk++;
      worstA = Math.max(worstA, Math.abs(ea)); worstC = Math.max(worstC, Math.abs(ec));
      rows.push(`${g.name} e_a ${(ea * 1000).toFixed(2)}・e_c ${(ec * 1000).toFixed(2)}mm・向き ${m.phi.toFixed(2)}°${ok ? "" : " ★つかめない"}`);
    });
    console.log(`[種 ${set}・${tr}] ${rows.join(" / ")}`);
    if (PNG && tr === 0) {
      png(path.join(OUT, `set${set}_camera.png`), im.W, im.H, im.rgb);
      const img = SO101.overheadImage(calib.cam, 60, im.W, im.H), ov = { W: im.W, H: im.H, rgb: Uint8ClampedArray.from(im.rgb) };
      cT.pieces.forEach(p => {
        const d = det.layout.pieces[p.idx], h = cT.cell / 2;
        V.cellsOf(p.shape, cT.pitch, d.x / 1000, d.z / 1000, d.yaw).forEach(c => {
          const cs = Math.cos(d.yaw * Math.PI / 180), sn = Math.sin(d.yaw * Math.PI / 180);
          const cor = [[-h, -h], [h, -h], [h, h], [-h, h]].map(o => img.project([c[0] + o[0] * cs - o[1] * sn, cT.cell, c[1] + o[0] * sn + o[1] * cs]));
          for (let k = 0; k < 4; k++) drawLine(ov, cor[k], cor[(k + 1) % 4], [255, 255, 255]);
        });
      });
      png(path.join(OUT, `set${set}_detected.png`), ov.W, ov.H, ov.rgb);
      fs.writeFileSync(path.join(OUT, `set${set}_layout_detected.json`), JSON.stringify(det.layout, null, 1));
      fs.writeFileSync(path.join(OUT, `set${set}_layout_truth.json`), JSON.stringify(truth, null, 1));
    }
  }
}
console.log(`つかむ ${nPick} 回のうち つかめる ${nOk} 回・見つけられない/組めない ${nFail} 回。見つけた置き場のずれ: マスの真ん中 いちばん大きい ${(worstPos * 1000).toFixed(2)}mm・向き ${worstYaw.toFixed(2)}°。` +
            `つかむ場所のずれ: 閉じる向き ${(worstA * 1000).toFixed(2)}mm・直角 ${(worstC * 1000).toFixed(2)}mm(余裕 FIT_MARGIN ${(base_fm() * 1000).toFixed(1)}mm)`);
function base_fm() { return M.buildFromUi({ SET: sets[0], SOL: 0, NO_REFINE: true, NO_SYM })._check.grip.fitMargin; }
if (nOk < nPick || nFail || bad) process.exitCode = 1;
