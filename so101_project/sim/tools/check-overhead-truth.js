// 検算: overhead-truth.js が書く「ピースのマスの角の画素」が、画面(index.html?_cam=top)で撮った絵と合うか(計画書_完成形まで の S-9)。
//   正解は so101.js の overheadImage の式で、絵は three.js の PerspectiveCamera で描く。別の作り方の 2 つが合えば、式の向き・画角・原点の取り違えは無い。
//   撮り方: tools/mock-shot.sh(headless Chrome・GPU)を 640×480 の窓で。絵は sim/out/overhead-truth/<名前>.png に残す(K6 の練習用)。
//   測り方(色は、色合いがいちばん近いピースの色。鮮やかさ 0.35 未満は「ピースでない」。机は 0.28 まで。
//     最初は 0.5 にしたが、緑のピースが絵では 0.47 で描かれ、4×5 で 40 点を取りこぼした。2026-11-17 の夜):
//     内: マスごとに、上の面の真ん中と、4 つの角から真ん中へ 2 画素入った点 → 全部そのピースの色であること
//     外: マスの立方体の 8 つの角を絵に打った多角形の、角から 3 画素外の点(同じピースのほかのマス・棒の多角形の中は除く)
//         → 95% 以上が、そのピースの色でないこと(縁のぼかしと影のため、全部とはしない)
//   負の検算(同じ絵で、正解の側だけ壊す): 左右を裏返す(2026-11-17 の夜まで tools の式はこの向きだった)・
//     ピースを前へ 5mm ずらす・縦の画角を 55° と取り違える・角だけ yaw を逆に回す(回したピースのある組だけ)・
//     立方体の辺を 28mm と取り違える(外が働くか)。どの組でも、内か外で合格の条件を外れること(点検 C13 の 6)。
//     上の面を机の高さ 0 と取り違えるのは、外れない組があった(2026-11-17 の夜)ので、目安として出すだけにした。
//   あわせて: 正解の画素を、その高さの水平の面へ戻すと、元の角に 0.05mm 以内で戻ること(同じ式を逆にたどるだけなので、丸めの確認)。
//   ★確かめているのは、式と three.js の「向きの決め方」(右・上・画角・縦横の比・原点)と、ピースの回し方が同じこと。
//     カメラの置き方(位置・傾き)・光の中心・光軸の位置は、絵と正解が同じ値(scene-data.js の overheadCamera)を使うので確かめていない(点検 C13 の 2)。
//     見分けの細かさは向きで違う(上の面のすぐ下に同じ色の横の面が写るため)。出力の「目安」の行を見る。
//   見ていないもの: レンズのゆがみ・実物の色と光。ピースを見つける道具そのもの(K6)。
//   実行: node tools/check-overhead-truth.js(2 分ほど)
const fs = require("fs"), path = require("path"), zlib = require("zlib"), { execFileSync } = require("child_process");
const T = require("./overhead-truth");
const { ROOT } = require("./_load");
const OUT = path.join(ROOT, "out", "overhead-truth"), W = T.W, H = T.H;
fs.mkdirSync(OUT, { recursive: true });

function readPng(file) {                    // 8 ビットの RGB / RGBA・飛び越しなし だけ読む
  const b = fs.readFileSync(file);
  let p = 8, w, h, ct, idat = [];
  while (p < b.length) {
    const len = b.readUInt32BE(p), type = b.toString("ascii", p + 4, p + 8), d = b.subarray(p + 8, p + 8 + len);
    if (type === "IHDR") { w = d.readUInt32BE(0); h = d.readUInt32BE(4); ct = d[9]; if (d[8] !== 8 || d[12] !== 0) throw new Error("この PNG は読めない"); }
    if (type === "IDAT") idat.push(d);
    p += 12 + len;
  }
  const bpp = ct === 6 ? 4 : ct === 2 ? 3 : 0;
  if (!bpp) throw new Error("色の型 " + ct + " は読めない");
  const raw = zlib.inflateSync(Buffer.concat(idat)), stride = w * bpp, px = Buffer.alloc(h * stride);
  for (let y = 0; y < h; y++) {
    const f = raw[y * (stride + 1)], src = raw.subarray(y * (stride + 1) + 1, (y + 1) * (stride + 1));
    for (let x = 0; x < stride; x++) {
      const a = x >= bpp ? px[y * stride + x - bpp] : 0, up = y ? px[(y - 1) * stride + x] : 0, ul = (y && x >= bpp) ? px[(y - 1) * stride + x - bpp] : 0;
      let v = src[x];
      if (f === 1) v += a; else if (f === 2) v += up; else if (f === 3) v += (a + up) >> 1;
      else if (f === 4) { const q = a + up - ul, pa = Math.abs(q - a), pb = Math.abs(q - up), pc = Math.abs(q - ul); v += (pa <= pb && pa <= pc) ? a : pb <= pc ? up : ul; }
      px[y * stride + x] = v & 255;
    }
  }
  return { w, h, at: (u, v) => { const i = Math.floor(v) * stride + Math.floor(u) * bpp; return [px[i], px[i + 1], px[i + 2]]; } };
}
function hsv(c) { const [r, g, b] = c.map(x => x / 255), mx = Math.max(r, g, b), mn = Math.min(r, g, b), d = mx - mn;
  let hh = 0; if (d) hh = mx === r ? ((g - b) / d) % 6 : mx === g ? (b - r) / d + 2 : (r - g) / d + 4;
  return [(hh * 60 + 360) % 360, mx ? d / mx : 0]; }
const hex = s => [1, 3, 5].map(i => parseInt(s.slice(i, i + 2), 16));
function classify(rgb, palette) {           // いちばん近い色合いのピースの番号。鮮やかでなければ -1
  const [hh, s] = hsv(rgb); if (s < 0.35) return -1;
  let best = -1, bd = 1e9;
  palette.forEach((c, i) => { const d0 = Math.abs(hsv(c)[0] - hh), d = Math.min(d0, 360 - d0); if (d < bd) { bd = d; best = i; } });
  return best;
}
const hull = T.hull;
function inPoly(q, poly) { let c = false; for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
  const a = poly[i], b = poly[j]; if ((a[1] > q[1]) !== (b[1] > q[1]) && q[0] < (b[0] - a[0]) * (q[1] - a[1]) / (b[1] - a[1]) + a[0]) c = !c; } return c; }

function shoot(job) {
  const png = path.join(OUT, job.name + ".png"), tmp = path.join(OUT, "_shot").split(path.sep).join("/");
  fs.rmSync(tmp, { recursive: true, force: true });
  const q = [`SET=${job.ui.SET}`, `GAP=${job.ui.GAP}`, job.ui.LAYOUT ? "LAYOUT=" + encodeURIComponent(JSON.stringify(job.ui.LAYOUT)) : "", "_cam=top", "_t=0"].filter(Boolean).join("&");
  const out = execFileSync("bash", [path.join(ROOT, "tools", "mock-shot.sh"), ".", tmp, "4"], { cwd: ROOT, env: Object.assign({}, process.env, { WINDOW: "640,480", QUERY: q, PREFIX: "s_" }) }).toString();
  const f = path.join(tmp, "s_t4s.png");
  if (!fs.existsSync(f)) throw new Error("撮れなかった: " + out);
  fs.renameSync(f, png); fs.rmSync(tmp, { recursive: true, force: true });
  return png;
}
// 正解(t)と絵(img)を比べる。project(点) → 画素 を差し替えるか、geo(マス, ピース) → { top, bot, center }(机 = 0・m)を差し替えると、正解の側だけ壊せる
function compare(t, img, project, geo) {
  const pal = t.pieces.map(p => hex(p.color)), r = { inN: 0, inBad: 0, outN: 0, outBad: 0, bad: [] };
  const P = project || (q => t._img.project(q));
  t.pieces.forEach((p, i) => {
    const G = p.cells.map(c => geo ? geo(c, p) : { top: c._top, bot: c._bot, center: [c.center[0] / 1000, t.cell_mm / 1000, c.center[1] / 1000] });
    const polys = p.cells.map((c, k) => hull(G[k].top.concat(G[k].bot).map(P)));
    // 棒(scene-data.js と同じ: 隣のマスの間、幅・高さ 10mm、高さの真ん中)の多角形
    const rods = [];
    p.cells.forEach((a, ia) => p.cells.forEach((b, ib) => {
      if (ib <= ia) return;
      const ca = [a.center[0] / 1000, a.center[1] / 1000], cb = [b.center[0] / 1000, b.center[1] / 1000];
      if (Math.abs(Math.hypot(cb[0] - ca[0], cb[1] - ca[1]) - (t.cell_mm + t.gap_mm) / 1000) > 1e-4) return;
      const ux = (cb[0] - ca[0]), uz = (cb[1] - ca[1]), n = Math.hypot(ux, uz), px = -uz / n * 0.005, pz = ux / n * 0.005, y0 = t.cell_mm / 2000 - 0.005, y1 = y0 + 0.010;
      const pts = []; [ca, cb].forEach(e => [-1, 1].forEach(s => [y0, y1].forEach(y => pts.push(P([e[0] + s * px, y, e[1] + s * pz])))));
      rods.push(hull(pts));
    }));
    p.cells.forEach((c, k) => {
      const tc = P(G[k].center), tops = G[k].top.map(P);
      [tc].concat(tops.map(q => { const d = [tc[0] - q[0], tc[1] - q[1]], n = Math.hypot(...d); return [q[0] + d[0] / n * 2, q[1] + d[1] / n * 2]; })).forEach(q => {
        r.inN++; const got = classify(img.at(q[0], q[1]), pal);
        if (got !== i) { r.inBad++; if (r.bad.length < 4) r.bad.push(`${p.name} マス${k} (${q[0].toFixed(0)},${q[1].toFixed(0)}) → ${got < 0 ? "ピースでない" : t.pieces[got].name}`); }
      });
      const poly = polys[k], m = poly.reduce((s, q) => [s[0] + q[0] / poly.length, s[1] + q[1] / poly.length], [0, 0]);
      poly.forEach(q => {
        const d = [q[0] - m[0], q[1] - m[1]], n = Math.hypot(...d), o = [q[0] + d[0] / n * 3, q[1] + d[1] / n * 3];
        if (polys.some((pp, j) => j !== k && inPoly(o, pp)) || rods.some(pp => inPoly(o, pp))) return;
        if (o[0] < 0 || o[0] >= W || o[1] < 0 || o[1] >= H) return;
        r.outN++; if (classify(img.at(o[0], o[1]), pal) === i) r.outBad++;
      });
    });
  });
  return r;
}
const jobs = T.SETS.map(o => ({ name: "set" + o.value, ui: T.uiForSet(o.value) }))
  .concat(["example_3x4_rot30", "example_3x4_mixed"].map(n => ({ name: n, ui: T.uiForLayout(path.join(ROOT, "layouts", n + ".json")) })));
let ng = 0;
const neg = { mirror: [], shift: [], fov: [], yaw: [], half: [], flat: [] };
const sens = {}, add = (k, r) => { sens[k] = sens[k] || [0, 0, 0, 0, true]; sens[k][0] += r.inBad; sens[k][1] += r.inN; sens[k][2] += r.outBad; sens[k][3] += r.outN;
  if (r.inBad === 0 && (r.outN - r.outBad) / r.outN >= 0.95) sens[k][4] = sens[k][4] && true; else sens[k][4] = false; };   // [4]: どの組でも合格のまま
jobs.forEach(job => {
  const t0 = Date.now(), t = T.buildTruth(job.ui), png = shoot(job), img = readPng(png);
  if (img.w !== W || img.h !== H) { console.log(`★NG [${job.name}] 絵が ${img.w}×${img.h}(640×480 でない)`); ng++; return; }
  // 正解の画素を、その高さの面へ戻す
  let back = 0;
  t.pieces.forEach(p => p.cells.forEach(c => c.top_px.forEach((q, j) => {
    const b = t._img.unproject(q[0], q[1], c._top[j][1]); back = Math.max(back, Math.hypot(b[0] - c._top[j][0], b[2] - c._top[j][2]));
  })));
  const r = compare(t, img), outOk = r.outN && (r.outN - r.outBad) / r.outN >= 0.95, ok = r.inBad === 0 && outOk && back < 5e-5;
  if (!ok) ng++;
  console.log(`[${job.name}] ${t.pieces.map(p => p.name).join(" ")} / 内 ${r.inN - r.inBad}/${r.inN} 点がそのピースの色 / 外 ${r.outN - r.outBad}/${r.outN} 点がそのピースの色でない` +
              ` / 画素から面へ戻した差 最大 ${(back * 1000).toFixed(3)}mm  ${ok ? "OK" : "★NG"}${r.bad.length ? " 例: " + r.bad.join("、") : ""}  (${((Date.now() - t0) / 1000).toFixed(0)}s) → ${path.relative(ROOT, png)}`);
  // 負の検算(正解の側だけ壊す)
  neg.mirror.push(compare(t, img, q => { const u = t._img.project(q); return [W - u[0], u[1]]; }));
  neg.shift.push(compare(t, img, q => t._img.project([q[0] + 0.005, q[1], q[2]])));
  // 角だけ yaw の向きを逆にする(マスの真ん中は正しいまま)。回したピースのある組だけで見る
  const anyYaw = t.pieces.some(p => Math.abs(p.pose.yaw % 90) > 1);
  if (anyYaw) neg.yaw.push(compare(t, img, null, (c, p) => {
    const ce = [c.center[0] / 1000, c.center[1] / 1000], h = t.cell_mm / 2000;
    const cor = [[h, h], [h, -h], [-h, -h], [-h, h]].map(o => T.rotY([o[0], 0, o[1]], -p.pose.yaw));
    return { top: cor.map(o => [ce[0] + o[0], t.cell_mm / 1000, ce[1] + o[2]]), bot: cor.map(o => [ce[0] + o[0], 0, ce[1] + o[2]]), center: [ce[0], t.cell_mm / 1000, ce[1]] };
  }));
  // 立方体の辺を 28mm と取り違える(「外」が働いているか。内は真ん中寄りなので合いうる)
  neg.half.push(compare(t, img, null, c => {
    const ce = [c.center[0] / 1000, t.cell_mm / 1000, c.center[1] / 1000], sc = (q, y) => [ce[0] + (q[0] - ce[0]) * 2, y, ce[2] + (q[2] - ce[2]) * 2];
    return { top: c._top.map(q => sc(q, 2 * t.cell_mm / 1000)), bot: c._bot.map(q => sc(q, 0)), center: [ce[0], 2 * t.cell_mm / 1000, ce[2]] };
  }));
  // 上の面を机の高さ 0 と取り違える
  neg.flat.push(compare(t, img, null, c => ({ top: c._top.map(q => [q[0], 0, q[2]]), bot: c._bot, center: [c.center[0] / 1000, 0, c.center[1] / 1000] })));
  // 細かさ(合否には使わない): 机の上で前・右へ ±1〜3mm、絵の上で上下左右へ ±0.5〜2 画素ずらしたときの、内・外で合わない点
  [-3, -2, -1, 1, 2, 3].forEach(mm => ["x", "z"].forEach(ax => {
    const k = (ax === "x" ? "前" : "右") + (mm > 0 ? "+" : "") + mm + "mm";
    add(k, compare(t, img, q => t._img.project(ax === "x" ? [q[0] + mm / 1000, q[1], q[2]] : [q[0], q[1], q[2] + mm / 1000])));
  }));
  [-2, -1.5, -1, -0.5, 0.5, 1, 1.5, 2].forEach(px => [["u", 0], ["v", 1]].forEach(([nm, ax]) => {
    const k = (ax ? (px > 0 ? "下" : "上") : (px > 0 ? "右" : "左")) + Math.abs(px) + "画素";
    add(k, compare(t, img, q => { const u = t._img.project(q); u[ax] += px; return u; }));
  }));
  const img55 = SO101.overheadImage(t._data.overheadCamera, 55, W, H);
  neg.fov.push(compare(t, img, q => img55.project(q)));
});
const NEG_LABEL = { mirror: "左右を裏返す", shift: "ピースを前へ 5mm ずらす", fov: "縦の画角を 55° と取り違える",
                   yaw: "角だけ yaw を逆に回す(回したピースのある組だけ)", half: "立方体の辺を 28mm と取り違える", flat: "上の面を机の高さ 0 と取り違える" };
// 合否: 内か外のどちらかで、合格の条件(内 0 点・外 95% 以上)を外れること。どの組でも
Object.keys(neg).forEach(k => {
  const fails = neg[k].map(r => r.inBad > 0 || (r.outN - r.outBad) / r.outN < 0.95);
  const hit = neg[k].length > 0 && fails.every(Boolean), info = k === "flat";
  console.log(`  ${info ? "(目安・合否に使わない)" : "負の検算"}(${NEG_LABEL[k]}): 組ごとに 内で合わない ${neg[k].map(r => r.inBad + "/" + r.inN).join("・")}、外でそのピースの色 ${neg[k].map(r => r.outBad + "/" + r.outN).join("・")}  ` +
              (info ? `合格の条件を外れた組 ${fails.filter(Boolean).length}/${fails.length}(高さの取り違えは、上の面のすぐ下に同じ色の横の面が写るので、色の比べでは見分けきれない)`
                    : hit ? "OK(どの組でも合格の条件を外れた)" : "★NG(外れない組がある = 検算が鈍い)"));
  if (!hit && !info) ng++;
});
console.log(`  (目安)正解をずらしたときの、内で合わない点 / 外でそのピースの色の点(* はどの組でも合格の条件を外れない = 見分けられない。内 ${sens[Object.keys(sens)[0]][1]} 点・外 ${sens[Object.keys(sens)[0]][3]} 点):`);
console.log("    " + Object.keys(sens).map(k => k + " " + sens[k][0] + "/" + sens[k][2] + (sens[k][4] ? "*" : "")).join("・"));
console.log(ng ? `★NG ${ng} 件` : `OK(${jobs.length} 組とも、正解の画素に絵のピースの色があり、角の外に無い。負の検算 ${Object.keys(neg).length - 1} つは、どの組でも合格の条件を外れた)`);
process.exitCode = ng ? 1 : 0;
