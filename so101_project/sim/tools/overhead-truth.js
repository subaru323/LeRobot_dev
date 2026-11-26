// 上のカメラの絵の中で、ピースのマスの角が何画素目に写るかを、正解として書き出す(計画書_完成形まで の S-9)。
//   使い道: ピースを見つける道具(K6)を、実物の前に、画面で撮った同じ絵(index.html?_cam=top)で試し、この正解と比べる。
//   見ている世界: scene-data.js の 4 種(既定のすき間・台あり)か、置き場のファイル(--layout)。最初のコマ(写す姿勢)で、置き場に置いたピース。
//     上のカメラは so101.js の OVERHEAD の置き方【仮】・絵の上の向き【仮】(4 通りに付けられる)、640×480、縦の画角は --fov(既定 60°。品の値は分からない)。
//     式は so101.js の overheadImage(画面の three.js のカメラと同じ向き)。
//   書くもの(sim/out/overhead-truth/<名前>.json): 画素の数え方・角の並び・写す姿勢、ピースごとに、名前・色・置き場(前 x・右 z・向き yaw)、マスごとに
//     上の面(高さ = 辺)の真ん中と 4 つの角の画素、底(高さ 0)の 4 つの角の画素と、立方体自身に隠れないか、立方体の外形(8 つの角の凸包)、絵に入るか。
//     座標は机 = 0・mm。画素は左上の角が 0(画素 i の真ん中は i + 0.5。OpenCV は真ん中を整数にするので 0.5 引く)・u が右・v が下。
//   止める: 置き場が使えない・解けない(scene-data.js の layoutWhy・solved)・ファイルの name の順がパズルのピースと違う・すき間が画面の範囲(0〜30mm)の外。
//   見ていないもの: レンズのゆがみ・ピントのぼけ・光と影・光の中心(レンズの先とした)・光軸が絵の真ん中を通るか・腕やほかのピースに隠れるか
//     (こちらで決めた置き場の写す姿勢では腕に隠れない。overhead-occlusion.js)・ピースをつなぐ細い棒・絵の上の向きのほかの 3 通り。
//   実行: node tools/overhead-truth.js(4 種)/ node tools/overhead-truth.js --layout layouts/example_3x4_rot30.json / --fov 70
//   画面で撮った絵と比べる検算は tools/check-overhead-truth.js。
const fs = require("fs"), path = require("path");
const { load, ROOT } = require("./_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
     "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js");
const M = SCENE_MODEL, W = 640, H = 480;

function rotY(v, deg) {                    // scene-data.js の rotY と同じ(上から見て X → Z 向きに回す)
  const c = Math.cos(deg * Math.PI / 180), s = Math.sin(deg * Math.PI / 180);
  return [v[0] * c - v[2] * s, v[1], v[0] * s + v[2] * c];
}
// ui: buildFromUi に渡す値。opts.fov: 縦の画角。opts.cam: 上のカメラを差し替える(負の検算用)
function buildTruth(ui, opts) {
  opts = opts || {};
  const fov = opts.fov || SO101.CAM_FOV.top, d = M.buildFromUi(ui), c = d._check;
  if (!d.overheadCamera) throw new Error("上のカメラの台が無い(STAND=0)");
  if (c.layoutWhy || !c.solved) throw new Error("置き場が使えない・解けない: " + (c.layoutWhy || c.why));   // 点検 C13 の 4
  if (ui.LAYOUT) ui.LAYOUT.pieces.forEach((e, i) => {
    const nm = d.legend[i] && d.legend[i].label.replace(/^ピース /, "");
    if (e.name != null && e.name !== nm) throw new Error(`置き場のファイルの ${i} 番は ${e.name} だが、パズルの ${i} 番は ${nm}(ファイルは並びの番号で読まれる)`);
  });
  const cam = opts.cam || d.overheadCamera, img = SO101.overheadImage(cam, fov, W, H);
  const h = c.cell / 2, P = (q) => { const p = img.project(q); return p && [+p[0].toFixed(2), +p[1].toFixed(2)]; };
  const inside = q => !!q && q[0] >= 0 && q[0] < W && q[1] >= 0 && q[1] < H;
  const pieces = c.pieceCells.map((cells, i) => {
    const yaw = c.slotRot[i], lg = d.legend[i];
    return {
      name: lg.label.replace(/^ピース /, ""), color: lg.color,
      pose: { x: +(c.slots[i][0] * 1000).toFixed(2), z: +(c.slots[i][1] * 1000).toFixed(2), yaw: +yaw.toFixed(3) },
      cells: cells.map(xz => {
        const corners = [[h, h], [h, -h], [-h, -h], [-h, h]].map(o => rotY([o[0], 0, o[1]], yaw));
        const top = corners.map(o => [xz[0] + o[0], c.cell, xz[1] + o[2]]), bot = corners.map(o => [xz[0] + o[0], 0, xz[1] + o[2]]);
        const tc = P([xz[0], c.cell, xz[1]]), tp = top.map(P), bp = bot.map(P);
        // 底の角が立方体自身に隠れないか: その角に接する 2 つの横の面のどちらかがカメラを向いていれば見える
        const faceSees = (n, q) => { const w = rotY(n, yaw); return w[0] * (cam.tip[0] - q[0]) + w[2] * (cam.tip[2] - q[2]) > 0; };
        const LOC = [[1, 1], [1, -1], [-1, -1], [-1, 1]];
        const bvis = bot.map((q, j) => faceSees([LOC[j][0], 0, 0], q) || faceSees([0, 0, LOC[j][1]], q));
        return { center: [+(xz[0] * 1000).toFixed(2), +(xz[1] * 1000).toFixed(2)], top_center_px: tc, top_px: tp, bottom_px: bp,
                 bottom_visible: bvis, silhouette_px: hull(tp.concat(bp)),
                 in_image: inside(tc) && tp.every(inside) && bp.every(inside),
                 _top: top, _bot: bot };
      })
    };
  });
  return { image: { width: W, height: H, fov_vertical_deg: fov, note: "ピンホール・ゆがみなし・光軸は絵の真ん中・光の中心はレンズの先。上のカメラの置き方と絵の上の向き(4 通りのうち既定の 1 つ)は【仮】",
                    pixel_convention: "画素の左上の角が (0,0)、u は右・v は下。画素 i の真ん中は i + 0.5(OpenCV の画素の真ん中を整数とする数え方では、両方から 0.5 引く)" },
           corner_order: "top_px・bottom_px の角は、ピースの局所の (+x,+z)・(+x,−z)・(−x,−z)・(−x,+z) の順(x は腕の前、z は腕の右を、yaw だけ回した向き)。絵の上の並びは yaw で回る",
           frame: { t: 0, photo_q_deg: c.photoQ, note: "最初のコマ(写す姿勢)。置き場に置いたピース" },
           camera: { tip_mm: cam.tip.map(v => +(v * 1000).toFixed(2)), dir: cam.dir, up: cam.up },
           cell_mm: +(c.cell * 1000).toFixed(2), gap_mm: +ui.GAP, pieces, _img: img, _data: d };
}
function hull(pts) {                        // 凸包(絵の上の多角形)
  const p = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]), cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  p.forEach(q => { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], q) <= 0) lo.pop(); lo.push(q); });
  p.slice().reverse().forEach(q => { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], q) <= 0) up.pop(); up.push(q); });
  return lo.slice(0, -1).concat(up.slice(0, -1));
}
function strip(t) {                         // ファイルに書かない物(_ で始まる)を外す
  return JSON.parse(JSON.stringify(t, (k, v) => k.startsWith("_") ? undefined : v));
}
const SETS = M.controls.find(x => x.key === "SET").options, DEF = M.defaultUiValues();
function uiForSet(v) { return { SET: v, GAP: DEF.GAP, SOL: 0 }; }
function uiForLayout(file) {
  const L = JSON.parse(fs.readFileSync(path.resolve(file), "utf8"));
  const gc = M.controls.find(x => x.key === "GAP"), gap = L.gap ?? DEF.GAP;
  if (!(gap >= gc.min && gap <= gc.max)) throw new Error(`すき間 ${gap}mm は画面の範囲(${gc.min}〜${gc.max}mm)の外(画面は範囲に収めるので、絵と正解が食い違う)`);   // 点検 C13 の 8
  return { SET: +(L.set ?? DEF.SET), GAP: gap, SOL: 0, LAYOUT: L };   // check-layout.js と同じ読み方
}
module.exports = { buildTruth, strip, SETS, uiForSet, uiForLayout, W, H, rotY, hull };

if (require.main === module) {
  const a = process.argv.slice(2), arg = k => { const i = a.indexOf(k); return i >= 0 ? a[i + 1] : null; };
  const fov = parseFloat(arg("--fov")) || SO101.CAM_FOV.top, outDir = path.join(ROOT, "out", "overhead-truth");
  fs.mkdirSync(outDir, { recursive: true });
  const jobs = arg("--layout") ? [{ name: path.basename(arg("--layout"), ".json"), ui: uiForLayout(arg("--layout")) }]
                               : SETS.map(o => ({ name: "set" + o.value, ui: uiForSet(o.value) }));
  jobs.forEach(j => {
    const t = buildTruth(j.ui, { fov }), out = path.join(outDir, j.name + (fov !== SO101.CAM_FOV.top ? "_fov" + fov : "") + ".json");
    fs.writeFileSync(out, JSON.stringify(strip(t), null, 1));
    const nIn = t.pieces.reduce((s, p) => s + p.cells.filter(x => x.in_image).length, 0), nAll = t.pieces.reduce((s, p) => s + p.cells.length, 0);
    const sz = t.pieces.flatMap(p => p.cells.map(x => Math.hypot(x.top_px[0][0] - x.top_px[2][0], x.top_px[0][1] - x.top_px[2][1]) / Math.SQRT2));
    console.log(`[${j.name}] ${t.pieces.map(p => p.name).join(" ")} / マス ${nIn}/${nAll} が絵に入る / 上の面の 1 辺 ${Math.min(...sz).toFixed(1)}〜${Math.max(...sz).toFixed(1)} 画素 → ${path.relative(ROOT, out)}`);
  });
}
