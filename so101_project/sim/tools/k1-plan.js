// K1 (3)(4) の段取り表を作る(計画書_完成形まで の K1 と「安全に流す道具の決まり」)。実機は動かさない。
//   盤と置き場の範囲に格子の点を置き、道具の点(爪の先)を各点の真上「机 + 高さ」に、爪を真下へ向けて行かせる関節角を sim で解き、
//   sim/calib/k1_<id>.json の換算(sim/robot-map.js)で LeRobot の角度に直す。tools\k1_run.py がこの表を流す。
//   道のり: 基準の姿勢(tools\k1_calib.py と同じ REF_URDF)→ 点 1 → … → 点 N → 基準の姿勢。点と点の間は、LeRobot の角度を
//     直線で結び、STEP_DEG おきに刻んで送る(k1_run.py が同じ刻みで送る。モーターごとの速さの違いで道が曲がらないように)。
//   流す前に止める(1 つでもあれば表を書かない):
//     (a) 換算して戻した角度が元と合わない (b) 刻んだ全部の姿勢で、腕(土台の外)と爪の点のいちばん低い所が 机 + TABLE_MARGIN より下
//     (c) キャリブレーションの範囲(余裕 5°)の外 (d) 腕どうしの当たり(爪をいちばん閉じた・開いた の両方で。self-collision.js)
//     (e) --stand のとき: 台(C の柱・足の金具・吊る腕・吊る板・カメラ。2026-11-19 から hardware/C_台の設計.md の値で本家の板なし。scene-data.js の standClearance)まで STAND_MARGIN 未満。
//         腕の原点は合板から 2.4mm(C の台は本家の板なし。2026-11-19 まで板ありで 9.6mm、机の余裕に板の厚さ 7.2mm を足していた)
//   見ていないもの: 台なしでは上のカメラの台・机の上の物・ケーブル。換算そのものの誤り(符号・0 のずれ)は、この表では
//     分からない(換算で行って換算で戻るため)。だから最初は高さ +50mm(--lift 50)で、空中で流す。
//   実行: node tools/k1-plan.js --k1 calib/k1_follower1.json [--lift 80] [--yaw 0] [--stand] [--out out/k1-plan.json]
//     高さ(--lift、既定 80mm): 最初は 80mm で流す(点検 C33 の 2: 基準の姿勢の取り方で 0 が ±10° ずれると、50mm では爪の余りが 1〜6mm)。
//       爪を真下へ向けたまま格子の 11 点へ届くのは 80mm まで(90mm は一部、100mm より上は届かない。2026-11-18 に sim の IK で確かめた)。
//     tools\k1_run.py は 50mm 未満の表を流さない
//     2 つ目の高さ(--lift2、例 50): 各点で、同じ真上の低い所へも下りる(点の名前に L)。1 つの高さの紙の測りでは、W3 の手首の確かめの後に
//       残るずれ(肩 + 肘で決まる)が決まらない。2 つの高さで分かるのは「肩 + 肘」と残るずれで、関節ごとには強く相関して当てにならない
//       (tools/k1-err.js。2026-11-18、点検 C41)。最初の 80mm の理由(0 が ±10° ずれると 50mm では爪の余りが 1〜6mm)があるので、
//       tools\k1_run.py は、同じ k1 で 80mm だけの表を流し、どの点も狙った所から 10mm 以内だった記録が無いと、2 つ目の高さのある表を流さない
//         node tools/k1-plan.js --selftest   … 作り物の換算で、止める仕組み (a)〜(d) が効くかの検算
const { load } = require("./_load");
const fs = require("fs"), path = require("path");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js", "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js",
     "collide.js", "chikuwa-collision.js", "self-collision.js", "robot-map.js");
global.window = { SCENE_SKIP_DEFAULT: 1 }; load("scene-data.js"); delete global.window;
const M = SCENE_MODEL;
const argv = process.argv.slice(2), arg = k => { const i = argv.indexOf("--" + k); return i >= 0 ? argv[i + 1] : undefined; }, has = k => argv.includes("--" + k);
const REF_URDF = [0.0, -13.97, 16.18, 83.19, 0.0];       // tools\k1_calib.py の REF_URDF と同じ(上腕は真上・前腕は水平・爪は真下)
const STEP_DEG = 2;                                        // 刻み[度](k1_run.py の STEP_DEG と同じ)
const TABLE_MARGIN = 0.015;                                // 机 + これ より下の点があれば止める
const STAND_MARGIN = 0.010;                                // --stand: 台(C の柱・吊る腕・吊る板・カメラ・足の金具)まで これ 未満なら止める【仮】
const PAN_X = SO101.joints[0].origin[0];
const D = Math.PI / 180;

// 格子: 肩の回転軸から 130・170・210mm、向き −40・0・40°(9 点)と、150mm・±20°(2 点)。置き場と盤の輪(85〜235mm)の中
function gridPoints() {
  const pts = [];
  for (const r of [0.13, 0.17, 0.21]) for (const a of [-40, 0, 40]) pts.push({ r, a });
  pts.push({ r: 0.15, a: -20 }, { r: 0.15, a: 20 });
  return pts.map((p, i) => ({ name: "P" + (i + 1), x: PAN_X + p.r * Math.cos(p.a * D), z: p.r * Math.sin(p.a * D) }));
}
function armLow(q, jaw, ROBOT_Y) {                                  // 腕(土台の外)と爪の点のいちばん低い所(机 = 0)
  const st = KIN.fk(SO101.axes, q, SO101.tool);
  let low = Infinity, at = "", tip = Infinity;
  Object.keys(M.ARM.points).forEach(g => {
    if (g === "base") return;
    const fr = st.frames[(g === "jaw" || g === "cam") ? 4 : (+g.slice(1) - 1)], P = M.ARM.points[g];
    for (const p0 of P) {
      const p = g === "jaw" ? M.jawRotate(p0, jaw) : p0, w = KIN.add(fr.p, KIN.matVec(fr.R, KIN.sub(p, fr.o)));
      if (w[1] + ROBOT_Y < low) { low = w[1] + ROBOT_Y; at = g; }
      if ((g === "j5" || g === "jaw") && w[1] + ROBOT_Y < tip) tip = w[1] + ROBOT_Y;
    }
  });
  return { low, at, tip };
}

function plan(k1, opt) {
  if (k1.verified !== true) return { stops: ["k1 の換算が確かめの姿勢で確かめられていない(verified が無い。tools\\k1_calib.py をやり直す)"], out: null };
  const grip = M.gripFor(opt.cell || M.CELL_DEFAULT / 1000);
  const map = ROBOT_MAP.make(k1, grip);
  const jawMin = grip.angleFor(0), jawMax = grip.angleFor(grip.gapMax);
  const lift = opt.lift / 1000, stops = [];
  // 腕の原点の高さ: 台なし(机に直に留める)は机から 2.4mm、--stand(C の台。hardware/C_台の設計.md)も 2.4mm(2026-11-19 から本家の板なし。それまで 9.6mm)
  const ROBOT_Y = M.robotY(!!opt.stand), floor = TABLE_MARGIN + (opt.stand ? SO101.OVERHEAD_C.plateMm / 1000 : 0);   // C の台は板なし(plateMm 0。2026-11-19 まで本家の板 7.2mm を足していた)
  let standMin = Infinity, standPair = "", standWhere = "";
  // 道の要所(URDF)
  const way = [{ name: "基準", q: REF_URDF.slice() }];
  let seed = REF_URDF.slice();
  for (const p of gridPoints()) {
    const r = MOTION.topDown([p.x, lift - ROBOT_Y, p.z], opt.yaw, seed, {});
    if (!r.ok) { stops.push(`${p.name}: 逆運動学で解けない(${r.reason})`); continue; }
    way.push({ name: p.name, q: r.q.slice(), target: [p.x, lift, p.z] }); seed = r.q;
    if (opt.lift2 != null) {                                       // 同じ真上の 2 つ目の高さ
      const r2 = MOTION.topDown([p.x, opt.lift2 / 1000 - ROBOT_Y, p.z], opt.yaw, seed, {});
      if (!r2.ok) { stops.push(`${p.name}L: 逆運動学で解けない(${r2.reason})`); continue; }
      way.push({ name: p.name + "L", q: r2.q.slice(), target: [p.x, opt.lift2 / 1000, p.z] }); seed = r2.q;
    }
  }
  way.push({ name: "基準", q: REF_URDF.slice() });
  // (a) 換算して戻す
  way.forEach(w => {
    w.L = map.toLeRobot(w.q);
    const back = map.toUrdf(w.L).q, e = Math.max(...back.map((v, i) => Math.abs(v - w.q[i])));
    if (e > 1e-6) stops.push(`${w.name}: 換算して戻すと ${e.toFixed(6)}° 違う`);
  });
  // 刻んだ姿勢(LeRobot の角度を直線で。URDF でも直線)
  const frames = [];
  let low = { low: Infinity, at: "", where: "" }, tipLow = Infinity;   // tipLow = 爪(j5・動く爪)のいちばん低い所(点検 C33 の 5。low はいつも肩の部品になる)
  for (let i = 0; i + 1 < way.length; i++) {
    const A = way[i].L, B = way[i + 1].L, js = map.joints;
    const n = Math.max(1, Math.ceil(Math.max(...js.map(m => Math.abs(B[m] - A[m]))) / STEP_DEG));
    for (let k = 0; k <= n; k++) {
      if (i > 0 && k === 0) continue;
      const L = {}; js.forEach(m => { L[m] = A[m] + (B[m] - A[m]) * k / n; });
      const q = map.toUrdf(L).q;
      const out = map.outOfRange(L);                                                   // (c)
      if (out.length) stops.push(`${way[i].name} → ${way[i + 1].name}: キャリブレーションの範囲の外 ${out.map(m => m + " " + L[m].toFixed(1)).join("・")}`);
      for (const jaw of [jawMin, jawMax]) {                                            // (b)
        const a = armLow(q, jaw, ROBOT_Y);
        if (a.low < low.low) low = { low: a.low, at: a.at, where: `${way[i].name} → ${way[i + 1].name}` };
        if (a.tip < tipLow) tipLow = a.tip;
        if (opt.stand) {                                                               // (e) 台(scene-data.js の standClearance。C の台)
          const r = M.standClearance(q, jaw, opt.standShift);   // 何も出ない(15mm より遠い)ときは Infinity のまま
          Object.keys(r.pairs).forEach(k => { if (r.pairs[k] < standMin) { standMin = r.pairs[k]; standPair = k; standWhere = `${way[i].name} → ${way[i + 1].name}`; } });
        }
      }
      frames.push({ q, t: frames.length, L });
    }
  }
  if (low.low < floor) stops.push(`${low.where}: ${low.at} が机から ${(low.low * 1000).toFixed(1)}mm(机 + ${(floor * 1000).toFixed(1)}mm より下)`);
  if (opt.stand && standMin < STAND_MARGIN) stops.push(`${standWhere}: 台まで ${(standMin * 1000).toFixed(1)}mm(${standPair}。${STAND_MARGIN * 1000}mm より近い)`);
  let selfMin = Infinity, selfPair = "";
  for (const jaw of [jawMin, jawMax]) {                                                // (d)
    const r = SELF_COLLIDE.motionCheck(frames.map(f => ({ q: f.q, jaw, t: f.t })));
    if (r.min < selfMin) { selfMin = r.min; selfPair = r.pair; }
    if (r.min < 0 || r.worstBound < 0) stops.push(`腕どうしが当たりうる(${r.pair} ${(r.min * 1000).toFixed(1)}mm・下限 ${(r.worstBound * 1000).toFixed(1)}mm)`);
  }
  return {
    stops: [...new Set(stops)],
    out: {
      what: "K1 の段取り表(sim/tools/k1-plan.js)", robot_id: k1.robot_id, calibKey: map.calibKey, k1_date: k1.date,
      lift_mm: opt.lift, lift2_mm: opt.lift2 ?? null, yaw_deg: opt.yaw, step_deg: STEP_DEG, robotY_mm: ROBOT_Y * 1000, table_margin_mm: floor * 1000,
      stand: !!opt.stand, stand_min_mm: opt.stand ? (isFinite(standMin) ? standMin * 1000 : null) : null, stand_pair: opt.stand ? standPair : null,
      lowest_mm: low.low * 1000, lowest_at: low.at, tip_lowest_mm: tipLow * 1000, k1_sha256: opt.k1sha || null, self_min_mm: selfMin * 1000, self_pair: selfPair, nFrames: frames.length,
      ref_urdf: REF_URDF,
      waypoints: way.map(w => ({ name: w.name, target_mm: w.target ? w.target.map(v => +(v * 1000).toFixed(2)) : null,
                                 q_urdf: w.q.map(v => +v.toFixed(4)), lerobot: Object.fromEntries(map.joints.map(m => [m, +w.L[m].toFixed(4)])) })),
    },
  };
}

// 作り物の換算(検算用): 符号とずれをでたらめに、キャリブレーションの範囲は広め(±150°)
function fakeK1(opt) {
  const cal = {}, sign = {}, offset = {};
  ROBOT_MAP.JOINTS.concat(["gripper"]).forEach((m, i) => { cal[m] = { id: i + 1, drive_mode: 0, homing_offset: 0, range_min: 2048 - (opt.half || 1700), range_max: 2048 + (opt.half || 1700) }; });
  ROBOT_MAP.JOINTS.forEach((m, i) => { sign[m] = i % 2 ? -1 : 1; offset[m] = (i - 2) * 7.5; });
  return { robot_id: "fake", date: "-", verified: true, calibration: cal, sign, offset, gripper: [{ lerobot: 2, gap_mm: 0 }, { lerobot: 60, gap_mm: 30 }] };
}

function cli() {
if (has("selftest")) {
  let bad = 0;
  const ok = plan(fakeK1({}), { lift: 50, yaw: 0 });
  console.log(`ふつう(高さ 50mm): 止める理由 ${ok.stops.length}・いちばん低い ${ok.out.lowest_mm.toFixed(1)}mm(${ok.out.lowest_at})・腕どうし ${ok.out.self_min_mm.toFixed(1)}mm・${ok.out.nFrames} 姿勢`);
  if (ok.stops.length) { bad++; ok.stops.forEach(s => console.log("  ★" + s)); }
  const st = plan(fakeK1({}), { lift: 50, yaw: 0, stand: true });
  console.log(`C の台あり(--stand): 止める理由 ${st.stops.length}・台まで ${st.out.stand_min_mm != null ? st.out.stand_min_mm.toFixed(1) + "mm(" + st.out.stand_pair + ")" : "15mm より遠い"}・いちばん低い ${st.out.lowest_mm.toFixed(1)}mm`);
  if (st.stops.length) { bad++; st.stops.forEach(s => console.log("  ★" + s)); }
  const two = plan(fakeK1({}), { lift: 80, lift2: 50, yaw: 0 });
  console.log(`2 つの高さ(80・50mm): 止める理由 ${two.stops.length}・点 ${two.out ? two.out.waypoints.length - 2 : "-"}・爪のいちばん低い ${two.out ? two.out.tip_lowest_mm.toFixed(1) : "-"}mm`);
  if (two.stops.length || two.out.waypoints.length - 2 !== 22) { bad++; two.stops.forEach(s => console.log("  ★" + s)); }
  const cases = [
    ["2 つ目の高さが机より下(高さ 5mm)", plan(fakeK1({}), { lift: 80, lift2: 5, yaw: 0 }), /机から/],
    ["C の台あり・柱を格子の真ん中へずらす([-150, -50]mm)", plan(fakeK1({}), { lift: 50, yaw: 0, stand: true, standShift: [-150, -50] }), /台まで/],
    ["机より下へ(高さ 5mm)", plan(fakeK1({}), { lift: 5, yaw: 0 }), /机から/],
    ["キャリブレーションの範囲が狭い(±30°)", plan(fakeK1({ half: 340 }), { lift: 50, yaw: 0 }), /範囲の外/],
  ];
  // (a) 換算の戻しが壊れたら止まるか: toUrdf を 0.01° ずらした換算で作り直す
  const k = fakeK1({}), mk = ROBOT_MAP.make;
  ROBOT_MAP.make = (a, b, c) => { const m = mk(a, b, c), t = m.toUrdf; m.toUrdf = L => { const r = t(L); r.q[2] += 0.01; return r; }; return m; };
  cases.push(["換算の戻しが 0.01° ずれる", plan(k, { lift: 50, yaw: 0 }), /換算して戻す/]);
  ROBOT_MAP.make = mk;
  for (const [label, r, re] of cases) {
    const hit = r.stops.some(s => re.test(s));
    if (!hit) bad++;
    console.log(`${label}: ${hit ? "OK(止まる)" : "★NG(止まらない)"} — ${r.stops[0] || "理由なし"}`);
  }
  console.log(bad ? `NG ${bad}` : "OK");
  process.exit(bad ? 1 : 0);
}

const k1file = arg("k1");
if (!k1file) { console.log("使い方: node tools/k1-plan.js --k1 calib/k1_follower1.json [--lift 80] [--lift2 50] [--yaw 0] [--stand]  /  --selftest"); process.exit(2); }
const k1buf = fs.readFileSync(path.resolve(k1file)), k1 = JSON.parse(k1buf.toString("utf8"));
const k1sha = require("crypto").createHash("sha256").update(k1buf).digest("hex");   // tools\k1_run.py が、いまの k1 のファイルと比べる
const res = plan(k1, { k1sha, lift: +(arg("lift") ?? 80), lift2: arg("lift2") != null ? +arg("lift2") : null, yaw: +(arg("yaw") ?? 0), stand: has("stand") });
if (!res.out) { console.log("★流さない: " + res.stops.join(" / ")); process.exit(1); }
console.log(`点 ${res.out.waypoints.length - 2}・刻んだ姿勢 ${res.out.nFrames}・いちばん低い ${res.out.lowest_mm.toFixed(1)}mm(${res.out.lowest_at})・爪のいちばん低い ${res.out.tip_lowest_mm.toFixed(1)}mm・腕どうし ${res.out.self_min_mm.toFixed(1)}mm` +
            (res.out.stand ? `・台まで ${res.out.stand_min_mm != null ? res.out.stand_min_mm.toFixed(1) + "mm(" + res.out.stand_pair + ")" : "15mm より遠い"}` : "・台なし"));
if (res.out.lift2_mm != null) console.log(`2 つ目の高さ ${res.out.lift2_mm}mm: k1_run.py は、同じ k1 で 80mm だけの表を先に流し、どの点も 10mm 以内だった記録が無いと流さない`);
if (res.stops.length) { console.log("★流さない(表を書かない):"); res.stops.forEach(s => console.log("  " + s)); process.exit(1); }
const file = path.resolve(arg("out") ?? path.join(__dirname, "..", "out", "k1-plan.json"));
fs.mkdirSync(path.dirname(file), { recursive: true });
fs.writeFileSync(file, JSON.stringify(res.out, null, 1));
console.log("書いた: " + path.relative(process.cwd(), file));
}
if (require.main === module) cli();
else module.exports = { plan, fakeK1, gridPoints, REF_URDF };   // tools/k1-err.js が読む(読み込みはこのファイルで済む)
