// K1 の記録(tools\k1_run.py が書く sim/calib/k1_run_<日時>.json)を読み、W3 で手首の確かめの後に残る爪のずれ[mm]と、関節の 0 のずれ[度]を
// 出す(計画書_完成形まで の K1 (3)(4)、進め方_利用者の稼働を絞る の A-6。2026-11-18、点検 C41 の後)。
//   模型: 本当の腕 = 読んだ角度(lerobot_present を k1 の換算で URDF に)+ 関節ごとの 0 のずれ b。紙で測った爪の先 =(1 + 縮尺のずれ)×
//     本当の腕の道具の点(x, z)+ 紙の置き方のずれ(dx, dz)。b(shoulder_pan・shoulder_lift・elbow_flex・wrist_flex)・dx・dz・縮尺を最小二乗で解く。
//     wrist_roll は、爪の先が回転の軸の近くにあり紙では見えないので解かない。紙を回して置いたずれは shoulder_pan に入る。
//   決まるもの・決まらないもの: 1 つの高さだけでは、肩・肘・手首の曲げのずれも、W3 で残るずれも決まらない(高さ 80mm の 11 点では shoulder_lift の
//     標準偏差が 1mm の測りの誤差あたり 2000° を超える)。k1-plan.js --lift2 50 の 2 つの高さでも、関節ごとには強く相関して当てにならない
//     (1mm・1 回で 1〜2°)。よく決まるのは「肩 + 肘」と W3 で残るずれ。
//   W3 で残るずれ: 手首のカメラは高さ HOVER_MM で撮り、爪は PICK_MM まで下りてつかむ。撮るときのカメラの本当の姿勢と思っている姿勢の
//     違いが「ピースの見え方」をずらし(斜めの筋を平面と交わらせる。w3Residual)、下りた所の爪のずれがそれに足される。格子の 11 点 ×
//     爪の向き YAWS ごとに、爪の向きに沿う e_a と横の e_c に分け、plan-cli.js の score と同じ上限(e_a は FIT_MARGIN、e_c は EC_MAX_MM)と比べる。
//     手首で測ったずれが FIX_TOL_MM 以下なら puzzle_run.py は直さないので、その分(最大 0.5mm)を上に足す。
//   判定の前提: 手首のカメラの取り付けは URDF の値(W4 の前)。測りの誤差はでたらめな物だけを見る。爪の先が紙から浮いているのを斜めから
//     読む誤差(視差)は高さに比例し、肩と肘のずれと区別できない(2 つの高さの差が肩と肘の手がかりなので)。測るときは直角定規か下げ振りで
//     爪の先を紙へ真下に落とす。紙の縮尺は解くが、紙の 150mm の定規 2 本(x と z)を鋼の定規で測っておく。
//   見ていないもの: 腕のたわみ(0 のずれと区別しない)、手首のカメラの取り付けの誤差(W4。--selftest で感度だけ出す)
//   実行: node tools/k1-err.js --run calib/k1_run_<日時>.json [--plan out/k1-plan.json] [--k1 calib/k1_<id>.json] [--sigma 1]
//         node tools/k1-err.js --selftest   … 作り物の記録(ずれを知っている)で、種を変えて誤った OK・NG が出ないか、1 つの高さでは決まらないと言うか
const kp = require("./k1-plan.js");      // 読み込み(kinematics.js・motion.js・scene-data.js・robot-map.js など)はここで済む
const fs = require("fs"), path = require("path");
const M = SCENE_MODEL;
const argv = process.argv.slice(2), arg = k => { const i = argv.indexOf("--" + k); return i >= 0 ? argv[i + 1] : undefined; }, has = k => argv.includes("--" + k);
const NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"];   // 解く関節(URDF の 0〜3)
const NP = NAMES.length + 3;             // + dx・dz・縮尺(千分率)
const HOVER_MM = 50;                     // scene-data.js の HOVER_Y(つかむ場所の真上。手首のカメラで確かめる高さ)
const PICK_MM = M.GRASP_Y * 1000;        // つかむときの道具の点の高さ(scene-data.js の GRASP_Y)
const YAWS = [0, 45, 90, 135];           // 爪の向き[度](180° 回すと同じ)
const EC_MAX_MM = 2.0;                   // plan-cli.js の score の EC_MAX_MM【仮】
const EA_MAX_MM = 2.0;                   // scene-data.js の FIT_MARGIN_DEFAULT(固定の爪がピースに乗らない)
const FIX_TOL_MM = 0.5;                  // tools\puzzle_run.py の FIX_TOL_MM(これ以下なら直さない)
const SIGMA_FLOOR = 0.3;                 // 測りの誤差の下限[mm](残りから出した値がこれより小さくても、これを使う。たまたま小さく出て言い過ぎないため)
const SV_MIN = 0.05;                     // これより小さい特異値の向きは「決まらない」
const JOINT_SD_CMD = 0.3;                // 関節ごとの標準偏差がみなこれ以下のときだけ、puzzle_run.py --arm-bias の行を出す

const tipXZ = q => { const p = KIN.fk(SO101.axes, q, SO101.tool).pos; return [p[0] * 1000, p[2] * 1000]; };
const addB = (q, b) => q.map((v, i) => v + (i < b.length ? b[i] : 0));
const T9 = R => [0, 3, 6, 1, 4, 7, 2, 5, 8].map(i => R[i]);

// 撮るときの手首の本当の姿勢 qh+b、思っている姿勢 qh。つかむ姿勢 qp。手首で確かめた後に残る爪のずれ[mm](x, z)
//   カメラは光の筋でピースを見て、その筋を平面と交わらせて置き場を出す(vision-wrist.js)。本当のカメラからピース(つかむ所 p)への筋を、
//   カメラの中の向きのまま、思っているカメラの姿勢から引き直し、同じ高さの平面と交わらせた所が「見えた所」。
//   肩の誤りは肩の軸の回りの回転なので、点で見れば下りた爪のずれと打ち消し合うが、筋は斜めなので、手の上下のずれが横のずれに変わって残る
//   (点で見る模型では 0 になり、puzzle_run.py --arm-bias の 1 度で 0.9〜1.5mm と合わなかった)。
//   camShift[mm]: カメラの取り付けの位置を URDF からカメラの軸の向きにずらす(W4 で測って入れた後、という前提。--selftest の感度用)。
//   取り付けの向き(カメラの中心の回りの回転)は、本当と思っているの両方に同じく掛かり、筋は変わらないので効かない
function camAt(q, camShift) {
  const cf = SO101.urdfFK(q.map(x => x * Math.PI / 180), 0).wrist_camera_link;
  if (!camShift) return { p: cf.p, R: cf.R };
  return { p: KIN.add(cf.p, KIN.matVec(cf.R, camShift.map(v => v / 1000))), R: cf.R };
}
function w3Residual(qh, qp, b, camShift) {
  const p = KIN.fk(SO101.axes, qp, SO101.tool).pos, pt = KIN.fk(SO101.axes, addB(qp, b), SO101.tool).pos;   // つかむ所(計画)と本当の爪
  const C = camAt(qh, camShift), Ct = camAt(addB(qh, b), camShift);
  const dLoc = KIN.matVec(T9(Ct.R), KIN.sub(p, Ct.p));                 // カメラの中から見た筋の向き
  const d = KIN.matVec(C.R, dLoc), s = (p[1] - C.p[1]) / d[1];          // 思っているカメラから引き直し、高さ p[1] の平面と交わらせる
  const seen = [C.p[0] + s * d[0], C.p[2] + s * d[2]];
  return [(seen[0] - p[0] + pt[0] - p[0]) * 1000, (seen[1] - p[2] + pt[2] - p[2]) * 1000];
}
// 格子の点 × 爪の向きごとに、W3 で残るずれ(e_a, e_c)の b についての係数(各関節 0.01° の差で)
function w3Coeffs(ROBOT_Y, camShift) {
  const rows = [];
  for (const yaw of YAWS) {
    let seed = kp.REF_URDF.slice();
    for (const p of kp.gridPoints()) {
      const rh = MOTION.topDown([p.x, HOVER_MM / 1000 - ROBOT_Y, p.z], yaw, seed, {});
      if (!rh.ok) { rows.push({ name: p.name, yaw, why: "撮る高さへ解けない" }); continue; }
      seed = rh.q;
      const rp = MOTION.topDown([p.x, PICK_MM / 1000 - ROBOT_Y, p.z], yaw, rh.q, {});
      if (!rp.ok) { rows.push({ name: p.name, yaw, why: "つかむ高さへ解けない" }); continue; }
      const u = [Math.cos(yaw * Math.PI / 180), Math.sin(yaw * Math.PI / 180)];
      const G = NAMES.map((_, k) => { const b = [0, 0, 0, 0]; b[k] = 0.01; const e = w3Residual(rh.q, rp.q, b, camShift); return [e[0] / 0.01, e[1] / 0.01]; });
      const pad = [0, 0, 0];
      rows.push({ name: p.name, yaw, ga: G.map(g => g[0] * u[0] + g[1] * u[1]).concat(pad), gc: G.map(g => -g[0] * u[1] + g[1] * u[0]).concat(pad) });
    }
  }
  return rows;
}

// 小さな行列の道具
const T = A => A[0].map((_, j) => A.map(r => r[j]));
const mul = (A, B) => A.map(r => B[0].map((_, j) => r.reduce((s, v, k) => s + v * B[k][j], 0)));
function eigSym(A0) {                                                   // ヤコビ法(対称)
  const n = A0.length, A = A0.map(r => r.slice()), V = A.map((r, i) => r.map((_, j) => +(i === j)));
  for (let sw = 0; sw < 60; sw++) for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
    if (Math.abs(A[p][q]) < 1e-15) continue;
    const th = (A[q][q] - A[p][p]) / (2 * A[p][q]), t = (th >= 0 ? 1 : -1) / (Math.abs(th) + Math.sqrt(th * th + 1)), c = 1 / Math.sqrt(t * t + 1), s = t * c;
    for (let k = 0; k < n; k++) { const a = A[k][p], b = A[k][q]; A[k][p] = c * a - s * b; A[k][q] = s * a + c * b; }
    for (let k = 0; k < n; k++) { const a = A[p][k], b = A[q][k]; A[p][k] = c * a - s * b; A[q][k] = s * a + c * b; }
    for (let k = 0; k < n; k++) { const a = V[k][p], b = V[k][q]; V[k][p] = c * a - s * b; V[k][q] = s * a + c * b; }
  }
  return { val: A.map((r, i) => r[i]), V };
}

// obs: [{q (URDF, 読んだ角度), meas [x, z] mm}]。x = [b0..b3(度), dx, dz(mm), 縮尺(千分率)]。戻り: x, 共分散(擬似逆), 決まらない向き
function fit(obs, sigmaGiven) {
  let x = new Array(NP).fill(0);
  const model = (o, x) => { const t = tipXZ(addB(o.q, x.slice(0, 4))), k = 1 + x[6] / 1000; return [k * t[0] + x[4], k * t[1] + x[5]]; };
  let J, r, eg, pinv, null_ = [];
  for (let it = 0; it < 4; it++) {                                      // ガウス・ニュートン(ずれは数度なのでほぼ線形)
    J = []; r = [];
    for (const o of obs) {
      const m0 = model(o, x), cols = [];
      for (let k = 0; k < NP; k++) { const xx = x.slice(); xx[k] += 0.01; const m1 = model(o, xx); cols.push([(m1[0] - m0[0]) / 0.01, (m1[1] - m0[1]) / 0.01]); }
      for (const c of [0, 1]) { J.push(cols.map(v => v[c])); r.push(o.meas[c] - m0[c]); }
    }
    const N = mul(T(J), J), g = mul(T(J), r.map(v => [v]));
    eg = eigSym(N);
    pinv = N.map(() => new Array(NP).fill(0)); null_ = [];
    eg.val.forEach((lam, k) => {
      const sv = Math.sqrt(Math.max(lam, 0)), v = eg.V.map(row => row[k]);
      if (sv < SV_MIN) { null_.push(v); return; }
      for (let i = 0; i < NP; i++) for (let j = 0; j < NP; j++) pinv[i][j] += v[i] * v[j] / lam;
    });
    const dx = mul(pinv, g).map(v => v[0]);
    x = x.map((v, i) => v + dx[i]);
  }
  const res = obs.map(o => { const m = model(o, x); return Math.hypot(o.meas[0] - m[0], o.meas[1] - m[1]); });
  const rss = obs.reduce((s, o) => { const m = model(o, x); return s + (o.meas[0] - m[0]) ** 2 + (o.meas[1] - m[1]) ** 2; }, 0);
  const dof = 2 * obs.length - (NP - null_.length);
  const sigmaRes = dof > 0 ? Math.sqrt(rss / dof) : NaN;
  const sigma = Math.max(SIGMA_FLOOR, sigmaGiven || 0, isFinite(sigmaRes) ? sigmaRes : 0);
  return { x, cov: pinv.map(r => r.map(v => v * sigma * sigma)), null_, sigma, sigmaRes, resMax: Math.max(...res) };
}
// 線形の量 g·x の値と標準偏差。決まらない向きに乗っていれば determined = false
function linear(f, g) {
  const v = g.reduce((s, gi, i) => s + gi * f.x[i], 0);
  let s2 = 0; for (let i = 0; i < g.length; i++) for (let j = 0; j < g.length; j++) s2 += g[i] * f.cov[i][j] * g[j];
  const gn = Math.hypot(...g) || 1, leak = Math.max(0, ...f.null_.map(u => Math.abs(u.reduce((s, ui, i) => s + ui * g[i], 0)) / gn));
  return { v, sd: Math.sqrt(s2), determined: leak < 0.02 };
}

const COEF_CACHE = {};
function analyse(rec, plan, map, opt) {
  opt = opt || {};
  const ROBOT_Y = (plan.robotY_mm ?? 2.4) / 1000;
  const obs = [], byName = {}, raw = [];
  for (const p of rec.points) {
    if (!Array.isArray(p.measured_xz_mm) || p.measured_xz_mm.length !== 2) continue;
    const q = map.toUrdf(p.lerobot_present).q, h = p.target_mm[1];
    obs.push({ q, meas: p.measured_xz_mm, name: p.name, h, loop: p.loop });
    raw.push({ name: p.name, loop: p.loop, h, err: [p.measured_xz_mm[0] - p.target_mm[0], p.measured_xz_mm[1] - p.target_mm[2]] });
    (byName[p.name] = byName[p.name] || []).push(p.measured_xz_mm);
  }
  const out = { n: obs.length, heights_mm: [...new Set(obs.map(o => o.h))].sort((a, b) => b - a) };
  if (obs.length < 6) return Object.assign(out, { why: "測った点が 6 つより少ない" });
  // (3) 狙った所との差(紙の置き方のずれも入ったまま)
  out.target_err_max_mm = Math.max(...raw.map(r => Math.hypot(...r.err)));
  // (4) 繰り返しの差: 同じ点の測りのばらつき(2 回以上ある点)。腕そのもののばらつきも入るので、測りの誤差には使わない
  const spreads = Object.values(byName).filter(v => v.length > 1).map(v => {
    const m = [0, 1].map(c => v.reduce((s, a) => s + a[c], 0) / v.length);
    return Math.sqrt(v.reduce((s, a) => s + (a[0] - m[0]) ** 2 + (a[1] - m[1]) ** 2, 0) / (v.length - 1) / 2);
  });
  out.repeat_sd_mm = spreads.length ? Math.sqrt(spreads.reduce((s, v) => s + v * v, 0) / spreads.length) : null;
  // 2 つの高さの差(模型なし): 同じ回の P と PL
  const pairs = [];
  for (const o of obs) if (/L$/.test(o.name)) { const up = obs.find(u => u.name === o.name.slice(0, -1) && u.loop === o.loop); if (up) pairs.push({ name: up.name, d: [o.meas[0] - up.meas[0], o.meas[1] - up.meas[1]], dh: up.h - o.h }); }
  out.down_pairs = pairs.map(p => ({ name: p.name, dh_mm: p.dh, dx_mm: +p.d[0].toFixed(2), dz_mm: +p.d[1].toFixed(2) }));
  // 合わせ
  const f = fit(obs, opt.sigma);
  out.sigma_mm = f.sigma; out.sigma_from_residual_mm = f.sigmaRes; out.residual_max_mm = f.resMax;
  out.joints = NAMES.map((nm, i) => { const e = new Array(NP).fill(0); e[i] = 1; const l = linear(f, e); return { joint: nm, deg: l.determined ? +l.v.toFixed(3) : null, sd: l.determined ? +l.sd.toFixed(3) : null }; });
  out.paper = { dx_mm: +f.x[4].toFixed(2), dz_mm: +f.x[5].toFixed(2), scale_permil: +f.x[6].toFixed(2), note: "紙の置き方のずれと縮尺。紙を回したずれは shoulder_pan に入る" };
  const le = linear(f, [0, 1, 1, 0, 0, 0, 0]);
  out.lift_plus_elbow_deg = le.determined ? { v: +le.v.toFixed(3), sd: +le.sd.toFixed(3) } : null;
  // W3 で残るずれ(格子 × 爪の向き。e_a と e_c)
  const key = ROBOT_Y + "|" + JSON.stringify(opt.camShift || null);
  const coef = COEF_CACHE[key] || (COEF_CACHE[key] = w3Coeffs(ROBOT_Y, opt.camShift));
  const rows = coef.filter(c => !c.why).map(c => {
    const a = linear(f, c.ga), e = linear(f, c.gc);
    return { name: c.name, yaw: c.yaw, ea: a, ec: e, det: a.determined && e.determined,
             // 上の見積もり(直さない場合の FIX_TOL を足す)と下の見積もり
             hiA: Math.abs(a.v) + 2 * a.sd + FIX_TOL_MM, loA: Math.abs(a.v) - 2 * a.sd, hiC: Math.abs(e.v) + 2 * e.sd + FIX_TOL_MM, loC: Math.abs(e.v) - 2 * e.sd };
  });
  out.w3_unreached = coef.filter(c => c.why).map(c => `${c.name}・向き ${c.yaw}°: ${c.why}`);
  const det = rows.length > 0 && rows.every(r => r.det);
  const worst = (k) => rows.reduce((m, r) => (r[k].v !== undefined && Math.abs(r[k].v) > Math.abs(m[k].v) ? r : m), rows[0]);
  out.w3 = { from_mm: HOVER_MM, to_mm: PICK_MM, yaws: YAWS, determined: det };
  if (det) {
    const wc = worst("ec"), wa = worst("ea");
    out.w3.ec = { max_mm: +Math.abs(wc.ec.v).toFixed(2), sd_mm: +wc.ec.sd.toFixed(2), at: `${wc.name}・${wc.yaw}°`, hi_mm: +Math.max(...rows.map(r => r.hiC)).toFixed(2), lo_mm: +Math.max(...rows.map(r => r.loC)).toFixed(2) };
    out.w3.ea = { max_mm: +Math.abs(wa.ea.v).toFixed(2), sd_mm: +wa.ea.sd.toFixed(2), at: `${wa.name}・${wa.yaw}°`, hi_mm: +Math.max(...rows.map(r => r.hiA)).toFixed(2), lo_mm: +Math.max(...rows.map(r => r.loA)).toFixed(2) };
  }
  // A-6 の判定(点と向きごとに): どこでも「値 + 2σ + FIX_TOL」が上限より小さければ OK、どこかで「値 − 2σ」が上限を超えれば NG、その間は「分からない」
  if (!det) out.a6 = { verdict: "決まらない", why: "高さが 1 つだけ(k1-plan.js --lift2 50 の記録が要る)" };
  else if (out.w3.ec.lo_mm > EC_MAX_MM || out.w3.ea.lo_mm > EA_MAX_MM)
    out.a6 = { verdict: "NG", why: `残るずれが −2σ でも上限を超える所がある(e_c ${out.w3.ec.lo_mm}mm・e_a ${out.w3.ea.lo_mm}mm。上限 ${EC_MAX_MM}・${EA_MAX_MM}mm)。換算の 0 を直す` };
  else if (out.w3.ec.hi_mm < EC_MAX_MM && out.w3.ea.hi_mm < EA_MAX_MM)
    out.a6 = { verdict: "OK", why: `+2σ と直さない分 ${FIX_TOL_MM}mm を足しても上限より小さい(e_c ${out.w3.ec.hi_mm}mm・e_a ${out.w3.ea.hi_mm}mm)` };
  else out.a6 = { verdict: "分からない", why: `残るずれの幅が上限をまたぐ(e_c ${out.w3.ec.lo_mm}〜${out.w3.ec.hi_mm}mm・e_a ${out.w3.ea.lo_mm}〜${out.w3.ea.hi_mm}mm)。2mm 前後は K7(手首のカメラで本当に回す)で見る` };
  if (out.joints.every(j => j.sd != null && j.sd <= JOINT_SD_CMD))
    out.puzzle_run = "python tools\\puzzle_run.py --sim --arm-bias " + out.joints.map(j => j.deg.toFixed(2)).join(",") + ",0   (shoulder_pan には紙の回しも入る)";
  return out;
}

function report(out) {
  const L = [];
  L.push(`測った点 ${out.n}(高さ ${out.heights_mm.join("・")}mm)`);
  if (out.why) { L.push("★" + out.why); return L; }
  L.push(`(3) 狙った所との差: いちばん大きい ${out.target_err_max_mm.toFixed(1)}mm(紙の置き方のずれも入ったまま)`);
  L.push(`(4) 繰り返しの差: ${out.repeat_sd_mm != null ? out.repeat_sd_mm.toFixed(2) + "mm(同じ点を 2 回以上測った点の標準偏差。腕のばらつきも入る)" : "同じ点を 2 回測っていない(--loops 2 で分かる)"}`);
  if (out.down_pairs.length) {
    const m = [0, 1].map(c => out.down_pairs.reduce((s, p) => s + (c ? p.dz_mm : p.dx_mm), 0) / out.down_pairs.length);
    L.push(`下りたときの爪の動き(模型なし。低い所 − 高い所の測り、${out.down_pairs.length} 組の平均): x ${m[0].toFixed(2)}・z ${m[1].toFixed(2)}mm`);
  }
  L.push(`合わせ: 測りの誤差 ${out.sigma_mm.toFixed(2)}mm(残りから。でたらめな誤差だけ)・残りのいちばん大きい ${out.residual_max_mm.toFixed(2)}mm・紙 dx ${out.paper.dx_mm}・dz ${out.paper.dz_mm}mm・縮尺 ${out.paper.scale_permil}‰`);
  L.push(`  肩 + 肘(URDF の度。本当 = 読んだ角度 + これ): ${out.lift_plus_elbow_deg ? out.lift_plus_elbow_deg.v.toFixed(2) + " ± " + out.lift_plus_elbow_deg.sd.toFixed(2) + "°" : "決まらない"}`);
  L.push(`  関節ごと(強く相関する。± が大きければ当てにならない): ` +
         out.joints.map(j => `${j.joint} ${j.deg != null ? (j.deg >= 0 ? "+" : "") + j.deg.toFixed(2) + " ± " + j.sd.toFixed(2) : "決まらない"}`).join("、"));
  if (out.w3.determined)
    L.push(`W3 で手首の確かめ(${out.w3.from_mm}mm)の後、つかむ高さ(${out.w3.to_mm}mm)で残る爪のずれ(格子 × 爪の向き ${out.w3.yaws.join("・")}°): ` +
           `e_c いちばん大きい ${out.w3.ec.max_mm} ± ${out.w3.ec.sd_mm}mm(${out.w3.ec.at})、e_a ${out.w3.ea.max_mm} ± ${out.w3.ea.sd_mm}mm(${out.w3.ea.at})。手首のカメラの取り付けは URDF の値の前提`);
  else L.push("W3 で残る爪のずれ: 決まらない");
  if (out.w3_unreached.length) L.push(`  解けなかった所: ${out.w3_unreached.join("、")}`);
  L.push(`A-6: ${out.a6.verdict}(${out.a6.why})`);
  if (out.puzzle_run) L.push(`sim で確かめる: ${out.puzzle_run}`);
  return L;
}

// ---- 検算: 作り物の k1・ずれを知っている記録
//   sys: 系統の誤り { parallax: [x, z](度。高さ h の爪の先を斜めに読み、h·tan だけずれる), scale: 千分率 }
function synthRecord(planOut, map, b, paper, sigma, loops, seed, sys) {
  let s = seed; const rnd = () => { s = (s * 16807) % 2147483647; return s / 2147483647; };
  const gauss = () => Math.sqrt(-2 * Math.log(rnd() + 1e-12)) * Math.cos(2 * Math.PI * rnd());
  const pts = [], par = (sys && sys.parallax) || [0, 0], k = 1 + ((sys && sys.scale) || 0) / 1000;
  for (let lp = 1; lp <= loops; lp++) for (const w of planOut.waypoints) {
    if (!w.target_mm) continue;
    const pres = {}; Object.keys(w.lerobot).forEach(m => { pres[m] = w.lerobot[m] + 0.3 * gauss(); });   // 追いつきの誤差
    const t = tipXZ(addB(map.toUrdf(pres).q, b)), h = w.target_mm[1];
    pts.push({ loop: lp, name: w.name, target_mm: w.target_mm, lerobot_cmd: w.lerobot, lerobot_present: pres,
               measured_xz_mm: [k * t[0] + paper[0] + h * Math.tan(par[0] * Math.PI / 180) + sigma * gauss(), k * t[1] + paper[1] + h * Math.tan(par[1] * Math.PI / 180) + sigma * gauss()] });
  }
  return { points: pts };
}
// 本当の残るずれ(b を知っているので直接): 格子 × 向きの e_c・e_a のいちばん大きい所
function truthW3(b, ROBOT_Y, camShift) {
  const c = w3Coeffs(ROBOT_Y, camShift).filter(r => !r.why), dot = (g) => g.slice(0, 4).reduce((s, v, i) => s + v * b[i], 0);
  return { ec: Math.max(...c.map(r => Math.abs(dot(r.gc)))), ea: Math.max(...c.map(r => Math.abs(dot(r.ga)))) };
}
function selftest() {
  let bad = 0;
  const k1 = kp.fakeK1({}), map = ROBOT_MAP.make(k1, M.gripFor(M.CELL_DEFAULT / 1000));
  const p2 = kp.plan(k1, { lift: 80, lift2: 50, yaw: 0 }).out, p1 = kp.plan(k1, { lift: 80, yaw: 0 }).out, RY = p2.robotY_mm / 1000;
  const check = (label, ok, lines) => { if (!ok) bad++; console.log(`${label}: ${ok ? "OK" : "★NG"}`); (lines || []).forEach(l => console.log("    " + l)); };
  // 1. 誤差なしで当たるか
  const b = [0.4, 0.8, 0.6, -0.5], tr = truthW3(b, RY);
  const o1 = analyse(synthRecord(p2, map, b, [3, -2], 0, 1, 7, { scale: 4 }), p2, map, {});
  check(`2 つの高さ・誤差なし・縮尺 4‰(本当の e_c ${tr.ec.toFixed(2)}・e_a ${tr.ea.toFixed(2)}mm)`,
        o1.joints.every((j, i) => Math.abs(j.deg - b[i]) < 0.05) && Math.abs(o1.w3.ec.max_mm - tr.ec) < 0.1 && Math.abs(o1.paper.scale_permil - 4) < 0.1, report(o1));
  // 2. 1 つの高さでは決まらないと言う
  const o2 = analyse(synthRecord(p1, map, b, [3, -2], 0, 1, 7), p1, map, {});
  check("1 つの高さ・誤差なし: 決まらないと言う", !o2.w3.determined && o2.a6.verdict === "決まらない" && o2.joints[1].deg === null);
  // 3. 種を変えて、誤った判定が出ないか(測りの誤差 0.3mm・2 回)
  const runs = (bb, sig, loops, n) => { const v = { OK: 0, NG: 0, "分からない": 0 }; for (let s = 1; s <= n; s++) v[analyse(synthRecord(p2, map, bb, [1, -1], sig, loops, s * 7919 + 13), p2, map, {}).a6.verdict]++; return v; };
  const zero = runs([0, 0, 0, 0], 0.3, 2, 20), big = [0, 1.2, 1.2, 0], tb = truthW3(big, RY), vb = runs(big, 0.3, 2, 20), z1 = runs([0, 0, 0, 0], 1, 2, 10);
  check(`ずれなし・誤差 0.3mm・2 回・種 20: NG が 0 ${JSON.stringify(zero)}`, zero.NG === 0);
  const z4 = runs([0, 0, 0, 0], 0.2, 4, 10);
  check(`ずれなし・誤差 0.2mm・4 回・種 10: NG が 0(OK の数は、紙でどこまで言えるかの目安) ${JSON.stringify(z4)}`, z4.NG === 0);
  check(`肩・肘 +1.2°(本当の e_c ${tb.ec.toFixed(2)}mm)・誤差 0.3mm・2 回・種 20: OK が 0 ${JSON.stringify(vb)}`, vb.OK === 0);
  check(`ずれなし・誤差 1mm・2 回・種 10: OK も NG も言わない ${JSON.stringify(z1)}`, z1.NG === 0);
  // 4. 系統の誤り(模型に無い物)がどれだけ化けるか(合否ではなく、数を出す)
  for (const [label, sys] of [["視差 x 1°", { parallax: [1, 0] }], ["視差 z 1°", { parallax: [0, 1] }]]) {
    const o = analyse(synthRecord(p2, map, [0, 0, 0, 0], [0, 0], 0, 1, 3, sys), p2, map, {});
    console.log(`(数だけ) ずれなし・${label}: e_c ${o.w3.ec.max_mm}・e_a ${o.w3.ea.max_mm}mm と出る。合わせの残り ${o.residual_max_mm.toFixed(2)}mm(残りには出ない)・A-6 ${o.a6.verdict}`);
  }
  // 5. 手首のカメラの取り付けの位置が URDF から 5mm 違うと、残るずれの係数がどれだけ変わるか(W4 の前提の感度)
  for (const sh of [[5, 0, 0], [0, 5, 0], [0, 0, 5]]) { const t = truthW3(b, RY, sh); console.log(`(数だけ) カメラの取り付けの位置 ${sh.join("・")}mm(カメラの軸の向き): 本当の e_c ${tr.ec.toFixed(2)} → ${t.ec.toFixed(2)}mm・e_a ${tr.ea.toFixed(2)} → ${t.ea.toFixed(2)}mm`); }
  console.log(bad ? `NG ${bad}` : "OK");
  process.exit(bad ? 1 : 0);
}

function cli() {
  if (has("selftest")) selftest();
  const runFile = arg("run");
  if (!runFile) { console.log("使い方: node tools/k1-err.js --run calib/k1_run_<日時>.json [--plan out/k1-plan.json] [--k1 calib/k1_<id>.json] [--sigma 1]  /  --selftest"); process.exit(2); }
  const rec = JSON.parse(fs.readFileSync(path.resolve(runFile), "utf8"));
  const planFile = arg("plan") ?? rec.plan;
  const plan = JSON.parse(fs.readFileSync(path.resolve(planFile), "utf8"));
  const k1file = path.resolve(arg("k1") ?? path.join(__dirname, "..", "calib", `k1_${plan.robot_id}.json`));
  const k1buf = fs.readFileSync(k1file);
  if (require("crypto").createHash("sha256").update(k1buf).digest("hex") !== plan.k1_sha256) { console.log("★表を作った k1 のファイルと違う(k1_calib.py をやり直した?)。その時の k1 のファイルを --k1 で渡す"); process.exit(1); }
  const map = ROBOT_MAP.make(JSON.parse(k1buf.toString("utf8")), M.gripFor(M.CELL_DEFAULT / 1000));
  const out = analyse(rec, plan, map, { sigma: arg("sigma") != null ? +arg("sigma") : undefined });
  report(out).forEach(l => console.log(l));
  const file = path.join(__dirname, "..", "out", "k1-err", path.basename(runFile));
  fs.mkdirSync(path.dirname(file), { recursive: true });
  fs.writeFileSync(file, JSON.stringify(Object.assign({ what: "K1 の記録の読み(sim/tools/k1-err.js)", run: path.resolve(runFile) }, out), null, 1));
  console.log("書いた: " + path.relative(process.cwd(), file));
}
if (require.main === module) cli();
else module.exports = { analyse, synthRecord, w3Residual, w3Coeffs, truthW3, HOVER_MM, PICK_MM };
