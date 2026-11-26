// 書き出した関節の列の速さ・加速度を、実機のサーボ(Feetech STS3215-C001)の上限と比べる(計画書_完成形まで の S-6 の続き。点検 C30 の (2))。
//   読むもの: tools/export-motion.js が書いた JSON(30Hz に割り直した列 samples を使う。実機へ流すのはこの列なので)。
//     既定は sim/out/servo-limits/motion_set{0..3}_stand{1,0}.json(この道具のために export-motion.js で書いたもの)。
//     無ければ sim/out/motion_set*_gap15_stand1.json。--files a.json,b.json で選べる。
//   比べるもの(関節ごと。度/秒・度/秒²):
//     (a) 列の最大の速さ・加速度(30Hz の列の差分。加速度は 2 階差分なので、割り直しの角で大きめに出る。爪は開け閉めが急で特に大きい)
//     (b) 自分たちの道具が limit_follower で入れている上限(Goal_Velocity[step/s]・Acceleration[100 step/s²] を度に直す)
//     (c) サーボそのものの上限(無負荷の速さ。5V は比例で見積もった【推定】)
//   出すもの: 関節ごとの表と、上限を超える所の印(★)。速さの上限 V・加速度の上限 A を守るように列を引き延ばすと、全体が何秒遅くなるか。
//     30Hz で 1 コマの動きに上限(safe_teleop.py の --max-step。既定 4°)を掛けたときの余裕。
//   検算: (1) 負の検算: 既定の 3×4(種 2・台あり)を 2 倍の速さで流した列(同じ角度の並びを 60Hz とみなす)で、
//             元の列が通る上限に ★ が出るか(速さは 2 倍、加速度は 4 倍になるはず)
//         (2) 引き延ばした列が、本当に V・A を守っているか(測り直す。30Hz に割り直して書いた CSV も measure で測る)
//   書くもの: sim/out/servo-limits/servo-limits.json・.txt と、推す計画の上限で引き延ばした 30Hz の列 *_plan_v*_a*.csv(流す側がそのまま使える形)
//   実行: node tools/servo-limits.js [--files a.json,b.json] [--max-step 4] [--out sim/out/servo-limits]
//         node tools/servo-limits.js --export-relook   (寄り道ありの列を先に書く。4 本で 3 分ほど)
//   ★ 角度は URDF の角度[度]。サーボの軸は関節に直に付く(SO-101 は歯車の外に減速が無い)ので、関節の度 = サーボの軸の度とみなす。
//     LeRobot の角度との 0 と向きの対応(計画書の V-4)は速さの大きさには効かない。爪は URDF の爪の角度[度]を、そのままサーボの度とみなした【仮】。
const fs = require("fs"), path = require("path");
const ROOT = path.resolve(__dirname, "..");
const arg = k => { const i = process.argv.indexOf("--" + k); return i > 0 ? process.argv[i + 1] : undefined; };

// ---- 単位(STS3215。4096 step で 1 回転) ----
const STEP_DEG = 360 / 4096;                       // 1 step = 0.0879°
const vDeg = steps => steps * STEP_DEG;            // Goal_Velocity[step/s] → 度/秒
const aDeg = reg => reg * 100 * STEP_DEG;          // Acceleration[100 step/s²] → 度/秒²
const vSteps = deg => deg / STEP_DEG;
const aReg = deg => deg / (100 * STEP_DEG);

// ---- サーボそのもの(出どころは sim/README.md の「列の速さとサーボの上限」) ----
//   ST-3215-C001(1:345)。仕様: 無負荷 0.192 秒/60°(52RPM)@7.4V・0.238 秒/60°(42RPM)@6V、止めたときの力 19.5kg·cm@7.4V・16.5kg·cm@6V。
//   このフォロワーは 5V(組み立て手順書 STEP 0)。5V の値は仕様に無いので、電圧に比例するとして見積もった【推定】。
const SERVO = {
  "7.4V": { v: 52 * 6, torque: 19.5 },             // 1RPM = 6 度/秒
  "6V": { v: 42 * 6, torque: 16.5 },
  "5V【推定】": { v: 42 * 6 * 5 / 6, torque: 16.5 * 5 / 6 }
};
const V_FREE = SERVO["5V【推定】"].v;              // 210 度/秒
const LOAD_FRAC = 0.5;                             // 負荷が掛かると速さが落ちるので、無負荷の半分を「余裕のある上限」の目安にする【仮】
const A_REG_MAX = 254;                             // Acceleration の最大(LeRobot の configure_motors が書く値。0 は「最大」で、度では書かれていない)

// ---- 自分たちの道具が入れている上限(tools/ を読んだ。2026-11-18) ----
const OURS = [
  { name: "k1_run / safe_wiggle", speed: 150, accel: 10, src: "tools/k1_run.py・tools/safe_wiggle.py の limit_follower(bus, 150, 10, 700)" },
  { name: "safe_teleop", speed: 1500, accel: 50, src: "tools/safe_teleop.py の limit_follower(fbus, 1500, 50, 800)" },
  { name: "prime_goal(本家の前)", speed: 2000, accel: 254, src: "tools/prime_goal.py の --speed 2000。加速度は書かないので、LeRobot の configure_motors の 254 が残る" },
  { name: "LeRobot の既定", speed: null, accel: 254, src: "configure_motors が Acceleration=254。Goal_Velocity は書かない(0 なら最大の速さ)" }
];
const MAX_STEP = +(arg("max-step") ?? 4);          // safe_teleop.py の --max-step の既定(度/コマ)

// ---- 寄り道ありの列を書く(--export-relook。live.js と同じ ui = defaultUiValues + RELOOK: true、台あり、種 0〜3) ----
//   export-motion.js は RELOOK を渡さない(書き換えない約束)ので、同じ形の JSON(samples だけ)をここで書く。1 本 40 秒ほど。
if (process.argv.includes("--export-relook")) {
  const { load } = require("./_load");
  load("kinematics.js", "so101.js", "motion.js", "puzzle.js",
       "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js", "collide.js", "scene-data.js");
  const M = SCENE_MODEL, HZ = 30, r3 = v => Math.round(v * 1000) / 1000;
  const at = (seq, t) => {   // export-motion.js の at と同じ(となりのコマを線形に。間は次のコマの持ち方)
    let i = 1; while (i < seq.length - 1 && seq[i].t < t) i++;
    const a = seq[i - 1], b = seq[i], u = b.t > a.t ? Math.min(1, Math.max(0, (t - a.t) / (b.t - a.t))) : 1;
    return { q: a.q.map((v, k) => v + (b.q[k] - v) * u), jaw: a.jaw + (b.jaw - a.jaw) * u, held: u > 0 ? b.held : a.held };
  };
  for (const SET of [0, 1, 2, 3]) {
    const ui = Object.assign(M.defaultUiValues(), { SET, STAND: 1, SOL: 0, RELOOK: true });
    const c = M.buildFromUi(ui)._check;
    const label = M.controls.find(x => x.key === "SET").options.find(o => o.value === SET).label;
    if (!c.solved) { console.log(`${label}: 解けない ${c.why}`); continue; }
    const samples = [];
    for (let k = 0; k / HZ <= c.T + 1e-9; k++) { const s = at(c.motion, k / HZ); samples.push({ t: Math.round(k / HZ * 1e6) / 1e6, q: s.q.map(r3), jaw: r3(s.jaw), held: s.held }); }
    const doc = { what: "SO-101 パズルの動き(live.js と同じ ui・RELOOK あり。tools/servo-limits.js --export-relook)", made: new Date().toISOString(),
      ui: { SET, GAP: ui.GAP, SOL: 0, STAND: 1, RELOOK: true }, set: label + "・寄り道あり", solved: true, hits: c.hitFrames, order: c.order, T: r3(c.T),
      joints: SO101.joints.map(j => j.name), hz: HZ, samples };
    const f = path.join(ROOT, "out", "servo-limits", `motion_set${SET}_stand1_relook.json`);
    fs.mkdirSync(path.dirname(f), { recursive: true });
    fs.writeFileSync(f, JSON.stringify(doc));
    console.log(`${label}・寄り道あり・台あり: 解けた ${c.order.join("→")} 当たり ${c.hitFrames} 全体 ${c.T.toFixed(2)} 秒 → ${path.relative(ROOT, f)}`);
  }
  process.exit(0);
}

// ---- 読む ----
function defaultFiles() {
  const dir = path.join(ROOT, "out", "servo-limits"), list = [];
  // _relook は live.js と同じ ui(RELOOK あり。写す姿勢への寄り道)で書いた列。export-motion.js は RELOOK を渡さないので、
  // 上の --export-relook で書く。live.js が実際に流すのはこちら(点検 C38 の 2)
  for (const tag of ["_relook", ""]) for (const st of [1, 0]) for (const s of [0, 1, 2, 3]) {
    const f = path.join(dir, `motion_set${s}_stand${st}${tag}.json`);
    if (fs.existsSync(f)) list.push(f);
  }
  if (list.length) return list;
  return fs.readdirSync(path.join(ROOT, "out")).filter(n => /^motion_set\d_gap\d+_stand\d\.json$/.test(n)).map(n => path.join(ROOT, "out", n));
}
const files = arg("files") ? arg("files").split(",") : defaultFiles();
if (!files.length) { console.log("読む列が無い: 先に node tools/export-motion.js --set 2 などで書き出す"); process.exit(1); }
const OUT = arg("out") || path.join(ROOT, "out", "servo-limits");

function readSeq(f) {
  const d = JSON.parse(fs.readFileSync(f, "utf8"));
  const names = d.joints.concat(["gripper"]);
  const q = d.samples.map(s => s.q.concat([s.jaw]));
  return { file: path.relative(ROOT, f), label: `${d.set}・台${d.ui.STAND ? "あり" : "なし"}`, ui: d.ui, names, hz: d.hz, dt: 1 / d.hz, q, T: (q.length - 1) / d.hz,
    held: d.samples.map(s => s.held), relook: !!d.ui.RELOOK };
}

// ---- 測る(30Hz の列の差分。始まりと終わりは止まっているとみなし、外側に速さ 0 を置く) ----
function measure(seq) {
  const { q, dt, names } = seq, n = names.length;
  const out = names.map(() => ({ v: 0, vAt: 0, a: 0, aAt: 0, step: 0 }));
  const vel = [];                                   // 区間 k(k → k+1)の速さ
  for (let k = 0; k + 1 < q.length; k++) vel.push(q[k].map((x, j) => (q[k + 1][j] - x) / dt));
  for (let j = 0; j < n; j++) {
    for (let k = 0; k < vel.length; k++) {
      const v = Math.abs(vel[k][j]);
      if (v > out[j].v) { out[j].v = v; out[j].vAt = (k + 0.5) * dt; }
      out[j].step = Math.max(out[j].step, v * dt);
    }
    for (let k = 0; k <= vel.length; k++) {         // 点 k の加速度(外側は 0)
      const v1 = k > 0 ? vel[k - 1][j] : 0, v2 = k < vel.length ? vel[k][j] : 0;
      const a = Math.abs(v2 - v1) / dt;
      if (a > out[j].a) { out[j].a = a; out[j].aAt = k * dt; }
    }
  }
  return out;
}

// ---- 引き延ばす: 同じ角度の並び(道)を、どの関節も |速さ| ≤ V・|加速度| ≤ A になる時刻で通り直す ----
//   道の進み s(点の番号)の速さ u = ds/dt を点ごとに決める(区間 k の時間 τ_k = 2/(u_k + u_{k+1}))。
//   点 k の関節 j の加速度 ≒ u_k²·(Δq_k − Δq_{k−1}) + (Δq_k + Δq_{k−1})/2 · du/dt(曲がりの分 + 道に沿って速める分)。
//   u_k の上限(速さ・曲がり・元の時刻より速くしない u ≤ 1/dt)を決め、前から速める・後ろから遅める の 2 回で u を決める(ロボットの
//   時間の最適化でよく使うやり方の簡単な形)。これは下書きで、そのあと下の「直す」で上限に収める。
//   ★ 加速度の決め方: となりの区間の速さの差 ÷ dt(1/30 秒)。区間が dt より長くても dt で割る。こうしないと、引き延ばした列を
//     30Hz に割り直したとき、角の速さの差が 1 コマに集まって上限を超える(試したら 264 → 592 度/秒² になった)。流すのは 30Hz の列なので、
//     「30Hz のとなりのコマで、速さが A·dt より変わらない」を上限とする。最短の時間とは限らない(上から押さえた時間)。始まりと終わりは止まっている。
function retime(seq, V, A) {
  const { q, dt } = seq, n = seq.names.length, N = q.length - 1;
  const dq = []; for (let k = 0; k < N; k++) dq.push(q[k].map((x, j) => q[k + 1][j] - x));
  const zero = new Array(n).fill(0);
  const dqL = k => (k > 0 ? dq[k - 1] : zero), dqR = k => (k < N ? dq[k] : zero);
  const big = d => Math.max(...d.map(Math.abs));
  // 点 k で、u² = w のときに取れる du/dt の範囲(取れなければ null)
  function dRange(k, w, Ae) {
    const L = dqL(k), R = dqR(k);
    let lo = -Infinity, hi = Infinity;
    for (let j = 0; j < n; j++) {
      const c = w * (R[j] - L[j]), b = (R[j] + L[j]) / 2;
      if (Math.abs(b) < 1e-12) { if (Math.abs(c) > Ae) return null; continue; }
      const x1 = (-Ae - c) / b, x2 = (Ae - c) / b;
      lo = Math.max(lo, Math.min(x1, x2)); hi = Math.min(hi, Math.max(x1, x2));
    }
    return lo <= hi ? [lo, hi] : null;
  }
  function solve(Ae) {
    const cap = new Array(N + 1);
    for (let k = 0; k <= N; k++) {
      const um = Math.min(1 / dt, V / Math.max(big(dqL(k)), big(dqR(k)), 1e-12));
      let hi = um * um;
      if (!dRange(k, hi, Ae)) { let lo = 0; for (let i = 0; i < 50; i++) { const m = (lo + hi) / 2; if (dRange(k, m, Ae)) lo = m; else hi = m; } hi = lo; }
      cap[k] = hi;
    }
    cap[0] = 0; cap[N] = 0;
    const w = cap.slice();
    for (let k = 0; k < N; k++) { const r = dRange(k, w[k], Ae); w[k + 1] = Math.min(w[k + 1], w[k] + 2 * (r ? Math.max(0, r[1]) : 0)); }
    for (let k = N; k > 0; k--) { const r = dRange(k, w[k], Ae); w[k - 1] = Math.min(w[k - 1], w[k] + 2 * (r ? Math.max(0, -r[0]) : 0)); }
    const tau = [];
    for (let k = 0; k < N; k++) { const s = Math.sqrt(w[k]) + Math.sqrt(w[k + 1]); tau.push(s > 0 ? Math.max(dt, 2 / s) : Infinity); }
    return tau;
  }
  // 前後の 2 回で決めた時刻は近似なので、measure と同じ差分で測って超えた点が残れば、その点の前後の区間を
  // f = √(a/A) 倍に伸ばす(その点の加速度は 1/f² になる)のを、どの点も上限内になるまで繰り返す。
  const tau = solve(A).map(t => (isFinite(t) ? t : 1));
  let it = 0, worst = Infinity;
  for (; it < 200000 && worst > 1 + 1e-4; it++) {
    worst = 0;
    for (let k = 0; k <= N; k++) {
      const tl = k > 0 ? tau[k - 1] : 0, tr = k < N ? tau[k] : 0, L = dqL(k), R = dqR(k);
      let r = 0;
      for (let j = 0; j < n; j++) r = Math.max(r, Math.abs((tr ? R[j] / tr : 0) - (tl ? L[j] / tl : 0)) / dt / A);
      if (r > 1 + 1e-4) { const f = Math.pow(r, 0.1); if (k > 0) tau[k - 1] *= f; if (k < N) tau[k] *= f; worst = Math.max(worst, r); }
    }
  }
  const T = tau.reduce((s, x) => s + x, 0), chk = remeasure({ tau, dq }, n, dt);
  return { T, tau, dq, iter: it, converged: chk.v <= V * 1.001 && chk.a <= A * 1.001 };
}
// 引き延ばした列を測り直す(検算 (2))
function remeasure(rt, n, dt) {
  const { tau, dq } = rt, N = tau.length;
  let v = 0, a = 0;
  for (let k = 0; k <= N; k++) for (let j = 0; j < n; j++) {
    const v1 = k > 0 ? dq[k - 1][j] / tau[k - 1] : 0, v2 = k < N ? dq[k][j] / tau[k] : 0;
    if (k < N) v = Math.max(v, Math.abs(v2));
    a = Math.max(a, Math.abs(v2 - v1) / dt);
  }
  return { v, a };
}
// 全体を同じ倍率で遅くする(流す側で hz を下げるだけで済む)ときの倍率
const uniformFactor = (m, V, A) => Math.max(...m.map(x => Math.max(x.v / V, Math.sqrt(x.a / A))));

// ---- 比べる ----
const f0 = x => x.toFixed(0), f1 = x => x.toFixed(1), f2 = x => x.toFixed(2);
const pad = (s, w) => { s = String(s); const len = [...s].reduce((c, ch) => c + (ch.charCodeAt(0) > 255 ? 2 : 1), 0); return s + " ".repeat(Math.max(0, w - len)); };
const report = { what: "関節の列の速さ・加速度とサーボの上限(sim/tools/servo-limits.js)", made: new Date().toISOString(),
  units: { v: "度/秒", a: "度/秒²", speed: "Goal_Velocity[step/s]", accel: "Acceleration[100 step/s²]" },
  servo: SERVO, ours: OURS.map(o => ({ ...o, vDeg: o.speed == null ? null : vDeg(o.speed), aDeg: aDeg(o.accel) })), maxStep: MAX_STEP, seqs: [] };
const lines = [];
const log = s => { console.log(s); lines.push(s); };
log(`サーボ(STS3215-C001)の無負荷の速さ: 7.4V ${f0(SERVO["7.4V"].v)}・6V ${f0(SERVO["6V"].v)}・5V【推定】${f0(V_FREE)} 度/秒。負荷の余裕の目安(5V の半分)${f0(V_FREE * LOAD_FRAC)} 度/秒`);
log(`自分たちの上限: ${report.ours.map(o => `${o.name} = ${o.vDeg == null ? "速さ 最大" : f1(o.vDeg) + " 度/秒"}・${f0(o.aDeg)} 度/秒²`).join(" / ")}`);
log("");

const seqs = files.map(readSeq);
const V_LIST = [150, 300, 500, 750, 1000, 1200, 1500, 2000];  // step/s
const A_LIST = [10, 20, 30, 40, 50, 100, 254];                    // 100 step/s²
const all = [];
for (const s of seqs) {
  const m = measure(s);
  all.push({ s, m });
  log(`[${s.label}] ${s.file}  全体 ${f1(s.T)} 秒・${s.q.length} 点(${s.hz}Hz)`);
  log("  " + pad("関節", 14) + pad("最大の速さ", 16) + pad("最大の加速度", 16) + pad("1コマ", 9) + OURS.map(o => pad(o.name.slice(0, 12), 14)).join("") + "サーボ(5V推定)");
  const rows = [];
  m.forEach((x, j) => {
    const marks = OURS.map(o => {
      const ov = o.speed != null && x.v > vDeg(o.speed), oa = x.a > aDeg(o.accel);
      return pad((ov ? "★速" : "") + (oa ? "★加" : "") || "OK", 14);
    }).join("");
    const servo = x.v > V_FREE ? "★超え" : x.v > V_FREE * LOAD_FRAC ? "△半分超え" : "OK";
    log("  " + pad(s.names[j], 14) + pad(`${f0(x.v)}(${f2(x.vAt)}s)`, 16) + pad(`${f0(x.a)}(${f2(x.aAt)}s)`, 16) + pad(f2(x.step) + "°", 9) + marks + servo);
    rows.push({ joint: s.names[j], v: +f1(x.v), vAt: +f2(x.vAt), a: +f0(x.a), aAt: +f2(x.aAt), stepDeg: +f2(x.step), servo });
  });
  const stepMax = Math.max(...m.map(x => x.step));
  const tight = m.reduce((b, x, j) => (x.v > m[b].v ? j : b), 0);
  log(`  30Hz で 1 コマの上限 ${MAX_STEP}° を掛けたとき: 列の 1 コマの最大 ${f2(stepMax)}°(${s.names[m.findIndex(x => x.step === stepMax)]})→ 余裕 ${f2(MAX_STEP - stepMax)}°(${f0((1 - stepMax / MAX_STEP) * 100)}%)` +
      (stepMax > MAX_STEP ? " ★上限で削られる(遅れる)" : ""));
  // 引き延ばし
  // 上限 (V', A') を守る列は、それより緩い (V ≥ V', A ≥ A') も守る。引き延ばしは最短でないので、表の各升は
  // 「緩くない升で得た時刻のうち、いちばん短いもの」を使う(表が上限を緩めるほど短くなるように)。
  const grid = [], best = {};
  for (const V of V_LIST) for (const A of A_LIST) {
    const rt = retime(s, vDeg(V), aDeg(A));
    let b = rt.converged ? { T: rt.T, tau: rt.tau, from: [V, A] } : null;
    for (const key of [`${V_LIST[V_LIST.indexOf(V) - 1]},${A}`, `${V},${A_LIST[A_LIST.indexOf(A) - 1]}`]) {
      const o = best[key]; if (o && (!b || o.T < b.T)) b = o;
    }
    best[`${V},${A}`] = b;
    grid.push({ speed: V, accel: A, T: b ? +f2(b.T) : null, plus: b ? +f2(b.T - s.T) : null, from: b ? b.from : null,
      uniform: +f2(s.T * Math.max(1, uniformFactor(m, vDeg(V), aDeg(A)))), converged: !!b });
  }
  s.best = best;
  report.seqs.push({ file: s.file, label: s.label, ui: s.ui, T: +f2(s.T), joints: rows, stepMax: +f2(stepMax), tightest: s.names[tight], retime: grid });
  log("");
}

// ---- 引き延ばしの表(既定の 3×4・台ありを詳しく。ほかは推す値だけ) ----
const base = all.find(x => x.s.ui.SET === 2 && x.s.ui.STAND === 1 && x.s.relook) || all.find(x => x.s.ui.SET === 2 && x.s.ui.STAND === 1) || all[0];
log(`[引き延ばし] ${base.s.label}(元 ${f1(base.s.T)} 秒)。速さの上限 V[step/s(度/秒)] × 加速度の上限 A[レジスタ(度/秒²)] で、全体が何秒になるか(+遅れ)。かっこは全体を同じ倍率で遅くしたとき`);
log("  " + pad("V \\ A", 14) + A_LIST.map(A => pad(`${A}(${f0(aDeg(A))})`, 22)).join(""));
for (const V of V_LIST) {
  const r = report.seqs.find(x => x.file === base.s.file).retime.filter(g => g.speed === V);
  log("  " + pad(`${V}(${f0(vDeg(V))})`, 14) + r.map(g => pad(g.T == null ? `—[${f1(g.uniform)}]` : `${f1(g.T)}(+${f1(g.plus)})[${f1(g.uniform)}]`, 22)).join(""));
}
log("");

// ---- 推す値 ----
//   考え方: 列そのものを「計画の上限」(V_PLAN・A_PLAN)で引き延ばしてから 30Hz で流す。形を整えるのは列の側。
//   サーボの Goal_Velocity は計画の少し上に置き、おかしな目標が来たときの歯止めにする。
//   サーボの Acceleration は低くしない(点検 C38 の 1): Acceleration が 0 でないと、サーボは目標ごとに「そこで止まる」つもりの台形の動きを作るので、
//   目標が先へ進み続ける間も、速さ v で走るには目標との差が v²/(2a) 要る【モデル。実機で測る】。この遅れは関節ごとに速さの 2 乗で違うので、
//   道具の点が sim で当たりを調べた線から外れる。なので列を整えたうえで、Acceleration は高く(254 = LeRobot が接続のたびに書く値)する。
const V_PLAN = 1000, A_PLAN = 40, MARGIN = 1.25;   // step/s・100 step/s²【仮】
const capV = Math.round(V_PLAN * MARGIN / 50) * 50, capA = A_REG_MAX;
const lag = (vdeg, reg) => vdeg * vdeg / (2 * aDeg(reg));   // 速さ vdeg で走るときの、目標との差[度]【モデル】
log(`[推す値【仮】] 計画の上限 V ${V_PLAN} step/s(${f0(vDeg(V_PLAN))} 度/秒。5V の無負荷の推定の ${f0(vDeg(V_PLAN) / V_FREE * 100)}%)・A ${A_PLAN}(${f0(aDeg(A_PLAN))} 度/秒²)で列を引き延ばして流し、`);
log(`  サーボは limit_follower(bus, speed=${capV}, accel=${capA}, ...)(${f0(vDeg(capV))} 度/秒は計画の ${MARGIN} 倍の歯止め。加速度は高いまま)。接続の後に書く(LeRobot の接続は Acceleration=254 を書き直す)。`);
log(`  サーボの Acceleration ごとの、計画の速さ ${f0(vDeg(V_PLAN))} 度/秒 で走るときの目標との差【モデル】: ` +
    [10, 40, 50, 100, 254].map(r => `${r} → ${f1(lag(vDeg(V_PLAN), r))}°`).join("・") + `(D4 の空中の 1 回で Goal と Present_Position を記録して確かめる)`);
log(`  ★ 5V で負荷があると速さは無負荷より落ち、limit_follower の Torque_Limit 700〜800 でさらに落ちる。V_PLAN は D4 の空中で Present_Velocity・Present_Load・Present_Voltage を見て決め直す【推定】`);
log("  形ごとの全体の秒(元 → 引き延ばし後):");
const rec = [];
for (const { s } of all) {
  const b = s.best[`${V_PLAN},${A_PLAN}`];
  const rt = b ? { T: b.T, tau: b.tau, dq: s.q.slice(1).map((x, k) => x.map((v, j) => v - s.q[k][j])) } : { T: NaN, tau: [], dq: [] };
  const chk = b ? remeasure(rt, s.names.length, s.dt) : { v: NaN, a: NaN };
  const ok = !!b && chk.v <= vDeg(V_PLAN) * 1.001 && chk.a <= aDeg(A_PLAN) * 1.001;
  // 引き延ばした列を 30Hz に割り直して書く(流す側がそのまま使える形。時刻で線形に結ぶ)
  let plan30 = null;
  if (b) {
    const tk = [0]; rt.tau.forEach(t => tk.push(tk[tk.length - 1] + t));
    const rows30 = [], tSim = [], held30 = []; let i = 1;
    for (let k = 0; k * s.dt <= tk[tk.length - 1] + 1e-9; k++) {
      const t = k * s.dt; while (i < tk.length - 1 && tk[i] < t) i++;
      const u = Math.min(1, Math.max(0, (t - tk[i - 1]) / (tk[i] - tk[i - 1])));
      rows30.push(s.q[i - 1].map((v, j) => v + (s.q[i][j] - v) * u));
      tSim.push((i - 1 + u) * s.dt);                                   // 元の列(sim)の時刻。撮る時刻・持ち替えを sim の時刻で選ぶ道具のため(点検 C38 の 2)
      held30.push(u > 0 ? s.held[i] : s.held[i - 1]);                 // export-motion.js と同じく、間は次の点の持ち方
    }
    const mp = measure({ ...s, q: rows30 });
    plan30 = { v: Math.max(...mp.map(x => x.v)), a: Math.max(...mp.map(x => x.a)), step: Math.max(...mp.map(x => x.step)) };
    const name = path.basename(s.file, ".json") + `_plan_v${V_PLAN}_a${A_PLAN}.csv`;
    fs.mkdirSync(OUT, { recursive: true });
    fs.writeFileSync(path.join(OUT, name), ["t," + s.names.join(",") + ",held,t_sim"].concat(rows30.map((r, k) =>
      [(k * s.dt).toFixed(6)].concat(r.map(v => v.toFixed(3)), [held30[k], tSim[k].toFixed(6)]).join(","))).join("\n") + "\n");
    plan30.file = name;
  }
  const ok30 = ok && !!plan30 && plan30.v <= vDeg(V_PLAN) * 1.001 && plan30.a <= aDeg(A_PLAN) * 1.001;
  rec.push({ ok30,  label: s.label, T: +f2(s.T), Tplan: +f2(rt.T), vAfter: +f1(chk.v), aAfter: +f0(chk.a), ok, plan30 });
  log(`    ${pad(s.label, 34)} ${f1(s.T)} → ${f1(rt.T)} 秒(+${f1(rt.T - s.T)})  検算(2) 引き延ばした列の最大 ${f0(chk.v)} 度/秒・${f0(chk.a)} 度/秒² ${ok ? "OK" : "★NG"}` +
      (plan30 ? `  30Hz に割り直した列(${plan30.file}): ${f0(plan30.v)} 度/秒・${f0(plan30.a)} 度/秒²・1 コマ ${f2(plan30.step)}°` : ""));
}
log(`  引き延ばした列を 30Hz で流すとき、1 コマの動きは どの関節も ${f2(vDeg(V_PLAN) / 30)}° 以下(速さ ${f0(vDeg(V_PLAN))} 度/秒 ÷ 30)。1 コマの上限 ${MAX_STEP}° に対して余裕 ${f2(MAX_STEP - vDeg(V_PLAN) / 30)}°`);
const V_FIRST = 500, A_FIRST = 20;   // D4 の最初の空中の 1 回【仮】
const bf = all.find(x => x === base).s.best[`${V_FIRST},${A_FIRST}`];
log(`  D4 の最初の空中の 1 回は、半分の V ${V_FIRST}(${f0(vDeg(V_FIRST))} 度/秒)・A ${A_FIRST}(${f0(aDeg(A_FIRST))} 度/秒²)で引き延ばす案【仮】: ${base.s.label} で ${bf ? f1(bf.T) : "—"} 秒`);
report.recommend = { first: { speed: V_FIRST, accel: A_FIRST, T: bf ? +f2(bf.T) : null }, plan: { speed: V_PLAN, accel: A_PLAN, vDeg: +f1(vDeg(V_PLAN)), aDeg: +f0(aDeg(A_PLAN)) }, cap: { speed: capV, accel: capA, vDeg: +f1(vDeg(capV)), aDeg: +f0(aDeg(capA)) }, margin: MARGIN, seqs: rec };
log("");

// ---- いちばんきつい関節(全部の形を通して、列の最大の速さ / 推す計画の V) ----
const worstJ = {};
for (const { s, m } of all) m.forEach((x, j) => { const n = s.names[j]; worstJ[n] = worstJ[n] || { v: 0, a: 0, vWhere: "", aWhere: "" };
  if (x.v > worstJ[n].v) { worstJ[n].v = x.v; worstJ[n].vWhere = s.label; } if (x.a > worstJ[n].a) { worstJ[n].a = x.a; worstJ[n].aWhere = s.label; } });
log(`[関節ごとの最大(${all.length} 本の列を通して。列ごとの値は上の表)]`);
Object.entries(worstJ).sort((a, b) => b[1].v - a[1].v).forEach(([n, w]) =>
  log(`  ${pad(n, 14)} 速さ ${f0(w.v)} 度/秒(${w.vWhere})= 5V の無負荷の ${f0(w.v / V_FREE * 100)}%  加速度 ${f0(w.a)} 度/秒²(${w.aWhere})= レジスタ ${f1(aReg(w.a))}`));
report.worstJoints = worstJ;
log(`  ★ 加速度は 30Hz の 2 階差分なので、刻みで値が変わる。爪の値は、一定の速さ(約 56 度/秒)が 1 コマで始まって止まる形の影(56×30 ≒ 1670 が上の端)で、`);
log(`    物理の値ではない。爪は URDF の度をそのままサーボの度とみなした【仮】(LeRobot の爪は 0〜100%。safe_teleop の --max-step も爪は %)`);
log("");

// ---- 検算 (1) 負の検算: 2 倍の速さ ----
let ng = 0;
{
  const s = base.s, m = measure(s);
  // 元の列がちょうど通る上限(最大の 1.1 倍)を作り、元は OK・2 倍は ★ になるか
  const V = Math.max(...m.map(x => x.v)) * 1.1, A = Math.max(...m.map(x => x.a)) * 1.1;
  const fast = { ...s, hz: s.hz * 2, dt: s.dt / 2, T: s.T / 2 };
  const mf = measure(fast);
  const okOrig = m.every(x => x.v <= V && x.a <= A);
  const flagged = mf.filter(x => x.v > V || x.a > A).length;
  const ratioV = Math.max(...mf.map(x => x.v)) / Math.max(...m.map(x => x.v)), ratioA = Math.max(...mf.map(x => x.a)) / Math.max(...m.map(x => x.a));
  const servoOver = mf.filter(x => x.v > V_FREE).length, servoOverOrig = m.filter(x => x.v > V_FREE).length;
  const pass = okOrig && flagged > 0 && servoOver > servoOverOrig && Math.abs(ratioV - 2) < 1e-6 && Math.abs(ratioA - 4) < 1e-6;
  // limit_follower の値でも: 推す歯止めで元は通るか、2 倍は ★ か
  const capOrig = m.filter(x => x.v > vDeg(capV) || x.a > aDeg(capA)).length, capFast = mf.filter(x => x.v > vDeg(capV) || x.a > aDeg(capA)).length;
  if (!pass) ng++;
  log(`検算(1) 負の検算: ${s.label} を 2 倍の速さで流す → 速さ ${f2(ratioV)} 倍・加速度 ${f2(ratioA)} 倍。元の列が通る上限(${f0(V)} 度/秒・${f0(A)} 度/秒²)で、元は ${okOrig ? "全部 OK" : "★あり"}、2 倍は ${flagged} 関節に ★。サーボの無負荷の速さ(5V【推定】${f0(V_FREE)} 度/秒)の超えは 元 ${servoOverOrig} → 2 倍 ${servoOver} 関節 ${pass ? "OK(超えを見つけた)" : "★NG(見逃した)"}`);
  log(`        参考: 引き延ばす前の元の列を歯止め(${capV}・${capA})にそのまま流すと ★ ${capOrig} 関節、2 倍なら ★ ${capFast} 関節`);
  // (1b) 列を壊す: 真ん中の 1 コマだけ shoulder_lift を 10° 跳ばす。測り方が跳びを見逃さないか(2 倍の検算は measure の式からそのまま
  //      出るので、測り方そのものを試すにはこちらが要る。点検 C38 の 7)。跳んだ所は 10°/dt = 300 度/秒 になるはず
  const bad = s.q.map(r => r.slice()), mid = Math.floor(bad.length / 2);
  bad[mid][1] += 10;
  const mb = measure({ ...s, q: bad });
  const passB = mb[1].v >= 10 / s.dt - m[1].v - 1e-6 && mb[1].v > V && mb[1].v > V_FREE && Math.abs(mb[1].vAt - (mid - 0.5) * s.dt) < s.dt + 1e-9;
  if (!passB) ng++;
  log(`検算(1b) 負の検算: 真ん中の 1 コマ(${f2(mid * s.dt)} 秒)だけ shoulder_lift を 10° 跳ばす → shoulder_lift の最大 ${f0(mb[1].v)} 度/秒(${f2(mb[1].vAt)} 秒)・` +
      `1 コマ ${f2(mb[1].step)}° ${passB ? "OK(跳びを見つけた。元の上限とサーボの無負荷の速さの両方を超える)" : "★NG(見逃した)"}`);
  report.negative = { ratioV: +f2(ratioV), ratioA: +f2(ratioA), flagged, pass, capOrig, capFast, jump: { v: +f0(mb[1].v), at: +f2(mb[1].vAt), pass: passB } };
}
if (rec.some(r => !r.ok || !r.ok30)) ng++;
log(ng ? `NG ${ng} 件` : "OK");

fs.mkdirSync(OUT, { recursive: true });
fs.writeFileSync(path.join(OUT, "servo-limits.json"), JSON.stringify(report, null, 1));
fs.writeFileSync(path.join(OUT, "servo-limits.txt"), lines.join("\n") + "\n");
console.log(`書いた: ${path.relative(ROOT, path.join(OUT, "servo-limits.json"))} と .txt`);
process.exit(ng ? 1 : 0);
