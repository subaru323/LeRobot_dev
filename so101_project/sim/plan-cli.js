// 計画の口(08 §5 の (2)(3)(4) と (6) の測る所。09 の I-3。W3 の制御プログラム tools/puzzle_run.py が子のプロセスとして立ち上げる)。
//   標準入力に 1 行 1 つの JSON の頼み、標準出力に 1 行 1 つの JSON の答え。読み込みに数秒かかるので、1 本のプロセスで何度も頼む。
//   当たり判定の持ち主を sim/ の 1 つにするため(08 §5「(3) を sim/ にやらせる理由」)、計画・判定・換算(robot-map.js)は全部ここでする。Python は換算しない。
//   頼み(cmd):
//     init   {set, gap, cell, height, fit_margin_mm, grasp, stand, k1: ファイル | "fake", calib: "sim" | {top, wrist}, calibMm, calibDeg, noise, seed}
//              height・fit_margin_mm・grasp は S1(2026-11-22。渡さなければ前と同じ): 高さ mm(辺と違うと絵の道具は断る)・指の両側の余裕 mm(正の数)・
//              {名前 か "#番号": つかみ方の番号}(盤に無い名前は断る。同じ名前のピースを分けるのは "#番号")  → 始まりの姿勢・ピース・calibKey・組む順(order) など
//              calib の形(2026-11-20。tools/puzzle_run.py が K5 のファイルから作る): top = {tip: [m], dir, up, fov(縦の画角・度), panDeg}(机 = 0 の、紙の座標。ゆがみを取った絵の
//              ピンホール。絵の真ん中が光の軸。panDeg があれば肩の回転の軸の回りに −panDeg 回して計画の座標にする)、wrist = {t: [m], R: 9 個, fov} または null(URDF の手首のカメラ)。t・R は vision-wrist.js cameraAt の cig
//     truth  {seed, shiftMm, yawDeg}           (シミュレーションだけ) 本当の置き場を引く(vision-loop.js と同じ引き方)
//     render {cam: "top"|"wrist", truth, hide: [pid], lerobot: {関節: 度, gripper}, out, occluders, light}   (シミュレーションだけ) 絵を書く(.rgb = 生の RGB)。light は vision.js の opts.light(S3。渡さなければ前と同じ絵)
//              occluders(S2。2026-11-22。渡さなければ前と同じ): 絵だけに描く箱の並び(関数 occluderTris)。遮蔽の試験に使う
//     map    {lerobot} | {urdf, jaw}          (S2。2026-11-22) 換算だけ(LeRobot の度 ⇄ URDF の度)。tools/puzzle_run.py --mj の世界が使う
//     detect {img, W, H, only, board, board_fix} 上の絵 → 置き場(mm・度)。only: [pid] なら、そのピースだけ探し、ほかは計画の置き場(init の既定)に置いたことにする(filled)
//            board: true で同じ絵の盤を読んで、board_fix: board の答えで、見つけたピースを盤に対する位置にする(較正のカメラの誤りを打ち消す)
//     board  {img, W, H}                      上の絵の盤のマスの縁 → 較正のカメラで読んだ机の上の点を本当の盤に合わせる回しとずらし(盤が空いているときに読む)
//     plan   {layout, air, pieces, place_offset, sym, sym_pids} 置き場 → 関節の列(URDF と LeRobot)・判定。換算の戻し・キャリブレーションの範囲も見る。pieces: 先頭の N 手だけ。
//            断るとき(置き場そのものの断りを除く)は、対称なピース(O・X5 は 3 通り、I・S・Z・I3・D2・I5・Z5 は 1 通り)のほかの表し方を試す(sym: false で試さない。
//            sym_pids で試すピースと順を決める)。答えの layout は使った表し方、sym はその中身、sym_tried は試した数
//            place_offset: [{pid, dx_mm, dz_mm, dyaw_deg}] 置く所だけをずらす(つかむ所は同じ。2026-11-20)
//            offset_is_arm: true … place_offset を、置く姿勢で腕がくり返す誤りを打ち消す分と見る。置く段(盤の真上から、置いて上へ戻るまで)の
//              当たりを、爪のまわり(j5・動く爪・手首のカメラ)と持っているピースをずらす前の所(本当の所)へ戻して、ほかのピース・台と比べる
//              (既定 false: ずらした動きのまま。2026-11-22)。戻した所と、爪の閉じる向きに直角に ±offset_perp_mm(既定 4)動かした所の 3 つで測る。
//              使ったときは答えに offset_is_arm: true
//            arm_margin_mm: M … つかむ段(つかむ所の真上から上げ終えるまで)と置く段(盤の真上から上へ戻るまで)で、爪のまわりと持っているピースを
//              机の x・z の 8 つの向きへ M 動かした所でも、ほかのピースとの当たりを測る(腕が言った所からずれて着く分。既定 無し = 前と同じ。2026-11-22)。
//              答えに arm_margin_mm と、動かして測った組のいちばん近い所 arm_margin_clear_mm・arm_margin_pair・arm_margin_t
//            true_bias_deg: [5 つ] … (シミュレーションだけ) 本当の腕 = 言った角度 + これ(URDF の度)。同じ段で、本当の腕とほかのピースのいちばん近い所を
//              答えに true_clear_mm・true_clear_pair・true_clear_t(合否には入れない)
//            phase_clear: true … (S1。2026-11-22) 答えに verdict_kind(ok / 拒否 = 寸法で決まる / 未解決 = 探しきれない・当たりが残る)、dims_mm(辺・高さ・すき間・指の厚さ・
//              挟める幅など)、phase_clear(1 手ごと・段ごとの、ほかのピースと机までのいちばん近い所。null は 20mm 以上)。断った答えにも付く
//            layout.pieces[i].on_board: true … 盤の上に置いた後のピースを、そこからつかみ直す(置き直し。盤に重なっても断らない。2026-11-20)
//            layout.pieces[i].keep: true … 盤の上に置き終えたピース。動かさない(組む手から外す。pieces の N にも数えない)が、当たりの相手にはなる
//     wrist  {img, W, H, lerobot, pid}         手首の絵 → 狙うピースの置き場。top_frac(S2。2026-11-22): 見えた上の面の広さ ÷ 形の上の面の広さ(隠れの目安。ここでは断らない)
//     match  {pid, est, meas}                  見積もり → 測った置き場 の重ね合わせ(ずれ mm・度と、見積もりの表し方のままの直した置き場)
//     score  {truth, picks: [{pid, est, cS, yp, lerobot, place_lerobot}]}   (シミュレーションだけ) つかめたかの採点(vision-loop.js と同じ e_a・e_c)と、
//              place_lerobot があれば置いた所のずれ place_mm(< 3mm【仮】も合格の条件)
//     simcalib {}                              (検算用) sim のカメラを calib の形で返す(top・wrist と、絵の大きさ W・H)。puzzle_run.py --make-sim-calib が K5 の形のファイルにする
//   どの答えにも ok(true/false)と、ok が false なら why。
"use strict";
const fs = require("fs"), path = require("path"), crypto = require("crypto"), readline = require("readline");
const { load } = require("./tools/_load");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js", "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js",
     "collide.js", "chikuwa-collision.js", "self-collision.js");
global.window = { SCENE_SKIP_DEFAULT: 1 }; load("scene-data.js"); delete global.window;
load("vision.js", "vision-wrist.js", "robot-map.js");
const M = SCENE_MODEL, V = VISION, VW = VISION_WRIST;
const ROUNDTRIP_TOL_DEG = 1e-6;

let S = null;   // init の後の状態

// 対称なピースのほかの表し方(同じマスの集まりを、向き +90°k と別の原点で表す。O は 3 通り、I・S・Z・I3・D2 などは 1 通り)。mm・度
function symReps(shape, e) {
  const P = S.c0.pitch, cells = (x, z, y) => V.cellsOf(shape, P, x / 1000, z / 1000, y);
  const A = cells(e.x, e.z, e.yaw), ca = [A.reduce((s, c) => s + c[0], 0) / A.length, A.reduce((s, c) => s + c[1], 0) / A.length], out = [];
  for (const k of [1, 2, 3]) {
    const y = e.yaw + 90 * k, B0 = cells(0, 0, y), cb = [B0.reduce((s, c) => s + c[0], 0) / B0.length, B0.reduce((s, c) => s + c[1], 0) / B0.length];
    const x = (ca[0] - cb[0]) * 1000, z = (ca[1] - cb[1]) * 1000, B = cells(x, z, y);
    if (B.every(b => A.some(a => Math.hypot(a[0] - b[0], a[1] - b[1]) < 1e-6))) out.push(Object.assign({}, e, { x: +x.toFixed(3), z: +z.toFixed(3), yaw: +y.toFixed(3) }));
  }
  return out;
}

function fakeK1() {   // tools/k1-plan.js の fakeK1 と同じ形(符号と 0 のずれを関節ごとに変えて、換算の道を通す)
  const cal = {}, sign = {}, offset = {};
  ROBOT_MAP.JOINTS.concat(["gripper"]).forEach((m, i) => { cal[m] = { id: i + 1, drive_mode: 0, homing_offset: 0, range_min: 2048 - 1700, range_max: 2048 + 1700 }; });
  ROBOT_MAP.JOINTS.forEach((m, i) => { sign[m] = i % 2 ? -1 : 1; offset[m] = (i - 2) * 7.5; });
  return { robot_id: "fake", date: "-", verified: true, calibration: cal, sign, offset, gripper: [{ lerobot: 2, gap_mm: 0 }, { lerobot: 60, gap_mm: 30 }] };
}
function perturbCam(cam, mm, deg, r) {   // vision-loop.js と同じ
  const u = () => { const v = [V.gauss(r), V.gauss(r), V.gauss(r)], n = Math.hypot(...v); return v.map(x => x / n); };
  const dp = u().map(x => x * mm / 1000), ax = u(), R = KIN.rotAA(ax, deg * Math.PI / 180);
  return { tip: cam.tip.map((v, i) => v + dp[i]), dir: KIN.matVec(R, cam.dir), up: KIN.matVec(R, cam.up) };
}
function readRgb(p, W, H) {
  const b = fs.readFileSync(p);
  if (b.length !== W * H * 3) throw new Error(`${p}: ${b.length} バイト(${W}×${H}×3 = ${W * H * 3} のはず)`);
  return { W, H, rgb: new Uint8ClampedArray(b.buffer, b.byteOffset, b.length) };
}
function slotsOf(layout) {   // LAYOUT(mm・度) → worldTris の {slots(m), rots}
  return { slots: layout.pieces.map(p => [p.x / 1000, p.z / 1000]), rots: layout.pieces.map(p => p.yaw) };
}
function toUrdfQ(L) { const u = S.map.toUrdf(L); return { q: u.q, jaw: u.jaw }; }
// 逆運動学の可動域を、URDF の可動域と k1 の範囲(キャリブレーション − 余裕。robot-map.js の limits を URDF の度に直した物)の重なりに狭めて fn を呼ぶ。
//   motion.js は毎回 SO101.limitsDeg を読むので、その間だけ差し替える(4×5 で、sim の wrist_roll 155° に作り物の 0 のずれ 15° が乗り、
//   ±170 の外に出て止まった。2026-11-18)。待ちの姿勢・写す姿勢・経由点への関節の直線は可動域を見ないので、plan の outOfRange が止める
function narrowed(fn) {
  const limSave = SO101.limitsDeg;
  SO101.limitsDeg = limSave.map((l, i) => {
    const m = ROBOT_MAP.JOINTS[i], kl = S.map.limits[m], ends = kl.map(v => S.k1.sign[m] * (v - S.k1.offset[m]));
    return [Math.max(l[0], Math.min(...ends)), Math.min(l[1], Math.max(...ends))];
  });
  try { return fn(); } finally { SO101.limitsDeg = limSave; }
}
// 本当の腕の姿勢(シミュレーションだけ): 読んだ角度(LeRobot)を URDF に直し、関節の誤差 bias(URDF の度。SimRobot が持つ)を足す
function trueQ(L, bias) { const u = toUrdfQ(L); return { q: u.q.map((v, i) => v + ((bias && bias[i]) || 0)), jaw: u.jaw }; }

// render の occluders(S2。2026-11-22。シミュレーションだけ): 絵だけに描く箱(手・紙・ケーブルなどがカメラとピースの間に入った扱い。当たりは無い)。
//   [{x_mm, z_mm: 真ん中, y_mm: 下の面の高さ(机 = 0), w_mm: x の幅, d_mm: z の幅, t_mm: 厚さ(既定 2), rgb: [r, g, b](既定 灰)}]
function occluderTris(list) {
  if (list === undefined || list === null) return { tris: [] };
  if (!Array.isArray(list)) return { why: "occluders は箱の並び" };
  const out = [];
  for (const o of list) {
    const t = o.t_mm ?? 2, rgb = o.rgb ?? [120, 120, 125];
    if (![o.x_mm, o.z_mm, o.y_mm, o.w_mm, o.d_mm, t].every(Number.isFinite) || o.w_mm <= 0 || o.d_mm <= 0 || t <= 0 || o.y_mm < 0)
      return { why: "occluders の箱は x_mm・z_mm・y_mm(0 以上)・w_mm・d_mm(正)・t_mm(正)の数" };
    if (!(Array.isArray(rgb) && rgb.length === 3 && rgb.every(v => Number.isFinite(v) && v >= 0 && v <= 255))) return { why: "occluders の rgb は 0〜255 の 3 つ" };
    const x0 = (o.x_mm - o.w_mm / 2) / 1000, x1 = (o.x_mm + o.w_mm / 2) / 1000, z0 = (o.z_mm - o.d_mm / 2) / 1000, z1 = (o.z_mm + o.d_mm / 2) / 1000, y0 = o.y_mm / 1000, y1 = (o.y_mm + t) / 1000;
    const V8 = [[x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1], [x0, y1, z0], [x1, y1, z0], [x1, y1, z1], [x0, y1, z1]], pts = [];
    [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]].forEach(f => pts.push(V8[f[0]], V8[f[1]], V8[f[2]], V8[f[0]], V8[f[2]], V8[f[3]]));
    out.push({ pts, color: rgb, tag: "occluder" });
  }
  return { tris: out };
}
// 盤を読んだ T の答えの形。sim_resid_mm(sim だけ): 本当のカメラで写した盤のマスの真ん中を較正のカメラで戻し、T を掛けたときに本当の所から離れる最大
function boardOut(T) {
  const out = { dx_mm: T.dx * 1000, dz_mm: T.dz * 1000, dyaw_deg: T.yaw, about_mm: T.about.map(v => v * 1000), n_edge: T.nEdge, n: T.n, cost: T.cost, levels: T.levels };
  const tc = S.d0.overheadCamera;
  if (S.calibCam !== tc && S.base && tc) {
    const W = 640, H = 480, it = SO101.overheadImage(tc, SO101.CAM_FOV.top, W, H), ic = SO101.overheadImage(S.calibCam, S.topFov, W, H);
    let worst = 0, worst0 = 0;
    for (const hgt of [0.0003, S.c0.cell]) for (const b of S.c0.boardCells) {
      const pr = it.project([b[0], hgt, b[1]]), w = ic.unproject(pr[0], pr[1], hgt), o = V.boardApply(T, w[0], w[2], 0);
      worst = Math.max(worst, Math.hypot(o[0] - b[0], o[1] - b[1])); worst0 = Math.max(worst0, Math.hypot(w[0] - b[0], w[2] - b[1]));
    }
    out.sim_resid_mm = worst * 1000; out.sim_before_mm = worst0 * 1000;
  }
  return out;
}
const cmds = {
  init(a) {
    const def = M.defaultUiValues();
    const base = { SET: a.set ?? 2, GAP: a.gap ?? def.GAP, STAND: a.stand ?? 1, SOL: 0, NO_REFINE: true };
    if (a.cell !== undefined) base.CELL = a.cell;
    // S1(2026-11-22。どれも渡さなければ前と同じ): height = ピースの高さ mm(既定は辺。辺と違うと絵の道具は断る)、fit_margin_mm = 指の両側の余裕(既定 2)、
    //   grasp = {ピースの名前: つかみ方の番号}(puzzle.js の graspOptions の並び。既定は並びの順)
    if (a.height !== undefined && a.height !== null) { if (!(Number.isFinite(a.height) && a.height > 0)) return { ok: false, why: "height は正の数(mm)" }; base.HEIGHT = a.height; }
    if (a.fit_margin_mm !== undefined && a.fit_margin_mm !== null) { if (!(Number.isFinite(a.fit_margin_mm) && a.fit_margin_mm > 0)) return { ok: false, why: "fit_margin_mm は正の数(mm)" }; base._FIT_MARGIN = a.fit_margin_mm; }
    if (a.grasp !== undefined && a.grasp !== null) {
      if (typeof a.grasp !== "object" || Array.isArray(a.grasp) || Object.values(a.grasp).some(v => !Number.isInteger(v) || v < 0)) return { ok: false, why: "grasp は {ピースの名前: 0 以上の整数}" };
      base.GRASP = a.grasp;
    }
    const d0 = M.buildFromUi(base), c0 = d0._check;
    if (!c0.solved) return { ok: false, why: "既定の置き場で解けない: " + c0.why, verdict_kind: c0.verdict };
    let k1, sha = null;
    if (a.k1 === "fake") k1 = fakeK1();
    else {
      const buf = fs.readFileSync(a.k1); sha = crypto.createHash("sha256").update(buf).digest("hex"); k1 = JSON.parse(buf.toString("utf8"));
      if (k1.verified !== true) return { ok: false, why: "k1 の換算が確かめの姿勢で確かめられていない(tools\\k1_calib.py をやり直す)" };
    }
    const grip = M.gripFor(c0.cell, c0.height), map = ROBOT_MAP.make(k1, grip);
    if (!map.gripperOk) return { ok: false, why: "k1 に爪の測りが無い" };
    let calibCam = null, topFov = SO101.CAM_FOV.top, wristCal = null, wristFov = SO101.CAM_FOV.wrist;
    if (a.calib === "sim") {
      if (!d0.overheadCamera) return { ok: false, why: "上のカメラの台が無い(--stand 0 では上の絵を撮れない)" };
      calibCam = a.calibMm || a.calibDeg ? perturbCam(d0.overheadCamera, a.calibMm || 0, a.calibDeg || 0, V.rng((a.seed || 1) * 104729)) : d0.overheadCamera;
    } else if (a.calib && typeof a.calib === "object" && a.calib.top) {
      const t = a.calib.top, w = a.calib.wrist, v3 = x => Array.isArray(x) && x.length === 3 && x.every(Number.isFinite);
      if (!v3(t.tip) || !v3(t.dir) || !v3(t.up) || !(t.fov > 5 && t.fov < 150)) return { ok: false, why: "calib.top の形が違う(tip・dir・up は 3 つの数、fov は 5〜150 度)" };
      const nd = Math.hypot(...t.dir), nu = Math.hypot(...t.up), dot = (t.dir[0] * t.up[0] + t.dir[1] * t.up[1] + t.dir[2] * t.up[2]) / nd / nu;
      if (Math.abs(nd - 1) > 1e-3 || Math.abs(nu - 1) > 1e-3 || Math.abs(dot) > 1e-3) return { ok: false, why: `calib.top の dir・up が単位の直交でない(|dir| ${nd}・|up| ${nu}・内積 ${dot})` };
      calibCam = { tip: t.tip.slice(), dir: t.dir.slice(), up: t.up.slice() }; topFov = t.fov;
      if (t.panDeg) {   // 紙の座標 → 計画の座標: 肩の回転の軸の回りに −panDeg 回す(k1-err の shoulder_pan。紙の回しと 0 のずれの両方。sim/README の K5 の紙)
        const ax = SO101.axes[0], o = [ax.origin[0], ax.origin[1] + c0.robotY, ax.origin[2]], R = KIN.rotAA(ax.axis, -t.panDeg * Math.PI / 180);
        const d = KIN.matVec(R, calibCam.tip.map((v, i) => v - o[i]));
        calibCam = { tip: d.map((v, i) => v + o[i]), dir: KIN.matVec(R, calibCam.dir), up: KIN.matVec(R, calibCam.up) };
      }
      if (w) {
        if (!v3(w.t) || !(Array.isArray(w.R) && w.R.length === 9 && w.R.every(Number.isFinite)) || !(w.fov > 5 && w.fov < 150)) return { ok: false, why: "calib.wrist の形が違う(t は 3 つ、R は 9 つの数、fov は 5〜150 度)" };
        wristCal = { t: w.t.slice(), R: w.R.slice() }; wristFov = w.fov;
      }
    } else return { ok: false, why: "calib は \"sim\" か {top, wrist}" };
    S = { imgWhy: Math.abs(c0.height - c0.cell) > 1e-9 ? "高さが辺と違うピースは、絵の道具(描く・読む)が立方体だけなので扱えない(S1 は計画だけ)" : "", base, d0, c0, k1, map, calibCam, topFov, wristCal, wristFov, noise: a.noise ?? 3, seed: a.seed ?? 1, nShot: 0, sheet: !!a.sheet };   // sheet: 盤の下に K1/K5 の紙(vision.js の sheetTris)
    const start = c0.motion[0];
    return { ok: true, start_urdf: start.q, start_jaw: start.jaw, start_lerobot: map.toLeRobot(start.q, start.jaw), calibKey: map.calibKey,
             robot_id: k1.robot_id, k1_sha256: sha, cell_mm: c0.cell * 1000, ...(base.HEIGHT !== undefined ? { height_mm: c0.height * 1000 } : {}), pitch_mm: c0.pitch * 1000, fit_margin_mm: c0.grip.fitMargin * 1000,
             wrist_settle: c0.wristCheck ? c0.wristCheck.settle : null, pieces: c0.pieces.map(p => ({ idx: p.idx, name: p.name, shape: p.shape, color: p.color })),
             limits: map.limits, order: c0.stepTimes.map(s => s.pid) };
  },
  truth(a) {   // vision-loop.js の 1) と同じ引き方
    const r = V.rng(a.seed * 1000 + S.base.SET), c = S.c0, SH = a.shiftMm ?? 8, YW = a.yawDeg ?? 20;
    for (let g = 0; g < 50; g++) {
      const L = { pieces: c.pieces.map(p => ({ name: p.name, x: c.slots[p.idx][0] * 1000 + (r() * 2 - 1) * SH, z: c.slots[p.idx][1] * 1000 + (r() * 2 - 1) * SH, yaw: (c.slotRot[p.idx] || 0) + (r() * 2 - 1) * YW })) };
      if (M.buildFromUi(Object.assign({}, S.base, { LAYOUT: L, LAYOUT_ONLY: true }))._check.layoutWhy) continue;
      const cA = narrowed(() => M.buildFromUi(Object.assign({}, S.base, { LAYOUT: L }))._check);   // 計画と同じ可動域で組める物だけ(点検 C35 の 7)
      if (cA.ok) return { ok: true, layout: L, tries: g + 1 };
    }
    return { ok: false, why: "使える本当の置き場が 50 回で引けない" };
  },
  render(a) {
    if (S.imgWhy) return { ok: false, why: S.imgWhy };
    const u = trueQ(a.lerobot, a.bias), lay = slotsOf(a.truth);   // 絵は本当の腕の姿勢で撮る(関節の誤差 bias を足す)
    (a.hide || []).forEach(pid => { lay.slots[pid] = [5, 5]; });   // つかんだ・置いたピースは絵から外す(置いた所は描かない。見ていないもの)
    const occ = occluderTris(a.occluders);
    if (occ.why) return { ok: false, why: occ.why };
    const tris = V.worldTris(S.d0, lay, u.q, u.jaw ?? S.c0.homeJaw, { noCam: a.cam === "wrist" }).concat(S.sheet ? V.sheetTris() : [], occ.tris);
    const cam = a.cam === "wrist" ? VW.cameraAt(u.q, u.jaw ?? 0, S.c0.robotY) : S.d0.overheadCamera;
    const im = V.render(tris, cam, { fov: a.cam === "wrist" ? SO101.CAM_FOV.wrist : SO101.CAM_FOV.top, noise: S.noise, seed: S.seed * 7919 + (++S.nShot) * 31, ...(a.light ? { light: a.light } : {}) });   // light: S3 (vision.js opts.light)
    fs.mkdirSync(path.dirname(a.out), { recursive: true });
    fs.writeFileSync(a.out, Buffer.from(im.rgb.buffer, im.rgb.byteOffset, im.rgb.length));
    return { ok: true, W: im.W, H: im.H, out: a.out };
  },
  board(a) {   // {img, W, H} → {dx_mm, dz_mm, dyaw_deg, about_mm, n_edge, ...}。上の絵で盤のマスの縁を読み、較正のカメラで読んだ机の上の点を本当の盤に合わせる平面の回しとずらし(vision.js の readBoard)
    if (S.imgWhy) return { ok: false, why: S.imgWhy };
    const T = V.readBoard(readRgb(a.img, a.W, a.H), { cam: S.calibCam, fov: S.topFov }, { cells: S.c0.boardCells, cell: S.c0.cell });
    if (T.why) return { ok: false, why: T.why, n_edge: T.nEdge };
    if (T.edge) return { ok: false, why: `ずらしが探す幅(±${V.BOARD_SRCH_MM}mm)の端: カメラの置き方が大きく違う` };
    return Object.assign({ ok: true }, boardOut(T));
  },
  detect(a) {
    if (S.imgWhy) return { ok: false, why: S.imgWhy };
    const only = Array.isArray(a.only) ? a.only : null;
    const rgb = readRgb(a.img, a.W, a.H);
    const det = V.detect(rgb, { cam: S.calibCam, fov: S.topFov },
                         { pieces: S.c0.pieces.filter(p => !only || only.includes(p.idx)), pitch: S.c0.pitch, cell: S.c0.cell });
    if (det.why) return { ok: false, why: det.why, per: det.per };
    let board = null;
    if (a.board || a.board_fix) {   // 見つけたピースを盤に対する位置にする(較正のカメラの置き方の誤りを打ち消す。探さなかったピースは既定の置き場のまま)
      //   board: true = 同じ絵で盤を読む(盤が空いていないと読めない)。board_fix = 前に board で読んだ答え(カメラは動かないので、始めに 1 回読んだ物を使い回す)
      let T;
      if (a.board_fix) {
        const f = a.board_fix;
        if (![f.dx_mm, f.dz_mm, f.dyaw_deg].every(Number.isFinite) || !(Array.isArray(f.about_mm) && f.about_mm.length === 2 && f.about_mm.every(Number.isFinite)))
          return { ok: false, why: "board_fix は board の答え {dx_mm, dz_mm, dyaw_deg, about_mm}" };
        T = { dx: f.dx_mm / 1000, dz: f.dz_mm / 1000, yaw: f.dyaw_deg, about: f.about_mm.map(v => v / 1000) };
      } else {
        T = V.readBoard(rgb, { cam: S.calibCam, fov: S.topFov }, { cells: S.c0.boardCells, cell: S.c0.cell });
        if (T.why) return { ok: false, why: "盤を読めない: " + T.why };
        if (T.edge) return { ok: false, why: "盤を読んだずらしが探す幅の端(" + (T.dx * 1000).toFixed(1) + "," + (T.dz * 1000).toFixed(1) + "mm): カメラの置き方が大きく違う" };
      }
      Object.keys(det.layout.pieces).forEach(k => {
        const e = det.layout.pieces[k], o = V.boardApply(T, e.x / 1000, e.z / 1000, e.yaw);
        det.layout.pieces[k] = Object.assign({}, e, { x: +(o[0] * 1000).toFixed(2), z: +(o[1] * 1000).toFixed(2), yaw: +o[2].toFixed(2) });
      });
      board = a.board_fix ? { applied: a.board_fix } : boardOut(T);
    }
    const filled = [];
    S.c0.pieces.forEach(p => {   // 探さなかったピース: 既定の置き場にあることにする(机に無くても、腕はそこを避ける = 控えめ)
      if (det.layout.pieces[p.idx]) return;
      const s = S.c0.slots[p.idx];
      det.layout.pieces[p.idx] = { name: p.name, x: +(s[0] * 1000).toFixed(2), z: +(s[1] * 1000).toFixed(2), yaw: +(S.c0.slotRot[p.idx] || 0) };
      filled.push(p.idx);
    });
    return { ok: true, layout: det.layout, per: det.per, filled, board };
  },
  plan(a) {
    const ui = Object.assign({}, S.base, { LAYOUT: a.layout });
    if (a.air) ui._AIR = a.air / 1000;
    if (a.pieces) ui._PIECES = a.pieces;
    if (a.place_offset) {   // [{pid, dx_mm, dz_mm, dyaw_deg}]: 置く所だけをずらす(scene-data.js の _PLACE_OFF。つかむ所は変えない)
      if (!Array.isArray(a.place_offset) || a.place_offset.some(o => !Number.isInteger(o.pid) || !S.c0.pieces[o.pid] ||
          ["dx_mm", "dz_mm", "dyaw_deg"].some(k => o[k] !== undefined && !Number.isFinite(o[k]))))
        return { ok: false, why: "place_offset は [{pid, dx_mm, dz_mm, dyaw_deg}](pid はピースの番号、値は数)" };
      ui._PLACE_OFF = a.place_offset.map(o => ({ pid: o.pid, x: o.dx_mm || 0, z: o.dz_mm || 0, yaw: o.dyaw_deg || 0 }));
    }
    if (a.offset_is_arm !== undefined && a.offset_is_arm !== null) {   // true: place_offset は置く姿勢で腕がくり返す誤りを打ち消す分。置く段の当たりは本当の所(ずらす前)で測る
      if (typeof a.offset_is_arm !== "boolean") return { ok: false, why: "offset_is_arm は true か false" };
      ui._PLACE_OFF_ARM = a.offset_is_arm;
    }
    if (a.offset_perp_mm !== undefined && a.offset_perp_mm !== null) {   // offset_is_arm の窓で、爪の閉じる向きに直角に足す余裕(既定 4mm【仮】。点検 C54 の 1)
      if (!Number.isFinite(a.offset_perp_mm) || a.offset_perp_mm < 0) return { ok: false, why: "offset_perp_mm は 0 以上の数(mm)" };
      ui._PLACE_OFF_PERP_MM = a.offset_perp_mm;
    }
    if (a.arm_margin_mm !== undefined && a.arm_margin_mm !== null) {   // つかむ段・置く段で、爪のまわりと持っているピースを 8 つの向きへこれだけ動かしても、ほかのピースに当たらないこと(2026-11-22)
      if (!Number.isFinite(a.arm_margin_mm) || a.arm_margin_mm < 0) return { ok: false, why: "arm_margin_mm は 0 以上の数(mm)" };
      ui._ARM_MARGIN_MM = a.arm_margin_mm;
    }
    if (a.true_bias_deg !== undefined && a.true_bias_deg !== null) {   // (シミュレーションだけ) 本当の腕 = 言った角度 + これ(URDF の度、5 つ)。同じ段のほかのピースまでを答えに出す
      if (!Array.isArray(a.true_bias_deg) || a.true_bias_deg.length !== 5 || !a.true_bias_deg.every(Number.isFinite))
        return { ok: false, why: "true_bias_deg は 5 つの数(URDF の度)" };
      ui._TRUE_BIAS = a.true_bias_deg;
    }
    if (a.phase_clear !== undefined && a.phase_clear !== null && typeof a.phase_clear !== "boolean") return { ok: false, why: "phase_clear は true か false" };
    const mm = v => (isFinite(v) ? +(v * 1000).toFixed(3) : null);   // null = 判定が組を出さない遠さ(包む球のすき間 20mm 以上)
    const s1Out = c => !a.phase_clear ? {} : { verdict_kind: c.verdict, verdict_rule: c.verdictRule || "", min_clear_mm: mm(c.minClear), min_clear_pair: c.minClearPair, min_clear_t: c.minClearAt, dims_mm: c.dims ? Object.fromEntries(Object.entries(c.dims).map(([k, v]) => [k, k === "maxRun" ? v : mm(v)])) : null,
      phase_clear: (c.phaseClear || []).map(p => Object.fromEntries([["pid", p.pid], ["name", p.name]].concat(Object.keys(p).filter(k => p[k] && typeof p[k] === "object")
        .map(k => [k, { piece_mm: mm(p[k].pc), piece_pair: p[k].pcPair, piece_t: p[k].pcT, own_mm: mm(p[k].own), own_pair: p[k].ownPair, own_t: p[k].ownT, table_mm: mm(p[k].tb), n: p[k].n }])))) };   // S1: 段ごとのいちばん近い所
    const marginOut = c => ({ ...(c.armMargin ? { arm_margin_mm: c.armMargin.m * 1000, arm_margin_clear_mm: c.armMargin.d * 1000, arm_margin_pair: c.armMargin.pair, arm_margin_t: c.armMargin.t } : {}),
                              ...(c.trueClear ? { true_clear_mm: c.trueClear.d * 1000, true_clear_pair: c.trueClear.pair, true_clear_t: c.trueClear.t } : {}) });
    // 可動域は k1 の範囲との重なり(narrowed)
    const finish = (lay) => {   // その置き場で組み、断る(当たり・解けない・換算・範囲)か、関節の列を返す
      const d = narrowed(() => M.buildFromUi(Object.assign({}, ui, { LAYOUT: lay }))), c = d._check;
      if (!c.solved) return { ok: false, why: "解けない: " + (c.layoutWhy || c.why), layoutWhy: !!c.layoutWhy, ...s1Out(c) };
      if (!c.ok) {   // 断った中身(2026-11-20、隣の会話の列 P の頼み): 当たりの区間・どの手の間か・腕どうしのいちばん近い所
        //   どの手の間か: 待ちの姿勢に着くまで(tReady)は "start"、最後のピースを置いて上がった後は "return"(点検 C48 の 4)
        const st = c.stepTimes || [];
        const stepAt = t => { if (t < (c.tReady || 0) - 1e-9) return { phase: "start" };
                              const s = st.find(s => t <= s.tDone + 1e-9); return s ? { pid: s.pid, name: s.name } : { phase: "return" }; };
        const stepName = x => x.name ? `${x.name} の手` : x.phase === "start" ? "始めの待ちの姿勢まで" : "最後に戻る所";
        const self = c.self ? { min_mm: c.self.min * 1000, pair: c.self.pair, t: c.self.t, step: stepAt(c.self.t),
                                bound_mm: c.self.worstBound * 1000, bound_t: c.self.boundAt, bound_pair: c.self.boundPair, bound_step: stepAt(c.self.boundAt) } : null;
        const hits = (d.hits || []).map(h => ({ t0: h.t0, t1: h.t1, ids: h.ids, step: stepAt(h.t0) }));
        return { ok: false, why: `当たりがある: 当たり ${c.hitFrames || 0} コマ・腕どうし ${c.self ? (c.self.min * 1000).toFixed(1) + "mm" : "-"}`
                                 + (hits.length ? `(${hits.map(h => `${h.ids.join("|")} ${h.t0.toFixed(2)}〜${h.t1.toFixed(2)} 秒` + ` ${stepName(h.step)}`).join("・")})` : ""),
                 hits, first_hit: c.firstHit ? { t: c.firstHit.t, ids: c.firstHit.ids, bad: c.firstHit.bad } : null, self,
                 verdict: (d.info && d.info[0] && d.info[0].value) || "", ...marginOut(c), ...s1Out(c) };   // 画面の表の「判定」の文(点検 C48 の 3: _check.why は当たりのときは空)
      }
      const frames = [];
      for (const f of c.motion) {
        const L = S.map.toLeRobot(f.q, f.jaw), back = S.map.toUrdf(L);
        const e = Math.max(...back.q.map((v, i) => Math.abs(v - f.q[i])), Math.abs(back.jaw - f.jaw));
        if (!(e < ROUNDTRIP_TOL_DEG)) return { ok: false, why: `換算して戻すと ${e}° 違う(${f.t.toFixed(2)} 秒)` };
        const out = S.map.outOfRange(L);
        if (out.length) return { ok: false, why: `キャリブレーションの範囲の外(${f.t.toFixed(2)} 秒): ` + out.map(m => `${m} ${L[m].toFixed(1)}`).join(", ") };
        frames.push({ t: f.t, q: f.q, jaw: f.jaw, held: f.held, L });
      }
      return { ok: true, layout: lay, sym: [], frames, steps: c.stepTimes, grasps: c.grasps, slots_mm: c.slots.map(s => [s[0] * 1000, s[1] * 1000]), slot_rot: c.slotRot,
               min_tip_mm: c.minTip * 1000, self_min_mm: c.self ? c.self.min * 1000 : null, T: c.T, air_mm: a.air || 0,
               ...(ui._PLACE_OFF_ARM && ui._PLACE_OFF ? { offset_is_arm: true, offset_perp_mm: ui._PLACE_OFF_PERP_MM ?? 4 } : {}), ...marginOut(c), ...s1Out(c) };   // 点検 C54 の 7: どちらの座標で測ったか
    };
    const r0 = finish(a.layout);
    if (r0.ok || r0.layoutWhy || a.sym === false || !a.layout || !Array.isArray(a.layout.pieces)) return r0;
    //   断ったら(置き場そのものの断りを除く)、対称なピースのほかの表し方を 1 つずつ試し、通った最初の物を使う(2026-11-21・点検 C52。O は
    //   つかむ爪の向きが表し方の向きのままなので、同じ置き場でも表し方で腕どうしが当たる: 種 1〜8 の O で 4 組。sim/README)。断らなかった計画は前と同じ。
    //   試すピース: sym_pids があればその順(手首で直した後の組み直しは、直したピースだけ)、無ければ keep でないピースの番号の順。2 つの組み合わせは試さない
    const pids = Array.isArray(a.sym_pids) ? a.sym_pids : a.layout.pieces.map((e, i) => i);
    let tried = 0;
    for (const i of pids) {
      const e = a.layout.pieces[i];
      if (!e || e.keep || !S.c0.pieces[i]) continue;
      for (const r of symReps(S.c0.pieces[i].shape, e)) {
        const L2 = JSON.parse(JSON.stringify(a.layout)); L2.pieces[i] = r; tried++;
        const r2 = finish(L2);
        if (r2.ok) return Object.assign(r2, { sym: [{ pid: i, name: e.name, from: { x: e.x, z: e.z, yaw: e.yaw }, to: { x: r.x, z: r.z, yaw: r.yaw } }], sym_tried: tried });
      }
    }
    return Object.assign(r0, { sym_tried: tried }, tried ? { why: r0.why + `(対称なほかの表し方 ${tried} 通りも断った)` } : {});   // ほかの表し方も全部断った
  },
  //   手首のカメラの置き方は、読んだ角度から出す(本当の腕の姿勢は知らない。関節の誤差があれば、そのぶん測りがずれる = 実機と同じ)。
  //   もっともらしさ(点検 C35 の 2): 合わせの cost(点が形の外へ出る量の 2 乗の重み付き平均・m²)が COST_MAX を超えたら、測れなかったとする
  wrist(a) {
    if (S.imgWhy) return { ok: false, why: S.imgWhy };
    const COST_MAX = 1e-7;   // 【仮】正しく合ったときは 1e-17〜1e-9、隠れたマスに 1 ピッチずれて合ったとき 1.2e-8、向きを 40° 外したとき 1e-5(点検 C35 の 2 の作り物の点)。
                             //   1 ピッチずれは cost では分けられないので、puzzle_run.py が直す量(ピッチの半分)で止める
    const u = toUrdfQ(a.lerobot), p = S.c0.pieces[a.pid];
    const cam = VW.cameraAt(u.q, u.jaw ?? 0, S.c0.robotY, S.wristCal);
    const r = VW.detectOne(readRgb(a.img, a.W, a.H), cam, p, { pitch: S.c0.pitch, cell: S.c0.cell }, S.wristFov, a.est);
    if (r.why) return { ok: false, why: r.why, nPix: r.nPix };
    if (!(r.cost <= COST_MAX)) return { ok: false, why: `手首の絵で ${p.name} の形が合わない(cost ${r.cost.toExponential(2)} > ${COST_MAX})`, nPix: r.nPix, cost: r.cost };
    return { ok: true, piece: { name: p.name, x: r.x, z: r.z, yaw: r.yaw }, nPix: r.nPix, nTop: r.nTop, cost: r.cost, top_frac: r.topFrac };
  },
  match(a) {
    const p = S.c0.pieces[a.pid], e = a.est, m = a.meas;
    const mt = V.match(p.shape, S.c0.pitch, [e.x / 1000, e.z / 1000, e.yaw], [m.x / 1000, m.z / 1000, m.yaw]);
    const o = mt.T.apply([e.x / 1000, e.z / 1000]);
    return { ok: true, worst_cell_mm: mt.worstCell * 1000, phi: mt.phi, fixed: Object.assign({}, e, { name: e.name, x: +(o[0] * 1000).toFixed(2), z: +(o[1] * 1000).toFixed(2), yaw: +(e.yaw + mt.phi).toFixed(2) }) };
    //   fixed は est のほかの印(on_board・keep)をそのまま持つ(点検 C48 の 1: 手首で直すと on_board が落ち、直した計画が「盤に重なる」で断られた)
  },
  map(a) {   // (S2。2026-11-22) 換算だけ(robot-map.js): {lerobot} → {urdf: 5 つの度, jaw}、または {urdf, jaw} → {lerobot}。tools/puzzle_run.py --mj の世界(MuJoCo の関節は URDF)が使う
    if (a.lerobot && typeof a.lerobot === "object") { const u = toUrdfQ(a.lerobot); return { ok: true, urdf: u.q, jaw: u.jaw }; }
    if (Array.isArray(a.urdf) && a.urdf.length === 5 && a.urdf.every(Number.isFinite) && Number.isFinite(a.jaw)) return { ok: true, lerobot: S.map.toLeRobot(a.urdf, a.jaw) };
    return { ok: false, why: "map は {lerobot} か {urdf: 5 つの度, jaw}" };
  },
  simcalib() {   // sim の絵と同じカメラ(render が使う物)。W・H は vision.js render の既定
    const oc = S.d0.overheadCamera;
    return { ok: true, top: { tip: oc.tip, dir: oc.dir, up: oc.up, fov: SO101.CAM_FOV.top, W: 640, H: 480 },
             wrist: Object.assign(VW.urdfCamInGripper(), { fov: SO101.CAM_FOV.wrist, W: 640, H: 480 }) };
  },
  //   つかんだ瞬間の本当の腕の姿勢(k.lerobot と bias)から、道具の点と爪の向きを順運動学で出し、本当のピースのつかむ所と比べる(vision-loop.js の 5) と同じ e_a・e_c)。
  //   合否: |e_a| + 辺/2·|sin(向きのずれ)| < FIT_MARGIN(固定の爪がピースに乗らない)、かつ |e_c| < EC_MAX_MM【仮】
  //   (e_c は挟んでも消えず、置いた所にそのまま残る。点検 C35 の 1: e_c 5mm でも合格にしていた。2mm は S-8 の置く段の上限 3.1mm(O)より内側)。
  //   見ていないもの: 本当の置き場での当たり判定(判定は見積もりの置き場で)、置いたピースのずれの積み重なり
  score(a) {
    const EC_MAX_MM = 2.0, PLACE_MAX_MM = 3.0;   // PLACE_MAX_MM【仮】: 置いた所のマスのいちばん遠い差(tools/puzzle_mj.py の画像の確認の 3mm と同じ)
    const rows = a.picks.map(k => {
      const p = S.c0.pieces[k.pid], e0 = k.est, t = a.truth.pieces[k.pid];
      const m = V.match(p.shape, S.c0.pitch, [e0.x / 1000, e0.z / 1000, e0.yaw], [t.x / 1000, t.z / 1000, t.yaw]);
      const pT = m.T.apply([k.cS[0], k.cS[2]]);                       // 本当のピースの、つかむつもりだった所
      let tcp = [k.cS[0], k.cS[2]], yp = k.yp;
      if (k.lerobot && a.bias) {                                        // 本当の腕(読んだ角度 + 誤差)は、計画の姿勢(読んだ角度そのまま)から
        const uP = trueQ(k.lerobot, null), uT = trueQ(k.lerobot, a.bias);   //   どれだけ動いたか。つかむ所 cS を、手先の動き(平行移動 + 回転)で動かす
        const sP = KIN.fk(SO101.axes, uP.q, SO101.tool), sT = KIN.fk(SO101.axes, uT.q, SO101.tool);
        const dy = MOTION.wrap180(2 * (MOTION.jawYawDeg(uT.q) - MOTION.jawYawDeg(uP.q))) / 2, r = dy * Math.PI / 180;
        const v = [k.cS[0] - sP.pos[0], k.cS[2] - sP.pos[2]];
        tcp = [sT.pos[0] + v[0] * Math.cos(r) - v[1] * Math.sin(r), sT.pos[2] + v[0] * Math.sin(r) + v[1] * Math.cos(r)];
        yp = k.yp + dy;
      }
      const e = [pT[0] - tcp[0], pT[1] - tcp[1]];
      const u = [Math.cos(yp * Math.PI / 180), Math.sin(yp * Math.PI / 180)], ea = e[0] * u[0] + e[1] * u[1], ec = -e[0] * u[1] + e[1] * u[0];
      const dyaw = MOTION.wrap180(2 * (yp - k.yp)) / 2;                // 爪の向きの誤差。爪とピースの向きの差 = ピースの向きのずれ − これ
      const need = Math.abs(ea) + S.c0.cell / 2 * Math.abs(Math.sin((m.phi - dyaw) * Math.PI / 180));
      // 置いた所(2026-11-20。隣の会話の列 P の頼み 4: --arm-err ±2° で、つかむ所は合格でも MuJoCo の置いた所は 2.5〜6mm ずれた)。
      //   挟んだ後のピース(点検 C47 の 1。vision-loop.js の 5) の仮定と同じ): 閉じる向きのずれと向きのずれは平らな爪で挟むと消え、直角のずれ e_c だけ残る。
      //   = 見積もりのピースのマスを、計画の手(読んだ角度そのまま)に対する計画どおりの位置で本当の手(読んだ角度 + bias)へ移し、爪に直角へ e_c だけずらした物。
      //   それを本当の手のつかむ姿勢 → 置く姿勢の動きで運ぶ。比べる相手は、見積もりのマスを計画の手の同じ動きで運んだ物(= 計画の置いた所)。
      //   マスどうしのいちばん遠い差(対称の組は match と同じ)。place_rigid_mm は、挟んでもピースが動かない(つかんだ瞬間の本当の置き場のまま運ぶ)とした値(比べる用)。
      //   手の動きは水平の面の上の剛体(道具の点 gripper_frame の x・z と、その x 軸を水平に写した向き。腕の傾きは見ない)。
      //   見ていないもの: 閉じるときにピースが机の上をすべる・回る途中の当たり、開くときに動く分、置いた後のすべり、手の傾き
      let place_mm = null, place_rigid_mm = null, placed_true = null;
      if (k.place_lerobot) {
        const hand = (L, bias) => { const u = trueQ(L, bias), f = SO101.urdfFK(u.q.map(d => d * Math.PI / 180), (u.jaw || 0) * Math.PI / 180).gripper_frame;
                                    return { o: [f.p[0], f.p[2]], a: Math.atan2(f.R[6], f.R[0]) }; };
        const move = (H1, H2, pts) => { const r = H2.a - H1.a, c = Math.cos(r), s = Math.sin(r);
                                        return pts.map(q => { const x = q[0] - H1.o[0], z = q[1] - H1.o[1]; return [H2.o[0] + x * c - z * s, H2.o[1] + x * s + z * c]; }); };
        const A = V.cellsOf(p.shape, S.c0.pitch, e0.x / 1000, e0.z / 1000, e0.yaw), B = V.cellsOf(p.shape, S.c0.pitch, t.x / 1000, t.z / 1000, t.yaw);
        const hP1 = hand(k.lerobot, null), hT1 = hand(k.lerobot, a.bias), hP2 = hand(k.place_lerobot, null), hT2 = hand(k.place_lerobot, a.bias);
        const pl = move(hP1, hP2, A), n = [-u[1], u[0]];
        const sq = move(hP1, hT1, A).map(q => [q[0] + ec * n[0], q[1] + ec * n[1]]);
        place_mm = V.matchPts(p.shape, pl, move(hT1, hT2, sq)).worstCell * 1000;
        place_rigid_mm = V.matchPts(p.shape, pl, move(hT1, hT2, B)).worstCell * 1000;
        //   挟んだ後のピースの置き場(形の [0,0] のマス・向き)。puzzle_run.py --sim --place-check が、置いた後の世界(絵)をこれで描く
        const o1 = move(hP1, hT1, [[e0.x / 1000, e0.z / 1000]])[0], o2 = move(hT1, hT2, [[o1[0] + ec * n[0], o1[1] + ec * n[1]]])[0];
        placed_true = { name: p.name, x: o2[0] * 1000, z: o2[1] * 1000, yaw: e0.yaw + (hT2.a - hP1.a) * 180 / Math.PI };
      }
      return { pid: k.pid, name: p.name, e_a_mm: ea * 1000, e_c_mm: ec * 1000, phi: m.phi - dyaw, need_mm: need * 1000, place_mm, place_rigid_mm, placed_true,
               ok: need < S.c0.grip.fitMargin && Math.abs(ec) * 1000 < EC_MAX_MM && (place_mm === null || place_mm < PLACE_MAX_MM) };
    });
    return { ok: true, rows, all_ok: rows.every(r => r.ok), fit_margin_mm: S.c0.grip.fitMargin * 1000, ec_max_mm: EC_MAX_MM, place_max_mm: PLACE_MAX_MM };
  },
};

const rl = readline.createInterface({ input: process.stdin });
rl.on("line", line => {
  let res;
  try {
    const a = JSON.parse(line);
    if (!cmds[a.cmd]) res = { ok: false, why: "知らない頼み: " + a.cmd };
    else if (a.cmd !== "init" && !S) res = { ok: false, why: "init が先" };
    else res = cmds[a.cmd](a);
  } catch (e) { res = { ok: false, why: "plan-cli の誤り: " + (e && e.stack || e) }; }
  process.stdout.write(JSON.stringify(res) + "\n");
});
