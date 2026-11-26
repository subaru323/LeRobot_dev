// 許せるずれの表(計画書_完成形まで の S-8 の後半。V-3 の「いちばん危ない前提」を、向きごとの数にする)。
//   1 つのピースの 1 段だけ、腕の狙いをずらして解かせ(scene-data.js の _ERR)、その段で当たらずに済むずれの上限を、向きごとに探す。
//     段: pick = つかむ場所の真上に着く〜閉じる(下ろす・横に寄せる・閉じる)。place = 盤の真上から下ろす〜離して上へ戻る。
//     向き: 爪の向き u に対して 8 方向(0° = +a = 固定の爪の側へ、90° = +c)。a・c は tools/vision-loop.js の e_a・e_c と同じ。
//     ほかに、位置はそのままで爪の向きだけを ± 回す(yaw)。
//   上限: 0.5mm(向きは 1°)おきに大きくして、当たりが出た所と出ない所の間を 4 回半分にする(0.03mm・0.06° まで)。
//   「当たり」は、その段の時間のふつうの組(腕・持っているピース ⇄ ほかのピース・机・台)のいちばん近い所が 0 未満。
//     触れてよい組(つかむピース ⇄ 爪)は見ない:つかむ段で固定の爪がピースを押すのは、ピースが滑るだけと読む【仮】。
//     腕どうし(self-collision)はその段だけ測って、負なら当たりに数える。
//   ずらしたとき、選ばれた爪の向き(yp)や見方が名目と変わったら「試し方が変わった」と書く(その数は名目の動きのずれではない)。
//   見ていないもの: ずれがあるとき実物でピースが滑る・回る動き(sim のピースは爪に対して名目の所に付いたまま)。
//     判定の甘さ(ピースどうし・台は点の間 最大 2mm ほど。爪のまわりとピースは三角形で測り直す。コマの間の割り方 約 0.6mm)。
//     ずれが段の中で変わる場合(いまは段のあいだ一定)。
//   位置と向きを一緒にずらす(--yaws。2026-11-18 の 2 つ目の表): 向きを決めた値に固定し、その上で位置の上限を向きごとに探す。
//     a・c は名目の爪の向き u の座標(ずらした向きの u ではない)。向きは scene-data.js の作りのまま、ピースを運ぶ道の真ん中の回りで回す
//     (爪の真ん中・道具の点の回りではない。ずれの意味が違う。点検 C33 の 6)。
//   実行: node tools/allow-err.js [--set 2] [--gap 15] [--cell 14] [--max 6] [--pieces 0,1]  → out/allow-err-set<N>-gap<G>-cell<C>.json
//     絞る: --dirs 0,90,180,270(向き)・--step 1(位置の刻み mm)・--phases pick,place・--noyaw(向きだけの ± を探さない)
//     一緒にずらす: --yaws -15,-10,-5,0,5,10,15(向きごとに位置の上限。向きだけの ± は探さない)
//     --tag <名前> → ファイル名の末尾に -<名前> を足す(1 つ目の表を上書きしない)
//     scene-data.js の口(2026-11-18 に本体の会話が足した。点検 C37 の頼み): --pivot tcp(向きを道具の点の回りで回す)・
//       --carry(つかむ段の c のずれを持ったまま置く段まで運ぶ。当たりはつかむ〜置いて上へ戻るまで測る。a は閉じると消えるので運ばない)・
//       --fit 2.5(ui._FIT_MARGIN mm。つかむとき固定の爪をピースから離す量と、爪を開く余り。既定 2。指が入るすき間も 指 + 2×これ に変わる)・
//       --prior 0:0:1:0,1:0:-1:0(先に置くピース pid を、置く向きの a:c(mm):yaw(度)だけずらして置く)
//     1 行ごとに json を書き直す。--resume でそのファイルの済んだ行(同じ pid・段・向き、同じ条件 opts・同じ scene-data の版)を飛ばす
//     --nominal → 名目だけ組んで、解けたか・_check.ok・腕どうし(self.min)を書いて終わる(すき間・辺を選ぶ表の「名目で NG」の出どころ。点検 C37 の 3)
//     --compare <out の json> → 向き 0 の行を、そのファイルの同じ段・向きと比べる(差が --tol mm(既定 0.07)を超えたら NG)
const { load } = require("./_load");
const fs = require("fs"), path = require("path"), crypto = require("crypto");
load("kinematics.js", "so101.js", "motion.js", "puzzle.js", "models/so101_visuals.js", "models/so101_points.js", "models/so101_tris.js",
     "collide.js", "chikuwa-collision.js", "self-collision.js");
global.window = { SCENE_SKIP_DEFAULT: 1 }; load("scene-data.js"); delete global.window;
// scene-data.js は別の会話が書き換えている最中のことがある。どの版で測ったかを残す
const sceneSrc = fs.readFileSync(path.join(__dirname, "..", "scene-data.js"));
const sceneVer = { sha1: crypto.createHash("sha1").update(sceneSrc).digest("hex").slice(0, 12), mtime: fs.statSync(path.join(__dirname, "..", "scene-data.js")).mtime.toISOString() };
const M = SCENE_MODEL;
const argv = process.argv.slice(2), arg = k => { const i = argv.indexOf("--" + k); return i >= 0 ? argv[i + 1] : undefined; }, flag = k => argv.includes("--" + k);
const nums = s => s.split(",").map(Number);
const def = M.defaultUiValues();
const SET = +(arg("set") ?? 2), GAP = +(arg("gap") ?? def.GAP), CELL = arg("cell") !== undefined ? +arg("cell") : undefined;
const MAXMM = +(arg("max") ?? 6), MAXDEG = +(arg("maxdeg") ?? 15), STEP = +(arg("step") ?? 0.5);
const DIRS = arg("dirs") ? nums(arg("dirs")) : [0, 45, 90, 135, 180, 225, 270, 315];
const PHASES = arg("phases") ? arg("phases").split(",") : ["pick", "place"];
const YAWS = arg("yaws") ? nums(arg("yaws")) : null;          // 一緒にずらす向き(度)。null = 1 つ目の表と同じ(向き 0 で位置 + 向きだけの ±)
const DO_YAW = !YAWS && !flag("noyaw");
const FIT = arg("fit") !== undefined ? +arg("fit") : undefined;
const PIVOT = arg("pivot"), CARRY = flag("carry"), RESUME = flag("resume");
const PRIOR = arg("prior") ? arg("prior").split(",").map(t => { const [pid, a, c, yaw] = nums(t.replace(/:/g, ",")); return { pid, a, c, yaw: yaw || 0 }; }) : null;
const TAG = arg("tag"), CMP = arg("compare"), TOL = +(arg("tol") ?? 0.07);
const base = Object.assign({}, def, { SET, GAP, SOL: 0, NO_RESLOT: true }, CELL !== undefined ? { CELL } : {}, PRIOR ? { _ERR_PRIOR: PRIOR } : {}, FIT !== undefined ? { _FIT_MARGIN: FIT } : {});
const t0 = Date.now();
const nom = M.buildFromUi(base)._check;
const fileFor = cellMm => path.join(__dirname, "..", "out", `allow-err-set${SET}-gap${GAP}-cell${cellMm}${TAG ? "-" + TAG : ""}.json`);
const nomRec = { solved: !!nom.solved, ok: nom.ok, why: nom.why || "", selfMin: nom.self ? +(nom.self.min * 1000).toFixed(2) : null, selfPair: nom.self ? nom.self.pair : null };
if (flag("nominal")) {
  console.log(`名目: 解けた ${nomRec.solved}・ok ${nomRec.ok}・腕どうし ${nomRec.selfMin}mm(${nomRec.selfPair})${nomRec.why ? "・" + nomRec.why : ""}`);
  fs.mkdirSync(path.join(__dirname, "..", "out"), { recursive: true });
  const c = CELL !== undefined ? CELL : def.CELL;
  fs.writeFileSync(path.join(__dirname, "..", "out", `allow-err-set${SET}-gap${GAP}-cell${c}${FIT !== undefined ? "-fit" + FIT : ""}-nominal.json`),
    JSON.stringify({ set: SET, gap: GAP, cell: c, fit: FIT, nom: nomRec, scene: sceneVer }, null, 1));
  process.exit(0);
}
if (!nom.solved) {   // 解けない組も、理由をファイルに残す(すき間・辺を選ぶ表で「解けない」も答えのうち)
  console.log("名目で解けない: " + nom.why);
  fs.mkdirSync(path.join(__dirname, "..", "out"), { recursive: true });
  const c = CELL !== undefined ? CELL : def.CELL;
  fs.writeFileSync(fileFor(c), JSON.stringify({ set: SET, gap: GAP, cell: c, solved: false, why: nom.why, scene: sceneVer }, null, 1));
  process.exit(2);
}
const pids = arg("pieces") ? nums(arg("pieces")) : nom.stepTimes.map(s => s.pid);

function run(pid, phase, a, c, yaw) {
  const E = { pid, phase, a, c, yaw };
  if (PIVOT) E.pivot = PIVOT;
  if (CARRY && phase === "pick") E.carry = true;
  const ch = M.buildFromUi(Object.assign({}, base, { _ERR: E }))._check;
  if (!ch.solved || !ch.err) return { ok: false, solved: false, why: ch.why };
  const e = ch.err, selfMin = ch.self ? ch.self.min : Infinity;
  return { ok: e.d >= 0 && !e.hits && selfMin >= 0, solved: true, d: e.d, pair: e.pair, hitPairs: e.hitPairs, selfMin, selfPair: ch.self && ch.self.pair, yp: e.yp, view: e.view };
}
// 0 から step おきに大きくし、当たりの境を 4 回半分にして詰める。f(r) → run の結果
function limit(f, step, max) {
  let lo = 0, hi = null, loR = f(0), hiR = null;
  if (!loR.ok) return { lim: 0, at0: loR, by: loR };
  for (let r = step; r <= max + 1e-9; r += step) { const R = f(r); if (R.ok) { lo = r; loR = R; } else { hi = r; hiR = R; break; } }
  if (hi === null) return { lim: Infinity, last: loR };
  for (let k = 0; k < 4; k++) { const m = (lo + hi) / 2, R = f(m); if (R.ok) { lo = m; loR = R; } else { hi = m; hiR = R; } }
  return { lim: lo, by: hiR };
}
const why = R => !R ? "" : !R.solved ? "解けない(" + (R.why || "").slice(0, 40) + ")" : R.selfMin < 0 ? "腕どうし " + R.selfPair : (R.hitPairs && R.hitPairs.length ? R.hitPairs.join("・") : R.pair);
// JSON は Infinity を null にするので、上限が --max を超えた向きは over: true も書く
const limRec = (L, ch) => ({ lim: isFinite(L.lim) ? L.lim : null, over: !isFinite(L.lim), by: why(L.by), changed: ch || undefined });
const out = { set: SET, gap: GAP, cell: nom.cell * 1000, fitMargin: nom.grip.fitMargin * 1000, scene: sceneVer, nom: nomRec,
              opts: { max: MAXMM, maxdeg: MAXDEG, step: STEP, dirs: DIRS, phases: PHASES, yaws: YAWS, yawOnly: DO_YAW, pivot: PIVOT || "center", carry: CARRY, prior: PRIOR }, rows: [] };
const file = fileFor((nom.cell * 1000).toFixed(0));
// 前の走りの行を使うのは、条件(opts)と scene-data の版が同じときだけ(点検 C37 の 8)
const prev = RESUME && fs.existsSync(file) ? JSON.parse(fs.readFileSync(file, "utf8")) : null;
const done0 = prev && prev.scene && prev.scene.sha1 === sceneVer.sha1 && JSON.stringify(prev.opts) === JSON.stringify(out.opts) ? prev.rows || [] : [];
if (prev && !done0.length) console.log("--resume: 前のファイルは条件か scene-data の版が違うので使わない");
const save = () => { fs.mkdirSync(path.dirname(file), { recursive: true }); out.minutes = +((Date.now() - t0) / 60000).toFixed(1); fs.writeFileSync(file, JSON.stringify(out, null, 1)); };
console.log(`[許せるずれ] 種 ${SET}・すき間 ${GAP}mm・辺 ${(nom.cell * 1000).toFixed(1)}mm・FIT_MARGIN ${(nom.grip.fitMargin * 1000).toFixed(1)}mm・scene-data ${sceneVer.sha1}。` +
            `名目 ok ${nomRec.ok}・腕どうし ${nomRec.selfMin}mm(${nomRec.selfPair})。向きは爪の u から(0° = +a 固定の爪の側・90° = +c)・刻み ${STEP}mm・上限 ${MAXMM}mm${YAWS ? "・向きを " + YAWS.join("/") + "° に固定して位置を探す" : ""}`);
let changed = 0;
const f2 = v => v.over ? ">" + MAXMM : v.lim.toFixed(2);
for (const pid of pids) {
  const st = nom.stepTimes.find(s => s.pid === pid);
  for (const phase of PHASES) {
    const R0 = run(pid, phase, 0, 0, 0);
    for (const yaw of (YAWS || [0])) {
      const old = done0.find(r => r.pid === pid && r.phase === phase && (YAWS ? r.yawFix === yaw : r.yawFix === undefined) && DIRS.every(th => r.dirs[th]));
      if (old) { out.rows.push(old); console.log(`${st.name} ${phase} 向き ${yaw}°: 前の走りの行を使う(--resume)`); continue; }
      const row = { pid, name: st.name, phase, d0: R0.d, pair0: R0.pair, dirs: {} };
      if (YAWS) row.yawFix = yaw;   // 固定した向き(度)
      if (DO_YAW) row.yaw = {};     // 向きだけの ±
      const note = [];
      let chDir = false;
      const chk = R => { if (R && R.solved && (R.yp !== R0.yp || R.view !== R0.view)) { changed++; chDir = true; note.push("試し方が変わった"); } return R; };
      for (const th of DIRS) {
        const ca = Math.cos(th * Math.PI / 180), sa = Math.sin(th * Math.PI / 180);
        chDir = false;
        const L = limit(r => chk(run(pid, phase, +(r * ca).toFixed(4), +(r * sa).toFixed(4), yaw)), STEP, MAXMM);
        row.dirs[th] = limRec(L, chDir);
      }
      if (DO_YAW) for (const sg of [1, -1]) {
        chDir = false;
        const L = limit(r => chk(run(pid, phase, 0, 0, sg * r)), 1, MAXDEG);
        row.yaw[sg > 0 ? "+" : "-"] = limRec(L, chDir);
      }
      row.note = [...new Set(note)];
      const ws = DIRS.map(th => row.dirs[th]).filter(v => !v.over).map(v => v.lim);
      row.worst = ws.length ? Math.min(...ws) : null;
      out.rows.push(row);
      save();
      console.log(`${st.name} ${phase === "pick" ? "つかむ" : "置く"}${YAWS ? ` 向き ${yaw >= 0 ? "+" : ""}${yaw}°` : ""}: 名目のいちばん近い ${(R0.d * 1000).toFixed(2)}mm(${R0.pair})`);
      console.log("  " + DIRS.map(th => `${th}° ${f2(row.dirs[th])}`).join(" / ") + `  いちばん狭い ${row.worst !== null ? row.worst.toFixed(2) : ">" + MAXMM}mm`);
      console.log("  当たる組: " + DIRS.map(th => `${th}° ${row.dirs[th].by || "-"}`).join(" / "));
      if (DO_YAW) console.log(`  向きだけ: + ${row.yaw["+"].over ? ">" + MAXDEG : row.yaw["+"].lim.toFixed(1)}°(${row.yaw["+"].by || "-"})・− ${row.yaw["-"].over ? ">" + MAXDEG : row.yaw["-"].lim.toFixed(1)}°(${row.yaw["-"].by || "-"})${row.note.length ? " ★" + row.note.join("・") : ""}`);
      else if (row.note.length) console.log("  ★" + row.note.join("・"));
    }
  }
}
// 検算 1: つかむ段の +a(固定の爪の側)の上限は、名目でその段の固定の爪(j5)とつかむピースの近さとほぼ同じはず(下ろすとき固定の爪を FIT_MARGIN 離す)。向き 0 の行だけ
let bad = 0;
out.rows.filter(r => r.phase === "pick" && r.dirs[0] && (YAWS ? r.yawFix === 0 : true)).forEach(r => {
  const lim = r.dirs[0].lim, d0 = r.d0 * 1000;
  if (/^j5\|pc/.test(r.pair0) && (lim === null || Math.abs(lim - d0) > 0.3)) { bad++; console.log(`  ★検算 NG: ${r.name} のつかむ段 +a の上限 ${lim === null ? ">" + MAXMM : lim.toFixed(2)}mm と名目の近さ ${d0.toFixed(2)}mm が 0.3mm より離れている`); }
  else if (/^j5\|pc/.test(r.pair0)) console.log(`  検算 1 OK: ${r.name} のつかむ段 +a ${lim.toFixed(2)}mm ≒ 名目の固定の爪とピース ${d0.toFixed(2)}mm`);
});
// 検算 2(--compare): 向き 0 の行が、前の表の同じ段・向きと合うか(どちらも上限超えなら合う)
if (CMP) {
  const ref = JSON.parse(fs.readFileSync(CMP, "utf8"));
  let n = 0;
  out.rows.filter(r => (YAWS ? r.yawFix === 0 : true)).forEach(r => {
    const q = ref.rows.find(x => x.pid === r.pid && x.phase === r.phase && (x.yawFix === undefined || x.yawFix === 0));
    if (!q) return;
    DIRS.forEach(th => {
      const a = r.dirs[th], b = q.dirs[th];
      if (!b) return;
      n++;
      const aOver = a.over, bOver = b.lim === null || b.over;
      const okk = aOver && bOver || (!aOver && !bOver && Math.abs(a.lim - b.lim) <= TOL);
      if (!okk) { bad++; console.log(`  ★検算 2 NG: ${r.name} ${r.phase} ${th}° いま ${f2(a)} / 前の表 ${bOver ? ">" + (ref.opts ? ref.opts.max : 6) : b.lim.toFixed(2)}`); }
    });
  });
  console.log(`  検算 2: 向き 0 の ${n} 個を ${path.basename(CMP)} と比べた(許す差 ${TOL}mm)`);
}
out.done = true;
save();
console.log(`${changed ? "★試し方が変わった回 " + changed + "(その向きの数は名目の動きではない)。" : ""}${bad ? "検算 NG " + bad : "検算 OK"}・${out.minutes} 分・${path.relative(process.cwd(), file)}`);
process.exit(bad ? 1 : 0);
