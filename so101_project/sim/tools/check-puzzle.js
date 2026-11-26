// 敷き詰めの速い版(puzzle.js の solveX)が、前からの版(solve)と同じ答えの集まりを出すかを確かめ、大きい問題での速さを比べる。
//   1. 既定の 4 種・穴のある形・同じ形のピースが多い組と、でたらめに作った小さい目標(ピースもでたらめ)で、両方の答えを全部集め、集まりと数が一致するか。速い版の答えの中身(k と origin から作ったマスが cells と同じ・重ならずに目標をちょうど埋める)も見る
//   2. 負の検算: solveX の答えから 1 つ抜いた集まり・1 マスずらした答えは「一致しない」と出るか、origin をずらした答え・ピースを落とした答えを中身の検査が捕まえるか
//   3. 大きい問題(ペントミノの長方形・形の目標)で、最初の答えまでの時間。前の版は打ち切り時間つきで別のプロセスで流す
//   実行: node tools/check-puzzle.js [--quick]
const { load } = require("./_load");
const { execFileSync } = require("child_process");
const path = require("path");
load("puzzle.js");
const P = PUZZLE, quick = process.argv.includes("--quick");

// 前の版を打ち切り時間つきで流す(子プロセス)
if (process.argv[2] === "--old") {
  const job = JSON.parse(process.argv[3]);
  const t0 = Date.now(), r = P.solve(job.target, job.pieces, job.max);
  console.log(JSON.stringify({ n: r.solutions.length, ms: Date.now() - t0 }));
  process.exit(0);
}

const sig = sol => sol.map(s => s.shape + ":" + s.cells.map(c => c.join(",")).sort().join(";")).sort().join("|");
const setOf = sols => new Set(sols.map(sig));
const same = (a, b) => a.size === b.size && [...a].every(x => b.has(x));
const pcs = names => names.split(" ").map((s, i) => ({ name: s, shape: s, idx: i }));
let bad = 0;

// ---- 1. 小さい問題で答えの集まりを比べる
const cases = [
  ["4×5 I O L J S", P.rect(4, 5), "I O L J S"],
  ["4×5 I O L J Z", P.rect(4, 5), "I O L J Z"],
  ["3×4 L J O", P.rect(3, 4), "L J O"],
  ["3×3 I3 V3 V3", P.rect(3, 3), "I3 V3 V3"],
  ["4×4 T T T T", P.rect(4, 4), "T T T T"],
  ["十字 X5 + 4×D2", P.fromArt(["..##..", "..##..", "######", "..#...", "..#..."]), "X5 D2 D2 D2 D2 I3"],
  // 穴のある形・同じ形のピースが多い組(点検 C18 の 8)
  ["ロの字 I L J", P.fromArt(["####", "#..#", "#..#", "####"]), "I L J"],
  ["4×4 V3×4 + D2×2", P.rect(4, 4), "V3 V3 V3 V3 D2 D2"],
  ["4×4 D2×8", P.rect(4, 4), "D2 D2 D2 D2 D2 D2 D2 D2"],
  ["3×4 I3×2 + V3×2", P.rect(3, 4), "I3 I3 V3 V3"],
  ["穴あき 5×5 V3×8", P.fromArt(["#####", "#####", "##.##", "#####", "#####"]), "V3 V3 V3 V3 V3 V3 V3 V3"]
];
// でたらめ: 種つきの乱数で、ピースを選び、そのマスの合計に合う形(長方形か、長方形から角を欠いた形)を作る
let seed = 12345; const rnd = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648;
const small = ["I", "O", "T", "L", "J", "S", "Z", "I3", "V3", "D2"];
for (let i = 0; i < (quick ? 15 : 60); i++) {
  const n = 2 + Math.floor(rnd() * 3), names = [];
  for (let j = 0; j < n; j++) names.push(small[Math.floor(rnd() * small.length)]);
  const area = names.reduce((s, x) => s + P.SHAPES[x].length, 0);
  // 幅 w の長方形を作り、余りは最後の行に左から
  const w = 2 + Math.floor(rnd() * 3), t = [];
  for (let k = 0; k < area; k++) t.push([Math.floor(k / w), k % w]);
  cases.push([`でたらめ ${i} (${names.join(" ")} / 幅 ${w})`, t, names.join(" ")]);
}
// 答えの中身が正しいか: k と origin から作ったマスが cells と同じ・ピースを 1 回ずつ使う・重ならずに目標をちょうど埋める
function valid(sol, target, pieces) {
  const used = new Set(), cover = new Set();
  for (const s of sol) {
    if (used.has(s.piece)) return "同じピースを 2 回";
    used.add(s.piece);
    const re = P.SHAPES[s.shape].map(c => { const r = P.rotK(c, s.k); return [r[0] + s.origin[0], r[1] + s.origin[1]]; });
    const k1 = re.map(c => c.join(",")).sort().join(";"), k2 = s.cells.map(c => c.join(",")).sort().join(";");
    if (k1 !== k2) return "k と origin から作ったマスが cells と違う";
    for (const c of s.cells) { const kk = c.join(","); if (cover.has(kk)) return "重なる"; cover.add(kk); }
  }
  if (used.size !== pieces.length) return "使わないピースがある";
  const tset = new Set(target.map(c => c.join(",")));
  if (cover.size !== tset.size || [...cover].some(k => !tset.has(k))) return "目標をちょうど埋めない";
  return "";
}
let agree = 0, withSol = 0, invalid = 0;
for (const [name, target, names] of cases) {
  const a = P.solve(target, pcs(names), 1e9), b = P.solveX(target, pcs(names), 1e9);
  const bad1 = b.solutions.map(x => valid(x, target, pcs(names))).find(Boolean);
  if (bad1) { invalid++; console.log(`★NG ${name}: 速い版の答えが正しくない(${bad1})`); }
  // 集まりと数の両方(速い版が同じ答えを 2 度出すと、数が合わない)
  const ok = same(setOf(a.solutions), setOf(b.solutions)) && a.solutions.length === b.solutions.length && !bad1;
  if (!ok) { bad++; console.log(`★NG ${name}: 前 ${a.solutions.length} 通り・速い版 ${b.solutions.length} 通り`); }
  else { agree++; if (a.solutions.length) withSol++; }
  if (cases.indexOf(cases.find(c => c[0] === name)) < 11) console.log(`[${name}] 前 ${a.solutions.length} 通り・速い版 ${b.solutions.length} 通り ${ok ? "一致" : "★不一致"}`);
}
console.log(`答えの集まりと数: ${agree}/${cases.length} 組で一致(うち答えのある組 ${withSol})・中身の正しくない答え ${invalid} 組`);

// ---- 2. 負の検算
{
  const a = P.solve(P.rect(4, 5), pcs("I O L J S"), 1e9).solutions, b = P.solveX(P.rect(4, 5), pcs("I O L J S"), 1e9).solutions;
  const drop = setOf(b.slice(1)), moved = setOf(b.map((s, i) => i ? s : s.map((x, j) => j ? x : Object.assign({}, x, { cells: x.cells.map(c => [c[0] + 1, c[1]]) }))));
  const n1 = !same(setOf(a), drop), n2 = !same(setOf(a), moved);
  console.log(`負の検算: 1 つ抜く → ${n1 ? "不一致(よい)" : "★一致してしまう"}・1 マスずらす → ${n2 ? "不一致(よい)" : "★一致してしまう"}`);
  // 中身の検査の負の検算: origin を 1 マスずらした答え・ピースを 1 つ落とした答えを「正しくない」と言うか
  const t45 = P.rect(4, 5), p45 = pcs("I O L J S");
  const n3 = !!valid(b[0].map((x, j) => j ? x : Object.assign({}, x, { origin: [x.origin[0] + 1, x.origin[1]] })), t45, p45);
  const n4 = !!valid(b[0].slice(1), t45, p45);
  console.log(`負の検算(中身): origin をずらす → ${n3 ? "正しくない(よい)" : "★通ってしまう"}・ピースを 1 つ落とす → ${n4 ? "正しくない(よい)" : "★通ってしまう"}`);
  if (!n1 || !n2 || !n3 || !n4) bad++;
}

// ---- 3. 大きい問題の速さ(最初の答えまで)
const OLD_LIMIT = quick ? 10 : 60;
const big = [
  ["5×5 の十字ぬき・ペントミノ 4", P.fromArt(["#####", "##.##", "#...#", "##.##", "#####"]), "L5 P5 U5 N5"],
  ["3×10・ペントミノ 6", P.rect(3, 10), "I5 L5 P5 U5 Y5 N5"],
  ["5×6・ペントミノ 6", P.rect(5, 6), "F5 L5 P5 T5 W5 Y5"],
  ["6×10・ペントミノ 12(裏返さない)", P.rect(6, 10), "F5 I5 L5 N5 P5 T5 U5 V5 W5 X5 Y5 Z5"]
];
for (const [name, target, names] of big) {
  const b = P.solveX(target, pcs(names), 1, { timeLimit: 60 });
  let old = "—";
  try {
    const out = execFileSync(process.execPath, [path.join(__dirname, "check-puzzle.js"), "--old", JSON.stringify({ target, pieces: pcs(names), max: 1 })], { timeout: OLD_LIMIT * 1000 }).toString();
    const r = JSON.parse(out); old = `${r.n ? "見つけた" : "無し"} ${(r.ms / 1000).toFixed(2)} 秒`;
  } catch (e) { old = `${OLD_LIMIT} 秒で終わらない(打ち切り)`; }
  console.log(`[${name}] 速い版: ${b.solutions.length ? "見つけた" : b.reason || "無し"} ${(b.ms / 1000).toFixed(2)} 秒(探した節 ${b.nodes})/ 前の版: ${old}`);
}
console.log(bad ? `NG ${bad} 件` : "OK(速い版と前の版の答えの集まりと数が一致・中身も正しい。負の検算 4 つは外れた)");
if (bad) process.exitCode = 1;
