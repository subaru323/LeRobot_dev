// 前提 U-1 の検査: 生の URDF の関節の数字 ⇄ so101.js の写し。
//   見ている世界: SO-ARM100 リポジトリの Simulation/SO101/so101_new_calib_camera.urdf(2026-11-16 に取得)。
//   これは so101_new_calib.urdf(2026-11-12 取得・2026-11-16 に本家とバイト一致を確かめた)に、手首のカメラの 2 リンクを足したもの
//   見ていないもの: 慣性・質量・伝達(使っていない)
const fs = require("fs"), path = require("path");
const { load, ROOT } = require("./_load");
load("so101.js");
const xml = fs.readFileSync(path.join(ROOT, "models/src/so101_new_calib_camera.urdf"), "utf8");
const joints = {};
for (const m of xml.matchAll(/<joint\s+name="([^"]+)"\s+type="([^"]+)"[\s\S]*?<\/joint>/g)) {
  const b = m[0];
  const o = b.match(/<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"/);
  const l = b.match(/<limit[^>]*lower="([^"]+)"[^>]*upper="([^"]+)"/);
  joints[m[1]] = { xyz: o[1].trim().split(/\s+/).map(Number), rpy: o[2].trim().split(/\s+/).map(Number),
                   lim: l ? [Number(l[1]), Number(l[2])] : null };
}
const name = { gripper_frame: "gripper_frame_joint", wrist_camera_mount: "wrist_camera_mount_joint", wrist_camera: "wrist_camera_joint" };
let ng = 0, n = 0;
for (const [k, v] of Object.entries(SO101.URDF)) {
  const r = joints[name[k] || k];
  if (!r) { console.log("NG 生の URDF に無い: " + k); ng++; continue; }
  const cmp = (a, b, what) => a.forEach((x, i) => { n++; if (x !== b[i]) { ng++; console.log(`NG ${k}.${what}[${i}] 写し ${x} ⇄ 生 ${b[i]}`); } });
  cmp(v.xyz, r.xyz, "xyz"); cmp(v.rpy, r.rpy, "rpy"); if (v.lim) cmp(v.lim, r.lim, "lim");
}
console.log(`[so101.js ⇄ 生の URDF] 数字 ${n} 個を完全一致で比べた: ` + (ng ? `NG ${ng} 個` : "OK(全部一致)"));
process.exit(ng ? 1 : 0);
