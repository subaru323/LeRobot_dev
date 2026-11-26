// 見て動かす画面(live.html)。docs/Simulation_Logic_Documents/09_画像だけで動かすシミュレーション.md の §6(利用者 2026-11-17)。
//   ■ 何をするか
//     1) 撮る: 写す姿勢で、上のカメラの絵を作る(世界の側。vision.js の render。本当の置き場から)。
//     2) 見つける: 絵と、ずらしたカメラの置き方・ピースの形と色だけから、置き場を見積もる(制御の側。vision.js の detect)。
//     3) 選ぶ: 見積もった置き場だけで動きを組む(制御の側。scene-data.js の buildFromUi に LAYOUT として渡す。当たり判定はいつもと同じ)。
//     4) 動く: 組んだ動きを流す。各ピースをつかみに行く前に、上の絵を撮り直して見積もり直し、
//        つかめる余裕より動いていたら「止まる」、そうでなければ「つかみに行く」を選ぶ(行動の選択)。
//        つかんだ瞬間に、本当の置き場で「つかめたか」を採点する(vision-loop.js と同じ式)。つかみ損ねたピースは机に残る。
//   ■ 世界と制御を分ける(09 §3・§6-4)
//     WORLD だけが本当の置き場(truth)を持つ。CONTROL は絵・カメラの置き方(ずらしたもの)・ピースの形と色・自分の計画だけを受け取る。
//   ■ まだできないこと(画面にも出す)
//     - 1 手だけ組む口が scene-data.js に無いので、選ぶ段は全部の手をまとめて組む(ブラウザで形 2 を 3 回測って 11〜13 秒。その間は画面が止まる)。止まったあとは、写す姿勢から全部やり直す。
//     - つかむ前の撮り直し: 計画に写す姿勢への寄り道(scene-data.js の RELOOK。2026-11-18)があれば、そこで止まっている間に撮る。
//       無ければ、前のピースを置いてから次のピースの上に来るまでのうち、腕が隠さないと制御が予測した時に撮る(CONTROL.chooseLook)。
//       隠れない時が無いピースがあれば、動き出す前に止まる。
//     - 手首のカメラは窓に出すだけで、判断には使っていない。
//     - 画角は 60°【仮】(品が決まっていない)。カメラの置き方のずれ・画素の雑音の幅も【仮】。
//   ■ 読むだけのファイル: scene-data.js・vision.js・so101.js・kinematics.js・puzzle.js ほか(書き換えない)。
(function () {
  "use strict";
  var DEG = Math.PI / 180;
  var FOV_TOP = SO101.CAM_FOV.top, FOV_WRIST = SO101.CAM_FOV.wrist;   // 縦の画角 [度]【仮】。so101.js の 1 か所から読む(2026-11-17 に持ち主がまとめた)
  var NOISE = 3;                // 画素の雑音(標準偏差 0〜255)【仮】(vision-loop.js の既定と同じ)
  var CAL_MM = 1, CAL_DEG = 0.2;  // 制御が信じるカメラを、世界のカメラからこれだけずらす【仮】(09 §3 の 2 行目。絵からの校正はまだ無い)
  var SHIFT_MM = 8, YAW_DEG = 20; // 本当の置き場を、scene-data.js の既定の置き場からずらす幅(vision-loop.js と同じ)
  var KICK_MM = 20, KICK_DEG = 15; // かき乱すときに動かす量【仮】。向きは乱数・回しは ±KICK_DEG
  var HIDE_RATIO = 0.85;        // 撮り直しで、ピースの画素の数が最初の絵のこの割合より少なければ「隠れている」【仮】(根拠は無い)
  var SEED = 1;
  var $ = function (id) { return document.getElementById(id); };
  var M = null, V = null;       // SCENE_MODEL・VISION(読み込んでから入る)

  // =========================================================================
  //  記録と表示
  // =========================================================================
  var t0Wall = performance.now();
  function status(msg) { $("status").textContent = msg || ""; }
  function stage(name) { ["look", "find", "choose", "move"].forEach(function (s) { $("st-" + s).className = s === name ? "on" : ""; }); }
  function log(msg, cls) {
    var div = document.createElement("div"), sec = ((performance.now() - t0Wall) / 1000).toFixed(1);
    div.innerHTML = "<span class='t'>" + sec + "s</span>" + (cls ? "<span class='" + cls + "'>" : "<span>") + msg + "</span>";
    $("log").insertBefore(div, $("log").firstChild);
  }
  // 文字を画面に出してから重い計算に入る。画面が隠れていると requestAnimationFrame が来ないので、時間でも進める
  function nextPaint() { return new Promise(function (ok) { var done = false, go = function () { if (!done) { done = true; setTimeout(ok, 30); } }; requestAnimationFrame(go); setTimeout(go, 150); }); }
  function loadScript(src) {
    // ?t= を付けて毎回読み直す(scene-data.js が直っても、ブラウザが前の版を覚えていて使っていた。2026-11-18)
    return new Promise(function (ok, ng) { var s = document.createElement("script"); s.src = src + (location.protocol === "file:" ? "" : "?t=" + Date.now()); s.onload = ok; s.onerror = function () { ng(new Error(src)); }; document.body.appendChild(s); });
  }
  function mm(v) { return (v * 1000).toFixed(1); }
  function ang(d) { d = ((d % 360) + 360) % 360; return d > 180 ? d - 360 : d; }   // −180〜180 度に

  // =========================================================================
  //  3D(index.html と同じ部品の描き方。index.html の buildWorld・applyAnimation の要る所だけの写し)
  // =========================================================================
  var holder = $("canvas-holder");
  var scene = new THREE.Scene();
  scene.background = new THREE.Color("#dfe6ee");
  var camera = new THREE.PerspectiveCamera(45, 1, 0.02, 50);
  var renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  holder.appendChild(renderer.domElement);
  var controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true; controls.dampingFactor = 0.08;
  if (THREE.MOUSE) { controls.mouseButtons.LEFT = THREE.MOUSE.PAN; controls.mouseButtons.RIGHT = THREE.MOUSE.ROTATE; }   // index.html と同じ割り当て
  scene.add(new THREE.AmbientLight(0xffffff, 0.55));
  var dl = new THREE.DirectionalLight(0xffffff, 0.85); dl.position.set(4, 8, 6); scene.add(dl);
  var dl2 = new THREE.DirectionalLight(0xffffff, 0.3); dl2.position.set(-6, 4, -5); scene.add(dl2);
  var grid = new THREE.GridHelper(3, 60, 0x8a94a6, 0xaab2bd); grid.position.y = 0.0005; scene.add(grid);
  var worldRoot = new THREE.Group(); scene.add(worldRoot);
  var overlayRoot = new THREE.Group(); scene.add(overlayRoot);
  function onResize() {
    var w = holder.clientWidth, h = holder.clientHeight;
    renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix();
  }
  window.addEventListener("resize", onResize); onResize();

  var nodes = {}, anims = [], geomCache = {}, meshGeoCache = {};
  function sampleTrack(track, t) {
    var n = track.length; if (t <= track[0].t) return track[0].v; if (t >= track[n - 1].t) return track[n - 1].v;
    var i = 0; while (i < n - 1 && track[i + 1].t <= t) i++;
    var a = track[i], b = track[i + 1]; if (b.hold) return a.v;
    var u = b.t > a.t ? (t - a.t) / (b.t - a.t) : 1;
    if (b.ease === "inOut") u = u < 0.5 ? 2 * u * u : -1 + (4 - 2 * u) * u; else if (b.ease === "in") u = u * u; else if (b.ease === "out") u = u * (2 - u);
    if (typeof a.v === "number") return a.v + (b.v - a.v) * u;
    return [a.v[0] + (b.v[0] - a.v[0]) * u, a.v[1] + (b.v[1] - a.v[1]) * u, a.v[2] + (b.v[2] - a.v[2]) * u];
  }
  function makeGeometry(p) {
    var s = p.size || [], key = p.shape + ":" + s.join(",");
    if (geomCache[key]) return geomCache[key];
    var g;
    if (p.shape === "box") g = new THREE.BoxGeometry(s[0] || 1, s[1] || 1, s[2] || 1);
    else if (p.shape === "cylinder") g = new THREE.CylinderGeometry(s[0] || 0.5, s[1] || 0.5, s[2] || 1, s[3] || 32);
    else if (p.shape === "sphere") g = new THREE.SphereGeometry(s[0] || 0.5, 24, 16);
    else if (p.shape === "cone") g = new THREE.ConeGeometry(s[0] || 0.5, s[1] || 1, s[2] || 32);
    else g = new THREE.BoxGeometry(0.01, 0.01, 0.01);
    return geomCache[key] = g;
  }
  function b64ToBytes(s) { var bin = atob(s), out = new Uint8Array(bin.length); for (var i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i); return out; }
  function meshGeos(name) {
    if (meshGeoCache[name]) return meshGeoCache[name];
    var src = (typeof MESHES !== "undefined") ? MESHES[name] : null;
    if (!src) return meshGeoCache[name] = [];
    return meshGeoCache[name] = src.groups.map(function (g) {
      var qp = new Uint16Array(b64ToBytes(g.p).buffer), qn = new Int8Array(b64ToBytes(g.n).buffer), raw = b64ToBytes(g.i);
      var idx = g.i32 ? new Uint32Array(raw.buffer) : new Uint16Array(raw.buffer), pos = new Float32Array(qp.length), nor = new Float32Array(qn.length);
      for (var i = 0; i < qp.length / 3; i++) { pos[i * 3] = g.q[0] + qp[i * 3] * g.q[3]; pos[i * 3 + 1] = g.q[1] + qp[i * 3 + 1] * g.q[4]; pos[i * 3 + 2] = g.q[2] + qp[i * 3 + 2] * g.q[5]; }
      for (var k = 0; k < qn.length; k++) nor[k] = qn[k] / 127;
      var geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.BufferAttribute(pos, 3)); geo.setAttribute("normal", new THREE.BufferAttribute(nor, 3)); geo.setIndex(new THREE.BufferAttribute(idx, 1));
      return { geo: geo, color: g.c };
    });
  }
  function mat(color, op) {
    op = op === undefined ? 1 : op;
    return new THREE.MeshStandardMaterial({ color: color || "#7c8794", transparent: op < 1, opacity: op, depthWrite: op >= 1, roughness: 0.68, metalness: 0.05 });
  }
  function buildWorld(d) {
    while (worldRoot.children.length) worldRoot.remove(worldRoot.children[0]);
    nodes = {}; anims = [];
    var pending = (d.parts || []).slice(), safety = 0;
    while (pending.length && safety++ < 30) {
      for (var i = pending.length - 1; i >= 0; i--) {
        var p = pending[i], parent = p.parent ? nodes[p.parent] : worldRoot;
        if (p.parent && !parent) continue;
        var g = new THREE.Group(), pos = p.position || [0, 0, 0];
        g.position.set(pos[0] || 0, pos[1] || 0, pos[2] || 0);
        if (p.axis) g.quaternion.setFromAxisAngle(new THREE.Vector3(p.axis[0], p.axis[1], p.axis[2]).normalize(), (p.angleDeg || 0) * DEG);
        else { var r = p.rotationDeg || [0, 0, 0]; g.rotation.set(r[0] * DEG, r[1] * DEG, r[2] * DEG); }
        if (p.scale) g.scale.set(p.scale[0], p.scale[1], p.scale[2]);
        if (p.shape === "mesh") meshGeos(p.mesh).forEach(function (mg) { g.add(new THREE.Mesh(mg.geo, mat(p.color || mg.color, p.opacity))); });
        else if (p.shape && p.shape !== "group") g.add(new THREE.Mesh(makeGeometry(p), mat(p.color, p.opacity)));
        if (p.axis) g.userData.axis = p.axis.slice();
        g.userData.id = p.id;
        parent.add(g); nodes[p.id] = g; pending.splice(i, 1);
      }
    }
    (d.animations || []).forEach(function (a) {
      if (a.type !== "keys" || !a.keys) return;
      var tr = {};
      ["pos", "rot", "angle"].forEach(function (ch) {
        var arr = a.keys.filter(function (k) { return k[ch] !== undefined; }).map(function (k) { return { t: k.t || 0, v: k[ch], hold: !!k.hold, ease: k.ease }; }).sort(function (x, y) { return x.t - y.t; });
        if (arr.length) tr[ch] = arr;
      });
      a._tracks = tr; anims.push(a);
    });
  }
  function applyAnims(t) {
    anims.forEach(function (a) {
      var n = nodes[a.target]; if (!n) return;
      var tr = a._tracks, tt = Math.min(Math.max(t, 0), a.duration || t);
      if (tr.pos) { var v = sampleTrack(tr.pos, tt); n.position.set(v[0], v[1], v[2]); }
      if (tr.rot) { var r = sampleTrack(tr.rot, tt); n.rotation.set(r[0] * DEG, r[1] * DEG, r[2] * DEG); }
      if (tr.angle && n.userData.axis) { var ax = n.userData.axis; n.quaternion.setFromAxisAngle(new THREE.Vector3(ax[0], ax[1], ax[2]).normalize(), sampleTrack(tr.angle, tt) * DEG); }
    });
  }

  // ---- 画角の四角錐と、机の上の足跡 ----
  function makeFrustum(color) {
    var lineGeo = new THREE.BufferGeometry(); lineGeo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(16 * 3), 3));
    var lines = new THREE.LineSegments(lineGeo, new THREE.LineBasicMaterial({ color: color }));
    var footGeo = new THREE.BufferGeometry(); footGeo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(6 * 3), 3));
    var foot = new THREE.Mesh(footGeo, new THREE.MeshBasicMaterial({ color: color, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false }));
    overlayRoot.add(lines); overlayRoot.add(foot);
    return { lines: lines, foot: foot };
  }
  // cam: {tip, dir, up}(机 = 0 の座標)。四隅の光線を机の面(y = 0.0008)まで伸ばす。机に届かない光線は 0.6m で切る
  function updateFrustum(F, cam, W, H, fov) {
    var img = SO101.overheadImage(cam, fov, W, H), C = img.tip, corners = [[0, 0], [W, 0], [W, H], [0, H]].map(function (px) {
      var dx = (px[0] - W / 2) / img.f, dy = (H / 2 - px[1]) / img.f;
      var d = [img.fwd[0] + dx * img.right[0] + dy * img.up[0], img.fwd[1] + dx * img.right[1] + dy * img.up[1], img.fwd[2] + dx * img.right[2] + dy * img.up[2]];
      var s = d[1] < -1e-6 ? (0.0008 - C[1]) / d[1] : 0.6 / Math.hypot(d[0], d[1], d[2]);
      s = Math.min(s, 1.5);
      return [C[0] + d[0] * s, C[1] + d[1] * s, C[2] + d[2] * s];
    });
    var a = F.lines.geometry.attributes.position.array, k = 0;
    function seg(p, q) { a[k++] = p[0]; a[k++] = p[1]; a[k++] = p[2]; a[k++] = q[0]; a[k++] = q[1]; a[k++] = q[2]; }
    corners.forEach(function (c, i) { seg(C, c); seg(c, corners[(i + 1) % 4]); });
    F.lines.geometry.attributes.position.needsUpdate = true; F.lines.geometry.computeBoundingSphere();
    var f = F.foot.geometry.attributes.position.array, o = [0, 1, 2, 0, 2, 3];
    o.forEach(function (ci, j) { f[j * 3] = corners[ci][0]; f[j * 3 + 1] = 0.0008; f[j * 3 + 2] = corners[ci][2]; });
    F.foot.geometry.attributes.position.needsUpdate = true; F.foot.geometry.computeBoundingSphere();
  }
  var frTop = makeFrustum(0xff9f1c), frWrist = makeFrustum(0x2ec4f0);

  // =========================================================================
  //  世界(本当の置き場を持つ。絵を作り、採点する)。CONTROL からは呼ばれない
  // =========================================================================
  var WORLD = (function () {
    var geo = null, truth = null, rnd = null, truthMeshes = [];
    function rotY(v, deg) { var c = Math.cos(deg * DEG), s = Math.sin(deg * DEG); return [v[0] * c - v[2] * s, v[1], v[0] * s + v[2] * c]; }
    // 本当の置き場: 既定の置き場を種で決まる乱数でずらす。置き場として使えない(重なる・届かない)ものは引き直す(vision-loop.js と同じ考え)
    // keep: 止まった後のやり直しでは、かき乱した後の本当の置き場をそのまま使う(置いたピースは机の元の所へ戻す)
    function init(g, set, keep) {
      if (keep && truth && geo === g) { buildTruthMeshes(); return; }
      geo = g; var c = g._check; rnd = V.rng(SEED * 1000 + set);
      truth = null;
      for (var k = 0; k < 50 && !truth; k++) {
        var L = { pieces: c.pieces.map(function (p) {
          return { name: p.name, x: c.slots[p.idx][0] * 1000 + (rnd() * 2 - 1) * SHIFT_MM, z: c.slots[p.idx][1] * 1000 + (rnd() * 2 - 1) * SHIFT_MM,
                   yaw: (c.slotRot[p.idx] || 0) + (rnd() * 2 - 1) * YAW_DEG };
        }) };
        if (!M.buildFromUi({ SET: set, SOL: 0, LAYOUT: L, LAYOUT_ONLY: true })._check.layoutWhy) truth = L;
      }
      if (!truth) throw new Error("使える本当の置き場が引けない");
      buildTruthMeshes();
    }
    function buildTruthMeshes() {
      truthMeshes.forEach(function (m) { overlayRoot.remove(m); }); truthMeshes = [];
      var c = geo._check, s = c.cell;
      c.pieces.forEach(function (pc) {
        var g = new THREE.Group(), m = mat(pc.color);
        PUZZLE.SHAPES[pc.shape].forEach(function () { var b = new THREE.Mesh(new THREE.BoxGeometry(s, s, s), m); g.add(b); });
        overlayRoot.add(g); truthMeshes.push(g);
      });
      placeTruthMeshes();
    }
    function placeTruthMeshes() {
      var c = geo._check;
      c.pieces.forEach(function (pc) {
        var P = truth.pieces[pc.idx], g = truthMeshes[pc.idx];
        PUZZLE.SHAPES[pc.shape].forEach(function (cl, i) {
          var o = rotY([cl[0] * c.pitch, 0, cl[1] * c.pitch], P.yaw), b = g.children[i];
          b.position.set(P.x / 1000 + o[0], c.cell / 2, P.z / 1000 + o[2]); b.rotation.set(0, -P.yaw * DEG, 0);   // vision.js の rotY(度) = three.js の y まわり −度
        });
      });
    }
    function setTruthVisible(pid, on) { truthMeshes[pid].visible = on; }
    // ピースの三角形(机の上: 本当の置き場 / 持っている・置いた: 3D の部品の位置から)
    function cubeTris(corners) {
      var pts = [];
      [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]].forEach(function (f) { pts.push(corners[f[0]], corners[f[1]], corners[f[2]], corners[f[0]], corners[f[2]], corners[f[3]]); });
      return pts;
    }
    function pieceTris(onTable) {
      var c = geo._check, h = c.cell / 2, out = [], v = new THREE.Vector3();
      var box = [[-h, -h, -h], [h, -h, -h], [h, -h, h], [-h, -h, h], [-h, h, -h], [h, h, -h], [h, h, h], [-h, h, h]];
      c.pieces.forEach(function (pc) {
        var g = onTable[pc.idx] ? truthMeshes[pc.idx] : nodes["pc" + pc.idx], pts = [];
        if (!g) return;
        g.updateMatrixWorld(true);
        g.children.forEach(function (b) {
          if (!b.isMesh && !/_c\d+$/.test(b.userData.id || "")) return;   // 計画の部品は立方体(_cN)だけ。つなぐ棒は絵に入れない(vision.js の worldTris と同じ)
          var mesh = b.isMesh ? b : b.children[0];
          if (!mesh) return;
          pts = pts.concat(cubeTris(box.map(function (q) { v.set(q[0], q[1], q[2]).applyMatrix4(mesh.matrixWorld); return [v.x, v.y, v.z]; })));
        });
        out.push({ pts: pts, color: V.hex(pc.color), tag: "piece" + pc.idx });
      });
      return out;
    }
    function trisAt(q, jaw, onTable, noCam) {
      var lay = { slots: truth.pieces.map(function (p) { return [p.x / 1000, p.z / 1000]; }), rots: truth.pieces.map(function (p) { return p.yaw; }) };
      var base = V.worldTris(geo, lay, q, jaw, { noCam: !!noCam }).filter(function (o) { return !/^piece/.test(o.tag); });
      return base.concat(pieceTris(onTable));
    }
    var shotN = 0;
    function shootTop(q, jaw, onTable) { return V.render(trisAt(q, jaw, onTable, false), geo.overheadCamera, { fov: FOV_TOP, noise: NOISE, seed: SEED * 7919 + (++shotN) }); }
    function wristCam(q, jaw) {
      var cf = SO101.urdfFK(q.map(function (x) { return x * DEG; }), jaw * DEG).wrist_camera_link, RY = geo._check.robotY;
      return { tip: [cf.p[0], cf.p[1] + RY, cf.p[2]], dir: [cf.R[2], cf.R[5], cf.R[8]], up: [-cf.R[1], -cf.R[4], -cf.R[7]] };   // scene-data.js の視点の一覧と同じ(光軸 = z・絵の下 = y【仮】)
    }
    function shootWrist(q, jaw, onTable) { return V.render(trisAt(q, jaw, onTable, true), wristCam(q, jaw), { fov: FOV_WRIST, W: 320, H: 240 }); }
    // つかめたかの採点(vision-loop.js と同じ式): 制御の見積もりで決めたつかむ場所を、本当のピースの上に移したずれ
    function grade(pid, grasp, est) {
      var c = geo._check, pc = c.pieces[pid], t = truth.pieces[pid];
      var m = V.match(pc.shape, c.pitch, [est.x / 1000, est.z / 1000, est.yaw], [t.x / 1000, t.z / 1000, t.yaw]);
      var pT = m.T.apply([grasp.cS[0], grasp.cS[2]]), e = [pT[0] - grasp.cS[0], pT[1] - grasp.cS[2]];
      var u = [Math.cos(grasp.yp * DEG), Math.sin(grasp.yp * DEG)], ea = e[0] * u[0] + e[1] * u[1], ec = -e[0] * u[1] + e[1] * u[0];
      var need = Math.abs(ea) + c.cell / 2 * Math.abs(Math.sin(m.phi * DEG));
      return { ok: need < c.grip.fitMargin, ea: ea, ec: ec, phi: m.phi, need: need, margin: c.grip.fitMargin };
    }
    function kick(pid) {
      var t = truth.pieces[pid], a = rnd() * 2 * Math.PI;
      t.x += Math.cos(a) * KICK_MM; t.z += Math.sin(a) * KICK_MM; t.yaw += (rnd() < 0.5 ? -1 : 1) * KICK_DEG;
      placeTruthMeshes();
      return { dx: Math.cos(a) * KICK_MM, dz: Math.sin(a) * KICK_MM };
    }
    // 見る人のためだけ(判断には渡さない): 見積もりと本当の差
    //   マスどうしで比べる(O は 90°、I・S・Z は 180° 回すと同じ形。置き場の x, z は形の基準のマスなので、そのまま引くと差が大きく見える)
    function diffFor(pid, est) {
      var t = truth.pieces[pid], pc = geo._check.pieces[pid];
      var m = V.match(pc.shape, geo._check.pitch, [est.x / 1000, est.z / 1000, est.yaw], [t.x / 1000, t.z / 1000, t.yaw]);
      return { d: m.worstCell * 1000, dyaw: m.phi };
    }
    function camForControl() {   // 制御が信じるカメラ = 世界のカメラを【仮】の幅でずらしたもの(vision-loop.js の perturbCam と同じ考え)
      var cam = geo.overheadCamera, r = V.rng(SEED * 31 + 7);
      var u = function () { var v = [V.gauss(r), V.gauss(r), V.gauss(r)], n = Math.hypot(v[0], v[1], v[2]); return v.map(function (x) { return x / n; }); };
      var dp = u().map(function (x) { return x * CAL_MM / 1000; }), R = KIN.rotAA(u(), CAL_DEG * DEG);
      return { tip: cam.tip.map(function (v, i) { return v + dp[i]; }), dir: KIN.matVec(R, cam.dir), up: KIN.matVec(R, cam.up) };
    }
    return { init: init, shootTop: shootTop, shootWrist: shootWrist, wristCam: wristCam, grade: grade, kick: kick, diffFor: diffFor,
             camForControl: camForControl, setTruthVisible: setTruthVisible, topCam: function () { return geo.overheadCamera; } };
  })();

  // =========================================================================
  //  制御(絵だけから決める)。本当の置き場を受け取らない
  // =========================================================================
  var CONTROL = (function () {
    var known = null, calib = null, basePix = null;
    function setup(k, cam) { known = k; calib = { cam: cam, fov: FOV_TOP }; basePix = null; }
    // 見つける。最初の絵の各ピースの画素の数を覚えておき、撮り直しで減っていたら「隠れている」と分かるようにする(絵だけから分かること)
    function look(im, first) { var det = V.detect(im, calib, known); if (first && !det.why) basePix = det.per.map(function (p) { return p.nPix; }); return det; }
    // RELOOK: ピースとピースの間で写す姿勢に寄り道する(真上のカメラ C では、それが無いと撮り直しで腕がピースを隠す。2026-11-18 に sim/ の持ち主に頼んだ。
    //   scene-data.js が知らないうちは無視され、寄り道の無い動きになる → chooseLook が隠れない時刻を見つけられず、撮り直しで止まる)
    function plan(set, estimate) { return M.buildFromUi(Object.assign(M.defaultUiValues(), { SET: set, LAYOUT: estimate, RELOOK: true })); }
    // つかみに行く前の見直し: いまの絵の見積もりが、計画に使った見積もりから、つかめる余裕より動いていないか
    function recheck(det, plannedEst, pid, grasp, c) {
      // 確かめられないときは止まる(進まない)。写す姿勢から撮り直せば確かめられる
      if (det.why) return { action: "stop", blind: true, why: det.why };
      var n0 = basePix ? basePix[pid] : 0, n1 = det.per[pid] ? det.per[pid].nPix : 0;
      if (basePix && n1 < HIDE_RATIO * n0) return { action: "stop", blind: true, why: "腕に隠れているかもしれない（このピースの画素が " + n0 + " → " + n1 + "。" + HIDE_RATIO * 100 + "% 未満【仮】）" };
      var now = det.layout.pieces[pid], pc = c.pieces[pid];
      var m = V.match(pc.shape, c.pitch, [plannedEst.x / 1000, plannedEst.z / 1000, plannedEst.yaw], [now.x / 1000, now.z / 1000, now.yaw]);
      var pT = m.T.apply([grasp.cS[0], grasp.cS[2]]), e = [pT[0] - grasp.cS[0], pT[1] - grasp.cS[2]];
      var u = [Math.cos(grasp.yp * DEG), Math.sin(grasp.yp * DEG)], ea = e[0] * u[0] + e[1] * u[1], ec = -e[0] * u[1] + e[1] * u[0];
      var turn = c.cell / 2 * Math.abs(Math.sin(m.phi * DEG)), need = Math.abs(ea) + turn, moved = Math.hypot(e[0], e[1]);
      // つかめる式(vision-loop.js)は閉じる向きのずれ e_a と回しだけを見る。直角のずれ e_c は見ないので、ここでは e_c も同じ余裕で見る【仮】(動いたら止まる側に倒す)
      var ok = need < c.grip.fitMargin && Math.abs(ec) < c.grip.fitMargin;
      return { action: ok ? "go" : "stop", need: need, turn: turn, ec: ec, margin: c.grip.fitMargin, moved: moved, phi: m.phi, now: now };
    }
    // 撮り直す時刻を選ぶ(2026-11-18。上のカメラを真上の C にしたら、前のピースを置いた後の姿勢で撮ると腕が次のピースを隠して止まった。08 §3-2 の 7)。
    //   制御が知っているものだけで決める: 自分の計画の動き(c.motion)・自分のカメラ(calib)・計画に使った見積もり(est)。本当の置き場は読まない。
    //   前のピースを置き終えた時(tDone)から、次のピースの上で止まる時(tHover)までを LOOK_DT 秒おきに見て、
    //   見積もったピースの上の面の点(マスごとに 5 点)へのカメラの線に、腕の点が LOOK_TOL より近い点の数を数える。
    //   隠れる点が 0 の時刻のうち、いちばん遅い時(つかむ直前に近いほど新しい絵になる)を選ぶ。0 の時が無ければ、いちばん少ない時(画素の見張りで止まる見込み)
    var LOOK_DT = 0.1, LOOK_TOL = 0.008;   // 秒・m【仮】(overhead-occlusion.js の 2.5mm より太らせて、際どい時刻を避ける)
    function armPtsAt(q, jaw, RY) {
      var st = KIN.fk(SO101.axes, q, SO101.tool), out = [];
      Object.keys(M.ARM.points).forEach(function (g) {
        if (g === "base") return;   // 土台は動かず、置き場より後ろなので見ない
        var fr = st.frames[(g === "jaw" || g === "cam") ? 4 : (+g.slice(1) - 1)], P = M.ARM.points[g];
        for (var i = 0; i < P.length; i += 2) {   // 2 つおき(点の間は 2.5mm 前後なので、太らせた LOOK_TOL で埋まる)
          var p = g === "jaw" ? M.jawRotate(P[i], jaw) : P[i], w = KIN.add(fr.p, KIN.matVec(fr.R, KIN.sub(p, fr.o)));
          out.push([w[0], w[1] + RY, w[2]]);
        }
      });
      return out;
    }
    function hiddenCount(samples, arm) {
      var C = calib.cam.tip, n = 0;
      samples.forEach(function (T) {
        var d = [T[0] - C[0], T[1] - C[1], T[2] - C[2]], L = Math.hypot(d[0], d[1], d[2]); d = [d[0] / L, d[1] / L, d[2] / L];
        for (var i = 0; i < arm.length; i++) {
          var P = arm[i], v = [P[0] - C[0], P[1] - C[1], P[2] - C[2]], s = v[0] * d[0] + v[1] * d[1] + v[2] * d[2];
          if (s <= 0 || s >= L - 0.003) continue;
          var x = v[0] - s * d[0], y = v[1] - s * d[1], z = v[2] - s * d[2];
          if (x * x + y * y + z * z < LOOK_TOL * LOOK_TOL) { n++; return; }
        }
      });
      return n;
    }
    function chooseLook(c, estPiece, pc, tA, tB, qAtFn) {
      var h = c.cell * 0.4, samples = [];
      V.cellsOf(pc.shape, c.pitch, estPiece.x / 1000, estPiece.z / 1000, estPiece.yaw).forEach(function (cc) {
        // 5 点を見積もりの向きで回す(回さないと、向き 17° を超えるとマスの外へ出る。点検 C29 の 7。drawDetected と同じ式)
        var cs = Math.cos(estPiece.yaw * DEG), sn = Math.sin(estPiece.yaw * DEG);
        [[0, 0], [h, h], [h, -h], [-h, h], [-h, -h]].forEach(function (o) { samples.push([cc[0] + o[0] * cs - o[1] * sn, c.cell, cc[1] + o[0] * sn + o[1] * cs]); });
      });
      var best = null, bestZero = null;
      for (var t = tA; t <= tB + 1e-9; t += LOOK_DT) {
        var qa = qAtFn(t), n = hiddenCount(samples, armPtsAt(qa.q, qa.jaw, c.robotY));
        if (!best || n < best.n) best = { t: t, n: n };
        if (n === 0) bestZero = { t: t, n: 0 };
      }
      var pick = bestZero || best;
      return { t: pick.t, hidden: pick.n, of: samples.length, clear: !!bestZero };
    }
    return { setup: setup, look: look, plan: plan, recheck: recheck, chooseLook: chooseLook, calib: function () { return calib; } };
  })();

  // =========================================================================
  //  窓に絵を出す
  // =========================================================================
  function putImage(canvas, im) {
    var ctx = canvas.getContext("2d"), id = ctx.createImageData(im.W, im.H);
    for (var q = 0, n = im.W * im.H; q < n; q++) { id.data[q * 4] = im.rgb[q * 3]; id.data[q * 4 + 1] = im.rgb[q * 3 + 1]; id.data[q * 4 + 2] = im.rgb[q * 3 + 2]; id.data[q * 4 + 3] = 255; }
    ctx.putImageData(id, 0, 0);
    return ctx;
  }
  // 見つけたマスの枠を重ねる(制御が信じるカメラで写す = 制御が思っている位置)
  function drawDetected(ctx, det, c, highlight) {
    var img = SO101.overheadImage(CONTROL.calib().cam, FOV_TOP, 640, 480), h = c.cell / 2;
    ctx.lineWidth = 1.5; ctx.font = "13px sans-serif";
    c.pieces.forEach(function (pc) {
      var d = det.layout.pieces[pc.idx]; if (!d || !isFinite(d.x)) return;
      ctx.strokeStyle = pc.idx === highlight ? "#ffd479" : "#ffffff";
      var cs = Math.cos(d.yaw * DEG), sn = Math.sin(d.yaw * DEG), cx = 0, cy = 0, n = 0;
      V.cellsOf(pc.shape, c.pitch, d.x / 1000, d.z / 1000, d.yaw).forEach(function (cc) {
        var cor = [[-h, -h], [h, -h], [h, h], [-h, h]].map(function (o) { return img.project([cc[0] + o[0] * cs - o[1] * sn, c.cell, cc[1] + o[0] * sn + o[1] * cs]); });
        if (cor.some(function (p) { return !p; })) return;
        ctx.beginPath(); ctx.moveTo(cor[0][0], cor[0][1]); for (var k = 1; k < 4; k++) ctx.lineTo(cor[k][0], cor[k][1]); ctx.closePath(); ctx.stroke();
        cor.forEach(function (p) { cx += p[0]; cy += p[1]; n++; });
      });
      if (n) { ctx.fillStyle = ctx.strokeStyle; ctx.fillText(pc.name, cx / n + 8, cy / n - 8); }
    });
  }

  // ---- 見積もりの枠(3D) ----
  var estRoot = new THREE.Group(); overlayRoot.add(estRoot);
  function showEstimate(det, c) {
    while (estRoot.children.length) estRoot.remove(estRoot.children[0]);
    if (!det || det.why && !det.layout) return;
    var s = c.cell, eg = new THREE.EdgesGeometry(new THREE.BoxGeometry(s * 1.02, s * 1.02, s * 1.02)), lm = new THREE.LineBasicMaterial({ color: 0xffffff });
    c.pieces.forEach(function (pc) {
      var d = det.layout.pieces[pc.idx]; if (!d || !isFinite(d.x)) return;
      V.cellsOf(pc.shape, c.pitch, d.x / 1000, d.z / 1000, d.yaw).forEach(function (cc) {
        var e = new THREE.LineSegments(eg, lm); e.position.set(cc[0], s / 2, cc[1]); e.rotation.set(0, -d.yaw * DEG, 0); estRoot.add(e);
      });
    });
    estRoot.visible = $("showEst").checked;
  }
  $("showEst").addEventListener("change", function () { estRoot.visible = this.checked; });

  // =========================================================================
  //  流れ
  // =========================================================================
  var S = { set: 2, geo: null, plan: null, c: null, est: null, t: 0, playing: false, steps: [], stepIdx: 0, onTable: {}, graded: {}, busy: false, stopped: false, wristAt: -1 };
  function qAt(t) {
    var mo = S.c.motion, n = mo.length;
    if (t <= mo[0].t) return { q: mo[0].q, jaw: mo[0].jaw };
    if (t >= mo[n - 1].t) return { q: mo[n - 1].q, jaw: mo[n - 1].jaw };
    var lo = 0, hi = n - 1; while (hi - lo > 1) { var m = (lo + hi) >> 1; if (mo[m].t <= t) lo = m; else hi = m; }
    var a = mo[lo], b = mo[hi], u = (t - a.t) / (b.t - a.t);
    return { q: a.q.map(function (v, i) { return v + (b.q[i] - v) * u; }), jaw: a.jaw + (b.jaw - a.jaw) * u };
  }
  function setPieceDisplay() {
    // 机の上のピースは本当の置き場(色付き)で出す。つかめたピースは、つかんだ瞬間から 3D の部品(計画の動き)で出す
    S.c.pieces.forEach(function (pc) {
      var onTable = !!S.onTable[pc.idx];
      WORLD.setTruthVisible(pc.idx, onTable);
      if (nodes["pc" + pc.idx]) nodes["pc" + pc.idx].visible = !onTable;
    });
  }

  async function startRun() {
    if (S.busy) return;
    S.busy = true; S.playing = false; $("start").disabled = true; $("play").disabled = true; $("kick").disabled = true;
    try {
      var set = +$("set").value;
      // 世界の形(台・盤・ピースの形。置き場には関係しない)。既定の形なら読み込みのときに組んだものを使う
      if (!S.geo || S.geo._set !== set) {
        status("世界の形を組んでいます（数秒〜）…"); await nextPaint();
        S.geo = (typeof SCENE_DATA !== "undefined" && SCENE_DATA && SCENE_DATA._check && set === M.defaultUiValues().SET) ? SCENE_DATA
              : M.buildFromUi(Object.assign(M.defaultUiValues(), { SET: set, NO_REFINE: true, SELF: 0 }));   // 形だけに使う(動きは使わない)
        S.geo._set = set;
      }
      var keep = S.stopped && S.set === set;
      S.stopped = false;
      WORLD.init(S.geo, set, keep); S.set = set;
      if (keep) log("やり直す: かき乱した後の置き場のまま、写す姿勢から撮り直して選び直す（置いたピースは机の元の所へ戻す）", "act");
      var c0 = S.geo._check;
      CONTROL.setup({ pieces: c0.pieces, pitch: c0.pitch, cell: c0.cell }, WORLD.camForControl());
      // 3D は、計画ができるまで写す姿勢の腕と本当の置き場で出す
      buildWorld(S.geo); S.c = c0; S.onTable = {}; c0.pieces.forEach(function (p) { S.onTable[p.idx] = true; });
      setPieceDisplay(); applyAnims(0); S.t = 0; updateOverlays(0, true);
      // 1) 撮る
      stage("look"); status("撮る: 写す姿勢で、上のカメラの絵を作っています…"); await nextPaint();
      var q0 = c0.photoQ, im = WORLD.shootTop(q0, c0.homeJaw, S.onTable);
      // 2) 見つける
      stage("find"); await nextPaint();
      var det = CONTROL.look(im, true), ctx = putImage($("win-top"), im);
      $("cap-top").textContent = "t = 0.0 秒・写す姿勢で撮った絵（640×480・雑音 " + NOISE + "・制御のカメラは " + CAL_MM + "mm / " + CAL_DEG + "° ずらしてある【仮】）";
      if (det.why) { log("見つけられない: " + det.why + " → 動かない", "ng"); stage(null); status("止まった（見つけられない）"); return; }
      drawDetected(ctx, det, c0, -1); showEstimate(det, c0);
      log("見つけた: " + det.layout.pieces.map(function (p) { return p.name + "（" + p.x.toFixed(0) + ", " + p.z.toFixed(0) + " mm・" + ang(p.yaw).toFixed(0) + "°）"; }).join(" / "), "act");
      c0.pieces.forEach(function (pc) { var df = WORLD.diffFor(pc.idx, det.layout.pieces[pc.idx]); log("　（見る人だけ）" + pc.name + " の見積もりと本当の差: マスのずれ最大 " + df.d.toFixed(2) + "mm・向き " + df.dyaw.toFixed(2) + "°"); });
      // 3) 選ぶ
      stage("choose");
      status("選ぶ: 見積もった置き場だけで、動きを組み、当たり判定を通しています。\n形 2 でブラウザ 11〜13 秒（形や置き場で変わる）かかり、その間は画面が止まります（1 手だけ組む口がまだ無いので、全部の手をまとめて組む）。"); await nextPaint();
      var tp = performance.now(), d = CONTROL.plan(set, det.layout), c = d._check, sec = (performance.now() - tp) / 1000;
      // 止めるのは _check.ok(解けた・当たりなし・腕どうしが 0 以上。画面の判定の表と同じ)。solved は動きが組めたかだけで当たりを含まない。
      //   2026-11-18 から、絵やファイルから読んだ置き場は台の足跡まで 20mm 無くても弾かれず、当たりは判定で見る(scene-data.js の持ち主。点検 C29b)
      if (!c.ok) {
        var judge = (d.info || []).find(function (r) { return r.label === "判定"; });
        log("動かせる計画にならない（" + sec.toFixed(1) + " 秒）: " + (judge ? judge.value : (c.layoutWhy || c.why)) + " → 動かない", "ng"); stage(null); status("止まった（計画が判定を通らない）"); return;
      }
      if (c.layoutNotes && c.layoutNotes.length) log("置き場の書き置き: " + c.layoutNotes.join("／"), "act");
      S.plan = d; S.c = c; S.est = JSON.parse(JSON.stringify(det.layout));
      buildWorld(d); S.onTable = {}; c.pieces.forEach(function (p) { S.onTable[p.idx] = true; });
      // 撮り直す時刻 tLook: 1 つ目は動き出す前(0 秒・写す姿勢)。2 つ目からは、寄り道(RELOOK)があれば写す姿勢で止まっている間、
      //   無ければ前のピースを置き終えた時からそのピースの上で止まる時まで、のうち腕が見積もったピースを隠さないと制御が予測した、いちばん遅い時
      //   (CONTROL.chooseLook。2026-11-18。寄り道の間は腕が写す姿勢で止まっているので、ふつうは止まり終える時 tLook1 になる)
      var tl = performance.now();
      S.steps = c.stepTimes.map(function (st, k) {
        var s = { pid: st.pid, name: st.name, tLook: 0, look: null, tHover: st.tHover, tPick: st.tPick, tPlace: st.tPlace, checked: false, graded: false, grasp: c.grasps.find(function (g) { return g.pid === st.pid; }) };
        // 寄り道(RELOOK)があれば、写す姿勢で止まっている間 [tLook0, tLook1] から選ぶ(動いている途中で 1 コマだけ隠れない時を拾わないため。点検 C28 の 1)。
        //   無ければ、前のピースを置き終えた時からそのピースの上で止まる時まで
        if (k) {
          var relook = st.tLook1 > 0, tA = relook ? st.tLook0 : c.stepTimes[k - 1].tDone, tB = relook ? st.tLook1 : st.tHover;
          s.look = CONTROL.chooseLook(c, det.layout.pieces[st.pid], c.pieces[st.pid], tA, tB, qAt); s.look.relook = relook; s.tLook = s.look.t;
        }
        return s;
      });
      var lookSec = (performance.now() - tl) / 1000;
      log("選んだ（" + sec.toFixed(1) + " 秒）: 順番 " + c.order.join("→") + "。" + S.steps.map(function (s) {
        return s.name + " は爪の向き " + ang(s.grasp.yp).toFixed(0) + "°";
      }).join("・") + "。腕どうし " + (c.self ? mm(c.self.min) + "mm（" + c.self.pair + "）" : "見ていない") + "・いちばん近い " + mm(c.minClear) + "mm（" + c.minClearPair + "）", "act");
      log("撮り直す時刻（腕が隠さないと予測した、いちばん遅い時。" + lookSec.toFixed(1) + " 秒で選んだ）: " + S.steps.map(function (s) {
        return s.look ? s.name + " " + s.tLook.toFixed(1) + " 秒（" + (s.look.relook ? "寄り道した写す姿勢で" : "寄り道なし・動きの途中で") + "。隠れる点 " + s.look.hidden + " / " + s.look.of + (s.look.clear ? "" : "。隠れない時が無い") + "）" : s.name + " 0 秒（写す姿勢）";
      }).join("・") + (c.relook ? "。写す姿勢への寄り道 " + c.relook.n + " 回（全体 " + c.T.toFixed(1) + " 秒。寄り道で延びた分は、寄り道なしで組んだ時間と比べないと分からない）" : "。寄り道なし"), "act");
      // 隠れない時が無いピースがあれば、動き出さずに止まる(動いても撮り直しで止まり、「開始」で組み直しても同じ動きになるため。点検 C29 の 8)
      var blind = S.steps.filter(function (s) { return s.look && !s.look.clear; });
      if (blind.length) {
        log("撮り直せない: " + blind.map(function (s) { return s.name; }).join("・") + " は、腕が隠さない時が計画の中に無い" + (c.relook ? "" : "（写す姿勢への寄り道が入っていない。台なし・scene-data.js が古い、など）") + " → 動かない", "ng");
        S.plan = null; stage(null); status("止まった（撮り直せる時が無い）"); return;
      }
      // 4) 動く
      setPieceDisplay(); S.t = 0; S.playing = true; stage("move"); status("動く");
      $("play").disabled = false; $("play").textContent = "一時停止"; $("kick").disabled = false;
    } catch (e) {
      log("エラー: " + e.message, "ng"); console.error(e); status("エラー: " + e.message);
    } finally { S.busy = false; $("start").disabled = false; }
  }

  // つかみに行く前の見直し(行動の選択)。tLook の腕の姿勢で上の絵を撮り、計画に使った見積もりと比べる
  async function recheckBefore(step) {
    S.playing = false; stage("look"); status("撮り直す: " + step.name + " を取りに行く前に、上の絵を撮り直しています…"); await nextPaint();
    var qa = qAt(S.t), im = WORLD.shootTop(qa.q, qa.jaw, S.onTable);
    stage("find"); await nextPaint();
    var det = CONTROL.look(im), ctx = putImage($("win-top"), im);
    $("cap-top").textContent = "t = " + S.t.toFixed(1) + " 秒・" + step.name + " を取りに行く前に撮り直した絵（" + (!step.look ? "写す姿勢" : step.look.relook ? "寄り道した写す姿勢" : "腕が隠さないと制御が予測した時刻。写す姿勢ではない") + "）";
    stage("choose");
    var r = CONTROL.recheck(det, S.est.pieces[step.pid], step.pid, step.grasp, S.c);
    if (r.blind) {
      log(step.name + ": 撮り直したが確かめられない（" + r.why + "）→ 止まる", "ng");
      log("　「開始」を押すと、写す姿勢から撮り直して全部の手を選び直す", "act");
      S.stopped = true; stage(null); status("止まった: " + step.name + " を確かめられない"); $("play").disabled = true; $("kick").disabled = true;
      return;
    } else {
      drawDetected(ctx, det, S.c, step.pid);
      if (r.action === "go") log(step.name + ": 見積もりは計画から " + (r.moved * 1000).toFixed(2) + "mm・" + r.phi.toFixed(2) + "° → つかめる余裕の中（閉じる向き＋回し " + (r.need * 1000).toFixed(2) + " < " + mm(r.margin) + "mm・直角 " + Math.abs(r.ec * 1000).toFixed(2) + " < " + mm(r.margin) + "mm【仮】）→ つかみに行く", "ok");
      else {
        log(step.name + ": 見積もりが計画から " + mm(r.moved) + "mm・" + r.phi.toFixed(1) + "° 動いた → つかめる余裕を超える（閉じる向き＋回し " + mm(r.need) + "mm［うち回しの分 " + mm(r.turn) + "］・直角 " + mm(Math.abs(r.ec)) + "mm、余裕 " + mm(r.margin) + "mm）→ 止まる", "ng");
        log("　選び直すには「開始」を押す: いまの置き場のまま、写す姿勢から撮り直して全部の手を選び直す（1 手だけ組み直す口がまだ無い。置いたピースは机の元の所へ戻る）", "act");
        S.stopped = true; stage(null); status("止まった: " + step.name + " が計画の位置から動いている"); $("play").disabled = true; $("kick").disabled = true;
        return;
      }
    }
    step.checked = true; stage("move"); status("動く"); S.playing = true;
  }
  function gradeAt(step) {
    step.graded = true;
    var g = WORLD.grade(step.pid, step.grasp, S.est.pieces[step.pid]);
    if (g.ok) { S.onTable[step.pid] = false; log(step.name + ": つかめた（本当の置き場で採点。閉じる向きのずれ " + mm(g.ea) + "mm・向き " + g.phi.toFixed(1) + "°。直角のずれ " + mm(g.ec) + "mm は採点の式に入っていない）", "ok"); }
    else log(step.name + ": つかみ損ねた（閉じる向きのずれ " + mm(g.ea) + "mm・向き " + g.phi.toFixed(1) + "° → 余裕 " + mm(g.margin) + "mm を超える）。ピースは机に残る", "ng");
    setPieceDisplay();
  }

  $("start").addEventListener("click", startRun);
  $("play").addEventListener("click", function () {
    if (S.stopped || S.busy) return;
    S.playing = !S.playing; this.textContent = S.playing ? "一時停止" : "再生";
  });
  $("kick").addEventListener("click", function () {
    if (!S.steps.length) return;
    var next = S.steps.find(function (s) { return !s.checked && s.tLook > S.t + 0.3; });   // まだ撮り直していないピースだけ(撮り直した後に動かすと、気づかずにつかみ損ねる)
    if (!next) { log("かき乱せるピースが無い（どれも取りに行った後）", "act"); return; }
    var k = WORLD.kick(next.pid);
    log("かき乱した: 次に取りに行く " + next.name + " を、乱数の向きに " + KICK_MM + "mm・回し ±" + KICK_DEG + "° 動かした（世界の側。制御はまだ知らない。動かした後の置き場が使えるかは見ていない）", "act");
    updateOverlays(S.t, true);
  });

  function updateOverlays(t, forceWrist) {
    updateFrustum(frTop, WORLD.topCam(), 640, 480, FOV_TOP);
    var qa = S.c && S.c.motion ? qAt(t) : { q: S.geo._check.photoQ, jaw: S.geo._check.homeJaw };
    updateFrustum(frWrist, WORLD.wristCam(qa.q, qa.jaw), 640, 480, FOV_WRIST);
    // 手首の窓は 1 秒おき(1 枚 0.3〜0.5 秒かかるので、毎コマは出さない)
    var now = performance.now();
    if (forceWrist || now - S.wristAt > 1000) {
      S.wristAt = now;
      var im = WORLD.shootWrist(qa.q, qa.jaw, S.onTable); putImage($("win-wrist"), im);
      $("cap-wrist").textContent = "t = " + t.toFixed(1) + " 秒・320×240・1 秒おきに更新。表示だけ（判断にはまだ使っていない。09 §6-3）";
    }
  }

  var last = performance.now();
  function tick() {
    requestAnimationFrame(tick);
    var now = performance.now(), dt = Math.min(0.1, (now - last) / 1000); last = now;
    advance(dt);
    controls.update();
    renderer.render(scene, camera);
  }
  // 時間を dt 秒進める(tick から毎コマ呼ぶ。確かめの道具からも呼べるように window.LIVE に出す)
  function advance(dt) {
    dt = Math.min(dt, 0.1);   // 大きく進めると、前のピースの採点より先に次の撮り直しが走るので切る
    if (S.plan && S.playing && !S.busy) {
      var tNext = Math.min(S.t + dt, S.c.T);
      var step = S.steps.find(function (s) { return !s.checked && s.tLook <= tNext; });
      if (step) { S.t = Math.max(S.t, step.tLook); applyAnims(S.t); recheckBefore(step); }
      else {
        S.t = tNext;
        S.steps.forEach(function (s) { if (!s.graded && s.checked && s.tPick <= S.t) gradeAt(s); });
        if (S.t >= S.c.T) { S.playing = false; stage(null); status("終わり"); log("終わり: つかめた " + S.steps.filter(function (s) { return !S.onTable[s.pid]; }).length + " / " + S.steps.length, "act"); $("kick").disabled = true; }
      }
      applyAnims(S.t);
      if (S.c && S.geo) updateOverlays(S.t, false);
    }
  }
  window.LIVE = { S: S, advance: advance, start: startRun };   // 確かめ用(画面が隠れていて requestAnimationFrame が来ないときに、外から進める)。判断には関係しない

  // =========================================================================
  //  読み込み
  // =========================================================================
  async function boot() {
    $("limits").innerHTML = "画角 上 " + FOV_TOP + "°・手首 " + FOV_WRIST + "°【仮】・雑音 " + NOISE + "【仮】・制御のカメラのずれ " + CAL_MM + "mm / " + CAL_DEG + "°【仮】。" +
      "<br>まだできないこと: 1 手だけ組み直す口が無い（選ぶ間は画面が止まる・止まったら全部やり直し）／撮り直しは写す姿勢への寄り道（RELOOK）で撮る。寄り道の分だけ時間が延びる。寄り道が無いときは、動きの途中で腕が隠さないと予測した時に撮り、そういう時が無ければ動き出さない／確かめられないときは止まる／当たりは見積もった置き場でしか見ていない（採点はつかめたかだけ）／手首の絵は判断に使っていない。";
    $("loading").textContent = "scene-data.js を読み込み、最初の形を軽く組んでいます（全部の判定はしない。測った時間は 09 §6-6）…";
    await nextPaint();
    try {
      await loadScript("scene-data.js");
      await loadScript("vision.js");
    } catch (e) { $("loading").textContent = "読み込めないファイル: " + e.message; return; }
    M = SCENE_MODEL; V = VISION;
    var sel = $("set");
    M.controls.find(function (c) { return c.key === "SET"; }).options.forEach(function (o) {
      var op = document.createElement("option"); op.value = o.value; op.textContent = o.label; sel.appendChild(op);
    });
    sel.value = M.defaultUiValues().SET;
    // 最初の表示: 既定の形の世界を、写す姿勢で出す
    var set = +sel.value;
    S.geo = (typeof SCENE_DATA !== "undefined" && SCENE_DATA && SCENE_DATA._check) ? SCENE_DATA : M.buildFromUi(Object.assign(M.defaultUiValues(), { NO_REFINE: true, SELF: 0 }));
    S.geo._set = set;
    WORLD.init(S.geo, set);
    var c0 = S.geo._check;
    CONTROL.setup({ pieces: c0.pieces, pitch: c0.pitch, cell: c0.cell }, WORLD.camForControl());
    buildWorld(S.geo); S.c = null; S.onTable = {}; c0.pieces.forEach(function (p) { S.onTable[p.idx] = true; });
    c0.pieces.forEach(function (pc) { WORLD.setTruthVisible(pc.idx, true); if (nodes["pc" + pc.idx]) nodes["pc" + pc.idx].visible = false; });
    // 写す姿勢へ(既定の動きの 0 秒 = 写す姿勢)
    applyAnims(0);
    updateOverlays(0, true);
    var sh = (S.geo.cameraShots || [])[0];
    if (sh && sh.position) { camera.position.set(sh.position[0], sh.position[1], sh.position[2]); controls.target.set(sh.target[0], sh.target[1], sh.target[2]); }
    else { camera.position.set(0.5, 0.45, 0.6); controls.target.set(0.12, 0.05, 0); }
    $("loading").style.display = "none";
    status("「開始」で、撮る → 見つける → 選ぶ → 動く を始める");
    tick();
  }
  boot();
})();
