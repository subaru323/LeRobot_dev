// node から画面用のファイル(グローバル変数を作る .js)を読み込む。
// 画面と同じファイルを同じ順で読むので、検算と絵が別の値を見ることがない。
const fs = require("fs"), path = require("path"), vm = require("vm");
const ROOT = path.resolve(__dirname, "..");
function load(...names) {
  for (const n of names) vm.runInThisContext(fs.readFileSync(path.join(ROOT, n), "utf8"), { filename: n });
}
module.exports = { load, ROOT };
