#!/usr/bin/env bash
# モックを headless Chrome で複数タイミング撮影する（Bashツールから実行）
#
#   bash .claude/scripts/mock-shot.sh <モックフォルダ> [出力先] [秒 秒 ...]
#   例: bash .claude/scripts/mock-shot.sh "ちくわ排除機構" "$SCRATCH/_check" 2 6 11
#
# 【これは暫定手段】本来は mock3d スキルの scripts/verify_mock.js を使う。
# スキルが入っていない環境向けの代替。スキルが入ったらそちらを優先すること。
set -u

SRC="${1:?モックフォルダを指定してください}"
OUT="${2:-_check}"
shift 2 2>/dev/null || shift $#
TIMES=("$@")
[ ${#TIMES[@]} -eq 0 ] && TIMES=(2 6 11)

WINDOW="${WINDOW:-1280,800}"

# GPU（RTX 2050）で描く。--disable-gpu を付けて SwiftShader に落とすと、
# 同じ絵でも1枚1秒が最悪108秒まで暴れる（実測。特に高解像度＋長い秒数で顕著）。
# GPUなら解像度・秒数によらず1〜2秒で安定するので、解像度を上げても割に合う。
# GPUが使えない環境に持っていったときだけ SwiftShader に自動で落ちる（下のフォールバック）。
GPU_FLAGS=(--use-angle=d3d11 --disable-software-rasterizer)
SW_FLAGS=(--disable-gpu --use-gl=angle --use-angle=swiftshader --enable-unsafe-swiftshader)

# 専用プロファイルを固定パスで使い回す。ユーザーの普段のChromeと分離しつつ、
# シェーダキャッシュを残すため。
#   ・毎回 mktemp で使い捨てにすると初回コンパイルを毎回踏む
#   ・逆にプロファイルを省略すると既存Chromeとシングルトン競合して待たされる
# ★2026-09-15: プロファイルが壊れる(Chrome を強制終了した後など)と GPU 側の撮影が失敗し、
#   SwiftShader へ落ちて 1 枚が 55 分回った(F-164)。壊れたら消して作り直す。
PROFILE="${PROFILE:-${TMPDIR:-/tmp}/claude-mock-chrome-v2}"
mkdir -p "$PROFILE"

CHROME="/c/Program Files/Google/Chrome/Application/chrome.exe"
[ -x "$CHROME" ] || CHROME="/c/Program Files (x86)/Google/Chrome/Application/chrome.exe"
[ -x "$CHROME" ] || { echo "Chrome が見つかりません"; exit 1; }

INDEX="$SRC/index.html"
[ -f "$INDEX" ] || { echo "index.html がありません: $INDEX"; exit 1; }

mkdir -p "$OUT"
# 日本語フォルダ名があるので file:// URL は encodeURI で作る
URL=$(node -e 'const p=require("path").resolve(process.argv[1]).replace(/\\/g,"/");process.stdout.write(encodeURI("file:///"+p))' "$INDEX")
# 条件やカメラを指定したいときは QUERY 環境変数に付ける
#   例: QUERY='V=90&GAP=150&_shot=2&_t=3.0' bash mock-shot.sh "ちくわ排除機構" _check 4
[ -n "${QUERY:-}" ] && URL="$URL?$QUERY"
# ファイル名の衝突を避けたいときは PREFIX を付ける
PREFIX="${PREFIX:-}"
# Chrome の --screenshot は Windows 形式の絶対パスでないと書けない
OUT_WIN=$(node -e 'process.stdout.write(require("path").resolve(process.argv[1]))' "$OUT")

shoot(){ # $1=出力PNG(Windows形式) $2=秒 / 以降=描画フラグ
  local png=$1 t=$2; shift 2
  "$CHROME" --headless=new \
    --user-data-dir="$PROFILE" --no-first-run --no-default-browser-check \
    "$@" \
    --allow-file-access-from-files --window-size="$WINDOW" \
    --virtual-time-budget=$((t * 1000)) \
    --screenshot="$png" "$URL" >/dev/null 2>&1
}

for t in "${TIMES[@]}"; do
  NAME="${PREFIX}t${t}s.png"
  shoot "$OUT_WIN\\$NAME" "$t" "${GPU_FLAGS[@]}"
  MODE="GPU"
  if [ ! -f "$OUT/$NAME" ]; then
    # ★SwiftShader は暴れる(最悪 55 分回った。F-164)。180 秒で打ち切る
    timeout 180 "$CHROME" --headless=new --user-data-dir="$PROFILE" --no-first-run --no-default-browser-check "${SW_FLAGS[@]}" --allow-file-access-from-files --window-size="$WINDOW" --virtual-time-budget=$((t * 1000)) --screenshot="$OUT_WIN\$NAME" "$URL" >/dev/null 2>&1
    MODE="SwiftShader(GPU失敗のため)"
  fi
  if [ -f "$OUT/$NAME" ]; then
    echo "OK  t=${t}s  [$MODE]  -> $OUT/$NAME"
  else
    echo "NG  t=${t}s  撮影に失敗（scene-data.js の構文エラーを疑う）"
  fi
done
