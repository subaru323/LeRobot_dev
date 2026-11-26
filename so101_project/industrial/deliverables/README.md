# 初回に返す 4 点（架空案件の見本）

[計画書](../../docs/plan/計画書_産業用フィジカルAIと事業化.md) が初回に返すと言う 4 点の置き場。**どれも架空案件の見本**で、数字は架空セル（`industrial/`）の出力だけから写した。2026-11-19 夜（列 I の 1 回目、点検 C42f）。

| 4 点 | 置き場 | 作り直し方（`MAIN_PG\lerobot\.venv` の python） |
|---|---|---|
| ① 比較の動画 | `outputs\industrial\20261119_225444_video\compare_normal_swap_visible.mp4`（正常と取り違え × いまのやり方と提案）、`outputs\industrial\20261119_225454_video\compare_miss_once_stale_image.mp4`（空振りと古い絵）。同じ所に要所の絵（`_keyframes.png`）と時刻の表（`_timeline.json`） | `python industrial\video.py`、`python industrial\video.py --cases miss_once,stale_image` |
| ② 判断の履歴 | `outputs\industrial\20261119_231558_history\判断の履歴.md`（31 ケース × 2 つのやり方。2 回目に初見 2・3 の 8 ケースを足して作り直した。前の版は `20261119_225504_history`。ケース番号・入力の版・流し方・出力の場所・時刻ごとの受け取った物と判断） | `python industrial\history_md.py` |
| ③ 機種の差分表 | `outputs\industrial\20261119_225414_transfer\機種の差分表.md`、同じ所に `検証の依頼票.md` と `drafts.json` | `python industrial\transfer.py selftest` |
| ④ 営業の提案見本 | [営業の提案見本_架空案件.md](営業の提案見本_架空案件.md) | 手で書いた（数字は `python industrial\run.py unseen` の出力から） |

## 読むときの注意

- 動画の各枠の絵・動作・判断は、その制御が受け取った物だけ。枠の題（ケースの名前と説明）は見る人のための物で、制御には渡していない。「採点」の帯は、制御に渡さない本当の記録から出し、その回の工程が終わった時刻の後に出る（点検 C42f で、古い絵のケースで帯が早く出ていたのを直した）。
- 機種の差分表の SO-101 の行も、要る機能の一部が「一部」「無い」（時間の期限・設備の信号の口・爪の閉じた信号）なので、変換は断る形にした。
- mp4 は OpenCV の `mp4v` で書いた。Windows の「メディア プレーヤー」で開ける見込み（この PC で再生はしていない。OpenCV で全部のコマを読み戻せることだけ確かめた）。ブラウザでは開けないことがある。
- 機種の差分表は、メーカーの公開ページで確かめた値だけを書いた。メーカーの環境（ROBOGUIDE・WINCAPS III）では何も試していない。デンソーウェーブの機種はまだ選んでいない。
- 初見のケースは、制御を書いた者が作った。「調整に使っていない」の意味で、目隠しではない。
- 2026-11-19 夜の列 I の 2 回目（点検 C42h）で、候補を v3 に、採点を「追跡番号の無い部品を箱に入れたら誤排除」に直した。① の動画と ④ の数字は 1 回目の版（v1）のまま。開発用の結果は v1 と v3 で同じ。初見の数は [industrial/README](../README.md) の「2 回目の直し」。
