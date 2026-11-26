# core のマスター（このフォルダだけ。埋め込みには含めない）

`core_master/` は core のマスター。プロジェクトに埋め込むと `docs/core/` になる。`MASTER.md`、`CHANGELOG.md`、`tools/seed.py` は埋め込まない。

## プロジェクトに埋め込む

```
python core_master/tools/seed.py <プロジェクトのフォルダ>
```

- `docs/core/` に core を写し、版とファイルごとのハッシュを `docs/core/MANIFEST.json` に残す。ファイルには読み取り専用の印を付ける。
- `.claude/settings.json` が無ければ、`docs/core/` の編集と新規作成を禁じる決まりで作る（Claude Code の Edit・Write が止まる。2026-10-14 に確かめた）。シェルのコマンドで消すことまでは止められない。**既にあれば書き換えず**、足りない決まりを表示するので、手で足す。何もしないときは `--no-settings`。
- 無ければ作る: `CLAUDE.md`、`STATUS.md`、`docs/README.md`、`docs/records/R001_LOG.md`、`docs/rules`・`guides`・`plan`・`knowledge`・`core_proposals`。**既にあるファイルは、どれも上書きしない**（CLAUDE.md が既にあれば、`docs/core/templates/CLAUDE.md` を見て手で足す。既にあるファイルが変わらないことを 2026-10-14 に確かめた）。

**自分の置き場所をすでに持つ、中盤以降のプロジェクト**には `--core-only` を付ける。`docs/core/` と `docs/core_proposals/` だけを作り、CLAUDE.md や docs のほかのフォルダは作らない。CLAUDE.md から `docs/core/方針.md` を指す1行は手で足す（自動で読まれる入口から指さないと、core は読まれない見込みが高い）。

## 新しい版に入れ替える

```
python core_master/tools/seed.py <プロジェクトのフォルダ> --update
```

前の版の `MANIFEST.json` と今のファイルを比べ、プロジェクトが core を変えていたら一覧を出して止まる（変えた中身を修正案として拾ってから、`--force` で入れ替える）。古い版は消さずに `docs/core_previous/v<版>/` に移す。そこに同じ版が既にあれば、上書きせずに止まる。入れ替えた後、CLAUDE.md と `docs/README.md` の版の表記は手で直す。

## 修正案の流れ

1. プロジェクトの AI が、`docs/core_proposals/` に修正案を1件1ファイルで書く。
2. 利用者が、このフォルダの会話に「〇〇プロジェクトの core の修正案を見て」と持ってくる。
3. ここで、受け入れる・見送る・確かめる、を決めて直し、版を上げる。

## マスターの直し方

- 足りなければ足し、直すべきならその場で直す。どこでも完全に当てはまることは目指さない（中〜大規模のソフト・ハードのプロジェクトで使えればよい）。
- 直したら `VERSION` を上げ、`CHANGELOG.md` に1行書く（何を、なぜ、どの修正案から）。根拠が変わったら `根拠.md` も直す。
- 言い回しや誤字: 機械検査（`tools/doc_lint.py`）とリンクの確認だけ。
- 方針・ルールの追加や変更: R-001 の点検を通す。根拠の強さ（確かめた / 利用者の判断 / 先行研究 / 試行中）を書く。大きな変更は、必要なら「ルールが守られるかの試験」（このフォルダの `harness/rulefollow.py`）で確かめる。
