# 前期レポート リサーチパック

Research material for two 前期レポート課題:

| Folder / file | Report |
|---|---|
| `01_dl_shokugyou-henyou.md` | ディープラーニング「AI の進化がもたらす 3〜10 年後の職業変容」 |
| `02_ml_freshmart.md` | 機械学習「理想と現実の板挟みになる惣菜売り場」ケーススタディ |
| `sources.md` | All sources (web + YouTube), with what each one is good for |

---

## How to use this in VS Code

Suggested layout:

```
report/
├── research/          ← this pack (read-only reference)
├── dl_report.md       ← you write here
├── ml_report.md       ← you write here
└── out/               ← exported .docx / .pdf
```

Useful extensions:
- **Markdown All in One** — TOC, preview, word count
- **markdownlint** — keeps headings consistent
- **Pandoc** (`pandoc dl_report.md -o out/dl_report.docx`) — Markdown → Word.
  A4 3〜5 枚 is roughly **4,500〜7,500 字** in Japanese. Check with word count as you go.

Write in Markdown, export to .docx at the end. Do not write directly in Word —
you lose version history and it is harder to restructure.

---

## Important cautions before you use any of this

**1. YouTube is weak as a citation.**
For an academic report, a YouTube lecture is acceptable only when the speaker is an
authority (e.g. 松尾豊 / 東大, 日本ディープラーニング協会). Use videos to *understand*
and to find the names of concepts; then cite the underlying 白書・研究・統計 where possible.
Cite format for video: 話者名「動画タイトル」チャンネル名, 公開日, URL (最終閲覧日).

**2. I have not watched these videos.**
The list in `sources.md` was assembled from titles and descriptions. Verify before
citing anything — do not attribute a claim to a video you have not confirmed.

**3. Both rubrics demand things no external source can supply.**
- DL 課題: 「人工知能学科で学んだ AI の技術的な特徴」— it must connect to *your* lectures.
- ML 課題: 「講義で学んだ『データ分析プロジェクトが陥るどんな失敗や罠』」— same.

So: open your 講義スライド next to this pack. Where this pack says
「技術的根拠：〜」, replace it with the terminology your instructor actually used.
That substitution is most of the difference between a passing report and a good one.

**4. Everything below is 素材, not 文章.**
Bullet points and candidate lists on purpose. If you paste them as-is, the report
will read like a listicle and will not satisfy 「論理的に納得できる」.
