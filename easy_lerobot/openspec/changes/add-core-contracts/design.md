# Design

## Context

- 動機は proposal.md の Why を参照。要件は specs/ の 3 本（step-contract、project-state、command-runner）を参照。
- 本番環境はラズパイ（Linux、arm64）、開発環境は各自の Windows PC。どちらも CPU のみ・uv の `.venv`（Python 3.12）。配置全体は add-raspberry-pi-deployment を参照。プロジェクトのパスに全角文字と空白を含みうる。
- Streamlit は操作のたびにスクリプト全体を再実行し、`session_state` はタブごとに消える。長い処理はプロセスを分け、状態はファイルで受け渡す。
- LeRobot 本体（`src/lerobot`）は改変しない。ソースから確認した入力待ちは次の 3 種類で、届け方がそれぞれ違う。
  - `input()`（lerobot-find-port の抜き差し待ち、lerobot-calibrate の「既存を使うか」「中間姿勢で ENTER」）→ どちらの OS でも標準入力に届く。
  - `enter_pressed()`（lerobot-calibrate の可動域記録の停止、`src/lerobot/utils/utils.py`）→ Linux では標準入力を `select` して 1 行読むため届く。Windows では `msvcrt.kbhit()` でコンソールを直接読むため、パイプでは届かない。
  - 録画のキー操作（lerobot-record の → ← Esc、`src/lerobot/utils/keyboard_input.py`）→ Linux で画面表示がない（`DISPLAY` も `WAYLAND_DISPLAY` もない）ときは、標準入力が端末（`isatty()`）であれば端末からキーを読む。標準入力がパイプだと何もしない。画面表示があると pynput の全体監視を使う。Windows では常に pynput。

## Goals / Non-Goals

**Goals:**
- E 班が core を作る前でも、A〜D 班が同じ型・同じファイル形式を前提に各ページを書き始められること。
- 画面プロセスの再起動・ブラウザの再読み込みに対して、実行と状態遷移が壊れないこと。

**Non-Goals:**
- 画面の文言・配置・見た目。
- 学習をどこで回すか（HF Jobs・学校の GPU・Colab）。`train` ステップの引数の組み立てに閉じるため、この設計の構造には影響しない。
- 複数 PC・複数利用者での同時利用。

## Decisions

### D1. Runner と lerobot-* の間に「見張り役」プロセスを挟む（確定: 2026-10-15 承認）
画面 → 見張り役（`python -m core.run_wrapper`）→ lerobot-* の 3 段にする。見張り役が子の出力をログへ書き、終了時に run.json へ `exit_code`・`ended_at`・`status` を書く。
- 理由: 画面が `Popen` で直接 lerobot-* を起動すると、画面を再起動した時点で終了コードを受け取る手段がなくなる（command-runner の「画面を落としている間に学習が終わる」を満たせない）。
- 代替案: 画面側で PID を監視して終了を推定する → 終了コードが取れず成功・失敗を区別できないため不採用。

### D2. プロセスの起動方法
- 本番（Linux）: 見張り役を `start_new_session=True`（新しいセッションとプロセスグループ）で起動する。見張り役は `pty.openpty()` で仮想端末を作り、その子側を lerobot-* の標準入力・標準出力・標準エラー出力にする。lerobot-* の環境変数からは `DISPLAY` と `WAYLAND_DISPLAY` を外す。
- 開発（Windows）: 見張り役を `CREATE_NEW_PROCESS_GROUP` で起動し、lerobot-* とは標準入力・標準出力のパイプでつなぐ。
- 理由: 新しいセッションにしないと、Streamlit を止めたときに録画・学習まで止まる。仮想端末にすると、Context に挙げた 3 種類の入力待ちがすべて「端末への書き込み」で届く。画面表示の変数を外すのは、デスクトップ付きのラズパイでも lerobot-record に端末からキーを読ませるため。
- 代替案: 標準入力のパイプだけにする → 録画のキー操作が届かない（端末でないと読まない）ため不採用。pynput のキー送信 → 画面表示が要り、フォーカスに依存するため不採用。
- 仮想端末は出力を行単位で返さず、改行が `
` になるため、見張り役はログへ書く前に `
` を `
` にそろえる。

### D3. 子プロセスの環境変数
`PYTHONUNBUFFERED=1`、`PYTHONIOENCODING=utf-8`、`PYTHONUTF8=1` を付けて起動する。
- 理由: バッファリングがあるとログが 2 秒以内に出ない。Windows の既定（cp932）では `·` などの出力で `UnicodeEncodeError` になる（本仕様の作成中にも同じ現象を確認済み）。

### D4. 停止の手順
- 本番（Linux）: lerobot-record にはまず Esc を送る（lerobot 自身の「録画をやめる」操作で、撮った分を保存して終わる）。それ以外のコマンド、または Esc の後も終わらないときは、プロセスグループに `SIGINT` を送る。合計 10 秒で終わらなければ `SIGKILL` をプロセスグループに送る。
- 開発（Windows）: `CTRL_BREAK_EVENT` をプロセスグループに送り、10 秒待って終わらなければ `taskkill /T /F /PID <見張り役>` で子ごと止める。
- 理由: Esc は lerobot-record の正規の終了経路なので、データセットの確定が保証されやすい。`SIGINT` は Python では `KeyboardInterrupt` になり、後片付けの処理が走る。
- 注意: Windows の CTRL_BREAK は `KeyboardInterrupt` ではなく `SIGBREAK` として届き、既定ではそのまま終了する。開発環境では録画データの保全は保証しない。

### D5. 入力の中継
画面 → 見張り役は stdin のパイプでつなぐ。見張り役は受けたバイト列を、本番では仮想端末の親側へ、開発では子の stdin へそのまま書く。キーは次のバイト列で送る: Enter は `
`、→ は `[C`、← は `[D`、Esc は `` のみ。パイプが閉じたら（画面が終了したら）入力の中継をやめる。
- 理由: 見張り役を挟んでも入力待ちを画面のボタンから進められる。矢印キーの 3 バイトは 1 回の書き込みで送り、Esc の単独送信と区別させる（lerobot は Esc の後 0.02 秒以内に続きが来なければ単独の Esc とみなす）。
- 画面再起動後は送れなくなる。find-port と calibrate は短い対話処理なので、中止してやり直す運用で足りる。録画中に画面を再起動した場合は、停止（D4）だけを行える（command-runner の「画面を再起動した後」）。

### D6. 状態遷移の反映を 1 か所に集める（確定: 2026-10-15 承認）
- `Step.status` は project.json と run.json から計算するだけの純粋な読み取りにする。
- 終了の反映は core の `sync(project)` だけが行う。画面の定期読み込み（`st.fragment(run_every=1s)`）から毎回呼び、D9 のロックを取って project.json を読み直したうえで、run.json が終了済みかつ `run_id != project.last_applied_run_id` のときだけ、該当ステップの `on_finished(run)` を呼んで遷移と `progress` を確定し、`last_applied_run_id` を更新して保存する。
- `on_finished` は step-contract の 3 操作に加える 4 つ目のフック（各班が実装）。成功の意味（例: record は規定数そろったか）はステップごとに違うため、ステップ側に置く。
- 代替案: 各ページがそれぞれ終了を検知して遷移させる → 二重反映や班ごとの解釈違いが起きやすいため不採用。

### D7. check() の戻り値（確定: 2026-10-15 承認）
`CheckResult(ok: bool, reasons: list[str])` を返す。UML 図では `bool` だったが、画面に「なぜ押せないか」を出すために理由の一覧を足す。

### D8. ファイル操作
- project.json は同じディレクトリの一時ファイルに書いてから `os.replace` で置き換える。
- プロセスの生存確認は、本番では `/proc/<pid>/stat`（開始時刻を含む）、開発では `ctypes` の `OpenProcess` / `GetExitCodeProcess` を使い、新しい依存（psutil など）は増やさない。PID の再利用対策として、run.json の `started_at` とプロセスの作成時刻も照合する。

### D9. プロジェクト単位のロックで同時更新を排他する（確定: 2026-10-16 承認）
`core/project.py` に `project_lock(name)` を用意し、次の処理をすべてその中で行う。
- `sync()` による終了の反映
- `Step.run()` による開始（run.json の確認 → 見張り役の起動 → run.json と project.json の書き込み）
- 利用者の操作による project.json の更新（設定の保存、プロジェクトの新規作成など）

ロックの中では必ず project.json をディスクから読み直す。`session_state` に置いた Project は表示用の写しとして扱い、書き込みの元にはしない。

ロックは 2 段にする。
1. **プロセス内**: プロジェクトごとの `threading.Lock`。Streamlit はすべてのタブを 1 つのサーバープロセスのスレッドとして動かすため、2 タブ問題の大半はここで防げる。
2. **プロセス間**: `projects/<名前>/project.lock` を開き、本番では `fcntl.flock(LOCK_EX | LOCK_NB)`、開発では `msvcrt.locking(LK_NBLCK)` で先頭 1 バイトをロックする。取れなければ 0.1 秒おきに再試行し、5 秒で諦める。Streamlit を 2 つ起動した場合や、今後 CLI から触る場合に備える。

- 理由: OS のファイルロックはハンドルを閉じたとき、またはプロセスが終了したときに OS が自動で外す。そのため、画面が落ちてもロックが残らない（project-state の「ロックを持ったまま画面が落ちた」）。標準ライブラリだけで済み、依存も増えない。
- 代替案 1: `project.lock` を排他作成（`O_CREAT | O_EXCL`）し、終わったら消す → 異常終了するとファイルが残り、手作業で消すまで誰も操作できなくなるため不採用。
- 代替案 2: `filelock` ライブラリ → torch や huggingface_hub の依存として環境には入っているが、使うなら直接の依存として宣言する必要がある（依存の追加は承認が要る）。中身は同じ OS ロックなので、標準ライブラリで足りると判断した。
- ロックは長く持たない。lerobot の処理の完了を待つ間は持たず、起動とファイルの書き込みの間だけ持つ。

### D10. OS ごとの差を 1 か所に閉じ込める
D2・D4・D5・D8・D9 の OS ごとの処理は `core/platform.py` の中だけに書き、`sys.platform` で切り替える。ほかの core とページのコードは OS を意識しない。
- 理由: 本番はラズパイだが、メンバーの PC は Windows で、ダミーのコマンドによるテストは各自の PC で回したい。
- テスト: OS に依存しないテストはどちらでも動かす。仮想端末やシグナルを使うテストは Windows では skip し、ラズパイ（または WSL）で動かす。

## Risks / Trade-offs

- [Esc や SIGINT で止めたときに lerobot-record がデータセットを確定しない] → 実機で確かめるタスク（5.3）を入れる。駄目なら停止方式を後続の変更で決める。
- [lerobot の今後の更新で、端末からキーを読む条件が変わる] → 条件は `keyboard_input.py` の `create_key_listener` にある。upstream を取り込むときに 5.2・5.3 を再確認する。
- [仮想端末の出力に色付けの制御文字が混じり、ログから進捗を読み取りにくい] → 進捗の読み取りは数値の正規表現で行い、制御文字はログ表示の前に取り除く。
- [見張り役が異常終了すると run.json が running のまま残る] → 開くときの生存確認で検出し、`failed` として確定させる。
- [PID の再利用で、別のプロセスを実行中と誤認する] → D8 の作成時刻照合で防ぐ。
- [ロックの中の処理が遅いと、もう一方のタブで「別の画面で処理中です」が出る] → ロック中はファイルの読み書きとプロセスの起動だけにし、カメラの確認などの時間がかかる処理はロックの外で行う。
- [プロジェクト間で同じアームを同時に使う] → 本変更では 1 プロジェクト内の排他のみ。複数プロジェクトを同時に開く運用はしないと決めておく。

## Migration Plan

新規ツールのため移行はない。project.json は `schema_version: 1` から始め、項目を増やすときは版を上げ、読み込み時に既定値で補う。

## Open Questions

- 学習を回す場所（HF Jobs・学校の GPU・Colab）。`train` の引数組み立てと進捗の読み方だけが変わる。
- Hub アカウントを利用者ごとにするか、チーム共有にするか。`dataset_repo_id` / `policy_repo_id` の既定値だけが変わる。
- `connected` 以降の状態でアームが抜けたときの扱い（現状は `connected → disconnected` のみ定義）。
- 規定数に届く前に「録画をやめる」で止めた場合、撮れた分のエピソードを `progress.episodes_done` に数えて `has_data` にするか（現状は `fallback_state` に戻る）。
