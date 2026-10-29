# Tasks

## 1. 足場づくり（E 班）

- [ ] 1.1 `easy_lerobot/` に `app.py`、`pages/`、`core/`、`templates/`、`tests/` を作り、`projects/` を `.gitignore` に入れる。`git status` に `projects/` が出ないことで確認する
- [x] 1.2 Streamlit と pytest を依存に加える（依存の追加はすばる様の承認後）。`uv run streamlit hello` と `uv run pytest --version` が通ることで確認する

## 2. project.json（E 班）

- [ ] 2.1 `core/project.py` に Project の読み込み・保存（一時ファイル＋`os.replace`）と新規作成を実装し、新規作成・同名拒否・未知キー保持・`ext.<班名>` 保持のテストが通ることで確認する
- [ ] 2.2 状態遷移表を実装し、許可された全遷移が通り、表にない遷移（例: `connected → training`）が拒否されるテストで確認する
- [ ] 2.3 `schema_version` が古いファイル・`state` が定義外のファイルの読み込みテストが通ることで確認する
- [ ] 2.4 `project_lock(name)`（プロセス内の `threading.Lock` と `project.lock` の OS ロックの 2 段。OS ロックは design D9・D10 のとおり `core/platform.py` に置く）を実装する。次の 3 つのテストが通ることで確認する: 2 スレッドが同時に `progress` を 1 ずつ増やして合計が 2 になる／別プロセスがロックを持っている間は 5 秒で諦めて何も書かない／ロックを持ったプロセスを強制終了した直後に別プロセスがロックを取れる

## 3. Runner と見張り役（E 班）

- [ ] 3.0 `core/platform.py` に OS ごとの処理（起動・入力の中継・停止・生存確認・ファイルロック）の入口を作り、Windows と Linux の両方で import でき、未対応の操作が分かりやすい例外を出すことをテストで確認する
- [ ] 3.1 `core/run_wrapper.py` を実装する。ダミーの Python スクリプトを子にして、ログが UTF-8 で 2 秒以内に出ること、終了時に run.json の `exit_code`・`ended_at`・`status` が書かれることをテストで確認する
- [ ] 3.2 `core/runner.py` に起動（design D2 の新しいセッション／プロセスグループ、仮想端末、環境変数、`.prev.log` への退避）と二重起動の拒否を実装する。拒否は `project_lock` の中で行う。全角・空白を含む一時ディレクトリでの起動テストと、2 スレッドから同時に起動して子が 1 つだけになるテストで確認する
- [ ] 3.3 入力の中継（design D5）を実装する。`input()` を 2 回呼ぶダミースクリプトが空行と `c` を受け取って進むテスト（両 OS）と、端末からキーを読むダミースクリプトが →・←・Esc を区別して受け取るテスト（Linux のみ）で確認する
- [ ] 3.4 停止（design D4: Linux は Esc → SIGINT → SIGKILL、Windows は CTRL_BREAK → `taskkill /T /F`）を実装し、合図を無視するダミーでも 10 秒強で止まり `cancelled` になるテストで確認する
- [ ] 3.5 画面を止めた状態でダミーが終了しても run.json に結果が残ること、生存確認（PID＋作成時刻）で異常終了を `failed` と判定できることをテストで確認する

## 4. Step の型と状態の反映（E 班）

- [ ] 4.1 `core/step.py` に Step の基底（`check`/`run`/`cancel`/`on_finished`/`status`）と `CheckResult` を定め、全 5 ステップのダミー実装で status 値の表（例: `has_data` のときの各ステップの値）のテストが通ることで確認する
- [ ] 4.2 `core/flow.py` に `sync(project)` を実装する。同じ run.json に対して 3 回呼んでも遷移と `progress` 更新が 1 回だけ起きるテストと、2 スレッドから同時に呼んでも 1 回だけ起きるテストで確認する
- [ ] 4.3 各班向けに、Step の実装例と project.json の例を `easy_lerobot/README.md` に書き、例のコードがテストから import できることで確認する

## 5. 実コマンドでの確認（A・B 班と E 班）

- [ ] 5.1 lerobot-find-port を Runner 経由で起動し、画面ボタンからの改行で抜き差し待ちを進められることを実機で確認し、結果を design.md に追記する
- [ ] 5.2 ラズパイで lerobot-calibrate を Runner 経由で起動し、`input()` の 2 問と可動域記録の停止がすべて画面のボタンから進むことを実機で確認する
- [ ] 5.3 ラズパイで lerobot-record を Runner 経由で起動し、→・← が画面から効くこと、Esc で止めたときに録画済みエピソードとデータセットが読める状態で残ることを確認する。デスクトップ画面のあるラズパイでも同じ結果になることを確かめる。駄目なら停止方式の変更を別の変更提案として起こす

## 6. 通し確認

- [ ] 6.1 ダミーステップ 5 つで `disconnected → … → demo → trained` まで画面から一周でき、途中でブラウザを閉じて開き直しても続きから再開できることを確認する
- [ ] 6.2 同じプロジェクトを 2 つのタブで開いたまま 6.1 を行い、`progress` の数が二重に増えないこと、開始ボタンを両方のタブで押しても 1 つしか起動しないことを確認する
