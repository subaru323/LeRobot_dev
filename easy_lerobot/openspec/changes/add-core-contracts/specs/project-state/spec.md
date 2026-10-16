# Spec Delta

## Purpose

プロジェクトごとの設定と進み具合を `projects/<名前>/project.json` に保存し、ツールやブラウザを閉じても続きから再開できるようにする。5 班のページが同じ項目と同じ状態遷移を前提に動くための約束を定める。

## ADDED Requirements

### Requirement: プロジェクトの置き場所
システムは 1 つのプロジェクトを `easy_lerobot/projects/<プロジェクト名>/` の 1 ディレクトリとして扱わなければならない（SHALL）。その直下に `project.json`、`run.json`、`project.lock`、`logs/` を置く。`projects/` は git 管理外とする（MUST）。

#### Scenario: 新規作成
- **WHEN** 利用者がプロジェクト名 `cube_pick` とタスク雛形を選んで新規作成する
- **THEN** `projects/cube_pick/project.json` が作られ、`state` は `disconnected` になる

#### Scenario: 同名のプロジェクトがある
- **WHEN** 既に存在するプロジェクト名で新規作成しようとする
- **THEN** システムは上書きせず、既存プロジェクトを開くか別名にするよう利用者に求める

### Requirement: project.json の項目
`project.json` は UTF-8 の JSON とし、少なくとも次の項目を持たなければならない（SHALL）。

| キー | 型 | 内容 |
|---|---|---|
| `schema_version` | 整数 | この形式の版。初版は `1` |
| `name` | 文字列 | プロジェクト名（ディレクトリ名と一致） |
| `task_template` | 文字列 | 使うタスク雛形の名前（`templates/<名前>.yaml`） |
| `state` | 文字列 | 現在の状態（後述の 8 種のいずれか） |
| `fallback_state` | 文字列または null | 実行中状態のとき、失敗・中止・異常終了で戻る先の状態。実行中状態でなければ null |
| `robot_id` | 文字列 | フォロワーの `--robot.id` |
| `teleop_id` | 文字列 | リーダーの `--teleop.id` |
| `hardware.follower_port` | 文字列または null | フォロワーの COM ポート |
| `hardware.leader_port` | 文字列または null | リーダーの COM ポート |
| `hardware.cameras` | 配列 | カメラ設定（名前・インデックス・解像度・fps） |
| `dataset_repo_id` | 文字列 | 録画先データセット `<HF_USER>/<task>` |
| `policy_repo_id` | 文字列 | 学習済み方策 `<HF_USER>/act_<task>` |
| `progress.episodes_done` | 整数 | 録画済みエピソード数 |
| `progress.train_step` | 整数 | 学習の到達ステップ |
| `progress.demo_trials` | 整数 | デモの試行回数 |
| `progress.demo_success` | 整数 | デモの成功回数 |
| `last_applied_run_id` | 文字列または null | 状態遷移に反映済みの最後の実行（run.json の `run_id`） |
| `updated_at` | 文字列 | 最終更新時刻（ISO 8601、タイムゾーン付き） |

未知のキーがあっても読み込みを失敗させてはならない（MUST NOT）。班が独自に追加するキーは `ext.<班名>` の下に置かなければならない（SHALL）。

#### Scenario: 班独自の項目を追加する
- **WHEN** D 班が評価メモを保存したい
- **THEN** `ext.eval` の下に保存し、他班のコードはそれを無視しても正常に読み込める

#### Scenario: 古い版のファイルを読む
- **WHEN** `schema_version` が現行より小さい project.json を読み込む
- **THEN** システムは不足項目を既定値で補って読み込み、保存時に現行の版へ更新する

### Requirement: 安全な保存
project.json の保存は、途中で処理が止まっても壊れたファイルが残らない方法で行わなければならない（SHALL）。project.json に書き込むのは画面側（Streamlit を動かしているプロセス）だけとし、lerobot-* コマンドの子プロセスが書き込んではならない（MUST NOT）。

#### Scenario: 保存中に強制終了した
- **WHEN** 保存の途中でツールが強制終了される
- **THEN** 次回起動時の project.json は保存前か保存後のどちらかの完全な内容であり、JSON として読み込める

### Requirement: 同時更新の排他
同じプロジェクトを複数のブラウザタブ（または複数の画面プロセス）で開いていても、project.json の「読む → 判定する → 書く」と、コマンドの起動（run.json の確認と書き込みを含む）は、プロジェクト単位のロックファイル `projects/<名前>/project.lock` で排他しなければならない（SHALL）。

- ロックを持っている間は、project.json をディスクから読み直し、その内容をもとに変更しなければならない（MUST）。タブごとに手元に持っている古い内容で上書きしてはならない（MUST NOT）。
- ロックを待つ時間は最大 5 秒とする。取れなければ何も変更せず、利用者の操作であれば「別の画面で処理中です」と表示する（SHALL）。画面の定期読み込みによる反映であれば、次回の読み込みでやり直す。
- ロックを持ったプロセスが異常終了しても、ロックが残り続けて操作できなくなってはならない（MUST NOT）。

#### Scenario: 2 つのタブが同時に終了を反映しようとする
- **WHEN** タブ A とタブ B が、同じ学習の終了をほぼ同時に検知する
- **THEN** 一方だけが `trained` への遷移と `progress` の更新を行い、もう一方はロック取得後に読み直した `last_applied_run_id` を見て何もしない

#### Scenario: 2 つのタブで同時に開始を押す
- **WHEN** タブ A とタブ B で、ほぼ同時に「学習させる」を押す
- **THEN** lerobot-train は 1 つだけ起動し、もう一方のタブには実行中である旨が表示される

#### Scenario: 古い内容での上書きを防ぐ
- **WHEN** タブ A を開いたまま、タブ B で録画を進めて `progress.episodes_done` が 12 になった後、タブ A で操作する
- **THEN** タブ A の保存後も `progress.episodes_done` は 12 以上のままで、タブ B の更新は失われない

#### Scenario: ロックを持ったまま画面が落ちた
- **WHEN** ロックを持っている最中に画面のプロセスが強制終了される
- **THEN** 次に起動した画面は、手作業でファイルを消さなくてもロックを取得できる

#### Scenario: ロックが取れない
- **WHEN** 別のタブがロックを 5 秒以上持ち続けている間に、利用者がボタンを押す
- **THEN** project.json と run.json は変わらず、「別の画面で処理中です」と表示される

### Requirement: プロジェクトの状態
`state` は次の 8 種のいずれかでなければならない（SHALL）。画面に出す日本語名は括弧内のとおりとする。

- `disconnected`（未接続）
- `connected`（接続済み）
- `calibrated`（キャリブ済み）
- `recording`（録画中）※実行中状態
- `has_data`（データあり）
- `training`（学習中）※実行中状態
- `trained`（学習済み）
- `demo`（デモ中）※実行中状態

#### Scenario: 定義外の値
- **WHEN** project.json の `state` が上記以外の値になっている
- **THEN** システムは読み込みを中断せず、`state` を `disconnected` とみなして利用者に警告を表示する

### Requirement: 許される状態遷移
状態は次の遷移だけを許さなければならない（SHALL）。これ以外の遷移を要求された場合は状態を変えずに拒否する（MUST）。

| 遷移前 | 遷移後 | きっかけ |
|---|---|---|
| `disconnected` | `connected` | ポートとカメラを検出した |
| `connected` | `disconnected` | 接続確認でアームが見つからない |
| `connected` | `calibrated` | キャリブレーション完了 |
| `calibrated` / `has_data` / `trained` | `recording` | 録画開始 |
| `recording` | `has_data` | 規定数のエピソードがそろって録画が正常終了 |
| `has_data` | `training` | 学習開始 |
| `training` | `trained` | 学習が正常終了 |
| `trained` | `demo` | デモ開始 |
| `demo` | `trained` | デモ終了（成否を問わない） |
| 実行中状態 | `fallback_state` の値 | 失敗・中止・異常終了 |

実行中状態へ入るときは、遷移前の状態を `fallback_state` に記録しなければならない（SHALL）。

#### Scenario: 学習に失敗した
- **WHEN** `training` 中に lerobot-train が 0 以外の終了コードで終わる
- **THEN** `state` は `has_data` に戻り、`fallback_state` は null になる

#### Scenario: 学習済みからお手本を追加
- **WHEN** `trained` の状態で録画を開始する
- **THEN** `state` は `recording`、`fallback_state` は `trained` になる

#### Scenario: 飛び越しを拒否する
- **WHEN** `connected` の状態で学習開始を要求する
- **THEN** 状態は変わらず、システムは前提が満たされていない旨を返す

### Requirement: 再開
プロジェクトを開いたとき、`state` が実行中状態であれば、`run.json` を確認して実際にプロセスが動いているかを判定しなければならない（SHALL）。

#### Scenario: 学習が続いている
- **WHEN** `state` が `training` で、`run.json` に記録されたプロセスが動いている
- **THEN** 画面は学習中として進捗表示を再開する

#### Scenario: 実行中のまま止まっていた
- **WHEN** `state` が `recording` だが、`run.json` が終了を示しているか、記録されたプロセスが存在しない
- **THEN** `run.json` の結果に従って遷移を確定させる。結果が成功でなければ `state` を `fallback_state` に戻す
