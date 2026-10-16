# Spec Delta

## Purpose

5 つのステップ（つなぐ・合わせる・教える・学習させる・動かす）が同じ型を持つことで、各班が他班のステップの完成を待たずに自分のステップを実装・差し替えできるようにする。

## ADDED Requirements

### Requirement: ステップの一覧と識別子
システムは次の 5 ステップをこの順で持たなければならない（SHALL）。識別子はログ名・画面のページ名と共通とする。

| 識別子 | 画面名 | 担当 | 主に呼ぶコマンド |
|---|---|---|---|
| `connect` | 1. つなぐ | A 班 | lerobot-find-port、lerobot-find-cameras |
| `calibrate` | 2. 合わせる | A 班 | lerobot-calibrate |
| `record` | 3. 教える | B 班 | lerobot-record |
| `train` | 4. 学習させる | C 班 | lerobot-train |
| `demo` | 5. 動かす | D 班 | lerobot-record --policy.path、lerobot-rollout |

#### Scenario: ログの名前
- **WHEN** `record` ステップが実行される
- **THEN** そのログは `logs/record.log` に書かれる

### Requirement: 共通の操作
各ステップは `check()`、`run()`、`cancel()` の 3 操作、終了時に一度だけ呼ばれる `on_finished(run)`、読み取り専用の `status` を持たなければならない（SHALL）。

- `on_finished(run)` は run.json の内容を受け取り、遷移先の状態と `progress` の更新内容を返す。project.json の保存は呼び出し側（core）が行う。

- `check()` は前提がそろっているかを判定し、可否と、不可の場合の理由（利用者に見せられる日本語の文の一覧）を返す。副作用を持ってはならない（MUST NOT）。
- `run()` は Runner を通じてコマンドを起動し、完了を待たずに戻る（MUST）。起動前に `check()` と同じ判定を行い、不可なら起動しない。
- `cancel()` は実行中のコマンドを止める。実行中でなければ何もしない。
- `status` は project.json と run.json から計算し、呼ぶだけで状態を変えてはならない（MUST NOT）。

#### Scenario: 前提がそろっていない
- **WHEN** `state` が `connected` のときに `train` の `check()` を呼ぶ
- **THEN** 不可と、「お手本の録画がまだです」のような理由が返り、ファイルは何も変わらない

#### Scenario: run は待たずに戻る
- **WHEN** `train` の `run()` を呼ぶ
- **THEN** lerobot-train の起動直後に制御が画面に戻り、学習はその後も別プロセスで続く

#### Scenario: 実行中でない cancel
- **WHEN** 何も動いていないときに `cancel()` を呼ぶ
- **THEN** エラーにならず、状態も変わらない

### Requirement: ステップの status 値
`status` は次のいずれかでなければならない（SHALL）。

- `locked`: 前のステップが終わっておらず、まだ実行できない
- `ready`: 実行できる
- `running`: このステップのコマンドが動いている
- `done`: 一度以上成功し、その成果が現在の状態に反映されている
- `failed`: 直前の実行が失敗・中止した

`done` のステップも、状態遷移の表が許す範囲で再実行できなければならない（SHALL）。

#### Scenario: サイドバーの表示
- **WHEN** `state` が `has_data` である
- **THEN** `connect`・`calibrate`・`record` は `done`、`train` は `ready`、`demo` は `locked` になる

### Requirement: ステップと状態の対応
各ステップは、成功・失敗時にプロジェクトの状態を project-state の遷移表どおりに動かさなければならない（SHALL）。ステップが遷移表にない遷移を起こしてはならない（MUST NOT）。

| ステップ | 実行できる状態 | 開始時 | 成功時 | 失敗・中止時 |
|---|---|---|---|---|
| `connect` | すべて | 変えない | `disconnected` なら `connected` へ | 変えない |
| `calibrate` | `connected` 以降 | 変えない | `connected` なら `calibrated` へ | 変えない |
| `record` | `calibrated` / `has_data` / `trained` | `recording` へ | 規定数そろえば `has_data` へ | `fallback_state` へ |
| `train` | `has_data` | `training` へ | `trained` へ | `fallback_state` へ |
| `demo` | `trained` | `demo` へ | `trained` へ | `fallback_state` へ |

#### Scenario: 録画が規定数に届かず終わった
- **WHEN** 規定 30 回のところ 12 回で録画を中止した
- **THEN** `state` は `fallback_state` に戻り、`progress.episodes_done` は 12 として保存される

### Requirement: 完了の反映は一度だけ
コマンドの終了は、画面の再実行やブラウザの再読み込みが何度起きても、状態遷移として一度だけ反映されなければならない（SHALL）。

#### Scenario: 再読み込みしても二重にならない
- **WHEN** 学習完了後に画面を 3 回再読み込みする
- **THEN** `trained` への遷移と `progress` の更新は 1 回分だけ行われる

#### Scenario: 複数のタブで開いていても二重にならない
- **WHEN** 同じプロジェクトを 2 つのタブで開いたまま、デモが 1 回終わる
- **THEN** `progress.demo_trials` は 1 だけ増える（project-state の「同時更新の排他」による）

### Requirement: LeRobot 本体を改変しない
ステップの実装は `src/lerobot` 以下を変更してはならず（MUST NOT）、LeRobot の機能は lerobot-* コマンドを Runner 経由で呼ぶことでのみ使わなければならない（SHALL）。

#### Scenario: upstream の更新を取り込む
- **WHEN** upstream の LeRobot を更新する
- **THEN** easy_lerobot 側のファイルとの競合は起きない
