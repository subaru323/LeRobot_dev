# Spec Delta

## Purpose

lerobot-* コマンドを画面とは別のプロセスで動かし、その出力・実行状態・終了結果をファイルで受け渡す。Streamlit の再実行やブラウザを閉じる操作で、録画や学習が止まらないようにする。

## ADDED Requirements

### Requirement: 別プロセスでの起動
Runner はコマンドを画面のプロセスとは別のプロセスとして起動し、起動直後に戻らなければならない（SHALL）。コマンドは画面と同じ Python 環境（easy_lerobot を動かしている `.venv`）の lerobot-* を使う（MUST）。引数はシェルを介さず文字列の配列で渡す（MUST）。

#### Scenario: 日本語と空白を含むパス
- **WHEN** プロジェクトが `C:\２年後期-osaka high tech\...` のような全角文字と空白を含むパスにある
- **THEN** コマンドは正しい引数で起動し、ログと run.json も同じディレクトリに書かれる

#### Scenario: ラズパイ上のパス
- **WHEN** 本番環境でプロジェクトが `/home/<利用者>/かんたん lerobot/...` のような日本語と空白を含むパスにある
- **THEN** コマンドは正しい引数で起動し、ログと run.json も同じディレクトリに書かれる

#### Scenario: ブラウザを閉じても続く
- **WHEN** 学習中にブラウザのタブを閉じる
- **THEN** lerobot-train は動き続け、ログも書かれ続ける

### Requirement: 同時実行は 1 つまで
1 つのプロジェクトで同時に動かせるコマンドは 1 つまでとしなければならない（SHALL）。run.json に記録されたプロセスが動いている間の起動要求は拒否する（MUST）。

#### Scenario: 録画中に学習を押す
- **WHEN** `record` の実行中に `train` の起動を要求する
- **THEN** Runner は起動せず、実行中のステップ名を添えて拒否する

#### Scenario: 確認と起動の間に割り込まれない
- **WHEN** 2 つのタブから同時に起動を要求する
- **THEN** run.json の確認から起動・run.json の書き込みまでが project-state の「同時更新の排他」のロックの中で行われ、起動するのは 1 つだけになる

### Requirement: ログファイル
Runner はコマンドの標準出力と標準エラー出力をまとめて `logs/<ステップ識別子>.log` に書かなければならない（SHALL）。ログは UTF-8 とし、コマンドの出力は 2 秒以内にファイルへ反映される（MUST）。各実行の開始時に、前回のログは `logs/<ステップ識別子>.prev.log` に移す（SHALL）。

#### Scenario: 進捗をログから読む
- **WHEN** lerobot-train が `step:12400 loss:0.071` を出力する
- **THEN** 2 秒以内に `logs/train.log` にその行が現れ、画面はそこから進捗を読み取れる

#### Scenario: 日本語や記号を含む出力
- **WHEN** コマンドが cp932 で表せない文字（例：`·`）を出力する
- **THEN** コマンドは文字コードのエラーで落ちず、ログには UTF-8 で記録される

### Requirement: run.json
Runner は実行ごとに `run.json` を書かなければならない（SHALL）。項目は次のとおりとする。

| キー | 内容 |
|---|---|
| `run_id` | 実行ごとに一意な識別子 |
| `step` | ステップ識別子 |
| `args` | 実行した引数の配列 |
| `pid` | プロセス ID |
| `started_at` / `ended_at` | 開始・終了時刻（ISO 8601、終了前は null） |
| `status` | `running` / `succeeded` / `failed` / `cancelled` |
| `exit_code` | 終了コード（終了前は null） |

終了コードと `ended_at` は、画面のプロセスが途中で終了・再起動していても記録されなければならない（SHALL）。

#### Scenario: 画面を落としている間に学習が終わる
- **WHEN** Streamlit を停止している間に lerobot-train が終了コード 0 で終わる
- **THEN** run.json は `status: succeeded`、`exit_code: 0` になっており、次に画面を開いたとき学習完了を反映できる

### Requirement: 端末入力の送信
本番環境（Linux）では、Runner はコマンドを仮想端末（pty）につないで起動し、実行中のコマンドへ次の入力を送れなければならない（SHALL）。
- 任意の 1 行（空行＝ Enter のみを含む）
- 1 つのキー（→、←、Esc）

この機能は、lerobot-find-port と lerobot-calibrate の Enter 待ち（`input()` と可動域記録の停止）、および lerobot-record の録画操作を、画面のボタンから進めるために使う。開発環境（Windows）では仮想端末を使えないため、標準入力のパイプで 1 行の送信だけを行えればよい（SHALL）。

#### Scenario: キャリブレーションの「できた」
- **WHEN** lerobot-calibrate が「中間姿勢にして ENTER」で止まっているときに、利用者が「できた」を押す
- **THEN** Runner は改行を 1 つ送り、コマンドは次の手順へ進む

#### Scenario: 既存のキャリブレーションを作り直す
- **WHEN** 既存のキャリブレーションファイルがあり、lerobot-calibrate が「ENTER で既存を使う／c で再キャリブ」と尋ねている
- **THEN** 画面の選択に応じて Runner は空行または `c` を送れる

#### Scenario: 可動域の記録を終える
- **WHEN** 本番環境で lerobot-calibrate が可動域を記録しているときに、利用者が「記録を終える」を押す
- **THEN** Runner は改行を 1 つ送り、記録が終わってキャリブレーションファイルが保存される

#### Scenario: 画面を再起動した後
- **WHEN** 画面のプロセスを再起動した後に、入力待ちのコマンドへ入力を送ろうとする
- **THEN** Runner は送れないことを返し、画面は中止とやり直しを案内する

### Requirement: 停止
Runner の停止は、まずコマンドに正常終了を求め、一定時間（既定 10 秒）以内に終わらなければ強制終了しなければならない（SHALL）。停止によって終わった実行の `status` は `cancelled` とする（MUST）。本番環境では、lerobot-record への正常終了の要求は Esc の送信で行い、録画済みのエピソードがデータセットとして保存されるようにしなければならない（SHALL）。

#### Scenario: 録画をやめる
- **WHEN** 録画中に「録画をやめる」を押す
- **THEN** lerobot-record に終了が求められ、10 秒以内に終わらなければ強制終了し、run.json は `cancelled` になる

#### Scenario: 録画をやめても撮った分は残る
- **WHEN** 本番環境で 12 エピソード録った時点で「録画をやめる」を押し、lerobot-record が 10 秒以内に終わる
- **THEN** 12 エピソード分のデータセットが読める状態で残る

### Requirement: 録画中のキー操作
本番環境では、録画画面の「この回を終える」「撮り直す」「録画をやめる」は、lerobot-record にそれぞれ →、←、Esc のキーを送ることで実現しなければならない（SHALL）。lerobot-record がこれらのキーを端末から読むよう、Runner は lerobot-record を画面表示（`DISPLAY` / `WAYLAND_DISPLAY`）のない環境として起動しなければならない（MUST）。開発環境（Windows）では、これらの操作に対応しなくてよい。

#### Scenario: この回を終える
- **WHEN** 本番環境で録画中に「この回を終える」を押す
- **THEN** Runner は → を送り、lerobot-record は現在のエピソードを保存して次へ進む

#### Scenario: 撮り直す
- **WHEN** 本番環境で録画中に「撮り直す」を押す
- **THEN** Runner は ← を送り、lerobot-record は直前のエピソードを破棄して撮り直す

#### Scenario: 画面付きのラズパイで録画する
- **WHEN** デスクトップ画面のあるラズパイで録画を始める
- **THEN** lerobot-record はキーを端末から読み、画面のボタンからの操作が効く

#### Scenario: 開発環境での録画画面
- **WHEN** Windows の開発環境で録画画面を開く
- **THEN** 「録画をやめる」は動作し、「この回を終える」「撮り直す」は開発環境では使えない旨が表示される
