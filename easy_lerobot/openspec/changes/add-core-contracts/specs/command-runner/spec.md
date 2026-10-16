# Spec Delta

## Purpose

lerobot-* コマンドを画面とは別のプロセスで動かし、その出力・実行状態・終了結果をファイルで受け渡す。Streamlit の再実行やブラウザを閉じる操作で、録画や学習が止まらないようにする。

## ADDED Requirements

### Requirement: 別プロセスでの起動
Runner はコマンドを画面のプロセスとは別のプロセスとして起動し、起動直後に戻らなければならない（SHALL）。コマンドは画面と同じ Python 環境（easy_lerobot を動かしている `.venv`）の lerobot-* を使う（MUST）。引数はシェルを介さず文字列の配列で渡す（MUST）。

#### Scenario: 日本語と空白を含むパス
- **WHEN** プロジェクトが `C:\２年後期-osaka high tech\...` のような全角文字と空白を含むパスにある
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

### Requirement: 標準入力への行送信
Runner は実行中のコマンドの標準入力に、任意の 1 行（空行＝ Enter のみを含む）を送れなければならない（SHALL）。この機能は lerobot-calibrate と lerobot-find-port が内部で `input()` を呼んで止まるのを、画面のボタンから進めるために使う。

#### Scenario: キャリブレーションの「できた」
- **WHEN** lerobot-calibrate が「中間姿勢にして ENTER」で止まっているときに、利用者が「できた」を押す
- **THEN** Runner は改行を 1 つ送り、コマンドは次の手順へ進む

#### Scenario: 既存のキャリブレーションを作り直す
- **WHEN** 既存のキャリブレーションファイルがあり、lerobot-calibrate が「ENTER で既存を使う／c で再キャリブ」と尋ねている
- **THEN** 画面の選択に応じて Runner は空行または `c` を送れる

#### Scenario: 画面を再起動した後
- **WHEN** 画面のプロセスを再起動した後に、入力待ちのコマンドへ行を送ろうとする
- **THEN** Runner は送れないことを返し、画面は中止とやり直しを案内する

### Requirement: 標準入力では届かない Enter 待ち（未確定）
lerobot-calibrate の可動域記録（「各関節を動かし、ENTER で記録を止める」）は、Windows では標準入力ではなくコンソールのキー入力（`msvcrt.kbhit()`）で Enter を検出するため、標準入力への行送信では止められない。画面のボタンからこれを止める方法は本変更では確定しない。確定するまでのあいだ、Runner は lerobot-calibrate を利用者が Enter を押せるコンソールウィンドウ付きで起動し、画面はそのウィンドウで Enter を押すよう案内しなければならない（SHALL）。

#### Scenario: 可動域の記録を終える
- **WHEN** lerobot-calibrate が可動域を記録している
- **THEN** 画面は「表示された黒い画面をクリックして Enter を押してください」と案内し、利用者がそのウィンドウで Enter を押すと記録が終わる

### Requirement: 停止
Runner の停止は、まずコマンドに正常終了を求め、一定時間（既定 10 秒）以内に終わらなければ強制終了しなければならない（SHALL）。停止によって終わった実行の `status` は `cancelled` とする（MUST）。

#### Scenario: 録画をやめる
- **WHEN** 録画中に「録画をやめる」を押す
- **THEN** lerobot-record に終了が求められ、10 秒以内に終わらなければ強制終了し、run.json は `cancelled` になる

### Requirement: 録画中のキー操作（未確定）
lerobot-record は Windows では pynput によるキー入力（→ 次へ、← 撮り直し、Esc 終了）で操作されるため、標準入力への送信では操作できない。画面のボタンからこれらを行う方法は本変更では確定せず、確定するまで `record` は 1 エピソードの終了・撮り直しを画面から操作できなくてよい（SHALL）。ただし「録画をやめる」は停止の要件で必ず実現する（MUST）。

#### Scenario: 方式が未確定のあいだ
- **WHEN** 利用者が録画画面を開く
- **THEN** 「録画をやめる」は動作し、「この回を終える」「撮り直す」は未対応である旨が表示される
