# 作業記録

- 2026-11-12 10:23｜実装｜―｜FR｜―
  - 変更内容：easy_lerobot に Streamlit の Web アプリを新規作成。起動画面のセレクトボックスで mirrorMODE / LearnMODE を選び、「次へ」で各モードのページへ遷移する。各モードのページは説明と「モード選択に戻る」のみ（機能は未実装）
  - 変更ファイル：`easy_lerobot/app.py`、`easy_lerobot/pages/home.py`、`easy_lerobot/pages/mirror_mode.py`、`easy_lerobot/pages/learn_mode.py`
  - コミット：未コミット
  - テスト：`uv run --locked streamlit run app.py` で起動し、ブラウザで両モードの選択 → 遷移（`/mirror`、`/learn`）→ 戻るを確認。サーバー側のエラーなし
  - 未検証：モード未選択時に「次へ」が押せないこと（目視のみ）

- 2026-11-12 10:24｜実装｜―｜FR｜―
  - 変更内容：モード選択をセレクトボックス＋「次へ」から、2 つの箱型ボタン（押すと即遷移）に変更。ボタンの key に付く `st-key-mode_*` クラスを CSS で指定し、高さ 180px・モード名を大きく表示
  - 変更ファイル：`easy_lerobot/pages/home.py`
  - コミット：未コミット
  - テスト：ブラウザで両ボタン → 各ページ → 戻るを確認
  - 未検証：初回の確認で LearnMODE を座標指定で押した際に `/mirror` へ遷移した。要素指定での再試行では再現せず。同じブラウザ画面をすばる様も操作中で、押した時点の画面が想定と違っていた可能性がある（確度：推測）

- 2026-11-12 10:30｜実装｜―｜FR｜―
  - 変更内容：mirrorMODE にフォロワーのトルク ON/OFF トグルを追加。トルク操作は `core/follower_torque.py`（status / on / off、結果を JSON で出力）を別プロセスで呼ぶ。ON の前に各モーターの Goal_Position を Present_Position に合わせ、古い目標位置へ跳ねないようにする。トグルは操作後にモーターから読み直した値を表示する。未接続時はトグルを無効にしてエラーを表示
  - 変更ファイル：`easy_lerobot/core/__init__.py`、`easy_lerobot/core/follower_torque.py`、`easy_lerobot/pages/mirror_mode.py`
  - コミット：未コミット
  - テスト：アーム未接続の状態で、`follower_torque status` が JSON でエラーを返すこと、画面でトグルが無効・エラー表示・「状態を読み直す」「モード選択に戻る」が例外なく動くことを確認
  - 未検証：アーム接続時のトルク ON/OFF（実機なし）。ポートは COM20 に固定（project.json 未実装のため）

- 2026-11-12 10:47｜実装｜―｜FR｜―
  - 変更内容：mirrorMODE にミラーの開始・停止ボタン（大きめの箱型 1 つ、状態で「▶ 開始」「■ 停止」が切り替わる）を追加。`dev_scripts/mirror_leader.py` を改変せず別プロセスで起動し、停止は CTRL_BREAK → 5 秒で強制終了。`core/break_as_interrupt.py` で SIGBREAK を KeyboardInterrupt に変え、mirror_leader の後片付け（ポートを閉じる）を走らせる。プロセスは `st.cache_resource` で保持し、ブラウザの再読み込みでは失われない。ミラー中はトルクのトグルと「状態を読み直す」を無効化。「← 戻る」でミラーを止める。止まった理由はログ末尾から日本語の案内に変換（ポート不可・応答なし・姿勢差）
  - 変更ファイル：`easy_lerobot/core/break_as_interrupt.py`、`easy_lerobot/pages/mirror_mode.py`
  - コミット：未コミット
  - テスト：ダミーのループで CTRL_BREAK → KeyboardInterrupt → finally まで走り終了コード 0 を確認。画面でアーム未接続のまま開始し、「リーダーアーム（COM19）に接続できません」が出ることを確認
  - 未検証：アーム接続時のミラー中の表示と停止（実機なし）。Streamlit サーバーを再起動するとミラーのプロセスを見失う（プロセスは動き続ける）

- 2026-11-12 10:49｜実装｜―｜FR｜―
  - 変更内容：mirrorMODE をスクロールなしで 1 画面に収まるよう再配置。トルクとミラーを枠付きの左右 2 列に並べ、説明文を短縮し、ページ上部の余白を 3.5rem に詰めた
  - 変更ファイル：`easy_lerobot/pages/mirror_mode.py`
  - コミット：未コミット
  - テスト：ブラウザ（幅 800・高さ 386）でスクロールなしに全要素が表示されることを確認
  - 未検証：幅の狭い画面（スマートフォン等）では 2 列が縦に積まれ、スクロールが必要になる見込み（確度：推測）

- 2026-11-12 10:53｜実装｜―｜FR｜―
  - 変更内容：mirrorMODE の画面を操作盤型に作り直し。上部にリーダー・フォロワーの接続バッジ（`serial.tools.list_ports` でポートの有無を確認、ポートは開かない）、中央に高さ 150px の開始・停止ボタン（停止中は緑「▶ ミラー開始」、ミラー中は赤「■ 停止」）、下部にトルクの 1 行（名前・状態・読み直しアイコン・トグル）。大きな警告ボックスを廃止し、短い色付き文字に変更。ポートが見えない間は開始ボタンを無効化
  - 変更ファイル：`easy_lerobot/pages/mirror_mode.py`
  - コミット：未コミット
  - テスト：アーム未接続で、バッジ・無効の開始ボタン・トルク行が高さ 652px にスクロールなしで収まることを確認。緑の開始ボタンの見た目はブラウザ上で一時的に無効化を外して確認（ソースは変更せず、再読み込みで戻した）。「← 戻る」の遷移を確認
  - 未検証：アーム接続時のバッジ・ミラー中（赤い停止ボタン）の表示・トルク切り替え（実機なし）

- 2026-11-12 10:55｜実装｜―｜FR｜―
  - 変更内容：LearnMODE に全体の流れの画面を作成。「1. 録画 → 2. 学習 → 3. 実行」の 3 枚のカードを矢印でつなぎ、各カードに状態の札（準備OK / 完了 / ロック中）・説明・進み具合（お手本の回数、学習済みか、成功率）・「始める」ボタンを置いた。前の段階が終わるまで次のカードはロック。いま進めるべき段階のボタンは緑。各段階の画面は未作成のため、押すとトーストで「これから作ります」と案内
  - 変更ファイル：`easy_lerobot/pages/learn_mode.py`
  - コミット：未コミット
  - テスト：ブラウザで表示・トースト・「← 戻る」を確認。ロック判定 `step_states` を 4 通り（0 回／29 回／30 回で未学習／学習済み）で検算し全て一致
  - 未検証：進み具合は `learn_progress()` の仮の値（0 / 30 回・未学習）で、実データとは未連携（project.json 未実装のため）

- 2026-11-12 11:07｜問題発見｜P-001｜FR｜―
  - 現象：lerobot-record がアームへの接続に失敗すると、空のデータセットのフォルダーが残り、次回の開始で `FileExistsError`（WinError 183）になる
  - 再現手順：アーム未接続で、同じデータ名の lerobot-record を 2 回起動する
  - 影響範囲：録画画面で接続に一度失敗すると、以後そのデータ名で録画できなくなる
  - 発見の経緯：録画画面の実装中に、HF_LEROBOT_HOME をスクラッチパッドに向けて 2 回起動して確認

- 2026-11-12 11:07｜原因解明｜P-001｜FR｜―
  - 原因：`lerobot_record.record()` はデータセットを作成（`mkdir(exist_ok=False)`）してからロボットに接続する。また `LeRobotDataset.resume()` は `--dataset.root` を明示しないと `ValueError` を出す
  - 根拠：`src/lerobot/scripts/lerobot_record.py` 441–490 行、`datasets/dataset_metadata.py` 842 行、`datasets/lerobot_dataset.py` 943 行と、実行時のトレースバック
  - 確度：確定

- 2026-11-12 11:07｜実装｜P-001｜FR｜―
  - 変更内容：LearnMODE の録画画面を作成。lerobot-record を `python -m` で別プロセス起動し、`--dataset.root` を常に明示、`meta/info.json` があれば `--resume=true`（P-001 の対策）。`--dataset.no_stamp=true` でデータ名を固定。キャリブレーションは `dev_calibration` の follower_F / leader_L を指定し、照合の入力待ちに備えて標準入力に改行を 1 つ渡す。「この回を終える」「撮り直す」は pynput で → / ← キーを OS に送る（lerobot-record の pynput グローバル監視が受け取ることを別プロセス間で確認）。「録画を終える」は CTRL_BREAK → KeyboardInterrupt で `dataset.finalize()` を走らせる。ログから録画中・片付け中・保存中と回数を表示。プロセス管理を `core/procs.py` に共通化し、ミラーと録画を排他。ポート定義を `core/hardware.py` に集約。LearnMODE の録画カードを実データ（info.json の total_episodes）に連携し、録画画面へ遷移
  - 変更ファイル：`easy_lerobot/app.py`、`easy_lerobot/core/break_as_interrupt.py`、`easy_lerobot/core/procs.py`、`easy_lerobot/core/hardware.py`、`easy_lerobot/core/recording.py`、`easy_lerobot/core/key_input.py`、`easy_lerobot/pages/record.py`、`easy_lerobot/pages/learn_mode.py`、`easy_lerobot/pages/mirror_mode.py`
  - コミット：未コミット
  - テスト：アーム未接続で lerobot-record を新規・再開とも起動し、引数がすべて解釈され接続段階まで進むことを確認。ログの状態判定 6 通りとエラー文の言い換えを見本で検算し全て一致。画面で録画ページの表示・無効ボタン・遷移、ミラー画面の再表示を確認
  - 未検証：アーム接続時の録画全体（録画中の表示、→ / ← キー送信での操作、CTRL_BREAK 停止後のデータセット確定、照合入力待ちが起きた場合の挙動）。キー送信は画面を表示している PC 上でのみ有効（ラズパイ・ヘッドレスでは別方式が必要）

- 2026-11-12 11:22｜問題発見｜P-002｜FR｜―
  - 現象：画面のファイルを編集した後、テストモードが OFF に戻り、動いていたミラー（偽）が画面から見えなくなった。プロセスは動き続けていた
  - 再現手順：ミラー開始 → views/ のファイルを編集 → 画面を再読み込み
  - 影響範囲：本物のミラー・録画でも、ファイル編集やサーバー再起動の後に画面からアームのプロセスを止められなくなる
  - 発見の経緯：テストモードの通し確認中

- 2026-11-12 11:22｜原因解明｜P-002｜FR｜―
  - 原因：実行中プロセスの一覧とテストモードをモジュール変数（メモリ）に持っていたため、Streamlit のモジュール再読み込みやサーバー再起動で失われた。加えて `.venv\Scripts\python.exe` は中継プロセスで実体の Python を子に持つため、`Popen.kill()` では子が残る
  - 根拠：取り残されたプロセスを `Win32_Process` で確認（中継と実体の 2 件）。`taskkill /T` で両方が止まることを確認
  - 確度：確定

- 2026-11-12 11:22｜実装｜P-002｜FR｜―
  - 変更内容：`core/procs.py` を、実行中プロセスの PID・作成時刻・停止要求をファイル（一時フォルダー easy_lerobot/procs）に保存する方式に変更。生存確認は PID と作成時刻の照合（PID 再利用対策）。終了コードは `core/break_as_interrupt.py` が `EASY_LEROBOT_EXIT_FILE` に書き残す。強制終了は `taskkill /T /F` でプロセスツリーごと。利用者の停止による非 0 終了（KeyboardInterrupt の 130）は失敗扱いにしない。テストモードはフラグファイルで保持
  - 変更ファイル：`easy_lerobot/core/procs.py`、`easy_lerobot/core/break_as_interrupt.py`、`easy_lerobot/core/ui.py`
  - コミット：未コミット
  - テスト：単体試験 9 項目（起動・モジュール再読み込み後の追跡・利用者停止は成功扱い・sys.exit(msg) は失敗扱い・CTRL_BREAK を無視するプロセスのツリーごと強制終了・取り残し 0 件）すべて成功

- 2026-11-12 11:22｜問題発見｜P-003｜FR｜―
  - 現象：`/record` などを URL で直接開くと、左に Streamlit 標準のページ一覧（app / home / learn mode …）が表示される
  - 再現手順：サーバー再起動直後に http://localhost:8501/record を開く
  - 影響範囲：全ページ。利用者が案内を無視して任意のページへ飛べてしまう
  - 発見の経緯：テストモードの通し確認中

- 2026-11-12 11:22｜原因解明｜P-003｜FR｜―
  - 原因：ページのフォルダー名が `pages/` だったため、Streamlit が従来方式のマルチページとして自動登録した（確度：推測。`views/` に改名すると再現しなくなったことは確認済み）
  - 根拠：改名前後で同じ手順を比較
  - 確度：推測

- 2026-11-12 11:22｜実装｜P-003｜FR｜―
  - 変更内容：`easy_lerobot/pages/` を `easy_lerobot/views/` に改名し、`st.Page` と `st.switch_page` の参照を更新
  - 変更ファイル：`easy_lerobot/app.py`、`easy_lerobot/views/*.py`
  - コミット：未コミット
  - テスト：サーバー再起動後に `/record` を直接開き、ページ一覧が出ないことを確認

- 2026-11-12 11:22｜実装｜―｜FR｜―
  - 変更内容：モード選択画面の右上に歯車ボタンを追加し、テストモードの切り替えを置いた。テストモードではポートを接続扱い、トルクは画面上の値のみ、ミラーと録画は偽プロセス（`core/fake_mirror.py`、`core/fake_record.py`）に置き換える。偽の録画は lerobot-record と同じログ文言・同じキー（→ / ←）・同じ停止方法（Ctrl+Break）で動き、データは一時フォルダー easy_lerobot/test_datasets に保存。各ページのタイトル横に橙色の「テストモード」札。あわせて次を修正：①開始ボタンの色が Streamlit のホバー色に上書きされる（`!important` で固定）②過去の実行結果が開いた直後に表示される（この画面から開始した実行に限定）③撮り直し時に「次は 2 回目」と誤表示（lerobot の "rerecord the last episode" 出力で判定し「↺ 撮り直し 次も N 回目」と表示）④録画設定がページ移動で消える（値を通常のセッションキーへ写す）
  - 変更ファイル：`easy_lerobot/core/ui.py`、`easy_lerobot/core/fake_mirror.py`、`easy_lerobot/core/fake_record.py`、`easy_lerobot/core/recording.py`、`easy_lerobot/views/home.py`、`easy_lerobot/views/mirror_mode.py`、`easy_lerobot/views/record.py`、`easy_lerobot/views/learn_mode.py`
  - コミット：未コミット
  - テスト：ブラウザで通し確認。テストモード ON → ミラー（トルク切替・開始・停止）→ 録画（開始・この回を終える・撮り直し・時間切れでの完了・途中停止・保存件数）→ LearnMODE の回数表示（3 / 10）。撮り直しの判定を見本ログ 4 通りで検算。取り残しプロセス 0 件
  - 未検証：実機。テストモード中に本物のアームがつながっていても偽物として動く（意図どおり）

- 2026-11-19 01:01｜問題発見｜P-004｜FR｜―
  - 現象：L→F ミラーで F5（wrist_roll）だけが追従せず、目標と逆向きに回って機械の端（3984〜3991 付近）に押し付け、過負荷になる
  - 再現手順：`dev_scripts/mirror_leader.py` で L5→F5 を位置制御（Goal_Position）で追従させる
  - 影響範囲：F5 のミラー、公式 lerobot-teleoperate（同じ位置制御）
  - 発見の経緯：ミラー動作確認中

- 2026-11-19 01:01｜原因解明｜P-004｜FR｜―
  - 原因：L5・F5 の可動範囲（約334°）がダイヤルの 4095/0 の継ぎ目をまたいでいる（F5 2184→0→1906、L5 1745→0→1453）。位置制御モードのサーボは継ぎ目をまたげず、目標が反対側に来ると遠回りして動かない区間（約26°）の端に当たる
  - 根拠：すばる様が高速ポーリング（`dev_scripts/poll_joint.py`）で両端と継ぎ目越えを確認。速度制御で範囲に沿って回すと継ぎ目を両方向に越えて追従した
  - 確度：確定（遠回りの仕組みはサーボ内部仕様未確認のため推測）
  - 付随：Goal_Position を書くとトルクが自動で入る（3 回再現、確度：推測）。動作モード変更で再起動し、読み取りは生の値（原点オフセット無視）になる
  - 付随：easy_lerobot の Streamlit アプリ稼働中に wrist_roll の原点オフセットが元の値へ戻っていた（アプリが校正値を書き戻した可能性、確度：推測）

- 2026-11-19 01:01｜実装｜P-004｜FR｜―
  - 変更内容：mirror_leader.py に継ぎ目をまたぐ範囲の比例対応（`--range-files`）と、速度モードでの範囲に沿った位置ループ（`--velocity-ids`、`--vel-max`、`--vel-kp`）を追加。終了時は位置モードへ戻しトルクを切る。hold_follower_pose.py は目標を書いた後にトルクを切る順に修正
  - 変更ファイル：`dev_scripts/mirror_leader.py`、`dev_scripts/hold_follower_pose.py`、`dev_scripts/poll_joint.py`、`dev_calibration/joint5_maxmin.json`、`.vscode/launch.json`
  - コミット：d92043b8、4fb6a9e7、15c26d17、477a66fd、82d02892 ほか
  - テスト：実機で F5 が継ぎ目を両方向に越えて L5 に追従（約 1900 tick を約 1 秒、停止時の差 ±3 tick）
  - 未検証：L5 を素早く往復させたときの振動、両端の対応の向き（すばる様の記載順で設定）

- 2026-11-19 04:37｜問題発見｜P-005｜FR｜―
  - 現象：easy_lerobot の録画画面から lerobot-record を起動すると、モーターの校正値が `dev_calibration/*.json` と違う場合に、json の Homing_Offset と位置制限が確認なしでモーターへ書き込まれる
  - 根拠：`easy_lerobot/views/record.py` は stdin に空行を渡して起動する。`src/lerobot/robots/so_follower/so_follower.py` の `calibrate()` は空行を「校正ファイルを使う」と解釈し、`bus.write_calibration()` を呼ぶ（コードで確認）
  - 影響範囲：L/F 全関節の Homing_Offset と Min/Max_Position_Limit。以前 wrist_roll の Homing が戻っていた件の原因の候補（確度：推測。実際に録画画面が使われたかは未確認）
  - 発見の経緯：PR #6（ラズパイ4対応）の差分点検中。この挙動は PR #6 以前から main にある

- 2026-11-19 04:54｜問題発見｜P-006｜FR｜―
  - 現象：mirror_leader.py の停止時、速度モードの関節（F5）が速度モードのまま残り、最後の速度で回り続けるおそれがある
  - 経路：(1) 後片付け中に2度目の停止信号が来ると後片付けが中断する（easy_lerobot は 5 秒後に SIGKILL）、(2) 速度モードへの切り替え中に止めると後片付けの対象に入っていない、(3) SIGTERM で直接止めると後片付けが走らない、(4) ホストの電源断・強制終了
  - 影響範囲：F5（wrist_roll）。回り続けると機械的な止まりに当たる
  - 発見の経緯：PR #6（ラズパイ4対応）の差分点検中

- 2026-11-19 04:54｜実装｜P-006｜FR｜―
  - 変更内容：後片付けの開始時に停止信号を無視し、全速度モード関節の速度を先に 0 にしてから位置モードへ戻す（関節ごとに例外を捕捉）。速度モード切り替えの前に後片付け対象へ登録。SIGTERM を Ctrl+C と同様に扱う。起動時に位置モード以外の関節を見つけたら停止して位置モードへ戻す
  - 変更ファイル：`dev_scripts/mirror_leader.py`
  - テスト：偽の SDK で4通りを確認。起動時の残留速度モードの復旧、後片付け中の2度目の SIGINT、速度モード切り替え中の停止（修正前は mode=1 が残り、修正後は mode=0）、いずれも最終状態は mode=0・速度 0・トルク 0
  - 未検証：実機での動作。ホストの電源断（経路4）はソフトでは防げない（サーボ側に通信途絶時に止める機能があるかは未確認。確度：推測）
