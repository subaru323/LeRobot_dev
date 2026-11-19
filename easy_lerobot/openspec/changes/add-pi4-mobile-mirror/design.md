# Design

## Context

- 動機は proposal.md の Why、要件は specs/pi4-mobile-mirror を参照。最終形（ラズパイ 5・推論・学習）は add-raspberry-pi-deployment のとおりで、本提案はその手前の段階にあたる。
- 実機で確認した事実（2026-11-19）:
  - ラズパイは Raspberry Pi 4 Model B Rev 1.2、メモリ 3.7GiB、Debian 13（trixie）の 64 ビット版、Python 3.13.5。
  - ラズパイは検証用の Wi-Fi 上にいて、IP アドレスで SSH できる。開発に使っている Windows PC からは `<ホスト名>.local` を引けなかった。
  - ラズパイから PyPI への速度は約 29KB/s だった。torch のような大きな依存を入れるのは現実的でない。
- ソースで確認した事実:
  - `dev_scripts/mirror_leader.py` と `core/follower_torque.py` の依存は `scservo_sdk`（feetech-servo-sdk）と pyserial だけで、lerobot 本体を import しない。
  - `core/procs.py` は Win32 API（`OpenProcess`・`GetProcessTimes`）と `CTRL_BREAK_EVENT`・`taskkill` を使い、Linux では import の時点で失敗する。
  - `core/key_input.py` と `core/fake_record.py` は pynput を使う。画面のない Linux では pynput の import が失敗する（X サーバーがない）。

## Goals / Non-Goals

**Goals:**
- ラズパイ 4 の電源を入れ、スマホのブラウザから mirrorMODE を開いて、ミラーの開始・停止とフォロワーのトルクの切り替えができること。
- 同じコードが Windows の開発機でもこれまでどおり動くこと。

**Non-Goals:**
- ラズパイ 4 での録画・学習・推論（録画は後回し）。ラズパイは画面とアームの中継役とし、将来 GPU のあるパソコンが来たら録画のデータはそこへ送る。ラズパイの SD カードには貯めない（容量が足りなくなるため）。
- カメラの映像を画面に出すこと。カメラの接続のしかたと名前の付け方だけを決める。

## Decisions

### D1. ポートの指定
`core/hardware.py` の `arm_ports()` が、次の順でリーダーとフォロワーのポートを決める。
1. 環境変数 `EASY_LEROBOT_LEADER_PORT`・`EASY_LEROBOT_FOLLOWER_PORT` があればそれを使う。
2. Linux（ラズパイ）では自動で判定する。`/dev/serial/by-id/` の基板ごとに、ID 1〜6 のモーターの Homing_Offset（アドレス 31）を読み、`dev_calibration/leader_L.json`・`follower_F.json` の `homing_offset` と ±30 以内で 6 台とも一致した方の役割を割り当てる。読むだけで、トルクも目標位置も書かない。
3. 判定できないとき、Linux では「未判定」（どのポートにも一致しない名前）にして、ミラーとトルク操作を始められないようにする。Windows は `COM19`・`COM20` のまま。
- 理由: `/dev/ttyACM*` の番号は差す順で入れ替わる。by-id の名前は差し込み口を変えても同じだが、基板を別のアームにつなぎ替えると追えない。モーターに書かれた原点の値はアームごとに違う（実測でリーダーとフォロワーの差は平均 1549）ので、どの基板・どの差し込み口でもアームそのもので見分けられる。
- 判定できないときに `/dev/ttyACM0` に戻すと、差した順によってはフォロワーをリーダーとして扱い、リーダーにトルクを入れて動かしてしまう。そのため Linux では既定値に戻さない。
- 判定はポートを開くので、ミラーや録画が動いている間は行わない。結果は差さっている基板の組み合わせが変わるまで使い回し、判定できなかったときは 5 秒ごとにやり直す。
- 限界: 校正をやり直したら `dev_calibration/` のファイルも更新しないと判定できない。
- ポートが見えているかの判定は、by-id のようなシンボリックリンクも実体（`/dev/ttyACM0`）に解決して比べる。

### D2. Linux でのプロセス管理
`core/procs.py` を OS で分ける。Windows の処理は今のまま残す。Linux では次のようにする。
- 起動: `start_new_session=True` で新しいプロセスグループを作る。
- 生存確認: `/proc/<pid>/stat` の起動時刻（22 番目の値）を作成時刻として使う。PID の再利用を見分ける考え方は Windows 版と同じ。
- 停止: プロセスグループに `SIGINT` を送る（Ctrl+C と同じ）。`break_as_interrupt.py` を通さなくても Python は `KeyboardInterrupt` を上げるので、mirror_leader の後片付けが走る。待ってもまだ動いていればグループごと `SIGKILL` する。
- 理由: Windows 版と同じ「後片付けを走らせてから止める・止まらなければ子まで殺す」を、Linux の標準の仕組みで実現できる。

### D3. キー送信が使えない環境
`core/key_input.py` の pynput の import を、関数を呼んだときまで遅らせる。`available()` で使えるかを返し、録画画面は使えないときに「この回を終える」「撮り直す」を無効にして理由を表示する。テストモードの偽の録画も同じ扱いにする。
- 理由: 画面のないラズパイで pynput を import すると、mirrorMODE まで含めた画面全体が起動できなくなる。録画をラズパイで動かす方法は本提案の対象外なので、まず画面が落ちないようにする。

### D4. 画面の公開・パスワード・自動起動
add-raspberry-pi-deployment の D2・D3・D4 のとおりに実装する。
- `.streamlit/config.toml` に `server.address = "0.0.0.0"`・`server.port = 8501`・`browser.gatherUsageStats = false`・`server.headless = true` を書く。スマホからは `http://<ラズパイの IP アドレス>:8501` で開く。
- `app.py` で `.streamlit/secrets.toml` の `app_password` と入力値を比べ、一致するまで `st.navigation` を呼ばない。`secrets.toml` は `.gitignore` に入れる。
- `deploy/easy-lerobot.service` を用意する。ラズパイ 4 では OS 付属の Python で作った仮想環境から streamlit を動かす（uv と Python 3.12 は使わない。lerobot 本体を入れないため `uv.lock` の制約が要らない）。

### D5b. 電源を入れた後の準備状況
モード選択の画面に「準備状況」のログを出す（`core/startup.py`）。画面のサーバーの起動時刻（ラズパイの起動から何秒か）、リーダー・フォロワーの基板、5 番関節の範囲ファイル、カメラの台数を 2 秒ごとに確かめ、必須のものがそろったら「準備完了」を出す。カメラは必須にしない。
- 理由: スマホからは、ミラーを始めてよいかが分からない。ログなら部品も配線も要らない。
- 限界: スマホがつながるのは画面のサーバーが起動した後なので、電源を入れてからそれまでの間は表示できない。その間を見せたいときは LED を足す（GPIO と sudo の設定が要るため、本提案では扱わない）。
- シリアルポートは開かない（ポートがあるかだけを見る）。動いているミラーとぶつからないようにするため。

### D5. カメラの接続と名前
USB ハブにカメラ 2 台をつなぐ。名前は `wrist`（フォロワーに付けるカメラ）と `front`（定点カメラ）にする。ラズパイでは `/dev/v4l/by-id/` の名前で区別する。本提案ではコードに入れず、録画をラズパイで動かす段階で `core/recording.py` の `--robot.cameras` に反映する。

## Risks / Trade-offs

- [カメラ 2 台とサーボ基板 2 枚を同じハブにつなぐと帯域や給電が足りない] → 電源付きのハブを使い、ラズパイ 4 の USB 3.0（青い差し込み口）に挿す。実物で確かめる（tasks 4）。
- [パスワードが HTTP で流れる] → 学内の Wi-Fi に限る。add-raspberry-pi-deployment の D3 と同じ限界。
- [streamlit のプロセスを systemd が止めると、ミラーの子プロセスが残る] → `KillMode=process` にして子は残し、画面が戻ったら procs の状態ファイルから見つけ直して止められるようにする（今の作りのまま）。

## Open Questions

- フォロワーのカメラをどこに付けるか（手首かグリッパーか）。
- スマホとラズパイのつなぎ方（いったん保留）。学校の open Wi-Fi は端末同士の通信ができないため、そのままでは使えない。端末同士を直接つなぐ方法は別に決める。それまでの確認は検証用の Wi-Fi で行う。
