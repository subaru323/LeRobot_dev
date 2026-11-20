# ラズパイ 4 で easy_lerobot を動かす

スマホのブラウザから mirrorMODE を操作するための手順。仕様は `openspec/changes/add-pi4-mobile-mirror/` を参照。

## 前提

- Raspberry Pi 4 Model B、Raspberry Pi OS Lite（64 ビット）
- 利用者 `lerobot`、リポジトリは `~/LeRobot_dev`
- ラズパイとスマホが同じ Wi-Fi にいる

## 1. 依存を入れる

ラズパイ 4 では lerobot 本体と torch を入れない。ミラーに要る分だけを OS 付属の Python の仮想環境に入れる。

```bash
python3 -m venv ~/venv
~/venv/bin/pip install feetech-servo-sdk pyserial "streamlit>=1.40,<2.0"
```

## 2. パスワードとポートを設定する

```bash
cd ~/LeRobot_dev/easy_lerobot
cp .streamlit/secrets.toml.example .streamlit/secrets.toml    # app_password を書き換える（変えたらサーバーを起動し直す）
```

リーダーとフォロワーのポートは、画面が自動で見分ける（モーターに書かれた原点の値を `dev_calibration/` の校正ファイルと照らし合わせる。読むだけで動かさない）。どの差し込み口にさしてもよい。モード選択の画面の準備状況に「モーター6台が校正と一致」と出れば判定できている。

自動判定を使わずにポートを固定したいときだけ、`deploy/easy-lerobot.env.example` を `deploy/easy-lerobot.env` にコピーして by-id の名前を書く（`ls -l /dev/serial/by-id/` で確認できる）。

判定できないうち（アームの電源が切れている、校正ファイルと合わない）は「未判定」になり、ミラーを始められない。`/dev/ttyACM0`・`1` の番号は差す順で入れ替わるので、既定値としては使わない。

## 3. 手で起動して確かめる

```bash
cd ~/LeRobot_dev/easy_lerobot
set -a; . deploy/easy-lerobot.env; set +a     # ポートの設定を読む（ファイルがあるとき）
~/venv/bin/python -m streamlit run app.py
```

スマホで `http://<ラズパイの IP アドレス>:8501` を開く。IP アドレスはラズパイで `hostname -I` を実行すると分かる。

## 4. 電源を入れると立ち上がるようにする（sudo が要る）

```bash
sudo usermod -aG dialout,video lerobot
sudo cp ~/LeRobot_dev/easy_lerobot/deploy/easy-lerobot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now easy-lerobot
systemctl status easy-lerobot            # active (running) なら成功
journalctl -u easy-lerobot -f            # 画面のログを見る
```

止めるときは `sudo systemctl stop easy-lerobot`。自動起動をやめるときは `sudo systemctl disable easy-lerobot`。

## 電源ボタン（任意）

GPIO3（5 番ピン）と GND（6 番ピン）の間にタクトスイッチをつなぎ、`/boot/firmware/config.txt` の最後に `dtoverlay=gpio-shutdown` を足して再起動する。短く押すとシャットダウン、止まっているときに短く押すと起動する。

- 5 秒以上の長押しは何もしない（systemd-logind の既定 `HandlePowerKeyLongPress=ignore`）
- デスクトップが自動ログインで動いていると、デスクトップの部品（rpd-common の `pwrkey.desktop`）が電源ボタンの処理を止めるため、シャットダウンしない。画面をつながずに使うなら、`sudo raspi-config nonint do_boot_behaviour B1` で文字の画面で起動するようにする
- シャットダウンでは全プロセスに SIGTERM が送られる。ミラー中でも後片付けが走る（`core/break_as_interrupt.py`）

## ログ

`easy_lerobot/logs/` に残る（git 管理外。再起動しても消えない）。

| ファイル | 中身 |
|---|---|
| `mirror.log` / `record.log` | いちばん新しいミラー・録画の出力 |
| `mirror-YYYYmmdd-HHMMSS.log` など | それより前の回。次の回を始めたときに名前を変えて残す。種類ごとに新しい 200 回分まで |
| `server.log` | 画面のサーバーの出力（systemd で起動したとき） |

```bash
ls -lt ~/LeRobot_dev/easy_lerobot/logs | head          # 新しい順
tail -n 30 ~/LeRobot_dev/easy_lerobot/logs/mirror.log    # 直近のミラー
```

## 分かっている限界

- 録画の「この回を終える」「撮り直す」は、キー送信を使うため画面のないラズパイでは押せない。時間が来ると自動で進む
- 通信は HTTP。パスワードは学内の Wi-Fi で使う前提

## インターネット経由の中継（展示用・任意）

来場者が自分のスマホのモバイル回線から開けるように、VPS を中継にする。仕組みと決めたことは `openspec/changes/add-festival-relay/` を参照。

1. ラズパイでトンネル用の鍵を作る（sudo は要らない）

   ```bash
   ssh-keygen -t ed25519 -N "" -C "lerobot-pi-relay" -f ~/.ssh/relay_ed25519
   ```

2. VPS に、ログインできない専用ユーザー `relay` を作り、公開鍵を制限つきで登録する（`/home/relay/.ssh/authorized_keys`）

   ```
   restrict,port-forwarding,permitlisten="127.0.0.1:8501",permitlisten="127.0.0.1:2222" ssh-ed25519 AAAA... lerobot-pi-relay
   ```

3. `deploy/relay-tunnel.service.example` の `<VPS のホスト名>` を書き換えて登録する（sudo が要る）

   ```bash
   sudo cp relay-tunnel.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable --now relay-tunnel
   systemctl status relay-tunnel            # active (running) なら成功
   ```

4. スマホで `https://<VPS のホスト名>/` を開く。開発用の PC からは `ProxyJump` で VPS を経由し、VPS の `127.0.0.1:2222` へ ssh するとラズパイに入れる

展示が終わったら `sudo systemctl disable --now relay-tunnel` で止める。
