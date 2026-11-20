# Proposal

## Why

add-pi4-mobile-mirror では、スマホとラズパイを同じ Wi-Fi に入れて `http://<ラズパイ>:8501` を開く形にした。来場者が自分のスマホで操作する展示では、この形だと次の問題がある。

- 来場者に毎回 Wi-Fi を切り替えてもらう必要があり、その間スマホのインターネットが使えないことがある
- 学校の open Wi-Fi は端末同士の通信ができないため使えず、使えるのは検証用の Wi-Fi かテザリングに限られる
- ラズパイの Wi-Fi を切り替えると、来場者のスマホも全部つなぎ直しになり、URL も変わる

そこで、インターネット上の中継サーバー（VPS）を通して、来場者はモバイル回線のまま QR の URL を開くだけで操作できるようにする。

## What Changes

- **中継（実装済み・検証済み）**: ラズパイから VPS へ ssh の逆向きトンネルを張り、VPS の nginx が HTTPS で受けた通信をラズパイの画面（8501 番）へ渡す。ラズパイの ssh（22 番）も同じトンネルで VPS に出し、開発用の PC はどの Wi-Fi からでも VPS 経由でラズパイに入れる。
- **ラズパイの Wi-Fi は人が切り替える**: 自動の切り替えは入れない。ラズパイがインターネットに出られれば、どの Wi-Fi でも来場者の URL は変わらない。
- **命令の通り道を MQTT にする（提案・未実装）**: 測ったところ、Streamlit の画面をトンネル越しに開くと非常に遅い（後述）。来場者の画面を VPS から配る小さなページにし、ラズパイとは MQTT でボタンと状態だけをやり取りする。
- **手首カメラの映像はラズパイの HDMI のディスプレイに出す**: 映像はネットで送らない。
- add-raspberry-pi-deployment の design.md にある「インターネット側への転送（ポート開放やトンネル）は設定しない」を、展示の構成に限って改める。

## Capabilities

### New Capabilities
- `festival-relay`: VPS を通した来場者のスマホからの操作、ラズパイの Wi-Fi の扱い、MQTT による命令と状態のやり取り、手首カメラの表示。

### Modified Capabilities
（なし。add-raspberry-pi-deployment と add-pi4-mobile-mirror はまだアーカイブ前のため、design の記述の改めは本提案の design.md に書く）

## Impact

- 追加するファイル: `easy_lerobot/deploy/relay-tunnel.service.example`、`easy_lerobot/deploy/README.md` の中継の節。MQTT を実装する段階で、ラズパイ側の小さな常駐プログラムと、VPS に置く来場者用のページを足す。
- 外部のもの: VPS 1 台（ssh・nginx・Let's Encrypt。MQTT を入れる段階で mosquitto）。展示が終わったら止める前提。
- 依存: 中継だけなら追加なし。MQTT を実装する段階でラズパイに `paho-mqtt` を足す。
- 未確認: 来場者のスマホ（モバイル回線）から開いたときの速さ。会場でのモバイル回線の混み具合。
