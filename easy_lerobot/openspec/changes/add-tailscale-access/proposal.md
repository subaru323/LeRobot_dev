# Proposal

## Why

ラズパイの画面には、これまで 2 つの経路でつないできた。どちらにも問題がある。

- 同じ Wi-Fi から `http://<ラズパイ>:8501` を開く経路（add-pi4-mobile-mirror）：学校の Wi-Fi は端末同士の通信を遮断していることがあり、同じ SSID にいてもラズパイに届かない。
- VPS を通す経路（add-festival-relay）：VPS の契約に費用がかかる。また、インターネットに公開するため、edge-deployment の「インターネット側へは公開しない（MUST NOT）」と矛盾する（P-009）。

ほかにも、ラズパイの Wi-Fi はインターネット側が遅く（GitHub から約 9KB/秒、2026-11-23 計測）、保守のたびに同じ LAN にいる PC が必要だった。

そこで、費用をかけずに、端末同士が遮断された Wi-Fi でも、決められた端末からラズパイの画面と ssh に届くようにする。仕組みには Tailscale（無料の個人プラン）を使う。

## What Changes

- ラズパイを Tailscale のネットワーク（tailnet）に参加させ、電源を入れると自動でつながるようにする。
- 操作する端末（開発用の PC、係の人のスマホ・タブレット）も同じ tailnet に参加させる。参加した端末は、どの Wi-Fi やモバイル回線にいても `http://lerobot-pi:8501` で画面を開き、`ssh lerobot@lerobot-pi` でラズパイに入れる。
- 来場者は自分のスマホを使わず、会場に置いた係の端末（tailnet に参加済み）で操作する。
- Tailscale の公開機能（Funnel）は使わない。画面は tailnet の外へ公開しない。
- tailnet の中でも、ラズパイに届くのは画面（8501 番）と ssh（22 番）だけに限る（Tailscale のアクセス制御）。
- VPS の中継（relay-tunnel）は使わない。本提案の経路が動くことを確かめたあと、ラズパイの relay-tunnel を止める。
- 同じ Wi-Fi から直接開く経路は残す。端末同士が通じる Wi-Fi では、今までどおり使える。

## Capabilities

### New Capabilities
- `tailnet-access`: Tailscale を通したラズパイへの接続、参加させる端末とアクセス制御、公開の禁止、接続経路（直接か中継か）の確認、自動接続。

### Modified Capabilities
（なし。add-raspberry-pi-deployment と add-festival-relay はアーカイブ前のため、両提案との関係は本提案の design.md に書く）

## Impact

- 追加・変更するファイル：`easy_lerobot/deploy/README.md`（Tailscale の節を追加し、中継の節に「使わない」旨を書く）、`easy_lerobot/deploy/tailscale-acl.example.json`（新規。アクセス制御の雛形）。アプリのコード（`app.py`、`core/`、`views/`）は変えない。
- 外部のもの：Tailscale のアカウント（無料の個人プラン）。VPS は不要になる。
- ラズパイ：`tailscale` パッケージを入れる（OS のパッケージ管理の外にある公式リポジトリから）。Python の依存は増えない。
- 未確認：
  - 端末同士が遮断された Wi-Fi で、Tailscale の通信が直接つながるか、中継サーバー（DERP）経由になるか。中継になると、速さはその Wi-Fi のインターネット回線に左右される。
  - ラズパイの遅い回線で、パッケージの取得とログインにどれだけかかるか。
