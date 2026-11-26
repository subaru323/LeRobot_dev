# Spec Delta

## Purpose

端末同士が遮断された Wi-Fi やモバイル回線からでも、参加を認めた端末だけがラズパイの画面と ssh に届くようにする。そのための tailnet への参加、アクセス制御、公開の禁止、経路の確認、自動接続を定める。

## ADDED Requirements

### Requirement: tailnet を通した画面と ssh
tailnet に参加した端末は、ラズパイと同じ Wi-Fi にいなくても、ラズパイのホスト名 `lerobot-pi` で画面（8501 番）を開き、ssh（22 番）で入れなければならない（SHALL）。ラズパイ側では、インターネットに向けてポートを開けてはならない（MUST NOT）。

#### Scenario: 端末同士が遮断された Wi-Fi から開く
- **WHEN** ラズパイと係の端末が、端末同士の通信を遮断する同じ Wi-Fi にいて、係の端末で `http://lerobot-pi:8501` を開く
- **THEN** アプリのパスワード入力欄が表示され、正しいパスワードを入れるとモード選択の画面になる

#### Scenario: モバイル回線から ssh で入る
- **WHEN** モバイル回線につないだ開発用の PC から `ssh lerobot@lerobot-pi` を実行する
- **THEN** ラズパイのパスワードを聞かれ、入力するとラズパイに入れる

### Requirement: 参加を認めた端末だけが届く
tailnet に参加していない端末は、tailnet の経路でラズパイに届いてはならない（MUST NOT）。tailnet の中でも、ラズパイへ通す通信は tcp 8501 と tcp 22 に限らなければならない（SHALL）。

#### Scenario: 参加していない端末から開く
- **WHEN** tailnet に参加していないスマホで、ラズパイの tailnet のアドレス（100.x.y.z）の 8501 番を開く
- **THEN** 画面は開けない

#### Scenario: 許していないポートに届くか
- **WHEN** tailnet に参加した端末から、ラズパイの 8501 番と 22 番以外のポートへつなぐ
- **THEN** つながらない

### Requirement: tailnet の外へ公開しない
ラズパイの画面と ssh を、Tailscale の Funnel などの機能で tailnet の外（インターネット）へ公開してはならない（MUST NOT）。

#### Scenario: 公開の設定を確かめる
- **WHEN** ラズパイで `tailscale funnel status` を実行する
- **THEN** 公開している待ち受けは 1 つもない

### Requirement: 電源を入れると tailnet につながる
ラズパイは、電源を入れてインターネットに出られる Wi-Fi に入ると、人の操作なしで tailnet につながらなければならない（SHALL）。ラズパイのノード鍵は、期限切れで切断されてはならない（MUST NOT）。

#### Scenario: 電源を入れ直す
- **WHEN** ラズパイの電源を入れ直し、起動と Wi-Fi への接続が終わるまで待つ
- **THEN** 係の端末から `http://lerobot-pi:8501` を開ける

### Requirement: 使う前に経路を確かめる
会場で使う前に、係の端末からラズパイへの経路が直接か中継（DERP）かを確かめ、画面が開くまでの時間とあわせて記録しなければならない（SHALL）。確かめる手順は `deploy/README.md` に書く（SHALL）。

#### Scenario: 経路を確かめる
- **WHEN** 係の端末で `tailscale ping lerobot-pi` を実行する
- **THEN** 経路が直接か `via DERP` かが表示され、その結果と画面が開くまでの秒数を記録できる
