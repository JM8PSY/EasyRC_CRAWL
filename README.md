# CuGo V3 Web遠隔操縦

Raspberry Pi 4 から Cytron MD20A ×2 を PWM + DIR で駆動し、同一LAN内のブラウザから USB カメラ映像を見ながら CuGo V3 を操縦します。

- 操作: 仮想ジョイスティック（スマホ）/ WASD キー（PC）
- 通信途絶・切断・タブ非表示などで自動停止

設計: [docs/superpowers/specs/2026-11-10-cugo-webrc-design.md](docs/superpowers/specs/2026-11-10-cugo-webrc-design.md)

## 配線

| 信号 | Pi 4 GPIO (BCM) | ピン番号 |
|---|---|---|
| 左 MD20A PWM | GPIO12（ハードウェアPWM0） | 32 |
| 左 MD20A DIR | GPIO5 | 29 |
| 右 MD20A PWM | GPIO13（ハードウェアPWM1） | 33 |
| 右 MD20A DIR | GPIO6 | 31 |
| GND | GND | 30 / 34 など |

- **Pi と MD20A ×2 の GND を必ず共通化**してください。
- PWM 周波数は 20kHz です。
- **推奨: 各 PWM 線に 10kΩ のプルダウン抵抗（PWM–GND 間）。** Pi の電源断・ハング時など、ソフトウェアで止められない状況での暴走を防ぎます。
- **推奨: モーター電源を遮断できる物理的な非常停止スイッチ。**

## セットアップ（Raspberry Pi OS Bookworm）

```bash
sudo apt update
sudo apt install -y pigpio python3-venv git
sudo systemctl enable --now pigpiod

git clone <このリポジトリ> ~/EasyRC_CRAWL
cd ~/EasyRC_CRAWL
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## 起動

```bash
# 実機
.venv/bin/python -m cugo_rc.server

# Pi 無しで UI を試す（モーター・カメラはダミー。PC でも動作）
.venv/bin/python -m cugo_rc.server --mock
```

ブラウザで `http://<PiのIPアドレス>:8000` を開きます。

### 自動起動（systemd）

`systemd/cugo-rc.service` のパスとユーザーを環境に合わせて編集してから:

```bash
sudo cp systemd/cugo-rc.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now cugo-rc
journalctl -u cugo-rc -f   # ログ確認
```

## 操作方法

| 操作 | PC | スマホ |
|---|---|---|
| 前進 / 後退 | W / S | ジョイスティック上下 |
| 旋回 | A / D | ジョイスティック左右 |
| 低速モード（×0.4） | Shift を押しながら | — |
| 停止 | Space | 赤い停止ボタン |

- 操縦できるのは最初に接続した1台のみです。2台目以降は「閲覧のみ」になります（操縦者が切断すると、次に接続した端末が操縦者になります）。

## 自動停止の条件

次のいずれかで即座にモーター出力を 0 にします。

| 条件 | 内容 |
|---|---|
| 指令途絶 | 最後の指令から 300ms 経過（Wi-Fi 不調・ブラウザのフリーズ） |
| WebSocket 切断 | 切断を検知した時点 |
| タブ非表示 | ブラウザのタブ切替・画面オフ |
| 停止操作 | 停止ボタン / Space |
| サーバー終了 | Ctrl+C、systemd 停止、例外 |
| 起動時 | PWM=0、DIR=Low で初期化 |

## 設定（config.yaml）

| 項目 | 既定値 | 説明 |
|---|---|---|
| `motor.left/right.pwm_pin` | 12 / 13 | PWM ピン（BCM） |
| `motor.left/right.dir_pin` | 5 / 6 | DIR ピン（BCM） |
| `motor.left/right.invert` | false / true | 回転方向の反転 |
| `motor.pwm_freq` | 20000 | PWM 周波数 [Hz] |
| `motor.max_speed` | 1.0 | 出力デューティの上限（0〜1） |
| `motor.max_accel` | 2.0 | 1秒あたりの速度変化の上限 |
| `watchdog.timeout_ms` | 300 | 指令途絶で停止するまでの時間 |
| `camera.device` | 0 | カメラ番号（/dev/video0） |
| `camera.width/height/fps` | 640 / 480 / 15 | 解像度・フレームレート |
| `camera.jpeg_quality` | 70 | JPEG 品質 |
| `server.host/port` | 0.0.0.0 / 8000 | 待ち受けアドレス |

書いた項目だけが既定値を上書きします。綴りを間違えたキーは起動時にエラーになります。

**前進で片輪が逆転する場合**は、そのモーターの `invert` を切り替えてください。

## 実機チェックリスト

初回は必ず **タイヤを浮かせた状態** で確認してください。

1. [ ] 前進・後退・左旋回・右旋回の向きが正しい（違えば `invert` を調整）
2. [ ] 走行中に Wi-Fi を切断 → 停止する
3. [ ] 走行中にブラウザのタブを切り替える → 停止する
4. [ ] W と S を素早く切り替えても急反転せず、なめらかに加減速する
5. [ ] サーバーを Ctrl+C で終了 → 停止する

## 開発

```bash
python -m venv .venv
.venv/bin/pip install -r requirements-dev.txt   # Windows は .venv\Scripts\pip
.venv/bin/python -m pytest
```
