# CuGo V3 Web遠隔操縦システム 設計仕様

- 日付: 2026-11-10
- 対象: Raspberry Pi 4 + CuGo V3 (左右2モーター) + Cytron MD20A ×2 + USB Webカメラ ×1

## 1. 目的と成功基準

同一LAN内のPC/スマホのブラウザから、カメラ映像を見ながらCuGo V3を遠隔操縦する。

成功基準:
- ブラウザで `http://<PiのIP>:8000` を開くと映像が表示され、仮想ジョイスティックおよびWASDキーで前後・旋回できる
- 通信途絶・切断・タブ非表示・サーバー終了のいずれでもモーターが停止する（自動停止は必須要件）
- Piが無くてもPC上で `--mock` により全体（Web UI含む）を動かせる

ユーザー要件として確定した事項:
- モーター駆動は MD20A（PWM + DIR）を Pi の GPIO から直接制御する
- 利用形態は同一LAN内の遠隔操縦
- 通信途絶時の自動停止は必須
- Raspberry Pi 4
- 操作は仮想ジョイスティック（スマホ）とWASD（PC）の両方

前提（設計上の仮定）:
- MD20A は1chのドライバのため左右に1台ずつ計2台使う
- エンコーダは使わずオープンループ制御とする

## 2. 全体構成

```
[ブラウザ(PC/スマホ)]
   ├─ <img src="/video">  ←── MJPEG ── camera.py ── USBカメラ(/dev/video0)
   └─ WebSocket /ws  ── {"l":-1..1, "r":-1..1} ──→ server.py
                                                    ├─ watchdog.py（指令途絶で停止）
                                                    └─ motor.py ── pigpio ── MD20A ×2 ── CuGo V3
```

技術スタック: Python 3 / FastAPI + uvicorn / pigpio（ハードウェアPWM） / OpenCV（カメラ取得・JPEG化） / フロントエンドはビルド不要の素のHTML+JS。

## 3. コンポーネント

| モジュール | 責務 | 依存 |
|---|---|---|
| `cugo_rc/motor.py` | 左右速度 (-1.0〜1.0) を PWMデューティ比と DIR に変換して出力。範囲制限・最大速度制限・加速度制限を担当。`set(l, r)` / `stop()` を提供。GPIO操作はバックエンドとして注入可能（`PigpioBackend` / `MockBackend`） | pigpio |
| `cugo_rc/watchdog.py` | 最終指令時刻を監視し、タイムアウト（既定300ms）で `motor.stop()` を呼ぶ。独立スレッドで動作。時計は注入可能 | motor |
| `cugo_rc/camera.py` | USBカメラからフレームを取得し JPEG 化、MJPEG ストリームとして提供。失敗時は数秒間隔で再接続。mock時はテスト画像を配信 | OpenCV |
| `cugo_rc/server.py` | FastAPI アプリ。`/`（UI配信）、`/ws`（操縦指令）、`/video`（MJPEG）。操縦権の管理、切断時・終了時の停止 | 上記すべて |
| `cugo_rc/config.py` | `config.yaml` の読み込みと既定値 | PyYAML |
| `cugo_rc/static/` | `index.html` / `app.js` / `style.css`。映像表示、仮想ジョイスティック、WASD、停止ボタン、接続状態表示 | なし |

## 4. ハードウェア配線（Pi 4）

| 信号 | GPIO (BCM) | 備考 |
|---|---|---|
| 左 MD20A PWM | GPIO12 | ハードウェアPWM0 |
| 左 MD20A DIR | GPIO5 | |
| 右 MD20A PWM | GPIO13 | ハードウェアPWM1 |
| 右 MD20A DIR | GPIO6 | |
| GND | GND | Pi・MD20A×2 で共通化 |

- PWM周波数: 20kHz（可聴ノイズ回避。MD20A の上限 20kHz 以内）
- 推奨: 各PWM線に 10kΩ プルダウン抵抗（Pi電源断・ハング時の暴走防止）
- 推奨: 物理的な非常停止スイッチ（モーター電源を遮断）
- 左右どちらかのモーター回転方向が逆の場合は `config.yaml` の `invert` で反転する

## 5. 設定 (`config.yaml`)

```yaml
motor:
  left:  { pwm_pin: 12, dir_pin: 5, invert: false }
  right: { pwm_pin: 13, dir_pin: 6, invert: true }
  pwm_freq: 20000
  max_speed: 1.0        # 0.0〜1.0。出力デューティの上限
  max_accel: 2.0        # 1秒あたりの速度変化量の上限
watchdog:
  timeout_ms: 300
camera:
  device: 0
  width: 640
  height: 480
  fps: 15
  jpeg_quality: 70
server:
  host: 0.0.0.0
  port: 8000
```

## 6. データフロー

### 6.1 操縦指令
1. ブラウザは入力を左右速度 `{l, r}` に変換し、**50ms周期で常時送信**する（入力ゼロでも送信＝生存信号）
   - ミキシング: `l = 前後 + 旋回`, `r = 前後 − 旋回`。どちらかの絶対値が1を超えたら両方を最大絶対値で割って正規化
   - WASD: W/S = 前後 ±1、A/D = 旋回 ±0.6、Shift押下中は ×0.4（低速モード）
   - 仮想ジョイスティック: Y軸 = 前後、X軸 = 旋回。指を離すと 0 に戻る
2. メッセージ形式: `{"l": float, "r": float}`。停止は `{"stop": true}`
3. サーバー側処理:
   - 不正なJSON・数値でない値は無視（watchdog はリセットしない）
   - `[-1, 1]` にクランプ → `max_speed` を乗算
   - 加速度制限（`max_accel`）を適用した値を出力。制御ループは 50Hz で motor の目標値へ漸近させる
   - **安全停止（`{"stop": true}`・watchdog・切断・終了時の `stop()`）は加速度制限をバイパスし即時 PWM=0**
   - 通常指令の `{"l":0,"r":0}`（スティックを離した等）は加速度制限に従って減速する
4. 出力: 速度 `v` → DIR = (`v` ≥ 0 ? Low : High)（`invert` で反転）、デューティ = `|v|`

### 6.2 操縦権
- 操縦は同時に1つの WebSocket 接続のみ。最初の接続が操縦権を持つ
- 2番目以降の接続には `{"role": "viewer"}` を通知し、指令は無視する（映像は閲覧可）
- 操縦者が切断されたら停止し、操縦権を解放する（次に接続したクライアントが取得）
- 接続時に `{"role": "driver"}` または `{"role": "viewer"}` をサーバーから送る

### 6.3 映像
- `/video` は `multipart/x-mixed-replace` の MJPEG。`<img>` で表示
- カメラ取得は単一スレッドで行い、最新フレームを複数クライアントに配信
- カメラ異常時は配信停止・再接続試行。UI に「映像なし」警告を表示。**操縦は継続可能**

## 7. 自動停止（安全要件）

以下のいずれかで PWM を 0 にする:

| 条件 | 検出 |
|---|---|
| 指令途絶（Wi-Fi不調、ブラウザフリーズ） | watchdog: 最後の有効指令から 300ms |
| WebSocket 切断 | 切断イベントで即時 |
| ブラウザのタブ非表示 | UI が `visibilitychange` で `{"stop": true}` を送り送信停止 |
| UIの停止ボタン / スペースキー | `{"stop": true}` |
| サーバー終了（Ctrl+C / SIGTERM / 例外） | lifespan の終了処理と `atexit` で `stop()`、pigpio 解放 |
| 起動時 | 初期化で PWM=0、DIR=Low |

- watchdog はメインのイベントループとは独立したスレッドで動作し、処理は停止のみの最小限とする
- ソフトウェアで防げないケース（Pi電源断・カーネルハング）は第4節のプルダウン抵抗と非常停止スイッチで対処する（README に明記）

## 8. エラーハンドリング

- pigpiod に接続できない場合: 起動時にエラーメッセージ（`sudo systemctl enable --now pigpiod` を案内）を出して終了
- カメラが開けない場合: 警告ログを出し、再接続を試行しながらサーバーは起動を継続
- WebSocket の不正メッセージ: 無視してログに記録（停止はwatchdogに任せる）

## 9. テスト

PC上で pytest により実行可能:
- `motor`: MockBackend で、速度→デューティ/DIR 変換、`invert`、クランプ、`max_speed`、加速度制限、停止の即時性
- `watchdog`: 注入した時計で、タイムアウトで停止・指令受信でリセット
- `server`: FastAPI TestClient で、WebSocket 指令がモーターに届く、切断で停止、2番目の接続は viewer になり指令が無視される、`stop` メッセージで即停止
- `--mock` 起動: Pi 無しで UI を含めた全体を手動確認できる

実機チェックリスト（README）:
1. タイヤを浮かせて前進・後退・旋回の向きを確認（必要なら `invert` を調整）
2. 走行中に Wi-Fi を切断 → 停止すること
3. 走行中にタブを切り替え → 停止すること
4. 急な前後切替でも加速度制限が効いていること
5. サーバーを Ctrl+C で終了 → 停止すること

## 10. 起動・運用

- `pip install -r requirements.txt`
- `sudo systemctl enable --now pigpiod`
- 手動起動: `python -m cugo_rc.server [--config config.yaml] [--mock]`
- 自動起動: `systemd/cugo-rc.service`（`Restart=on-failure`）
- 認証なし（LAN内利用前提）

## 11. ディレクトリ構成

```
EasyRC_CRAWL/
├─ cugo_rc/
│   ├─ __init__.py
│   ├─ motor.py
│   ├─ watchdog.py
│   ├─ camera.py
│   ├─ server.py
│   ├─ config.py
│   └─ static/ (index.html, app.js, style.css)
├─ tests/
├─ config.yaml
├─ requirements.txt
├─ systemd/cugo-rc.service
└─ README.md
```

## 12. スコープ外

エンコーダによる速度フィードバック制御、録画、インターネット越しの操縦、WebRTC映像、認証。
