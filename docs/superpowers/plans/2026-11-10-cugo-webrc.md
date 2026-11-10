# CuGo V3 Web遠隔操縦 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Raspberry Pi 4 上で動き、同一LAN内のブラウザからUSBカメラ映像を見つつ CuGo V3（MD20A×2）を操縦できるWebサーバーを作る。

**Architecture:** FastAPI サーバーが `/ws`（操縦指令）と `/video`（MJPEG）を提供する。`MotorController` は差し替え可能なバックエンド（pigpio / mock）経由でPWM+DIRを出し、50Hz制御ループで加速度制限をかける。独立スレッドの `Watchdog` が指令途絶時に即停止する。

**Tech Stack:** Python 3.11+, FastAPI, uvicorn, PyYAML, OpenCV (headless), numpy, pigpio, pytest, httpx

**Spec:** `docs/superpowers/specs/2026-11-10-cugo-webrc-design.md`

## Global Constraints

- Python >= 3.11（Raspberry Pi OS Bookworm 標準）。Pi では venv を使う
- `pigpio` は `PigpioBackend` 内で遅延 import する（PC/Windows で pytest と `--mock` が動くこと）
- 既定ピン: 左 PWM=GPIO12 / DIR=GPIO5、右 PWM=GPIO13 / DIR=GPIO6（BCM番号）、`right.invert: true`
- PWM周波数 20000Hz、watchdog タイムアウト 300ms、UI送信周期 50ms、制御ループ 50Hz、既定 `max_accel` 2.0/s、`max_speed` 1.0
- 安全停止（`{"stop": true}`・watchdog・切断・終了）は加速度制限をバイパスして即 duty=0。通常の `{l:0,r:0}` は加速度制限に従う
- DIR: 出力速度 v（invert適用後）が v>=0 なら 0(Low)、v<0 なら 1(High)。duty=|v|
- サーバー既定 `0.0.0.0:8000`。認証なし
- フロントエンドはビルド不要の素の HTML/JS/CSS

## Review Focus

1. **NaN / Infinity / bool / 文字列の速度値**: `{"l": NaN}` や `{"l": true}` がモーターに届くと暴走しうる → 無視され watchdog もリセットされないこと（Task 2, Task 5 にテスト）
2. **操縦者が切断した直後の再接続**: 切断で停止し、操縦権が解放され、次の接続が driver になること（Task 5 にテスト）
3. **watchdog と制御ループの競合**: watchdog スレッドの `stop()` 直後に制御ループの `tick()` が古い target で再加速しないこと（`stop()` は target も 0 にする。Task 2 にテスト）
4. **config.yaml の部分指定**: `camera:` だけ書いた YAML でも他は既定値で起動すること（Task 1 にテスト）
5. **カメラ不在で起動**: `/dev/video0` が無くてもサーバーが起動し、操縦でき、`/status` が `camera:false` を返すこと（Task 4, Task 5 にテスト）

---

## File Structure

| File | Responsibility |
|---|---|
| `cugo_rc/__init__.py` | 空 |
| `cugo_rc/config.py` | dataclass 定義と YAML 読み込み（既定値マージ） |
| `cugo_rc/motor.py` | バックエンド（Mock/Pigpio）と `MotorController` |
| `cugo_rc/watchdog.py` | 指令途絶監視スレッド |
| `cugo_rc/camera.py` | カメラ取得スレッド、JPEG化、MJPEG ジェネレータ、mock 画像 |
| `cugo_rc/server.py` | FastAPI アプリ生成、WS/映像/状態エンドポイント、lifespan、`main()` |
| `cugo_rc/static/index.html`, `app.js`, `style.css` | 操縦UI |
| `config.yaml` | 既定設定（spec §5 と同値） |
| `requirements.txt`, `requirements-dev.txt` | 依存 |
| `systemd/cugo-rc.service` | 自動起動 |
| `README.md` | 配線・セットアップ・実機チェックリスト |
| `tests/test_config.py`, `test_motor.py`, `test_watchdog.py`, `test_camera.py`, `test_server.py` | テスト |

---

### Task 1: プロジェクト雛形と設定読み込み

**Files:**
- Create: `cugo_rc/__init__.py`, `cugo_rc/config.py`, `config.yaml`, `requirements.txt`, `requirements-dev.txt`, `.gitignore`, `pytest.ini`
- Test: `tests/test_config.py`

**Interfaces:**
- Produces:
  - `@dataclass MotorSide(pwm_pin: int, dir_pin: int, invert: bool = False)`
  - `@dataclass MotorConfig(left: MotorSide, right: MotorSide, pwm_freq: int = 20000, max_speed: float = 1.0, max_accel: float = 2.0)`
  - `@dataclass WatchdogConfig(timeout_ms: int = 300)`
  - `@dataclass CameraConfig(device: int = 0, width: int = 640, height: int = 480, fps: int = 15, jpeg_quality: int = 70)`
  - `@dataclass ServerConfig(host: str = "0.0.0.0", port: int = 8000)`
  - `@dataclass Config(motor, watchdog, camera, server)`
  - `default_config() -> Config`（左 12/5/False、右 13/6/True）
  - `load_config(path: str | None) -> Config` — `None` なら既定値。YAML の各セクションを既定値に浅くマージ（motor.left/right はさらに1段マージ）。未知キーは `ValueError`

- [ ] **Step 1: 雛形作成**
  - `requirements.txt`: `fastapi`, `uvicorn[standard]`, `pyyaml`, `numpy`, `opencv-python-headless`, `pigpio`
  - `requirements-dev.txt`: `-r requirements.txt`, `pytest`, `httpx`
  - `pytest.ini`: `testpaths = tests`
  - `.gitignore`: `__pycache__/`, `.venv/`, `.pytest_cache/`
  - `config.yaml`: spec §5 の内容そのまま
  - `python -m venv .venv` → `pip install -r requirements-dev.txt`

- [ ] **Step 2: 失敗するテストを書く** (`tests/test_config.py`)

```python
def test_default_when_none():
    c = load_config(None)
    assert (c.motor.left.pwm_pin, c.motor.left.dir_pin, c.motor.left.invert) == (12, 5, False)
    assert (c.motor.right.pwm_pin, c.motor.right.dir_pin, c.motor.right.invert) == (13, 6, True)
    assert c.motor.pwm_freq == 20000 and c.watchdog.timeout_ms == 300
    assert c.server.port == 8000

def test_repo_config_yaml_matches_defaults():
    assert load_config("config.yaml") == default_config()

def test_partial_yaml_keeps_defaults(tmp_path):
    p = tmp_path / "c.yaml"; p.write_text("camera:\n  width: 320\nmotor:\n  right:\n    invert: false\n")
    c = load_config(str(p))
    assert c.camera.width == 320 and c.camera.height == 480
    assert c.motor.right.invert is False and c.motor.right.pwm_pin == 13

def test_unknown_key_raises(tmp_path):
    p = tmp_path / "c.yaml"; p.write_text("motor:\n  max_sped: 0.5\n")
    with pytest.raises(ValueError):
        load_config(str(p))
```

- [ ] **Step 3: 失敗を確認** — Run: `pytest tests/test_config.py -v` / Expected: FAIL (ImportError)
- [ ] **Step 4: `cugo_rc/config.py` を実装**
- [ ] **Step 5: 通過を確認** — Run: `pytest tests/test_config.py -v` / Expected: 4 passed
- [ ] **Step 6: Commit** — `git add -A && git commit -m "feat: project scaffold and config loader"`

---

### Task 2: モーター制御

**Files:**
- Create: `cugo_rc/motor.py`
- Test: `tests/test_motor.py`

**Interfaces:**
- Consumes: `MotorConfig`, `MotorSide` (Task 1)
- Produces:
  - `class BackendError(RuntimeError)`
  - `class MockBackend`: `set_pwm(pin: int, freq: int, duty: float) -> None`, `set_dir(pin: int, level: int) -> None`, `close() -> None`; 記録用属性 `duty: dict[int, float]`, `freq: dict[int, int]`, `dir: dict[int, int]`, `closed: bool`
  - `class PigpioBackend`: 同じ3メソッド。`__init__` で `import pigpio; pi = pigpio.pi()`、`not pi.connected` なら `BackendError("pigpiod に接続できません。sudo systemctl enable --now pigpiod を実行してください")`。`set_pwm` は `pi.hardware_PWM(pin, freq, int(duty * 1_000_000))`、`set_dir` は `set_mode(OUTPUT)` 済みピンへ `pi.write`、`close` は両PWMを0にして `pi.stop()`
  - `class MotorController(cfg: MotorConfig, backend)`:
    - `set_target(l: float, r: float) -> None` — 有限な数値（bool不可）でなければ `ValueError`。`[-1,1]` にクランプ後 `× max_speed`
    - `tick(dt: float) -> tuple[float, float]` — current を target へ `max_accel * dt` 以内で近づけて出力し、current を返す
    - `stop() -> None` — target と current を (0,0) にし即出力（duty=0）
    - `close() -> None` — `stop()` 後 `backend.close()`
    - プロパティ `target: tuple[float, float]`, `current: tuple[float, float]`
    - 全メソッドを `threading.Lock` で保護（watchdog スレッドから `stop()` が呼ばれる）
    - コンストラクタで両輪 duty=0、DIR=0 を出力

- [ ] **Step 1: 失敗するテストを書く** (`tests/test_motor.py`)。ヘルパ `make(max_accel=2.0, max_speed=1.0)` で既定ピン（右 invert=True）の `MotorController` と `MockBackend` を返す

```python
def test_init_outputs_zero():
    m, b = make()
    assert b.duty == {12: 0.0, 13: 0.0} and b.dir == {5: 0, 6: 0}
    assert b.freq[12] == 20000

def test_forward_direction_and_invert():
    m, b = make(max_accel=1000)
    m.set_target(0.5, 0.5); m.tick(0.02)
    assert b.duty[12] == pytest.approx(0.5) and b.dir[5] == 0
    assert b.duty[13] == pytest.approx(0.5) and b.dir[6] == 1   # 右は invert

def test_reverse_left():
    m, b = make(max_accel=1000)
    m.set_target(-0.3, 0); m.tick(0.02)
    assert b.duty[12] == pytest.approx(0.3) and b.dir[5] == 1

def test_clamp_and_max_speed():
    m, b = make(max_accel=1000, max_speed=0.5)
    m.set_target(3.0, -3.0)
    assert m.target == (0.5, -0.5)

@pytest.mark.parametrize("bad", [float("nan"), float("inf"), True, "1", None])
def test_invalid_values_rejected(bad):
    m, b = make()
    with pytest.raises(ValueError):
        m.set_target(bad, 0)
    assert m.target == (0.0, 0.0)

def test_accel_limit():
    m, b = make(max_accel=2.0)
    m.set_target(1.0, 1.0)
    assert m.tick(0.1) == pytest.approx((0.2, 0.2))
    m.set_target(0, 0)
    assert m.tick(0.05) == pytest.approx((0.1, 0.1))   # 0指令は減速

def test_stop_bypasses_accel_and_clears_target():
    m, b = make(max_accel=2.0)
    m.set_target(1, 1); [m.tick(0.1) for _ in range(5)]
    m.stop()
    assert b.duty == {12: 0.0, 13: 0.0} and m.current == (0.0, 0.0)
    m.tick(0.1)
    assert m.current == (0.0, 0.0)   # 古い target で再加速しない

def test_close():
    m, b = make(); m.close()
    assert b.closed and b.duty == {12: 0.0, 13: 0.0}
```

- [ ] **Step 2: 失敗を確認** — Run: `pytest tests/test_motor.py -v` / Expected: FAIL (ImportError)
- [ ] **Step 3: `cugo_rc/motor.py` を実装**
- [ ] **Step 4: 通過を確認** — Run: `pytest tests/test_motor.py -v` / Expected: all passed
- [ ] **Step 5: Commit** — `git commit -am "feat: motor controller with mock and pigpio backends"`（新規ファイルは `git add` してから）

---

### Task 3: Watchdog

**Files:**
- Create: `cugo_rc/watchdog.py`
- Test: `tests/test_watchdog.py`

**Interfaces:**
- Produces: `class Watchdog(timeout_s: float, on_timeout: Callable[[], None], clock: Callable[[], float] = time.monotonic)`
  - `feed() -> None` — 最終指令時刻を更新し「武装」する
  - `check() -> bool` — 武装中かつ `clock() - last > timeout_s` なら `on_timeout()` を呼び、武装解除して True。それ以外 False（初回 feed 前・発火後は次の feed まで発火しない）
  - `start(interval_s: float = 0.02) -> None` — daemon スレッドで `check()` を周期実行。`on_timeout` の例外はログして継続
  - `shutdown() -> None` — スレッド停止・join

- [ ] **Step 1: 失敗するテストを書く**（`FakeClock` で `t` を手動で進める）

```python
def test_no_fire_before_first_feed():
    clk, calls = FakeClock(), []
    w = Watchdog(0.3, lambda: calls.append(1), clock=clk)
    clk.t = 10; assert w.check() is False and calls == []

def test_fires_after_timeout_once():
    clk, calls = FakeClock(), []
    w = Watchdog(0.3, lambda: calls.append(1), clock=clk)
    w.feed(); clk.t = 0.29; assert w.check() is False
    clk.t = 0.31; assert w.check() is True
    clk.t = 1.0;  assert w.check() is False
    assert calls == [1]

def test_feed_resets():
    clk, calls = FakeClock(), []
    w = Watchdog(0.3, lambda: calls.append(1), clock=clk)
    w.feed(); clk.t = 0.2; w.feed(); clk.t = 0.45
    assert w.check() is False

def test_thread_fires_with_real_clock():
    ev = threading.Event()
    w = Watchdog(0.05, ev.set); w.start(0.01); w.feed()
    assert ev.wait(1.0); w.shutdown()
```

- [ ] **Step 2: 失敗を確認** — Run: `pytest tests/test_watchdog.py -v` / Expected: FAIL
- [ ] **Step 3: `cugo_rc/watchdog.py` を実装**
- [ ] **Step 4: 通過を確認** — Run: `pytest tests/test_watchdog.py -v` / Expected: 4 passed
- [ ] **Step 5: Commit** — `git add cugo_rc/watchdog.py tests/test_watchdog.py && git commit -m "feat: command watchdog"`

---

### Task 4: カメラ

**Files:**
- Create: `cugo_rc/camera.py`
- Test: `tests/test_camera.py`

**Interfaces:**
- Consumes: `CameraConfig` (Task 1)
- Produces: `class Camera(cfg: CameraConfig, mock: bool = False, opener: Callable[[], Any] | None = None, retry_s: float = 3.0)`
  - `opener` 既定は `cv2.VideoCapture(cfg.device)` に幅・高さ・fps を設定して返す。テストで差し替える
  - `start()` / `shutdown()` — 取得 daemon スレッド。`read()` 失敗または `isOpened()` False なら `ok=False`、release して `retry_s` 後に再オープン
  - `ok: bool` — 直近フレーム取得成功か
  - `latest_jpeg() -> bytes | None` — 最新の JPEG（`cv2.imencode` quality=`jpeg_quality`）
  - `async def mjpeg(self) -> AsyncIterator[bytes]` — `b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"` を `1/fps` 間隔で yield。`latest_jpeg()` が None の間は yield せず待つ
  - mock=True: opener を使わず、灰色背景に時刻文字列を描いた `width×height` 画像を生成（`ok=True`）
  - 定数 `BOUNDARY = "frame"`

- [ ] **Step 1: 失敗するテストを書く**

```python
def test_mock_produces_jpeg():
    c = Camera(CameraConfig(width=160, height=120), mock=True); c.start()
    assert wait_until(lambda: c.latest_jpeg() is not None)
    assert c.latest_jpeg()[:2] == b"\xff\xd8" and c.ok
    c.shutdown()

def test_unavailable_camera_sets_not_ok_and_retries():
    opens = []
    class Dead:
        def isOpened(self): return False
        def read(self): return False, None
        def release(self): pass
        def set(self, *a): pass
    def opener(): opens.append(1); return Dead()
    c = Camera(CameraConfig(), opener=opener, retry_s=0.05); c.start()
    assert wait_until(lambda: len(opens) >= 2)
    assert c.ok is False and c.latest_jpeg() is None
    c.shutdown()

def test_mjpeg_chunk_format():
    c = Camera(CameraConfig(width=160, height=120), mock=True); c.start()
    chunk = asyncio.run(anext(c.mjpeg()))
    assert chunk.startswith(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n\xff\xd8")
    c.shutdown()
```

（`wait_until(pred, timeout=2.0)` はテスト内ヘルパ。tests/conftest.py に置き Task 5 でも使う）

- [ ] **Step 2: 失敗を確認** — Run: `pytest tests/test_camera.py -v` / Expected: FAIL
- [ ] **Step 3: `cugo_rc/camera.py` を実装**
- [ ] **Step 4: 通過を確認** — Run: `pytest tests/test_camera.py -v` / Expected: 3 passed
- [ ] **Step 5: Commit** — `git add cugo_rc/camera.py tests/test_camera.py && git commit -m "feat: camera capture and MJPEG stream"`

---

### Task 5: サーバー

**Files:**
- Create: `cugo_rc/server.py`, `cugo_rc/static/index.html`（Task 6 で中身を作る。ここでは `<title>CuGo RC</title>` だけの仮ファイル）
- Test: `tests/test_server.py`

**Interfaces:**
- Consumes: `Config`, `load_config` (T1); `MotorController`, `MockBackend`, `PigpioBackend`, `BackendError` (T2); `Watchdog` (T3); `Camera` (T4)
- Produces:
  - `create_app(cfg: Config, motor: MotorController, camera: Camera) -> FastAPI`
  - lifespan 開始: `camera.start()`、`Watchdog(cfg.watchdog.timeout_ms/1000, motor.stop).start()`、50Hz の asyncio タスクで `motor.tick(実測dt)`
  - lifespan 終了: 制御タスク停止 → `watchdog.shutdown()` → `motor.close()` → `camera.shutdown()`
  - `GET /` → `static/index.html`、`/static/*` 静的配信
  - `GET /video` → `StreamingResponse(camera.mjpeg(), media_type="multipart/x-mixed-replace; boundary=frame")`
  - `GET /status` → `{"camera": camera.ok, "driver_connected": bool}`
  - `WS /ws`:
    - 接続時、操縦者がいなければ driver。`{"role": "driver"}` または `{"role": "viewer"}` を送信
    - driver の `{"stop": true}` → `motor.stop()`
    - driver の `{"l": x, "r": y}` → `motor.set_target(x, y)` 成功時のみ `watchdog.feed()`。JSON不正・キー欠落・`ValueError` は無視して `logging.warning`
    - viewer のメッセージは無視
    - driver 切断（例外含む）→ `motor.stop()`、操縦権解放
  - `main(argv: list[str] | None = None) -> None` — `--config`（既定 `config.yaml`、存在しなければ既定値）、`--mock`（`MockBackend` と `Camera(mock=True)`）。`BackendError` はメッセージを stderr に出して `sys.exit(1)`。終了時用に `atexit.register(motor.stop)`。`uvicorn.run(app, host, port)`
  - `if __name__ == "__main__": main()`（`python -m cugo_rc.server`）

- [ ] **Step 1: 失敗するテストを書く**。fixture: `MockBackend` + `MotorController` + `Camera(mock=True)` で `create_app` し `TestClient(app)` を `with` で開く

```python
def test_index(client):
    assert "CuGo RC" in client.get("/").text

def test_driver_command_sets_target(client, motor):
    with client.websocket_connect("/ws") as ws:
        assert ws.receive_json() == {"role": "driver"}
        ws.send_json({"l": 0.5, "r": -0.5})
        assert wait_until(lambda: motor.target == (0.5, -0.5))

def test_second_connection_is_viewer_and_ignored(client, motor):
    with client.websocket_connect("/ws") as d:
        d.receive_json()
        with client.websocket_connect("/ws") as v:
            assert v.receive_json() == {"role": "viewer"}
            v.send_json({"l": 1, "r": 1})
            d.send_json({"l": 0.1, "r": 0.1})
            assert wait_until(lambda: motor.target == (0.1, 0.1))

def test_stop_message_is_immediate(client, motor, backend):
    with client.websocket_connect("/ws") as ws:
        ws.receive_json(); ws.send_json({"l": 1, "r": 1})
        wait_until(lambda: motor.current[0] > 0.1)
        ws.send_json({"stop": True})
        assert wait_until(lambda: backend.duty == {12: 0.0, 13: 0.0} and motor.target == (0, 0))

def test_disconnect_stops_and_releases(client, motor):
    with client.websocket_connect("/ws") as ws:
        ws.receive_json(); ws.send_json({"l": 1, "r": 1})
        wait_until(lambda: motor.target == (1, 1))
    assert wait_until(lambda: motor.target == (0, 0))
    with client.websocket_connect("/ws") as ws2:
        assert ws2.receive_json() == {"role": "driver"}

@pytest.mark.parametrize("bad", ['{"l": NaN, "r": 0}', '{"l": true, "r": 0}', '{"l": "1", "r": 0}', 'xx', '{"l": 1}'])
def test_invalid_messages_ignored(client, motor, bad):
    with client.websocket_connect("/ws") as ws:
        ws.receive_json(); ws.send_text(bad)
        ws.send_json({"l": 0.2, "r": 0.2})          # 接続が生きていることの確認
        assert wait_until(lambda: motor.target == (0.2, 0.2))

def test_watchdog_stops_when_commands_cease(client, motor):
    with client.websocket_connect("/ws") as ws:
        ws.receive_json(); ws.send_json({"l": 1, "r": 1})
        assert wait_until(lambda: motor.target == (0, 0), timeout=1.0)   # 300ms 後

def test_status_reports_camera_down(motor):
    cam = Camera(CameraConfig(), opener=dead_opener, retry_s=0.05)   # Task 4 の Dead と同等を conftest に
    with TestClient(create_app(default_config(), motor, cam)) as c:
        assert c.get("/status").json()["camera"] is False
        with c.websocket_connect("/ws") as ws:                         # カメラ無しでも操縦可
            assert ws.receive_json() == {"role": "driver"}

def test_shutdown_closes_motor(backend):
    with TestClient(app): pass
    assert backend.closed
```

注: Python の `json.loads` は `NaN` を受理するため、`set_target` 側の有限チェック（T2）で弾かれることを確認するテスト。

- [ ] **Step 2: 失敗を確認** — Run: `pytest tests/test_server.py -v` / Expected: FAIL
- [ ] **Step 3: `cugo_rc/server.py` を実装**
- [ ] **Step 4: 通過を確認** — Run: `pytest -v` / Expected: 全テスト passed
- [ ] **Step 5: mock 起動の手動確認** — Run: `python -m cugo_rc.server --mock`、`curl -s localhost:8000/status` / Expected: `{"camera":true,"driver_connected":false}`
- [ ] **Step 6: Commit** — `git add -A && git commit -m "feat: FastAPI server with websocket control and MJPEG"`

---

### Task 6: 操縦UI

**Files:**
- Modify: `cugo_rc/static/index.html`
- Create: `cugo_rc/static/app.js`, `cugo_rc/static/style.css`

**Interfaces:**
- Consumes: `WS /ws` プロトコル、`GET /video`、`GET /status` (Task 5)
- Produces: なし（エンドユーザー向け）

画面要素: 映像 `<img src="/video">`（全幅）、状態表示（接続中/切断/viewer、カメラ警告）、仮想ジョイスティック（右下、タッチ＋マウス対応、Pointer Events）、大きな赤い停止ボタン、現在の l/r 値表示、キー操作の説明。

`app.js` で決めること:
- `mix(fwd, turn) -> [l, r]`: `l = fwd + turn`, `r = fwd - turn`、`m = max(1, |l|, |r|)` で両方を割る
- WASD: W/S=前後 ±1、A/D=旋回 ±0.6（A が左旋回 = turn 負）、Shift 押下中は ×0.4。ジョイスティック操作中はジョイスティック優先
- ジョイスティック: Y上=前進+1、X右=旋回+1、半径で正規化、指を離すと 0
- 50ms 周期で `{l, r}` 送信（driver のときのみ、入力ゼロでも送信）
- Space または停止ボタン → `{"stop": true}` を送り、入力状態を 0 にリセット
- `visibilitychange` で hidden → `{"stop": true}` 送信して送信停止。visible で再開
- `window.blur` でキー押下状態を全解除（キーが押しっぱなし扱いになるのを防ぐ）
- WS 切断時は 1 秒後に自動再接続、状態表示を「切断」に
- 2秒ごとに `/status` を取得し `camera:false` なら「映像なし」警告
- スマホでスクロール/ズームしないよう `touch-action: none`、viewport meta 設定

- [ ] **Step 1: 3ファイルを実装**
- [ ] **Step 2: 既存テストが通ることを確認** — Run: `pytest -v` / Expected: 全 passed（`test_index` が `CuGo RC` を含むこと）
- [ ] **Step 3: mock で手動確認** — `python -m cugo_rc.server --mock` を起動し、PCブラウザで `http://localhost:8000`:
  - W 押下で表示 l/r が加速、離すと 0、Shift で 0.4 倍
  - ジョイスティックをドラッグして値が変わる
  - Space / 停止ボタンで 0
  - 別タブで開くと viewer 表示になる
  - タブ切替でサーバーログに stop、サーバー停止→再起動で自動再接続
- [ ] **Step 4: Commit** — `git add cugo_rc/static && git commit -m "feat: web control UI with joystick and WASD"`

---

### Task 7: 運用ファイルとドキュメント

**Files:**
- Create: `systemd/cugo-rc.service`, `README.md`

- [ ] **Step 1: `systemd/cugo-rc.service`**
  - `After=network-online.target pigpiod.service`, `Wants=pigpiod.service`
  - `WorkingDirectory=/home/pi/EasyRC_CRAWL`, `ExecStart=/home/pi/EasyRC_CRAWL/.venv/bin/python -m cugo_rc.server --config config.yaml`
  - `Restart=on-failure`, `RestartSec=2`, `User=pi`, `WantedBy=multi-user.target`
  - パス・ユーザーは環境に合わせて変更する旨をコメントで記載
- [ ] **Step 2: `README.md`**（日本語）
  - 配線表（spec §4）、GND共通化、PWM線 10kΩ プルダウン、物理非常停止の推奨
  - セットアップ: `sudo apt install pigpio python3-venv`、`sudo systemctl enable --now pigpiod`、venv 作成、`pip install -r requirements.txt`
  - 起動: 手動・`--mock`・systemd 登録手順（`sudo cp systemd/cugo-rc.service /etc/systemd/system/ && sudo systemctl enable --now cugo-rc`）
  - 操作方法（WASD/Shift/Space、ジョイスティック）
  - 自動停止の条件一覧（spec §7）
  - 実機チェックリスト（spec §9 の5項目）
  - 設定項目（`config.yaml`）と `invert` での向き調整
- [ ] **Step 3: 確認** — Run: `pytest -v` / Expected: 全 passed
- [ ] **Step 4: Commit** — `git add systemd README.md && git commit -m "docs: README and systemd unit"`
