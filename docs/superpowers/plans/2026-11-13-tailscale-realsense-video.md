# Tailscale 接続・RealSense SDK ソースビルド・カメラ映像配信 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 自宅からでも Jetson に SSH と Web UI で入れるようにする。D455 の IMU を使えるようにする。Web UI で RealSense のカラー映像を見られるようにする。

**Architecture:**
- **Tailscale**
  - 公式 apt リポジトリから入れる。ログインはユーザーが行う。
  - Web UI の自己署名証明書に Tailscale の IP（100.x）と MagicDNS 名を含める。足りなければ作り直す。
- **librealsense**
  - RSUSB バックエンドと CUDA を有効にしてソースからビルドし、`/usr/local` に入れる。
  - realsense-ros もそれに合わせて `~/cugo_ws` でソースからビルドする。apt 版の librealsense と realsense2_camera は外す。
- **映像配信**
  - 送るもの：realsense2_camera が出す JPEG（`.../image_raw/compressed`）をそのまま使う。
  - 方式：cugo_web の aiohttp から MJPEG（`multipart/x-mixed-replace`）で送る。
  - 見る人がいる間だけトピックを購読する。

**Tech Stack:**
- 使うもの：ROS 2 Jazzy、librealsense 2.58.4、realsense-ros 4.58.4（実機の apt 版に合わせた）、aiohttp、image_transport_plugins、Tailscale。
- 対象機：Jetson Orin Nano（JetPack 6、カーネル 6.8 tegra）。

**Spec:** 別の設計書はありません。このファイルとユーザーの依頼（2026-11-13）が仕様です。
- 依頼の内容：「Tailscaleとの接続」「RealsenseのSDKビルド」「Realsenseの映像を伝送」の 3 つ。
- 依頼外として保留：振動の改善。後で行います。

## Global Constraints

- **ロボットを動かさない**：ユーザーが見ていないときに、モーターへ 0 以外の速度指令やナビのゴールを送りません。
  - 確認はカメラと IMU を読むだけにします。
  - `use_base:=false` で起動するか、cugo サービスを止めた状態で行います。
- **sudo はユーザーが実行する**：パスワードをチャットで求めません。
  - 対象：`tailscale up`、`apt remove`、udev ルールの追加、`systemctl restart cugo` など。
- **触らないもの**：`/dev/ydlidar` を開きません。`99-robot.rules`、`~/2026hairo`、third_party のドライバコードも変更しません。
- **外部ソース**：このファイルでユーザーの承認を得てからビルドします。リビジョンはスクリプトに固定します（`setup_jetson.sh` と同じ方式）。
- **公開範囲**：Tailscale Funnel は使いません。Web UI は tailnet 内からだけ見えるようにします（Web UI には認証がないため）。
- **GitHub**：push はユーザーの指示があるときだけ行います。
- **テストの実行**：pytest はパッケージのディレクトリから実行します（`cd jetson/src/cugo_web && python3 -m pytest -q`）。
- **配信の既定値**
  - 送るフレームの上限は `video_fps` で、既定値は 10 です。
  - トピックは `video_topic` で、既定値は `/camera/camera/color/image_raw/compressed` です。

## Review Focus

1. 映像を見ているタブを閉じたときは、配信処理が終わり、視聴者数が 0 に戻ります。数秒以内に購読もやめます。CPU を無駄に使い続けません。（Task 3 でテスト）
2. カメラが止まっていてフレームが来ないときも、Web UI と他の機能は落ちません。画面に「映像なし」と出ます。（Task 3 でテスト）
3. 2 台以上の端末で同時に見たときは、それぞれに映像が届きます。1 台が切れても他は止まりません。（Task 3 でテスト）
4. Tailscale を入れた後の初回起動では、証明書を 1 回だけ作り直します。それ以降の起動では作り直しません。（Task 1 でテスト）
5. Tailscale が入っていないか止まっているときも、証明書づくりと Web UI はこれまで通り動きます。（Task 1 でテスト）

---

## 作業順（Jetson 側）

1. **疎通確認**：Jetson の電源を入れ、Mac から研究室のネットワーク経由で SSH が通ることを確かめる。
2. **Task 1**：Tailscale を入れる。これで自宅からも作業を続けられるようになる。
3. **Task 2**：librealsense のビルドを裏で始める。Orin Nano では 40〜90 分かかる見込み。
4. ビルドを待つ間に、Task 3 の映像配信のコードをエージェントが Mac で書いてテストする。
5. Task 2 の後半：realsense-ros をビルドし、IMU を確認する。
6. Task 3 を Jetson に送って確認する。

Task 1 と Task 3 は、コードの上では互いに独立しています。

---

### Task 1: Tailscale の導入と、証明書に Tailscale のアドレスを含める

**Files:**
- Create: `jetson/scripts/setup_tailscale.sh`
- Modify: `jetson/src/cugo_web/cugo_web/tls.py`
- Test: `jetson/src/cugo_web/test/test_tls.py`
- Modify: `jetson/README.md`（「Tailscale で入る」節を追加）

**Interfaces:**
- Produces（`tls.py`）:
  - `cert_names(crt_path: str) -> set[str]`
    - 証明書の SAN を `{'IP:1.2.3.4', 'DNS:host', ...}` の形で返す。
    - `openssl x509 -noout -ext subjectAltName` で読む。
    - 読めないときは空の集合を返す。
  - `wanted_names(host: str, ips: list[str], ts_dns: str | None) -> set[str]`
    - 証明書に載せたい名前の集合。
    - 中身：今の SAN の作り方と同じもの（ホスト名、`<host>.local`、localhost、127.0.0.1、各 IP）に、`ts_dns` を加える。
  - `_tailscale_dns() -> str | None`
    - `tailscale status --json` の `Self.DNSName` を、末尾の `.` を取り除いて返す。
    - コマンドがない、エラーになる、時間切れのときは None を返す。
  - `ensure_cert(cert_dir=CERT_DIR)`
    - 戻り値は今と同じ `(crt, key)`。
    - 既存の証明書が `wanted_names` を全部含んでいなければ作り直す。作り直したらログ用に印を返す必要はない。

- [ ] **Step 1: 失敗するテストを書く**（`test_tls.py`。`openssl` は Mac にも Jetson にもある）

```python
def test_cert_contains_wanted_names(tmp_path, monkeypatch):
    monkeypatch.setattr(tls, '_local_ips', lambda: ['10.0.0.5', '100.64.1.2'])
    monkeypatch.setattr(tls, '_tailscale_dns', lambda: 'cugo.tail1234.ts.net')
    crt, _ = tls.ensure_cert(str(tmp_path))
    names = tls.cert_names(crt)
    assert {'IP:100.64.1.2', 'IP:10.0.0.5', 'DNS:cugo.tail1234.ts.net'} <= names

def test_cert_regenerated_once_when_tailscale_ip_appears(tmp_path, monkeypatch):
    monkeypatch.setattr(tls, '_local_ips', lambda: ['10.0.0.5'])
    monkeypatch.setattr(tls, '_tailscale_dns', lambda: None)
    crt, _ = tls.ensure_cert(str(tmp_path)); m0 = os.path.getmtime(crt)
    monkeypatch.setattr(tls, '_local_ips', lambda: ['10.0.0.5', '100.64.1.2'])
    os.utime(crt, (m0 - 10, m0 - 10))
    tls.ensure_cert(str(tmp_path)); m1 = os.path.getmtime(crt)
    assert m1 > m0 - 10 and 'IP:100.64.1.2' in tls.cert_names(crt)
    tls.ensure_cert(str(tmp_path))
    assert os.path.getmtime(crt) == m1          # 2 回目は作り直さない

def test_no_tailscale_still_works(tmp_path, monkeypatch):
    monkeypatch.setattr(tls, '_local_ips', lambda: ['10.0.0.5'])
    monkeypatch.setattr(tls.subprocess, 'run', _fail_only_tailscale(tls.subprocess.run))
    crt, key = tls.ensure_cert(str(tmp_path))
    assert os.path.exists(crt) and tls._tailscale_dns() is None

def test_cert_names_unreadable_is_empty(tmp_path):
    p = tmp_path / 'x.crt'; p.write_text('garbage')
    assert tls.cert_names(str(p)) == set()
```

`_fail_only_tailscale(run)` はテストファイル内の小さな関数です。引数の先頭が `tailscale` のときだけ `FileNotFoundError` を出し、それ以外は元の `run` を呼びます。

- [ ] **Step 2: テストを実行し、失敗することを確かめる**
  - 実行：`cd jetson/src/cugo_web && python3 -m pytest test/test_tls.py -q`
  - 期待：`cert_names` がないため FAIL。

- [ ] **Step 3: `tls.py` に上の Interfaces の関数を実装する**
  - 作り直すときは、古い crt と key を上書きします。
  - 作り直したことを示すため、ログ用に標準エラーへ 1 行出します。

- [ ] **Step 4: テストを実行し、通ることを確かめる**
  - 実行：`python3 -m pytest -q`（cugo_web 全体。test_geometry は Mac では PIL がないので除外してよい）
  - 期待：すべて PASS。

- [ ] **Step 5: `setup_tailscale.sh` を書く**
  - 使い方：`bash setup_tailscale.sh`。中でユーザーが sudo する。
  - インストール：`curl -fsSL https://tailscale.com/install.sh | sh`。内容は公式の apt リポジトリ追加なので、実行前にユーザーへスクリプトの URL を示す。
  - ログイン：`sudo tailscale up --hostname=cugo-jetson --ssh=false`。表示されたログイン URL をユーザーがブラウザで開く。
  - 最後に表示するもの：`tailscale ip -4` と `tailscale status`。
  - あわせて注意書きを echo する：「管理画面で Disable key expiry を押すと、180 日ごとの再ログインが要らなくなる」。

- [ ] **Step 6: Jetson で確かめる**（ユーザーと一緒に行う）
  1. Jetson でユーザーがスクリプトを実行し、ログインする。
  2. Mac にも Tailscale を入れ、同じアカウントでログインする（ユーザーが行う）。
  3. Mac で `ssh jetson@<100.x>` を実行する。期待：`hostname` が `thisisjetson` を返す。
  4. ユーザーが `sudo systemctl restart cugo` でサービスを再起動する。
  5. Mac で `curl -sk https://<100.x>:8443/ -o /dev/null -w '%{http_code}'` を実行する。期待：`200`。
  6. Mac で `openssl s_client -connect <100.x>:8443 </dev/null 2>/dev/null | openssl x509 -noout -ext subjectAltName` を実行する。期待：`IP Address:100.x` が含まれる。
  7. ブラウザでは、証明書が変わったので一度だけ「詳細設定 → 進む」を押す必要がある。README にも書く。

- [ ] **Step 7: コミットする**

```bash
git add jetson/scripts/setup_tailscale.sh jetson/src/cugo_web/cugo_web/tls.py jetson/src/cugo_web/test/test_tls.py jetson/README.md
git commit -m "feat(remote): Tailscale setup and include tailnet names in the web cert"
```

---

### Task 2: librealsense（RSUSB と CUDA）と realsense-ros をソースからビルドする

ねらい：D455 の IMU を `/camera/camera/imu` に出し、EKF で使う。いまはカーネルに HID センサーのドライバがなく、apt 版では IMU が出ない（README「既知の問題」）。

**Files:**
- Create: `jetson/scripts/build_realsense.sh`
- Modify: `jetson/scripts/setup_jetson.sh`
  - apt の一覧から `ros-jazzy-realsense2-camera` を外す。
  - `ros-jazzy-image-transport-plugins` を加える（Task 3 で使う）。
  - 最後に「RealSense は build_realsense.sh で入れる」と案内する。
- Modify: `jetson/README.md`
  - 「既知の問題」の IMU の項目を、解決方法に書き換える。
  - 戻し方を書く。

**Interfaces:**
- Produces：
  - `/usr/local/lib/librealsense2.so.2.58`
  - `~/cugo_ws/src/third_party/realsense-ros`
  - `/camera/camera/imu`（sensor_msgs/Imu、約 200 Hz）
  - `/camera/camera/color/image_raw/compressed`（Task 3 で使う）
- 決めること：`LIBRS_REV` と `RSROS_REV` は、Step 1 で調べた今の apt 版に合わせる。
  - 例：apt が 2.58.4 なら、`LIBRS_REV=v2.58.4` と、それに対応する realsense-ros `4.58.4`。
  - スクリプトにはタグ名で固定する。

- [ ] **Step 1: 今の状態を記録する**（読み取りだけ）
  - 実行：
    - `dpkg -l | grep -i realsense`
    - `ros2 pkg prefix realsense2_camera`
    - `free -h; swapon --show; df -h ~`
    - `nvcc --version || ls /usr/local/cuda/bin`
  - 期待：apt 版の librealsense のバージョンがわかる。空きディスクが 5 GB 以上あり、swap が 4 GB 以上ある。
  - 足りない場合は、swap を一時的に増やすようユーザーに頼む（sudo が必要）。

- [ ] **Step 2: `build_realsense.sh` を書く**
  - `setup_jetson.sh` と同じ書き方にする（`set -euo pipefail`、`step` と `clone_at` の関数、固定リビジョン）。
  - 前半：librealsense
    - 取得先：`https://github.com/IntelRealSense/librealsense.git`、`$LIBRS_REV`。置き場所は `~/third_party_build/librealsense`。
    - cmake の設定：`-DFORCE_RSUSB_BACKEND=ON -DBUILD_WITH_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=87 -DBUILD_EXAMPLES=ON -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_PYTHON_BINDINGS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local`
    - ビルドとインストール：`make -j4`（6 だとメモリが足りないことがある）、`sudo make install`、`sudo ldconfig`。
    - udev：`config/99-realsense-libusb.rules` を `/etc/udev/rules.d/` にコピーし、reload と trigger をする。別ファイルなので `99-robot.rules` には触らない。
  - 後半：realsense-ros
    - 取得先：`https://github.com/IntelRealSense/realsense-ros.git`、`$RSROS_REV`。置き場所は `~/cugo_ws/src/third_party/realsense-ros`。
    - ビルド：`colcon build --symlink-install --packages-up-to realsense2_camera --cmake-args -DCMAKE_BUILD_TYPE=Release`
  - apt 版は自動では外さない。外す必要があれば、`sudo apt remove ros-jazzy-realsense2-camera ros-jazzy-librealsense2 ...` のコマンドを表示して止まる。実行するのはユーザー。
  - 各段階は、すでに入っていれば飛ばす（何度実行しても同じ結果になるように）。

- [ ] **Step 3: Jetson でビルドする**
  1. cugo サービスを止める。カメラを掴んだままだと確認できないため（ユーザーが `sudo systemctl stop cugo`）。
  2. `nohup bash build_realsense.sh > ~/build_realsense.log 2>&1 &` で裏で実行する。sudo の部分は、ユーザーが同じスクリプトをターミナルで実行し、パスワードを打つ形にする。
  3. ログを見て、完了まで待つ。
  - 期待：最後に `rs-enumerate-devices -s` で D455 が見え、`ldd $(ros2 pkg prefix realsense2_camera)/lib/realsense2_camera/realsense2_camera_node | grep realsense2` が `/usr/local/lib` を指す。

- [ ] **Step 4: IMU を確かめる**（読み取りだけ。モーターにはつながない）
  - 実行：
    - `rs-enumerate-devices | grep -E 'Accel|Gyro'`
    - `ros2 launch cugo_bringup bringup.launch.py use_base:=false use_nav:=false &`
    - `ros2 topic hz /camera/camera/imu`
    - `ros2 topic hz /camera/camera/depth/color/points`
    - `ros2 topic list | grep compressed`
    - `ros2 node list | grep cliff`（段差検出が今までどおり点群を受け取っているか）
  - 期待：
    - Accel と Gyro が表示される。
    - IMU が約 200 Hz、点群が 7 Hz 以上。
    - `/camera/camera/color/image_raw/compressed` がある。
    - ログに `No HID info provided` が出ない。
  - 点群フィルタのパラメータ名が変わっている場合（`pointcloud__neon_` と `pointcloud`）：`ros2 param list /camera/camera | grep pointcloud` で調べ、`sensors.launch.py` を直す。

- [ ] **Step 5: EKF で IMU が使われていることを確かめる**
  - 実行：`ros2 topic echo /odometry/filtered --once` と、`ros2 node info /ekf_filter_node | grep imu`
  - 期待：`/camera/camera/imu` を購読している。ロボットは静止させておき、角速度 z が 0 付近に落ち着いている。
  - IMU が使えるようになったことで EKF の設定を変える場合は、別の作業とする。ここでは「動く」ことの確認まで。

- [ ] **Step 6: 戻し方を README に書き、コミットする**
  - 戻し方の中身：`sudo apt install ros-jazzy-realsense2-camera` を実行し、`~/cugo_ws/src/third_party/realsense-ros` を削除して、ビルドし直す。

```bash
git add jetson/scripts/build_realsense.sh jetson/scripts/setup_jetson.sh jetson/README.md jetson/src/cugo_bringup/launch/sensors.launch.py
git commit -m "build(realsense): build librealsense with RSUSB+CUDA from source to enable the D455 IMU"
```

---

### Task 3: RealSense のカラー映像を Web UI に MJPEG で配信する

**Files:**
- Create: `jetson/src/cugo_web/cugo_web/video.py`（ROS に依存しない部分）
- Modify: `jetson/src/cugo_web/cugo_web/node.py`
  - パラメータ、必要なときだけの購読、ルート `/video.mjpg` を加える。
- Modify: `jetson/src/cugo_web/web/index.html`, `web/app.js`, `web/style.css`
  - 「カメラ」ボタンと映像パネルを加える。
- Modify: `jetson/src/cugo_web/package.xml`
  - `<exec_depend>image_transport_plugins</exec_depend>` を加える。
- Test: `jetson/src/cugo_web/test/test_video.py`

**Interfaces:**
- Produces（`video.py`）:
  - `BOUNDARY = 'frame'`
  - `mjpeg_part(jpeg: bytes) -> bytes`
    - `b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: <n>\r\n\r\n' + jpeg + b'\r\n'` を返す。
  - `class FrameHub`（スレッドセーフ。ROS のコールバックから `put`、asyncio から `wait_newer` を呼ぶ）
    - `put(jpeg: bytes) -> None`：最新の 1 枚と版番号を更新し、待っている側を起こす。asyncio のループには `loop.call_soon_threadsafe` で通知する。
    - `async wait_newer(version: int, timeout: float) -> tuple[int, bytes] | None`：版番号が `version` より新しいフレームを返す。時間切れなら None。
    - `viewers: int`（読み取り専用）、`viewer() -> contextmanager`：視聴者数を増やし、終わったら必ず減らす。
    - `bind(loop: asyncio.AbstractEventLoop) -> None`
  - `make_mjpeg_handler(hub: FrameHub, fps: float, idle_timeout: float = 5.0) -> Callable[[web.Request], Awaitable[web.StreamResponse]]`
    - `multipart/x-mixed-replace; boundary=frame` で送り続ける。
    - 送る間隔は `1/fps` 以上空ける。
    - `idle_timeout` の間フレームが来なければ、何も送らずに待ち続ける。
    - 接続が切れたら（`ConnectionResetError` か `asyncio.CancelledError`）静かに終わる。
    - 応答ヘッダに `Cache-Control: no-cache` を付ける。
- Consumes（node.py 側）:
  - トピック：`video_topic`（`sensor_msgs/CompressedImage`、`qos_profile_sensor_data`）。`msg.format` に `jpeg` を含まない場合は捨てる。
  - パラメータ：`video_topic`（既定値 `/camera/camera/color/image_raw/compressed`）、`video_fps`（既定値 10.0）。

- [ ] **Step 1: 失敗するテストを書く**（`test_video.py`。`aiohttp.test_utils` の `TestServer` と `TestClient` を `asyncio.run` で使う。プラグインは不要）

```python
def test_mjpeg_part_format():
    p = mjpeg_part(b'\xff\xd8abc')
    assert p.startswith(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 5\r\n\r\n')
    assert p.endswith(b'\xff\xd8abc\r\n')

def test_wait_newer_returns_frame_put_from_other_thread():
    async def main():
        hub = FrameHub(); hub.bind(asyncio.get_running_loop())
        threading.Timer(0.05, hub.put, args=(b'J1',)).start()
        return await hub.wait_newer(0, timeout=1.0)
    v, data = asyncio.run(main())
    assert v == 1 and data == b'J1'

def test_wait_newer_times_out_without_frames():          # Review Focus 2
    async def main():
        hub = FrameHub(); hub.bind(asyncio.get_running_loop())
        return await hub.wait_newer(0, timeout=0.1)
    assert asyncio.run(main()) is None

def test_stream_sends_parts_and_viewer_count_returns_to_zero():   # Review Focus 1
    # TestServer で handler を立て、クライアントが 2 パートを読んだら切断する
    # → 0.5 秒以内に hub.viewers == 0
    ...

def test_two_viewers_both_receive_and_one_leaving_keeps_other():  # Review Focus 3
    ...

def test_fps_limit():
    # put を 100 Hz で 1 秒間行い、fps=5 のストリームで受け取ったパート数が 4〜6 に収まる
    ...
```

`...` の 3 つは、上の 1 行のシナリオとアサーションをそのまま書けば決まります。

- [ ] **Step 2: テストを実行し、失敗することを確かめる**
  - 実行：`cd jetson/src/cugo_web && python3 -m pytest test/test_video.py -q`
  - 期待：`cugo_web.video` がないため FAIL。

- [ ] **Step 3: `video.py` を実装する**
  - Interfaces の通りに書きます。
  - `FrameHub` は `threading.Lock` で最新のフレームを守ります。
  - 待つ側は、`bind` したループ上の `asyncio.Condition` を、`put` から `call_soon_threadsafe` で通知して起こします。

- [ ] **Step 4: テストを実行し、通ることを確かめる**
  - 実行：`python3 -m pytest -q`
  - 期待：すべて PASS。

- [ ] **Step 5: `node.py` につなぐ**
  - **ノード側**
    - パラメータ `video_topic` と `video_fps` を追加する。
    - `self.video = FrameHub()` を持たせる。
    - 1 Hz のタイマーで購読を付け外しする：`self.video.viewers > 0` で購読していなければ `create_subscription`、0 になったら `destroy_subscription`。ROS のスレッドの中で行うので安全。
    - コールバックは、`'jpeg' in msg.format` のときだけ `self.video.put(bytes(msg.data))` を呼ぶ。
  - **WebServer 側**
    - `run()` で `self.node.video.bind(self.loop)` を呼ぶ。
    - `app.router.add_get('/video.mjpg', make_mjpeg_handler(self.node.video, self.node.video_fps))` を加える。`add_static` より前に書く。
  - **確認**
    - 実行：`python3 -m pytest -q` と `python3 -m flake8 --max-line-length 100 cugo_web`
    - 期待：PASS。flake8 の指摘なし。

- [ ] **Step 6: Web UI にカメラのパネルを追加する**
  - ボタンとパネル
    - index.html のツールバーに `<button id="camBtn">カメラ</button>` を置く。
    - 映像は、地図の右上に重ねる `<div id="camPanel" hidden><img id="cam" alt=""><span id="camMsg"></span></div>` で出す。スマホでは幅 100%。
  - 表示の切り替え（app.js）
    - 表示するとき：`cam.src = '/video.mjpg?t=' + Date.now()` を設定する。
    - 隠すとき：`cam.removeAttribute('src')` で接続を切る。
  - 「映像なし」の表示（Review Focus 2）
    - 表示してから 5 秒以内に `cam.onload` が来なければ、`camMsg` に「映像なし（カメラが動いていない可能性）」と出す。
    - `onload` が来たら消す。

- [ ] **Step 7: Jetson で確かめる**（読み取りだけ）
  1. Jetson にコードを送ってビルドする（rsync と `colcon build --packages-select cugo_web`）。
  2. ユーザーにサービスを再起動してもらう。
  3. 映像の流れを確かめる：Mac から Tailscale 経由で `curl -sk --max-time 3 https://<100.x>:8443/video.mjpg | head -c 200` を実行する。期待：`--frame` と `Content-Type: image/jpeg` が見える。
  4. ブラウザで確かめる：http と https の両方でカメラボタンを押し、映像が出ることを確かめる。
  5. 負荷を確かめる：映像を出している間の `top` で、cugo_web の CPU が 15% 以下であること。
  6. 後片付けを確かめる：タブを閉じて 2 秒後に `ros2 topic info -v /camera/camera/color/image_raw/compressed` を実行し、cugo_web の購読がないこと。

- [ ] **Step 8: コミットする**

```bash
git add jetson/src/cugo_web
git commit -m "feat(cugo_web): stream RealSense color video as MJPEG"
```

---

## この計画に含めないこと

- **振動の改善**：ユーザーの指示で後回しにします。
- **深度映像の配信**：今回はカラーだけです。必要になったら、`video_topic` を変えるか、ルートを追加します。
- **WebRTC による低遅延配信**：Tailscale 経由で MJPEG の遅延が問題になったら検討します。
- **IMU を使った EKF の設定見直し**：Task 2 は「IMU が出て、EKF が受け取る」ところまでです。
