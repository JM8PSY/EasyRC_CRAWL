# Jetson 自律移動システム 設計

- 日付: 2026-11-10
- 対象: CuGo V3（クローラー、左右 2 モーター、エンコーダなし）+ Cytron MD20A ×2 + ESP32 DevKit + Jetson Orin Nano Super + RealSense D455 + YDLIDAR X4 PRO
- 目的: Jetson を制御の主、ESP32 を足回りの管理役として、2D SLAM と自律移動（Nav2）を行う

## 実機環境（確認済み）

| 項目 | 値 |
|---|---|
| Jetson | Orin Nano Engineering Reference Developer Kit Super / JetPack 7.2.1 / Ubuntu 24.04 / kernel 6.8 tegra |
| ROS | ROS 2 **Jazzy**（/opt/ros/jazzy） |
| SSH | `ssh jetson@192.168.0.143`（鍵認証・sudo 可） |
| ESP32 | CP2102、`/dev/ttyUSB0`、by-path `platform-3610000.usb-usb-0:2.1:1.0-port0` |
| X4 PRO | CP2102、`/dev/ttyUSB1`、by-path `platform-3610000.usb-usb-0:2.4:1.0-port0`、128000bps |
| D455 | USB3（8086:0b5c）、ros-jazzy-realsense2-camera 4.58 |

ESP32 と X4 PRO は同じ USB シリアル番号（CP2102 / 0001）のため、**USB ポート位置（ID_PATH）で udev ルールを作り** `/dev/cugo_esp32` と `/dev/cugo_lidar` に固定する（`/dev/ydlidar` は同じ Jetson の別プロジェクトの udev ルールが ESP32 側のポートに割り当てているため使わない）。

## 全体構成

```
Jetson (ROS 2 Jazzy)
  realsense2_camera ── /camera/camera/imu, /camera/camera/depth/color/points
  ydlidar_ros2_driver ── /scan (laser_frame)
  rf2o_laser_odometry ── /odom_rf2o
  cugo_base ── /wheel_odom（指令ベースの推定）, /cugo/mode, /diagnostics
  robot_localization (ekf) ── /odometry/filtered, TF odom→base_footprint
  slam_toolbox ── /map, TF map→odom
  Nav2 ── /cmd_vel_nav
  twist_mux (/cmd_vel_teleop > /cmd_vel_nav) ── /cmd_vel ── cugo_base ── USB シリアル ── ESP32
ESP32
  安全（ウォッチドッグ・加速度制限・上限）、モード調停（Web 手動 > Jetson 自律）、MD20A へ PWM/DIR
```

## ESP32 ⇔ Jetson シリアルプロトコル（v1）

- 115200bps、8N1、改行 `\n` 区切りの ASCII 行
- 形式: `$<BODY>*<HH>\n`。`HH` は BODY（`$` と `*` の間）の全バイトの XOR を 2 桁大文字 16 進で表したもの
- `$` で始まらない行は人間向けログ（既存の日本語ログ）。受信側は無視する
- チェックサム不一致・形式不正の行は無視する（ESP32 は不正行数を数え、ステータスで報告）
- 数値は `.` 小数点の 10 進表記（例 `0.250`、`-0.100`）

### Jetson → ESP32

| 行 | 意味 |
|---|---|
| `$D,<l>,<r>*HH` | 左右の **絶対 duty**（-1.0〜1.0、前進が正。invert 適用前の論理値）。20〜50Hz で送り続ける（ハートビートを兼ねる） |
| `$S*HH` | 即時停止（加速度制限を無視して 0） |

例: `$D,0.250,-0.250*HH`

### ESP32 → Jetson（20Hz）

`$T,<mode>,<l>,<r>,<cap>,<flags>,<bad>*HH`

| フィールド | 内容 |
|---|---|
| mode | `I` 待機（有効な指令源なし）/ `A` 自律（Jetson の指令を出力中）/ `M` 手動（Web 操縦者が操縦中）/ `X` 動作テスト中 |
| l, r | 現在の出力 duty（加速度制限後、invert 適用前の論理値、小数 3 桁） |
| cap | 自律モードの duty 上限 `AUTO_MAX_DUTY`（小数 2 桁） |
| flags | ビットフラグ（10 進）: bit0 = 自律許可中、bit1 = Web 操縦者接続中、bit2 = Jetson 指令受信中（ウォッチドッグ内） |
| bad | 起動以降の不正行の数 |

### ESP32 のモード調停

優先順位: **動作テスト > Web 手動 > Jetson 自律**。

1. `autoAllowed`（自律許可）
   - Web 操縦者がいないとき: `true`（Jetson 単独で自律走行できる）
   - Web 操縦者が接続した時点: `false`（手動）。Web UI の「自律走行を許可」トグル（`{"auto":true}` / `{"auto":false}`）で切り替える
   - 自律許可中に操縦者が **0 でない** 走行指令（l または r の絶対値 > 0.05）を送ったら、即座に `false`（手動に奪還）
   - 停止ボタン（`{"stop":true}`）: 停止し、`false`
   - 操縦者が切断: 停止し、`true` に戻る
2. 出力の決定
   - 動作テスト中: テストの値
   - Web 操縦者がいて `autoAllowed == false`: Web の指令（Web の出力上限スライダーを掛ける。従来通り）
   - `autoAllowed == true` かつ Jetson 指令がウォッチドッグ内: Jetson の duty を `±AUTO_MAX_DUTY` にクランプして使う（**Web スライダーは掛けない**。Jetson 側の速度キャリブレーションを崩さないため）
   - それ以外: 0
3. ウォッチドッグ: Web 指令・Jetson 指令それぞれ 300ms。使用中の指令源が途絶えたら停止
4. 加速度制限（`MAX_ACCEL`）はすべてのモードで適用。`$S` と停止ボタンは即時 0
5. `AUTO_MAX_DUTY` 既定 0.5（config.h）

### Web UI の追加

- ヘッダーにモード表示（待機 / 自律 / 手動 / テスト）
- 「自律走行を許可」トグル（操縦者のみ操作可）
- ESP32 はモードまたは許可状態が変わったら全クライアントへ `{"mode":"I|A|M|X","auto":true|false}` を送る（接続時にも送る）

## Jetson 側パッケージ（jetson/src/）

| パッケージ | 種類 | 内容 |
|---|---|---|
| `cugo_base` | ament_python | ESP32 ブリッジノード `cugo_base_node`。プロトコル・運動学は ROS 非依存のモジュールに分けて pytest |
| `cugo_description` | ament_cmake | URDF（xacro）。base_footprint / base_link / laser_frame / camera_link。センサー取付位置は xacro 引数（既定値は仮、要実測） |
| `cugo_bringup` | ament_cmake | launch と設定 YAML（センサー、オドメトリ、EKF、SLAM、Nav2、twist_mux） |

外部ソース（`~/cugo_ws/src/third_party/`、リポジトリ外。`jetson/scripts/setup_jetson.sh` で取得）: YDLidar-SDK、ydlidar_ros2_driver、rf2o_laser_odometry。

### cugo_base_node

- 購読: `/cmd_vel`（`geometry_msgs/Twist`。パラメータ `use_stamped` が true なら `TwistStamped`）
- 配信: `/wheel_odom`（`nav_msgs/Odometry`、ESP32 が報告した出力 duty から速度を逆算して積分。TF は出さない。共分散は大きめ）、`/cugo/mode`（`std_msgs/String`、`idle|auto|manual|test|disconnected`）、`/diagnostics`
- 運動学: `v_l = v - ω·W/2`、`v_r = v + ω·W/2` → duty = `sign(v)·(deadband + |v|/max_wheel_speed·(1 - deadband))`（|v| が `min_speed` 未満なら 0）。`max_duty` でクランプ。片輪が上限を超える場合は左右の比を保って縮小
- 逆変換（odom 用）: `|duty| <= deadband` → 0、それ以外 `v = sign·(|duty| - deadband)/(1 - deadband)·max_wheel_speed`
- `/cmd_vel` が `cmd_timeout`（0.5s）途絶えたら 0 を送る。送信周期 `rate`（30Hz）で常に `$D` を送る（ハートビート）
- シリアル断で再接続を繰り返す（1s 間隔）
- パラメータ（既定値は仮、キャリブレーションで決める）: `port=/dev/cugo_esp32`、`baud=115200`、`track_width=0.40`、`max_wheel_speed=1.0`（duty 1.0 時の m/s）、`deadband=0.08`、`min_speed=0.01`、`max_duty=0.5`、`cmd_timeout=0.5`、`rate=30.0`、`odom_frame=odom`、`base_frame=base_footprint`、`use_stamped=false`

### TF とフレーム

`map → odom`（slam_toolbox / AMCL）、`odom → base_footprint`（EKF）、`base_footprint → base_link → laser_frame / camera_link`（robot_state_publisher）。RealSense の光学系・IMU フレームは realsense2_camera が `camera_link` 以下に出す（`publish_tf: true`）。

### オドメトリ（エンコーダなし）

- rf2o_laser_odometry: `/scan` → `/odom_rf2o`（TF は出さない）
- D455 IMU: `enable_gyro/enable_accel: true`、`unite_imu_method: 2` → `/camera/camera/imu`（角速度 z を EKF に使う）
- robot_localization ekf: `two_d_mode: true`、`publish_tf: true`、入力 = rf2o（x, y, yaw）、IMU（vyaw）、wheel_odom（vx, vyaw、大きい共分散）

### SLAM / ナビゲーション

- slam_toolbox `online_async`（解像度 0.05、`max_laser_range` 10m）
- Nav2: コントローラは Regulated Pure Pursuit、プランナーは NavFn。速度上限 0.3 m/s / 0.6 rad/s。コストマップは `/scan`（obstacle_layer）+ D455 点群（voxel_layer、床と天井を除外）。フットプリントは仮の矩形（要実測）
- Nav2 の出力は `/cmd_vel_nav` にリマップし、twist_mux 経由で `/cmd_vel`

### 起動

`ros2 launch cugo_bringup bringup.launch.py mode:=slam|nav map:=<yaml>`

- `slam`: 地図を作りながら Nav2 で移動できる
- `nav`: 保存した地図で AMCL + Nav2

## 安全

- 実機のモーターを動かすテストは、人が立ち会って行う。自動テストや開発中のエージェントは ESP32 に 0 以外の指令を送らない
- 速度上限は 3 段: Nav2 の速度制限 → cugo_base の `max_duty` → ESP32 の `AUTO_MAX_DUTY`
- ESP32 のウォッチドッグにより、Jetson・ROS・USB のどれが止まっても 300ms で停止する
