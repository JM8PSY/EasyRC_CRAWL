# Jetson 自律移動（ROS 2 Jazzy）

CuGo V3 を Jetson Orin Nano Super で 2D SLAM と Nav2 で動かすためのパッケージ群です。
設計は `docs/superpowers/specs/2026-11-10-jetson-autonomy-design.md` を参照。

| パッケージ | 内容 |
|---|---|
| `cugo_base` | ESP32 ブリッジ（/cmd_vel → ESP32、/wheel_odom、/cugo/mode） |
| `cugo_description` | URDF（base_footprint → base_link → laser_frame / camera_link） |
| `cugo_bringup` | launch と設定（センサー、rf2o + EKF、slam_toolbox、Nav2、twist_mux、RViz） |

```
/scan ─┬─ rf2o ── /odom_rf2o ─┐
       │  D455 /camera/camera/imu ─┼─ EKF ── /odometry/filtered, TF odom→base_footprint
       │  cugo_base /wheel_odom ───┘
       ├─ slam_toolbox（slam）または AMCL（nav）── /map, TF map→odom
       └─ Nav2 ─ /cmd_vel_nav → velocity_smoother → collision_monitor ─ /cmd_vel_nav_safe ─┐
teleop_twist_keyboard ─ /cmd_vel_teleop（優先）──────────────────────────────── twist_mux ─ /cmd_vel ─ cugo_base ─ ESP32
```

## 1. セットアップ（初回のみ）

1. ROS 2 パッケージ（apt）

   ```bash
   sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
     ros-jazzy-robot-localization ros-jazzy-realsense2-camera ros-jazzy-xacro ros-jazzy-rviz2 \
     ros-jazzy-twist-mux ros-jazzy-teleop-twist-keyboard ros-jazzy-robot-state-publisher
   ```

2. 外部ソース（YDLidar-SDK、ydlidar_ros2_driver、rf2o_laser_odometry）を `~/cugo_ws` にビルドし、
   udev ルールを入れる: `~/EasyRC_CRAWL/jetson/scripts/setup_jetson.sh`

3. シリアルポート（USB の差し込み位置で固定。差し替えたら作り直し）

   | デバイス | パス |
   |---|---|
   | YDLIDAR X4 PRO | `/dev/cugo_lidar` |
   | ESP32 | `/dev/cugo_esp32` |

   **注意:** この Jetson では別プロジェクトの udev ルールにより `/dev/ydlidar` が **ESP32** を指しています。
   LiDAR には `/dev/ydlidar` を使わないこと（設定済みの既定値は `/dev/cugo_lidar`）。

4. RealSense の IMU（後述「既知の問題」）

## 2. ビルド

```bash
mkdir -p ~/ws/src && cd ~/ws/src
ln -s ~/EasyRC_CRAWL/jetson/src/cugo_base ~/EasyRC_CRAWL/jetson/src/cugo_description ~/EasyRC_CRAWL/jetson/src/cugo_bringup .
cd ~/ws
source /opt/ros/jazzy/setup.bash
source ~/cugo_ws/install/setup.bash     # ydlidar_ros2_driver, rf2o_laser_odometry
colcon build --symlink-install
source install/setup.bash
```

新しい端末では毎回 `source ~/ws/install/setup.bash`（`~/.bashrc` に書いておくと楽）。

## 3. 起動

### Web UI（おすすめ）

bringup を起動すると Web UI も立ち上がる（`use_web:=false` で無効）。同じ LAN のスマホ・PC のブラウザで
`http://<JetsonのIP>:8080/` を開く。

- ジョイスティック（PC は WASD）で手動操縦（`/cmd_vel_teleop`、Nav2 より優先。離して 0.5 秒で Nav2 に戻る）
- 地図・ロボット・LiDAR の点・経路の表示（ピンチ / ホイールで拡大、ドラッグで移動）
- 「目的地」→ 地図をタップ（ドラッグで向き）で Nav2 のゴール、「中止」で取り消し
- 「地図保存」で `~/maps/<名前>.yaml`（mode:=slam のとき）
- 停止ボタン / Space: 0 を送り、ゴールも取り消す。タブ切替・切断でも止まる

### 地図を作る（mode:=slam）

```bash
ros2 launch cugo_bringup bringup.launch.py mode:=slam use_rviz:=true
```

- 手で走らせて地図を作るなら Nav2 は不要: `use_nav:=false`
- 別端末で手動操縦（下の「手動操縦」）しながら部屋を一周する。ゆっくり、旋回は特にゆっくり
- RViz の「Nav2 Goal」で目的地を与えると、作りかけの地図上で自律移動もできる

### 地図を保存する

どちらか（両方でもよい）。

```bash
mkdir -p ~/maps
# Nav2 / AMCL 用（.yaml + .pgm）
ros2 run nav2_map_server map_saver_cli -f ~/maps/room
# slam_toolbox 用（.posegraph + .data。あとで地図の続きを作れる）
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/maps/room'}"
```

### 保存した地図で自律移動（mode:=nav）

```bash
ros2 launch cugo_bringup bringup.launch.py mode:=nav map:=$HOME/maps/room.yaml use_rviz:=true
```

1. RViz の「2D Pose Estimate」でロボットの現在位置と向きを地図上に置く（AMCL の初期位置）
2. ロボットを少し手動で回すと位置が収束する
3. 「Nav2 Goal」で目的地を指定

### 手動操縦（teleop）

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop
```

- `/cmd_vel_teleop` は Nav2 より優先。キーを離して 0.5 秒経つと Nav2 の指令に戻る
- Nav2 だけ一時停止: `ros2 topic pub -r 5 /pause_navigation std_msgs/msg/Bool "data: true"`（止めている間 teleop は効く）
- ESP32 の Web UI で操縦者が接続すると、ESP32 側で Web 手動が優先される（「自律走行を許可」で自律に戻す）

### 個別の launch（確認用）

| launch | 内容 |
|---|---|
| `cugo_description description.launch.py` | robot_state_publisher |
| `cugo_bringup sensors.launch.py` | D455 + X4 PRO（`use_camera:=false` / `use_lidar:=false` で片方だけ） |
| `cugo_bringup odometry.launch.py` | rf2o + EKF（description が必要） |
| `cugo_bringup slam.launch.py` | slam_toolbox |
| `cugo_bringup localization.launch.py map:=...` | map_server + AMCL |
| `cugo_bringup navigation.launch.py` | Nav2 |
| `cugo_bringup twist_mux.launch.py` | twist_mux |

`bringup.launch.py` の引数: `mode:=slam|nav`、`map:=`、`use_rviz:=false`、`use_base:=true`（false で ESP32 につながない）、
`use_nav:=true`、`base_port:=/dev/cugo_esp32`、`lidar_port:=/dev/cugo_lidar`。

### 動作確認コマンド

```bash
ros2 topic hz /scan                                  # 約 10Hz
ros2 topic hz /camera/camera/depth/color/points      # 約 15Hz
ros2 topic hz /camera/camera/imu                     # IMU が使える場合 200Hz 前後
ros2 topic hz /odometry/filtered                     # 30Hz
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /cugo/mode
```

## 4. メッセージ型（Twist / TwistStamped）

すべて `geometry_msgs/msg/Twist` で揃えています。

- Jazzy の Nav2: `enable_stamped_cmd_vel` の既定は false（Twist）。`nav2_params.yaml` で明示的に false
- twist_mux 4.x: `use_stamped` の既定が **true** なので `twist_mux.yaml` で false にしている
- teleop_twist_keyboard: `stamped` の既定 false（Twist）
- cugo_base: `use_stamped: false`（bringup からも false を渡す）

将来 TwistStamped に切り替えるときは 4 か所すべてを同時に変えること。

## 5. 実測が必要な値（要実測）

今は仮の値です。実測して書き換えてください。

| 値 | 場所 | 仮の値 |
|---|---|---|
| 車体の長さ・幅・高さ・最低地上高 | `cugo_description/urdf/cugo.urdf.xacro` の `body_*` | 0.50 / 0.45 / 0.20 / 0.03 m |
| LiDAR の位置（地面・車体中心基準）と向き | 同 `lidar_x/y/z/yaw` | (0, 0, 0.28) m、yaw 0 |
| カメラ（D455 の camera_link）の位置と角度 | 同 `camera_x/y/z/roll/pitch/yaw` | (0.24, 0, 0.25) m、角度 0 |
| フットプリント（車体外形） | `cugo_bringup/config/nav2_params.yaml` の `footprint`（local / global の 2 か所） | 0.54 x 0.48 m の矩形 |
| 点群の床・天井除外の高さ | 同 voxel_layer の `min/max_obstacle_height` | 0.05 / 0.5 m |
| cugo_base のキャリブレーション（track_width、max_wheel_speed、deadband） | `cugo_base/config/cugo_base.yaml`（`cugo_calibrate`） | 0.40 / 1.0 / 0.08 |

測り方のメモ:

- 位置はすべて **base_footprint（車体中心の真下の地面）** から。x 前方、y 左、z 上 [m]
- LiDAR の z はスキャン面（回転部の中ほど）の高さ。yaw は取付向きで決まるので、
  前方だけに壁（または箱）がある状態で RViz を見て、`/scan` の点が車体の前に出るように `lidar_yaw` を合わせる
  （左右が鏡写しになる場合は `ydlidar_x4pro.yaml` の `reversion` / `inverted` を見直す）
- camera_link は D455 の左 IR カメラ（正面から見て右側のレンズの近く、筐体中心から約 47.5mm）。
  下向きに付けたら `camera_pitch` を正の値（rad）
- xacro 引数は launch から上書きもできる:
  `ros2 launch cugo_description description.launch.py xacro_args:="lidar_z:=0.30 camera_pitch:=0.1"`
  （bringup で使う値は xacro の既定値を書き換えるのが確実）
- カメラの位置と角度を入れたら、何もない床を前にして RViz の `D455 PointCloud` と Local Costmap を見て、
  床が障害物として塗られないことを確認（塗られる場合は pitch を見直す / `min_obstacle_height` を上げる）

## 6. 初めて走らせるときのチェックリスト（必ず人が立ち会う）

準備

- [ ] ロボットをジャッキアップ（クローラーを床から浮かせる）するか、周りに 2m 以上何もない広い場所に置く
- [ ] ESP32 の Web UI をスマホで開き、すぐに **停止ボタン** を押せるようにしておく（手元に電源スイッチも）
- [ ] ESP32 の `AUTO_MAX_DUTY`（既定 0.5）と cugo_base の `max_duty`（既定 0.5）を低め（例 0.3）にしておく
- [ ] 上の「要実測」の値を入れ、`cugo_base` のキャリブレーションを済ませる

センサーとオドメトリ（`use_base:=false` でモーターにつながずに）

- [ ] `ros2 launch cugo_bringup bringup.launch.py mode:=slam use_base:=false use_rviz:=true`
- [ ] `/scan` が約 10Hz、RViz で壁の形が車体の向きと合っている（前の壁が前に見える）
- [ ] ロボットを手で押して動かし、RViz でロボットが同じ向き・同じ距離だけ動く（`/odometry/filtered`）
- [ ] 手で回したときの回転方向が合っている
- [ ] Local Costmap に床が塗られていない

手動で動かす

- [ ] `use_base` を外して起動し、`ros2 topic echo /cugo/mode` が `idle`
- [ ] teleop で **ごく低速** から（teleop の `z` キーで速度を下げる）。前進キーで前に進む / 左旋回キーで左に回る
- [ ] キーを離すと止まる。teleop を Ctrl+C すると 0.5 秒以内に止まる
- [ ] Web UI の停止ボタンで止まる
- [ ] 地図を作り、保存する

自律移動

- [ ] 最初のゴールは 1〜2m 先、障害物のない場所
- [ ] 走行中に teleop のキーを押すと teleop が優先される / `/pause_navigation` で止まる
- [ ] 人や箱を前に置くと減速・停止する（collision_monitor とコストマップ）
- [ ] 問題なければ速度上限（ESP32 → cugo_base → Nav2）を少しずつ上げる

## 7. 既知の問題・注意

- **D455 の IMU が出ない**: この Jetson のカーネル（6.8 tegra、`CONFIG_HID_SENSOR_HUB` 無効）には
  HID センサーのドライバがなく、apt 版 librealsense（V4L バックエンド）では IMU が使えません
  （ログに `No HID info provided, IMU is disabled`）。`/camera/camera/imu` は出ず、EKF は rf2o と /wheel_odom だけで動きます。
  IMU を使うには librealsense を RSUSB バックエンド（`-DFORCE_RSUSB_BACKEND=ON`）でビルドして
  realsense2_camera もそれでビルドし直すか、HID センサーのカーネルモジュールを用意する必要があります。
- Jetson（ARM）の librealsense では点群フィルタのパラメータ名が `pointcloud__neon_.*` になります（`sensors.launch.py` で対応済み）。
- Jazzy の navigation_launch.py は docking_server の `cmd_vel` をリマップしないので、`navigation.launch.py` で
  `SetRemap(cmd_vel → cmd_vel_nav)` を掛けて twist_mux を迂回しないようにしています。
- rf2o は起動時に base_footprint → laser_frame の TF を読むので、robot_state_publisher より後に起動すること（bringup では自動）。
