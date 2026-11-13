"""CuGo Web UI ノード。

aiohttp で静的ファイル（web/）と WebSocket（/ws）を配信し、ROS 2 とつなぐ。

- 手動操縦: ブラウザの {"t":"drive"} → /cmd_vel_teleop（twist_mux で Nav2 より優先）
  - 指令が drive_timeout（0.3s）途絶えたら 0 を数回送って配信をやめる（Nav2 に制御を戻す）
- 停止: {"t":"stop"} → 0 を送り、Nav2 のゴールを取り消す
- 目的地: {"t":"goal"} → Nav2 の navigate_to_pose アクション
- 地図保存: {"t":"save_map"} → slam_toolbox の save_map（~/maps/<name>.yaml / .pgm）
- 配信: 地図（PNG、変化時・最大 1Hz）、ロボット姿勢・モード・ナビ状態（5Hz）、スキャン（5Hz）、経路
- カメラ映像: /video.mjpg（MJPEG）。見ている人がいる間だけ video_topic を購読する
- 自動探索の記録: explore_log:=true（既定）なら 1 回ごとに explore_log_dir/explore_*.jsonl（explog.py）

rclpy は別スレッドの executor で回し、aiohttp は main スレッドの asyncio で動かす。
ROS のコールバックは最新値を保存するだけにして、送信は asyncio 側のループがまとめて行う。
"""
import asyncio
import json
import math
import os
import signal
import threading
import time
from typing import Optional, Set

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import CompressedImage, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
import tf2_ros

from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory

import numpy as np

from cugo_web import calibration, explog, floor, keepout, tls
from cugo_web.seen import SeenFloor
from cugo_web.explorer import Explorer
from cugo_web.geometry import (clamp_drive, occupancy_to_png, path_to_points, png_b64,
                               quat_from_yaw, scan_to_points, valid_map_name, yaw_from_quat)
from cugo_web.video import FrameHub, make_mjpeg_handler

try:
    from slam_toolbox.srv import SaveMap
except ImportError:  # slam_toolbox が無い環境でも UI 自体は動かす
    SaveMap = None
try:
    from slam_toolbox.srv import Reset as SlamReset
except ImportError:  # 古い slam_toolbox には Reset が無い
    SlamReset = None

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_ABORTED: 'aborted',
    GoalStatus.STATUS_CANCELED: 'canceled',
}


class CugoWebNode(Node):
    def __init__(self):
        super().__init__('cugo_web')
        p = self.declare_parameter
        self.port = int(p('port', 8080).value)
        self.https_port = int(p('https_port', 8443).value)  # 0 で無効
        self.host = str(p('host', '0.0.0.0').value)
        self.web_dir = str(p('web_dir', '').value) or os.path.join(
            get_package_share_directory('cugo_web'), 'web')
        self.map_frame = str(p('map_frame', 'map').value)
        self.base_frame = str(p('base_frame', 'base_footprint').value)
        self.max_v = float(p('max_linear', 0.4).value)
        self.max_w = float(p('max_angular', 1.5).value)
        self.drive_timeout = float(p('drive_timeout', 0.3).value)
        self.map_dir = os.path.expanduser(str(p('map_dir', '~/maps').value))
        # カメラ映像（realsense2_camera の JPEG をそのまま MJPEG で送る）
        self.video_topic = str(p('video_topic',
                                 '/camera/camera/color/image_raw/compressed').value)
        self.video_fps = float(p('video_fps', 10.0).value)

        self._lock = threading.Lock()
        # 手動操縦
        self._drive = (0.0, 0.0)
        self._drive_stamp = 0.0
        self._zero_burst = 0  # 途絶・停止後に 0 を送る残り回数
        # 表示用の最新値
        self.mode = 'disconnected'
        self.map_msg: Optional[dict] = None
        self.map_version = 0
        self.scan_pts: list = []
        self.scan_version = 0
        self.plan_pts: list = []
        self.plan_version = 0
        self.cliff_pts: list = []      # 下り段差（map 座標の x,y を平らに並べたもの）
        self.cliff_version = 0
        self.on_cliff = None  # WebServer が通知関数を入れる
        self._cliff_warned = 0.0
        # カメラが床として見たセル（自動探索の判断用の地図で、未知を空きにする。/map は変えない）
        self.use_camera_floor = bool(p('use_camera_floor', True).value)
        self.seen_floor: Optional[SeenFloor] = SeenFloor() if self.use_camera_floor else None
        # ナビ
        self.nav_state = 'idle'
        self.map_raw: Optional[OccupancyGrid] = None
        # 進入禁止エリア。保存した地図（map_yaml）で起動したときは <地図>.keepout.yaml を使う
        map_yaml = str(p('map_yaml', '').value)
        self.keepout_file = (os.path.splitext(map_yaml)[0] + '.keepout.yaml' if map_yaml
                             else os.path.join(calibration.CONFIG_DIR, 'keepout.yaml'))
        self.zones = keepout.load(self.keepout_file)
        self._zone_seq = len(self.zones)
        self.explorer = None  # WebServer が作る（通知先が必要なため）
        # 自動探索の評価用の記録（explog.py）。explore_variant は比べるときの名前（例: "cam_off"）
        self.explore_log = bool(p('explore_log', True).value)
        self.explore_log_dir = str(p('explore_log_dir', '~/cugo_logs').value)
        self.explore_variant = str(p('explore_variant', '').value)
        self._area_cache = (None, 0.0, 0.0)  # (map_raw, 既知 m², 空き m²)
        self.goal: Optional[dict] = None
        self._goal_handle = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        # Nav2 の KeepoutFilter へ（config/nav2_params.yaml の keepout_filter）
        self.mask_pub = self.create_publisher(OccupancyGrid, '/keepout_mask', latched)
        self.filter_info_pub = self.create_publisher(CostmapFilterInfo, '/costmap_filter_info',
                                                     latched)
        info = CostmapFilterInfo()
        info.header.frame_id = self.map_frame
        info.type = 0  # KEEPOUT
        info.filter_mask_topic = '/keepout_mask'  # costmap は名前空間内なので絶対名
        info.base = 0.0
        info.multiplier = 1.0
        self.filter_info_pub.publish(info)
        self.publish_keepout()
        self.create_subscription(String, 'cugo/mode', self._on_mode, latched)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, 'plan', self._on_plan, 10)
        self.create_subscription(PointCloud2, '/cliff/points', self._on_cliff, 5)
        if self.seen_floor is not None:
            self.create_subscription(PointCloud2, '/cliff/floor', self._on_floor, 5)
            # RICOH THETA の床（theta_floor）。LiDAR が届かない廊下の奥など
            self.create_subscription(PointCloud2, '/theta/floor', self._on_floor, 5)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.save_client = (self.create_client(SaveMap, 'slam_toolbox/save_map')
                            if SaveMap else None)
        self.create_timer(0.05, self._drive_tick)  # 20Hz
        self._last_map_encode = 0.0
        self._pending_map: Optional[OccupancyGrid] = None
        self.create_timer(0.5, self._encode_map_tick)
        # カメラ映像。見ている人がいる間だけ購読する（付け外しは 1Hz のタイマーで）
        self.video = FrameHub()
        self._video_sub = None
        self.create_timer(1.0, self._video_tick)
        if self.explore_log:
            self.create_timer(float(p('explore_log_period', 1.0).value), self._explore_log_tick)

        # 実機調整（LiDAR の取付向き）
        self.xacro_file = str(p('xacro_file', '').value) or os.path.join(
            get_package_share_directory('cugo_description'), 'urdf', 'cugo.urdf.xacro')
        self.rsp_param_client = self.create_client(
            SetParameters, 'robot_state_publisher/set_parameters')
        self.reset_client = (self.create_client(SlamReset, 'slam_toolbox/reset')
                             if SlamReset else None)
        self.cloud_topic = str(p('cloud_topic', '/camera/camera/depth/color/points').value)
        self._floor_busy = False
        self.calib = calibration.load()
        if 'lidar_yaw' not in self.calib:
            # 保存値が無ければ URDF（xacro の既定値）から読む
            try:
                yaw = calibration.lidar_yaw_from_urdf(
                    calibration.build_urdf(self.xacro_file, {}))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('xacro failed: %s' % exc)
                yaw = None
            self.lidar_yaw = yaw if yaw is not None else 0.0
        else:
            self.lidar_yaw = self.calib['lidar_yaw']

    # ---------------- 手動操縦 ----------------
    def set_drive(self, v, w):
        v, w = clamp_drive(v, w, self.max_v, self.max_w)
        if (v or w) and self.explorer is not None and self.explorer.state == 'running':
            self.get_logger().info('manual drive v=%.3f w=%.3f stops exploration' % (v, w))
            self.explorer.stop('手動で操縦したので自動探索をやめました', cause='manual_drive')
        with self._lock:
            self._drive = (v, w)
            self._drive_stamp = time.monotonic()
            self._zero_burst = 5

    def stop(self):
        with self._lock:
            self._drive = (0.0, 0.0)
            self._drive_stamp = 0.0
            self._zero_burst = 10
        self._publish(0.0, 0.0)
        if self.explorer is not None:
            self.explorer.stop('停止しました（自動探索も終了）')
        self.cancel_goal()

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def _drive_tick(self):
        with self._lock:
            active = time.monotonic() - self._drive_stamp < self.drive_timeout
            if active:
                v, w = self._drive
            elif self._zero_burst > 0:
                self._zero_burst -= 1
                v, w = 0.0, 0.0
            else:
                return  # 何も送らない → twist_mux が Nav2 に切り替える
        self._publish(v, w)

    @property
    def teleop_active(self) -> bool:
        return time.monotonic() - self._drive_stamp < self.drive_timeout

    # ---------------- 購読 ----------------
    def _on_mode(self, msg: String):
        self.mode = msg.data

    def _on_map(self, msg: OccupancyGrid):
        self.map_raw = msg  # 自動探索用（そのまま）
        self._pending_map = msg  # エンコードはタイマーで（最大 1Hz）

    def _encode_map_tick(self):
        msg = self._pending_map
        if msg is None or time.monotonic() - self._last_map_encode < 1.0:
            return
        self._pending_map = None
        self._last_map_encode = time.monotonic()
        info = msg.info
        try:
            png = occupancy_to_png(msg.data, info.width, info.height)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('map encode failed: %s' % exc)
            return
        self.map_msg = {
            't': 'map', 'w': info.width, 'h': info.height, 'res': info.resolution,
            'ox': info.origin.position.x, 'oy': info.origin.position.y, 'png': png_b64(png),
        }
        self.map_version += 1

    def _on_scan(self, msg: LaserScan):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, Time())
        except tf2_ros.TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        self.scan_pts = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                       msg.range_min, msg.range_max,
                                       t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        self.scan_version += 1

    def _cloud_to_map_xy(self, msg: PointCloud2, exact: bool = False) -> Optional[np.ndarray]:
        """点群の x, y を map 座標の (N, 2) にする。読めない・TF が無いときは None（点が 0 個なら空）。

        exact=True なら撮影時刻の姿勢を使い、無ければ None（最新の姿勢で代用しない）。
        旋回中に最新の姿勢を使うと、2 m 先で数十 cm ずれて壁の向こうまで「床」になるため。
        """
        try:
            xyz = point_cloud2.read_points_numpy(msg, field_names=('x', 'y'), skip_nans=True)
        except Exception:  # noqa: BLE001
            return None
        if not len(xyz):
            return np.zeros((0, 2))
        stamp = Time.from_msg(msg.header.stamp) if exact else Time()
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, msg.header.frame_id, stamp)
        except tf2_ros.TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        c, s_ = math.cos(yaw), math.sin(yaw)
        x = t.x + c * xyz[:, 0] - s_ * xyz[:, 1]
        y = t.y + s_ * xyz[:, 0] + c * xyz[:, 1]
        return np.column_stack([x, y])

    def _on_cliff(self, msg: PointCloud2):
        """cliff_detector の段差セル（base_footprint）を map 座標にして Web に出す。"""
        xy = self._cloud_to_map_xy(msg)
        if xy is None:
            return
        pts: list = []
        if len(xy):
            pts = np.round(xy.ravel(), 3).tolist()
            now = time.monotonic()
            if now - self._cliff_warned > 10.0 and self.on_cliff:
                self._cliff_warned = now
                self.on_cliff(len(xy))
        if pts or self.cliff_pts:
            self.cliff_pts = pts
            self.cliff_version += 1

    def _on_floor(self, msg: PointCloud2):
        """cliff_detector の床セル（/cliff/floor）を覚える。自動探索の判断用の地図だけに使う。"""
        xy = self._cloud_to_map_xy(msg, exact=True)
        if xy is not None and len(xy) and self.seen_floor is not None:
            self.seen_floor.add(xy)

    def _video_tick(self):
        viewers = self.video.viewers
        if viewers > 0 and self._video_sub is None:
            self._video_sub = self.create_subscription(
                CompressedImage, self.video_topic, self._on_video, qos_profile_sensor_data)
            self.get_logger().info('video: subscribe %s' % self.video_topic)
        elif viewers == 0 and self._video_sub is not None:
            self.destroy_subscription(self._video_sub)
            self._video_sub = None
            self.get_logger().info('video: unsubscribe (no viewers)')

    def _on_video(self, msg: CompressedImage):
        if 'jpeg' in msg.format:  # png などはブラウザの MJPEG では出せないので捨てる
            self.video.put(bytes(msg.data))

    def _on_plan(self, msg: Path):
        self.plan_pts = path_to_points((ps.pose.position.x, ps.pose.position.y)
                                       for ps in msg.poses)
        self.plan_version += 1

    def robot_pose(self) -> Optional[dict]:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return {'x': round(t.x, 3), 'y': round(t.y, 3),
                'yaw': round(yaw_from_quat(q.x, q.y, q.z, q.w), 3)}

    # ---------------- 自動探索の記録 ----------------
    def new_explore_log(self, explorer_config: dict):
        """Explorer.start() から呼ばれる。1 回分の記録を始める（設定の写しを 1 行目に書く）。"""
        pubs = {}
        for topic in ('/cliff/floor', '/theta/floor', '/map'):
            try:
                pubs[topic] = self.count_publishers(topic)
            except Exception:  # noqa: BLE001
                pubs[topic] = None
        msg = self.map_raw
        config = {
            'variant': self.explore_variant,
            'use_camera_floor': self.use_camera_floor,
            # 床を出しているノードが実際にいるか（use_theta 相当: /theta/floor の発行者が 1 以上）
            'publishers': pubs,
            'use_theta': bool(self.use_camera_floor and pubs.get('/theta/floor')),
            'use_cliff_floor': bool(self.use_camera_floor and pubs.get('/cliff/floor')),
            'seen_floor': ({'cell': self.seen_floor.cell, 'min_hits': self.seen_floor.min_hits}
                           if self.seen_floor is not None else None),
            'map_resolution': msg.info.resolution if msg is not None else None,
            'keepout_zones': len(self.zones),
        }
        config.update(explorer_config)
        return explog.ExploreLog(self.explore_log_dir, config)

    def _map_areas(self):
        """slam_toolbox の /map の既知・空き面積 [m²]（地図が変わったときだけ数え直す）。"""
        msg = self.map_raw
        if msg is None:
            return None
        if self._area_cache[0] is not msg:
            known, free = explog.map_areas(np.asarray(msg.data, dtype=np.int16),
                                           msg.info.resolution)
            self._area_cache = (msg, known, free)
        return self._area_cache[1], self._area_cache[2]

    def _explore_log_tick(self):
        ex = self.explorer
        log = ex.log if ex is not None else None
        if log is None:
            return
        try:
            pose = self.robot_pose()
            areas = self._map_areas()
            if pose is None or areas is None:
                return
            seen = len(self.seen_floor) if self.seen_floor is not None else 0
            log.sample(pose['x'], pose['y'], pose['yaw'], areas[0], areas[1], seen,
                       nf=ex.frontiers, nav=self.nav_state)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn('explore log sample: %s' % exc)

    # ---------------- ナビ ----------------
    @staticmethod
    def _parse_goal(x, y, yaw):
        try:
            x, y, yaw = float(x), float(y), float(yaw)
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return None
        return x, y, yaw

    def _make_goal(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = x, y
        qx, qy, qz, qw = quat_from_yaw(yaw)
        ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
        goal.pose = ps
        return goal

    def navigate(self, x, y, yaw, on_result=None, timeout=5.0):
        """目的地を送り、受け付けられるまで待つ（ROS の executor 以外のスレッドから呼ぶ）。

        戻り値は {'status': None | 'succeeded' | 'aborted' | 'canceled'} を持つ追跡用の dict。
        受け付けられなければ None。
        """
        if not self.nav_client.server_is_ready():
            return None
        self.goal = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
        self.nav_state = 'active'
        accepted = threading.Event()
        box = {}

        def on_accepted(fut):
            handle = fut.result()
            if handle is None or not handle.accepted:
                box['tracker'] = None
                accepted.set()
                return
            tracker = {'status': None}
            self._goal_handle = handle
            box['tracker'] = tracker

            def done(rfut):
                status = STATUS_NAMES.get(rfut.result().status, 'aborted')
                tracker['status'] = status
                if self._goal_handle is handle:  # 新しいゴールに置き換わっていなければ
                    self.nav_state = status
                    self._goal_handle = None
                    # 終わった目的地と経路を地図に残さない（止まっているのに移動中に見えるため）
                    self.goal = None
                    self.plan_pts = []
                    self.plan_version += 1
                self.get_logger().info('goal (%.2f, %.2f) -> %s' % (x, y, status))
                if on_result:
                    on_result(status)
            handle.get_result_async().add_done_callback(done)
            accepted.set()
        self.nav_client.send_goal_async(self._make_goal(x, y, yaw)).add_done_callback(on_accepted)
        if not accepted.wait(timeout) or box.get('tracker') is None:
            self.nav_state = 'aborted'
            return None
        return box['tracker']

    @staticmethod
    def goal_status(tracker):
        return tracker['status']

    def send_goal(self, x, y, yaw, notify):
        """ブラウザからの目的地。受け付けを待つので別スレッドで実行する。"""
        g = self._parse_goal(x, y, yaw)
        if g is None:
            notify('error', '目的地の値が不正です')
            return
        if not self.nav_client.server_is_ready():
            notify('error', 'Nav2 が起動していません')
            return
        if self.explorer is not None:
            self.explorer.stop('目的地が指定されたので自動探索をやめました', cause='manual_goal')

        def on_result(status):
            notify({'succeeded': 'info'}.get(status, 'warn'),
                   {'succeeded': '目的地に到着しました',
                    'canceled': '移動を中止しました'}.get(status, '移動に失敗しました'))

        def worker():
            if self.navigate(*g, on_result=on_result) is None:
                notify('error', '目的地が受け付けられませんでした')
            else:
                notify('info', '目的地へ移動します')
        threading.Thread(target=worker, daemon=True).start()

    def cancel_goal(self):
        handle = self._goal_handle
        if handle is not None:
            self.get_logger().info('cancel goal')
            handle.cancel_goal_async()

    # ---------------- 進入禁止エリア ----------------
    def publish_keepout(self):
        data, w, h, (ox, oy) = keepout.mask_grid(self.zones)
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = 0.05
        msg.info.width, msg.info.height = w, h
        msg.info.origin.position.x, msg.info.origin.position.y = ox, oy
        msg.info.origin.orientation.w = 1.0
        msg.data = data.tolist()
        self.mask_pub.publish(msg)

    def _zones_changed(self, notify):
        try:
            keepout.save(self.zones, self.keepout_file)
        except OSError as exc:
            notify('warn', '進入禁止エリアを保存できませんでした: %s' % exc)
        self.publish_keepout()

    def add_zone(self, pts, notify, on_change):
        pts = keepout.validate_points(pts)
        if pts is None:
            notify('error', '進入禁止エリアの形が不正です（3 点以上で囲んでください）')
            return
        if len(self.zones) >= keepout.MAX_ZONES:
            notify('error', '進入禁止エリアは %d 個までです' % keepout.MAX_ZONES)
            return
        self._zone_seq += 1
        self.zones = self.zones + [{'id': 'z%d' % self._zone_seq, 'pts': pts}]
        self._zones_changed(notify)
        notify('info', '進入禁止エリアを追加しました')
        on_change()

    def remove_zone(self, zone_id, notify, on_change):
        zones = [z for z in self.zones if z['id'] != zone_id]
        if len(zones) == len(self.zones):
            return
        self.zones = zones
        self._zones_changed(notify)
        notify('info', '進入禁止エリアを削除しました')
        on_change()

    def clear_zones(self, notify, on_change):
        self.zones = []
        self._zones_changed(notify)
        notify('info', '進入禁止エリアをすべて消しました')
        on_change()

    # ---------------- 地図保存 ----------------
    def save_map(self, name, notify):
        if not valid_map_name(name):
            notify('error', '地図の名前は英数字・-・_ のみ（64 文字まで）')
            return
        if self.save_client is None or not self.save_client.service_is_ready():
            notify('error', 'slam_toolbox が起動していません（mode:=slam のときだけ保存できます）')
            return
        os.makedirs(self.map_dir, exist_ok=True)
        req = SaveMap.Request()
        req.name.data = os.path.join(self.map_dir, name)

        def done(fut):
            res = fut.result()
            if res is not None and res.result == SaveMap.Response.RESULT_SUCCESS:
                if self.zones:
                    try:
                        keepout.save(self.zones, req.name.data + '.keepout.yaml')
                    except OSError:
                        pass
                notify('info', '保存しました: %s.yaml' % req.name.data)
            else:
                notify('error', '地図の保存に失敗しました')
        self.save_client.call_async(req).add_done_callback(done)


    # ---------------- 実機調整 ----------------
    def apply_calibration(self, updates, notify, on_done, label, wait=False):
        """URDF を作り直して robot_state_publisher に反映し、calibration.yaml に保存する。

        wait=True なら反映完了まで待って成功したかを返す（別スレッドから呼ぶとき）。
        """
        values = dict(self.calib)
        values.update({k: round(float(v), 4) for k, v in updates.items()})
        try:
            urdf = calibration.build_urdf(self.xacro_file, values)
        except Exception as exc:  # noqa: BLE001
            notify('error', 'URDF の生成に失敗しました: %s' % exc)
            return False
        if not self.rsp_param_client.service_is_ready():
            notify('error', 'robot_state_publisher が見つかりません')
            return False
        req = SetParameters.Request()
        req.parameters = [ParameterMsg(
            name='robot_description',
            value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=urdf))]
        finished = threading.Event()
        ok = [False]

        def done(fut):
            try:
                res = fut.result()
                if res is None or not res.results or not res.results[0].successful:
                    reason = res.results[0].reason if res and res.results else ''
                    notify('error', '%sを反映できませんでした %s' % (label, reason))
                    return
                try:
                    calibration.save(values)
                except OSError as exc:
                    notify('warn', '反映しましたが保存に失敗しました: %s' % exc)
                self.calib = values
                if 'lidar_yaw' in values:
                    self.lidar_yaw = values['lidar_yaw']
                self.get_logger().info('calibration: %s' % values)
                ok[0] = True
                on_done()
            finally:
                finished.set()
        self.rsp_param_client.call_async(req).add_done_callback(done)
        if wait:
            finished.wait(5.0)
        return ok[0]

    def set_lidar_yaw(self, yaw, notify, on_done):
        try:
            yaw = float(yaw)
        except (TypeError, ValueError):
            notify('error', '角度の値が不正です')
            return
        if not math.isfinite(yaw):
            notify('error', '角度の値が不正です')
            return
        yaw = round(calibration.wrap_angle(yaw), 4)

        def after():
            notify('info', 'LiDAR の向きを %+.0f° にしました（保存済み）' % math.degrees(yaw))
            # slam_toolbox と rf2o は起動時に一度だけ LiDAR の取付向きを読む。
            # 再起動しないと地図は古い向きで作られ続け、costmap とずれる（2026-11-13 に実機で発生）
            notify('warn', '地図づくり（slam_toolbox）と rf2o には再起動まで反映されません。'
                           'sudo systemctl restart cugo で再起動してください（地図もリセットされます）')
            on_done()
        self.apply_calibration({'lidar_yaw': yaw}, notify, after, 'LiDAR の向き')

    def camera_pose(self):
        """現在の URDF での camera_link の (x, y, z, roll, pitch)（base_footprint 基準）。"""
        tf = self.tf_buffer.lookup_transform(self.base_frame, 'camera_link', Time())
        t, q = tf.transform.translation, tf.transform.rotation
        R = floor.rotation_matrix((q.x, q.y, q.z, q.w))
        pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
        roll = math.atan2(R[2, 1], R[2, 2])
        return t.x, t.y, t.z, roll, pitch

    def calib_floor(self, notify, on_done):
        if self._floor_busy:
            notify('warn', 'カメラ補正を実行中です')
            return
        self._floor_busy = True
        threading.Thread(target=self._calib_floor_worker, args=(notify, on_done),
                         daemon=True).start()

    def _collect_cloud(self, frames=5, timeout=4.0):
        """点群を数フレーム集めて base_footprint 座標の (N, 3) にする。"""
        got = []
        done = threading.Event()

        def cb(msg):
            if done.is_set():
                return
            try:
                tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
            except tf2_ros.TransformException:
                return
            pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'),
                                                 skip_nans=True)[::3]
            t, q = tf.transform.translation, tf.transform.rotation
            R = floor.rotation_matrix((q.x, q.y, q.z, q.w))
            got.append(pts @ R.T + [t.x, t.y, t.z])
            if len(got) >= frames:
                done.set()
        sub = self.create_subscription(PointCloud2, self.cloud_topic, cb, qos_profile_sensor_data)
        done.wait(timeout)
        self.destroy_subscription(sub)
        return np.vstack(got) if got else np.zeros((0, 3))

    def _calib_floor_worker(self, notify, on_done):
        try:
            notify('info', 'カメラを床で補正しています…')
            for i in range(3):  # 反映 → 測り直し を繰り返して収束させる
                pts = self._collect_cloud()
                fit = floor.fit_floor(pts)
                if fit is None:
                    notify('error', '床が見つかりません。前方 0.5〜2m に平らな床が見えるようにしてください'
                                    '（点 %d）' % len(pts))
                    return
                x, y, z, roll, pitch = self.camera_pose()
                new_pitch, new_roll, new_z = floor.corrected_pose(fit, pitch, roll, z, x, y)
                self.get_logger().info(
                    'floor fit %d: a=%.3f b=%.3f c=%.3f inliers=%d/%d -> pitch %.3f roll %.3f z %.3f'
                    % (i, fit.a, fit.b, fit.c, fit.inliers, fit.candidates,
                       new_pitch, new_roll, new_z))
                if abs(math.degrees(new_pitch)) > 45 or abs(math.degrees(new_roll)) > 30 \
                        or not 0.03 < new_z < 1.5:
                    notify('error', '補正値が大きすぎるので中止しました（床以外を拾った可能性）')
                    return
                if not self.apply_calibration(
                        {'camera_pitch': new_pitch, 'camera_roll': new_roll, 'camera_z': new_z},
                        notify, on_done, 'カメラの姿勢', wait=True):
                    return
                if abs(fit.b) < 0.005 and abs(fit.c) < 0.005 and abs(fit.a) < 0.005:
                    break
                time.sleep(1.0)  # 新しい TF が行き渡るのを待つ
            c = self.calib
            notify('info', 'カメラ: 下向き %.1f° / 横 %.1f° / 高さ %.2f m（保存済み）'
                   % (math.degrees(c['camera_pitch']), math.degrees(c['camera_roll']),
                      c['camera_z']))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error('calib_floor: %s' % exc)
            notify('error', 'カメラ補正に失敗しました: %s' % exc)
        finally:
            self._floor_busy = False

    def reset_map(self, notify):
        if self.reset_client is None or not self.reset_client.service_is_ready():
            notify('error', '地図をリセットできません（mode:=slam で起動していないか、'
                            'slam_toolbox が対応していません）')
            return
        self.map_msg = None
        if self.explorer is not None:
            self.explorer.forget()  # 地図の座標が変わるので、覚えた場所は使えない
        if self.seen_floor is not None:
            self.seen_floor.clear()  # 床として見たセルも同じ理由で消す

        def done(fut):
            if fut.result() is None:
                notify('error', '地図のリセットに失敗しました')
            else:
                notify('info', '地図をリセットしました')
        self.reset_client.call_async(SlamReset.Request()).add_done_callback(done)


# ---------------- Web サーバー ----------------
class WebServer:
    def __init__(self, node: CugoWebNode):
        self.node = node
        self.clients: Set[web.WebSocketResponse] = set()
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        node.explorer = Explorer(node, self.notify)
        if node.explore_log:
            node.explorer.log_factory = node.new_explore_log
        node.on_cliff = lambda n: self.notify('warn', '下り段差を検出しました（%d セル）。障害物として避けます' % n)

    def notify(self, level, text):
        """ROS スレッドからも呼べる通知（全クライアントへ）。"""
        if self.loop is None:
            return
        msg = {'t': 'msg', 'level': level, 'text': text}
        asyncio.run_coroutine_threadsafe(self.broadcast(msg), self.loop)

    def calib_msg(self):
        msg = {'t': 'calib', 'lidar_yaw': self.node.lidar_yaw}
        for k in ('camera_pitch', 'camera_roll', 'camera_z'):
            if k in self.node.calib:
                msg[k] = self.node.calib[k]
        return msg

    def keepout_msg(self):
        return {'t': 'keepout', 'zones': [{'id': z['id'], 'pts': [list(p) for p in z['pts']]}
                                          for z in self.node.zones]}

    def send_keepout(self):
        if self.loop is not None:
            asyncio.run_coroutine_threadsafe(self.broadcast(self.keepout_msg()), self.loop)

    def send_calib(self):
        """ROS スレッドから呼ぶ。"""
        if self.loop is not None:
            asyncio.run_coroutine_threadsafe(self.broadcast(self.calib_msg()), self.loop)

    async def broadcast(self, obj):
        data = json.dumps(obj, separators=(',', ':'))
        for ws in list(self.clients):
            try:
                await ws.send_str(data)
            except (ConnectionError, RuntimeError):
                self.clients.discard(ws)

    async def index(self, request):
        return web.FileResponse(os.path.join(self.node.web_dir, 'index.html'),
                                headers={'Cache-Control': 'no-cache'})

    async def ws_handler(self, request):
        ws = web.WebSocketResponse(heartbeat=5.0)
        await ws.prepare(request)
        self.clients.add(ws)
        node = self.node
        self.node.get_logger().info('web client connected: %s' % request.remote)
        if node.map_msg is not None:
            await ws.send_str(json.dumps(node.map_msg))
        await ws.send_str(json.dumps(self.calib_msg()))
        await ws.send_str(json.dumps(self.keepout_msg()))
        drove = False
        try:
            async for m in ws:
                if m.type != WSMsgType.TEXT:
                    continue
                try:
                    msg = json.loads(m.data)
                except ValueError:
                    continue
                if not isinstance(msg, dict):
                    continue
                t = msg.get('t')
                if t == 'drive':
                    node.set_drive(msg.get('v'), msg.get('w'))
                    drove = True
                elif t == 'stop':
                    node.stop()
                elif t == 'goal':
                    node.send_goal(msg.get('x'), msg.get('y'), msg.get('yaw'), self.notify)
                elif t == 'cancel':
                    node.cancel_goal()
                elif t == 'save_map':
                    node.save_map(msg.get('name'), self.notify)
                elif t == 'set_lidar_yaw':
                    node.set_lidar_yaw(msg.get('yaw'), self.notify, self.send_calib)
                elif t == 'reset_map':
                    node.reset_map(self.notify)
                elif t == 'explore':
                    if msg.get('on'):
                        node.explorer.start()
                    else:
                        node.explorer.stop('自動探索を止めました')
                elif t == 'explore_skip':
                    node.explorer.skip()
                elif t == 'keepout_add':
                    node.add_zone(msg.get('pts'), self.notify, self.send_keepout)
                elif t == 'keepout_remove':
                    node.remove_zone(msg.get('id'), self.notify, self.send_keepout)
                elif t == 'keepout_clear':
                    node.clear_zones(self.notify, self.send_keepout)
                elif t == 'calib_floor':
                    node.calib_floor(self.notify, self.send_calib)
        finally:
            self.clients.discard(ws)
            if drove:
                # 操縦していた端末が切れたら止める（ウォッチドッグでも止まるが即時に）
                with node._lock:
                    node._drive_stamp = 0.0
                    node._zero_burst = 5
            self.node.get_logger().info('web client disconnected: %s' % request.remote)
        return ws

    async def publisher(self):
        """状態・スキャン・経路・地図を定期配信する。"""
        node = self.node
        sent = {'map': node.map_version, 'scan': -1, 'plan': -1, 'cliff': -1}
        while True:
            await asyncio.sleep(0.2)
            if not self.clients:
                continue
            await self.broadcast({
                't': 'state', 'mode': node.mode, 'pose': node.robot_pose(),
                'nav': node.nav_state, 'goal': node.goal, 'teleop': node.teleop_active,
                'explore': node.explorer.state, 'frontiers': node.explorer.frontiers,
            })
            if node.scan_version != sent['scan']:
                sent['scan'] = node.scan_version
                await self.broadcast({'t': 'scan', 'pts': node.scan_pts})
            if node.cliff_version != sent['cliff']:
                sent['cliff'] = node.cliff_version
                await self.broadcast({'t': 'cliff', 'pts': node.cliff_pts})
            if node.plan_version != sent['plan']:
                sent['plan'] = node.plan_version
                await self.broadcast({'t': 'plan', 'pts': node.plan_pts})
            if node.map_version != sent['map'] and node.map_msg is not None:
                sent['map'] = node.map_version
                await self.broadcast(node.map_msg)

    async def run(self):
        self.loop = asyncio.get_running_loop()
        # SIGINT（ros2 launch / systemd の停止）と SIGTERM でこのタスクを止める。
        # rclpy の既定のハンドラだと ROS だけ止まって aiohttp が動き続け、プロセスが終わらなかった
        task = asyncio.current_task()
        for sig in (signal.SIGINT, signal.SIGTERM):
            self.loop.add_signal_handler(sig, task.cancel)
        self.node.video.bind(self.loop)
        app = web.Application()
        app.router.add_get('/', self.index)
        app.router.add_get('/ws', self.ws_handler)
        app.router.add_get('/video.mjpg', make_mjpeg_handler(self.node.video, self.node.video_fps))
        # --symlink-install だと web/ の中身がシンボリックリンクになるので follow_symlinks が必要
        app.router.add_static('/', self.node.web_dir, show_index=False, follow_symlinks=True)
        runner = web.AppRunner(app, access_log=None)
        await runner.setup()
        site = web.TCPSite(runner, self.node.host, self.node.port)
        await site.start()
        self.node.get_logger().info(
            'web UI: http://<JetsonのIP>:%d/  (static: %s)' % (self.node.port, self.node.web_dir))
        if self.node.https_port > 0:
            try:
                ctx = tls.ssl_context()
                await web.TCPSite(runner, self.node.host, self.node.https_port,
                                  ssl_context=ctx).start()
                self.node.get_logger().info(
                    'web UI (https, DualShock 用): https://<JetsonのIP>:%d/'
                    % self.node.https_port)
            except Exception as exc:  # noqa: BLE001  https が無くても http は使える
                self.node.get_logger().warn('https を起動できませんでした: %s' % exc)
        try:
            await self.publisher()
        finally:
            self.node.video.close()  # 開いている /video.mjpg を終わらせる（cleanup が待たないように）
            for ws in list(self.clients):
                await ws.close()
            await runner.cleanup()


def main(args=None):
    # シグナルは WebServer.run（asyncio）で受ける
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CugoWebNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    server = WebServer(node)
    try:
        asyncio.run(server.run())
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        try:
            node._publish(0.0, 0.0)  # 念のため停止指令
        except Exception:  # noqa: BLE001
            pass
        try:  # 探索中なら記録を閉じる（通知は asyncio が終わっているので出さない）
            if node.explorer is not None:
                node.explorer.stop(None, cause='shutdown')
        except Exception:  # noqa: BLE001
            pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
