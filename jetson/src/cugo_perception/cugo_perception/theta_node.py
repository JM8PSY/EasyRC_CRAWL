"""RICOH THETA（ライブストリーミングモード、UVC）で前方の床がどこまで続くかを見るノード。

SLAM の補助: LiDAR が「反射なし」の方位（廊下の奥）や LiDAR の死角で、床が続いている所を
/theta/floor（base_footprint の点、/cliff/floor と同じ形）に出す。cugo_web の自動探索が
「未知 → 空き」として判断用の地図にだけ使う。slam_toolbox / Nav2 の地図は変えない。

- /theta/floor : 床が続いている所の点（PointCloud2 xyz、base_footprint、撮影時刻）
- /theta/scan  : 方位ごとの床の距離（LaserScan、base_footprint、確認用）
- /theta/image/compressed : 正距円筒に直した画像と床の境界（JPEG、確認用）

カメラの高さが距離の精度を決める（d = h / tan 俯角）。LiDAR の壁までの距離から推定した高さを
ときどきログに出すので、それを theta_height に入れる。動作指令は一切出さない。
"""
import glob
import math
import os
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, LaserScan, PointCloud2
from std_msgs.msg import Header
import tf2_ros

from cugo_perception import theta
from cugo_perception.cliff_node import make_cloud

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


def find_theta_device() -> Optional[str]:
    """/sys の名前に THETA を含む /dev/video* を探す。"""
    for dev in sorted(glob.glob('/dev/video*')):
        name_file = '/sys/class/video4linux/%s/name' % os.path.basename(dev)
        try:
            with open(name_file) as f:
                if 'theta' in f.read().lower():
                    return dev
        except OSError:
            continue
    return None


class ThetaFloor(Node):
    def __init__(self):
        super().__init__('theta_floor')
        p = self.declare_parameter
        self.device = str(p('device', '').value)  # 空なら名前で探す
        self.cap_w = int(p('capture_width', 1280).value)
        self.cap_h = int(p('capture_height', 720).value)
        self.rate = float(p('rate', 2.0).value)
        self.cam_height = float(p('theta_height', 0.5).value)       # 床からレンズまで [m]
        self.offset = (float(p('offset_x', 0.0).value), float(p('offset_y', 0.0).value))
        self.front = str(p('front_lens', 'left').value)              # 画面の左右どちらの円が前か
        self.fov = float(p('fov_deg', 195.0).value)
        self.roll = (math.radians(float(p('roll_left_deg', 0.0).value)),
                     math.radians(float(p('roll_right_deg', 0.0).value)))
        self.yaw_offset = math.radians(float(p('yaw_offset_deg', 0.0).value))
        self.eq_w = int(p('equirect_width', 720).value)
        self.eq_h = int(p('equirect_height', 360).value)
        self.near = float(p('near', 0.6).value)
        self.max_range = float(p('max_range', 5.0).value)
        self.sector = math.radians(float(p('sector_deg', 100.0).value))
        self.color_k = float(p('color_k', 4.0).value)
        self.clip_lidar = bool(p('clip_to_lidar', True).value)
        self.base_frame = str(p('base_frame', 'base_footprint').value)
        self.publish_image = bool(p('publish_image', True).value)

        self.pub_floor = self.create_publisher(PointCloud2, '/theta/floor', 5)
        self.pub_scan = self.create_publisher(LaserScan, '/theta/scan', 5)
        self.pub_img = self.create_publisher(CompressedImage, '/theta/image/compressed', 2)
        self.scan: Optional[LaserScan] = None
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._lock = threading.Lock()
        self._frame = None
        self._stamp = None
        self._stop = threading.Event()
        self._maps = None
        self._height_samples = []
        self._last_height_log = 0.0
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.create_timer(1.0 / max(self.rate, 0.1), self._tick)
        self.get_logger().info('THETA 床検出: 高さ %.2f m、前方 ±%.0f°、最大 %.1f m → /theta/floor'
                               % (self.cam_height, math.degrees(self.sector), self.max_range))

    # ---- 取り込み（別スレッド。常に最新の 1 枚だけ持つ）----
    def _open(self):
        dev = self.device or find_theta_device()
        if not dev or cv2 is None:
            return None
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cap_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cap_h)
        if not cap.isOpened():
            return None
        self.get_logger().info('THETA を開きました: %s' % dev)
        return cap

    def _read_loop(self):
        cap = None
        warned = 0.0
        while not self._stop.is_set():
            if cap is None:
                cap = self._open()
                if cap is None:
                    if time.monotonic() - warned > 30.0:
                        warned = time.monotonic()
                        self.get_logger().warn('THETA が見つかりません（ライブストリーミングモード'
                                               '（LIVE ランプ）か確認）')
                    self._stop.wait(2.0)
                    continue
            ok, frame = cap.read()
            if not ok or frame is None:
                cap.release()
                cap = None
                self._stop.wait(1.0)
                continue
            with self._lock:
                self._frame = frame
                self._stamp = self.get_clock().now()
        if cap is not None:
            cap.release()

    def _on_scan(self, msg: LaserScan):
        self.scan = msg

    # ---- 処理 ----
    def _build_maps(self, frame):
        circles = theta.find_lens_circles(frame)
        left, right = circles
        front_c, back_c = (left, right) if self.front == 'left' else (right, left)
        roll_f, roll_b = self.roll if self.front == 'left' else self.roll[::-1]
        lenses = [theta.Lens(*front_c, fov_deg=self.fov, axis_yaw=self.yaw_offset, roll=roll_f),
                  theta.Lens(*back_c, fov_deg=self.fov, axis_yaw=self.yaw_offset + math.pi,
                             roll=roll_b)]
        self.get_logger().info('レンズの円: 前 (%.0f, %.0f, r=%.0f) 後 (%.0f, %.0f, r=%.0f)'
                               % (*front_c, *back_c))
        self._maps = theta.equirect_maps(lenses, self.eq_w, self.eq_h)
        self._valid = self._maps[0] >= 0

    def _lidar_in_base(self):
        """LiDAR の (距離, base_footprint での方位)。無ければ None。"""
        s = self.scan
        if s is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, s.header.frame_id, Time())
        except tf2_ros.TransformException:
            return None
        q = tf.transform.rotation
        # レーザーが上下逆さまでないことは URDF で決まっている（z 軸まわりの回転だけを見る）
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        r = np.asarray(s.ranges, float)
        b = s.angle_min + np.arange(len(r)) * s.angle_increment + yaw
        return r, (b + math.pi) % (2 * math.pi) - math.pi

    def _tick(self):
        with self._lock:
            frame, stamp = self._frame, self._stamp
            self._frame = None
        if frame is None:
            return
        if self._maps is None:
            self._build_maps(frame)
        eq = theta.to_equirect(frame, self._maps)
        lab = theta.to_lab(eq)
        ranges, ok = theta.floor_ranges(lab, self.cam_height, near=self.near,
                                        max_range=self.max_range, k=self.color_k,
                                        valid=self._valid)
        b = theta.bearings(self.eq_w)
        lidar = self._lidar_in_base() if self.clip_lidar else None
        used = ranges
        if lidar is not None:
            used = theta.clip_to_lidar(ranges, b, lidar[0], lidar[1])
            self._check_height(ranges, b, lidar)
        header = Header()
        header.stamp = stamp.to_msg()
        header.frame_id = self.base_frame
        pts2 = theta.free_points(used, b, near=self.near, sector=self.sector, offset=self.offset)
        pts = np.column_stack([pts2, np.zeros(len(pts2))]) if len(pts2) else np.zeros((0, 3))
        self.pub_floor.publish(make_cloud(header, pts))
        self._publish_scan(header, used, b)
        if self.publish_image and self.pub_img.get_subscription_count() > 0:
            self._publish_image(header, eq, used)

    def _check_height(self, ranges, b, lidar):
        """壁の根元（カメラで床が終わる所）と LiDAR の距離が合う高さを推定してログに出す。"""
        lr, lb = lidar
        idx = np.searchsorted(np.sort(lb), b)
        order = np.argsort(lb)
        idx = np.clip(idx, 0, len(lb) - 1)
        lidar_at = lr[order][idx]
        hit = (ranges > self.near) & (ranges < self.max_range - 0.1) & np.isfinite(lidar_at)
        if hit.sum() < 10:
            return
        dep = theta.depression_for_range(ranges[hit], self.cam_height)
        h = theta.estimate_height(dep, lidar_at[hit])
        if h is not None:
            self._height_samples = (self._height_samples + [h])[-60:]
        now = time.monotonic()
        if now - self._last_height_log > 60.0 and len(self._height_samples) >= 10:
            self._last_height_log = now
            est = float(np.median(self._height_samples))
            self.get_logger().info('THETA の高さの推定 %.2f m（設定 %.2f m）。'
                                   '大きく違えば theta_height を直す' % (est, self.cam_height))

    def _publish_scan(self, header, ranges, b):
        order = np.argsort(b)
        msg = LaserScan()
        msg.header = header
        msg.angle_min = float(b[order][0])
        msg.angle_max = float(b[order][-1])
        msg.angle_increment = float(2 * math.pi / len(b))
        msg.range_min = float(self.near)
        msg.range_max = float(self.max_range)
        msg.ranges = [float(r) if r > self.near else float('nan') for r in ranges[order]]
        self.pub_scan.publish(msg)

    def _publish_image(self, header, eq, ranges):
        img = eq.copy()
        dep = theta.depression_for_range(np.maximum(ranges, 1e-3), self.cam_height)
        rows = ((math.pi / 2 + dep) / math.pi * self.eq_h).astype(int).clip(0, self.eq_h - 1)
        for c, r in enumerate(rows):
            if ranges[c] > self.near:
                img[r, c] = (0, 255, 0)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            msg = CompressedImage()
            msg.header = header
            msg.format = 'jpeg'
            msg.data = jpg.tobytes()
            self.pub_img.publish(msg)

    def destroy_node(self):
        self._stop.set()
        self._reader.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThetaFloor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
