"""下り階段・段差落ち（崖）検出ノード。

D455 の点群を base_footprint 座標に変換して cliff.detect() にかけ、
- /cliff/points: 崖セルの点（ObstacleLayer の marking 用）
- /cliff/floor : 床が見えたセルの点（ObstacleLayer の clearing 用）
を base_footprint 座標の PointCloud2（xyz float32）で出す。空の点群も毎回出す
（崖が無くなったことを clearing に伝えるため）。動作指令は一切出さない。
"""
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import tf2_ros

from cugo_perception import cliff

_XYZ_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


def read_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 → (N, 3) float。NaN は除く。古い sensor_msgs_py にも対応。"""
    if hasattr(point_cloud2, 'read_points_numpy'):
        return point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    if isinstance(pts, np.ndarray) and pts.dtype.names:
        return np.column_stack([pts['x'], pts['y'], pts['z']])
    return np.array([tuple(p) for p in pts], dtype=np.float64).reshape(-1, 3)


def make_cloud(header, pts: np.ndarray) -> PointCloud2:
    """(N, 3) → xyz float32 の PointCloud2（N = 0 も可）。"""
    data = np.ascontiguousarray(pts, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(data)
    msg.fields = _XYZ_FIELDS
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(data)
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


class CliffDetector(Node):
    def __init__(self):
        super().__init__('cliff_detector')
        p = self.declare_parameter
        self.cloud_topic = p('cloud_topic', '/camera/camera/depth/color/points').value
        self.base_frame = p('base_frame', 'base_footprint').value
        rate = float(p('rate', 5.0).value)
        self.min_period = 1.0 / rate if rate > 0 else 0.0
        # True: 低い点を視線に沿って床の高さ（z = 0）の位置へ移してから印を付ける（cliff.py 参照）
        self.project = bool(p('project_to_floor', True).value)
        # True: 毎フレーム近くの床に平面を当て、その平面からの高さで判定する（傾き・調整ずれ対策）
        self.level = bool(p('fit_floor', True).value)
        self.params = {
            'cell': float(p('cell', 0.05).value),
            'drop_threshold': float(p('drop_threshold', 0.05).value),
            # 遠いほど閾値を深くする [m/m]（光沢床・遠距離の深度誤差対策。cliff.detect 参照）
            'drop_per_meter': float(p('drop_per_meter', 0.05).value),
            # 周り 8 セルに崖セルがこれ以上ある崖セルだけ残す（ぽつんと出る誤検出を消す）
            'min_neighbors': int(p('min_neighbors', 2).value),
            'floor_band': float(p('floor_band', 0.03).value),
            'min_points': int(p('min_points', 5).value),
            'x_range': (float(p('x_min', 0.2).value), float(p('x_max', 2.0).value)),
            'y_abs_max': float(p('y_abs_max', 1.5).value),
            'mark_z': float(p('mark_z', 0.1).value),
        }

        self.tf_buffer = tf2_ros.Buffer()
        # 別スレッドで TF を受け取る（コールバック内の lookup の待ち時間に TF が届くように）
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.pub_marks = self.create_publisher(PointCloud2, '/cliff/points', 10)
        self.pub_clears = self.create_publisher(PointCloud2, '/cliff/floor', 10)
        self.create_subscription(PointCloud2, self.cloud_topic, self._on_cloud,
                                 qos_profile_sensor_data)
        self._last = 0.0
        self.get_logger().info('崖検出: %s → /cliff/points, /cliff/floor（%s, %.1f Hz）'
                               % (self.cloud_topic, self.base_frame, rate))

    def _lookup(self, msg):
        frame = msg.header.frame_id
        try:
            return self.tf_buffer.lookup_transform(self.base_frame, frame,
                                                   Time.from_msg(msg.header.stamp),
                                                   timeout=Duration(seconds=0.05))
        except tf2_ros.TransformException:
            pass
        try:
            return self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn('TF %s ← %s が取れないのでフレームを捨てます: %s'
                                   % (self.base_frame, frame, e), throttle_duration_sec=5.0)
            return None

    def _on_cloud(self, msg: PointCloud2):
        now = time.monotonic()
        if now - self._last < self.min_period:
            return  # rate を超える分は捨てる
        self._last = now

        tf = self._lookup(msg)
        if tf is None:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        origin = (t.x, t.y, t.z)
        pts = cliff.transform_points(read_xyz(msg), origin, (q.x, q.y, q.z, q.w))
        if self.level:
            pts, origin, plane = cliff.level_to_floor(pts, origin)
            self.get_logger().debug('floor plane a=%.3f b=%.3f c=%.3f' % plane)
        marks, clears = cliff.detect(pts, camera_origin=origin if self.project else None,
                                     **self.params)

        header = msg.header
        header.frame_id = self.base_frame
        self.pub_marks.publish(make_cloud(header, marks))
        self.pub_clears.publish(make_cloud(header, clears))
        if len(marks):
            self.get_logger().info('崖セル %d 個（床セル %d 個）' % (len(marks), len(clears)),
                                   throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = CliffDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # systemd の SIGINT で静かに終わる
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
