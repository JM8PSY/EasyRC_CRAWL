"""LiDAR の「反射なし」を 0 から +inf に直す（REP 117）。

YDLIDAR の ROS 2 ドライバ（humble 版）は反射が返らなかった方向を 0 で出す。
invalid_range_is_inf パラメータは宣言だけで使われていない。
Nav2 の ObstacleLayer は range_min 未満の値（0）を捨てるので、その方向では障害物が消えず、
人が通り過ぎた跡などが「幽霊の障害物」として残る。
+inf にすると、ObstacleLayer（inf_is_valid: true）が「その方向は最大距離まで何もない」として
raytrace_max_range まで障害物を消す。slam_toolbox・rf2o・AMCL・Web UI は inf を扱える。

/scan_raw（ドライバ）→ /scan（このノード）

車体（柱・カメラの支柱など）が LiDAR を遮る方向も 0 で返る。これを +inf にすると
「遮られた先まで何もない」として壁や障害物を消してしまう。そこで ignore_sectors で指定した角度
（laser_frame、度、[開始, 終了, 開始, 終了, ...]）は NaN（データなし: 消しも書きもしない）にする。
遮られる角度は scan_sectors（ros2 run cugo_perception scan_sectors）で調べられる。
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from cugo_perception.scan_utils import fix_ranges, sector_mask


class ScanInf(Node):
    def __init__(self):
        super().__init__('scan_inf')
        # RELIABLE で出すと BEST_EFFORT / RELIABLE どちらの購読者ともつながる
        self.pub = self.create_publisher(
            LaserScan, 'scan', QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(LaserScan, 'scan_raw', self._on_scan, qos_profile_sensor_data)
        # 車体に遮られる角度 [deg]（laser_frame）。[開始, 終了, ...]。空なら無し
        self.sectors = list(self.declare_parameter('ignore_sectors', [0.0]).value)
        if len(self.sectors) < 2:
            self.sectors = []
        self._mask_key = None
        self._mask = None
        self.get_logger().info('scan_raw → scan（反射なし 0 → +inf、無視する角度 %s）'
                               % (self.sectors or 'なし'))

    def _on_scan(self, msg: LaserScan):
        ignore = None
        if self.sectors:
            key = (len(msg.ranges), msg.angle_min, msg.angle_increment)
            if key != self._mask_key:
                self._mask_key = key
                self._mask = sector_mask(len(msg.ranges), msg.angle_min, msg.angle_increment,
                                         self.sectors)
            ignore = self._mask
        msg.ranges = fix_ranges(msg.ranges, msg.range_min, ignore)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanInf()
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
