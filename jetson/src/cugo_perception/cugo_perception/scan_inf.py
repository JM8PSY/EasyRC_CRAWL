"""LiDAR の「反射なし」を 0 から +inf に直す（REP 117）。

YDLIDAR の ROS 2 ドライバ（humble 版）は反射が返らなかった方向を 0 で出す。
invalid_range_is_inf パラメータは宣言だけで使われていない。
Nav2 の ObstacleLayer は range_min 未満の値（0）を捨てるので、その方向では障害物が消えず、
人が通り過ぎた跡などが「幽霊の障害物」として残る。
+inf にすると、ObstacleLayer（inf_is_valid: true）が「その方向は最大距離まで何もない」として
raytrace_max_range まで障害物を消す。slam_toolbox・rf2o・AMCL・Web UI は inf を扱える。

/scan_raw（ドライバ）→ /scan（このノード）
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

INF = float('inf')


def fix_ranges(ranges, range_min: float = 0.0):
    """0 以下・NaN を +inf にする。0 < r < range_min（近すぎる）は -inf（REP 117）。"""
    out = []
    for r in ranges:
        if r != r or r <= 0.0:          # NaN または 0（反射なし）
            out.append(INF)
        elif r < range_min:
            out.append(-INF)
        else:
            out.append(r)
    return out


class ScanInf(Node):
    def __init__(self):
        super().__init__('scan_inf')
        # RELIABLE で出すと BEST_EFFORT / RELIABLE どちらの購読者ともつながる
        self.pub = self.create_publisher(
            LaserScan, 'scan', QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(LaserScan, 'scan_raw', self._on_scan, qos_profile_sensor_data)
        self.get_logger().info('scan_raw → scan（反射なし 0 → +inf）')

    def _on_scan(self, msg: LaserScan):
        msg.ranges = fix_ranges(msg.ranges, msg.range_min)
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
