"""LiDAR が車体に遮られている角度を調べる（読むだけ。何も publish しない）。

  ros2 run cugo_perception scan_sectors            # /scan_raw を 10 秒
  ros2 run cugo_perception scan_sectors --ros-args -p topic:=/scan_raw -p seconds:=20

ロボットを開けた場所に置き、できればゆっくり回しながら実行する。どの向きでもずっと 0（反射なし）
または 0.3 m 未満が返る角度は、車体（支柱・カメラ・配線など）に遮られている可能性が高い。
結果を scan_inf の ignore_sectors に入れる（例の行をそのまま launch/パラメータにコピーできる）。
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from cugo_perception.scan_utils import blocked_sectors


class ScanSectors(Node):
    def __init__(self):
        super().__init__('scan_sectors')
        self.topic = self.declare_parameter('topic', '/scan_raw').value
        self.seconds = float(self.declare_parameter('seconds', 10.0).value)
        self.near = float(self.declare_parameter('near', 0.3).value)
        self.count = 0
        self.bad = None
        self.angles = None
        self.create_subscription(LaserScan, self.topic, self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg):
        n = len(msg.ranges)
        if self.bad is None or len(self.bad) != n:
            self.bad = [0] * n
            self.count = 0
            self.angles = [math.degrees(msg.angle_min + i * msg.angle_increment) for i in range(n)]
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0 or r < self.near:
                self.bad[i] += 1
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ScanSectors()
    end = time.monotonic() + node.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    if not node.count:
        print('%s からスキャンが来ませんでした' % node.topic)
    else:
        rates = [b / node.count for b in node.bad]
        secs = blocked_sectors(rates, node.angles)
        print('スキャン %d 回。ずっと無効（または %.1f m 未満）の角度 [deg]: %s'
              % (node.count, node.near, secs or 'なし'))
        if secs:
            flat = ', '.join('%.1f, %.1f' % s for s in secs)
            print('scan_inf の設定例:  ignore_sectors: [%s]' % flat)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
