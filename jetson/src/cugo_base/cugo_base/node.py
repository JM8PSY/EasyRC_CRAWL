"""cugo_base_node: ROS 2 /cmd_vel ⇔ ESP32（USB シリアル、プロトコル v1）のブリッジ。

- /cmd_vel（Twist、use_stamped=true なら TwistStamped）→ 運動学 → ``$D`` を ``rate`` Hz で送り続ける
  （cmd_timeout を過ぎたら 0。ESP32 側のウォッチドッグのハートビートを兼ねる）
- ESP32 の ``$T`` → /wheel_odom（出力 duty から逆算して積分、TF なし）、/cugo/mode、/diagnostics
- シリアル断・未接続は 1 秒ごとに再接続。終了時は ``$S``
"""
from __future__ import annotations

import math
import signal
import threading
import time
from typing import Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper, Updater
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from cugo_base.kinematics import (DiffDriveParams, OdometryIntegrator, duty_to_speed,
                                  twist_to_duty)
from cugo_base.protocol import Status, format_drive, format_stop, parse_line, parse_status
from cugo_base.serial_link import SerialLink

DISCONNECTED = 'disconnected'
MODE_REPUBLISH_PERIOD = 1.0  # [s] 変化がなくても再配信する間隔

# エンコーダなしの指令ベース推定なので大きめ。EKF は vx, vyaw のみ使う想定
_BIG = 1e6
POSE_COVARIANCE = [
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, _BIG, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, _BIG, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, _BIG, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
]
TWIST_COVARIANCE = [
    0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, _BIG, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, _BIG, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, _BIG, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.2,
]


class CugoBaseNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('cugo_base_node', **kwargs)
        self._declare_parameters()
        self._p = self._read_kinematics()
        self._lock = threading.Lock()
        self._cmd: Tuple[float, float] = (0.0, 0.0)
        self._cmd_time = -math.inf
        self._sent: Tuple[float, float] = (0.0, 0.0)
        self._status: Optional[Status] = None
        self._status_time = -math.inf
        self._rx_ok = 0
        self._rx_bad = 0
        self._mode_pub_value = ''
        self._mode_pub_time = -math.inf
        self._last_tick = time.monotonic()
        self._closed = False
        self._odom = OdometryIntegrator(self._p.track_width)

        self._setup_ros_interfaces()
        self._link = SerialLink(self._param('port'), int(self._param('baud')),
                                self._on_line, float(self._param('reconnect_interval')),
                                on_event=lambda msg: self.get_logger().info(msg),
                                silence_timeout=float(self._param('silence_reset')))
        self._link.start()
        self.get_logger().info(
            'cugo_base_node: port=%s baud=%d rate=%.1fHz max_duty=%.2f'
            % (self._link.port, self._link.baud, self._rate, self._p.max_duty))

    # ---- setup ----

    def _declare_parameters(self) -> None:
        defaults = {
            'port': '/dev/cugo_esp32',
            'baud': 115200,
            'track_width': 0.40,
            'max_wheel_speed': 1.0,
            'deadband': 0.08,
            'min_speed': 0.01,
            'max_duty': 0.5,
            'cmd_timeout': 0.5,
            'rate': 30.0,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'use_stamped': False,
            # 仕様書にない補助パラメータ
            'status_timeout': 0.5,      # $T がこの時間途絶えたら disconnected
            'reconnect_interval': 1.0,  # 再接続の間隔 [s]
            'silence_reset': 3.0,       # ESP32 から何も届かないときにリセットするまで [s]（0 で無効）
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _read_kinematics(self) -> DiffDriveParams:
        p = DiffDriveParams(
            track_width=float(self._param('track_width')),
            max_wheel_speed=float(self._param('max_wheel_speed')),
            deadband=float(self._param('deadband')),
            min_speed=float(self._param('min_speed')),
            max_duty=float(self._param('max_duty')),
        )
        if p.track_width <= 0 or p.max_wheel_speed <= 0 or not 0 <= p.deadband < 1:
            raise ValueError('invalid kinematics parameters: %r' % (p,))
        return p

    def _setup_ros_interfaces(self) -> None:
        self._rate = max(1.0, float(self._param('rate')))
        self._cmd_timeout = float(self._param('cmd_timeout'))
        self._status_timeout = float(self._param('status_timeout'))
        self._odom_frame = str(self._param('odom_frame'))
        self._base_frame = str(self._param('base_frame'))
        if bool(self._param('use_stamped')):
            self.create_subscription(TwistStamped, 'cmd_vel',
                                     lambda m: self._on_twist(m.twist), 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self._on_twist, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._mode_pub = self.create_publisher(String, 'cugo/mode', latched)
        self._odom_pub = self.create_publisher(Odometry, 'wheel_odom', 10)
        self._diag = Updater(self)
        self._diag.setHardwareID('cugo_esp32')
        self._diag.add('ESP32 link', self._diagnose)
        self._timer = self.create_timer(1.0 / self._rate, self._tick)

    # ---- callbacks ----

    def _on_twist(self, msg: Twist) -> None:
        with self._lock:
            self._cmd = (msg.linear.x, msg.angular.z)
            self._cmd_time = time.monotonic()

    def _on_line(self, line: str) -> None:
        """シリアル受信スレッドから呼ばれる。"""
        if not line.startswith('$'):
            self.get_logger().debug('ESP32: %s' % line)
            return
        status = parse_status(line)
        with self._lock:
            if status is None:
                if parse_line(line) is None:
                    self._rx_bad += 1
                return
            self._status = status
            self._status_time = time.monotonic()
            self._rx_ok += 1

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(now - self._last_tick, 1.0)
        self._last_tick = now
        duty = self._current_duty(now)
        if not self._closed:
            self._link.write(format_drive(*duty))
        self._sent = duty
        status = self._fresh_status(now)
        self._publish_mode(now, status.mode_name if status else DISCONNECTED)
        self._publish_odom(status, dt)

    # ---- helpers ----

    def _current_duty(self, now: float) -> Tuple[float, float]:
        with self._lock:
            cmd, age = self._cmd, now - self._cmd_time
        if age > self._cmd_timeout:
            return 0.0, 0.0
        return twist_to_duty(cmd[0], cmd[1], self._p)

    def _fresh_status(self, now: float) -> Optional[Status]:
        with self._lock:
            status, age = self._status, now - self._status_time
        if not self._link.connected or age > self._status_timeout:
            return None
        return status

    def _publish_mode(self, now: float, mode: str) -> None:
        if mode == self._mode_pub_value and now - self._mode_pub_time < MODE_REPUBLISH_PERIOD:
            return
        if mode != self._mode_pub_value:
            self.get_logger().info('mode: %s' % mode)
        self._mode_pub_value, self._mode_pub_time = mode, now
        self._mode_pub.publish(String(data=mode))

    def _publish_odom(self, status: Optional[Status], dt: float) -> None:
        if status is None:
            v_l = v_r = 0.0
        else:
            v_l, v_r = duty_to_speed(status.l, self._p), duty_to_speed(status.r, self._p)
        self._odom.update(v_l, v_r, dt)
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = self._odom.x
        msg.pose.pose.position.y = self._odom.y
        msg.pose.pose.orientation.z = math.sin(self._odom.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._odom.yaw / 2.0)
        msg.pose.covariance = POSE_COVARIANCE
        msg.twist.twist.linear.x = self._odom.v
        msg.twist.twist.angular.z = self._odom.w
        msg.twist.covariance = TWIST_COVARIANCE
        self._odom_pub.publish(msg)

    def _diagnose(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        now = time.monotonic()
        status = self._fresh_status(now)
        with self._lock:
            cmd_age, rx_ok, rx_bad = now - self._cmd_time, self._rx_ok, self._rx_bad
        if not self._link.connected:
            stat.summary(DiagnosticStatus.ERROR, 'シリアル未接続: %s' % self._link.last_error)
        elif status is None:
            stat.summary(DiagnosticStatus.WARN, 'ESP32 からステータス ($T) が届かない')
        else:
            stat.summary(DiagnosticStatus.OK, 'mode=%s' % status.mode_name)
        stat.add('port', self._link.port)
        stat.add('connected', str(self._link.connected))
        stat.add('connect_count', str(self._link.connect_count))
        stat.add('sent_duty_l', '%.3f' % self._sent[0])
        stat.add('sent_duty_r', '%.3f' % self._sent[1])
        stat.add('cmd_age_s', '%.2f' % cmd_age if math.isfinite(cmd_age) else 'never')
        stat.add('rx_status_lines', str(rx_ok))
        stat.add('rx_bad_lines', str(rx_bad))
        if status is not None:
            stat.add('mode', status.mode_name)
            stat.add('out_duty_l', '%.3f' % status.l)
            stat.add('out_duty_r', '%.3f' % status.r)
            stat.add('auto_max_duty', '%.2f' % status.cap)
            stat.add('auto_allowed', str(status.auto_allowed))
            stat.add('web_connected', str(status.web_connected))
            stat.add('jetson_active', str(status.jetson_active))
            stat.add('esp32_bad_lines', str(status.bad))
        return stat

    # ---- shutdown ----

    def close(self) -> None:
        """$S を送ってシリアルを閉じる（何度呼んでもよい）。"""
        if self._closed:
            return
        self._closed = True
        self._timer.cancel()
        if self._link.write(format_stop()):
            self.get_logger().info('sent $S')
        self._link.stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[CugoBaseNode] = None
    try:
        node = CugoBaseNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch は Ctrl+C を二重に送ることがあるので、後始末中の SIGINT は無視する
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
