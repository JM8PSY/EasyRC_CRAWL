"""cugo_base_node を pty の偽 ESP32 につないで動かす結合テスト（実機不要）。

安全のため:
- 実デバイス（/dev/ttyUSB*, /dev/cugo_esp32）は一切開かない。port は必ず pty を指定する
- 専用の ROS_DOMAIN_ID と名前空間で動かし、実機の cugo_base とトピックが混ざらないようにする
"""
import os
import select
import threading
import time
import tty
from typing import Callable, List, Optional

import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from cugo_base.node import CugoBaseNode  # noqa: E402
from cugo_base.protocol import LineAssembler, format_line, parse_line  # noqa: E402

TEST_DOMAIN_ID = 87
NS = '/cugo_base_test'


class FakeEsp:
    """pty のマスター側で ESP32 のふりをする。受けた BODY を記録し、$T を 20Hz で返す。"""

    def __init__(self) -> None:
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.bodies: List[str] = []
        self.garbage_lines = 0
        self.status: Optional[str] = 'T,A,0.264,0.264,0.50,5,0'
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> 'FakeEsp':
        # ブート ROM 風のゴミと日本語ログを先に流しておく
        os.write(self.master, bytes(range(128, 200)) + b'\r\n' + 'ESP32 起動\n'.encode())
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass

    def received(self) -> List[str]:
        with self._lock:
            return list(self.bodies)

    def _run(self) -> None:
        asm = LineAssembler()
        next_tx = time.monotonic()
        while not self._stop.is_set():
            ready, _, _ = select.select([self.master], [], [], 0.01)
            if ready:
                try:
                    data = os.read(self.master, 1024)
                except OSError:
                    return
                for line in asm.feed(data):
                    body = parse_line(line)
                    with self._lock:
                        if body is None:
                            self.garbage_lines += 1
                        else:
                            self.bodies.append(body)
            if self.status is not None and time.monotonic() >= next_tx:
                next_tx += 0.05
                try:
                    os.write(self.master, format_line(self.status))
                except OSError:
                    return


class Probe:
    def __init__(self, ctx) -> None:
        self.node = rclpy.create_node('cugo_base_probe', namespace=NS, context=ctx)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.modes: List[str] = []
        self.odoms: List[Odometry] = []
        self.node.create_subscription(String, 'cugo/mode',
                                      lambda m: self.modes.append(m.data), latched)
        self.node.create_subscription(Odometry, 'wheel_odom', self.odoms.append, 10)
        self.cmd_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)


@pytest.fixture
def ros():
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=TEST_DOMAIN_ID)
    executor = SingleThreadedExecutor(context=ctx)
    nodes = []

    def make_base(port: str, **params) -> CugoBaseNode:
        overrides = [Parameter('port', value=port), Parameter('cmd_timeout', value=0.3),
                     Parameter('status_timeout', value=0.5)]
        overrides += [Parameter(k, value=v) for k, v in params.items()]
        node = CugoBaseNode(namespace=NS, context=ctx, parameter_overrides=overrides)
        nodes.append(node)
        executor.add_node(node)
        return node

    probe = Probe(ctx)
    executor.add_node(probe.node)

    def spin_until(pred: Callable[[], bool], timeout: float = 3.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            if pred():
                return True
        return False

    yield make_base, probe, spin_until
    for node in nodes:
        node.close()
        node.destroy_node()
    probe.node.destroy_node()
    executor.shutdown()
    rclpy.shutdown(context=ctx)


@pytest.fixture
def esp():
    fake = FakeEsp().start()
    yield fake
    fake.stop()


def _drive_bodies(bodies: List[str]) -> List[str]:
    return [b for b in bodies if b.startswith('D,')]


def test_heartbeat_mode_and_odom(ros, esp):
    make_base, probe, spin_until = ros
    make_base(esp.path)
    assert spin_until(lambda: 'auto' in probe.modes)
    assert spin_until(lambda: len(_drive_bodies(esp.received())) >= 10)
    # /cmd_vel なし → 0 を送り続ける
    assert set(_drive_bodies(esp.received())) == {'D,0.000,0.000'}
    assert spin_until(lambda: len(probe.odoms) >= 10)
    odom = probe.odoms[-1]
    assert odom.header.frame_id == 'odom'
    assert odom.child_frame_id == 'base_footprint'
    # ESP の報告 0.264 → 0.2 m/s
    assert odom.twist.twist.linear.x == pytest.approx(0.2, abs=1e-6)
    assert odom.pose.pose.position.x > 0.0
    assert odom.twist.covariance[0] > 0.0


def test_cmd_vel_then_timeout(ros, esp):
    make_base, probe, spin_until = ros
    make_base(esp.path)
    assert spin_until(lambda: 'auto' in probe.modes)
    msg = Twist()
    msg.linear.x = 0.2

    def publishing() -> bool:
        probe.cmd_pub.publish(msg)
        return 'D,0.264,0.264' in esp.received()

    assert spin_until(publishing)
    n = len(esp.received())
    # 指令が途絶えると cmd_timeout 後に 0 に戻る
    assert spin_until(lambda: esp.received()[-1] == 'D,0.000,0.000'
                      and len(esp.received()) > n, timeout=2.0)


def test_cmd_vel_clamped_by_max_duty(ros, esp):
    make_base, probe, spin_until = ros
    make_base(esp.path, max_duty=0.3)
    msg = Twist()
    msg.linear.x = 5.0

    def publishing() -> bool:
        probe.cmd_pub.publish(msg)
        return 'D,0.300,0.300' in esp.received()

    assert spin_until(publishing)


def test_status_loss_reports_disconnected(ros, esp):
    make_base, probe, spin_until = ros
    make_base(esp.path)
    assert spin_until(lambda: 'auto' in probe.modes)
    esp.status = None
    assert spin_until(lambda: probe.modes[-1] == 'disconnected')


def test_mode_names_follow_esp(ros, esp):
    make_base, probe, spin_until = ros
    esp.status = 'T,M,0.000,0.000,0.50,2,0'
    make_base(esp.path)
    assert spin_until(lambda: probe.modes and probe.modes[-1] == 'manual')
    esp.status = 'T,X,0.000,0.000,0.50,2,0'
    assert spin_until(lambda: probe.modes[-1] == 'test')


def test_missing_port_then_reconnect(ros, esp, tmp_path):
    make_base, probe, spin_until = ros
    link = tmp_path / 'esp'
    make_base(str(link))
    assert spin_until(lambda: 'disconnected' in probe.modes)
    os.symlink(esp.path, link)
    assert spin_until(lambda: probe.modes[-1] == 'auto', timeout=4.0)


def test_serial_loss_recovers(ros, tmp_path):
    make_base, probe, spin_until = ros
    first = FakeEsp().start()
    link = tmp_path / 'esp'
    os.symlink(first.path, link)
    make_base(str(link))
    try:
        assert spin_until(lambda: 'auto' in probe.modes)
        first.stop()  # 抜かれた
        assert spin_until(lambda: probe.modes[-1] == 'disconnected')
        second = FakeEsp()
        second.status = 'T,I,0.000,0.000,0.50,1,0'
        second.start()
        os.unlink(link)
        os.symlink(second.path, link)
        assert spin_until(lambda: probe.modes[-1] == 'idle', timeout=4.0)
        second.stop()
    finally:
        first.stop()


def test_close_sends_stop(ros, esp):
    make_base, probe, spin_until = ros
    node = make_base(esp.path)
    assert spin_until(lambda: len(_drive_bodies(esp.received())) >= 3)
    node.close()
    assert spin_until(lambda: 'S' in esp.received(), timeout=1.0)
