"""CuGo Web UI ノード。

aiohttp で静的ファイル（web/）と WebSocket（/ws）を配信し、ROS 2 とつなぐ。

- 手動操縦: ブラウザの {"t":"drive"} → /cmd_vel_teleop（twist_mux で Nav2 より優先）
  - 指令が drive_timeout（0.3s）途絶えたら 0 を数回送って配信をやめる（Nav2 に制御を戻す）
- 停止: {"t":"stop"} → 0 を送り、Nav2 のゴールを取り消す
- 目的地: {"t":"goal"} → Nav2 の navigate_to_pose アクション
- 地図保存: {"t":"save_map"} → slam_toolbox の save_map（~/maps/<name>.yaml / .pgm）
- 配信: 地図（PNG、変化時・最大 1Hz）、ロボット姿勢・モード・ナビ状態（5Hz）、スキャン（5Hz）、経路

rclpy は別スレッドの executor で回し、aiohttp は main スレッドの asyncio で動かす。
ROS のコールバックは最新値を保存するだけにして、送信は asyncio 側のループがまとめて行う。
"""
import asyncio
import json
import math
import os
import threading
import time
from typing import Optional, Set

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros

from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory

from cugo_web.geometry import (clamp_drive, occupancy_to_png, path_to_points, png_b64,
                               quat_from_yaw, scan_to_points, valid_map_name, yaw_from_quat)

try:
    from slam_toolbox.srv import SaveMap
except ImportError:  # slam_toolbox が無い環境でも UI 自体は動かす
    SaveMap = None

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
        self.host = str(p('host', '0.0.0.0').value)
        self.web_dir = str(p('web_dir', '').value) or os.path.join(
            get_package_share_directory('cugo_web'), 'web')
        self.map_frame = str(p('map_frame', 'map').value)
        self.base_frame = str(p('base_frame', 'base_footprint').value)
        self.max_v = float(p('max_linear', 0.4).value)
        self.max_w = float(p('max_angular', 1.5).value)
        self.drive_timeout = float(p('drive_timeout', 0.3).value)
        self.map_dir = os.path.expanduser(str(p('map_dir', '~/maps').value))

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
        # ナビ
        self.nav_state = 'idle'
        self.goal: Optional[dict] = None
        self._goal_handle = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, 'cugo/mode', self._on_mode, latched)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, 'plan', self._on_plan, 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.save_client = (self.create_client(SaveMap, 'slam_toolbox/save_map')
                            if SaveMap else None)
        self.create_timer(0.05, self._drive_tick)  # 20Hz
        self._last_map_encode = 0.0
        self._pending_map: Optional[OccupancyGrid] = None
        self.create_timer(0.5, self._encode_map_tick)

    # ---------------- 手動操縦 ----------------
    def set_drive(self, v, w):
        v, w = clamp_drive(v, w, self.max_v, self.max_w)
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

    # ---------------- ナビ ----------------
    def send_goal(self, x, y, yaw, notify):
        try:
            x, y, yaw = float(x), float(y), float(yaw)
        except (TypeError, ValueError):
            notify('error', '目的地の値が不正です')
            return
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            notify('error', '目的地の値が不正です')
            return
        if not self.nav_client.server_is_ready():
            notify('error', 'Nav2 が起動していません')
            return
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = x, y
        qx, qy, qz, qw = quat_from_yaw(yaw)
        ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
        ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
        goal.pose = ps
        self.goal = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}
        self.nav_state = 'active'
        future = self.nav_client.send_goal_async(goal)

        def on_accepted(fut):
            handle = fut.result()
            if handle is None or not handle.accepted:
                self.nav_state = 'aborted'
                notify('error', '目的地が受け付けられませんでした')
                return
            self._goal_handle = handle
            notify('info', '目的地へ移動します')

            def on_result(rfut):
                if self._goal_handle is not handle:
                    return  # 新しいゴールに置き換わった
                status = rfut.result().status
                self.nav_state = STATUS_NAMES.get(status, 'aborted')
                self._goal_handle = None
                notify({'succeeded': 'info'}.get(self.nav_state, 'warn'),
                       {'succeeded': '目的地に到着しました',
                        'canceled': '移動を中止しました'}.get(self.nav_state,
                                                             '移動に失敗しました'))
            handle.get_result_async().add_done_callback(on_result)
        future.add_done_callback(on_accepted)

    def cancel_goal(self):
        handle = self._goal_handle
        if handle is not None:
            handle.cancel_goal_async()

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
                notify('info', '保存しました: %s.yaml' % req.name.data)
            else:
                notify('error', '地図の保存に失敗しました')
        self.save_client.call_async(req).add_done_callback(done)


# ---------------- Web サーバー ----------------
class WebServer:
    def __init__(self, node: CugoWebNode):
        self.node = node
        self.clients: Set[web.WebSocketResponse] = set()
        self.loop: Optional[asyncio.AbstractEventLoop] = None

    def notify(self, level, text):
        """ROS スレッドからも呼べる通知（全クライアントへ）。"""
        if self.loop is None:
            return
        msg = {'t': 'msg', 'level': level, 'text': text}
        asyncio.run_coroutine_threadsafe(self.broadcast(msg), self.loop)

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
        sent = {'map': node.map_version, 'scan': -1, 'plan': -1}
        while True:
            await asyncio.sleep(0.2)
            if not self.clients:
                continue
            await self.broadcast({
                't': 'state', 'mode': node.mode, 'pose': node.robot_pose(),
                'nav': node.nav_state, 'goal': node.goal, 'teleop': node.teleop_active,
            })
            if node.scan_version != sent['scan']:
                sent['scan'] = node.scan_version
                await self.broadcast({'t': 'scan', 'pts': node.scan_pts})
            if node.plan_version != sent['plan']:
                sent['plan'] = node.plan_version
                await self.broadcast({'t': 'plan', 'pts': node.plan_pts})
            if node.map_version != sent['map'] and node.map_msg is not None:
                sent['map'] = node.map_version
                await self.broadcast(node.map_msg)

    async def run(self):
        self.loop = asyncio.get_running_loop()
        app = web.Application()
        app.router.add_get('/', self.index)
        app.router.add_get('/ws', self.ws_handler)
        # --symlink-install だと web/ の中身がシンボリックリンクになるので follow_symlinks が必要
        app.router.add_static('/', self.node.web_dir, show_index=False, follow_symlinks=True)
        runner = web.AppRunner(app, access_log=None)
        await runner.setup()
        site = web.TCPSite(runner, self.node.host, self.node.port)
        await site.start()
        self.node.get_logger().info(
            'web UI: http://<JetsonのIP>:%d/  (static: %s)' % (self.node.port, self.node.web_dir))
        try:
            await self.publisher()
        finally:
            for ws in list(self.clients):
                await ws.close()
            await runner.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = CugoWebNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    server = WebServer(node)
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish(0.0, 0.0)  # 念のため停止指令
        except Exception:  # noqa: BLE001
            pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
