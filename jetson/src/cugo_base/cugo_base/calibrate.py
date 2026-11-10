"""cugo_calibrate: 人が立ち会って行う速度キャリブレーション補助ツール。

ESP32 に直接 ``$D`` を送って一定 duty で直進・その場旋回させ、
外部オドメトリ（/odom_rf2o または /odometry/filtered）で実測した速度から
``max_wheel_speed`` / ``deadband`` / ``track_width`` の推奨値を表示する。

- 既定はドライラン（何をするか表示するだけ。シリアルも開かない）
- 実際に動かすには ``--yes`` と、起動後の確認入力が必要
- Ctrl+C でいつでも ``$S``（即時停止）
- cugo_base_node とはシリアルを取り合うので、cugo_base_node は止めておくこと
  （センサーと rf2o / EKF は動かしておく）
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from typing import List, Optional, Sequence, Tuple

from cugo_base.calibration import (format_suggestion, mean_speed, mean_yaw_rate,
                                   suggest_speed_params, track_width_from_spin)
from cugo_base.kinematics import DiffDriveParams
from cugo_base.protocol import Status, format_drive, format_stop, parse_status

SEND_PERIOD = 1.0 / 30.0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        prog='cugo_calibrate',
        description='CuGo の速度キャリブレーション（要立ち会い。既定はドライラン）')
    ap.add_argument('--yes', action='store_true',
                    help='実際にモーターを動かす（指定しなければドライラン）')
    ap.add_argument('--port', default='/dev/cugo_esp32')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--odom-topic', default='/odom_rf2o',
                    help='実測に使うオドメトリ（/odom_rf2o か /odometry/filtered）')
    ap.add_argument('--duties', default='0.12,0.18,0.25,0.32,0.40',
                    help='直進テストの duty（カンマ区切り、ESP32 の AUTO_MAX_DUTY 以下）')
    ap.add_argument('--duration', type=float, default=3.0, help='1 段あたりの走行時間 [s]')
    ap.add_argument('--settle', type=float, default=1.0,
                    help='加速制限のため測定から除く先頭の時間 [s]')
    ap.add_argument('--reverse', action='store_true', help='直進テストを後退で行う')
    ap.add_argument('--spin-duty', type=float, default=0.30, help='旋回テストの duty')
    ap.add_argument('--spin-duration', type=float, default=4.0, help='旋回テストの時間 [s]')
    ap.add_argument('--skip-straight', action='store_true')
    ap.add_argument('--skip-spin', action='store_true')
    ap.add_argument('--max-wheel-speed', type=float, default=None,
                    help='旋回テストだけ行う場合に使う既知の max_wheel_speed')
    ap.add_argument('--deadband', type=float, default=None,
                    help='旋回テストだけ行う場合に使う既知の deadband')
    args = ap.parse_args(argv)
    args.duty_list = [float(x) for x in args.duties.split(',') if x.strip()]
    for d in args.duty_list + [args.spin_duty]:
        if not 0.0 < d <= 1.0:
            ap.error('duty は 0 < duty <= 1.0: %s' % d)
    return args


def describe_plan(args: argparse.Namespace) -> List[str]:
    sign = -1.0 if args.reverse else 1.0
    lines = ['ポート %s（%d bps）に直接 $D を送り、%s で実測します。'
             % (args.port, args.baud, args.odom_topic),
             '各テストの後は停止し、Enter を押すまで待ちます（ロボットを戻してください）。']
    if not args.skip_straight:
        for d in args.duty_list:
            lines.append('  直進: $D,%.3f,%.3f を %.1f 秒' % (sign * d, sign * d, args.duration))
    if not args.skip_spin:
        lines.append('  旋回: $D,%.3f,%.3f を %.1f 秒（左回り）'
                     % (-args.spin_duty, args.spin_duty, args.spin_duration))
    return lines


class Esp:
    """ESP32 との直接通信（cugo_base_node を使わない）。"""

    def __init__(self, port: str, baud: int) -> None:
        from cugo_base.serial_link import SerialLink
        self.status: Optional[Status] = None
        self._lock = threading.Lock()
        self.link = SerialLink(port, baud, self._on_line)
        self.link.start()

    def _on_line(self, line: str) -> None:
        st = parse_status(line)
        if st is not None:
            with self._lock:
                self.status = st

    def latest(self) -> Optional[Status]:
        with self._lock:
            return self.status

    def drive(self, l: float, r: float) -> None:  # noqa: E741
        self.link.write(format_drive(l, r))

    def stop(self) -> None:
        for _ in range(3):
            self.link.write(format_stop())
            time.sleep(0.02)

    def close(self) -> None:
        self.stop()
        self.link.stop()


class OdomRecorder:
    def __init__(self, topic: str) -> None:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.signals import SignalHandlerOptions
        # Ctrl+C は素の KeyboardInterrupt にして、確実に $S を送る
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._rclpy = rclpy
        self.node = rclpy.create_node('cugo_calibrate')
        self.samples: List[Tuple[float, float, float, float]] = []  # t, x, y, yaw
        self.node.create_subscription(Odometry, topic, self._on_odom, 20)

    def _on_odom(self, msg) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self.samples.append((time.monotonic(), p.x, p.y, yaw))

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._rclpy.spin_once(self.node, timeout_sec=0.005)

    def node_running(self, name: str) -> bool:
        self.spin_for(1.0)  # discovery を待つ
        return name in self.node.get_node_names()

    def close(self) -> None:
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


def run_segment(esp: Esp, odom: OdomRecorder, l: float, r: float,  # noqa: E741
                duration: float) -> List[Tuple[float, float, float, float]]:
    """duration 秒 (l, r) を送り続け、その間のオドメトリを返す。最後に $S。"""
    odom.samples.clear()
    end = time.monotonic() + duration
    try:
        while time.monotonic() < end:
            esp.drive(l, r)
            odom.spin_for(SEND_PERIOD)
    finally:
        esp.stop()
    samples = list(odom.samples)
    odom.spin_for(0.5)
    return samples


def check_ready(esp: Esp, odom: OdomRecorder, max_duty: float, topic: str) -> Optional[str]:
    """動かす前の確認。問題があればメッセージを返す。"""
    odom.samples.clear()
    for _ in range(30):
        esp.drive(0.0, 0.0)
        odom.spin_for(0.1)
    st = esp.latest()
    if not esp.link.connected or st is None:
        return 'ESP32 からステータスが来ない（%s）' % esp.link.last_error
    if not st.auto_allowed:
        return 'ESP32 が自律許可になっていない（Web UI の「自律走行を許可」を確認）mode=%s' % st.mode_name
    if max_duty > st.cap + 1e-6:
        return 'duty %.2f が ESP32 の AUTO_MAX_DUTY %.2f を超えている' % (max_duty, st.cap)
    if not odom.samples:
        return 'オドメトリ %s が来ていない' % topic
    return None


def confirm(prompt: str) -> bool:
    try:
        return input(prompt).strip().lower() in ('y', 'yes')
    except EOFError:
        return False


def straight_test(args, esp: Esp, odom: OdomRecorder) -> List[Tuple[float, float]]:
    sign = -1.0 if args.reverse else 1.0
    results = []
    for d in args.duty_list:
        input('\n直進 duty=%.2f（%.1f 秒）。前方 %.1fm 以上空けて Enter（Ctrl+C で中止）'
              % (d, args.duration, args.duration * 1.0))
        samples = run_segment(esp, odom, sign * d, sign * d, args.duration)
        try:
            v = mean_speed([(t, x, y) for t, x, y, _ in samples], args.settle)
        except ValueError as exc:
            print('  測定できず: %s' % exc)
            continue
        print('  duty=%.2f → %.3f m/s' % (d, v))
        results.append((d, v))
    return results


def spin_test(args, esp: Esp, odom: OdomRecorder) -> Optional[float]:
    input('\n旋回 duty=%.2f（%.1f 秒、左回り）。周囲を空けて Enter（Ctrl+C で中止）'
          % (args.spin_duty, args.spin_duration))
    samples = run_segment(esp, odom, -args.spin_duty, args.spin_duty, args.spin_duration)
    try:
        w = mean_yaw_rate([(t, yaw) for t, _, _, yaw in samples], args.settle)
    except ValueError as exc:
        print('  測定できず: %s' % exc)
        return None
    print('  ω = %.3f rad/s' % w)
    return w


def run(args: argparse.Namespace) -> int:
    odom = OdomRecorder(args.odom_topic)
    esp: Optional[Esp] = None
    try:
        if odom.node_running('cugo_base_node'):
            print('cugo_base_node が動いています。止めてから実行してください。')
            return 1
        esp = Esp(args.port, args.baud)
        problem = check_ready(esp, odom, max(args.duty_list + [args.spin_duty]),
                              args.odom_topic)
        if problem:
            print('中止: %s' % problem)
            return 1
        if not confirm('\n本当にモーターを動かします。立ち会っていますか？ [y/N] '):
            print('中止しました')
            return 1
        return report(args, esp, odom)
    except KeyboardInterrupt:
        print('\nCtrl+C: 停止します')
        return 130
    finally:
        if esp is not None:
            esp.close()
        odom.close()


def report(args, esp: Esp, odom: OdomRecorder) -> int:
    p = DiffDriveParams()
    lines = []
    if not args.skip_straight:
        results = straight_test(args, esp, odom)
        try:
            mws, db = suggest_speed_params(results)
            p = DiffDriveParams(max_wheel_speed=mws, deadband=db)
            lines += ['max_wheel_speed: %.3f' % mws, 'deadband: %.3f' % db]
        except ValueError as exc:
            lines.append('max_wheel_speed / deadband: 推定できず（%s）' % exc)
    if args.max_wheel_speed is not None or args.deadband is not None:
        p = DiffDriveParams(
            max_wheel_speed=args.max_wheel_speed or p.max_wheel_speed,
            deadband=p.deadband if args.deadband is None else args.deadband)
    if not args.skip_spin:
        w = spin_test(args, esp, odom)
        if w is not None:
            try:
                lines.append('track_width: %.3f  # 実効値（横滑り込み）'
                             % track_width_from_spin(args.spin_duty, w, p))
            except ValueError as exc:
                lines.append('track_width: 推定できず（%s）' % exc)
    print(format_suggestion(lines))
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    print('\n'.join(describe_plan(args)))
    if not args.yes:
        print('\nドライランです（何も送信していません）。実行するには --yes を付けてください。')
        return 0
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
