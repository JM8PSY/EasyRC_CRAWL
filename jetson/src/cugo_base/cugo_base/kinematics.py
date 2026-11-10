"""差動二輪（クローラー）の運動学とオドメトリ積分（ROS 非依存）。

duty ⇔ 車輪速度のモデル（エンコーダなし、キャリブレーションで決める）::

    |duty| = deadband + |v| / max_wheel_speed * (1 - deadband)   (|v| >= min_speed)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class DiffDriveParams:
    track_width: float = 0.40      # 左右クローラー中心間距離 [m]
    max_wheel_speed: float = 1.0   # duty 1.0 時の車輪速度 [m/s]
    deadband: float = 0.08         # これ以下の duty では動かない
    min_speed: float = 0.01        # これ未満の車輪速度指令は 0 にする [m/s]
    max_duty: float = 0.5          # 出力 duty の上限


def _sign(x: float) -> float:
    return -1.0 if x < 0 else 1.0


def speed_to_duty(v: float, p: DiffDriveParams) -> float:
    """車輪速度 [m/s] → duty（上限クランプ前）。"""
    if abs(v) < p.min_speed:
        return 0.0
    return _sign(v) * (p.deadband + abs(v) / p.max_wheel_speed * (1.0 - p.deadband))


def duty_to_speed(duty: float, p: DiffDriveParams) -> float:
    """duty → 車輪速度 [m/s]（odom 用の逆変換）。"""
    if abs(duty) <= p.deadband:
        return 0.0
    return _sign(duty) * (abs(duty) - p.deadband) / (1.0 - p.deadband) * p.max_wheel_speed


def _max_speed_for_duty(duty_limit: float, p: DiffDriveParams) -> float:
    """duty_limit で出せる最大の車輪速度。"""
    return max(0.0, (duty_limit - p.deadband) / (1.0 - p.deadband) * p.max_wheel_speed)


def twist_to_duty(v: float, w: float, p: DiffDriveParams) -> Tuple[float, float]:
    """(v [m/s], ω [rad/s]) → (duty_l, duty_r)。

    片輪が max_duty を超える場合は、左右の車輪速度の比（＝曲率）を保ったまま
    縮小する。deadband があるため duty の比ではなく速度の比を保つ。
    """
    if not (math.isfinite(v) and math.isfinite(w)):
        return 0.0, 0.0
    v_l = v - w * p.track_width / 2.0
    v_r = v + w * p.track_width / 2.0
    limit = min(abs(p.max_duty), 1.0)
    if limit <= p.deadband:
        return 0.0, 0.0
    peak = max(abs(v_l), abs(v_r))
    if peak > 0.0 and speed_to_duty(peak, p) > limit:
        scale = _max_speed_for_duty(limit, p) / peak
        v_l, v_r = v_l * scale, v_r * scale
    d_l = max(-limit, min(limit, speed_to_duty(v_l, p)))
    d_r = max(-limit, min(limit, speed_to_duty(v_r, p)))
    return d_l, d_r


def duty_to_twist(d_l: float, d_r: float, p: DiffDriveParams) -> Tuple[float, float]:
    """(duty_l, duty_r) → (v, ω)。"""
    v_l, v_r = duty_to_speed(d_l, p), duty_to_speed(d_r, p)
    return (v_l + v_r) / 2.0, (v_r - v_l) / p.track_width


def normalize_angle(a: float) -> float:
    """(-π, π] に正規化。"""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


class OdometryIntegrator:
    """差動二輪のオドメトリ（中点積分）。"""

    def __init__(self, track_width: float) -> None:
        self.track_width = track_width
        self.reset()

    def reset(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0

    def update(self, v_l: float, v_r: float, dt: float) -> None:
        """左右の車輪速度 [m/s] で dt [s] 進める。"""
        if dt <= 0.0:
            return
        self.v = (v_l + v_r) / 2.0
        self.w = (v_r - v_l) / self.track_width
        yaw_mid = self.yaw + self.w * dt / 2.0
        self.x += self.v * math.cos(yaw_mid) * dt
        self.y += self.v * math.sin(yaw_mid) * dt
        self.yaw = normalize_angle(self.yaw + self.w * dt)
