"""キャリブレーション結果の解析（ROS 非依存）。

モデル: v = (|duty| - deadband) / (1 - deadband) * max_wheel_speed
       → v = k·duty + c と直線近似すると deadband = -c/k、max_wheel_speed = k·(1 - deadband)
"""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from cugo_base.kinematics import DiffDriveParams, duty_to_speed

MIN_MOVING_SPEED = 0.02  # [m/s] これ未満は「動いていない」とみなしてフィットから外す


def fit_line(xs: Sequence[float], ys: Sequence[float]) -> Tuple[float, float]:
    """最小二乗で y = k·x + c。"""
    n = len(xs)
    if n < 2 or n != len(ys):
        raise ValueError('need at least 2 points')
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 1e-12:
        raise ValueError('x values must differ')
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    k = sxy / sxx
    return k, my - k * mx


def suggest_speed_params(samples: Sequence[Tuple[float, float]],
                         min_moving: float = MIN_MOVING_SPEED) -> Tuple[float, float]:
    """(duty, 実測速度) の組から (max_wheel_speed, deadband) を推定する。"""
    moving = [(abs(d), abs(v)) for d, v in samples if abs(v) >= min_moving]
    if len(moving) < 2:
        raise ValueError('動いた測定点が 2 つ以上必要')
    k, c = fit_line([d for d, _ in moving], [v for _, v in moving])
    if k <= 0:
        raise ValueError('duty を上げても速くならない（測定を確認）')
    deadband = min(max(-c / k, 0.0), 0.95)
    return k * (1.0 - deadband), deadband


def mean_speed(poses: Sequence[Tuple[float, float, float]], settle: float) -> float:
    """(t, x, y) の列から、開始 settle 秒後以降の平均速さ [m/s]（経路長 / 時間）。"""
    if not poses:
        raise ValueError('no samples')
    t0 = poses[0][0] + settle
    use = [p for p in poses if p[0] >= t0]
    if len(use) < 2 or use[-1][0] <= use[0][0]:
        raise ValueError('not enough samples after settle time')
    length = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(use, use[1:]))
    return length / (use[-1][0] - use[0][0])


def mean_yaw_rate(samples: Sequence[Tuple[float, float]], settle: float) -> float:
    """(t, yaw) の列から、開始 settle 秒後以降の平均角速度 [rad/s]（折り返しを展開）。"""
    if not samples:
        raise ValueError('no samples')
    t0 = samples[0][0] + settle
    use = [s for s in samples if s[0] >= t0]
    if len(use) < 2 or use[-1][0] <= use[0][0]:
        raise ValueError('not enough samples after settle time')
    total = 0.0
    for (_, a), (_, b) in zip(use, use[1:]):
        total += math.atan2(math.sin(b - a), math.cos(b - a))
    return total / (use[-1][0] - use[0][0])


def track_width_from_spin(duty: float, yaw_rate: float, p: DiffDriveParams) -> float:
    """その場旋回（左 -duty、右 +duty）の実測角速度から実効トレッド [m]。

    クローラーは横滑りするため幾何学的な幅より大きくなるのが普通。
    """
    v = abs(duty_to_speed(duty, p))
    if abs(yaw_rate) < 1e-3 or v <= 0.0:
        raise ValueError('旋回していない（duty が deadband 以下か、オドメトリが来ていない）')
    return 2.0 * v / abs(yaw_rate)


def format_suggestion(lines: List[str]) -> str:
    return '\n'.join(['', '==== 推奨パラメータ（config/cugo_base.yaml） ===='] + lines)
