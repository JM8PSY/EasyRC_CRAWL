"""運動学（ROS 非依存）の単体テスト。"""
import math

import pytest

from cugo_base.kinematics import (
    DiffDriveParams,
    OdometryIntegrator,
    duty_to_speed,
    duty_to_twist,
    speed_to_duty,
    twist_to_duty,
)

P = DiffDriveParams(track_width=0.40, max_wheel_speed=1.0, deadband=0.08,
                    min_speed=0.01, max_duty=0.5)


# ---- speed_to_duty ----

def test_speed_to_duty_zero_and_below_min_speed():
    assert speed_to_duty(0.0, P) == 0.0
    assert speed_to_duty(0.0099, P) == 0.0
    assert speed_to_duty(-0.0099, P) == 0.0


def test_speed_to_duty_formula():
    # deadband + |v|/max_wheel_speed * (1 - deadband)
    assert speed_to_duty(0.2, P) == pytest.approx(0.08 + 0.2 * 0.92)
    assert speed_to_duty(-0.2, P) == pytest.approx(-(0.08 + 0.2 * 0.92))
    assert speed_to_duty(0.01, P) == pytest.approx(0.08 + 0.01 * 0.92)


def test_duty_to_speed_inverse():
    assert duty_to_speed(0.0, P) == 0.0
    assert duty_to_speed(0.08, P) == 0.0
    assert duty_to_speed(-0.05, P) == 0.0
    assert duty_to_speed(1.0, P) == pytest.approx(1.0)
    assert duty_to_speed(-0.264, P) == pytest.approx(-0.2)


@pytest.mark.parametrize('v', [-0.9, -0.3, -0.05, 0.02, 0.15, 0.4, 0.9])
def test_roundtrip(v):
    assert duty_to_speed(speed_to_duty(v, P), P) == pytest.approx(v)


# ---- twist_to_duty ----

def test_twist_straight():
    l, r = twist_to_duty(0.2, 0.0, P)
    assert l == pytest.approx(0.264) and r == pytest.approx(0.264)


def test_twist_zero():
    assert twist_to_duty(0.0, 0.0, P) == (0.0, 0.0)


def test_twist_spin_in_place():
    # v_l = -w*W/2 = -0.1, v_r = +0.1
    l, r = twist_to_duty(0.0, 0.5, P)
    assert l == pytest.approx(-(0.08 + 0.1 * 0.92))
    assert r == pytest.approx(0.08 + 0.1 * 0.92)


def test_twist_left_turn_right_wheel_faster():
    l, r = twist_to_duty(0.2, 0.5, P)
    assert r > l > 0


def test_twist_clamped_to_max_duty():
    l, r = twist_to_duty(2.0, 0.0, P)
    assert l == pytest.approx(0.5) and r == pytest.approx(0.5)
    l, r = twist_to_duty(-2.0, 0.0, P)
    assert l == pytest.approx(-0.5) and r == pytest.approx(-0.5)


def test_twist_clamp_preserves_wheel_speed_ratio():
    # v_l = 0.6, v_r = 1.0 → 上限で縮小しても速度比（曲率）を保つ
    l, r = twist_to_duty(0.8, 1.0, P)
    assert max(abs(l), abs(r)) == pytest.approx(0.5)
    vl, vr = duty_to_speed(l, P), duty_to_speed(r, P)
    assert vl / vr == pytest.approx(0.6)


def test_twist_clamp_with_opposite_signs():
    l, r = twist_to_duty(0.0, 10.0, P)
    assert l == pytest.approx(-0.5) and r == pytest.approx(0.5)


def test_twist_no_scaling_when_within_limit():
    l, r = twist_to_duty(0.1, 0.2, P)
    assert l == pytest.approx(speed_to_duty(0.1 - 0.2 * 0.2, P))
    assert r == pytest.approx(speed_to_duty(0.1 + 0.2 * 0.2, P))


def test_twist_max_duty_below_deadband_gives_zero():
    p = DiffDriveParams(0.4, 1.0, 0.08, 0.01, 0.05)
    assert twist_to_duty(0.5, 0.0, p) == (0.0, 0.0)


def test_twist_max_duty_capped_at_one():
    p = DiffDriveParams(0.4, 1.0, 0.08, 0.01, 1.5)
    l, r = twist_to_duty(5.0, 0.0, p)
    assert l == pytest.approx(1.0) and r == pytest.approx(1.0)


def test_twist_nonfinite_input_gives_zero():
    assert twist_to_duty(float('nan'), 0.0, P) == (0.0, 0.0)
    assert twist_to_duty(0.1, float('inf'), P) == (0.0, 0.0)


# ---- duty_to_twist ----

def test_duty_to_twist():
    v, w = duty_to_twist(-0.264, 0.264, P)
    assert v == pytest.approx(0.0)
    assert w == pytest.approx(0.4 / 0.4)


# ---- OdometryIntegrator ----

def test_odom_straight():
    odo = OdometryIntegrator(track_width=0.4)
    for _ in range(10):
        odo.update(0.5, 0.5, 0.1)
    assert odo.x == pytest.approx(0.5)
    assert odo.y == pytest.approx(0.0)
    assert odo.yaw == pytest.approx(0.0)
    assert odo.v == pytest.approx(0.5)
    assert odo.w == pytest.approx(0.0)


def test_odom_spin_in_place():
    odo = OdometryIntegrator(track_width=0.4)
    odo.update(-0.2, 0.2, 1.0)  # w = 0.4/0.4 = 1 rad/s
    assert odo.x == pytest.approx(0.0)
    assert odo.y == pytest.approx(0.0)
    assert odo.yaw == pytest.approx(1.0)
    assert odo.w == pytest.approx(1.0)


def test_odom_midpoint_single_step():
    odo = OdometryIntegrator(track_width=0.4)
    # v = 0.5, w = 1.0, dt = 1 → 中点 yaw = 0.5
    odo.update(0.3, 0.7, 1.0)
    assert odo.x == pytest.approx(0.5 * math.cos(0.5))
    assert odo.y == pytest.approx(0.5 * math.sin(0.5))
    assert odo.yaw == pytest.approx(1.0)


def test_odom_full_circle_returns_home():
    odo = OdometryIntegrator(track_width=0.4)
    n = 1000
    dt = 2 * math.pi / n  # w = 1 rad/s で 1 周
    for _ in range(n):
        odo.update(0.3, 0.7, dt)
    assert odo.x == pytest.approx(0.0, abs=1e-6)
    assert odo.y == pytest.approx(0.0, abs=1e-6)
    assert odo.yaw == pytest.approx(0.0, abs=1e-6)


def test_odom_yaw_normalized():
    odo = OdometryIntegrator(track_width=0.4)
    odo.update(-0.2, 0.2, 4.0)  # +4 rad
    assert -math.pi < odo.yaw <= math.pi
    assert odo.yaw == pytest.approx(4.0 - 2 * math.pi)


def test_odom_ignores_nonpositive_dt():
    odo = OdometryIntegrator(track_width=0.4)
    odo.update(0.5, 0.5, 0.0)
    odo.update(0.5, 0.5, -1.0)
    assert odo.x == 0.0


def test_odom_reset():
    odo = OdometryIntegrator(track_width=0.4)
    odo.update(0.5, 0.5, 1.0)
    odo.reset()
    assert (odo.x, odo.y, odo.yaw, odo.v, odo.w) == (0.0, 0.0, 0.0, 0.0, 0.0)
