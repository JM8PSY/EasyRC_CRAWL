"""キャリブレーションの解析関数（ROS 非依存）の単体テスト。"""
import math

import pytest

from cugo_base.calibration import (
    fit_line,
    mean_speed,
    mean_yaw_rate,
    suggest_speed_params,
    track_width_from_spin,
)
from cugo_base.kinematics import DiffDriveParams, duty_to_speed


def test_fit_line_exact():
    k, c = fit_line([1.0, 2.0, 3.0], [3.0, 5.0, 7.0])
    assert k == pytest.approx(2.0) and c == pytest.approx(1.0)


def test_fit_line_needs_two_distinct_x():
    with pytest.raises(ValueError):
        fit_line([1.0], [1.0])
    with pytest.raises(ValueError):
        fit_line([1.0, 1.0], [1.0, 2.0])


def test_suggest_speed_params_recovers_model():
    truth = DiffDriveParams(max_wheel_speed=0.8, deadband=0.12)
    duties = [0.1, 0.2, 0.3, 0.4, 0.5]
    samples = [(d, duty_to_speed(d, truth)) for d in duties]
    mws, db = suggest_speed_params(samples)
    assert mws == pytest.approx(0.8)
    assert db == pytest.approx(0.12)


def test_suggest_speed_params_ignores_stalled_points():
    truth = DiffDriveParams(max_wheel_speed=1.2, deadband=0.1)
    samples = [(0.05, 0.0), (0.08, 0.001)] + [
        (d, duty_to_speed(d, truth)) for d in (0.2, 0.35, 0.5)]
    mws, db = suggest_speed_params(samples)
    assert mws == pytest.approx(1.2)
    assert db == pytest.approx(0.1)


def test_suggest_speed_params_not_enough_data():
    with pytest.raises(ValueError):
        suggest_speed_params([(0.1, 0.0), (0.2, 0.1)])


def test_mean_speed_skips_settle_time():
    # 0〜1s は加速中（0.1 m/s）、1s 以降 0.3 m/s 一定
    poses = []
    x = 0.0
    for i in range(41):
        t = i * 0.1
        poses.append((t, x * math.cos(0.3), x * math.sin(0.3)))
        x += (0.1 if t < 1.0 else 0.3) * 0.1
    assert mean_speed(poses, settle=1.05) == pytest.approx(0.3)


def test_mean_speed_too_few_samples():
    with pytest.raises(ValueError):
        mean_speed([(0.0, 0.0, 0.0)], settle=0.0)


def test_mean_yaw_rate_unwraps():
    samples = []
    for i in range(101):
        t = i * 0.05
        yaw = math.atan2(math.sin(1.5 * t), math.cos(1.5 * t))  # (-π, π] で折り返す
        samples.append((t, yaw))
    assert mean_yaw_rate(samples, settle=0.5) == pytest.approx(1.5)
    neg = [(t, -y) for t, y in samples]
    assert mean_yaw_rate(neg, settle=0.5) == pytest.approx(-1.5)


def test_track_width_from_spin():
    p = DiffDriveParams(max_wheel_speed=1.0, deadband=0.08)
    v = duty_to_speed(0.3, p)
    # 実効トレッド 0.5 m なら ω = 2v / 0.5
    assert track_width_from_spin(0.3, 2 * v / 0.5, p) == pytest.approx(0.5)
    assert track_width_from_spin(0.3, -2 * v / 0.5, p) == pytest.approx(0.5)


def test_track_width_from_spin_rejects_no_rotation():
    p = DiffDriveParams()
    with pytest.raises(ValueError):
        track_width_from_spin(0.3, 0.0, p)


# ---- cugo_calibrate の安全装置 ----

def test_calibrate_defaults_to_dry_run(monkeypatch, capsys):
    from cugo_base import calibrate

    def forbidden(*_a, **_k):
        raise AssertionError('ドライランでシリアル / ROS を開いてはいけない')

    monkeypatch.setattr(calibrate, 'Esp', forbidden)
    monkeypatch.setattr(calibrate, 'OdomRecorder', forbidden)
    assert calibrate.main(['--duties', '0.1,0.2']) == 0
    out = capsys.readouterr().out
    assert 'ドライラン' in out
    assert '$D,0.100,0.100' in out and '$D,-0.300,0.300' in out


def test_calibrate_rejects_bad_duty():
    from cugo_base import calibrate
    with pytest.raises(SystemExit):
        calibrate.parse_args(['--duties', '1.5'])
