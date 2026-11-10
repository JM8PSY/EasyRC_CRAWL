import pytest

from cugo_rc.config import MotorConfig
from cugo_rc.motor import MockBackend, MotorController


def make(max_accel=2.0, max_speed=1.0):
    b = MockBackend()
    m = MotorController(MotorConfig(max_accel=max_accel, max_speed=max_speed), b)
    return m, b


def test_init_outputs_zero():
    m, b = make()
    assert b.duty == {12: 0.0, 13: 0.0} and b.dir == {5: 0, 6: 0}
    assert b.freq[12] == 20000


def test_forward_direction_and_invert():
    m, b = make(max_accel=1000)
    m.set_target(0.5, 0.5)
    m.tick(0.02)
    assert b.duty[12] == pytest.approx(0.5) and b.dir[5] == 0
    assert b.duty[13] == pytest.approx(0.5) and b.dir[6] == 1  # 右は invert


def test_reverse_left():
    m, b = make(max_accel=1000)
    m.set_target(-0.3, 0)
    m.tick(0.02)
    assert b.duty[12] == pytest.approx(0.3) and b.dir[5] == 1


def test_clamp_and_max_speed():
    m, b = make(max_accel=1000, max_speed=0.5)
    m.set_target(3.0, -3.0)
    assert m.target == (0.5, -0.5)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), True, "1", None])
def test_invalid_values_rejected(bad):
    m, b = make()
    with pytest.raises(ValueError):
        m.set_target(bad, 0)
    assert m.target == (0.0, 0.0)


def test_accel_limit():
    m, b = make(max_accel=2.0)
    m.set_target(1.0, 1.0)
    assert m.tick(0.1) == pytest.approx((0.2, 0.2))
    m.set_target(0, 0)
    assert m.tick(0.05) == pytest.approx((0.1, 0.1))  # 0指令は減速


def test_stop_bypasses_accel_and_clears_target():
    m, b = make(max_accel=2.0)
    m.set_target(1, 1)
    for _ in range(5):
        m.tick(0.1)
    m.stop()
    assert b.duty == {12: 0.0, 13: 0.0} and m.current == (0.0, 0.0)
    m.tick(0.1)
    assert m.current == (0.0, 0.0)  # 古い target で再加速しない


def test_close():
    m, b = make()
    m.close()
    assert b.closed and b.duty == {12: 0.0, 13: 0.0}


def test_tick_dt_is_capped_after_stall():
    m, b = make(max_accel=2.0)
    m.set_target(1.0, -1.0)
    assert m.tick(1.0) == pytest.approx((0.2, -0.2))  # 1秒停滞しても 0.1秒分しか進まない
