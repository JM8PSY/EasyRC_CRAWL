"""MD20A (PWM + DIR) ×2 によるモーター制御。"""
from __future__ import annotations

import math
import threading

from .config import MotorConfig, MotorSide


# 制御ループが停滞した後でも1回の tick で進める時間の上限 [s]（加速度制限の飛び越し防止）
MAX_TICK_DT = 0.1


class BackendError(RuntimeError):
    pass


class MockBackend:
    """GPIO 出力を記録するだけのバックエンド（テスト・--mock 用）。"""

    def __init__(self) -> None:
        self.duty: dict[int, float] = {}
        self.freq: dict[int, int] = {}
        self.dir: dict[int, int] = {}
        self.closed = False

    def set_pwm(self, pin: int, freq: int, duty: float) -> None:
        self.freq[pin] = freq
        self.duty[pin] = duty

    def set_dir(self, pin: int, level: int) -> None:
        self.dir[pin] = level

    def close(self) -> None:
        self.closed = True


class PigpioBackend:
    """pigpiod 経由でハードウェア PWM と GPIO を操作する。"""

    def __init__(self) -> None:
        import pigpio  # Pi 以外で import エラーにならないよう遅延 import

        self._pigpio = pigpio
        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise BackendError(
                "pigpiod に接続できません。sudo systemctl enable --now pigpiod を実行してください"
            )
        self._pwm_pins: set[int] = set()
        self._dir_pins: set[int] = set()

    def set_pwm(self, pin: int, freq: int, duty: float) -> None:
        self._pwm_pins.add(pin)
        self._pi.hardware_PWM(pin, freq, int(duty * 1_000_000))

    def set_dir(self, pin: int, level: int) -> None:
        if pin not in self._dir_pins:
            self._pi.set_mode(pin, self._pigpio.OUTPUT)
            self._dir_pins.add(pin)
        self._pi.write(pin, level)

    def close(self) -> None:
        for pin in self._pwm_pins:
            self._pi.hardware_PWM(pin, 0, 0)
        self._pi.stop()


def _clamp(v: float) -> float:
    return max(-1.0, min(1.0, v))


def _validate(v: object) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError(f"不正な速度値: {v!r}")
    return float(v)


def _approach(current: float, target: float, step: float) -> float:
    if abs(target - current) <= step:
        return target
    return current + math.copysign(step, target - current)


class MotorController:
    """左右速度 (-1.0〜1.0) を加速度制限つきで出力する。全メソッドはスレッドセーフ。"""

    def __init__(self, cfg: MotorConfig, backend) -> None:
        self._cfg = cfg
        self._backend = backend
        self._lock = threading.Lock()
        self._target = (0.0, 0.0)
        self._current = (0.0, 0.0)
        with self._lock:
            self._output()

    @property
    def target(self) -> tuple[float, float]:
        return self._target

    @property
    def current(self) -> tuple[float, float]:
        return self._current

    def set_target(self, l: float, r: float) -> None:
        l, r = _validate(l), _validate(r)
        s = self._cfg.max_speed
        with self._lock:
            self._target = (_clamp(l) * s, _clamp(r) * s)

    def tick(self, dt: float) -> tuple[float, float]:
        step = self._cfg.max_accel * min(dt, MAX_TICK_DT)
        with self._lock:
            self._current = (
                _approach(self._current[0], self._target[0], step),
                _approach(self._current[1], self._target[1], step),
            )
            self._output()
            return self._current

    def stop(self) -> None:
        with self._lock:
            self._target = (0.0, 0.0)
            self._current = (0.0, 0.0)
            self._output()

    def close(self) -> None:
        self.stop()
        self._backend.close()

    def _output(self) -> None:
        self._write(self._cfg.left, self._current[0])
        self._write(self._cfg.right, self._current[1])

    def _write(self, side: MotorSide, v: float) -> None:
        if side.invert:
            v = -v
        self._backend.set_dir(side.dir_pin, 0 if v >= 0 else 1)
        self._backend.set_pwm(side.pwm_pin, self._cfg.pwm_freq, abs(v))
